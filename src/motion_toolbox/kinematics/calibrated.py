"""Refine analytic UR branches against a calibrated URDF serial chain."""
import math
import xml.etree.ElementTree as ET
import numpy as np
from .solver import URKinematics
from .ur import inverse_kinematics
from ..geometry import Plane, as_plane, rigid_inverse
from ..recording import recorded


def _rotation(axis, angle):
    x,y,z = axis
    skew = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(angle)*skew+(1-math.cos(angle))*(skew@skew)


def _origin(joint):
    element = joint.find('origin')
    T = np.eye(4)
    if element is not None:
        T[:3,3] = [float(x) for x in element.get('xyz','0 0 0').split()]
        r,p,y = [float(x) for x in element.get('rpy','0 0 0').split()]
        T[:3,:3] = _rotation((0,0,1),y)@_rotation((0,1,0),p)@_rotation((1,0,0),r)
    return T


class CalibratedURKinematics(URKinematics):
    """Nominal analytic seeds, Newton refinement, and explicit FK acceptance.

    No collision or joint bounds are removed: normal planner filtering follows
    refinement. Controller calibration and TCP transforms stay independent of
    the captured link geometry. The solver does not query a live robot.
    """
    @recorded
    def __init__(self, urdf, joint_names, *, controller_link, end_link, fixed_joint_values=None, **options):
        super().__init__(**options)
        root = ET.fromstring(urdf)
        by_child = {j.find('child').get('link'):j for j in root.findall('joint')}
        names = list(joint_names)
        fixed = fixed_joint_values or {}
        def chain(link):
            result, seen = [], set()
            while link in by_child:
                if link in seen:
                    raise ValueError('Cyclic URDF chain')
                seen.add(link)
                j = by_child[link]
                result.append(j)
                link = j.find('parent').get('link')
            return list(reversed(result))
        def fixed_transform(j):
            T = _origin(j)
            kind = j.get('type')
            if kind == 'fixed':
                return T
            if j.get('name') not in fixed:
                raise ValueError('Missing fixed joint value: '+j.get('name'))
            element = j.find('axis')
            axis = np.array([float(x) for x in (element.get('xyz') if element is not None else '1 0 0').split()])
            axis /= np.linalg.norm(axis)
            value = fixed[j.get('name')]
            if kind == 'prismatic':
                T[:3,3] += T[:3,:3]@axis*value
            elif kind in ('revolute','continuous'):
                T[:3,:3] = T[:3,:3]@_rotation(axis,value)
            else:
                raise ValueError('Unsupported fixed joint type: '+kind)
            return T
        controller = np.eye(4)
        for j in chain(controller_link):
            if j.get('name') in names:
                raise ValueError('Controller link must be upstream of arm')
            controller = controller@fixed_transform(j)
        pending = rigid_inverse(controller)
        self.origins, self.axes, found = [], [], []
        for j in chain(end_link):
            if j.get('name') in names:
                pending = pending@_origin(j)
                self.origins.append(pending)
                element = j.find('axis')
                axis = np.array([float(x) for x in (element.get('xyz') if element is not None else '1 0 0').split()])
                self.axes.append(axis/np.linalg.norm(axis))
                found.append(j.get('name'))
                pending = np.eye(4)
            else:
                pending = pending@fixed_transform(j)
        if found != names or len(names) != 6:
            raise ValueError('Calibrated chain must contain the six arm joints in order')
        self.end_transform = pending
        self._skews = [np.array([[0,-z,y],[z,0,-x],[-y,x,0]]) for x,y,z in self.axes]
        self._skew_squares = [s@s for s in self._skews]
        self.model_accuracy = 'calibrated URDF FK, 1e-7 m/rad solver tolerance'

    def forward(self, q, jacobian=False):
        T = np.eye(4)
        points, axes = [], []
        for value,origin,axis in zip(q,self.origins,self.axes):
            T = T@origin
            if jacobian:
                points.append(T[:3,3].copy())
                axes.append(T[:3,:3]@axis)
            T[:3,:3] = T[:3,:3]@_rotation(axis,float(value))
        T = T@self.end_transform
        if not jacobian:
            return T
        axes = np.asarray(axes)
        J = np.vstack((np.cross(axes,T[:3,3]-np.asarray(points)).T,axes.T))
        return T,J

    def refine(self, target, seed):
        q = np.asarray(seed,dtype=float).copy()
        for _ in range(16):
            actual,J = self.forward(q,True)
            position = target[:3,3]-actual[:3,3]
            rotation = .5*np.cross(actual[:3,:3].T,target[:3,:3].T).sum(axis=0)
            if np.linalg.norm(position) <= 1e-7 and np.linalg.norm(actual[:3,:3]-target[:3,:3]) <= 1.4e-7:
                return q.tolist()
            error = np.r_[position,rotation]
            step = np.linalg.lstsq(J,error,rcond=1e-8)[0]
            step *= min(1.,.25/max(np.max(abs(step)),1e-12))
            q += step
        return None

    def _forward_batch(self, configurations):
        """The same calibrated FK/Jacobian, evaluated for all active branches."""
        q = np.asarray(configurations, dtype=float)
        T = np.broadcast_to(np.eye(4), (len(q), 4, 4)).copy()
        points = np.empty((len(q), 6, 3))
        axes = np.empty_like(points)
        for j, (origin, axis, skew, square) in enumerate(zip(
                self.origins, self.axes, self._skews, self._skew_squares)):
            T = T @ origin
            points[:, j] = T[:, :3, 3]
            axes[:, j] = T[:, :3, :3] @ axis
            angles = q[:, j, None, None]
            rotation = np.eye(3) + np.sin(angles)*skew + (1-np.cos(angles))*square
            T[:, :3, :3] = T[:, :3, :3] @ rotation
        T = T @ self.end_transform
        jacobian = np.concatenate((np.cross(axes, T[:, None, :3, 3]-points).transpose(0, 2, 1),
                                   axes.transpose(0, 2, 1)), axis=1)
        return T, jacobian

    def _refine_seeds(self, target, seeds):
        """Retain seed order and scalar refinement tolerances/iteration limits.

        Only FK and Jacobian arithmetic is batched. Each branch still uses the
        same least-squares solve and acceptance test, including singular cases.
        """
        if not len(seeds):
            return []
        q = np.asarray(seeds, dtype=float).copy()
        active = np.arange(len(q))
        accepted = np.zeros(len(q), dtype=bool)
        for _ in range(16):
            actual, jacobian = self._forward_batch(q[active])
            position = target[:3, 3]-actual[:, :3, 3]
            rotation = .5*np.cross(actual[:, :3, :3].transpose(0, 2, 1), target[:3, :3].T).sum(axis=1)
            converged = ((np.linalg.norm(position, axis=1) <= 1e-7)
                         & (np.linalg.norm(actual[:, :3, :3]-target[:3, :3], axis=(1, 2)) <= 1.4e-7))
            accepted[active[converged]] = True
            active = active[~converged]
            if not len(active):
                break
            error = np.concatenate((position[~converged], rotation[~converged]), axis=1)
            steps = np.array([np.linalg.lstsq(J, e, rcond=1e-8)[0]
                              for J, e in zip(jacobian[~converged], error)])
            steps *= np.minimum(1., .25/np.maximum(np.max(abs(steps), axis=1), 1e-12))[:, None]
            q[active] += steps
        return q[accepted].tolist()

    @recorded(detail=True)
    def __call__(self, target, base):
        T = self._arm_inverse@rigid_inverse(as_plane(base).matrix)@as_plane(target).matrix@self._tcp_to_flange
        return self._refine_seeds(T, inverse_kinematics(Plane.from_matrix(T),self.parameters))
