"""Persistent, isolated PyBullet collision worlds; no dependency on Rhino."""
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import itertools
import numpy as np
from .geometry import Plane, as_plane


class PybulletServer:
    def __init__(self, urdf_path=None, *, robot=None, gui=False, joint_names=None,
                 allowed_pairs=(), package_paths=None, asset_root=None, base=None):
        from pybullet_utils.bullet_client import BulletClient
        import pybullet as p
        self.p = BulletClient(connection_mode=p.GUI if gui else p.DIRECT)
        self.robot = None
        self.environment = []
        self.tools = []
        self._temporary = TemporaryDirectory(prefix='motion-toolbox-')
        self._base = Plane.world_xy()
        self.joints = []
        self.links = {}
        self.allowed_pairs = {frozenset(pair) for pair in allowed_pairs}
        try:
            if robot is not None:
                if urdf_path is not None:
                    raise ValueError('Provide a robot object or a URDF path, not both')
                from .robot_adapter import export_robot
                urdf_path, attachments = export_robot(robot, Path(self._temporary.name), package_paths, asset_root)
                if base is None:
                    base = getattr(robot, 'BCF', None)
            else:
                attachments = []
            if urdf_path is not None:
                self.load_robot(urdf_path, joint_names=joint_names, base=base)
                for acm in attachments:
                    self.attach_mesh(**acm)
        except Exception:
            self.close()
            raise

    def load_robot(self, urdf_path, *, joint_names=None, base=None):
        if self.robot is not None:
            raise ValueError('Create a new world to replace the robot')
        path = Path(urdf_path).resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        p = self.p
        self.robot = p.loadURDF(str(path), useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
        self.links = {p.getBodyInfo(self.robot)[0].decode(): -1}
        movable = {}
        adjacent = set()
        for i in range(p.getNumJoints(self.robot)):
            info = p.getJointInfo(self.robot, i)
            self.links[info[12].decode()] = i
            adjacent.add(frozenset((i, info[16])))
            if info[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                movable[info[1].decode()] = i
        names = list(movable) if joint_names is None else list(joint_names)
        if len(set(names)) != len(names) or any(name not in movable for name in names):
            raise ValueError('Unknown or duplicate planning joint name')
        self.joints = [movable[name] for name in names]
        self.joint_names = names
        self.all_joints = movable
        self._joint_limits = []
        for i in self.joints:
            info = p.getJointInfo(self.robot, i)
            self._joint_limits.append((info[8], info[9]))
        unknown = set().union(*self.allowed_pairs) - set(self.links) if self.allowed_pairs else set()
        if unknown:
            raise ValueError('Unknown allowed collision links: ' + ', '.join(unknown))
        allowed = {frozenset(self.links[n] for n in pair) for pair in self.allowed_pairs}
        self.self_pairs = [(a, b) for a, b in itertools.combinations(self.links.values(), 2)
                           if frozenset((a, b)) not in adjacent | allowed]
        self.set_base(Plane.world_xy() if base is None else base)
        return self.robot

    def _pose(self, plane):
        plane = as_plane(plane)
        # Bullet matrix conversion via stable Euler extraction; quaternion is xyzw.
        R = plane.matrix[:3, :3]
        pitch = math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0]))
        if abs(math.cos(pitch)) > 1e-9:
            roll, yaw = math.atan2(R[2, 1], R[2, 2]), math.atan2(R[1, 0], R[0, 0])
        else:
            roll, yaw = math.atan2(-R[1, 2], R[1, 1]), 0.0
        return plane.origin.tolist(), self.p.getQuaternionFromEuler([roll, pitch, yaw])

    def set_base(self, base):
        self._base = as_plane(base)
        position, orientation = self._pose(self._base)
        # resetBasePositionAndOrientation expects the inertial frame, not URDF root.
        dynamics = self.p.getDynamicsInfo(self.robot, -1)
        position, orientation = self.p.multiplyTransforms(position, orientation, dynamics[3], dynamics[4])
        self.p.resetBasePositionAndOrientation(self.robot, position, orientation)

    def set_fixed_joints(self, values):
        """Set nonplanned lift/wheel joints explicitly by URDF joint name."""
        for name, value in values.items():
            if name not in self.all_joints or not math.isfinite(value):
                raise ValueError('Invalid fixed joint')
            self.p.resetJointState(self.robot, self.all_joints[name], value)

    def _body(self, shape, plane=None):
        pos, orn = self._pose(Plane.world_xy() if plane is None else plane)
        return self.p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape,
                                      basePosition=pos, baseOrientation=orn)

    def add_mesh(self, mesh, *, plane=None, scale=1.0):
        """Environment mesh: filename or (vertices, faces), in metres."""
        p = self.p
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError('Positive mesh scale required')
        kwargs = self._mesh_args(mesh)
        shape = p.createCollisionShape(p.GEOM_MESH, meshScale=[scale]*3,
            flags=p.GEOM_FORCE_CONCAVE_TRIMESH, **kwargs)
        body = self._body(shape, plane)
        self.environment.append((body, set()))
        return body

    @staticmethod
    def _mesh_args(mesh):
        if isinstance(mesh, (str, Path)):
            path = Path(mesh).resolve()
            if not path.is_file():
                raise FileNotFoundError(path)
            return dict(fileName=str(path))
        if hasattr(mesh, 'to_vertices_and_faces'):
            vertices, faces = mesh.to_vertices_and_faces()
        elif hasattr(mesh, 'Vertices'):
            vertices = [[v.X, v.Y, v.Z] for v in mesh.Vertices]
            faces = [[f.A, f.B, f.C, f.D] if f.IsQuad else [f.A, f.B, f.C] for f in mesh.Faces]
        else:
            vertices, faces = mesh
        triangles = [index for face in faces for i in range(1, len(face)-1) for index in (face[0], face[i], face[i+1])]
        return dict(vertices=vertices, indices=triangles)

    def add_box(self, half_extents, *, plane=None):
        shape = self.p.createCollisionShape(self.p.GEOM_BOX, halfExtents=half_extents)
        body = self._body(shape, plane)
        self.environment.append((body, set()))
        return body

    def add_ground(self, z=0.0, *, support_links=()):
        unknown = set(support_links)-set(self.links)
        if unknown:
            raise ValueError('Unknown ground support links')
        shape = self.p.createCollisionShape(self.p.GEOM_PLANE, planeNormal=[0, 0, 1])
        body = self._body(shape, Plane((0, 0, z), (1, 0, 0), (0, 1, 0)))
        self.environment.append((body, {self.links[n] for n in support_links}))
        return body

    def attach_mesh(self, mesh, link_name, *, frame=None, touch_links=()):
        if link_name not in self.links or any(n not in self.links for n in touch_links):
            raise ValueError('Unknown tool attachment/touch link')
        # A convex hull is conservative for attached tool collision geometry.
        # Supply separate convex meshes to retain nonconvex tool detail.
        arguments = self._mesh_args(mesh)
        # Supplying triangle indices creates a triangle mesh rather than a
        # solid hull; enclosed obstacles could then be missed entirely.
        arguments.pop('indices', None)
        shape = self.p.createCollisionShape(self.p.GEOM_MESH, **arguments)
        body = self._body(shape)
        self.tools.append((body, self.links[link_name], as_plane(frame) if frame is not None else Plane.world_xy(),
                           {self.links[n] for n in touch_links} | {self.links[link_name]}))
        return body

    def _update_tools(self):
        for body, link, frame, _ in self.tools:
            if link == -1:
                pos, orn = self._pose(self._base)
            else:
                state = self.p.getLinkState(self.robot, link, computeForwardKinematics=True)
                pos, orn = state[4], state[5]
            local_pos, local_orn = self._pose(frame)
            pos, orn = self.p.multiplyTransforms(pos, orn, local_pos, local_orn)
            self.p.resetBasePositionAndOrientation(body, pos, orn)

    def is_valid(self, configuration, base=None, *, clearance=0.0):
        if self.robot is None:
            raise ValueError('Load a robot before checking collisions')
        q = list(configuration)
        if len(q) != len(self.joints) or not all(math.isfinite(v) for v in q):
            raise ValueError('Configuration must match explicitly selected URDF joints')
        if clearance < 0 or not math.isfinite(clearance):
            raise ValueError('Clearance must be nonnegative')
        for value, (lo, hi) in zip(q, self._joint_limits):
            if lo <= hi and not lo <= value <= hi:
                return False
        if base is not None:
            self.set_base(base)
        p = self.p
        for index, value in zip(self.joints, q):
            p.resetJointState(self.robot, index, value)
        self._update_tools()
        for a, b in self.self_pairs:
            if p.getClosestPoints(self.robot, self.robot, clearance, linkIndexA=a, linkIndexB=b):
                return False
        for obstacle, ignored in self.environment:
            contacts = p.getClosestPoints(self.robot, obstacle, clearance)
            if any(c[3] not in ignored for c in contacts):
                return False
            if any(p.getClosestPoints(body, obstacle, clearance) for body, _, _, _ in self.tools):
                return False
        for body, _, _, touch in self.tools:
            if any(c[4] not in touch for c in p.getClosestPoints(body, self.robot, clearance)):
                return False
        return True

    def edge_is_valid(self, q0, base0, q1, base1, *, joint_resolution=0.05, base_resolution=0.02,
                      yaw_resolution=0.05, periodic=None):
        """Sample a synchronized arm/base edge, including both endpoints.

        This is resolution-dependent collision checking, not a continuous proof.
        Base planes must be upright; yaw is interpolated along the shortest arc.
        """
        if min(joint_resolution, base_resolution, yaw_resolution) <= 0:
            raise ValueError('Positive sampling resolutions required')
        b0, b1 = as_plane(base0), as_plane(base1)
        if not np.allclose(b0.zaxis, [0, 0, 1]) or not np.allclose(b1.zaxis, [0, 0, 1]):
            raise ValueError('Mobile edge interpolation requires upright base planes')
        q0, q1 = np.asarray(q0), np.asarray(q1)
        delta = q1-q0
        if periodic is not None:
            mask = np.asarray(periodic, dtype=bool)
            delta[mask] = (delta[mask]+np.pi) % (2*np.pi)-np.pi
        y0, y1 = math.atan2(b0.xaxis[1], b0.xaxis[0]), math.atan2(b1.xaxis[1], b1.xaxis[0])
        dy = (y1-y0+math.pi) % (2*math.pi)-math.pi
        steps = max(1, math.ceil(np.max(np.abs(delta))/joint_resolution) if len(delta) else 1,
                    math.ceil(np.linalg.norm(b1.origin-b0.origin)/base_resolution), math.ceil(abs(dy)/yaw_resolution))
        for t in np.linspace(0, 1, steps+1):
            yaw = y0+t*dy
            base = Plane((1-t)*b0.origin+t*b1.origin, (math.cos(yaw), math.sin(yaw), 0), (-math.sin(yaw), math.cos(yaw), 0))
            if not self.is_valid(q0+t*delta, base):
                return False
        return True

    def close(self):
        if self.p.isConnected():
            self.p.disconnect()
        self._temporary.cleanup()

    disconnect = close

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
