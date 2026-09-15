"""Persistent, isolated PyBullet collision worlds; no dependency on Rhino."""
from motion_toolbox.recording import recorded, event
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import itertools
import shutil
import xml.etree.ElementTree as ET
import numpy as np
from .geometry import Plane, as_plane


COLLISION_API_VERSION = 10


def _validate_urdf(path):
    """Surface common importer errors that Rhino can hide from Bullet stdout."""
    root = ET.parse(path).getroot()
    links = [link.get('name') for link in root.findall('link')]
    if root.tag != 'robot' or not links or len(links) != len(set(links)):
        raise ValueError('URDF must contain a robot with uniquely named links')
    for joint in root.findall('joint'):
        name = joint.get('name')
        for end in ('parent', 'child'):
            element = joint.find(end)
            if element is None or element.get('link') not in links:
                raise ValueError('URDF joint {} has an invalid {} link'.format(name, end))
        if joint.get('type') in ('revolute', 'prismatic'):
            limit = joint.find('limit')
            # COMPAS can omit zero-valued attributes; Bullet accepts those.
            if limit is None:
                raise ValueError('URDF joint {} is missing required limits'.format(name))
    for link in root.findall('link'):
        for element in link.findall('.//mesh'):
            filename = element.get('filename', '')
            mesh = Path(filename)
            if not mesh.is_absolute():
                mesh = path.parent / mesh
            if not filename or not mesh.is_file() or mesh.stat().st_size == 0:
                raise ValueError('URDF link {} references a missing or empty mesh: {}'.format(link.get('name'), mesh))
            event('collision.mesh_asset', path=mesh, link=link.get('name'))
    return root


class PybulletServer:
    @recorded
    def __init__(self, urdf_path=None, *, robot=None, gui=False, joint_names=None,
                 allowed_pairs=(), package_paths=None, asset_root=None, base=None,
                 check_static_self_collisions=True):
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
        self.check_static_self_collisions = check_static_self_collisions
        semantics = getattr(robot, 'semantics', None)
        self.allowed_pairs.update(frozenset(pair) for pair in getattr(semantics, 'disabled_collisions', ()))
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

    @recorded
    def load_robot(self, urdf_path, *, joint_names=None, base=None):
        if self.robot is not None:
            raise ValueError('Create a new world to replace the robot')
        path = Path(urdf_path).resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        p = self.p
        try:
            _validate_urdf(path)
            self.robot = p.loadURDF(path.as_posix(), useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
        except Exception as error:
            diagnostic_path = path
            if path.parent == Path(self._temporary.name).resolve():
                # Preserve the generated model and meshes after this world closes.
                from tempfile import mkdtemp
                folder = Path(mkdtemp(prefix='motion-toolbox-failed-'))
                shutil.copytree(path.parent, folder, dirs_exist_ok=True)
                diagnostic_path = folder / path.name
                try:
                    tree = ET.parse(diagnostic_path)
                    for mesh in tree.iter('mesh'):
                        original = Path(mesh.get('filename', ''))
                        if original.is_absolute() and original.parent == path.parent:
                            mesh.set('filename', (folder / original.name).as_posix())
                    ET.indent(tree, space='  ')
                    tree.write(diagnostic_path, encoding='utf-8', xml_declaration=True)
                except ET.ParseError:
                    pass  # Preserve malformed XML verbatim for inspection.
            message = 'PyBullet URDF load failed: {}. Inspect retained URDF: {}'.format(error, diagnostic_path)
            raise RuntimeError(message) from error
        self.links = {p.getBodyInfo(self.robot)[0].decode(): -1}
        movable = {}
        adjacent = set()
        parents = {}
        self.fixed_neighbors = {}
        for i in range(p.getNumJoints(self.robot)):
            info = p.getJointInfo(self.robot, i)
            self.links[info[12].decode()] = i
            adjacent.add(frozenset((i, info[16])))
            parents[i] = info[16]
            if info[2] == p.JOINT_FIXED:
                self.fixed_neighbors.setdefault(i, set()).add(info[16])
                self.fixed_neighbors.setdefault(info[16], set()).add(i)
            if info[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                movable[info[1].decode()] = i
        names = list(movable) if joint_names is None else list(joint_names)
        if len(set(names)) != len(names) or any(name not in movable for name in names):
            raise ValueError('Unknown or duplicate planning joint name')
        self.joints = [movable[name] for name in names]
        self.joint_names = names
        self.all_joints = movable
        self.link_names = {index: name for name, index in self.links.items()}
        self.collision_links = tuple(i for i in self.links.values()
                                     if p.getCollisionShapeData(self.robot, i))
        def follows_planned_joint(index):
            while index != -1:
                if index in self.joints:
                    return True
                index = parents[index]
            return False
        self.static_links = {i for i in self.links.values() if not follows_planned_joint(i)}
        self._joint_limits = []
        self._joint_types = []
        for i in self.joints:
            info = p.getJointInfo(self.robot, i)
            self._joint_limits.append((info[8], info[9]))
            self._joint_types.append(info[2])
        unknown = set().union(*self.allowed_pairs) - set(self.links) if self.allowed_pairs else set()
        if unknown:
            raise ValueError('Unknown allowed collision links: ' + ', '.join(unknown))
        allowed = {frozenset(self.links[n] for n in pair) for pair in self.allowed_pairs}
        self.self_pairs = [(a, b) for a, b in itertools.combinations(self.collision_links, 2)
                           if frozenset((a, b)) not in adjacent | allowed and
                           (self.check_static_self_collisions or a not in self.static_links or b not in self.static_links)]
        self._self_pair_keys = {frozenset(pair) for pair in self.self_pairs}
        # Tell Bullet which self pairs matter so its broadphase can skip fixed
        # assembly and explicitly allowed contacts before narrow-phase testing.
        for a, b in itertools.combinations(self.links.values(), 2):
            p.setCollisionFilterPair(self.robot, self.robot, a, b,
                                     int(frozenset((a,b)) in self._self_pair_keys))
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

    @recorded(detail=True)
    def set_base(self, base):
        self._base = as_plane(base)
        position, orientation = self._pose(self._base)
        # resetBasePositionAndOrientation expects the inertial frame, not URDF root.
        dynamics = self.p.getDynamicsInfo(self.robot, -1)
        position, orientation = self.p.multiplyTransforms(position, orientation, dynamics[3], dynamics[4])
        self.p.resetBasePositionAndOrientation(self.robot, position, orientation)

    @recorded
    def set_fixed_joints(self, values):
        """Set nonplanned lift/wheel joints explicitly by URDF joint name."""
        for name, value in values.items():
            if name not in self.all_joints or name in self.joint_names or not math.isfinite(value):
                raise ValueError('Invalid fixed joint')
            info = self.p.getJointInfo(self.robot, self.all_joints[name])
            if info[8] <= info[9] and not info[8] <= value <= info[9]:
                raise ValueError('Fixed joint value outside URDF limits: ' + name)
            self.p.resetJointState(self.robot, self.all_joints[name], value)

    def _body(self, shape, plane=None):
        pos, orn = self._pose(Plane.world_xy() if plane is None else plane)
        body = self.p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape,
                                     basePosition=pos, baseOrientation=orn)
        # These bodies are checked explicitly with getClosestPoints. Exclude
        # them from global contact generation, used only for robot self checks,
        # so environment/tool contacts are not computed twice per configuration.
        self.p.setCollisionFilterGroupMask(body, -1, 0, 0)
        return body

    @recorded
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

    @recorded
    def add_box(self, half_extents, *, plane=None):
        shape = self.p.createCollisionShape(self.p.GEOM_BOX, halfExtents=half_extents)
        body = self._body(shape, plane)
        self.environment.append((body, set()))
        return body

    @recorded
    def add_ground(self, z=0.0, *, support_links=()):
        unknown = set(support_links)-set(self.links)
        if unknown:
            raise ValueError('Unknown ground support links')
        shape = self.p.createCollisionShape(self.p.GEOM_PLANE, planeNormal=[0, 0, 1])
        body = self._body(shape, Plane((0, 0, z), (1, 0, 0), (0, 1, 0)))
        self.environment.append((body, {self.links[n] for n in support_links}))
        return body

    @recorded
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
        # flange/tool0 can be empty frames rigidly attached to wrist_3. Contact
        # with that same rigid mounting body is attachment contact, not collision.
        mounting_links = {self.links[link_name]}
        pending = list(mounting_links)
        while pending:
            for index in self.fixed_neighbors.get(pending.pop(), ()):
                if index not in mounting_links:
                    mounting_links.add(index)
                    pending.append(index)
        self.tools.append((body, self.links[link_name], as_plane(frame) if frame is not None else Plane.world_xy(),
                           {self.links[n] for n in touch_links} | mounting_links))
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

    @recorded(detail=True)
    def is_base_valid(self, base, *, clearance=0.0):
        """Check non-arm robot links against obstacles, without assigning arm joints.

        Internal assembly contacts are not part of this base-placement test.
        Moving arm links and attached tools are checked later by is_valid.
        """
        self.last_failure = None
        if clearance < 0 or not math.isfinite(clearance):
            raise ValueError('Clearance must be nonnegative')
        self.set_base(base)
        for obstacle_index, (obstacle, ignored) in enumerate(self.environment):
            for link in self.static_links - ignored:
                if self.p.getClosestPoints(self.robot, obstacle, clearance, linkIndexA=link):
                    self.last_failure = 'base collision: {} / collision_meshes[{}]'.format(
                        self.link_names[link], obstacle_index)
                    return False
        return True

    def configuration_cache_key(self, configuration):
        """Physical-pose key for one target's collision checks, preserving limits."""
        q = list(configuration)
        if len(q) != len(self.joints) or any(lo <= hi and not lo <= v <= hi
                for v, (lo, hi) in zip(q, self._joint_limits)):
            return 'out_of_range', tuple(q)
        return tuple(round((v+math.pi) % (2*math.pi)-math.pi, 10)
                     if kind == self.p.JOINT_REVOLUTE else v for v, kind in zip(q, self._joint_types))

    def configuration_group_cache_key(self, alternatives):
        """Validate an entire Cartesian product of equivalent bounded joint turns."""
        if len(alternatives) != len(self.joints) or any(not values for values in alternatives):
            return None
        for values, (lo, hi), kind in zip(alternatives, self._joint_limits, self._joint_types):
            if any(not math.isfinite(v) or (lo <= hi and not lo <= v <= hi) for v in values):
                return None
            for v in values[1:]:
                turns = (v-values[0])/(2*math.pi)
                if kind != self.p.JOINT_REVOLUTE or abs(turns-round(turns)) > 1e-12:
                    return None
        return self.configuration_cache_key([values[0] for values in alternatives])

    @staticmethod
    def _bounds_overlap(a, b, clearance):
        # Conservative broad-phase rejection only; narrow-phase remains authoritative.
        margin = clearance + 1e-9
        return all(a[0][i] <= b[1][i]+margin and b[0][i] <= a[1][i]+margin for i in range(3))

    @recorded(detail=True)
    def is_valid(self, configuration, base=None, *, clearance=0.0):
        self.last_failure = None
        if self.robot is None:
            raise ValueError('Load a robot before checking collisions')
        q = list(configuration)
        if len(q) != len(self.joints) or not all(math.isfinite(v) for v in q):
            raise ValueError('Configuration must match explicitly selected URDF joints')
        if clearance < 0 or not math.isfinite(clearance):
            raise ValueError('Clearance must be nonnegative')
        for name, value, (lo, hi) in zip(self.joint_names, q, self._joint_limits):
            if lo <= hi and not lo <= value <= hi:
                self.last_failure = 'joint limit: {} ({} outside [{}, {}])'.format(name, value, lo, hi)
                return False
        if base is not None:
            self.set_base(base)
        p = self.p
        for index, value in zip(self.joints, q):
            p.resetJointState(self.robot, index, value)
        self._update_tools()
        # Contact generation uses Bullet's broadphase. getClosestPoints(body,
        # body) instead tests every link pair, including distant/static ones.
        if clearance == 0:
            p.performCollisionDetection()
            contacts = p.getContactPoints(self.robot, self.robot)
        else:
            contacts = p.getClosestPoints(self.robot, self.robot, clearance)
        for contact in contacts:
            if contact[8] > clearance:
                continue
            if frozenset((contact[3], contact[4])) in self._self_pair_keys:
                self.last_failure = 'self collision: {} / {}'.format(self.link_names[contact[3]], self.link_names[contact[4]])
                return False
        link_bounds = {link: p.getAABB(self.robot, link) for link in self.collision_links}
        if self.environment:
            bounds = list(link_bounds.values()) or [p.getAABB(self.robot, -1)]
            robot_bounds = (tuple(min(b[0][i] for b in bounds) for i in range(3)),
                            tuple(max(b[1][i] for b in bounds) for i in range(3)))
            tool_bounds = [p.getAABB(body) for body, _, _, _ in self.tools]
        for obstacle_index, (obstacle, ignored) in enumerate(self.environment):
            # Query current bounds so external scene movement cannot leave a stale cache.
            obstacle_bounds = p.getAABB(obstacle)
            contacts = (p.getClosestPoints(self.robot, obstacle, clearance)
                        if self._bounds_overlap(robot_bounds, obstacle_bounds, clearance) else ())
            for c in contacts:
                if c[3] not in ignored:
                    self.last_failure = 'environment collision: {} / collision_meshes[{}]'.format(self.link_names[c[3]], obstacle_index)
                    return False
            for tool_index, (body, _, _, _) in enumerate(self.tools):
                if (self._bounds_overlap(tool_bounds[tool_index], obstacle_bounds, clearance)
                        and p.getClosestPoints(body, obstacle, clearance)):
                    self.last_failure = 'tool collision: tool[{}] / collision_meshes[{}]'.format(tool_index, obstacle_index)
                    return False
        for tool_index, (body, _, _, touch) in enumerate(self.tools):
            bounds = p.getAABB(body)
            for link in self.collision_links:
                if link in touch or not self._bounds_overlap(bounds, link_bounds[link], clearance):
                    continue
                if p.getClosestPoints(body, self.robot, clearance, linkIndexB=link):
                    self.last_failure = 'tool collision: tool[{}] / {}'.format(tool_index, self.link_names[link])
                    return False
        return True

    @recorded(detail=True)
    def edge_is_valid(self, q0, base0, q1, base1, *, joint_resolution=0.05, base_resolution=0.02,
                      yaw_resolution=0.05, periodic=None, clearance=0.0):
        """Sample a synchronized arm/base edge, including both endpoints.

        This is resolution-dependent collision checking, not a continuous proof.
        Base planes must be upright; yaw is interpolated along the shortest arc.
        """
        if any(not math.isfinite(v) or v <= 0 for v in (joint_resolution, base_resolution, yaw_resolution)):
            raise ValueError('Positive sampling resolutions required')
        b0, b1 = as_plane(base0), as_plane(base1)
        same_rotation = np.allclose(b0.matrix[:3, :3], b1.matrix[:3, :3], atol=1e-10, rtol=0)
        if not same_rotation and (not np.allclose(b0.zaxis, [0, 0, 1]) or not np.allclose(b1.zaxis, [0, 0, 1])):
            raise ValueError('Mobile edge interpolation requires upright base planes')
        q0, q1 = np.asarray(q0, dtype=float), np.asarray(q1, dtype=float)
        if q0.shape != (len(self.joints),) or q1.shape != q0.shape or not np.isfinite(q0).all() or not np.isfinite(q1).all():
            raise ValueError('Edge endpoints must match the selected joints and be finite')
        delta = q1-q0
        if periodic is not None:
            mask = np.asarray(periodic, dtype=bool)
            if mask.shape != delta.shape:
                raise ValueError('Periodic mask must match the selected joints')
            delta[mask] = (delta[mask]+np.pi) % (2*np.pi)-np.pi
        y0, y1 = math.atan2(b0.xaxis[1], b0.xaxis[0]), math.atan2(b1.xaxis[1], b1.xaxis[0])
        dy = (y1-y0+math.pi) % (2*math.pi)-math.pi
        steps = max(1, math.ceil(np.max(np.abs(delta))/joint_resolution) if len(delta) else 1,
                    math.ceil(np.linalg.norm(b1.origin-b0.origin)/base_resolution), math.ceil(abs(dy)/yaw_resolution))
        for t in np.linspace(0, 1, steps+1):
            yaw = y0+t*dy
            base = Plane((1-t)*b0.origin+t*b1.origin,
                         b0.xaxis if same_rotation else (math.cos(yaw), math.sin(yaw), 0),
                         b0.yaxis if same_rotation else (-math.sin(yaw), math.cos(yaw), 0))
            if not self.is_valid(q0+t*delta, base, clearance=clearance):
                return False
        return True

    @recorded
    def close(self):
        if self.p.isConnected():
            self.p.disconnect()
        self._temporary.cleanup()

    disconnect = close

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
