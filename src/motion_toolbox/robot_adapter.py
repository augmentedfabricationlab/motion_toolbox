"""COMPAS FAB 1.x / mobile_robot_control adapters, imported on demand."""
from motion_toolbox.recording import recorded
from pathlib import Path
import xml.etree.ElementTree as ET
from .geometry import as_plane, Plane, to_compas


STATIONARY_ADAPTER_VERSION = 4


DEFAULT_ARM_JOINT_NAMES = (
    'robot_arm_shoulder_pan_joint',
    'robot_arm_shoulder_lift_joint',
    'robot_arm_elbow_joint',
    'robot_arm_wrist_1_joint',
    'robot_arm_wrist_2_joint',
    'robot_arm_wrist_3_joint',
)


def _active_tool(robot, group=None):
    tools = getattr(robot, 'attached_tools', {})
    if group is not None:
        return tools.get(group)
    if len(tools) == 1:
        return next(iter(tools.values()))
    if not tools:
        return None
    if getattr(robot, 'semantics', None) is not None:
        return robot.attached_tool
    raise ValueError('Specify group when multiple tools are attached without semantics')


def resolve_arm_joint_names(robot=None, names=None, *, group=None):
    """Resolve six UR arm joints in chain order, excluding lift and side branches.

    Prefer the active tool attachment or semantic end link. Without either,
    require exactly one distinct six-revolute-joint chain in the model.
    Explicit names remain available for models with additional rotary axes.
    """
    selected = list(names) if names is not None else []
    if robot is None or getattr(robot, 'model', None) is None:
        selected = selected or list(DEFAULT_ARM_JOINT_NAMES)
        if len(selected) != 6 or len(set(selected)) != 6:
            raise ValueError('Expected six distinct arm joint names')
        return selected
    model = robot.model
    joints = {j.name: j for j in model.get_configurable_joints()}
    if not selected:
        tool = _active_tool(robot, group)
        end = getattr(tool, 'connected_to', None)
        if end is None and getattr(robot, 'semantics', None) is not None:
            end = robot.get_end_effector_link_name(group)
        def chain(link):
            return tuple(j.name for j in model.iter_joint_chain(link_end_name=link)
                         if j.name in joints and j.type in (0, 1))
        if end is not None:
            selected = list(chain(end))
        else:
            chains = {chain(link.name) for link in model.links if not link.joints}
            chains = {c for c in chains if len(c) == 6}
            if len(chains) != 1:
                raise ValueError('Cannot identify a unique six-joint arm; specify group or arm_joint_names')
            selected = list(next(iter(chains)))
    if len(selected) != 6 or len(set(selected)) != 6 or any(n not in joints or joints[n].type not in (0, 1) for n in selected):
        raise ValueError('Expected six distinct revolute arm joints; specify arm_joint_names in UR analytic order')
    return selected


@recorded
def export_robot(robot, directory, package_paths=None, asset_root=None):
    """Export robot.model and loaded meshes without copying native CAD objects.

    No paths or tool catalogs are inferred from sibling repositories. Unresolved
    package:// resources require a package_paths mapping, or loaded model meshes.
    Collision and visual origins/scales remain in the URDF. Tool collision mesh
    frames are link-local; the TCP offset is deliberately not applied to meshes.
    """
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    model = robot.model
    # Scene/artist caches can contain Rhino meshes that cannot be deep-copied
    # or pickled. Patch a separate XML tree instead of cloning the robot model.
    tree = ET.ElementTree(ET.fromstring(model.to_urdf_string()))
    xml_joints = {joint.attrib['name']: joint for joint in tree.getroot().findall('joint')}
    # COMPAS serialization can drop numeric zeros, including lower="0".
    # Restore values from the supplied model, without inventing missing limits.
    for joint in model.joints:
        if joint.limit is None:
            continue
        element = xml_joints[joint.name].find('limit')
        if element is None:
            element = ET.SubElement(xml_joints[joint.name], 'limit')
        for name in ('lower', 'upper', 'effort', 'velocity'):
            value = getattr(joint.limit, name, None)
            if value is not None:
                element.set(name, str(value))
    xml_links = {link.attrib['name']: link for link in tree.getroot().findall('link')}
    for link_index, link in enumerate(model.links):
        for kind in ('visual', 'collision'):
            for item_index, item in enumerate(getattr(link, kind)):
                shape = item.geometry.shape
                if not hasattr(shape, 'filename'):
                    continue
                meshes = getattr(shape, 'meshes', None)
                if meshes:
                    path = directory / ('mesh_%s_%s_%s.obj' % (link_index, kind, item_index))
                    meshes = meshes if isinstance(meshes, (list, tuple)) else [meshes]
                    with path.open('w', encoding='utf-8') as f:
                        count = 0
                        for mesh in meshes:
                            if hasattr(mesh, 'to_vertices_and_faces'):
                                vertices, faces = mesh.to_vertices_and_faces()
                            elif hasattr(mesh, 'Vertices') and hasattr(mesh, 'Faces'):
                                vertices = [[v.X, v.Y, v.Z] for v in mesh.Vertices]
                                faces = [[f.A, f.B, f.C, f.D] if f.IsQuad else [f.A, f.B, f.C] for f in mesh.Faces]
                            else:
                                raise TypeError('Robot mesh must be a COMPAS or Rhino mesh')
                            import math
                            if len(vertices) == 0 or len(faces) == 0:
                                raise ValueError('Empty {} mesh on robot link {}'.format(kind, link.name))
                            if any(len(v) != 3 or not all(math.isfinite(float(x)) for x in v) for v in vertices):
                                raise ValueError('Invalid mesh vertices on robot link ' + link.name)
                            if any(len(face) < 3 or any(int(i) != i or i < 0 or i >= len(vertices) for i in face) for face in faces):
                                raise ValueError('Invalid mesh faces on robot link ' + link.name)
                            for v in vertices:
                                f.write('v %s %s %s\n' % tuple(v))
                            for face in faces:
                                f.write('f ' + ' '.join(str(i+count+1) for i in face) + '\n')
                            count += len(vertices)
                else:
                    name = shape.filename
                    if name.startswith('package://'):
                        package, relative = name[len('package://'):].split('/', 1)
                        if not package_paths or package not in package_paths:
                            raise ValueError('Unresolved URDF package: ' + package)
                        path = Path(package_paths[package]) / relative
                    elif name.startswith('file://'):
                        path = Path(name[len('file://'):])
                    else:
                        path = Path(name)
                        if not path.is_absolute():
                            if asset_root is None:
                                raise ValueError('Relative mesh paths require asset_root: ' + name)
                            path = Path(asset_root) / path
                    if not path.is_file():
                        raise FileNotFoundError(path)
                mesh_element = xml_links[link.name].findall(kind)[item_index].find('geometry/mesh')
                if mesh_element is None:
                    raise ValueError('Missing URDF mesh element for link: ' + link.name)
                mesh_element.set('filename', path.resolve().as_posix())
    urdf_path = directory / 'robot.urdf'
    # Materials may contain unrelated texture paths that Bullet tries to load.
    for material in tree.iter('material'):
        for texture in list(material.findall('texture')):
            material.remove(texture)
    # Bullet's native reader can split very long XML lines inside filenames.
    ET.indent(tree, space='  ')
    tree.write(urdf_path, encoding='utf-8', xml_declaration=True)
    attached = []
    tools = getattr(robot, 'attached_tools', None)
    if tools is None:
        tool = getattr(robot, 'attached_tool', None)
        tools = {'main': tool} if tool is not None else {}
    for tool in tools.values():
        for acm in tool.attached_collision_meshes:
            cm = acm.collision_mesh
            attached.append(dict(mesh=cm.mesh, link_name=acm.link_name,
                                 frame=cm.frame, touch_links=acm.touch_links))
    return urdf_path, attached


def _mount_from_urdf(robot, group, fixed_joint_values, arm_joint_names):
    """Resolve a UR controller 'base' link without querying robot.RCF/ROS."""
    names = resolve_arm_joint_names(robot, arm_joint_names, group=group)
    suffix = 'shoulder_pan_joint'
    controller_link = names[0][:-len(suffix)] + 'base' if names[0].endswith(suffix) else None
    model = robot.model
    if controller_link is None or model.get_link_by_name(controller_link) is None:
        raise ValueError('Provide arm_in_base or initialize robot._RCF offline; '
                         'cannot identify the UR controller base link in this model')
    chain = list(model.iter_joint_chain(link_end_name=controller_link))
    if any(j.name in names for j in chain):
        raise ValueError('UR controller base link must be upstream of the arm joints')
    movable = [j for j in chain if j.type != 3]
    values = dict(fixed_joint_values)
    # MobileRobot.lift_height refers to the single upstream prismatic lift.
    lifts = [j for j in movable if j.type == 2]
    for joint in movable:
        if joint.name not in values:
            if len(lifts) == 1 and joint.type == 2 and hasattr(robot, 'lift_height'):
                values[joint.name] = float(robot.lift_height)
            else:
                raise ValueError('Provide fixed_joint_values for upstream mounting joint: ' + joint.name)
        import math
        value = values[joint.name]
        if not math.isfinite(value) or (joint.limit is not None and
            ((joint.limit.lower is not None and value < joint.limit.lower) or
             (joint.limit.upper is not None and value > joint.limit.upper))):
            raise ValueError('Invalid upstream mounting joint value: ' + joint.name)
    return as_plane(model.forward_kinematics(values, controller_link)), values, controller_link


@recorded
def kinematics_from_robot(robot, *, parameters=None, arm_in_base=None, group=None,
                          fixed_joint_values=None, arm_joint_names=None):
    """Use active MultiTool TCP and explicit or offline robot arm calibration.

    Does not access robot.RCF, because that property can initiate ROS traffic.
    If calibration is unavailable, use the recognized UR controller base link
    and configured upstream joints from the model. Never assume an identity mount.
    """
    from .kinematics.solver import URKinematics, UR20
    fixed = dict(fixed_joint_values or {})
    source = 'explicit arm_in_base'
    if arm_in_base is None:
        calibration = getattr(robot, '_RCF', None)
        if calibration is None:
            arm_in_base, fixed, link = _mount_from_urdf(robot, group, fixed, arm_joint_names)
            source = 'URDF controller link ' + link
        else:
            calibration = as_plane(calibration)
            origin = calibration.origin.copy()
            origin[2] += float(getattr(robot, 'lift_height', 0.0))
            arm_in_base = Plane(origin, calibration.xaxis, calibration.yaxis)
            source = 'offline robot._RCF + lift_height'
    tool = _active_tool(robot, group)
    solver = URKinematics(parameters=parameters if parameters is not None else UR20,
                        tool=tool.frame if tool is not None else None, arm_in_base=arm_in_base)
    solver.fixed_joint_values = fixed
    solver.mounting_source = source
    return solver


@recorded
def attach_tool(robot, visual, tcp, *, collision=None, name='tool', group=None, touch_links=None):
    """Attach caller-owned geometry and TCP; no project-specific tool catalog."""
    from compas_fab.robots import Tool
    tool = Tool(visual, to_compas(tcp), collision=collision, name=name)
    robot.attach_tool(tool, group=group, touch_links=touch_links)
    return tool


def configuration_from_values(values, joint_names, joint_types):
    from compas_robots import Configuration
    if len(values) != len(joint_names) or len(values) != len(joint_types):
        raise ValueError('Joint values, names and types must have equal length')
    return Configuration(list(values), list(joint_types), list(joint_names))


@recorded
def tcp_from_joints(robot, configurations, *, tool=None, group=None, flange_link=None):
    """Offline FK in the model frame, then apply the chosen TCP offset."""
    tool = tool if tool is not None else _active_tool(robot, group)
    tcp = as_plane(tool.frame) if tool is not None else Plane.world_xy()
    if flange_link is None:
        flange_link = getattr(tool, 'connected_to', None) if tool is not None else None
    if flange_link is None:
        flange_link = robot.get_end_effector_link_name(group) if getattr(robot, 'semantics', None) is not None else robot.model.get_end_effector_link().name
    frames = []
    for configuration in configurations:
        # model FK is local/offline, unlike robot FK which can select a backend.
        frame = robot.model.forward_kinematics(configuration, flange_link)
        frames.append(Plane.from_matrix(as_plane(frame).matrix @ tcp.matrix))
    return frames
