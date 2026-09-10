"""COMPAS FAB 1.x / mobile_robot_control adapters, imported on demand."""
from pathlib import Path
import copy
import xml.etree.ElementTree as ET
from .geometry import as_plane, Plane, to_compas


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


def export_robot(robot, directory, package_paths=None, asset_root=None):
    """Serialize a copy of robot.model and its loaded meshes for PyBullet.

    No paths or tool catalogs are inferred from sibling repositories. Unresolved
    package:// resources require a package_paths mapping, or loaded model meshes.
    Collision and visual origins/scales remain in the URDF. Tool collision mesh
    frames are link-local; the TCP offset is deliberately not applied to meshes.
    """
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    model = copy.deepcopy(robot.model)
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
                            vertices, faces = mesh.to_vertices_and_faces()
                            for v in vertices:
                                f.write('v %s %s %s\n' % tuple(v))
                            for face in faces:
                                f.write('f ' + ' '.join(str(i+count+1) for i in face) + '\n')
                            count += len(vertices)
                    shape.filename = str(path)
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
                    shape.filename = str(path.resolve())
    urdf_path = directory / 'robot.urdf'
    model.to_urdf_file(str(urdf_path))
    # Materials may contain unrelated texture paths that Bullet tries to load.
    tree = ET.parse(urdf_path)
    for material in tree.iter('material'):
        for texture in list(material.findall('texture')):
            material.remove(texture)
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


def kinematics_from_robot(robot, *, parameters=None, arm_in_base=None, group=None):
    """Use active MultiTool TCP and explicit or offline robot arm calibration.

    Does not access robot.RCF, because that property can initiate ROS traffic.
    If calibration is unavailable, require it instead of assuming a robot height.
    """
    from .kinematics.solver import URKinematics, UR20
    if arm_in_base is None:
        calibration = getattr(robot, '_RCF', None)
        if calibration is None:
            raise ValueError('Provide arm_in_base or initialize robot._RCF offline')
        calibration = as_plane(calibration)
        origin = calibration.origin.copy()
        origin[2] += float(getattr(robot, 'lift_height', 0.0))
        arm_in_base = Plane(origin, calibration.xaxis, calibration.yaxis)
    tool = _active_tool(robot, group)
    return URKinematics(parameters=parameters if parameters is not None else UR20,
                        tool=tool.frame if tool is not None else None, arm_in_base=arm_in_base)


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


def tcp_from_joints(robot, configurations, *, tool=None, group=None):
    """Offline FK in the model frame, then apply the chosen TCP offset."""
    tool = tool if tool is not None else _active_tool(robot, group)
    tcp = as_plane(tool.frame) if tool is not None else Plane.world_xy()
    flange_link = robot.get_end_effector_link_name(group)
    frames = []
    for configuration in configurations:
        # model FK is local/offline, unlike robot FK which can select a backend.
        frame = robot.model.forward_kinematics(configuration, flange_link)
        frames.append(Plane.from_matrix(as_plane(frame).matrix @ tcp.matrix))
    return frames
