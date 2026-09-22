"""Robot-aligned cover boxes for the chassis/four wheels and configured lift."""
import xml.etree.ElementTree as ET
import numpy as np


def cover_boxes(p, path, root, mode, fixed_values, planned_joints):
    if mode not in ('detailed', 'auto', 'boxes'):
        raise ValueError('base_collision_model must be detailed, auto or boxes')
    metadata = dict(requested=mode, effective='detailed', boxes=[], omitted_links=[], replaced_links=[])
    if mode == 'detailed':
        return metadata
    links = {link.get('name'): link for link in root.findall('link')}
    prefixes = [name[:-len('chassis_link')] for name in links if name.endswith('chassis_link')]
    groups = None
    for prefix in prefixes:
        chassis = prefix+'chassis_link'
        lift = prefix+'ewellix_lift_base_link'
        arm = prefix+'arm_base_link'
        candidates = {chassis: [chassis]+[prefix+corner+'_wheel' for corner in
                      ('front_left', 'front_right', 'back_left', 'back_right')],
                      lift: [lift, prefix+'ewellix_lift_top_link']}
        if arm in links and all(name in links for group in candidates.values() for name in group):
            groups = candidates
            break
    if groups is None:
        if mode == 'boxes':
            raise ValueError('Cover boxes require chassis, four wheel, Ewellix lift and arm base links')
        return metadata
    # Preserve the entire arm subtree, including its fixed mounting geometry.
    keep = {arm}
    joints = root.findall('joint')
    for _ in range(len(joints)):
        expanded = keep | {j.find('child').get('link') for j in joints
                           if j.find('parent').get('link') in keep}
        if expanded == keep:
            break
        keep = expanded
    if planned_joints is None or any(j.get('name') in planned_joints and
            j.find('child').get('link') not in keep for j in joints):
        raise ValueError('Cover boxes require explicitly selected arm-only planning joints')
    body = p.loadURDF(path.as_posix(), useFixedBase=True)
    try:
        indices = {p.getBodyInfo(body)[0].decode(): -1}
        movable = {}
        for i in range(p.getNumJoints(body)):
            info = p.getJointInfo(body, i)
            indices[info[12].decode()] = i
            if info[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                movable[info[1].decode()] = i
        for name, value in fixed_values.items():
            if name not in movable or name in planned_joints or not np.isfinite(value):
                raise ValueError('Invalid fixed joint for cover bounds: '+name)
            info = p.getJointInfo(body, movable[name])
            if info[8] <= info[9] and not info[8] <= value <= info[9]:
                raise ValueError('Fixed joint value outside URDF limits: '+name)
            p.resetJointState(body, movable[name], value)
        inertia = p.getDynamicsInfo(body, -1)[3:5]
        def root_pose(position=(0,0,0), orientation=(0,0,0,1)):
            p.resetBasePositionAndOrientation(body, *p.multiplyTransforms(position, orientation, *inertia))
        for anchor, members in groups.items():
            root_pose()
            state = p.getLinkState(body, indices[anchor], computeForwardKinematics=True)
            root_pose(*p.invertTransform(state[4], state[5]))
            if any(not p.getCollisionShapeData(body, indices[name]) for name in members):
                raise ValueError('Missing collision geometry for cover bounds')
            bounds = np.asarray([p.getAABB(body, indices[name]) for name in members])
            low, high = bounds[:,0].min(axis=0)-1e-6, bounds[:,1].max(axis=0)+1e-6
            metadata['boxes'].append(dict(link=anchor, source_links=members,
                center=((low+high)/2).tolist(), size=(high-low).tolist()))
        metadata['fixed_joint_values'] = {name: float(fixed_values.get(name, 0.))
            for name in movable if name not in planned_joints}
    finally:
        p.removeBody(body)
    for name, link in links.items():
        if name not in keep:
            if link.findall('collision') and name not in groups:
                metadata['omitted_links'].append(name)
            for collision in link.findall('collision'):
                link.remove(collision)
    for box in metadata['boxes']:
        collision = ET.SubElement(links[box['link']], 'collision')
        ET.SubElement(collision, 'origin', xyz=' '.join(map(str,box['center'])), rpy='0 0 0')
        ET.SubElement(ET.SubElement(collision, 'geometry'), 'box', size=' '.join(map(str,box['size'])))
    metadata['effective'] = 'boxes'
    metadata['replaced_links'] = list(groups)
    return metadata
