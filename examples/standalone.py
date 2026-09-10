"""Run after installing motion-toolbox. No Rhino, ROS or robot assets required."""
from motion_toolbox.geometry import Plane
from motion_toolbox.kinematics.ur import forward_kinematics
from motion_toolbox.planning import calculate_partial_trajectory

start = [.4, -1.3, 1.1, -.6, .9, .7]
targets = [forward_kinematics([start[0]+i*.01]+start[1:]) for i in range(10)]
result = calculate_partial_trajectory(start, targets, rotation_mode='n_steps', rotation_steps=8)
assert len(result['configurations']) == len(targets)
print('Planned', len(targets), 'targets; cost', result['path_length'])
