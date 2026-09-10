"""Small standalone demonstration of the two different base-planning problems."""
from motion_toolbox.geometry import Plane
from motion_toolbox.base_planning import grid_bases, find_stationary_base, plan_mobile_base
from motion_toolbox.kinematics.solver import URKinematics
from motion_toolbox.kinematics.ur import forward_kinematics

start = [.4, -1.3, 1.1, -.6, .9, .7]
targets = [forward_kinematics([start[0]+i*.01]+start[1:]) for i in range(5)]
domain = grid_bases([-.05, 0, .05], [0], [0])
solver = URKinematics()
fixed = find_stationary_base(targets, domain, start, ik_solver=solver)
moving = plan_mobile_base(targets, [domain]*len(targets), ik_solver=solver,
                         current_pose=start, start_base=Plane.world_xy(), max_base_step=.1)
assert len(fixed.base_planes) == 1
assert len(moving.base_planes) == len(targets)
print('Stationary:', fixed.base_plane.to_dict())
print('Mobile:', len(moving.base_planes), 'base planes')
