"""Paste into a Rhino 8 Python 3 component after installing the packages.

Inputs: robot, target_planes, base_planes, current_pose, arm_in_base,
        arm_joint_names, collision_meshes, model_units_to_metres, run.
Outputs: joint_plan, base_result, status. For long-lived worlds use an explicitly
owned PybulletServer and close/rebuild it when robot or environment changes.
"""
from motion_toolbox.geometry import as_plane
from motion_toolbox.robot_adapter import kinematics_from_robot
from motion_toolbox.collision import PybulletServer
from motion_toolbox.planning import calculate_partial_trajectory


def plan(robot, targets, bases, current_pose, arm_in_base, arm_joint_names,
         collision_meshes=(), model_units_to_metres=1.0):
    targets = [as_plane(p, model_units_to_metres) for p in targets]
    bases = [as_plane(p, model_units_to_metres) for p in bases]
    solver = kinematics_from_robot(robot, arm_in_base=as_plane(arm_in_base, model_units_to_metres))
    with PybulletServer(robot=robot, joint_names=arm_joint_names) as world:
        for mesh in collision_meshes:
            world.add_mesh(mesh, scale=model_units_to_metres)
        return calculate_partial_trajectory(current_pose, targets, base_planes=bases,
            ik_solver=solver, collision=world.is_valid, transition_check=world.edge_is_valid,
            rotation_mode='n_steps', rotation_steps=24)
