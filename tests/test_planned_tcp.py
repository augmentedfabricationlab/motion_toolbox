import json
import math

import numpy as np

from motion_toolbox.geometry import Plane
from motion_toolbox.planning import calculate_partial_trajectory


def test_selected_tcp_tracks_rotation_through_filtering_and_graph(tmp_path):
    targets = [Plane((i*.1, .2, 1.), (1,0,0), (0,1,0)) for i in range(3)]
    def solver(target, base):
        angle = math.atan2(target.xaxis[1], target.xaxis[0]) % (2*math.pi)
        return [[angle]]
    output = tmp_path/'trajectory.json'
    result = calculate_partial_trajectory(None, targets, ik_solver=solver,
        rotation_mode='n_steps', rotation_steps=4, joint_ranges=[[0, math.pi]],
        collision=lambda q,b: q[0] > .1, max_joint_step=1,
        ik_solutions_output_path=output)
    np.testing.assert_allclose(result['configurations'], [[math.pi/2]]*3)
    np.testing.assert_allclose(result['selected_tcp_rotations'], [math.pi/2]*3)
    for selected, original in zip(result['selected_target_planes'], targets):
        np.testing.assert_allclose(selected.matrix, original.rotated_z(math.pi/2).matrix)
    assert len(json.loads(output.read_text())['selected_target_planes']) == 3


def test_disconnected_graph_has_no_planned_tcp():
    targets = [Plane((i,0,0),(1,0,0),(0,1,0)) for i in range(2)]
    result = calculate_partial_trajectory(None, targets,
        ik_solver=lambda t,b: [[t.origin[0]]], max_joint_step=.1)
    assert result['configurations'] == []
    assert result['selected_target_planes'] == []
    assert result['selected_tcp_rotations'] == []
