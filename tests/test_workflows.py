import json
from motion_toolbox.utilities.static_planning import run_all, run_job
from motion_toolbox.utilities.edit import edit_solutions
from motion_toolbox.utilities.io import export_planes, load_planes, select_ik_solution
from motion_toolbox.geometry import Plane


def test_run_all_edit_collision_and_optimum():
    result = run_all([[[.1, 0], [.3, 0]], [[.2, 0], [.4, 0]]], [0, 0],
                     collision=lambda q, b: q[0] != .1, tolerance=1)
    assert result['configurations'] == [[.3, 0], [.2, 0]]
    assert edit_solutions([[[0, 0, 3]]], [0, 0, 0], tolerance=1) == [[]]


def test_job_and_json(tmp_path):
    path = tmp_path/'bases.json'
    export_planes([Plane.world_xy()], path, metadata={'name': 'demo'})
    assert len(load_planes(path)) == 1
    job = dict(solutions=[[[.1]], [[.2]]], bases='bases.json', current_pose=[0])
    assert len(run_job(job, root=tmp_path)['configurations']) == 2
    assert select_ik_solution([], 0) is None
    assert select_ik_solution([[], [[1]]], 100) == [1]
