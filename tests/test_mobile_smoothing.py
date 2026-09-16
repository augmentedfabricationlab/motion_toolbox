import numpy as np
from motion_toolbox.base_planning import BasePlan
from motion_toolbox.geometry import Plane
from motion_toolbox.mobile_smoothing import smooth_mobile_solution


class Refiner:
    _arm_inverse = np.eye(4)
    _tcp_to_flange = np.eye(4)
    def refine(self,target,seed):
        return list(seed)


def plan():
    bases = [Plane((x,0,0),(1,0,0),(0,1,0)) for x in (0,.2,.4,.2,0)]
    return BasePlan(bases,[[0.] for _ in bases],.8,[1]*5,[])


def test_smoothing_revalidates_every_configuration_and_edge():
    original = plan()
    seen,edges = [],[]
    result = smooth_mobile_solution([Plane.world_xy()]*5,original,windows=[3],ik_solver=Refiner(),
        collision=lambda q,b: seen.append(b) or True,
        transition_check=lambda *args: edges.append(args) or True,
        periodic=np.array([False]),joint_weights=np.array([1.]))
    assert len(seen)==5 and len(edges)==4
    assert len(result.configurations)==5
    assert result.diagnostics[-1]['selected_window']==3


def test_rejected_smoothing_keeps_complete_original_path():
    original = plan()
    result = smooth_mobile_solution([Plane.world_xy()]*5,original,windows=[3],ik_solver=Refiner(),
        collision=lambda q,b: not .04<b.origin[0]<.1)
    assert result is original
    assert len(result.configurations)==5
    assert result.diagnostics[-1]['selected_window'] is None


def test_smoothing_does_not_ignore_start_speed_constraint():
    original = plan()
    result = smooth_mobile_solution([Plane.world_xy()]*5,original,windows=[3],ik_solver=Refiner(),
        current_pose=[0.],start_base=Plane.world_xy(),time_intervals=[1.]*5,max_base_speed=.01)
    assert result is original
    assert result.diagnostics[-1]['trials'][0]['failure']['rejection_counts']['base_speed']==1
