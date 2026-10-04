import numpy as np
import pytest
from motion_toolbox.geometry import Plane
from motion_toolbox.adaptive_stationary import find_adaptive_stationary_base, target_probes, polygon_grid
from motion_toolbox.execution import PlanningCancelled
from motion_toolbox.graph import shortest_path


def plane(x=0):return Plane((x,0,0),(1,0,0),(0,1,0))


def arguments(solver, **options):
    return dict(targets=[plane(0),plane(1)],ik_solver=solver,arm_in_base=plane(),
                candidate_planes=[plane(0),plane(1)],rotation_steps=4,probe_count=1,
                probe_rotations=1,validation_probe_count=1,connected_finalists=2,max_full_checks=2,max_joint_step=.5,**options)


def test_probes_cover_extremes_deterministically_and_polygon_grid_inside():
    targets=[plane(i) for i in range(30)]
    probes=target_probes(targets,8)
    assert len(probes)==len(set(probes))==8
    assert 0 in probes and 29 in probes
    assert probes==target_probes(targets,8)
    points=polygon_grid([[0,0],[2,0],[1,2]],5)
    assert len(points)==25
    assert all(y/2<x<2-y/2 for x,y in points)


def test_disconnected_candidate_teaches_transition_then_returns_connected_base():
    def solver(t,b):
        return [[float(t.origin[0])*(2 if b.origin[0]==0 else .2)]]
    calls=[]
    def collision(q,b):calls.append((tuple(q),tuple(b.origin)));return True
    out=find_adaptive_stationary_base(**arguments(solver,collision=collision))
    assert out['found'].base_plane.origin[0]==1
    assert out['found'].configurations==[[0.],[.2]]
    assert out['learned_bottlenecks']==[(0,1)]
    assert out['stats']['full_checks']==2
    assert out['counts_complete'] is False and out['found'].candidate_counts==[]
    assert out['found'].ik_option_count is None
    assert out['stats']['ik_cache_hits']>0 and out['stats']['collision_cache_hits']>0
    assert out['found'].cost==pytest.approx(.2)


def test_clearance_preference_is_not_a_hard_rejection_and_supplied_bases_are_exclusive():
    def solver(t,b):return [[float(t.origin[0])*.1]]
    options=arguments(solver,collision=lambda q,b:True,clearance_measure=lambda b:-100.)
    options['candidate_planes']=[plane(5)]
    out=find_adaptive_stationary_base(**options)
    assert out['found'].base_plane.origin[0]==5
    assert len(out['search_diagnostics'])==1
    assert out['found'].configurations
    assert out['search_diagnostics'][0]['clearance']==-100.


def test_best_validated_path_is_exact_for_requested_rotations():
    def solver(t,b):return [[float(t.origin[0])*(.2 if b.origin[0] else .4)],[.1+float(t.origin[0])*.3]]
    out=find_adaptive_stationary_base(**arguments(solver,collision=lambda q,b:q[0]!=.1))
    found=out['found']
    layers=[[q for q in solver(t,found.base_plane) if q[0]!=.1] for t in [plane(0),plane(1)]]
    exact=shortest_path(layers,max_step=.5,count_paths=False)
    assert found.configurations==exact.configurations
    assert found.cost==exact.cost
    assert len(found.selected_target_planes)==2
    assert len(found.selected_tcp_rotations)==2


def test_failed_probes_do_not_publish_partial_path_or_exact_counts():
    out=find_adaptive_stationary_base(**arguments(lambda t,b:[]))
    assert not out['found'].configurations
    assert not out['found'].base_planes
    assert out['status']=='search_budget_exhausted_without_connected_path'
    assert out['counts_complete'] is False


def test_final_validation_checks_targets_not_in_probes():
    def solver(t,b):return [[0.]] if t.origin[0]==0 else []
    out=find_adaptive_stationary_base(**arguments(solver,collision=lambda q,b:True))
    assert not out['found'].configurations
    assert out['stats']['full_checks']==0
    assert out['stats']['validation_attempts']==1
    assert out['learned_failed_targets']==[1]
    assert out['search_diagnostics'][1]['failed_priority_target']==1


def test_failed_target_is_learned_without_excluding_a_different_viable_base():
    def solver(t,b):return [[0.]] if t.origin[0]==0 or b.origin[0]==1 else []
    out=find_adaptive_stationary_base(**arguments(solver,collision=lambda q,b:True))
    assert out['learned_failed_targets']==[1]
    assert out['found'].base_plane.origin[0]==1
    assert out['found'].configurations==[[0.],[0.]]
    assert out['stats']['full_checks']==1
    assert out['stats']['validation_attempts']==2


def test_base_collision_and_starting_collision_block_before_probe():
    out=find_adaptive_stationary_base(**arguments(lambda t,b:pytest.fail('IK should not run'),
        base_collision=lambda b:False))
    assert not out['found'].configurations
    assert out['stats']['ik_calls']==0
    out=find_adaptive_stationary_base(**arguments(lambda t,b:pytest.fail('IK should not run'),
        current_pose=[0.],collision=lambda q,b:False))
    assert not out['found'].configurations and out['stats']['ik_calls']==0
    assert 'starting configuration' in out['found'].initial_state_failure


def test_cancellation_and_invalid_start():
    with pytest.raises(PlanningCancelled):
        find_adaptive_stationary_base(**arguments(lambda t,b:[[0.]],cancel_check=lambda:True))
    with pytest.raises(ValueError,match='Starting configuration'):
        find_adaptive_stationary_base(**arguments(lambda t,b:[[0.]],current_pose=[2.],joint_ranges=[[-1,1]]))


def test_build_path_false_only_proves_full_reachability():
    out=find_adaptive_stationary_base(**arguments(lambda t,b:[[0.]],build_path=False))
    assert out['found'].base_planes and not out['found'].configurations
    assert out['found'].path_search_count==0


def test_caches_do_not_cross_calls_with_changed_scene():
    first=find_adaptive_stationary_base(**arguments(lambda t,b:[[0.]],collision=lambda q,b:True))
    second=find_adaptive_stationary_base(**arguments(lambda t,b:[[0.]],collision=lambda q,b:False))
    assert first['found'].configurations and not second['found'].configurations


def test_disconnected_reachable_base_survives_later_unreachable_candidate():
    def solver(t,b):
        if b.origin[0]==1 and t.origin[0]==1:return []
        return [[float(t.origin[0])*2]]
    out=find_adaptive_stationary_base(**arguments(solver))
    assert out['found'].base_planes and not out['found'].configurations
    assert out['found'].disconnected_detail['to_target']==1


def test_start_cost_and_per_target_speed_limits_are_preserved():
    options=arguments(lambda t,b:[[.2+float(t.origin[0])*.2]],current_pose=[0.],
                      step_limits=[[.3],[.1]])
    out=find_adaptive_stationary_base(**options)
    assert not out['found'].configurations
    assert out['learned_bottlenecks']==[(0,1)]
    options['step_limits']=[[.3],[.3]]
    out=find_adaptive_stationary_base(**options)
    assert out['found'].cost==pytest.approx(.4)
    assert out['found'].configurations==[[.2],[.4]]


def test_nearly_identical_proposals_do_not_consume_two_full_validations():
    options=arguments(lambda t,b:[[0.]])
    options['candidate_planes']=[plane(2),plane(2+1e-12)]
    out=find_adaptive_stationary_base(**options)
    assert out['candidate_count']==out['stats']['full_checks']==1


def test_probe_scoring_uses_collision_free_options_not_only_raw_ik_counts():
    def solver(t,b):return [[0.],[.1],[.2],[.3]]
    def collision(q,b):return b.origin[0]==1 or q==[0.]
    options=arguments(solver,collision=collision)
    options['max_full_checks']=1
    out=find_adaptive_stationary_base(**options)
    assert out['found'].base_plane.origin[0]==1
    rows=out['search_diagnostics']
    assert rows[0]['probe_min_ik']==rows[1]['probe_min_ik']
    assert rows[0]['probe_estimated_min_free']<rows[1]['probe_estimated_min_free']


def test_footprint_preference_includes_every_static_collision_link():
    from motion_toolbox.adaptive_stationary import footprint_clearance
    class World:
        static_links={0,1,2}
        collision_links={0,1}
        p=None
        def set_base(self,base):self.base=base
        def getAABB(self,robot,link):
            return ((0,0,0),(1 if link==0 else 3,1,1))
    world=World();world.p=world;world.robot=0
    # Target +Z is +X, so the farthest static link reduces signed clearance.
    measure=footprint_clearance(world,[Plane((4,0,0),(0,1,0),(0,0,1))])
    assert measure(plane())==pytest.approx(1.)


def test_adaptive_error_restores_windows_cpu_policy(monkeypatch):
    from motion_toolbox import execution
    states=[]
    class Controller:
        def read(self):return (1,2,2)
        def write(self,state):states.append(state)
    monkeypatch.setattr(execution,'_controller',lambda:Controller())
    def fail(t,b):raise RuntimeError('solver failure')
    with pytest.raises(RuntimeError,match='solver failure'):
        find_adaptive_stationary_base(**arguments(fail))
    assert states==[(1,3,2),(1,2,2)]


def test_dense_screen_learns_failed_targets_without_consuming_full_check_budget():
    options=arguments(lambda t,b:[[0.]] if t.origin[0]==0 else [])
    options['validation_probe_count']=2
    out=find_adaptive_stationary_base(**options)
    assert out['stats']['full_checks']==0
    assert out['learned_failed_targets']==[1]
    assert out['found'].diagnostics[-1]['failed_target_details']['target_index']==1
    assert not out['found'].configurations


def test_dense_screen_completes_requested_rotations_before_rejecting_base():
    # The single sampled rotation fails at target 1, but a quarter turn works.
    def solver(t,b):
        return [[0.]] if t.origin[0]==0 or t.xaxis[1]>.9 else []
    options=arguments(solver)
    options['validation_probe_count']=2
    out=find_adaptive_stationary_base(**options)
    assert len(out['found'].configurations)==2
    assert out['found'].selected_tcp_rotations[1]==pytest.approx(np.pi/2)


@pytest.mark.parametrize('limit',[1,2])
def test_bounded_lazy_validation_preserves_exhaustive_optimum_and_ties(limit):
    layers=[[[j*.1] for j in range(8)] for _ in range(5)]
    allowed=lambda i,j:j>=i
    exact=shortest_path([[q for j,q in enumerate(layer) if allowed(i,j)]
                         for i,layer in enumerate(layers)],max_step=1.,count_paths=False)
    stats={}
    lazy=shortest_path(layers,max_step=1.,count_paths=False,node_valid=allowed,
                       max_lazy_passes=limit,stats=stats)
    assert lazy.configurations==exact.configurations
    assert lazy.cost==exact.cost
    assert stats['graph_solves']<=limit+1
    assert all(allowed(i,j) for i,j in enumerate(lazy.indices))


def test_bounded_lazy_validation_reports_original_disconnection_layer():
    layers=[[[0.],[1.]] for _ in range(3)]
    out=shortest_path(layers,count_paths=False,node_valid=lambda i,j:i!=1,
                      max_lazy_passes=1)
    assert not out.configurations and out.failure_layer==1


def test_fast_default_can_stop_at_first_connected_finalist():
    options=arguments(lambda t,b:[[float(t.origin[0])*.1]])
    options['connected_finalists']=1
    out=find_adaptive_stationary_base(**options)
    assert out['stats']['full_checks']==1 and out['found'].configurations


def test_integer_valued_json_numbers_are_accepted():
    options=arguments(lambda t,b:[[0.]])
    for name in ('rotation_steps','probe_count','probe_rotations','validation_probe_count','max_full_checks'):
        options[name]=float(options[name])
    out=find_adaptive_stationary_base(**options)
    assert out['found'].configurations


def test_joint_limit_failure_remains_distinct_from_missing_ik():
    out=find_adaptive_stationary_base(**arguments(lambda t,b:[[2.]],joint_ranges=[[-1.,1.]]))
    assert not out['found'].base_planes
    assert out['found'].diagnostics[-1]['reason']=='joint_limits'
