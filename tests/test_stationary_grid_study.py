import numpy as np
import pytest
from motion_toolbox.geometry import Plane
from motion_toolbox.stationary_region import StationaryRegion
from validation import study_stationary_grid as study


def test_stratified_grid_reproducible_inside_polygon():
    polygon = [[0,0],[2,0],[1,2]]
    points = study.stratified_grid(polygon)
    assert len(points)==64
    assert points == study.stratified_grid(polygon)
    assert points != study.stratified_grid(polygon, seed=7)
    for x,y in points:
        assert 0<y<2 and y/2<x<2-y/2


def test_proposals_place_arm_at_requested_xy_with_mount_offset():
    mount=Plane((.275,.1,1.),(1,0,0),(0,1,0))
    region=StationaryRegion([Plane((0,1,1),(1,0,0),(0,0,-1))],mount,projected=True)
    proposals=study.propose([[0,0]],region,'coarse')
    assert len(proposals)==4 and len({p['id'] for p in proposals})==4
    for p in proposals:
        arm=region.metrics(p['base'])['arm_origin']
        assert np.allclose(arm,[0,0,1])


def test_score_coverage_first_and_keep_tradeoff():
    def row(id, coverage, minimum, mean):
        return dict(id=id, reachable_targets=coverage, minimum_solutions=minimum,
                    mean_solutions=mean,total_solutions=mean*10)
    a,b,c=row('a',10,2,20),row('b',10,4,10),row('c',9,0,200)
    ranked=study.score_rows([a,b,c])
    assert ranked[-1]['id']=='c'
    assert a['score']==pytest.approx(b['score'])
    assert study.score_rows([a,b,c],'total')[0]['id']=='a'
    assert study.score_rows([a,b,c],'minimum')[0]['id']=='b'


def test_refinement_can_cross_old_boundary():
    points=study.refinement_points([dict(arm_xy=[1.,1.])],np.array([.2,.3]))
    assert len(points)==9
    assert np.allclose(np.max(points,axis=0),[1.2,1.3])
    assert np.allclose(np.min(points,axis=0),[.8,.7])


def test_evaluate_keeps_zero_target_and_counts_all_remaining_targets(tmp_path,monkeypatch):
    class World:
        last_failure=None
        def is_base_valid(self,b,**kwargs):return True
        def is_valid(self,q,b,**kwargs):return q[0]>.5
    targets=[Plane((i,0,0),(1,0,0),(0,1,0)) for i in range(3)]
    def solver(target,base):
        return [[float(i),0,0,0,0,0] for i in range(int(target.origin[0])+1)]
    monkeypatch.setattr(study,'_WORLD',World())
    monkeypatch.setattr(study,'_SOLVER',solver)
    monkeypatch.setattr(study,'_REPLAY',dict(targets=targets,current_pose=None,collision_options={},joint_ranges=None))
    monkeypatch.setattr(study,'_OUTPUT',tmp_path)
    monkeypatch.setattr(study,'_OFFSETS',[0.])
    for name in ('layers','candidates','workers'):(tmp_path/name).mkdir()
    region=StationaryRegion(targets,Plane.world_xy(),projected=True)
    proposal=study.propose([[0,0]],region,'coarse',yaw_steps=1)[0]
    row=study.evaluate(proposal)
    assert row['counts']==[0,1,2]
    assert row['targets_ik_checked']==3 and row['counts_complete']
    assert row['reachable_targets']==2 and row['total_solutions']==3
    saved=np.load(tmp_path/'layers'/(row['id']+'.npz'))
    assert saved['configurations'].shape==(3,6)


def test_invalid_base_skips_ik_and_marks_blocked(tmp_path,monkeypatch):
    class World:
        last_failure='body collision'
        def is_base_valid(self,b,**kwargs):return False
    monkeypatch.setattr(study,'_WORLD',World())
    monkeypatch.setattr(study,'_SOLVER',lambda *a:pytest.fail('Invalid base reached IK'))
    monkeypatch.setattr(study,'_REPLAY',dict(targets=[Plane.world_xy()],current_pose=None,collision_options={}))
    monkeypatch.setattr(study,'_OUTPUT',tmp_path)
    (tmp_path/'candidates').mkdir()
    region=StationaryRegion([Plane.world_xy()],Plane.world_xy())
    row=study.evaluate(study.propose([[0,0]],region,'coarse',yaw_steps=1)[0])
    assert row['base_blocked'] and row['counts']==[0]
    assert row['targets_ik_checked']==0
    assert study.score_rows([row])==[]
