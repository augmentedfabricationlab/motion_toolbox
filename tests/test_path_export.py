import json
from pathlib import Path
import runpy

import numpy as np
import pytest

from motion_toolbox.geometry import Plane
from motion_toolbox.utilities.path_export import build_path_export, export_robot_paths, _quaternion


def paths():
    tcp = [Plane((0,0,1.1),(1,0,0),(0,1,0)), Plane((0,0,1.5),(0,1,0),(-1,0,0)),
           Plane((.3,0,1.9),(1,0,0),(0,1,0))]
    base = [Plane((x,0,0),(1,0,0),(0,1,0)) for x in (0,.2,1.)]
    return tcp, base


def test_shared_timing_uses_3d_tcp_distance_and_preserves_both_paths():
    tcp,base = paths()
    result = build_path_export(tcp,base,4.)
    assert result['timestamps']==[{'sec':0,'nanosec':0},{'sec':10,'nanosec':0},{'sec':22,'nanosec':500000000}]
    for name,planes in [('arm_data',tcp),('base_data',base)]:
        data=result[name]
        assert data['frame_id']=='vicon_world'
        assert [p['stamp'] for p in data['poses']]==result['timestamps']
        np.testing.assert_allclose([list(p['position'].values()) for p in data['poses']],[p.origin for p in planes])
    # A shared clock does not prescribe equal arm and base speeds.
    assert .2/10 != .4/10


@pytest.mark.parametrize('axis',[(1,0,0),(0,1,0),(0,0,1)])
@pytest.mark.parametrize('angle',[0.,1.,np.pi,-np.pi+1e-8])
def test_quaternion_reconstructs_rotation_at_trace_branch_boundaries(axis,angle):
    v=np.asarray(axis,dtype=float)
    skew=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    rotation=np.eye(3)+np.sin(angle)*skew+(1-np.cos(angle))*(skew@skew)
    q=_quaternion(Plane((0,0,0),rotation[:,0],rotation[:,1]))
    np.testing.assert_allclose(np.linalg.norm(q),1)
    x,y,z,w=q
    reconstructed=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                            [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                            [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    np.testing.assert_allclose(reconstructed,rotation,atol=1e-12)


def test_units_and_nanosecond_carry():
    tcp,base=paths()
    scaled=lambda ps:[Plane(p.origin*1000,p.xaxis,p.yaxis) for p in ps]
    a=build_path_export(tcp,base,3.)
    b=build_path_export(scaled(tcp),scaled(base),3.,model_units_to_metres=.001)
    assert a['timestamps']==b['timestamps']
    for key in ('arm_data','base_data'):
        for left,right in zip(a[key]['poses'],b[key]['poses']):
            np.testing.assert_allclose(list(left['position'].values()),list(right['position'].values()),atol=1e-14)
            assert left['stamp']==right['stamp'] and left['orientation']==right['orientation']
    assert all(0<=s['nanosec']<10**9 for s in a['timestamps'])


def test_unique_folders_and_failed_validation_writes_nothing(tmp_path):
    a=export_robot_paths(*paths(),4,documents_folder=tmp_path)
    saved=Path(a['tcp_file']).read_bytes()
    b=export_robot_paths(*paths(),4,documents_folder=tmp_path)
    assert a['export_folder']!=b['export_folder'] and Path(a['tcp_file']).read_bytes()==saved
    assert json.loads(Path(b['base_file']).read_text())==b['base_data']
    before=list(tmp_path.iterdir())
    with pytest.raises(ValueError,match='same length'):export_robot_paths(paths()[0],paths()[1][:1],4,documents_folder=tmp_path)
    assert list(tmp_path.iterdir())==before


def test_failed_second_write_removes_partial_pair(tmp_path,monkeypatch):
    original=Path.write_text
    def fail(path,*a,**k):
        if path.name=='base_path.json':raise OSError('simulated failure')
        return original(path,*a,**k)
    monkeypatch.setattr(Path,'write_text',fail)
    with pytest.raises(OSError,match='simulated'):export_robot_paths(*paths(),4,documents_folder=tmp_path)
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('speed',[0,-1,float('nan'),float('inf')])
def test_invalid_speed_rejected(speed):
    with pytest.raises(ValueError,match='speed'):build_path_export(*paths(),speed)


def test_original_sanity_checks_and_repeated_points():
    tcp,base=paths()
    with pytest.raises(ValueError,match='above 1.0'):build_path_export(base,base,4)
    with pytest.raises(ValueError,match='extent'):build_path_export(tcp,[base[0]]*3,4)
    result=build_path_export([tcp[0]]*3,base,4,sanity_checks=False)
    assert result['duration_seconds']==0 and result['diagnostics']


def test_actual_gh_component_export_preview_and_failure(tmp_path):
    script=Path(__file__).resolve().parents[1]/'examples/grasshopper_export_paths.py'
    tcp,base=paths()
    inputs=dict(tcp_planes=tcp,base_planes=base,speed=4.,documents_folder=str(tmp_path))
    result=runpy.run_path(str(script),init_globals=inputs)
    assert result['status'].startswith('Exported 3') and Path(result['tcp_file']).is_file()
    before=list(tmp_path.iterdir())
    preview=runpy.run_path(str(script),init_globals=dict(inputs,write_files=False))
    assert preview['time_seconds']==[0.,10.,22.5] and preview['tcp_file'] is None
    assert list(tmp_path.iterdir())==before
    failed=runpy.run_path(str(script),init_globals=dict(inputs,speed=0))
    assert failed['result'] is None and failed['tcp_file'] is None and failed['diagnostics']


@pytest.mark.parametrize('loader', ['no_file', 'explicit_source', 'gh_generated_file'])
def test_gh_text_execution_without_package_context(tmp_path,loader):
    root=Path(__file__).resolve().parents[1]
    script=root/'examples/grasshopper_export_paths.py'
    tcp,base=paths()
    scope=dict(__name__='__main__',__package__=None,tcp_planes=tcp,base_planes=base,
               speed=4.,documents_folder=str(tmp_path))
    if loader=='explicit_source':scope['toolbox_src']=str(root/'src')
    if loader=='gh_generated_file':scope['__file__']=str(tmp_path/'generated_component.py')
    exec(compile(script.read_text(encoding='utf-8'),str(script),'exec'),scope)
    assert scope['status'].startswith('Exported 3'),scope['status']
    assert Path(scope['tcp_file']).is_file() and Path(scope['base_file']).is_file()
    assert scope['time_seconds']==[0.,10.,22.5]


def test_helper_file_can_load_without_a_parent_package():
    root=Path(__file__).resolve().parents[1]
    namespace=runpy.run_path(str(root/'src/motion_toolbox/utilities/path_export.py'))
    assert callable(namespace['build_path_export'])


def test_export_records_aggregate_metrics_and_complete_payload(tmp_path):
    import sqlite3
    from motion_toolbox.recording import ResearchRun, read_artifact
    with ResearchRun(tmp_path,name='export logging regression') as run:
        result=export_robot_paths(*paths(),4,write_files=False)
    with sqlite3.connect(run.path/'run.sqlite3') as db:
        names=[r[0] for r in db.execute('select name from metrics')]
        assert 'result.arm_data.poses.count' in names and 'result.timestamps.count' in names
        assert not any('.poses[' in n or '.timestamps[' in n for n in names)
        digest=db.execute('select output_artifact from steps where parent_id is null').fetchone()[0]
    payload=json.loads(read_artifact(run.path,digest))
    assert payload['arm_data']==result['arm_data'] and payload['base_data']==result['base_data']
