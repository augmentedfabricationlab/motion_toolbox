import numpy as np
import pytest
from motion_toolbox.geometry import Plane
from motion_toolbox.xy_sections import classify_sections, SectionGeometry, settings
from motion_toolbox.mobile_base_workflow import generate_base_path, plan_base_path


def arc(count=81, radius=2., sweep=120., center=(3., -2.), reverse=False):
    angle = np.linspace(-np.deg2rad(sweep)/2, np.deg2rad(sweep)/2, count)
    if reverse:
        angle = angle[::-1]
    radial = np.column_stack((np.cos(angle), np.sin(angle)))
    return np.asarray(center)+radius*radial, -radial


def targets(points, normals):
    return [Plane((*p, 1.), (0,0,1), (n[1],-n[0],0)) for p,n in zip(points,normals)]


def prepared(points, normals, raw=None, longitudinal=None):
    if longitudinal is None:
        longitudinal = np.arange(len(points))*.05
    sections, ids, opts = classify_sections(points if raw is None else raw, points, longitudinal)
    x = np.column_stack((normals, np.zeros(len(points))))
    y = np.column_stack((-x[:,1], x[:,0], np.zeros(len(points))))
    return SectionGeometry(points, dict(x_axes=x,y_axes=y), sections, ids, opts)


@pytest.mark.parametrize('reverse', [False, True])
@pytest.mark.parametrize('tangent', [-1.3, 0., 1.3])
def test_arc_radius_signed_arc_distance_and_tangent_heading(reverse, tangent):
    points, normals = arc(reverse=reverse)
    geometry = prepared(points, normals)
    assert all(s['kind']=='arc' for s in geometry.sections)
    frames = geometry.frames([1., tangent])
    center = np.array([3., -2.])
    for p, b in zip(points, frames):
        radial = b.origin[:2]-center
        assert np.linalg.norm(radial)==pytest.approx(3.,abs=1e-9)
        np.testing.assert_allclose(b.xaxis[:2], -radial/3., atol=1e-9)
        assert np.dot(b.yaxis[:2], radial)==pytest.approx(0., abs=1e-9)
        theta0, theta1 = np.arctan2(*(p-center)[::-1]), np.arctan2(*radial[::-1])
        assert ((theta1-theta0+np.pi)%(2*np.pi)-np.pi)*3. == pytest.approx(-tangent,abs=1e-9)


def test_repeated_passes_keep_radii_and_share_center_without_heading_flip():
    a, na = arc(radius=2.)
    # Shared turn sample belongs to the second pass; use duplicate endpoint
    # radius only for fitting the first pass to avoid an intentional connector.
    a, na = arc(radius=2.)
    b, nb = arc(radius=2., reverse=True)
    geometry = prepared(np.vstack((a,b)), np.vstack((na,nb)),
                        longitudinal=np.r_[np.linspace(0,4,81),np.linspace(4,0,81)])
    assert len({s.get('arc_group') for s in geometry.sections})==1
    f=geometry.frames([1.,1.3])
    np.testing.assert_allclose([v.origin for v in f[:81]], [v.origin for v in f[81:][::-1]],atol=1e-9)
    # Independent per-pass radius fitting with a common center.
    from motion_toolbox.xy_sections import _shared_center, _measure
    different,_=arc(radius=2.3)
    center=_shared_center([a,different],[3.05,-2.05])
    np.testing.assert_allclose(center,[3,-2],atol=1e-8)
    assert _measure(a,center)['radius']==pytest.approx(2.)
    assert _measure(different,center)['radius']==pytest.approx(2.3)


def test_spatial_sampling_ignores_dwell_and_noisy_arc_fits_original_points():
    points,normals=arc()
    noisy=points+np.random.default_rng(7).normal(0,.003,points.shape)
    ids=np.r_[np.arange(40),np.full(300,40),np.arange(40,81)]
    geometry=prepared(points[ids],normals[ids],raw=noisy[ids])
    assert all(s['kind']=='arc' for s in geometry.sections)
    np.testing.assert_allclose(geometry.sections[0]['center'],[3,-2],atol=.015)


def test_mixed_line_arc_has_straight_and_arc_sections_and_smooth_joins():
    line=np.column_stack((np.linspace(-2,0,81),np.zeros(81)))
    angle=np.linspace(-np.pi/2,0,81)
    curved=np.column_stack((2*np.cos(angle),2+2*np.sin(angle)))
    points=np.vstack((line[:-1],curved))
    normals=np.vstack((np.tile([0,1.],(80,1)),np.column_stack((-np.cos(angle),-np.sin(angle)))))
    geometry=prepared(points,normals)
    kinds={s['kind'] for s in geometry.sections}
    assert kinds=={'straight','arc'}
    assert geometry.transitions
    frames=geometry.frames([.5,.4])
    assert max(np.linalg.norm(a.origin-b.origin) for a,b in zip(frames,frames[1:])) < .15
    full=geometry.frames([.8,.9])
    indices=[0,75,80,85,160]
    subset=geometry.frames([.8,.9],indices)
    np.testing.assert_allclose([b.matrix for b in subset],[full[i].matrix for i in indices],atol=1e-12)


def test_mostly_straight_uses_identical_legacy_geometry():
    points,normals=arc(sweep=30.)
    ts=targets(points,normals)
    a=generate_base_path(ts)
    b=generate_base_path(ts,geometry_mode='legacy')
    assert {s['kind'] for s in a['path_sections']}=={'straight'}
    np.testing.assert_array_equal([b.matrix for b in a['base_planes']],[b.matrix for b in b['base_planes']])


def test_radial_repairs_recompute_heading_and_final_configuration_check(monkeypatch):
    from test_mobile_base_workflow import Solver,World
    from motion_toolbox import mobile_base_workflow as workflow
    points,normals=arc(count=33,center=(0,0))
    ts=targets(points,normals)
    geometry=prepared(points,normals)
    def generate(*args,**kwargs):
        kwargs['_geometry_out'].append(geometry)
        return dict(base_planes=geometry.frames([.4,.3]),smoothing={},centerline={},target_indices=list(range(33)))
    monkeypatch.setattr(workflow,'generate_base_path',generate)
    class ZeroSolver(Solver):
        def __call__(self,t,b):return [[0.]*6]
    world=World()
    world.last_failure='full 3D collision'
    checked=[]
    def valid(q,b,**kw):
        radius=np.linalg.norm(b.origin[:2])
        np.testing.assert_allclose(b.xaxis[:2],-b.origin[:2]/radius,atol=1e-9)
        checked.append(b)
        return radius >= 2.5-1e-10
    world.is_valid=valid
    result=plan_base_path(ts,solver=ZeroSolver(),world=world,joint_ranges=[[-3,3]]*6,
        periodic=[False]*6,normal_offset=.4,tangent_offset=.3,rotation_steps=1,max_base_step=.3,base_yaw_margin_degrees=0.)
    assert result['fabrication_validated'],result['status']
    assert result['repair_attempts'] and checked
    assert all(np.linalg.norm(b.origin[:2])>=2.5-1e-10 for b in result['base_planes'])


def test_invalid_settings_and_degenerate_fit_are_explicit():
    for option in ({'arc_turn_threshold_deg':0},{'arc_fit_rms':float('nan')},{'unknown':1}):
        with pytest.raises(ValueError):settings(option)
    from motion_toolbox.xy_sections import _circle
    assert _circle(np.ones((20,2))) is None
    assert _circle(np.column_stack((np.arange(20),np.zeros(20)))) is None


def test_unreliable_original_tcp_arc_is_reported_and_planning_stops(monkeypatch):
    from motion_toolbox import mobile_base_workflow as workflow
    points,normals=arc()
    # Smoothing is coherent, but the original samples cannot support a circle
    # within the fit limits. Subdivision must not hide the strong curvature.
    raw=points+normals*np.where(np.arange(len(points))%2, .25, -.25)[:,None]
    sections,ids,_=classify_sections(raw,points,np.arange(len(points))*.05)
    unresolved=[s['id'] for s in sections if s['kind']=='unresolved']
    assert unresolved
    assert all(s['kind']!='straight' for s in sections)
    assert len(ids)==len(points)
    monkeypatch.setattr(workflow,'generate_base_path',lambda *a,**k:
        dict(unresolved_sections=unresolved))
    with pytest.raises(ValueError,match='Unresolved strong curvature'):
        plan_base_path(targets(points,normals),solver=None,world=None,
                       joint_ranges=[[-3,3]]*6,periodic=[False]*6)


def test_fit_and_pose_rotation_translation_equivariance():
    points,normals=arc()
    rotation=np.array([[.6,-.8],[.8,.6]])
    a=prepared(points,normals).frames([.7,-.8])
    b=prepared(points@rotation+[10,5],normals@rotation).frames([.7,-.8])
    np.testing.assert_allclose([p.origin[:2] for p in b],np.array([p.origin[:2] for p in a])@rotation+[10,5],atol=1e-9)
    np.testing.assert_allclose([p.xaxis[:2] for p in b],np.array([p.xaxis[:2] for p in a])@rotation,atol=1e-9)


def test_s_curve_is_split_at_inflection_with_two_centers():
    a=np.linspace(-np.pi/2,0,81)
    b=np.linspace(np.pi,np.pi/2,81)
    points=np.vstack((np.c_[2*np.cos(a),2+2*np.sin(a)],np.c_[4+2*np.cos(b),2+2*np.sin(b)][1:]))
    sections,ids,_=classify_sections(points,points,np.arange(len(points))*.05)
    assert {s['kind'] for s in sections}=={'arc'}
    assert len({s['arc_group'] for s in sections})==2
    assert len(ids)==len(points)


def test_open_arc_over_180_degrees_does_not_use_single_valued_legacy_headings():
    points,normals=arc(count=161,sweep=240.)
    result=generate_base_path(targets(points,normals),max_xy_deviation=.02)
    assert {s['kind'] for s in result['path_sections']}=={'arc'}
    assert len(result['base_planes'])==161
    assert not result['unresolved_sections']


@pytest.mark.parametrize('yaw',[30.,-30.])
def test_arc_yaw_is_applied_after_radial_offset(yaw):
    from test_mobile_base_workflow import Solver,World
    points,normals=arc(count=25,center=(0,0))
    ts=targets(points,normals)
    world=World()
    checked=[]
    class ZeroSolver(Solver):
        def __call__(self,t,b):return [[0.]*6]
    world.is_valid=lambda q,b,**kw:checked.append(b) or True
    kwargs=dict(solver=ZeroSolver(),world=world,joint_ranges=[[-3,3]]*6,periodic=[False]*6,
                normal_offset=.5,tangent_offset=.2,rotation_steps=1,max_base_step=.5,adapt_offsets=False)
    base=plan_base_path(ts,**kwargs)
    rotated=plan_base_path(ts,base_yaw_degrees=yaw,**kwargs)
    assert base['fabrication_validated'] and rotated['fabrication_validated']
    angle=np.radians(yaw)
    for a,b in zip(base['base_planes'],rotated['base_planes']):
        np.testing.assert_array_equal(a.origin,b.origin)
        np.testing.assert_allclose(b.xaxis,np.cos(angle)*a.xaxis+np.sin(angle)*a.yaxis,atol=1e-12)
    assert checked
