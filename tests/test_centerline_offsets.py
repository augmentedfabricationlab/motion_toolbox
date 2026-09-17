import numpy as np
from motion_toolbox.xy_offset import centerline_offset_frames


def test_normal_offset_is_upright_and_does_not_flip_on_return_pass():
    curve = np.column_stack((np.linspace(0,3,30), np.zeros(30)))
    mapped = np.vstack((curve,curve[::-1]))
    # Target Z faces world +Y. Base +Y therefore points world -X.
    tx = np.tile([0,0,1.],(len(mapped),1))
    ty = np.tile([1.,0,0],(len(mapped),1))
    f = centerline_offset_frames(curve,mapped,tx,ty)
    expected = np.column_stack((mapped[:,0]-1.2, np.full(len(mapped),-.9), np.zeros(len(mapped))))
    np.testing.assert_allclose(f['origins'],expected,atol=1e-12)
    np.testing.assert_allclose(f['origins'][:30],f['origins'][30:][::-1],atol=1e-12)
    np.testing.assert_allclose(f['z_axes'],np.tile([0,0,1.],(60,1)))


def test_curve_normals_follow_geometry_not_target_jitter():
    s=np.linspace(-1,1,100)
    curve=np.column_stack((s,.15*s*s))
    tx=np.tile([0.,0,1],(100,1))
    angle=.2*np.sin(s*50)
    ty=np.column_stack((np.cos(angle),np.sin(angle),np.zeros(100)))
    f=centerline_offset_frames(curve,curve,tx,ty)
    shift=f['origins']-f['line_origins']
    np.testing.assert_allclose(np.sum(shift*f['x_axes'],axis=1),-.9,atol=1e-12)
    np.testing.assert_allclose(np.sum(shift*f['y_axes'],axis=1),1.2,atol=1e-12)
    np.testing.assert_allclose(np.cross(f['x_axes'],f['y_axes']),f['z_axes'],atol=1e-12)
    assert np.max(np.linalg.norm(np.diff(f['x_axes'],axis=0),axis=1)) < .01
