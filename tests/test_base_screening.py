import numpy as np
import pytest
from validation.base_screening import SlabProjection,StaticBaseScreen
from motion_toolbox.geometry import Plane
from motion_toolbox.collision import PybulletServer
from test_collision import URDF


def cube(low,high):
    lo,hi=np.asarray(low),np.asarray(high)
    vertices=[np.where(bits,hi,lo).tolist() for bits in
              [(0,0,0),(1,0,0),(1,1,0),(0,1,0),(0,0,1),(1,0,1),(1,1,1),(0,1,1)]]
    return dict(vertices=vertices,faces=[[0,3,2,1],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]])


def test_projection_clips_crossing_faces_and_detects_enclosed_rectangle():
    mesh=cube([-2,-2,-1],[2,2,2])
    p=SlabProjection(**mesh)
    assert p.overlaps([0,0],np.eye(2),[.1,.1])
    assert not p.overlaps([3,0],np.eye(2),[.1,.1])
    elevated=SlabProjection(**cube([-2,-2,1.01],[2,2,2]))
    assert not elevated.overlaps([0,0],np.eye(2),[.5,.5])
    boundary=SlabProjection(**cube([-2,-2,1],[2,2,2]))
    assert boundary.overlaps([0,0],np.eye(2),[.5,.5])


def test_projection_rotated_rectangle_and_concave_gap():
    left=cube([-2,-1,0],[-1,1,2])
    right=cube([1,-1,0],[2,1,2])
    vertices=left['vertices']+right['vertices']
    faces=left['faces']+[[i+8 for i in f] for f in right['faces']]
    p=SlabProjection(vertices,faces)
    assert not p.overlaps([0,0],np.eye(2),[.8,.1])
    assert p.overlaps([0,0],np.eye(2),[1.1,.1])
    rotation=np.array([[0,-1],[1,0]])
    assert not p.overlaps([0,0],rotation,[1.1,.1])
    tiny=SlabProjection(**cube([-.01,-.01,.2],[.01,.01,.3]))
    assert tiny.overlaps([0,0],rotation,[1.1,.1])


def test_overlapping_disconnected_solids_do_not_cancel_containment():
    left=cube([-2,-2,-1],[1,2,2]);right=cube([-1,-2,-1],[2,2,2])
    projection=SlabProjection(left['vertices']+right['vertices'],
        left['faces']+[[i+8 for i in f] for f in right['faces']])
    assert projection.overlaps([0,0],np.eye(2),[.1,.1])


@pytest.mark.parametrize('method',['rectangle','box'])
@pytest.mark.parametrize('clearance',[0.,.025])
def test_experimental_screen_matches_exact_and_retains_final_arm_check(tmp_path,method,clearance):
    path=tmp_path/'robot.urdf';path.write_text(URDF)
    # Elevated arm collision must still be rejected by detailed is_valid even
    # when the base is clear. AABB/rectangle overlap is only a fallback trigger.
    mesh=cube([.58,-.02,-.02],[.62,.02,.02])
    with PybulletServer(path) as world:
        world.add_mesh((mesh['vertices'],mesh['faces']))
        screen=StaticBaseScreen(world,[mesh],method)
        try:
            for angle in np.linspace(-np.pi,np.pi,11):
                for x in [0,.5,3.]:
                    base=Plane((x,0,0),(np.cos(angle),np.sin(angle),0),(-np.sin(angle),np.cos(angle),0))
                    expected=world.is_base_valid(base,clearance=clearance),world.last_failure
                    actual=screen.is_base_valid(base,clearance=clearance),world.last_failure
                    assert actual==expected
                    np.testing.assert_allclose(world._base.matrix,base.matrix)
            assert screen.is_base_valid(Plane.world_xy())
            assert not world.is_valid([0],Plane.world_xy())
        finally:
            screen.close()
