"""Execute complete component scripts with real Rhino and Grasshopper types."""
import argparse
import json
from pathlib import Path
import runpy
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rhino-system', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import rhinoinside
    rhinoinside._RhinoSystem.from_path(args.rhino_system).load(rhinoinside._DotNetFramework(48, 0))
    import clr
    clr.AddReference('RhinoCommon')
    clr.AddReference(str(Path(args.rhino_system).parent/'Plug-ins/Grasshopper/Grasshopper.dll'))
    import Rhino
    core = Rhino.Runtime.InProcess.RhinoCore(['/NOSPLASH', '/NOTEMPLATE'], Rhino.Runtime.InProcess.WindowStyle.NoWindow)
    try:
        from motion_toolbox.geometry import Plane, to_rhino
        from motion_toolbox.utilities.io import solutions_to_tree
        repo = Path(__file__).resolve().parents[1]
        # Reuse the synthetic six-joint fixture; real .NET geometry/DataTree types.
        fixture = runpy.run_path(str(repo/'tests/test_grasshopper_entrypoints.py'))['robot_fixture']
        robot, q, targets, names = fixture()
        inputs = dict(robot=robot, target_planes=[to_rhino(p) for p in targets], current_pose=q,
            arm_in_base=to_rhino(Plane.world_xy()),
            base_planes=[to_rhino(Plane.world_xy())], rotation_steps=1)
        output = runpy.run_path(str(repo/'examples/grasshopper.py'), init_globals=inputs)
        assert output['status'].startswith('Planned'), output['status']
        assert output['joint_plan'].BranchCount == 3
        assert output['joint_plan'].DataCount == 18
        assert len(output['configurations']) == 3
        inputs.update(base_plane=to_rhino(Plane.world_xy()), tcp_plane=to_rhino(Plane.world_xy()))
        simple = runpy.run_path(str(repo/'examples/grasshopper_motion_plan.py'), init_globals=inputs)
        assert simple['status'].startswith('Planned'), simple['status']
        assert simple['joint_path'].DataCount == 18
        inputs['candidate_planes'] = [to_rhino(Plane.world_xy())]
        stationary = runpy.run_path(str(repo/'examples/grasshopper_stationary_base.py'), init_globals=inputs)
        assert stationary['status'].startswith('Found base'), stationary['status']
        assert stationary['joint_plan'].DataCount == 18
        assert stationary['base_plane'].Origin.DistanceTo(Rhino.Geometry.Point3d.Origin) < 1e-9
        tree = solutions_to_tree([[[1., 2.]], [], [[3., 4.]]])
        assert tree.BranchCount == 3 and tree.DataCount == 4
        args.output.write_text(json.dumps(dict(result='passed', rhino=str(Rhino.RhinoApp.Version),
            components=['grasshopper.py', 'grasshopper_motion_plan.py', 'grasshopper_stationary_base.py'], branches=3, values=18,
            robot_object=True, pybullet_collision=True, grasshopper_datatree=True), indent=2))
        print('All three Grasshopper component scripts passed using real Rhino/Grasshopper types')
    finally:
        core.Dispose()


if __name__ == '__main__':
    main()
