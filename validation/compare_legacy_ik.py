"""Read-only replay of original IK against the extracted solver."""
import argparse
import json
from pathlib import Path
import numpy as np
from motion_toolbox.geometry import Plane
from motion_toolbox.kinematics.ur import inverse_kinematics, forward_kinematics
from motion_toolbox.kinematics.solver import URKinematics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source = args.workspace/'mobile_motion_planning/mobile_motion_planning/src/mobile_motion_planning/ik_offline/ik.py'
    text = source.read_text(encoding='utf-8').replace('from .geometry import Plane', 'from motion_toolbox.geometry import Plane')
    legacy = {}
    exec(compile(text, str(source), 'exec'), legacy)
    rng = np.random.default_rng(101)
    checked = 0
    for _ in range(100):
        q = rng.uniform(-2, 2, 6)
        plane = forward_kinematics(q)
        old = legacy['inverse_kinematics'](plane)
        new = inverse_kinematics(plane)
        np.testing.assert_allclose(new, old, atol=1e-12)
        checked += 1
    # Compare actual source JSON targets, without copying project data into this repo.
    from motion_toolbox.utilities.io import load_planes
    targets = load_planes(args.workspace/'sprayed_earth_am/data/260520_targets.json')
    bases = load_planes(args.workspace/'sprayed_earth_am/data/260520_base_position.json')
    if len(bases) == 1:
        bases *= len(targets)
    # This supplied tool is the original default, used only for migration parity.
    tool = Plane((-.429, .0027, .093), (0, 0, 1), (0, 1, 0))
    solver = URKinematics(tool=tool)
    from motion_toolbox.geometry import rigid_inverse
    count = 0
    for target, base in list(zip(targets, bases))[:100]:
        flange = Plane.from_matrix(rigid_inverse(base.matrix) @ target.matrix @ rigid_inverse(tool.matrix))
        np.testing.assert_allclose(solver(target, base), legacy['inverse_kinematics'](flange), atol=1e-12)
        count += 1
    # Verify the convention against the supplied URDF, not just the legacy solver.
    # UR's controller "base" frame is rotated relative to the URDF root base_link.
    from compas_robots import RobotModel
    model = RobotModel.from_urdf_file(str(args.workspace/'slab_net_zero/data/URDF/ur20.urdf'))
    from motion_toolbox.geometry import as_plane
    names = model.get_configurable_joint_names()
    for _ in range(20):
        q = rng.uniform(-2, 2, 6)
        state = dict(zip(names, q))
        base = as_plane(model.forward_kinematics(state, 'base'))
        tool0 = as_plane(model.forward_kinematics(state, 'tool0'))
        np.testing.assert_allclose(rigid_inverse(base.matrix) @ tool0.matrix, forward_kinematics(q).matrix, atol=1e-7)
    args.output.write_text(json.dumps(dict(random_poses_checked=checked, source_json_targets_checked=count,
        urdf_fk_checks=20, analytic_frame='UR controller base to tool0; not URDF base_link to tool0',
        result='Original IK candidates preserved; FK wrist correction separately round-trip tested.'), indent=2))
    print('IK parity: {} synthetic and {} recorded targets passed'.format(checked, count))


if __name__ == '__main__':
    main()
