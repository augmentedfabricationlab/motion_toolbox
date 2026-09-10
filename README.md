# Motion toolbox

Reusable offline IK, PyBullet collision checking, exact layered motion planning,
stationary base placement, mobile base planning, and rolling replanning.
The numeric core needs Python 3.9+ and NumPy. Rhino, COMPAS, PyBullet and ROS are
optional integrations. Lengths are **metres**, joint angles **radians** and target
indices **zero-based**.

This repository was extracted from `mobile_motion_planning`, `slab_net_zero`, and
`sprayed_earth_am`. It does not import those packages. Their code and project assets
remain in their original locations. See [migration.md](docs/migration.md) for the
functionality map and deliberate behavior changes.

## Install and try

From the parent directory containing the new repositories:

```powershell
python -m venv motion_toolbox/.venv
motion_toolbox/.venv/Scripts/python -m pip install -e './motion_toolbox[collision,compas,test]' -e ./toolpath_toolbox
motion_toolbox/.venv/Scripts/python motion_toolbox/examples/standalone.py
motion_toolbox/.venv/Scripts/motion-plan motion_toolbox/examples/static_job.json --output joint_plan.json
```

On Linux/macOS use `.venv/bin/python` and `.venv/bin/motion-plan`. Install the same
packages in Rhino 8's Python 3 environment to use the APIs from Grasshopper. This
is not an IronPython 2 package. `compas-fab` is constrained to 1.x, which provides
the Robot/Tool interface used by the existing `MobileRobot`; 2.x has a different API.

## Plan TCP targets at prescribed base positions

```python
from motion_toolbox.geometry import as_plane
from motion_toolbox.kinematics.solver import URKinematics
from motion_toolbox.planning import calculate_partial_trajectory

solver = URKinematics(tool=tcp_in_flange, arm_in_base=arm_frame_in_footprint)
result = calculate_partial_trajectory(
    current_pose, targets, base_planes=bases, ik_solver=solver,
    rotation_mode='n_steps', rotation_steps=24,
    joint_ranges=joint_limits, max_joint_step=0.5,
)
joint_plan = result['configurations']
```

`targets` accepts numeric `Plane`, Rhino/rhino3dm planes, COMPAS frames or JSON
plane dictionaries. `bases` contains one footprint plane or one per target.
Omit `number_of_nodes_to_calculate` for the full path; set it for a prefix.
`dont_build_graph=True` returns candidate/fitness information only.

The solver removes duplicates, filters joint limits before collision queries,
and minimizes total weighted joint travel over every feasible adjacent layer.
TCP rotations are around each target's own Z axis with its origin fixed. Supported
modes are off, `n_steps` over a full turn, and `step_angle` between negative CCW
and positive CW bounds (degrees at that convenience API only).

The default analytic model is UR20, preserving the source solver's flange/joint
convention. Other UR dimensions are accepted as six parameters. Other robot types
can provide `ik_solver(target, base) -> list[joint_vector]`. This is not a generic
URDF-derived analytic solver. Validate the convention against the selected URDF
and actual robot calibration. No hardcoded spraying TCP or mobile lift height
is supplied; identity frames mean flange targets at the arm origin.

## Collision checking

```python
from motion_toolbox.collision import PybulletServer
from motion_toolbox.robot_adapter import kinematics_from_robot

solver = kinematics_from_robot(robot, arm_in_base=arm_frame_in_footprint)
with PybulletServer(robot=robot, joint_names=arm_joint_names) as scene:
    scene.set_fixed_joints(fixed_joint_values)  # lift/wheels, by URDF name
    for mesh in collision_meshes:
        scene.add_mesh(mesh)
    result = calculate_partial_trajectory(
        current_pose, targets, base_planes=bases, ik_solver=solver,
        collision=scene.is_valid, transition_check=scene.edge_is_valid,
        rotation_mode='n_steps', rotation_steps=24,
    )
```

`robot.model` is copied and exported to a temporary URDF. Loaded visual/collision
meshes are exported once; `package_paths={'package_name': '/package/root'}` or
`asset_root` resolve unloaded mesh resources. Each attached tool collision mesh is
placed relative to its attachment link. The active `MultiTool` TCP is used for IK;
it is not applied a second time to flange-local collision geometry. `robot.BCF`
provides initial placement when present. No `robot.RCF` network subscription is
started. An explicit `arm_in_base` is preferred; the initialized offline `_RCF`
and `lift_height` may be used by the IK adapter.

For standalone manual loading, use `PybulletServer('robot.urdf', joint_names=...)`.
URDF mesh paths must resolve; tool meshes can be included in that URDF or attached
with `scene.attach_mesh(mesh, link_name, frame=..., touch_links=...)`. The selected
joint names define configuration order exactly. Values are never silently truncated
to the last six joints. `set_fixed_joints` controls other movable joints.

Reuse a world across batches and close it explicitly. Each world owns a separate
Bullet client. There is no automatic persistent collision-result cache that can
become stale when the environment changes. Rebuild a world if the model/tool changes.
Environment meshes can be filenames, COMPAS/Rhino meshes or `(vertices, faces)`.
Moving tool meshes use convex hulls: supply convex pieces to preserve concavities.
Environment meshes are static triangle surfaces, so use closed, well-oriented
meshes for physical obstacles and consider primitive solids where appropriate.

Parent/child link self contacts are excluded; additional accepted contacts use
explicit `allowed_pairs`. Ground support links are configured separately through
`add_ground(..., support_links=...)`; environment collisions are still checked for
those links. Swept collision checks sample synchronized joint/base motion at
configurable resolutions. They are not a continuous collision proof or Cartesian
path-following guarantee between targets. Dense TCP targets and appropriate edge
resolutions remain necessary.

## Stationary printing and printing while driving

```python
from motion_toolbox.base_planning import (
    grid_bases, bases_around_targets, find_stationary_base, plan_mobile_base,
)

stationary = find_stationary_base(
    targets, grid_bases(x_values, y_values, yaw_values), current_pose,
    ik_solver=solver, collision=scene.is_valid,
    transition_check=scene.edge_is_valid,
)
# stationary.base_plane is a single plane when feasible.

domains = bases_around_targets(targets, distances, bearings, yaw_offsets)
mobile = plan_mobile_base(
    targets, domains, ik_solver=solver, collision=scene.is_valid,
    transition_check=scene.edge_is_valid,
    max_base_step=0.1, max_yaw_step=0.15,
    time_intervals=durations, max_base_speed=0.05,
)
# mobile.base_planes and mobile.configurations each have one entry per target.
```

Stationary search requires a connected arm path through **all** targets, not just
a large total IK count. `evaluate_base_locations` supplies candidate-only counts,
unreachable indices and the original fitness formula for Grasshopper exploration.
Mobile planning searches coupled `(base pose, arm joints)` states. Both return
empty results and infinite cost if no complete path exists; neither silently skips
unreachable targets. Candidate generators define a finite search domain: optimum
means optimum within that domain, not globally over continuous space.

Mobile motion currently assumes an upright omnidirectional base. It supports
translation/yaw/joint-step and time-dependent speed bounds. Use the transition
callback for additional steering constraints. Acceleration, nonholonomic steering,
time parameterization and dynamic obstacles are not built-in models. Increase or
refine the candidate domain when reachability fails. No arbitrary candidate cap
or beam search silently discards an optimal path.

Bounded joints use actual angle deltas. Set `periodic` only for physically
continuous joints; otherwise an apparent short wrap can exceed real limits. The
UR candidate evaluator enumerates equivalent revolutions inside explicit finite
joint limits. Custom solvers may declare `revolute_joints` to enable that behavior.
When using periodic joints, configure the same mask in the swept-edge callback.

## Rolling and static workflows

`RollingPlanner(targets, lookahead=10, buffer_size=2, **options)` works without ROS.
Call `update(-1, current_pose, [measured_base])` to seed and then update with the
executed target index. It preserves committed buffered targets and extends the
tail, with a lookahead solution seeded from the last committed joint target.
It accepts a measured single base or a complete predicted base sequence. A failed
plan publishes nothing and can be retried. Reset by constructing a new planner.

The sibling **motion_toolbox_ros** package supplies odometry, joint-state and
execution-index subscriptions, durable indexed target messages, CSV timing and
configurable base velocity commands. The sibling **toolpath_toolbox** package
contains surface/curve algorithms and UR toolpath JSON conversion.

`utilities.static_planning.run_all` retains edit → collision filter → optimum
workflow with explicit inputs and returned diagnostics. `motion-plan` reads a
JSON job, resolves paths relative to the job and writes a joint plan only on
success. Live pose acquisition belongs to the caller; there is no network access
or fabricated fallback starting pose inside the utility.

## Performance and verification

See [benchmarks/results.json](benchmarks/results.json): exact graph-only solving
was 7.7–57.1× faster than the materialized NetworkX reference in three synthetic
cases, with identical costs. At 200 targets × 48 candidates, median time was
56 ms versus 1.72 s and traced Python peak memory 0.54 MB versus 216 MB. These are
local synthetic measurements, not an end-to-end robot performance guarantee.

Run `python -m pytest tests` after installing test and COMPAS extras. Tests cover
exhaustive optimum comparisons, transforms, FK/IK round trips, partial planning,
rolling buffer behavior, stationary/mobile feasibility, joint limits, isolated
Bullet worlds, tool collisions, swept edges and actual COMPAS object export.
Source replay matched 100 synthetic and 100 recorded targets; see
[validation/ik_parity.json](validation/ik_parity.json).
The actual source `MobileRobot` and `MultiTool` classes also passed an offline
adapter smoke test with a synthetic model; see
[validation/mobile_robot_results.json](validation/mobile_robot_results.json).

`validation/verify_sources.py --workspace <parent>` verifies the initial SHA-256
snapshot of 104 original Python files. Original repositories may already contain
uncommitted work; this migration neither cleans nor commits it.
