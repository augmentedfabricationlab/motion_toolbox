# Motion toolbox

See [CHANGELOG.md](CHANGELOG.md) for changes between package versions.

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

For a ready-to-paste Rhino 8 Python 3 component, use
[examples/grasshopper_motion_plan.py](examples/grasshopper_motion_plan.py).
Its header lists the exact Grasshopper inputs and outputs. It returns a joint
DataTree with one branch per target and supports optional TCP-Z rotation sampling.
This minimal component uses an explicit arm-base/TCP frame and does not check
collisions. [examples/grasshopper.py](examples/grasshopper.py) is the complete
robot-object component with PyBullet configuration collision checks.
Sampled transition checks are opt-in with `check_edges=True` (also for JSON jobs).
`current_pose` and `arm_in_base` are optional in the robot-object component and
`plan_robot`: mounting comes from offline calibration or the URDF controller link.
The robot-object component uses the installed package without a `toolbox_src`
input. It automatically refreshes a cached planner that still requires a starting
pose, and reloads planning dependencies when their source files change between
recomputes. Unchanged recomputes reuse the imports. `toolbox_src` is only an
optional development override; the package must be installed in Rhino's Python
environment for normal use.
The `timings` output separates setup, candidate generation, IK, joint expansion,
collision filtering and graph search (seconds). Candidate time includes the IK,
expansion and collision subtotals; these are not additive independent stages.
The `diagnostics` output explains failed targets with raw IK counts, counts after
limits and collisions, and collision rejection reasons. Full per-target records,
including actual collision-query and cache-hit counts, are in
`result['target_diagnostics']`. The total timer covers the planner body; outer
research-recording serialization and Grasshopper output conversion add overhead.
Default `rotation_steps=24` samples a full turn per target. Use `1` when target
orientations must remain fixed; this reduces the search and can change feasibility.
The plane-only component also accepts an empty starting pose, but still requires
explicit controller-base and TCP frames because it has no robot model.
It returns both a six-arm-joint DataTree and named COMPAS configurations containing
the prismatic lift joints followed by the six arm joints in IK order. Wheel joints
are excluded from this output. Its header specifies input access and units.
Use `collision_scene` to reuse a prepared world across component evaluations.
The same workflow is available outside Grasshopper as
`motion_toolbox.robot_planning.plan_robot`.

`robot_adapter.resolve_arm_joint_names` uses model-based inference when a robot
model is present. Without one, it returns the six `robot_arm_...` names from
`mobile_robot_control` in shoulder-pan, shoulder-lift, elbow, wrist-1/2/3 order.
Explicit names override the fallback. ROS configuration may omit `joint_names`
to use these defaults. Direct URDF collision loading still reads joints from
the supplied URDF model.

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
The analytic frame is the UR controller `base` frame to `tool0`, not necessarily
URDF `base_link` to `tool0`. In the source UR20 URDF those base frames differ by
a half-turn around Z. Set `arm_in_base` to the controller-base frame relative to
the footprint, including the lift and mounting rotation; `base_planes` are the
footprint placements. Explicit `ur_parameters` selects other UR dimensions.

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

`robot.model` is serialized to a separate XML tree and exported to a temporary
URDF without deep-copying native CAD objects or changing the original model. Loaded visual/collision
meshes are exported once; `package_paths={'package_name': '/package/root'}` or
`asset_root` resolve unloaded mesh resources. Each attached tool collision mesh is
placed relative to its attachment link. The active `MultiTool` TCP is used for IK;
it is not applied a second time to flange-local collision geometry. `robot.BCF`
provides initial placement when present. No `robot.RCF` network subscription is
started. `arm_in_base` is an optional override; otherwise the adapter uses offline
`_RCF` plus `lift_height`, or the recognized URDF controller link with configured
upstream joints. Inferred fixed joints are shared by IK, collision checks and
returned named configurations. Robot-object planning ignores internal static
assembly contacts by default; `collision_options.check_static_self_collisions`
can enable them. Moving-arm and environment collision checks remain active.

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

The self-collision matrix includes only links with collision geometry, honoring
the same adjacency and SRDF/allowed-pair exclusions. Empty frame links are removed
before building the matrix. Attached tools query only nearby non-touch links.
Environment and tool bodies use explicit distance queries; their automatic contact
generation is disabled to avoid computing those contacts again during robot self
checks. This changes query scheduling, not which geometry can reject a pose.

## Stationary printing and printing while driving

### Printing while driving in Grasshopper

Use **[examples/grasshopper_mobile_base.py](examples/grasshopper_mobile_base.py)**
as the dedicated base-motion component. Paste it into a Rhino 8 Python 3
component, with `robot` (Item) and `target_planes` (List, Plane). It generates
footprint candidates from the stationary finder's placement region at each
target; no external path generator is needed. The calibrated **arm-base origin**
must be behind projected target +Z and no farther than **1.75 m in XY**.
Footprint origins account for the mounting offset at each heading. These rules
also apply to every interpolated target, with actual IK and collisions checked.
Optional `seed_base_planes` supplies proposals subject to the same rules.
`grid_spacing` (default 0.5 metre) and `base_height` use Rhino model units;
`yaw_steps` defaults to four headings per arm-origin candidate.

The main output **`base_planes`** has one footprint plane per original TCP
target on success. `joint_plan` and named `configurations` also contain the
validated arm plan. To use a separate arm-planning component, connect
`base_planes` to `grasshopper.py` with the same targets and robot settings.
The dedicated component defaults to sparse search, fixed TCP orientation,
and configuration plus sampled transition collision checks. Like the stationary
finder, region candidates are ordered by standoff, with base-body checks before
arm IK. Sparse mode retains the first four feasible base candidates at each
searched target to bound graph size; `max_feasible_bases` can override this
budget (null removes it). This is approximate and may miss a connected path.
`sparse:false` searches the complete sampled region without this default cap.
Candidate IK/collision checks are reused during fallback within the same call.
`xy_offsets`, `yaw_offsets`, and the old `seed_distance` do not control the
dedicated region search. `mobile_options` configures sampling and motion limits.
With `current_pose`, connect
the matching `start_base` plane in model units. Further options are listed in
the component's header.

The existing geometric generators in the sibling `sprayed_earth_am` repository
are `src/sprayed_earth_am/generate_base_path.py` (`wall_pts` -> `base_frames`)
and `src/sprayed_earth_am/utilities/base_path/create_basepath_from_surface.py`
(surface edge -> offset planes). These propose footprint paths; they do not
test arm reach or collision feasibility.

Feed their ordered planes into `base_planes` on
[examples/grasshopper.py](examples/grasshopper.py), alongside the full ordered
`target_planes` and `robot`. This component handles **both stationary and mobile
arm planning**: one base plane means stationary, one per target means moving.
Without `mobile_options`, it uses the supplied path unchanged.

To also optimize the base path, add an optional Item input `mobile_options`
and supply JSON, for example:

```json
{"sparse":true,"max_gap":20,"max_distance":0.2,"angle":0.15,
 "xy_offsets":[[0,0],[0.1,0],[-0.1,0],[0,0.1],[0,-0.1]],
 "yaw_offsets":[0,0.1,-0.1],"max_base_step":0.25,"max_yaw_step":0.25}
```

Offsets are in **world XY metres**, yaw/angle in radians, regardless of the
Rhino input scale. Candidate offsets are relative to each supplied footprint
seed, not the TCP. A single seed is also accepted. Output `base_result` and
`configurations` each have exactly one entry per original TCP target on success;
failed mobile planning returns empty paths. The cost combines base, yaw and
joint travel. `mobile_options={}` checks only the supplied seeds, without adding
offset candidates. Keep `current_pose` empty to omit the approach; if supplied,
also provide `mobile_options.start_base` as a metre-based numeric plane dictionary
(`origin`, `x_axis`, `y_axis`).

### Smooth fabrication base path (dedicated mobile component default)

Only `grasshopper_mobile_base.py` has a hardcoded **45-minute wait limit**. Its
planning job runs in a worker; on expiry the component clears its outputs,
returns a timeout status and requests cancellation without joining the worker.
The same component cannot launch a duplicate job until the old worker finishes
cleanup. The worker never updates Grasshopper outputs. Cancellation occurs at
the next toolbox function call, letting owned resources unwind normally. A
native call must return before it can be cancelled; this does not forcibly kill
Rhino threads, and a native call holding Python's GIL can delay Python execution.
`grasshopper.py` and direct/shared planning APIs have no automatic time limit.

`grasshopper_mobile_base.py` now defaults to `strategy:"adaptive"`.
It smooths the world-XY target trend and local normals, searches varying wall
offset, lateral offset and yaw controls together with arm branches, and adds
control points where full-resolution validation fails. Good distant regions
are retained while overlapping neighbourhoods of failures are searched again.
Only a connected, fully checked path returns output frames. A final smoothing
pass is accepted only after repeating target, placement, joint and swept-edge
validation; otherwise the already valid path is retained.

```json
{"strategy":"adaptive", "adaptive_window":50,
 "adaptive_knot_gap":20, "adaptive_rounds":8}
```

Default candidate wall distances are `[0.6,0.8,1.0,1.2,1.4]` metres; lateral
offsets start at `[0.6,0.8,1.0,1.2]` metres and mirror to the other side if needed.
`wall_distances`, `lateral_offsets` and `yaw_offsets` override the search domain;
default heading offsets are `[-0.3,0,0.3]` radians from the smoothed wall normal.
The sparse roadmap is a proposal generator, not a validation shortcut. Original
motion constraints are applied to every original target and adjacent transition.
Arm-path proposal validation is bounded before a failed tested transition is
used to refine the base controls; it does not exhaustively optimize every rejected
proposal. Such a failure is not proof that no connected arm path exists.
Recognized URDF UR chains use calibrated FK refinement of analytic IK branches,
including the actual TCP and independent arm mounting transform.

See [offline replay instructions](validation/MOBILE_REPLAY.md) for bounded,
killable testing using exported assets without Rhino. A successful geometric
plan is not a timed controller trajectory; speed bounds require supplied timing.

#### Fixed-offset smooth proposal strategy

Set `strategy:"smooth_offset"` to retain the previous whole-path proposal search.
The planner creates whole smooth paths first, then validates the robot on each
path. It does not minimize base travel or choose independent per-target bases.
Footprint **+X faces the wall**; the roughly one-metre offset is along footprint
**+Y or -Y**, beside the robot. The wall distance along X is searched separately.
Offsets are measured from the footprint; reach is checked from the calibrated
arm origin, with maximum XY reach 1.75 m and the required negative-TCP-Z side.

Fixed-offset strategy defaults (metres regardless of Rhino document units):

```json
{"strategy":"smooth_offset", "lateral_distance":1.0,
 "wall_distances":[0.4,0.6,0.8,1.0,1.2],
 "smoothing_windows":[10,25,50,100,200], "smooth_max_attempts":12}
```

Centred moving averages smooth XY positions and projected wall normals. Whole
paths are ranked by position/heading second differences and departure from the
TCP trend, rather than total distance travelled. Window sizes count input points;
uneven input spacing therefore changes their physical smoothing extent. Curves
use local normal offsets, without assuming a circular shape or scaling centre.
Both sideways signs are tested; `lateral_offsets` can explicitly override them,
for example `[-1.2,-1.0,-0.8,0.8,1.0,1.2]`.

Cheap placement checks eliminate unsuitable proposals before IK. The remaining
paths are tried across smoothing windows and both sideways signs, starting each
family near 1 m wall clearance before exploring other distances. This prevents
small score differences from consuming the budget on only one window.

If whole-path proposals fail, `connect_sections:true` (default) tries valid
partial paths and joins them. Defaults are `section_size:100`,
`section_proposals:6` and `section_beam_width:2`. Sections overlap by half their
target count. Position and shortest-arc yaw blend across the overlap using a
cubic smoothstep. Each joined prefix is solved again as a single arm/base path;
independent arm configurations are never simply concatenated. The bounded search
keeps up to two alternatives ranked by smoothness, reuses IK and identical
transition checks within the same scene, and returns output planes only after
the entire path passes validation. Logs include section ranges, joins, rejected
constraints and validated prefix length. Set `connect_sections:false` to disable.
`benchmarks/mobile_sections_benchmark.py` measures a synthetic stitching case.

If section joining also fails, up to `smooth_repair_attempts` (default 8, zero disables) adjust
the most promising path around its blocking target. Cosine-tapered X/Y shifts of
0.15 or 0.30 m extend over at least 50 targets on each side; accepted improvements
are retained. These are bounded geometric proposals, not relaxed constraints.
All original 3D TCPs, joint limits and enabled collision/transition
checks remain enforced, with one base plane per target on success. Diagnostics
report tried windows, wall distances, sideways offsets and validation failures.
Failure summaries refer to the best attempt rather than the last one;
`checked_in_any_attempt` reports aggregate target coverage. Counts and IK states
from different proposed paths are never combined into a fabricated solution.
Failure means the bounded proposal search was exhausted, not that fabrication
is impossible. There is no automatic dense grid fallback in this mode. Seed
planes, grid spacing, yaw samples and XY keyframe tolerance apply to the discrete
strategy only. Compare synthetic timing with `benchmarks/smooth_mobile_benchmark.py`.

### Previous discrete/sparse strategy

Use `{"strategy":"discrete","sparse":true}` in the dedicated component for
the previous regional search, or `{"strategy":"discrete","sparse":false}`
for uncapped full-domain search. The general numeric adapter retains its existing
discrete default unless `strategy:"smooth_offset"` or `strategy:"adaptive"` is explicitly supplied.

Sparse mode simplifies **world XY positions** with `xy_tolerance=0.05`
metres by default. Small sinusoidal ripples within that geometric tolerance can
be removed while larger square-wave turns, reversals and excursions remain.
It uses point-to-segment error on the ordered path, not local turn angles or
every Nth point. Z and TCP roll/pitch do not drive this simplification.
Projected wall-normal heading changes of `normal_angle=0.35` radians still add
keyframes to help preserve the correct placement side; set it to null to use
positions only. `z_tolerance` optionally retains large height changes (default
null). All original 3D targets are still validated.

The dedicated component has an optional `xy_tolerance` Item input in model
units (default 0.05 m / 50 mm). No new input is required to use the default.
The same option in `mobile_options` uses metres and takes precedence, e.g.
`{"sparse":true,"xy_tolerance":0.05}`. Default `max_gap` and `max_distance` are
null, so dense sampling and ripple arc length do not force extra keyframes.
They can be supplied as optional index and XY-displacement safeguards.
`sampling:"legacy"` restores the old 3D turn/orientation/gap selector.

It searches these keyframes, interpolates
base translation and shortest-arc upright yaw, then solves arm IK and checks
constraints at **every original target**. It never interpolates arm joint values.
Interpolation uses monotone projected progress along the simplified XY segments,
or elapsed time when `time_intervals` is given. Stationary XY segments use index
progress; small waves and Z oscillation do not accumulate extra base travel.
Failed full-resolution validation triggers a dense search of the original candidate
layers. If a keyframe itself exhausts its candidate region without any feasible
state, the search stops immediately: dense fallback would have the same empty
layer. Later regions are generated lazily, so they are not searched after that
failure. `unreachable_points` lists confirmed failed targets; `unchecked_points`
lists targets whose feasibility remains unknown. Their candidate counts are
null, not zero. This is infeasibility of the sampled domain, not proof that no
continuous placement exists. Diagnostics report retained indices, fallback and
elapsed time. `sparse:false` retains the full-domain diagnostic search.

Sparse mode is an approximate search: its interpolated placements may lie outside
the discrete offset set and its cost may differ from the full optimum. Use
`sparse:false` for exact search over the supplied discrete states. Both modes
share robot/tool calibration, fixed lift values, model joint limits and collision
setup with stationary arm planning. Stationary whole-path placement-region rules
are not imposed on a moving base. Supply environment meshes for obstacle checks.
Set `collision_check=True` and `check_edges=True` for sampled full-body transition
collision checks. This is a holonomic geometric planner, not a timed driving
controller; speed limits require positive `time_intervals` and explicit
`max_base_speed`, `max_yaw_speed`, and/or `max_joint_speed`. Acceleration and
nonholonomic steering constraints are not generated by this component.

The numeric APIs are `plan_mobile_base` (exact discrete search) and
`motion_toolbox.mobile_planning.plan_mobile_sparse` (validated sparse proposal).
The latter accepts lazy candidate layers and optional collision/transition
callbacks. Run `benchmarks/mobile_benchmark.py` for a synthetic comparison;
real robot and obstacle timings depend on the scene and candidate count.

Bounded sparse fallback retains states connected to the preceding target before
counting them toward `max_feasible_bases`. It first tries continuing previous
footprints, then the regional candidates. A blocked transition triggers up to
two local expansions (16, then 64 bases) at that target and its two predecessors;
IK results are reused. Exhausting this bounded search is not proof that no
physical path exists. Uncapped supplied-domain search remains exact.
Failures report the zero-based target pair, rejection counts, measured values
and limits for base/joint steps and speeds, and available collision-pair details.
Counts identify the first violated constraint per tested pair; an arbitrary
transition callback without collision details is reported as `transition_check`.
The fallback stops at a blocked prefix and marks later targets untested.

Minimum-cost graph search tests transitions in cost order and stops when it
finds the best valid predecessor. It preserves the same optimum and tie order;
selected transitions receive the same collision sampling. Exact path-count
mode still examines all admissible edges. Base-body placement checks skip
clearly separated link/obstacle pairs using conservative bounding boxes, refreshed
each call to account for moved geometry. Normal research recording retains
all rows but batches nested writes at step boundaries (0.5-second checkpoint
interval), with forced commits at root call boundaries and run close. An
interrupted process can lose its current uncommitted batch. Package version
and loaded-code fingerprints remain recorded. Compare these optimizations with
`python benchmarks/mobile_performance.py --baseline 4ef10ee`; saved component
measurements are in `benchmarks/mobile_performance.json`.

For target planes to **one stationary footprint plane** in Grasshopper, paste
[examples/grasshopper_stationary_base.py](examples/grasshopper_stationary_base.py)
into a Rhino 8 Python 3 component. Connect the ordered target planes and robot.
Mark optional parameters as **Optional** in Grasshopper so unconnected inputs
allow the script to run; `current_pose` and `arm_in_base` can also be removed
from the component inputs entirely.
When using the repository source, connect `toolbox_src` to its `src` directory.
The component refreshes an outdated cached adapter and reloads the planning
dependency chain together on first use or when its source files change. This
prevents mixed old/new functions from passing unsupported arguments downstream.
Unchanged recomputes reuse the loaded modules. Diagnostics report the loaded
`robot_adapter.py` path. If Rhino has already
loaded a different installation, restart Rhino after setting `toolbox_src`;
changing `sys.path` alone cannot replace modules already cached in that session.
`current_pose` is optional: when omitted, the objective includes only printing
motion between targets, and no approach from an initial arm pose is checked.
`arm_in_base` is an optional mounting-frame override: by default the adapter
uses the robot's initialized offline `_RCF` calibration plus `lift_height`.
It describes the arm controller frame relative to the footprint, including
mounting position and orientation. Without that calibration, the adapter uses
the URDF's recognized UR controller `base` link (for example `robot_arm_base`),
including its axis rotation. It reads upstream joint positions from
`fixed_joint_values`, or `robot.lift_height` for a single upstream prismatic lift;
the stationary component uses those same values for collision checking. No ROS
query is made. If neither calibration nor a recognized controller frame exists,
supply the override. `diagnostics` reports the mounting source and position.
For example, a plane at `(0, 0, 0.8)` with XY axes means the arm controller
origin sits 0.8 metres above the footprint with aligned axes. This is a fixed
mounting relationship, not the base's world placement or the arm's joint angles.
Arm joint names are
inferred from the robot/tool chain; an optional override handles ambiguous models. The
component generates placements from the common reach/side region automatically.
Every target's +Z must point away from the robot. The calibrated arm-base origin
must lie on the **negative-Z side of every projected target** and within **1.75 m
of every target origin in XY**. The guess deliberately ignores height; final IK
checks actual 3D reach. These geometric constraints also
apply to manually supplied `candidate_planes`, before IK is attempted.
The component ranks bases using **ground-plane geometry only**, maximizing the
minimum distance behind the projected target planes. It checks stationary robot
links against environment meshes without assigning any arm configuration. It then
validates the chosen position with target IK and configuration collision checks.
Collision failures retry other headings at the same shoulder position before
returning to the standoff-ranked candidates. IK failures prefer inward positions.
Retries check previously failed targets first, preserving original target order
in successful configuration outputs. The search allows up to `max_validation_attempts`
(default **3**; set **1** for a single final-position check). It stops at the first
fully reachable position, rather than evaluating IK for every candidate. The
winner's cached IK results feed **one** shortest joint-path search. Set
`build_path=False` to skip that search. It samples eight rotations around each target's local
Z axis by default (`rotation_steps`), retaining its position and normal. Counts
are specific to this discrete search, not a continuous-space optimum.
`ik_option_count` reports the selected base's IK combinations as text: these are independent target
configuration combinations, not verified motion paths. `path_search_count` is
0 or 1. Exact path counting is off by default; `path_count` is `not counted`.
Set `count_paths=True` to compute it as part of the final path search. This does
not change the chosen path or its cost. If joint-step limits disconnect that path,
the selected base is still returned with an empty joint plan and explanatory
status; the component does not build paths for the other bases.
`solution_counts` reports its feasible configurations per target. `diagnostics`
reports each base's coverage and failure reason, including wrong-side and
over-distance target indices (zero-based). Failed arm checks report raw IK counts,
counts after joint limits, and the links/tool/environment mesh indices responsible
for collision rejection. This uses existing checks, without repeating IK or collision
queries. Status uses one-based target numbers, such as `Failed at target 1 of 413`,
instead of ambiguous coverage fractions. Detailed counts stay in `diagnostics`.
Base-body collision diagnostics identify the robot link and obstacle mesh index.
`standoff` and `max_target_distance`
report the selected base's distances in metres. Failed searches also print
`status` and show a Grasshopper runtime warning, so a null plane has an explanation.
`validation_attempts` and `base_collision_checks` expose the work performed.
`timings` separates geometry generation, collision-world setup, IK, joint expansion,
collision checking and final path search; the same breakdown appears in diagnostics.
Per-base diagnostics summarize the minimum and maximum solution counts; the full
list remains available in `solution_counts`. The final path solver indexes full-turn
variants when every arm axis is revolute and joint step limits are below pi. It
keeps the same bounded configurations, path cost and optional exact path count,
while skipping impossible connections between winding variants.
`initial_base_plane` previews the first base-body-clear guess even if arm validation
fails; this output is not an arm-validated placement. A failed bounded search is
not proof that the whole segment is unreachable. The mobile base is freely placed,
but remains at one selected position for the segment.
Inputs default to **metres** (`units_to_metres=1`). The region combines XY reach
disks with the negative half-space of every projected target
plane. Initial guesses maximize minimum standoff in that region; additional
samples cover inward positions, the boundary and its XY interior. Footprints
account for the rotated mounting offset for each sampled heading.
No side-flip, arbitrary guess distance or search margin is needed. Legacy
`flip_side`, `guess_distance`, `search_margin` and `smart_initial_guess` inputs
are ignored by this component and can be removed.
`initial_guesses` previews geometry-valid seeds; full IK and collision checks
are still required. The circular reach sections use conservative 128-sided
polygons (at most 0.53 mm radial loss at 1.75 m). All candidates are rechecked
against the exact projected constraints. A finite search can miss feasible placements.
Supplied `candidate_planes` remain the exclusive search set and are checked
directly, without that polygon approximation.
The generic Python API retains minimum-travel ranking by default. Pass a
`StationaryRegion(projected=True)` as `placement_region`, `objective='heuristic'`,
and a `base_collision` callback for this
component's constraints and ranking.
PyBullet checks collisions **only at target configurations**, not between them.
Configuration checks default to enabled; **supply the wall mesh** as well as other environment obstacles to
check full robot/body clearance. Target planes alone do not describe solid wall
geometry. The strict side test excludes on-plane arm origins, but is not a body
clearance test. For this arm-planning component, contacts between two non-arm
assembly links (chassis/wheels/lift) do not reject arm poses: those links cannot
move relative to one another under the six planned arm joints. Their collisions
with the environment and all arm-versus-base contacts remain checked. Robot SRDF
disabled-collision pairs are honored. The generic collision class keeps static
self-collision checking enabled unless `check_static_self_collisions=False`.
Tool touch links include the rigid assembly connected to the attachment frame by
fixed joints (for example tool0/flange/wrist_3), but never extend across a movable
joint. Other tool/arm/environment collisions remain active. Full-turn-equivalent
joint configurations share one collision query per target/base while retaining
their distinct bounded joint values for path planning.
At zero clearance, self-collision checks use Bullet contact generation with spatial
filtering and allowed pairs configured upfront; positive-clearance checks keep
the distance-query route. Tool and environment checks remain distance queries.
Large path layers use joint-step intervals to exclude impossible predecessor pairs
before computing costs, preserving the shortest-path result.
One collision world is reused throughout the search. Default analytic IK geometry is UR20.
If PyBullet cannot load the robot, the error includes validation details and a
retained URDF path. Failed generated exports and their meshes are kept in a
`motion-toolbox-failed-*` temporary directory for inspection; successful exports
are cleaned up when the collision world closes. Collision geometry always comes
from `robot.model`; UR20 is the analytic IK default unless `ur_parameters` is supplied.

`stationary_base_candidates(targets, margin=1.5, spacing=0.5, yaw_steps=4)`
provides an unconstrained grid independently of Grasshopper (metres/radians). You can feed
the same domain to each layer of `plan_mobile_base`, or use the existing
`bases_around_targets` for separate local domains along a driving toolpath.
Stationary search optimizes one fixed placement; mobile search optimizes a
sequence with base-motion constraints. Neither automatically refines the grid.

```python
from motion_toolbox.base_planning import (
    grid_bases, bases_around_targets, find_stationary_base, plan_mobile_base,
)
from motion_toolbox.stationary_region import StationaryRegion

region = StationaryRegion(targets, solver.arm_in_base, max_distance=1.75, projected=True)
bases, guesses, explanation = region.candidates(spacing=0.5, yaw_steps=4)
stationary = find_stationary_base(
    targets, bases, current_pose,
    ik_solver=solver, collision=scene.is_valid,
    placement_region=region, objective='heuristic', build_path=True,
    base_collision=scene.is_base_valid, max_validation_attempts=3,
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


## Research recording

Meaningful operations now record structured SQLite runs and hashed, compressed input/output
artifacts automatically in the shared `~/Documents/GitHub/research_runs` folder,
independent of the working directory. Set `TOOLBOX_LOG_DIR` to override storage, or wrap related calls in
`motion_toolbox.recording.ResearchRun` to keep one experiment together.
`TOOLBOX_RECORDING=0` disables automatic recording; `TOOLBOX_LOG_LEVEL=detail` adds
individual IK/collision operations. See the [recording guide](docs/research-recording.md)
for coverage, metrics export, replay, storage and benchmark timing limitations.
