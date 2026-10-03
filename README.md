# Motion toolbox

See [CHANGELOG.md](CHANGELOG.md) for changes between package versions.

Reusable offline IK, PyBullet collision checking, exact layered motion planning,
stationary base placement, XY position averaging, and rolling replanning.
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
collisions. [examples/grasshopper_arm.py](examples/grasshopper_arm.py) is the complete
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
The arm, stationary-base and mobile-base components default to `rotation_steps=16`,
sampling a full turn per target. Use `1` when target
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

## Adaptive mobile base planning

`geometry_mode="auto"` preserves existing offsets for mostly straight sections
and uses outward radial offsets for accepted circular sections. Repeated passes
share an arc center fitted to the original XY TCP positions, with separate
pass radii. `tangent_offset` is signed distance along the enlarged base circle;
nominal base +X points toward its center, +Y is tangent, and +Z stays upright. Traversal
reversals do not flip orientation. The yaw slider still rotates each final frame.
`geometry_mode="legacy"` selects the previous geometry for comparison.

Optional `geometry_options` (a dictionary, or JSON in Grasshopper) uses metres
and degrees: `arc_turn_threshold_deg=15`, `arc_fit_rms=0.03`,
`arc_fit_max=0.075`, `transition_length=0.5`, `reversal_excursion=0.5`.
Shallow arcs must fit the original points over the complete pass; their turn is
measured from that fit, so averaging cannot erase the curvature evidence.
If the whole-pass fit fails, subdivision still requires at least 45 degrees of
strong-curvature evidence (or the configured threshold when higher). This avoids
turning small ripples into a sequence of short arcs. To restore the higher
arc threshold, use `geometry_options={"arc_turn_threshold_deg":45}`.
Strong sections that cannot be fitted are reported explicitly. The result's
`path_sections`, `section_ids`, `transition_regions`, and
`classification_seconds` explain the selection and fitting. Each section's
inclusive `first`/`last` is its fit interval; `section_ids` assigns unique target
ownership at shared endpoints. Curved `applied_offsets` are radial/arc distances.
Grasshopper exposes section and transition records as readable JSON strings;
the numeric result retains structured records and arrays. Position and yaw blend
with quintic ramps over at most 0.5 m of spatial path at geometry boundaries.
These settings are also available in the offline replay command through
`--geometry-mode`, `--geometry-options`, `--normal-offset`, `--tangent-offset`,
and `--base-yaw-degrees`.

Mobile-created collision worlds now use `collision_options={"base_collision_model":"auto"}`
by default. For the chassis/four-wheel/Ewellix robot this installs two actual 3D
collision boxes: the robot-aligned bounds of the chassis and four wheels, and
the bounds of both lift bodies at the configured extension. These boxes rotate
with the base and fill the chassis chamfers to represent its cover. All other
base accessory collision shapes, including lidar, cameras and GPS, are omitted.
The complete arm subtree and attached tool retain their original collision
geometry. Both preliminary and final environment/self/tool checks use this model.

`base_collision_model="boxes"` requires the recognized link structure;
`"auto"` keeps other robot models detailed. `"detailed"` restores the original
base collision bodies for comparison. The generic `PybulletServer` defaults to
`"detailed"`; set its `base_collision_model` explicitly and pass
`fixed_joint_values` at construction for cover boxes. A different lift/wheel
configuration requires a new box world, preventing stale bounds. Frames and
source robot assets remain unchanged. `base_collision_geometry` reports effective
mode, box centers/sizes in their mounting-link frames, contributing links,
replaced/omitted collision links and fixed-joint values. Box bounds conservatively
include PyBullet's collision margins. Offline replay accepts
`--base-collision-model auto|boxes|detailed`.

In detailed mode, mobile-created collision worlds omit the absent GPS antenna (`gps_base_link`,
`gps_link`, including robot-prefixed names) from all collision checks. Frames
remain intact. `collision_options` accepts `exclude_gps=False` to include it or
`excluded_collision_links` for additional explicitly absent hardware. Results
report effective exclusions. External scenes retain their supplied geometry;
configure their exclusions when creating the scene. Offline replay excludes GPS
by default; `--include-gps` restores it. Full base/arm/tool 3D configuration
checks remain authoritative.
Restoring GPS or other accessories requires detailed mode; cover mode intentionally
omits them. Previously validated detailed-model paths must be revalidated for the
cover: 85 configurations in today's old trajectory overlap the enlarged chassis.
The new cover-aware replay validates all 1,481 targets as a connected trajectory;
see the [cover model validation report](docs/cover-collision-validation-20260922.md).
Preliminary checks skip empty frame links. On 1,481 identical cover-model poses,
three matched trials reduced base-check time by 14–38% with unchanged decisions
and failure reasons; this is a base-check measurement, not total planning time.

Export paired controller paths with
[examples/grasshopper_export_paths.py](examples/grasshopper_export_paths.py), a
Rhino 8 Python 3 component. Its file header documents all inputs and outputs.
Connect a Panel to the built-in `out` output for the printed export destination,
file paths, warnings and errors. Connect `planned_tcp` to its `tcp_planes` List/Plane
input, the corresponding `base_planes` List/Plane input, and `speed` in **cm/s**.
It creates `arm_path.json` and `base_path.json` in a new
`Documents/yyMMdd_HHmm_robot_path` folder, with numbered suffixes to preserve
earlier exports. Both files use `vicon_world`, metre positions, normalized xyzw
quaternions and identical `{sec, nanosec}` timestamps derived from full 3D TCP
travel. Base geometry and index pairing are unchanged; equal timestamps do not
mean equal base and TCP speeds.

Outputs include `tcp_file`, `base_file`, `export_folder`, `time_seconds`,
`duration_seconds`, `diagnostics` and `status`. Every recompute exports by default;
set `write_files=False` for a timing preview. Optional `documents_folder`,
`frame_id`, and `model_units_to_metres` (default 1) override location/frame/units.
The supplied exporter's checks are preserved: TCP maximum height must exceed
1 m, and base XY bounding-box extent must be at least 0.9 m. Set
`sanity_checks=False` for intentionally smaller paths. Rotation-only/repeated TCP
origins receive no extra duration; diagnostics report repeated timestamps.
Exporting does not revalidate IK, collisions or speed/acceleration limits.

`validation/plot_export_speeds.py INPUT_FOLDER --output OUTSIDE_GIT` plots
consecutive-pose average arm/base translational speed (requires Matplotlib), and
saves a PDF, PNG, segment CSV and summary with source hashes. These are planned
speeds inferred from exported positions/timestamps, not measured velocities.

Load [examples/grasshopper_mobile_base.py](examples/grasshopper_mobile_base.py)
by file path and recompute. Required inputs are `robot` (Item) and
`target_planes` (List). The robot must carry its active calibrated tool.
The component generates smooth XY passes, chooses straight or radial geometry,
offsets **each pass** 1.0 m outward and 1.3 m tangentially, and validates
one upright ground base plane per original TCP. By default it samples 16
rotations (22.5-degree spacing) around each TCP local Z axis, keeping its position
and extrusion direction fixed. `rotation_steps=1` restores fixed orientation.
The exact shortest arm trajectory for the selected base path is found among all sampled orientations;
`selected_target_planes` and `selected_tcp_rotations` (radians) report the
chosen orientations only when the complete path validates.

It checks calibrated arm-origin XY reach (1.75 m), negative target-Z placement,
base-body collisions, real calibrated IK, joint limits, robot/tool/environment
collisions at every original target, base translation/yaw steps and joint continuity.
Transitions are **not collision-checked** (`check_edges=False` in results).
Optional speed limits require `time_intervals`.

`adapt_offsets=True` treats the supplied offsets as preferences. Failures trigger
local offset searches with overlapping, expanding intervals, 0.10 m coarse spacing
and refinement to 0.025 m. Quintic ramps retain smooth joins; all original targets
and movement constraints are checked after anchor screening. Failed sampled repair
searches do not prove that no continuous solution exists. `adapt_offsets=False`
keeps the original fixed-offset behavior.

Arc repairs reuse the prepared centers and section mapping. Anchor groups get
placement and detailed base checks before generating IK. Other known failures
outside the current repair interval remain for later repairs; each proposed path
still receives complete original-target validation. All path geometries pursue
the best improving repair first and retain alternatives for a dead end.

The production strategy starts with graph search. For every path geometry, a
collision in the chosen path triggers exact checks of every candidate in that
target layer.
All verified colliding candidates are removed together before solving the graph
again. This avoids repeated full graph rebuilds in collision-heavy sections while
preserving distinct joint states, limits, costs and tie breaks. Every selected
configuration must pass the detailed checker. Layers whose selected candidate
passes remain lazy; their alternatives do not need immediate collision checks.

`base_planes` and `base_path` remain visible on a failed proposal. Only `valid`
(or `result['fabrication_validated']`) indicates a complete validated path;
`configurations` and `joint_plan` remain empty on failure. `diagnostics` reports
per-target rejection categories, collision pairs when available, placement
measurements and disconnected transitions. `target_indices` preserves input
order; `averaged_line` and `centerline` expose the geometric stages.
`applied_offsets` reports normal/tangential offsets in metres, `repair_attempts`
records repair decisions, and `research_run` locates the structured research record.
Unused candidate collisions are explicitly untested, not labeled collision-free.

Input geometry defaults to metres, matching the captured cases, regardless of
Rhino's document units. Set `model_units_to_metres` or
`units_to_metres` to 0.001 for millimetre-valued geometry. Component geometric inputs are model units:
`max_xy_deviation` defaults to 0.25 m, `normal_offset` to 1.0 m,
`tangent_offset` to 1.3 m and `max_base_step` to 0.25 m after unit conversion.
Add an Item input named `base_yaw_degrees` and connect a Number Slider (suggested
range -180 to 180, default 0). Positive angles rotate the base counterclockwise
viewed from above, at each base origin, with Z upright. Offset directions remain
relative to the prepared straight/arc geometry. IK, placement and collision checks use the rotated
base orientations, including during adaptive repairs.
`base_yaw_margin_degrees` defaults to **30**, allowing adaptive rotations up to
**+/-30 degrees** around the prepared straight/arc orientation plus
`base_yaw_degrees`. Set the margin to **0** to restore fixed-yaw repairs.
Already-valid paths keep their nominal yaw. With `adapt_offsets=False`, neither
offsets nor yaw adapt. The margin accepts values from 0 to 180 degrees and is
independent of model units and the per-transition `max_yaw_step` limit.

Repairs try rotation at the current position before translated candidates and
retain one viable angle per offset candidate to bound the search. Angles use
5-degree spacing with the exact margin endpoints included, then local refinement
to 2.5 and 1.25 degrees. Position and yaw use quintic repair ramps; every original
target and join must still pass reach, base/arm/tool collision, joint, translation
and yaw movement checks. Combined radial/tangential/yaw repairs regenerate the
section geometry before applying the additional rotation. A sampled search can
miss a feasible continuous solution.

`applied_yaw_adjustments_degrees` reports each target's additional signed adaptive
rotation. `applied_base_yaw_degrees` adds the fixed yaw slider to that adjustment;
both are relative to the prepared geometric heading, not world compass headings.
`applied_offsets` remains a two-column normal/tangential array. Grasshopper accepts
an optional Item input named `base_yaw_margin_degrees` and exposes
`applied_yaw_adjustments_degrees`; offline replay accepts
`--base-yaw-margin-degrees 30`.
The [adaptive-yaw capture replay](docs/adaptive-yaw-validation-20260922.md)
validates all 1,481 targets with final adjustments between -21.25 and +21.25 degrees.
`max_yaw_step` defaults to 0.25 rad, `max_joint_step` to 2.5 rad.
Robot models, tool calibration, fixed joints and collision-option lengths use
metres/radians. See the script docstring for all inputs. Old switches disabling
configuration collision checking are rejected explicitly. Legacy `check_edges` inputs
do not enable swept checks in the mobile component.

The component, shared planning APIs and offline replay have no runtime timeout.
On Windows, mobile planning requests HighQoS for its calling thread during the
calculation and restores the prior thread policy afterward. This prevents hidden
or background windows from silently reducing planning throughput. The request is
advisory; it does not change the process priority, CPU affinity or system power
plan. `execution_policy` and the research events report application/restoration
and any unsupported-platform fallback.
An optional current pose describes the arm at the first proposed base; approach
motion from another footprint is not included. Stationary planning is unchanged.

Run captures offline in separate processes:

```powershell
python validation/validate_mobile_base_case.py CASE --output OUTSIDE_GIT --rotation-steps 16
```

The harness verifies READY and every manifest hash, restores captured URDF,
calibration, tool/body/environment collisions, fixed joints and allowed pairs,
and records results with normal research logging. End-to-end timings include process
startup and setup; `--fixed-offsets` disables repairs for comparison. Captures and
trial outputs stay outside Git.
`validation/compare_section_modes.py CASE --output OUTSIDE_GIT` compares old
captures in an unchanged collision world. It asserts identical auto/legacy base
matrices, validates both joint paths and compares their configurations and cost.
Its default offsets are 0.9/1.2 m, matching the historical validated captures;
it defaults to detailed base geometry for those historical comparisons. Pass
`--base-collision-model boxes` to compare using the cover. The planner's current
offset defaults remain 1.0/1.3 m.

Experimental base screens are available only through the offline harness:
`--base-screening rectangle` or `--base-screening box`. The rectangle checks a
rotated robot-derived footprint against obstacle triangles clipped to world
Z=0–1 m. It retains concavities and tests containment. The box uses the configured
static robot bounds in PyBullet. Both include fixed-joint transforms and omit GPS.
Possible overlap always falls back to the exact base checker. Unsupported slab
heights, clearance or scene geometry also fall back; final 3D configuration checks
are unchanged. Screens require an immutable scene and fixed-joint settings.

Reproduce three matched screening trials with
`validation/benchmark_base_screening.py CASE --result VALID_RESULT_JSON --output OUTSIDE_GIT_JSON`.
On today's 2,962 preferred and repaired poses, both screens produced identical
base decisions and failure reasons but were slower in all three trials. The
production default therefore remains the existing checker. Enabling a shortcut
would additionally require at least 10% lower screening time and three matched
end-to-end runs without slowdown or changed final results. See the
[curvature validation report](docs/curvature-validation-20260922.md).

The geometry-only modules (`xy_averaging`, `xy_smoothing`, `xy_centerline`,
`xy_offset`) and their plotting scripts remain available for experimentation.

## Stationary base planning

For target planes to **one stationary footprint plane** in Grasshopper, paste
[examples/grasshopper_stationary_base.py](examples/grasshopper_stationary_base.py)
into a Rhino 8 Python 3 component. Connect the ordered target planes and robot.
Attach the calibrated active tool first (for the selected `group`, when supplied).
The stationary component rejects a missing active tool before IK or collision
setup, even with `collision_check=False`; it reports the error in `status` and
the component warning and leaves placement/path outputs empty.
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
Supply six arm radians, or set the input to Item access with no type hint and
supply a named COMPAS `Configuration`. Named values are reordered into the arm's
analytic order; non-arm values (including lift) become fixed joints. Explicit
`fixed_joint_values` override values from that configuration. The starting pose
must satisfy joint limits and, when collision checking is enabled, must be
collision-free at the candidate base. A start collision retries other headings
under the same bounded placement policy. `configurations` returns named
configurations containing all fixed non-arm joints followed by the six arm joints;
`joint_plan` retains its six-arm-values-per-target DataTree format.
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
winner's cached IK results feed **one** joint-path search (which may solve the
graph repeatedly as colliding configurations are rejected). Set
`build_path=False` to skip that search. It samples sixteen rotations around each target's local
Z axis by default (`rotation_steps`), retaining its position and normal. Counts
are specific to this discrete search, not a continuous-space optimum.
`fast_validation=True` is the default: establish collision-free reachability at
every target, then validate configurations on proposed shortest paths. Collision
geometry, placement retries, joint limits and the optimal selected path are
unchanged. Unused alternatives are not exhaustively checked: `solution_counts=[]`,
`ik_option_count='not counted'`, and `counts_complete=False` explicitly report
that exact alternative counts are unavailable. Set `fast_validation=False` for
exhaustive checking and exact counts. `count_paths=True` automatically forces
exhaustive checking. With `build_path=False`, fast validation stops after proving
per-target reachability. Windows planning requests scoped HighQoS and restores
the previous thread policy afterward; it does not change affinity or priority.
For Python callers, this toggle applies only to `objective='heuristic'`;
`BasePlan.counts_complete` is false and `BasePlan.ik_option_count` is `None`
when fast validation omits exact counts. Other objectives retain their behavior.
The reproducible [stationary benchmark](validation/benchmark_stationary_fast.py)
uses synthetic IK candidates and real Bullet collisions; its
[measured results](benchmarks/stationary_fast.json) are workload-specific.
In exhaustive mode, `ik_option_count` reports the selected base's IK combinations as text: these are independent target
configuration combinations, not verified motion paths. `path_search_count` is
0 or 1. Exact path counting is off by default; `path_count` is `not counted`.
Set `count_paths=True` to compute it as part of the final path search. This does
not change the chosen path or its cost. If joint-step limits disconnect that path,
the selected base is still returned with an empty joint plan and explanatory
status; the component does not build paths for the other bases.
`time_intervals` supplies positive seconds per transition: N-1 values without a
starting pose, or N with one (first duration is the approach to target 0).
`max_joint_speed` accepts one positive rad/sec value or six values in arm joint
order. Each transition uses the smaller of `max_joint_step` and speed times
duration. Times alone do not impose a speed limit. This constrains sampled joint
motion; it does not certify acceleration, controller timing, or swept collisions.
`joint_ranges` optionally overrides arm ranges using a JSON list of [min,max]
radians (null for an unbounded joint).
`planned_tcp` is a list of world TCP planes in input model units, including the
TCP-Z rotations selected for the joint path, in original target order. Add an
output named `planned_tcp` to the GH component. It is empty when no complete
path exists, `build_path=False`, or an error occurs. Collision validation follows
`collision_check`; transitions remain unchecked.
In exhaustive mode, `solution_counts` reports its feasible configurations per target. `diagnostics`
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
list remains available in `solution_counts`. Candidate generation retains only
the IK solver's returned joint representatives, without +/-360-degree expansion.
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

The stationary component also exposes the following outputs. Add parameters with
these exact names to the Grasshopper component when needed:

| Output | Meaning |
| --- | --- |
| `path_complete` | A connected joint path covers every target. |
| `valid` | That complete path was checked for configuration collisions; false when collision checking is off, path building is off, or planning fails. Transitions remain unchecked. |
| `disconnected_detail` | JSON with the blocked transition, effective joint-step limits, and nearest candidate deltas/limit ratio. Indices are zero-based; -1 means the starting pose. |
| `initial_state_failure` | Starting-configuration collision reason when candidate attempts fail. |
| `unreachable_points` | Failed target indices when no base succeeds. |
| `progress_messages` | JSON messages for setup, placement, IK, collisions, graph search and completion; also printed to `out`. |
| `cancelled` | Cancellation cleared the trajectory and placement outputs. |
| `effective_settings` | Resolved units, calibration, joint settings, fixed values, collision policy and search settings as JSON. |
| `loaded_code` | Package/API versions, module locations, file hashes and loaded-code hashes as JSON. |
| `research_run` | The single recording folder covering setup through the final result, or None when recording is disabled. |
| `result` | Full Python workflow result including named configurations, numeric planes and diagnostics. |

`cancel=True` skips the run. For interruption during computation, `cancel_check`
accepts a callable that returns True or raises
`motion_toolbox.execution.PlanningCancelled`. Checks occur between geometry,
IK, collision and graph operations; a single native operation cannot be interrupted
mid-call. A Grasshopper boolean toggle alone cannot interrupt an already running
synchronous script. `progress_callback` optionally accepts progress dictionaries.
Cancellation returns no partial trajectory and restores the previous CPU policy.

`collision_options` accepts JSON. `clearance` and `ground_z` are always metres,
independent of input geometry units. For example:

```json
{"clearance": 0.01, "ground_z": 0.0, "support_links": ["actual_wheel_link_name"]}
```

Use real robot link names. `support_links` exempts those links from ground contact
checks; other robot/environment and tool contacts remain checked. `allowed_pairs`
specifies pairs of robot link names with intentional contact. Additional options
are `check_static_self_collisions`, `exclude_gps`, `excluded_collision_links`,
`base_collision_model` (`detailed`, `auto`, `boxes`), `package_paths`, `asset_root`,
and `gui`. Unknown options are rejected. Defaults retain detailed base geometry,
GPS coverage and the existing static-self-collision policy. Changing exclusions
or geometry is explicit and reported in `effective_settings`.

`collision_scene` accepts an already configured `PybulletServer`. It must match
the current robot, active tool and environment; joint order is checked, while the
caller remains responsible for model/geometry freshness. The scene remains open
after success, failure or cancellation. Its fixed joints and current pose are
updated during planning. Omit `collision_meshes` and scene-construction options;
only `clearance` may accompany a supplied scene. Rebuild or update the scene when
geometry/tool settings change, and do not share a mutable scene across concurrent
planning calls. An internally created scene is always closed by the workflow.

For Python callers, `motion_toolbox.stationary_workflow.plan_stationary_base`
provides the same robot-facing workflow and keyword inputs, with `scene` for
`collision_scene`, `progress` for `progress_callback`, and `parameters` for
`ur_parameters`. It returns a dictionary; errors and cancellation raise exceptions.
`effective_settings` and `loaded_modules` remain dictionaries there. The workflow
uses one automatic recording (or reuses an active `ResearchRun`); setting
`TOOLBOX_RECORDING=0` disables automatic recording. Native/custom objects and
externally created scenes can require additional assets for replay; see
[research recording](docs/research-recording.md).

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
PyBullet checks collisions **at target configurations and the supplied starting
configuration**, not between them.
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
provides an unconstrained grid independently of Grasshopper (metres/radians).
Stationary search optimizes one fixed placement within its finite candidate domain.

```python
from motion_toolbox.base_planning import (
    grid_bases, find_stationary_base,
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

```

Stationary search requires a connected arm path through **all** targets, not just
a large total IK count. `evaluate_base_locations` supplies candidate-only counts,
unreachable indices and the original fitness formula for Grasshopper exploration.
A finite candidate domain does not establish a global continuous-space optimum.

Bounded joints use actual angle deltas. Set `periodic` only for physically
continuous joints; otherwise an apparent short wrap can exceed real limits. The
shared candidate evaluator filters the IK solver's original joint angles against
joint limits without adding equivalent revolutions. This applies to all planners.
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
rolling buffer behavior, stationary feasibility, joint limits, isolated
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

The `grasshopper_arm.py` and `grasshopper_mobile_base.py` components expose
`planned_tcp`: add an output with that name to receive the selected world TCP
planes in model units, including each chosen rotation about TCP Z. Output order
matches the planned configurations. It is empty if no complete path is found.
These are planned waypoints, not sampled controller interpolation between them;
the regular component's collision coverage still follows its collision settings.

Meaningful operations now record structured SQLite runs and hashed, compressed input/output
artifacts automatically in the shared `~/Documents/GitHub/research_runs` folder,
independent of the working directory. Set `TOOLBOX_LOG_DIR` to override storage, or wrap related calls in
`motion_toolbox.recording.ResearchRun` to keep one experiment together.
`TOOLBOX_RECORDING=0` disables automatic recording; `TOOLBOX_LOG_LEVEL=detail` adds
individual IK/collision operations. See the [recording guide](docs/research-recording.md)
for coverage, metrics export, replay, storage and benchmark timing limitations.
