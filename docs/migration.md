# Extraction map and design decisions

Three independent local repositories:

| Repository | Responsibility | Required dependencies |
| --- | --- | --- |
| motion_toolbox | Numeric geometry, IK, collision, graph, placement, partial/rolling planning, planning I/O | NumPy; optional PyBullet and COMPAS FAB 1.x |
| toolpath_toolbox | Surface/curve toolpaths, geometric base candidates, path sampling and UR JSON | NumPy; RhinoCommon only for `rhino` modules |
| motion_toolbox_ros | ROS 2 transport and execution buffer integration | motion_toolbox and ROS 2 |

This avoids cyclic dependencies and avoids making offline users install ROS or
Rhino. Surface generation can change independently of robot planning. A separate
repository just for a small PyBullet wrapper would add packaging overhead without
an independent responsibility here.

## Retained functionality

| Original code | New location / behavior |
| --- | --- |
| mobile_motion_planning / ik_offline | `geometry`, `kinematics.ur`, `kinematics.solver`; explicit TCP and arm calibration |
| partial_trajectory | `planning.calculate_partial_trajectory`; optional prefix, rotations, diagnostics, collision, ranges |
| rolling_replan_node | `rolling.RollingPlanner` plus separate ROS package |
| slab / graph_based_optimum, graph_tool_optimum | `graph.shortest_path`; exact layered DP replaces both explicit graph engines |
| slab / ik_tools TCP rotations and IK | shared candidate generation and `URKinematics` |
| slab / PyBullet server and batch collision checker | `collision.PybulletServer`, callbacks and static workflow |
| slab / run_all.py | `utilities.static_planning.run_all`, `run_job` and `motion-plan` CLI |
| slab / attach_tool.py | `robot_adapter.attach_tool`; caller supplies geometry and TCP instead of numeric project catalog |
| slab / find_printlocation.py | `base_planning.find_stationary_base` and `evaluate_base_locations` |
| slab / check_for_nodes_without_valid_config.py | `utilities.io.unreachable_nodes` |
| slab / edit.py | `utilities.edit`; snapping, tolerance filtering and unwrapping |
| slab / utils.py | explicit `utilities.io`, pathlib and returned diagnostics; no repo path discovery |
| sprayed / plan_targets_with_base_positions.py | `planning.plan_targets_with_base_positions` alias and JSON job CLI |
| sprayed / generate_base_path.py | coupled `base_planning.plan_mobile_base`; IK/collision validated rather than a fixed offset line |
| sprayed / create_printpath_from_surface.py | toolpath toolbox `rhino.surface_toolpath.create_toolpath_from_surface` |
| sprayed / create_basepath_from_surface.py | `rhino.surface_edge_base_candidates` |
| sprayed / minimal_distance_from_curve.py | `rhino.curve_distance_base_candidates.solve`; geometric proposal, not IK proof |
| sprayed / fitness_plan_targets_no_graph_construction.py | shared `evaluate_base_locations` / `dont_build_graph=True`; counts, original fitness formula, base grid search |
| sprayed / gh_ik_solution_selector.py | `utilities.io.select_ik_solution` and `solutions_to_tree` |
| sprayed / ik_values_to_configuration.py | `robot_adapter.configuration_from_values`; explicit names/types including lift |
| sprayed / tcp_from_joints.py | `robot_adapter.tcp_from_joints`; offline model FK + active tool |
| sprayed / export_planes_to_json.py | `utilities.io.export_planes`; caller-chosen paths, optional metadata |
| sprayed / convert_joint_plan_to_ur_toolpath.py | toolpath toolbox `ur_toolpath`, same `rad` / `joints` / `move` schema |

Automatic filename conventions and component-global variables were replaced with
explicit arguments/return values. Display styling, widget tooltips, default research
log locations and sibling-repo bootstrap code are not library behavior. Diagnostic
results can be saved explicitly. Surface curves and geometric base proposals
remain optional Rhino algorithms; they do not decide physical reachability.

## Excluded as requested

Project vault/beam/ceiling/wall generation; `fab_manager`, `gcode`,
`information_model`, localization, fabrication tasks, nonoptimal heuristic/evolution
planners, Grasshopper internal collision detection, `run_current.py`, project
datasets, tool catalogs, IP addresses and machine paths. `docs/quadruped.py` loads
PyBullet's Minitaur example and displays inertial geometry; it is not part of the
fabrication planner and is omitted. No source repository content was removed.

## Deliberate corrections

- The original graph optionally sampled random start/end nodes, so bounded
  iteration mode was not guaranteed optimal. The new solver considers all states.
- The first motion edge obeys the joint-step limit too; previously it was exempt.
- Bounded joints do not wrap by default. Continuous joints can opt into wrapping.
- Joint range filtering checks all specified joints and does not append a candidate
  prematurely while still examining it.
- Missing/failed live robot pose reads no longer fall back to hardcoded joints.
- FK no longer changes its input list and now applies both joint convention offsets
  needed to invert the existing IK wrapper.
- Numeric plane construction does not mutate caller-owned NumPy arrays.
- Collision loading uses an isolated Bullet client and explicit joint name order.
- Moving-base collision checking uses each target's base placement.
- Rolling odometry uses calibrated full-pose mapping, replacing a hardcoded X-to-Y
  translation conversion. Committed targets seed buffer extensions.
- All missing-path conditions remain visible; no target is silently omitted.

## Rhino geometry: recommendation and tradeoffs

Keep numeric planes/transforms inside the IK/graph loops and convert at the API
boundary. Rhino planes are accepted as inputs; there is no need to require Rhino
throughout the solver merely to use them in Grasshopper.

RhinoCommon's strengths are its modeling operations, trimmed surfaces, curve
evaluation, tolerances, meshing and direct Grasshopper interoperability. Those are
useful in the retained surface/curve package. Costs include a Rhino runtime,
deployment/licensing requirements, .NET interop, and less convenient vectorized
array processing. Conversion has a cost too, so convert once per input batch, not
for every candidate edge. This recommendation follows the operations these planners
perform; no claim is made that all Rhino geometry operations are slower.

`rhino3dm` provides a RhinoCommon-style openNURBS API independently of Rhino, useful
for `.3dm` interchange and basic geometry, but it is not a replacement for every
RhinoCommon modeling operation. `Rhino.Inside` embeds the Rhino engine for richer
operations outside its UI, while retaining a Rhino installation/runtime dependency.
Sources: [rhino3dm](https://www.rhino3d.com/features/developer/rhino3dm/),
[Rhino.Inside](https://www.rhino3d.com/features/rhino-inside/),
[RhinoCommon guides](https://developer.rhino3d.com/guides/rhinocommon/).

## Validation boundary

The automated tests use synthetic robots/obstacles and COMPAS FAB 1.x objects,
plus read-only source IK replay. A separate smoke test also loads the actual
`MobileRobot` and `MultiTool` classes and verifies model export, active TCP,
lift/calibration and attached mesh collisions with a synthetic model. Actual
MobileRobot calibration, attached geometry,
allowed collision pairs and controller conventions must be supplied by the caller.
ROS transport must be exercised in the deployed ROS distribution. The mobile planner
is a discretized holonomic planner with sampled edge validation, not an acceleration-
constrained navigation stack or continuous whole-body trajectory optimizer.

Rhino 8.30 headless tests passed for raster (322 points), sine (338 points),
surface-edge base sampling (20 planes), curve clearance and plane conversion.
The source files remain in place; SHA-256 verification covers all 104 original
Python files captured at the beginning of this extraction.
