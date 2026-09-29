# Changelog

Notable changes by package version, newest first. Historical entries were
reconstructed from Git commits and `pyproject.toml`; dates are commit dates,
not evidence of a published release. Commit references identify the source.

## Unreleased

- Keep the Grasshopper export input/output reference in the component file;
  remove the separate document and clarify which script Grasshopper should load.

## 0.1.45 - 2026-09-29

- Restore printed export summaries and file locations in the Grasshopper
  exporter's built-in out output, including preview notices, warnings and errors.
- Document every GH input/output, access mode, type hint, default and unit in
  the script header and docs/grasshopper-export-paths.md. Timing and export
  payloads are unchanged; no dependency changes.
- Verify printed success paths, preview behavior and errors in the actual
  component regression test.

## 0.1.44 - 2026-09-29

- Fix controller-path export imports when a helper is loaded without a Python
  package context. The GH entry point remains examples/grasshopper_export_paths.py.
- Allow the GH exporter to run as script text without __file__, with an explicit
  toolbox_src, or with a generated GH script path. Discover adjacent source only
  when present; otherwise use the installed toolbox. Export timing is unchanged.
- All 27 exporter tests pass, including file loading and all three GH text-loading forms.
  No dependency changes.

## 0.1.43 - 2026-09-29

- Add grasshopper_export_paths.py and reusable numeric controller-path export:
  paired metre poses, normalized xyzw quaternions and one shared TCP-distance
  timestamp list. Preserve the supplied sanity checks and timestamped Documents
  folders; add preview and unit/frame/location overrides. No resampling is added
  because the supplied example already has matching timestamps at every index.
- Add a reproducible arm/base speed plot and CSV/JSON summary from controller
  exports. The 1,760-pose example has 4.00 cm/s TCP speed and 0.22–7.49 cm/s base
  speed; all paired timestamps match. Outputs remain outside Git.
- Record export timing/files and aggregate pose/timestamp counts while keeping
  full payload artifacts, package version and source fingerprints. Update GH
  recorder refresh guards (recording API version 8); no planner changes or new
  runtime dependency. Matplotlib is optional for the analysis script.
- All 43 export/recording tests pass, covering shared 3D timing, quaternion branches, units, invalid inputs, unique
  folders, incomplete-write cleanup, GH preview/export and recording. The actual
  component reproduces all 1,760 example positions and orientations; independently
  recalculated timestamps differ from the old export by at most 1 ns while the
  new arm/base timestamps match exactly.

## 0.1.42 - 2026-09-29

- Lower the configurable arc detection threshold from 45 to 15 degrees. Shallow
  arcs require an acceptable whole-pass fit to original TCP XY positions; use
  that fit's turn to avoid smoothing erasing or inventing curvature evidence.
  Preserve the 45-degree strong-curvature gate for subdividing failed fits, so
  lowering the threshold does not turn meander ripples into short fitted arcs.
- Keep fit tolerances, planner constraints and collision coverage unchanged.
  Update GH version guards; no dependency or input changes.
- Geometry replay of the actual 872-target input identifies all 19 passes as
  arcs. All 34 geometry/yaw regression tests pass, covering shallow arcs, overrides, ripple
  rejection, mixed sections and upright radial offsets. This is geometry
  validation, not an IK/collision-certified solution or a runtime claim. All four
  earlier captures retain their geometry classifications; see
  docs/arc-threshold-validation-20260929.md for evidence and reproduction.

## 0.1.41 - 2026-09-29

- Apply exact failed-layer collision completion to non-arc mobile paths too.
  Remove all verified colliding candidates in a failing target layer together,
  avoiding one whole-path graph solve per colliding alternative. Preserve exact
  joint-path costs, ties, constraints and configuration collision coverage.
- Pursue the best improving offset/yaw repair first for every path geometry,
  retaining the existing alternative beam for dead ends. Record the traversal
  policy. Already-clear layers and single-target anchor probes remain lazy.
- Refresh Grasshopper version guards. No dependency or input changes, runtime
  cutoff, joint-turn expansion or transition collision checks are introduced.
- Validation: 96 relevant tests pass, covering non-arc success/failure and collision-free fast-path regressions,
  eager/lazy optimality comparisons, offset/yaw repairs and GH entry-point tests.
  A synthetic 872-target benchmark with normal research logging reduces graph
  solves from 32 to 2 (54.98 to 3.28 s graph time), with identical configurations,
  cost and 27,904 collision checks. This is not captured-robot validation; see
  docs/non-arc-validation-20260929.md for measurements and limitations.

## 0.1.40 - 2026-09-25

- Add `planned_tcp` to the mobile and regular Grasshopper components: selected
  world TCP planes in model units, ordered with the planned configurations and
  including chosen TCP-Z rotations. Clear the output on failure.
- Retain rotation provenance through candidate deduplication, joint/collision
  filtering and exact graph selection in the prescribed-base planner. Reuse
  existing selected planes in mobile planning; no extra IK or FK pass is added.
- Validation: 57 relevant tests pass, including rotated target selection after
  filtering, disconnected-path clearing, JSON export, units and GH entry points.

## 0.1.39 - 2026-09-22

- Add base_yaw_margin_degrees, default 30, for adaptive orientation changes
  within +/-30 degrees of the prepared heading plus the fixed yaw slider.
  A zero margin preserves fixed-yaw repair behavior; already-valid paths and
  adapt_offsets=False retain nominal orientations.
- Search yaw at the current position before translated candidates, then refine
  combined offset/yaw repairs. Angular spacing is 5 degrees with exact margin
  endpoints, refined to 1.25 degrees. Quintic ramps and complete original-target
  checks retain reach, detailed configured collisions, joint and movement limits.
  Arc geometry is regenerated before applying each adaptive yaw adjustment.
- Preserve two-column applied_offsets; add per-target adaptive/total relative
  yaw outputs and repair metadata. Forward margin settings through numeric APIs,
  Grasshopper and offline replay. No new runtime dependencies.
- Validation: all 263 tests pass, including both rotation directions, wrapped
  headings, custom hard margins, combined signed arc offsets/yaw, complete
  configuration checks and yaw-step constraints. Today's cover-model capture
  validates all 1,481 targets with final yaw adjustments within +/-21.25 degrees
  (observed runtime 7m 50s). Fresh committed-code checks confirm the regenerated
  poses, hard margin and every final configuration. See
  docs/adaptive-yaw-validation-20260922.md for provenance and timing limits.

## 0.1.38 - 2026-09-22

- Represent the mobile robot's cover with one aligned collision box enclosing
  the chassis and four wheels, plus one box enclosing the configured lift.
  Use these bodies in preliminary and final 3D environment/self/tool checks.
  Retain detailed arm/tool geometry and every coordinate frame; omit lidar,
  cameras, GPS and other base accessory shapes without editing source assets.
- Mobile worlds and replay default to base_collision_model="auto", which uses
  boxes on the recognized chassis/Ewellix robot and retains other models.
  Explicit "boxes" requires that profile; "detailed" restores prior geometry.
  Generic collision worlds remain detailed by default. Pass fixed_joint_values
  when constructing a box world; changing them requires rebuilding its bounds.
- Report contributing links, local box dimensions, replaced/omitted shapes and
  fixed-joint settings. Record the generated collision URDF through the existing
  recorder. Forward the model option through GH and offline replay.
- Actual capture check: the chassis/wheel box is approximately
  1.282 x 0.872 x 0.669 m; the lift box is 0.208 x 0.208 x 1.000 m, including
  collision margins. The enlarged chassis rejects 85 of the prior trajectory's
  1,481 configurations, so old detailed-model validation does not transfer.
- Regression coverage includes corner filling, base rotation, lift extension,
  stale-bound prevention, preserved arm collisions, inertial-frame offsets,
  source/frame preservation and fallback for other robot models. No new runtime
  dependencies. Historical mode comparisons retain detailed geometry by default.
- Preliminary checks now skip links with no collision shapes. Three matched
  1,481-pose trials with the same cover model return identical decisions/reasons
  and reduce base-check time by 14–38%. This does not claim an end-to-end speedup.
- Validation: all 252 tests pass. A new cover-aware replay validates all 1,481
  targets in one connected trajectory (observed runtime 17m 20s). A separate
  fresh-world check with committed code confirms every final configuration.
  See docs/cover-collision-validation-20260922.md for timing/provenance details.

## 0.1.37 - 2026-09-22

- Make adaptive arc repairs screen complete anchor groups before IK, retain
  progress past other known failure intervals, and pursue the best improving
  sequence while keeping alternatives. Prepared geometry updates both position
  and heading; existing offset spacing, refinement and movement limits remain.
- Complete exact collision checks in failed candidate layers on curved paths
  before rebuilding the graph. Preserve distinct joint states and exact path
  costs; straight paths retain the previous selected-node policy.
- Avoid legacy heading construction on pure arcs, including arcs beyond 180
  degrees. Spatial fit samples retain original points without chord shrinkage.
  Expose readable section/transition records in GH and effective exclusions in
  numeric results. Phase timings include early repair screening.
- Add reproducible historical mode comparisons and experimental rectangle/box
  base-screening harnesses. Both screens were slower across three matched trials
  with identical base decisions; the detailed checker remains the default.
  Detailed final 3D configuration collision validation is unchanged.
- Validate all three historical captures with identical auto/legacy poses,
  configurations and costs at matched historical settings. Today's 1,481-target
  curved capture validates a complete connected trajectory from the current
  1.0/1.3 m preferences with adaptive repairs. GPS exclusions match across runs.
  The full suite passes: 247 tests. See docs/curvature-validation-20260922.md
  for evidence and timing limitations.
- No new runtime dependencies. Versions, GH reload checks, recording metadata
  documentation and regression coverage are synchronized.

## 0.1.36 - 2026-09-22

- Add automatic straight/arc sections, original-TCP circle fitting with shared
  centers and separate pass radii, radial displacement and signed arc-distance
  offsets. Adaptive repairs regenerate arc position and orientation together.
- Add geometry_mode/options and section diagnostics to numeric planning,
  Grasshopper and offline replay. Legacy mode retains previous geometry.
- Mobile collision worlds exclude the unmounted GPS antenna by default;
  explicit excluded_collision_links and exclude_gps settings retain frames and
  leave input robot files unchanged. Other collision geometry stays detailed.
- Validation: 232 tests pass. Geometry-only replay gives identical auto/legacy
  base poses on all three prior captures and identifies all 19 current passes
  as arcs. Full current-capture feasibility and shortcut benchmarks are pending.

## 0.1.35 - 2026-09-22

- User confirmation: the Grasshopper mobile base example now works well for
  the last segment.
- Increase the preferred normal base offset from 0.9 m to 1.0 m in the shared
  mobile planner and GH component. The tangential default remains 1.3 m;
  explicit inputs override both defaults. Prior capture results used 0.9 m.
- Validation: 43 mobile and GH regression tests pass; full captures were not
  rerun for this default adjustment.

## 0.1.34 - 2026-09-22

- Increase the preferred tangential base offset from 1.2 m to 1.3 m in the
  shared mobile planner and GH component. Explicit input values still override
  the default; the normal offset remains 0.9 m. Previous captured-case timing
  and feasibility results describe the former 1.2 m setting.
- Validation: 43 mobile and GH regression tests pass; full captures were not
  rerun for this default adjustment.

## 0.1.33 - 2026-09-22

- Add GH Number Slider input `base_yaw_degrees` (default 0): rotate base
  orientations counterclockwise about upright Z at their own origins before
  calibrated IK, placement and collision validation. Adaptive repairs retain
  centerline-relative offset directions and validate the requested yaw.
- Include the requested angle in results and readable effective settings.
  Validate nonzero positive/negative yaw through collision repair and GH input
  forwarding. Existing zero-angle behavior is preserved.
- Validation: 43 mobile workflow, adaptive repair and GH entry-point tests pass.

## 0.1.32 - 2026-09-21

- Request Windows HighQoS only on the mobile planning thread for the duration of
  computation, restoring its previous policy afterward. Apply the same scope to
  offline replay setup/planning. Nested scopes are shared; unsupported APIs fall
  back without changing numerical results. No priority, CPU affinity, system
  power-plan or runtime-limit setting is changed.
- Record the requested and restored policy and show it in GH diagnostics. The
  policy operates independently of research logging. Numerical planning,
  calibration, candidate coverage and collision settings are unchanged.
- A scheduling probe found 2388 identical IK configurations took 1.7-2.0 s with
  explicit performance requests versus 3-18 s with default scheduling. Two final
  fresh-process runs of capture 20260921_121845_a3a4e122 completed in 143.28 s and
  187.19 s, including setup, normal research logging and result serialization.
  Both validate all 1319 targets with identical optimal joint cost
  60.77748430022862 and restore the previous thread policy. No saved trajectory
  is reused. Preferred offsets suffice for all three captured regressions;
  adaptive recovery is additionally covered by controlled regression tests.
- Validation: 216 tests pass, including nested policy scopes, interruption,
  restoration failures and unsupported-API fallback. Run metadata includes the
  process ID and native thread ID to correlate scheduling observations.

## 0.1.31 - 2026-09-21

- Use the shared exact layered graph for mobile configuration collision checking:
  solve, reject colliding selected nodes, and solve again until the shortest path
  is fully checked. Preserve every sampled IK candidate and bounded joint/speed
  constraint. Mobile planning performs configuration checks only, as requested;
  stationary and other planners retain their collision policies.
- Select graph-first from a full captured-case comparison: 9.52 s versus 100.19 s
  for collision-first after common IK generation, with the same optimal arm cost
  60.77748430022862. Collision checks fall from 108394 to 1319 on this case.
- Add adapt_offsets=True to the shared GH/offline mobile workflow. Repair failed
  regions with overlapping quintic blends, 0.10 m proposals and refinement to
  0.025 m, retaining alternatives until all original targets and joins validate.
  Scalar offsets remain preferences; False preserves fixed-offset planning.
  Exact pose/configuration caches are scoped to one unchanged planning scene.
- Keep configurable 16 TCP-Z rotations, calibrated tool-aware IK, placement and
  1.75 m arm-origin XY reach checks. No automatic joint-turn expansion, runtime
  cutoff or extra production FK audit is introduced.
- Batch calibrated FK/Jacobian evaluation across analytic IK branches, retaining
  each branch's least-squares refinement, iteration limit and FK acceptance
  tolerance. Targeted regressions compare scalar/batched branches, Jacobians,
  singular cases, tilted axes and convergence rejection.
- Extend the existing research recorder to geometry and calibrated IK boundaries,
  aggregate mobile metrics, record collision-order evidence and repairs, and mark
  explicit interruptions. GH exposes readable diagnostics and the run directory.
  Offline replay includes normal logging and end-to-end timing without saved-path
  reuse. Document the public-tool recording audit and Slab Net Zero comparison.
- Validation: 210 tests pass, including eager/lazy cost equivalence, collisions,
  rotations, movement/speed constraints, repair joins, caching and recording.
  Before batched IK, the latest capture validated all 1319 targets in 273.90 s,
  but its fresh-process confirmation took 440.67 s and missed the runtime goal.
  With batched IK, two full runs again validated all targets at the identical
  optimal cost, but took 1141.65 s and 2600.27 s under variable Windows scheduling.
  Previous captures also validate all 1607 and 1561 targets. Their recorded
  overall times include a documented CPU-policy change during graph validation.
  Scoped scheduling and final timing acceptance are addressed in 0.1.32.
- Add this version history and require changelog updates with future version bumps.

## 0.1.30 - 2026-09-17

- Remove automatic +/-360-degree joint candidate expansion from the shared
  evaluator used by mobile, stationary, rolling and prescribed-base planners.
  Filter only the original IK representatives against joint limits; bounded
  joints still use actual angle deltas for transition checks.
- Remove the mobile GH 45-minute deadline and the offline replay timeout/CLI
  option. No elapsed-time limit is imposed on planning. Git metadata collection
  retains its unrelated subprocess timeout.
- Refresh cached shared planning code in the GH entry points. Keep 16 TCP-Z
  rotations, collision coverage and all configured motion constraints.
- Validation: 190 tests pass after updating expansion-specific expectations.
  Both complete captures validate with 16 TCP rotations and sampled swept joins:
  1607 targets in 1357.83 s and 1561 targets in 1294.45 s (parallel offline runs).
  Independent FK checks verify all returned TCP orientations and configurations;
  maximum TCP position errors are below 2.7e-8 m. Full results remain outside Git.

## 0.1.29 - 2026-09-17

- Refresh mobile component IK, adapter, graph and collision dependencies in
  dependency order on recompute, so Rhino cannot retain older solver bindings.
- Restore the capture-compatible metres default for input geometry; millimetre
  inputs require an explicit 0.001 scale instead of document-unit inference.
- Emit readable diagnostic strings for GH panels, including effective units,
  rotations, tool/mount calibration, solver, code path and first target/base.
- Validation: 190 tests pass. Component regression covers stale IK imports and metre inputs in
  a simulated millimetre document. Offline execution of the component with each
  full capture's base proposal validates the first three targets and swept joins;
  these bounded probes do not establish full-trajectory validity or reproduce
  the user's live target-0 failure.

## 0.1.28 - 2026-09-17

- Enable 16 equally spaced TCP local-Z rotations by default in mobile validation.
  Keep target positions/extrusion axes and the base proposal unchanged; evaluate
  real calibrated tool-aware IK and collisions across all sampled orientations.
- Connect the combined candidate layers under the existing joint/edge constraints,
  and return the selected TCP planes and rotation angles with a complete path.
  Track angles correctly through bounded joint-revolution expansion.
- Expose rotation_steps in the component and bounded offline replay harness;
  rotation_steps=1 restores fixed-orientation behavior.
- Reuse equivalent full-turn swept checks per transition, retaining winding
  deltas and sample counts in cache keys; add offline progress checkpoints.
- Validation: 190 tests pass, including rotation selection, bounded revolutions,
  and cached versus uncached swept-path results. Both captured recordings have
  collision-free IK states at every target with 16 rotations (1607 and 1561).
  Both 600-second replays timed out during the joint graph search; complete
  connected trajectories remain unvalidated, not proven infeasible.

## 0.1.27 - 2026-09-17

- Replace the mobile Grasshopper geometry experiment with single-proposal base
  generation followed by calibrated IK, placement/body checks, full configured
  robot/tool/environment collisions, and connected joint/sampled swept checks.
  Keep failed base proposals inspectable; emit arm configurations only for a
  complete validated trajectory. Preserve original TCP orientation and ordering.
- Add base/yaw step and optional timed speed checks, detailed rejection states,
  and a component-only cooperative 45-minute limit (not native-call termination).
  Shared planning APIs have no timeout. Stationary planning remains unchanged.
- Add an integrity-verified offline capture harness with killable subprocesses.
- Validation: full suite 187 passed. Both real captures were checked in full;
  fixed-offset proposals fail (first no-IK indices 584 and 104 respectively),
  with separate configuration-collision diagnostics. No fabrication-ready path
  is claimed, and graph connectivity is untested when target layers fail.

## 0.1.26 - 2026-09-17

- Expose one Rhino base plane per original TCP in the existing component when
  bounded smoothing is enabled. X faces the wall, Y follows the centerline,
  Z is global up; offsets originate on each smooth pass. Add base_path,
  centerline and target_indices outputs. Preserve original target order.
- Convert physical offsets using Rhino document units or units_to_metres.
  create_base_planes=false retains the previous curve-only experiment.
  Outputs remain unvalidated geometric proposals, with no arm planning.
- Validation: 43 tests passed, including component mapping and millimetre offsets.
  Export explicit target indices and show sampled 3D base-plane axes.

## 0.1.25 - 2026-09-17

- Correct the centerline-guided offset experiment to displace every original
  smooth_xy pass point rather than its centerline projection. Use the centerline
  only to derive consistent headings; retain pass spacing and target order.
- Export updated top/3D plots and per-target base planes for both captures.
- Validation: 34 tests passed, including preservation of separated return passes.
  All 3,168 captured frames checked against offsets from their original pass
  points and global +Z; both comparison images inspected.

## 0.1.24 - 2026-09-17

- Derive upright base frames from spatial centerline tangents/normals, using
  target normals only to select one consistent wall side. Offset -0.9 m along
  base X and +1.2 m along base Y without flipping headings on return passes.
- Add per-target frame exports and top/3D comparison plots. Geometry only;
  robot reach, motion limits and collisions are not validated.
- Validation: 33 centerline/offset/component tests passed; both real recordings
  checked per target for exact offsets, right-handed frames and global +Z.
  Top/3D figures inspected.

## 0.1.23 - 2026-09-17

- Add spatial centerline extraction from repeated smooth XY passes: identify the
  dominant PCA direction, retain the full longitudinal extent, and average only
  the perpendicular coordinate using spatial bins and local-linear smoothing.
- Preserve a per-input mapping with unchanged longitudinal coordinates and
  original traversal order. Export comparison plots for both recordings.
- Geometry only: no new offsets, robot planning or stationary algorithm changes.
- Validation: 31 centerline/component tests passed. Both captured smooth lines
  retain their longitudinal extrema and every mapped longitudinal coordinate
  within 1e-12 m; rendered comparison plots inspected.

## 0.1.22 - 2026-09-16

- Add a geometry-only diagonal-offset experiment: one upright ground frame per
  saved smooth_xy point, global +Z up, local X aligned with projected target Z,
  then offsets of -0.9 m X and +1.2 m Y. Reject undefined horizontal normals.
- Add a two-recording overlay graphic and per-target frame exports with capture
  and input-line fingerprints. No collision/reach claims or stationary changes.
- Validation: 47 smoothing/component tests passed; all 3,168 exported frames
  checked for global +Z, right-handed orthonormal axes, ground-plane origins
  and 1.5 m diagonal displacement.

## 0.1.21 - 2026-09-16

- Add geometry-only whole-path XY smoothing with a per-target deviation bound,
  squared-step/curvature objective, and explicit convergence diagnostics. Large
  horizontal sweeps survive; height-only motion needs no XY movement.
- Add reproducible comparison plots for captured recordings, including ordered
  horizontal/height traces, reversal diagnostics and measured target deviations.
- The existing Grasshopper line component accepts optional max_xy_deviation in
  model units. Stationary planning is unchanged; no mobile robot planning added.
- Validation: full suite 176 passed, then 11 averaging/component tests passed
  after adding bounded-component coverage. Both real captures were processed
  and plotted; deviation bounds hold. First-capture 0.25/0.50 m runs reach the
  iteration cap and are explicitly reported as unconverged.

## 0.1.20 - 2026-09-16

- Remove mobile-base search strategies, transition/runtime helpers, mobile-only
  replay tools and benchmarks, and the mobile_options branch of robot planning.
  Remove the mobile planner API and its per-target base candidate generator.
  Keep stationary algorithms, component, calibrated kinematics and collision
  support intact. Arm planning at caller-prescribed bases remains available.
- Replace the historical grasshopper_mobile_base.py filepath with an XY-only
  moving-average experiment. It ignores height/orientation, compares window
  sizes by length divided by their own endpoint distance (raw length remains
  available), and returns only averaged geometry. Former base/arm
  outputs are cleared; no robot or collision world is constructed.
- Add a NumPy window sweep and an offline Matplotlib comparison graphic with
  the minimum ratio and shortest raw length highlighted separately. Save line
  coordinates, length and ratio scores, and provenance outside Git. Length minima are not asserted to prove frequency.
- Captured 1,607-position sweep over every integer window 10–200 takes about
  0.06 s: minimum L/D is 1.069312 at 120 points (2.991373 m). Raw length
  instead selects 200 points (2.987453 m), with a local minimum at 122 points
  (2.9906 m). Windows are clipped/renormalized at the endpoints.
- Validation: 168 tests passed, including stationary workflows and XY averaging.
  Stationary component/region/adapter/calibration files remain byte-identical;
  retained stationary base-planning functions are AST-identical.

## 0.1.19 - 2026-09-16

- Bound complete arm-trajectory proposal checks inside the adaptive strategy.
  A failed tested transition triggers base-control refinement instead of an
  exhaustive minimum-cost arm search for that rejected proposal. Accepted paths
  still validate every original target and transition. Failure diagnostics
  distinguish the bounded search from proof of graph disconnection.
- Retain exhaustive fallback and exact results by default in shared planners.
  No shared API timeout or collision/step-limit relaxation is introduced.
- Validation: 240 tests passed. A fresh captured-case replay returned all 1,607
  base/arm states in 403.171 seconds; an independent audit checked every target
  and 1,606 transitions with zero rejections. Optional smoothing was rejected
  for that particular result, so its validated original path was retained.

## 0.1.18 - 2026-09-16

- Make the dedicated mobile Grasshopper component use an adaptive wall/lateral
  offset and yaw roadmap. Insert failing original targets, retain useful distant
  regions and solve arm continuity over the complete trajectory. Existing shared
  API strategy defaults and exact discrete planning remain unchanged.
- Smooth successful base paths only when calibrated branch continuation passes
  all original placement, joint, configuration and swept-transition constraints.
  Add an independent saved-result auditor using captured URDF PyBullet FK at
  every original target; retain source hashes and reproducible replay metadata.
- Solve the exported 1,607-target fabrication case with a connected arm path.
  Independent full-target/full-transition audit passes, including captured tool,
  body and environment collision checks. The capture supplies no speed timing;
  this validates a geometric trajectory, not a timed controller program.
- Refresh cached Grasshopper dependencies and version gates automatically; no
  repasting of the filepath-loaded component is required. No new dependency or
  shared planning API timeout is introduced.
- Validation: 237 tests passed, including failed-target refinement, disconnected
  joins, full smoothing revalidation and retained start-speed constraints.

## 0.1.17 - 2026-09-16

- Refine analytic UR branches against recognized URDF controller/tool0 chains,
  including factory joint origins/rotations, fixed lift, mounting and TCP.
  Accept refined states only after calibrated FK converges; retain subsequent
  joint-limit and collision filtering. No new dependency is required.
- Reload calibrated kinematics with the Grasshopper dependency chain. Replace
  stationary entrypoint's positional module references with named lookups.
- Captured robot FK audit at targets 0, 57 and 1000 improves from millimetres
  to under 0.0003 mm (PyBullet FK precision); this is sampled calibration
  validation, not yet a claim of a complete fabrication path.
- Validation: 232 tests passed, including calibrated Jacobian/FK, independent
  mounting/TCP transforms and the actual Grasshopper entrypoints.

## 0.1.16 - 2026-09-16

- Screen common base step bounds once for fixed smooth proposals; retain exact
  per-edge checks for variable bases and speed constraints. Reject requests to
  count paths when lazy validation leaves alternative edges untested.
- Add captured-source comparisons, prefix-only runs, independent sampled URDF
  FK audits, loaded-code fingerprints and capture integrity tests to replay.
- Synchronize Grasshopper entrypoint version checks with the package version.
- Validation: 230 tests passed, including exact lazy/eager path comparisons,
  collision/continuity checks, capture integrity and component entrypoints.
- Captured-source replay confirms that the original 100-point/+1 m lateral/
  1 m wall proposal also blocks at transition 1060 to 1061: all 512 tested
  predecessor pairs exceed the 2.5 rad joint-step bound. This is a proposal
  failure, not proof of global infeasibility. No fabrication-ready path claimed.

## 0.1.15 - 2026-09-16

- Defer expensive mobile swept-edge checks to complete candidate trajectories,
  with exact rejection and bounded fallback to ordinary graph search. Preserve
  joint bounds, costs and predecessor tie-breaking; do not relax collisions.
- Add a manifest-verified offline capture replay in a killable subprocess,
  explicit proposal validation, FK audits and source fingerprints. Runtime
  bounds remain in the replay harness, not shared planning APIs.
- Extend explicitly increased smooth repair budgets to smaller displacements
  and shorter smooth windows, and stop repeating exhausted repairs.
- Validation: 227 tests passed. Captured 1,607-target case reproduced analytic
  IK rejection at target 1062. Captured calibrated URDF FK differs from nominal
  analytic IK by millimetres in sampled checks; this case is NOT yet solved or
  fabrication-ready. Geometry and experimental results remain outside Git.

## 0.1.14 - 2026-09-16

- Revert 0.1.13's shared planning deadline and all injected planner/collision
  checkpoints. Shared APIs and grasshopper.py again have no automatic deadline.
- Scope the 45-minute wait limit to grasshopper_mobile_base.py. Run its planning
  job in a worker; return empty outputs and timeout status without waiting for
  worker cleanup. Request cancellation at toolbox function boundaries and block
  duplicate runs of that component until the prior worker finishes.
- Never forcibly terminate native threads. An in-progress native operation must
  return for cancellation; a native call holding the GIL can delay Python code.
- Validation: 214 tests passed, covering worker timeout/cancellation, cleanup, duplicate-run protection,
  mobile-only scope and component entrypoints tested. Synthetic 1,607-target
  workload retained identical IK/collision/transition counts (0.51 s direct,
  0.94 s through the worker). No live 45-minute Rhino run or dependency changes.

## 0.1.13 - 2026-09-16 (reverted in 0.1.14)

- Introduced a shared 45-minute planning deadline. Reverted at the user's request
  because timeout handling belongs specifically to grasshopper_mobile_base.py.

## 0.1.12 - 2026-09-16

- Add overlapping partial-path construction and connection after whole smooth
  proposals fail. Default sections contain 100 targets with 50-target overlaps;
  test six proposals and retain two smooth alternatives. Blend position and
  shortest-arc yaw gradually, then solve each joined prefix as one arm/base path.
- Preserve placement, IK, joint-step, timing and enabled collision constraints
  across joins. Reuse IK and exact endpoint transition checks within a fixed
  search scene. Never output an unvalidated concatenation or incomplete path.
- Expose connect_sections, section_size, section_proposals and section_beam_width;
  report section ranges, join rejection details and validated prefix coverage.
  No dependency changes; both Grasshopper script reload markers updated.
- Validation: full suite 210 passed; 22 focused tests passed after transition
  caching. A synthetic 1,607-target, 32-section workload completed in 24.38 s;
  identical endpoint caching reduced transition callbacks from 59,973 to 3,784.
  This uses synthetic callbacks; the full user robot scene was not rerun.

## 0.1.11 - 2026-09-16

- Spread smooth-path attempts across windows and both sideways signs; try wall
  clearance near one metre before other distances within each family.
- Add up to eight cosine-tapered local X/Y repair proposals around the best
  attempt's blocking target, reusing IK results and retaining reach, joint and
  collision checks. Repairs can be disabled with smooth_repair_attempts=0.
- Report the best failed attempt and aggregate checked-target coverage rather
  than the last attempt. Label placement/body rejection before IK correctly.
- Add regressions for local repair, window/sign coverage, and best-attempt
  diagnostics: 202 tests passed. Synthetic 1,607-target benchmark: 1.05 seconds,
  preserving 1,607 IK/collision and 1,606 transition calls. No dependency changes.
  Full user robot scene not rerun; synthetic timing is not a robot estimate.

## 0.1.10 - 2026-09-16

- Make smooth whole-path proposals the dedicated mobile Grasshopper component
  default. Footprint +X faces the wall; the nominal one-metre offset is along
  footprint +/-Y. Search wall distance along X independently, checking calibrated
  arm-origin XY reach and negative-TCP-Z placement.
- Compare moving-average windows of 10, 25, 50, 100 and 200 targets, ranking
  position/heading smoothness and tracking rather than minimizing base travel.
  Curved walls use local normal offsets. Validate every original 3D target and
  all enabled transition/collision constraints before returning a path.
- Bound proposal attempts, stop failed IK validation early, reuse candidate
  results, and defer plane construction until validation. No automatic dense
  fallback; retain the previous planner through strategy=discrete. Record
  attempted smoothing/offset settings and validation failures. Correct the
  misleading unchecked-target keyframe message. No dependency changes.
- Validation: 199 tests passed. Synthetic 1,607-target benchmark completed in
  1.59 seconds with 1,607 IK/collision calls and 1,606 transition calls; callbacks
  were synthetic, not a real-robot runtime estimate. Full user scene not rerun.

## 0.1.9 - 2026-09-16

- Retain connected base/arm states in bounded mobile fallback, try continued
  footprints first, and expand locally at blocked transitions while reusing IK.
  Preserve placement, joint, speed and sampled collision constraints.
- Report the blocked zero-based target pair, first-rejection counts, measured
  values/limits, joint indices and available collision details in Grasshopper
  diagnostics and recording events, including graph-prefilter rejections.
- Stop blocked fallback before evaluating later targets; distinguish untested
  targets from infeasible targets. Uncapped supplied-domain search stays exact.
- Refresh both Grasshopper script entrypoints and dependency reload markers.
  No dependency changes. Validation: 188 tests passed; after predecessor-array
  caching, 28 mobile regression tests passed. Synthetic dense/sparse comparison
  preserved all 201 outputs and cost 1.0 (2,412 versus 223 IK calls).

## 0.1.8 - 2026-09-15

- Speed up minimum-cost graph search by checking transitions in stable cost
  order and stopping at the first valid predecessor. Preserve selected paths,
  costs, tie ordering, and full collision sampling for selected transitions.
  Exact path-count mode still evaluates all admissible transitions.
- Batch nested recording writes, retaining every step, event and metric.
  Root call boundaries and run close force commits; interrupted processes may
  lose the current in-flight batch. Version and loaded-code fingerprints remain.
- Reuse region standoff and mounting calculations without changing candidate
  planes or ordering. Avoid repeated target matrices and base-state prefixes.
- Filter distant static-body/obstacle pairs with conservative current bounding
  boxes before Bullet distance queries, preserving collision results/clearance.
- Refresh cached Grasshopper planner and recording modules on recompute.
  No new inputs or dependencies.
- Validation: 176 tests passed, including two new bounding-box equivalence cases;
  174 tests were also exercised with automatic recording enabled;
  reproducible benchmarks against `4ef10ee` verify
  exact graph/region parity and equal recording row counts. Synthetic graph and
  PyBullet transition workloads improved about 24x and 80x; recording about 7.7x,
  base-body collision checks about 9.8x, region generation about 1.3x.
  These are component measurements, not a replay
  of the corrected full robot scene. See `benchmarks/mobile_performance.json`.

## 0.1.7 - 2026-09-15

Source commit: `4ef10ee`.

- Stop sparse mobile search when a keyframe exhausts its sampled candidate
  region; skip dense fallback that cannot repair the same empty layer.
- Generate later regions lazily; distinguish confirmed failures from untested
  targets using `unchecked_points` and null candidate counts.
- Normalize signed zero in candidate cache keys without merging nearby poses.
- Validation at that checkpoint: 152 tests passed.

## 0.1.6 - 2026-09-15

Source commit: `6350b45`.

- Simplify sparse mobile paths in world XY, with a default tolerance of 0.05 m,
  so small ripples do not force extra base-motion keyframes.
- Retain projected wall-normal heading changes; optionally retain height changes.
  Validate every original 3D target after interpolation.
- Use projected XY progress for interpolation. Index and distance gap limits now
  default to disabled; `sampling: "legacy"` restores the earlier selector.
- Expose `xy_tolerance` in the dedicated Grasshopper component in model units.

## 0.1.5 - 2026-09-15

Source commit: `6569a0c`.

- Generate mobile candidates using stationary placement constraints: the arm-base
  origin must be behind projected target +Z and within 1.75 m in XY, accounting
  for the mounting offset. Apply the same constraints to interpolated targets.
- Order region candidates by standoff and check base-body collisions before IK.
  Sparse search defaults to four feasible bases per searched target; this cap
  can miss connected paths. Dense mode searches the complete sampled region.
- Reuse candidate checks during fallback and fix planning/recording diagnostics.
- Replace dedicated-component offset/seed-distance search controls with region
  sampling controls (`grid_spacing`, `yaw_steps`, and `mobile_options`).

## 0.1.4 - 2026-09-15

Source commit: `fa3a823`.

- Add `examples/grasshopper_mobile_base.py`, a dedicated mobile base-motion
  component with automatic footprint seeds and optional supplied seed paths.
- Return a base plane, joint plan, and named configuration for every original
  target on success; expose sparse search and start-base settings.

## 0.1.3 - 2026-09-15

Source commit: `55787fd`.

- Add sparse mobile base planning and Grasshopper `mobile_options` integration.
- Search retained keyframes, interpolate base motion, and solve arm IK and check
  constraints at every original target; fall back to dense search on failure.
- Add sparse-planning tests and a mobile planning benchmark.

## 0.1.2 - 2026-09-15

Source commit: `edc5d0d`.

- Add collision-cache grouping for equivalent bounded joint turns; advance the
  collision API version to 10 and refine graph/planning implementation.
- Record loaded module versions, file hashes, and loaded-code fingerprints to
  distinguish cached Rhino imports from files on disk.
- Advance recording version to 3, save recording errors, and improve repository
  metadata collection. Add recording tests and revision benchmark results.

## 0.1.1 - 2026-09-15

Source commit: `1040b33`.

- Add projected stationary placement regions, bounded candidate validation,
  detailed failure diagnostics, and dedicated Grasshopper planning examples.
- Improve collision filtering and reuse checks for full-turn-equivalent joint
  configurations; optimize graph search while retaining bounded configurations.
- Add automatic research recording with SQLite runs, compressed artifacts,
  metrics export through `toolbox-metrics`, and a recording guide.
- Expand adapter, collision, planner, Grasshopper, and recording regression
  coverage; add validation and benchmark artifacts.
- Establish package versioning and checkpoint-commit workflow instructions.

## 0.1.0 - 2026-09-10

Source commit: `8b3d6fa`.

- Extract the reusable motion planning package from the existing project code.
- Provide numeric IK, optional PyBullet collision checking, layered motion
  planning, stationary/mobile base planning, rolling replanning, and the
  `motion-plan` command, with optional COMPAS integration.
