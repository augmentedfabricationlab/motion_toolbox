# Changelog

Notable changes by package version, newest first. Historical entries were
reconstructed from Git commits and `pyproject.toml`; dates are commit dates,
not evidence of a published release. Commit references identify the source.

## Unreleased

### Documentation

- Add this version history and require changelog updates with future version bumps.

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
