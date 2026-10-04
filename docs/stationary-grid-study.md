# Stationary grid study

This reproducible grid experiment was developed on `study/stationary-grid-search`
and is now available in the main checkout. It remains an offline validation tool;
running it does not change Grasshopper settings or captured cases. The resulting
[adaptive stationary solver](adaptive-stationary-study.md) is available separately
through `search_strategy="adaptive"`.

The input case is verified against every manifest hash before loading. For the
20261003_220724_0d413599 case, the captured rotation setting is 1; this experiment
explicitly uses 24 to match the latest stationary component. It retains the
captured calibrated robot, TCP, joint ranges, fixed joints, environment, tools
and allowed collision pairs. Detailed base geometry and GPS checks are enabled.
Configuration collision checks are performed. Transition collision checks are
not included; the capture's check_edges flag is recorded as an unapplied setting,
consistent with the stationary planner's existing scope.

## Procedure

1. Run the existing greatest-common-standoff stationary heuristic as a baseline.
2. Divide the initial region into eight horizontal bands; in each band choose a
   reproducible jittered Y, intersect the convex region at that Y, and divide its
   X interval into eight strata. One jittered X per stratum gives exactly 64
   inside-region positions. This follows the polygon rather than discarding
   points from an enclosing rectangular grid. Randomize their evaluation order.
3. At every position test four footprint headings, 90 degrees apart, oriented
   relative to the target centroid and calibrated mount. The arm-origin position
   stays fixed while the footprint changes with heading/mount translation.
4. Screen base collisions first, then any supplied starting pose. For every
   surviving base, evaluate all targets and all sampled IK options exhaustively.
   Continue after unreachable targets so totals and minima are exact. Preserve
   the existing solver's unique joint representatives; do not inflate counts by
   adding full-turn equivalents. Invalid base/start poses have zero admissible
   configurations and are explicitly marked as screened, without IK evaluation.
5. Rank coverage first. Within equal coverage, balance normalized minimum and
   mean counts: `(minimum / max_minimum)^w * (mean / max_mean)^(1-w)`, with w=0.5
   by default, 0.25 for total preference and 0.75 for minimum preference.
   Since every candidate has the same targets, mean and total rankings agree.
   If minimum is zero, coverage then total resolves the otherwise zero score.
   Zero-count targets cannot be hidden by a high overall total.
6. Keep the score winner, total-count winner, minimum-count winner, and both
   spatial extremes along the target cloud's principal XY axis among the
   best-coverage candidates. Deduplicate positions. Refine a 3x3 neighborhood
   around each, initially at one coarse bounding-box cell spacing and then at
   half that spacing. Try all four headings again. Refinement is deliberately
   not clipped to the initial 1.75 m region or its target-side boundaries.
   Actual IK, limits and collision geometry determine admissibility.
7. Check path connectivity and exact shortest joint travel on the saved,
   fully collision-checked layers of the top three score candidates, the count/
   spatial extremes and the baseline. Preserve the captured joint-step limits.
   Report disconnected paths separately from all-target reachability.

This preserves promising side positions without claiming that a high solution
count guarantees path connectivity. It is a finite local study, not a global
optimization certificate. The random seed and exact points are saved. Four
headings and two refinement levels remain a limited search of pose space.

## Running and resuming

From the main checkout, using its existing Python environment:

```powershell
$env:TOOLBOX_RECORDING='0'
.\.venv\Scripts\python.exe validation/study_stationary_grid.py `
  C:\Users\david\Documents\motion_planning_cases\20261003_220724_0d413599 `
  --output ..\research_runs\stationary_grid_20261003_220724 --workers 6
```

Repeat the identical command to resume. Completed candidate JSON files are the
checkpoints; compressed configuration layers are written before each checkpoint.
A source/settings mismatch rejects resume. Each worker owns its collision world
and scopes/restores the Windows CPU policy. No shared collision cache crosses
candidate or worker boundaries. Worker progress and overall progress are saved.

Create a self-contained interactive report, including from partial checkpoints:

```powershell
.\.venv\Scripts\python.exe -m validation.report_stationary_grid `
  ..\research_runs\stationary_grid_20261003_220724
```

Results and captured geometry stay outside Git. The result settings include the
package version, input and study-source hashes; loaded-code fingerprints are
saved separately. Candidate elapsed/CPU time, exact counts, collision calls,
path checks and the baseline are retained for comparison.
