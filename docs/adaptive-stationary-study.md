# Adaptive stationary search and validation

This implementation was developed on `study/stationary-grid-search` and is now
available in the main checkout. The default remains `"heuristic"`. Select
`search_strategy="adaptive"` in the updated stationary Grasshopper example or
`plan_stationary_base`; switch back to `"heuristic"` to restore the original search.
Point `toolbox_src` at the main checkout's `src` directory and restart Rhino when
switching from the study checkout. No captured robot, tool or environment files
are committed.

## What the exhaustive study showed

The original 1,102-target capture used a tool extending about 0.727 m behind its
TCP. The 416-pose study found 21 fully reachable bases. Auditing the cached IK
layers of all 21 found 12 with a connected path under the existing 2.5 rad joint
step limit. All 12 had headings between 311.27 and 323.81 degrees in this scene;
the nine disconnected placements had headings between 36.56 and 44.13 degrees.
This supports searching side-on headings, rather than ranking only common standoff.
It is a scene-specific prior, not a geometric exclusion rule.

Seven connected placements exceeded the initial 1.75 m XY reach envelope. Connected
placements had minimum collision-free option counts from 2 to 47. A large minimum
count alone did not predict connectivity: the standoff winner had minimum 48 and
105,270 total options but no connected path. Successful candidates must satisfy
both per-target reachability and adjacent-layer continuity.

## Search and exact validation

1. Sample the initial arm-origin region and a larger reach envelope, including
   standoff seeds. Test headings around the entire circle.
2. Probe spatial/orientation extremes and farthest-point coverage. Use a subset
   of the requested TCP rotations. Rank coverage first, then the geometric mean
   of estimated minimum and mean collision-free options, with configurable heading
   and footprint-clearance preferences. Probe estimates are **not exact counts**.
3. Preserve different heading families and refine their XY positions and yaw.
   Refinement is not clipped to the starting region.
4. Validate finalists using **every target and every requested TCP rotation**.
   A denser 64-target screen first checks likely finalists; a screen rejection
   requires completing the failed target's requested rotations. Low targets are
   visited early because of their proximity to chassis/lift geometry.
   Failed targets and disconnected transitions become early exact tests for
   subsequent bases. IK and collision results are cached only for this call.
5. Use the existing lazy graph validator. After two optimistic passes, complete
   remaining collision checks before the next solve to bound graph rebuilds;
   this preserves exact path costs and ties. The fast default stops at the first
   connected finalist. Increase `connected_finalists` to compare more connected
   bases and return their lowest joint-path cost. No global base optimum is claimed.

The collision model retains detailed base, GPS, tool and environment coverage.
Joint limits, starting poses, periodic joints and per-transition speed limits
remain active. Validation checks configurations, not swept transitions. If only
disconnected reachable placements are found, one is returned with an empty joint
path. With `build_path=False`, validation proves per-target reachability only.

`solution_counts=[]`, `ik_option_count="not counted"` and `counts_complete=False`
make omitted counts explicit. To restore exact counts and the original search,
set `fast_validation=False`; `count_paths=True` also forces that workflow.
`planned_tcp` retains selected target order and TCP rotations for a complete path.

The footprint preference uses the XY bounds of all configured static collision
links. It measures signed distance behind projected target planes, conservatively
including empty AABB space and ignoring height. It is neither a collision test
nor a required 1 m separation. The preference does not resize the attached tool.

## Options and diagnostics

`search_options` is a dictionary or JSON object. All distances below are metres,
independent of Grasshopper model units.

| Option | Default | Meaning |
| --- | ---: | --- |
| `grid_size` | 5 | Interior grid rows and columns |
| `yaw_steps` | 8 | Initial headings per sampled arm origin |
| `probe_count` | 16 | Representative targets |
| `probe_rotations` | 8 | Subset of requested TCP rotations |
| `validation_probe_count` | 64 | Denser reachability screen before full validation |
| `beam_width` | 4 | Distinct refinement parents |
| `refinement_steps` | `[0.16,0.08]` | Local XY step sizes; yaw is refined too |
| `max_full_checks` | 3 | Maximum finalists reaching every target (path may disconnect) |
| `connected_finalists` | 1 | Stop after this many connected finalists; compare their path costs |
| `initial_reach` | 1.75 | Initial common XY reach envelope |
| `exploration_reach` | 2.1 | Larger initial search envelope |
| `tool_clearance` | 1.0 | Soft rear footprint preference; 0 disables it |
| `clearance_weight` | 0.15 | Strength of the footprint preference |
| `heading_bias` | 1.0 | Side-on heading bonus, up to 2x at the default |

Supplied `candidate_planes` form an exclusive set: no grid or refinement is added
and the 1.75 m envelope does not reject supplied adaptive candidates. Their IK and
collisions are still fully validated. The existing top-level `yaw_steps`,
`grid_spacing` and `max_validation_attempts` configure the heuristic workflow;
use `search_options` for adaptive controls.

Early reachability failures do not consume the finalist budget. The finite
proposal set bounds the search; every discovered failed target is checked first
on subsequent proposals. `stats.validation_attempts` counts validations started,
while `stats.full_checks` counts finalists that reached every target.

Add `search_summary` and `search_diagnostics` outputs to Grasshopper for JSON
results. They report tested poses, probe estimates, learned failures, exact finalist
results, call counts and timings. Both clear on failed recomputes. Scoped Windows
CPU performance policy is restored after success, errors and cancellation.

## Reproduction and tool update

Measured offline on Windows on 2026-10-04, with 1,102 targets and 24 TCP rotations,
recording disabled, and no Rhino/UI overhead included.
Single captured-case runs are workstation observations, not a universal speedup.
Planning wall time includes setup and unlisted candidate/diagnostic overhead;
IK, collision and graph columns measure their respective computation buckets.

| Original tool, same capture | Setup (s) | IK (s) | Collision (s) | Graph (s) | Planning wall (s) | Collision calls | Result |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| Existing standoff heuristic, fast validation | 2.35 | 50.00 | 131.20 | 12.13 | 212.94 | 129,000 | No connected path; targets 923 to 924 |
| Adaptive search, default settings | 1.01 | 103.40 | 222.88 | 22.30 | 378.91 | 247,597 | Complete path, cost 160.66386 |

Adaptive search tested 352 proposals and accepted its first fully validated
finalist. Independent replay in a fresh scene took another 2.97 s (381.88 s
overall): every returned configuration passed collision, joint-limit and
joint-step checks. Maximum calibrated FK matrix error was 3.28e-8. The footprint
origin was `[-2.211920, 1.886218, 0]` m at yaw 316.376 degrees; the calibrated arm
origin was `[-2.012852, 1.696489]` m in XY. Its farthest XY target was 1.80979 m
away, outside the initial region.

The original exhaustive 416-pose search took about 6 h 9 min, excluding separate
baseline/count and path-audit stages. Its best audited connected path cost was
155.35766: the fast result is about 3.4% higher. That exhaustive study computed
exact alternative counts over many bases, so its runtime is not a like-for-like
validation benchmark. The existing heuristic is faster than adaptive search on
this capture, but returns a disconnected path. These tradeoffs are explicit.

The longer-hose capture is infeasible for the sampled rotations (see audit below).
Adaptive screening rejects the tested proposals without constructing a full path
graph, but is substantially slower than the original failed-target-first search:

| Updated hose | Setup (s) | IK (s) | Collision (s) | Graph (s) | Planning wall (s) | Collision calls |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Existing heuristic | 2.23 | 0.67 | 0.55 | 0 | 4.51 | 394 |
| Adaptive | 0.83 | 67.63 | 74.02 | 0 | 156.39 | 100,255 |

This is a limitation of broad proposal screening on an already infeasible scene,
not a speedup. Keep the original strategy available for such inputs.

Separately, `validation/benchmark_stationary_fast.py --max-lazy-passes 2` compares
exhaustive and bounded-lazy **selected-base validation**, using synthetic IK and
real Bullet collisions (150 targets, 48 options each, median of three runs):

| Case | Exhaustive wall (s) / calls | Fast wall (s) / calls |
| --- | ---: | ---: |
| Clear | 1.293 / 7,200 | 0.403 / 150 |
| Collision-heavy | 0.650 / 7,200 | 0.487 / 3,750 |

Both modes return identical configurations and costs. Setup, IK, collision and
graph timings are also recorded by that runner. These small synthetic workloads
do not predict the captured-case adaptive search runtime.

Run `validation/benchmark_adaptive_stationary.py CASE --output OUTSIDE_CHECKOUT`.
Add `--strategy heuristic` for the existing fast-validation baseline. The runner
uses calibrated IK, 24 TCP rotations, detailed base/GPS collisions, case integrity
checks, package/code fingerprints and a freshly reconstructed scene for final
collision, calibrated FK, joint-limit and joint-step verification.

The intermediate `spraying_ee/one_collision_WS2.stl` used for that hose test extended about 1.008 m behind the
captured TCP. A derived case replaces only that collision mesh; the original
capture remains intact. At target index 6 (target 7), all 24 sampled TCP rotations
intersect `collision_meshes[18]` (the 19th obstacle). Direct tool placement was
checked against calibrated IK-driven tool poses. This contact is independent of
base placement for those target/tool poses. It is not evidence that unsampled TCP
rotations are impossible, nor a reason to remove an obstacle from collision checking.

## Ten-case validation with the shortened tool

On 2026-10-04, ten stationary-printing captures were evaluated with the same
adaptive settings: 24 TCP rotations, detailed base and GPS collision geometry,
and a 1 m soft footprint-clearance preference. All captures contained the updated
tool extending 0.908 m behind the TCP; all 5,402 collision triangles matched the
source STL. Captured calibration, joint limits, fixed joints and environments
were retained. Case suffixes below are labels, not additional planner constraints.

| Case label | Targets | Result | Total time (min) |
| --- | ---: | --- | ---: |
| `0_8_35` | 1,102 | Verified path | 9.3 |
| `8_16_35` | 1,017 | Verified path | 8.4 |
| `16_28_40` | 1,805 | Verified path | 13.3 |
| `28_40_40` | 1,951 | Verified path | 13.3 |
| `40_50_40` | 1,372 | Verified path | 12.0 |
| `50_56_35` | 907 | No path found | 2.2 |
| `56_62_35` | 949 | Verified path | 8.8 |
| `50_54_35` | 583 | Verified path | 4.8 |
| `50_55_35` | 744 | No path found | 2.8 |
| `20261004_113701_6ba1788e` | 297 | Verified path | 2.3 |

Eight cases produced complete paths covering 9,076 targets. Each successful path
was independently replayed in a fresh collision scene and checked against
calibrated TCP FK, joint limits and joint steps. Windows CPU policy restoration
and input/code fingerprints were verified for all ten runs. Swept transitions
were not checked.

Up to two independent planner processes ran concurrently. The initial seven-case
batch took 33.8 minutes; the three-case extension took 5.2 minutes. Per-case times
include setup and fresh verification and reflect concurrent CPU load, so they
are not directly comparable with the isolated benchmarks above.

The bounded search found no path for `50_56_35` or `50_55_35`. Their last recorded
failed targets were respectively 901 and 744 (one-based); all 192 tested
configurations at those candidate/target pairs collided with robot geometry.
Tool/chassis and robot self-collisions were prominent. Separate tool-only audits
found no tool/environment contact at any solver-flagged target in either case
across all 24 sampled rotations. These failures do not prove global infeasibility.
The shorter `50_54_35` capture succeeded; the result does not establish an exact
print-range boundary because the exported target sets differ.

Research artifacts and input geometry remain outside the repository. Use the
captured-case runner above to reproduce evaluations from the original exports.

Offline Grasshopper tests use mocked Rhino types; live Rhino UI recomputes have
not been exercised in this study. The complete repository suite passes 465 tests,
including exhaustive/fast/capped graph equivalence, deterministic ties, starting
poses, limits, reachability failures, cache isolation, count completeness,
recompute/reload behavior, output clearing and CPU-policy restoration.
