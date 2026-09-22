# Curvature-aware planning validation — 2026-09-22

These results describe version **0.1.37**, using detailed base geometry with GPS
excluded. Version 0.1.38 introduces chassis/lift cover boxes; this changes the
physical collision model and requires new validation. In particular, 85 of the
old curved trajectory's configurations overlap the enlarged chassis box.

The curved 1,481-target capture produces a fully validated connected trajectory
with automatic arc geometry and adaptive offsets. All three earlier captures
retain identical auto/legacy base poses, configurations and graph costs with
matching settings. Simplified base screens remain experimental because neither
passed the required screening speed threshold.

## Geometry and full collision validation

| Capture | Targets | Result | Graph cost |
| --- | ---: | --- | ---: |
| `20260916_165102_2ae63c0d` | 1,607 | Both modes validated; exact equality | 147.83457410306698 |
| `20260916_204312_5c6b046f` | 1,561 | Both modes validated; exact equality | 88.96974680147162 |
| `20260921_121845_a3a4e122` | 1,319 | Both modes validated; exact equality | 60.77748430022862 |
| `20260922_151951_888e67f7` | 1,481 | Auto geometry and repairs validated | 54.75398516462936 |

Historical comparisons use 0.9 m normal and 1.2 m tangential offsets, matching
the earlier validated runs. They compare full-precision base matrices before
sharing an exact IK/configuration cache in an unchanged world. Both modes solve
their graphs; no saved trajectory supplies the answer. Geometry-only checks also
confirm exact auto/legacy equality on these captures at today's 1.0/1.3 m defaults.

The curved replay starts at the current 1.0/1.3 m defaults, zero yaw adjustment,
16 TCP rotations, calibrated IK and captured joint limits. It identifies 19 arc
passes sharing center XY `(-1.1790578711, -0.4009691867)` m. Fitted pass radii span
1.7052–2.0778 m; the largest original-TCP fit residual is 58.675 mm, within the
75 mm maximum. All pass RMS errors are below 30 mm. There are no unresolved
sections. Repairs yield radial offsets 0.725–1.0 m and arc distances 0.925–1.3 m.
Position and orientation regenerate together; reversing the pass does not reverse
the robot frame.

Every selected configuration passes detailed base, arm, tool, environment and
self-collision checks using the captured allowed pairs and fixed joints. GPS
collision geometry is excluded consistently in all compared worlds; effective
links are `robot_gps_base_link` and `robot_gps_link`. Source assets and coordinate
frames remain unchanged. All 1,481 selected configurations and original-target
base poses are present, with zero unreachable/unchecked targets and no movement
or connectivity failures. Swept transitions are outside this validation; the
existing configuration-only semantics remain explicit in the result.

For arcs, a failed selected candidate triggers exact checks of the remaining
candidates in that layer, then one graph rebuild removes all verified collisions.
This preserves the exact shortest path through the sampled joint states. Repairs
first pursue an improving sequence while retaining alternatives. A development
comparison with the earlier breadth traversal returned identical bases and final
cost while reducing recorded repair proposals from 393 to 108; overlapping runs
and reporting changes make those elapsed times unsuitable as a speedup claim.

## Timing

The acceptance replay uses normal research recording, Windows HighQoS for the
planning thread, Python 3.9.13 and PyBullet (build Jul 31, 2026). Phase timings and
the complete output are stored with the replay outside Git. Repair and validation
times include IK, collision and graph work; these inclusive durations must not be
added to the individual phase timings. Setup and process startup are included in
end-to-end time. The source snapshots and loaded-code fingerprints identify the
executed implementation.

| Phase | Seconds |
| --- | ---: |
| Section classification/fitting (within geometry) | 0.117 |
| Geometry preprocessing and initial base generation | 0.707 |
| Scene/solver setup | 0.927 |
| Placement checks, including repair anchors | 15.034 |
| Calibrated IK | 229.320 |
| Joint filtering | 1.091 |
| Collision checks, including repair anchors | 76.340 |
| Graph solves | 33.874 |
| Adaptive repair, inclusive | 381.863 |
| Complete planning, inclusive | 438.538 |
| End-to-end replay | 442.552 |

This is one acceptance run, with background scheduling uncontrolled, rather than
a matched end-to-end speedup experiment. Final bases, configurations and cost
match the prior successful replay exactly. Recorded planner source hashes match
the tested source; the final reporting changes do not alter the trajectory.

## Simplified base screening experiment

Three matched trials use 2,962 poses: all preferred and final repaired base poses
from the curved capture. Method order rotates between trials. The scene and fixed
joints remain unchanged; construction is measured separately. Collision booleans
and failure reasons match the exact base checker for every pose in every trial.

| Trial | Existing checker (s) | XY rectangle (s) | PyBullet box (s) |
| --- | ---: | ---: | ---: |
| 1 | 0.8951 | 1.3858 | 1.0567 |
| 2 | 1.6849 | 2.2493 | 1.9503 |
| 3 | 1.7240 | 2.6179 | 2.2363 |

The rectangle clears 871 poses and requests exact confirmation for 2,091; the box
clears 2,676 and requests confirmation for 286. Rectangle setup takes
0.100–0.184 s; box setup takes 0.0008–0.0019 s. Both are slower in every trial,
even before setup is counted. Neither passes the required 10% screening reduction.
Consequently, no shortcut is enabled and no end-to-end acceleration is claimed.
Three additional matched end-to-end runs would be required before enabling any
future shortcut that first passes the screening gate.

The rectangle clips obstacle triangles to world Z=0–1 m, retains concavities,
uses rectangle rotation and tests both small contained obstacles and enclosed
rectangles. Unsupported scene/slab/clearance cases fall back to exact checking.
The box encloses static collision links after fixed-joint transforms. Both omit
GPS. Simplified overlap only requests exact confirmation; it never by itself
discards a candidate. All accepted configurations still undergo detailed 3D tests.

## Reproduction and provenance

```powershell
python validation/compare_section_modes.py OLD_CASE --output OUTSIDE_GIT
python validation/validate_mobile_base_case.py CURVED_CASE --output OUTSIDE_GIT --rotation-steps 16
python validation/benchmark_base_screening.py CURVED_CASE --result VALID_RESULT_JSON --output OUTSIDE_GIT_JSON
$env:TOOLBOX_RECORDING='0'
python -m pytest -q
```

The full suite passes: **247 tests**. Tests cover exact signed radial/arc displacement, clockwise/counterclockwise
travel, repeated/reversed passes, separate radii, dwell/noise, mixed sections,
inflections, unresolved fits, yaw, units, joins, radial repair regeneration,
projection boundaries, rotated rectangles, concavities, containment, GPS
exclusion and final 3D rejection after a preliminary pass. Graph regression
tests preserve configurations, original indices, limits, costs and tie behavior.

Captured geometry, full benchmark JSON, replay results and research databases
remain outside Git under
`C:\Users\david\AppData\Local\Temp\motion-arc-20260922`.
Relevant runs are `regression_<capture>`, `curved_acceptance_final` and
`screen_benchmark_final.json`. These are local artifacts; archive the complete
research directories, including their compressed artifacts, for long-term use.

Verified `case.json` SHA-256 values:

| Capture | SHA-256 |
| --- | --- |
| `20260916_165102_2ae63c0d` | `83e2b53b1457899f7f35b3c8f3775a90e64ee67e48cd33276c61996b13a8ef2f` |
| `20260916_204312_5c6b046f` | `c31b65bbed108736bba4e2bfa44dac237ec07db7e4f75debc32633befc45f055` |
| `20260921_121845_a3a4e122` | `ec735f77ff22dd75a214b3ab4c37a4484404d89a3df7b6540e1b4a4420fd6f7a` |
| `20260922_151951_888e67f7` | `2bd6cd4c0fcb08c36f2d20363b3b5c9d906226beb41168ca7346c4fe6aad6469` |
