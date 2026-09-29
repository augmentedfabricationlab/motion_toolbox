# Non-arc failed-layer optimization, 2026-09-29

Version 0.1.41 applies the existing arc-path optimizations to every mobile path:
complete exact collision checks in a failing target layer before solving the
whole graph again, and pursue the best improving repair while retaining the
existing alternative beam. All configured constraints and final exact arm-path
selection remain in force. Transition collision checks remain disabled.

## Reproducible synthetic measurement

Run `python validation/benchmark_failed_collision_layers.py --output <outside-Git-directory>`.
The fixture has 872 straight-wall targets, 32 sampled configurations per target,
and 16 TCP rotation steps. Its deterministic collision predicate rejects the
first 31 configurations. It uses fresh caches for each policy, identical input
geometry, and normal `ResearchRun` recording, including source fingerprints.

| Measurement | Selected-node rejection | Complete failing layer |
| --- | ---: | ---: |
| Whole-path graph solves | 32 | 2 |
| Graph time (seconds) | 54.98 | 3.28 |
| Total wall time with setup/recording (seconds) | 90.69 | 26.18 |
| Process CPU time (seconds) | 82.28 | 23.52 |
| Configuration collision checks | 27,904 | 27,904 |
| Summed joint-path cost | 1.742 | 1.742 |

Selected configurations are exactly identical. Neither recording reported a
failure. These are single observations on a shared workstation; geometry and IK
timings also varied. The reduction in graph solves is the strongest comparison.
The synthetic solver/predicate do not measure calibrated robot IK or PyBullet
costs, offset repair speed, or end-to-end feasibility of the current Rhino input.
No claim is made that the current 872-target fabrication case finishes in five
minutes or has been solved by this benchmark.

Results and complete research databases are archived outside the repository at
`C:\Users\david\Documents\GitHub\research_runs\mobile_non_arc_20260929`.
The summary records original temporary and archived run locations.

The 96 relevant regression tests pass, covering non-arc success, complete collision rejection, already-clear
paths that avoid checking alternatives, eager/lazy cost and index equivalence,
bounded joint states, movement/speed limits, offset and yaw repair, and absence
of transition collision checks. Single-target repair probes still use selected
nodes only because they do not incur a long-path graph rebuild.

The file-loaded Grasshopper component picks up 0.1.41 on its next recompute.
Saving this fix does not replace code in an invocation already in progress.
