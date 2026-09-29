# Shallow arc detection, 2026-09-29

Version 0.1.42 lowers `arc_turn_threshold_deg` from 45 to 15 degrees. The current
872-target input was extracted losslessly from the root input artifact of research
run `21528739a5cd440ea85505526294dd15`, with its recorded metre scale and nominal
1.0 m normal / 1.3 m tangential offsets. This probe only regenerates geometry.

The old classifier measured approximately 19.65–32.51 degrees across its 19 passes,
so the 45-degree threshold marked all passes straight. A threshold of 20 still
left one pass straight; 15 recognizes all 19. Fits retain the existing limits:
0.03 m RMS and 0.075 m maximum residual on the original TCP XY positions.

Simply lowering the threshold also subdivided ripple patterns in older captures
into short arcs. The final implementation accepts shallow arcs only when a full
interval fits the original points; it uses that fit's turn instead of potentially
flattened or degenerate smoothed-point curvature. Failed fits retain the original
45-degree strong-curvature threshold for subdivision (or a higher configured
threshold). Strong-parent curvature continues to propagate through subdivisions.

| Input | Targets | Final section classification |
| --- | ---: | --- |
| `20260916_165102_2ae63c0d` | 1,607 | 1 straight |
| `20260916_204312_5c6b046f` | 1,561 | 19 straight |
| `20260921_121845_a3a4e122` | 1,319 | 19 straight |
| `20260922_151951_888e67f7` | 1,481 | 19 arcs |
| Current recorded input | 872 | 19 arcs |

The earlier four captures retain their section classifications. No input above
has unresolved sections. Current base frames remain upright at ground level;
maximum adjacent translation is 0.089646 m and yaw change is 0.037403 rad, below
the recorded 0.25 m/rad limits. A fixed yaw slider does not change these step
magnitudes. This does not check mounting-aware reach, IK, arm continuity, or
configuration collisions, and does not certify a fabrication-ready trajectory.

Research databases, extracted target planes, generated base planes, capture
hashes and comparisons are outside Git at
`C:\Users\david\Documents\GitHub\research_runs\arc_threshold_20260929`.
Use `regressions_final.json` and `base_planes_final.json` for the final result;
earlier files retain the initial threshold-only experiment for comparison.
The saved `targets.json` can be supplied through `as_plane()` to
`generate_base_path()` to reproduce the current geometry with default settings.

The existing GH `geometry_options` input can override the default, for example
`{"arc_turn_threshold_deg":20}`. A running invocation keeps its loaded code;
the file-loaded component uses the new default on its next recompute.
