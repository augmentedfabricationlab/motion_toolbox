# Adaptive base yaw validation — 0.1.39

`base_yaw_margin_degrees=30` permits up to +/-30 degrees of additional rotation
around the prepared straight/arc heading plus the fixed `base_yaw_degrees`
slider. The additional rotation is applied at the base origin, after regenerating
any radial/tangential offsets. The margin is a hard bound at every original
target, including overlapping repair ramps. Setting it to zero restores the
previous fixed-yaw search. Already-valid paths keep nominal yaw, and
`adapt_offsets=False` disables all repairs.

The search tries rotation at the current position first, then combined offset
and yaw candidates. Its angular grid is 5 degrees, includes both exact margin
endpoints, and refines to 2.5 and 1.25 degrees. It retains one viable angle per
coarse XY candidate to bound the beam. Quintic repair ramps smooth the changes;
full original-target validation still checks reach, configured collisions,
joint connectivity, and base translation/yaw limits. This is a finite search,
not a guarantee of the global optimum over continuous base poses.

## Tests and capture replay

All **263 tests pass**, including positive/negative rotations, headings crossing
the +/-180-degree wrap, custom margins, combined radial and signed arc-distance
repairs, final arm/tool rejection after a base pass, yaw-step limits, valid-path
invariance and the zero-margin compatibility path.

Capture `20260922_151951_888e67f7` was replayed with the chassis/lift cover boxes,
the unchanged 1.0/1.3 m preferred offsets, zero fixed yaw, 30-degree margin,
16 TCP rotations, calibrated IK and normal research recording:

| Measurement | Result |
| --- | --- |
| Original targets / validated configurations | 1,481 / 1,481 |
| Connected trajectory | Validated |
| Graph cost for the selected base path | 63.88523430857744 |
| Targets with nonzero adaptive yaw | 477 |
| Final adaptive yaw range | -21.25 to +21.25 degrees |
| Final radial offset range | 0.750 to 1.025 m |
| Final signed arc-distance offset range | 0.900 to 1.300 m |
| Recorded repair proposals | 138 |
| Geometry preprocessing | 0.809 s |
| Calibrated IK | 257.182 s |
| Collision checking | 40.297 s |
| Graph solving | 21.480 s |
| Repair time, inclusive | 428.867 s |
| Complete planning, inclusive | 466.791 s |
| End-to-end replay | **469.976 s — 7m 50s** |

Inclusive phase times overlap and must not be added. Other work overlapped the
run; the observed time is not a controlled speedup comparison with the earlier
17m 20s fixed-yaw cover replay. Some exploratory proposals used larger angles;
the final accepted path uses at most 21.25 degrees of its 30-degree allowance.

A fresh-world confirmation using committed code `dad9d8a`:

- Reconstructs all base poses from the prepared arc geometry, saved two-column
  offsets, fixed yaw and per-target adaptive yaw. Matrices match to 1e-12.
- Checks the yaw allowance at every target.
- Rechecks all 1,481 saved base/configuration pairs against the cover boxes and
  detailed arm/tool/environment/self collision model: **zero failures**.

Configuration collisions are validated; swept transitions remain outside this
workflow. The accepted path has no unreachable/unchecked targets or movement
transition failures. Exact shortest-arm-path certification applies to the
selected base poses and sampled TCP orientations.

## Reproduction and artifacts

```powershell
python validation/validate_mobile_base_case.py CASE --output OUTSIDE_GIT --base-collision-model boxes --base-yaw-margin-degrees 30
```

The unchanged case SHA-256 is
`2bd6cd4c0fcb08c36f2d20363b3b5c9d906226beb41168ca7346c4fe6aad6469`.
Full results, captured geometry and research archives remain outside Git in
`C:\Users\david\AppData\Local\Temp\motion-yaw-20260922`.
`result.json` holds the trajectory and yaw arrays; `configured_yaw_check.json`
and `confirmation/` contain the committed-code confirmation and source snapshots.
