# Chassis cover and lift collision model — 0.1.38

The mobile robot now uses two robot-aligned 3D boxes in preliminary and final
collision checks. The first encloses the chassis and all four wheels, filling
the chamfered corners. The second encloses both lift bodies at the configured
extension. The arm subtree and attached tool retain detailed collision geometry.
Base accessories (including lidar, cameras, IMU and GPS) contribute no collision
shapes; all coordinate frames and original robot assets remain intact.

For capture `20260922_151951_888e67f7`, the generated dimensions in metres are:

| Box | X | Y | Z |
| --- | ---: | ---: | ---: |
| Chassis and four wheels | 1.282002 | 0.872002 | 0.669142 |
| Lift at captured extension | 0.208002 | 0.208002 | 1.000402 |

Bounds include PyBullet's collision margins. They are measured once in each
mounting link's axes, including fixed-joint transforms. They rotate with the
robot. Changing the configured lift/wheel joints requires constructing a new
world with those values, so a stale box cannot silently remain in use.

## Validation

- All **252 tests pass**, including corner filling, rotated bases, extended lift,
  rejection of stale bounds, retained arm collisions, unchanged frames/assets,
  inertial-frame offsets and detailed fallback for other robot models.
- Checking all 1,481 configurations from the previous detailed-model trajectory
  finds **85 environment collisions** with the enlarged chassis. Earlier
  detailed-model acceptance therefore does not apply to the cover.
- A new automatic arc replay with adaptive offsets returns **1,481 validated
  configurations in one connected path**, cost **63.53452027004483**. Final radial
  offsets span **0.7–1.0 m** and signed arc offsets **0.8–1.3 m**, starting from
  the unchanged 1.0/1.3 m preferences. It records 116 repair proposals.
- A separate fresh-world confirmation using committed code `397fe6f` checks
  every final base/configuration again: **1,481 checked, zero failures**. Its box
  definitions exactly match the planning run. This confirmation includes the
  final optimization that skips empty collision links.
- Collision validation remains configuration-only. Base/joint movement limits
  and connectivity are validated; swept transitions are not collision-checked.

The full replay took **1,039.542 s (17m 20s)** with normal research recording.
Reported phases include geometry 2.282 s, setup 3.400 s, IK 527.034 s, collision
129.905 s and graph solves 108.086 s. Repair time is inclusive (907.941 s) and
must not be added to those phases. Unit tests and other work overlapped the run;
this is an observed acceptance time, not a controlled comparison against the
earlier 7m 23s result with a different physical collision model.

## Empty-link optimization

The preliminary checker now visits only static links with collision shapes.
Three matched trials check the same 1,481 poses using the same cover geometry,
rotating measurement order. Every collision decision and failure reason matches.

| Trial | Previous loop (s) | Active collision links only (s) |
| --- | ---: | ---: |
| 1 | 1.0913 | 0.7894 |
| 2 | 1.2380 | 0.7630 |
| 3 | 1.1543 | 0.9907 |

This reduces preliminary base-check time by **14–38%**. It does not replace
geometric collision checks or claim an end-to-end speedup.

## Reproduction and local artifacts

```powershell
python validation/validate_mobile_base_case.py CASE --output OUTSIDE_GIT --base-collision-model boxes --base-yaw-margin-degrees 0
```

Use `--base-collision-model detailed` for the previous geometry. Grasshopper's
`collision_options` accepts the same `base_collision_model` setting; `auto` is
the mobile default and selects these boxes for the recognized robot profile.

The case SHA-256 remains
`2bd6cd4c0fcb08c36f2d20363b3b5c9d906226beb41168ca7346c4fe6aad6469`.
Full trajectory, model metadata, source snapshots and research artifacts remain
outside Git in `C:\Users\david\AppData\Local\Temp\motion-cover-20260922`.
`result.json` is the replay output; `configured_model_check.json` and
`confirmation/` contain the fresh-world confirmation. The base-loop benchmark is
stored alongside that directory as `motion_cover_base_benchmark.json`.
