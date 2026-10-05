# Mobile configuration continuity validation — 2026-10-05

Case: `20261004_215354_587eccf7`, 2,721 targets, 24 TCP rotations, default
adaptive mobile base planning. No captured geometry is included here.

The previously saved straight-line fallback result switched elbow branch at
731 → 732 and wrist branch at 2488 → 2489 (zero-based indices). Both passed the
2.5 rad joint step limit. This reproduces why joint step limits alone were
insufficient.

With the new branch constraint, the complete adaptive replay found a valid
trajectory in 278.28 s (279.32 s including worker startup). It retains nominal
UR shoulder/elbow/wrist signs `[-1, +1, -1]` throughout. The graph still selects
the exact shortest arm path for each tested base proposal, restricted to allowed
branches and the existing joint, movement and collision constraints.

An independent recheck loaded the captured robot, calibration, tool, environment
and result collision settings. It checked all 2,721 returned configurations:

- Zero branch changes; shoulder sign also checked independently using nominal
  forward-kinematics wrist-center geometry.
- Every base and robot configuration collision check passed.
- Joint ranges and movement limits passed; maximum joint step: 0.29344865 rad.
- Maximum base translation step: 0.04065564 m; yaw step: 0.01734505 rad.
- Maximum calibrated FK matrix error: 7.0037e-8. TCP origins and Z axes preserved.

Replay command, from the repository with the test dependencies installed:

```powershell
.venv/Scripts/python validation/validate_mobile_base_case.py CASE --output OUTSIDE_REPOSITORY --rotation-steps 24
```

The branch definition is nominal UR geometry, also used for calibrated solvers.
This does not certify arbitrary interpolated motion, every calibrated singularity,
or swept collision clearance. Live Rhino was not exercised.
