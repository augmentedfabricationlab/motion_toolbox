# Offline mobile capture replay

Use `replay_mobile_case.py` from the repository virtual environment. No Rhino,
Grasshopper, ROS connection or COMPAS robot object is needed. NumPy and PyBullet
must be installed. Keep the capture and output directory outside Git.

```powershell
.venv\Scripts\python.exe validation/replay_mobile_case.py C:\path\to\capture --output C:\path\to\results --timeout 180
```

The parent process kills the separate worker when its time budget expires,
including a worker stuck in native code. This does not change shared planning
API deadlines or the Grasshopper component's thread cancellation behavior.

The replay verifies READY and every manifest hash, then loads the captured
URDF, fixed joints, allowed collision pairs, environment meshes, attached tool
geometry, arm mounting, TCP and planner settings. Sampling and collision
resolution follow the existing component. Automatic research recording is
disabled only in the replay worker; explicit results and logs remain enabled.

Optional arguments:

- `--settings-file settings.json`: override mobile planning settings.
- `--proposal-file bases.json`: validate an explicit list of per-target base
  frames, with original placement, joint and collision constraints.
- `--target-count N`: diagnostic prefix only. With an explicit proposal the
  original full-path bases are truncated; without one, smoothing is recomputed
  on the shortened targets, so its endpoint changes. Never confuse these runs.
- `--captured-source`: replay the verified capture's source snapshot for an
  algorithm comparison. Otherwise use the working repository source.

`run.json` reports completion, errors or timeout. `worker.log` includes progress
and periodic stack samples. A completed worker writes `result.json`, including
per-target states, diagnostic failures, timing, source hashes at worker start,
loaded-code fingerprints, version and whether source changed during execution.
`fk_audit.json` independently compares sampled analytic IK solutions against
the captured URDF's tool0 FK with the captured TCP applied. A calibrated URDF
can differ from the nominal six-parameter analytic model: inspect this audit
before treating analytic reachability as model-accurate target attainment.

Worker completion does not imply a successful plan. A prefix is never a full
fabrication trajectory, and a sampled FK audit is not all-target validation.
No failed bounded search proves that no feasible base path exists.
