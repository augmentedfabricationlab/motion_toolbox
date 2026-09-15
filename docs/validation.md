# Validation record

Completed in the new `motion_toolbox/.venv` on Windows, Python 3.9.13,
NumPy 2.0.2, PyBullet 3.2.7, COMPAS FAB 1.1.4, COMPAS 2.15.1.

- 45 automated tests passed across the three packages, including exhaustive
  shortest-path comparisons, analytic IK/FK round trips, tool/base transforms,
  joint limits/revolutions, partial planning, stationary/mobile feasibility,
  rolling buffer retries, collision worlds and meshes, JSON workflows, numeric
  toolpaths, full odometry transformation and ROS callback/message contracts.
- Standalone trajectory, base-search and static CLI examples completed.
- Original IK candidate outputs matched for 100 random poses and 100 recorded
  source JSON targets. Project datasets were read, not copied into the packages.
- Analytic FK matched 20 random configurations of the source UR20 URDF using
  its controller `base` frame and `tool0`. See `validation/ik_parity.json`.
- Both complete Grasshopper component scripts executed with actual Rhino planes,
  Grasshopper DataTrees, COMPAS objects and (for the robot component) PyBullet.
  Each returned three branches containing 18 joint values. The added stationary
  placement component also passed with actual Rhino/Grasshopper types and
  PyBullet, returning one base and the full three-target joint path. See
  `validation/grasshopper_results.json` and `validation/grasshopper_smoke.py`.
  This is a headless runtime test, not a manually wired Grasshopper document.
- The actual source MobileRobot and MultiTool classes passed a smoke test with
  a synthetic URDF: active tool frame, explicit offline arm calibration/lift,
  model export, attached collision geometry, and tool motion were checked.
- Rhino 8.30.26103.11001 headless tests passed for raster/sine surface paths,
  ribbon construction, TCP offset/tilt, plane conversion, surface-edge base
  candidates and curve-distance placement. Python.NET collection conversions
  were fixed in the extracted code after the first runtime test exposed them.
- All 104 original Python files match the initial SHA-256 snapshot. No original
  repository was cleaned, rewritten or committed.
- Editable installation, wheel builds of all three packages, compilation of all
  source modules and `pip check` completed successfully.
- Reproducible graph benchmarks and measurements are in `benchmarks/`.

Tests use synthetic collision fixtures and selected recorded IK input. They do
not certify the physical robot, real tool/environment meshes, calibrated URDF,
controller limits or live ROS middleware. No motion was commanded to a robot.
Acceleration constraints, dynamic-obstacle prediction and nonholonomic navigation
are outside the built-in mobile planner; it assumes an upright holonomic base.

Re-run the suite from the parent directory:

```sh
python -m pytest motion_toolbox/tests toolpath_toolbox/tests motion_toolbox_ros/tests -q
python motion_toolbox/validation/verify_sources.py --workspace .
```

The Rhino smoke test requires Rhino 8 and Python.NET/Rhino.Inside in the test
environment; it explicitly requests a windowless host and does not save documents.
The first sandboxed host startup failed with COM E_FAIL. Running the same
windowless test with access to the installed Rhino runtime succeeded.

The follow-up completeness audit replaced the helper-only `examples/grasshopper.py`
with a runnable component and a standalone reusable `plan_robot` API. Regression
checks now cover component missing-input/error/blocked states, named fixed-joint outputs,
scene reuse, static JSON option validation, limits after target editing, rolling
buffer boundaries and previous-base transitions, stationary edge callbacks,
tilted stationary collision checks, clearance and ROS execution-index handling.
The audit also corrected a Python.NET DataTree type incompatibility found by the
real Grasshopper runtime, offline tool-link FK selection, and surface input and
degenerate-frame validation. No unfinished implementation markers remain in the
new packages' source or motion-planning examples.
