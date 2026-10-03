# Research recording

All three toolboxes use `motion_toolbox.recording`. Normal public workflows record
automatically. Importing a package does not create files. Each outer operation gets
a UUID directory under `~/Documents/GitHub/research_runs/`; nested operations
share the run. ROS nodes keep one run for their lifetime. Prefer explicit runs for
experiments so toolpath generation, scene setup, planning, and exports stay together.

On David's computer the shared default is
`C:\Users\david\Documents\GitHub\research_runs`. It does not depend on where
Python, Rhino or ROS starts. Restart existing Python/Rhino/ROS sessions after
updating the package to load this change. An explicit `directory` or
`TOOLBOX_LOG_DIR` still overrides the shared default.

```python
from motion_toolbox.recording import ResearchRun
from motion_toolbox.graph import shortest_path

with ResearchRun("D:/research/runs", name="graph-comparison",
                 config={"description": "two-layer smoke experiment"},
                 seed=42, tags={"trial": 1, "dataset": "example"}) as run:
    result = shortest_path([[[0.0]], [[0.5]]], start=[0.0])
    run.metric("experiment.joint_travel", result.cost, "rad")
    run.event("experiment.note", note="First trial")
    # run.file("robot.urdf", name="experiment.robot")
print(run.path)
```

`seed` records provenance; it does **not** change global random state. Initialize
your RNG explicitly and save its settings/state when using random algorithms.
Close runs with a context manager; for callbacks in other threads, use
`with run.activate():` inside each worker. Separate processes should use separate
runs and a common experiment identifier in `tags`. An explicit run overrides the
environment opt-out. Do not re-enter the same run with `with run:`; use `activate()`.

Environment settings (set before launching Python/Rhino/ROS):

| Variable | Default | Meaning |
| --- | --- | --- |
| `TOOLBOX_LOG_DIR` | `~/Documents/GitHub/research_runs` | Shared parent directory for automatic runs |
| `TOOLBOX_RECORDING` | `1` | Set `0`, `false`, or `off` to disable automatic capture |
| `TOOLBOX_LOG_LEVEL` | normal | Set `detail` to capture individual IK/collision operations |

An explicit run selects detail capture with `ResearchRun(..., detail=True)`.
The toolpath package now depends on motion-toolbox so both share one recorder and
schema, including when called in the same Grasshopper script. Install the current
local checkouts together: `python -m pip install -e ../motion_toolbox -e ../toolpath_toolbox`.
No database server or telemetry service is used. Archives remain local.

The stationary Grasshopper component delegates to
`stationary_workflow.plan_stationary_base`, so robot/calibration setup, collision
scene loading, candidate generation, validation and the final result share one
run. Its `research_run` output identifies that folder; `effective_settings` reports
resolved values and `loaded_code` exposes the loaded-module fingerprints already
captured by the recorder. An active `ResearchRun` is reused, and automatic capture
still honors `TOOLBOX_RECORDING=0`. Scene setup that happened before a supplied
external scene enters this workflow is not retroactively archived: construct that
scene inside an explicit enclosing run and retain any required native assets.

## Stored data

Normal recording stores **top-level inputs, final outputs and process statistics**.
Nested steps retain timings, counts, errors and diagnostics, but no separate input,
output or object-state snapshots. Generated IK candidate arrays are replaced by
explicit omission markers and per-target counts in planner outputs; final planned
joint values remain complete. A standalone graph call still records its supplied
layers because they are its top-level input. Set `TOOLBOX_LOG_LEVEL=detail` (or
`ResearchRun(..., detail=True)`) to archive internal candidate arrays and nested
payloads for debugging. Normal logs cannot directly replay an internal graph;
regenerate its candidates from the original inputs instead.

Compression uses gzip level 1. Metrics/events emitted during a step are committed
at step boundaries, avoiding a disk synchronization per individual counter.

Each run contains `run.sqlite3` plus `artifacts/<sha256>.gz`. SHA-256 addresses the
**uncompressed** bytes. Identical payloads within a run are stored once. Copy the
entire closed run directory to archive it; do not copy only the database. While a
run is active SQLite may also use `-wal` and `-shm` sidecars.

| Table | Contents |
| --- | --- |
| `run` | Schema version, UUID, name, status, UTC start, elapsed time, configuration, tags, seed, Python/package versions, hardware/OS, numerical thread settings |
| `steps` | Operation name, parent step, UTC start, monotonic elapsed nanoseconds, thread CPU nanoseconds, status, input/output artifact hashes, exception and traceback |
| `metrics` | Step, metric name, numeric value, unit, exact JSON representation |
| `events` | Ordered sequence, parent step, UTC and monotonic timestamps, name, payload hash |
| `artifacts` | Hash, relative file path, original byte count, media type |
| `sources` | Python source snapshots and Git commit, dirty status, binary diff for participating source repositories |

All Python modules under each participating checkout's `src` are archived, including
untracked new modules. Installed distributions without Git still archive called
module source. Package versions identify external dependencies; this is not a full
environment/container image. No environment dump or credentials are collected;
explicit function arguments, configuration and input files **are** archived.

Input artifacts bind parameter names and defaults before execution. Outputs are
captured before returning to the caller. NumPy arrays retain dtype and shape;
nonfinite floats use `{"__float__":"inf"}`/`"-inf"`/`"nan"` for valid JSON. Large
integers remain exact in `exact_json`; SQL `value` is a convenience floating-point
column and must not be used for exact path counts. Dataclasses retain field names.
Metrics for sequence outputs include counts; final values remain in artifacts,
except explicitly omitted generated search arrays in normal mode.
Units are explicit for counters and timing. Generic result scalars use
`unspecified` unless supplied by the caller; weighted graph cost is not generally
a physical distance. Use custom metrics with units for research quantities.

## Coverage

- Motion: base generation/search, stationary regions, trajectory candidates and
  filtering, graph search/layer reachability/disconnections, rolling buffer updates,
  robot adapters, scene construction/assets, path editing, JSON input/output and CLI jobs.
- Candidate filtering: raw IK count, expanded solutions within joint limits,
  collision-free count, rejection reasons, separate IK/joint-filter/collision durations. The legacy `joint_expansion_seconds`
  timing field now measures filtering only; no revolutions are expanded.
  `collision_checks` counts actual checker calls, `collision_rejections` counts
  rejected calls, and `collision_cache_hits` counts reused results for equivalent
  joint turns. Rejection-reason totals count rejected configurations, not contacts.
  `collision_check_applied` distinguishes candidate filtering without collision tests.
- Graph: layer/node/possible inter-layer edge counts, per-layer reachability and
  minimum cost, final configurations, indices, exact complete-path count and cost.
- Detail mode: individual FK/IK calls, configuration/base/transition collision tests
  and resulting checker state, including rejection diagnostics. Low-level numeric
  helper arithmetic is not instrumented.
- Toolpath: resampling, parametric surfaces, Rhino surface generation stages, base
  candidate generation, and joint-plan conversion, including output files.
- ROS: resolved parameters/configuration, sensor messages (including ROS headers),
  progress, skipped/failed replans, accepted target publications, base velocity and
  stop publications. Outbound events include ROS-clock time as well as recorder UTC
  and monotonic time. Each node owns a run across callbacks. Existing CSV remains
  available. Publication means the publisher call returned, not confirmed execution.

Scene setup should happen **inside the experiment run**. URDF and referenced mesh
files are archived during loading, and path arguments at supported boundaries are
snapshotted. Inline geometry and native objects use the serializer where supported.
Rhino geometry with `ToJSON` is archived using RhinoCommon serialization. Other
native objects, callbacks/closures, and iterators are explicitly marked with
`replay_gap` or `__opaque__`; generators are never consumed for logging. Supply
native `.3dm`/robot/assets and callback code with `run.file(...)` for these cases.
This is a reproducibility record, not automatic robot-command replay, and does not
replace rosbag for an independent recording of all ROS topics or transport timing.

## Query, export and replay

Export all runs to a tidy CSV, retaining exact values and failure status:

```console
python -m motion_toolbox.recording D:/research/runs --output D:/research/metrics.csv
```

The installed equivalent is `toolbox-metrics`. Open the database with Python's
standard `sqlite3` module, pandas, or a SQLite browser. For example:

```sql
SELECT s.operation, s.status, COUNT(*) AS samples,
       AVG(s.elapsed_ns)/1000000.0 AS mean_ms
FROM steps AS s GROUP BY s.operation, s.status;

SELECT s.operation, m.name, m.value, m.unit, m.exact_json
FROM metrics AS m LEFT JOIN steps AS s ON s.id=m.step_id
WHERE m.name IN ('raw_ik', 'collision_free', 'result.path_count');
```

Read artifacts with checksum verification. A graph run with ordinary list inputs
and no callback can be replayed as follows (no robot execution):

```python
import json
import sqlite3
from pathlib import Path
from motion_toolbox.graph import shortest_path
from motion_toolbox.recording import ResearchRun, read_artifact

directory = Path("D:/research/runs/PASTE_RUN_ID")
with sqlite3.connect(directory / "run.sqlite3") as db:
    digest, = db.execute("SELECT input_artifact FROM steps WHERE operation = ?",
                        ("motion_toolbox.graph.shortest_path",)).fetchone()
inputs = json.loads(read_artifact(directory, digest))
with ResearchRun("D:/research/replays", tags={"original": directory.name}):
    replayed = shortest_path(**inputs)
```

For tagged NumPy/native/dataclass inputs, reconstruct the documented types before
calling the operation. Callables require the original implementation. Never assume
that an opaque marker contains enough information to reproduce an experiment.

## Timing and durability

`duration`/`elapsed_ns` measure function execution, excluding that step's own input
and output serialization. They **include nested recording overhead**. `thread_cpu`
measures the calling thread, not total CPU across native worker threads.
`recording_overhead` estimates source capture and that step's input/output recording
outside its function body; it is not a correction to subtract from all ancestors.
Use run-level elapsed time to measure total observed experiment cost. For clean
algorithm benchmarks, compare repeated trials with automatic logging disabled,
store the externally measured timing with `run.metric(...)`, and document warmup,
thread count, CPU, seed, dataset and repeat count. Do not compare normal and detail
timings as though instrumentation were equal.

Within an active run, `with suspend_recording():` (imported from the recording
module) temporarily disables all automatic instrumentation in the current context.
The existing `benchmarks/graph_benchmark.py` uses this around both algorithms' timed
regions and memory passes, records every timing sample, performs equal warmups,
and archives the generated datasets outside those regions.

SQLite uses WAL and synchronous commits. Recording is synchronous and can delay
callbacks on slow disks; it is not suitable for hard real-time servo loops. Normal
ROS base publications write one event rather than a full state/timing step. Nothing
is sampled or automatically expired. Plan disk capacity and archive closed runs.
On logging failure a warning is emitted once per run and the run is marked
`incomplete` when storage remains writable; original results/exceptions are
preserved. A killed process leaves the run or step `running`, identifying an
interrupted capture. A disk that cannot accept status updates can also leave
`running`; absence of a clean finish must never be interpreted as success.

Validation covers graph replay, nested failures, exact values/nonfinite arrays,
file snapshots and checksums, disabled/detail modes, thread isolation, automatic
runs, and recording I/O failures. Run the normal numerical suites with
`TOOLBOX_RECORDING=0` to avoid archiving test fixtures; recorder tests use explicit
runs or override that setting.


## Mobile planning and coverage audit (0.1.31)

The existing recorder is shared by all motion-toolbox public workflows; no second
logging implementation is introduced. The audit covers these operation boundaries:

| Tool family | Normal record | Optional detail / exclusions |
| --- | --- | --- |
| XY averaging, smoothing, centerline and offset construction | Inputs/outputs at outer calls; nested durations, counts, convergence and geometric metrics | Scalar/vector arithmetic helpers are not separately recorded |
| Robot adapters and calibrated IK setup | Model/tool/mount settings, source fingerprints, fixed joints and setup duration | Individual IK calls require detail; internal solver FK acceptance remains active |
| Candidates and filtering | IK/filter time, original candidate counts, rejection categories and collision policy | Generated candidate arrays omitted from nested normal logs |
| Graph, stationary, prescribed-base and rolling planning | Graph sizes, reachability, selected path/cost, errors and high-level phase durations | Lazy node validation requires `count_paths=False`; it does not claim a count of unchecked paths |
| Mobile geometry, validation and offset repair | Exactness flag, configuration-only policy, applied offsets, trial intervals, anchors, results, cache hits and phase timings | No transition collision checking; no additional production FK audit |
| Collision setup and batches through planners | Scene/model assets, allowed pairs, counters and rejection reasons | Per-configuration/base/edge calls require detail; planner summaries always report the policy |
| File I/O, static CLI and joint editing | Existing public operation records and artifact references | Small indexing/conversion helpers inherit the calling operation's record |

Mobile GH returns the research-run directory and readable diagnostics. Offline
replay explicitly groups loading, setup and planning under one recorded operation
and writes `result.json`, phase progress and a `research/` run under the requested
output directory. Input capture hashes, calibration, collision settings, source
hashes and version identify the run. `end_to_end_seconds` includes worker startup,
scene setup, planning, normal recording and result serialization. The reported
`recording_overhead_seconds` sums measured per-step recorder overhead; do not add
nested phase durations to estimate total time. Geometry/setup, placement, IK,
filtering, collision and graph timings explain work; `repair_seconds` and total
planning/validation durations are inclusive and overlap those phases.

Curvature-aware runs also retain `geometry_mode`, effective `geometry_options`,
section classifications and inclusive fit intervals, unique `section_ids`, shared
centers, separate pass radii, RMS/maximum fit errors, transition indices/weights,
and unresolved section IDs. `classification_seconds` measures section preparation;
`geometry_seconds` includes smoothing and base generation. Fits are prepared once
per planning call and reused by repairs. Arc `applied_offsets` contain radial
distance and signed distance along the enlarged circle, in metres.
Version 0.1.39 additionally records `base_yaw_margin_degrees` (default 30),
per-target `applied_yaw_adjustments_degrees`, and `applied_base_yaw_degrees`
(fixed slider plus adaptive adjustment, relative to geometric heading).
`mobile.offset_repair` events retain proposed `yaw_adjustment_degrees` and
`yaw_refinement_degrees` alongside the existing two offset values. Numeric API,
GH effective settings and offline configuration report the same margin. Setting
it to zero preserves the prior fixed-yaw repair search.

`excluded_collision_links` reports effective omissions from the configured world.
From 0.1.38, `base_collision_geometry` and the `collision.base_geometry` event
also report the requested/effective cover model, local box dimensions/centers,
contributing original links, replaced and omitted shapes, and frozen fixed-joint
values. This physical model change applies to preliminary and final checks;
compare timings/results only with the same effective model. The original assets
and the generated collision-world source remain covered by the shared recorder.
The capture event retains requested collision options; source URDFs are archived
unchanged. GPS frame links remain present. `mobile.validation_policy` and
`collision_failed_layer_policy` report completion of exact checks in failed
candidate layers for all path geometries from 0.1.41. Earlier versions enabled
this only on paths containing arcs. Single-target repair probes retain lazy
selected-node checks. `mobile.repair_policy` records the traversal strategy and
retained beam width; production repairs now use `progress_first` on all geometries.
Additional graph rejections count only configurations actually checked and found
invalid. Unselected, unchecked candidates are never described as collision-free.
Placement and collision phase totals include the inexpensive repair-anchor group
screening as well as full validators; repair time includes those phases and IK.

Package versions, loaded-module fingerprints (including `xy_sections`) and source
snapshots use the existing recorder. Offline results additionally include source
hashes, manifest/case hashes, calibration and effective run settings. Experimental
base screens are explicit in replay configuration and report setup, screen and
fallback timings/counters in `base_screening`. Their standalone benchmark disables
automatic recording in timed loops and writes three matched trial records outside
Git; it does not claim end-to-end acceleration from a screening measurement.

Normal logs retain final paths but aggregate per-target frame metrics and repair
counts instead of recording thousands of repeated scalar metrics. Detailed tracing
remains opt-in. Explicit KeyboardInterrupt/SystemExit exits are marked interrupted;
a forcibly terminated process may retain running status. Logging errors remain
visible and do not change numerical results.

The collision-order comparison is reproducible with
`validation/benchmark_collision_order.py CASE --output OUTSIDE_GIT`. This benchmark
compares identical full-case IK layers using existing toolbox functions; it is not
a selectable production planner mode. Its standalone call recording overhead is
included in its measurements; acceptance runs use the actual grouped workflow.

Slab Net Zero's inspected PyBullet workflow uses `ur20_spraying_tool.urdf`, a fixed
spawn pose per batch, and separate environment/self/ground checks (currently enabled
by its wrapper, ground offset -1 m). The mobile captures use the chassis/lift/arm
model, calibrated flange TCP, attached tool collision mesh and captured allowed
pairs/environment. Individual Bullet collision queries are fast in both designs;
these different models/checks prevent a direct historical runtime comparison.
The new capture benchmark measured 108394 configuration checks before graph search
versus 1319 after graph search, both with optimal joint cost 60.77748430022862.

## Windows execution policy (0.1.32)

Mobile planning and the offline replay operation request HighQoS on the calling
thread for the scope of the operation. Nested scopes reuse the request, and the
prior policy is restored on success, failure or interruption. The request also
applies with recording disabled. Unsupported APIs retain the default policy and
report the error without changing the numerical result. The result's
`execution_policy` and `execution.cpu_policy` / `execution.cpu_policy_finished`
events retain the previous/requested Windows state and restoration outcome.
State arrays contain version, control mask and state mask, respectively.

This is an advisory scheduling request, not a process-priority increase, CPU
affinity restriction, power-plan change or runtime limit. Microsoft documents
visibility/focus-dependent scheduling and explicit thread requests in
[Windows Quality of Service](https://learn.microsoft.com/en-us/windows/win32/procthread/quality-of-service).
A disposable-process probe of 2388 identical IK configurations measured 1.7-2.0 s
with explicit throughput requests versus 3-18 s under the default policy. Full
capture acceptance, rather than this probe, determines whether the runtime goal
is met. Earlier runs affected by background throttling are retained outside Git.

## Captured-case acceptance (2026-09-21)

Version 0.1.32 at implementation commit `29dfbec` completed the latest capture
`20260921_121845_a3a4e122` twice in fresh processes with identical source hashes,
normal research logging, 16 TCP-Z rotations and no saved-trajectory reuse:

| Measurement | First final run | Fresh-process confirmation |
| --- | ---: | ---: |
| End to end, including setup and recording | 143.28 s | 187.19 s |
| Setup | 2.02 s | 3.24 s |
| Geometry | 1.90 s | 3.33 s |
| Placement screening | 3.33 s | 5.75 s |
| Calibrated IK | 73.62 s | 125.27 s |
| Joint filtering | 0.28 s | 0.42 s |
| Configuration collision checks | 2.37 s | 2.62 s |
| Exact graph search | 15.99 s | 21.31 s |
| Measured recorder overhead | 26.46 s | 7.23 s |

Phase timers are not an additive decomposition of end-to-end time. Both runs
return 1319 upright base frames and connected arm configurations, with exact
minimum summed arm-joint travel 60.77748430022862 over the sampled configurations
for that base path. Graph-first checks 1319 selected configurations from 108394
candidates and certifies the first shortest path. The separate full-case order
comparison measured 9.52 s graph-first versus 100.19 s collision-first after
common candidate generation, with identical optimal cost.

The base path has maximum translation step 0.04665 m, yaw step 0.01567 rad and
joint step 0.43488 rad, within the capture's 0.25 m, 0.25 rad and 2.5 rad limits.
Maximum calibrated arm-origin XY reach is 1.603 m against the 1.75 m limit.
Configuration collisions are checked at every original target; transitions are
not collision-checked. These captures supply no target timing, so speed checks
are covered by targeted tests rather than claimed for the captured trajectories.

Earlier captures `20260916_165102_2ae63c0d` and `20260916_204312_5c6b046f`
validate all 1607 and 1561 targets with the same numerical implementation before
the final scheduling wrapper. They ran concurrently under variable scheduling,
with a recorded process-local CPU-policy change late in validation, and took
2867.09 s and 2607.44 s. Those are correctness regressions, not performance
acceptance measurements. All three captures succeed at the preferred 0.9 m
normal / 1.2 m tangential offsets; controlled tests exercise adaptive repair.

The 216-test suite passes. GH loading, defaults and diagnostics are covered by
tests, and offline replay uses the same numerical workflow; the Rhino UI itself
was not executed for this acceptance. Results, per-target frames/configurations,
settings, source/capture hashes, research databases, CSV metrics and plots are
stored outside Git under `motion_planning_cases/mobile_adaptive_20260921`.
Reproduce a fresh run with:

```powershell
.\.venv\Scripts\python.exe validation\validate_mobile_base_case.py CASE_DIRECTORY --output NEW_OUTSIDE_GIT_DIRECTORY --rotation-steps 16
```
