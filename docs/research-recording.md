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
  collision-free count, rejection reasons, separate IK/expansion/collision durations.
  `collision_checks` counts actual checker calls, `collision_rejections` counts
  rejected calls, and `collision_cache_hits` counts reused results for equivalent
  joint turns. Rejection-reason totals count expanded configurations, not contacts.
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
