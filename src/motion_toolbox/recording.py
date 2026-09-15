"""Local, versioned research records. No services or third-party storage required.

Use ``with ResearchRun(directory, config=..., seed=...):`` to group operations.
Otherwise runs share ~/Documents/GitHub/research_runs, independent of working directory.
TOOLBOX_LOG_DIR can explicitly override that central location.
Set TOOLBOX_RECORDING=0 to disable automatic recording, or TOOLBOX_LOG_LEVEL=detail
to include individual IK/collision calls. Explicit runs override the environment.
"""
from contextlib import contextmanager
from contextvars import ContextVar
from functools import wraps, partial
from pathlib import Path
import argparse
from array import array
import csv
import dataclasses
import gzip
import hashlib
import importlib.metadata
import inspect
import json
import math
import marshal
import os
import platform
import sqlite3
import subprocess
import sys
import threading
import time
import traceback
import uuid
import warnings


_active = globals().get('_active', ContextVar('toolbox_research_run', default=None))
_parent = globals().get('_parent', ContextVar('toolbox_research_step', default=None))
_suspended = globals().get('_suspended', ContextVar('toolbox_recording_suspended', default=False))
SCHEMA_VERSION = 1
RECORDING_VERSION = 3
DEFAULT_LOG_DIRECTORY = Path.home() / 'Documents' / 'GitHub' / 'research_runs'


def current_run():
    return None if _suspended.get() else _active.get()


@contextmanager
def suspend_recording():
    """Exclude instrumentation from a benchmark's timed region in this context."""
    token = _suspended.set(True)
    try:
        yield
    finally:
        _suspended.reset(token)


def _json(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def _type(value):
    return type(value).__module__ + '.' + type(value).__qualname__


def loaded_versions():
    """Identify loaded code separately from files/install metadata (Rhino caches imports)."""
    result = {}
    for name, module in list(sys.modules.items()):
        if name.split('.')[0] not in ('motion_toolbox', 'motion_toolbox_ros', 'toolpath_toolbox'):
            continue
        path = getattr(module, '__file__', None)
        if not path or not Path(path).is_file():
            continue
        functions = []
        for key, value in sorted(vars(module).items()):
            if inspect.isfunction(value) and value.__module__ == name:
                functions.append((key, inspect.unwrap(value).__code__))
            elif inspect.isclass(value) and value.__module__ == name:
                for member, method in sorted(vars(value).items()):
                    if isinstance(method, (staticmethod, classmethod)):
                        method = method.__func__
                    if inspect.isfunction(method):
                        functions.append((key+'.'+member, inspect.unwrap(method).__code__))
        digest = hashlib.sha256()
        for key, code in functions:
            digest.update(key.encode()); digest.update(marshal.dumps(code))
        result[name] = dict(version=getattr(module, '__version__', None), path=str(Path(path).resolve()),
                            file_sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),
                            loaded_code_sha256=digest.hexdigest(),
                            api_versions={k:v for k,v in vars(module).items()
                                          if k.endswith('_VERSION') and isinstance(v,(int,str))})
    return result


class ResearchRun:
    """One directory per run; committed SQLite steps survive interrupted processes.

    A run left with status ``running`` was not cleanly closed. Recording failures
    warn and mark the run incomplete when possible; they never stop robot control.
    Seed is metadata only: callers must initialize their random generators.
    """

    def __init__(self, directory=None, *, name='experiment', config=None, seed=None,
                 detail=False, tags=None):
        self.id = uuid.uuid4().hex
        self.path = Path(directory or os.getenv('TOOLBOX_LOG_DIR') or DEFAULT_LOG_DIRECTORY).expanduser().resolve() / self.id
        self.name, self.config, self.seed = name, config, seed
        self.detail, self.tags = detail, tags or {}
        self.lock = threading.RLock()
        self.db = None
        self.failed = False
        self.closed = False
        self._sources = set()
        self._repositories = {}
        self.started = time.perf_counter_ns()
        self._safe(self._open)

    def _safe(self, function, *args, **kwargs):
        try:
            return function(*args, **kwargs)
        except Exception as error:
            try:
                with (self.path/'recording_errors.jsonl').open('a', encoding='utf-8') as stream:
                    stream.write(_json({'error': _type(error), 'message': str(error)})+'\n')
            except Exception:
                pass
            if not self.failed:
                # Warning filters must not turn telemetry failures into control failures.
                try:
                    warnings.warn('Research recording incomplete at {}: {}'.format(self.path, error),
                                  RuntimeWarning, stacklevel=2)
                except Exception:
                    pass
            self.failed = True
            return None

    def _open(self):
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path / 'artifacts').mkdir()
        self.db = sqlite3.connect(str(self.path / 'run.sqlite3'), check_same_thread=False)
        self.db.executescript('''
            PRAGMA journal_mode=WAL;
            PRAGMA synchronous=FULL;
            CREATE TABLE run(id TEXT PRIMARY KEY, schema_version INTEGER, name TEXT,
                started_utc_ns INTEGER, elapsed_ns INTEGER, status TEXT, metadata_json TEXT);
            CREATE TABLE steps(id TEXT PRIMARY KEY, parent_id TEXT, operation TEXT,
                started_utc_ns INTEGER, elapsed_ns INTEGER, cpu_ns INTEGER, status TEXT,
                input_artifact TEXT, output_artifact TEXT, error_json TEXT);
            CREATE TABLE metrics(step_id TEXT, name TEXT, value REAL, unit TEXT, exact_json TEXT);
            CREATE TABLE events(sequence INTEGER PRIMARY KEY AUTOINCREMENT, step_id TEXT,
                utc_ns INTEGER, monotonic_ns INTEGER, name TEXT, payload_artifact TEXT);
            CREATE TABLE artifacts(sha256 TEXT PRIMARY KEY, relative_path TEXT, bytes INTEGER,
                media_type TEXT);
            CREATE TABLE sources(path TEXT PRIMARY KEY, source_artifact TEXT, git_json TEXT);
            CREATE INDEX steps_operation ON steps(operation, status);
            CREATE INDEX metrics_name ON metrics(name, step_id);
            CREATE INDEX events_name ON events(name, sequence);
        ''')
        metadata = dict(python=sys.version, platform=platform.platform(), machine=platform.machine(),
                        processor=platform.processor(), cpu_count=os.cpu_count(),
                        argv=sys.argv, cwd=str(Path.cwd()), seed=self.encode(self.seed), tags=self.encode(self.tags),
                        packages={d.metadata['Name']: d.version for d in importlib.metadata.distributions()
                                  if d.metadata['Name']}, detail=self.detail,
                        config=self.encode(self.config), loaded_modules=loaded_versions(),
                        timing='perf_counter_ns; thread_time_ns',
                        thread_settings={k: os.environ[k] for k in
                            ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS') if k in os.environ})
        self.db.execute('INSERT INTO run VALUES(?,?,?,?,?,?,?)',
                        (self.id, SCHEMA_VERSION, self.name, time.time_ns(), None, 'running', _json(metadata)))
        self.db.commit()

    def artifact_bytes(self, data, media_type='application/octet-stream'):
        digest = hashlib.sha256(data).hexdigest()
        relative = 'artifacts/' + digest + '.gz'
        with self.lock:
            destination = self.path / relative
            if not destination.exists():
                destination.write_bytes(gzip.compress(data, compresslevel=1, mtime=0))
            self.db.execute('INSERT OR IGNORE INTO artifacts VALUES(?,?,?,?)',
                            (digest, relative, len(data), media_type))
        return digest

    def artifact(self, value):
        return self.artifact_bytes(_json(self.encode(value)).encode('utf-8'), 'application/json')

    def _planning_payload(self, operation, value, *, inputs=False, nested=False):
        """Normal logs retain inputs/final paths, not regenerated search arrays."""
        if self.detail:
            return value
        def summary(layers):
            return {'__omitted__': 'generated joint candidates; enable detail recording to archive',
                    'layer_counts': [len(layer) for layer in layers]}
        if inputs and nested and operation == 'motion_toolbox.graph.shortest_path':
            return dict(value, layers=summary(value['layers']))
        if not inputs and operation == 'motion_toolbox.planning.candidates':
            return {'candidate_count': len(value[0]), 'raw_ik': value[1], 'collision_free': value[2]}
        if not inputs and operation in ('motion_toolbox.planning.calculate_partial_trajectory',
                                       'motion_toolbox.robot_planning.plan_robot'):
            return dict(value, ik_solutions_per_node=summary(value['ik_solutions_per_node']))
        return value

    def encode(self, value, seen=None):
        """Lossless numeric payloads; opaque/native objects explicitly flag replay gaps.

        Never iterate generators or invoke arbitrary object repr/property methods.
        File arguments are snapshotted, not merely named. Cycles are marked.
        """
        if value is None or isinstance(value, (bool, int, str)):
            return value
        if isinstance(value, float):
            return value if math.isfinite(value) else {'__float__': str(value)}
        seen = set() if seen is None else seen
        if id(value) in seen:
            return {'__cycle__': _type(value)}
        seen = seen | {id(value)}
        encode = lambda v: self.encode(v, seen)
        if isinstance(value, Path):
            result = {'__path__': str(value.resolve())}
            if value.is_file():
                result['sha256'] = self.artifact_bytes(value.read_bytes())
            return result
        if isinstance(value, bytes):
            return {'__bytes__': self.artifact_bytes(value)}
        if isinstance(value, dict):
            if all(isinstance(k, str) for k in value):
                return {k: encode(v) for k, v in value.items()}
            return {'__mapping__': [[encode(k), encode(v)] for k, v in value.items()]}
        if isinstance(value, (list, tuple, array)):
            return [encode(v) for v in value]
        if isinstance(value, (set, frozenset)):
            return {'__set__': sorted([encode(v) for v in value], key=_json)}
        if type(value).__module__.startswith('numpy'):
            if hasattr(value, 'shape') and value.shape:
                return {'__ndarray__': encode(value.tolist()), 'dtype': str(value.dtype),
                        'shape': list(value.shape)}
            return encode(value.item())
        if dataclasses.is_dataclass(value) and not isinstance(value, type):
            return {'__type__': _type(value), **{f.name: encode(getattr(value, f.name))
                                              for f in dataclasses.fields(value)}}
        if isinstance(value, partial):
            return {'__partial__': encode(value.func), 'args': encode(value.args),
                    'keywords': encode(value.keywords)}
        if inspect.ismethod(value):
            return {'__callable__': value.__module__ + '.' + value.__qualname__,
                    'owner': encode(value.__self__)}
        if inspect.isfunction(value):
            return {'__callable__': value.__module__ + '.' + value.__qualname__,
                    'replay_gap': 'Callable code/closures must be supplied by the experiment'}
        # Our numerical state objects and ROS messages are ordinary data containers.
        if type(value).__module__.startswith('motion_toolbox_ros') and hasattr(value, '__dict__'):
            keys = ('exec_index', 'current_pose', 'current_odom', 'first_odom', 'initial_base',
                    'velocity', 'base_running', 'joint_names', 'lookahead', 'buffer_size')
            return {'__type__': _type(value), 'state':
                    {k: encode(vars(value)[k]) for k in keys if k in vars(value)}}
        if type(value).__module__.startswith(('motion_toolbox', 'toolpath_toolbox')) and hasattr(value, '__dict__'):
            return {'__type__': _type(value), 'state': {k: encode(v) for k, v in vars(value).items()
                    if k not in ('recording', 'db', 'lock') and not k.startswith('_')}}
        if hasattr(value, 'get_fields_and_field_types'):
            return {'__type__': _type(value), 'fields':
                    {k: encode(getattr(value, k)) for k in value.get_fields_and_field_types()}}
        if _type(value) == 'types.SimpleNamespace':
            return {k: encode(v) for k, v in vars(value).items()}
        # RhinoCommon archives preserve the native geometry when supported.
        if type(value).__module__.startswith('Rhino'):
            if type(value).__name__ in ('Point3d', 'Vector3d', 'Point3f', 'Vector3f'):
                return {'__type__': _type(value), 'xyz': [encode(value.X), encode(value.Y), encode(value.Z)]}
            if type(value).__name__ == 'Point2d':
                return {'__type__': _type(value), 'xy': [encode(value.X), encode(value.Y)]}
            if type(value).__name__ == 'Plane':
                return {'__type__': _type(value), 'origin': encode(value.Origin),
                        'xaxis': encode(value.XAxis), 'yaxis': encode(value.YAxis)}
        if type(value).__module__.startswith('Rhino') and hasattr(value, 'ToJSON'):
            try:
                import Rhino
                return {'__rhino_json__': value.ToJSON(Rhino.FileIO.SerializationOptions())}
            except Exception:
                pass
        return {'__opaque__': _type(value), 'replay_gap': 'Archive this input explicitly with run.file()'}

    def file(self, path, *, name='file'):
        """Archive an input/output file at the time it is used, including its hash."""
        return self.event(name, path=Path(path))

    def source(self, function):
        path = inspect.getsourcefile(function)
        if path is None or path in self._sources or not Path(path).is_file():
            return
        self._sources.add(path)
        def save():
            source = Path(path).resolve()
            root = next((p for p in source.parents if (p / '.git').exists()), None)
            git = {}
            if root and str(root) in self._repositories:
                git = self._repositories[str(root)]
            elif root:
                def command(*args):
                    result = subprocess.run(['git', '-c', 'safe.directory='+str(root), '-C', str(root), *args], capture_output=True,
                                            timeout=5, check=True)
                    return result.stdout
                git = {'root': str(root), 'commit': command('rev-parse', 'HEAD').decode().strip(),
                       'status': command('status', '--porcelain').decode(),
                       'diff_artifact': self.artifact_bytes(command('diff', 'HEAD', '--binary'))}
                self._repositories[str(root)] = git
                # Include unchanged helpers and untracked new modules, not just called files.
                for module in (root / 'src').rglob('*.py'):
                    self.db.execute('INSERT OR IGNORE INTO sources VALUES(?,?,?)',
                                    (str(module), self.artifact_bytes(module.read_bytes()), _json(git)))
            self.db.execute('INSERT OR IGNORE INTO sources VALUES(?,?,?)',
                            (str(source), self.artifact_bytes(source.read_bytes()), _json(git)))
            self.db.commit()
        with self.lock:
            self._safe(save)

    def metric(self, name, value, unit='count', *, step_id=None):
        def save():
            encoded = self.encode(value)
            numeric = None
            if isinstance(encoded, (int, float)):
                try:
                    numeric = float(encoded)
                    if not math.isfinite(numeric):
                        numeric = None
                except OverflowError:
                    pass
            self.db.execute('INSERT INTO metrics VALUES(?,?,?,?,?)',
                            (step_id or _parent.get(), name, numeric, unit, _json(encoded)))
            if _parent.get() is None:
                self.db.commit()
        with self.lock:
            self._safe(save)

    def event(self, name, **payload):
        utc, monotonic = time.time_ns(), time.perf_counter_ns()
        def save():
            self.db.execute('INSERT INTO events(step_id,utc_ns,monotonic_ns,name,payload_artifact) VALUES(?,?,?,?,?)',
                            (_parent.get(), utc, monotonic, name, self.artifact(payload)))
            if _parent.get() is None:
                self.db.commit()
        with self.lock:
            self._safe(save)

    def _result_metrics(self, value, prefix='result'):
        if dataclasses.is_dataclass(value) and not isinstance(value, type):
            value = {f.name: getattr(value, f.name) for f in dataclasses.fields(value)}
        if isinstance(value, dict):
            for key, item in value.items():
                # Candidate steps already record these per-target metrics and
                # events. Keep the full diagnostics in the result artifact,
                # without thousands of duplicate SQLite commits at each parent.
                if key == 'target_diagnostics' and isinstance(item, list):
                    self.metric(prefix + '.target_diagnostics.count', len(item))
                    continue
                self._result_metrics(item, prefix + '.' + str(key))
        elif isinstance(value, (int, float, bool)):
            unit = 's' if prefix.endswith('_seconds') else 'unspecified'
            self.metric(prefix, value, unit)
        elif isinstance(value, (list, tuple)) or type(value).__module__.startswith('numpy') and hasattr(value, '__len__'):
            self.metric(prefix + '.count', len(value))
            if isinstance(value, (list, tuple)):
                for index, item in enumerate(value):
                    if isinstance(item, dict) or dataclasses.is_dataclass(item):
                        self._result_metrics(item, prefix + '[{}]'.format(index))

    @contextmanager
    def activate(self):
        """Bind this run in the current thread/task; use inside worker callbacks."""
        if self.closed:
            raise RuntimeError('ResearchRun is already closed')
        token = _active.set(self)
        parent_token = _parent.set(None) if token.old_value is not self else None
        try:
            yield self
        finally:
            if parent_token is not None:
                _parent.reset(parent_token)
            _active.reset(token)

    def __enter__(self):
        self._context = self.activate()
        self._context.__enter__()
        return self

    def __exit__(self, kind, error, tb):
        try:
            self.close('error' if kind else 'ok')
        finally:
            self._context.__exit__(kind, error, tb)

    def close(self, status='ok'):
        if self.closed:
            return
        with self.lock:
            def finish():
                self.db.execute('UPDATE run SET elapsed_ns=?,status=?',
                                (time.perf_counter_ns()-self.started, 'incomplete' if self.failed else status))
                self.db.commit()
            self._safe(finish)
            if self.db is not None:
                self._safe(self.db.close)
            self.closed = True

    def call(self, function, args, kwargs):
        recording_started = time.perf_counter_ns()
        operation = function.__module__ + '.' + function.__qualname__
        self.source(function)
        step = uuid.uuid4().hex
        parent = _parent.get()
        utc = time.time_ns()
        def begin():
            try:
                bound = inspect.signature(function).bind(*args, **kwargs)
                bound.apply_defaults()
                arguments = dict(bound.arguments)
            except TypeError:
                arguments = {'__unbound_args__': args, '__unbound_kwargs__': kwargs}
            for key in ('path', 'urdf_path', 'input_path', 'output_path', 'mesh'):
                if isinstance(arguments.get(key), str):
                    arguments[key] = Path(arguments[key])
            self.db.execute('INSERT INTO steps VALUES(?,?,?,?,?,?,?,?,?,?)',
                            (step, parent, operation, utc, None, None, 'running',
                             self.artifact(self._planning_payload(operation, arguments, inputs=True,
                                                                  nested=parent is not None))
                             if parent is None or self.detail else None, None, None))
            self.db.commit()
        with self.lock:
            self._safe(begin)
        token = _parent.set(step)
        started, cpu = time.perf_counter_ns(), time.thread_time_ns()
        result, error = None, None
        try:
            result = function(*args, **kwargs)
            return result
        except BaseException as exc:
            error = dict(type=_type(exc), message=str(exc), traceback=traceback.format_exc())
            raise
        finally:
            elapsed, cpu_elapsed = time.perf_counter_ns()-started, time.thread_time_ns()-cpu
            def finish():
                output = (self.artifact(self._planning_payload(operation, result))
                          if error is None and (parent is None or self.detail) else None)
                self.db.execute('UPDATE steps SET elapsed_ns=?,cpu_ns=?,status=?,output_artifact=?,error_json=? WHERE id=?',
                                (elapsed, cpu_elapsed, 'error' if error else 'ok', output,
                                 _json(error) if error else None, step))
                self.db.commit()
                self.metric('duration', elapsed, 'ns')
                self.metric('thread_cpu', cpu_elapsed, 'ns')
                self._result_metrics(result)
                if self.detail and args and function.__qualname__.find('.') >= 0:
                    self.event('state.after', state=args[0])
            try:
                with self.lock:
                    self._safe(finish)
                    self.metric('recording_overhead', time.perf_counter_ns()-recording_started-elapsed, 'ns')
                    self._safe(self.db.commit)
            finally:
                _parent.reset(token)


def recorded(_function=None, *, detail=False):
    """Record stable operation boundaries, preserving signatures and exceptions."""
    def decorate(function):
        @wraps(function)
        def wrapped(*args, **kwargs):
            if _suspended.get():
                return function(*args, **kwargs)
            run = current_run()
            owner_run = getattr(args[0], 'recording', None) if args else None
            if run is None and isinstance(owner_run, ResearchRun) and not owner_run.closed:
                with owner_run.activate():
                    return owner_run.call(function, args, kwargs)
            if run is None:
                if os.getenv('TOOLBOX_RECORDING', '1').lower() in ('0', 'false', 'off'):
                    return function(*args, **kwargs)
                if detail and os.getenv('TOOLBOX_LOG_LEVEL', '') != 'detail':
                    return function(*args, **kwargs)
                with ResearchRun(name=function.__module__ + '.' + function.__qualname__,
                                 detail=os.getenv('TOOLBOX_LOG_LEVEL', '') == 'detail') as run:
                    return run.call(function, args, kwargs)
            if detail and not run.detail:
                return function(*args, **kwargs)
            return run.call(function, args, kwargs)
        return wrapped
    return decorate(_function) if _function is not None else decorate


def event(name, **payload):
    run = current_run()
    if run is not None:
        run.event(name, **payload)


def metric(name, value, unit='count'):
    run = current_run()
    if run is not None:
        run.metric(name, value, unit)


def read_artifact(run_directory, digest):
    """Return original bytes, checking content integrity before analysis/replay."""
    if len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest):
        raise ValueError('Expected SHA-256 digest')
    raw = gzip.decompress((Path(run_directory) / 'artifacts' / (digest + '.gz')).read_bytes())
    if hashlib.sha256(raw).hexdigest() != digest:
        raise ValueError('Artifact checksum mismatch')
    return raw


def main():
    parser = argparse.ArgumentParser(description='Export all research metrics to a single tidy CSV.')
    parser.add_argument('directory', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with args.output.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['run_id', 'run_name', 'run_status', 'step_id', 'parent_id', 'operation',
                         'step_status', 'started_utc_ns', 'metric', 'value', 'unit', 'exact_json'])
        for path in sorted(args.directory.rglob('run.sqlite3')):
            db = sqlite3.connect(path.resolve().as_uri() + '?mode=ro', uri=True)
            try:
                writer.writerows(db.execute('''SELECT r.id,r.name,r.status,m.step_id,s.parent_id,s.operation,
                    s.status,s.started_utc_ns,m.name,m.value,m.unit,m.exact_json
                    FROM metrics m CROSS JOIN run r LEFT JOIN steps s ON s.id=m.step_id'''))
            finally:
                db.close()


if __name__ == '__main__':
    main()
