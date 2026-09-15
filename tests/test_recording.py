import gzip
import json
import sqlite3
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import pytest

from motion_toolbox.recording import ResearchRun, current_run, read_artifact, recorded
from motion_toolbox.graph import shortest_path


def rows(run, sql):
    with sqlite3.connect(run.path / 'run.sqlite3') as db:
        db.row_factory = sqlite3.Row
        return db.execute(sql).fetchall()


def test_graph_is_reproducible_from_saved_inputs(tmp_path):
    layers = [[[0., 0.], [1., 1.]], [[.2, .3], [1.2, 1.3]]]
    with ResearchRun(tmp_path, config={'experiment': 'test'}, seed=42) as run:
        result = shortest_path(layers, start=[0., 0.])
    step = rows(run, "SELECT * FROM steps WHERE operation LIKE '%.shortest_path'")[0]
    payload = json.loads(read_artifact(run.path, step['input_artifact']))
    with ResearchRun(tmp_path) as replay:
        repeated = shortest_path(**payload)
    assert repeated == result
    assert step['elapsed_ns'] > 0
    assert step['cpu_ns'] >= 0
    assert rows(run, 'SELECT status FROM run')[0][0] == 'ok'
    assert rows(run, "SELECT value FROM metrics WHERE name='graph.nodes'")[0][0] == 4
    assert rows(run, 'SELECT * FROM sources')
    assert rows(replay, 'SELECT * FROM steps')


def test_nested_failures_and_defaults(tmp_path):
    @recorded
    def inner(x=7):
        raise ValueError('bad configuration')

    @recorded
    def outer():
        inner()

    with pytest.raises(ValueError, match='bad configuration'):
        with ResearchRun(tmp_path) as run:
            outer()
    steps = rows(run, 'SELECT * FROM steps ORDER BY started_utc_ns')
    assert len(steps) == 2
    assert steps[1]['parent_id'] == steps[0]['id']
    assert all(s['status'] == 'error' for s in steps)
    assert 'ValueError' in steps[1]['error_json']
    assert steps[1]['input_artifact'] is None  # Nested steps retain stats/errors, not payload copies.
    assert rows(run, 'SELECT status FROM run')[0][0] == 'error'
    assert current_run() is None


def test_disabled_is_side_effect_free_and_detail_opt_in(tmp_path, monkeypatch):
    monkeypatch.setenv('TOOLBOX_LOG_DIR', str(tmp_path))
    monkeypatch.setenv('TOOLBOX_RECORDING', '0')

    @recorded(detail=True)
    def numeric(x):
        return x + 1

    assert numeric(3) == 4
    assert not list(tmp_path.iterdir())
    with ResearchRun(tmp_path) as run:
        numeric(3)
    assert not rows(run, 'SELECT * FROM steps')
    with ResearchRun(tmp_path, detail=True) as run:
        numeric(3)
    assert len(rows(run, 'SELECT * FROM steps')) == 1


def test_artifacts_preserve_arrays_files_and_exact_large_metrics(tmp_path):
    asset = tmp_path / 'mesh.bin'
    asset.write_bytes(b'original mesh')
    with ResearchRun(tmp_path / 'runs') as run:
        digest = run.artifact({'array': np.array([1., np.inf, np.nan]), 'file': asset})
        run.metric('path_count', 10**100, 'count')
        run.event('saved', payload=digest)
    saved = json.loads(read_artifact(run.path, digest))
    assert saved['array']['__ndarray__'][1] == {'__float__': 'inf'}
    asset.write_bytes(b'changed')
    assert read_artifact(run.path, saved['file']['sha256']) == b'original mesh'
    assert json.loads(rows(run, 'SELECT exact_json FROM metrics')[0][0]) == 10**100
    (run.path / 'artifacts' / (digest + '.gz')).write_bytes(gzip.compress(b'corrupted'))
    with pytest.raises(ValueError, match='checksum'):
        read_artifact(run.path, digest)


def test_failure_to_record_does_not_change_result_or_original_exception(tmp_path, monkeypatch):
    @recorded
    def operation(fail=False):
        if fail:
            raise ValueError('original failure')
        return 123

    with ResearchRun(tmp_path) as run:
        def broken(*args, **kwargs):
            raise OSError('disk full')
        monkeypatch.setattr(run, 'artifact', broken)
        with pytest.warns(RuntimeWarning, match='incomplete'):
            assert operation() == 123
        with pytest.raises(ValueError, match='original failure'):
            operation(True)
    assert rows(run, 'SELECT status FROM run')[0][0] == 'incomplete'
    assert 'disk full' in (run.path/'recording_errors.jsonl').read_text()


def test_metadata_records_package_and_loaded_code_versions(tmp_path):
    from motion_toolbox import __version__
    with ResearchRun(tmp_path) as run:
        shortest_path([[[0.]], [[.1]]])
    metadata = json.loads(rows(run, 'SELECT metadata_json FROM run')[0][0])
    assert metadata['loaded_modules']['motion_toolbox']['version'] == __version__
    info = metadata['loaded_modules']['motion_toolbox.graph']
    assert len(info['loaded_code_sha256']) == 64
    assert len(info['file_sha256']) == 64
    assert rows(run, 'SELECT status FROM run')[0][0] == 'ok'


def test_thread_binding_and_nested_run_restore(tmp_path):
    @recorded
    def operation(value):
        return value * 2

    with ResearchRun(tmp_path) as run:
        with ResearchRun(tmp_path) as nested:
            assert current_run() is nested
        assert current_run() is run
        def worker(value):
            with run.activate():
                return operation(value)
        with ThreadPoolExecutor(max_workers=3) as pool:
            assert list(pool.map(worker, range(8))) == [i*2 for i in range(8)]
    assert len(rows(run, 'SELECT * FROM steps')) == 8
    assert all(row[0] is None for row in rows(run, 'SELECT parent_id FROM steps'))


def test_candidate_filter_metrics(tmp_path):
    from motion_toolbox.geometry import Plane
    from motion_toolbox.planning import candidates
    with ResearchRun(tmp_path) as run:
        result = candidates(Plane.world_xy(), Plane.world_xy(),
                            lambda target, base: [[0.], [1.]], [0.],
                            collision=lambda q, base: q[0] == 0.)
    assert result == ([[0.]], 2, 1)
    values = {r[0]: r[1] for r in rows(run, 'SELECT name,value FROM metrics')}
    assert values['raw_ik'] == 2
    assert values['collision_free'] == 1
    assert values['collision_checks'] == 2
    assert values['collision_rejections'] == 1
    assert values['collision_seconds'] >= 0


@pytest.mark.parametrize('detail', [False, True])
def test_planning_recording_omits_generated_arrays_only_in_normal_mode(tmp_path, detail):
    from motion_toolbox.geometry import Plane
    from motion_toolbox.planning import calculate_partial_trajectory
    with ResearchRun(tmp_path, detail=detail) as run:
        result = calculate_partial_trajectory(None, [Plane.world_xy()]*3,
            ik_solver=lambda t,b: [[0.], [1.]])
    steps = rows(run, 'SELECT * FROM steps')
    planner = next(s for s in steps if s['operation'].endswith('.calculate_partial_trajectory'))
    graph = next(s for s in steps if s['operation'].endswith('.shortest_path'))
    saved = json.loads(read_artifact(run.path, planner['output_artifact']))
    assert saved['configurations'] == result['configurations']
    if detail:
        graph_input = json.loads(read_artifact(run.path, graph['input_artifact']))
        assert saved['ik_solutions_per_node'] == result['ik_solutions_per_node']
        assert graph_input['layers'] == result['ik_solutions_per_node']
    else:
        assert saved['ik_solutions_per_node']['layer_counts'] == [2]*3
        assert graph['input_artifact'] is None
        assert graph['output_artifact'] is None
    assert rows(run, 'SELECT status FROM run')[0][0] == 'ok'


def test_automatic_run(tmp_path, monkeypatch):
    monkeypatch.setenv('TOOLBOX_RECORDING', '1')
    monkeypatch.setenv('TOOLBOX_LOG_DIR', str(tmp_path))
    assert shortest_path([[[0.]], [[.5]]]).cost == .5
    paths = list(tmp_path.glob('*/run.sqlite3'))
    assert len(paths) == 1
    with sqlite3.connect(paths[0]) as db:
        assert db.execute('SELECT status FROM run').fetchone() == ('ok',)


def test_default_directory_is_shared_across_working_directories(tmp_path, monkeypatch):
    import motion_toolbox.recording as recording
    central = tmp_path / 'central'
    monkeypatch.setattr(recording, 'DEFAULT_LOG_DIRECTORY', central)
    monkeypatch.delenv('TOOLBOX_LOG_DIR', raising=False)
    for folder in ('motion', 'toolpath', 'ros'):
        cwd = tmp_path / folder
        cwd.mkdir()
        monkeypatch.chdir(cwd)
        with ResearchRun() as run:
            assert run.path.parent == central
        assert not (cwd / 'research_runs').exists()
    assert len(list(central.glob('*/run.sqlite3'))) == 3


def test_suspended_benchmarks_do_not_instrument_inner_calls(tmp_path, monkeypatch):
    from motion_toolbox.recording import suspend_recording
    monkeypatch.setenv('TOOLBOX_RECORDING', '1')
    monkeypatch.setenv('TOOLBOX_LOG_DIR', str(tmp_path / 'unexpected'))
    with ResearchRun(tmp_path / 'runs') as run:
        with suspend_recording():
            assert current_run() is None
            assert shortest_path([[[0.]], [[1.]]]).cost == 1.
        assert current_run() is run
        run.metric('external.duration', .01, 's')
    assert not rows(run, 'SELECT * FROM steps')
    assert not (tmp_path / 'unexpected').exists()


def test_export_keeps_run_level_metrics_and_exact_values(tmp_path, monkeypatch):
    import csv
    import sys
    from motion_toolbox.recording import main
    with ResearchRun(tmp_path / 'runs', name='trial') as run:
        run.metric('large_count', 10**30)
        shortest_path([[[0.]], [[.1]]])
    output = tmp_path / 'metrics.csv'
    monkeypatch.setattr(sys, 'argv', ['toolbox-metrics', str(tmp_path / 'runs'), '--output', str(output)])
    main()
    with output.open(newline='') as stream:
        exported = list(csv.DictReader(stream))
    count = next(row for row in exported if row['metric'] == 'large_count')
    assert count['run_id'] == run.id
    assert count['step_id'] == ''
    assert json.loads(count['exact_json']) == 10**30
    assert any(row['operation'].endswith('shortest_path') for row in exported)


def test_interrupted_process_retains_started_step(tmp_path):
    import os
    import subprocess
    import sys
    script = '''
import os, sys
from motion_toolbox.recording import ResearchRun, recorded
@recorded
def interrupted():
    os._exit(23)
with ResearchRun(sys.argv[1]):
    interrupted()
'''
    environment = dict(os.environ)
    from pathlib import Path
    source = str(Path(__file__).resolve().parents[1] / 'src')
    environment['PYTHONPATH'] = source + os.pathsep + environment.get('PYTHONPATH', '')
    process = subprocess.run([sys.executable, '-c', script, str(tmp_path)], env=environment,
                             capture_output=True, timeout=30)
    assert process.returncode == 23, process.stderr.decode()
    with sqlite3.connect(next(tmp_path.glob('*/run.sqlite3'))) as db:
        assert db.execute('SELECT status FROM run').fetchone() == ('running',)
        assert db.execute('SELECT status,input_artifact FROM steps').fetchone()[0] == 'running'
