import hashlib
import importlib.util
import json
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location('replay_mobile_case',
    Path(__file__).resolve().parents[1]/'validation/replay_mobile_case.py')
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


def capture(tmp_path):
    (tmp_path/'robot').mkdir()
    (tmp_path/'robot/robot.urdf').write_text('<robot/>')
    (tmp_path/'case.json').write_text(json.dumps(dict(schema='motion-mobile-case/1',
                                                    replay=dict(units='metres/radians'))))
    manifest = {n:hashlib.sha256((tmp_path/n).read_bytes()).hexdigest()
                for n in ('case.json','robot/robot.urdf')}
    (tmp_path/'manifest.json').write_text(json.dumps(manifest))
    (tmp_path/'READY').write_text('ready')


def test_capture_requires_ready_and_verified_robot(tmp_path):
    capture(tmp_path)
    assert replay.load_case(tmp_path)['schema'] == 'motion-mobile-case/1'
    (tmp_path/'robot/robot.urdf').write_text('changed geometry')
    with pytest.raises(ValueError,match='integrity failure'):
        replay.load_case(tmp_path)
    (tmp_path/'READY').unlink()
    with pytest.raises(ValueError,match='not READY'):
        replay.load_case(tmp_path)


def test_incomplete_manifest_rejected(tmp_path):
    capture(tmp_path)
    (tmp_path/'manifest.json').write_text('{}')
    with pytest.raises(ValueError,match='Manifest must include'):
        replay.load_case(tmp_path)
