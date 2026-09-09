"""Metadata and authorization gates; no real candidate or truth access."""
import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest

PATH = Path(__file__).resolve().parents[1] / 'scripts/evaluate_phase207_bias_density.py'
spec = importlib.util.spec_from_file_location('phase207_tests', PATH)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def test_verify_opens_metadata_only():
    original = Path.open
    opened = []

    def guarded(path, *args, **kwargs):
        assert path.suffix in {'.json', '.py'}
        opened.append(path)
        return original(path, *args, **kwargs)

    with patch.object(Path, 'open', guarded):
        assert runner.verify()['phase'] == 207
    assert opened


def test_changed_metric_rejected():
    real = runner.read

    def changed(path):
        value = real(path)
        if path == runner.MANIFEST:
            value['metric_contract'] = {}
        return value

    with patch.object(runner, 'read', changed), pytest.raises(ValueError, match='metric changed'):
        runner.verify()


def test_bad_authorization_never_scores():
    manifest = runner.verify()
    with patch.object(runner, 'verify', return_value=manifest), \
         patch.object(runner, 'read', return_value={}), \
         patch.object(Path, 'open', side_effect=AssertionError('unexpected open')), \
         patch.object(runner, 'static_bytes', return_value=b'fake'), \
         pytest.raises(ValueError, match='authorization mismatch'):
        runner.evaluate()
