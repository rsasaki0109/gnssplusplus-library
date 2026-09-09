"""Metadata and authorization gates; no real candidate or truth access."""
import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest

PATH = Path(__file__).resolve().parents[1] / 'scripts/evaluate_phase255_affine_geometry.py'
spec = importlib.util.spec_from_file_location('phase255_tests', PATH)
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
        assert runner.verify()['phase'] == 255
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


@pytest.mark.parametrize("field", ["tdcp_only_affine_factors_inserted",
                                  "gnss_first_tdcp_only_affine_factors_inserted"])
def test_missing_affine_insertion_rejected(field):
    real = runner.read

    def changed(path):
        value = real(path)
        if path.name == "smartphone_r5_phase254_h_native_phase253_affine_geometry_result_v1.json":
            value["tdcp_contract"][field] = 0
        return value

    with patch.object(runner, "read", changed), pytest.raises(ValueError, match="affine insertion coverage"):
        runner.verify()


@pytest.mark.parametrize('field', ['interpolated_epochs', 'edge_hold_epochs',
                                  'unresolved_epochs'])
def test_nonexact_output_rejected(field):
    real = runner.read

    def changed(path):
        value = real(path)
        if path.name == 'smartphone_r5_phase254_h_native_phase253_affine_geometry_result_v1.json':
            value['raw_utc_key_contract'][field] = 1
        return value

    with patch.object(runner, 'read', changed), pytest.raises(ValueError, match='non-exact output'):
        runner.verify()
