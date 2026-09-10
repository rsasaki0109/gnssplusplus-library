"""Metadata and authorization gates; no real candidate or truth access."""
import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest

PATH = Path(__file__).resolve().parents[1] / 'scripts/evaluate_phase261_epoch_heading.py'
spec = importlib.util.spec_from_file_location('phase261_tests', PATH)
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
        assert runner.verify()['phase'] == 261
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
def test_unexpected_affine_insertion_rejected(field):
    real = runner.read

    def changed(path):
        value = real(path)
        if path.name == "smartphone_r5_phase260_h_native_phase259_epoch_heading_result_v1.json":
            value["tdcp_contract"][field] = 1
        return value

    with patch.object(runner, "read", changed), pytest.raises(ValueError, match="affine insertion coverage"):
        runner.verify()


@pytest.mark.parametrize('field', ['interpolated_epochs', 'edge_hold_epochs',
                                  'unresolved_epochs'])
def test_nonexact_output_rejected(field):
    real = runner.read

    def changed(path):
        value = real(path)
        if path.name == 'smartphone_r5_phase260_h_native_phase259_epoch_heading_result_v1.json':
            value['raw_utc_key_contract'][field] = 1
        return value

    with patch.object(runner, 'read', changed), pytest.raises(ValueError, match='non-exact output'):
        runner.verify()


def test_missing_heading_coverage_rejected():
    real = runner.read

    def changed(path):
        value = real(path)
        if path.name == 'smartphone_r5_phase260_h_native_phase259_epoch_heading_result_v1.json':
            value['tdcp_contract']['epoch_heading_attitude_seeds_inserted'] = 3139
        return value

    with patch.object(runner, 'read', changed), pytest.raises(ValueError, match='heading insertion coverage'):
        runner.verify()


def test_changed_gnss_first_rejected():
    real = runner.read

    def changed(path):
        value = real(path)
        if path.name == 'smartphone_r5_phase260_h_native_phase259_epoch_heading_result_v1.json':
            value['comparison_vs_phase234']['gnss_first'] = False
        return value

    with patch.object(runner, 'read', changed), pytest.raises(ValueError, match='baseline composition'):
        runner.verify()
