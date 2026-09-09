"""Output-policy CLI admission; no raw or truth payloads."""
import pytest

from test_smartphone_source_tdcp_meter_sigma import invoke, LANE

FLAG = '--android-include-first-native-epoch'


@pytest.mark.parametrize('args', [
    [FLAG],
    [*LANE, '--all-epochs', FLAG],
    [*LANE, '--android-raw-utc-keys', FLAG],
    [*LANE, '--android-raw-utc-keys', '--all-epochs', '--skip-epochs', '1', FLAG],
])
def test_incomplete_output_lane_rejected(args):
    result = invoke(args)
    assert result.returncode == 2
    assert 'Including first native epoch requires' in result.stderr


def test_valid_output_lane_reaches_raw_input_guard():
    result = invoke(['--dataset-id', 'synthetic/pixel5', *LANE,
                     '--android-raw-utc-keys', '--all-epochs', FLAG])
    assert result.returncode == 2
    assert 'Including first native epoch requires' not in result.stderr
    assert 'Unknown argument' not in result.stderr
    assert 'requires' in result.stderr
