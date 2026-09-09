"""CLI admission only: no raw, saved positioning, MAT, or truth payloads."""
import pytest
from test_smartphone_source_tdcp_meter_sigma import invoke, LANE

FLAG = '--native-relative-height-pairs'
STOP = '--native-upstream-stop-constraints'


@pytest.mark.parametrize('args', [
    [FLAG],
    ['--dataset-id', 'synthetic/pixel5', *LANE, FLAG],
    ['--dataset-id', 'synthetic/pixel4', *LANE, STOP, FLAG],
    ['--dataset-id', 'synthetic/pixel5', *LANE[:-1], STOP, FLAG],
    ['--dataset-id', 'synthetic/pixel5', *LANE, STOP, FLAG,
     '--native-phase135-official-affine-measurement-family'],
])
def test_invalid_lane(args):
    result = invoke(args)
    assert result.returncode == 2
    assert 'Relative height requires' in result.stderr


def test_valid_lane_reaches_missing_raw_guard():
    result = invoke(['--dataset-id', 'synthetic/pixel5', *LANE, STOP, FLAG])
    assert result.returncode == 2
    assert 'Phase171 requires the pinned raw-clock-only Android' in result.stderr
