"""CLI admission only: no raw, road-map or truth payloads."""
import pytest
from test_smartphone_source_tdcp_meter_sigma import invoke, LANE

FLAG = ['--native-road-map', '/nonexistent/road-map.csv']
MESSAGE = '--native-road-map requires Phase171 ECEF-D and excludes Phase135'


@pytest.mark.parametrize('args', [
    [*FLAG],
    ['--dataset-id', 'synthetic/pixel5', *[flag for flag in LANE if 'ecef-doppler' not in flag], *FLAG],
    ['--dataset-id', 'synthetic/pixel5', *LANE, *FLAG,
     '--native-phase135-official-affine-measurement-family'],
])
def test_invalid_lane(args):
    result = invoke(args)
    assert result.returncode == 2
    assert MESSAGE in result.stderr


@pytest.mark.parametrize('value', ['0', '-0.5', 'wide'])
def test_rejects_a_non_positive_sigma(value):
    result = invoke(['--dataset-id', 'synthetic/pixel5', *LANE, *FLAG, '--native-road-map-sigma-m', value])
    assert result.returncode == 2
    assert MESSAGE not in result.stderr


def test_valid_lane_reaches_missing_raw_guard():
    result = invoke(['--dataset-id', 'synthetic/pixel5', *LANE, *FLAG, '--native-road-map-sigma-m', '0.5'])
    assert result.returncode == 2
    assert MESSAGE not in result.stderr
    assert 'Phase171 requires the pinned raw-clock-only Android' in result.stderr
