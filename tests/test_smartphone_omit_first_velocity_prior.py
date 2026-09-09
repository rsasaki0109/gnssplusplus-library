"""CLI admission only; no raw or truth inputs."""
import pytest
from test_smartphone_source_tdcp_meter_sigma import invoke, LANE

FLAG = '--native-omit-first-imu-velocity-prior'
D = '--native-phase213-main-doppler'


@pytest.mark.parametrize('args', [
    [FLAG],
    ['--dataset-id', 'synthetic/pixel5', *LANE, FLAG],
    ['--dataset-id', 'synthetic/pixel4', *LANE, D, FLAG],
    ['--dataset-id', 'synthetic/pixel5', *LANE[:-1], D, FLAG],
    ['--dataset-id', 'synthetic/pixel5', *LANE, D, FLAG,
     '--native-phase135-official-affine-measurement-family'],
])
def test_invalid_lane(args):
    result = invoke(args)
    assert result.returncode == 2
    assert 'Omitting first velocity prior requires' in result.stderr


def test_valid_lane_reaches_missing_raw_guard():
    result = invoke(['--dataset-id', 'synthetic/pixel5', *LANE, D, FLAG,
                     '--native-source-tdcp-meter-sigma',
                     '--native-phase217-main-pose3-motion'])
    assert result.returncode == 2
    assert 'Phase171 requires the pinned raw-clock-only Android' in result.stderr
