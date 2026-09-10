"""TDCP-only affine CLI admission, without raw payload access."""
import pytest
from test_smartphone_source_tdcp_meter_sigma import invoke, LANE

FLAG = '--native-tdcp-only-affine-geometry'


@pytest.mark.parametrize('args', [
    [FLAG],
    ['--dataset-id', 'synthetic/pixel4', *LANE, FLAG],
    ['--dataset-id', 'synthetic/pixel5', *LANE[:-1], FLAG],
    ['--dataset-id', 'synthetic/pixel5', *LANE, FLAG,
     '--native-phase135-official-affine-measurement-family'],
    ['--dataset-id', 'synthetic/pixel5', *LANE, FLAG,
     '--native-phase138-affine-tdcp-anchor-range-constant'],
])
def test_invalid_lane(args):
    result = invoke(args)
    assert result.returncode == 2
    assert 'TDCP-only affine geometry requires' in result.stderr


def test_combined_lane_reaches_raw_recipe_guard():
    result = invoke(['--dataset-id', 'synthetic/pixel5', *LANE, FLAG,
                     '--native-source-tdcp-meter-sigma',
                     '--native-phase213-main-doppler',
                     '--native-phase217-main-pose3-motion'])
    assert result.returncode == 2
    assert 'Phase171 requires the pinned raw-clock-only Android' in result.stderr
    assert 'TDCP-only affine geometry requires' not in result.stderr
