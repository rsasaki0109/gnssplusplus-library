"""CLI admission only: no raw or truth payload paths."""
import pytest
from test_smartphone_source_tdcp_meter_sigma import invoke

FLAG = '--android-hardware-clock-reset-clock-jumps'
MESSAGE = '--android-hardware-clock-reset-clock-jumps requires Android raw input'
MISSING = '/nonexistent/hardware-clock-reset/'
OUTPUTS = ['--nav', MISSING + 'brdc.nav', '--out', MISSING + 'solution.csv',
           '--summary-json', MISSING + 'summary.json']
ANDROID = ['--android-gnss', MISSING + 'device_gnss.csv',
           '--android-imu', MISSING + 'device_imu.csv', *OUTPUTS]


@pytest.mark.parametrize('args', [
    ['--obs', MISSING + 'rover.obs', '--imu', MISSING + 'imu.csv', *OUTPUTS, FLAG],
    [*ANDROID, FLAG],
])
def test_requires_raw_clock_only_android_input(args):
    result = invoke(args)
    assert result.returncode == 2
    assert MESSAGE in result.stderr


def test_admitted_lane_reaches_input_loading():
    result = invoke([*ANDROID, '--android-raw-clock-only', FLAG])
    assert result.returncode != 0
    assert MESSAGE not in result.stderr
