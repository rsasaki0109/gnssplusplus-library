"""CLI admission only: no raw or truth payload paths."""
import json
from pathlib import Path

import pytest
from test_smartphone_source_tdcp_meter_sigma import invoke

FLAG = ['--native-course-attitude-seeds-mount-deg', '178.9']
MESSAGE = '--native-course-attitude-seeds-mount-deg requires Android raw Phase171 ECEF-D'
MISSING = '/nonexistent/course-attitude-seeds/'


def recipe_args():
    root = Path(__file__).resolve().parents[1]
    manifest = root / 'docs/use_cases/records/smartphone_r5_phase234_h_native_phase233_meter_sigma_manifest_v1.json'
    args = json.loads(manifest.read_text())['argv'][1:]
    for flag in ['--android-gnss', '--android-imu', '--nav', '--out', '--summary-json']:
        args[args.index(flag) + 1] = MISSING + flag[2:]
    return args


def test_rejects_epoch_heading_seeds():
    result = invoke([*recipe_args(), '--native-epoch-heading-attitude-seeds', *FLAG])
    assert result.returncode == 2
    assert MESSAGE in result.stderr


def test_admits_source_imu_initialization():
    # Course seeds replace the source heading seeds in the initial IMU pass.
    result = invoke([*recipe_args(), '--native-source-imu-initialization', *FLAG])
    assert result.returncode != 0
    assert MESSAGE not in result.stderr
    assert 'failed to open raw Android GNSS CSV' in result.stderr


def test_requires_android_raw_input():
    result = invoke(['--obs', MISSING + 'rover.obs', '--imu', MISSING + 'imu.csv',
                     '--nav', MISSING + 'brdc.nav', '--out', MISSING + 'solution.csv',
                     '--summary-json', MISSING + 'summary.json', *FLAG])
    assert result.returncode == 2
    assert MESSAGE in result.stderr


@pytest.mark.parametrize('value', ['north', 'nan', '1.0deg'])
def test_rejects_non_numeric_mount(value):
    result = invoke([*recipe_args(), '--native-course-attitude-seeds-mount-deg', value])
    assert result.returncode == 2
    assert MESSAGE not in result.stderr


def test_admitted_lane_reaches_input_loading():
    result = invoke([*recipe_args(), *FLAG])
    assert result.returncode != 0
    assert MESSAGE not in result.stderr
    assert 'failed to open raw Android GNSS CSV' in result.stderr
