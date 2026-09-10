"""Runtime rejection tests for the opt-in final-graph motion; no raw data."""
import os
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / 'build/apps/gnss_fgo_imu_no_base'
FLAG = '--native-phase217-main-pose3-motion'
LANE = [
    '--native-phase165-raw-p-no-doppler-graph',
    '--native-phase167-raw-p-no-doppler-lm-termination-budget',
    '--native-phase171-raw-p-no-doppler-imu-main',
    '--native-phase171-raw-p-ecef-doppler-gnss-first',
    '--android-utc-wall-clock-fallback',
]


@pytest.mark.parametrize('args', [
    [FLAG],
    ['--dataset-id', 'synthetic/pixel4', *LANE, FLAG],
    ['--dataset-id', 'synthetic/pixel5', *LANE[:-1], FLAG],
    ['--dataset-id', 'synthetic/pixel5', *LANE, FLAG,
     '--native-phase201-source-inclusive-forward-imu-schedule'],
    ['--dataset-id', 'synthetic/pixel5', *LANE, FLAG,
     '--native-phase205-source-count-bias-density'],
    ['--dataset-id', 'synthetic/pixel5', *LANE, FLAG,
     '--native-phase209-source-separate-imu-factors'],
])
def test_reject_invalid_configuration(args):
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = '/home/sasaki/.local/lib:' + env.get('LD_LIBRARY_PATH', '')
    result = subprocess.run([str(APP), *args], cwd=ROOT, env=env,
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 2, result.stdout + result.stderr
    assert 'Phase217 requires' in result.stderr
    assert 'Unknown argument' not in result.stderr


@pytest.mark.parametrize('source_tdcp', [False, True])
def test_combined_selectors_reach_raw_recipe_validation(source_tdcp):
    # This proves selector admission only, not a successful raw-data solve.
    # No input paths are supplied, so the later Phase171 recipe guard must fail.
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = '/home/sasaki/.local/lib:' + env.get('LD_LIBRARY_PATH', '')
    result = subprocess.run(
        [str(APP), '--dataset-id', 'synthetic/pixel5', *LANE, FLAG,
         '--native-phase213-main-doppler',
         *(['--native-phase184-source-tdcp-huber-k'] if source_tdcp else [])], cwd=ROOT, env=env,
        capture_output=True, text=True, timeout=10)
    assert result.returncode == 2, result.stdout + result.stderr
    assert 'Phase171 requires the pinned raw-clock-only Android' in result.stderr
    assert 'Phase217 requires' not in result.stderr
    assert 'Phase213 requires' not in result.stderr
