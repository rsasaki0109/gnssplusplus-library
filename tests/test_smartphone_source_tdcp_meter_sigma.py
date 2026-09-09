"""CLI admission tests only; never provide raw input paths."""
import os
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
FLAG = '--native-source-tdcp-meter-sigma'
LANE = ['--native-phase165-raw-p-no-doppler-graph',
        '--native-phase167-raw-p-no-doppler-lm-termination-budget',
        '--native-phase171-raw-p-no-doppler-imu-main',
        '--native-phase171-raw-p-ecef-doppler-gnss-first',
        '--android-utc-wall-clock-fallback']


def invoke(args):
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = '/home/sasaki/.local/lib:' + env.get('LD_LIBRARY_PATH', '')
    return subprocess.run([str(ROOT / 'build/apps/gnss_fgo_imu_no_base'), *args],
                          cwd=ROOT, env=env, capture_output=True, text=True, timeout=10)


@pytest.mark.parametrize('args', [
    [FLAG],
    ['--dataset-id', 'synthetic/pixel4', *LANE, FLAG],
    ['--dataset-id', 'synthetic/pixel5', *LANE[:-1], FLAG],
])
def test_invalid_lane(args):
    result = invoke(args)
    assert result.returncode == 2
    assert 'Source TDCP metre sigma requires' in result.stderr


@pytest.mark.parametrize('conflict', [
    '--native-phase117-tdcp-snr-type-sigma',
    '--native-phase118-official-tdcp-huber-k',
    '--native-phase120-official-tdcp-resl-atmosphere-cancellation',
])
def test_conflicting_noise_modes(conflict):
    result = invoke(['--dataset-id', 'synthetic/pixel5', *LANE, FLAG, conflict])
    assert result.returncode == 2
    assert 'Source TDCP metre sigma requires' in result.stderr
    assert 'Unknown argument' not in result.stderr


@pytest.mark.parametrize('source_k', [False, True])
def test_combined_lane_reaches_missing_raw_recipe(source_k):
    result = invoke(['--dataset-id', 'synthetic/pixel5', *LANE, FLAG,
                     '--native-phase213-main-doppler', '--native-phase217-main-pose3-motion',
                     *(['--native-phase184-source-tdcp-huber-k'] if source_k else [])])
    assert result.returncode == 2
    assert 'Phase171 requires the pinned raw-clock-only Android' in result.stderr
    assert 'Source TDCP metre sigma requires' not in result.stderr
