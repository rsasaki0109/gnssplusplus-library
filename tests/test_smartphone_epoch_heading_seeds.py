"""CLI admission only: no raw or truth payload paths."""
import pytest
import json
from pathlib import Path
from test_smartphone_source_tdcp_meter_sigma import invoke, LANE

FLAG = '--native-epoch-heading-attitude-seeds'


@pytest.mark.parametrize('args', [
    [FLAG],
    ['--dataset-id', 'synthetic/pixel4', *LANE, FLAG],
    ['--dataset-id', 'synthetic/pixel5', *LANE[:-1], FLAG],
    ['--dataset-id', 'synthetic/pixel5', *LANE, FLAG,
     '--native-phase135-official-affine-measurement-family'],
])
def test_invalid_lane(args):
    result = invoke(args)
    assert result.returncode == 2
    assert 'Epoch heading seeds require' in result.stderr


def test_valid_lane_reaches_missing_raw_guard():
    result = invoke(['--dataset-id', 'synthetic/pixel5', *LANE, FLAG,
                     '--native-source-tdcp-meter-sigma',
                     '--native-phase213-main-doppler',
                     '--native-phase217-main-pose3-motion'])
    assert result.returncode == 2
    assert 'Phase171 requires the pinned raw-clock-only Android' in result.stderr


@pytest.mark.parametrize('all_epochs', [True, False])
def test_sparse_staging_heading_admission(all_epochs):
    root = Path(__file__).resolve().parents[1]
    manifest = root / 'docs/use_cases/records/smartphone_r5_phase234_h_native_phase233_meter_sigma_manifest_v1.json'
    args = json.loads(manifest.read_text())['argv'][1:]
    for flag in ['--android-gnss', '--android-imu', '--nav', '--out', '--summary-json']:
        args[args.index(flag) + 1] = '/nonexistent/heading-sparse-admission/' + flag[2:]
    args += ['--native-sparse-p-staging', FLAG]
    if not all_epochs:
        args.remove('--all-epochs')
    result = invoke(args)
    assert result.returncode != 0
    if all_epochs:
        assert 'failed to open raw Android GNSS CSV' in result.stderr
        assert 'native-sparse-p-staging requires' not in result.stderr
    else:
        assert result.returncode == 2
        assert 'Phase171 requires the pinned raw-clock-only Android' in result.stderr
        assert 'failed to open raw Android GNSS CSV' not in result.stderr
