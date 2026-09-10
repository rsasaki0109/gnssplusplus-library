"""Verify the frozen candidate's structure and numerics, not accuracy."""
import json
import math
import re

from run_phase486_nhc_monitor import ROOT, digest
from verify_phase479_lambda_floor import check_stage


def parse_joint(log):
    patterns = (
        r'^\[native-joint-ionosphere\] states=(\d+) priors=(\d+) code=(\d+) tdcp=(\d+) anchor_sigma=(\S+) density=(\S+) max_gap=(\S+)$',
        r'^\[native-joint-ionosphere-solved\] states=(\d+) max_abs_m=(\S+) rms_m=(\S+) max_code_m=(\S+) max_tdcp_m=(\S+) clipped=0$')
    rows = [re.findall(pattern, log, re.M) for pattern in patterns]
    if any(len(row) != 1 for row in rows):
        raise ValueError('missing or duplicate joint diagnostic')
    counts = tuple(map(int, rows[0][0][:4]))
    parameters = tuple(map(float, rows[0][0][4:]))
    solved_count = int(rows[1][0][0])
    magnitudes = tuple(map(float, rows[1][0][1:]))
    if counts != (3140, 3140, 101916, 69270) or solved_count != counts[0]:
        raise ValueError('incomplete H observation/state coverage')
    if parameters != (3.0, 0.02, 1.5):
        raise ValueError('candidate parameters differ from preregistration')
    if not all(math.isfinite(x) and x >= 0 for x in magnitudes):
        raise ValueError('nonfinite or negative magnitude')
    if magnitudes[1] > magnitudes[0] + 1e-9:
        raise ValueError('RMS exceeds maximum')
    return dict(zip(('max_abs_m', 'rms_m', 'max_code_m', 'max_tdcp_m'), magnitudes))


def main():
    directory = ROOT / 'output/smartphone-r5/phase536-h-joint-ionosphere-v1/mtv-h'
    baseline_dir = ROOT / 'output/smartphone-r5/phase535-h-ionosphere-monitor-v1/mtv-h'
    manifest = json.loads((directory / 'manifest.json').read_text())
    done = json.loads((directory / 'completed.json').read_text())
    assert done['return_code'] == 0
    assert digest(baseline_dir / 'manifest.json') == manifest['previous_manifest_sha256']
    for name, sha in manifest['source_pins'].items():
        assert digest(ROOT / name) == sha, name
    for pin in manifest['inputs'].values():
        assert digest(ROOT / pin['path']) == pin['sha256']
        assert (ROOT / pin['path']).stat().st_size == pin['bytes']
    assert digest(ROOT / manifest['argv'][0]) == manifest['binary_sha256']
    assert digest(directory / 'opaque_solution_output.csv') == done['output_sha256']
    summary = json.loads((directory / 'native_summary.json').read_text())
    baseline = json.loads((baseline_dir / 'native_summary.json').read_text())
    assert summary['gnss_first'] == baseline['gnss_first']
    assert summary['epochs'] == baseline['epochs']
    assert summary['native_joint_ionosphere'] == dict(
        enabled=True, main_only=True, anchor_sigma_m=3.0,
        density_m_sqrt_s=0.02, max_gap_s=1.5)
    assert summary['graph']['converged']
    for field in ('factors', 'values'):
        assert summary['graph'][field] == baseline['graph'][field] + 3140
    for stage in ('main', 'gnss_first'):
        check_stage(summary['phase143_termination'][stage], stage)
    magnitudes = parse_joint((directory / 'stderr.log').read_text())
    print(json.dumps(dict(numerical_structure_verified=True,
                         solved_magnitudes=magnitudes,
                         physical_magnitude_review_required=True,
                         accuracy_evaluated=False, promoted=False), indent=2))


if __name__ == '__main__':
    main()
