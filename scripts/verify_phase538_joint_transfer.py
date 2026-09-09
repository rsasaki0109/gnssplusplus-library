"""Post-run provenance and finite-state checks; no coordinate/truth parsing."""
import json
import math
import re
import sys
from run_phase486_nhc_monitor import ROOT, digest
from verify_phase479_lambda_floor import check_stage


def main(route):
    if route not in ('u', 'lax'):
        raise ValueError('unknown route')
    directory = ROOT / f'output/smartphone-r5/phase538-joint-transfer-v1/{route}'
    manifest = json.loads((directory / 'manifest.json').read_text())
    done = json.loads((directory / 'completed.json').read_text())
    assert done['return_code'] == 0
    for field in ('graph_increments_valid', 'epochs_identical', 'gnss_first_identical', 'converged'):
        assert done[field], field
    for name, sha in manifest['source_pins'].items():
        assert digest(ROOT / name) == sha, name
    assert digest(ROOT / 'scripts/run_phase538_joint_transfer.py') == manifest['launcher_sha256']
    assert digest(ROOT / manifest['argv'][0]) == manifest['binary_sha256']
    for item in manifest['inputs'].values():
        path = ROOT / item['path']
        assert digest(path) == item['sha256'] and path.stat().st_size == item['bytes']
    assert digest(directory / 'opaque_solution_output.csv') == done['output_sha256']
    summary = json.loads((directory / 'native_summary.json').read_text())
    log = (directory / 'stderr.log').read_text()
    rows = re.findall(r'^\[native-joint-ionosphere-solved\] states=(\d+) max_abs_m=(\S+) rms_m=(\S+) max_code_m=(\S+) max_tdcp_m=(\S+) clipped=0$', log, re.M)
    assert len(rows) == 1 and int(rows[0][0]) == summary['epochs']['problem']
    magnitudes = list(map(float, rows[0][1:]))
    assert all(math.isfinite(x) and x >= 0 for x in magnitudes)
    assert magnitudes[1] <= magnitudes[0] + 1e-9
    for stage in ('main', 'gnss_first'):
        check_stage(summary['phase143_termination'][stage], stage)
    print(json.dumps(dict(route=route, provenance_and_finite_verified=True,
        magnitudes=dict(zip(('max_abs_m', 'rms_m', 'max_code_m', 'max_tdcp_m'), magnitudes)),
        iterations=summary['graph']['iterations'], elapsed_seconds=done['elapsed_seconds'],
        accuracy_evaluated=False, physical_review_required=True), indent=2))


if __name__ == '__main__':
    main(sys.argv[1])
