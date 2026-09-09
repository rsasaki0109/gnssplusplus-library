"""Verify same-row candidate intersection without scoring or coordinate parsing."""
import json
import re
from run_phase486_nhc_monitor import ROOT, digest
from verify_phase479_lambda_floor import check_stage

def parse_intersection(log):
    raw = re.findall(r'^\[native-code-edge-candidates\] raw_candidates=(\d+) factors_added=0$', log, re.M)
    shadow = re.findall(r'^\[native-code-edge-shadow\] geometry_quality_pass=(\d+) residual_pass=(\d+) retained_epoch_pass=(\d+) factors_added=0$', log, re.M)
    if len(raw) != 1 or len(shadow) != 1:
        raise ValueError('missing or duplicated candidate stage diagnostic')
    count = int(raw[0])
    quality, residual, retained = map(int, shadow[0])
    if not 0 <= retained <= residual <= quality <= count:
        raise ValueError('candidate counts are not nested')
    return dict(raw_candidates=count, quality=quality, residual=residual, retained=retained)

def main():
    directory = ROOT / 'output/smartphone-r5/phase505-h-code-monitor-v1/mtv-h'
    m = json.loads((directory / 'manifest.json').read_text())
    done = json.loads((directory / 'completed.json').read_text())
    assert done['return_code'] == 0 and done['output_identical']
    for name, sha in m['source_pins'].items():
        assert digest(ROOT / name) == sha, name
    assert digest(ROOT / m['argv'][0]) == m['binary_sha256']
    assert digest(directory / 'opaque_solution_output.csv') == m['expected_output_sha256']
    s = json.loads((directory / 'native_summary.json').read_text())
    baseline = json.loads((ROOT / 'output/smartphone-r5/phase503-h-code-monitor-v1/mtv-h/native_summary.json').read_text())
    for field in ('graph','gnss_first'):
        assert s[field] == baseline[field], field
    for stage in ('main','gnss_first'):
        check_stage(s['phase143_termination'][stage],stage)
    counts = parse_intersection((directory / 'stderr.log').read_text())
    # The standalone raw candidate audit used the same raw H file.
    assert counts['raw_candidates'] == 426
    print(json.dumps(dict(verified=True, output_identical=True, counts=counts,
                         truth_reads=0, accuracy_evaluations=0), indent=2))

if __name__ == '__main__':
    main()
