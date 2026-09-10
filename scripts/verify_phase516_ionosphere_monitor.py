"""Raw replay invariance and aggregate diagnostics; no coordinate parsing."""
import json
import math
import re
from run_phase486_nhc_monitor import ROOT, digest
from verify_phase479_lambda_floor import check_stage


def parse_monitor(log):
    rows = re.findall(
        r'^\[native-code-ionosphere-information\] epochs=(\d+) invalid=(\d+) '
        r'median_fraction=(\S+) median_information_per_m2=(\S+) '
        r'factors_changed=0 weights_changed=0$', log, re.M)
    if len(rows) != 1:
        raise ValueError('missing or duplicated information monitor')
    epochs, invalid = map(int, rows[0][:2])
    fraction, information = map(float, rows[0][2:])
    if epochs <= 0 or invalid or not all(map(math.isfinite, (fraction, information))):
        raise ValueError('invalid information aggregate')
    if not 0 <= fraction <= 1 + 1e-10 or information < 0:
        raise ValueError('out-of-range information aggregate')
    return dict(epochs=epochs, median_fraction=fraction,
                median_information_per_m2=information)


def main():
    directory = ROOT / 'output/smartphone-r5/phase516-h-ionosphere-monitor-v1/mtv-h'
    manifest = json.loads((directory / 'manifest.json').read_text())
    done = json.loads((directory / 'completed.json').read_text())
    assert done['return_code'] == 0 and done['output_identical']
    for name, sha in manifest['source_pins'].items():
        assert digest(ROOT / name) == sha, name
    for pin in manifest['inputs'].values():
        assert digest(ROOT / pin['path']) == pin['sha256']
    assert digest(ROOT / manifest['argv'][0]) == manifest['binary_sha256']
    assert digest(directory / 'opaque_solution_output.csv') == manifest['expected_output_sha256']
    summary = json.loads((directory / 'native_summary.json').read_text())
    baseline = json.loads((ROOT / 'output/smartphone-r5/phase505-h-code-monitor-v1/mtv-h/native_summary.json').read_text())
    for field in ('graph', 'gnss_first'):
        assert summary[field] == baseline[field], field
    for stage in ('main', 'gnss_first'):
        check_stage(summary['phase143_termination'][stage], stage)
    monitor = parse_monitor((directory / 'stderr.log').read_text())
    assert monitor['epochs'] == summary['epochs']['problem'] == 3140
    print(json.dumps(dict(verified=True, output_identical=True, monitor=monitor,
                         truth_reads=0, accuracy_evaluations=0), indent=2))


if __name__ == '__main__':
    main()
