"""Structural verification only; no position rows or truth access."""
import json
from run_phase486_nhc_monitor import ROOT, digest
from verify_phase479_lambda_floor import check_stage

def main():
    directory = ROOT / 'output/smartphone-r5/phase488-h-batch-nhc-v1/mtv-h'
    manifest = json.loads((directory / 'manifest.json').read_text())
    completed = json.loads((directory / 'completed.json').read_text())
    assert completed['return_code'] == 0
    assert digest(ROOT / manifest['argv'][0]) == manifest['binary_sha256']
    for name, sha in manifest['source_pins'].items():
        assert digest(ROOT / name) == sha, name
    assert digest(directory / 'opaque_solution_output.csv') == completed['output_sha256']
    summary = json.loads((directory / 'native_summary.json').read_text())
    baseline = json.loads((ROOT / 'output/smartphone-r5/phase479-h-robust-diagnostic-v1/mtv-h/native_summary.json').read_text())
    assert summary['gnss_first'] == baseline['gnss_first']
    assert summary['graph']['converged']
    assert summary['graph']['factors'] == baseline['graph']['factors'] + 1627
    for key, value in baseline['graph'].items():
        if key not in ('factors', 'iterations', 'initial_cost', 'final_cost'):
            assert summary['graph'][key] == value, key
    for stage in ('main', 'gnss_first'):
        check_stage(summary['phase143_termination'][stage], stage)
    assert completed['monitor'] == [
        '[native-nhc-monitor] intervals=3139 supported=3138 admitted=1627 '
        'min_speed_mps=2 max_angular_speed_radps=0.2 max_gap_s=0.05 factors_added=1627']
    print(json.dumps(dict(structural_verified=True, factors_added=1627,
                         gnss_first_unchanged=True,
                         output_sha256=completed['output_sha256'],
                         main_iterations=summary['graph']['iterations'],
                         truth_reads=0, accuracy_evaluations=0), indent=2))

if __name__ == '__main__':
    main()
