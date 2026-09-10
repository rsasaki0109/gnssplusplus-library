"""Verify main-only weighting isolation without truth or coordinate parsing."""
import json
from run_phase486_nhc_monitor import ROOT, digest
from verify_phase479_lambda_floor import check_stage

def main():
    directory = ROOT / 'output/smartphone-r5/phase495-h-main-code-floor-v1/mtv-h'
    manifest = json.loads((directory / 'manifest.json').read_text())
    done = json.loads((directory / 'completed.json').read_text())
    assert done['return_code'] == 0
    for name, sha in manifest['source_pins'].items():
        assert digest(ROOT / name) == sha, name
    assert digest(ROOT / manifest['argv'][0]) == manifest['binary_sha256']
    assert digest(directory / 'opaque_solution_output.csv') == done['output_sha256']
    s = json.loads((directory / 'native_summary.json').read_text())
    old = json.loads((ROOT / 'output/smartphone-r5/phase493-h-code-monitor-v1/mtv-h/native_summary.json').read_text())
    assert s['gnss_first'] == old['gnss_first']
    assert s['graph']['converged']
    for key, value in old['graph'].items():
        if key not in ('iterations', 'initial_cost', 'final_cost'):
            assert s['graph'][key] == value, key
    for stage in ('main', 'gnss_first'):
        check_stage(s['phase143_termination'][stage], stage)
    assert done['monitor'] == ['[native-main-code-uncertainty-floor] retained=101916 changed=101916 gnss_first_changed=0 selection_changed=0']
    print(json.dumps(dict(structural_verified=True, changed_sigmas=101916,
        gnss_first_unchanged=True, graph_counts_unchanged=True,
        output_sha256=done['output_sha256'], iterations=s['graph']['iterations'],
        truth_reads=0, accuracy_evaluations=0), indent=2))

if __name__ == '__main__':
    main()
