"""Verify diagnostic-only replay without truth or coordinate-row access."""
import json
from run_phase486_nhc_monitor import ROOT, digest
from verify_phase479_lambda_floor import check_stage

def main():
    directory = ROOT / 'output/smartphone-r5/phase503-h-code-monitor-v1/mtv-h'
    manifest = json.loads((directory / 'manifest.json').read_text())
    done = json.loads((directory / 'completed.json').read_text())
    assert done['return_code'] == 0 and done['output_identical']
    assert digest(directory / 'opaque_solution_output.csv') == manifest['expected_output_sha256']
    assert digest(ROOT / manifest['argv'][0]) == manifest['binary_sha256']
    for name, sha in manifest['source_pins'].items():
        assert digest(ROOT / name) == sha, name
    summary = json.loads((directory / 'native_summary.json').read_text())
    baseline = json.loads((ROOT / 'output/smartphone-r5/phase493-h-code-monitor-v1/mtv-h/native_summary.json').read_text())
    for field in ('graph', 'gnss_first'):
        assert summary[field] == baseline[field], field
    for stage in ('main', 'gnss_first'):
        check_stage(summary['phase143_termination'][stage], stage)
    assert done['monitor'] == ['[native-masked-code-shadow] geometry_quality_pass=2087 baseline_residual_pass=807 residual_unavailable=0 factors_added=0 median_rows_added=0']
    print(json.dumps(dict(verified=True, output_identical=True,
                         graph_and_handoff_aggregates_identical=True,
                         shadow_geometry_quality_pass=2087, shadow_residual_pass=807,
                         truth_reads=0, accuracy_evaluations=0), indent=2))

if __name__ == '__main__':
    main()
