"""Verify the numerical experiment without parsing positions or reading truth."""
import json
import math
from pathlib import Path
from verify_native_imu_bias_diagnostic import digest, parse_failed_lambda

ROOT = Path(__file__).resolve().parents[1]


def check_stage(stage, name):
    expected = {'main': ('MULTIFRONTAL_QR', 'EliminateQR'),
                'gnss_first': ('MULTIFRONTAL_CHOLESKY', 'EliminatePreferCholesky')}[name]
    assert stage['lambda_lower_bound'] == 1e-8
    assert stage['initial_lambda'] == 1e-5
    assert stage['linear_solver'] == expected[0]
    assert stage['elimination'] == expected[1]
    assert not stage['diagonal_damping']
    assert stage['costs_finite'] and stage['termination_trace_complete']
    assert math.isfinite(stage['final_cost']) and stage['final_cost'] >= 0


def main():
    path = ROOT / 'docs/use_cases/records/smartphone_r5_phase479_h_baseline_replay_manifest_v1.json'
    manifest = json.loads(path.read_text())
    directory = (ROOT / manifest['launcher_contract']['execution_metadata_path']).parent
    completed = json.loads((directory / 'launcher_completed.json').read_text())
    assert completed['return_code'] == 0 and completed['native_invocations'] == 1
    assert completed['manifest_sha256'] == digest(path)
    for key in ('source_pins', 'test_pins', 'algorithm_source_pins'):
        for name, sha in manifest[key].items():
            assert digest(ROOT / name) == sha, name
    for item in (manifest['binary'], *manifest['inputs'].values()):
        source = ROOT / item['path']
        assert source.stat().st_size == item['bytes'] and digest(source) == item['sha256']
    assert '--native-lm-lambda-floor' in manifest['argv']
    summary = json.loads((directory / 'native_summary.json').read_text())
    assert summary['graph']['converged'] and summary['gnss_first']['converged']
    for stage in ('main', 'gnss_first'):
        check_stage(summary['phase143_termination'][stage], stage)
    ranges = parse_failed_lambda((directory / 'native.stderr.log').read_text())
    failures = summary['native_source_clock_c0d_factor']['indeterminate_linear_solve_count']
    assert type(failures) is int and failures >= 0
    if failures:
        assert ranges and ranges[-1]['count'] == failures
    for entry in ranges:
        assert entry['min'] >= 1e-8 * (1-1e-12)
    candidate_sha = digest(directory / 'opaque_solution_output.csv')
    print(json.dumps(dict(
        baseline_output_identical=candidate_sha == manifest['expected_candidate_sha256'],
        candidate_sha256=candidate_sha, main_linear_failures=failures,
        failed_lambda_ranges=ranges, elapsed_seconds=completed['elapsed_seconds'],
        stage_results={stage: {k: summary['phase143_termination'][stage][k] for k in
            ('accepted_outer_iterations', 'total_inner_lambda_attempts', 'final_cost',
             'lambda_lower_bound', 'final_lambda')} for stage in ('main', 'gnss_first')},
        truth_reads=0, accuracy_evaluations=0, promoted=False), indent=2))


if __name__ == '__main__':
    main()
