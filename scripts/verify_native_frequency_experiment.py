"""Post-inference aggregate checks for Phase416/417; never open ground truth."""
import hashlib
import csv
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def verify(phase):
    tag = {416: 'h_baseline_replay', 417: 'lax_t_comparison',
           422: 'h_baseline_replay', 423: 'lax_t_comparison'}[phase]
    path = ROOT / f'docs/use_cases/records/smartphone_r5_phase{phase}_{tag}_manifest_v1.json'
    manifest = json.loads(path.read_text())
    directory = (ROOT / manifest['launcher_contract']['execution_metadata_path']).parent
    completed = json.loads((directory / 'launcher_completed.json').read_text())
    assert completed['return_code'] == 0 and completed['native_invocations'] == 1
    assert completed['manifest_sha256'] == digest(path)
    for group in ('source_pins', 'test_pins', 'algorithm_source_pins'):
        for name, expected in manifest[group].items():
            assert digest(ROOT / name) == expected, name
    for item in (manifest['binary'], *manifest['inputs'].values()):
        source = ROOT / item['path']
        assert source.stat().st_size == item['bytes'] and digest(source) == item['sha256']
    summary = json.loads((directory / 'native_summary.json').read_text())
    assert summary['graph']['converged'] and summary['gnss_first']['converged']
    tdcp = summary['tdcp_contract']
    states = tdcp['frequency_residual_state_count']
    wrapped = tdcp['frequency_residual_factor_count']
    n = tdcp['finite_residuals']
    assert states > 0 and wrapped == 2 * states and wrapped <= n
    assert n == tdcp['factors_built'] == tdcp['factors_inserted'] == {
        416: 69270, 417: 24964, 422: 69270, 423: 24964}[phase]
    assert tdcp['nonfinite_residuals'] == tdcp['reconstructed_cost_nonfinite_count'] == 0
    tail = tdcp['reconstructed_huber_tail_count']
    robust = tdcp['reconstructed_huber_cost_sum']
    quadratic = tdcp['reconstructed_quadratic_cost_sum']
    assert 0 <= tail <= n and math.isfinite(robust) and math.isfinite(quadratic)
    assert 0 <= robust <= quadratic
    assert math.isclose(quadratic, 0.5*n*tdcp['normalized_residual_rms']**2, rel_tol=1e-12)
    groups = tdcp['reconstructed_signal_aggregates']
    assert sum(g['count'] for g in groups) == n
    assert sum(g['tail_count'] for g in groups) == tail
    assert math.isclose(sum(g['huber_cost_sum'] for g in groups), robust, rel_tol=1e-12)
    assert math.isclose(sum(g['residual_squared_sum_m2'] for g in groups),
                        n*tdcp['residual_rms_m']**2, rel_tol=1e-12)
    candidate = directory / 'opaque_solution_output.csv'
    # Parse identifiers only, never coordinates; post-inference validation only.
    with candidate.open(newline='') as stream:
        output_keys = [(row['phone'], int(row['UnixTimeMillis']))
                       for row in csv.DictReader(stream)]
    raw_path = ROOT / manifest['inputs']['android_gnss']['path']
    with raw_path.open(newline='') as stream:
        raw_times = sorted({int(row['utcTimeMillis']) for row in csv.DictReader(stream)
                            if row['MessageType'] == 'Raw'})
    if '--android-include-first-native-epoch' not in manifest['argv']:
        raw_times = raw_times[1:]
    assert output_keys == [(manifest['dataset_id'], t) for t in raw_times]
    rows = len(output_keys)
    assert rows == manifest['expected_structural_contract']['expected_output_rows']
    if phase in (422, 423):
        assert tdcp['frequency_residual_prior_count'] == states
        assert math.isfinite(tdcp['backend_residual_rms_m'])
        assert abs(tdcp['backend_residual_rms_m'] - tdcp['residual_rms_m']) <= 1e-7
        assert digest(candidate) == manifest['expected_diagnostic_replay_sha256']
    return dict(phase=phase, states=states, wrapped_factors=wrapped,
                tdcp_factors=n, output_rows=rows, candidate_sha256=digest(candidate),
                residual_rms_m=tdcp['residual_rms_m'], huber_cost=robust,
                native_elapsed_seconds=completed['elapsed_seconds'], truth_reads=0,
                exact_raw_output_keys=True,
                scope=('aggregate, raw keys, prior count, backend RMS and replay identity'
                       if phase in (422, 423) else
                       'aggregate and raw key checks; backend RMS still requires verification'))


if __name__ == '__main__':
    print(json.dumps(verify(int(sys.argv[1])), indent=2))
