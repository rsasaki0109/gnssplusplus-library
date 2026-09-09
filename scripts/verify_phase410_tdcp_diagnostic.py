"""Verify the frozen raw replay and report aggregates; never read truth."""
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / 'docs/use_cases/records/smartphone_r5_phase410_h_baseline_replay_manifest_v1.json'


def verify():
    payload = MANIFEST.read_bytes()
    manifest = json.loads(payload)
    directory = (ROOT / manifest['launcher_contract']['execution_metadata_path']).parent
    completed = json.loads((directory / 'launcher_completed.json').read_text())
    assert completed['return_code'] == 0
    assert completed['native_invocations'] == 1
    assert completed['manifest_sha256'] == hashlib.sha256(payload).hexdigest()
    for group in ['source_pins', 'test_pins', 'algorithm_source_pins']:
        for name, expected in manifest[group].items():
            assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == expected, name
    candidate = directory / 'opaque_solution_output.csv'
    # Opaque hashing only: no coordinates are parsed or fed into a solver.
    actual = hashlib.sha256(candidate.read_bytes()).hexdigest()
    assert actual == manifest['expected_output_identity']['sha256'], 'replay changed output'
    summary = json.loads((directory / 'native_summary.json').read_text())
    assert summary['graph']['converged'] and summary['gnss_first']['converged']
    tdcp = summary['tdcp_contract']
    assert tdcp['frequency_residual_state_count'] == 0
    assert tdcp['frequency_residual_factor_count'] == 0
    n = tdcp['finite_residuals']
    assert n == tdcp['factors_built'] == tdcp['factors_inserted'] == 69270
    assert tdcp['nonfinite_residuals'] == tdcp['reconstructed_cost_nonfinite_count'] == 0
    tail = tdcp['reconstructed_huber_tail_count']
    robust = tdcp['reconstructed_huber_cost_sum']
    quadratic = tdcp['reconstructed_quadratic_cost_sum']
    assert 0 <= tail <= n and math.isfinite(robust) and math.isfinite(quadratic)
    assert 0 <= robust <= quadratic
    assert math.isclose(quadratic, 0.5 * n * tdcp['normalized_residual_rms']**2,
                        rel_tol=1e-12)
    assert summary['upstream_observable_quality']['tdcp_sigma_m_unchanged'] is None
    groups = tdcp['reconstructed_signal_aggregates']
    assert sum(g['count'] for g in groups) == n
    assert sum(g['tail_count'] for g in groups) == tail
    assert math.isclose(sum(g['huber_cost_sum'] for g in groups), robust, rel_tol=1e-12)
    assert math.isclose(sum(g['residual_squared_sum_m2'] for g in groups),
                        n * tdcp['residual_rms_m']**2, rel_tol=1e-12)
    for group in groups:
        assert 0 <= group['tail_count'] <= group['count']
    return {'signal_aggregates': groups, 'candidate_sha256': actual, 'byte_identical_to_phase234': True,
            'native_elapsed_seconds': completed['elapsed_seconds'],
            'finite_tdcp_residuals': n, 'huber_tail_count': tail,
            'huber_tail_fraction': tail / n, 'reconstructed_huber_cost': robust,
            'reconstructed_quadratic_cost': quadratic,
            'huber_to_quadratic_cost_ratio': robust / quadratic,
            'truth_reads': 0, 'accuracy_evaluations': 0}


if __name__ == '__main__':
    print(json.dumps(verify(), indent=2, sort_keys=True))
