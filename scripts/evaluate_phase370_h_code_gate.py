"""One-shot frozen development evaluation. No solver or raw GNSS access."""
import hashlib
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RECORDS = ROOT / 'docs/use_cases/records'
MANIFEST = RECORDS / 'smartphone_r5_phase370_h_code_gate_accuracy_manifest_v1.json'
RESULT = RECORDS / 'smartphone_r5_phase370_h_code_gate_accuracy_result_v1.json'


def verify():
    m = json.loads(MANIFEST.read_text())
    assert m['phase'] == 370
    assert m['role'] == 'development/train; not heldout or leaderboard proof'
    for pin in m['authority'].values():
        path = Path(pin['path'])
        assert not path.is_absolute() and '..' not in path.parts
        assert path.suffix in ('.py', '.json')
        assert hashlib.sha256((ROOT / path).read_bytes()).hexdigest() == pin['sha256']
    s = json.loads((ROOT / m['authority']['structural']['path']).read_text())
    assert s['execution']['return_code'] == 0 and s['truth_used'] is False
    assert s['candidate_identity'] == m['candidate']
    assert s['graph']['converged'] and s['handoff']['converged']
    assert s['handoff']['epoch_identity_alignment_valid']
    assert s['tdcp_contract']['code_phase_jump_gate_disabled_requested'] is True
    assert s['tdcp_contract']['rejected_code_phase_jump'] == 0
    assert s['base_enabled'] is False
    assert s['epochs']['pseudorange_factors'] == 101916
    assert s['raw_utc_key_contract']['exact_solution_epochs'] == 3139
    assert all(s['raw_utc_key_contract'][k] == 0 for k in
               ('interpolated_epochs', 'edge_hold_epochs', 'unresolved_epochs'))
    for stage in ('main', 'gnss_first'):
        assert all(s['termination'][stage][k] for k in
                   ('costs_finite', 'no_fallback', 'termination_trace_complete', 'configuration_valid'))
    old = json.loads((ROOT / m['authority']['previous_evaluation']['path']).read_text())
    assert m['truth'] == old['truth'] and m['metric_contract'] == old['metric_contract']
    reference = json.loads((ROOT / m['authority']['baseline_result']['path']).read_text())
    assert m['baseline_m'] == reference['metric']['route_score_m']
    return m


def evaluate():
    m = verify()
    assert not RESULT.exists()
    # Exclusive marker prevents another score even after a failed attempt.
    with RESULT.with_suffix('.attempt.json').open('x') as handle:
        json.dump({'manifest_sha256': hashlib.sha256(MANIFEST.read_bytes()).hexdigest()}, handle)
    path = ROOT / m['authority']['phase203_evaluator']['path']
    spec = importlib.util.spec_from_file_location('phase203_for_370', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    score = module.score_payloads(ROOT / m['candidate']['path'], m['candidate'],
                                  ROOT / m['truth']['path'], m['truth'],
                                  route=m['route'], result_path=RESULT)
    result = dict(score, phase=370, role=m['role'], baseline_m=m['baseline_m'],
                  delta_m=score['metric']['route_score_m'] - m['baseline_m'])
    with RESULT.open('x') as handle:
        json.dump(result, handle, indent=2, allow_nan=False)
    return result


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evaluate', action='store_true')
    args = parser.parse_args()
    print(json.dumps(evaluate() if args.evaluate else {'verified': bool(verify())}, indent=2))
