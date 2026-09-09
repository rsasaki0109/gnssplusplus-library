"""Frozen paired development evaluation; no native inference."""
import hashlib
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / 'docs/use_cases/records/smartphone_r5_phase386_lax_t_accuracy_manifest_v1.json'
RESULT = ROOT / 'docs/use_cases/records/smartphone_r5_phase386_lax_t_accuracy_result_v1.json'


def verify():
    m = json.loads(MANIFEST.read_text())
    for pin in m['authority'].values():
        assert hashlib.sha256((ROOT / pin['path']).read_bytes()).hexdigest() == pin['sha256']
    for name in ('baseline_structure', 'variant_structure'):
        s = json.loads((ROOT / m['authority'][name]['path']).read_text())
        assert s['execution']['return_code'] == 0 and s['truth_used'] is False
        assert s['graph']['converged'] and s['handoff']['converged']
        assert s['handoff']['epoch_identity_alignment_valid']
        assert s['tdcp_contract']['sparse_p_staging_requested']
        assert s['tdcp_contract']['code_phase_jump_gate_disabled_requested'] == (name == 'variant_structure')
        for stage in ('main', 'gnss_first'):
            assert all(s['termination'][stage][k] for k in
                       ('costs_finite', 'no_fallback', 'termination_trace_complete', 'configuration_valid'))
    projection = json.loads((ROOT / m['authority']['projection']['path']).read_text())
    for candidate, projected in zip(m['candidates'], projection['projected']):
        assert all(candidate[k] == projected[k] for k in ('path','sha256','bytes','rows'))
    return m


def evaluate():
    m = verify()
    assert not RESULT.exists()
    with RESULT.with_suffix('.attempt.json').open('x') as f:
        json.dump({'manifest_sha256': hashlib.sha256(MANIFEST.read_bytes()).hexdigest()}, f)
    spec = importlib.util.spec_from_file_location('phase203_for_386', ROOT / m['authority']['phase203_evaluator']['path'])
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    scores = []
    for c in m['candidates']:
        scores.append(module.score_payloads(ROOT/c['path'], c, ROOT/m['truth']['path'],
                      m['truth'], route=m['route'], result_path=RESULT))
    result = {'phase':386, 'role':m['role'], 'scores':scores,
              'delta_m':scores[1]['metric']['route_score_m']-scores[0]['metric']['route_score_m']}
    with RESULT.open('x') as f:
        json.dump(result,f,indent=2,allow_nan=False)
    return result


if __name__ == '__main__':
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('--evaluate',action='store_true')
    args=parser.parse_args()
    print(json.dumps(evaluate() if args.evaluate else {'verified':bool(verify())},indent=2))
