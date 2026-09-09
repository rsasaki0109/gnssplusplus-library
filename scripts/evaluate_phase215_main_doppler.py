"""One-shot, frozen H development evaluation; never invoke a native solver."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RECORDS = ROOT / 'docs/use_cases/records'
MANIFEST = RECORDS / 'smartphone_r5_phase215_h_accuracy_manifest_v1.json'
AUTH = RECORDS / 'smartphone_r5_phase215_h_accuracy_authorization_v1.json'
RESULT = RECORDS / 'smartphone_r5_phase215_h_accuracy_result_v1.json'
ROUTE = '2021-08-24-20-32-us-ca-mtv-h/pixel5'
BASELINE = 1.2751561666667786


def require(condition, message):
    if not condition:
        raise ValueError(message)


def static_bytes(path):
    require(path.suffix in {'.json', '.py'}, 'non-metadata access forbidden')
    return path.read_bytes()


def digest(data):
    return hashlib.sha256(data).hexdigest()


def read(path):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            require(key not in result, 'duplicate JSON key')
            result[key] = value
        return result
    return json.loads(static_bytes(path), object_pairs_hook=unique)


def verify():
    m = read(MANIFEST)
    require(m['phase'] == 215 and m['route'] == ROUTE, 'wrong experiment')
    require(m['baseline_m'] == BASELINE, 'baseline changed')
    require(m['role'] == 'development/train; not heldout or leaderboard proof', 'role changed')
    for name, pin in m['authority'].items():
        path = Path(pin['path'])
        require(not path.is_absolute() and '..' not in path.parts, 'unsafe authority path')
        require(digest(static_bytes(ROOT / path)) == pin['sha256'], f'pin mismatch: {name}')
    a = m['authority']
    require(a['evaluator']['path'] == str(Path(__file__).resolve().relative_to(ROOT)), 'wrong evaluator')
    old = read(ROOT / a['phase203_manifest']['path'])
    structural = read(ROOT / a['phase214_result']['path'])
    require(structural['execution']['return_code'] == 0, 'native run failed')
    require(structural['accuracy_and_payload_policy']['truth_used'] is False, 'native truth boundary')
    require(structural['graph']['phase213_main_doppler'] == {'requested': True, 'enabled': True, 'factors': 66685}, 'main Doppler counts changed')
    require(all(structural['unchanged_vs_phase198'].values()), 'baseline composition changed')
    require(m['truth'] == old['truth'], 'truth metadata changed')
    require(m['metric_contract'] == old['metric_contract'], 'metric changed')
    candidate = structural['candidate_identity']
    require(m['candidate'] == {k: candidate[k] for k in ('path', 'sha256', 'bytes')} | {'rows': 3139}, 'candidate changed')
    require(candidate['published_rows'] == 3139, 'candidate coverage changed')
    for name in ('evaluator', 'phase199_evaluator', 'phase196_thin_wrapper',
                 'phase189_kernel', 'phase76_parser', 'phase74_metric_helper'):
        target = 'phase203_evaluator' if name == 'evaluator' else name
        require(a[target]['path'] == old['authority'][name]['path'] and
                a[target]['sha256'] == old['authority'][name]['sha256'], 'metric dependency changed')
    return m


def evaluate():
    m = verify()
    auth = read(AUTH)
    require(auth == {'phase': 215, 'allow_truth_read': True,
                     'manifest_sha256': digest(static_bytes(MANIFEST)),
                     'candidate_sha256': m['candidate']['sha256'],
                     'truth_sha256': m['truth']['sha256']}, 'authorization mismatch')
    require(not RESULT.exists(), 'refusing result overwrite')
    # Exclusive claim also prevents a second score after a failed attempt.
    claim = RESULT.with_suffix('.attempt.json')
    with claim.open('x') as f:
        json.dump(auth, f, indent=2)
    path = ROOT / m['authority']['phase203_evaluator']['path']
    spec = importlib.util.spec_from_file_location('phase203_for_215', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    score = module.score_payloads(ROOT / m['candidate']['path'], m['candidate'],
                                  ROOT / m['truth']['path'], m['truth'],
                                  route=ROUTE, result_path=RESULT)
    value = score['metric']['route_score_m']
    result = {'phase': 215, 'execution_label': 'primary agent',
              'manifest_sha256': auth['manifest_sha256'], 'role': m['role'],
              'baseline_m': BASELINE, 'delta_m': value - BASELINE,
              'improved_on_h': value < BASELINE, **score}
    with RESULT.open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evaluate', action='store_true')
    args = parser.parse_args()
    print(json.dumps(evaluate() if args.evaluate else verify(), indent=2))
