"""Frozen development scoring only; no native inference or parameter tuning."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

from verify_native_frequency_experiment import verify

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def load_module(path):
    spec = importlib.util.spec_from_file_location('frequency_frozen_scoring', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def preflight(manifest_path):
    manifest = json.loads(manifest_path.read_text())
    assert manifest['role'] == 'development/train; not heldout or leaderboard proof'
    assert manifest['evaluator_path'] == 'apps/commands/benchmarks/gnss_smartphone_phase203_phase202_h_accuracy.py'
    assert manifest['evaluator_path'] in manifest['source_pins']
    # Complete prospective inference checks before any truth payload access.
    checks = {phase: verify(phase) for phase in (422, 423)}
    for name, expected in manifest['source_pins'].items():
        assert digest(ROOT / name) == expected, name
    assert {c['phase'] for c in manifest['candidates']} == {422, 423}
    assert len(manifest['candidates']) == 2
    for candidate in manifest['candidates']:
        assert candidate['native_sha256'] == checks[candidate['phase']]['candidate_sha256']
        path = ROOT / candidate['path']
        assert path.stat().st_size == candidate['bytes']
        assert digest(path) == candidate['sha256']
    return manifest


def evaluate(manifest_path):
    manifest = preflight(manifest_path)
    result_path = ROOT / manifest['result_path']
    assert not result_path.exists()
    # Exclusive claim prevents another evaluation after success or failure.
    with result_path.with_suffix('.attempt.json').open('x') as stream:
        json.dump({'manifest_sha256': digest(manifest_path)}, stream)
    kernel = load_module(ROOT / manifest['evaluator_path'])
    scores = []
    for candidate in manifest['candidates']:
        truth = candidate['truth']
        score = kernel.score_payloads(ROOT / candidate['path'], candidate,
                                      ROOT / truth['path'], truth,
                                      route=candidate['route'], result_path=result_path)
        scores.append({'phase': candidate['phase'], 'score': score,
                       'baseline_m': candidate['baseline_m'],
                       'delta_m': score['metric']['route_score_m']-candidate['baseline_m']})
    result = {'role': manifest['role'], 'manifest_sha256': digest(manifest_path),
              'scores': scores, 'native_invocations': 0}
    with result_path.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('--evaluate', action='store_true')
    args = parser.parse_args()
    print(json.dumps(evaluate(args.manifest) if args.evaluate else
                     {'verified': bool(preflight(args.manifest))}, indent=2))
