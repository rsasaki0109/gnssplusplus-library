"""Freeze, then score the completed NHC candidate once; evaluation only."""
import argparse
import importlib.util
import json
from run_phase486_nhc_monitor import ROOT, digest
from verify_phase488_batch_nhc import main as verify_structure

RECORDS = ROOT / 'docs/use_cases/records'
MANIFEST = RECORDS / 'smartphone_r5_phase489_batch_nhc_accuracy_manifest_v1.json'
RESULT = RECORDS / 'smartphone_r5_phase489_batch_nhc_accuracy_result_v1.json'

def freeze():
    verify_structure()
    old = json.loads((RECORDS / 'smartphone_r5_phase362_stationary_gyro_accuracy_manifest_v1.json').read_text())
    directory = ROOT / 'output/smartphone-r5/phase488-h-batch-nhc-v1/mtv-h'
    candidate = directory / 'opaque_solution_output.csv'
    authorities = {k: v for k, v in old['authority'].items()
                   if k.startswith('phase')}
    for pin in authorities.values():
        assert digest(ROOT / pin['path']) == pin['sha256']
    for path in (directory / 'manifest.json', directory / 'completed.json',
                 directory / 'native_summary.json',
                 ROOT / 'scripts/verify_phase488_batch_nhc.py',
                 ROOT / 'scripts/evaluate_phase489_batch_nhc.py'):
        name = str(path.relative_to(ROOT))
        authorities[name] = dict(path=name, sha256=digest(path))
    manifest = dict(phase=489, route=old['route'], role=old['role'],
                    truth=old['truth'], metric_contract=old['metric_contract'],
                    baseline_m=old['baseline_m'], authority=authorities,
                    candidate=dict(path=str(candidate.relative_to(ROOT)),
                                   sha256=digest(candidate), bytes=candidate.stat().st_size,
                                   rows=3139),
                    policy='one evaluation of frozen candidate; no solver, no tuning or retry')
    with MANIFEST.open('x') as handle:
        json.dump(manifest, handle, indent=2)
    print('Evaluation manifest frozen; truth not read.')

def evaluate():
    manifest = json.loads(MANIFEST.read_text())
    assert manifest['candidate']['sha256'] == '0bbc0d1b4bdc18ee29972e74ca9b5d9666b766b361b9cf8c600b762e584098eb'
    assert not RESULT.exists()
    for pin in manifest['authority'].values():
        assert digest(ROOT / pin['path']) == pin['sha256']
    with RESULT.with_suffix('.attempt.json').open('x') as handle:
        json.dump(dict(manifest_sha256=digest(MANIFEST)), handle)
    path = ROOT / manifest['authority']['phase203_evaluator']['path']
    spec = importlib.util.spec_from_file_location('phase203_for_489', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    score = module.score_payloads(ROOT / manifest['candidate']['path'], manifest['candidate'],
                                  ROOT / manifest['truth']['path'], manifest['truth'],
                                  route=manifest['route'], result_path=RESULT)
    result = dict(score, phase=489, role=manifest['role'], baseline_m=manifest['baseline_m'],
                  delta_m=score['metric']['route_score_m'] - manifest['baseline_m'])
    with RESULT.open('x') as handle:
        json.dump(result, handle, indent=2, allow_nan=False)
    print(json.dumps({k: result[k] for k in ('phase', 'role', 'metric', 'baseline_m', 'delta_m')}, indent=2))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('freeze', 'evaluate'))
    args = parser.parse_args()
    freeze() if args.action == 'freeze' else evaluate()
