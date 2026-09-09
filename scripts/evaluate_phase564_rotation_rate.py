"""One separately frozen H development evaluation; never invokes inference."""
import argparse
import importlib.util
import json
from run_phase486_nhc_monitor import ROOT, digest
from run_phase563_rotation_rate import DIRECTORY
from verify_phase563_rotation_rate import verify

RECORDS = ROOT / 'docs/use_cases/records'
MANIFEST = RECORDS / 'smartphone_r5_phase564_rotation_rate_accuracy_manifest_v1.json'
RESULT = RECORDS / 'smartphone_r5_phase564_rotation_rate_accuracy_result_v1.json'


def freeze():
    verify('candidate')
    old = json.loads((RECORDS / 'smartphone_r5_phase362_stationary_gyro_accuracy_manifest_v1.json').read_text())
    authority = {k: v for k, v in old['authority'].items() if k.startswith('phase')}
    for pin in authority.values():
        if digest(ROOT / pin['path']) != pin['sha256']:
            raise ValueError('legacy metric authority changed')
    paths = [DIRECTORY / 'manifest.json', ROOT / 'scripts/verify_phase563_rotation_rate.py',
             ROOT / 'scripts/evaluate_phase564_rotation_rate.py',
             ROOT / 'scripts/run_phase563_rotation_rate.py',
             ROOT / 'scripts/run_phase486_nhc_monitor.py']
    for stage in ('baseline', 'candidate'):
        paths += [DIRECTORY / stage / name for name in ('completed.json', 'native_summary.json')]
    for path in paths:
        name = str(path.relative_to(ROOT))
        authority[name] = dict(path=name, sha256=digest(path))
    record = dict(phase=564, route=old['route'], role=old['role'], truth=old['truth'],
                  metric_contract=old['metric_contract'], authority=authority,
                  policy='one fixed development comparison, no tuning, no inference, not held-out or leaderboard')
    for stage in ('baseline', 'candidate'):
        path = DIRECTORY / stage / 'opaque_solution_output.csv'
        record[stage] = dict(path=str(path.relative_to(ROOT)), sha256=digest(path),
                             bytes=path.stat().st_size, rows=3139)
    with MANIFEST.open('x') as stream:
        json.dump(record, stream, indent=2)
    print('Accuracy manifest frozen; truth not read.')


def evaluate():
    record = json.loads(MANIFEST.read_text())
    if RESULT.exists():
        raise ValueError('evaluation already completed')
    for pin in record['authority'].values():
        if digest(ROOT / pin['path']) != pin['sha256']:
            raise ValueError(f'authority changed: {pin["path"]}')
    with RESULT.with_suffix('.attempt.json').open('x') as stream:
        json.dump(dict(manifest_sha256=digest(MANIFEST)), stream)
    path = ROOT / record['authority']['phase203_evaluator']['path']
    spec = importlib.util.spec_from_file_location('phase203_for_564', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    score = module.score_payloads(ROOT / record['candidate']['path'], record['candidate'],
                                  ROOT / record['truth']['path'], record['truth'],
                                  route=record['route'], result_path=RESULT)
    baseline = module.score_payloads(ROOT / record['baseline']['path'], record['baseline'],
                                     ROOT / record['truth']['path'], record['truth'], route=record['route'])
    baseline_m = baseline['metric']['route_score_m']
    result = dict(score, phase=564, role=record['role'], baseline_m=baseline_m,
                  matched_baseline_metric=baseline['metric'],
                  paired_evaluation_read_accounting=dict(candidate_payload_reads=1,
                      baseline_payload_reads=1, truth_payload_reads=2),
                  delta_m=score['metric']['route_score_m']-baseline_m)
    with RESULT.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps({k: result[k] for k in ('phase', 'role', 'metric', 'baseline_m', 'delta_m')}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('freeze', 'evaluate'))
    freeze() if parser.parse_args().action == 'freeze' else evaluate()
