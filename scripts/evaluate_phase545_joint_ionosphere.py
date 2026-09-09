"""Freeze, then score the joint candidate and matched baseline; evaluation only."""
import argparse
import importlib.util
import json
from run_phase486_nhc_monitor import ROOT, digest
from verify_phase538_joint_transfer import main as verify_structure

RECORDS = ROOT / 'docs/use_cases/records'
MANIFEST = RECORDS / 'smartphone_r5_phase545_joint_ionosphere_accuracy_manifest_v1.json'
RESULT = RECORDS / 'smartphone_r5_phase545_joint_ionosphere_accuracy_result_v1.json'

def freeze():
    verify_structure('lax')
    old = json.loads((RECORDS / 'smartphone_r5_phase386_lax_t_accuracy_manifest_v1.json').read_text())
    directory = ROOT / 'output/smartphone-r5/phase538-joint-transfer-v1/lax'
    candidate = directory / 'opaque_solution_output.csv'
    recipe = json.loads((RECORDS / 'smartphone_r5_phase476_lax_t_comparison_manifest_v1.json').read_text())
    baseline = ROOT / recipe['argv'][recipe['argv'].index('--out') + 1]
    import csv, io
    native = json.loads((directory / 'manifest.json').read_text())
    raw = next(p for p in native['inputs'].values() if p['path'].endswith('/device_gnss.csv'))
    assert digest(ROOT / raw['path']) == raw['sha256']
    keys = sorted({int(r['utcTimeMillis']) for r in csv.DictReader(
        io.StringIO((ROOT / raw['path']).read_text())) if r['MessageType'] == 'Raw'})
    assert len(keys) == 1466
    target = ROOT / 'output/smartphone-r5/phase545-lax-evaluation-only-v1'
    originals = []
    payloads = []
    for label, path in (('baseline', baseline), ('candidate', candidate)):
        b = path.read_bytes()
        rows = list(csv.DictReader(io.StringIO(b.decode())))
        lines = b.splitlines(keepends=True)
        assert len(rows) == 1466 and len(lines) == 1467
        assert [int(r['UnixTimeMillis']) for r in rows] == keys
        assert all(r['phone'] == old['route'] for r in rows)
        originals.append(dict(label=label, path=str(path.relative_to(ROOT)),
                              sha256=digest(path), bytes=len(b), rows=1466))
        payloads.append((label, lines[0] + b''.join(lines[2:])))
    target.mkdir(parents=True, exist_ok=False)
    with (target / 'projection_manifest.json').open('x') as handle:
        json.dump(dict(originals=originals, raw=raw, removed='first raw UTC key only',
            script_sha256=digest(ROOT / 'scripts/evaluate_phase545_joint_ionosphere.py'),
            inference_use_forbidden=True, truth_reads=0), handle, indent=2)
    for label, payload in payloads:
        with (target / (label + '.csv')).open('xb') as handle:
            handle.write(payload)
    baseline, candidate = target / 'baseline.csv', target / 'candidate.csv'
    authorities = {k: v for k, v in old['authority'].items()
                   if k.startswith('phase')}
    for pin in authorities.values():
        assert digest(ROOT / pin['path']) == pin['sha256']
    for path in (target / 'projection_manifest.json', directory / 'manifest.json', directory / 'completed.json',
                 directory / 'native_summary.json',
                 ROOT / 'scripts/verify_phase538_joint_transfer.py',
                 ROOT / 'scripts/evaluate_phase545_joint_ionosphere.py'):
        name = str(path.relative_to(ROOT))
        authorities[name] = dict(path=name, sha256=digest(path))
    manifest = dict(phase=545, route=old['route'], role=old['role'],
                    truth=old['truth'], metric_contract=old['metric_contract'],
                    historical_baseline_m=None, authority=authorities,
                    baseline=dict(path=str(baseline.relative_to(ROOT)),
                                  sha256=digest(baseline), bytes=baseline.stat().st_size,
                                  rows=1465),
                    candidate=dict(path=str(candidate.relative_to(ROOT)),
                                   sha256=digest(candidate), bytes=candidate.stat().st_size,
                                   rows=1465),
                    policy='one evaluation of frozen candidate; no solver, no tuning or retry')
    with MANIFEST.open('x') as handle:
        json.dump(manifest, handle, indent=2)
    print('Evaluation manifest frozen; truth not read.')

def evaluate():
    manifest = json.loads(MANIFEST.read_text())
    assert not RESULT.exists()
    for pin in manifest['authority'].values():
        assert digest(ROOT / pin['path']) == pin['sha256']
    with RESULT.with_suffix('.attempt.json').open('x') as handle:
        json.dump(dict(manifest_sha256=digest(MANIFEST)), handle)
    path = ROOT / manifest['authority']['phase203_evaluator']['path']
    spec = importlib.util.spec_from_file_location('phase203_for_545', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    score = module.score_payloads(ROOT / manifest['candidate']['path'], manifest['candidate'],
                                  ROOT / manifest['truth']['path'], manifest['truth'],
                                  route=manifest['route'], result_path=RESULT)
    matched = module.score_payloads(ROOT / manifest['baseline']['path'], manifest['baseline'],
                                    ROOT / manifest['truth']['path'], manifest['truth'],
                                    route=manifest['route'])
    baseline_m = matched['metric']['route_score_m']
    result = dict(score, phase=545, role=manifest['role'], baseline_m=baseline_m,
                  matched_baseline_metric=matched['metric'],
                  paired_evaluation_read_accounting=dict(candidate_payload_reads=1,
                      baseline_payload_reads=1, truth_payload_reads=2,
                      note='Inherited score/read_accounting describes candidate call only'),
                  delta_m=score['metric']['route_score_m'] - baseline_m)
    with RESULT.open('x') as handle:
        json.dump(result, handle, indent=2, allow_nan=False)
    print(json.dumps({k: result[k] for k in ('phase', 'role', 'metric', 'baseline_m', 'delta_m')}, indent=2))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('freeze', 'evaluate'))
    args = parser.parse_args()
    freeze() if args.action == 'freeze' else evaluate()
