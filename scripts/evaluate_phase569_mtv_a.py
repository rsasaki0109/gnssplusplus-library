"""Separately frozen MTV-A development score; no inference or output repair."""
import argparse
import importlib.util
import json
import math
from pathlib import Path
from run_phase486_nhc_monitor import ROOT, digest
from run_phase568_mtv_a_transfer import DIRECTORY

RECORDS=ROOT/'docs/use_cases/records'
MANIFEST=RECORDS/'smartphone_r5_phase569_mtv_a_accuracy_manifest_v1.json'
RESULT=RECORDS/'smartphone_r5_phase569_mtv_a_accuracy_result_v1.json'
ROUTE='2021-03-16-18-59-us-ca-mtv-a/pixel5'


def freeze():
    run=json.loads((DIRECTORY/'manifest.json').read_text())
    done=json.loads((DIRECTORY/'completed.json').read_text())
    if done['return_code'] or not done['pins_verified']:
        raise ValueError('native completion failed')
    for name,sha in run['source_pins'].items():
        if digest(ROOT/name)!=sha: raise ValueError('source changed')
    if digest(ROOT/run['argv'][0])!=run['binary_sha256']: raise ValueError('binary changed')
    for item in run['inputs'].values():
        if digest(ROOT/item['path'])!=item['sha256']: raise ValueError('raw changed')
    summary=json.loads((DIRECTORY/'native_summary.json').read_text())
    for stage in ('graph','gnss_first'):
        if not summary[stage]['converged'] or not math.isfinite(summary[stage]['final_cost']):
            raise ValueError('unconverged/nonfinite stage')
    if summary['epochs']['problem']!=2159: raise ValueError('unexpected epoch count')
    if summary['gnss_first']['coordinates_source']!='in-memory GNSS-first result only':
        raise ValueError('invalid handoff')
    old=json.loads((RECORDS/'smartphone_r5_phase146_phase144_accuracy_gate_manifest_v1.json').read_text())
    legacy=json.loads((RECORDS/'smartphone_r5_phase564_rotation_rate_accuracy_manifest_v1.json').read_text())
    authority={k:v for k,v in legacy['authority'].items() if k in ('phase76_parser','phase74_metric_helper')}
    if len(authority)!=2: raise ValueError('missing metric pins')
    for item in authority.values():
        if digest(ROOT/item['path'])!=item['sha256']: raise ValueError('metric changed')
    for path in (Path(__file__),DIRECTORY/'manifest.json',DIRECTORY/'completed.json',DIRECTORY/'native_summary.json'):
        name=str(path.resolve().relative_to(ROOT))
        authority[name]=dict(path=name,sha256=digest(path))
    output=DIRECTORY/'opaque_solution_output.csv'
    if digest(output)!=done['output_sha256']: raise ValueError('output changed')
    record=dict(phase=569,route=ROUTE,role='previously evaluated development, not heldout',
        truth=old['truth_reference']['routes'][ROUTE],authority=authority,
        candidate=dict(path=str(output.relative_to(ROOT)),sha256=digest(output),bytes=output.stat().st_size,rows=2158),
        policy='one score; historical first-key exclusion only; no repair, interpolation or inference')
    with MANIFEST.open('x') as stream: json.dump(record,stream,indent=2)
    print('Frozen evaluation; truth not read.')


def evaluate():
    record=json.loads(MANIFEST.read_text())
    if RESULT.exists(): raise ValueError('already evaluated')
    for item in record['authority'].values():
        if digest(ROOT/item['path'])!=item['sha256']: raise ValueError('authority changed')
    with RESULT.with_suffix('.attempt.json').open('x') as stream:
        json.dump(dict(manifest_sha256=digest(MANIFEST)),stream)
    spec=importlib.util.spec_from_file_location('phase76_for_569',ROOT/record['authority']['phase76_parser']['path'])
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    c=record['candidate']; t=record['truth']
    payload=module._read_once(ROOT/c['path'],c['sha256'],c['bytes'],'candidate')
    ordered,prediction=module.P74._parse_submission(payload,ROUTE)
    if len(ordered)!=c['rows']: raise ValueError('candidate domain mismatch; no truth read')
    truth=module._parse_truth_dictreader(module._read_once(ROOT/t['path'],t['sha256'],t['bytes'],'truth'),ROUTE)
    if len(truth)!=t['rows']: raise ValueError('truth domain mismatch')
    score=module._score_prediction(prediction,truth,t['expected_missing_truth_key'],ROUTE,ordered)
    result=dict(phase=569,role=record['role'],metric=score,
                reads=dict(candidate=1,truth=1),inference_calls=0)
    with RESULT.open('x') as stream: json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('freeze','evaluate'))
    freeze() if parser.parse_args().action=='freeze' else evaluate()
