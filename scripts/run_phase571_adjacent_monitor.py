"""Frozen H/A raw replays; output identity and adjacent residual diagnostics only."""
import argparse
import json
import os
import re
import subprocess
import time
from pathlib import Path
from run_phase486_nhc_monitor import ROOT, digest

DIRECTORY=ROOT/'output/smartphone-r5/phase571-adjacent-monitor-v1'


def write(path,value):
    with path.open('x') as stream: json.dump(value,stream,indent=2)


def main(route):
    previous=ROOT/('output/smartphone-r5/phase563-rotation-rate-v1' if route=='h'
                   else 'output/smartphone-r5/phase568-mtv-a-transfer-v1')
    old=json.loads((previous/'manifest.json').read_text())
    argv=list(old['argv']['baseline'] if route=='h' else old['argv'])
    previous_output=ROOT/argv[argv.index('--out')+1]
    previous_summary=ROOT/argv[argv.index('--summary-json')+1]
    expected=digest(previous_output)  # Byte identity check only; never inference.
    for item in old['inputs'].values():
        if digest(ROOT/item['path'])!=item['sha256']: raise ValueError('raw input changed')
    pins={}
    for name,sha in old['source_pins'].items():
        value=digest(ROOT/name)
        if value!=sha and name not in ('apps/native/gnss_fgo_imu_no_base.cpp','tests/CMakeLists.txt'):
            raise ValueError(f'unexpected source change: {name}')
        pins[name]=value
    for name in ('include/libgnss++/algorithms/adjacent_residual_moments.hpp',
                 'tests/test_adjacent_residual_moments.cpp','scripts/run_phase571_adjacent_monitor.py'):
        pins[name]=digest(ROOT/name)
    directory=DIRECTORY/route
    for flag,name in (('--out','opaque_solution_output.csv'),('--summary-json','native_summary.json')):
        argv[argv.index(flag)+1]=str((directory/name).relative_to(ROOT))
    record=dict(route=route,argv=argv,source_pins=pins,inputs=old['inputs'],
                binary_sha256=digest(ROOT/argv[0]),expected_output_sha256=expected,
                previous_manifest_sha256=digest(previous/'manifest.json'),
                previous_summary_sha256=digest(previous_summary),truth_reads=0,scoring=False)
    directory.mkdir(parents=True,exist_ok=False)
    write(directory/'manifest.json',record)
    env=os.environ.copy(); env['LD_LIBRARY_PATH']='/home/sasaki/.local/lib:'+env.get('LD_LIBRARY_PATH','')
    start=time.monotonic()
    with (directory/'stdout.log').open('x') as out,(directory/'stderr.log').open('x') as err:
        process=subprocess.Popen(argv,cwd=ROOT,env=env,stdout=out,stderr=err)
        write(directory/'started.json',dict(pid=process.pid))
        print(json.dumps(dict(route=route,pid=process.pid)),flush=True)
        code=process.wait()
    done=dict(return_code=code,elapsed_seconds=time.monotonic()-start)
    if code==0:
        done['output_sha256']=digest(directory/'opaque_solution_output.csv')
        done['output_identical']=done['output_sha256']==expected
        before=json.loads(previous_summary.read_text())
        after=json.loads((directory/'native_summary.json').read_text())
        done['summary_identical']=all(before[k]==after[k] for k in ('epochs','graph','gnss_first','tdcp_contract'))
        done['monitor']=re.findall(r'^\[native-tdcp-adjacent-residual\].*$',(directory/'stderr.log').read_text(),re.M)
    done['pins_verified']=(all(digest(ROOT/name)==sha for name,sha in pins.items())
        and digest(ROOT/argv[0])==record['binary_sha256']
        and all(digest(ROOT/item['path'])==item['sha256'] for item in old['inputs'].values()))
    write(directory/'completed.json',done)
    print(json.dumps(done),flush=True)
    if code or not done.get('output_identical') or not done.get('summary_identical') or not done['pins_verified']:
        raise ValueError('replay identity/pins failed')
    if len(done['monitor'])!=1: raise ValueError('missing or duplicate monitor')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('route',choices=('h','a'))
    main(parser.parse_args().route)
