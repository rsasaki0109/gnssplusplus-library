"""Frozen single H batch NHC candidate; raw inference only, no scoring."""
import json
import os
import re
import subprocess
import time
from run_phase486_nhc_monitor import ROOT, digest

def main():
    previous = ROOT / 'output/smartphone-r5/phase486-h-nhc-monitor-v1/mtv-h/manifest.json'
    old = json.loads(previous.read_text())
    allowed = {'apps/native/gnss_fgo_imu_no_base.cpp',
               'include/libgnss++/algorithms/fgo_config.hpp',
               'src/algorithms/fgo.cpp', 'src/algorithms/fgo_gtsam_backend.cpp'}
    pins = {}
    for name, expected in old['source_pins'].items():
        actual = digest(ROOT / name)
        if actual != expected and name not in allowed:
            raise RuntimeError(f'unexpected change: {name}')
        pins[name] = actual
    for name in ('scripts/run_phase488_batch_nhc.py', 'scripts/native_nhc_frame_audit.cpp'):
        pins[name] = digest(ROOT / name)
    for item in old['inputs'].values():
        path = ROOT / item['path']
        if digest(path) != item['sha256'] or path.stat().st_size != item['bytes']:
            raise RuntimeError('raw input mismatch')
    directory = ROOT / 'output/smartphone-r5/phase488-h-batch-nhc-v1/mtv-h'
    argv = list(old['argv']) + ['--native-batch-nhc']
    for flag, filename in (('--out', 'opaque_solution_output.csv'),
                           ('--summary-json', 'native_summary.json')):
        argv[argv.index(flag) + 1] = str((directory / filename).relative_to(ROOT))
    directory.mkdir(parents=True, exist_ok=False)
    record = dict(phase=488, argv=argv, inputs=old['inputs'], source_pins=pins,
                  previous_manifest_sha256=digest(previous),
                  binary_sha256=digest(ROOT / argv[0]),
                  policy='one raw H run, frozen Phase487 settings, no truth or retry',
                  expected_nhc_factors=1627)
    with (directory / 'manifest.json').open('x') as handle:
        json.dump(record, handle, indent=2)
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = '/home/sasaki/.local/lib:' + env.get('LD_LIBRARY_PATH', '')
    start = time.monotonic()
    with (directory / 'stdout.log').open('x') as out, (directory / 'stderr.log').open('x') as err:
        process = subprocess.Popen(argv, cwd=ROOT, env=env, stdout=out, stderr=err)
        with (directory / 'started.json').open('x') as handle:
            json.dump({'pid': process.pid}, handle)
        print(json.dumps({'phase': 488, 'pid': process.pid}), flush=True)
        code = process.wait()
    result = dict(return_code=code, elapsed_seconds=time.monotonic() - start)
    if code == 0:
        result['output_sha256'] = digest(directory / 'opaque_solution_output.csv')
        result['monitor'] = re.findall(r'^\[native-nhc-monitor\].*$',
                                      (directory / 'stderr.log').read_text(), re.M)
    with (directory / 'completed.json').open('x') as handle:
        json.dump(result, handle, indent=2)
    print(json.dumps(result), flush=True)
    if code:
        return code
    if len(result['monitor']) != 1 or not result['monitor'][0].endswith('factors_added=1627'):
        raise RuntimeError('unexpected NHC factor count')
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
