"""Single raw H replay, diagnostic only; freeze hashes before launching."""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]

def digest(path):
    with path.open('rb') as handle:
        return hashlib.file_digest(handle, 'sha256').hexdigest()

def main():
    previous = ROOT / 'docs/use_cases/records/smartphone_r5_phase479_h_baseline_replay_manifest_v1.json'
    old = json.loads(previous.read_text())
    for item in old['inputs'].values():
        path = ROOT / item['path']
        if path.stat().st_size != item['bytes'] or digest(path) != item['sha256']:
            raise RuntimeError('raw input pin mismatch')
    allowed_changes = {'apps/native/gnss_fgo_imu_no_base.cpp',
                       'src/algorithms/fgo_gtsam_backend.cpp'}
    pins = {}
    for group in ('source_pins', 'test_pins', 'algorithm_source_pins'):
        for name, expected in old[group].items():
            actual = digest(ROOT / name)
            if actual != expected and name not in allowed_changes:
                raise RuntimeError(f'unexpected source change: {name}')
            pins[name] = actual
    for name in ('include/libgnss++/algorithms/native_nhc_gate.hpp',
                 'tests/test_native_nhc_gate.cpp', 'scripts/run_phase486_nhc_monitor.py'):
        pins[name] = digest(ROOT / name)
    directory = ROOT / 'output/smartphone-r5/phase486-h-nhc-monitor-v1/mtv-h'
    argv = list(old['argv'])
    for flag, filename in (('--out', 'opaque_solution_output.csv'),
                           ('--summary-json', 'native_summary.json')):
        argv[argv.index(flag) + 1] = str((directory / filename).relative_to(ROOT))
    argv.append('--native-nhc-monitor')
    for flag, name in (('--android-gnss', 'android_gnss'),
                       ('--android-imu', 'android_imu'), ('--nav', 'nav')):
        assert argv[argv.index(flag) + 1] == old['inputs'][name]['path']
    directory.mkdir(parents=True, exist_ok=False)
    record = dict(phase=486, argv=argv, inputs=old['inputs'], source_pins=pins,
                  previous_manifest_sha256=digest(previous),
                  binary_sha256=digest(ROOT / argv[0]),
                  policy='one raw replay; no truth; no factors added; no retries',
                  expected_output_sha256='4a0c4ef8822c72aa9134368cd57fe769817895e63b5dbe925783a9c8d091fa8e')
    with (directory / 'manifest.json').open('x') as handle:
        json.dump(record, handle, indent=2)
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = '/home/sasaki/.local/lib:' + env.get('LD_LIBRARY_PATH', '')
    start = time.monotonic()
    with (directory / 'stdout.log').open('x') as out, (directory / 'stderr.log').open('x') as err:
        process = subprocess.Popen(argv, cwd=ROOT, env=env, stdout=out, stderr=err)
        with (directory / 'started.json').open('x') as handle:
            json.dump({'pid': process.pid}, handle)
        print(json.dumps({'pid': process.pid, 'phase': 486}), flush=True)
        code = process.wait()
    result = dict(return_code=code, elapsed_seconds=time.monotonic() - start)
    if code == 0:
        result['output_sha256'] = digest(directory / 'opaque_solution_output.csv')
        result['output_identical'] = result['output_sha256'] == record['expected_output_sha256']
        result['monitor'] = re.findall(r'^\[native-nhc-monitor\].*$',
                                      (directory / 'stderr.log').read_text(), re.M)
    with (directory / 'completed.json').open('x') as handle:
        json.dump(result, handle, indent=2)
    print(json.dumps(result), flush=True)
    if code != 0:
        return code
    if not result['output_identical'] or len(result['monitor']) != 1:
        raise RuntimeError('diagnostic invariance or coverage marker check failed')
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
