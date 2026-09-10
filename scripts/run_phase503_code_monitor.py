"""One raw H masked-code shadow diagnostic replay; no truth or weight changes."""
import json
import os
import re
import subprocess
import time
from run_phase486_nhc_monitor import ROOT, digest

def main():
    previous = ROOT / 'output/smartphone-r5/phase495-h-main-code-floor-v1/mtv-h/manifest.json'
    old = json.loads(previous.read_text())
    pins = {}
    for name, expected in old['source_pins'].items():
        pins[name] = digest(ROOT / name)
        if pins[name] != expected and name != 'src/algorithms/fgo_problems.cpp':
            raise RuntimeError(f'unexpected source change: {name}')
    pins['scripts/run_phase503_code_monitor.py'] = digest(ROOT / 'scripts/run_phase503_code_monitor.py')
    for item in old['inputs'].values():
        path = ROOT / item['path']
        assert digest(path) == item['sha256'] and path.stat().st_size == item['bytes']
    argv = [arg for arg in old['argv'] if arg not in ('--native-batch-nhc', '--native-nhc-monitor', '--native-main-code-uncertainty-floor')]
    assert '--native-android-sv-time-uncertainty-sigma-floor' not in argv
    directory = ROOT / 'output/smartphone-r5/phase503-h-code-monitor-v1/mtv-h'
    for flag, filename in (('--out', 'opaque_solution_output.csv'), ('--summary-json', 'native_summary.json')):
        argv[argv.index(flag) + 1] = str((directory / filename).relative_to(ROOT))
    directory.mkdir(parents=True, exist_ok=False)
    record = dict(phase=503, argv=argv, source_pins=pins, inputs=old['inputs'],
                  binary_sha256=digest(ROOT / argv[0]), previous_manifest_sha256=digest(previous),
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
        print(json.dumps({'phase': 503, 'pid': process.pid}), flush=True)
        code = process.wait()
    result = dict(return_code=code, elapsed_seconds=time.monotonic() - start)
    if code == 0:
        result['output_sha256'] = digest(directory / 'opaque_solution_output.csv')
        result['output_identical'] = result['output_sha256'] == record['expected_output_sha256']
        result['monitor'] = re.findall(r'^\[native-masked-code-shadow\].*$',
                                      (directory / 'stderr.log').read_text(), re.M)
    with (directory / 'completed.json').open('x') as handle:
        json.dump(result, handle, indent=2)
    print(json.dumps(result), flush=True)
    if code: return code
    assert result['output_identical'] and len(result['monitor']) == 1
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
