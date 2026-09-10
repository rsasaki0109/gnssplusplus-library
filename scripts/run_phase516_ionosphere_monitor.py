"""One raw-only H baseline replay with read-only ionosphere information."""
import json
import os
import re
import subprocess
import time
from run_phase486_nhc_monitor import ROOT, digest


def main():
    previous = ROOT / 'output/smartphone-r5/phase507-h-code-monitor-v1/mtv-h/manifest.json'
    old = json.loads(previous.read_text())
    pins = {}
    for name, expected in old['source_pins'].items():
        pins[name] = digest(ROOT / name)
        if pins[name] != expected and name not in (
                'apps/native/gnss_fgo_imu_no_base.cpp',
                'include/libgnss++/algorithms/pseudorange_position_information.hpp',
                'tests/test_pseudorange_position_information.cpp'):
            raise RuntimeError(f'unexpected source change: {name}')
    for name in ('scripts/run_phase516_ionosphere_monitor.py',
                 'include/libgnss++/algorithms/pseudorange_position_information.hpp',
                 'tests/test_pseudorange_position_information.cpp'):
        pins[name] = digest(ROOT / name)
    for item in old['inputs'].values():
        path = ROOT / item['path']
        assert digest(path) == item['sha256'] and path.stat().st_size == item['bytes']
    argv = [arg for arg in old['argv'] if arg != '--native-main-code-edge-readmission']
    directory = ROOT / 'output/smartphone-r5/phase516-h-ionosphere-monitor-v1/mtv-h'
    for flag, filename in (('--out', 'opaque_solution_output.csv'), ('--summary-json', 'native_summary.json')):
        argv[argv.index(flag) + 1] = str((directory / filename).relative_to(ROOT))
    directory.mkdir(parents=True, exist_ok=False)
    record = dict(phase=516, argv=argv, source_pins=pins, inputs=old['inputs'],
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
        print(json.dumps({'phase': 516, 'pid': process.pid}), flush=True)
        code = process.wait()
    result = dict(return_code=code, elapsed_seconds=time.monotonic() - start)
    if code == 0:
        result['output_sha256'] = digest(directory / 'opaque_solution_output.csv')
        result['output_identical'] = result['output_sha256'] == record['expected_output_sha256']
        result['monitor'] = re.findall(r'^\[native-code-ionosphere-information\].*$',
                                      (directory / 'stderr.log').read_text(), re.M)
    with (directory / 'completed.json').open('x') as handle:
        json.dump(result, handle, indent=2)
    print(json.dumps(result), flush=True)
    if code:
        return code
    assert result['output_identical']
    assert len(result['monitor']) == 1
    assert ' invalid=0 ' in result['monitor'][0]
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
