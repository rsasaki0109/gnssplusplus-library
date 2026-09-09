"""One frozen raw H candidate; no truth or saved positioning inputs."""
import json
import os
import subprocess
import time
from pathlib import Path
from run_phase486_nhc_monitor import ROOT, digest

DIRECTORY = ROOT / 'output/smartphone-r5/phase579-endpoint-sigma-v1'


def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2)


def main():
    previous = ROOT / 'output/smartphone-r5/phase578-adr-metadata-v1/manifest.json'
    old = json.loads(previous.read_text())
    argv = list(old['argv']) + ['--native-tdcp-adr-endpoint-sigma']
    for item in old['inputs'].values():
        if digest(ROOT / item['path']) != item['sha256']:
            raise ValueError('raw input changed')
    pins = {}
    for folder in ('src', 'include', 'apps/native'):
        for path in sorted((ROOT / folder).rglob('*')):
            if path.is_file() and path.suffix in ('.cpp', '.hpp', '.h'):
                pins[str(path.relative_to(ROOT))] = digest(path)
    for path in (Path(__file__), ROOT / 'tests/test_tdcp_endpoint_covariance.cpp'):
        pins[str(path.resolve().relative_to(ROOT))] = digest(path)
    for flag, name in (('--out', 'opaque_solution_output.csv'),
                       ('--summary-json', 'native_summary.json')):
        argv[argv.index(flag) + 1] = str((DIRECTORY / name).relative_to(ROOT))
    record = dict(phase=579, argv=argv, inputs=old['inputs'], source_pins=pins,
                  binary_sha256=digest(ROOT / argv[0]),
                  previous_manifest_sha256=digest(previous),
                  acceptance=old['acceptance'], role=old['role'], scoring=False,
                  truth_reads=0, model='hypot of raw ADR endpoint uncertainties; diagonal only')
    DIRECTORY.mkdir(parents=True, exist_ok=False)
    write(DIRECTORY / 'manifest.json', record)
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = '/home/sasaki/.local/lib:' + env.get('LD_LIBRARY_PATH', '')
    start = time.monotonic()
    with (DIRECTORY / 'stdout.log').open('x') as out, (DIRECTORY / 'stderr.log').open('x') as err:
        process = subprocess.Popen(argv, cwd=ROOT, env=env, stdout=out, stderr=err)
        write(DIRECTORY / 'started.json', dict(pid=process.pid))
        print(json.dumps(dict(phase=579, pid=process.pid)), flush=True)
        code = process.wait()
    done = dict(return_code=code, elapsed_seconds=time.monotonic()-start, accuracy_pass=None)
    if code == 0:
        done['output_sha256'] = digest(DIRECTORY / 'opaque_solution_output.csv')
    done['pins_verified'] = (
        all(digest(ROOT / name) == sha for name, sha in pins.items())
        and digest(ROOT / argv[0]) == record['binary_sha256']
        and all(digest(ROOT / item['path']) == item['sha256'] for item in old['inputs'].values()))
    write(DIRECTORY / 'completed.json', done)
    print(json.dumps(done), flush=True)
    if code or not done['pins_verified']:
        raise ValueError('native execution/pins failed')


if __name__ == '__main__':
    main()
