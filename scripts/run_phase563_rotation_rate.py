"""Frozen raw-only H baseline/candidate; no truth or position-input reads."""
import argparse
import json
import os
import subprocess
import time
from run_phase486_nhc_monitor import ROOT, digest

DIRECTORY = ROOT / 'output/smartphone-r5/phase563-rotation-rate-v1'
PREVIOUS = ROOT / 'output/smartphone-r5/phase535-h-ionosphere-monitor-v1/mtv-h/manifest.json'
EXPECTED = '4a0c4ef8822c72aa9134368cd57fe769817895e63b5dbe925783a9c8d091fa8e'


def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2)


def validate(record):
    for path, sha in record['source_pins'].items():
        if digest(ROOT / path) != sha:
            raise RuntimeError(f'source changed: {path}')
    if digest(ROOT / record['argv']['baseline'][0]) != record['binary_sha256']:
        raise RuntimeError('binary changed')
    for item in record['inputs'].values():
        path = ROOT / item['path']
        if digest(path) != item['sha256'] or path.stat().st_size != item['bytes']:
            raise RuntimeError('raw input changed')


def freeze():
    old = json.loads(PREVIOUS.read_text())
    pins = {}
    for folder in ('src', 'include', 'apps/native'):
        for path in sorted((ROOT / folder).rglob('*')):
            if path.is_file() and path.suffix in ('.cpp', '.hpp', '.h'):
                pins[str(path.relative_to(ROOT))] = digest(path)
    for name in ('CMakeLists.txt', 'tests/CMakeLists.txt',
                 'scripts/run_phase563_rotation_rate.py'):
        pins[name] = digest(ROOT / name)
    commands = {}
    for stage in ('baseline', 'candidate'):
        argv = list(old['argv'])
        for flag, filename in (('--out', 'opaque_solution_output.csv'),
                               ('--summary-json', 'native_summary.json')):
            argv[argv.index(flag)+1] = str((DIRECTORY / stage / filename).relative_to(ROOT))
        if stage == 'candidate':
            argv.append('--native-doppler-rotation-rate')
        commands[stage] = argv
    record = dict(phase=563, argv=commands, source_pins=pins, inputs=old['inputs'],
                  binary_sha256=digest(ROOT / commands['baseline'][0]),
                  expected_baseline_sha256=EXPECTED, previous_manifest_sha256=digest(PREVIOUS),
                  scoring=False, truth_reads=0, saved_position_inputs=False)
    validate(record)
    DIRECTORY.mkdir(parents=True, exist_ok=False)
    write(DIRECTORY / 'manifest.json', record)
    print('Frozen baseline and one fixed candidate; no accuracy reads.', flush=True)


def run(stage):
    record = json.loads((DIRECTORY / 'manifest.json').read_text())
    validate(record)
    if stage == 'candidate':
        done = json.loads((DIRECTORY / 'baseline/completed.json').read_text())
        if done['return_code'] != 0 or done.get('output_sha256') != EXPECTED or not done.get('pins_verified'):
            raise RuntimeError('disabled baseline gate failed')
        if digest(DIRECTORY / 'baseline/opaque_solution_output.csv') != EXPECTED:
            raise RuntimeError('baseline output changed')
    directory = DIRECTORY / stage
    directory.mkdir(exist_ok=False)
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = '/home/sasaki/.local/lib:' + env.get('LD_LIBRARY_PATH', '')
    start = time.monotonic()
    with (directory / 'stdout.log').open('x') as out, (directory / 'stderr.log').open('x') as err:
        process = subprocess.Popen(record['argv'][stage], cwd=ROOT, env=env, stdout=out, stderr=err)
        write(directory / 'started.json', dict(pid=process.pid))
        print(json.dumps(dict(stage=stage, pid=process.pid)), flush=True)
        code = process.wait()
    done = dict(return_code=code, elapsed_seconds=time.monotonic()-start)
    if code == 0:
        done['output_sha256'] = digest(directory / 'opaque_solution_output.csv')
    try:
        validate(record)
        done['pins_verified'] = True
    finally:
        write(directory / 'completed.json', done)
    print(json.dumps(done), flush=True)
    if code != 0 or (stage == 'baseline' and done['output_sha256'] != EXPECTED):
        raise RuntimeError('native run or baseline invariance failed')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('freeze', 'baseline', 'candidate'))
    action = parser.parse_args().action
    freeze() if action == 'freeze' else run(action)
