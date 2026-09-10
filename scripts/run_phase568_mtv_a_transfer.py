"""Current raw-only baseline on previously evaluated MTV-A; no scoring."""
import json
import os
import subprocess
import time
from run_phase486_nhc_monitor import ROOT, digest

DIRECTORY = ROOT / 'output/smartphone-r5/phase568-mtv-a-transfer-v1'


def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2)


def main():
    authority = ROOT / 'docs/use_cases/records/smartphone_r5_phase144_telemetry_serializer_raw_authorization_v1.json'
    old = json.loads(authority.read_text())
    route = next(item for item in old['routes'] if item['target'] == 'MTV-A')
    inputs = route['raw_inputs']
    for item in inputs.values():
        path = ROOT / item['path']
        if digest(path) != item['sha256'] or path.stat().st_size != item['bytes']:
            raise ValueError('historical raw input mismatch')
    recipe = ROOT / 'output/smartphone-r5/phase563-rotation-rate-v1/manifest.json'
    previous = json.loads(recipe.read_text())
    argv = list(previous['argv']['baseline'])
    for flag, value in (
        ('--dataset-id', route['dataset_id']),
        ('--android-gnss', inputs['device_gnss.csv']['path']),
        ('--android-imu', inputs['device_imu.csv']['path']),
        ('--nav', inputs['brdc.nav']['path']),
        ('--out', str((DIRECTORY / 'opaque_solution_output.csv').relative_to(ROOT))),
        ('--summary-json', str((DIRECTORY / 'native_summary.json').relative_to(ROOT)))):
        argv[argv.index(flag)+1] = value
    # Same executable as the byte-identical H replay. No new solver option.
    if digest(ROOT / argv[0]) != previous['binary_sha256']:
        raise ValueError('qualified binary changed')
    pins = {name: digest(ROOT / name) for name in previous['source_pins']}
    pins['scripts/run_phase568_mtv_a_transfer.py'] = digest(Path(__file__))
    record = dict(phase=568, role='previously evaluated development route, not heldout',
                  dataset_id=route['dataset_id'], argv=argv, inputs=inputs,
                  binary_sha256=previous['binary_sha256'], source_pins=pins,
                  recipe_sha256=digest(recipe), input_authority_sha256=digest(authority),
                  truth_reads=0, scoring=False, saved_position_inputs=False,
                  policy='same H baseline flags; no route-specific tuning or output repair')
    DIRECTORY.mkdir(parents=True, exist_ok=False)
    write(DIRECTORY / 'manifest.json', record)
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = '/home/sasaki/.local/lib:' + env.get('LD_LIBRARY_PATH', '')
    start = time.monotonic()
    with (DIRECTORY / 'stdout.log').open('x') as out, (DIRECTORY / 'stderr.log').open('x') as err:
        process = subprocess.Popen(argv, cwd=ROOT, env=env, stdout=out, stderr=err)
        write(DIRECTORY / 'started.json', dict(pid=process.pid))
        print(json.dumps(dict(phase=568, pid=process.pid)), flush=True)
        code = process.wait()
    done = dict(return_code=code, elapsed_seconds=time.monotonic()-start)
    if code == 0:
        done['output_sha256'] = digest(DIRECTORY / 'opaque_solution_output.csv')
    done['pins_verified'] = (all(digest(ROOT/name) == sha for name,sha in pins.items())
        and digest(ROOT/argv[0]) == record['binary_sha256']
        and all(digest(ROOT/item['path']) == item['sha256'] for item in inputs.values()))
    write(DIRECTORY / 'completed.json', done)
    print(json.dumps(done), flush=True)
    return code if code else (0 if done['pins_verified'] else 1)


if __name__ == '__main__':
    from pathlib import Path
    raise SystemExit(main())
