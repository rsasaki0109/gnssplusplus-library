"""Single predeclared raw-only joint residual-ionosphere experiment, no scoring."""
import json
import os
import subprocess
import time

from run_phase486_nhc_monitor import ROOT, digest
from verify_phase535_ionosphere_monitor import main as verify_disabled


def main():
    # Fail closed until the current-binary disabled replay has passed.
    verify_disabled()
    previous = ROOT / 'output/smartphone-r5/phase535-h-ionosphere-monitor-v1/mtv-h/manifest.json'
    old = json.loads(previous.read_text())
    argv = list(old['argv'])
    assert '--native-joint-ionosphere' not in argv
    argv += ['--native-joint-ionosphere', '3', '0.02', '1.5']
    directory = ROOT / 'output/smartphone-r5/phase536-h-joint-ionosphere-v1/mtv-h'
    for flag, filename in (('--out', 'opaque_solution_output.csv'),
                           ('--summary-json', 'native_summary.json')):
        argv[argv.index(flag) + 1] = str((directory / filename).relative_to(ROOT))
    pins = dict(old['source_pins'])
    for name in ('scripts/run_phase536_joint_ionosphere.py',
                 'scripts/verify_phase536_joint_ionosphere.py',
                 'tests/test_joint_ionosphere_verifier.py',
                 'scripts/verify_phase535_ionosphere_monitor.py',
                 'docs/use_cases/records/smartphone_r5_phase536_joint_preregister_v1.md'):
        pins[name] = digest(ROOT / name)
    record = dict(phase=536, argv=argv, source_pins=pins, inputs=old['inputs'],
                  binary_sha256=old['binary_sha256'],
                  previous_manifest_sha256=digest(previous),
                  priors=dict(anchor_sigma_m=3.0, density_m_sqrt_s=0.02, max_gap_s=1.5),
                  scoring=False, saved_position_inputs=False)
    directory.mkdir(parents=True, exist_ok=False)
    with (directory / 'manifest.json').open('x') as handle:
        json.dump(record, handle, indent=2)
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = '/home/sasaki/.local/lib:' + env.get('LD_LIBRARY_PATH', '')
    start = time.monotonic()
    with (directory / 'stdout.log').open('x') as out, (directory / 'stderr.log').open('x') as err:
        process = subprocess.Popen(argv, cwd=ROOT, env=env, stdout=out, stderr=err)
        with (directory / 'started.json').open('x') as handle:
            json.dump(dict(pid=process.pid), handle)
        print(json.dumps(dict(phase=536, pid=process.pid)), flush=True)
        code = process.wait()
    result = dict(return_code=code, elapsed_seconds=time.monotonic()-start)
    if code == 0:
        result['output_sha256'] = digest(directory / 'opaque_solution_output.csv')
    with (directory / 'completed.json').open('x') as handle:
        json.dump(result, handle, indent=2)
    print(json.dumps(result), flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
