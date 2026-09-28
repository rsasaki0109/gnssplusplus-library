"""Replay a frozen native run with and without main Doppler; no truth inputs."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--control-run', type=Path, required=True)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--candidate-flag', default='--native-phase213-main-doppler')
    parser.add_argument('--additional-candidate-flag', action='append', default=[])
    parser.add_argument('--candidate-name', default='doppler')
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--candidate-only', action='store_true',
                        help='Run only the candidate for report-only instrumentation replays.')
    selection.add_argument('--control-only', action='store_true')
    selection.add_argument('--candidate-first', action='store_true',
                           help='Run candidate then control; stop if the candidate fails.')
    args = parser.parse_args()
    source = json.loads(args.control_run.read_text())
    assert source['returncode'] == 0
    base = source['argv'][1:]
    candidate_flags = [args.candidate_flag] + args.additional_candidate_flag
    assert len(set(candidate_flags)) == len(candidate_flags)
    assert all(flag.startswith('--') and flag not in base for flag in candidate_flags)
    assert args.candidate_name not in ['control', '.', '..']
    assert Path(args.candidate_name).name == args.candidate_name
    # Pin every existing input file named on the command line.
    inputs = {v: sha(v) for v in base if not v.startswith('--')
              and Path(v).is_file() and v not in
              [base[base.index(k) + 1] for k in ['--out', '--summary-json']]}
    args.output_dir.mkdir(parents=True, exist_ok=False)
    binary_hash = sha(args.binary)
    env = os.environ.copy()
    env['PATH'] = 'E:/gtsam/install/bin;C:/vcpkg/installed/x64-windows/bin;' + env['PATH']
    modes = (['control'] if args.control_only else [args.candidate_name]
             if args.candidate_only else ['control', args.candidate_name])
    if args.candidate_first:
        modes = [args.candidate_name, 'control']
    for mode in modes:
        folder = args.output_dir / mode
        folder.mkdir()
        argv = [str(args.binary.resolve())] + base.copy()
        for flag, name in [('--out', 'solution.csv'), ('--summary-json', 'summary.json')]:
            argv[argv.index(flag) + 1] = str((folder / name).resolve())
        if mode != 'control':
            argv.extend(candidate_flags)
        assert sha(args.binary) == binary_hash
        assert all(sha(path) == digest for path, digest in inputs.items())
        record = dict(argv=argv, binary_sha256=binary_hash, inputs_sha256=inputs,
                      source_run_sha256=sha(args.control_run), state='running')
        started = time.time()
        with (folder / 'stdout.log').open('w') as out, (folder / 'stderr.log').open('w') as err:
            process = subprocess.Popen(argv, stdout=out, stderr=err, env=env)
            record['pid'] = process.pid
            (folder / 'run.json').write_text(json.dumps(record, indent=2))
            rc = process.wait()
        record.update(returncode=rc, wall_s=time.time() - started,
                      state='complete' if rc == 0 else 'failed')
        if rc == 0:
            record['solution_sha256'] = sha(folder / 'solution.csv')
            record['summary_sha256'] = sha(folder / 'summary.json')
        (folder / 'run.json').write_text(json.dumps(record, indent=2))
        print(mode, rc, record['wall_s'], flush=True)
        if rc:
            raise SystemExit(rc)


if __name__ == '__main__':
    main()
