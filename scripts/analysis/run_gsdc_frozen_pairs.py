"""Execute frozen two- or three-arm comparisons, at most two workers per runner."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import os
from pathlib import Path
import subprocess
import time

from run_gsdc_doppler_segment_diagnostic import sha


def write(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n', encoding='utf-8')
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plan', type=Path)
    parser.add_argument('--workers', type=int, choices=[1, 2], default=2)
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text())
    plan_hash = sha(args.plan)
    arm_order = plan.get('arm_order', ['control', 'candidate'])
    assert arm_order in [['control', 'candidate'], ['control', 'clock_only', 'candidate']]
    assert plan['runs'], 'empty comparison plan'
    assert all(set(entry['argv']) == set(arm_order) for entry in plan['runs'].values()), 'arm mismatch'
    assert sha(plan['binary']) == plan['binary_sha256']
    # Create exclusively: an existing launch is inspected, never restarted.
    with (args.plan.parent / 'execution.started.json').open('x', encoding='utf-8') as stream:
        json.dump(dict(pid=os.getpid(), workers=args.workers,
                       plan_sha256=plan_hash, started_unix=time.time()), stream)
    env = os.environ.copy()
    env['PATH'] = 'E:/gtsam/install/bin;C:/vcpkg/installed/x64-windows/bin;' + env['PATH']

    def run_case(item):
        case, entry = item
        outcomes = []
        for mode in arm_order:
            command = entry['argv'][mode]
            folder = Path(command[command.index('--out') + 1]).parent
            folder.mkdir(parents=True, exist_ok=False)
            record = dict(dataset_id=case, argv=command, state='preflight',
                          binary_sha256=plan['binary_sha256'],
                          inputs_sha256=entry['inputs_sha256'], source_run_sha256=plan_hash)
            try:
                assert sha(args.plan) == plan_hash
                assert sha(command[0]) == plan['binary_sha256']
                assert all(sha(p) == digest for p, digest in entry['inputs_sha256'].items())
                started = time.time()
                with (folder / 'stdout.log').open('w') as out, (folder / 'stderr.log').open('w') as err:
                    process = subprocess.Popen(command, stdout=out, stderr=err, env=env)
                    record.update(state='running', pid=process.pid, started_unix=started)
                    write(folder / 'run.json', record)
                    rc = process.wait()
                record.update(returncode=rc, wall_s=time.time() - started,
                              state='complete' if rc == 0 else 'failed')
                if rc == 0:
                    record['solution_sha256'] = sha(folder / 'solution.csv')
                    record['summary_sha256'] = sha(folder / 'summary.json')
            except Exception as exc:
                record.update(state='failed', failure=repr(exc))
            write(folder / 'run.json', record)
            outcomes.append(dict(mode=mode, state=record['state'], run=str(folder / 'run.json')))
            print(case, mode, record['state'], record.get('wall_s'), flush=True)
            if record['state'] != 'complete':
                break
        return case, outcomes

    outcomes = {}
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = [pool.submit(run_case, item) for item in plan['runs'].items()]
        for future in as_completed(futures):
            case, result = future.result()
            outcomes[case] = result
            write(args.plan.parent / 'execution.progress.json', outcomes)
    complete = len(outcomes) == len(plan['runs']) and all(
        len(items) == len(arm_order) and all(item['state'] == 'complete' for item in items)
        for items in outcomes.values())
    write(args.plan.parent / 'execution.done.json', dict(
        plan_sha256=plan_hash, native_execution_complete=complete, scored=False, runs=outcomes))
    if not complete:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
