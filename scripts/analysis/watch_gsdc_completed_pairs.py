"""Audit completed pairs as they arrive; aggregate only a complete frozen cohort.

Never launches native inference or changes a frozen plan. On Windows an optional
runner handle makes a stopped batch fail closed instead of waiting forever.
"""
import argparse
import ctypes
from ctypes import wintypes
import json
from pathlib import Path
import subprocess
import sys
import time

from gsdc_development_plan_audit import audit_height_independence, provenance_hash
from run_gsdc_doppler_segment_diagnostic import sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--truth-root', type=Path, required=True)
    parser.add_argument('--progress-out', type=Path, required=True)
    parser.add_argument('--full-out', type=Path, required=True)
    parser.add_argument('--watch-pid', type=int)
    parser.add_argument('--once', action='store_true')
    args = parser.parse_args()
    plan_hash = sha(args.plan)
    plan = json.loads(args.plan.read_text())
    here = Path(__file__).parent
    sources = set()
    for case, entry in plan['runs'].items():
        audit_height_independence(plan, case)
        provenance_hash(args.plan, plan, case)
        sources.add(Path(entry.get('provenance_plan', args.plan)).parent / 'execution.done.json')
    handle = None
    if args.watch_pid is not None:
        api = ctypes.WinDLL('kernel32', use_last_error=True)
        api.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
        api.OpenProcess.restype = wintypes.HANDLE
        api.GetExitCodeProcess.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD)]
        api.CloseHandle.argtypes = [wintypes.HANDLE]
        handle = api.OpenProcess(0x1000, False, args.watch_pid)
        if not handle:
            raise RuntimeError('Cannot open the expected runner process')
    try:
        while True:
            assert sha(args.plan) == plan_hash, 'Frozen plan changed'
            changed, completed = False, 0
            for case, entry in plan['runs'].items():
                folders = {mode: Path(argv[argv.index('--out') + 1]).parent
                           for mode, argv in entry['argv'].items()}
                states = []
                for folder in folders.values():
                    path = folder / 'run.json'
                    state = json.loads(path.read_text())['state'] if path.exists() else 'not-started'
                    assert state != 'failed', f'{case}: selected native run failed'
                    states.append(state)
                if states != ['complete', 'complete']:
                    continue
                completed += 1
                output = folders['candidate'].parent / 'comparison.json'
                if output.exists():
                    continue
                flags = plan['candidate_flags']
                command = [sys.executable, str(here / 'compare_gsdc_doppler_diagnostic.py'),
                    '--control', str(folders['control']), '--candidate', str(folders['candidate']),
                    '--candidate-flag=' + flags[0], '--truth', str(args.truth_root / case / 'ground_truth.csv'),
                    '--out', str(output)]
                command += ['--additional-candidate-flag=' + flag for flag in flags[1:]]
                subprocess.run(command, check=True, stdout=subprocess.DEVNULL)
                changed = True
                print('New completed pair:', case, flush=True)
            if changed or not args.progress_out.exists():
                subprocess.run([sys.executable, str(here / 'record_gsdc_frozen_pairs_progress.py'),
                    '--plan', str(args.plan), '--out', str(args.progress_out)], check=True)
            if completed == len(plan['runs']) and all(p.exists() for p in sources):
                subprocess.run([sys.executable, str(here / 'score_gsdc_frozen_pairs.py'), str(args.plan),
                    '--truth-root', str(args.truth_root), '--out', str(args.full_out)], check=True)
                return
            if args.once:
                print(f'{completed}/{len(plan["runs"])} pairs complete; full aggregate pending')
                return
            if handle:
                code = wintypes.DWORD()
                if not api.GetExitCodeProcess(handle, ctypes.byref(code)):
                    raise ctypes.WinError(ctypes.get_last_error())
                if code.value != 259:
                    raise RuntimeError('Runner stopped before selected cohort and source manifests completed')
            time.sleep(30)
    finally:
        if handle:
            api.CloseHandle(handle)


if __name__ == '__main__':
    main()
