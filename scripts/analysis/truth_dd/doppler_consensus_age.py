"""doppler_consensus_age.py DATA_ROOT RUN debug.csv pos

Outcome of the opt-in fixed Doppler consensus check by the age of its track
(time since the last accepted FIX re-anchored it). For every epoch where the
check measured a distance, the FIX candidate is labelled correct (3D <= 0.5 m
against the reference) or wrong; for correct candidates the measured distance
is close to the track's own error. The reference is used for labels only.

DATA_ROOT/RUN/reference.csv, e.g. data/PPC-Dataset/tokyo run1.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

from truth_dd_common import load, read_reference_csv

BINS = (0, 2, 5, 15, 30, 60, 120, 600, math.inf)


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def main():
    data_root, run, debug_csv, pos = sys.argv[1:5]
    ref = {round(r.tow, 3): r for r in read_reference_csv(Path(data_root) / run / 'reference.csv')}
    _, rows = load(run, pos, debug_csv, data_root=data_root)
    table = {f'[{lo},{hi})': {'correct_accepted': 0, 'correct_rejected': 0, 'wrong_accepted': 0,
                              'wrong_rejected': 0, 'track_error_from_correct_p50_p95': []}
             for lo, hi in zip(BINS[:-1], BINS[1:])}
    longest, current, start = (0, None), 0, None
    for tow in sorted(rows):
        r = rows[tow]
        rejected = r['reject_reason'] == 'fixed_doppler_consensus'
        if rejected:
            start = tow if current == 0 else start
            current += 1
            longest = max(longest, (current, start), key=lambda x: x[0])
        elif r['status'] == '4':
            current = 0
        distance = num(r['fixed_candidate_doppler_consensus_distance_m'])
        candidate = np.array([num(r[f'fixed_candidate_{a}_m']) for a in 'xyz'])
        truth = ref.get(tow)
        if not math.isfinite(distance) or truth is None or not np.all(np.isfinite(candidate)):
            continue
        age = num(r['fixed_candidate_history_dt_s'])
        if not math.isfinite(age):
            continue
        key = next(f'[{lo},{hi})' for lo, hi in zip(BINS[:-1], BINS[1:]) if lo <= age < hi)
        error = float(np.linalg.norm(candidate - truth.ecef))
        label = 'correct' if error <= 0.5 else 'wrong'
        table[key][f'{label}_{"rejected" if rejected else "accepted"}'] += 1
        if error <= 0.2:
            table[key]['track_error_from_correct_p50_p95'].append(distance)
    for v in table.values():
        a = v['track_error_from_correct_p50_p95']
        v['track_error_from_correct_p50_p95'] = (
            [round(float(x), 2) for x in np.percentile(a, (50, 95))] if a else None)
    print(json.dumps({'run': run, 'by_track_age_s': table,
                      'longest_reject_run_without_fix': {'epochs': longest[0], 'start_tow': longest[1]}},
                     indent=1))


if __name__ == '__main__':
    main()
