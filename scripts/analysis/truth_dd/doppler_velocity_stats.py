"""doppler_velocity_stats.py DIR RUN [RUN ...]

Summarize doppler_residuals output (DIR/<run>-obs.csv, DIR/<run>-epoch.csv):

- |v - v_ref| of the library Doppler solve and of the robust variant, both at
  the reference position;
- per-signal |range-rate residual| against the reference velocity;
- drift of a track integrated from a reference anchor with the previous
  epoch's velocity, as processRTKEpoch() propagates the Doppler continuity
  track (a gap over 2 s ends the window).

The reference is used for analysis only.
"""
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import pandas as pd

WINDOWS_S = (1, 2, 5, 15, 60, 300)


def q(a, p=(50, 95, 99)):
    a = np.asarray(a, dtype=float)
    a = a[np.isfinite(a)]
    return [round(float(x), 3) for x in np.percentile(a, p)] if len(a) else None


def drift(e, key, window):
    t = e['tow'].to_numpy()
    v = e[[f'{key}_vx', f'{key}_vy', f'{key}_vz']].to_numpy()
    p = e[['ref_x', 'ref_y', 'ref_z']].to_numpy()
    ok = e[f'{key}_ok'].to_numpy() == 1
    out = []
    for i0 in range(0, len(t), 25):
        if not ok[i0]:
            continue
        pos, vel, last_v, i = p[i0].copy(), v[i0], t[i0], i0
        while i + 1 < len(t) and t[i + 1] - t[i0] <= window + 1e-6:
            dt = t[i + 1] - t[i]
            if dt > 2.0 or t[i + 1] - last_v > 2.0:
                break
            pos = pos + vel * dt
            i += 1
            if ok[i]:
                vel, last_v = v[i], t[i]
        if abs(t[i] - t[i0] - window) < 1e-6:
            out.append(float(np.linalg.norm(pos - p[i])))
    return q(out, (50, 95))


def main():
    root = Path(sys.argv[1])
    summary, signals = {}, defaultdict(list)
    for run in sys.argv[2:]:
        e = pd.read_csv(root / f'{run}-epoch.csv')
        ref_v = e[['ref_vx', 'ref_vy', 'ref_vz']].to_numpy()
        s = {'epochs': len(e)}
        for key in ('lib', 'rob'):
            err = np.linalg.norm(e[[f'{key}_vx', f'{key}_vy', f'{key}_vz']].to_numpy() - ref_v, axis=1)
            s[f'{key}_ok'] = int((e[f'{key}_ok'] == 1).sum())
            s[f'{key}_err_p50_p95_p99'] = q(err[e[f'{key}_ok'].to_numpy() == 1])
            s[f'{key}_drift_p50_p95'] = {w: drift(e, key, w) for w in WINDOWS_S}
        s['rob_dropped_mean'] = round(float(e.loc[e['rob_ok'] == 1, 'rob_dropped'].mean()), 3)
        summary[run] = s
        o = pd.read_csv(root / f'{run}-obs.csv', usecols=['sys', 'signal', 'resid_mps'])
        for (sys_, sig), g in o.groupby(['sys', 'signal']):
            signals[(int(sys_), int(sig))].append(g['resid_mps'].abs().to_numpy())
    per_signal = {}
    for k in sorted(signals):
        a = np.concatenate(signals[k])
        per_signal[f'sys{k[0]}/sig{k[1]}'] = {'n': int(len(a)), 'p50_p95': q(a, (50, 95)),
                                              'frac_gt_1mps': round(float(np.mean(a > 1.0)), 3)}
    print(json.dumps({'runs': summary, 'per_signal': per_signal}, indent=1))


if __name__ == '__main__':
    main()
