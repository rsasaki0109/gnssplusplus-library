"""maxz_truth_rank.py truth_dd.csv debug.csv pos drive

Is the solver's max-|z| pair the one whose truth code error is largest?
For every accepted AR fix (hold excluded, 3D <= 0.5 m vs the reference, which is
used for labels and analysis only) with a max-|z| pair REF-SAT/Lf and lock n:
per-satellite between-receiver code error e(t) = SD(P) - SD(range), minus the
per-epoch median of its system (receiver clock); over the last n rover epochs,
c(pair) = |mean(e_SAT - e_REF)|. The max-|z| pair's c is ranked among all pairs
with the same REF visible over the window (percentile 1 = largest). The same is
done with the phase residual (arc-demeaned) for comparison.
"""
import json
import sys

import numpy as np
import pandas as pd

from truth_dd_common import load

ddp, dbg, pos, ds = sys.argv[1:5]
df = pd.read_csv(ddp, usecols=['tow', 'sat', 'obs_type', 'lambda', 'rov_L', 'base_L', 'rov_P', 'base_P', 'rho_rov',
                               'rho_base', 'el_deg', 'rov_lli', 'base_lli'])
first = {'G': 'L1C', 'J': 'L1C', 'E': 'L1X', 'C': 'L2I'}
df = df[df.apply(lambda r: first.get(r['sat'][0]) == r['obs_type'], axis=1)].copy()
df['sys'] = df['sat'].str[0]
df['tkey'] = np.round(df['tow'] * 10).astype(np.int64)
df['eP'] = (df['rov_P'] - df['base_P']) - (df['rho_rov'] - df['rho_base'])
df['eP'] -= df.groupby(['tkey', 'sys'])['eP'].transform('median')
df['eL'] = ((df['rov_L'] - df['base_L']) * df['lambda']) - (df['rho_rov'] - df['rho_base'])
df['eL'] -= df.groupby(['tkey', 'sys'])['eL'].transform('median')
P = df.pivot_table(index='tkey', columns='sat', values='eP')
L = df.pivot_table(index='tkey', columns='sat', values='eL')

err, D = load(ds, pos, dbg)
ranks_c, ranks_l = [], []
null_top, null_p50 = [], []
for t, (s, e) in err.items():
    r = D.get(t)
    if s != 4 or r is None or r['hold_fix_applied'] == '1' or r['lambda_shadow_solved'] != '1' or e > 0.5:
        continue
    pair = r['lambda_shadow_max_z_pair']
    if not pair or '-' not in pair:
        continue
    ref, sat = pair.split('/')[0].split('-')
    n = int(float(r['lambda_shadow_max_z_lock_count'] or 0))
    tk = int(round(t * 10))
    if n < 10 or ref not in P.columns or sat not in P.columns:
        continue
    w = P.loc[tk - n + 1:tk]
    wl = L.loc[tk - n + 1:tk]
    if len(w) < 0.8 * n or w[ref].isna().mean() > 0.2:
        continue
    cand = [c for c in w.columns if c != ref and c[0] == ref[0] and w[c].notna().mean() > 0.8]
    if sat not in cand or len(cand) < 3:
        continue
    cc = {c: abs(np.nanmean(w[c] - w[ref])) for c in cand}
    # phase: change of the SD residual over the window (integer offsets cancel inside an arc)
    cl = {c: abs(np.nanmean((wl[c] - wl[ref]) - np.nanmean((wl[c] - wl[ref]).iloc[:max(1, n // 10)]))) for c in cand}
    order = sorted(cand, key=lambda c: cc[c])
    ranks_c.append((order.index(sat) + 1) / len(order))
    k = len(order)
    null_top.append(sum(1 for i in range(1, k + 1) if i / k > 2 / 3) / k)
    null_p50.append((k + 1) / (2 * k))
    order_l = sorted(cand, key=lambda c: cl[c])
    ranks_l.append((order_l.index(sat) + 1) / len(order_l))
res = {'drive': ds, 'correct_ar_fixes_used': len(ranks_c),
       'code_rank_percentile_p50': round(float(np.median(ranks_c)), 2) if ranks_c else None,
       'code_rank_top_third_share': round(float(np.mean(np.array(ranks_c) > 2 / 3)), 2) if ranks_c else None,
       'phase_rank_percentile_p50': round(float(np.median(ranks_l)), 2) if ranks_l else None,
       'phase_rank_top_third_share': round(float(np.mean(np.array(ranks_l) > 2 / 3)), 2) if ranks_l else None,
       'null_top_third_share': round(float(np.mean(null_top)), 2) if null_top else None,
       'null_rank_mean': round(float(np.mean(null_p50)), 2) if null_p50 else None,
       'code_rank_mean': round(float(np.mean(ranks_c)), 2) if ranks_c else None,
       'phase_rank_mean': round(float(np.mean(ranks_l)), 2) if ranks_l else None}
print(json.dumps(res, indent=1))
