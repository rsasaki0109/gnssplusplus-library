"""dispersive_share.py truth_dd.csv : how much of the slow DD carrier residual on the first
frequency is dispersive (ionosphere). Reference trajectory used for analysis only.

Per system with two carriers (GPS L1C/L2W, QZS L1C/L2X, GAL L1X/L5X, BDS L2I/L7I):
fixed reference satellite carrying both (most present, el >= 15), DD residuals
eps1, eps2 [m] after removing the arc integer. Arcs split on LLI or > 0.3 cycle
steps on either carrier. Ionosphere on carrier 1: I1 = (eps1 - eps2) / (gamma - 1),
since phase = rho - I + N lambda and I2 = gamma I1. Slow parts = 30 s centred
moving average; compare std(slow eps1), std(slow I1 term), std(slow eps1 + I1),
and the regression slope of slow eps1 on slow (-I1).
"""
import json
import sys

import numpy as np
import pandas as pd

F = {'G': ('L1C', 'L2W', 1575.42, 1227.60), 'J': ('L1C', 'L2X', 1575.42, 1227.60),
     'E': ('L1X', 'L5X', 1575.42, 1176.45), 'C': ('L2I', 'L7I', 1561.098, 1207.14)}
df = pd.read_csv(sys.argv[1])
df['sys'] = df['sat'].str[0]
df['tkey'] = np.round(df['tow'] * 10).astype(np.int64)
df['res'] = (df['rov_L'] - df['base_L']) - (df['rho_rov'] - df['rho_base']) / df['lambda']
out = {}
for s, (o1, o2, f1, f2) in F.items():
    g = df[df['sys'] == s]
    a = g[g['obs_type'] == o1].set_index(['tkey', 'sat'])
    b = g[g['obs_type'] == o2].set_index(['tkey', 'sat'])
    j = a.join(b, how='inner', lsuffix='1', rsuffix='2')
    j = j[j['el_deg1'] >= 15].reset_index()
    if j.empty:
        continue
    ref = j['sat'].value_counts().idxmax()
    r = j[j['sat'] == ref].set_index('tkey')
    o = j[(j['sat'] != ref) & j['tkey'].isin(r.index)].copy()
    rr = r.loc[o['tkey']]
    o['d1'] = rr['res1'].values - o['res1'].values
    o['d2'] = rr['res2'].values - o['res2'].values
    o['lli'] = ((o['rov_lli1'].values | o['base_lli1'].values | o['rov_lli2'].values | o['base_lli2'].values |
                 rr['rov_lli1'].values | rr['base_lli1'].values | rr['rov_lli2'].values | rr['base_lli2'].values) & 1)
    o = o.sort_values(['sat', 'tkey'])
    arc = []
    cur, prev = 0, None
    for sat, tk, d1, d2, lli in zip(o['sat'], o['tkey'], o['d1'], o['d2'], o['lli']):
        if prev is None or prev[0] != sat or tk - prev[1] > 1 or lli or abs(d1 - prev[2]) > 0.3 or abs(d2 - prev[3]) > 0.3:
            cur += 1
        arc.append(cur)
        prev = (sat, tk, d1, d2)
    o['arc'] = arc
    gamma = (f1 / f2) ** 2
    rows = []
    for _, a_ in o.groupby('arc'):
        if len(a_) < 900:
            continue
        e1 = (a_['d1'] - np.round(a_['d1'].median())).values * a_['lambda1'].values
        e2 = (a_['d2'] - np.round(a_['d2'].median())).values * a_['lambda2'].values
        I1 = (e1 - e2) / (gamma - 1)
        sm = lambda x: pd.Series(x).rolling(300, center=True, min_periods=150).mean().values
        s1, sI, sc = sm(e1), sm(-I1), sm(e1 + I1)
        m = np.isfinite(s1) & np.isfinite(sI) & np.isfinite(sc)
        s1, sI, sc = s1[m] - s1[m].mean(), sI[m] - sI[m].mean(), sc[m] - sc[m].mean()
        rows.append({'std_slow_eps1_mm': 1000 * s1.std(), 'std_slow_iono_mm': 1000 * sI.std(),
                     'std_slow_eps1_minus_iono_mm': 1000 * sc.std(),
                     'slope': float(np.dot(s1, sI) / max(np.dot(sI, sI), 1e-12)),
                     'corr': float(np.corrcoef(s1, sI)[0, 1]),
                     'mean_eps1_mm': 1000 * float(np.mean(e1)), 'mean_iono_mm': 1000 * float(np.mean(-I1))})
    if not rows:
        continue
    R = pd.DataFrame(rows)
    out[s] = {'ref': ref, 'arcs_ge_90s': len(R), **{k: round(float(R[k].median()), 3) for k in R.columns}}
print(json.dumps(out, indent=1))
json.dump(out, open(sys.argv[1] + '.iono.json', 'w'), indent=1)
