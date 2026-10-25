"""code_residual_correlation.py truth_dd.csv : truth-geometry DD code residuals against the solver's
code model (varerr a = b = 0.3 m, DD = sat + ref). Reference trajectory used for
analysis only.

Per (system, obs type), fixed most-present reference (el >= 15). eP = DD(P) -
DD(range) [m]. Reports amplitude vs model by elevation, correlation time on arcs
>= 60 s, and the white-noise z of the running mean after L s (what a code-driven
float ambiguity would carry) together with that mean in cycles.
"""
import json
import sys
from collections import defaultdict

import numpy as np
import pandas as pd

df = pd.read_csv(sys.argv[1])
df['sys'] = df['sat'].str[0]
df['tkey'] = np.round(df['tow'] * 10).astype(np.int64)
df['resP'] = (df['rov_P'] - df['base_P']) - (df['rho_rov'] - df['rho_base'])


def varerr(el_deg, a=0.3, b=0.3):
    s = np.maximum(np.sin(np.radians(el_deg)), 0.1)
    return 2.0 * (a * a + b * b / (s * s))


parts = []
for (sysc, ot), g in df.groupby(['sys', 'obs_type']):
    hi = g[g['el_deg'] >= 15]
    if hi.empty:
        continue
    ref = hi['sat'].value_counts().idxmax()
    r = g[g['sat'] == ref].set_index('tkey')
    o = g[(g['sat'] != ref) & (g['el_deg'] >= 15)]
    o = o[o['tkey'].isin(r.index)]
    rr = r.loc[o['tkey']]
    parts.append(pd.DataFrame({'tkey': o['tkey'].values, 'sig': f'{sysc} {ot}', 'sat': o['sat'].values,
                               'eP': rr['resP'].values - o['resP'].values, 'lam': o['lambda'].values,
                               'el': o['el_deg'].values, 'el_ref': rr['el_deg'].values,
                               'snr': o['rov_snr'].values}))
dd = pd.concat(parts, ignore_index=True).sort_values(['sig', 'sat', 'tkey']).reset_index(drop=True)
dd['model'] = np.sqrt(varerr(dd['el']) + varerr(dd['el_ref']))
dd['ratio'] = dd['eP'] / dd['model']
out = {'rows': int(len(dd)),
       'abs_eP_m_p50_p95': [round(float(x), 2) for x in dd['eP'].abs().quantile([.5, .95])],
       'model_m_p50': round(float(dd['model'].median()), 2),
       'robust_std_ratio': round(float(1.4826 * (dd['ratio'] - dd['ratio'].median()).abs().median()), 2),
       'std_ratio': round(float(dd['ratio'].std()), 2),
       'share_abs_ratio_gt3': round(float((dd['ratio'].abs() > 3).mean()), 3),
       'by_elevation': {}}
for lo, hi in ((15, 30), (30, 45), (45, 60), (60, 90)):
    m = (dd['el'] >= lo) & (dd['el'] < hi)
    x = dd.loc[m, 'ratio']
    out['by_elevation'][f'{lo}-{hi}'] = {'rows': int(m.sum()), 'std_ratio': round(float(x.std()), 2),
                                         'abs_eP_m_p50': round(float(dd.loc[m, 'eP'].abs().median()), 2)}
# arcs: continuous presence
arc = (dd['sig'] != dd['sig'].shift()) | (dd['sat'] != dd['sat'].shift()) | (dd['tkey'] - dd['tkey'].shift() > 1)
dd['arc'] = arc.cumsum()
taus, zs, cyc = [], defaultdict(list), defaultdict(list)
for _, g in dd.groupby('arc'):
    if len(g) < 600:
        continue
    e = g['eP'].values
    v = e - e.mean()
    n = len(v)
    acf = np.correlate(v, v, mode='full')[n - 1:n - 1 + 1200] / (v.var() * n)
    zero = np.argmax(acf <= 0) if np.any(acf <= 0) else len(acf)
    taus.append(0.1 * (0.5 + acf[1:zero].sum()))
    sm = g['model'].values
    for L in (10, 30, 60):
        k = L * 10
        if n >= k:
            zs[L].append(abs(np.mean(e[:k])) / (np.median(sm[:k]) / np.sqrt(k)))
            cyc[L].append(abs(np.mean(e[:k])) / g['lam'].iloc[0])
out['arcs_ge_60s'] = len(taus)
out['tau_s_p25_p50_p75'] = [round(float(x), 1) for x in np.percentile(taus, [25, 50, 75])] if taus else None
out['running_mean_abs_z'] = {L: round(float(np.median(v)), 1) for L, v in zs.items()}
out['running_mean_cycles_p50_p90'] = {L: [round(float(x), 2) for x in np.percentile(v, [50, 90])] for L, v in cyc.items()}
print(json.dumps(out, indent=1))
json.dump(out, open(sys.argv[1] + '.code.json', 'w'), indent=1)
