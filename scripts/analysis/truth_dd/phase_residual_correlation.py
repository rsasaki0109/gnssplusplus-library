"""phase_residual_correlation.py truth_dd.csv [out.json]

Time correlation of truth-geometry DD carrier residuals at the rover rate
(reference trajectory used for analysis only).

1. Per (system, obs type): fixed reference = most-present satellite (el >= 15),
   DD residual d = DD(phase) - DD(range)/lambda in cycles.
2. Arcs: rover epochs <= 0.15 s apart, no LLI bit 0, |step| <= 0.3 cycle.
   eps = (d - round(median d)) * lambda  [m].
3. Per epoch, a common 3D offset (reference-trajectory error) is fitted by least
   squares over all DD rows (|eps| < 0.1 m, >= 8 rows) and removed: r = eps - g.dx.
4. Per arc >= 60 s: std, integrated correlation time, and the white-noise
   z of the running mean of r after L seconds of lock, with the solver's model
   sigma (varerr: 2 (a^2 + b^2 / sin^2 el), a = b = 2 mm, DD = sat + ref).
"""
import json
import sys
from collections import defaultdict

import numpy as np
import pandas as pd

path = sys.argv[1]
out_path = sys.argv[2] if len(sys.argv) > 2 else path + '.corr.json'
df = pd.read_csv(path)
df['sys'] = df['sat'].str[0]
df['tkey'] = np.round(df['tow'] * 10).astype(np.int64)
df['res'] = (df['rov_L'] - df['base_L']) - (df['rho_rov'] - df['rho_base']) / df['lambda']


def varerr(el_deg, a=0.002, b=0.002):
    s = np.maximum(np.sin(np.radians(el_deg)), 0.1)
    return 2.0 * (a * a + b * b / (s * s))


rows = []  # DD rows
for (sysc, ot), g in df.groupby(['sys', 'obs_type']):
    hi = g[g['el_deg'] >= 15]
    if hi.empty:
        continue
    ref = hi['sat'].value_counts().idxmax()
    r = g[g['sat'] == ref].set_index('tkey')
    o = g[(g['sat'] != ref) & (g['el_deg'] >= 15)]
    o = o[o['tkey'].isin(r.index)]
    rr = r.loc[o['tkey']]
    dd = pd.DataFrame({
        'tkey': o['tkey'].values, 'sig': f'{sysc} {ot}', 'sat': o['sat'].values, 'ref': ref,
        'lam': o['lambda'].values,
        'd': rr['res'].values - o['res'].values,
        'lli': ((o['rov_lli'].values | o['base_lli'].values | rr['rov_lli'].values | rr['base_lli'].values) & 1),
        'el': o['el_deg'].values, 'el_ref': rr['el_deg'].values, 'snr': o['rov_snr'].values,
        'gx': o['ux'].values - rr['ux'].values, 'gy': o['uy'].values - rr['uy'].values,
        'gz': o['uz'].values - rr['uz'].values,
    })
    rows.append(dd)
dd = pd.concat(rows, ignore_index=True).sort_values(['sig', 'sat', 'tkey']).reset_index(drop=True)

# arcs
arc_id = np.zeros(len(dd), dtype=np.int64)
cur = 0
prev = None
for i, (sig, sat, tk, d, lli) in enumerate(zip(dd['sig'], dd['sat'], dd['tkey'], dd['d'], dd['lli'])):
    if prev is None or prev[0] != sig or prev[1] != sat or tk - prev[2] > 1 or lli or abs(d - prev[3]) > 0.3:
        cur += 1
    arc_id[i] = cur
    prev = (sig, sat, tk, d)
dd['arc'] = arc_id
med = dd.groupby('arc')['d'].transform('median')
dd['eps'] = (dd['d'] - np.round(med)) * dd['lam']

# per-epoch common offset
dd['r'] = np.nan
for tk, g in dd.groupby('tkey'):
    m = g['eps'].abs() < 0.1
    if m.sum() < 8:
        continue
    G = g.loc[m, ['gx', 'gy', 'gz']].values
    y = g.loc[m, 'eps'].values
    x, *_ = np.linalg.lstsq(G, y, rcond=None)
    res = y - G @ x
    s = 1.4826 * np.median(np.abs(res)) + 1e-4
    keep = np.abs(res) < 5 * s
    if keep.sum() >= 8:
        x, *_ = np.linalg.lstsq(G[keep], y[keep], rcond=None)
    dd.loc[g.index, 'r'] = g['eps'].values - g[['gx', 'gy', 'gz']].values @ x

dd['sig_model'] = np.sqrt(varerr(dd['el']) + varerr(dd['el_ref']))
stats = defaultdict(list)
zL = {10: [], 30: [], 60: [], 120: []}
zL_raw = {10: [], 30: [], 60: [], 120: []}
for arc, g in dd.groupby('arc'):
    if len(g) < 600:
        continue
    r = g['r'].values
    if np.isnan(r).mean() > 0.05:
        continue
    r = pd.Series(r).interpolate(limit_direction='both').values
    e = g['eps'].values
    sm = g['sig_model'].values
    v = r - r.mean()
    n = len(v)
    acf = np.correlate(v, v, mode='full')[n - 1:n - 1 + 1200] / (v.var() * n)
    zero = np.argmax(acf <= 0) if np.any(acf <= 0) else len(acf)
    tau = 0.1 * (0.5 + acf[1:zero].sum())
    stats['sig'].append(g['sig'].iloc[0])
    stats['std_r'].append(float(np.std(r)))
    stats['std_eps'].append(float(np.std(e)))
    stats['model'].append(float(np.median(sm)))
    stats['tau_s'].append(float(tau))
    stats['el'].append(float(g['el'].median()))
    stats['len_s'].append(n / 10)
    for L in zL:
        k = int(L * 10)
        if n >= k:
            # running mean after L s of lock, against the white-noise sigma of a mean of k samples
            zL[L].append(float(np.mean(r[:k]) / (np.median(sm[:k]) / np.sqrt(k))))
            zL_raw[L].append(float(np.mean(e[:k]) / (np.median(sm[:k]) / np.sqrt(k))))

S = pd.DataFrame(stats)
summary = {
    'arcs_ge_60s': int(len(S)),
    'std_r_mm_p50': round(1000 * S['std_r'].median(), 2),
    'std_eps_mm_p50': round(1000 * S['std_eps'].median(), 2),
    'model_sigma_mm_p50': round(1000 * S['model'].median(), 2),
    'std_ratio_p50': round(float((S['std_r'] / S['model']).median()), 2),
    'tau_s_p25_p50_p75': [round(float(x), 1) for x in S['tau_s'].quantile([.25, .5, .75])],
    'predicted_white_overconfidence_p50': round(float(((S['std_r'] / S['model']) ** 2 * 2 * S['tau_s'] * 10).median()), 1),
    'abs_z_running_mean_postfit': {L: {'n': len(v), 'p50': round(float(np.median(np.abs(v))), 2),
                                       'share_gt3': round(float(np.mean(np.abs(v) > 3)), 2)} for L, v in zL.items() if v},
    'abs_z_running_mean_raw': {L: {'n': len(v), 'p50': round(float(np.median(np.abs(v))), 2),
                                   'share_gt3': round(float(np.mean(np.abs(v) > 3)), 2)} for L, v in zL_raw.items() if v},
    'by_signal': {s: {'arcs': int(len(g)), 'std_r_mm': round(1000 * g['std_r'].median(), 2),
                      'model_mm': round(1000 * g['model'].median(), 2), 'tau_s': round(float(g['tau_s'].median()), 1)}
                  for s, g in S.groupby('sig')},
}
json.dump(summary, open(out_path, 'w'), indent=1)
print(json.dumps(summary, indent=1))
