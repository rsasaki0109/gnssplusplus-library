"""ar_consistency_stats.py DRIVE POS DEBUG_CSV [--integer-seconds]

LAMBDA-shadow consistency of accepted AR fixes (hold fixes excluded), split by
the reference label (3D <= 0.5 m correct): cost per pair, max |z|, share with
any |z| > 3, lock count of the max-|z| pair and pair count. Needs a run with
--lambda-shadow-candidates and --debug-epoch-log.
"""
import json
import sys

from truth_dd_common import load, num, quantile

drive, pos, debug_csv = sys.argv[1:4]
integer_seconds = '--integer-seconds' in sys.argv
err, rows = load(drive, pos, debug_csv)
out = {}
for group in ('correct', 'wrong'):
    sel = [rows[t] for t, (s, e) in err.items()
           if s == 4 and t in rows and rows[t]['hold_fix_applied'] != '1' and rows[t]['lambda_shadow_solved'] == '1'
           and (e <= 0.5) == (group == 'correct') and (not integer_seconds or abs(t - round(t)) < 1e-6)]
    cost = [num(r, 'lambda_shadow_best_cost') / num(r, 'pair_count') for r in sel if num(r, 'pair_count')]
    z = [num(r, 'lambda_shadow_max_abs_z') for r in sel if num(r, 'lambda_shadow_max_abs_z') is not None]
    lock = [num(r, 'lambda_shadow_max_z_lock_count') for r in sel if num(r, 'lambda_shadow_max_z_lock_count') is not None]
    out[group] = {'ar_fix': len(sel),
                  'cost_per_pair_p50': round(quantile(cost, .5), 2) if cost else None,
                  'max_abs_z_p50': round(quantile(z, .5), 2) if z else None,
                  'any_abs_z_gt3': round(sum(v > 3 for v in z) / len(z), 2) if z else None,
                  'max_z_lock_p50': quantile(lock, .5),
                  'pairs_p50': quantile([num(r, 'pair_count') for r in sel], .5) if sel else None}
print(json.dumps(out, indent=1))
