"""Post-hoc directional-error diagnostic; never alters a trajectory or inference.

An error/speed association is not proof of timing error: route geometry,
multipath and antenna offsets can confound it. No fitted values are exported
as solver inputs or applied to coordinates.
"""
import argparse
import json
from pathlib import Path
import statistics

import numpy as np
import pandas as pd

from run_gsdc_doppler_segment_diagnostic import sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--truth-root', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text())
    assert len(plan['runs']) == 40
    reports = {}
    for case, source in plan['runs'].items():
        folder = Path(source['folder'])
        record = json.loads((folder / 'run.json').read_text())
        assert record['returncode'] == 0 and record['dataset_id'] == case
        for filename in ['solution.csv', 'summary.json']:
            assert sha(folder / filename) == record['outputs'][filename]
        summary = json.loads((folder / 'summary.json').read_text())
        assert summary['truth_used'] is False and summary['graph']['converged']
        solution = pd.read_csv(folder / 'solution.csv').set_index('UnixTimeMillis').sort_index()
        truth_path = args.truth_root / case / 'ground_truth.csv'
        truth = pd.read_csv(truth_path).set_index('UnixTimeMillis').sort_index()
        assert solution.index.is_unique and truth.index.is_unique
        assert set(solution.phone) == {case} and set(truth.index) <= set(solution.index)
        contract = summary['raw_utc_key_contract']
        assert len(solution) == contract['exact_solution_epochs'] == contract['raw_epoch_keys']
        assert all(contract[k] == 0 for k in ['interpolated_epochs', 'edge_hold_epochs', 'unresolved_epochs'])
        t = solution.index.to_numpy(dtype=float) / 1000
        lat = np.radians(solution.LatitudeDegrees.to_numpy())
        lon = np.radians(solution.LongitudeDegrees.to_numpy())
        dt = t[2:] - t[:-2]
        vn = np.full(len(t), np.nan)
        ve = vn.copy()
        vn[1:-1] = 6371008.8 * (lat[2:] - lat[:-2]) / dt
        ve[1:-1] = 6371008.8 * np.cos(lat[1:-1]) * (lon[2:] - lon[:-2]) / dt
        adjacent = np.zeros(len(t), dtype=bool)
        adjacent[1:-1] = ((t[1:-1] - t[:-2] > 0) & (t[2:] - t[1:-1] > 0) &
                         (t[1:-1] - t[:-2] <= 2.5) & (t[2:] - t[1:-1] <= 2.5))
        aligned = truth.reindex(solution.index)
        tn = np.radians(aligned.LatitudeDegrees.to_numpy())
        te = np.radians(aligned.LongitudeDegrees.to_numpy())
        error_n = 6371008.8 * (tn - lat)
        error_e = 6371008.8 * np.cos((tn + lat) / 2) * (te - lon)
        speed = np.hypot(vn, ve)
        valid = adjacent & np.isfinite(error_n) & np.isfinite(error_e) & (speed >= 5) & (speed < 50)
        v = speed[valid]
        along = (error_n[valid] * vn[valid] + error_e[valid] * ve[valid]) / v
        across = (error_e[valid] * vn[valid] - error_n[valid] * ve[valid]) / v
        bins = []
        bounds = [5, 10, 15, 20, 25, 30, 40, 50]
        for low, high in zip(bounds[:-1], bounds[1:]):
            mask = (v >= low) & (v < high)
            if mask.sum() >= 10:
                bins.append(dict(low_mps=low, high_mps=high, count=int(mask.sum()),
                    median_speed_mps=float(np.median(v[mask])),
                    median_along_error_m=float(np.median(along[mask])),
                    median_across_error_m=float(np.median(across[mask]))))
        result = dict(solution_sha256=sha(folder / 'solution.csv'), summary_sha256=sha(folder / 'summary.json'),
            truth_sha256=sha(truth_path), native_position_offset=summary['native_upstream_position_offset'],
            native_keys=len(solution), truth_keys=len(truth), moving_samples=len(v), speed_bins=bins,
            median_along_error_m=float(np.median(along)) if len(v) else None,
            median_across_error_m=float(np.median(across)) if len(v) else None)
        if len(v) >= 50 and np.std(v) >= 2.5:
            intercept, slope = np.linalg.lstsq(np.c_[np.ones(len(v)), v], along, rcond=None)[0]
            result['descriptive_ols'] = dict(intercept_m=float(intercept), speed_coefficient_s=float(slope))
        reports[case] = result
    phones = {}
    for case, result in reports.items():
        phone = case.split('/')[1]
        phones.setdefault(phone, []).append(result)
    groups = {}
    for phone, values in phones.items():
        slopes = [v['descriptive_ols']['speed_coefficient_s'] for v in values if 'descriptive_ols' in v]
        offsets = [v['median_along_error_m'] for v in values if v['median_along_error_m'] is not None]
        groups[phone] = dict(cases=len(values), estimable_cases=len(slopes),
            median_route_speed_coefficient_s=statistics.median(slopes) if slopes else None,
            positive_slopes=sum(s > 0 for s in slopes), negative_slopes=sum(s < 0 for s in slopes),
            median_route_along_error_m=statistics.median(offsets) if offsets else None)
    output = dict(status='posthoc-descriptive-diagnostic', official_score=False, heldout=False,
        truth_used_for_evaluation=True, truth_used_in_inference=False, coordinates_modified=False,
        plan_sha256=sha(args.plan), analyzer_sha256=sha(__file__),
        baseline='Frozen corrected train40 recipe, before upstream position offset and height policy',
        interpretation='Positive along error means truth lies ahead of the native trajectory. Speed coefficients are descriptive associations, not identified timing delays or inference parameters.',
        cases=reports, phones=groups)
    args.out.write_text(json.dumps(output, indent=2) + '\n')
    print(json.dumps(groups, indent=2))


if __name__ == '__main__':
    main()
