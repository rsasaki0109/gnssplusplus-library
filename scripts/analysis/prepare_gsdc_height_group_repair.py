"""Rebuild one unsafe development height map and preserve the fixed cohort."""
import argparse
import copy
import json
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

from gsdc_development_plan_audit import audit_height_independence, evaluation_group
from run_gsdc_doppler_segment_diagnostic import sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--parent-plan', type=Path, required=True)
    parser.add_argument('--case', required=True)
    parser.add_argument('--out-dir', type=Path, required=True)
    args = parser.parse_args()
    original = json.loads(args.plan.read_text())
    parent = json.loads(args.parent_plan.read_text())
    assert sha(args.parent_plan) == original['parent_plan_sha256']
    entry = copy.deepcopy(original['runs'][args.case])
    try:
        audit_height_independence(original, args.case)
    except AssertionError:
        pass
    else:
        raise ValueError('Selected case has no established height-group violation')
    source = parent['runs'][args.case]['argv']
    seed = Path(source[source.index('--out') + 1])
    assert sha(seed) == entry['map']['seed_solution_sha256']
    group = evaluation_group(args.case.split('/')[0])
    included, excluded, arrays = [], [], []
    for file, digest in original['train_gt_sha256'].items():
        assert sha(file) == digest
        if evaluation_group(Path(file).name.split('__')[0]) == group:
            excluded.append(file)
        else:
            included.append(file)
            arrays.append(pd.read_csv(file)[['LatitudeDegrees', 'LongitudeDegrees', 'AltitudeMeters']].to_numpy())
    points = np.vstack(arrays)
    trajectory = pd.read_csv(seed)
    def en(lat, lon):
        return np.c_[np.radians(lon) * 6371008.8 * np.cos(np.radians(37.4)),
                     np.radians(lat) * 6371008.8]
    query = en(trajectory.LatitudeDegrees, trajectory.LongitudeDegrees)
    distance, _ = cKDTree(query).query(en(points[:, 0], points[:, 1]), distance_upper_bound=30)
    kept = points[np.isfinite(distance)]
    coverage = (np.isfinite(cKDTree(en(kept[:, 0], kept[:, 1])).query(query, distance_upper_bound=15)[0]).mean()
                if len(kept) else 0.0)
    args.out_dir.mkdir(parents=True, exist_ok=False)
    map_path = args.out_dir / 'height_map.csv'
    pd.DataFrame(kept, columns=['lat_deg', 'lon_deg', 'height_m']).to_csv(map_path, index=False, float_format='%.9f')
    old_map = entry['map']['map']
    del entry['inputs_sha256'][old_map]
    use_map = len(kept) > 0 and coverage >= .1
    if use_map:
        entry['inputs_sha256'][str(map_path)] = sha(map_path)
    entry['map'].update(map=str(map_path), points=len(kept), coverage_15m=float(coverage),
                        excluded_evaluation_group=group, excluded_gt_files=excluded, included_gt_files=included)
    for mode, argv in entry['argv'].items():
        index = argv.index('--native-height-map')
        argv[index:index + 2] = ['--native-height-map', str(map_path)] if use_map else ['--native-relative-height-pairs']
        for flag, name in [('--out', 'solution.csv'), ('--summary-json', 'summary.json')]:
            argv[argv.index(flag) + 1] = str(args.out_dir / 'runs' / '00' / mode / name)
    replacement = copy.deepcopy(original)
    replacement.update(status='prepared-not-started', runs={args.case: entry},
                       selection='One predetermined evaluation-group map repair; no score-driven selection',
                       repair_original_plan=str(args.plan), repair_original_plan_sha256=sha(args.plan),
                       repair_preparer_sha256=sha(__file__))
    audit_height_independence(replacement, args.case)
    replacement_path = args.out_dir / 'plan.json'
    replacement_path.write_text(json.dumps(replacement, indent=2) + '\n')
    combined = copy.deepcopy(original)
    combined.update(schema='gsdc-repaired-frozen-development.v1', status='waiting-for-all-source-pairs',
                    original_plan=str(args.plan), original_plan_sha256=sha(args.plan),
                    height_policy='Same 10% coverage policy, excluding every phone in the evaluated route group, including established aliases',
                    repair_case=args.case, repair_preparer_sha256=sha(__file__))
    combined['runs'][args.case] = copy.deepcopy(entry)
    for case, selected in combined['runs'].items():
        provenance = replacement_path if case == args.case else args.plan
        selected.update(provenance_plan=str(provenance), provenance_plan_sha256=sha(provenance))
        audit_height_independence(combined, case)
    combined_path = args.out_dir / 'evaluation_plan.json'
    combined_path.write_text(json.dumps(combined, indent=2) + '\n')
    print(json.dumps(dict(replacement_plan=str(replacement_path), replacement_sha256=sha(replacement_path),
        evaluation_plan=str(combined_path), evaluation_sha256=sha(combined_path),
        map_points=len(kept), coverage_15m=float(coverage), excluded_gt_files=excluded), indent=2))


if __name__ == '__main__':
    main()
