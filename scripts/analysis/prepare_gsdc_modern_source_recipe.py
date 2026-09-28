"""Prepare the established eleven-phone cohort with group-excluded height maps."""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

from gsdc_development_plan_audit import audit_height_independence, evaluation_group
from run_gsdc_doppler_segment_diagnostic import sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native-root', type=Path, required=True)
    parser.add_argument('--build-manifest', type=Path, required=True)
    parser.add_argument('--train-gt', type=Path, required=True)
    parser.add_argument('--out-dir', type=Path, required=True)
    args = parser.parse_args()
    root = args.native_root
    parent_path = root / 'train40_corrected_recipe_plan.json'
    cohort_path = root / 'train_expanded_bias_tdcp_sigma_plan.json'
    parent, cohort = [json.loads(p.read_text()) for p in [parent_path, cohort_path]]
    assert sha(parent_path) == '67efef7c87fc883ffe56fb28974692f96a8f77ee7ed51a47fe1ad0d61554a7f3'
    cases = list(cohort['runs'])
    assert len(cases) == 11 and set(cases) <= set(parent['runs'])
    assert {case.split('/')[1] for case in cases} == {'sm-g988b', 'pixel6pro', 'pixel7pro', 'sm-s908b'}
    build = json.loads(args.build_manifest.read_text(encoding='utf-8-sig'))
    assert build['status'] == 'built-and-raw-p-tests-passed-native-replay-pending'
    assert build['failures'] == 0 and build['tests'] == 40
    binary = Path(build['binary'])
    assert sha(binary) == build['binary_sha256']
    assert sha(build['test_report']) == build['test_report_sha256']
    gt = {str(p.resolve()): sha(p) for p in sorted(args.train_gt.glob('*ground_truth.csv'))}
    assert len(gt) == 156
    arrays = {p: pd.read_csv(p)[['LatitudeDegrees', 'LongitudeDegrees', 'AltitudeMeters']].to_numpy()
              for p in gt}
    flags = ['--native-source-tdcp-meter-sigma', '--native-phase184-source-tdcp-huber-k',
             '--native-phase213-main-doppler']
    common = ['--native-upstream-position-offset',
              '--native-base-pseudorange-preserve-additional-frequency-bands',
              '--android-continuous-clock-reference']
    plan = dict(schema='gsdc-modern-source-recipe-development.v1', status='prepared-not-started',
        official_score=False, heldout=False, promoted=False, binary=str(binary), binary_sha256=sha(binary),
        parent_plan=str(parent_path), parent_plan_sha256=sha(parent_path),
        cohort_plan=str(cohort_path), cohort_plan_sha256=sha(cohort_path),
        build_manifest=str(args.build_manifest), build_manifest_sha256=sha(args.build_manifest),
        preparer_sha256=sha(__file__), train_gt_sha256=gt, candidate_flags=flags, common_flags=common,
        selection='All eleven cases from the established expanded-bias cohort; no per-route score selection',
        height_policy='Both arms: exclude the entire evaluation group; 30 m map radius, 15 m coverage, map if coverage >= 10%, otherwise relative pairs',
        execution_gate='Not launched by preparation; reserve one inference slot after A205U v2 replay', runs={})
    args.out_dir.mkdir(parents=True, exist_ok=False)
    maps = args.out_dir / 'maps'
    maps.mkdir()

    def en(lat, lon):
        return np.c_[np.radians(lon) * 6371008.8 * np.cos(np.radians(37.4)),
                     np.radians(lat) * 6371008.8]

    for index, case in enumerate(cases):
        source = parent['runs'][case]
        folder = Path(source['folder'])
        run = json.loads((folder / 'run.json').read_text())
        summary = json.loads((folder / 'summary.json').read_text())
        assert run['returncode'] == 0 and summary['truth_used'] is False
        seed = folder / 'solution.csv'
        trajectory = pd.read_csv(seed)
        assert set(trajectory.phone) == {case}
        query = en(trajectory.LatitudeDegrees, trajectory.LongitudeDegrees)
        group = evaluation_group(case.split('/')[0])
        excluded = [p for p in gt if evaluation_group(Path(p).name.split('__')[0]) == group]
        included = [p for p in gt if p not in excluded]
        assert excluded
        points = np.vstack([arrays[p] for p in included])
        distances = cKDTree(query).query(en(points[:, 0], points[:, 1]), distance_upper_bound=30)[0]
        kept = points[np.isfinite(distances)]
        coverage = (np.isfinite(cKDTree(en(kept[:, 0], kept[:, 1])).query(query, distance_upper_bound=15)[0]).mean()
                    if len(kept) else 0.0)
        map_path = maps / (case.replace('/', '__') + '.csv')
        pd.DataFrame(kept, columns=['lat_deg', 'lon_deg', 'height_m']).to_csv(map_path, index=False, float_format='%.9f')
        height = ['--native-height-map', str(map_path)] if len(kept) and coverage >= .1 else ['--native-relative-height-pairs']
        inputs = {item['path']: item['sha256'] for item in source['inputs']}
        assert all(sha(p) == digest for p, digest in inputs.items())
        argv = [str(binary)] + source['argv'][1:]
        assert all(flag not in argv for flag in flags + common + ['--native-height-map', '--native-relative-height-pairs'])
        assert '--android-raw-clock-only' in argv
        argv += common + height
        if height[0] == '--native-height-map':
            inputs[str(map_path)] = sha(map_path)
        entry = dict(dataset_id=case, route_group=group, inputs_sha256=inputs,
            source_run=str(folder / 'run.json'), source_run_sha256=sha(folder / 'run.json'),
            map=dict(map=str(map_path), points=len(kept), coverage_15m=float(coverage),
                excluded_evaluation_group=group, excluded_gt_files=excluded, included_gt_files=included,
                seed_solution=str(seed), seed_solution_sha256=sha(seed)), argv={})
        for mode in ['control', 'candidate']:
            command = argv.copy()
            for flag, filename in [('--out', 'solution.csv'), ('--summary-json', 'summary.json')]:
                command[command.index(flag) + 1] = str(args.out_dir / 'runs' / f'{index:02d}' / mode / filename)
            if mode == 'candidate':
                command += flags
            entry['argv'][mode] = command
        plan['runs'][case] = entry
        audit_height_independence(plan, case)
        print(case, len(kept), float(coverage), flush=True)
    path = args.out_dir / 'plan.json'
    path.write_text(json.dumps(plan, indent=2) + '\n')
    print(json.dumps(dict(plan=str(path), plan_sha256=sha(path), executed=False)))


if __name__ == '__main__':
    main()
