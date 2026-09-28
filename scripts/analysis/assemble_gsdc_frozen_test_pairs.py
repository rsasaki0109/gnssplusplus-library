"""Assemble a local test candidate only after complete development and native audits."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from compare_gsdc_doppler_diagnostic import audited_run, normalized_argv, positions
from run_gsdc_doppler_segment_diagnostic import sha
from score_gsdc_frozen_pairs import audit_recipe_effects


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def native_rows(path, case):
    checked = positions(path, case)
    with path.open(newline='', encoding='utf-8-sig') as stream:
        rows = {int(r['UnixTimeMillis']): (r['LatitudeDegrees'], r['LongitudeDegrees'])
                for r in csv.DictReader(stream)}
    assert set(rows) == set(checked)
    return rows


def displacement(control, candidate):
    assert set(control) == set(candidate)
    keys = sorted(control)
    a, b = (np.radians([table[k] for k in keys]) for table in [control, candidate])
    h = np.sin((a[:, 0]-b[:, 0])/2)**2 + np.cos(a[:, 0])*np.cos(b[:, 0])*np.sin((a[:, 1]-b[:, 1])/2)**2
    metres = 2*6371008.8*np.arcsin(np.sqrt(np.clip(h, 0, 1)))
    return dict(p50=float(np.percentile(metres, 50)), p95=float(np.percentile(metres, 95)),
                maximum=float(metres.max()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--development-report', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--out-dir', type=Path, required=True)
    args = parser.parse_args()
    plan, development = read(args.plan), read(args.development_report)
    assert plan['schema'] == 'gsdc-pixel5-submitted-test-recipe.v1'
    assert development['status'] == 'all-planned-pairs-audited', 'development comparison incomplete'
    assert development['expected_pairs'] == development['audited_pairs'] == 15
    assert development['native_recipe_effects_verified'] is True
    assert development['height_map_group_independence_verified'] is True
    assert development['plan_sha256'] == plan['development_plan_sha256']
    plan_hash = sha(args.plan)
    done = read(args.plan.parent / 'execution.done.json')
    assert done['plan_sha256'] == plan_hash and done['native_execution_complete'] is True
    assert set(done['runs']) == set(plan['runs']) and len(plan['runs']) == 17
    assert len(plan['retained_sources']) == 23
    assert sha(args.reference) == plan['reference_submission_sha256']
    with args.reference.open(newline='', encoding='utf-8-sig') as stream:
        reference = list(csv.DictReader(stream))
    reference_keys = [(r['tripId'], int(r['UnixTimeMillis'])) for r in reference]
    assert len(set(reference_keys)) == len(reference_keys) == 71936
    expected_cases = set(plan['runs']) | set(plan['retained_sources'])
    assert len(expected_cases) == 40 and expected_cases == {k[0] for k in reference_keys}
    output, source_evidence = {}, {}
    for case, entry in plan['runs'].items():
        arms = {}
        folders = {}
        for mode in ['control', 'candidate']:
            argv = entry['argv'][mode]
            folder = Path(argv[argv.index('--out')+1]).parent
            record, summary, solution = audited_run(folder)
            assert record['argv'] == argv and record['source_run_sha256'] == plan_hash
            assert record['binary_sha256'] == plan['binary_sha256']
            assert record['inputs_sha256'] == entry['inputs_sha256']
            assert summary['dataset_id'] == case
            arms[mode] = (record, summary, solution)
            folders[mode] = folder
        a, b = arms['control'], arms['candidate']
        assert normalized_argv(b[0]['argv']) == normalized_argv(a[0]['argv']) + plan['candidate_flags']
        audit_recipe_effects(dict(runs={mode:dict(tdcp=data[1]['tdcp_contract'],
            main_doppler=data[1]['graph']['phase213_main_doppler']) for mode, data in arms.items()}),
            plan['candidate_flags'])
        published = entry['published_source']
        old = Path(published['solution'])
        assert sha(old) == published['solution_sha256']
        assert (folders['control'] / 'solution.csv').read_bytes() == old.read_bytes(), \
            'control does not reproduce the published native source: ' + case
        output[case] = native_rows(folders['candidate'] / 'solution.csv', case)
        source_evidence[case] = dict(mode='candidate', native_run=str(folders['candidate'] / 'run.json'),
            run_sha256=sha(folders['candidate'] / 'run.json'), solution_sha256=b[0]['solution_sha256'],
            summary_sha256=b[0]['summary_sha256'], control_matches_published_source=True,
            candidate_vs_control_displacement_m=displacement(a[2], b[2]))
    for case, entry in plan['retained_sources'].items():
        path = Path(entry['solution'])
        assert sha(path) == entry['solution_sha256']
        assert sha(entry['run']) == entry['run_sha256']
        assert sha(path.parent / 'summary.json') == entry['summary_sha256']
        record, summary = read(Path(entry['run'])), read(path.parent / 'summary.json')
        assert record['returncode'] == 0 and summary['truth_used'] is False
        assert summary['native_pdc_imu_tdcp_no_bridge'] is True and summary['graph']['converged']
        assert all(summary['raw_utc_key_contract'][k] == 0 for k in
                   ['interpolated_epochs', 'edge_hold_epochs', 'unresolved_epochs'])
        output[case] = native_rows(path, case)
        source_evidence[case] = dict(mode='retained-native', **entry)
    rows = []
    for original in reference:
        case, utc = original['tripId'], int(original['UnixTimeMillis'])
        lat, lon = output[case][utc]  # Exact lookup only; missing keys fail.
        if case in plan['retained_sources']:
            assert (lat, lon) == (original['LatitudeDegrees'], original['LongitudeDegrees'])
        rows.append(dict(tripId=case, UnixTimeMillis=original['UnixTimeMillis'],
                         LatitudeDegrees=lat, LongitudeDegrees=lon))
    args.out_dir.mkdir(parents=True, exist_ok=False)
    path = args.out_dir / 'submission.csv'
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=['tripId', 'UnixTimeMillis',
            'LatitudeDegrees', 'LongitudeDegrees'], lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    manifest = dict(status='assembled-locally-not-submitted', submitted=False, official_score=False,
        rows=len(rows), drives=40, replaced_drives=17, retained_drives=23,
        plan_sha256=plan_hash, development_report_sha256=sha(args.development_report),
        reference_submission_sha256=sha(args.reference), submission_sha256=sha(path),
        all_output_rows_native=True, evaluation_truth_used=False,
        reference_coordinates_used_for_inference=False, all_control_sources_reproduced=True,
        source_evidence=source_evidence)
    (args.out_dir / 'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k:v for k,v in manifest.items() if k != 'source_evidence'}, indent=2))


if __name__ == '__main__':
    main()
