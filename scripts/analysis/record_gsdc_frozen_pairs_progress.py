"""Record audited finished pairs without estimating an incomplete batch mean."""
import argparse
import json
from pathlib import Path
import time

from compare_gsdc_doppler_diagnostic import audited_run, sha
from score_gsdc_frozen_pairs import audit_recipe_effects
from gsdc_development_plan_audit import audit_height_independence, provenance_hash


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text())
    plan_hash = sha(args.plan)
    completed = []
    for case, entry in plan['runs'].items():
        folders = {mode: Path(argv[argv.index('--out') + 1]).parent
                   for mode, argv in entry['argv'].items()}
        comparison = folders['candidate'].parent / 'comparison.json'
        if not comparison.exists():
            continue
        audit_height_independence(plan, case)
        report = json.loads(comparison.read_text())
        assert report['status'] == 'audited-development-pair'
        assert report['dataset_id'] == case
        assert report['binary_sha256'] == plan['binary_sha256']
        assert report['candidate_flags'] == plan['candidate_flags']
        for mode, folder in folders.items():
            record, summary, positions = audited_run(folder)
            assert record['argv'] == entry['argv'][mode]
            assert record['source_run_sha256'] == provenance_hash(args.plan, plan, case)
            assert record['inputs_sha256'] == entry['inputs_sha256']
            assert record['binary_sha256'] == plan['binary_sha256']
            for key in ['solution_sha256', 'summary_sha256']:
                assert report['runs'][mode][key] == record[key]
            assert report['runs'][mode]['native_raw_keys'] == len(positions)
        if 'train_gt_sha256' in plan:
            name = case.replace('/', '__') + '__ground_truth.csv'
            pinned = [digest for path, digest in plan['train_gt_sha256'].items()
                      if Path(path).name == name]
            assert len(pinned) == 1 and report['truth_sha256'] == pinned[0]
        audit_recipe_effects(report, plan['candidate_flags'])
        completed.append(dict(dataset_id=case, comparison=str(comparison),
            comparison_sha256=sha(comparison),
            scores={mode: report['runs'][mode]['score'] for mode in ['control', 'candidate']},
            delta_phone_score_m=report['delta_phone_score_m'],
            native_recipe_effects_verified=True))
    result = dict(status='partial-development-evaluation', official_score=False,
        promoted=False, checked_unix=time.time(), expected_pairs=len(plan['runs']),
        audited_pairs=len(completed), aggregate_score=None,
        selection=plan['selection'], plan_sha256=plan_hash, completed=completed,
        recorder_sha256=sha(__file__))
    args.out.write_text(json.dumps(result, indent=2) + '\n')
    print(f"Audited {len(completed)}/{len(plan['runs'])} pairs; no partial aggregate")


if __name__ == '__main__':
    main()
