"""Freeze all 15 existing Pixel5 development cases for a heading-seed ablation."""
import argparse
import copy
import json
from pathlib import Path

from compare_gsdc_doppler_diagnostic import audited_run
from gsdc_development_plan_audit import audit_height_independence, provenance_hash
from run_gsdc_doppler_segment_diagnostic import sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--parent-plan', type=Path, required=True)
    parser.add_argument('--parent-audit', type=Path, required=True)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--root', type=Path, required=True)
    args = parser.parse_args()
    parent = json.loads(args.parent_plan.read_text())
    evidence = json.loads(args.parent_audit.read_text())
    assert evidence['status'] == 'all-planned-pairs-audited'
    assert evidence['plan_sha256'] == sha(args.parent_plan)
    assert evidence['audited_pairs'] == evidence['expected_pairs'] == 15
    assert evidence['height_map_group_independence_verified'] is True
    assert len(parent['runs']) == 15 and all(c.endswith('/pixel5') for c in parent['runs'])
    assert not args.root.exists(), 'preserve any previous experiment'
    flag = '--native-epoch-heading-attitude-seeds'
    diagnostic = '--native-imu-state-diagnostic'
    plan = dict(schema='gsdc-pixel5-heading-development.v1',
                status='prepared-not-started', official_score=False, heldout=False,
                selection='All 15 previously exposed Pixel5 development cases; no route selection.',
                parent_plan=str(args.parent_plan.resolve()), parent_plan_sha256=sha(args.parent_plan),
                parent_audit=str(args.parent_audit.resolve()), parent_audit_sha256=sha(args.parent_audit),
                binary=str(args.binary.resolve()), binary_sha256=sha(args.binary),
                arm_order=['control', 'candidate'], candidate_flags=[flag], common_flags=[diagnostic],
                train_gt_sha256=parent['train_gt_sha256'], height_policy=parent['height_policy'], runs={})
    for index, (case, entry) in enumerate(parent['runs'].items()):
        audit_height_independence(parent, case)
        provenance = provenance_hash(args.parent_plan, parent, case, require_done=True)
        command = entry['argv']['candidate']
        folder = Path(command[command.index('--out') + 1]).parent
        record, summary, positions = audited_run(folder)
        assert record['argv'] == command and record['source_run_sha256'] == provenance
        assert record['binary_sha256'] == parent['binary_sha256']
        assert record['inputs_sha256'] == entry['inputs_sha256']
        assert summary['dataset_id'] == case
        assert flag not in command and diagnostic not in command
        assert '--native-source-imu-initialization' not in command
        assert summary['imu_initialization']['source_velocity_attitude_zero_bias_initialization'] is False
        source = dict(run=str(folder / 'run.json'), run_sha256=sha(folder / 'run.json'),
                      solution_sha256=record['solution_sha256'], summary_sha256=record['summary_sha256'],
                      native_keys=len(positions))
        new = dict(dataset_id=case, route_group=entry['route_group'],
                   inputs_sha256=copy.deepcopy(entry['inputs_sha256']),
                   map=copy.deepcopy(entry.get('map')), baseline_source=source, argv={})
        for mode in plan['arm_order']:
            argv = [plan['binary']] + command[1:] + [diagnostic]
            dest = args.root / 'runs' / f'{index:02d}' / mode
            for option, name in [('--out', 'solution.csv'), ('--summary-json', 'summary.json')]:
                argv[argv.index(option) + 1] = str((dest / name).resolve())
            if mode == 'candidate':
                argv.append(flag)
            new['argv'][mode] = argv
        plan['runs'][case] = new
        audit_height_independence(plan, case)
    args.root.mkdir(parents=True, exist_ok=False)
    target = args.root / 'plan.json'
    target.write_text(json.dumps(plan, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(dict(status=plan['status'], cases=15, native_runs=30,
                          plan=str(target), plan_sha256=sha(target))))


if __name__ == '__main__':
    main()
