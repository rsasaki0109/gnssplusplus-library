"""Freeze all 17 Pixel5 test cases for a uniform heading-only comparison."""
import argparse
import copy
import json
from pathlib import Path

from compare_gsdc_doppler_diagnostic import audited_run
from run_gsdc_doppler_segment_diagnostic import sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['parent-plan', 'development-audit', 'binary', 'root']:
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    assert not args.root.exists(), 'preserve previous experiments'
    parent = json.loads(args.parent_plan.read_text())
    development = json.loads(args.development_audit.read_text())
    assert parent['schema'] == 'gsdc-pixel5-submitted-test-recipe.v1'
    assert len(parent['runs']) == 17
    assert development['status'] == 'all-pixel5-heading-development-pairs-audited'
    assert development['audited_pairs'] == development['expected_pairs'] == 15
    assert development['heading_contracts_verified'] is True
    assert development['height_map_group_independence_verified'] is True
    assert development['evaluation_truth_used_for_inference'] is False
    assert development['candidate_mean_m'] < development['control_mean_m']
    done = json.loads((args.parent_plan.parent / 'execution.done.json').read_text())
    assert done['native_execution_complete'] is True
    assert done['plan_sha256'] == sha(args.parent_plan)
    assert set(done['runs']) == set(parent['runs'])
    plan = dict(schema='gsdc-pixel5-heading-test.v1', status='prepared-not-started',
                submitted=False, official_score=False, accuracy_evaluated=False,
                selection='All 17 Pixel5 test cases, uniform heading option; no route selection.',
                arm_order=['control', 'candidate'],
                candidate_flags=['--native-epoch-heading-attitude-seeds'],
                common_flags=['--native-imu-state-diagnostic'],
                parent_plan=str(args.parent_plan.resolve()), parent_plan_sha256=sha(args.parent_plan),
                development_audit=str(args.development_audit.resolve()),
                development_audit_sha256=sha(args.development_audit),
                binary=str(args.binary.resolve()), binary_sha256=sha(args.binary),
                reference_submission_sha256=parent['reference_submission_sha256'],
                height_policy=copy.deepcopy(parent['height_policy']), runs={})
    for index, (case, entry) in enumerate(parent['runs'].items()):
        assert case.endswith('/pixel5')
        assert [x['mode'] for x in done['runs'][case]] == ['control', 'candidate']
        assert all(x['state'] == 'complete' for x in done['runs'][case])
        command = entry['argv']['candidate']
        folder = Path(command[command.index('--out') + 1]).parent
        record, summary, positions = audited_run(folder)
        assert record['argv'] == command and record['source_run_sha256'] == sha(args.parent_plan)
        assert record['binary_sha256'] == parent['binary_sha256']
        assert record['inputs_sha256'] == entry['inputs_sha256']
        assert summary['dataset_id'] == case
        assert summary['imu_initialization']['source_velocity_attitude_zero_bias_initialization'] is False
        for flag in plan['candidate_flags'] + plan['common_flags'] + ['--native-source-imu-initialization']:
            assert flag not in command
        new = dict(dataset_id=case, inputs_sha256=copy.deepcopy(entry['inputs_sha256']),
                   baseline_source=dict(run=str(folder / 'run.json'), run_sha256=sha(folder / 'run.json'),
                       solution_sha256=record['solution_sha256'], summary_sha256=record['summary_sha256'],
                       native_keys=len(positions)), argv={})
        for mode in plan['arm_order']:
            argv = [plan['binary']] + command[1:] + plan['common_flags']
            if mode == 'candidate':
                argv += plan['candidate_flags']
            dest = args.root / 'runs' / f'{index:02d}' / mode
            for option, name in [('--out', 'solution.csv'), ('--summary-json', 'summary.json')]:
                argv[argv.index(option) + 1] = str((dest / name).resolve())
            new['argv'][mode] = argv
        plan['runs'][case] = new
    args.root.mkdir(parents=True, exist_ok=False)
    target = args.root / 'plan.json'
    target.write_text(json.dumps(plan, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(dict(plan=str(target), plan_sha256=sha(target), cases=17, native_runs=34)))


if __name__ == '__main__':
    main()
