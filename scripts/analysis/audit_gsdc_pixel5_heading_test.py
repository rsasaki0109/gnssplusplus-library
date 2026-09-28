"""Audit all 17 frozen Pixel5 test heading pairs without reading evaluation truth."""
import argparse
import json
from pathlib import Path
import statistics

from audit_gsdc_imu_state_diagnostic import inspect_states
from compare_gsdc_doppler_diagnostic import audited_run, normalized_argv, positions, score
from assemble_gsdc_frozen_test_pairs import displacement
from run_gsdc_doppler_segment_diagnostic import sha


FLAG = '--native-epoch-heading-attitude-seeds'
DIAGNOSTIC = '--native-imu-state-diagnostic'


def heading_contract(a, b, keys):
    assert a['dataset_id'] == b['dataset_id']
    ta, tb = a['tdcp_contract'], b['tdcp_contract']
    assert ta['epoch_heading_attitude_seeds_requested'] is False
    assert ta['epoch_heading_attitude_seeds_inserted'] == 0
    assert tb['epoch_heading_attitude_seeds_requested'] is True
    assert tb['epoch_heading_attitude_seeds_inserted'] == len(keys)
    for summary in [a, b]:
        assert summary['imu_initialization']['source_velocity_attitude_zero_bias_initialization'] is False
    for section, fields in {
        'graph': ['factors', 'upstream_stop_velocity_factors', 'upstream_stop_pose_factors',
                  'phase205_bias_density'],
        'tdcp_contract': ['first_imu_bias_priors_inserted', 'first_imu_bias_priors_omitted',
                          'factors_built', 'factors_inserted', 'sigma_m',
                          'height_map_factors_inserted', 'relative_height_factors_inserted'],
    }.items():
        for field in fields:
            assert a[section][field] == b[section][field], (section, field)
    return {mode: inspect_states(s, keys) for mode, s in [('control', a), ('candidate', b)]}


def audit_native(plan_path):
    plan = json.loads(plan_path.read_text())
    digest = sha(plan_path)
    assert plan['schema'] == 'gsdc-pixel5-heading-test.v1'
    assert plan['arm_order'] == ['control', 'candidate']
    assert plan['candidate_flags'] == [FLAG] and plan['common_flags'] == [DIAGNOSTIC]
    for name in ['parent_plan', 'development_audit', 'binary']:
        assert sha(plan[name]) == plan[name + '_sha256']
    parent = json.loads(Path(plan['parent_plan']).read_text())
    assert set(plan['runs']) == set(parent['runs']) and len(plan['runs']) == 17
    done_path = plan_path.parent / 'execution.done.json'
    done = json.loads(done_path.read_text())
    assert done['native_execution_complete'] is True and done['plan_sha256'] == digest
    assert set(done['runs']) == set(plan['runs'])
    result, solutions = {}, {}
    for case, entry in plan['runs'].items():
        assert case.endswith('/pixel5')
        original = parent['runs'][case]
        assert entry['inputs_sha256'] == original['inputs_sha256']
        source = entry['baseline_source']
        old_dir = Path(source['run']).parent
        assert sha(source['run']) == source['run_sha256']
        old = json.loads(Path(source['run']).read_text())
        assert old['argv'] == original['argv']['candidate']
        assert sha(old_dir / 'summary.json') == source['summary_sha256']
        assert sha(old_dir / 'solution.csv') == source['solution_sha256']
        assert normalized_argv(entry['argv']['control'])[1:] == normalized_argv(old['argv'])[1:] + [DIAGNOSTIC]
        outcomes = done['runs'][case]
        assert [x['mode'] for x in outcomes] == plan['arm_order']
        arms, folders = {}, {}
        for mode, outcome in zip(plan['arm_order'], outcomes):
            argv = entry['argv'][mode]
            folder = Path(argv[argv.index('--out') + 1]).parent
            assert outcome['state'] == 'complete'
            assert Path(outcome['run']).resolve() == (folder / 'run.json').resolve()
            r, s, p = audited_run(folder)
            assert r['argv'] == argv and r['source_run_sha256'] == digest
            assert r['binary_sha256'] == plan['binary_sha256']
            assert r['inputs_sha256'] == entry['inputs_sha256']
            assert s['dataset_id'] == case
            arms[mode], folders[mode] = (r, s, p), folder
        a, b = arms['control'], arms['candidate']
        assert normalized_argv(b[0]['argv']) == normalized_argv(a[0]['argv']) + [FLAG]
        assert set(a[2]) == set(b[2]) and len(a[2]) == source['native_keys']
        assert (folders['control'] / 'solution.csv').read_bytes() == (old_dir / 'solution.csv').read_bytes()
        seed = 'summary.json.initialization.json'
        assert (folders['control'] / seed).read_bytes() == (folders['candidate'] / seed).read_bytes()
        states = heading_contract(a[1], b[1], a[2])
        result[case] = dict(control_source_byte_parity=True, states=states, displacement_m=displacement(a[2], b[2]),
            solution_sha256={m: x[0]['solution_sha256'] for m, x in arms.items()},
            summary_sha256={m: x[0]['summary_sha256'] for m, x in arms.items()})
        solutions[case] = {m: x[2] for m, x in arms.items()}
    return plan, result, solutions, sha(done_path)



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    plan, records, solutions, done_hash = audit_native(args.plan)
    report = dict(status='all-pixel5-heading-test-pairs-audited',
        official_score=False, accuracy_evaluated=False, truth_consumed=False, submitted=False,
        plan_sha256=sha(args.plan), execution_done_sha256=done_hash,
        expected_pairs=17, audited_pairs=17, all_control_sources_reproduced=True,
        heading_contracts_verified=True, runs=records)
    args.out.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k != 'runs'}, indent=2))


if __name__ == '__main__':
    main()
