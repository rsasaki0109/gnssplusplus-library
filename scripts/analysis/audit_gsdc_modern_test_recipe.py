"""Audit a complete three-arm test experiment without loading test truth."""
import argparse
import json
from pathlib import Path

from assemble_gsdc_frozen_test_pairs import displacement
from compare_gsdc_doppler_diagnostic import audited_run, normalized_argv
from run_gsdc_doppler_segment_diagnostic import sha
from score_gsdc_frozen_pairs import audit_recipe_effects


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    plan = read(args.plan)
    assert plan['schema'] == 'gsdc-modern-submitted-test-three-arm.v1'
    assert plan['arm_order'] == ['control', 'clock_only', 'candidate']
    assert len(plan['runs']) == 9
    done_path = args.plan.parent / 'execution.done.json'
    assert done_path.exists(), 'native experiment not finished'
    done = read(done_path)
    assert done['native_execution_complete'] and done['plan_sha256'] == sha(args.plan)
    assert set(done['runs']) == set(plan['runs'])
    for field in ['source_audit', 'development_plan', 'development_report', 'extra_band_audit', 'reference_submission']:
        assert sha(plan[field]) == plan[field + '_sha256']
    flags = plan['candidate_flags']
    assert flags == ['--native-source-tdcp-meter-sigma', '--native-phase184-source-tdcp-huber-k',
                     '--native-phase213-main-doppler']
    assert plan['arm_flags'] == dict(control=[], clock_only=['--android-continuous-clock-reference'],
                                    candidate=['--android-continuous-clock-reference'] + flags)
    result = {}
    for case, entry in plan['runs'].items():
        outcomes = done['runs'][case]
        assert [item['mode'] for item in outcomes] == plan['arm_order']
        assert all(item['state'] == 'complete' for item in outcomes)
        arms = {}
        for mode in plan['arm_order']:
            command = entry['argv'][mode]
            assert normalized_argv(command) == normalized_argv(entry['argv']['control']) + plan['arm_flags'][mode]
            folder = Path(command[command.index('--out') + 1]).parent
            outcome = next(item for item in outcomes if item['mode'] == mode)
            assert Path(outcome['run']).resolve() == (folder / 'run.json').resolve()
            record, summary, positions = audited_run(folder)
            assert record['argv'] == command and record['source_run_sha256'] == sha(args.plan)
            assert record['binary_sha256'] == plan['binary_sha256']
            assert record['inputs_sha256'] == entry['inputs_sha256']
            assert summary['dataset_id'] == case
            arms[mode] = (record, summary, positions)
        published = entry['published_source']
        assert sha(published['run']) == published['run_sha256']
        assert sha(published['solution']) == published['solution_sha256']
        assert arms['control'][0]['solution_sha256'] == published['solution_sha256'], 'published control mismatch: ' + case
        assert 'reset when successive' in arms['control'][1]['android_gnss_diagnostics']['timing_formula']
        audit_recipe_effects(dict(runs={label: dict(tdcp=arms[mode][1]['tdcp_contract'],
            main_doppler=arms[mode][1]['graph']['phase213_main_doppler'])
            for label, mode in [('control', 'clock_only'), ('candidate', 'candidate')]}), flags)
        result[case] = dict(native_keys=len(arms['candidate'][2]), control_matches_published=True,
            solution_sha256={mode: item[0]['solution_sha256'] for mode, item in arms.items()},
            run_evidence={mode: dict(summary_sha256=item[0]['summary_sha256'],
                wall_s=item[0]['wall_s']) for mode, item in arms.items()},
            displacement_m={label: displacement(arms[a][2], arms[b][2]) for label, a, b in [
                ('clock_vs_control', 'control', 'clock_only'),
                ('recipe_vs_clock', 'clock_only', 'candidate'),
                ('candidate_vs_control', 'control', 'candidate')]})
    report = dict(status='all-modern-test-three-arms-audited', official_score=False,
        accuracy_evaluated=False, submitted=False, truth_consumed=False,
        plan_sha256=sha(args.plan), execution_done_sha256=sha(done_path),
        audited_cases=len(result), audited_arms=3 * len(result), runs=result)
    args.out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(dict(status=report['status'], audited_cases=len(result))))


if __name__ == '__main__':
    main()
