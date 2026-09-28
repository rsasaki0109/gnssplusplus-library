"""Freeze a three-arm modern-phone test experiment without executing it."""
import argparse
import json
from pathlib import Path

from gsdc_development_plan_audit import audit_height_independence
from run_gsdc_doppler_segment_diagnostic import sha


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native-root', type=Path, required=True)
    parser.add_argument('--source-audit', type=Path, required=True)
    parser.add_argument('--development-plan', type=Path, required=True)
    parser.add_argument('--development-report', type=Path, required=True)
    parser.add_argument('--out-dir', type=Path, required=True)
    args = parser.parse_args()
    source_audit, development, report = map(read, [args.source_audit, args.development_plan, args.development_report])
    assert source_audit['status'] == 'published-modern-test-sources-audited'
    assert all(sha(p) == h for p, h in source_audit['input_evidence'].items())
    assert sha(args.development_report) in source_audit['input_evidence'].values()
    assert report['status'] == 'all-planned-pairs-audited'
    assert report['plan_sha256'] == sha(args.development_plan)
    assert report['expected_pairs'] == report['audited_pairs'] == len(development['runs']) == 11
    assert set(report['runs']) == set(development['runs'])
    assert report['native_recipe_effects_verified'] and report['height_map_group_independence_verified']
    assert report['delta_mean_m'] < 0
    for case, item in report['runs'].items():
        assert sha(item['comparison']) == item['comparison_sha256']
        audit_height_independence(development, case)
    binary = Path(development['binary'])
    assert sha(binary) == development['binary_sha256']
    recipe = development['candidate_flags']
    assert recipe == ['--native-source-tdcp-meter-sigma', '--native-phase184-source-tdcp-huber-k',
                      '--native-phase213-main-doppler']
    clock = ['--android-continuous-clock-reference']
    arms = dict(control=[], clock_only=clock, candidate=clock + recipe)
    extra_audit_path = args.native_root / 'test40_offset_extra_bands_v1/audit.json'
    extra = read(extra_audit_path)
    assert extra['complete']
    receipt = read(args.native_root / 'all40_height_submission_v1/kaggle_receipt.json')
    reference = args.native_root / 'all40_height_submission_v1/submission.csv'
    assert receipt['submitted'] and receipt['submission_ref'] == 56536540
    assert sha(reference) == receipt['submission_sha256']
    plan = dict(schema='gsdc-modern-submitted-test-three-arm.v1', status='prepared-not-started',
        submitted=False, official_score=False, binary=str(binary), binary_sha256=sha(binary),
        arm_order=list(arms), arm_flags=arms, candidate_flags=recipe,
        reference_submission=str(reference), reference_submission_sha256=sha(reference),
        reference_submission_ref=receipt['submission_ref'],
        source_audit=str(args.source_audit.resolve()), source_audit_sha256=sha(args.source_audit),
        development_plan=str(args.development_plan.resolve()), development_plan_sha256=sha(args.development_plan),
        development_report=str(args.development_report.resolve()), development_report_sha256=sha(args.development_report),
        extra_band_audit=str(extra_audit_path), extra_band_audit_sha256=sha(extra_audit_path),
        preparer_sha256=sha(__file__),
        selection='All nine test phones matching the four models in the fixed modern11 development cohort; no per-route winner selection',
        height_policy='Preserve each published native source height recipe',
        execution_gate='Requires a three-arm runner and no more than two native processes globally; do not launch alongside the active Pixel5 two-worker queue',
        assembly_gate='No assembly or submission here. Require all three arms for all nine, published-control byte parity, native key/convergence/input audits, and separate clock-only and recipe displacement audits.',
        runs={})
    assert len(source_audit['runs']) == 9
    for index, (case, evidence) in enumerate(sorted(source_audit['runs'].items())):
        published = evidence['published_source']
        assert sha(published['run']) == published['run_sha256']
        assert sha(published['solution']) == published['solution_sha256']
        source = read(published['run'])
        historical_path = Path(extra['runs'][case]['source_run_record'])
        historical = read(historical_path)
        assert historical['returncode'] == source['returncode'] == 0
        inputs = {item['path']: item['sha256'] for item in historical['inputs']}
        argv = [str(binary)] + source['argv'][1:]
        assert '--android-raw-clock-only' in argv
        assert all(flag not in argv for flag in clock + recipe)
        assert '--truth' not in argv
        raw_path = argv[argv.index('--android-gnss') + 1]
        assert sha(raw_path) == evidence['raw_sha256']
        for flag in ['--android-gnss', '--android-imu', '--nav', '--native-base-rinex']:
            assert argv[argv.index(flag) + 1] in inputs
        if published['height_mode'] == 'map':
            map_path = argv[argv.index('--native-height-map') + 1]
            inputs[map_path] = sha(map_path)
        else:
            assert published['height_mode'] == 'relative' and '--native-relative-height-pairs' in argv
        assert all(sha(p) == h for p, h in inputs.items())
        entry = dict(dataset_id=case, published_source=published, inputs_sha256=inputs,
                     historical_input_manifest=str(historical_path),
                     historical_input_manifest_sha256=sha(historical_path), argv={})
        for arm, flags in arms.items():
            command = argv.copy()
            folder = args.out_dir / 'runs' / f'{index:02d}' / arm
            for flag, filename in [('--out', 'solution.csv'), ('--summary-json', 'summary.json')]:
                command[command.index(flag) + 1] = str((folder / filename).resolve())
            entry['argv'][arm] = command + flags
        plan['runs'][case] = entry
    args.out_dir.mkdir(parents=True, exist_ok=False)
    path = args.out_dir / 'plan.json'
    path.write_text(json.dumps(plan, indent=2) + '\n')
    print(json.dumps(dict(status=plan['status'], phones=len(plan['runs']), arms=plan['arm_order'],
                         plan=str(path), plan_sha256=sha(path)), indent=2))


if __name__ == '__main__':
    main()
