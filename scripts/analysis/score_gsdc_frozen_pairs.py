"""Audit every planned pair before reporting a full development-set mean."""
import argparse
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys

from run_gsdc_doppler_segment_diagnostic import sha
from gsdc_development_plan_audit import audit_height_independence, provenance_hash


def audit_recipe_effects(report, flags):
    """Require native telemetry to show the requested factors and weights."""
    if '--android-continuous-clock-reference' in flags:
        clock = report['continuous_clock_reference']
        assert clock['candidate_clock_discontinuities'] == 0
        assert 'retained across forward gaps' in clock['timing_formula']
    control, candidate = (report['runs'][mode] for mode in ['control', 'candidate'])
    a, b = control['tdcp'], candidate['tdcp']
    if '--native-source-tdcp-meter-sigma' in flags:
        assert a['source_tdcp_meter_sigma_requested'] is False
        assert b['source_tdcp_meter_sigma_requested'] is True
        assert b['fixed_sigma_m'] is None
        assert b['factors_built'] > 0 and b['factors_inserted'] > 0
        assert math.isfinite(b['sigma_m']) and b['sigma_m'] > 0
    if '--native-phase184-source-tdcp-huber-k' in flags:
        assert a['phase184_source_tdcp_huber_k_enabled'] is False
        assert b['phase184_source_tdcp_huber_k_enabled'] is True
        setting = b['phase184_source_tdcp_setting_type']
        assert setting in ['Highway', 'Street', 'Mix']
        assert b['official_huber_k'] == (0.5 if setting == 'Highway' else 0.2)
    if '--native-phase213-main-doppler' in flags:
        x, y = control['main_doppler'], candidate['main_doppler']
        assert x['requested'] is False and x['enabled'] is False and x['factors'] == 0
        assert y['requested'] is True and y['enabled'] is True and y['factors'] > 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plan', type=Path)
    parser.add_argument('--truth-root', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text())
    plan_hash = sha(args.plan)
    assert plan['runs']
    for case in plan['runs']:
        audit_height_independence(plan, case)
        provenance_hash(args.plan, plan, case, require_done=True)
    if plan.get('schema') != 'gsdc-repaired-frozen-development.v1':
        done = json.loads((args.plan.parent / 'execution.done.json').read_text())
        assert done['plan_sha256'] == plan_hash and done['native_execution_complete'] is True
        assert set(done['runs']) == set(plan['runs'])
    flags = plan['candidate_flags']
    assert flags
    reports = {}
    comparator = Path(__file__).with_name('compare_gsdc_doppler_diagnostic.py')
    for case, entry in plan['runs'].items():
        folders = {}
        for mode in ['control', 'candidate']:
            argv = entry['argv'][mode]
            folder = Path(argv[argv.index('--out') + 1]).parent
            record = json.loads((folder / 'run.json').read_text())
            assert record['argv'] == argv
            assert record['source_run_sha256'] == provenance_hash(args.plan, plan, case)
            assert record['binary_sha256'] == plan['binary_sha256']
            assert record['inputs_sha256'] == entry['inputs_sha256']
            folders[mode] = folder
        output = folders['candidate'].parent / 'comparison.json'
        truth_path = args.truth_root / case / 'ground_truth.csv'
        if 'train_gt_sha256' in plan:
            name = case.replace('/', '__') + '__ground_truth.csv'
            pinned = [digest for path, digest in plan['train_gt_sha256'].items()
                      if Path(path).name == name]
            assert len(pinned) == 1 and sha(truth_path) == pinned[0]
        command = [sys.executable, str(comparator),
                   '--control', str(folders['control']),
                   '--candidate', str(folders['candidate']),
                   '--candidate-flag=' + flags[0],
                   '--truth', str(truth_path),
                   '--out', str(output)]
        command += ['--additional-candidate-flag=' + flag for flag in flags[1:]]
        subprocess.run(command, check=True, stdout=subprocess.DEVNULL)
        report = json.loads(output.read_text())
        assert report['dataset_id'] == case
        audit_recipe_effects(report, flags)
        reports[case] = dict(
            comparison=str(output), comparison_sha256=sha(output),
            control=report['runs']['control']['score'],
            candidate=report['runs']['candidate']['score'],
            delta_phone_score_m=report['delta_phone_score_m'])
        print(case, reports[case]['delta_phone_score_m'], flush=True)
    result = dict(status='all-planned-pairs-audited', official_score=False,
                  heldout=False, plan_sha256=plan_hash,
                  expected_pairs=len(plan['runs']), audited_pairs=len(reports),
                  native_recipe_effects_verified=True,
                  height_map_group_independence_verified=True,
                  aggregation='unweighted mean of per-phone (P50 + P95) / 2',
                  runs=reports)
    for mode in ['control', 'candidate']:
        result[mode + '_mean_m'] = statistics.mean(
            report[mode]['phone_score_m'] for report in reports.values())
    result['delta_mean_m'] = result['candidate_mean_m'] - result['control_mean_m']
    result['regressed_pairs'] = sum(
        report['delta_phone_score_m'] > 0 for report in reports.values())
    args.out.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k != 'runs'}, indent=2))


if __name__ == '__main__':
    main()
