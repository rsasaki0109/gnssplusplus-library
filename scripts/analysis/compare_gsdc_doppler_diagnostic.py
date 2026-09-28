"""Audit and score a completed diagnostic pair on exposed development truth."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from run_gsdc_doppler_segment_diagnostic import sha


def positions(path, phone=None):
    with path.open(newline='', encoding='utf-8-sig') as stream:
        rows = list(csv.DictReader(stream))
    result = {}
    for row in rows:
        key = int(row['UnixTimeMillis'])
        assert key not in result, 'duplicate UTC key'
        if phone is not None:
            assert row['phone'] == phone
        xy = float(row['LatitudeDegrees']), float(row['LongitudeDegrees'])
        assert np.isfinite(xy).all() and abs(xy[0]) <= 90 and abs(xy[1]) <= 180
        result[key] = xy
    assert result
    return result


def audited_run(folder):
    record = json.loads((folder / 'run.json').read_text())
    assert record['state'] == 'complete' and record['returncode'] == 0, 'run incomplete'
    assert sha(record['argv'][0]) == record['binary_sha256']
    assert sha(folder / 'solution.csv') == record['solution_sha256']
    assert sha(folder / 'summary.json') == record['summary_sha256']
    assert all(sha(path) == digest for path, digest in record['inputs_sha256'].items())
    summary = json.loads((folder / 'summary.json').read_text())
    assert summary['truth_used'] is False and summary['native_pdc_imu_tdcp_no_bridge'] is True
    if '--android-continuous-clock-reference' in record['argv']:
        clock = summary['android_gnss_diagnostics']
        assert 'retained across forward gaps' in clock['timing_formula']
        assert clock['clock_discontinuities'] == 0
    assert summary['status'] == 'imu-combined-factor' and summary['graph']['converged']
    contract = summary['raw_utc_key_contract']
    assert all(contract[key] == 0 for key in ['interpolated_epochs', 'edge_hold_epochs', 'unresolved_epochs'])
    output = positions(folder / 'solution.csv', summary['dataset_id'])
    raw_path = record['argv'][record['argv'].index('--android-gnss') + 1]
    with Path(raw_path).open(newline='', encoding='utf-8-sig') as stream:
        raw_keys = {int(row['utcTimeMillis']) for row in csv.DictReader(stream)}
    assert set(output) == raw_keys
    assert contract['exact_solution_epochs'] == len(output) == contract['raw_epoch_keys']
    return record, summary, output


def score(solution, truth):
    keys = sorted(truth)
    assert set(keys) <= solution.keys(), 'missing truth keys'
    a, b = (np.radians([table[k] for k in keys]) for table in [solution, truth])
    h = np.sin((a[:, 0] - b[:, 0]) / 2)**2 + np.cos(a[:, 0]) * np.cos(b[:, 0]) * np.sin((a[:, 1] - b[:, 1]) / 2)**2
    error = 2 * 6371008.8 * np.arcsin(np.sqrt(np.clip(h, 0, 1)))
    p50, p95 = np.percentile(error, [50, 95], method='linear')
    return dict(truth_keys=len(keys), p50_m=float(p50), p95_m=float(p95), phone_score_m=float((p50 + p95) / 2))


def normalized_argv(argv):
    result = argv.copy()
    for flag in ['--out', '--summary-json']:
        result[result.index(flag) + 1] = '<output>'
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--control', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--candidate-flag', required=True)
    parser.add_argument('--additional-candidate-flag', action='append', default=[])
    parser.add_argument('--truth', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    control, candidate = audited_run(args.control), audited_run(args.candidate)
    a, b = control[0], candidate[0]
    assert a['binary_sha256'] == b['binary_sha256']
    assert a['inputs_sha256'] == b['inputs_sha256']
    assert a['source_run_sha256'] == b['source_run_sha256']
    candidate_flags = [args.candidate_flag] + args.additional_candidate_flag
    assert normalized_argv(b['argv']) == normalized_argv(a['argv']) + candidate_flags
    assert control[1]['dataset_id'] == candidate[1]['dataset_id']
    assert set(control[2]) == set(candidate[2])
    truth = positions(args.truth)
    report = dict(status='audited-development-pair', official_score=False, heldout=False,
                  dataset_id=control[1]['dataset_id'], truth_sha256=sha(args.truth),
                  binary_sha256=a['binary_sha256'], candidate_flag=args.candidate_flag, runs={})
    report['candidate_flags'] = candidate_flags
    for name, (record, summary, solution) in [('control', control), ('candidate', candidate)]:
        report['runs'][name] = dict(score=score(solution, truth),
            solution_sha256=record['solution_sha256'], summary_sha256=record['summary_sha256'],
            wall_s=record['wall_s'], native_raw_keys=len(solution),
            tdcp=summary['tdcp_contract'], main_doppler=summary['graph']['phase213_main_doppler'])
    report['delta_phone_score_m'] = report['runs']['candidate']['score']['phone_score_m'] - report['runs']['control']['score']['phone_score_m']
    if '--android-continuous-clock-reference' in candidate_flags:
        c, d = control[1]['android_gnss_diagnostics'], candidate[1]['android_gnss_diagnostics']
        assert 'reset when successive' in c['timing_formula']
        assert 'retained across forward gaps' in d['timing_formula']
        assert d['clock_discontinuities'] == 0
        report['continuous_clock_reference'] = dict(
            control_clock_discontinuities=c['clock_discontinuities'],
            candidate_clock_discontinuities=d['clock_discontinuities'],
            timing_formula=d['timing_formula'])
    if args.candidate_flag == '--native-refinement-no-doppler-initialization':
        c, d = control[1]['imu_initialization'], candidate[1]['imu_initialization']
        assert c['refinement_no_doppler_initialization_requested'] is False
        assert d['refinement_no_doppler_initialization_requested'] is True
        assert c['refinement_initial_main_doppler_factors'] > 0 and d['refinement_initial_main_doppler_factors'] == 0
        assert candidate[1]['graph']['phase213_main_doppler']['factors'] > 0
        stage = 'summary.json.gnss-first.csv'
        assert (args.control / stage).read_bytes() == (args.candidate / stage).read_bytes()
        report['gnss_first_stage_byte_identical'] = True
        if d.get('refinement_gnss_drift_handoff'):
            assert c['refinement_gnss_drift_handoff'] is False
            stage = 'summary.json.gnss-first.clocks.csv'
            assert (args.control / stage).read_bytes() == (args.candidate / stage).read_bytes()
            report['same_run_gnss_drift_handoff'] = True
    if '--native-refinement-observed-clock-drift' in candidate_flags:
        c, d = control[1]['imu_initialization'], candidate[1]['imu_initialization']
        assert c['refinement_observed_clock_drift_requested'] is False
        assert c['refinement_observed_drift_epochs'] == 0
        assert d['refinement_observed_clock_drift_requested'] is True
        assert d['refinement_gnss_drift_handoff'] is False
        assert d['refinement_observed_drift_epochs'] == len(candidate[2])
        stage = 'summary.json.gnss-first.clocks.csv'
        assert (args.control / stage).read_bytes() == (args.candidate / stage).read_bytes()
        report['observed_drift_all_epochs'] = True
    args.out.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k != 'runs'}, indent=2))


if __name__ == '__main__':
    main()
