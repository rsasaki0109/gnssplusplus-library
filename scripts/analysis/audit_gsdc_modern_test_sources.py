"""Audit published sources and clock evidence for the modern-phone test cohort.

This prepares evidence only; it neither runs inference nor creates a submission.
"""
import argparse
import csv
import json
from pathlib import Path

from compare_gsdc_doppler_diagnostic import positions
from run_gsdc_doppler_segment_diagnostic import sha


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference-plan', type=Path, required=True)
    parser.add_argument('--development-report', type=Path, required=True)
    parser.add_argument('--clock-inventory', type=Path, required=True)
    parser.add_argument('--clock-admission', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    plan = read(args.reference_plan)
    development = read(args.development_report)
    assert development['status'] == 'all-planned-pairs-audited'
    assert development['expected_pairs'] == development['audited_pairs'] == 11
    assert development['native_recipe_effects_verified']
    assert development['height_map_group_independence_verified']
    phones = {case.split('/')[1] for case in development['runs']}
    assert phones == {'pixel6pro', 'pixel7pro', 'sm-g988b', 'sm-s908b'}
    inventory = {row['dataset_id']: row for row in read(args.clock_inventory)['rows']}
    admission = read(args.clock_admission)['runs']
    cases = {case: source for case, source in plan['retained_sources'].items()
             if case.split('/')[1] in phones}
    assert len(cases) == 9 and plan['reference_submission_ref'] == 56536540
    output = {}
    for case, source in cases.items():
        folder = Path(source['run']).parent
        assert sha(source['run']) == source['run_sha256']
        assert sha(source['solution']) == source['solution_sha256']
        assert sha(folder / 'summary.json') == source['summary_sha256']
        run, summary = read(source['run']), read(folder / 'summary.json')
        assert run['returncode'] == 0 and run['dataset_id'] == case
        assert sha(run['argv'][0]) == run['binary_sha256']
        assert not summary['truth_used'] and summary['graph']['converged']
        assert summary['native_pdc_imu_tdcp_no_bridge']
        contract = summary['raw_utc_key_contract']
        assert all(contract[k] == 0 for k in ['interpolated_epochs', 'edge_hold_epochs', 'unresolved_epochs'])
        solution = positions(Path(source['solution']), case)
        raw = Path(run['argv'][run['argv'].index('--android-gnss') + 1])
        with raw.open(newline='', encoding='utf-8-sig') as stream:
            keys = {int(row['utcTimeMillis']) for row in csv.DictReader(stream)}
        assert keys == set(solution)
        assert len(keys) == contract['exact_solution_epochs'] == contract['raw_epoch_keys']
        clock = inventory[case]
        assert sha(clock['summary']) == clock['summary_sha256'] == source['summary_sha256']
        resets = summary['android_gnss_diagnostics']['clock_discontinuities']
        assert resets == clock['clock_discontinuities']
        evidence = None
        if resets:
            evidence = admission['test/' + case]
            assert sha(raw) == evidence['raw_sha256']
            for mode in ['legacy', 'continuous']:
                item = evidence['modes'][mode]
                assert item['returncode'] == 0
                assert sha(item['output']) == item['output_sha256']
            assert evidence['modes']['legacy']['keys_sha256'] == evidence['modes']['continuous']['keys_sha256']
        output[case] = dict(published_source=source, raw_sha256=sha(raw), native_keys=len(keys),
                            legacy_clock_resets=resets, loader_admission=evidence)
    report = dict(status='published-modern-test-sources-audited', inference_started=False,
                  truth_consumed=False, official_score=False, submitted=False,
                  selection='All test phones matching the four device models in the fixed development cohort',
                  input_evidence={str(p): sha(p) for p in [args.reference_plan, args.development_report,
                      args.clock_inventory, args.clock_admission]}, runs=output,
                  next_gate='Freeze a same-binary replay that separates continuous-clock changes from the three recipe flags; published-control parity and full native inference remain unverified.')
    args.out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(dict(status=report['status'], phones=len(output),
                         reset_cases=[case for case, row in output.items() if row['legacy_clock_resets']]), indent=2))


if __name__ == '__main__':
    main()
