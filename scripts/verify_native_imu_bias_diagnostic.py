"""Verify baseline-only raw IMU bias diagnostics; no truth or position parsing."""
import hashlib
import json
import math
import re
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def check_bias(tdcp, expected_epochs):
    assert tdcp['optimized_imu_bias_count'] == expected_epochs
    for key in ('optimized_accel_bias_max_norm_mps2', 'optimized_gyro_bias_max_norm_radps'):
        assert math.isfinite(tdcp[key]) and tdcp[key] >= 0, key
    for key in ('frequency_residual_state_count', 'frequency_residual_factor_count',
                'frequency_residual_prior_count'):
        assert tdcp[key] == 0, key


def verify(phase):
    tag, epochs = {431: ('h_baseline_replay', 3140),
                   432: ('lax_t_comparison', 1466),
                   438: ('h_baseline_replay', 3140),
                   439: ('lax_t_comparison', 1466),
                   442: ('lax_t_comparison', 1466),
                   448: ('lax_t_comparison', 1466),
                   451: ('lax_t_comparison', 1466),
                   455: ('lax_t_comparison', 1466),
                   469: ('lax_t_comparison', 1466),
                   472: ('lax_t_comparison', 1466)}[phase]
    path = ROOT / f'docs/use_cases/records/smartphone_r5_phase{phase}_{tag}_manifest_v1.json'
    manifest = json.loads(path.read_text())
    directory = (ROOT / manifest['launcher_contract']['execution_metadata_path']).parent
    completed = json.loads((directory / 'launcher_completed.json').read_text())
    assert completed['return_code'] == 0 and completed['native_invocations'] == 1
    assert completed['manifest_sha256'] == digest(path)
    for group in ('source_pins', 'test_pins', 'algorithm_source_pins'):
        for name, expected in manifest[group].items():
            assert digest(ROOT / name) == expected, name
    for item in (manifest['binary'], *manifest['inputs'].values()):
        source = ROOT / item['path']
        assert source.stat().st_size == item['bytes'] and digest(source) == item['sha256']
    assert '--native-tdcp-frequency-residual-states' not in manifest['argv']
    expected = (manifest['expected_output_identity']['sha256'] if phase in (431, 438)
                else manifest['expected_candidate_sha256'])
    assert digest(directory / 'opaque_solution_output.csv') == expected
    summary = json.loads((directory / 'native_summary.json').read_text())
    assert summary['graph']['converged'] and summary['gnss_first']['converged']
    tdcp = summary['tdcp_contract']
    check_bias(tdcp, epochs)
    information = {}
    if phase in (438, 439, 442, 448, 451, 455, 469, 472):
        check_information(tdcp, epochs)
        information = {key: value for key, value in tdcp.items()
                       if key.startswith('nominal_p_information_')}
    if phase in (442, 448, 451, 455, 469, 472):
        information['deficient_topology'] = parse_topology(
            (directory / 'native.stderr.log').read_text(), epochs,
            tdcp['nominal_p_information_rank_deficient_epochs'])
    if phase in (448, 451, 455, 469, 472):
        admission = parse_admission((directory / 'native.stderr.log').read_text(), epochs)
        for entry in information['deficient_topology']:
            assert admission[entry['epoch']]['after'] == entry['p_rows']
        information['deficient_admission'] = {
            entry['epoch']: admission[entry['epoch']]
            for entry in information['deficient_topology']}
        if phase in (451, 455, 469, 472):
            rejected = parse_rejected((directory / 'native.stderr.log').read_text(), admission)
            information['deficient_rejected_residuals'] = {
                entry['epoch']: rejected[entry['epoch']]
                for entry in information['deficient_topology']}
        if phase in (455, 469, 472):
            matched = parse_matched_pd((directory / 'native.stderr.log').read_text(), admission)
            information['deficient_matched_pd'] = {
                entry['epoch']: matched[entry['epoch']]
                for entry in information['deficient_topology']}
    if phase in (469, 472):
        log = (directory / 'native.stderr.log').read_text()
        ranges = parse_failed_lambda(log)
        # Last optimizer in this frozen recipe is main; its nonzero counter
        # requires a corresponding final emitted aggregate.
        failures = summary['native_source_clock_c0d_factor']['indeterminate_linear_solve_count']
        assert failures > 0 and ranges and ranges[-1]['count'] == failures
        information['failed_lambda_ranges_in_execution_order'] = ranges
    return {'phase': phase, 'baseline_output_identical': True,
            'optimized_bias_states': tdcp['optimized_imu_bias_count'],
            'accel_bias_max_norm_mps2': tdcp['optimized_accel_bias_max_norm_mps2'],
            'gyro_bias_max_norm_radps': tdcp['optimized_gyro_bias_max_norm_radps'],
            'truth_reads': 0, 'accuracy_evaluations': 0, **information}


def check_information(tdcp, expected_epochs):
    assert tdcp['nominal_p_information_epochs'] == expected_epochs
    deficient = tdcp['nominal_p_information_rank_deficient_epochs']
    assert type(deficient) is int and 0 <= deficient <= expected_epochs
    minimum = tdcp['nominal_p_information_min_eigenvalue_per_m2']
    assert math.isfinite(minimum) and minimum >= 0
    assert tdcp['nominal_p_information_scope'] == (
        'single-epoch nominal P; clocks projected; no robust or temporal weights')


def parse_topology(log, epochs, expected_count):
    entries = []
    pattern = re.compile(r'\[nominal-p-information\] epoch=(\d+) p_rows=(\d+) '
                         r'clock_rank=(\d+) incoming_tdcp=(\d+) outgoing_tdcp=(\d+) '
                         r'min_eigenvalue=([^\s]+)')
    for line in log.splitlines():
        if '[nominal-p-information]' not in line:
            continue
        match = pattern.fullmatch(line)
        assert match, 'malformed topology diagnostic'
        epoch, rows, rank, incoming, outgoing = map(int, match.groups()[:5])
        eigenvalue = float(match.group(6))
        assert 0 <= epoch < epochs and 0 <= rank <= min(7, rows)
        assert math.isfinite(eigenvalue) and eigenvalue >= 0
        entries.append(dict(epoch=epoch, p_rows=rows, clock_rank=rank,
                            incoming_tdcp=incoming, outgoing_tdcp=outgoing,
                            min_eigenvalue=eigenvalue))
    assert len(entries) == expected_count
    assert len({entry['epoch'] for entry in entries}) == len(entries)
    return entries


def parse_admission(log, epochs):
    result = {}
    pattern = re.compile(r'\[native-p-admission\] epoch=(\d+) '
                         r'before_centered_residual=(\d+) after_centered_residual=(\d+)')
    for line in log.splitlines():
        if '[native-p-admission]' not in line:
            continue
        match = pattern.fullmatch(line)
        assert match, 'malformed admission diagnostic'
        epoch, before, after = map(int, match.groups())
        assert 0 <= epoch < epochs and epoch not in result
        assert 0 <= after <= before and after < 8
        result[epoch] = dict(before=before, after=after)
    return result


def parse_failed_lambda(log):
    result = []
    pattern = re.compile(r'\[native-lm-failed-lambda\] count=(\d+) min=([^\s]+) '
                         r'max=([^\s]+) nearby_key=unavailable')
    for line in log.splitlines():
        if '[native-lm-failed-lambda]' not in line:
            continue
        match = pattern.fullmatch(line)
        assert match, 'malformed failed lambda diagnostic'
        count = int(match.group(1))
        low, high = map(float, match.group(2, 3))
        assert count > 0 and math.isfinite(low) and math.isfinite(high)
        assert 0 <= low <= high
        result.append(dict(count=count, min=low, max=high))
    return result


def parse_matched_pd(log, admission):
    result = {}
    pattern = re.compile(r'\[native-p-matched-pd\] epoch=(\d+) '
                         r'population=(retained|rejected) direction=(incoming|outgoing) '
                         r'missing=(\d+) positive=(\d+) negative=(\d+) zero=(\d+) max_abs_m=([^\s]+)')
    for line in log.splitlines():
        if '[native-p-matched-pd]' not in line:
            continue
        match = pattern.fullmatch(line)
        assert match, 'malformed matched P-D diagnostic'
        epoch = int(match.group(1))
        population, direction = match.group(2, 3)
        counts = list(map(int, match.group(4, 5, 6, 7)))
        maximum = float(match.group(8))
        assert epoch in admission
        key = population + '_' + direction
        groups = result.setdefault(epoch, {})
        assert key not in groups
        expected = admission[epoch]['after'] if population == 'retained' else (
            admission[epoch]['before'] - admission[epoch]['after'])
        assert sum(counts) == expected
        assert math.isfinite(maximum) and maximum >= 0
        if counts[1] + counts[2] == 0:
            assert maximum == 0
        groups[key] = dict(zip(('missing', 'positive', 'negative', 'zero'), counts),
                           max_abs_m=maximum)
    assert result.keys() == admission.keys()
    for groups in result.values():
        assert set(groups) == {p+'_'+d for p in ('retained', 'rejected')
                              for d in ('incoming', 'outgoing')}
    return result


def parse_rejected(log, admission):
    result = {}
    pattern = re.compile(r'\[native-p-rejected\] epoch=(\d+) positive=(\d+) '
                         r'negative=(\d+) zero=(\d+) nonfinite=(\d+) max_abs_m=([^\s]+)')
    for line in log.splitlines():
        if '[native-p-rejected]' not in line:
            continue
        match = pattern.fullmatch(line)
        assert match, 'malformed rejected residual diagnostic'
        epoch, positive, negative, zero, nonfinite = map(int, match.groups()[:5])
        maximum = float(match.group(6))
        assert epoch in admission and epoch not in result
        assert math.isfinite(maximum) and maximum >= 0
        assert positive+negative+zero+nonfinite == admission[epoch]['before']-admission[epoch]['after']
        result[epoch] = dict(positive=positive, negative=negative, zero=zero,
                             nonfinite=nonfinite, max_abs_m=maximum)
    assert result.keys() == admission.keys()
    return result


if __name__ == '__main__':
    print(json.dumps(verify(int(sys.argv[1])), indent=2))
