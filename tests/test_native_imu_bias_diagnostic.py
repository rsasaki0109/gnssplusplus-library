import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from verify_native_imu_bias_diagnostic import check_bias, check_information, parse_topology, parse_admission, parse_rejected, parse_matched_pd


class BiasDiagnosticTest(unittest.TestCase):
    def test_failed_lambda_ranges_reject_malformed_values(self):
        from verify_native_imu_bias_diagnostic import parse_failed_lambda
        line = '[native-lm-failed-lambda] count=2 min=1e-9 max=1e-8 nearby_key=unavailable'
        self.assertEqual(parse_failed_lambda(line)[0]['count'], 2)
        for bad in (line.replace('count=2', 'count=0'),
                    line.replace('min=1e-9', 'min=nan'),
                    line.replace('min=1e-9', 'min=1'),
                    line.replace('nearby_key=unavailable', 'nearby_key=c1')):
            with self.subTest(line=bad), self.assertRaises(AssertionError):
                parse_failed_lambda(bad)

    def test_matched_pd_conserves_every_population_and_direction(self):
        lines = [f'[native-p-matched-pd] epoch=1 population={p} direction={d} '
                 'missing=1 positive=1 negative=0 zero=0 max_abs_m=4'
                 for p in ('retained', 'rejected') for d in ('incoming', 'outgoing')]
        log = '\n'.join(lines)
        admission = {1: {'before': 4, 'after': 2}}
        self.assertEqual(len(parse_matched_pd(log, admission)[1]), 4)
        for bad in ('', '\n'.join(lines[:-1]), log+'\n'+lines[0],
                    log.replace('missing=1', 'missing=0'),
                    log.replace('max_abs_m=4', 'max_abs_m=nan'),
                    log.replace('epoch=1', 'epoch=2')):
            with self.subTest(log=bad), self.assertRaises(AssertionError):
                parse_matched_pd(bad, admission)

    def test_rejected_signs_must_account_for_all_removed_rows(self):
        line = '[native-p-rejected] epoch=1 positive=1 negative=2 zero=0 nonfinite=0 max_abs_m=30'
        admission = {1: {'before': 4, 'after': 1}}
        self.assertEqual(parse_rejected(line, admission)[1]['negative'], 2)
        for text in ('', line+'\n'+line, line.replace('negative=2','negative=1'),
                     line.replace('max_abs_m=30','max_abs_m=nan'),
                     line.replace('epoch=1','epoch=2')):
            with self.subTest(text=text), self.assertRaises(AssertionError):
                parse_rejected(text, admission)

    def test_admission_counts_cannot_increase_or_duplicate(self):
        line = '[native-p-admission] epoch=1 before_centered_residual=8 after_centered_residual=1'
        self.assertEqual(parse_admission(line, 4), {1: {'before': 8, 'after': 1}})
        for text in (line+'\n'+line, line.replace('epoch=1','epoch=4'),
                     line.replace('after_centered_residual=1','after_centered_residual=9'),
                     line+' extra=1'):
            with self.subTest(text=text), self.assertRaises(AssertionError):
                parse_admission(text, 4)

    def test_topology_parser_rejects_malformed_duplicate_and_missing_entries(self):
        line = ('[nominal-p-information] epoch=1 p_rows=3 clock_rank=2 '
                'incoming_tdcp=2 outgoing_tdcp=1 min_eigenvalue=0')
        self.assertEqual(parse_topology(line, 4, 1)[0]['clock_rank'], 2)
        for log, count in [(line+'\n'+line, 2), ('', 1),
                           (line.replace('epoch=1', 'epoch=4'), 1),
                           (line.replace('clock_rank=2', 'clock_rank=4'), 1),
                           (line.replace('min_eigenvalue=0', 'min_eigenvalue=nan'), 1),
                           (line+' unexpected=1', 1)]:
            with self.subTest(log=log), self.assertRaises(AssertionError):
                parse_topology(log, 4, count)

    def test_information_scope_and_aggregate_guards(self):
        valid = dict(nominal_p_information_epochs=2,
                     nominal_p_information_rank_deficient_epochs=1,
                     nominal_p_information_min_eigenvalue_per_m2=0.,
                     nominal_p_information_scope='single-epoch nominal P; clocks projected; no robust or temporal weights')
        check_information(valid, 2)
        for key, values in {
            'nominal_p_information_epochs': [0, 3],
            'nominal_p_information_rank_deficient_epochs': [-1, 3, 0.5, True],
            'nominal_p_information_min_eigenvalue_per_m2': [-1, math.nan, math.inf],
            'nominal_p_information_scope': ['full FGO posterior'],
        }.items():
            for value in values:
                with self.subTest(key=key, value=value), self.assertRaises(AssertionError):
                    check_information(dict(valid, **{key: value}), 2)

    def test_counts_finite_nonnegative_and_frequency_off(self):
        valid = dict(optimized_imu_bias_count=2,
                     optimized_accel_bias_max_norm_mps2=0.1,
                     optimized_gyro_bias_max_norm_radps=0.01,
                     frequency_residual_state_count=0,
                     frequency_residual_factor_count=0,
                     frequency_residual_prior_count=0)
        check_bias(valid, 2)
        for key, values in {
            'optimized_imu_bias_count': [0, 1, 3],
            'optimized_accel_bias_max_norm_mps2': [-1, math.nan, math.inf],
            'optimized_gyro_bias_max_norm_radps': [-1, math.nan, math.inf],
            'frequency_residual_state_count': [1],
            'frequency_residual_factor_count': [2],
            'frequency_residual_prior_count': [1],
        }.items():
            for value in values:
                with self.subTest(key=key, value=value), self.assertRaises(AssertionError):
                    check_bias(dict(valid, **{key: value}), 2)


if __name__ == '__main__':
    unittest.main()
