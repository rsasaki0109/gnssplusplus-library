import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('timing', Path(__file__).resolve().parents[1] /
    'scripts/validate_tdcp_frequency_raw_timing.py')
timing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(timing)


def row():
    return dict(MessageType='Raw', ConstellationType='1', utcTimeMillis='1000', Svid='1',
                TimeNanos='1000000000', FullBiasNanos='-1000000000000', BiasNanos='0',
                HardwareClockDiscontinuityCount='0', TimeOffsetNanos='0')


class TimingPreflightTest(unittest.TestCase):
    def test_matching_rows(self):
        self.assertEqual(timing.validate_rows([row(),row()])['validated_rows'], 2)

    def test_missing_nonfinite_and_nonzero_offsets(self):
        for value in ['', 'NaN', 'Infinity', '0.1', '-1']:
            with self.subTest(value=value), self.assertRaises(ValueError):
                timing.validate_rows([dict(row(),TimeOffsetNanos=value)])

    def test_each_clock_component_mismatch(self):
        for key in ['TimeNanos','FullBiasNanos','BiasNanos','HardwareClockDiscontinuityCount']:
            with self.subTest(key=key), self.assertRaises(ValueError):
                timing.validate_rows([row(),dict(row(),**{key:'2'})])

    def test_empty_scope_rejected(self):
        with self.assertRaises(ValueError): timing.validate_rows([])


if __name__ == '__main__': unittest.main()
