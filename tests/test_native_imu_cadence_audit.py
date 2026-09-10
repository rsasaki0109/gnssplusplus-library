import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    'cadence', Path(__file__).resolve().parents[1] /
    'scripts/audit_native_imu_cadence.py')
cadence = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cadence)


class CadenceAuditTest(unittest.TestCase):
    def test_staggered_clocks(self):
        r = cadence.support([0, 20, 40], [0, 5, 25, 40])
        self.assertEqual(r['exact'], 2)
        self.assertEqual(r['interpolation_span']['max_ms'], 20)
        self.assertEqual(r['interpolation_nearest_offset']['max_ms'], 5)
        self.assertEqual(r['outside_accel_range'], 0)

    def test_long_gap_and_outside(self):
        r = cadence.support([0, 1000], [-1, 1, 999, 1001])
        self.assertEqual(r['interpolation_spans_over_100ms'], 2)
        self.assertEqual(r['outside_accel_range'], 2)

    def test_deduplicates_and_sorts(self):
        r = cadence.support([20, 0, 20], [20, 0, 0])
        self.assertEqual(r['exact'], 2)
        self.assertEqual(r['accel_unique'], 2)
        self.assertEqual(r['interpolation_span'], {'count': 0})

    def test_empty(self):
        self.assertEqual(cadence.support([], [1])['outside_accel_range'], 1)


if __name__ == '__main__':
    unittest.main()
