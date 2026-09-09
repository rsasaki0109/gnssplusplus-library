"""Aggregate validation only; synthetic values, no data files."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from verify_phase519_ionosphere_monitor import parse_monitor


class IrlsMonitorTest(unittest.TestCase):
    valid = ('[native-code-ionosphere-irls] epochs=3 rows=30 invalid=0 '
             'downweighted=10 nominal_median=0.9 irls_median=0.2 factors_changed=0')

    def test_valid(self):
        self.assertEqual(parse_monitor(self.valid)['rows'], 30)

    def test_invalid(self):
        for text in ('', self.valid+'\n'+self.valid,
                     self.valid.replace('invalid=0', 'invalid=1'),
                     self.valid.replace('epochs=3', 'epochs=0'),
                     self.valid.replace('rows=30', 'rows=2'),
                     self.valid.replace('downweighted=10', 'downweighted=31'),
                     self.valid.replace('irls_median=0.2', 'irls_median=nan'),
                     self.valid.replace('irls_median=0.2', 'irls_median=-1'),
                     self.valid.replace('irls_median=0.2', 'irls_median=1.0')):
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_monitor(text)


if __name__ == '__main__':
    unittest.main()
