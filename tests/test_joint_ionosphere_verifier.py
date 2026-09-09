"""Negative controls for native joint diagnostic validation."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from verify_phase536_joint_ionosphere import parse_joint

LOG = ('[native-joint-ionosphere] states=3140 priors=3140 code=101916 tdcp=69270 '
       'anchor_sigma=3 density=0.02 max_gap=1.5\n'
       '[native-joint-ionosphere-solved] states=3140 max_abs_m=2 rms_m=1 '
       'max_code_m=8 max_tdcp_m=0.1 clipped=0\n')


class JointVerifier(unittest.TestCase):
    def test_valid(self):
        self.assertEqual(parse_joint(LOG)['max_abs_m'], 2)

    def test_rejects_invalid(self):
        controls = [('', LOG), ('code=101916', 'code=101915'),
                    ('density=0.02', 'density=0.03'),
                    ('rms_m=1', 'rms_m=3'), ('max_abs_m=2', 'max_abs_m=nan'),
                    ('max_code_m=8', 'max_code_m=inf'),
                    ('max_tdcp_m=0.1', 'max_tdcp_m=-1'),
                    ('clipped=0', 'clipped=1')]
        for old, new in controls:
            with self.subTest(old=old), self.assertRaises(ValueError):
                parse_joint(LOG.replace(old, new) if old else LOG + new)
        with self.assertRaises(ValueError):
            parse_joint('')


if __name__ == '__main__':
    unittest.main()
