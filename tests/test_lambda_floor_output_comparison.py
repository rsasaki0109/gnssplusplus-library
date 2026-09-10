import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from compare_lambda_floor_outputs import separation


class SeparationTest(unittest.TestCase):
    def test_identity_and_symmetry(self):
        a, b = (35., 139.), (35.001, 139.002)
        self.assertEqual(separation(a, a), 0.)
        self.assertAlmostEqual(separation(a, b), separation(b, a), places=9)

    def test_equator_and_dateline(self):
        self.assertAlmostEqual(separation((0., 0.), (0., 90.)),
                               6371008.8*math.pi/2, places=6)
        self.assertAlmostEqual(separation((0., 179.), (0., -179.)),
                               6371008.8*math.pi/90, places=6)
