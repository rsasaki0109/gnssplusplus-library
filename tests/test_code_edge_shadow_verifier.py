import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from verify_phase505_identity_shadow import parse_intersection

class CodeEdgeShadowVerifier(unittest.TestCase):
    def test_counts_and_missing_or_duplicate_markers(self):
        raw = '[native-code-edge-candidates] raw_candidates=10 factors_added=0\n'
        shadow = '[native-code-edge-shadow] geometry_quality_pass=8 residual_pass=5 retained_epoch_pass=4 factors_added=0\n'
        self.assertEqual(parse_intersection(raw+shadow)['retained'],4)
        for text in (raw, shadow, raw+raw+shadow, raw+shadow+shadow,
                     raw+shadow.replace('retained_epoch_pass=4','retained_epoch_pass=6'),
                     raw+shadow.replace('factors_added=0','factors_added=1')):
            with self.assertRaises(ValueError): parse_intersection(text)

if __name__ == '__main__': unittest.main()
