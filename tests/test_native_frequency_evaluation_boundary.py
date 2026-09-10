"""Synthetic evaluation-boundary tests; never access real GNSS/truth payloads."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch, Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import evaluate_native_frequency_experiment as evaluator


class FrequencyEvaluationBoundaryTest(unittest.TestCase):
    def test_failed_structure_never_loads_scoring_kernel(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = Path(directory) / 'manifest.json'
            kernel_path = 'apps/commands/benchmarks/gnss_smartphone_phase203_phase202_h_accuracy.py'
            manifest.write_text(json.dumps({
                'role': 'development/train; not heldout or leaderboard proof',
                'evaluator_path': kernel_path, 'source_pins': {kernel_path: 'unused'}}))
            with patch.object(evaluator, 'verify', side_effect=ValueError('structure failed')), \
                 patch.object(evaluator, 'load_module') as load:
                with self.assertRaisesRegex(ValueError, 'structure failed'):
                    evaluator.evaluate(manifest)
                load.assert_not_called()
            self.assertEqual(sorted(p.name for p in Path(directory).iterdir()), ['manifest.json'])

    def test_exclusive_claim_prevents_second_score_after_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest_path = root / 'manifest.json'
            manifest_path.write_text('{}')
            manifest = {'role': 'development/train; not heldout or leaderboard proof',
                        'result_path': 'result.json', 'evaluator_path': 'kernel.py',
                        'candidates': [{'path': 'candidate.csv', 'truth': {'path': 'truth.csv'},
                                        'route': 'synthetic/pixel5', 'phase': 422, 'baseline_m': 1.0}]}
            kernel = Mock()
            kernel.score_payloads.side_effect = ValueError('synthetic scoring failure')
            with patch.object(evaluator, 'ROOT', root), \
                 patch.object(evaluator, 'preflight', return_value=manifest), \
                 patch.object(evaluator, 'load_module', return_value=kernel):
                with self.assertRaisesRegex(ValueError, 'synthetic scoring failure'):
                    evaluator.evaluate(manifest_path)
                with self.assertRaises(FileExistsError):
                    evaluator.evaluate(manifest_path)
            self.assertEqual(kernel.score_payloads.call_count, 1)
            self.assertTrue((root / 'result.attempt.json').exists())
            self.assertFalse((root / 'result.json').exists())


if __name__ == '__main__':
    unittest.main()
