import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from verify_native_lambda_floor_experiment import check_stage


class LambdaFloorVerifierTest(unittest.TestCase):
    def test_requires_effective_floor_and_unchanged_solver(self):
        stage = dict(lambda_lower_bound=1e-8, initial_lambda=1e-5,
                     linear_solver='MULTIFRONTAL_QR', diagonal_damping=False,
                     costs_finite=True, termination_trace_complete=True, final_cost=1.)
        check_stage(stage)
        for key, value in [('lambda_lower_bound', 0.), ('initial_lambda', 1e-8),
                           ('linear_solver', 'MULTIFRONTAL_CHOLESKY'),
                           ('diagonal_damping', True), ('final_cost', float('nan')),
                           ('termination_trace_complete', False)]:
            with self.subTest(key=key), self.assertRaises(AssertionError):
                check_stage(dict(stage, **{key: value}))
