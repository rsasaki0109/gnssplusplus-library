import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from verify_native_lambda_floor_stages import check_stage


class LambdaFloorStagesTest(unittest.TestCase):
    def test_preserves_distinct_stage_solvers(self):
        for name, solver, elimination in (
            ('main', 'MULTIFRONTAL_QR', 'EliminateQR'),
            ('gnss_first', 'MULTIFRONTAL_CHOLESKY', 'EliminatePreferCholesky')):
            stage = dict(lambda_lower_bound=1e-8, initial_lambda=1e-5,
                         linear_solver=solver, elimination=elimination,
                         diagonal_damping=False, costs_finite=True,
                         termination_trace_complete=True, final_cost=1.)
            check_stage(stage, name)
            wrong = 'main' if name == 'gnss_first' else 'gnss_first'
            with self.assertRaises(AssertionError):
                check_stage(stage, wrong)
            with self.assertRaises(AssertionError):
                check_stage(dict(stage, lambda_lower_bound=0.), name)
