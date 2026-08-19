#!/usr/bin/env python3

import importlib.util
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "analysis" / "analyze_fgo_external_dr_witness.py"
SPEC = importlib.util.spec_from_file_location("external_dr_witness", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


def row(*, candidate_east_m: float, status: str = "FLOAT", accepted: bool = True) -> dict[str, str]:
    return {
        "tow": "1000.0",
        "status": status,
        "e_pos_m": "0",
        "n_pos_m": "0",
        "u_pos_m": "0",
        "ref_e_pos_m": "0",
        "ref_n_pos_m": "0",
        "ref_u_pos_m": "0",
        "x_ecef_m": "6378137",
        "y_ecef_m": "0",
        "z_ecef_m": "0",
        "lambda_candidate_valid": "1",
        "lambda_candidate_x_ecef_m": "6378137",
        "lambda_candidate_y_ecef_m": str(candidate_east_m),
        "lambda_candidate_z_ecef_m": "0",
        "external_dr_eval": "1",
        "external_dr_accept": "1" if accepted else "0",
    }


class FgoExternalDrWitnessTest(unittest.TestCase):
    def test_candidate_error_uses_candidate_not_reported_solution(self) -> None:
        horizontal, error_3d = AUDIT.candidate_error(row(candidate_east_m=2.0))
        self.assertAlmostEqual(horizontal, 2.0, places=6)
        self.assertAlmostEqual(error_3d, 2.0, places=6)

    def test_fixed_epochs_are_controls_not_rescue_opportunities(self) -> None:
        summary = AUDIT.analyze_rows([row(candidate_east_m=0.1, status="FIXED")])
        self.assertEqual(summary["population"]["evaluated_fixed_candidates"], 1)
        self.assertEqual(summary["population"]["evaluated_float_candidates"], 0)

    def test_gate_requires_correct_yield_and_zero_wrong(self) -> None:
        rows = [row(candidate_east_m=0.1) for _ in range(100)]
        self.assertTrue(AUDIT.analyze_rows(rows)["gate_passed"])
        rows[-1] = row(candidate_east_m=2.0)
        self.assertFalse(AUDIT.analyze_rows(rows)["gate_passed"])

    def test_insufficient_support_fails(self) -> None:
        summary = AUDIT.analyze_rows([row(candidate_east_m=0.1) for _ in range(59)])
        self.assertFalse(summary["gate_passed"])


if __name__ == "__main__":
    unittest.main()
