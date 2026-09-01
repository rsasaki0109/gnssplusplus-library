from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase35_matrix_eval.py"
SPEC = importlib.util.spec_from_file_location("phase35_matrix_eval", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def metric(value: float, *, p50: float = 1.0, p95: float = 2.0, over_70: int = 0) -> dict:
    return {
        "kaggle_diagnostic_score_variants_m": {key: value for key in MODULE.DIAGNOSTIC_KEYS},
        "horizontal_wgs84_m": {"p50_m": p50, "p95_m": p95},
        "horizontal_haversine_m": {"p50_m": p50, "p95_m": p95},
        "availability_ratio": 1.0,
        "truth_coverage_ratio": 1.0,
        "continuity": {"over_70_mps_count": over_70},
    }


def aggregate(rows: list[dict]) -> dict:
    return {
        "mean_kaggle_diagnostic_score_variants_m": {
            key: sum(row["kaggle_diagnostic_score_variants_m"][key] for row in rows) / len(rows)
            for key in MODULE.DIAGNOSTIC_KEYS
        },
        "mean_horizontal_wgs84_p50_m": sum(row["horizontal_wgs84_m"]["p50_m"] for row in rows) / len(rows),
        "mean_horizontal_wgs84_p95_m": sum(row["horizontal_wgs84_m"]["p95_m"] for row in rows) / len(rows),
        "mean_horizontal_haversine_p50_m": sum(row["horizontal_haversine_m"]["p50_m"] for row in rows) / len(rows),
        "mean_horizontal_haversine_p95_m": sum(row["horizontal_haversine_m"]["p95_m"] for row in rows) / len(rows),
        "mean_availability_ratio": 1.0,
        "mean_truth_coverage_ratio": 1.0,
        "sum_over_70_mps_count": sum(row["continuity"]["over_70_mps_count"] for row in rows),
    }


class Phase35MatrixEvalTests(unittest.TestCase):
    def test_route_gate_allows_non_regression_but_rejects_key_mismatch(self) -> None:
        control = metric(2.0, p50=2.0, p95=3.0)
        candidate = metric(2.0, p50=2.0, p95=3.0)
        equal = {"same_matched_key_set_all_lanes": True}
        self.assertTrue(MODULE._route_gate(candidate, control, equal)["passed"])
        mismatch = {"same_matched_key_set_all_lanes": False}
        gate = MODULE._route_gate(candidate, control, mismatch)
        self.assertFalse(gate["passed"])
        self.assertIn("matched_key_set_mismatch", gate["failures"])

    def test_macro_gate_requires_all_four_strict_variants(self) -> None:
        control = aggregate([metric(2.0, p50=2.0, p95=3.0), metric(2.0, p50=2.0, p95=3.0)])
        candidate = aggregate([metric(1.0, p50=1.0, p95=2.0), metric(1.0, p50=1.0, p95=2.0)])
        gate = MODULE._macro_gate(candidate, control)
        self.assertTrue(gate["passed"])
        self.assertEqual(set(gate["strictly_improved_variants"]), set(MODULE.DIAGNOSTIC_KEYS))
        candidate["mean_kaggle_diagnostic_score_variants_m"][MODULE.DIAGNOSTIC_KEYS[0]] = 2.0
        gate = MODULE._macro_gate(candidate, control)
        self.assertFalse(gate["passed"])

    def test_validation_gate_is_strict_and_truth_free_policy_is_explicit(self) -> None:
        gate = MODULE._validation_gate(metric(1.0, p50=1.0, p95=2.0), metric(2.0, p50=2.0, p95=3.0), {"same_matched_key_set_all_lanes": True})
        self.assertTrue(gate["passed"])
        self.assertEqual(set(gate["strictly_improved_variants"]), set(MODULE.DIAGNOSTIC_KEYS))
        self.assertEqual(MODULE.FREEZE_SHA256, "9be22fd18dd3f818a8c902dafde2aa7b5828ded13df98b138682863e8f14a92a")
        self.assertEqual(MODULE.PHASE34_TRUTH.suffix, ".csv")

    def test_mat_and_holdout_are_not_evaluation_inputs(self) -> None:
        self.assertNotIn(".mat", str(MODULE.PHASE34_TRUTH).lower())
        self.assertEqual(MODULE.VALIDATION_ROUTE, "2023-05-09-21-32-us-ca-mtv-pe1/pixel5")
        self.assertNotEqual(MODULE.VALIDATION_ROUTE, "2023-05-16-19-54-us-ca-mtv-xe1/pixel5")
        source = SCRIPT.read_text(encoding="utf-8").lower()
        self.assertIn("mat_read_or_generated", source)
        self.assertIn("future_holdout_truth_open_count", source)


if __name__ == "__main__":
    unittest.main()
