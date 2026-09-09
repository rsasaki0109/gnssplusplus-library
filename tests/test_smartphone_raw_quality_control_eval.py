#!/usr/bin/env python3
"""Contract tests for the train-only raw-quality evaluator."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_raw_quality_control_eval as evaluation  # noqa: E402


def _metric(h50: float, h95: float, v95: float, diagnostic: float) -> dict:
    return {
        "availability_ratio": 1.0,
        "truth_coverage_ratio": 1.0,
        "horizontal_wgs84_m": {"p50_m": h50, "p95_m": h95},
        "vertical_p95_abs_m": v95,
        "kaggle_diagnostic_score_variants_m": {
            key: diagnostic for key in evaluation.DIAGNOSTIC_KEYS
        },
    }


class RawQualityEvaluationTests(unittest.TestCase):
    def test_route_gate_requires_all_diagnostics_and_non_regression(self) -> None:
        baseline = _metric(2.0, 5.0, 8.0, 4.0)
        self.assertTrue(evaluation._compare_route(_metric(2.0, 5.0, 8.0, 4.0), baseline)["passed"])
        failed = evaluation._compare_route(_metric(2.0, 5.1, 8.0, 4.0), baseline)
        self.assertFalse(failed["passed"])
        self.assertIn("h_p95_regression", failed["failures"])

    def test_aggregate_gate_requires_strict_h95_and_diagnostic_improvement(self) -> None:
        baseline = {
            "mean_availability_ratio": 1.0,
            "mean_truth_coverage_ratio": 1.0,
            "mean_horizontal_wgs84_p50_m": 2.0,
            "mean_horizontal_wgs84_p95_m": 5.0,
            "mean_vertical_p95_abs_m": 8.0,
            "mean_kaggle_diagnostic_score_variants_m": {
                key: 4.0 for key in evaluation.DIAGNOSTIC_KEYS
            },
            "mean_kaggle_diagnostic_m": 4.0,
        }
        candidate = dict(baseline)
        candidate["mean_horizontal_wgs84_p95_m"] = 4.9
        candidate["mean_kaggle_diagnostic_score_variants_m"] = {
            key: 3.9 for key in evaluation.DIAGNOSTIC_KEYS
        }
        candidate["mean_kaggle_diagnostic_m"] = 3.9
        gate = evaluation._aggregate_gate(candidate, baseline)
        self.assertTrue(gate["passed"])
        self.assertTrue(gate["strict_h_p95_improvement"])
        self.assertTrue(gate["strict_diagnostic_mean_improvement"])

    def test_quality_bucket_boundaries_are_fixed(self) -> None:
        self.assertEqual(evaluation._quality_bucket(3.0, "sigma"), "le_3")
        self.assertEqual(evaluation._quality_bucket(3.0001, "sigma"), "le_5")
        self.assertEqual(evaluation._quality_bucket(35.0, "cn0"), "ge_35")
        self.assertEqual(evaluation._quality_bucket(None, "gdop"), "unavailable")


if __name__ == "__main__":
    unittest.main()
