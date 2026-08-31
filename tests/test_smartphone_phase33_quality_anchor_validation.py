from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase33_quality_anchor_validation.py"
SPEC = importlib.util.spec_from_file_location("phase33_quality_anchor_validation", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def metric(value: float, *, p50: float = 1.0, p95: float = 2.0, over_70: int = 0) -> dict:
    return {
        "kaggle_diagnostic_score_variants_m": {
            key: value for key in MODULE.DIAGNOSTIC_KEYS
        },
        "horizontal_wgs84": {"p50_m": p50, "p95_m": p95},
        "horizontal_wgs84_m": {"p50_m": p50, "p95_m": p95},
        "availability_ratio": 1.0,
        "truth_coverage_ratio": 1.0,
        "continuity": {"over_70_mps_count": over_70},
    }


class Phase33QualityAnchorValidationTests(unittest.TestCase):
    def test_recipe_flags_are_distinct_and_candidate_is_opt_in(self) -> None:
        self.assertEqual(MODULE.CONTROL_FLAGS.count("--android-raw-utc-keys"), 1)
        self.assertEqual(MODULE.CANDIDATE_FLAGS.count("--android-raw-utc-keys"), 1)
        self.assertNotIn("--native-quality-anchor", MODULE.CONTROL_FLAGS)
        self.assertIn("--native-quality-anchor", MODULE.CANDIDATE_FLAGS)

    def test_route_gate_allows_equal_horizontal_and_rejects_regression(self) -> None:
        self.assertTrue(MODULE._horizontal_compare(metric(1.0), metric(1.0))["passed"])
        rejected = MODULE._horizontal_compare(
            metric(1.1, p95=2.1), metric(1.0, p95=2.0)
        )
        self.assertFalse(rejected["passed"])
        self.assertIn("h_p95_m_regression", rejected["failures"])

    def test_validation_aggregate_requires_strict_four_variant_improvement(self) -> None:
        control = MODULE._aggregate([metric(2.0)])
        candidate = MODULE._aggregate([metric(1.0)])
        self.assertTrue(MODULE._aggregate_compare(candidate, control)["passed"])
        candidate["mean_kaggle_diagnostic_score_variants_m"][MODULE.DIAGNOSTIC_KEYS[0]] = control[
            "mean_kaggle_diagnostic_score_variants_m"
        ][MODULE.DIAGNOSTIC_KEYS[0]]
        self.assertFalse(MODULE._aggregate_compare(candidate, control)["passed"])


if __name__ == "__main__":
    unittest.main()
