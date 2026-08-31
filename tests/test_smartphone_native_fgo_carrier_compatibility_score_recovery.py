from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_native_fgo_carrier_compatibility_score_recovery as RECOVERY  # noqa: E402


def _diagnostics(value: float) -> dict[str, float]:
    return {key: value for key in RECOVERY.DIAGNOSTIC_KEYS}


class CarrierCompatibilityScoreRecoveryTests(unittest.TestCase):
    def test_metric_map_accepts_route_and_aggregate_schema(self) -> None:
        route = {"kaggle_diagnostic_score_variants_m": _diagnostics(1.0)}
        aggregate = {"mean_kaggle_diagnostic_score_variants_m": _diagnostics(2.0)}
        self.assertEqual(RECOVERY._metric_map(route), _diagnostics(1.0))
        self.assertEqual(RECOVERY._metric_map(aggregate), _diagnostics(2.0))

    def test_strict_comparison_uses_aggregate_scalar_aliases(self) -> None:
        baseline = {
            "mean_kaggle_diagnostic_score_variants_m": _diagnostics(2.0),
            "mean_kaggle_diagnostic_m": 2.0,
            "mean_availability_ratio": 1.0,
            "mean_truth_coverage_ratio": 1.0,
            "mean_vertical_p95_abs_m": 5.0,
        }
        candidate = {
            "mean_kaggle_diagnostic_score_variants_m": _diagnostics(1.0),
            "mean_kaggle_diagnostic_m": 1.0,
            "mean_availability_ratio": 1.0,
            "mean_truth_coverage_ratio": 1.0,
            "mean_vertical_p95_abs_m": 5.0,
        }
        result = RECOVERY.strict_comparison(candidate, baseline)
        self.assertTrue(result["passed"])
        self.assertEqual(result["failures"], [])

    def test_missing_diagnostic_schema_fails_closed(self) -> None:
        with self.assertRaises(RECOVERY.RecoveryError):
            RECOVERY._metric_map({"mean_kaggle_diagnostic_m": 1.0})


if __name__ == "__main__":
    unittest.main()
