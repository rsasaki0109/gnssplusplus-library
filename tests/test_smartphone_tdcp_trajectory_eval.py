from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_tdcp_trajectory_eval as EVAL  # noqa: E402


def _metrics(*, p50: float, p95: float, vertical: float, diagnostic: float) -> dict:
    return {
        "availability_ratio": 1.0,
        "truth_coverage_ratio": 1.0,
        "horizontal_wgs84_m": {"p50_m": p50, "p95_m": p95},
        "vertical_p95_abs_m": vertical,
        "kaggle_diagnostic_mean_m": diagnostic,
        "kaggle_diagnostic_score_variants_m": {
            key: diagnostic
            for key in (
                "haversine_sphere__linear_n_minus_1",
                "haversine_sphere__nearest_rank_ceiling",
                "wgs84_vincenty__linear_n_minus_1",
                "wgs84_vincenty__nearest_rank_ceiling",
            )
        },
    }


class SmartphoneTdcpTrajectoryEvaluationTests(unittest.TestCase):
    def test_route_gate_rejects_any_metric_regression(self) -> None:
        baseline = _metrics(p50=1.0, p95=2.0, vertical=3.0, diagnostic=2.0)
        candidate = _metrics(p50=1.1, p95=1.9, vertical=2.9, diagnostic=1.9)
        comparison = EVAL._compare(baseline, candidate)
        self.assertFalse(comparison["passed"])
        self.assertIn("horizontal_wgs84_m.p50_m_regression", comparison["failures"])

    def test_aggregate_gate_requires_both_strict_improvements(self) -> None:
        baseline = _metrics(p50=1.0, p95=2.0, vertical=3.0, diagnostic=2.0)
        candidate = _metrics(p50=1.0, p95=2.0, vertical=2.9, diagnostic=2.0)
        comparison = EVAL._compare(baseline, candidate, aggregate=True)
        self.assertFalse(comparison["passed"])
        self.assertIn("aggregate_horizontal_p95_not_strictly_improved", comparison["failures"])
        self.assertIn("aggregate_diagnostic_mean_not_strictly_improved", comparison["failures"])

    def test_aggregate_is_arithmetic_mean_of_fixed_routes(self) -> None:
        first = _metrics(p50=1.0, p95=2.0, vertical=3.0, diagnostic=2.0)
        second = _metrics(p50=3.0, p95=4.0, vertical=5.0, diagnostic=4.0)
        aggregate = EVAL._aggregate([first, second])
        self.assertEqual(aggregate["horizontal_wgs84_m"]["p50_m"], 2.0)
        self.assertEqual(aggregate["horizontal_wgs84_m"]["p95_m"], 3.0)
        self.assertEqual(aggregate["vertical_p95_abs_m"], 4.0)
        self.assertEqual(aggregate["kaggle_diagnostic_mean_m"], 3.0)


if __name__ == "__main__":
    unittest.main()
