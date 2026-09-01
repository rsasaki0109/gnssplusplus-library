from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_imu_motion_q_eval as EVAL  # noqa: E402


def _metrics(value: float, diagnostic: float) -> dict:
    return {
        "availability_ratio": 1.0,
        "truth_coverage_ratio": 1.0,
        "horizontal_wgs84_m": {"p50_m": value, "p95_m": value},
        "vertical_p95_abs_m": value,
        "kaggle_diagnostic_score_variants_m": {
            key: diagnostic for key in EVAL.DIAGNOSTIC_KEYS
        },
        "kaggle_diagnostic_mean_m": diagnostic,
    }


class SmartphoneImuMotionQEvaluationTests(unittest.TestCase):
    def test_strict_gate_requires_diagnostic_improvement(self) -> None:
        baseline = _metrics(10.0, 10.0)
        candidate = _metrics(9.0, 10.0)
        self.assertEqual(
            EVAL._strict_train_gate(candidate, baseline),
            ["aggregate_diagnostic_mean_not_strictly_improved"],
        )

    def test_strict_gate_rejects_route_regression(self) -> None:
        baseline = _metrics(10.0, 10.0)
        candidate = _metrics(11.0, 9.0)
        failures = EVAL._strict_train_gate(candidate, baseline)
        self.assertIn("h_p50_regression", failures)
        self.assertIn("h_p95_regression", failures)

    def test_sealed_holdout_route_is_rejected(self) -> None:
        with self.assertRaises(EVAL.ImuMotionEvaluationError):
            EVAL._parse_route_spec(
                "2023-09-06-00-01-us-ca-routen/pixel6pro|a|b|c|d|e"
            )

    def test_frozen_selection_record_loads(self) -> None:
        selection = EVAL._load_selection(
            ROOT
            / "docs/use_cases/records/smartphone_r5_gsdc2023_imu_motion_q_selection.json"
        )
        self.assertEqual(
            selection["candidate"]["id"], "causal_imu_motion_adaptive_q_v1"
        )


if __name__ == "__main__":
    unittest.main()
