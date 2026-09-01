#!/usr/bin/env python3
"""Truth/raw-free focused tests for the Phase51 evaluator contract."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
EVALUATOR_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase51_android_sv_time_uncertainty_sigma_floor.py"
SPEC = importlib.util.spec_from_file_location("phase51_evaluator", EVALUATOR_PATH)
assert SPEC is not None and SPEC.loader is not None
phase51 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(phase51)


class Phase51EvaluatorTest(unittest.TestCase):
    def test_metric_uses_exact_key_intersection_and_linear_percentile(self) -> None:
        route = "2021-03-16-18-59-us-ca-mtv-a/pixel5"
        rows = [(1000, 37.0, -122.0), (2000, 37.0, -122.0), (3000, 37.0, -122.0)]
        truth = {(route, 1000): (37.0, -122.0), (route, 2000): (37.00001, -122.0), (route, 3000): (37.0, -122.0)}
        metric = phase51.score_lane(rows, truth, route)
        self.assertEqual(metric["matched_rows"], 3)
        self.assertEqual(metric["missing_truth_rows"], 0)
        self.assertEqual(metric["extra_prediction_rows"], 0)
        self.assertEqual(metric["prediction_domain_coverage"], 1.0)
        self.assertEqual(metric["truth_row_coverage"], 1.0)
        self.assertAlmostEqual(metric["score_m"], (metric["p50_m"] + metric["p95_m"]) / 2.0)

    def test_accuracy_gate_is_fixed_and_route_local(self) -> None:
        reports = {}
        for route in phase51.ROUTES:
            reports[route] = {
                "candidate_metrics": {
                    "score_m": 1.0,
                    "p95_m": 2.0,
                    "prediction_domain_coverage": 1.0,
                    "truth_row_coverage": 1.0,
                    "over_70_mps_count": 0,
                },
                "control_metrics": {"score_m": 1.1},
            }
        gates = phase51.accuracy_gates(reports, {route: 1.5 for route in phase51.ROUTES})
        self.assertTrue(gates["all_routes_pass"])
        self.assertAlmostEqual(gates["macro_improvement_m"], 0.1)
        self.assertEqual(gates["stretch_target_reached"], False)

    def test_forbidden_paths_fail_closed(self) -> None:
        with self.assertRaises(phase51.Phase51Error):
            phase51.reject_forbidden(Path("validation/ground_truth.csv"))

    def test_route_matrix_and_control_identity_are_four_way(self) -> None:
        self.assertEqual(len(phase51.ROUTES), 4)
        self.assertEqual(set(phase51.FROZEN_CONTROL), set(phase51.ROUTES))
        self.assertEqual(phase51.CANDIDATE_FLAG, "--native-android-sv-time-uncertainty-sigma-floor")
        self.assertEqual(
            phase51.FROZEN_CONTROL[phase51.ROUTES[3]]["submission"],
            "524769cdf67aa857eefaafdadf943dd76586dda6a53e8b6d1df4a707d7699f71",
        )

    def test_phase37_invocations_remain_repository_relative(self) -> None:
        path = ROOT / "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2021-08-24-20-32-us-ca-mtv-h/pixel5/inputs/device_gnss.csv"
        self.assertFalse(phase51.invocation_path(path).startswith("/"))
        self.assertEqual(phase51.invocation_path(path), str(path.relative_to(ROOT)))


if __name__ == "__main__":
    unittest.main()
