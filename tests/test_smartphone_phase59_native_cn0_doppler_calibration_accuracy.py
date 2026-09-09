#!/usr/bin/env python3
"""Pre-truth contract tests for the Phase59 one-shot accuracy scorer."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))
import gnss_smartphone_phase59_native_cn0_doppler_calibration_accuracy as module  # noqa: E402


class SmartphonePhase59NativeCn0AccuracyTest(unittest.TestCase):
    def test_freeze_is_pre_truth_and_gates_are_fixed(self) -> None:
        freeze = module._verify_freeze()
        self.assertEqual(freeze["status"], "frozen-before-phase59-truth-read")
        self.assertEqual(tuple(freeze["cohort"]["route_order"]), module.ROUTES)
        gates = freeze["accuracy_gates"]
        self.assertEqual(gates["candidate_improves_each_route_by_at_least_m"], 0.05)
        self.assertEqual(gates["candidate_macro_improvement_at_least_m"], 0.1)
        self.assertEqual(gates["candidate_mtv_h_improvement_at_least_m"], 0.1)
        self.assertEqual(gates["candidate_prediction_domain_coverage"], 1.0)
        self.assertEqual(gates["candidate_macro_score_max_m"], 2.0)
        self.assertEqual(gates["candidate_route_score_max_m"], 3.0)
        self.assertEqual(gates["candidate_mtv_h_p95_max_m"], 5.0)
        self.assertEqual(gates["candidate_over_70_mps_count"], 0)
        self.assertTrue(gates["declared_before_truth"])

    def test_warmup_missing_truth_is_separate_from_prediction_domain(self) -> None:
        route = module.ROUTES[0]
        submission = (
            "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
            f"{route},2000,37.0,-122.0\n"
            f"{route},3000,37.0,-122.0\n"
        ).encode()
        truth_payload = (
            "UnixTimeMillis,LatitudeDegrees,LongitudeDegrees,AltitudeMeters\n"
            "1000,37.0,-122.0,10\n"
            "2000,37.0,-122.0,10\n"
            "3000,37.0,-122.0,10\n"
        ).encode()
        rows = module._parse_submission(submission, route, "fixture submission")
        truth = module._parse_truth(truth_payload, route, "fixture truth")
        metrics = module._metrics(rows, truth, 1, [route, 1000], "fixture")
        self.assertEqual(metrics["prediction_domain_coverage"], 1.0)
        self.assertAlmostEqual(metrics["truth_row_coverage"], 2.0 / 3.0)
        self.assertEqual(metrics["missing_truth_rows"], 1)
        self.assertEqual(metrics["extra_prediction_rows"], 0)
        self.assertEqual(metrics["score_m"], 0.0)

    def test_unexpected_missing_key_fails_closed(self) -> None:
        route = module.ROUTES[0]
        rows = [
            (route, 2000, 37.0, -122.0),
            (route, 3000, 37.0, -122.0),
        ]
        truth = {
            (route, 1000): (37.0, -122.0),
            (route, 2000): (37.0, -122.0),
            (route, 3000): (37.0, -122.0),
        }
        with self.assertRaises(module.Phase59Error):
            module._metrics(rows, truth, 1, [route, 999], "fixture")

    def test_source_has_no_native_solver_archive_or_raw_input_path(self) -> None:
        source = Path(module.__file__).read_text(encoding="utf-8")
        for forbidden in ("import subprocess", "from subprocess", "Popen(", "zipfile", "device_gnss.csv", "device_imu.csv", "brdc.nav"):
            self.assertNotIn(forbidden, source)

    def test_historical_phase51_reference_is_not_scoring_input(self) -> None:
        freeze = module._verify_freeze()
        historical = freeze["historical_reference_not_scoring_input"]
        self.assertFalse(historical["scoring_input"])
        self.assertFalse(historical["selection_or_tuning_input"])
        self.assertEqual(freeze["read_policy"]["truth_reads_before_manifest"], 0)
        self.assertEqual(freeze["read_policy"]["truth_reads_per_route"], 1)


if __name__ == "__main__":
    unittest.main()
