"""Contract tests for the one-shot Phase44 Pixel5 evaluator."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))
import gnss_smartphone_phase44_pixel5_development_accuracy as module  # noqa: E402


class SmartphonePhase44Pixel5DevelopmentAccuracyTest(unittest.TestCase):
    def test_freeze_is_pre_truth_and_route_order_is_fixed(self) -> None:
        freeze = module._verify_freeze()
        self.assertEqual(freeze["status"], "frozen-before-truth-materialization-and-read")
        self.assertEqual(tuple(freeze["cohort"]["route_order"]), module.ROUTES)
        self.assertEqual(freeze["truth_budget"]["total_route_truth_reads"], 4)
        self.assertEqual(freeze["truth_budget"]["single_process"], True)

    def test_timestamp_contract_disallows_synthesis(self) -> None:
        freeze = module._verify_freeze()
        alignment = freeze["timestamp_contract"]
        self.assertEqual(alignment["matching"], "exact integer UnixTimeMillis key intersection only")
        self.assertFalse(alignment["interpolation"]["enabled"])
        self.assertFalse(alignment["interpolation"]["edge_hold"])
        self.assertFalse(alignment["interpolation"]["extrapolation"])

    def test_haversine_score_and_continuity_are_finite(self) -> None:
        submission = (
            "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
            "pixel5,1000,37.0,-122.0\n"
            "pixel5,2000,37.0,-122.0\n"
        ).encode()
        truth = (
            "UnixTimeMillis,LatitudeDegrees,LongitudeDegrees,AltitudeMeters\n"
            "1000,37.0,-122.0,10\n"
            "2000,37.0,-122.0,10\n"
        ).encode()
        rows = module._parse_coordinate_rows(submission, "pixel5", "fixture submission")
        truth_rows = module._parse_truth_rows(truth, "pixel5", "fixture truth")
        metrics = module._lane_metrics(rows, truth_rows)
        self.assertTrue(metrics["finite"])
        self.assertEqual(metrics["coverage_ratio"], 1.0)
        self.assertEqual(metrics["mean_m"], 0.0)
        self.assertEqual(metrics["p50_m"], 0.0)
        self.assertEqual(metrics["p95_m"], 0.0)
        self.assertEqual(metrics["max_m"], 0.0)
        self.assertEqual(metrics["kaggle_score_m"], 0.0)
        self.assertEqual(metrics["over_70_mps_count"], 0)

    def test_summary_projection_removes_only_candidate_fields(self) -> None:
        candidate = {
            "native_fallback_seed_quality_anchor_recovery": True,
            "quality_anchor_initialization": {
                "normal_quality_anchor_candidates": 10,
                "selected": True,
                "recovery_trigger": False,
            },
            "graph": {"converged": True},
        }
        control = {
            "quality_anchor_initialization": {"selected": True},
            "graph": {"converged": True},
        }
        self.assertEqual(module._project_summary(candidate), module._project_summary(control))

    def test_phase43_target_control_is_explicitly_invalid(self) -> None:
        freeze = module._verify_freeze()
        target = freeze["sealed_artifacts"]["routes"][module.TARGET]
        self.assertEqual(target["control"]["return_code"], 1)
        self.assertIsNone(target["control"]["submission"])
        self.assertIsNone(target["control"]["summary"])

    def test_source_does_not_invoke_native_solver(self) -> None:
        source = module.__file__
        text = Path(source).read_text(encoding="utf-8")
        self.assertNotIn("subprocess", text)
        self.assertNotIn("Popen", text)
        self.assertNotIn("run_native", text)


if __name__ == "__main__":
    unittest.main()
