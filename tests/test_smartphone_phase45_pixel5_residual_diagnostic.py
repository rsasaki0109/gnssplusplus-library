"""Focused contract tests for the Phase45 Pixel5 residual diagnostic."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase45_pixel5_residual_diagnostic as diagnostic  # noqa: E402


class SmartphonePhase45Pixel5ResidualDiagnosticTest(unittest.TestCase):
    def test_v3_freeze_is_pre_truth_and_separates_coverages(self) -> None:
        freeze = diagnostic._verify_freeze()
        self.assertEqual(freeze["status"], "frozen-before-phase45-truth-read")
        self.assertEqual(tuple(freeze["cohort"]["route_order"]), diagnostic.ROUTES)
        gate = freeze["identifiability_gates"]["exact_model_route_count"]
        self.assertTrue(gate["prediction_domain_coverage_required"])
        self.assertTrue(gate["truth_row_coverage_informational_only"])
        self.assertTrue(gate["truth_row_coverage_does_not_gate_prediction_domain"])
        self.assertNotIn("truth_row_coverage 1.0", gate["criterion"])
        self.assertEqual(freeze["read_contract"]["truth_read_count_per_route"], 1)

    def test_v3_pins_v1_and_v2_reason(self) -> None:
        freeze = diagnostic._verify_freeze()
        self.assertEqual(freeze["supersedes"]["sha256"], diagnostic.FREEZE_V1_SHA256)
        self.assertEqual(freeze["supersedes"]["v2"]["sha256"], diagnostic.FREEZE_V2_SHA256)
        self.assertIn("warm-up", freeze["supersedes"]["reason"])

    def test_percentile_uses_linear_n_minus_1_rank(self) -> None:
        self.assertAlmostEqual(diagnostic._percentile([0.0, 10.0], 0.5), 5.0)
        self.assertAlmostEqual(diagnostic._percentile([0.0, 10.0, 20.0], 0.95), 19.0)

    def test_fixed_enu_projection_has_expected_east_sign(self) -> None:
        origin = diagnostic._ecef(37.0, -122.0, 10.0)
        east, north = diagnostic._enu_delta(37.0, -121.99999, 37.0, -122.0, 10.0, origin, 37.0, -122.0)
        self.assertGreater(east, 0.0)
        self.assertAlmostEqual(north, 0.0, delta=0.01)

    def test_summary_contains_mad_covariance_and_inliers(self) -> None:
        summary = diagnostic._summary([(1.0, 2.0), (1.1, 2.1), (1.2, 1.9), (100.0, -100.0)])
        self.assertEqual(summary["count"], 4)
        self.assertAlmostEqual(summary["median_enu_m"]["east"], 1.15)
        self.assertAlmostEqual(summary["median_enu_m"]["north"], 1.95)
        self.assertGreaterEqual(summary["component_3mad_inlier_count"], 3)
        self.assertEqual(len(summary["covariance_enu_m2"]), 2)

    def test_orientation_is_raw_coarse_label_only(self) -> None:
        gravity_axis, quality, heading = diagnostic._orientation_label((0.0, 0.0, 9.81), (20.0, 0.0, 40.0))
        self.assertEqual(gravity_axis, "z+")
        self.assertEqual(quality, "gravity_valid")
        self.assertTrue(heading.startswith("octant_"))

    def test_speed_bins_never_need_truth(self) -> None:
        candidate = [
            {"phone": "route/pixel5", "timestamp": 0, "lat": 37.0, "lon": -122.0},
            {"phone": "route/pixel5", "timestamp": 1000, "lat": 37.000001, "lon": -122.0},
        ]
        labels, report = diagnostic._prediction_speeds(candidate)
        self.assertEqual(labels[("route/pixel5", 0)], "unavailable")
        self.assertIn(labels[("route/pixel5", 1000)], {"<=1", ">1<=5"})
        self.assertEqual(report["transition_count"], 1)

    def test_loo_common_median_is_diagnostic(self) -> None:
        reports = {
            route: {"overall": {"median_enu_m": {"east": float(i), "north": 0.0}}, "_vectors": [(float(i), 0.0), (float(i), 0.0)]}
            for i, route in enumerate(diagnostic.ROUTES)
        }
        result = diagnostic._loo_diagnostics(reports)
        self.assertEqual(len(result["per_held_route"]), 4)
        self.assertTrue(result["diagnostic_only"])
        self.assertFalse(result["correction_applied_to_inference"])

    def test_forbidden_mat_path_is_rejected(self) -> None:
        with self.assertRaises(diagnostic.Phase45Error):
            diagnostic._read_bytes_once(Path("/tmp/phase45-forbidden.mat"), "test MAT")

    def test_source_has_no_solver_or_archive_execution(self) -> None:
        source = Path(diagnostic.__file__).read_text(encoding="utf-8")
        self.assertNotIn("subprocess", source)
        self.assertNotIn("zipfile", source)
        self.assertNotIn("Popen", source)
        self.assertNotIn("run_native", source)


if __name__ == "__main__":
    unittest.main()
