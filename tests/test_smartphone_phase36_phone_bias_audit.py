#!/usr/bin/env python3
"""Unit tests for the truth-read-closed Phase36 phone-bias audit."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase36_phone_bias_audit as audit  # noqa: E402


class Phase36PhoneBiasAuditTest(unittest.TestCase):
    def test_freeze_is_pre_truth_and_route_disjoint(self) -> None:
        freeze = audit._verify_freeze(audit.FREEZE)
        self.assertEqual(freeze["status"], "frozen-before-phase36-residual-truth-read")
        self.assertEqual(tuple(freeze["cohort"]["routes"]), audit.ROUTES)
        self.assertTrue(freeze["scope"]["validation_and_holdout_remain_sealed"])

    def test_percentile_uses_linear_n_minus_1(self) -> None:
        self.assertAlmostEqual(audit._percentile([0.0, 10.0], 0.5), 5.0)
        self.assertAlmostEqual(audit._percentile([0.0, 10.0, 20.0], 0.95), 19.0)

    def test_enu_projection_sign_and_ecef_surface(self) -> None:
        origin = audit._ecef(37.0, -122.0, 10.0)
        east, north = audit._enu_delta(
            37.0,
            -121.99999,
            37.0,
            -122.0,
            10.0,
            origin,
            37.0,
            -122.0,
        )
        self.assertGreater(east, 0.0)
        self.assertAlmostEqual(north, 0.0, delta=0.01)

    def test_summary_reports_median_mad_and_inlier_covariance(self) -> None:
        summary = audit._summary([(1.0, 2.0), (1.1, 2.1), (1.2, 1.9), (100.0, -100.0)])
        self.assertEqual(summary["count"], 4)
        self.assertAlmostEqual(summary["median_enu_m"]["east"], 1.15)
        self.assertAlmostEqual(summary["median_enu_m"]["north"], 1.95)
        self.assertGreaterEqual(summary["component_3mad_inlier_count"], 3)
        self.assertEqual(len(summary["covariance_enu_m2"]), 2)

    def test_orientation_is_raw_coarse_label_only(self) -> None:
        gravity_axis, quality, heading = audit._orientation_label(
            (0.0, 0.0, 9.81), (20.0, 0.0, 40.0)
        )
        self.assertEqual(gravity_axis, "z+")
        self.assertEqual(quality, "gravity_valid")
        self.assertTrue(heading.startswith("octant_"))

    def test_mat_is_rejected_before_read(self) -> None:
        path = Path("/tmp/phase36-forbidden-input.mat")
        with self.assertRaises(audit.Phase36Error):
            audit._read_bytes_once(path, "test MAT")

    def test_cross_route_cannot_identify_single_model_constant(self) -> None:
        routes = {
            route: {"overall": {"median_enu_m": {"east": 0.0, "north": 0.0}}}
            for route in audit.ROUTES
        }
        report = audit._cross_route(routes)
        self.assertFalse(report["constant_identifiable"])
        self.assertEqual(report["same_model_repeated_route_count"], 1)


if __name__ == "__main__":
    unittest.main()
