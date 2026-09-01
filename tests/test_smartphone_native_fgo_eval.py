from __future__ import annotations

import copy
import math
import sys
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_native_fgo_eval as FGO  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402


class NativeFgoEvaluationTests(unittest.TestCase):
    def test_route_argument_requires_frozen_six_field_shape(self) -> None:
        route = FGO._route_argument("route/phone|obs|nav|seed|device|truth")
        self.assertEqual(route["dataset_id"], "route/phone")
        with self.assertRaises(FGO.NativeFgoError):
            FGO._route_argument("route/phone|obs|nav|seed|device")
        with self.assertRaises(FGO.NativeFgoError):
            FGO._route_argument("route phone|obs|nav|seed|device|truth")

    def _summary(self) -> dict[str, object]:
        return {
            "backend": "eigen",
            "preset": "default",
            "input_epochs": 2,
            "optimized_epochs": 2,
            "valid_solutions": 2,
            "pseudorange_factors": 22,
            "tdcp_factors": 6,
            "motion_factors": 1,
            "single_difference_doppler_factors": 0,
            "single_difference_tdcp_factors": 0,
            "carrier_phase_factors": 0,
            "double_difference_pseudorange_factors": 0,
            "double_difference_carrier_factors": 0,
            "use_spp_seed": True,
            "use_pseudorange_factors": True,
            "use_single_difference_doppler_factors": False,
            "use_single_difference_tdcp_factors": False,
            "use_velocity_states": False,
            "use_velocity_motion_factors": False,
            "use_ambiguity_between_factors": False,
            "use_position_motion_factors": True,
            "use_clock_motion_factors": True,
            "use_ionosphere_model": True,
            "use_troposphere_model": True,
            "max_iterations": 8,
            "pseudorange_sigma_m": 3.0,
            "pseudorange_elevation_sigma_power": 1.0,
            "motion_sigma_m": 50.0,
            "clock_motion_sigma_m": 300.0,
            "tdcp_sigma_m": 0.03,
            "pseudorange_huber_threshold_sigma": 4.0,
            "tdcp_huber_threshold_sigma": 4.0,
            "max_tdcp_gap_s": 2.0,
            "seed_match_tolerance_s": 0.01,
            "seed_interpolation_max_gap_s": 0.0,
            "tdcp_slip_threshold_m": 10.0,
            "min_elevation_deg": 10.0,
            "min_snr_dbhz": 0.0,
            "min_satellites_per_epoch": 4,
        }

    def test_summary_accepts_required_factor_contract(self) -> None:
        summary = FGO._validate_summary(self._summary())
        self.assertEqual(summary["tdcp_factors"], 6)

    def test_summary_rejects_base_dependent_or_sd_factors(self) -> None:
        summary = self._summary()
        summary["single_difference_tdcp_factors"] = 1
        with self.assertRaises(FGO.NativeFgoError):
            FGO._validate_summary(summary)
        summary = self._summary()
        summary["double_difference_carrier_factors"] = 1
        with self.assertRaises(FGO.NativeFgoError):
            FGO._validate_summary(summary)

    def test_aggregate_gate_requires_strict_improvement(self) -> None:
        baseline = {
            "mean_availability_ratio": 1.0,
            "mean_truth_coverage_ratio": 1.0,
            "mean_horizontal_wgs84_p50_m": 4.0,
            "mean_horizontal_wgs84_p95_m": 8.0,
            "mean_vertical_p95_abs_m": 10.0,
            "mean_kaggle_diagnostic_m": 5.0,
            "mean_kaggle_diagnostic_score_variants_m": {
                key: 5.0 for key in FGO.DIAGNOSTIC_KEYS
            },
        }
        candidate = copy.deepcopy(baseline)
        candidate["mean_horizontal_wgs84_p95_m"] = 7.0
        candidate["mean_kaggle_diagnostic_m"] = 4.0
        candidate["mean_kaggle_diagnostic_score_variants_m"] = {
            key: 4.0 for key in FGO.DIAGNOSTIC_KEYS
        }
        gate = FGO._aggregate_gate(candidate, baseline)
        self.assertTrue(gate["passed"])
        candidate["mean_kaggle_diagnostic_m"] = 5.0
        self.assertFalse(FGO._aggregate_gate(candidate, baseline)["passed"])

    def test_empty_truth_match_preserves_null_diagnostic_schema(self) -> None:
        row = smoother.SmoothedRow(
            timestamp_ms=0,
            week=0,
            tow=0.0,
            ecef=np.array([6378137.0, 0.0, 0.0]),
            latitude=0.0,
            longitude=0.0,
            height=0.0,
            status=1,
            satellites=4,
            pdop=1.0,
            ratio=1.0,
            fixed_ambiguities=0,
            iterations=1,
            source="measured",
            segment_id=0,
            measurement_used=True,
            outlier_rejected=False,
            innovation_sigma=None,
            position_sigma_m=1.0,
        )
        metrics = smoother_eval._score_rows(
            [row],
            {},
            {1000: (0.0, 0.0, 0.0)},
            0,
            1,
            match_tolerance_ms=100,
        )
        self.assertEqual(metrics["truth_matched_epochs"], 0)
        self.assertTrue(
            all(
                metrics["kaggle_diagnostic_score_variants_m"][key] is None
                for key in FGO.DIAGNOSTIC_KEYS
            )
        )
        self.assertIsNone(metrics["kaggle_diagnostic_mean_m"])

    def test_normalized_null_schema_aggregates_fail_closed(self) -> None:
        metrics = FGO._normalize_score_schema(
            {
                "availability_ratio": 1.0,
                "truth_coverage_ratio": 0.0,
                "horizontal_wgs84_m": {"p50_m": None, "p95_m": None},
                "vertical_p95_abs_m": None,
                "kaggle_diagnostic_score_variants_m": {},
                "kaggle_diagnostic_mean_m": None,
            }
        )
        self.assertEqual(
            set(metrics["kaggle_diagnostic_score_variants_m"]),
            set(FGO.DIAGNOSTIC_KEYS),
        )
        aggregate = FGO._aggregate([metrics])
        self.assertTrue(math.isinf(aggregate["mean_horizontal_wgs84_p95_m"]))
        self.assertTrue(math.isinf(aggregate["mean_kaggle_diagnostic_m"]))


if __name__ == "__main__":
    unittest.main()
