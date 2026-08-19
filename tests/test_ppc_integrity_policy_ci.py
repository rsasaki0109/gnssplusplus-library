#!/usr/bin/env python3
"""Dependency-free CI regression tests for the frozen residual policy."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "scripts" / "analysis"))

import analyze_ppc_wrong_fix_residuals as audit  # noqa: E402
import evaluate_ppc_residual_integrity_policy as policy_audit  # noqa: E402


def epoch(
    tow_s: float,
    *,
    prefit_rms_m: float,
    outliers: int = 20,
    observations: int = 30,
    x_m: float = 0.0,
) -> audit.SolutionEpoch:
    return audit.SolutionEpoch(
        week=2300,
        tow_s=tow_s,
        ecef=(x_m, 0.0, 0.0),
        status=4,
        nsat=16,
        ratio=7.0,
        baseline_m=9000.0,
        outliers=outliers,
        prefit_rms_m=prefit_rms_m,
        prefit_max_m=prefit_rms_m,
        post_rms_m=0.7,
        post_max_m=1.0,
        nis_per_obs=1.0,
        observations=observations,
        telemetry_complete=True,
    )


class ResidualIntegrityPolicyCITest(unittest.TestCase):
    def test_staged_policy_keeps_low_satellite_base_gate(self) -> None:
        sample = epoch(1.0, prefit_rms_m=1.0)
        sample = audit.SolutionEpoch(**{**sample.__dict__, "nsat": 11, "ratio": 10.0})
        self.assertTrue(policy_audit.base_matches(sample, policy_audit.Policy()))

    def test_frozen_floor_rejects_odaiba_false_alarm_shape(self) -> None:
        policy = policy_audit.Policy()
        self.assertEqual(policy.streak_prefit_rms_m, 40.0)
        self.assertEqual(policy.streak_min_outliers, 12)
        epochs = [epoch(float(index), prefit_rms_m=24.0) for index in range(27)]
        streak, spike = policy_audit.selected_indices(epochs, policy)
        self.assertEqual(streak | spike, set())

    def test_frozen_outlier_floor_breaks_shinjuku_false_alarm_shape(self) -> None:
        epochs = [
            epoch(
                float(index),
                prefit_rms_m=55.0,
                outliers=11 if index == 4 else 12,
                observations=34,
            )
            for index in range(8)
        ]
        streak, spike = policy_audit.selected_indices(epochs, policy_audit.Policy())
        self.assertEqual(streak | spike, set())

    def test_frozen_floor_keeps_confirmed_nagoya2_shape(self) -> None:
        epochs = [epoch(float(index), prefit_rms_m=50.0) for index in range(8)]
        streak, _ = policy_audit.selected_indices(epochs, policy_audit.Policy())
        self.assertEqual(streak, set(range(8)))

    def test_urbannav_half_epoch_reference_match_is_bounded(self) -> None:
        index = policy_audit.build_reference_index({(2300, 1.1): (1.0, 2.0, 3.0)})
        self.assertEqual(
            policy_audit.nearest_reference(index, 2300, 1.0, 0.25),
            (1.0, 2.0, 3.0),
        )
        self.assertIsNone(policy_audit.nearest_reference(index, 2300, 0.8, 0.25))

    def test_urbannav_comma_space_reference_header_is_supported(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_urbannav_reference_") as temp_dir:
            path = Path(temp_dir) / "reference.csv"
            path.write_text(
                "GPS TOW (s), GPS Week, ECEF X (m), ECEF Y (m), ECEF Z (m)\n"
                "1.10, 2300, 1.0, 2.0, 3.0\n",
                encoding="utf-8",
            )
            self.assertEqual(
                audit.load_reference(path),
                {(2300, 1.1): (1.0, 2.0, 3.0)},
            )


if __name__ == "__main__":
    unittest.main()
