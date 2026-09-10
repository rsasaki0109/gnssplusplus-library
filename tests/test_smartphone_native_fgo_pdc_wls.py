#!/usr/bin/env python3
"""Contract tests for the development-only Doppler-velocity WLS lane."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_native_fgo_pdc_wls as wls  # noqa: E402


def _valid_summary() -> dict[str, object]:
    return {
        "backend": "eigen",
        "preset": "default",
        "input_epochs": 2,
        "optimized_epochs": 2,
        "valid_solutions": 2,
        "pseudorange_factors": 8,
        "undifferenced_doppler_factors": 8,
        "tdcp_factors": 4,
        "carrier_phase_factors": 4,
        "ambiguity_states": 2,
        "motion_factors": 2,
        "graph_factors": 30,
        "graph_values": 12,
        "initial_cost": 10.0,
        "final_cost": 5.0,
        "doppler_velocity_wls_valid_epochs": 2,
        "doppler_velocity_wls_propagated_epochs": 0,
        "doppler_velocity_wls_rejected_epochs": 0,
        "doppler_velocity_wls_max_condition_number": 12.0,
        "doppler_velocity_wls_max_normalized_rms": 1.5,
        "doppler_velocity_wls_max_velocity_norm_mps": 4.0,
        "doppler_velocity_wls_max_clock_rate_abs_mps": 3.0,
        "single_difference_doppler_factors": 0,
        "single_difference_tdcp_factors": 0,
        "use_undifferenced_doppler_factors": True,
        "use_corrected_undifferenced_doppler_factors": True,
        "use_doppler_velocity_wls_initialization": True,
        "use_velocity_states": True,
        "use_velocity_motion_factors": True,
    }


class NativeFgoPdcWlsTests(unittest.TestCase):
    def test_command_is_deterministic_and_enables_corrected_wls(self) -> None:
        route = {"obs": Path("rover.obs"), "nav": Path("brdc.nav")}
        first = wls._command(Path("gnss_fgo"), route, Path("tmp"), 30)
        second = wls._command(Path("gnss_fgo"), route, Path("tmp"), 30)
        self.assertEqual(first, second)
        self.assertIn("--corrected-undifferenced-doppler-factors", first)
        self.assertIn("--doppler-velocity-wls-initialization", first)
        self.assertIn("--velocity-states", first)
        self.assertIn("--velocity-motion-factors", first)
        self.assertIn("--no-dd-factors", first)
        self.assertNotIn("--base", first)

    def test_summary_requires_every_epoch_wls_and_physical_limits(self) -> None:
        summary = _valid_summary()
        self.assertEqual(wls._validate_summary(summary), summary)

        rejected = dict(summary)
        rejected["doppler_velocity_wls_rejected_epochs"] = 1
        with self.assertRaises(wls.WlsError):
            wls._validate_summary(rejected)

        unbounded = dict(summary)
        unbounded["doppler_velocity_wls_max_velocity_norm_mps"] = 70.0001
        with self.assertRaises(wls.WlsError):
            wls._validate_summary(unbounded)

    def test_summary_rejects_forbidden_base_dependent_factors(self) -> None:
        summary = _valid_summary()
        summary["single_difference_doppler_factors"] = 1
        with self.assertRaises(wls.WlsError):
            wls._validate_summary(summary)

    def test_summary_rejects_nonfinite_numeric_fields(self) -> None:
        summary = _valid_summary()
        summary["doppler_velocity_wls_max_normalized_rms"] = float("nan")
        with self.assertRaises(wls.WlsError):
            wls._validate_summary(summary)


if __name__ == "__main__":
    unittest.main()
