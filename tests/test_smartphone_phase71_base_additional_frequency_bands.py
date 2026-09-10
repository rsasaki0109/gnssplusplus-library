"""Focused tests for the Phase71 structural evaluator contract."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase71_base_additional_frequency_bands.py"
SPEC = importlib.util.spec_from_file_location("phase71_base_bands", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def summary_for(route: str, matched: int = 80, finite_matched: int = 80) -> dict:
    return {
        "dataset_id": route,
        "truth_used": False,
        "production_default_changed": False,
        "native_quality_anchor": True,
        "native_pdc_imu_tdcp_no_bridge": True,
        "native_pdc_state_bridge": False,
        "graph": {"converged": True},
        "epochs": {"problem": 3, "output": 3, "pseudorange_factors": 10},
        "raw_utc_key_contract": {"unresolved_epochs": 0, "target_epochs": 2, "exact_solution_epochs": 2},
        "tdcp_contract": {"factors_built": 2, "factors_inserted": 2, "nonfinite_residuals": 0},
        "native_base_pseudorange_compensation": {
            "enabled": True,
            "built": True,
            "applied": True,
            "preserve_additional_frequency_bands": True,
            "base_member_sha256": MODULE.BASE_INPUT_HASHES[route]["sha256"],
            "base_rinex_sha256": MODULE.BASE_INPUT_HASHES[route]["sha256"],
            "base_rinex_bytes": MODULE.BASE_INPUT_HASHES[route]["bytes"],
            "base_coordinate_xyz_m": MODULE.BASE_INPUT_HASHES[route]["xyz"],
            "observed_interval_s": MODULE.BASE_INPUT_HASHES[route]["dt_s"],
            "moving_mean_samples": MODULE.BASE_INPUT_HASHES[route]["window"],
            "same_satellite_signal_only": True,
            "spp_applied": False,
            "tdcp_applied": False,
            "doppler_applied": False,
            "no_extrapolation_or_endpoint_hold": True,
            "selected_band_observation_rows": 20,
            "selected_band_streams": 4,
            "selected_band_observation_rows_by_signal": {"GPS_L1CA": 10, "GPS_L5": 10},
            "selected_band_streams_by_signal": {"GPS_L1CA": 2, "GPS_L5": 2},
            "adopted_pseudorange_rows": 100,
            "matched_factor_rows": matched,
            "finite_correction_rows_among_matched": finite_matched,
            "adopted_rows_corrected": finite_matched,
            "matched_factor_fraction": matched / 100.0,
            "finite_correction_fraction_among_matched": finite_matched / matched if matched else None,
            "finite_correction_fraction": finite_matched / 100.0,
        },
    }


class Phase71BaseBandTests(unittest.TestCase):
    def test_candidate_command_adds_preserve_flag_only_for_candidate(self) -> None:
        route = MODULE.ROUTES[0]
        run_dir = Path("output/smartphone-r5/phase71-test/route/candidate/run1")
        candidate = MODULE.native_command(route, run_dir, True)
        control = MODULE.native_command(route, Path("output/smartphone-r5/phase71-test/route/control/run1"), False)
        self.assertEqual(candidate[-1], MODULE.PRESERVE_FLAG)
        self.assertNotIn(MODULE.PRESERVE_FLAG, control)

    def test_matched_denominator_is_independent_of_all_adopted_fraction(self) -> None:
        route = MODULE.ROUTES[0]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "summary.json"
            path.write_text(json.dumps(summary_for(route, 80, 80)), encoding="utf-8")
            result = MODULE.validate_summary(path, route, True)
        self.assertEqual(result["base"]["matched_factor_fraction"], 0.8)
        self.assertEqual(result["base"]["finite_correction_fraction_among_matched"], 1.0)

    def test_each_frozen_coverage_gate_fails_closed(self) -> None:
        route = MODULE.ROUTES[0]
        for matched, finite in ((79, 79), (80, 79)):
            with self.subTest(matched=matched, finite=finite), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "summary.json"
                path.write_text(json.dumps(summary_for(route, matched, finite)), encoding="utf-8")
                with self.assertRaises(MODULE.Phase71Error):
                    MODULE.validate_summary(path, route, True)

    def test_freeze_and_binary_pins_are_explicit(self) -> None:
        self.assertEqual(len(MODULE.FREEZE_SHA256), 64)
        self.assertEqual(len(MODULE.BINARY_SHA256), 64)
        self.assertEqual(MODULE.ROUTES, tuple(MODULE.ROUTES))
        self.assertEqual(len(MODULE.ROUTES), 4)
        self.assertEqual(MODULE.PRESERVE_FLAG, "--native-base-pseudorange-preserve-additional-frequency-bands")


if __name__ == "__main__":
    unittest.main()
