#!/usr/bin/env python3
"""Tests for the taroz position/velocity ambiguity PDC dogfood harness."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"
sys.path.insert(0, str(APPS_DIR))

import gnss_taroz_pos_vel_amb_pdc_dogfood  # noqa: E402


class TarozPosVelAmbPdcDogfoodTest(unittest.TestCase):
    def touch_inputs(self, root: Path) -> tuple[Path, Path, Path, Path]:
        obs = root / "rover.obs"
        base = root / "base.obs"
        nav = root / "base.nav"
        seed_pos = root / "seed.pos"
        for path in (obs, base, nav, seed_pos):
            path.write_text("", encoding="ascii")
        return obs, base, nav, seed_pos

    def canonical_summary(self) -> dict[str, object]:
        summary: dict[str, object] = {
            "preset": "taroz-amb-pdc",
            "backend": "eigen",
            "debug_problem_only": False,
            "use_spp_seed": False,
            "use_pseudorange_factors": False,
            "use_single_difference_doppler_factors": True,
            "use_single_difference_tdcp_factors": True,
            "use_velocity_states": True,
            "use_velocity_motion_factors": True,
            "use_ambiguity_between_factors": True,
            "linearize_double_difference_factors_at_seed": True,
            "use_epoch_lambda_fixed_output": True,
            "dd_ambiguity_per_epoch": True,
            "use_ambiguity_priors": False,
            "reject_rover_carrier_lli": True,
            "insert_fixed_interval_gaps": True,
            "exclude_glonass_qzss_sbas": True,
            "pseudorange_huber_threshold_sigma": 1.234,
            "carrier_phase_huber_threshold_sigma": 1.234,
            "tdcp_huber_threshold_sigma": 1.234,
            "velocity_motion_sigma_m": 0.01,
            "ambiguity_between_sigma_cycles": 0.001,
            "min_snr_dbhz": 35.0,
            "min_satellites_per_epoch": 0,
            "min_output_dd_carrier_factors_per_epoch": 6,
            "lambda_ambiguity_fix_solved": True,
            "lambda_ambiguity_fix_used": True,
            "partial_lambda_ambiguity_fix_used": False,
            "fixed_solution": True,
            "converged": True,
            "final_cost": 57158.72726,
            "total_processing_time_ms": 41508.1587,
        }
        summary.update(gnss_taroz_pos_vel_amb_pdc_dogfood.EXPECTED_FULL_COUNTS)
        return summary

    def test_dry_run_writes_full_parity_plan(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_taroz_amb_pdc_dogfood_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs, base, nav, seed_pos = self.touch_inputs(temp_root)
            summary_json = temp_root / "summary.json"
            out_dir = temp_root / "out"

            result = gnss_taroz_pos_vel_amb_pdc_dogfood.main(
                [
                    "--obs",
                    str(obs),
                    "--base",
                    str(base),
                    "--nav",
                    str(nav),
                    "--seed-pos",
                    str(seed_pos),
                    "--out-dir",
                    str(out_dir),
                    "--summary-json",
                    str(summary_json),
                    "--fgo-bin",
                    str(temp_root / "gnss_fgo"),
                    "--dry-run",
                ]
            )

            self.assertEqual(result, 0)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            command = payload["fgo_command"]
            self.assertEqual(payload["status"], "dry-run")
            self.assertIn("--preset", command)
            self.assertIn("taroz-amb-pdc", command)
            self.assertIn("--base", command)
            self.assertIn("--factor-debug-csv", command)
            self.assertIn("--sd-factor-debug-csv", command)
            self.assertIn("--lambda-debug-csv", command)
            self.assertEqual(payload["expected"]["counts"]["fixed_solutions"], 721)
            self.assertEqual(payload["expected"]["counts"]["float_solutions"], 243)

    def test_native_summary_accepts_canonical_full_run(self) -> None:
        failures = gnss_taroz_pos_vel_amb_pdc_dogfood.verify_native_summary(
            self.canonical_summary()
        )

        self.assertEqual(failures, [])

    def test_native_summary_flags_count_drift(self) -> None:
        summary = self.canonical_summary()
        summary["fixed_solutions"] = 720

        failures = gnss_taroz_pos_vel_amb_pdc_dogfood.verify_native_summary(summary)

        self.assertEqual(failures, ["taroz-amb-pdc fixed_solutions must be 721"])


if __name__ == "__main__":
    unittest.main()
