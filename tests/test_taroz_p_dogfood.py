#!/usr/bin/env python3
"""Tests for the taroz P dogfood harness."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"
sys.path.insert(0, str(APPS_DIR))

import gnss_taroz_p_dogfood  # noqa: E402


class TarozPDogfoodTest(unittest.TestCase):
    def touch_inputs(self, root: Path) -> tuple[Path, Path, Path]:
        obs = root / "rover.obs"
        nav = root / "base.nav"
        seed_pos = root / "seed.pos"
        for path in (obs, nav, seed_pos):
            path.write_text("", encoding="ascii")
        return obs, nav, seed_pos

    def test_dry_run_writes_bounded_smoke_plan(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_taroz_p_dogfood_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs, nav, seed_pos = self.touch_inputs(temp_root)
            summary_json = temp_root / "summary.json"
            out_dir = temp_root / "out"

            result = gnss_taroz_p_dogfood.main(
                [
                    "--obs",
                    str(obs),
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
            self.assertIn("taroz-p", command)
            self.assertIn("--max-epochs", command)
            self.assertIn(str(gnss_taroz_p_dogfood.DEFAULT_MAX_EPOCHS), command)
            self.assertIn("--max-iterations", command)
            self.assertIn(str(gnss_taroz_p_dogfood.DEFAULT_MAX_ITERATIONS), command)
            self.assertIn("--factor-debug-csv", command)
            self.assertIn("fgo_taroz_p_factor_debug.csv", " ".join(command))
            self.assertFalse(payload["expected"]["use_spp_seed"])
            self.assertFalse(payload["expected"]["debug_problem_only"])

    def test_dry_run_can_keep_preset_runtime_limits(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_taroz_p_dogfood_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs, nav, seed_pos = self.touch_inputs(temp_root)
            summary_json = temp_root / "summary.json"

            result = gnss_taroz_p_dogfood.main(
                [
                    "--obs",
                    str(obs),
                    "--nav",
                    str(nav),
                    "--seed-pos",
                    str(seed_pos),
                    "--summary-json",
                    str(summary_json),
                    "--fgo-bin",
                    str(temp_root / "gnss_fgo"),
                    "--max-epochs",
                    "0",
                    "--max-iterations",
                    "0",
                    "--dry-run",
                ]
            )

            self.assertEqual(result, 0)
            command = json.loads(summary_json.read_text(encoding="utf-8"))["fgo_command"]
            self.assertNotIn("--max-epochs", command)
            self.assertNotIn("--max-iterations", command)

    def test_dry_run_can_request_debug_problem_only(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_taroz_p_dogfood_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs, nav, seed_pos = self.touch_inputs(temp_root)
            summary_json = temp_root / "summary.json"

            result = gnss_taroz_p_dogfood.main(
                [
                    "--obs",
                    str(obs),
                    "--nav",
                    str(nav),
                    "--seed-pos",
                    str(seed_pos),
                    "--summary-json",
                    str(summary_json),
                    "--fgo-bin",
                    str(temp_root / "gnss_fgo"),
                    "--debug-problem-only",
                    "--dry-run",
                ]
            )

            self.assertEqual(result, 0)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            command = payload["fgo_command"]
            self.assertIn("--debug-problem-only", command)
            self.assertTrue(payload["expected"]["debug_problem_only"])

    def test_native_summary_accepts_taroz_p_shape(self) -> None:
        summary = {
            "preset": "taroz-p",
            "backend": "eigen",
            "use_spp_seed": False,
            "use_pseudorange_factors": True,
            "pseudorange_sigma_m": 3,
            "pseudorange_elevation_sigma_power": 0.5,
            "position_prior_sigma_m": 1000,
            "clock_prior_sigma_m": 1_000_000,
            "clock_motion_sigma_m": 100,
            "use_position_motion_factors": False,
            "use_clock_motion_factors": True,
            "optimized_epochs": 80,
            "valid_solutions": 80,
            "pseudorange_factors": 2492,
            "tdcp_factors": 0,
            "carrier_phase_factors": 0,
            "double_difference_pseudorange_factors": 0,
            "double_difference_carrier_factors": 0,
            "ambiguity_states": 0,
            "motion_factors": 79,
            "iterations": 8,
            "converged": True,
            "final_cost": 288.7,
            "residual_rms_m": 1.5,
        }

        failures = gnss_taroz_p_dogfood.verify_native_summary(
            summary,
            expected_max_epochs=80,
            expected_max_iterations=40,
        )

        self.assertEqual(failures, [])

    def test_native_summary_accepts_debug_problem_only_shape(self) -> None:
        summary = {
            "preset": "taroz-p",
            "backend": "eigen",
            "debug_problem_only": True,
            "use_spp_seed": False,
            "use_pseudorange_factors": True,
            "pseudorange_sigma_m": 3,
            "pseudorange_elevation_sigma_power": 0.5,
            "position_prior_sigma_m": 1000,
            "clock_prior_sigma_m": 1_000_000,
            "clock_motion_sigma_m": 100,
            "use_position_motion_factors": False,
            "use_clock_motion_factors": True,
            "optimized_epochs": 1141,
            "valid_solutions": 1141,
            "pseudorange_factors": 24092,
            "tdcp_factors": 0,
            "carrier_phase_factors": 0,
            "double_difference_pseudorange_factors": 0,
            "double_difference_carrier_factors": 0,
            "ambiguity_states": 0,
            "motion_factors": 1140,
            "iterations": 0,
            "converged": False,
            "final_cost": 0.0,
            "residual_rms_m": 0.0,
        }

        failures = gnss_taroz_p_dogfood.verify_native_summary(
            summary,
            expected_max_epochs=0,
            expected_max_iterations=40,
            debug_problem_only=True,
        )

        self.assertEqual(failures, [])

    def test_native_summary_flags_wrong_factor_family(self) -> None:
        summary = {
            "preset": "taroz-p",
            "backend": "eigen",
            "use_spp_seed": False,
            "use_pseudorange_factors": True,
            "pseudorange_sigma_m": 3,
            "pseudorange_elevation_sigma_power": 0.5,
            "position_prior_sigma_m": 1000,
            "clock_prior_sigma_m": 1_000_000,
            "clock_motion_sigma_m": 100,
            "use_position_motion_factors": False,
            "use_clock_motion_factors": True,
            "optimized_epochs": 80,
            "valid_solutions": 80,
            "pseudorange_factors": 2492,
            "tdcp_factors": 2,
            "carrier_phase_factors": 0,
            "double_difference_pseudorange_factors": 0,
            "double_difference_carrier_factors": 0,
            "ambiguity_states": 0,
            "motion_factors": 79,
            "iterations": 8,
            "converged": True,
            "final_cost": 288.7,
            "residual_rms_m": 1.5,
        }

        failures = gnss_taroz_p_dogfood.verify_native_summary(
            summary,
            expected_max_epochs=80,
            expected_max_iterations=40,
        )

        self.assertEqual(failures, ["taroz-p must keep tdcp_factors=0"])


if __name__ == "__main__":
    unittest.main()
