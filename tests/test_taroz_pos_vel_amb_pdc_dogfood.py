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
            "max_float_seed_position_divergence_m": 0.0,
            "max_float_position_jump_m": 0.0,
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
            taroz_root = temp_root / "taroz"
            taroz_example_dir = taroz_root / "examples"
            matlab_dump_script = temp_root / "dump_taroz.m"
            matlab_dir = temp_root / "matlab_oracle"

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
                    "--generate-matlab-dump",
                    "--matlab-bin",
                    str(temp_root / "matlab"),
                    "--taroz-root",
                    str(taroz_root),
                    "--taroz-example-dir",
                    str(taroz_example_dir),
                    "--matlab-dump-script",
                    str(matlab_dump_script),
                    "--matlab-dir",
                    str(matlab_dir),
                    "--dry-run",
                ]
            )

            self.assertEqual(result, 0)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            command = payload["fgo_command"]
            matlab_dump = payload["matlab_dump"]
            self.assertEqual(payload["status"], "dry-run")
            self.assertTrue(matlab_dump["enabled"])
            self.assertEqual(matlab_dump["dump_script"], str(matlab_dump_script))
            self.assertEqual(matlab_dump["taroz_root"], str(taroz_root))
            self.assertEqual(matlab_dump["example_dir"], str(taroz_example_dir))
            self.assertEqual(matlab_dump["out_dir"], str(matlab_dir))
            self.assertEqual(matlab_dump["command"][0], str(temp_root / "matlab"))
            self.assertEqual(matlab_dump["command"][1], "-batch")
            self.assertIn(str(matlab_dump_script), matlab_dump["command"][2])
            self.assertIn("--preset", command)
            self.assertIn("taroz-amb-pdc", command)
            self.assertIn("--base", command)
            self.assertIn("--factor-debug-csv", command)
            self.assertIn("--sd-factor-debug-csv", command)
            self.assertIn("--lambda-debug-csv", command)
            self.assertIn("--cost-trace-csv", command)
            self.assertIn("--max-float-seed-divergence", command)
            self.assertIn("--max-float-position-jump", command)
            self.assertEqual(payload["expected"]["counts"]["fixed_solutions"], 721)
            self.assertEqual(payload["expected"]["counts"]["float_solutions"], 243)

    def test_matlab_dump_env_uses_requested_oracle_paths(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_taroz_amb_pdc_dogfood_test_") as temp_dir:
            temp_root = Path(temp_dir)
            args = gnss_taroz_pos_vel_amb_pdc_dogfood.parse_args(
                [
                    "--generate-matlab-dump",
                    "--taroz-root",
                    str(temp_root / "taroz"),
                    "--taroz-example-dir",
                    str(temp_root / "taroz" / "examples"),
                    "--matlab-dir",
                    str(temp_root / "matlab_oracle"),
                ]
            )

            env = gnss_taroz_pos_vel_amb_pdc_dogfood.matlab_dump_env(args)

            self.assertEqual(
                env["GNSSPP_TAROZ_ROOT"],
                str(temp_root / "taroz"),
            )
            self.assertEqual(
                env["GNSSPP_TAROZ_POS_VEL_AMB_PDC_EXAMPLE_DIR"],
                str(temp_root / "taroz" / "examples"),
            )
            self.assertEqual(
                env["GNSSPP_TAROZ_POS_VEL_AMB_PDC_OUT_DIR"],
                str(temp_root / "matlab_oracle"),
            )

    def test_matlab_dump_env_resolves_relative_output_under_repo(self) -> None:
        args = gnss_taroz_pos_vel_amb_pdc_dogfood.parse_args(
            [
                "--generate-matlab-dump",
                "--taroz-root",
                "relative_taroz_root",
                "--matlab-dir",
                "relative_matlab_oracle",
            ]
        )

        env = gnss_taroz_pos_vel_amb_pdc_dogfood.matlab_dump_env(args)

        self.assertEqual(
            env["GNSSPP_TAROZ_ROOT"],
            str(ROOT_DIR / "relative_taroz_root"),
        )
        self.assertEqual(
            env["GNSSPP_TAROZ_POS_VEL_AMB_PDC_EXAMPLE_DIR"],
            str(ROOT_DIR / "relative_taroz_root" / "examples"),
        )
        self.assertEqual(
            env["GNSSPP_TAROZ_POS_VEL_AMB_PDC_OUT_DIR"],
            str(ROOT_DIR / "relative_matlab_oracle"),
        )

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
