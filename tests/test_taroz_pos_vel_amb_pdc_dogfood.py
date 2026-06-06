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

    def canonical_summary(self, profile_name: str = "default") -> dict[str, object]:
        profile = gnss_taroz_pos_vel_amb_pdc_dogfood.expectation_profile(
            profile_name
        )
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
            "use_epoch_lambda_fixed_output": profile[
                "use_epoch_lambda_fixed_output"
            ],
            "dd_ambiguity_per_epoch": True,
            "use_ambiguity_priors": False,
            "reject_rover_carrier_lli": True,
            "insert_fixed_interval_gaps": True,
            "exclude_glonass_qzss_sbas": True,
            "pseudorange_huber_threshold_sigma": 1.234,
            "carrier_phase_huber_threshold_sigma": 1.234,
            "tdcp_huber_threshold_sigma": 1.234,
            "seed_match_tolerance_s": profile["seed_match_tolerance_s"],
            "seed_interpolation_max_gap_s": profile[
                "seed_interpolation_max_gap_s"
            ],
            "lambda_ratio_threshold": profile["lambda_ratio_threshold"],
            "velocity_motion_sigma_m": 0.01,
            "ambiguity_between_sigma_cycles": 0.001,
            "min_snr_dbhz": 35.0,
            "min_satellites_per_epoch": 0,
            "min_output_dd_carrier_factors_per_epoch": 6,
            "max_float_seed_position_divergence_m": profile[
                "max_float_seed_position_divergence_m"
            ],
            "max_float_position_jump_m": profile["max_float_position_jump_m"],
            "lambda_ambiguity_fix_solved": True,
            "lambda_ambiguity_fix_used": profile["lambda_ambiguity_fix_used"],
            "partial_lambda_ambiguity_fix_used": False,
            "fixed_solution": profile["fixed_solution"],
            "converged": True,
            "final_cost": 57158.72726,
            "total_processing_time_ms": 41508.1587,
        }
        summary.update(profile["counts"])
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
            self.assertEqual(matlab_dump["skip_epochs"], 0)
            self.assertEqual(matlab_dump["max_epochs"], 0)
            self.assertIn("--preset", command)
            self.assertIn("taroz-amb-pdc", command)
            self.assertIn("--base", command)
            self.assertIn("--factor-debug-csv", command)
            self.assertIn("--sd-factor-debug-csv", command)
            self.assertIn("--lambda-debug-csv", command)
            self.assertIn("--cost-trace-csv", command)
            self.assertIn("--max-float-seed-divergence", command)
            self.assertIn("--max-float-position-jump", command)
            self.assertEqual(payload["expected"]["profile"], "default")
            self.assertEqual(payload["expected"]["lambda_ratio_threshold"], 2.0)
            self.assertEqual(payload["expected"]["counts"]["seed_matched_epochs"], 1141)
            self.assertEqual(
                payload["expected"]["counts"]["seed_interpolated_epochs"],
                34,
            )
            self.assertTrue(payload["expected"]["use_epoch_lambda_fixed_output"])
            self.assertEqual(payload["expected"]["counts"]["fixed_solutions"], 721)
            self.assertEqual(payload["expected"]["counts"]["float_solutions"], 243)

    def test_dry_run_records_nondefault_expectation_profile(self) -> None:
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
                    "--fgo-extra-arg=--lambda-ratio-threshold",
                    "--fgo-extra-arg=100",
                    "--expectation-profile",
                    "strict-lambda-ratio",
                    "--dry-run",
                ]
            )

            self.assertEqual(result, 0)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            command = payload["fgo_command"]
            self.assertIn("--lambda-ratio-threshold", command)
            self.assertIn("100", command)
            self.assertEqual(payload["expected"]["profile"], "strict-lambda-ratio")
            self.assertEqual(payload["expected"]["lambda_ratio_threshold"], 100.0)
            self.assertTrue(payload["expected"]["use_epoch_lambda_fixed_output"])
            self.assertFalse(payload["expected"]["lambda_ambiguity_fix_used"])
            self.assertFalse(payload["expected"]["fixed_solution"])
            self.assertEqual(payload["expected"]["counts"]["fixed_solutions"], 0)
            self.assertEqual(payload["expected"]["counts"]["float_solutions"], 964)

    def test_dry_run_records_seed_and_window_profiles(self) -> None:
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
                    "--fgo-extra-arg=--skip-epochs",
                    "--fgo-extra-arg=400",
                    "--fgo-extra-arg=--max-epochs",
                    "--fgo-extra-arg=120",
                    "--expectation-profile",
                    "shifted-120-window",
                    "--dry-run",
                ]
            )

            self.assertEqual(result, 0)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            command = payload["fgo_command"]
            self.assertIn("--skip-epochs", command)
            self.assertIn("400", command)
            self.assertIn("--max-epochs", command)
            self.assertIn("120", command)
            self.assertEqual(payload["expected"]["profile"], "shifted-120-window")
            self.assertEqual(payload["expected"]["counts"]["skip_epochs"], 400)
            self.assertEqual(payload["expected"]["counts"]["max_epochs"], 120)
            self.assertEqual(payload["expected"]["counts"]["valid_solutions"], 57)

    def test_dry_run_records_no_seed_interpolation_profile(self) -> None:
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
                    "--fgo-extra-arg=--seed-interpolation-max-gap",
                    "--fgo-extra-arg=0",
                    "--expectation-profile",
                    "no-seed-interpolation",
                    "--dry-run",
                ]
            )

            self.assertEqual(result, 0)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            command = payload["fgo_command"]
            self.assertIn("--seed-interpolation-max-gap", command)
            self.assertIn("0", command)
            self.assertEqual(payload["expected"]["profile"], "no-seed-interpolation")
            self.assertEqual(payload["expected"]["seed_interpolation_max_gap_s"], 0.0)
            self.assertEqual(payload["expected"]["counts"]["seed_matched_epochs"], 1107)
            self.assertEqual(payload["expected"]["counts"]["seed_interpolated_epochs"], 0)

    def test_dry_run_records_combined_ratio_window_profile(self) -> None:
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
                    "--fgo-extra-arg=--max-epochs",
                    "--fgo-extra-arg=120",
                    "--fgo-extra-arg=--lambda-ratio-threshold",
                    "--fgo-extra-arg=100",
                    "--expectation-profile",
                    "first-120-strict-lambda-ratio",
                    "--dry-run",
                ]
            )

            self.assertEqual(result, 0)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            command = payload["fgo_command"]
            self.assertIn("--max-epochs", command)
            self.assertIn("120", command)
            self.assertIn("--lambda-ratio-threshold", command)
            self.assertIn("100", command)
            self.assertEqual(
                payload["expected"]["profile"],
                "first-120-strict-lambda-ratio",
            )
            self.assertEqual(payload["expected"]["lambda_ratio_threshold"], 100.0)
            self.assertEqual(payload["expected"]["counts"]["max_epochs"], 120)
            self.assertEqual(payload["expected"]["counts"]["fixed_solutions"], 0)
            self.assertEqual(payload["expected"]["counts"]["float_solutions"], 120)
            self.assertFalse(payload["expected"]["lambda_ambiguity_fix_used"])

    def test_dry_run_records_combined_shifted_ratio_window_profile(self) -> None:
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
                    "--fgo-extra-arg=--skip-epochs",
                    "--fgo-extra-arg=400",
                    "--fgo-extra-arg=--max-epochs",
                    "--fgo-extra-arg=120",
                    "--fgo-extra-arg=--lambda-ratio-threshold",
                    "--fgo-extra-arg=100",
                    "--expectation-profile",
                    "shifted-120-strict-lambda-ratio",
                    "--dry-run",
                ]
            )

            self.assertEqual(result, 0)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            command = payload["fgo_command"]
            self.assertIn("--skip-epochs", command)
            self.assertIn("400", command)
            self.assertIn("--max-epochs", command)
            self.assertIn("120", command)
            self.assertIn("--lambda-ratio-threshold", command)
            self.assertIn("100", command)
            self.assertEqual(
                payload["expected"]["profile"],
                "shifted-120-strict-lambda-ratio",
            )
            self.assertEqual(payload["expected"]["lambda_ratio_threshold"], 100.0)
            self.assertEqual(payload["expected"]["counts"]["skip_epochs"], 400)
            self.assertEqual(payload["expected"]["counts"]["max_epochs"], 120)
            self.assertEqual(payload["expected"]["counts"]["valid_solutions"], 57)
            self.assertEqual(payload["expected"]["counts"]["fixed_solutions"], 0)
            self.assertEqual(payload["expected"]["counts"]["float_solutions"], 57)
            self.assertFalse(payload["expected"]["lambda_ambiguity_fix_used"])

    def test_matlab_window_follows_selected_expectation_profile(self) -> None:
        args = gnss_taroz_pos_vel_amb_pdc_dogfood.parse_args(
            [
                "--expectation-profile",
                "shifted-120-window",
            ]
        )

        self.assertEqual(
            gnss_taroz_pos_vel_amb_pdc_dogfood.matlab_window(args),
            (412, 120),
        )

        env = gnss_taroz_pos_vel_amb_pdc_dogfood.matlab_dump_env(args)

        self.assertEqual(env["GNSSPP_TAROZ_POS_VEL_AMB_PDC_SKIP_EPOCHS"], "412")
        self.assertEqual(env["GNSSPP_TAROZ_POS_VEL_AMB_PDC_MAX_EPOCHS"], "120")

    def test_matlab_window_override_takes_precedence(self) -> None:
        args = gnss_taroz_pos_vel_amb_pdc_dogfood.parse_args(
            [
                "--expectation-profile",
                "shifted-120-window",
                "--matlab-skip-epochs",
                "10",
                "--matlab-max-epochs",
                "20",
            ]
        )

        self.assertEqual(
            gnss_taroz_pos_vel_amb_pdc_dogfood.matlab_window(args),
            (10, 20),
        )

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
            self.assertNotIn("GNSSPP_TAROZ_POS_VEL_AMB_PDC_DATA_DIR", env)
            self.assertEqual(env["GNSSPP_TAROZ_POS_VEL_AMB_PDC_SKIP_EPOCHS"], "0")
            self.assertEqual(env["GNSSPP_TAROZ_POS_VEL_AMB_PDC_MAX_EPOCHS"], "0")

    def test_matlab_dump_env_uses_requested_data_dir(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_taroz_amb_pdc_dogfood_test_") as temp_dir:
            temp_root = Path(temp_dir)
            args = gnss_taroz_pos_vel_amb_pdc_dogfood.parse_args(
                [
                    "--generate-matlab-dump",
                    "--taroz-root",
                    str(temp_root / "taroz"),
                    "--matlab-data-dir",
                    str(temp_root / "ppc_data"),
                ]
            )

            env = gnss_taroz_pos_vel_amb_pdc_dogfood.matlab_dump_env(args)

            self.assertEqual(
                env["GNSSPP_TAROZ_POS_VEL_AMB_PDC_DATA_DIR"],
                str(temp_root / "ppc_data"),
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

    def test_native_summary_accepts_no_epoch_lambda_output_profile(self) -> None:
        failures = gnss_taroz_pos_vel_amb_pdc_dogfood.verify_native_summary(
            self.canonical_summary("no-epoch-lambda-fixed-output"),
            "no-epoch-lambda-fixed-output",
        )

        self.assertEqual(failures, [])

    def test_native_summary_accepts_strict_lambda_ratio_profile(self) -> None:
        failures = gnss_taroz_pos_vel_amb_pdc_dogfood.verify_native_summary(
            self.canonical_summary("strict-lambda-ratio"),
            "strict-lambda-ratio",
        )

        self.assertEqual(failures, [])

    def test_native_summary_accepts_no_seed_interpolation_profile(self) -> None:
        failures = gnss_taroz_pos_vel_amb_pdc_dogfood.verify_native_summary(
            self.canonical_summary("no-seed-interpolation"),
            "no-seed-interpolation",
        )

        self.assertEqual(failures, [])

    def test_seed_interpolation_profiles_are_distinct(self) -> None:
        default = gnss_taroz_pos_vel_amb_pdc_dogfood.expectation_profile("default")
        no_seed_interpolation = gnss_taroz_pos_vel_amb_pdc_dogfood.expectation_profile(
            "no-seed-interpolation"
        )

        self.assertEqual(default["seed_interpolation_max_gap_s"], 30.0)
        self.assertEqual(default["counts"]["seed_matched_epochs"], 1141)
        self.assertEqual(default["counts"]["seed_interpolated_epochs"], 34)
        self.assertEqual(no_seed_interpolation["seed_interpolation_max_gap_s"], 0.0)
        self.assertEqual(no_seed_interpolation["counts"]["seed_matched_epochs"], 1107)
        self.assertEqual(no_seed_interpolation["counts"]["seed_interpolated_epochs"], 0)
        self.assertEqual(
            no_seed_interpolation["counts"]["seed_matched_epochs"],
            default["counts"]["seed_matched_epochs"]
            - default["counts"]["seed_interpolated_epochs"],
        )

    def test_native_summary_accepts_window_profiles(self) -> None:
        for profile_name in (
            "first-120-window",
            "first-120-strict-lambda-ratio",
            "shifted-120-window",
            "shifted-120-strict-lambda-ratio",
        ):
            with self.subTest(profile_name=profile_name):
                failures = gnss_taroz_pos_vel_amb_pdc_dogfood.verify_native_summary(
                    self.canonical_summary(profile_name),
                    profile_name,
                )

                self.assertEqual(failures, [])

    def test_native_summary_accepts_seed_divergence_guard_profile(self) -> None:
        failures = gnss_taroz_pos_vel_amb_pdc_dogfood.verify_native_summary(
            self.canonical_summary("first-120-seed-divergence-guard"),
            "first-120-seed-divergence-guard",
        )

        self.assertEqual(failures, [])

    def test_native_summary_rejects_nondefault_profile_as_default(self) -> None:
        failures = gnss_taroz_pos_vel_amb_pdc_dogfood.verify_native_summary(
            self.canonical_summary("no-epoch-lambda-fixed-output")
        )

        self.assertIn(
            "taroz-amb-pdc use_epoch_lambda_fixed_output must be true "
            "for profile default",
            failures,
        )
        self.assertIn(
            "taroz-amb-pdc fixed_solutions must be 721 for profile default",
            failures,
        )
        self.assertIn(
            "taroz-amb-pdc lambda_ambiguity_fix_used must be true "
            "for profile default",
            failures,
        )

    def test_native_summary_flags_count_drift(self) -> None:
        summary = self.canonical_summary()
        summary["fixed_solutions"] = 720

        failures = gnss_taroz_pos_vel_amb_pdc_dogfood.verify_native_summary(summary)

        self.assertEqual(
            failures,
            ["taroz-amb-pdc fixed_solutions must be 721 for profile default"],
        )

    def test_native_summary_flags_seed_interpolation_drift(self) -> None:
        summary = self.canonical_summary()
        summary["seed_interpolated_epochs"] = 0

        failures = gnss_taroz_pos_vel_amb_pdc_dogfood.verify_native_summary(summary)

        self.assertEqual(
            failures,
            [
                "taroz-amb-pdc seed_interpolated_epochs must be 34 "
                "for profile default"
            ],
        )

    def test_native_summary_rejects_default_seed_interpolation_as_no_seed_profile(self) -> None:
        failures = gnss_taroz_pos_vel_amb_pdc_dogfood.verify_native_summary(
            self.canonical_summary("default"),
            "no-seed-interpolation",
        )

        self.assertIn("taroz-amb-pdc seed_interpolation_max_gap_s must be 0.0", failures)
        self.assertIn(
            "taroz-amb-pdc seed_matched_epochs must be 1107 "
            "for profile no-seed-interpolation",
            failures,
        )
        self.assertIn(
            "taroz-amb-pdc seed_interpolated_epochs must be 0 "
            "for profile no-seed-interpolation",
            failures,
        )

    def test_cost_trace_accepts_summary_contract(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_taroz_amb_pdc_cost_test_") as temp_dir:
            path = Path(temp_dir) / "cost_trace.csv"
            path.write_text(
                "phase,local_iteration,global_iteration,cost,"
                "absolute_decrease,relative_decrease,update_norm,converged\n"
                "float,0,0,100.0,NaN,NaN,1.0,0\n"
                "float,1,1,10.0,90.0,0.9,NaN,1\n",
                encoding="utf-8",
            )
            summary = self.canonical_summary()
            summary["iterations"] = 1
            summary["initial_cost"] = 100.0
            summary["final_cost"] = 10.0

            failures = gnss_taroz_pos_vel_amb_pdc_dogfood.verify_cost_trace(
                path,
                summary,
            )

            self.assertEqual(failures, [])

    def test_cost_trace_flags_broken_contract(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_taroz_amb_pdc_cost_test_") as temp_dir:
            path = Path(temp_dir) / "cost_trace.csv"
            path.write_text(
                "phase,local_iteration,global_iteration,cost\n"
                "float,0,1,10.0\n"
                "float,1,1,11.0\n",
                encoding="utf-8",
            )
            summary = self.canonical_summary()
            summary["iterations"] = 1
            summary["initial_cost"] = 10.0
            summary["final_cost"] = 11.0

            failures = gnss_taroz_pos_vel_amb_pdc_dogfood.verify_cost_trace(
                path,
                summary,
            )

            self.assertIn(
                "taroz-amb-pdc cost trace global iterations must be unique",
                failures,
            )
            self.assertIn(
                "taroz-amb-pdc cost trace final cost must decrease",
                failures,
            )


if __name__ == "__main__":
    unittest.main()
