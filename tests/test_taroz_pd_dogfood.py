#!/usr/bin/env python3
"""Tests for the taroz PD dogfood harness."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"
sys.path.insert(0, str(APPS_DIR))

import gnss_taroz_pd_dogfood  # noqa: E402


class TarozPdDogfoodTest(unittest.TestCase):
    def touch_inputs(self, root: Path) -> tuple[Path, Path, Path]:
        obs = root / "rover.obs"
        nav = root / "base.nav"
        seed_pos = root / "seed.pos"
        for path in (obs, nav, seed_pos):
            path.write_text("", encoding="ascii")
        return obs, nav, seed_pos

    def test_dry_run_writes_bounded_smoke_plan(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_taroz_pd_dogfood_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs, nav, seed_pos = self.touch_inputs(temp_root)
            summary_json = temp_root / "summary.json"
            out_dir = temp_root / "out"
            taroz_root = temp_root / "taroz"
            taroz_example_dir = taroz_root / "examples"
            matlab_dump_script = temp_root / "dump_taroz_pd.m"
            matlab_dir = temp_root / "matlab_oracle"

            result = gnss_taroz_pd_dogfood.main(
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
                    "--pd-bin",
                    str(temp_root / "gnss_pos_vel_pd"),
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
            command = payload["pd_command"]
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
            self.assertIn("--max-epochs", command)
            self.assertIn(str(gnss_taroz_pd_dogfood.DEFAULT_MAX_EPOCHS), command)
            self.assertIn("--max-iterations", command)
            self.assertIn(str(gnss_taroz_pd_dogfood.DEFAULT_MAX_ITERATIONS), command)
            self.assertIn("--factor-debug-csv", command)
            self.assertIn("factor_debug.csv", " ".join(command))

    def test_matlab_dump_env_uses_requested_oracle_paths(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_taroz_pd_dogfood_test_") as temp_dir:
            temp_root = Path(temp_dir)
            args = gnss_taroz_pd_dogfood.parse_args(
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

            env = gnss_taroz_pd_dogfood.matlab_dump_env(args)

            self.assertEqual(env["GNSSPP_TAROZ_ROOT"], str(temp_root / "taroz"))
            self.assertEqual(
                env["GNSSPP_TAROZ_PD_EXAMPLE_DIR"],
                str(temp_root / "taroz" / "examples"),
            )
            self.assertEqual(
                env["GNSSPP_TAROZ_PD_OUT_DIR"],
                str(temp_root / "matlab_oracle"),
            )

    def test_matlab_dump_env_resolves_relative_output_under_repo(self) -> None:
        args = gnss_taroz_pd_dogfood.parse_args(
            [
                "--generate-matlab-dump",
                "--taroz-root",
                "relative_taroz_root",
                "--matlab-dir",
                "relative_matlab_oracle",
            ]
        )

        env = gnss_taroz_pd_dogfood.matlab_dump_env(args)

        self.assertEqual(
            env["GNSSPP_TAROZ_ROOT"],
            str(ROOT_DIR / "relative_taroz_root"),
        )
        self.assertEqual(
            env["GNSSPP_TAROZ_PD_EXAMPLE_DIR"],
            str(ROOT_DIR / "relative_taroz_root" / "examples"),
        )
        self.assertEqual(
            env["GNSSPP_TAROZ_PD_OUT_DIR"],
            str(ROOT_DIR / "relative_matlab_oracle"),
        )

    def test_dry_run_can_keep_native_runtime_limits(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_taroz_pd_dogfood_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs, nav, seed_pos = self.touch_inputs(temp_root)
            summary_json = temp_root / "summary.json"

            result = gnss_taroz_pd_dogfood.main(
                [
                    "--obs",
                    str(obs),
                    "--nav",
                    str(nav),
                    "--seed-pos",
                    str(seed_pos),
                    "--summary-json",
                    str(summary_json),
                    "--pd-bin",
                    str(temp_root / "gnss_pos_vel_pd"),
                    "--max-epochs",
                    "0",
                    "--max-iterations",
                    "0",
                    "--dry-run",
                ]
            )

            self.assertEqual(result, 0)
            command = json.loads(summary_json.read_text(encoding="utf-8"))["pd_command"]
            self.assertNotIn("--max-epochs", command)
            self.assertNotIn("--max-iterations", command)

    def test_native_summary_accepts_taroz_pd_shape(self) -> None:
        summary = {
            "preset": "taroz-pd",
            "backend": "eigen",
            "pseudorange_sigma_zenith_m": 3,
            "doppler_sigma_zenith_mps": 0.2,
            "position_prior_sigma_m": 1000,
            "clock_prior_sigma_m": 1_000_000,
            "velocity_prior_sigma_mps": 1000,
            "clock_drift_prior_sigma_mps": 1000,
            "motion_sigma_m": 0.1,
            "clock_motion_sigma_m": 0.1,
            "clock_drift_between_sigma_mps": 0.1,
            "optimized_epochs": 80,
            "valid_position_epochs": 80,
            "valid_velocity_epochs": 80,
            "motion_factors": 79,
            "clock_motion_factors": 79,
            "clock_drift_between_factors": 79,
            "graph_values": 320,
            "pseudorange_factors": 2492,
            "doppler_factors": 2492,
            "iterations": 8,
            "converged": True,
            "initial_cost": 1000.0,
            "final_cost": 288.7,
            "residual_rms_mps": 1.5,
        }

        failures = gnss_taroz_pd_dogfood.verify_native_summary(
            summary,
            expected_max_epochs=80,
            expected_max_iterations=40,
        )

        self.assertEqual(failures, [])


if __name__ == "__main__":
    unittest.main()
