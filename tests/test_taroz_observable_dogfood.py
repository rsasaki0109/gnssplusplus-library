#!/usr/bin/env python3
"""Tests for the taroz observable-mode dogfood harness."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"
sys.path.insert(0, str(APPS_DIR))

import gnss_taroz_observable_dogfood  # noqa: E402


class TarozObservableDogfoodTest(unittest.TestCase):
    def touch_inputs(self, root: Path) -> tuple[Path, Path, Path]:
        obs = root / "rover.obs"
        nav = root / "base.nav"
        seed_pos = root / "seed.pos"
        for path in (obs, nav, seed_pos):
            path.write_text("", encoding="ascii")
        return obs, nav, seed_pos

    def test_dry_run_writes_reproducible_plan_for_each_mode(self) -> None:
        for mode, spec in gnss_taroz_observable_dogfood.MODE_SPECS.items():
            with self.subTest(mode=mode):
                with tempfile.TemporaryDirectory(
                    prefix="gnss_taroz_observable_dogfood_test_"
                ) as temp_dir:
                    temp_root = Path(temp_dir)
                    obs, nav, seed_pos = self.touch_inputs(temp_root)
                    summary_json = temp_root / "summary.json"
                    out_dir = temp_root / "out"
                    taroz_root = temp_root / "taroz"
                    taroz_example_dir = taroz_root / "examples"
                    matlab_dump_script = temp_root / f"dump_{mode}.m"
                    matlab_dir = temp_root / "matlab_oracle"

                    result = gnss_taroz_observable_dogfood.main(
                        [
                            "--mode",
                            mode,
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
                            "--native-bin",
                            str(temp_root / spec.target),
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
                    command = payload["native_command"]
                    matlab_dump = payload["matlab_dump"]
                    self.assertEqual(payload["status"], "dry-run")
                    self.assertEqual(payload["mode"], mode)
                    self.assertEqual(payload["expected"]["preset"], spec.preset)
                    self.assertTrue(matlab_dump["enabled"])
                    self.assertEqual(matlab_dump["dump_script"], str(matlab_dump_script))
                    self.assertEqual(matlab_dump["taroz_root"], str(taroz_root))
                    self.assertEqual(matlab_dump["example_dir"], str(taroz_example_dir))
                    self.assertEqual(matlab_dump["out_dir"], str(matlab_dir))
                    self.assertEqual(matlab_dump["command"][0], str(temp_root / "matlab"))
                    self.assertEqual(matlab_dump["command"][1], "-batch")
                    self.assertIn(str(matlab_dump_script), matlab_dump["command"][2])
                    self.assertEqual(command[0], str(temp_root / spec.target))
                    self.assertIn("--out-csv", command)
                    self.assertIn(spec.out_csv_name, " ".join(command))
                    self.assertIn("--factor-debug-csv", command)
                    self.assertIn("--graph-csv", command)
                    self.assertIn("--summary-json", command)
                    self.assertNotIn("--max-epochs", command)
                    self.assertIn("--max-iterations", command)
                    self.assertIn(
                        str(gnss_taroz_observable_dogfood.DEFAULT_MAX_ITERATIONS),
                        command,
                    )

    def test_matlab_dump_env_resolves_requested_paths(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_taroz_observable_dogfood_test_") as temp_dir:
            temp_root = Path(temp_dir)
            args = gnss_taroz_observable_dogfood.parse_args(
                [
                    "--mode",
                    "pos-pdc",
                    "--generate-matlab-dump",
                    "--taroz-root",
                    str(temp_root / "taroz"),
                    "--taroz-example-dir",
                    str(temp_root / "taroz" / "examples"),
                    "--matlab-dir",
                    str(temp_root / "matlab_oracle"),
                ]
            )
            spec = gnss_taroz_observable_dogfood.mode_spec(args)

            env = gnss_taroz_observable_dogfood.matlab_dump_env(args, spec)

            self.assertEqual(env["GNSSPP_TAROZ_ROOT"], str(temp_root / "taroz"))
            self.assertEqual(
                env["GNSSPP_TAROZ_POS_PDC_EXAMPLE_DIR"],
                str(temp_root / "taroz" / "examples"),
            )
            self.assertEqual(
                env["GNSSPP_TAROZ_POS_PDC_OUT_DIR"],
                str(temp_root / "matlab_oracle"),
            )

    def test_matlab_dump_env_resolves_relative_output_under_repo(self) -> None:
        args = gnss_taroz_observable_dogfood.parse_args(
            [
                "--mode",
                "pos-vel-pdc",
                "--generate-matlab-dump",
                "--taroz-root",
                "relative_taroz_root",
                "--matlab-dir",
                "relative_matlab_oracle",
            ]
        )
        spec = gnss_taroz_observable_dogfood.mode_spec(args)

        env = gnss_taroz_observable_dogfood.matlab_dump_env(args, spec)

        self.assertEqual(
            env["GNSSPP_TAROZ_ROOT"],
            str(ROOT_DIR / "relative_taroz_root"),
        )
        self.assertEqual(
            env["GNSSPP_TAROZ_POS_VEL_PDC_EXAMPLE_DIR"],
            str(ROOT_DIR / "relative_taroz_root" / "examples"),
        )
        self.assertEqual(
            env["GNSSPP_TAROZ_POS_VEL_PDC_OUT_DIR"],
            str(ROOT_DIR / "relative_matlab_oracle"),
        )

    def test_max_epochs_zero_keeps_native_full_run(self) -> None:
        args = gnss_taroz_observable_dogfood.parse_args(
            [
                "--mode",
                "d",
                "--max-epochs",
                "0",
                "--max-iterations",
                "0",
                "--dry-run",
            ]
        )
        spec = gnss_taroz_observable_dogfood.mode_spec(args)
        paths = gnss_taroz_observable_dogfood.output_paths(Path("out"), spec)

        command = gnss_taroz_observable_dogfood.build_native_command(args, spec, paths)

        self.assertNotIn("--max-epochs", command)
        self.assertNotIn("--max-iterations", command)

    def test_native_summary_accepts_mode_shapes(self) -> None:
        samples = {
            "d": {
                "preset": "taroz-d",
                "backend": "eigen",
                "optimized_epochs": 1141,
                "valid_velocity_epochs": 1141,
                "doppler_factors": 24092,
                "iterations": 8,
                "converged": True,
                "initial_cost": 1000.0,
                "final_cost": 10.0,
            },
            "pos-pd": {
                "preset": "taroz-pos-pd",
                "backend": "eigen",
                "optimized_epochs": 1141,
                "valid_position_epochs": 1141,
                "valid_clock_epochs": 1141,
                "pseudorange_factors": 24092,
                "doppler_factors": 24092,
                "iterations": 8,
                "converged": True,
                "initial_cost": 1000.0,
                "final_cost": 10.0,
            },
            "pos-pdc": {
                "preset": "taroz-pos-pdc",
                "backend": "eigen",
                "optimized_epochs": 1141,
                "valid_position_epochs": 1141,
                "valid_clock_epochs": 1141,
                "pseudorange_factors": 24092,
                "doppler_factors": 24092,
                "tdcp_factors": 20856,
                "iterations": 8,
                "converged": True,
                "initial_cost": 1000.0,
                "final_cost": 10.0,
            },
            "pos-vel-pdc": {
                "preset": "taroz-pdc",
                "backend": "eigen",
                "optimized_epochs": 1141,
                "valid_position_epochs": 1141,
                "valid_velocity_epochs": 1141,
                "pseudorange_factors": 24092,
                "doppler_factors": 24092,
                "tdcp_factors": 20856,
                "iterations": 8,
                "converged": True,
                "initial_cost": 1000.0,
                "final_cost": 10.0,
            },
        }

        for mode, summary in samples.items():
            with self.subTest(mode=mode):
                spec = gnss_taroz_observable_dogfood.MODE_SPECS[mode]
                failures = gnss_taroz_observable_dogfood.verify_native_summary(
                    summary,
                    spec,
                    expected_max_epochs=0,
                    expected_max_iterations=1000,
                )

                self.assertEqual(failures, [])


if __name__ == "__main__":
    unittest.main()
