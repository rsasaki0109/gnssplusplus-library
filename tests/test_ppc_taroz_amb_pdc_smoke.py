#!/usr/bin/env python3
"""Tests for the PPC taroz ambiguity PDC smoke harness."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"
sys.path.insert(0, str(APPS_DIR))

import gnss_ppc_taroz_amb_pdc_smoke  # noqa: E402


class PpcTarozAmbPdcSmokeTest(unittest.TestCase):
    def make_run(self, root: Path, spec: str) -> Path:
        site, run_name = spec.split("/")
        run_dir = root / site / run_name
        run_dir.mkdir(parents=True)
        for name in ("rover.obs", "base.obs", "base.nav", "reference.csv"):
            (run_dir / name).write_text("", encoding="ascii")
        return run_dir

    def test_discovers_nested_dataset_root(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_discover_test_") as temp_dir:
            outer_root = Path(temp_dir)
            dataset_root = outer_root / "PPC-Dataset"
            self.make_run(dataset_root, "nagoya/run1")
            self.make_run(dataset_root, "tokyo/run2")

            runs = gnss_ppc_taroz_amb_pdc_smoke.discover_runs(outer_root)

            self.assertEqual([run.spec for run in runs], ["nagoya/run1", "tokyo/run2"])

    def test_dry_run_writes_planned_commands(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_smoke_dry_run_test_") as temp_dir:
            temp_root = Path(temp_dir)
            dataset_root = temp_root / "PPC-Dataset"
            self.make_run(dataset_root, "nagoya/run1")
            summary_json = temp_root / "summary.json"

            result = gnss_ppc_taroz_amb_pdc_smoke.main(
                [
                    "--dataset-root",
                    str(temp_root),
                    "--run",
                    "nagoya/run1",
                    "--summary-json",
                    str(summary_json),
                    "--fgo-bin",
                    str(temp_root / "gnss_fgo"),
                    "--spp-bin",
                    str(temp_root / "gnss_spp"),
                    "--skip-epochs",
                    "7",
                    "--generate-spp-seed",
                    "--dry-run",
                ]
            )

            self.assertEqual(result, 0)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            command = payload["planned_commands"]["nagoya/run1"]
            seed_command = payload["planned_seed_commands"]["nagoya/run1"]
            self.assertEqual(payload["status"], "dry-run")
            self.assertEqual(payload["skip_epochs"], 7)
            self.assertTrue(payload["generate_spp_seed"])
            self.assertIn("--max-epochs", seed_command)
            self.assertIn("27", seed_command)
            self.assertIn("--skip-epochs", command)
            self.assertIn("7", command)
            self.assertIn("--seed-pos", command)
            self.assertIn("--preset", command)
            self.assertIn("taroz-amb-pdc", command)
            self.assertIn("--max-epochs", command)
            self.assertIn("20", command)

    def test_native_summary_validation_flags_low_candidate_run(self) -> None:
        class Args:
            generate_spp_seed = False
            max_epochs = 20
            require_valid_min = 1
            require_dd_carrier_min = 1
            require_lambda_attempts_min = 1
            require_fix_rate_min = 0.0
            allow_not_converged = False

        failures = gnss_ppc_taroz_amb_pdc_smoke.validate_native_summary(
            Args(),
            {
                "preset": "taroz-amb-pdc",
                "backend": "eigen",
                "optimized_epochs": 20,
                "valid_solutions": 0,
                "double_difference_carrier_factors": 0,
                "lambda_ambiguity_attempts": 0,
                "fix_rate_percent": 0.0,
                "converged": True,
            },
        )

        self.assertEqual(
            failures,
            [
                "valid_solutions 0 < 1",
                "double_difference_carrier_factors 0 < 1",
                "lambda_ambiguity_attempts 0 < 1",
            ],
        )

    def test_native_summary_validation_accepts_generated_seed_match(self) -> None:
        class Args:
            generate_spp_seed = True
            max_epochs = 20
            require_valid_min = 1
            require_dd_carrier_min = 1
            require_lambda_attempts_min = 1
            require_fix_rate_min = 0.0
            allow_not_converged = False

        failures = gnss_ppc_taroz_amb_pdc_smoke.validate_native_summary(
            Args(),
            {
                "preset": "taroz-amb-pdc",
                "backend": "eigen",
                "optimized_epochs": 20,
                "seed_pos": "out/run/spp_seed.pos",
                "seed_matched_epochs": 20,
                "seed_interpolated_epochs": 0,
                "valid_solutions": 20,
                "double_difference_carrier_factors": 120,
                "lambda_ambiguity_attempts": 20,
                "fix_rate_percent": 100.0,
                "converged": True,
            },
        )

        self.assertEqual(failures, [])

    def test_native_summary_validation_flags_generated_seed_drift(self) -> None:
        class Args:
            generate_spp_seed = True
            max_epochs = 20
            require_valid_min = 1
            require_dd_carrier_min = 1
            require_lambda_attempts_min = 1
            require_fix_rate_min = 0.0
            allow_not_converged = False

        failures = gnss_ppc_taroz_amb_pdc_smoke.validate_native_summary(
            Args(),
            {
                "preset": "taroz-amb-pdc",
                "backend": "eigen",
                "optimized_epochs": 20,
                "seed_pos": "",
                "seed_matched_epochs": 19,
                "seed_interpolated_epochs": 1,
                "valid_solutions": 20,
                "double_difference_carrier_factors": 120,
                "lambda_ambiguity_attempts": 20,
                "fix_rate_percent": 100.0,
                "converged": True,
            },
        )

        self.assertEqual(
            failures,
            [
                "generated SPP seed path is missing from summary",
                "seed_matched_epochs 19 != 20",
                "seed_interpolated_epochs 1 != 0",
            ],
        )

    def test_reference_summary_keeps_no_solution_out_of_valid_error(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_reference_summary_test_") as temp_dir:
            temp_root = Path(temp_dir)
            pos_path = temp_root / "solution.pos"
            reference_csv = temp_root / "reference.csv"
            pos_path.write_text(
                "\n".join(
                    [
                        "% synthetic solution",
                        "2325 10.000000 1.0 0.0 0.0 0 0 0 3 8",
                        "2325 10.200000 1000.0 0.0 0.0 0 0 0 0 0",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            reference_csv.write_text(
                "GPS Week,GPS TOW (s),ECEF X (m),ECEF Y (m),ECEF Z (m)\n"
                "2325,10.000000,0.0,0.0,0.0\n"
                "2325,10.200000,0.0,0.0,0.0\n",
                encoding="ascii",
            )

            summary = gnss_ppc_taroz_amb_pdc_smoke.summarize_reference_errors(
                pos_path,
                reference_csv,
                0.05,
            )

            self.assertEqual(summary["matched_epochs"], 1)
            self.assertEqual(summary["p95_3d_error_m"], 1.0)
            self.assertEqual(summary["no_solution"]["matched_epochs"], 1)
            self.assertEqual(summary["all_output"]["max_3d_error_m"], 1000.0)

    def test_reference_summary_validation_flags_large_valid_p95(self) -> None:
        class Args:
            require_valid_p95_3d_max = 2.0
            require_valid_max_3d_max = 0.0
            require_fixed_p95_3d_max = 0.5

        failures = gnss_ppc_taroz_amb_pdc_smoke.validate_reference_summary(
            Args(),
            {
                "p95_3d_error_m": 3.25,
                "max_3d_error_m": 10.0,
                "fixed": {"p95_3d_error_m": 0.75},
            },
        )

        self.assertEqual(
            failures,
            [
                "valid p95_3d_error_m 3.250000 > 2.0",
                "fixed p95_3d_error_m 0.750000 > 0.5",
            ],
        )


if __name__ == "__main__":
    unittest.main()
