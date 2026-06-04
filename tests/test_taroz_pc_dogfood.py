#!/usr/bin/env python3
"""Tests for the taroz PC dogfood harness."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"
sys.path.insert(0, str(APPS_DIR))

import gnss_taroz_pc_dogfood  # noqa: E402


class TarozPcDogfoodTest(unittest.TestCase):
    def touch_inputs(self, root: Path) -> tuple[Path, Path, Path, Path]:
        obs = root / "rover.obs"
        base = root / "base.obs"
        nav = root / "base.nav"
        seed_pos = root / "seed.pos"
        for path in (obs, base, nav, seed_pos):
            path.write_text("", encoding="ascii")
        return obs, base, nav, seed_pos

    def test_dry_run_writes_reproducible_plan(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_taroz_pc_dogfood_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs, base, nav, seed_pos = self.touch_inputs(temp_root)
            summary_json = temp_root / "summary.json"
            out_dir = temp_root / "out"

            result = gnss_taroz_pc_dogfood.main(
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
            self.assertIn("taroz-pc", command)
            self.assertIn("--backend", command)
            self.assertIn("gtsam-pc", command)
            self.assertIn("--lambda-debug-csv", command)
            self.assertNotIn("--no-spp-seed", command)
            self.assertFalse(payload["expected"]["use_spp_seed"])

    def test_byte_compare_reports_matching_artifacts(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_taroz_pc_compare_test_") as temp_dir:
            temp_root = Path(temp_dir)
            out_dir = temp_root / "out"
            baseline_cpp = temp_root / "baseline_cpp"
            baseline_lambda = temp_root / "baseline_lambda"
            for directory in (out_dir, baseline_cpp, baseline_lambda):
                directory.mkdir()

            for name in (
                gnss_taroz_pc_dogfood.POS_NAME,
                gnss_taroz_pc_dogfood.EPOCH_DEBUG_NAME,
                gnss_taroz_pc_dogfood.FACTOR_DEBUG_NAME,
            ):
                (baseline_cpp / name).write_text("same\n", encoding="ascii")
                (out_dir / name).write_text("same\n", encoding="ascii")
            (baseline_lambda / gnss_taroz_pc_dogfood.LAMBDA_DEBUG_NAME).write_text(
                "same\n",
                encoding="ascii",
            )
            (out_dir / gnss_taroz_pc_dogfood.LAMBDA_DEBUG_NAME).write_text(
                "same\n",
                encoding="ascii",
            )

            results = gnss_taroz_pc_dogfood.compare_file_pairs(
                out_dir,
                baseline_cpp,
                baseline_lambda,
            )

            self.assertEqual({result["status"] for result in results.values()}, {"same"})
            self.assertEqual(gnss_taroz_pc_dogfood.byte_comparison_failures(results), [])

    def test_byte_compare_failures_flag_differences(self) -> None:
        failures = gnss_taroz_pc_dogfood.byte_comparison_failures(
            {
                "pos": {"status": "same"},
                "factor_debug": {"status": "different"},
                "lambda_debug": {"status": "missing"},
            }
        )

        self.assertEqual(failures, ["factor_debug differs from baseline"])

    def test_summary_compare_allows_generated_metadata_additions(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_taroz_pc_summary_test_") as temp_dir:
            temp_root = Path(temp_dir)
            baseline_summary = temp_root / "baseline.json"
            generated_summary = temp_root / "generated.json"
            baseline_summary.write_text(
                json.dumps({"preset": "taroz-pc", "iterations": 23}),
                encoding="ascii",
            )
            generated_summary.write_text(
                json.dumps(
                    {
                        "preset": "taroz-pc",
                        "iterations": 23,
                        "pseudorange_elevation_sigma_power": 1.0,
                    }
                ),
                encoding="ascii",
            )

            result = gnss_taroz_pc_dogfood.compare_summaries(
                generated_summary,
                baseline_summary,
            )

            self.assertEqual(result["status"], "same")
            self.assertEqual(result["diffs"], [])
            self.assertIn(
                "pseudorange_elevation_sigma_power",
                result["extra_generated_keys"],
            )

    def test_native_summary_requires_taroz_seed_behavior(self) -> None:
        failures = gnss_taroz_pc_dogfood.verify_native_summary(
            {"preset": "taroz-pc", "backend": "gtsam-pc", "use_spp_seed": True}
        )

        self.assertEqual(failures, ["taroz-pc must report use_spp_seed=false"])


if __name__ == "__main__":
    unittest.main()
