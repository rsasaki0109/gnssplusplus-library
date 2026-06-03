#!/usr/bin/env python3
"""Tests for the taroz oracle suite harness."""

from __future__ import annotations

import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"
sys.path.insert(0, str(APPS_DIR))

import gnss_taroz_oracle_suite  # noqa: E402


class TarozOracleSuiteTest(unittest.TestCase):
    def run_suite(self, args: list[str]) -> int:
        with contextlib.redirect_stdout(io.StringIO()):
            return gnss_taroz_oracle_suite.main(args)

    def touch_inputs(self, root: Path) -> tuple[Path, Path, Path, Path]:
        obs = root / "rover.obs"
        base = root / "base.obs"
        nav = root / "base.nav"
        seed_pos = root / "seed.pos"
        for path in (obs, base, nav, seed_pos):
            path.write_text("", encoding="ascii")
        return obs, base, nav, seed_pos

    def test_dry_run_writes_all_mode_plan(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_taroz_suite_test_") as temp_dir:
            temp_root = Path(temp_dir)
            out_root = temp_root / "suite"
            summary_json = temp_root / "summary.json"

            result = self.run_suite(
                [
                    "--out-root",
                    str(out_root),
                    "--summary-json",
                    str(summary_json),
                    "--native-bin-dir",
                    str(temp_root / "bin"),
                    "--dry-run",
                ]
            )

            self.assertEqual(result, 0)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["status"], "dry-run")
            self.assertEqual(payload["modes"], list(gnss_taroz_oracle_suite.MODE_ORDER))
            p_command = payload["runs"]["p"]["command"]
            self.assertIn("taroz-p-dogfood", p_command)
            self.assertIn("--fgo-bin", p_command)
            self.assertIn(str(temp_root / "bin" / "gnss_fgo"), p_command)
            pos_pdc_command = payload["runs"]["pos-pdc"]["command"]
            self.assertIn("taroz-observable-dogfood", pos_pdc_command)
            self.assertIn("--mode", pos_pdc_command)
            self.assertIn("pos-pdc", pos_pdc_command)
            self.assertIn("--native-bin", pos_pdc_command)
            self.assertIn(str(temp_root / "bin" / "gnss_pos_pdc"), pos_pdc_command)
            pc_command = payload["runs"]["pc"]["command"]
            self.assertIn("--base", pc_command)
            self.assertIn("--no-byte-compare", pc_command)
            self.assertEqual(
                payload["runs"]["d"]["matlab_dir"],
                str(ROOT_DIR / "output/dogfood/taroz_matlab_d_debug"),
            )

    def test_generated_matlab_dirs_are_scoped_under_suite_root(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_taroz_suite_test_") as temp_dir:
            temp_root = Path(temp_dir)
            out_root = temp_root / "suite"
            summary_json = temp_root / "summary.json"

            result = self.run_suite(
                [
                    "--mode",
                    "pc",
                    "--mode",
                    "pos-pdc",
                    "--out-root",
                    str(out_root),
                    "--summary-json",
                    str(summary_json),
                    "--generate-matlab-dump",
                    "--matlab-bin",
                    str(temp_root / "matlab"),
                    "--taroz-root",
                    str(temp_root / "taroz"),
                    "--taroz-example-dir",
                    str(temp_root / "taroz" / "examples"),
                    "--dry-run",
                ]
            )

            self.assertEqual(result, 0)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["modes"], ["pc", "pos-pdc"])
            pc = payload["runs"]["pc"]
            self.assertEqual(pc["matlab_dir"], str(out_root / "matlab" / "pc"))
            self.assertIn("--generate-matlab-dump", pc["command"])
            self.assertIn(str(temp_root / "matlab"), pc["command"])
            self.assertIn(str(temp_root / "taroz"), pc["command"])
            self.assertIn(str(temp_root / "taroz" / "examples"), pc["command"])
            self.assertEqual(
                payload["runs"]["pos-pdc"]["matlab_dir"],
                str(out_root / "matlab" / "pos_pdc"),
            )

    def test_failed_mode_stops_later_modes_by_default(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_taroz_suite_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs, base, nav, seed_pos = self.touch_inputs(temp_root)
            summary_json = temp_root / "summary.json"
            calls: list[list[str]] = []

            def fake_run(command: list[str]) -> int:
                calls.append(command)
                return 7

            with mock.patch.object(
                gnss_taroz_oracle_suite, "run_command", side_effect=fake_run
            ):
                result = self.run_suite(
                    [
                        "--mode",
                        "p",
                        "--mode",
                        "pd",
                        "--obs",
                        str(obs),
                        "--base",
                        str(base),
                        "--nav",
                        str(nav),
                        "--seed-pos",
                        str(seed_pos),
                        "--out-root",
                        str(temp_root / "suite"),
                        "--summary-json",
                        str(summary_json),
                    ]
                )

            self.assertEqual(result, 1)
            self.assertEqual(len(calls), 1)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["status"], "failed")
            self.assertEqual(payload["runs"]["p"]["status"], "failed")
            self.assertEqual(payload["runs"]["p"]["returncode"], 7)
            self.assertEqual(payload["runs"]["pd"]["status"], "skipped")
            self.assertIn("stopped after p failed", payload["runs"]["pd"]["skip_reason"])

    def test_keep_going_runs_later_modes_after_failure(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_taroz_suite_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs, base, nav, seed_pos = self.touch_inputs(temp_root)
            summary_json = temp_root / "summary.json"

            with mock.patch.object(
                gnss_taroz_oracle_suite, "run_command", side_effect=[5, 0]
            ) as run_command:
                result = self.run_suite(
                    [
                        "--mode",
                        "p",
                        "--mode",
                        "pd",
                        "--obs",
                        str(obs),
                        "--base",
                        str(base),
                        "--nav",
                        str(nav),
                        "--seed-pos",
                        str(seed_pos),
                        "--out-root",
                        str(temp_root / "suite"),
                        "--summary-json",
                        str(summary_json),
                        "--keep-going",
                    ]
                )

            self.assertEqual(result, 1)
            self.assertEqual(run_command.call_count, 2)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["runs"]["p"]["status"], "failed")
            self.assertEqual(payload["runs"]["pd"]["status"], "passed")


if __name__ == "__main__":
    unittest.main()
