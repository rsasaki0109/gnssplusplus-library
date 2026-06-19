#!/usr/bin/env python3
"""Lightweight CLI user-experience smoke tests."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
GNSS_CLI = ROOT_DIR / "apps" / "gnss.py"


class CliUxTest(unittest.TestCase):
    def run_gnss(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [sys.executable, str(GNSS_CLI), *args],
            cwd=ROOT_DIR,
            check=False,
            capture_output=True,
            text=True,
        )

    def assert_no_traceback(self, result: subprocess.CompletedProcess[str]) -> None:
        combined = result.stdout + result.stderr
        self.assertNotIn("Traceback (most recent call last)", combined)
        self.assertNotIn("ModuleNotFoundError", combined)

    def test_top_level_help_keeps_primary_workflows_discoverable(self) -> None:
        result = self.run_gnss("--help")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assert_no_traceback(result)
        self.assertIn("Usage: gnss <command> [args...]", result.stdout)
        self.assertIn("Commands:", result.stdout)
        self.assertIn("Examples:", result.stdout)
        for command in (
            "doctor",
            "solve",
            "ppp",
            "clas-ppp",
            "qzss-l6-info",
            "ppc-rtk-signoff",
            "web",
        ):
            self.assertIn(f"  {command}", result.stdout)

        examples = [
            line.strip()
            for line in result.stdout.splitlines()
            if line.strip().startswith("python3 apps/gnss.py ")
        ]
        duplicates = sorted({line for line in examples if examples.count(line) > 1})
        self.assertEqual(duplicates, [])

    def test_representative_subcommand_help_uses_dispatcher_names(self) -> None:
        expectations = {
            "doctor": ("usage: gnss doctor", "--strict"),
            "qzss-l6-info": (
                "usage: gnss qzss-l6-info",
                "--compact-bias-row-materialization",
            ),
            "clas-ppp": (
                "usage: gnss clas-ppp",
                "--receiver-antenna-type",
                "--compact-code-bias-composition-policy",
            ),
            "web": ("usage: gnss web", "--artifact-manifest"),
        }

        for command, snippets in expectations.items():
            with self.subTest(command=command):
                result = self.run_gnss(command, "--help")
                self.assertEqual(result.returncode, 0, msg=result.stderr)
                self.assert_no_traceback(result)
                for snippet in snippets:
                    self.assertIn(snippet, result.stdout)

    def test_user_input_errors_are_actionable_not_tracebacks(self) -> None:
        cases = [
            (
                ("does-not-exist",),
                1,
                "Error: unknown command `does-not-exist`.",
                "Commands:",
            ),
            (
                ("qzss-l6-info",),
                2,
                "usage: gnss qzss-l6-info",
                "the following arguments are required: --input",
            ),
            (
                ("clas-ppp",),
                2,
                "usage: gnss clas-ppp",
                "the following arguments are required: --obs, --nav, --out",
            ),
        ]

        for args, expected_code, *snippets in cases:
            with self.subTest(args=args):
                result = self.run_gnss(*args)
                self.assertEqual(result.returncode, expected_code)
                self.assert_no_traceback(result)
                combined = result.stdout + result.stderr
                for snippet in snippets:
                    self.assertIn(snippet, combined)

    def test_doctor_json_stays_machine_readable_for_setup_tools(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_cli_ux_doctor_") as temp_dir:
            missing_root = Path(temp_dir) / "missing-root"
            expected_root = str(missing_root.resolve())
            result = self.run_gnss("doctor", "--root", str(missing_root), "--json")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assert_no_traceback(result)
        self.assertEqual(result.stderr, "")
        payload = json.loads(result.stdout)
        self.assertIs(payload["ok"], False)
        self.assertEqual(payload["root"], expected_root)
        self.assertIsInstance(payload["checks"], list)
        self.assertGreaterEqual(len(payload["checks"]), 3)
        self.assertIn("next_commands", payload)


if __name__ == "__main__":
    unittest.main()
