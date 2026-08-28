#!/usr/bin/env python3
"""Regression tests for the local-progress next-step guide."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
GNSS_CLI = ROOT_DIR / "apps" / "gnss.py"


class GnssNextTest(unittest.TestCase):
    def run_next(self, workspace: Path, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                sys.executable,
                str(GNSS_CLI),
                "next",
                "--workspace",
                str(workspace),
                *args,
            ],
            cwd=ROOT_DIR,
            check=False,
            capture_output=True,
            text=True,
        )

    def test_first_run_recommends_offline_demo(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss-next-first-") as temp_dir:
            result = self.run_next(Path(temp_dir), "--format", "json")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        payload = json.loads(result.stdout)
        self.assertEqual(payload["schema_version"], "gnss-next.v1")
        self.assertEqual(payload["stage"], "first-run")
        self.assertFalse(payload["demo"]["complete"])
        self.assertEqual(payload["recommendation"]["id"], "run-offline-demo")
        self.assertIn(
            "demo --output-dir output/self-contained-demo",
            payload["recommendation"]["command"],
        )

    def test_valid_demo_recommends_goal_selection(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss-next-complete-") as temp_dir:
            workspace = Path(temp_dir)
            summary = workspace / "output" / "self-contained-demo" / "demo_summary.json"
            summary.parent.mkdir(parents=True)
            summary.write_text(
                json.dumps(
                    {
                        "processed_epochs": 8,
                        "valid_solutions": 8,
                        "demo": {"schema_version": "self-contained-demo.v1"},
                    }
                ),
                encoding="utf-8",
            )
            result = self.run_next(workspace, "--format", "json")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        payload = json.loads(result.stdout)
        self.assertEqual(payload["stage"], "choose-goal")
        self.assertTrue(payload["demo"]["complete"])
        self.assertEqual(payload["recommendation"]["id"], "choose-goal")
        self.assertEqual(len(payload["available_goals"]), 6)

    def test_selected_goal_returns_one_actionable_command(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss-next-goal-") as temp_dir:
            workspace = Path(temp_dir)
            summary = workspace / "output" / "self-contained-demo" / "demo_summary.json"
            summary.parent.mkdir(parents=True)
            summary.write_text(
                json.dumps(
                    {
                        "processed_epochs": 8,
                        "valid_solutions": 8,
                        "demo": {"schema_version": "self-contained-demo.v1"},
                    }
                ),
                encoding="utf-8",
            )
            result = self.run_next(workspace, "--goal", "rtk", "--format", "json")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        payload = json.loads(result.stdout)
        self.assertEqual(payload["stage"], "apply-to-data")
        self.assertEqual(payload["selected_goal"], "rtk")
        self.assertEqual(payload["recommendation"]["id"], "rtk")
        self.assertIn(" solve ", payload["recommendation"]["command"])

    def test_invalid_demo_summary_does_not_count_as_complete(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss-next-invalid-") as temp_dir:
            workspace = Path(temp_dir)
            summary = workspace / "output" / "self-contained-demo" / "demo_summary.json"
            summary.parent.mkdir(parents=True)
            summary.write_text("{}\n", encoding="utf-8")
            result = self.run_next(workspace, "--format", "json")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        payload = json.loads(result.stdout)
        self.assertEqual(payload["stage"], "first-run")
        self.assertFalse(payload["demo"]["complete"])

    def test_source_checkout_uses_platform_python_launcher(self) -> None:
        result = self.run_next(ROOT_DIR, "--goal", "rtk", "--format", "json")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        payload = json.loads(result.stdout)
        expected_prefix = "py apps/gnss.py" if sys.platform == "win32" else "python3 apps/gnss.py"
        self.assertTrue(
            payload["recommendation"]["command"].startswith(expected_prefix),
            payload["recommendation"]["command"],
        )


if __name__ == "__main__":
    unittest.main()
