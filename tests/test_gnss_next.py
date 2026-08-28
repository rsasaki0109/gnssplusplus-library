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
        self.assertEqual(payload["schema_version"], "gnss-next.v2")
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
            for relative_path in (
                "data/short_baseline/TSK200JPN_R_20240010000_01D_30S_MO.rnx",
                "data/short_baseline/TSKB00JPN_R_20240010000_01D_30S_MO.rnx",
                "data/short_baseline/BRDC00IGS_R_20240010000_01D_MN.rnx",
            ):
                input_path = workspace / relative_path
                input_path.parent.mkdir(parents=True, exist_ok=True)
                input_path.touch()
            result = self.run_next(workspace, "--goal", "rtk", "--format", "json")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        payload = json.loads(result.stdout)
        self.assertEqual(payload["stage"], "apply-to-data")
        self.assertEqual(payload["selected_goal"], "rtk")
        self.assertEqual(payload["recommendation"]["id"], "rtk")
        self.assertTrue(payload["recommendation"]["inputs_ready"])
        self.assertIn(" solve ", payload["recommendation"]["command"])

    def test_missing_standard_inputs_recommends_preparation(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss-next-prepare-") as temp_dir:
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
        self.assertEqual(payload["recommendation"]["id"], "prepare-rtk-inputs")
        self.assertFalse(payload["recommendation"]["inputs_ready"])
        self.assertEqual(len(payload["recommendation"]["missing_inputs"]), 3)
        self.assertIn("help solve", payload["recommendation"]["command"])

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

    def test_completed_rtk_result_advances_to_inspection(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss-next-inspect-") as temp_dir:
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
            result_path = workspace / "output" / "rtk_solution.pos"
            result_path.write_text(
                "% header\n2024/01/01 00:00:00.000 35.0 139.0 10.0 1 12\n",
                encoding="utf-8",
            )
            result = self.run_next(workspace, "--goal", "rtk", "--format", "json")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        payload = json.loads(result.stdout)
        self.assertEqual(payload["stage"], "inspect-result")
        self.assertEqual(payload["detected_goal"], "rtk")
        self.assertEqual(payload["recommendation"]["epochs"], 1)
        self.assertIn(
            "pos2kml output/rtk_solution.pos output/rtk_solution.kml --status all",
            payload["recommendation"]["command"],
        )

    def test_comment_only_result_does_not_count_as_complete(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss-next-empty-result-") as temp_dir:
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
            (workspace / "output" / "ppp_solution.pos").write_text(
                "% no valid solutions\n",
                encoding="utf-8",
            )
            result = self.run_next(workspace, "--goal", "ppp", "--format", "json")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        payload = json.loads(result.stdout)
        self.assertEqual(payload["stage"], "apply-to-data")
        self.assertIsNone(payload["detected_goal"])

    def test_existing_kml_is_opened_instead_of_regenerated(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss-next-open-kml-") as temp_dir:
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
            (workspace / "output" / "spp_solution.pos").write_text(
                "2024/01/01 00:00:00.000 35.0 139.0 10.0 5 8\n",
                encoding="utf-8",
            )
            (workspace / "output" / "spp_solution.kml").write_text(
                "<kml></kml>\n",
                encoding="utf-8",
            )
            result = self.run_next(workspace, "--goal", "spp", "--format", "json")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        payload = json.loads(result.stdout)
        self.assertEqual(payload["stage"], "inspect-result")
        self.assertEqual(payload["recommendation"]["id"], "open-spp-result")
        self.assertEqual(payload["recommendation"]["kml"], "output/spp_solution.kml")
        self.assertNotIn("pos2kml", payload["recommendation"]["command"])

    def test_source_checkout_uses_platform_python_launcher(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss-next-source-") as temp_dir:
            workspace = Path(temp_dir)
            dispatcher = workspace / "apps" / "gnss.py"
            dispatcher.parent.mkdir(parents=True)
            dispatcher.touch()
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
        expected_prefix = "py apps/gnss.py" if sys.platform == "win32" else "python3 apps/gnss.py"
        self.assertTrue(
            payload["recommendation"]["command"].startswith(expected_prefix),
            payload["recommendation"]["command"],
        )


if __name__ == "__main__":
    unittest.main()
