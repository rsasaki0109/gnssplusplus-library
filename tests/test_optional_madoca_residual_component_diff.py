#!/usr/bin/env python3
"""Tests for the optional MADOCA residual-component CI runner."""

from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "ci" / "run_optional_madoca_residual_component_diff.py"

spec = importlib.util.spec_from_file_location("run_optional_madoca_residual_component_diff", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
runner = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = runner
spec.loader.exec_module(runner)


FIELDNAMES = [
    "week",
    "tow",
    "iteration",
    "sat",
    "row_type",
    "residual_m",
    "iono_state_m",
    "primary_observation_code",
    "secondary_observation_code",
    "frequency_index",
    "ionosphere_coefficient",
]


def write_residual_csv(path: Path, *, residual_m: str = "0.1") -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerow(
            {
                "week": "2360",
                "tow": "172800.000",
                "iteration": "1",
                "sat": "G01",
                "row_type": "code",
                "residual_m": residual_m,
                "iono_state_m": "0.5",
                "primary_observation_code": "C1C",
                "secondary_observation_code": "C2W",
                "frequency_index": "0",
                "ionosphere_coefficient": "1.0",
            }
        )


class OptionalMadocaResidualComponentDiffTest(unittest.TestCase):
    def test_build_step_plan_blocks_when_inputs_are_missing(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_residual_skip_") as temp_dir:
            output_dir = Path(temp_dir) / "output"

            steps = runner.build_step_plan(ROOT_DIR, output_dir, {})

            self.assertEqual([step.slug for step in steps], ["madoca_residual_component_diff"])
            self.assertIsNone(steps[0].command)
            self.assertIsNotNone(steps[0].skip_reason)
            self.assertIn("MADOCA residual-component input is unavailable", steps[0].skip_reason)

    def test_run_step_records_blocked_infrastructure_artifacts(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_residual_blocked_") as temp_dir:
            output_dir = Path(temp_dir) / "output"
            log_dir = output_dir / "logs"
            log_dir.mkdir(parents=True)
            step = runner.build_step_plan(ROOT_DIR, output_dir, {})[0]

            result = runner.run_step(step, ROOT_DIR, log_dir)

            self.assertEqual(result["status"], "blocked_infrastructure")
            self.assertIn("MADOCA residual-component input is unavailable", result["block_reason"])
            artifacts = result["artifacts"]
            log_artifact = artifacts[0]
            self.assertEqual(log_artifact["role"], "log")
            self.assertTrue(log_artifact["required"])
            self.assertTrue(log_artifact["exists"])
            self.assertGreater(log_artifact["bytes"], 0)
            self.assertTrue(str(log_artifact["sha256"]))

    def test_build_step_plan_uses_configured_inputs_and_filters(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_residual_plan_") as temp_dir:
            root = Path(temp_dir)
            base_csv = root / "madocalib.csv"
            candidate_csv = root / "native.csv"
            write_residual_csv(base_csv)
            write_residual_csv(candidate_csv)
            output_dir = root / "output"
            env = {
                "GNSSPP_MADOCA_RESIDUAL_BASE_CSV": str(base_csv),
                "GNSSPP_MADOCA_RESIDUAL_CANDIDATE_CSV": str(candidate_csv),
                "GNSSPP_MADOCA_RESIDUAL_COMPONENTS": "residual_m,iono_state_m",
                "GNSSPP_MADOCA_RESIDUAL_ROW_TYPE": "code",
                "GNSSPP_MADOCA_RESIDUAL_ITERATION": "1",
                "GNSSPP_MADOCA_RESIDUAL_THRESHOLD": "0.25",
                "GNSSPP_MADOCA_RESIDUAL_FAIL_ON_DIFF": "1",
            }

            steps = runner.build_step_plan(ROOT_DIR, output_dir, env)

            command = steps[0].command
            self.assertIsNotNone(command)
            assert command is not None
            self.assertIn(str(SCRIPT_PATH.parent.parent / "analysis" / "madoca_residual_component_diff.py"), command)
            self.assertIn(str(base_csv), command)
            self.assertIn(str(candidate_csv), command)
            self.assertIn("--component", command)
            self.assertIn("residual_m", command)
            self.assertIn("iono_state_m", command)
            self.assertIn("--row-type", command)
            self.assertIn("code", command)
            self.assertIn("--iteration", command)
            self.assertIn("1", command)
            self.assertIn("--component-threshold", command)
            self.assertIn("0.25", command)
            self.assertIn("--fail-on-diff", command)
            self.assertIn(str(output_dir / "madoca_residual_component_diff.json"), command)

    def test_run_step_collects_metrics_from_diff_report(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_residual_exec_") as temp_dir:
            root = Path(temp_dir)
            base_csv = root / "madocalib.csv"
            candidate_csv = root / "native.csv"
            write_residual_csv(base_csv, residual_m="0.10")
            write_residual_csv(candidate_csv, residual_m="0.15")
            output_dir = root / "output"
            log_dir = output_dir / "logs"
            log_dir.mkdir(parents=True)
            env = {
                "GNSSPP_MADOCA_RESIDUAL_BASE_CSV": str(base_csv),
                "GNSSPP_MADOCA_RESIDUAL_CANDIDATE_CSV": str(candidate_csv),
                "GNSSPP_MADOCA_RESIDUAL_COMPONENTS": "residual_m",
                "GNSSPP_MADOCA_RESIDUAL_ROW_TYPE": "code",
                "GNSSPP_MADOCA_RESIDUAL_ITERATION": "1",
            }
            step = runner.build_step_plan(ROOT_DIR, output_dir, env)[0]

            result = runner.run_step(step, ROOT_DIR, log_dir)

            self.assertEqual(result["status"], "passed")
            self.assertEqual(result["returncode"], 0)
            metrics = result["metrics"]
            self.assertEqual(metrics["schema"], "madoca_residual_component_diff.v1")
            self.assertEqual(metrics["common_rows"], 1)
            self.assertEqual(metrics["components_compared"], 1)
            self.assertAlmostEqual(metrics["max_abs_delta"], 0.05)

    def test_run_step_fails_when_declared_outputs_are_missing(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_residual_missing_") as temp_dir:
            root = Path(temp_dir)
            output_dir = root / "output"
            log_dir = output_dir / "logs"
            log_dir.mkdir(parents=True)
            missing_json = output_dir / "madoca_residual_component_diff.json"
            step = runner.DiffStep(
                name="MADOCA residual-component diff",
                slug="madoca_residual_component_diff",
                command=[sys.executable, "-c", "print('no artifacts')"],
                outputs=[str(missing_json)],
                summary_json=str(output_dir / "madoca_residual_component_diff_summary.json"),
            )

            result = runner.run_step(step, ROOT_DIR, log_dir)

            self.assertEqual(result["status"], "failed")
            self.assertEqual(result["returncode"], 0)
            self.assertEqual(result["missing_outputs"], [str(missing_json)])

    def test_render_markdown_summary_reports_status_table_and_metrics(self) -> None:
        markdown = runner.render_markdown_summary(
            [
                {
                    "name": "MADOCA residual-component diff",
                    "status": "passed",
                    "elapsed_s": 0.5,
                    "metrics": {
                        "common_rows": 12,
                        "base_only_rows": 1,
                        "candidate_only_rows": 2,
                        "components_compared": 24,
                        "max_abs_delta": 0.125,
                        "threshold_exceedances": 3,
                    },
                },
                {
                    "name": "MADOCA residual-component diff",
                    "status": "blocked_infrastructure",
                    "block_reason": "MADOCA residual-component input is unavailable.",
                },
                {
                    "name": "MADOCA residual-component diff",
                    "status": "failed",
                    "log_path": "/tmp/madoca.log",
                },
                {
                    "name": "MADOCA residual-component diff",
                    "status": "failed",
                    "missing_outputs": ["/tmp/missing.json"],
                    "log_path": "/tmp/ignored.log",
                },
            ]
        )

        self.assertIn("## Optional MADOCA Residual-Component Diff", markdown)
        self.assertIn("`passed`: `1`", markdown)
        self.assertIn("`failed`: `2`", markdown)
        self.assertIn("`blocked_infrastructure`: `1`", markdown)
        self.assertIn("common rows `12`", markdown)
        self.assertIn("unmatched `1/2`", markdown)
        self.assertIn("components `24`", markdown)
        self.assertIn("max |delta| 0.125", markdown)
        self.assertIn("threshold exceedances `3`", markdown)
        self.assertIn("see `/tmp/madoca.log`", markdown)
        self.assertIn("missing `/tmp/missing.json`", markdown)

    def test_write_summary_declares_schema(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_residual_summary_") as temp_dir:
            summary_path = Path(temp_dir) / "summary.json"
            steps = runner.build_step_plan(ROOT_DIR, Path(temp_dir) / "output", {})
            results = [
                {
                    "name": steps[0].name,
                    "slug": steps[0].slug,
                    "status": "blocked_infrastructure",
                    "block_reason": steps[0].skip_reason,
                    "outputs": steps[0].outputs,
                    "summary_json": steps[0].summary_json,
                    "log_path": str(Path(temp_dir) / "madoca_residual_component_diff.log"),
                }
            ]

            runner.write_summary(summary_path, steps, results)

            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["summary_schema"], runner.SUMMARY_SCHEMA)
            self.assertEqual(payload["contract"], "optional_diff_artifact_contract.v1")
            self.assertEqual(payload["status"], "blocked_infrastructure")
            self.assertIn("missing evidence", payload["next_actions"][1])
            self.assertEqual(payload["counts"]["passed"], 0)
            self.assertEqual(payload["counts"]["failed"], 0)
            self.assertEqual(payload["counts"]["blocked_infrastructure"], 1)
            self.assertEqual(payload["counts"]["skipped"], 0)


if __name__ == "__main__":
    unittest.main()
