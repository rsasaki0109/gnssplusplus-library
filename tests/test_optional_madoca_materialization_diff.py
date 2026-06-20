#!/usr/bin/env python3
"""Tests for the optional MADOCA materialization CI runner."""

from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "ci" / "run_optional_madoca_materialization_diff.py"

spec = importlib.util.spec_from_file_location("run_optional_madoca_materialization_diff", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
runner = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = runner
spec.loader.exec_module(runner)


FIELDNAMES = [
    "schema_version",
    "sat",
    "system",
    "prn",
    "week",
    "tow",
    "orbit_frame",
    "orbit_valid",
    "clock_valid",
    "code_bias_valid",
    "phase_bias_valid",
    "orbit_week",
    "orbit_tow",
    "clock_week",
    "clock_tow",
    "iode",
    "ssr_orbit_iod",
    "ssr_clock_iod",
    "orbit_radial_m",
    "orbit_along_m",
    "orbit_cross_m",
    "clock_m",
    "code_bias_count",
    "code_biases_m",
    "phase_bias_count",
    "phase_biases_m",
    "phase_bias_discnt",
]


def materialization_row(*, clock_m: str = "0.200", iode: str = "7") -> dict[str, str]:
    return {
        "schema_version": "madoca_materialization_snapshot.v1",
        "sat": "G14",
        "system": "GPS",
        "prn": "14",
        "week": "2299",
        "tow": "123.500",
        "orbit_frame": "rac",
        "orbit_valid": "1",
        "clock_valid": "1",
        "code_bias_valid": "1",
        "phase_bias_valid": "1",
        "orbit_week": "2299",
        "orbit_tow": "120.000",
        "clock_week": "2299",
        "clock_tow": "123.000",
        "iode": iode,
        "ssr_orbit_iod": "11",
        "ssr_clock_iod": "12",
        "orbit_radial_m": "0.100",
        "orbit_along_m": "-0.300",
        "orbit_cross_m": "0.400",
        "clock_m": clock_m,
        "code_bias_count": "1",
        "code_biases_m": "3:0.250",
        "phase_bias_count": "1",
        "phase_biases_m": "3:-0.125",
        "phase_bias_discnt": "3:0",
    }


def write_materialization_csv(path: Path, *, clock_m: str = "0.200", iode: str = "7") -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerow(materialization_row(clock_m=clock_m, iode=iode))


class OptionalMadocaMaterializationDiffTest(unittest.TestCase):
    def test_build_step_plan_blocks_when_inputs_are_missing(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_materialization_skip_") as temp_dir:
            output_dir = Path(temp_dir) / "output"

            steps = runner.build_step_plan(ROOT_DIR, output_dir, {})

            self.assertEqual([step.slug for step in steps], ["madoca_materialization_diff"])
            self.assertIsNone(steps[0].command)
            self.assertIsNotNone(steps[0].skip_reason)
            self.assertIn("MADOCA materialization input is unavailable", steps[0].skip_reason)

    def test_run_step_records_blocked_infrastructure_artifacts(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_materialization_blocked_") as temp_dir:
            output_dir = Path(temp_dir) / "output"
            log_dir = output_dir / "logs"
            log_dir.mkdir(parents=True)
            step = runner.build_step_plan(ROOT_DIR, output_dir, {})[0]

            result = runner.run_step(step, ROOT_DIR, log_dir)

            self.assertEqual(result["status"], "blocked_infrastructure")
            self.assertIn("MADOCA materialization input is unavailable", result["block_reason"])
            artifacts = result["artifacts"]
            log_artifact = artifacts[0]
            self.assertEqual(log_artifact["role"], "log")
            self.assertTrue(log_artifact["required"])
            self.assertTrue(log_artifact["exists"])
            self.assertGreater(log_artifact["bytes"], 0)
            self.assertTrue(str(log_artifact["sha256"]))

    def test_build_step_plan_uses_configured_inputs_and_threshold(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_materialization_plan_") as temp_dir:
            root = Path(temp_dir)
            base_csv = root / "madocalib.csv"
            candidate_csv = root / "native.csv"
            write_materialization_csv(base_csv)
            write_materialization_csv(candidate_csv)
            output_dir = root / "output"
            env = {
                "GNSSPP_MADOCA_MATERIALIZATION_BASE_CSV": str(base_csv),
                "GNSSPP_MADOCA_MATERIALIZATION_CANDIDATE_CSV": str(candidate_csv),
                "GNSSPP_MADOCA_MATERIALIZATION_COMPONENTS": "clock_m",
                "GNSSPP_MADOCA_MATERIALIZATION_THRESHOLD": "0.25",
                "GNSSPP_MADOCA_MATERIALIZATION_FAIL_ON_DIFF": "1",
            }

            steps = runner.build_step_plan(ROOT_DIR, output_dir, env)

            command = steps[0].command
            self.assertIsNotNone(command)
            assert command is not None
            self.assertIn(str(SCRIPT_PATH.parent.parent / "analysis" / "madoca_materialization_diff.py"), command)
            self.assertIn(str(base_csv), command)
            self.assertIn(str(candidate_csv), command)
            self.assertIn("--numeric-threshold", command)
            self.assertIn("0.25", command)
            self.assertIn("--fail-on-diff", command)
            self.assertNotIn("--component", command)
            self.assertIn(str(output_dir / "madoca_materialization_diff.json"), command)

    def test_run_step_collects_materialization_metrics(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_materialization_exec_") as temp_dir:
            root = Path(temp_dir)
            base_csv = root / "madocalib.csv"
            candidate_csv = root / "native.csv"
            write_materialization_csv(base_csv, clock_m="0.200")
            write_materialization_csv(candidate_csv, clock_m="0.275")
            output_dir = root / "output"
            log_dir = output_dir / "logs"
            log_dir.mkdir(parents=True)
            env = {
                "GNSSPP_MADOCA_MATERIALIZATION_BASE_CSV": str(base_csv),
                "GNSSPP_MADOCA_MATERIALIZATION_CANDIDATE_CSV": str(candidate_csv),
            }
            step = runner.build_step_plan(ROOT_DIR, output_dir, env)[0]

            result = runner.run_step(step, ROOT_DIR, log_dir)

            self.assertEqual(result["status"], "passed")
            self.assertEqual(result["returncode"], 0)
            metrics = result["metrics"]
            self.assertEqual(metrics["schema"], "madoca_materialization_diff.v1")
            self.assertEqual(metrics["common_rows"], 1)
            self.assertEqual(metrics["components_compared"], 6)
            self.assertEqual(metrics["threshold_exceedances"], 0)
            self.assertEqual(metrics["discrete_mismatches"], 0)
            self.assertAlmostEqual(metrics["max_abs_delta"], 0.075)

    def test_run_step_reports_discrete_mismatches(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_materialization_discrete_") as temp_dir:
            root = Path(temp_dir)
            base_csv = root / "madocalib.csv"
            candidate_csv = root / "native.csv"
            write_materialization_csv(base_csv, iode="7")
            write_materialization_csv(candidate_csv, iode="8")
            output_dir = root / "output"
            log_dir = output_dir / "logs"
            log_dir.mkdir(parents=True)
            env = {
                "GNSSPP_MADOCA_MATERIALIZATION_BASE_CSV": str(base_csv),
                "GNSSPP_MADOCA_MATERIALIZATION_CANDIDATE_CSV": str(candidate_csv),
            }
            step = runner.build_step_plan(ROOT_DIR, output_dir, env)[0]

            result = runner.run_step(step, ROOT_DIR, log_dir)

            self.assertEqual(result["status"], "passed")
            self.assertEqual(result["metrics"]["discrete_mismatches"], 1)

    def test_run_step_fails_when_declared_outputs_are_missing(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_materialization_missing_") as temp_dir:
            root = Path(temp_dir)
            output_dir = root / "output"
            log_dir = output_dir / "logs"
            log_dir.mkdir(parents=True)
            missing_json = output_dir / "madoca_materialization_diff.json"
            step = runner.DiffStep(
                name="MADOCA materialization diff",
                slug="madoca_materialization_diff",
                command=[sys.executable, "-c", "print('no artifacts')"],
                outputs=[str(missing_json)],
                summary_json=str(output_dir / "madoca_materialization_diff_summary.json"),
            )

            result = runner.run_step(step, ROOT_DIR, log_dir)

            self.assertEqual(result["status"], "failed")
            self.assertEqual(result["returncode"], 0)
            self.assertEqual(result["missing_outputs"], [str(missing_json)])

    def test_render_markdown_summary_reports_materialization_metrics(self) -> None:
        markdown = runner.render_markdown_summary(
            [
                {
                    "name": "MADOCA materialization diff",
                    "status": "passed",
                    "elapsed_s": 0.5,
                    "metrics": {
                        "common_rows": 12,
                        "base_only_rows": 1,
                        "candidate_only_rows": 2,
                        "components_compared": 72,
                        "max_abs_delta": 0.125,
                        "threshold_exceedances": 3,
                        "discrete_mismatches": 4,
                        "base_duplicate_keys": 0,
                        "candidate_duplicate_keys": 1,
                    },
                },
                {
                    "name": "MADOCA materialization diff",
                    "status": "blocked_infrastructure",
                    "block_reason": "MADOCA materialization input is unavailable.",
                },
            ]
        )

        self.assertIn("## Optional MADOCA Materialization Diff", markdown)
        self.assertIn("`passed`: `1`", markdown)
        self.assertIn("`blocked_infrastructure`: `1`", markdown)
        self.assertIn("common rows `12`", markdown)
        self.assertIn("unmatched `1/2`", markdown)
        self.assertIn("components `72`", markdown)
        self.assertIn("max |delta| 0.125", markdown)
        self.assertIn("threshold exceedances `3`", markdown)
        self.assertIn("discrete mismatches `4`", markdown)
        self.assertIn("duplicate keys `0/1`", markdown)

    def test_write_summary_declares_schema(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_materialization_summary_") as temp_dir:
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
                    "log_path": str(Path(temp_dir) / "madoca_materialization_diff.log"),
                }
            ]

            runner.write_summary(summary_path, steps, results)

            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["summary_schema"], runner.SUMMARY_SCHEMA)
            self.assertEqual(payload["contract"], "optional_diff_artifact_contract.v1")
            self.assertEqual(payload["status"], "blocked_infrastructure")
            self.assertIn("missing evidence", payload["next_actions"][1])
            self.assertEqual(payload["counts"]["blocked_infrastructure"], 1)


if __name__ == "__main__":
    unittest.main()
