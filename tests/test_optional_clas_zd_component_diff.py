#!/usr/bin/env python3
"""Tests for the optional CLAS ZD component CI runner."""

from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "ci" / "run_optional_clas_zd_component_diff.py"

spec = importlib.util.spec_from_file_location("run_optional_clas_zd_component_diff", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
runner = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = runner
spec.loader.exec_module(runner)


FIELDNAMES = [
    "week",
    "tow",
    "stage",
    "sat",
    "row_type",
    "freq",
    "pseudorange_rinex_code",
    "carrier_rinex_code",
    "bias_exact_identity",
    "observation_exact_identity_requested",
    "observation_exact_match",
    "observation_family_fallback",
    "code_bias_fallback",
    "prc_m",
    "code_bias_m",
]


def write_zd_component_csv(
    path: Path,
    *,
    prc_m: str = "0.1",
    bias_exact_identity: str = "",
    observation_exact_identity_requested: str = "",
    observation_exact_match: str = "",
    observation_family_fallback: str = "",
    code_bias_fallback: str = "",
) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerow(
            {
                "week": "2360",
                "tow": "172800.000",
                "stage": "accepted",
                "sat": "G01",
                "row_type": "code",
                "freq": "1",
                "pseudorange_rinex_code": "C2W",
                "carrier_rinex_code": "",
                "bias_exact_identity": bias_exact_identity,
                "observation_exact_identity_requested": observation_exact_identity_requested,
                "observation_exact_match": observation_exact_match,
                "observation_family_fallback": observation_family_fallback,
                "code_bias_fallback": code_bias_fallback,
                "prc_m": prc_m,
                "code_bias_m": "0.5",
            }
        )


class OptionalClasZdComponentDiffTest(unittest.TestCase):
    def test_build_step_plan_skips_when_inputs_are_missing(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_clas_zd_skip_") as temp_dir:
            output_dir = Path(temp_dir) / "output"

            steps = runner.build_step_plan(ROOT_DIR, output_dir, {})

            self.assertEqual([step.slug for step in steps], ["clas_zd_component_diff"])
            self.assertIsNone(steps[0].command)
            self.assertIsNotNone(steps[0].skip_reason)
            self.assertIn("CLAS ZD component input is unavailable", steps[0].skip_reason)

    def test_build_step_plan_uses_configured_inputs_and_filters(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_clas_zd_plan_") as temp_dir:
            root = Path(temp_dir)
            base_csv = root / "claslib.csv"
            candidate_csv = root / "native.csv"
            write_zd_component_csv(base_csv)
            write_zd_component_csv(candidate_csv)
            output_dir = root / "output"
            env = {
                "GNSSPP_CLAS_ZD_BASE_CSV": str(base_csv),
                "GNSSPP_CLAS_ZD_CANDIDATE_CSV": str(candidate_csv),
                "GNSSPP_CLAS_ZD_COMPONENTS": "prc_m,code_bias_m",
                "GNSSPP_CLAS_ZD_STAGE": "accepted",
                "GNSSPP_CLAS_ZD_ROW_TYPE": "code",
                "GNSSPP_CLAS_ZD_SAT": "G01",
                "GNSSPP_CLAS_ZD_FREQ": "1",
                "GNSSPP_CLAS_ZD_RINEX_CODE": "C2W",
                "GNSSPP_CLAS_ZD_DUPLICATE_POLICY": "mean",
                "GNSSPP_CLAS_ZD_THRESHOLD_M": "0.25",
                "GNSSPP_CLAS_ZD_FAIL_ON_DIFF": "1",
            }

            steps = runner.build_step_plan(ROOT_DIR, output_dir, env)

            command = steps[0].command
            self.assertIsNotNone(command)
            assert command is not None
            self.assertIn(str(SCRIPT_PATH.parent.parent / "analysis" / "clas_zd_component_diff.py"), command)
            self.assertIn(str(base_csv), command)
            self.assertIn(str(candidate_csv), command)
            self.assertIn("--component", command)
            self.assertIn("prc_m", command)
            self.assertIn("code_bias_m", command)
            self.assertIn("--stage", command)
            self.assertIn("accepted", command)
            self.assertIn("--row-type", command)
            self.assertIn("code", command)
            self.assertIn("--sat", command)
            self.assertIn("G01", command)
            self.assertIn("--freq", command)
            self.assertIn("1", command)
            self.assertIn("--rinex-code", command)
            self.assertIn("C2W", command)
            self.assertIn("--duplicate-policy", command)
            self.assertIn("mean", command)
            self.assertIn("--component-threshold-m", command)
            self.assertIn("0.25", command)
            self.assertIn("--fail-on-diff", command)
            self.assertIn(str(output_dir / "clas_zd_component_diff.json"), command)

    def test_run_step_collects_metrics_from_diff_report(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_clas_zd_exec_") as temp_dir:
            root = Path(temp_dir)
            base_csv = root / "claslib.csv"
            candidate_csv = root / "native.csv"
            write_zd_component_csv(base_csv, prc_m="0.10")
            write_zd_component_csv(candidate_csv, prc_m="0.15")
            output_dir = root / "output"
            log_dir = output_dir / "logs"
            log_dir.mkdir(parents=True)
            env = {
                "GNSSPP_CLAS_ZD_BASE_CSV": str(base_csv),
                "GNSSPP_CLAS_ZD_CANDIDATE_CSV": str(candidate_csv),
                "GNSSPP_CLAS_ZD_COMPONENTS": "prc_m",
                "GNSSPP_CLAS_ZD_STAGE": "accepted",
                "GNSSPP_CLAS_ZD_ROW_TYPE": "code",
                "GNSSPP_CLAS_ZD_SAT": "G01",
                "GNSSPP_CLAS_ZD_FREQ": "1",
                "GNSSPP_CLAS_ZD_RINEX_CODE": "C2W",
                "GNSSPP_CLAS_ZD_DUPLICATE_POLICY": "mean",
            }
            step = runner.build_step_plan(ROOT_DIR, output_dir, env)[0]

            result = runner.run_step(step, ROOT_DIR, log_dir)

            self.assertEqual(result["status"], "passed")
            self.assertEqual(result["returncode"], 0)
            metrics = result["metrics"]
            self.assertEqual(metrics["schema"], "clas_zd_component_diff.v1")
            self.assertEqual(metrics["common_rows"], 1)
            self.assertEqual(metrics["components_compared"], 1)
            self.assertEqual(metrics["sat_filter"], ["G01"])
            self.assertEqual(metrics["freq_filter"], [1])
            self.assertEqual(metrics["rinex_code_filter"], ["C2W"])
            self.assertEqual(metrics["duplicate_policy"], "mean")
            self.assertAlmostEqual(metrics["max_abs_delta"], 0.05)
            self.assertAlmostEqual(metrics["top_row_sum_abs_delta"], 0.05)
            self.assertAlmostEqual(metrics["top_row_max_abs_delta"], 0.05)

    def test_run_step_collects_identity_provenance_metrics(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_clas_zd_identity_") as temp_dir:
            root = Path(temp_dir)
            base_csv = root / "claslib.csv"
            candidate_csv = root / "native.csv"
            write_zd_component_csv(base_csv)
            write_zd_component_csv(
                candidate_csv,
                bias_exact_identity="1",
                observation_exact_identity_requested="1",
                observation_exact_match="1",
                observation_family_fallback="0",
                code_bias_fallback="0",
            )
            output_dir = root / "output"
            log_dir = output_dir / "logs"
            log_dir.mkdir(parents=True)
            env = {
                "GNSSPP_CLAS_ZD_BASE_CSV": str(base_csv),
                "GNSSPP_CLAS_ZD_CANDIDATE_CSV": str(candidate_csv),
                "GNSSPP_CLAS_ZD_STAGE": "accepted",
                "GNSSPP_CLAS_ZD_ROW_TYPE": "code",
            }
            step = runner.build_step_plan(ROOT_DIR, output_dir, env)[0]

            result = runner.run_step(step, ROOT_DIR, log_dir)

            self.assertEqual(result["status"], "passed")
            metrics = result["metrics"]
            self.assertEqual(metrics["identity_native_gps_l2w_rows"], 1)
            self.assertEqual(metrics["identity_native_gps_l2w_bias_exact_identity_rows"], 1)
            self.assertEqual(metrics["identity_native_gps_l2w_observation_exact_match_rows"], 1)
            self.assertEqual(metrics["identity_native_gps_l2w_observation_family_fallback_rows"], 0)
            self.assertEqual(metrics["identity_native_gps_l2w_code_bias_fallback_rows"], 0)

    def test_render_markdown_summary_reports_status_table_and_metrics(self) -> None:
        markdown = runner.render_markdown_summary(
            [
                {
                    "name": "CLAS ZD component diff",
                    "status": "passed",
                    "elapsed_s": 0.5,
                    "metrics": {
                        "common_rows": 12,
                        "base_only_rows": 1,
                        "candidate_only_rows": 2,
                        "components_compared": 24,
                        "max_abs_delta": 0.125,
                        "top_row_sum_abs_delta": 0.375,
                        "threshold_exceedances": 3,
                        "identity_native_gps_l2w_rows": 2,
                        "identity_native_gps_l2w_bias_exact_identity_rows": 1,
                        "identity_native_gps_l2w_observation_exact_match_rows": 1,
                        "identity_native_gps_l2w_observation_family_fallback_rows": 0,
                        "identity_native_gps_l2w_code_bias_fallback_rows": 1,
                    },
                },
                {
                    "name": "CLAS ZD component diff",
                    "status": "skipped",
                    "skip_reason": "CLAS ZD component input is unavailable.",
                },
                {
                    "name": "CLAS ZD component diff",
                    "status": "failed",
                    "log_path": "/tmp/clas.log",
                },
            ]
        )

        self.assertIn("## Optional CLAS ZD Component Diff", markdown)
        self.assertIn("`passed`: `1`", markdown)
        self.assertIn("`failed`: `1`", markdown)
        self.assertIn("`skipped`: `1`", markdown)
        self.assertIn("common rows `12`", markdown)
        self.assertIn("unmatched `1/2`", markdown)
        self.assertIn("components `24`", markdown)
        self.assertIn("max |delta| 0.125", markdown)
        self.assertIn("top row sum |delta| 0.375", markdown)
        self.assertIn("threshold exceedances `3`", markdown)
        self.assertIn("native L2W `2`", markdown)
        self.assertIn("native L2W exact bias/match `1/1`", markdown)
        self.assertIn("native L2W fallback obs/code `0/1`", markdown)
        self.assertIn("see `/tmp/clas.log`", markdown)

    def test_write_summary_declares_schema(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_clas_zd_summary_") as temp_dir:
            summary_path = Path(temp_dir) / "summary.json"
            steps = runner.build_step_plan(ROOT_DIR, Path(temp_dir) / "output", {})
            results = [
                {
                    "name": steps[0].name,
                    "slug": steps[0].slug,
                    "status": "skipped",
                    "skip_reason": steps[0].skip_reason,
                    "outputs": steps[0].outputs,
                    "summary_json": steps[0].summary_json,
                    "log_path": str(Path(temp_dir) / "clas_zd_component_diff.log"),
                }
            ]

            runner.write_summary(summary_path, steps, results)

            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["summary_schema"], runner.SUMMARY_SCHEMA)
            self.assertEqual(payload["counts"], {"passed": 0, "failed": 0, "skipped": 1})


if __name__ == "__main__":
    unittest.main()
