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
    "trop_correction_m",
    "iono_l1_m",
    "stec_tecu",
    "iono_scaled_m",
]


def write_zd_component_csv(
    path: Path,
    *,
    sat: str = "G01",
    tow: str = "172800.000",
    row_type: str = "code",
    pseudorange_code: str = "C2W",
    prc_m: str = "0.1",
    bias_exact_identity: str = "",
    observation_exact_identity_requested: str = "",
    observation_exact_match: str = "",
    observation_family_fallback: str = "",
    code_bias_fallback: str = "",
    code_bias_m: str = "0.5",
    trop_correction_m: str = "2.7",
    iono_l1_m: str = "-0.3",
    stec_tecu: str = "-1.9",
    iono_scaled_m: str = "-0.5",
) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerow(
            {
                "week": "2360",
                "tow": tow,
                "stage": "accepted",
                "sat": sat,
                "row_type": row_type,
                "freq": "1",
                "pseudorange_rinex_code": pseudorange_code,
                "carrier_rinex_code": "",
                "bias_exact_identity": bias_exact_identity,
                "observation_exact_identity_requested": observation_exact_identity_requested,
                "observation_exact_match": observation_exact_match,
                "observation_family_fallback": observation_family_fallback,
                "code_bias_fallback": code_bias_fallback,
                "prc_m": prc_m,
                "code_bias_m": code_bias_m,
                "trop_correction_m": trop_correction_m,
                "iono_l1_m": iono_l1_m,
                "stec_tecu": stec_tecu,
                "iono_scaled_m": iono_scaled_m,
            }
        )


class OptionalClasZdComponentDiffTest(unittest.TestCase):
    def test_build_step_plan_blocks_when_inputs_are_missing(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_clas_zd_skip_") as temp_dir:
            output_dir = Path(temp_dir) / "output"

            steps = runner.build_step_plan(ROOT_DIR, output_dir, {})

            self.assertEqual([step.slug for step in steps], ["clas_zd_component_diff"])
            self.assertIsNone(steps[0].command)
            self.assertIsNotNone(steps[0].skip_reason)
            self.assertIn("CLAS ZD component input is unavailable", steps[0].skip_reason)

    def test_run_step_records_blocked_infrastructure_artifacts(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_clas_zd_blocked_") as temp_dir:
            output_dir = Path(temp_dir) / "output"
            log_dir = output_dir / "logs"
            log_dir.mkdir(parents=True)
            step = runner.build_step_plan(ROOT_DIR, output_dir, {})[0]

            result = runner.run_step(step, ROOT_DIR, log_dir)

            self.assertEqual(result["status"], "blocked_infrastructure")
            self.assertIn("CLAS ZD component input is unavailable", result["block_reason"])
            artifacts = result["artifacts"]
            log_artifact = artifacts[0]
            self.assertEqual(log_artifact["role"], "log")
            self.assertTrue(log_artifact["required"])
            self.assertTrue(log_artifact["exists"])
            self.assertGreater(log_artifact["bytes"], 0)
            self.assertTrue(str(log_artifact["sha256"]))

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
            self.assertEqual(len(steps[0].pre_commands), 2)
            self.assertIn(
                str(SCRIPT_PATH.parent.parent / "analysis" / "clas_zd_component_summary.py"),
                steps[0].pre_commands[0],
            )
            self.assertIn(str(output_dir / "clas_zd_component_diff_claslib_snapshot_summary.json"), steps[0].outputs)
            self.assertIn(str(output_dir / "clas_zd_component_diff_native_snapshot_summary.json"), steps[0].outputs)

    def test_build_step_plan_falls_back_to_a4b_native_candidate(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_clas_zd_native_fallback_") as temp_dir:
            root = Path(temp_dir)
            base_csv = root / "claslib.csv"
            native_csv = root / "output" / "clas_a4b_native_selfdiff" / "native_code_dump.csv"
            write_zd_component_csv(base_csv)
            native_csv.parent.mkdir(parents=True)
            write_zd_component_csv(native_csv)
            output_dir = root / "output"
            env = {
                "GNSSPP_CLAS_ZD_BASE_CSV": str(base_csv),
                "GNSSPP_CLAS_ZD_NATIVE_CSV": str(native_csv),
            }

            steps = runner.build_step_plan(ROOT_DIR, output_dir, env)

            command = steps[0].command
            self.assertIsNotNone(command)
            assert command is not None
            self.assertIn(str(base_csv), command)
            self.assertIn(str(native_csv), command)
            self.assertNotIn("CLAS ZD component input is unavailable", steps[0].skip_reason or "")

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
            self.assertEqual(set(metrics["per_component_max_abs_delta"]), {"prc_m"})
            self.assertAlmostEqual(metrics["per_component_max_abs_delta"]["prc_m"], 0.05)
            self.assertEqual(metrics["top_delta_component"], "prc_m")
            self.assertAlmostEqual(metrics["top_delta_delta"], 0.05)
            self.assertAlmostEqual(metrics["top_delta_abs_delta"], 0.05)
            self.assertEqual(metrics["top_delta_key"], "2360/172800.0/code/G01/1/C2W")
            self.assertEqual(metrics["top_delta_sat"], "G01")
            self.assertEqual(metrics["top_delta_rinex_code"], "C2W")
            self.assertAlmostEqual(metrics["top_row_sum_abs_delta"], 0.05)
            self.assertAlmostEqual(metrics["top_row_max_abs_delta"], 0.05)
            self.assertEqual(metrics["top_row_sat"], "G01")
            self.assertEqual(metrics["top_row_rinex_code"], "C2W")
            self.assertEqual(metrics["top_row_dominant_component"], "prc_m")
            self.assertAlmostEqual(metrics["top_row_dominant_delta"], 0.05)
            self.assertAlmostEqual(metrics["top_row_dominant_abs_delta"], 0.05)
            self.assertEqual(metrics["claslib_snapshot_schema"], "clas_zd_component_summary.v2")
            self.assertEqual(metrics["native_snapshot_schema"], "clas_zd_component_summary.v2")
            self.assertEqual(metrics["claslib_snapshot_status"], "passed")
            self.assertEqual(metrics["native_snapshot_status"], "passed")
            self.assertEqual(metrics["claslib_snapshot_rows"], 1)
            self.assertEqual(metrics["native_snapshot_rows"], 1)
            self.assertEqual(metrics["claslib_snapshot_duplicate_groups"], 0)
            self.assertEqual(metrics["native_snapshot_duplicate_groups"], 0)
            artifact_paths = {artifact["path"] for artifact in result["artifacts"]}
            self.assertIn(str(output_dir / "clas_zd_component_diff_claslib_snapshot_summary.json"), artifact_paths)
            self.assertIn(str(output_dir / "clas_zd_component_diff_native_snapshot_summary.json"), artifact_paths)

    def test_load_diff_metrics_keeps_top_row_missing_component_context(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_clas_zd_metrics_") as temp_dir:
            report_path = Path(temp_dir) / "clas_zd_component_diff.json"
            report_path.write_text(
                json.dumps(
                    {
                        "schema": "clas_zd_component_diff.v1",
                        "top_row_component_breakdowns": [
                            {
                                "week": 2360,
                                "tow": 172800.0,
                                "row_type": "code",
                                "sat": "G14",
                                "freq": 1,
                                "rinex_code": "C2W",
                                "sum_abs_delta_m": 0.25,
                                "max_abs_delta_m": 0.2,
                                "components": [
                                    {
                                        "component": "prc_m",
                                        "delta_m": 0.2,
                                        "abs_delta_m": 0.2,
                                    }
                                ],
                                "missing_components": [
                                    {
                                        "component": "atmos_ref_tow",
                                        "base_present": False,
                                        "candidate_present": True,
                                        "base_value": None,
                                        "candidate_value": 230430.0,
                                    }
                                ],
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )

            metrics = runner.runner.load_diff_metrics(runner.CONFIG, report_path)

            self.assertEqual(metrics["top_row_key"], "2360/172800.0/code/G14/1/C2W")
            self.assertEqual(
                metrics["top_row_missing_components"],
                [
                    {
                        "component": "atmos_ref_tow",
                        "base_present": False,
                        "candidate_present": True,
                        "base_value": None,
                        "candidate_value": 230430.0,
                    }
                ],
            )

    def test_load_diff_metrics_keeps_top_row_highlight_component_values(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_clas_zd_highlight_") as temp_dir:
            report_path = Path(temp_dir) / "clas_zd_component_diff.json"
            report_path.write_text(
                json.dumps(
                    {
                        "schema": "clas_zd_component_diff.v1",
                        "top_row_component_breakdowns": [
                            {
                                "week": 2360,
                                "tow": 172800.0,
                                "row_type": "code",
                                "sat": "G14",
                                "freq": 1,
                                "rinex_code": "C2W",
                                "sum_abs_delta_m": 0.25,
                                "max_abs_delta_m": 0.2,
                                "components": [
                                    {
                                        "component": "iono_scaled_m",
                                        "base_value_m": -0.625,
                                        "candidate_value_m": -0.677,
                                        "delta_m": -0.052,
                                        "abs_delta_m": 0.052,
                                    },
                                    {
                                        "component": "stec_tecu",
                                        "base_value_m": -2.3,
                                        "candidate_value_m": -2.49,
                                        "delta_m": -0.19,
                                        "abs_delta_m": 0.19,
                                    },
                                    {
                                        "component": "unhighlighted_m",
                                        "base_value_m": 1.0,
                                        "candidate_value_m": 2.0,
                                        "delta_m": 1.0,
                                        "abs_delta_m": 1.0,
                                    },
                                ],
                                "missing_components": [],
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )

            metrics = runner.runner.load_diff_metrics(runner.CONFIG, report_path)

            self.assertEqual(
                metrics["top_row_highlight_components"],
                [
                    {
                        "component": "iono_scaled_m",
                        "base_value": -0.625,
                        "candidate_value": -0.677,
                        "delta": -0.052,
                        "abs_delta": 0.052,
                    },
                    {
                        "component": "stec_tecu",
                        "base_value": -2.3,
                        "candidate_value": -2.49,
                        "delta": -0.19,
                        "abs_delta": 0.19,
                    },
                ],
            )

    def test_run_step_fails_when_snapshot_summary_fails(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_clas_zd_input_summary_") as temp_dir:
            root = Path(temp_dir)
            base_csv = root / "claslib.csv"
            candidate_csv = root / "native.csv"
            write_zd_component_csv(base_csv)
            write_zd_component_csv(candidate_csv, sat="", pseudorange_code="")
            output_dir = root / "output"
            log_dir = output_dir / "logs"
            log_dir.mkdir(parents=True)
            env = {
                "GNSSPP_CLAS_ZD_BASE_CSV": str(base_csv),
                "GNSSPP_CLAS_ZD_CANDIDATE_CSV": str(candidate_csv),
            }
            step = runner.build_step_plan(ROOT_DIR, output_dir, env)[0]

            result = runner.run_step(step, ROOT_DIR, log_dir)

            self.assertEqual(result["status"], "failed")
            self.assertNotEqual(result["returncode"], 0)
            self.assertIn("clas_zd_component_summary.py", " ".join(result["failed_command"]))
            metrics = result["metrics"]
            self.assertEqual(metrics["claslib_snapshot_status"], "passed")
            self.assertEqual(metrics["native_snapshot_status"], "failed")
            self.assertEqual(metrics["native_snapshot_issue_counts"]["missing_satellite"], 1)

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
                        "per_component_max_abs_delta": {
                            "iono_l1_from_stec_m": 0.031,
                            "iono_l1_stec_closure_residual_m": 1e-15,
                            "iono_scaled_closure_residual_m": 2e-15,
                            "prc_closure_residual_m": 3e-15,
                        },
                        "missing_component_counts": [
                            {"component": "atmos_ref_tow", "rows": 12},
                            {"component": "clock_ref_tow", "rows": 12},
                            {"component": "atmos_clock_gap_s", "rows": 12},
                        ],
                        "top_delta_component": "prc_m",
                        "top_delta_abs_delta": 0.25,
                        "top_row_sum_abs_delta": 0.375,
                        "top_row_dominant_component": "prc_m",
                        "top_row_dominant_abs_delta": 0.25,
                        "top_row_highlight_components": [
                            {
                                "component": "stec_tecu",
                                "base_value": -2.3,
                                "candidate_value": -2.49,
                                "delta": -0.19,
                                "abs_delta": 0.19,
                            },
                            {
                                "component": "iono_scaled_m",
                                "base_value": -0.625,
                                "candidate_value": -0.677,
                                "delta": -0.052,
                                "abs_delta": 0.052,
                            },
                        ],
                        "top_row_missing_components": [
                            {
                                "component": "atmos_ref_tow",
                                "base_present": False,
                                "candidate_present": True,
                                "base_value": None,
                                "candidate_value": 230430.0,
                            },
                            {
                                "component": "clock_ref_tow",
                                "base_present": False,
                                "candidate_present": True,
                                "base_value": None,
                                "candidate_value": 230420.0,
                            },
                            {
                                "component": "atmos_clock_gap_s",
                                "base_present": False,
                                "candidate_present": True,
                                "base_value": None,
                                "candidate_value": 10.0,
                            },
                        ],
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
                    "status": "blocked_infrastructure",
                    "block_reason": "CLAS ZD component input is unavailable.",
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
        self.assertIn("`blocked_infrastructure`: `1`", markdown)
        self.assertIn("common rows `12`", markdown)
        self.assertIn("unmatched `1/2`", markdown)
        self.assertIn("components `24`", markdown)
        self.assertIn("max |delta| 0.125", markdown)
        self.assertIn("`iono_l1_from_stec_m` |delta| 0.031", markdown)
        self.assertIn("`iono_l1_stec_closure_residual_m` |delta| 1e-15", markdown)
        self.assertIn("`iono_scaled_closure_residual_m` |delta| 2e-15", markdown)
        self.assertIn("`prc_closure_residual_m` |delta| 3e-15", markdown)
        self.assertIn("`atmos_ref_tow` missing `12`", markdown)
        self.assertIn("`clock_ref_tow` missing `12`", markdown)
        self.assertIn("`atmos_clock_gap_s` missing `12`", markdown)
        self.assertIn("top row `atmos_ref_tow` native 230430", markdown)
        self.assertIn("top row `clock_ref_tow` native 230420", markdown)
        self.assertIn("top row `atmos_clock_gap_s` native 10", markdown)
        self.assertIn("top row `stec_tecu` claslib -2.3/native -2.49/delta -0.19", markdown)
        self.assertIn(
            "top row `iono_scaled_m` claslib -0.625/native -0.677/delta -0.052",
            markdown,
        )
        self.assertIn("top delta `prc_m` |delta| 0.25", markdown)
        self.assertIn("top row sum |delta| 0.375", markdown)
        self.assertIn("top component `prc_m` |delta| 0.25", markdown)
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
                    "status": "blocked_infrastructure",
                    "block_reason": steps[0].skip_reason,
                    "outputs": steps[0].outputs,
                    "summary_json": steps[0].summary_json,
                    "log_path": str(Path(temp_dir) / "clas_zd_component_diff.log"),
                }
            ]

            runner.write_summary(summary_path, steps, results)

            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["summary_schema"], runner.SUMMARY_SCHEMA)
            self.assertEqual(payload["summary_schema"], "ci_optional_clas_zd_component_diff.v12")
            self.assertEqual(payload["contract"], "optional_diff_artifact_contract.v1")
            self.assertEqual(payload["status"], "blocked_infrastructure")
            self.assertIn("missing evidence", payload["next_actions"][1])
            self.assertEqual(payload["counts"]["passed"], 0)
            self.assertEqual(payload["counts"]["failed"], 0)
            self.assertEqual(payload["counts"]["blocked_infrastructure"], 1)
            self.assertEqual(payload["counts"]["skipped"], 0)


if __name__ == "__main__":
    unittest.main()
