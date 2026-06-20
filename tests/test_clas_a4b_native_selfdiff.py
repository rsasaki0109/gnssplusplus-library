#!/usr/bin/env python3
"""Tests for the CLAS A4b native self-diff CI wrapper."""

from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "ci" / "run_clas_a4b_native_selfdiff.py"

spec = importlib.util.spec_from_file_location("run_clas_a4b_native_selfdiff", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
runner = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = runner
spec.loader.exec_module(runner)


def make_data_root(root: Path) -> Path:
    data_root = root / "data"
    data_root.mkdir()
    for name in runner.REQUIRED_DATA_FILES:
        (data_root / name).write_text(f"{name}\n", encoding="ascii")
    return data_root


class ClasA4bNativeSelfdiffTest(unittest.TestCase):
    def test_parse_config_defaults_to_public_claslib_fetch(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_a4b_config_") as temp_dir:
            config = runner.parse_config(
                runner.parse_args(["--repo-root", str(ROOT_DIR), "--output-dir", temp_dir]),
                {},
            )

            self.assertTrue(config.auto_fetch)
            self.assertTrue(config.fail_on_blocked)
            self.assertEqual(config.claslib_repo, runner.DEFAULT_CLASLIB_REPO)
            self.assertEqual(config.claslib_ref, runner.DEFAULT_CLASLIB_REF)
            self.assertEqual(config.max_epochs, runner.DEFAULT_MAX_EPOCHS)
            self.assertEqual(config.gps_l2w_rows_min, runner.DEFAULT_MAX_EPOCHS)

    def test_data_root_failures_reports_missing_public_inputs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_a4b_data_") as temp_dir:
            data_root = Path(temp_dir)
            (data_root / "0627239Q.obs").write_text("", encoding="ascii")

            failures = runner.data_root_failures(data_root)

            self.assertEqual(
                failures,
                ["sept_2019239.nav", "2019239Q.l6", "igs14_L5copy.atx"],
            )

    def test_build_native_command_sets_a4b_environment_and_policies(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_a4b_command_") as temp_dir:
            root = Path(temp_dir)
            data_root = make_data_root(root)
            output_dir = root / "output"
            paths = runner.default_paths(output_dir)
            config = runner.RunConfig(
                repo_root=ROOT_DIR,
                output_dir=output_dir,
                data_root=data_root,
                auto_fetch=False,
                fail_on_blocked=True,
                claslib_repo=runner.DEFAULT_CLASLIB_REPO,
                claslib_ref=runner.DEFAULT_CLASLIB_REF,
                max_epochs=30,
                gps_l2w_rows_min=30,
                receiver_antenna_type="TEST ANT",
                python_executable=sys.executable,
            )

            command, env = runner.build_native_command(config, paths, data_root)

            self.assertEqual(env["GNSS_PPP_CLAS_DD_FILTER"], "1")
            self.assertEqual(env["GNSS_PPP_CLAS_CODE_ROW_PARITY"], "bias,full-prc")
            self.assertEqual(env["GNSS_PPP_CLAS_RX_ANTENNA"], "1")
            self.assertEqual(env["GNSS_PPP_CLAS_ATMOS_GRID_MATRIX"], "1")
            self.assertEqual(env["GNSS_PPP_CLAS_ATMOS_LIFECYCLE"], "1")
            self.assertEqual(env["GNSS_PPP_CLAS_TROP_CLIMATOLOGY"], "1")
            self.assertEqual(env["GNSS_PPP_CLAS_CODE_DUMP"], str(paths.native_code_dump))
            self.assertIn("clas-ppp", command)
            self.assertIn("--compact-code-bias-composition-policy", command)
            self.assertIn("base-only-if-present", command)
            self.assertIn("--compact-code-bias-bank-policy", command)
            self.assertIn("latest-preceding-bank", command)
            self.assertIn("--compact-bias-row-materialization", command)
            self.assertIn("selected-satellite-base-extend", command)
            self.assertIn("--receiver-antenna-type", command)
            self.assertIn("TEST ANT", command)
            self.assertEqual(command[-2:], ["--max-epochs", "30"])

    def test_build_code_dump_summary_command_validates_native_dump(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_a4b_summary_command_") as temp_dir:
            output_dir = Path(temp_dir) / "output"
            paths = runner.default_paths(output_dir)
            config = runner.RunConfig(
                repo_root=ROOT_DIR,
                output_dir=output_dir,
                data_root=None,
                auto_fetch=True,
                fail_on_blocked=True,
                claslib_repo=runner.DEFAULT_CLASLIB_REPO,
                claslib_ref=runner.DEFAULT_CLASLIB_REF,
                max_epochs=30,
                gps_l2w_rows_min=30,
                receiver_antenna_type=runner.DEFAULT_RECEIVER_ANTENNA_TYPE,
                python_executable=sys.executable,
            )

            command = runner.build_code_dump_summary_command(config, paths)

            self.assertIn(str(ROOT_DIR / "scripts" / "analysis" / "clas_zd_component_summary.py"), command)
            self.assertIn(str(paths.native_code_dump), command)
            self.assertIn(str(paths.native_code_dump_summary_json), command)
            self.assertIn("--fail-on-issue", command)
            self.assertIn("--require-system", command)
            self.assertIn("GPS", command)
            self.assertIn("--require-row-type", command)
            self.assertIn("code", command)
            self.assertIn("--require-rinex-code", command)
            self.assertIn("C2W", command)
            self.assertIn("--require-gps-l2w-rows-min", command)
            self.assertIn("30", command)
            self.assertIn("--require-gps-l2w-exact-bias", command)
            self.assertIn("--require-gps-l2w-observation-exact-match", command)
            self.assertIn("--require-gps-l2w-no-observation-family-fallback", command)
            self.assertIn("--require-gps-l2w-no-code-bias-fallback", command)
            self.assertIn("--require-gps-l2w-no-phase-bias-fallback", command)
            self.assertIn("--require-no-duplicate-keys", command)

    def test_evaluate_accepts_exact_native_l2w_selfdiff(self) -> None:
        config = runner.RunConfig(
            repo_root=ROOT_DIR,
            output_dir=ROOT_DIR / "output",
            data_root=None,
            auto_fetch=True,
            fail_on_blocked=True,
            claslib_repo=runner.DEFAULT_CLASLIB_REPO,
            claslib_ref=runner.DEFAULT_CLASLIB_REF,
            max_epochs=30,
            gps_l2w_rows_min=30,
            receiver_antenna_type=runner.DEFAULT_RECEIVER_ANTENNA_TYPE,
            python_executable=sys.executable,
        )
        native_summary = {"epochs": 30, "ppp_solution_rate_pct": 100.0}
        code_dump_summary = {
            "schema": "clas_zd_component_summary.v2",
            "status": "passed",
            "rows": 540,
            "row_key": {"duplicate_groups": 0},
        }
        report = {
            "common_rows": 30,
            "base_only_rows": 0,
            "candidate_only_rows": 0,
            "components_compared": 630,
            "threshold_exceedances": 0,
            "per_component": [{"component": "prc_m", "max_abs_delta_m": 0.0}],
            "identity_provenance": {
                "native": {
                    "gps_l2w_rows": 30,
                    "gps_l2w_bias_exact_identity_rows": 30,
                    "gps_l2w_observation_exact_match_rows": 30,
                    "gps_l2w_observation_family_fallback_rows": 0,
                    "gps_l2w_code_bias_fallback_rows": 0,
                }
            },
        }

        metrics, thresholds, failures = runner.evaluate(
            config,
            native_summary,
            code_dump_summary,
            report,
            540,
        )

        self.assertEqual(failures, [])
        self.assertEqual(metrics["gps_l2w_rows"], 30)
        self.assertEqual(metrics["native_code_dump_rows"], 540)
        self.assertEqual(metrics["native_code_dump_summary_schema"], "clas_zd_component_summary.v2")
        self.assertEqual(metrics["native_code_dump_summary_status"], "passed")
        self.assertEqual(metrics["native_code_dump_summary_rows"], 540)
        self.assertEqual(metrics["native_code_dump_summary_duplicate_groups"], 0)
        self.assertEqual(thresholds["gps_l2w_rows_min"], 30)

    def test_evaluate_rejects_identity_fallback(self) -> None:
        config = runner.RunConfig(
            repo_root=ROOT_DIR,
            output_dir=ROOT_DIR / "output",
            data_root=None,
            auto_fetch=True,
            fail_on_blocked=True,
            claslib_repo=runner.DEFAULT_CLASLIB_REPO,
            claslib_ref=runner.DEFAULT_CLASLIB_REF,
            max_epochs=30,
            gps_l2w_rows_min=30,
            receiver_antenna_type=runner.DEFAULT_RECEIVER_ANTENNA_TYPE,
            python_executable=sys.executable,
        )
        native_summary = {"epochs": 30, "ppp_solution_rate_pct": 100.0}
        code_dump_summary = {
            "schema": "clas_zd_component_summary.v2",
            "status": "passed",
            "rows": 540,
            "row_key": {"duplicate_groups": 0},
        }
        report = {
            "common_rows": 30,
            "base_only_rows": 0,
            "candidate_only_rows": 0,
            "components_compared": 630,
            "threshold_exceedances": 0,
            "per_component": [{"component": "code_bias_m", "max_abs_delta_m": 0.0}],
            "identity_provenance": {
                "native": {
                    "gps_l2w_rows": 30,
                    "gps_l2w_bias_exact_identity_rows": 29,
                    "gps_l2w_observation_exact_match_rows": 30,
                    "gps_l2w_observation_family_fallback_rows": 1,
                    "gps_l2w_code_bias_fallback_rows": 0,
                }
            },
        }

        _metrics, _thresholds, failures = runner.evaluate(
            config,
            native_summary,
            code_dump_summary,
            report,
            540,
        )

        self.assertIn("exact bias rows 29 != GPS L2W rows 30", failures)
        self.assertIn("observation-family fallback rows 1 != 0", failures)

    def test_evaluate_rejects_bad_code_dump_summary(self) -> None:
        config = runner.RunConfig(
            repo_root=ROOT_DIR,
            output_dir=ROOT_DIR / "output",
            data_root=None,
            auto_fetch=True,
            fail_on_blocked=True,
            claslib_repo=runner.DEFAULT_CLASLIB_REPO,
            claslib_ref=runner.DEFAULT_CLASLIB_REF,
            max_epochs=30,
            gps_l2w_rows_min=30,
            receiver_antenna_type=runner.DEFAULT_RECEIVER_ANTENNA_TYPE,
            python_executable=sys.executable,
        )
        native_summary = {"epochs": 30, "ppp_solution_rate_pct": 100.0}
        code_dump_summary = {
            "schema": "clas_zd_component_summary.v2",
            "status": "failed",
            "rows": 539,
            "row_key": {"duplicate_groups": 1},
        }
        report = {
            "common_rows": 30,
            "base_only_rows": 0,
            "candidate_only_rows": 0,
            "components_compared": 630,
            "threshold_exceedances": 0,
            "per_component": [{"component": "prc_m", "max_abs_delta_m": 0.0}],
            "identity_provenance": {
                "native": {
                    "gps_l2w_rows": 30,
                    "gps_l2w_bias_exact_identity_rows": 30,
                    "gps_l2w_observation_exact_match_rows": 30,
                    "gps_l2w_observation_family_fallback_rows": 0,
                    "gps_l2w_code_bias_fallback_rows": 0,
                }
            },
        }

        _metrics, _thresholds, failures = runner.evaluate(
            config,
            native_summary,
            code_dump_summary,
            report,
            540,
        )

        self.assertIn("native code dump summary status failed != passed", failures)
        self.assertIn("native code dump summary rows 539 != dump rows 540", failures)

    def test_blocked_summary_contract_has_next_action(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_a4b_summary_") as temp_dir:
            output_dir = Path(temp_dir) / "output"
            paths = runner.default_paths(output_dir)
            paths.log_path.parent.mkdir(parents=True)
            paths.log_path.write_text("blocked\n", encoding="ascii")
            config = runner.RunConfig(
                repo_root=ROOT_DIR,
                output_dir=output_dir,
                data_root=None,
                auto_fetch=False,
                fail_on_blocked=False,
                claslib_repo=runner.DEFAULT_CLASLIB_REPO,
                claslib_ref=runner.DEFAULT_CLASLIB_REF,
                max_epochs=30,
                gps_l2w_rows_min=30,
                receiver_antenna_type=runner.DEFAULT_RECEIVER_ANTENNA_TYPE,
                python_executable=sys.executable,
            )

            payload = runner.build_payload(
                config=config,
                paths=paths,
                status="blocked_infrastructure",
                data_root=None,
                block_reason="no data",
            )
            runner.write_summary(paths, payload)
            written = json.loads(paths.summary_json.read_text(encoding="utf-8"))

        self.assertEqual(written["summary_schema"], runner.SUMMARY_SCHEMA)
        self.assertEqual(written["contract"], runner.CONTRACT)
        self.assertEqual(written["status"], "blocked_infrastructure")
        self.assertIn("missing native A4b evidence", written["next_actions"][1])
        self.assertFalse(written["configuration"]["fail_on_blocked"])
        self.assertEqual(written["configuration"]["selfdiff_filter"]["rinex_code"], "C2W")

    def test_artifacts_include_code_dump_summary(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_a4b_artifacts_") as temp_dir:
            paths = runner.default_paths(Path(temp_dir) / "output")
            artifacts = runner.summarize_artifacts(paths)

        roles = {artifact["role"]: artifact["path"] for artifact in artifacts}
        self.assertEqual(
            roles["native_zd_component_summary"],
            str(paths.native_code_dump_summary_json),
        )


if __name__ == "__main__":
    unittest.main()
