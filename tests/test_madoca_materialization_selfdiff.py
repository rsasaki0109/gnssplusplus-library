#!/usr/bin/env python3
"""Tests for the public MADOCA materialization self-diff CI runner."""

from __future__ import annotations

import importlib.util
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "ci" / "run_madoca_materialization_selfdiff.py"

spec = importlib.util.spec_from_file_location("run_madoca_materialization_selfdiff", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
selfdiff = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = selfdiff
spec.loader.exec_module(selfdiff)


class MadocaMaterializationSelfdiffTest(unittest.TestCase):
    def test_parse_config_defaults_to_public_madocalib_fetch(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_materialization_selfdiff_config_") as temp_dir:
            args = selfdiff.parse_args(["--repo-root", str(ROOT_DIR), "--output-dir", temp_dir])

            config = selfdiff.parse_config(args, {})

            self.assertEqual(config.repo_root, ROOT_DIR)
            self.assertEqual(config.output_dir, Path(temp_dir).resolve())
            self.assertTrue(config.auto_fetch)
            self.assertTrue(config.fail_on_blocked)
            self.assertEqual(config.madocalib_ref, selfdiff.DEFAULT_MADOCALIB_REF)
            self.assertEqual(config.min_rows, selfdiff.DEFAULT_MIN_ROWS)
            self.assertEqual(config.required_systems, selfdiff.DEFAULT_REQUIRED_SYSTEMS)
            self.assertEqual(config.required_code_bias_ids, selfdiff.DEFAULT_REQUIRED_BIAS_IDS)
            self.assertEqual(config.required_phase_bias_ids, selfdiff.DEFAULT_REQUIRED_BIAS_IDS)

    def test_parse_config_accepts_required_identity_overrides(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_materialization_selfdiff_identity_") as temp_dir:
            args = selfdiff.parse_args(["--repo-root", str(ROOT_DIR), "--output-dir", temp_dir])

            config = selfdiff.parse_config(
                args,
                {
                    "GNSSPP_MADOCA_MATERIALIZATION_REQUIRED_SYSTEMS": "GPS,QZSS",
                    "GNSSPP_MADOCA_MATERIALIZATION_REQUIRED_CODE_BIAS_IDS": "2,8",
                    "GNSSPP_MADOCA_MATERIALIZATION_REQUIRED_PHASE_BIAS_IDS": "14",
                },
            )

            self.assertEqual(config.required_systems, ("GPS", "QZSS"))
            self.assertEqual(config.required_code_bias_ids, ("2", "8"))
            self.assertEqual(config.required_phase_bias_ids, ("14",))

    def test_data_root_failures_require_nav_and_l6(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_materialization_selfdiff_data_") as temp_dir:
            data_root = Path(temp_dir)
            for relative in selfdiff.REQUIRED_DATA_FILES:
                path = data_root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("x", encoding="ascii")

            self.assertEqual(selfdiff.data_root_failures(data_root), [])
            os.remove(data_root / selfdiff.REQUIRED_DATA_FILES[1])
            self.assertEqual(
                selfdiff.data_root_failures(data_root),
                [selfdiff.REQUIRED_DATA_FILES[1]],
            )

    def test_build_commands_use_dump_only_and_selfdiff_outputs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_materialization_selfdiff_command_") as temp_dir:
            args = selfdiff.parse_args(["--repo-root", str(ROOT_DIR), "--output-dir", temp_dir])
            config = selfdiff.parse_config(args, {})
            paths = selfdiff.default_paths(config.output_dir)
            data_root = Path(temp_dir) / "madocalib"

            dump_command = selfdiff.build_dump_command(config, paths, data_root)
            summary_command = selfdiff.build_materialization_summary_command(config, paths)
            selfdiff_command = selfdiff.build_selfdiff_command(config, paths)

            self.assertIn("--madoca-materialization-dump-only", dump_command)
            self.assertIn("--madoca-materialization-dump", dump_command)
            self.assertIn(str(paths.native_dump), dump_command)
            self.assertNotIn("--out", dump_command)
            self.assertIn(str(ROOT_DIR / "scripts" / "analysis" / "madoca_materialization_summary.py"), summary_command)
            self.assertIn(str(paths.native_dump), summary_command)
            self.assertIn(str(paths.native_materialization_summary_json), summary_command)
            self.assertIn("--fail-on-issue", summary_command)
            self.assertIn("--require-no-duplicate-keys", summary_command)
            self.assertIn("--require-system", summary_command)
            self.assertIn("GPS", summary_command)
            self.assertIn("--require-code-bias-id", summary_command)
            self.assertIn("2", summary_command)
            self.assertIn("--require-phase-bias-id", summary_command)
            self.assertIn(str(ROOT_DIR / "scripts" / "analysis" / "madoca_materialization_diff.py"), selfdiff_command)
            self.assertIn(str(paths.native_dump), selfdiff_command)
            self.assertIn(str(paths.selfdiff_json), selfdiff_command)
            self.assertIn("--fail-on-diff", selfdiff_command)

    def test_evaluate_accepts_exact_selfdiff(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_materialization_selfdiff_eval_") as temp_dir:
            args = selfdiff.parse_args(["--repo-root", str(ROOT_DIR), "--output-dir", temp_dir])
            config = selfdiff.parse_config(args, {"GNSSPP_MADOCA_MATERIALIZATION_ROWS_MIN": "2"})
            paths = selfdiff.default_paths(config.output_dir)
            paths.work_dir.mkdir(parents=True, exist_ok=True)
            paths.native_dump.write_text("header\nrow1\nrow2\n", encoding="ascii")

            evaluation = selfdiff.evaluate(
                config=config,
                paths=paths,
                dump_summary={"madoca_materialization_rows": 2},
                materialization_summary={
                    "schema": "madoca_materialization_summary.v1",
                    "status": "passed",
                    "rows": 2,
                    "systems": {"GPS": 2, "GLONASS": 1, "Galileo": 1, "QZSS": 1, "BeiDou": 1},
                    "row_key": {"groups": 2, "duplicate_groups": 0, "max_duplicate_occurrences": 1},
                    "bias_identity": {
                        "code_bias_ids": {"2": 2, "8": 1, "9": 1, "14": 1, "22": 1},
                        "phase_bias_ids": {"2": 2, "8": 1, "9": 1, "14": 1, "22": 1},
                    },
                },
                selfdiff_report={
                    "schema": "madoca_materialization_diff.v1",
                    "base_only_rows": 0,
                    "candidate_only_rows": 0,
                    "row_set_complete": True,
                    "discrete_mismatches": 0,
                    "numeric_threshold_exceedances": 0,
                    "common_rows": 2,
                    "numeric_components_compared": 12,
                    "per_numeric_component": [{"component": "clock_m", "max_abs_delta": 0.0}],
                },
            )

            self.assertEqual(evaluation.status, "passed")
            self.assertEqual(evaluation.failures, [])
            self.assertEqual(evaluation.metrics["materialization_rows"], 2)
            self.assertEqual(evaluation.metrics["selfdiff_max_abs_delta"], 0.0)

    def test_evaluate_rejects_missing_identity_contract(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_materialization_selfdiff_identity_bad_") as temp_dir:
            args = selfdiff.parse_args(["--repo-root", str(ROOT_DIR), "--output-dir", temp_dir])
            config = selfdiff.parse_config(args, {"GNSSPP_MADOCA_MATERIALIZATION_ROWS_MIN": "1"})
            paths = selfdiff.default_paths(config.output_dir)
            paths.work_dir.mkdir(parents=True, exist_ok=True)
            paths.native_dump.write_text("header\nrow1\n", encoding="ascii")

            evaluation = selfdiff.evaluate(
                config=config,
                paths=paths,
                dump_summary={"madoca_materialization_rows": 1},
                materialization_summary={
                    "schema": "madoca_materialization_summary.v1",
                    "status": "passed",
                    "rows": 1,
                    "systems": {"GPS": 1},
                    "row_key": {"groups": 1, "duplicate_groups": 1, "max_duplicate_occurrences": 2},
                    "bias_identity": {"code_bias_ids": {"2": 1}, "phase_bias_ids": {}},
                },
                selfdiff_report={
                    "schema": "madoca_materialization_diff.v1",
                    "base_only_rows": 0,
                    "candidate_only_rows": 0,
                    "row_set_complete": True,
                    "discrete_mismatches": 0,
                    "numeric_threshold_exceedances": 0,
                    "common_rows": 1,
                    "numeric_components_compared": 12,
                    "per_numeric_component": [{"component": "clock_m", "max_abs_delta": 0.0}],
                },
            )

            self.assertEqual(evaluation.status, "failed")
            self.assertIn("materialization summary missing required system QZSS", evaluation.failures)
            self.assertIn("materialization summary duplicate groups 1 != 0", evaluation.failures)
            self.assertIn("materialization summary missing required code bias id 8", evaluation.failures)
            self.assertIn("materialization summary missing required phase bias id 2", evaluation.failures)

    def test_evaluate_rejects_missing_rows_and_selfdiff_mismatch(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_materialization_selfdiff_bad_") as temp_dir:
            args = selfdiff.parse_args(["--repo-root", str(ROOT_DIR), "--output-dir", temp_dir])
            config = selfdiff.parse_config(args, {"GNSSPP_MADOCA_MATERIALIZATION_ROWS_MIN": "10"})
            paths = selfdiff.default_paths(config.output_dir)
            paths.work_dir.mkdir(parents=True, exist_ok=True)
            paths.native_dump.write_text("header\nrow1\n", encoding="ascii")

            evaluation = selfdiff.evaluate(
                config=config,
                paths=paths,
                dump_summary={"madoca_materialization_rows": 1},
                materialization_summary={
                    "schema": "madoca_materialization_summary.v1",
                    "status": "failed",
                    "rows": 0,
                    "systems": {},
                    "row_key": {},
                    "bias_identity": {},
                },
                selfdiff_report={
                    "schema": "madoca_materialization_diff.v1",
                    "base_only_rows": 1,
                    "candidate_only_rows": 0,
                    "row_set_complete": False,
                    "discrete_mismatches": 2,
                    "numeric_threshold_exceedances": 3,
                    "per_numeric_component": [{"component": "clock_m", "max_abs_delta": 0.5}],
                },
            )

            self.assertEqual(evaluation.status, "failed")
            self.assertGreaterEqual(len(evaluation.failures), 4)

    def test_write_summary_records_blocked_infrastructure(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_madoca_materialization_selfdiff_summary_") as temp_dir:
            args = selfdiff.parse_args(["--repo-root", str(ROOT_DIR), "--output-dir", temp_dir])
            config = selfdiff.parse_config(args, {})
            paths = selfdiff.default_paths(config.output_dir)
            paths.log_path.parent.mkdir(parents=True, exist_ok=True)
            paths.log_path.write_text("blocked\n", encoding="ascii")

            selfdiff.write_summary(
                paths,
                config=config,
                status="blocked_infrastructure",
                block_reason="missing data",
            )

            payload = json.loads(paths.summary_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["summary_schema"], selfdiff.SUMMARY_SCHEMA)
            self.assertEqual(payload["contract"], "madoca_materialization_selfdiff.v1")
            self.assertEqual(payload["status"], "blocked_infrastructure")
            self.assertEqual(payload["block_reason"], "missing data")
            self.assertEqual(payload["configuration"]["required_systems"], list(selfdiff.DEFAULT_REQUIRED_SYSTEMS))
            self.assertIn("missing native MADOCA materialization evidence", payload["next_actions"][1])
            self.assertTrue(payload["artifacts"][0]["exists"])


if __name__ == "__main__":
    unittest.main()
