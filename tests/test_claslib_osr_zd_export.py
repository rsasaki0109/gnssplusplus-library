#!/usr/bin/env python3
"""Tests for the CLASLIB OSR ZD export CI wrapper."""

from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "ci" / "run_claslib_osr_zd_export.py"

spec = importlib.util.spec_from_file_location("run_claslib_osr_zd_export", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
runner = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = runner
spec.loader.exec_module(runner)


def make_source_root(root: Path) -> Path:
    source_root = root / "claslib"
    data_root = source_root / "data"
    config_root = source_root / "util" / "rnx2rtkp"
    data_root.mkdir(parents=True)
    config_root.mkdir(parents=True)
    for name in runner.REQUIRED_DATA_FILES:
        (data_root / name).write_text(f"{name}\n", encoding="ascii")
    (config_root / "static.conf").write_text(
        "file-cssrgridfile  =..\\..\\data\\clas_grid.def\n"
        "file-blqfile       =..\\..\\data\\clas_grid.blq\n",
        encoding="ascii",
    )
    return source_root


class ClaslibOsrZdExportTest(unittest.TestCase):
    def test_parse_config_defaults_to_public_claslib_source_fetch(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_claslib_osr_config_") as temp_dir:
            config = runner.parse_config(
                runner.parse_args(["--repo-root", str(ROOT_DIR), "--output-dir", temp_dir]),
                {},
            )

            self.assertTrue(config.auto_fetch)
            self.assertTrue(config.fail_on_blocked)
            self.assertEqual(config.claslib_repo, runner.DEFAULT_CLASLIB_REPO)
            self.assertEqual(config.claslib_ref, runner.DEFAULT_CLASLIB_REF)
            self.assertEqual(config.gps_week, runner.DEFAULT_GPS_WEEK)
            self.assertEqual(config.max_epochs, runner.DEFAULT_MAX_EPOCHS)
            self.assertEqual(config.gps_l2w_rows_min, runner.DEFAULT_MAX_EPOCHS)

    def test_data_root_failures_reports_missing_source_data(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_claslib_osr_data_") as temp_dir:
            source_root = Path(temp_dir)
            (source_root / "data").mkdir()
            (source_root / "data" / "0627239Q.obs").write_text("", encoding="ascii")

            failures = runner.data_root_failures(source_root)

            self.assertIn("sept_2019239.nav", failures)
            self.assertIn("clas_grid.def", failures)
            self.assertNotIn("0627239Q.obs", failures)

    def test_write_linux_config_rewrites_claslib_data_paths(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_claslib_osr_config_") as temp_dir:
            root = Path(temp_dir)
            source_root = make_source_root(root)
            paths = runner.default_paths(root / "output")
            paths.work_dir.mkdir(parents=True)

            runner.write_linux_config(source_root, paths)

            written = paths.claslib_config.read_text(encoding="utf-8")
            self.assertIn(str(source_root / "data") + "/clas_grid.def", written)
            self.assertIn(str(source_root / "data") + "/clas_grid.blq", written)
            self.assertNotIn("..\\..\\data\\", written)

    def test_build_rnx2rtkp_command_sets_osr_window_and_inputs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_claslib_osr_command_") as temp_dir:
            root = Path(temp_dir)
            source_root = make_source_root(root)
            output_dir = root / "output"
            paths = runner.default_paths(output_dir)
            config = runner.RunConfig(
                repo_root=ROOT_DIR,
                output_dir=output_dir,
                source_root=source_root,
                auto_fetch=False,
                fail_on_blocked=True,
                claslib_repo=runner.DEFAULT_CLASLIB_REPO,
                claslib_ref=runner.DEFAULT_CLASLIB_REF,
                gps_week=2068,
                max_epochs=30,
                gps_l2w_rows_min=30,
                start_date="2019/08/27",
                start_time="16:00:00",
                python_executable=sys.executable,
            )

            command = runner.build_rnx2rtkp_command(config, paths, source_root)

            self.assertIn("-s", command)
            self.assertIn("-l6w", command)
            self.assertIn("2068", command)
            self.assertIn("-te", command)
            self.assertIn("16:00:29", command)
            self.assertIn(str(paths.claslib_config), command)
            self.assertIn(str(paths.claslib_solution), command)
            self.assertIn(str(source_root / "data" / "0627239Q.obs"), command)
            self.assertIn(str(source_root / "data" / "2019239Q.l6"), command)

            normalize_command = runner.build_normalize_command(config, paths, source_root)
            self.assertIn("--clas-grid-def", normalize_command)
            self.assertIn(str(source_root / "data" / "clas_grid.def"), normalize_command)

    def test_evaluate_accepts_normalized_osr_summary(self) -> None:
        config = runner.RunConfig(
            repo_root=ROOT_DIR,
            output_dir=ROOT_DIR / "output",
            source_root=None,
            auto_fetch=True,
            fail_on_blocked=True,
            claslib_repo=runner.DEFAULT_CLASLIB_REPO,
            claslib_ref=runner.DEFAULT_CLASLIB_REF,
            gps_week=2068,
            max_epochs=30,
            gps_l2w_rows_min=30,
            start_date="2019/08/27",
            start_time="16:00:00",
            python_executable=sys.executable,
        )
        summary = {
            "schema": "clas_zd_component_summary.v2",
            "status": "passed",
            "rows": 100,
            "identity_provenance": {"gps_l2w_rows": 40},
        }

        metrics, thresholds, failures = runner.evaluate(config, summary, 100, 40)

        self.assertEqual(failures, [])
        self.assertEqual(metrics["normalized_rows"], 100)
        self.assertEqual(metrics["gps_l2w_rows"], 40)
        self.assertEqual(metrics["gps_l2w_grid_provenance_rows"], 40)
        self.assertEqual(thresholds["gps_l2w_rows_min"], 30)
        self.assertEqual(thresholds["gps_l2w_grid_provenance_rows_min"], 30)

    def test_evaluate_rejects_bad_normalized_summary(self) -> None:
        config = runner.RunConfig(
            repo_root=ROOT_DIR,
            output_dir=ROOT_DIR / "output",
            source_root=None,
            auto_fetch=True,
            fail_on_blocked=True,
            claslib_repo=runner.DEFAULT_CLASLIB_REPO,
            claslib_ref=runner.DEFAULT_CLASLIB_REF,
            gps_week=2068,
            max_epochs=30,
            gps_l2w_rows_min=30,
            start_date="2019/08/27",
            start_time="16:00:00",
            python_executable=sys.executable,
        )
        summary = {
            "schema": "clas_zd_component_summary.v2",
            "status": "failed",
            "rows": 99,
            "identity_provenance": {"gps_l2w_rows": 20},
        }

        _metrics, _thresholds, failures = runner.evaluate(config, summary, 100, 10)

        self.assertIn("normalized summary status failed != passed", failures)
        self.assertIn("normalized summary rows 99 != normalized rows 100", failures)
        self.assertIn("GPS L2W rows 20 < 30", failures)
        self.assertIn("GPS L2W grid provenance rows 10 < 30", failures)

    def test_blocked_summary_contract_has_next_action(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_claslib_osr_summary_") as temp_dir:
            output_dir = Path(temp_dir) / "output"
            paths = runner.default_paths(output_dir)
            paths.log_path.parent.mkdir(parents=True)
            paths.log_path.write_text("blocked\n", encoding="ascii")
            config = runner.RunConfig(
                repo_root=ROOT_DIR,
                output_dir=output_dir,
                source_root=None,
                auto_fetch=False,
                fail_on_blocked=False,
                claslib_repo=runner.DEFAULT_CLASLIB_REPO,
                claslib_ref=runner.DEFAULT_CLASLIB_REF,
                gps_week=2068,
                max_epochs=30,
                gps_l2w_rows_min=30,
                start_date="2019/08/27",
                start_time="16:00:00",
                python_executable=sys.executable,
            )

            payload = runner.build_payload(
                config=config,
                paths=paths,
                status="blocked_infrastructure",
                source_root=None,
                block_reason="no source",
            )
            runner.write_summary(paths, payload)
            written = json.loads(paths.summary_json.read_text(encoding="utf-8"))

        self.assertEqual(written["summary_schema"], "ci_claslib_osr_zd_export.v2")
        self.assertEqual(written["contract"], "claslib_osr_zd_export.v2")
        self.assertEqual(written["status"], "blocked_infrastructure")
        self.assertIn("missing CLASLIB OSR oracle evidence", written["next_actions"][1])
        self.assertEqual(written["configuration"]["zd_filter"]["rinex_code"], "C2W")

    def test_artifacts_include_normalized_dump_summary(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_claslib_osr_artifacts_") as temp_dir:
            paths = runner.default_paths(Path(temp_dir) / "output")
            artifacts = runner.summarize_artifacts(paths)

        roles = {artifact["role"]: artifact["path"] for artifact in artifacts}
        self.assertEqual(
            roles["normalized_zd_component_summary"],
            str(paths.normalized_summary_json),
        )


if __name__ == "__main__":
    unittest.main()
