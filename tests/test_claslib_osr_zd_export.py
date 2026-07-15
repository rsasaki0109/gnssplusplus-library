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
            self.assertEqual(config.observation_file, "0627239Q.obs")
            self.assertEqual(config.navigation_file, "sept_2019239.nav")
            self.assertEqual(config.l6_file, "2019239Q.l6")

    def test_parse_config_accepts_alternate_public_dataset_inputs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_claslib_osr_config_") as temp_dir:
            config = runner.parse_config(
                runner.parse_args(["--repo-root", str(ROOT_DIR), "--output-dir", temp_dir]),
                {
                    "GNSSPP_CLASLIB_OSR_OBSERVATION_FILE": "0161329A.obs",
                    "GNSSPP_CLASLIB_OSR_NAVIGATION_FILE": "tskc2018329.nav",
                    "GNSSPP_CLASLIB_OSR_L6_FILE": "2018328X_329A.l6",
                },
            )

            self.assertEqual(config.observation_file, "0161329A.obs")
            self.assertEqual(config.navigation_file, "tskc2018329.nav")
            self.assertEqual(config.l6_file, "2018328X_329A.l6")

    def test_data_root_failures_reports_missing_source_data(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_claslib_osr_data_") as temp_dir:
            source_root = Path(temp_dir)
            (source_root / "data").mkdir()
            (source_root / "data" / "0627239Q.obs").write_text("", encoding="ascii")

            failures = runner.data_root_failures(source_root)

            self.assertIn("sept_2019239.nav", failures)
            self.assertIn("clas_grid.def", failures)
            self.assertNotIn("0627239Q.obs", failures)

            alternate = runner.data_root_failures(source_root, ("alternate.obs",))
            self.assertIn("alternate.obs", alternate)

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

    def test_prepare_instrumented_source_copies_user_source_and_adds_dd_dump(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_claslib_dd_instrument_") as temp_dir:
            root = Path(temp_dir)
            source_root = make_source_root(root)
            source_path = source_root / "src" / "ppprtk.c"
            source_path.parent.mkdir()
            source_path.write_text(
                "enum ddres_order { FST = 1, SND, TRD };\n"
                "int ddres(void) {\n"
                "    double pos[3],lami,lamj,*Ri,*Rj,*Hi=NULL;\n"
                "    int refchgflg;\n"
                "    trace(3,\"ddres   :niter=%2d nx=%d n=%d\\n\",niter,rtk->nx,n);\n"
                "                if (H) { /* partial derivatives by rover position */\n"
                "                /* set valid data flags */\n"
                "    dops(n,azel,opt->elmin,rtk->sol.dop); /* {GDOP,PDOP,HDOP,VDOP} */\n"
                "}\n"
                "/* single-differenced phase/code residuals  ----------------------------------\n",
                encoding="utf-8",
            )
            paths = runner.default_paths(root / "output")
            paths.work_dir.mkdir(parents=True)

            build_root = runner.prepare_instrumented_source(source_root, paths)
            instrumented = (build_root / "src" / "ppprtk.c").read_text(encoding="utf-8")

            self.assertEqual(build_root, paths.instrumented_source_dir)
            self.assertNotIn("GNSS++ canonical", source_path.read_text(encoding="utf-8"))
            self.assertIn("GNSS++ canonical CLAS DD oracle instrumentation", instrumented)
            self.assertIn("CLASLIB_DD_ROW_DUMP", instrumented)
            self.assertIn("clas_dd_measurement.v3", instrumented)
            self.assertIn("dump_rows=begin_clas_dd_dump", instrumented)
            self.assertIn("GNSS++ canonical CLAS filter-state oracle instrumentation", instrumented)
            self.assertIn("CLASLIB_FILTER_STATE_DUMP", instrumented)
            self.assertIn("clas_dd_filter_state.v1", instrumented)

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

            rollover = runner.RunConfig(
                **{
                    **config.__dict__,
                    "start_date": "2018/11/24",
                    "start_time": "23:59:31",
                    "max_epochs": 30,
                }
            )
            rollover_command = runner.build_rnx2rtkp_command(
                rollover, paths, source_root
            )
            end_index = rollover_command.index("-te")
            self.assertEqual(
                rollover_command[end_index + 1:end_index + 3],
                ["2018/11/25", "00:00:00"],
            )

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

        self.assertEqual(written["summary_schema"], runner.SUMMARY_SCHEMA)
        self.assertEqual(written["contract"], runner.CONTRACT)
        self.assertEqual(written["status"], "blocked_infrastructure")
        self.assertIn("missing CLASLIB OSR oracle evidence", written["next_actions"][1])
        self.assertEqual(written["configuration"]["zd_filter"]["rinex_code"], "C2W")
        self.assertEqual(
            written["configuration"]["dd_measurement_dump"]["source_policy"],
            "generated-checkout-or-copied-user-source",
        )

    def test_artifacts_include_normalized_dump_summary(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_claslib_osr_artifacts_") as temp_dir:
            paths = runner.default_paths(Path(temp_dir) / "output")
            artifacts = runner.summarize_artifacts(paths)

        roles = {artifact["role"]: artifact["path"] for artifact in artifacts}
        self.assertEqual(
            roles["normalized_zd_component_summary"],
            str(paths.normalized_summary_json),
        )
        self.assertEqual(
            roles["claslib_dd_measurement_dump"],
            str(paths.claslib_dd_dump),
        )
        self.assertEqual(
            roles["claslib_dd_filter_state_dump"],
            str(paths.claslib_state_dump),
        )


if __name__ == "__main__":
    unittest.main()
