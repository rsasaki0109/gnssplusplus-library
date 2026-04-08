#!/usr/bin/env python3
"""Heavier CLASLIB-answer integration tests using the local 2019 CLAS dataset."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
PYTHON = Path(sys.executable)
WRAPPER = ROOT_DIR / "scripts" / "run_gnss_ppp_claslib_parity.sh"
BENCH = ROOT_DIR / "scripts" / "benchmark_fair_vs_claslib.sh"
COMPARE = ROOT_DIR / "tools" / "compare_ppp_solutions.py"

THESIS_ROOT = Path("/workspace/ai_coding_ws/gnssplusplus_thesis_ws")
CLAS_ROOT = THESIS_ROOT / "data" / "clas" / "claslib"
CLAS_DATA = CLAS_ROOT / "data"
CLAS_UTIL = CLAS_ROOT / "util" / "rnx2rtkp"
CLASLIB_BIN = CLAS_UTIL / "rnx2rtkp"
CLASLIB_STATIC_CONF = CLAS_UTIL / "static.conf"

OBS_2019 = CLAS_DATA / "0627239Q.obs"
NAV_2019 = CLAS_DATA / "sept_2019239.nav"
SSR_2019 = CLAS_DATA / "2019239Q.l6"

REF_X = -3957235.3717
REF_Y = 3310368.2257
REF_Z = 3737529.7179


def _resolve_gnss_ppp_bin() -> Path:
    for candidate in (ROOT_DIR / "build" / "gnss_ppp", ROOT_DIR / "build" / "apps" / "gnss_ppp"):
        if candidate.is_file():
            return candidate
    return ROOT_DIR / "build" / "gnss_ppp"


PPP_BIN = _resolve_gnss_ppp_bin()


def _integration_ready() -> bool:
    required = (
        PPP_BIN,
        WRAPPER,
        BENCH,
        COMPARE,
        CLASLIB_BIN,
        CLASLIB_STATIC_CONF,
        OBS_2019,
        NAV_2019,
        SSR_2019,
    )
    return all(path.is_file() for path in required) and os.access(CLASLIB_BIN, os.X_OK)


def _write_linux_static_conf(dst: Path) -> None:
    text = CLASLIB_STATIC_CONF.read_text(encoding="utf-8", errors="replace")
    text = text.replace("..\\..\\data\\", f"{CLAS_DATA}/")
    text += (
        "\n"
        "out-solformat      =xyz\n"
        "out-outhead        =on\n"
        "out-timesys        =gpst\n"
    )
    dst.write_text(text, encoding="utf-8")


def _time_window(max_epochs: int) -> tuple[str, str]:
    end_sec = max(max_epochs - 1, 0)
    return "2019/08/27 16:00:00", f"2019/08/27 16:00:{end_sec:02d}"


def _run_claslib_reference(out_pos: Path, *, max_epochs: int) -> subprocess.CompletedProcess[str]:
    with tempfile.TemporaryDirectory(prefix="gnsspp_claslib_cfg_") as tmp_str:
        tmp = Path(tmp_str)
        conf = tmp / "static_linux_xyz.conf"
        _write_linux_static_conf(conf)
        ts, te = _time_window(max_epochs)
        return subprocess.run(
            [
                str(CLASLIB_BIN),
                "-ti",
                "1",
                "-ts",
                ts,
                "-te",
                te,
                "-k",
                str(conf),
                str(OBS_2019),
                str(NAV_2019),
                str(SSR_2019),
                "-o",
                str(out_pos),
            ],
            cwd=CLAS_UTIL,
            capture_output=True,
            text=True,
            check=False,
        )


def _run_compare(candidate: Path, *, candidate_format: str | None = None) -> dict:
    with tempfile.TemporaryDirectory(prefix="gnsspp_clas_cmp_") as tmp_str:
        out_json = Path(tmp_str) / "compare.json"
        cmd = [
            str(PYTHON),
            str(COMPARE),
            "--candidate",
            str(candidate),
            "--ecef-x",
            str(REF_X),
            "--ecef-y",
            str(REF_Y),
            "--ecef-z",
            str(REF_Z),
            "--last-n",
            "10",
            "--json-out",
            str(out_json),
        ]
        if candidate_format is not None:
            cmd.extend(["--candidate-format", candidate_format])
        r = subprocess.run(cmd, capture_output=True, text=True, check=False)
        if r.returncode != 0:
            raise AssertionError(r.stderr + r.stdout)
        return json.loads(out_json.read_text(encoding="utf-8"))


@unittest.skipUnless(_integration_ready(), "local CLASLIB integration data or binaries missing")
class ClaslibAnswerIntegrationTest(unittest.TestCase):
    def test_claslib_reference_solution_is_the_answer_for_2019_static_window(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnsspp_clas_ans_") as tmp_str:
            tmp = Path(tmp_str)
            ref_pos = tmp / "claslib_2019_xyz.pos"
            r = _run_claslib_reference(ref_pos, max_epochs=10)
            self.assertEqual(r.returncode, 0, msg=r.stderr + r.stdout)
            self.assertTrue(ref_pos.is_file())

            report = _run_compare(ref_pos, candidate_format="rtklib-ecef")
            self.assertEqual(report["candidate_format"], "rtklib-ecef")
            self.assertGreaterEqual(report["candidate_summary"]["valid_epochs"], 8)
            self.assertGreater(report["error_vs_reference"]["epochs_used"], 0)
            self.assertLess(report["error_vs_reference"]["rms_3d_m"], 0.05)

    def test_wrapper_and_benchmark_accept_claslib_generated_answer_pos(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnsspp_clas_wrap_") as tmp_str:
            tmp = Path(tmp_str)
            ref_pos = tmp / "claslib_2019_xyz.pos"
            lib_pos = tmp / "libgnss_2019.pos"
            summary_json = tmp / "libgnss_summary.json"
            json_dir = tmp / "bench_json"

            ref_run = _run_claslib_reference(ref_pos, max_epochs=10)
            self.assertEqual(ref_run.returncode, 0, msg=ref_run.stderr + ref_run.stdout)
            self.assertTrue(ref_pos.is_file())

            env = {**os.environ, "GNSS_PPP": str(PPP_BIN)}
            lib_run = subprocess.run(
                [
                    str(WRAPPER),
                    "--obs",
                    str(OBS_2019),
                    "--nav",
                    str(NAV_2019),
                    "--ssr",
                    str(SSR_2019),
                    "--out",
                    str(lib_pos),
                    "--summary-json",
                    str(summary_json),
                    "--ref-x",
                    str(REF_X),
                    "--ref-y",
                    str(REF_Y),
                    "--ref-z",
                    str(REF_Z),
                    "--max-epochs",
                    "10",
                ],
                cwd=tmp,
                env=env,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(lib_run.returncode, 0, msg=lib_run.stderr + lib_run.stdout)
            self.assertTrue(lib_pos.is_file())
            self.assertTrue(summary_json.is_file())
            summary = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertTrue(summary.get("claslib_parity"))

            bench_env = {
                **os.environ,
                "REF_X": str(REF_X),
                "REF_Y": str(REF_Y),
                "REF_Z": str(REF_Z),
                "LAST_N": "10",
                "JSON_OUT_DIR": str(json_dir),
            }
            bench_run = subprocess.run(
                ["/usr/bin/env", "bash", str(BENCH), str(lib_pos), str(ref_pos)],
                cwd=ROOT_DIR,
                env=bench_env,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(bench_run.returncode, 0, msg=bench_run.stderr + bench_run.stdout)

            lib_report = json.loads((json_dir / "gnsspp_bench_libgnss.json").read_text(encoding="utf-8"))
            claslib_report = json.loads((json_dir / "gnsspp_bench_claslib.json").read_text(encoding="utf-8"))
            diff_report = json.loads((json_dir / "gnsspp_bench_diff.json").read_text(encoding="utf-8"))

            self.assertGreater(lib_report["candidate_summary"]["valid_epochs"], 0)
            self.assertEqual(claslib_report["candidate_format"], "rtklib-ecef")
            self.assertLess(claslib_report["error_vs_reference"]["rms_3d_m"], 0.05)
            self.assertEqual(diff_report["reference_format"], "rtklib-ecef")
            self.assertGreater(diff_report["error_vs_reference"]["epochs_used"], 0)

    def test_wrapper_and_benchmark_accept_claslib_generated_answer_pos_for_30_epochs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnsspp_clas_wrap30_") as tmp_str:
            tmp = Path(tmp_str)
            ref_pos = tmp / "claslib_2019_xyz_30.pos"
            lib_pos = tmp / "libgnss_2019_30.pos"
            summary_json = tmp / "libgnss_summary_30.json"
            json_dir = tmp / "bench_json_30"

            ref_run = _run_claslib_reference(ref_pos, max_epochs=30)
            self.assertEqual(ref_run.returncode, 0, msg=ref_run.stderr + ref_run.stdout)
            self.assertTrue(ref_pos.is_file())

            ref_report = _run_compare(ref_pos, candidate_format="rtklib-ecef")
            self.assertGreaterEqual(ref_report["candidate_summary"]["valid_epochs"], 25)
            self.assertLess(ref_report["error_vs_reference"]["rms_3d_m"], 0.05)

            env = {**os.environ, "GNSS_PPP": str(PPP_BIN)}
            lib_run = subprocess.run(
                [
                    str(WRAPPER),
                    "--obs",
                    str(OBS_2019),
                    "--nav",
                    str(NAV_2019),
                    "--ssr",
                    str(SSR_2019),
                    "--out",
                    str(lib_pos),
                    "--summary-json",
                    str(summary_json),
                    "--ref-x",
                    str(REF_X),
                    "--ref-y",
                    str(REF_Y),
                    "--ref-z",
                    str(REF_Z),
                    "--max-epochs",
                    "30",
                ],
                cwd=tmp,
                env=env,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(lib_run.returncode, 0, msg=lib_run.stderr + lib_run.stdout)
            self.assertTrue(lib_pos.is_file())
            self.assertTrue(summary_json.is_file())

            summary = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertTrue(summary.get("claslib_parity"))
            self.assertEqual(summary.get("processed_epochs"), 30)

            bench_env = {
                **os.environ,
                "REF_X": str(REF_X),
                "REF_Y": str(REF_Y),
                "REF_Z": str(REF_Z),
                "LAST_N": "30",
                "JSON_OUT_DIR": str(json_dir),
            }
            bench_run = subprocess.run(
                ["/usr/bin/env", "bash", str(BENCH), str(lib_pos), str(ref_pos)],
                cwd=ROOT_DIR,
                env=bench_env,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(bench_run.returncode, 0, msg=bench_run.stderr + bench_run.stdout)

            lib_report = json.loads((json_dir / "gnsspp_bench_libgnss.json").read_text(encoding="utf-8"))
            claslib_report = json.loads((json_dir / "gnsspp_bench_claslib.json").read_text(encoding="utf-8"))
            diff_report = json.loads((json_dir / "gnsspp_bench_diff.json").read_text(encoding="utf-8"))

            self.assertGreaterEqual(lib_report["candidate_summary"]["valid_epochs"], 25)
            self.assertGreaterEqual(lib_report["error_vs_reference"]["epochs_used"], 25)
            self.assertEqual(claslib_report["candidate_format"], "rtklib-ecef")
            self.assertGreaterEqual(claslib_report["error_vs_reference"]["epochs_used"], 25)
            self.assertLess(claslib_report["error_vs_reference"]["rms_3d_m"], 0.05)
            self.assertEqual(diff_report["reference_format"], "rtklib-ecef")
            self.assertGreaterEqual(diff_report["error_vs_reference"]["epochs_used"], 25)


if __name__ == "__main__":
    unittest.main()
