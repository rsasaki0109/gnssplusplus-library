#!/usr/bin/env python3
"""Regression tests for CLASLIB parity shell helpers (wrapper + fair compare script)."""

from __future__ import annotations

import math
import json
import os
import subprocess
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
WRAPPER = ROOT_DIR / "scripts" / "run_gnss_ppp_claslib_parity.sh"
BENCH = ROOT_DIR / "scripts" / "benchmark_fair_vs_claslib.sh"


def _write_minimal_libgnss_pos(path: Path, rows: list[tuple]) -> None:
    lines = [
        "% LibGNSS++ Position Solution",
        "% Format: pos",
    ]
    for r in rows:
        lines.append(
            f"{r[0]} {r[1]:.6f} {r[2]:.6f} {r[3]:.6f} {r[4]:.6f} "
            f"{r[5]:.6f} {r[6]:.6f} {r[7]:.6f} {r[8]} {r[9]} {r[10]:.6f}"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_minimal_rtklib_ecef_pos(path: Path, rows: list[tuple[int, float, float, float, float]]) -> None:
    gps0 = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc).timestamp()
    lines = ["% RTKLIB/CLASLIB ECEF pos (minimal test)"]
    for week, tow, x, y, z in rows:
        ts = gps0 + week * 604800.0 + tow
        dt = datetime.fromtimestamp(ts, tz=timezone.utc)
        lines.append(
            f"{dt.strftime('%Y/%m/%d')} {dt.strftime('%H:%M:%S.%f')} "
            f"{x:.6f} {y:.6f} {z:.6f} 2 12 0 0 0 0 2.000000"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


class ClaslibParityWrapperTest(unittest.TestCase):
    def test_wrapper_inserts_claslib_parity_first(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnsspp_wrap_") as tmp_str:
            tmp = Path(tmp_str)
            recorder = tmp / "record_argv.py"
            recorder.write_text(
                """#!/usr/bin/env python3
import pathlib
import sys
out = pathlib.Path(sys.argv[0]).with_suffix(".argv.txt")
out.write_text("\\n".join(sys.argv[1:]), encoding="utf-8")
""",
                encoding="utf-8",
            )
            recorder.chmod(0o755)
            env = {**os.environ, "GNSS_PPP": str(recorder)}
            r = subprocess.run(
                [str(WRAPPER), "--obs", "rover.obs", "--out", "out.pos"],
                cwd=tmp,
                env=env,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(r.returncode, 0, msg=r.stderr + r.stdout)
            argv_lines = recorder.with_suffix(".argv.txt").read_text(encoding="utf-8").splitlines()
            self.assertGreaterEqual(len(argv_lines), 1)
            self.assertEqual(argv_lines[0], "--claslib-parity")
            self.assertIn("--obs", argv_lines)
            self.assertIn("rover.obs", argv_lines)


class FairBenchScriptTest(unittest.TestCase):
    def test_benchmark_writes_json_to_json_out_dir(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnsspp_bench_") as tmp_str:
            tmp = Path(tmp_str)
            pos = tmp / "run.pos"
            _write_minimal_libgnss_pos(
                pos,
                [
                    (2068, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5, 8, 2.0),
                    (2068, 101.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5, 8, 2.0),
                ],
            )
            json_dir = tmp / "outjson"
            env = {
                **os.environ,
                "REF_X": "0",
                "REF_Y": "0",
                "REF_Z": "0",
                "LAST_N": "10",
                "JSON_OUT_DIR": str(json_dir),
            }
            r = subprocess.run(
                ["/usr/bin/env", "bash", str(BENCH), str(pos)],
                cwd=ROOT_DIR,
                env=env,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(r.returncode, 0, msg=r.stderr + r.stdout)
            out = json_dir / "gnsspp_bench_libgnss.json"
            self.assertTrue(out.is_file())
            data = json.loads(out.read_text(encoding="utf-8"))
            self.assertIn("candidate_summary", data)

    def test_benchmark_with_claslib_reference_writes_exact_numeric_jsons(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnsspp_bench_clasref_") as tmp_str:
            tmp = Path(tmp_str)
            cand = tmp / "candidate.pos"
            ref = tmp / "claslib.pos"
            truth = (-3957235.3717, 3310368.2257, 3737529.7179)
            _write_minimal_libgnss_pos(
                cand,
                [
                    (2068, 100.0, truth[0], truth[1], truth[2], 36.1, 140.0, 70.0, 5, 8, 2.0),
                    (2068, 101.0, truth[0] + 3.0, truth[1] + 4.0, truth[2], 36.1, 140.0, 70.0, 5, 8, 2.0),
                    (2068, 102.0, truth[0], truth[1], truth[2] + 12.0, 36.1, 140.0, 70.0, 5, 8, 2.0),
                ],
            )
            _write_minimal_rtklib_ecef_pos(
                ref,
                [
                    (2068, 100.0, truth[0], truth[1], truth[2]),
                    (2068, 101.0, truth[0], truth[1], truth[2]),
                    (2068, 102.0, truth[0], truth[1], truth[2]),
                ],
            )
            json_dir = tmp / "outjson"
            env = {
                **os.environ,
                "REF_X": f"{truth[0]}",
                "REF_Y": f"{truth[1]}",
                "REF_Z": f"{truth[2]}",
                "LAST_N": "10",
                "JSON_OUT_DIR": str(json_dir),
            }
            r = subprocess.run(
                ["/usr/bin/env", "bash", str(BENCH), str(cand), str(ref)],
                cwd=ROOT_DIR,
                env=env,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(r.returncode, 0, msg=r.stderr + r.stdout)

            lib_json = json.loads((json_dir / "gnsspp_bench_libgnss.json").read_text(encoding="utf-8"))
            clas_json = json.loads((json_dir / "gnsspp_bench_claslib.json").read_text(encoding="utf-8"))
            diff_json = json.loads((json_dir / "gnsspp_bench_diff.json").read_text(encoding="utf-8"))

            expected_rms_3d = math.sqrt((0.0 * 0.0 + 5.0 * 5.0 + 12.0 * 12.0) / 3.0)
            self.assertEqual(lib_json["candidate_summary"]["valid_epochs"], 3)
            self.assertEqual(lib_json["error_vs_reference"]["epochs_used"], 3)
            self.assertAlmostEqual(lib_json["error_vs_reference"]["rms_3d_m"], expected_rms_3d, places=6)

            self.assertEqual(clas_json["candidate_format"], "rtklib-ecef")
            self.assertEqual(clas_json["error_vs_reference"]["epochs_used"], 3)
            self.assertAlmostEqual(clas_json["error_vs_reference"]["rms_3d_m"], 0.0, places=6)

            self.assertEqual(diff_json["reference_format"], "rtklib-ecef")
            self.assertEqual(diff_json["error_vs_reference"]["epochs_used"], 3)
            self.assertAlmostEqual(diff_json["error_vs_reference"]["rms_3d_m"], expected_rms_3d, places=6)


if __name__ == "__main__":
    unittest.main()
