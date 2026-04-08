#!/usr/bin/env python3
"""Regression tests for CLASLIB parity shell helpers (wrapper + fair compare script)."""

from __future__ import annotations

import json
import os
import subprocess
import tempfile
import unittest
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


if __name__ == "__main__":
    unittest.main()
