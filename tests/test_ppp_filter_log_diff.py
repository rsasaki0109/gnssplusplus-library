#!/usr/bin/env python3
"""Tests for PPP filter log diff summaries."""

from __future__ import annotations

import csv
import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "ppp_filter_log_diff.py"

spec = importlib.util.spec_from_file_location("ppp_filter_log_diff", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
diff_mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diff_mod)


FIELDNAMES = [
    "week",
    "tow",
    "iteration",
    "position_x_m",
    "position_y_m",
    "position_z_m",
    "seed_position_x_m",
    "seed_position_y_m",
    "seed_position_z_m",
    "seed_clock_m",
    "code_residual_rms_m",
    "phase_residual_rms_m",
]


def row(tow: str, *, pos_x: str, seed_x: str = "0", seed_clock: str = "10") -> dict[str, str]:
    return {
        "week": "2360",
        "tow": tow,
        "iteration": "0",
        "position_x_m": pos_x,
        "position_y_m": "0",
        "position_z_m": "0",
        "seed_position_x_m": seed_x,
        "seed_position_y_m": "0",
        "seed_position_z_m": "0",
        "seed_clock_m": seed_clock,
        "code_residual_rms_m": "2",
        "phase_residual_rms_m": "1",
    }


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


class PPPFilterLogDiffTest(unittest.TestCase):
    def test_summarizes_seed_and_position_delta_separately(self) -> None:
        base = [row("10.0", pos_x="1", seed_x="5")]
        candidate = [row("10.0", pos_x="4", seed_x="5")]

        report = diff_mod.summarize(base, candidate, top=1)

        self.assertEqual(report["matched_rows"], 1)
        self.assertAlmostEqual(report["position_delta_3d_m"]["max"], 3.0)
        self.assertAlmostEqual(report["seed_position_delta_3d_m"]["max"], 0.0)
        self.assertAlmostEqual(report["top_position_deltas"][0]["position_delta_3d_m"], 3.0)

    def test_cli_writes_json_and_top_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="ppp_filter_log_diff_test_") as temp_dir:
            root = Path(temp_dir)
            base = root / "base.csv"
            candidate = root / "candidate.csv"
            json_out = root / "summary.json"
            top_csv = root / "top.csv"
            write_csv(base, [row("10.0", pos_x="1", seed_x="5")])
            write_csv(candidate, [row("10.0", pos_x="4", seed_x="6")])

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(base),
                    str(candidate),
                    "--json-out",
                    str(json_out),
                    "--top-position-csv",
                    str(top_csv),
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            report = json.loads(json_out.read_text(encoding="utf-8"))
            self.assertEqual(report["matched_rows"], 1)
            with top_csv.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(rows[0]["tow"], "10.0")


if __name__ == "__main__":
    unittest.main()
