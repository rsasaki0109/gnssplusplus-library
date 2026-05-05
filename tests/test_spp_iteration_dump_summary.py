#!/usr/bin/env python3
"""Tests for SPP iteration dump summaries."""

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
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "spp_iteration_dump_summary.py"

spec = importlib.util.spec_from_file_location("spp_iteration_dump_summary", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
summary_mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(summary_mod)


FIELDNAMES = [
    "week",
    "tow",
    "iteration",
    "sat",
    "signal",
    "residual_m",
    "residual_rms_m",
    "dx_norm_m",
    "rejected_after",
    "abs_residual_gt_threshold",
]


def row(
    *,
    tow: str = "10.000",
    iteration: str = "0",
    sat: str,
    signal: str = "GPS_L1CA",
    residual_m: str,
    residual_rms_m: str = "5.0",
    dx_norm_m: str = "1.0",
    rejected_after: str = "0",
    over: str = "0",
) -> dict[str, str]:
    return {
        "week": "2360",
        "tow": tow,
        "iteration": iteration,
        "sat": sat,
        "signal": signal,
        "residual_m": residual_m,
        "residual_rms_m": residual_rms_m,
        "dx_norm_m": dx_norm_m,
        "rejected_after": rejected_after,
        "abs_residual_gt_threshold": over,
    }


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


class SppIterationDumpSummaryTest(unittest.TestCase):
    def test_summarizes_top_satellites_and_residual_rows(self) -> None:
        rows = [
            row(sat="G01", residual_m="1.0"),
            row(sat="G02", residual_m="-30.0", rejected_after="1", over="1", residual_rms_m="20"),
            row(sat="G02", tow="11.000", iteration="1", residual_m="-10.0", over="1"),
        ]

        report = summary_mod.summarize(rows, top=2)

        self.assertEqual(report["rows"], 3)
        self.assertEqual(report["epochs"], 2)
        self.assertEqual(report["worst_epoch_by_rms"]["tow"], 10.0)
        self.assertEqual(report["satellites"][0]["sat"], "G02")
        self.assertEqual(report["satellites"][0]["rejected_rows"], 1)
        self.assertEqual(report["top_residual_rows"][0]["sat"], "G02")
        self.assertAlmostEqual(report["top_residual_rows"][0]["residual_m"], -30.0)

    def test_can_select_last_iteration_per_epoch(self) -> None:
        rows = [
            row(sat="G01", tow="10.000", iteration="0", residual_m="100.0"),
            row(sat="G01", tow="10.000", iteration="1", residual_m="2.0"),
            row(sat="G02", tow="11.000", iteration="0", residual_m="-50.0"),
        ]

        report = summary_mod.summarize(rows, top=3, last_iteration_per_epoch=True)

        self.assertTrue(report["last_iteration_per_epoch"])
        self.assertEqual(report["rows"], 2)
        self.assertEqual(report["top_residual_rows"][0]["sat"], "G02")
        self.assertAlmostEqual(report["top_residual_rows"][1]["residual_m"], 2.0)

    def test_cli_writes_json(self) -> None:
        with tempfile.TemporaryDirectory(prefix="spp_iteration_summary_test_") as temp_dir:
            temp_root = Path(temp_dir)
            dump_path = temp_root / "spp.csv"
            json_path = temp_root / "summary.json"
            write_csv(dump_path, [row(sat="G01", residual_m="2.5")])

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(dump_path),
                    "--json-out",
                    str(json_path),
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            report = json.loads(json_path.read_text(encoding="utf-8"))
            self.assertEqual(report["rows"], 1)
            self.assertIn('"rows": 1', result.stdout)


if __name__ == "__main__":
    unittest.main()
