#!/usr/bin/env python3
"""Tests for per-satellite PPP residual diff helpers."""

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
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "ppp_per_sat_residual_diff.py"

spec = importlib.util.spec_from_file_location("ppp_per_sat_residual_diff", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
residual_diff = importlib.util.module_from_spec(spec)
spec.loader.exec_module(residual_diff)


FIELDNAMES = [
    "week",
    "tow",
    "iteration",
    "sat",
    "row_type",
    "residual_m",
    "frequency_index",
    "primary_signal",
    "secondary_signal",
    "primary_observation_code",
    "secondary_observation_code",
]


def residual_row(
    *,
    tow: str = "10.000",
    iteration: str = "0",
    sat: str,
    row_type: str = "phase",
    residual_m: str,
    frequency_index: str = "0",
    primary_signal: str = "0",
    secondary_signal: str = "",
    primary_code: str = "L1C",
    secondary_code: str = "",
) -> dict[str, str]:
    return {
        "week": "2360",
        "tow": tow,
        "iteration": iteration,
        "sat": sat,
        "row_type": row_type,
        "residual_m": residual_m,
        "frequency_index": frequency_index,
        "primary_signal": primary_signal,
        "secondary_signal": secondary_signal,
        "primary_observation_code": primary_code,
        "secondary_observation_code": secondary_code,
    }


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


class PppPerSatResidualDiffTest(unittest.TestCase):
    def test_groups_matched_residual_deltas_by_satellite_and_frequency(self) -> None:
        base_rows = [
            residual_row(sat="G01", residual_m="1.0"),
            residual_row(sat="G01", tow="11.000", residual_m="2.0"),
            residual_row(sat="G02", residual_m="-1.0", frequency_index="1"),
            residual_row(sat="G03", row_type="code", residual_m="100.0"),
        ]
        candidate_rows = [
            residual_row(sat="G01", residual_m="1.5"),
            residual_row(sat="G01", tow="11.000", residual_m="3.5"),
            residual_row(sat="G02", residual_m="-2.0", frequency_index="1"),
            residual_row(sat="G04", residual_m="4.0"),
        ]

        report = residual_diff.summarize_residual_diff(
            base_rows,
            candidate_rows,
            row_type="phase",
            iteration=0,
            top=2,
        )

        self.assertEqual(report["base_rows"], 3)
        self.assertEqual(report["candidate_rows"], 4)
        self.assertEqual(report["matched_rows"], 3)
        self.assertEqual(report["base_only_rows"], 0)
        self.assertEqual(report["candidate_only_rows"], 1)
        self.assertAlmostEqual(report["overall"]["mean"], (0.5 + 1.5 - 1.0) / 3.0)
        self.assertEqual(report["top_deltas"][0]["sat"], "G01")
        self.assertAlmostEqual(report["top_deltas"][0]["delta_m"], 1.5)

        g01 = next(group for group in report["groups"] if group["sat"] == "G01")
        self.assertEqual(g01["rows"], 2)
        self.assertEqual(g01["frequency_index"], 0)
        self.assertAlmostEqual(g01["mean_delta_m"], 1.0)
        self.assertAlmostEqual(g01["rms_delta_m"], (0.5 * 0.5 + 1.5 * 1.5) ** 0.5 / 2 ** 0.5)

    def test_filters_iteration_and_tow_window(self) -> None:
        base_rows = [
            residual_row(sat="G01", tow="9.000", residual_m="1.0"),
            residual_row(sat="G01", tow="10.000", residual_m="1.0"),
            residual_row(sat="G01", tow="10.000", iteration="1", residual_m="10.0"),
        ]
        candidate_rows = [
            residual_row(sat="G01", tow="9.000", residual_m="2.0"),
            residual_row(sat="G01", tow="10.000", residual_m="3.0"),
            residual_row(sat="G01", tow="10.000", iteration="1", residual_m="30.0"),
        ]

        report = residual_diff.summarize_residual_diff(
            base_rows,
            candidate_rows,
            row_type="phase",
            iteration=0,
            tow_min=9.5,
            tow_max=10.5,
        )

        self.assertEqual(report["compared_rows"], 1)
        self.assertAlmostEqual(report["overall"]["mean"], 2.0)

    def test_cli_writes_json_and_group_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="ppp_per_sat_residual_diff_test_") as temp_dir:
            temp_root = Path(temp_dir)
            base_path = temp_root / "base.csv"
            candidate_path = temp_root / "candidate.csv"
            report_path = temp_root / "report.json"
            groups_path = temp_root / "groups.csv"
            write_csv(base_path, [residual_row(sat="G01", residual_m="1.0")])
            write_csv(candidate_path, [residual_row(sat="G01", residual_m="1.25")])

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(base_path),
                    str(candidate_path),
                    "--json-out",
                    str(report_path),
                    "--groups-csv",
                    str(groups_path),
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(report["compared_rows"], 1)
            self.assertAlmostEqual(report["overall"]["mean"], 0.25)
            with groups_path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(rows[0]["sat"], "G01")
            self.assertEqual(rows[0]["rows"], "1")


if __name__ == "__main__":
    unittest.main()
