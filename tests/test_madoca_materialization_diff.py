#!/usr/bin/env python3
"""Tests for MADOCA materialization snapshot diff helpers."""

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
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "madoca_materialization_diff.py"

spec = importlib.util.spec_from_file_location("madoca_materialization_diff", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
materialization_diff = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = materialization_diff
spec.loader.exec_module(materialization_diff)


FIELDNAMES = [
    "schema_version",
    "sat",
    "system",
    "prn",
    "week",
    "tow",
    "orbit_frame",
    "orbit_valid",
    "clock_valid",
    "code_bias_valid",
    "phase_bias_valid",
    "orbit_week",
    "orbit_tow",
    "clock_week",
    "clock_tow",
    "iode",
    "ssr_orbit_iod",
    "ssr_clock_iod",
    "orbit_radial_m",
    "orbit_along_m",
    "orbit_cross_m",
    "clock_m",
    "code_bias_count",
    "code_biases_m",
    "phase_bias_count",
    "phase_biases_m",
    "phase_bias_discnt",
]


def snapshot_row(
    *,
    sat: str,
    system: str = "GPS",
    prn: str = "14",
    tow: str = "123.500",
    iode: str = "7",
    clock_m: str = "0.25",
    code_biases_m: str = "9:0.125",
    phase_biases_m: str = "9:-0.5",
    phase_bias_discnt: str = "9:4",
) -> dict[str, str]:
    return {
        "schema_version": "madoca_materialization_snapshot.v1",
        "sat": sat,
        "system": system,
        "prn": prn,
        "week": "2299",
        "tow": tow,
        "orbit_frame": "rac",
        "orbit_valid": "1",
        "clock_valid": "1",
        "code_bias_valid": "1",
        "phase_bias_valid": "1",
        "orbit_week": "2299",
        "orbit_tow": "120.0",
        "clock_week": "2299",
        "clock_tow": "121.0",
        "iode": iode,
        "ssr_orbit_iod": "8",
        "ssr_clock_iod": "9",
        "orbit_radial_m": "1.0",
        "orbit_along_m": "2.0",
        "orbit_cross_m": "3.0",
        "clock_m": clock_m,
        "code_bias_count": str(len(code_biases_m.split(";")) if code_biases_m else 0),
        "code_biases_m": code_biases_m,
        "phase_bias_count": str(len(phase_biases_m.split(";")) if phase_biases_m else 0),
        "phase_biases_m": phase_biases_m,
        "phase_bias_discnt": phase_bias_discnt,
    }


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


class MadocaMaterializationDiffTest(unittest.TestCase):
    def test_compares_numeric_and_discrete_materialization_fields(self) -> None:
        base_rows = [
            snapshot_row(sat="G14"),
            snapshot_row(sat="E11", system="Galileo", prn="11", tow="124.000"),
        ]
        candidate_rows = [
            snapshot_row(
                sat="G14",
                iode="10",
                clock_m="0.35",
                code_biases_m="9:0.425",
                phase_bias_discnt="9:5",
            ),
            snapshot_row(sat="R03", system="GLONASS", prn="3", tow="124.000"),
        ]

        base = materialization_diff.normalize_rows(base_rows)
        candidate = materialization_diff.normalize_rows(candidate_rows)
        report = materialization_diff.build_report(
            base,
            candidate,
            base_label="oracle",
            candidate_label="native",
            threshold=0.1,
            top_deltas=10,
            top_unmatched=10,
            top_mismatches=10,
        )

        self.assertEqual(report["schema"], "madoca_materialization_diff.v1")
        self.assertEqual(report["base_rows"], 2)
        self.assertEqual(report["candidate_rows"], 2)
        self.assertEqual(report["common_rows"], 1)
        self.assertEqual(report["base_only_rows"], 1)
        self.assertEqual(report["candidate_only_rows"], 1)
        self.assertFalse(report["row_set_complete"])
        self.assertEqual(report["discrete_mismatches"], 2)
        self.assertEqual(report["numeric_threshold_exceedances"], 1)
        self.assertEqual(report["top_base_only"][0]["sat"], "E11")
        self.assertEqual(report["top_candidate_only"][0]["sat"], "R03")
        self.assertEqual(report["top_numeric_deltas"][0]["component"], "code_bias_m[9]")
        self.assertAlmostEqual(report["top_numeric_deltas"][0]["delta"], 0.3)
        fields = {item["field"] for item in report["top_discrete_mismatches"]}
        self.assertEqual(fields, {"iode", "phase_bias_discnt"})

    def test_duplicate_occurrences_are_matched_in_order(self) -> None:
        base = materialization_diff.normalize_rows(
            [
                snapshot_row(sat="G14", clock_m="0.1"),
                snapshot_row(sat="G14", clock_m="0.2"),
            ]
        )
        candidate = materialization_diff.normalize_rows(
            [
                snapshot_row(sat="G14", clock_m="0.1"),
                snapshot_row(sat="G14", clock_m="0.3"),
            ]
        )

        report = materialization_diff.build_report(
            base,
            candidate,
            base_label="oracle",
            candidate_label="native",
            threshold=0.05,
            top_deltas=10,
            top_unmatched=10,
            top_mismatches=10,
        )

        self.assertEqual(report["common_rows"], 2)
        self.assertEqual(report["base_only_rows"], 0)
        self.assertEqual(report["candidate_only_rows"], 0)
        self.assertEqual(report["numeric_threshold_exceedances"], 1)
        self.assertEqual(report["top_numeric_deltas"][0]["occurrence"], 1)
        self.assertEqual(report["top_numeric_deltas"][0]["component"], "clock_m")

    def test_cli_writes_schema_json_and_details_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="madoca_materialization_diff_test_") as temp_dir:
            root = Path(temp_dir)
            base_path = root / "oracle.csv"
            candidate_path = root / "native.csv"
            report_path = root / "report.json"
            details_path = root / "details.csv"
            write_csv(base_path, [snapshot_row(sat="G14", clock_m="0.1")])
            write_csv(candidate_path, [snapshot_row(sat="G14", clock_m="0.25")])

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(base_path),
                    str(candidate_path),
                    "--numeric-threshold",
                    "0.1",
                    "--json-out",
                    str(report_path),
                    "--details-csv",
                    str(details_path),
                    "--fail-on-diff",
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 2, msg=result.stderr)
            self.assertIn("madoca_materialization_diff:", result.stdout)
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(report["schema"], "madoca_materialization_diff.v1")
            self.assertEqual(report["numeric_threshold_exceedances"], 1)
            with details_path.open(newline="", encoding="utf-8") as details_file:
                rows = list(csv.DictReader(details_file))
            self.assertEqual(rows[0]["kind"], "numeric")
            self.assertEqual(rows[0]["sat"], "G14")
            self.assertEqual(rows[0]["name"], "clock_m")


if __name__ == "__main__":
    unittest.main()
