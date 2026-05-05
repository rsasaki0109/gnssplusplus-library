#!/usr/bin/env python3
"""Tests for PPP correction log analysis helpers."""

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
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "ppp_correction_log_diff.py"

spec = importlib.util.spec_from_file_location("ppp_correction_log_diff", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
ppp_log = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ppp_log)


def make_row(**overrides: str) -> dict[str, str]:
    row = {
        "week": "2360",
        "tow": "172800.000",
        "sat": "G01",
        "primary_signal": "0",
        "secondary_signal": "1",
        "primary_observation_code": "C1C",
        "secondary_observation_code": "C2W",
        "primary_code_bias_coeff": "1.0",
        "secondary_code_bias_coeff": "0.0",
        "ssr_available": "1",
        "ssr_orbit_iode": "77",
        "broadcast_iode": "77",
        "orbit_clock_applied": "1",
        "orbit_clock_skip_reason": "",
        "orbit_dx_m": "0.1",
        "orbit_dy_m": "0.2",
        "orbit_dz_m": "0.3",
        "clock_m": "0.4",
        "ura_sigma_m": "0.2",
        "code_bias_m": "0.25",
        "phase_bias_m": "-0.1",
        "trop_m": "2.5",
        "stec_tecu": "5.0",
        "iono_m": "0.8",
        "ionosphere_estimation_constraint": "0",
        "dcb_applied": "0",
        "dcb_bias_m": "0.0",
        "ionex_applied": "0",
        "ionex_iono_m": "0.0",
        "atmos_token_count": "3",
        "preferred_network_id": "7",
        "elevation_deg": "45.0",
        "variance_pr": "1.2",
        "variance_cp": "0.02",
        "valid_after_corrections": "1",
        "solution_status": "3",
    }
    row.update(overrides)
    return row


def write_log(path: Path, rows: list[dict[str, str]]) -> None:
    fieldnames = [
        "week",
        "tow",
        "sat",
        "primary_signal",
        "secondary_signal",
        "primary_observation_code",
        "secondary_observation_code",
        "primary_code_bias_coeff",
        "secondary_code_bias_coeff",
        "ssr_available",
        "ssr_orbit_iode",
        "broadcast_iode",
        "orbit_clock_applied",
        "orbit_clock_skip_reason",
        "orbit_dx_m",
        "orbit_dy_m",
        "orbit_dz_m",
        "clock_m",
        "ura_sigma_m",
        "code_bias_m",
        "phase_bias_m",
        "trop_m",
        "stec_tecu",
        "iono_m",
        "ionosphere_estimation_constraint",
        "dcb_applied",
        "dcb_bias_m",
        "ionex_applied",
        "ionex_iono_m",
        "atmos_token_count",
        "preferred_network_id",
        "elevation_deg",
        "variance_pr",
        "variance_cp",
        "valid_after_corrections",
        "solution_status",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


class PppCorrectionLogAnalysisTest(unittest.TestCase):
    def test_summarize_rows_counts_correction_families(self) -> None:
        rows = [
            make_row(),
            make_row(
                tow="172830.000",
                sat="G02",
                ssr_available="1",
                orbit_clock_applied="0",
                orbit_dx_m="0.0",
                orbit_dy_m="0.0",
                orbit_dz_m="0.0",
                clock_m="0.0",
                ura_sigma_m="0.0",
                code_bias_m="0.0",
                phase_bias_m="0.0",
                trop_m="0.0",
                stec_tecu="0.0",
                iono_m="0.0",
                atmos_token_count="0",
                valid_after_corrections="0",
                ssr_orbit_iode="99",
                broadcast_iode="77",
                orbit_clock_skip_reason="ssr_orbit_iode_no_matching_ephemeris",
            ),
        ]

        summary = ppp_log.summarize_rows(rows, top_satellites=2)

        self.assertEqual(summary["rows"], 2)
        self.assertEqual(summary["epochs"], 2)
        self.assertEqual(summary["satellites"], 2)
        self.assertEqual(summary["valid_rows"], 1)
        self.assertEqual(summary["ssr_available_rows"], 2)
        self.assertEqual(summary["orbit_clock_applied_rows"], 1)
        self.assertEqual(summary["ssr_orbit_iode_rows"], 2)
        self.assertEqual(summary["orbit_clock_skip_rows"], 1)
        self.assertEqual(
            summary["orbit_clock_skip_reasons"],
            {"ssr_orbit_iode_no_matching_ephemeris": 1},
        )
        self.assertEqual(summary["atmos_token_rows"], 1)
        self.assertEqual(summary["iono_nonzero_rows"], 1)
        self.assertEqual(summary["top_satellites"][0]["sat"], "G01")
        self.assertAlmostEqual(summary["absolute_sums"]["iono_m"], 0.8)

    def test_cli_compares_logs_and_writes_json_and_diff_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="ppp_correction_log_diff_test_") as temp_dir:
            temp_root = Path(temp_dir)
            base_path = temp_root / "base.csv"
            candidate_path = temp_root / "candidate.csv"
            report_path = temp_root / "report.json"
            diff_path = temp_root / "diff.csv"
            write_log(
                base_path,
                [
                    make_row(),
                    make_row(tow="172830.000", sat="G02", iono_m="0.0", valid_after_corrections="1"),
                ],
            )
            write_log(
                candidate_path,
                [
                    make_row(iono_m="1.1", valid_after_corrections="0"),
                    make_row(tow="172830.000", sat="G02", iono_m="0.2", valid_after_corrections="1"),
                    make_row(tow="172860.000", sat="G03", iono_m="0.5"),
                ],
            )

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(base_path),
                    str(candidate_path),
                    "--json-out",
                    str(report_path),
                    "--diff-csv",
                    str(diff_path),
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("comparison:", result.stdout)
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(report["comparison"]["common_rows"], 2)
            self.assertEqual(report["comparison"]["candidate_only_rows"], 1)
            self.assertEqual(report["comparison"]["bool_change_counts"]["valid_after_corrections"], 1)
            first_difference = report["comparison"]["first_differences"][0]
            self.assertEqual(first_difference["key"], "2360:172800.000:G01:0/1:C1C/C2W")
            self.assertIn("iono_m", first_difference["columns"])

            with diff_path.open(newline="", encoding="utf-8") as handle:
                diff_rows = list(csv.DictReader(handle))
            self.assertEqual(len(diff_rows), 2)
            self.assertAlmostEqual(float(diff_rows[0]["iono_m_delta"]), 0.3)
            self.assertIn("ssr_orbit_iode_delta", diff_rows[0])
            self.assertEqual(diff_rows[0]["valid_after_corrections_changed"], "1")

    def test_cli_compares_duplicate_satellite_rows_by_observation_code(self) -> None:
        with tempfile.TemporaryDirectory(prefix="ppp_correction_log_diff_test_") as temp_dir:
            temp_root = Path(temp_dir)
            base_path = temp_root / "base.csv"
            candidate_path = temp_root / "candidate.csv"
            report_path = temp_root / "report.json"
            diff_path = temp_root / "diff.csv"
            write_log(
                base_path,
                [
                    make_row(),
                    make_row(
                        primary_signal="2",
                        secondary_signal="3",
                        primary_observation_code="C5Q",
                        secondary_observation_code="C7Q",
                    ),
                ],
            )
            write_log(
                candidate_path,
                [
                    make_row(iono_m="0.9"),
                    make_row(
                        primary_signal="2",
                        secondary_signal="3",
                        primary_observation_code="C5Q",
                        secondary_observation_code="C7Q",
                        iono_m="1.0",
                    ),
                ],
            )

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(base_path),
                    str(candidate_path),
                    "--json-out",
                    str(report_path),
                    "--diff-csv",
                    str(diff_path),
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(report["comparison"]["common_rows"], 2)
            keys = {item["key"] for item in report["comparison"]["first_differences"]}
            self.assertIn("2360:172800.000:G01:0/1:C1C/C2W", keys)
            self.assertIn("2360:172800.000:G01:2/3:C5Q/C7Q", keys)

            with diff_path.open(newline="", encoding="utf-8") as handle:
                diff_rows = list(csv.DictReader(handle))
            self.assertEqual(len(diff_rows), 2)
            self.assertEqual(
                {
                    (
                        row["primary_signal"],
                        row["secondary_signal"],
                        row["primary_observation_code"],
                        row["secondary_observation_code"],
                    )
                    for row in diff_rows
                },
                {("0", "1", "C1C", "C2W"), ("2", "3", "C5Q", "C7Q")},
            )


if __name__ == "__main__":
    unittest.main()
