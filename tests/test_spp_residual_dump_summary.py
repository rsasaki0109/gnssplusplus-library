#!/usr/bin/env python3
"""Tests for SPP final residual dump summaries."""

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
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "spp_residual_dump_summary.py"

spec = importlib.util.spec_from_file_location("spp_residual_dump_summary", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
summary_mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(summary_mod)


FIELDNAMES = [
    "week",
    "tow",
    "sat",
    "signal",
    "obs_code",
    "ionosphere_free_code",
    "raw_pseudorange_m",
    "corrected_pseudorange_m",
    "primary_signal",
    "primary_obs_code",
    "secondary_signal",
    "secondary_obs_code",
    "primary_pseudorange_m",
    "secondary_pseudorange_m",
    "iflc_primary_coeff",
    "iflc_secondary_coeff",
    "primary_frequency_hz",
    "secondary_frequency_hz",
    "sat_clock_correction_m",
    "troposphere_delay_m",
    "ionosphere_delay_m",
    "group_delay_m",
    "ephemeris_variance_m2",
    "variance_m2",
    "sat_x_m",
    "sat_y_m",
    "sat_z_m",
    "elevation_deg",
    "weight",
    "residual_m",
    "geometric_range_m",
    "predicted_pseudorange_m",
    "clock_group",
    "reference_clock_group",
    "receiver_clock_bias_m",
    "system_bias_m",
    "pos_x_m",
    "pos_y_m",
    "pos_z_m",
    "num_measurements",
    "residual_rms_m",
]


def residual_row(*, sat: str, signal: str, residual: str, iono: str = "1.0") -> dict[str, str]:
    row = {name: "0" for name in FIELDNAMES}
    row.update(
        {
            "week": "2360",
            "tow": "10.000",
            "sat": sat,
            "signal": signal,
            "corrected_pseudorange_m": "20000000",
            "sat_clock_correction_m": "100",
            "troposphere_delay_m": "2.0",
            "ionosphere_delay_m": iono,
            "group_delay_m": "0.5",
            "ephemeris_variance_m2": "4.0",
            "variance_m2": "9.0",
            "elevation_deg": "30.0",
            "weight": "0.111111",
            "residual_m": residual,
            "residual_rms_m": "3.0",
        }
    )
    return row


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


class SppResidualDumpSummaryTest(unittest.TestCase):
    def test_groups_by_satellite_signal(self) -> None:
        rows = [
            residual_row(sat="G01", signal="GPS_L1CA", residual="1.0"),
            residual_row(sat="G01", signal="GPS_L1CA", residual="-3.0"),
            residual_row(sat="G02", signal="GPS_L1CA", residual="5.0", iono="2.0"),
        ]

        report = summary_mod.summarize(rows, top=2)

        self.assertEqual(report["rows"], 3)
        self.assertEqual(report["groups"][0]["sat"], "G02")
        self.assertAlmostEqual(report["groups"][0]["mean_iono_delay_m"], 2.0)
        self.assertEqual(report["top_residual_rows"][0]["sat"], "G02")

    def test_cli_writes_json_and_groups_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="spp_residual_summary_test_") as temp_dir:
            root = Path(temp_dir)
            dump = root / "spp.csv"
            json_out = root / "summary.json"
            groups_csv = root / "groups.csv"
            write_csv(dump, [residual_row(sat="G01", signal="GPS_L1CA", residual="2.5")])

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(dump),
                    "--json-out",
                    str(json_out),
                    "--groups-csv",
                    str(groups_csv),
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            report = json.loads(json_out.read_text(encoding="utf-8"))
            self.assertEqual(report["rows"], 1)
            with groups_csv.open(newline="", encoding="utf-8") as handle:
                groups = list(csv.DictReader(handle))
            self.assertEqual(groups[0]["sat"], "G01")


if __name__ == "__main__":
    unittest.main()
