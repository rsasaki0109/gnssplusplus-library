#!/usr/bin/env python3
"""Tests for CLASLIB ZD component dump normalization."""

from __future__ import annotations

import csv
import importlib.util
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
EXPORT_SCRIPT = ROOT_DIR / "scripts" / "analysis" / "claslib_zd_component_export.py"
DIFF_SCRIPT = ROOT_DIR / "scripts" / "analysis" / "clas_zd_component_diff.py"

export_spec = importlib.util.spec_from_file_location("claslib_zd_component_export", EXPORT_SCRIPT)
assert export_spec is not None and export_spec.loader is not None
export = importlib.util.module_from_spec(export_spec)
sys.modules[export_spec.name] = export
export_spec.loader.exec_module(export)

diff_spec = importlib.util.spec_from_file_location("clas_zd_component_diff", DIFF_SCRIPT)
assert diff_spec is not None and diff_spec.loader is not None
component_diff = importlib.util.module_from_spec(diff_spec)
sys.modules[diff_spec.name] = component_diff
diff_spec.loader.exec_module(component_diff)


RAW_FIELDNAMES = [
    "record",
    "stage",
    "week",
    "tow",
    "row_type",
    "sat",
    "sys",
    "prn",
    "freq",
    "code",
    "raw_m",
    "corr_meas_m",
    "raw_p_m",
    "corrected_p_m",
    "prc_m",
    "cpc_m",
    "trop_m",
    "iono_l1_m",
    "iono_scaled_m",
    "cbias_m",
    "pbias_m",
    "receiver_ant_m",
    "relativity_m",
    "windup_m",
    "orb_m",
    "clk_m",
    "zd_residual_m",
    "residual_m",
    "geo_m",
    "sat_clk_m",
    "model_m",
    "az_rad",
    "el_rad",
    "rx_x_m",
    "rx_y_m",
    "rx_z_m",
]


def write_csv(path: Path, rows: list[dict[str, str]], fieldnames: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fieldnames})


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


class ClaslibZdComponentExportTest(unittest.TestCase):
    def test_normalizes_gps_l2w_code_row(self) -> None:
        with tempfile.TemporaryDirectory(prefix="claslib_zd_export_test_") as temp_dir:
            input_path = Path(temp_dir) / "claslib_raw.csv"
            output_path = Path(temp_dir) / "normalized.csv"
            write_csv(
                input_path,
                [
                    {
                        "record": "ZD",
                        "week": "2068",
                        "tow": "230420.000",
                        "row_type": "code",
                        "sat": "14",
                        "sys": "1",
                        "prn": "14",
                        "freq": "1",
                        "code": "20",
                        "raw_m": "20486467.0",
                        "corr_meas_m": "20486468.0",
                        "prc_m": "1.25",
                        "cbias_m": "0.125",
                        "receiver_ant_m": "-0.02",
                        "trop_m": "2.4",
                        "zd_residual_m": "-0.5",
                    }
                ],
                RAW_FIELDNAMES,
            )

            rows_written = export.export_csv(input_path, output_path, stage_label="post")
            self.assertEqual(rows_written, 1)
            row = read_csv(output_path)[0]
            self.assertEqual(row["stage"], "post")
            self.assertEqual(row["sat"], "G14")
            self.assertEqual(row["row_type"], "code")
            self.assertEqual(row["freq"], "1")
            self.assertEqual(row["signal"], "C2W")
            self.assertEqual(row["pseudorange_rinex_code"], "C2W")
            self.assertEqual(row["carrier_rinex_code"], "L2W")
            self.assertEqual(row["pseudorange_rtklib_code"], "20")
            self.assertEqual(row["carrier_rtklib_code"], "20")
            self.assertEqual(row["code_bias_m"], "0.125")
            self.assertEqual(row["receiver_antenna_m"], "-0.02")
            self.assertEqual(row["trop_correction_m"], "2.4")
            self.assertEqual(row["residual_m"], "-0.5")

    def test_normalizes_qzss_phase_and_galileo_code_rows(self) -> None:
        with tempfile.TemporaryDirectory(prefix="claslib_zd_export_test_") as temp_dir:
            input_path = Path(temp_dir) / "claslib_raw.csv"
            output_path = Path(temp_dir) / "normalized.csv"
            write_csv(
                input_path,
                [
                    {
                        "record": "ZD",
                        "week": "2068",
                        "tow": "230420.000",
                        "row_type": "phase",
                        "sat": "194",
                        "sys": "16",
                        "prn": "193",
                        "freq": "1",
                        "code": "18",
                        "cpc_m": "-1.5",
                        "pbias_m": "0.75",
                    },
                    {
                        "record": "CODE",
                        "stage": "pre",
                        "week": "2068",
                        "tow": "230420.000",
                        "sat": "7",
                        "sys": "8",
                        "prn": "7",
                        "freq": "2",
                        "code": "26",
                        "raw_p_m": "23000000.0",
                        "corrected_p_m": "23000001.0",
                        "cbias_m": "-0.25",
                    },
                ],
                RAW_FIELDNAMES,
            )

            export.export_csv(input_path, output_path, stage_label="post")
            rows = read_csv(output_path)
            self.assertEqual(rows[0]["stage"], "post")
            self.assertEqual(rows[0]["sat"], "J01")
            self.assertEqual(rows[0]["row_type"], "phase")
            self.assertEqual(rows[0]["signal"], "L2X")
            self.assertEqual(rows[0]["pseudorange_rinex_code"], "C2X")
            self.assertEqual(rows[0]["carrier_rinex_code"], "L2X")
            self.assertEqual(rows[0]["phase_bias_m"], "0.75")
            self.assertEqual(rows[1]["stage"], "pre")
            self.assertEqual(rows[1]["sat"], "E07")
            self.assertEqual(rows[1]["row_type"], "code")
            self.assertEqual(rows[1]["freq"], "2")
            self.assertEqual(rows[1]["signal"], "C5X")
            self.assertEqual(rows[1]["pseudorange_rinex_code"], "C5X")
            self.assertEqual(rows[1]["carrier_rinex_code"], "L5X")

    def test_exported_csv_uses_native_diff_key_space(self) -> None:
        with tempfile.TemporaryDirectory(prefix="claslib_zd_export_test_") as temp_dir:
            base_input = Path(temp_dir) / "claslib_raw.csv"
            base_csv = Path(temp_dir) / "base_normalized.csv"
            candidate_csv = Path(temp_dir) / "native.csv"
            write_csv(
                base_input,
                [
                    {
                        "record": "ZD",
                        "week": "2068",
                        "tow": "230420.000",
                        "row_type": "code",
                        "sat": "14",
                        "sys": "1",
                        "prn": "14",
                        "freq": "1",
                        "code": "20",
                        "prc_m": "1.25",
                        "cbias_m": "0.125",
                    }
                ],
                RAW_FIELDNAMES,
            )
            export.export_csv(base_input, base_csv, stage_label="post")
            write_csv(
                candidate_csv,
                [
                    {
                        "record": "CODE",
                        "stage": "post",
                        "week": "2068",
                        "tow": "230420.000",
                        "sat": "G14",
                        "row_type": "code",
                        "freq": "1",
                        "signal": "C2W",
                        "pseudorange_rinex_code": "C2W",
                        "carrier_rinex_code": "L2W",
                        "pseudorange_rtklib_code": "20",
                        "carrier_rtklib_code": "20",
                        "prc_m": "1.50",
                        "code_bias_m": "0.125",
                    }
                ],
                export.FIELDNAMES,
            )

            base_rows = component_diff.normalize_rows(
                component_diff.read_csv_rows(base_csv),
                component_names=("prc_m", "code_bias_m"),
                stage_filter="post",
                row_type_filter="code",
            )
            candidate_rows = component_diff.normalize_rows(
                component_diff.read_csv_rows(candidate_csv),
                component_names=("prc_m", "code_bias_m"),
                stage_filter="post",
                row_type_filter="code",
            )
            report = component_diff.build_report(
                base_rows,
                candidate_rows,
                base_label="claslib",
                candidate_label="native",
                threshold_m=None,
                top_deltas=10,
                top_unmatched=10,
            )
            self.assertEqual(report["common_rows"], 1)
            self.assertEqual(report["base_only_rows"], 0)
            self.assertEqual(report["candidate_only_rows"], 0)
            prc_stats = next(item for item in report["per_component"] if item["component"] == "prc_m")
            self.assertAlmostEqual(prc_stats["max_abs_delta_m"], 0.25)

    def test_cli_rejects_unknown_observation_code(self) -> None:
        with tempfile.TemporaryDirectory(prefix="claslib_zd_export_test_") as temp_dir:
            input_path = Path(temp_dir) / "claslib_raw.csv"
            output_path = Path(temp_dir) / "normalized.csv"
            write_csv(
                input_path,
                [
                    {
                        "record": "ZD",
                        "week": "2068",
                        "tow": "230420.000",
                        "row_type": "code",
                        "sat": "14",
                        "sys": "1",
                        "prn": "14",
                        "freq": "1",
                        "code": "99",
                    }
                ],
                RAW_FIELDNAMES,
            )
            result = subprocess.run(
                [sys.executable, str(EXPORT_SCRIPT), str(input_path), "--output", str(output_path)],
                check=False,
                text=True,
                capture_output=True,
            )
            self.assertEqual(result.returncode, 2)
            self.assertIn("unsupported RTKLIB observation code", result.stderr)


if __name__ == "__main__":
    unittest.main()
