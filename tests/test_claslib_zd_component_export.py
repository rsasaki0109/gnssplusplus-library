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
    "stec_tecu",
    "iono_scaled_m",
    "iono_scale",
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

OSR_FIELDNAMES = [
    "msg",
    "tow",
    "sys",
    "prn",
    "pbias1",
    "pbias2",
    "pbias5",
    "cbias1",
    "cbias2",
    "cbias5",
    "trop",
    "iono",
    "antr1",
    "antr2",
    "antr5",
    "relatv",
    "wup1",
    "wup2",
    "wup5",
    "compI1",
    "compI2",
    "compI5",
    "compN",
    "CPC1",
    "CPC2",
    "CPC5",
    "PRC1",
    "PRC2",
    "PRC5",
    "orb",
    "clk",
    "lat",
    "lon",
    "alt",
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

    def test_normalizes_claslib_osrres_gps_l2w_rows(self) -> None:
        with tempfile.TemporaryDirectory(prefix="claslib_zd_export_test_") as temp_dir:
            input_path = Path(temp_dir) / "claslib.osr"
            output_path = Path(temp_dir) / "normalized.csv"
            write_csv(
                input_path,
                [
                    {
                        "msg": "OSRRES(ch0)",
                        "tow": "230420.0",
                        "sys": "1",
                        "prn": "14",
                        "pbias1": "0.101",
                        "pbias2": "0.202",
                        "pbias5": "0.505",
                        "cbias1": "0.011",
                        "cbias2": "0.022",
                        "cbias5": "0.055",
                        "trop": "2.345",
                        "iono": "1.234",
                        "antr1": "-0.010",
                        "antr2": "-0.020",
                        "antr5": "-0.050",
                        "relatv": "0.003",
                        "wup1": "0.100",
                        "wup2": "0.200",
                        "wup5": "0.500",
                        "compI1": "0.001",
                        "compI2": "0.002",
                        "compI5": "0.005",
                        "compN": "0.000",
                        "CPC1": "3.101",
                        "CPC2": "3.202",
                        "CPC5": "3.505",
                        "PRC1": "4.101",
                        "PRC2": "4.202",
                        "PRC5": "4.505",
                        "orb": "-0.321",
                        "clk": "0.654",
                        "lat": "35.0",
                        "lon": "139.0",
                        "alt": "10.0",
                    }
                ],
                OSR_FIELDNAMES,
            )

            rows_written = export.export_csv(input_path, output_path, stage_label="post", gps_week=2068)
            self.assertEqual(rows_written, 6)
            rows = read_csv(output_path)
            code_l1c = next(row for row in rows if row["row_type"] == "code" and row["signal"] == "C1C")
            code_l2w = next(row for row in rows if row["row_type"] == "code" and row["signal"] == "C2W")
            phase_l2w = next(row for row in rows if row["row_type"] == "phase" and row["signal"] == "L2W")
            l2w_iono_l1 = (4.202 - (2.345 - 0.020 + 0.003 + 0.022)) / export.GPS_IONO_SCALE_BY_SUFFIX["2"]
            self.assertEqual(code_l2w["stage"], "post")
            self.assertEqual(code_l2w["week"], "2068")
            self.assertEqual(code_l2w["tow"], "230420.0")
            self.assertEqual(code_l2w["sat"], "G14")
            self.assertEqual(code_l2w["freq"], "1")
            self.assertEqual(code_l2w["pseudorange_rtklib_code"], "20")
            self.assertEqual(code_l2w["carrier_rtklib_code"], "20")
            self.assertEqual(code_l2w["applied_pr_corr_m"], "4.202")
            self.assertEqual(code_l2w["prc_m"], "4.202")
            self.assertEqual(code_l2w["code_bias_m"], "0.022")
            self.assertEqual(code_l2w["trop_correction_m"], "2.345")
            self.assertAlmostEqual(float(code_l1c["iono_l1_m"]), 1.752)
            self.assertAlmostEqual(float(code_l2w["iono_l1_m"]), l2w_iono_l1)
            self.assertAlmostEqual(
                float(code_l2w["stec_tecu"]),
                l2w_iono_l1 / export.GPS_L1_TECU_TO_METERS,
            )
            self.assertAlmostEqual(float(code_l2w["iono_scaled_m"]), 1.852)
            self.assertAlmostEqual(
                float(code_l2w["iono_scale"]),
                export.GPS_IONO_SCALE_BY_SUFFIX["2"],
            )
            self.assertAlmostEqual(
                float(code_l2w["iono_l1_m"]) * export.GPS_IONO_SCALE_BY_SUFFIX["2"],
                float(code_l2w["iono_scaled_m"]),
            )
            self.assertEqual(code_l2w["receiver_antenna_m"], "-0.020")
            self.assertEqual(code_l2w["orbit_projection_m"], "-0.321")
            self.assertEqual(code_l2w["clock_correction_m"], "0.654")
            self.assertEqual(phase_l2w["week"], "2068")
            self.assertEqual(phase_l2w["freq"], "1")
            self.assertEqual(phase_l2w["carrier_correction_m"], "3.202")
            self.assertEqual(phase_l2w["cpc_m"], "3.202")
            self.assertEqual(phase_l2w["phase_bias_m"], "0.202")
            self.assertEqual(phase_l2w["phase_compensation_m"], "0.002")
            self.assertEqual(phase_l2w["windup_m"], "0.200")

    def test_claslib_osrres_requires_gps_week(self) -> None:
        with tempfile.TemporaryDirectory(prefix="claslib_zd_export_test_") as temp_dir:
            input_path = Path(temp_dir) / "claslib.osr"
            output_path = Path(temp_dir) / "normalized.csv"
            write_csv(
                input_path,
                [
                    {
                        "msg": "OSRRES(ch0)",
                        "tow": "230420.0",
                        "sys": "1",
                        "prn": "14",
                        "pbias2": "0.202",
                        "cbias2": "0.022",
                        "CPC2": "3.202",
                        "PRC2": "4.202",
                    }
                ],
                OSR_FIELDNAMES,
            )

            with self.assertRaisesRegex(export.ExportError, "pass --gps-week"):
                export.export_csv(input_path, output_path, stage_label="post")

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
