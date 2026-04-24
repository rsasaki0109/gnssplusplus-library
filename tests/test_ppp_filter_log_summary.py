#!/usr/bin/env python3
"""Tests for PPP filter/residual log summary helpers."""

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
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "ppp_filter_log_summary.py"

spec = importlib.util.spec_from_file_location("ppp_filter_log_summary", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
filter_summary = importlib.util.module_from_spec(spec)
spec.loader.exec_module(filter_summary)


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


class PppFilterLogSummaryTest(unittest.TestCase):
    def test_summarizes_filter_row_shapes(self) -> None:
        rows = [
            {
                "week": "2360",
                "tow": "172800.000",
                "iteration": "0",
                "rows": "24",
                "code_rows": "12",
                "phase_rows": "6",
                "ionosphere_constraint_rows": "6",
                "code_residual_rms_m": "2.5",
                "code_residual_max_abs_m": "8.0",
                "phase_residual_rms_m": "0.2",
                "phase_residual_max_abs_m": "0.5",
            },
            {
                "week": "2360",
                "tow": "172830.000",
                "iteration": "1",
                "rows": "30",
                "code_rows": "12",
                "phase_rows": "12",
                "ionosphere_constraint_rows": "6",
                "code_residual_rms_m": "1.5",
                "code_residual_max_abs_m": "4.0",
                "phase_residual_rms_m": "0.4",
                "phase_residual_max_abs_m": "0.7",
            },
            {
                "week": "2360",
                "tow": "172860.000",
                "iteration": "1",
                "rows": "30",
                "code_rows": "12",
                "phase_rows": "12",
                "ionosphere_constraint_rows": "6",
                "code_residual_rms_m": "1.0",
                "code_residual_max_abs_m": "2.0",
                "phase_residual_rms_m": "0.1",
                "phase_residual_max_abs_m": "0.3",
            },
        ]

        summary = filter_summary.summarize_filter_rows(rows)

        self.assertEqual(summary["rows"], 3)
        self.assertEqual(summary["epochs"], 3)
        self.assertEqual(summary["max_rows"], 30)
        self.assertEqual(summary["max_code_rows"], 12)
        self.assertEqual(summary["max_phase_rows"], 12)
        self.assertEqual(summary["max_ionosphere_constraint_rows"], 6)
        self.assertEqual(summary["row_shapes"][0]["phase_rows"], 12)
        self.assertEqual(summary["row_shapes"][0]["count"], 2)
        self.assertAlmostEqual(summary["max_code_residual_abs_m"], 8.0)

    def test_compares_filter_logs_by_epoch_iteration(self) -> None:
        base_rows = [
            {
                "week": "2360",
                "tow": "172800.000",
                "iteration": "0",
                "rows": "64",
                "code_rows": "54",
                "phase_rows": "0",
                "ionosphere_constraint_rows": "10",
                "pos_delta_m": "4.0",
                "position_x_m": "100.0",
            },
            {
                "week": "2360",
                "tow": "172830.000",
                "iteration": "0",
                "rows": "108",
                "code_rows": "54",
                "phase_rows": "30",
                "ionosphere_constraint_rows": "10",
                "pos_delta_m": "1.0",
                "position_x_m": "110.0",
            },
            {
                "week": "2360",
                "tow": "172860.000",
                "iteration": "0",
                "rows": "108",
                "code_rows": "54",
                "phase_rows": "30",
                "ionosphere_constraint_rows": "10",
                "pos_delta_m": "0.5",
                "position_x_m": "120.0",
            },
        ]
        candidate_rows = [
            {
                "week": "2360",
                "tow": "172800.000",
                "iteration": "0",
                "rows": "100",
                "code_rows": "54",
                "phase_rows": "36",
                "ionosphere_constraint_rows": "10",
                "pos_delta_m": "5.5",
                "position_x_m": "102.0",
            },
            {
                "week": "2360",
                "tow": "172830.000",
                "iteration": "0",
                "rows": "108",
                "code_rows": "54",
                "phase_rows": "30",
                "ionosphere_constraint_rows": "10",
                "pos_delta_m": "1.0",
                "position_x_m": "111.5",
            },
            {
                "week": "2360",
                "tow": "172890.000",
                "iteration": "0",
                "rows": "108",
                "code_rows": "54",
                "phase_rows": "30",
                "ionosphere_constraint_rows": "10",
                "pos_delta_m": "0.4",
                "position_x_m": "130.0",
            },
        ]

        comparison = filter_summary.summarize_filter_comparison(
            base_rows,
            candidate_rows,
            top_rows=5,
            top_columns=5,
        )

        self.assertEqual(comparison["common_rows"], 2)
        self.assertEqual(comparison["base_only_rows"], 1)
        self.assertEqual(comparison["candidate_only_rows"], 1)
        self.assertEqual(comparison["changed_rows"], 2)
        self.assertEqual(comparison["unchanged_rows"], 0)
        first_changed = comparison["first_changed_rows"][0]
        self.assertEqual(first_changed["week"], 2360)
        self.assertAlmostEqual(first_changed["tow"], 172800.0)
        first_delta_by_column = {item["column"]: item for item in first_changed["top_deltas"]}
        self.assertAlmostEqual(first_delta_by_column["rows"]["delta"], 36.0)
        self.assertAlmostEqual(first_delta_by_column["phase_rows"]["delta"], 36.0)
        max_by_column = {item["column"]: item for item in comparison["max_column_deltas"]}
        self.assertAlmostEqual(max_by_column["position_x_m"]["delta"], 2.0)
        self.assertEqual(comparison["first_base_only_rows"], [{"week": 2360, "tow": 172860.0, "iteration": 0}])
        self.assertEqual(comparison["first_candidate_only_rows"], [{"week": 2360, "tow": 172890.0, "iteration": 0}])

    def test_summarizes_residual_types_and_iono_rows(self) -> None:
        rows = [
            {
                "week": "2360",
                "tow": "172800.000",
                "iteration": "0",
                "row_index": "0",
                "sat": "G01",
                "row_type": "code",
                "residual_m": "1.2",
                "iono_state_m": "0.5",
            },
            {
                "week": "2360",
                "tow": "172800.000",
                "iteration": "0",
                "row_index": "1",
                "sat": "G01",
                "row_type": "phase",
                "residual_m": "-0.3",
                "iono_state_m": "0.5",
            },
            {
                "week": "2360",
                "tow": "172830.000",
                "iteration": "0",
                "row_index": "0",
                "sat": "E11",
                "row_type": "code",
                "residual_m": "-5.0",
                "iono_state_m": "0.0",
                "primary_signal": "9",
                "secondary_signal": "10",
                "primary_observation_code": "C1C",
                "secondary_observation_code": "C5Q",
                "frequency_index": "0",
                "ionosphere_coefficient": "1.0",
            },
            {
                "week": "2360",
                "tow": "172830.000",
                "iteration": "1",
                "row_index": "0",
                "sat": "G02",
                "row_type": "phase",
                "residual_m": "0.7",
                "iono_state_m": "0.0",
                "primary_signal": "3",
                "secondary_signal": "21",
                "primary_observation_code": "C2W",
                "secondary_observation_code": "",
                "frequency_index": "1",
                "ionosphere_coefficient": "1.646944",
            },
        ]

        summary = filter_summary.summarize_residual_rows(rows)

        self.assertEqual(summary["rows"], 4)
        self.assertEqual(summary["row_type_counts"], {"code": 2, "phase": 2})
        self.assertEqual(summary["nonzero_iono_state_rows_by_type"], {"code": 1, "phase": 1})
        self.assertAlmostEqual(summary["max_abs_residual_m_by_type"]["phase"], 0.7)
        self.assertEqual(summary["top_satellites_by_type"]["phase"][0]["sat"], "G01")
        top_code_satellite = summary["top_residual_satellites_by_type"]["code"][0]
        self.assertEqual(top_code_satellite["sat"], "E11")
        self.assertEqual(top_code_satellite["max_abs_primary_observation_code"], "C1C")
        self.assertEqual(top_code_satellite["max_abs_secondary_observation_code"], "C5Q")
        top_code_epoch = summary["top_residual_epochs_by_type"]["code"][0]
        self.assertAlmostEqual(top_code_epoch["max_abs_residual_m"], 5.0)
        self.assertEqual(top_code_epoch["primary_observation_code"], "C1C")
        final_summary = summary["final_iteration"]
        self.assertEqual(final_summary["rows"], 3)
        self.assertEqual(final_summary["row_type_counts"], {"code": 1, "phase": 2})
        self.assertAlmostEqual(final_summary["max_abs_residual_m_by_type"]["code"], 1.2)
        top_final_phase = final_summary["top_residual_satellites_by_type"]["phase"][0]
        self.assertEqual(top_final_phase["sat"], "G02")
        self.assertEqual(top_final_phase["max_abs_frequency_index"], 1)
        self.assertAlmostEqual(top_final_phase["max_abs_ionosphere_coefficient"], 1.646944)


    def test_joins_residual_rankings_to_correction_log(self) -> None:
        residual_rows = [
            {
                "week": "2360",
                "tow": "172830.000",
                "iteration": "1",
                "row_index": "0",
                "sat": "G02",
                "row_type": "code",
                "residual_m": "-4.5",
                "iono_state_m": "0.3",
                "primary_signal": "3",
                "secondary_signal": "21",
                "primary_observation_code": "C2W",
                "secondary_observation_code": "",
                "frequency_index": "1",
                "ionosphere_coefficient": "1.646944",
            },
            {
                "week": "2360",
                "tow": "172830.000",
                "iteration": "1",
                "row_index": "1",
                "sat": "G03",
                "row_type": "code",
                "residual_m": "1.0",
                "iono_state_m": "0.0",
                "primary_signal": "0",
                "secondary_signal": "2",
                "primary_observation_code": "C1C",
                "secondary_observation_code": "C2W",
                "frequency_index": "0",
                "ionosphere_coefficient": "1.0",
            },
        ]
        correction_rows = [
            {
                "week": "2360",
                "tow": "172830.000",
                "sat": "G02",
                "primary_signal": "3",
                "secondary_signal": "21",
                "primary_observation_code": "C2W",
                "secondary_observation_code": "",
                "ssr_available": "1",
                "orbit_clock_applied": "1",
                "clock_m": "0.25",
                "code_bias_m": "3.75",
                "phase_bias_m": "-0.5",
                "variance_pr": "0.42",
                "variance_cp": "0.0005",
                "valid_after_corrections": "1",
            }
        ]

        correction_index = filter_summary.index_correction_rows(correction_rows)
        summary = filter_summary.summarize_residual_rows(
            residual_rows,
            correction_index=correction_index,
        )

        self.assertEqual(summary["correction_join"]["matched_rows"], 1)
        self.assertEqual(summary["correction_join"]["unmatched_rows"], 1)
        top_code = summary["top_residual_satellites_by_type"]["code"][0]
        self.assertEqual(top_code["sat"], "G02")
        correction = top_code["max_abs_correction"]
        self.assertTrue(correction["matched"])
        self.assertEqual(correction["ssr_available"], 1)
        self.assertAlmostEqual(correction["code_bias_m"], 3.75)
        self.assertAlmostEqual(correction["variance_pr"], 0.42)

    def test_cli_writes_json_report(self) -> None:
        with tempfile.TemporaryDirectory(prefix="ppp_filter_log_summary_test_") as temp_dir:
            temp_root = Path(temp_dir)
            filter_path = temp_root / "filter.csv"
            residual_path = temp_root / "residual.csv"
            correction_path = temp_root / "correction.csv"
            report_path = temp_root / "report.json"
            candidate_filter_path = temp_root / "candidate_filter.csv"
            filter_fieldnames = [
                "week",
                "tow",
                "iteration",
                "rows",
                "code_rows",
                "phase_rows",
                "ionosphere_constraint_rows",
                "code_residual_rms_m",
                "phase_residual_rms_m",
            ]
            write_csv(
                filter_path,
                filter_fieldnames,
                [
                    {
                        "week": "2360",
                        "tow": "172800.000",
                        "iteration": "0",
                        "rows": "30",
                        "code_rows": "12",
                        "phase_rows": "12",
                        "ionosphere_constraint_rows": "6",
                        "code_residual_rms_m": "1.0",
                        "phase_residual_rms_m": "0.2",
                    }
                ],
            )
            write_csv(
                candidate_filter_path,
                filter_fieldnames,
                [
                    {
                        "week": "2360",
                        "tow": "172800.000",
                        "iteration": "0",
                        "rows": "36",
                        "code_rows": "12",
                        "phase_rows": "18",
                        "ionosphere_constraint_rows": "6",
                        "code_residual_rms_m": "1.5",
                        "phase_residual_rms_m": "0.2",
                    }
                ],
            )
            write_csv(
                residual_path,
                [
                    "week",
                    "tow",
                    "iteration",
                    "row_index",
                    "sat",
                    "row_type",
                    "residual_m",
                    "iono_state_m",
                    "primary_signal",
                    "secondary_signal",
                    "primary_observation_code",
                    "secondary_observation_code",
                    "frequency_index",
                    "ionosphere_coefficient",
                ],
                [
                    {
                        "week": "2360",
                        "tow": "172800.000",
                        "iteration": "0",
                        "row_index": "0",
                        "sat": "G01",
                        "row_type": "phase",
                        "residual_m": "0.1",
                        "iono_state_m": "1.0",
                        "primary_signal": "0",
                        "secondary_signal": "2",
                        "primary_observation_code": "C1C",
                        "secondary_observation_code": "C2W",
                        "frequency_index": "0",
                        "ionosphere_coefficient": "1.0",
                    }
                ],
            )
            write_csv(
                correction_path,
                [
                    "week",
                    "tow",
                    "sat",
                    "primary_signal",
                    "secondary_signal",
                    "primary_observation_code",
                    "secondary_observation_code",
                    "ssr_available",
                    "orbit_clock_applied",
                    "clock_m",
                    "code_bias_m",
                    "phase_bias_m",
                    "variance_pr",
                    "variance_cp",
                    "valid_after_corrections",
                ],
                [
                    {
                        "week": "2360",
                        "tow": "172800.000",
                        "sat": "G01",
                        "primary_signal": "0",
                        "secondary_signal": "2",
                        "primary_observation_code": "C1C",
                        "secondary_observation_code": "C2W",
                        "ssr_available": "1",
                        "orbit_clock_applied": "1",
                        "clock_m": "0.1",
                        "code_bias_m": "0.2",
                        "phase_bias_m": "-0.3",
                        "variance_pr": "0.4",
                        "variance_cp": "0.0005",
                        "valid_after_corrections": "1",
                    }
                ],
            )

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(filter_path),
                    "--compare-filter-log",
                    str(candidate_filter_path),
                    "--residual-log",
                    str(residual_path),
                    "--correction-log",
                    str(correction_path),
                    "--json-out",
                    str(report_path),
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("filter:", result.stdout)
            self.assertIn("filter_comparison:", result.stdout)
            self.assertIn("residual:", result.stdout)
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(report["filter_summary"]["max_phase_rows"], 12)
            self.assertEqual(report["filter_comparison"]["changed_rows"], 1)
            self.assertEqual(report["filter_comparison"]["max_column_deltas"][0]["column"], "phase_rows")
            self.assertEqual(report["correction_log_rows"], 1)
            self.assertEqual(report["residual_summary"]["row_type_counts"], {"phase": 1})
            self.assertEqual(report["residual_summary"]["correction_join"]["matched_rows"], 1)
            top_phase = report["residual_summary"]["final_iteration"]["top_residual_epochs_by_type"]["phase"][0]
            self.assertEqual(top_phase["sat"], "G01")
            self.assertTrue(top_phase["correction"]["matched"])
            self.assertAlmostEqual(top_phase["correction"]["phase_bias_m"], -0.3)


if __name__ == "__main__":
    unittest.main()
