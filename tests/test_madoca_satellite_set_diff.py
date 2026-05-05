#!/usr/bin/env python3
"""Tests for MADOCALIB/native satellite-set comparison helpers."""

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
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "madoca_satellite_set_diff.py"

spec = importlib.util.spec_from_file_location("madoca_satellite_set_diff", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
sat_diff = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sat_diff)


def write_stat(path: Path) -> None:
    path.write_text(
        "\n".join(
            [
                "$POS,2360,172800.000,0,0,0,0",
                "$SAT,2360,172800.000,G01,1,10.0,30.0,0.1,0.0,1,45.0,0,0,1,0,0,0",
                "$SAT,2360,172800.000,G02,1,20.0,35.0,0.2,0.0,1,46.0,0,0,1,0,0,0",
                "$SAT,2360,172800.000,G05,1,30.0,40.0,0.3,0.0,0,47.0,0,0,1,0,0,0",
                "$SAT,2360,172830.000,G02,1,20.0,35.0,0.2,0.0,1,46.0,0,0,1,0,0,0",
            ]
        )
        + "\n",
        encoding="utf-8",
    )


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_correction_log(path: Path) -> None:
    fieldnames = [
        "week",
        "tow",
        "sat",
        "primary_observation_code",
        "secondary_observation_code",
        "frequency_index",
        "ionosphere_coefficient",
        "has_carrier_phase",
        "ssr_available",
        "orbit_clock_applied",
        "phase_bias_m",
        "atmos_token_count",
        "stec_tecu",
        "iono_m",
        "ionosphere_estimation_constraint",
        "elevation_deg",
        "valid_after_corrections",
    ]
    write_csv(
        path,
        fieldnames,
        [
            {
                "week": "2360",
                "tow": "172800.000",
                "sat": "G01",
                "primary_observation_code": "C1C",
                "secondary_observation_code": "C2W",
                "frequency_index": "0",
                "ionosphere_coefficient": "1.0",
                "has_carrier_phase": "1",
                "ssr_available": "1",
                "orbit_clock_applied": "1",
                "phase_bias_m": "0.1",
                "atmos_token_count": "2",
                "stec_tecu": "5.0",
                "iono_m": "1.0",
                "ionosphere_estimation_constraint": "1",
                "elevation_deg": "30.0",
                "valid_after_corrections": "1",
            },
            {
                "week": "2360",
                "tow": "172800.000",
                "sat": "G03",
                "primary_observation_code": "C2W",
                "secondary_observation_code": "",
                "frequency_index": "1",
                "ionosphere_coefficient": "1.65",
                "has_carrier_phase": "0",
                "ssr_available": "1",
                "orbit_clock_applied": "1",
                "phase_bias_m": "0.0",
                "atmos_token_count": "0",
                "stec_tecu": "0.0",
                "iono_m": "0.0",
                "ionosphere_estimation_constraint": "0",
                "elevation_deg": "20.0",
                "valid_after_corrections": "1",
            },
            {
                "week": "2360",
                "tow": "172800.000",
                "sat": "G04",
                "primary_observation_code": "C2W",
                "secondary_observation_code": "",
                "frequency_index": "1",
                "ionosphere_coefficient": "1.65",
                "has_carrier_phase": "0",
                "ssr_available": "0",
                "orbit_clock_applied": "0",
                "phase_bias_m": "0.0",
                "atmos_token_count": "0",
                "stec_tecu": "9.0",
                "iono_m": "2.0",
                "ionosphere_estimation_constraint": "1",
                "elevation_deg": "5.0",
                "valid_after_corrections": "0",
            },
            {
                "week": "2360",
                "tow": "172830.000",
                "sat": "G02",
                "primary_observation_code": "C1C",
                "secondary_observation_code": "C2W",
                "frequency_index": "0",
                "ionosphere_coefficient": "1.0",
                "has_carrier_phase": "1",
                "ssr_available": "1",
                "orbit_clock_applied": "1",
                "phase_bias_m": "0.2",
                "atmos_token_count": "1",
                "stec_tecu": "2.0",
                "iono_m": "0.4",
                "ionosphere_estimation_constraint": "1",
                "elevation_deg": "35.0",
                "valid_after_corrections": "1",
            },
            {
                "week": "2360",
                "tow": "172830.000",
                "sat": "G04",
                "primary_observation_code": "C2W",
                "secondary_observation_code": "",
                "frequency_index": "1",
                "ionosphere_coefficient": "1.65",
                "has_carrier_phase": "0",
                "ssr_available": "1",
                "orbit_clock_applied": "0",
                "phase_bias_m": "0.0",
                "atmos_token_count": "1",
                "stec_tecu": "3.0",
                "iono_m": "0.7",
                "ionosphere_estimation_constraint": "1",
                "elevation_deg": "40.0",
                "valid_after_corrections": "1",
            },
        ],
    )


def write_residual_log(path: Path) -> None:
    fieldnames = [
        "week",
        "tow",
        "iteration",
        "row_index",
        "sat",
        "row_type",
        "residual_m",
        "frequency_index",
        "primary_observation_code",
        "secondary_observation_code",
        "phase_ready",
        "phase_skip_reason",
        "phase_limit_m",
        "ambiguity_lock_count",
        "required_lock_count",
    ]
    write_csv(
        path,
        fieldnames,
        [
            {"week": "2360", "tow": "172800.000", "iteration": "0", "sat": "G03", "row_type": "code", "residual_m": "100.0"},
            {"week": "2360", "tow": "172800.000", "iteration": "1", "sat": "G01", "row_type": "code", "residual_m": "0.4"},
            {"week": "2360", "tow": "172800.000", "iteration": "1", "sat": "G02", "row_type": "code", "residual_m": "0.5"},
            {"week": "2360", "tow": "172800.000", "iteration": "1", "sat": "G03", "row_type": "code", "residual_m": "3.0"},
            {
                "week": "2360",
                "tow": "172830.000",
                "iteration": "2",
                "sat": "G02",
                "row_type": "phase",
                "residual_m": "0.5",
                "frequency_index": "0",
                "primary_observation_code": "C1C",
                "secondary_observation_code": "C2W",
            },
            {
                "week": "2360",
                "tow": "172830.000",
                "iteration": "2",
                "row_index": "2",
                "sat": "G04",
                "row_type": "phase",
                "residual_m": "0.8",
                "frequency_index": "1",
                "primary_observation_code": "C2W",
                "secondary_observation_code": "",
            },
            {
                "week": "2360",
                "tow": "172830.000",
                "iteration": "2",
                "row_index": "3",
                "sat": "G02",
                "row_type": "phase_candidate",
                "residual_m": "19.2",
                "frequency_index": "1",
                "primary_observation_code": "C2W",
                "secondary_observation_code": "",
                "phase_ready": "1",
                "phase_skip_reason": "outlier",
                "phase_limit_m": "18",
                "ambiguity_lock_count": "2",
                "required_lock_count": "1",
            },
            {
                "week": "2360",
                "tow": "172830.000",
                "iteration": "2",
                "row_index": "4",
                "sat": "G04",
                "row_type": "phase_candidate",
                "residual_m": "2.0",
                "frequency_index": "1",
                "primary_observation_code": "C2W",
                "secondary_observation_code": "",
                "phase_ready": "0",
                "phase_skip_reason": "lock_count",
                "phase_limit_m": "18",
                "ambiguity_lock_count": "0",
                "required_lock_count": "1",
            },
        ],
    )


class MadocaSatelliteSetDiffTest(unittest.TestCase):
    def test_build_report_classifies_sets_corrections_and_final_residuals(self) -> None:
        with tempfile.TemporaryDirectory(prefix="madoca_satellite_set_diff_test_") as temp_dir:
            temp_root = Path(temp_dir)
            stat_path = temp_root / "bridge.pos.stat"
            correction_path = temp_root / "native_correction.csv"
            residual_path = temp_root / "native_residual.csv"
            write_stat(stat_path)
            write_correction_log(correction_path)
            write_residual_log(residual_path)

            args = sat_diff.parse_args(
                [
                    str(stat_path),
                    str(correction_path),
                    "--residual-log",
                    str(residual_path),
                    "--top-satellites",
                    "5",
                ]
            )
            report = sat_diff.build_report(args)

            comparison = report["satellite_set_comparison"]
            self.assertEqual(comparison["common_epochs"], 2)
            self.assertEqual(comparison["bridge_only_satellite_appearances"], 1)
            self.assertEqual(comparison["native_only_satellite_appearances"], 2)
            self.assertEqual(comparison["per_epoch"][0]["bridge_only"], ["G02"])
            self.assertEqual(comparison["per_epoch"][0]["native_only"], ["G03"])
            self.assertEqual(comparison["top_native_only_satellites"][0], {"sat": "G03", "epochs": 1})

            phase_comparison = report["satellite_set_comparison_phase_capable"]
            self.assertEqual(phase_comparison["native_only_satellite_appearances"], 0)
            self.assertEqual(phase_comparison["bridge_only_satellite_appearances"], 1)
            self.assertEqual(phase_comparison["per_epoch"][0]["bridge_only"], ["G02"])

            corrections = report["native_correction_summary"]["by_relation"]
            self.assertEqual(corrections["common"]["rows"], 2)
            self.assertEqual(corrections["common"]["phase_capable_rows"], 2)
            self.assertEqual(corrections["common"]["frequency_index_counts"], {"0": 2})
            self.assertEqual(corrections["native_only"]["rows"], 2)
            self.assertEqual(corrections["native_only"]["phase_capable_rows"], 0)
            self.assertEqual(corrections["native_only"]["frequency_index_counts"], {"1": 2})
            self.assertEqual(corrections["native_only"]["stec_rows"], 1)
            self.assertEqual(corrections["native_only"]["ionosphere_constraint_rows"], 1)
            self.assertAlmostEqual(corrections["native_only"]["mean_abs_iono_m"], 0.35)

            residuals = report["native_residual_summary"]
            self.assertEqual(residuals["final_iteration_rows"], 7)
            self.assertEqual(residuals["classified_rows"], 6)
            self.assertEqual(residuals["skipped_rows"], 1)
            native_code = next(
                item
                for item in residuals["by_relation_row_type_constellation"]
                if item["relation"] == "native_only" and item["row_type"] == "code"
            )
            self.assertEqual(native_code["rows"], 1)
            self.assertAlmostEqual(native_code["rms_residual_m"], 3.0)
            self.assertEqual(
                residuals["top_satellites_by_relation_type"]["native_only"]["code"][0]["sat"],
                "G03",
            )

            phase_frequency = report["phase_frequency_overlap"]
            self.assertEqual(phase_frequency["total_bridge_valid_frequency_rows"], 3)
            self.assertEqual(phase_frequency["total_native_final_phase_rows"], 2)
            phase_by_sat = {item["sat"]: item for item in phase_frequency["top_satellites"]}
            self.assertEqual(phase_by_sat["G02"]["bridge_valid_epochs"], 2)
            self.assertEqual(phase_by_sat["G02"]["native_phase_epochs"], 1)
            self.assertEqual(phase_by_sat["G02"]["bridge_frequency_epoch_counts"], {"1": 2})
            self.assertEqual(
                phase_by_sat["G02"]["native_phase_identities"],
                [{"identity": "f0:C1C/C2W", "rows": 1, "epochs": 1}],
            )
            self.assertEqual(phase_by_sat["G04"]["bridge_valid_epochs"], 0)
            self.assertEqual(phase_by_sat["G04"]["native_phase_epochs"], 1)

            phase_candidates = report["phase_candidate_summary"]
            self.assertEqual(phase_candidates["total_phase_candidate_rows"], 2)
            self.assertEqual(phase_candidates["total_ready_phase_candidate_rows"], 1)
            self.assertEqual(phase_candidates["total_accepted_phase_rows"], 2)
            phase_candidates_by_key = {
                (item["sat"], item["identity"]): item
                for item in phase_candidates["top_satellites"]
            }
            g02_candidate = phase_candidates_by_key[("G02", "f1:C2W/")]
            self.assertEqual(g02_candidate["candidate_rows"], 1)
            self.assertEqual(g02_candidate["ready_candidate_rows"], 1)
            self.assertEqual(g02_candidate["accepted_phase_rows"], 0)
            self.assertEqual(g02_candidate["skip_reasons"], [{"reason": "outlier", "rows": 1}])
            self.assertAlmostEqual(g02_candidate["first_candidate"]["residual_m"], 19.2)
            self.assertAlmostEqual(g02_candidate["min_candidate"]["residual_m"], 19.2)
            self.assertAlmostEqual(g02_candidate["first_ready_candidate"]["residual_m"], 19.2)
            self.assertAlmostEqual(g02_candidate["first_ready_candidate"]["phase_limit_m"], 18.0)
            g04_candidate = phase_candidates_by_key[("G04", "f1:C2W/")]
            self.assertEqual(g04_candidate["candidate_rows"], 1)
            self.assertEqual(g04_candidate["ready_candidate_rows"], 0)
            self.assertEqual(g04_candidate["accepted_phase_rows"], 1)
            self.assertAlmostEqual(g04_candidate["first_candidate"]["residual_m"], 2.0)
            self.assertAlmostEqual(g04_candidate["min_candidate"]["residual_m"], 2.0)
            self.assertEqual(g04_candidate["min_candidate"]["phase_skip_reason"], "lock_count")
            self.assertIsNone(g04_candidate["first_ready_candidate"])
            self.assertAlmostEqual(g04_candidate["first_accepted_phase"]["residual_m"], 0.8)

            initial_phase = report["initial_phase_admission_summary"]
            self.assertEqual(initial_phase["week"], 2360)
            self.assertAlmostEqual(initial_phase["tow"], 172830.0)
            self.assertEqual(initial_phase["iteration"], 2)
            self.assertEqual(initial_phase["rows"], 4)
            self.assertEqual(initial_phase["accepted_rows"], 2)
            self.assertEqual(initial_phase["skipped_rows"], 2)
            accepted_initial = initial_phase["by_status"]["accepted"]
            self.assertEqual(accepted_initial["correction_matches"], 2)
            self.assertEqual(accepted_initial["constellation_counts"], {"G": 2})
            self.assertEqual(accepted_initial["frequency_index_counts"], {"0": 1, "1": 1})
            self.assertEqual(accepted_initial["ssr_available_rows"], 2)
            self.assertEqual(accepted_initial["orbit_clock_applied_rows"], 1)
            self.assertEqual(accepted_initial["stec_nonzero_rows"], 2)
            self.assertEqual(accepted_initial["phase_bias_nonzero_rows"], 1)
            self.assertAlmostEqual(accepted_initial["mean_abs_residual_m"], 0.65)
            skipped_initial = initial_phase["by_status"]["skipped"]
            self.assertEqual(skipped_initial["correction_matches"], 1)
            self.assertEqual(
                skipped_initial["skip_reasons"],
                [{"reason": "outlier", "rows": 1}, {"reason": "lock_count", "rows": 1}],
            )
            self.assertAlmostEqual(skipped_initial["min_abs_residual_m"], 2.0)
            self.assertAlmostEqual(skipped_initial["max_abs_residual_m"], 19.2)

    def test_phase_frequency_overlap_filters_to_comparison_epochs(self) -> None:
        residual_rows = [
            {
                "week": "2360",
                "tow": "10.000",
                "iteration": "0",
                "sat": "G01",
                "row_type": "phase",
                "residual_m": "0.2",
                "frequency_index": "0",
                "primary_observation_code": "C1C",
                "secondary_observation_code": "C2W",
            }
        ]
        summary = sat_diff.summarize_phase_frequency_overlap(
            residual_rows,
            {("G01", 1): {(2360, 10000), (2360, 20000)}},
            comparison_epochs={(2360, 10000)},
            top_satellites=5,
        )

        self.assertEqual(summary["total_bridge_valid_frequency_rows"], 1)
        self.assertEqual(summary["top_satellites"][0]["bridge_valid_epochs"], 1)
        self.assertEqual(summary["top_satellites"][0]["bridge_frequency_epoch_counts"], {"1": 1})

    def test_cli_writes_json_and_epoch_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="madoca_satellite_set_diff_test_") as temp_dir:
            temp_root = Path(temp_dir)
            stat_path = temp_root / "bridge.pos.stat"
            correction_path = temp_root / "native_correction.csv"
            residual_path = temp_root / "native_residual.csv"
            report_path = temp_root / "report.json"
            epoch_path = temp_root / "epochs.csv"
            write_stat(stat_path)
            write_correction_log(correction_path)
            write_residual_log(residual_path)

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(stat_path),
                    str(correction_path),
                    "--residual-log",
                    str(residual_path),
                    "--json-out",
                    str(report_path),
                    "--epoch-csv",
                    str(epoch_path),
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("satellite_sets:", result.stdout)
            self.assertIn("satellite_sets_phase_capable:", result.stdout)
            self.assertIn("phase_frequency_overlap:", result.stdout)
            self.assertIn("phase_candidate_summary:", result.stdout)
            self.assertIn("initial_phase_admission_summary:", result.stdout)
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(report["satellite_set_comparison"]["common_epochs"], 2)
            self.assertEqual(
                report["satellite_set_comparison_phase_capable"]["native_only_satellite_appearances"],
                0,
            )
            self.assertEqual(report["phase_frequency_overlap"]["total_native_final_phase_rows"], 2)
            self.assertEqual(report["phase_candidate_summary"]["total_phase_candidate_rows"], 2)
            self.assertEqual(report["initial_phase_admission_summary"]["accepted_rows"], 2)
            self.assertEqual(report["initial_phase_admission_summary"]["skipped_rows"], 2)
            with epoch_path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["bridge_only"], "G02")
            self.assertEqual(rows[0]["native_only"], "G03")


if __name__ == "__main__":
    unittest.main()
