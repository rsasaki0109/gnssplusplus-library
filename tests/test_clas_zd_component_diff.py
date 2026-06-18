#!/usr/bin/env python3
"""Tests for CLAS zero-difference component comparison helpers."""

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
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "clas_zd_component_diff.py"

spec = importlib.util.spec_from_file_location("clas_zd_component_diff", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
component_diff = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = component_diff
spec.loader.exec_module(component_diff)


FIELDNAMES = [
    "record",
    "stage",
    "week",
    "tow",
    "sat",
    "freq",
    "signal",
    "pseudorange_rinex_code",
    "carrier_rinex_code",
    "prc_m",
    "PRC",
    "cpc_m",
    "code_bias_m",
    "code_bias",
    "phase_bias_m",
    "receiver_ant_m",
    "receiver_antenna_m",
    "trop_correction_m",
]


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def code_row(
    *,
    sat: str,
    freq: str,
    prc: str,
    code_bias: str,
    receiver_ant: str,
    record: str = "CODE",
    stage: str = "",
    signal: str = "",
    pseudorange_rinex_code: str = "",
    carrier_rinex_code: str = "",
) -> dict[str, str]:
    return {
        "record": record,
        "stage": stage,
        "week": "2068",
        "tow": "230425.000",
        "sat": sat,
        "freq": freq,
        "signal": signal,
        "pseudorange_rinex_code": pseudorange_rinex_code,
        "carrier_rinex_code": carrier_rinex_code,
        "prc_m": prc,
        "PRC": "",
        "cpc_m": "",
        "code_bias_m": code_bias,
        "code_bias": "",
        "phase_bias_m": "",
        "receiver_ant_m": receiver_ant,
        "receiver_antenna_m": "",
        "trop_correction_m": "2.0",
    }


class ClasZdComponentDiffTest(unittest.TestCase):
    def test_observation_identity_prefers_row_type_specific_rinex_code(self) -> None:
        self.assertEqual(
            component_diff.observation_identity(
                {
                    "record": "CODE",
                    "signal": "3",
                    "pseudorange_rinex_code": "C2W",
                    "carrier_rinex_code": "L2W",
                },
                "code",
            ),
            "C2W",
        )
        self.assertEqual(
            component_diff.observation_identity(
                {
                    "record": "PHASE",
                    "signal": "3",
                    "pseudorange_rinex_code": "C2W",
                    "carrier_rinex_code": "L2W",
                },
                "phase",
            ),
            "L2W",
        )

    def test_compares_common_component_rows_and_reports_unmatched(self) -> None:
        base_rows = [
            code_row(sat="G14", freq="1", prc="0.50", code_bias="0.10", receiver_ant="-0.02"),
            code_row(sat="J01", freq="0", prc="3.75", code_bias="0.00", receiver_ant="-0.05"),
        ]
        candidate_rows = [
            code_row(
                sat="G14",
                freq="1",
                prc="0.75",
                code_bias="0.10",
                receiver_ant="0.00",
                stage="accepted",
                signal="3",
                pseudorange_rinex_code="C2W",
            ),
            code_row(
                sat="J02",
                freq="0",
                prc="5.00",
                code_bias="0.00",
                receiver_ant="-0.08",
                stage="accepted",
                signal="18",
            ),
        ]

        base = component_diff.normalize_rows(
            base_rows,
            component_names=["prc_m", "code_bias_m", "receiver_antenna_m"],
            stage_filter=None,
            row_type_filter="code",
        )
        candidate = component_diff.normalize_rows(
            candidate_rows,
            component_names=["prc_m", "code_bias_m", "receiver_antenna_m"],
            stage_filter=None,
            row_type_filter="code",
        )
        report = component_diff.build_report(
            base,
            candidate,
            base_label="claslib",
            candidate_label="native",
            threshold_m=0.1,
            top_deltas=10,
            top_unmatched=10,
        )

        self.assertEqual(report["schema"], "clas_zd_component_diff.v1")
        self.assertEqual(report["base_rows"], 2)
        self.assertEqual(report["candidate_rows"], 2)
        self.assertEqual(report["common_rows"], 1)
        self.assertEqual(report["base_only_rows"], 1)
        self.assertEqual(report["candidate_only_rows"], 1)
        self.assertEqual(report["threshold_exceedances"], 1)
        self.assertEqual(report["top_base_only"][0]["sat"], "J01")
        self.assertEqual(report["top_candidate_only"][0]["sat"], "J02")
        self.assertEqual(report["top_component_deltas"][0]["component"], "prc_m")
        self.assertAlmostEqual(report["top_component_deltas"][0]["delta_m"], 0.25)
        self.assertEqual(report["top_component_deltas"][0]["candidate_signal"], "C2W")

    def test_cli_writes_schema_json_and_details_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="clas_zd_component_diff_test_") as temp_dir:
            root = Path(temp_dir)
            base_csv = root / "claslib.csv"
            candidate_csv = root / "native.csv"
            report_path = root / "report.json"
            details_path = root / "details.csv"
            write_csv(
                base_csv,
                [code_row(sat="G14", freq="1", prc="0.50", code_bias="0.10", receiver_ant="-0.02")],
            )
            write_csv(
                candidate_csv,
                [
                    code_row(
                        sat="G14",
                        freq="1",
                        prc="0.75",
                        code_bias="0.10",
                        receiver_ant="0.00",
                        signal="3",
                        pseudorange_rinex_code="C2W",
                    )
                ],
            )

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(base_csv),
                    str(candidate_csv),
                    "--base-label",
                    "claslib",
                    "--candidate-label",
                    "native",
                    "--row-type",
                    "code",
                    "--component",
                    "prc_m",
                    "--component",
                    "receiver_antenna_m",
                    "--component-threshold-m",
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
            self.assertIn("clas_zd_component_diff:", result.stdout)
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(report["schema"], "clas_zd_component_diff.v1")
            self.assertEqual(report["threshold_exceedances"], 1)
            self.assertEqual(report["component_names"], ["prc_m", "receiver_antenna_m"])
            with details_path.open(newline="", encoding="utf-8") as details_file:
                rows = list(csv.DictReader(details_file))
            self.assertEqual(rows[0]["component"], "prc_m")
            self.assertEqual(rows[0]["sat"], "G14")
            self.assertEqual(rows[0]["candidate_signal"], "C2W")
            self.assertEqual(float(rows[0]["delta_m"]), 0.25)


if __name__ == "__main__":
    unittest.main()
