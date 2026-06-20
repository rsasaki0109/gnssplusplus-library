#!/usr/bin/env python3
"""Tests for MADOCA residual-component snapshot summaries."""

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
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "madoca_residual_component_summary.py"

spec = importlib.util.spec_from_file_location("madoca_residual_component_summary", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
summary_script = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = summary_script
spec.loader.exec_module(summary_script)


FIELDNAMES = [
    "week",
    "tow",
    "iteration",
    "sat",
    "row_type",
    "residual_m",
    "iono_state_m",
    "trop_m",
    "primary_observation_code",
    "secondary_observation_code",
    "frequency_index",
    "ionosphere_coefficient",
]


def residual_row(
    *,
    sat: str = "G01",
    tow: str = "172800.000",
    iteration: str = "1",
    row_type: str = "code",
    residual_m: str = "0.1",
    iono_state_m: str = "0.5",
    trop_m: str = "2.3",
    primary_code: str = "C1C",
    secondary_code: str = "C2W",
    frequency_index: str = "0",
    ionosphere_coefficient: str = "1.0",
) -> dict[str, str]:
    return {
        "week": "2360",
        "tow": tow,
        "iteration": iteration,
        "sat": sat,
        "row_type": row_type,
        "residual_m": residual_m,
        "iono_state_m": iono_state_m,
        "trop_m": trop_m,
        "primary_observation_code": primary_code,
        "secondary_observation_code": secondary_code,
        "frequency_index": frequency_index,
        "ionosphere_coefficient": ionosphere_coefficient,
    }


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


class MadocaResidualComponentSummaryTest(unittest.TestCase):
    def test_summary_reports_row_identity_and_components(self) -> None:
        rows = [
            residual_row(sat="G01", row_type="code", primary_code="C1C"),
            residual_row(
                sat="E11",
                row_type="phase",
                tow="172830.000",
                iteration="2",
                primary_code="C5Q",
                secondary_code="",
                frequency_index="1",
                ionosphere_coefficient="1.64694444444444",
            ),
        ]

        summary = summary_script.summarize_rows(rows)

        self.assertEqual(summary["schema"], "madoca_residual_component_summary.v2")
        self.assertEqual(summary["status"], "passed")
        self.assertEqual(summary["rows"], 2)
        self.assertEqual(summary["systems"], {"Galileo": 1, "GPS": 1})
        self.assertEqual(summary["row_types"], {"code": 1, "phase": 1})
        self.assertEqual(summary["iterations"], {"1": 1, "2": 1})
        self.assertEqual(summary["time_range"]["start_tow"], 172800.0)
        self.assertEqual(summary["time_range"]["end_tow"], 172830.0)
        observation_identity = summary["observation_identity"]
        self.assertEqual(observation_identity["primary_observation_codes"]["C1C"], 1)
        self.assertEqual(observation_identity["primary_observation_codes"]["C5Q"], 1)
        self.assertEqual(observation_identity["frequency_indices"]["1"], 1)
        self.assertEqual(summary["component_presence"]["rows_with_component"]["residual_m"], 2)
        self.assertEqual(summary["component_presence"]["rows_with_component"]["iono_state_m"], 2)
        self.assertEqual(summary["row_key"]["duplicate_groups"], 0)

    def test_summary_detects_bad_rows_and_duplicate_keys(self) -> None:
        rows = [
            residual_row(sat="G01", residual_m=""),
            residual_row(sat="G01", residual_m=""),
            residual_row(sat="", row_type="", tow="bad", residual_m="nan", iono_state_m="", trop_m=""),
        ]

        summary = summary_script.summarize_rows(rows)

        self.assertEqual(summary["status"], "failed")
        self.assertEqual(summary["row_key"]["duplicate_groups"], 1)
        self.assertEqual(summary["issue_counts"]["missing_satellite"], 1)
        self.assertEqual(summary["issue_counts"]["missing_row_type"], 1)
        self.assertEqual(summary["issue_counts"]["bad_row_key"], 1)
        self.assertEqual(summary["issue_counts"]["missing_numeric_component"], 1)

    def test_cli_writes_json_and_enforces_requirements(self) -> None:
        with tempfile.TemporaryDirectory(prefix="madoca_residual_component_summary_cli_") as temp_dir:
            root = Path(temp_dir)
            snapshot = root / "residual.csv"
            summary_json = root / "summary.json"
            write_csv(snapshot, [residual_row(row_type="code")])

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(snapshot),
                    "--json-out",
                    str(summary_json),
                    "--require-rows-min",
                    "2",
                    "--require-row-type",
                    "phase",
                    "--require-iteration",
                    "2",
                    "--require-component",
                    "residual_m",
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 1)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["schema"], "madoca_residual_component_summary.v2")
            self.assertEqual(payload["status"], "failed")
            self.assertIn("rows 1 < required minimum 2", payload["failures"])
            self.assertIn("required row type phase is absent", payload["failures"])
            self.assertIn("required iteration 2 is absent", payload["failures"])

    def test_cli_passes_clean_snapshot_with_fail_on_issue(self) -> None:
        with tempfile.TemporaryDirectory(prefix="madoca_residual_component_summary_pass_") as temp_dir:
            root = Path(temp_dir)
            snapshot = root / "residual.csv"
            summary_json = root / "summary.json"
            write_csv(snapshot, [residual_row(row_type="code")])

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(snapshot),
                    "--json-out",
                    str(summary_json),
                    "--fail-on-issue",
                    "--require-row-type",
                    "code",
                    "--require-system",
                    "GPS",
                    "--require-primary-observation-code",
                    "C1C",
                    "--require-secondary-observation-code",
                    "C2W",
                    "--require-frequency-index",
                    "0",
                    "--require-ionosphere-coefficient",
                    "1",
                    "--require-component",
                    "residual_m",
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["status"], "passed")
            self.assertEqual(payload["failures"], [])

    def test_cli_rejects_missing_identity_requirements(self) -> None:
        with tempfile.TemporaryDirectory(prefix="madoca_residual_component_summary_identity_") as temp_dir:
            root = Path(temp_dir)
            snapshot = root / "residual.csv"
            summary_json = root / "summary.json"
            write_csv(snapshot, [residual_row(row_type="code")])

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(snapshot),
                    "--json-out",
                    str(summary_json),
                    "--require-system",
                    "Galileo",
                    "--require-primary-observation-code",
                    "C5Q",
                    "--require-secondary-observation-code",
                    "L5Q",
                    "--require-frequency-index",
                    "2",
                    "--require-ionosphere-coefficient",
                    "1.64694444444444",
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 1)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["status"], "failed")
            self.assertIn("required system Galileo is absent", payload["failures"])
            self.assertIn("required primary observation code C5Q is absent", payload["failures"])
            self.assertIn("required secondary observation code L5Q is absent", payload["failures"])
            self.assertIn("required frequency index 2 is absent", payload["failures"])
            self.assertIn(
                "required ionosphere coefficient 1.64694444444444 is absent",
                payload["failures"],
            )


if __name__ == "__main__":
    unittest.main()
