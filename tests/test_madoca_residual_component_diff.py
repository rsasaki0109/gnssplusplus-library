#!/usr/bin/env python3
"""Tests for MADOCA residual component comparison helpers."""

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
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "madoca_residual_component_diff.py"

spec = importlib.util.spec_from_file_location("madoca_residual_component_diff", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
component_diff = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = component_diff
spec.loader.exec_module(component_diff)


FIELDNAMES = [
    "week",
    "tow",
    "iteration",
    "row_index",
    "sat",
    "row_type",
    "residual_m",
    "iono_state_m",
    "trop_m",
    "primary_signal",
    "secondary_signal",
    "primary_observation_code",
    "secondary_observation_code",
    "frequency_index",
    "ionosphere_coefficient",
]


def residual_row(
    *,
    sat: str,
    row_type: str = "phase",
    residual_m: str = "0.1",
    iono_state_m: str = "0.0",
    trop_m: str = "2.3",
    tow: str = "172800.000",
    iteration: str = "1",
    row_index: str = "0",
    primary_signal: str = "2",
    secondary_signal: str = "21",
    primary_code: str = "C1C",
    secondary_code: str = "C2W",
    frequency_index: str = "0",
    ionosphere_coefficient: str = "1.0",
) -> dict[str, str]:
    return {
        "week": "2360",
        "tow": tow,
        "iteration": iteration,
        "row_index": row_index,
        "sat": sat,
        "row_type": row_type,
        "residual_m": residual_m,
        "iono_state_m": iono_state_m,
        "trop_m": trop_m,
        "primary_signal": primary_signal,
        "secondary_signal": secondary_signal,
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


class MadocaResidualComponentDiffTest(unittest.TestCase):
    def test_compares_common_row_components_and_reports_unmatched(self) -> None:
        base_rows = [
            residual_row(sat="G01", row_type="code", residual_m="0.10", iono_state_m="0.50"),
            residual_row(
                sat="G02",
                row_type="phase",
                residual_m="-0.20",
                iono_state_m="0.40",
                row_index="1",
                frequency_index="1",
                primary_code="C2W",
                secondary_code="",
                ionosphere_coefficient="1.64694444444444",
            ),
            residual_row(sat="E11", row_type="code", residual_m="1.0", row_index="2"),
        ]
        candidate_rows = [
            residual_row(sat="G01", row_type="code", residual_m="0.15", iono_state_m="0.55"),
            residual_row(
                sat="G02",
                row_type="phase",
                residual_m="-1.00",
                iono_state_m="0.35",
                row_index="1",
                frequency_index="1",
                primary_code="C2W",
                secondary_code="",
                ionosphere_coefficient="1.64694444444444",
            ),
            residual_row(sat="R03", row_type="code", residual_m="2.0", row_index="2"),
        ]

        base = component_diff.normalize_rows(
            base_rows,
            component_names=["residual_m", "iono_state_m"],
            row_type_filter=None,
            iteration_filter=1,
        )
        candidate = component_diff.normalize_rows(
            candidate_rows,
            component_names=["residual_m", "iono_state_m"],
            row_type_filter=None,
            iteration_filter=1,
        )
        report = component_diff.build_report(
            base,
            candidate,
            base_label="madocalib",
            candidate_label="native",
            threshold=0.1,
            top_deltas=10,
            top_unmatched=10,
        )

        self.assertEqual(report["schema"], "madoca_residual_component_diff.v1")
        self.assertEqual(report["base_rows"], 3)
        self.assertEqual(report["candidate_rows"], 3)
        self.assertEqual(report["common_rows"], 2)
        self.assertEqual(report["base_only_rows"], 1)
        self.assertEqual(report["candidate_only_rows"], 1)
        self.assertFalse(report["row_set_complete"])
        self.assertEqual(report["components_compared"], 4)
        self.assertEqual(report["threshold_exceedances"], 1)
        self.assertEqual(report["top_base_only"][0]["sat"], "E11")
        self.assertEqual(report["top_candidate_only"][0]["sat"], "R03")
        top = report["top_component_deltas"][0]
        self.assertEqual(top["sat"], "G02")
        self.assertEqual(top["component"], "residual_m")
        self.assertAlmostEqual(top["delta"], -0.8)
        self.assertEqual(top["primary_observation_code"], "C2W")
        self.assertEqual(top["ionosphere_coefficient"], "1.64694444444")

    def test_exact_observation_identity_separates_same_satellite_aliases(self) -> None:
        base = component_diff.normalize_rows(
            [
                residual_row(
                    sat="G14",
                    primary_code="C2W",
                    secondary_code="",
                    frequency_index="1",
                )
            ],
            component_names=["residual_m"],
            row_type_filter=None,
            iteration_filter=None,
        )
        candidate = component_diff.normalize_rows(
            [
                residual_row(
                    sat="G14",
                    primary_code="C2X",
                    secondary_code="",
                    frequency_index="1",
                )
            ],
            component_names=["residual_m"],
            row_type_filter=None,
            iteration_filter=None,
        )

        report = component_diff.build_report(
            base,
            candidate,
            base_label="madocalib",
            candidate_label="native",
            threshold=None,
            top_deltas=10,
            top_unmatched=10,
        )

        self.assertEqual(report["common_rows"], 0)
        self.assertEqual(report["base_only_rows"], 1)
        self.assertEqual(report["candidate_only_rows"], 1)
        self.assertEqual(report["top_base_only"][0]["primary_observation_code"], "C2W")
        self.assertEqual(report["top_candidate_only"][0]["primary_observation_code"], "C2X")

    def test_cli_writes_schema_json_and_details_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="madoca_residual_component_diff_test_") as temp_dir:
            root = Path(temp_dir)
            base_path = root / "madocalib.csv"
            candidate_path = root / "native.csv"
            report_path = root / "report.json"
            details_path = root / "details.csv"
            write_csv(
                base_path,
                [residual_row(sat="G01", row_type="code", residual_m="0.10")],
            )
            write_csv(
                candidate_path,
                [residual_row(sat="G01", row_type="code", residual_m="0.35")],
            )

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(base_path),
                    str(candidate_path),
                    "--row-type",
                    "code",
                    "--iteration",
                    "1",
                    "--component",
                    "residual_m",
                    "--component-threshold",
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
            self.assertIn("madoca_residual_component_diff:", result.stdout)
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(report["schema"], "madoca_residual_component_diff.v1")
            self.assertEqual(report["threshold_exceedances"], 1)
            self.assertEqual(report["component_names"], ["residual_m"])
            with details_path.open(newline="", encoding="utf-8") as details_file:
                rows = list(csv.DictReader(details_file))
            self.assertEqual(rows[0]["sat"], "G01")
            self.assertEqual(rows[0]["component"], "residual_m")
            self.assertAlmostEqual(float(rows[0]["delta"]), 0.25)


if __name__ == "__main__":
    unittest.main()
