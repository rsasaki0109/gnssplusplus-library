#!/usr/bin/env python3
"""Tests for PPP residual row-set comparison helpers."""

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
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "ppp_residual_row_set_diff.py"

spec = importlib.util.spec_from_file_location("ppp_residual_row_set_diff", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
row_diff = importlib.util.module_from_spec(spec)
spec.loader.exec_module(row_diff)


FIELDNAMES = [
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
    "phase_skip_reason",
    "phase_limit_m",
    "ambiguity_lock_count",
    "required_lock_count",
]


def residual_row(
    *,
    tow: str = "10.000",
    iteration: str = "0",
    row_index: str = "0",
    sat: str,
    row_type: str = "phase",
    primary_signal: str = "0",
    secondary_signal: str = "2",
    primary_code: str = "C1C",
    secondary_code: str = "C2W",
    frequency_index: str = "0",
    ionosphere_coefficient: str = "1.0",
    residual_m: str = "0.1",
    phase_skip_reason: str = "",
    phase_limit_m: str = "",
    ambiguity_lock_count: str = "",
    required_lock_count: str = "",
) -> dict[str, str]:
    return {
        "week": "2360",
        "tow": tow,
        "iteration": iteration,
        "row_index": row_index,
        "sat": sat,
        "row_type": row_type,
        "residual_m": residual_m,
        "iono_state_m": "0.0",
        "primary_signal": primary_signal,
        "secondary_signal": secondary_signal,
        "primary_observation_code": primary_code,
        "secondary_observation_code": secondary_code,
        "frequency_index": frequency_index,
        "ionosphere_coefficient": ionosphere_coefficient,
        "phase_skip_reason": phase_skip_reason,
        "phase_limit_m": phase_limit_m,
        "ambiguity_lock_count": ambiguity_lock_count,
        "required_lock_count": required_lock_count,
    }


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


class PppResidualRowSetDiffTest(unittest.TestCase):
    def test_compares_iteration_phase_row_identities(self) -> None:
        base_rows = [
            residual_row(sat="G01", row_index="0"),
            residual_row(
                sat="G02",
                row_index="1",
                primary_signal="2",
                secondary_signal="21",
                primary_code="C2W",
                secondary_code="",
                frequency_index="1",
                ionosphere_coefficient="1.64694444444444",
            ),
            residual_row(sat="E11", row_index="2", row_type="code"),
            residual_row(sat="G04", tow="10.000", iteration="1", row_index="3"),
        ]
        candidate_rows = [
            residual_row(sat="G01", row_index="0"),
            residual_row(sat="R05", row_index="1", primary_signal="5", secondary_signal="7"),
            residual_row(sat="E11", row_index="2", row_type="code"),
            residual_row(sat="G05", tow="10.000", iteration="1", row_index="3"),
        ]

        report = row_diff.compare_row_sets(
            base_rows,
            candidate_rows,
            row_type="phase",
            iteration=0,
        )

        self.assertEqual(report["groups_compared"], 1)
        self.assertEqual(report["groups_with_differences"], 1)
        self.assertEqual(report["base_rows"], 2)
        self.assertEqual(report["candidate_rows"], 2)
        self.assertEqual(report["common_rows"], 1)
        self.assertEqual(report["base_only_rows"], 1)
        self.assertEqual(report["candidate_only_rows"], 1)
        self.assertEqual(report["base_only_satellites"], [{"sat": "G02", "rows": 1}])
        self.assertEqual(report["candidate_only_satellites"], [{"sat": "R05", "rows": 1}])
        first = report["first_difference"]
        self.assertEqual(first["tow"], 10.0)
        self.assertEqual(first["iteration"], 0)
        self.assertEqual(first["base_only"][0]["frequency_index"], 1)
        self.assertEqual(first["base_only"][0]["ionosphere_coefficient"], "1.64694444444")

    def test_explains_base_only_phase_rows_with_candidate_phase_diagnostics(self) -> None:
        base_rows = [residual_row(sat="G02", row_index="0")]
        candidate_rows = [
            residual_row(
                sat="G02",
                row_index="0",
                row_type="phase_candidate",
                residual_m="22.5",
                phase_skip_reason="outlier",
                phase_limit_m="18",
                ambiguity_lock_count="2",
                required_lock_count="1",
            ),
            residual_row(sat="G03", row_index="1"),
        ]

        report = row_diff.compare_row_sets(
            base_rows,
            candidate_rows,
            row_type="phase",
            iteration=0,
            explain_phase_candidates=True,
        )

        explanation = report["base_only_phase_candidate_explanations"]
        self.assertEqual(explanation["explained_rows"], 1)
        self.assertEqual(explanation["unexplained_rows"], 0)
        self.assertEqual(explanation["skip_reasons"], [{"reason": "outlier", "rows": 1}])
        self.assertEqual(
            explanation["skip_reason_satellites"],
            [{"reason": "outlier", "sat": "G02", "rows": 1}],
        )
        self.assertEqual(explanation["examples"][0]["phase_skip_reason"], "outlier")
        self.assertEqual(explanation["examples"][0]["residual_m"], "22.5")
        self.assertEqual(explanation["examples"][0]["phase_limit_m"], "18")

    def test_cli_writes_json_and_details_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="ppp_residual_row_set_diff_test_") as temp_dir:
            temp_root = Path(temp_dir)
            base_path = temp_root / "base.csv"
            candidate_path = temp_root / "candidate.csv"
            report_path = temp_root / "report.json"
            details_path = temp_root / "details.csv"
            write_csv(
                base_path,
                [
                    residual_row(sat="G01", row_index="0"),
                    residual_row(sat="G02", row_index="1"),
                ],
            )
            write_csv(
                candidate_path,
                [
                    residual_row(sat="G01", row_index="0"),
                    residual_row(sat="G03", row_index="1"),
                ],
            )

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(base_path),
                    str(candidate_path),
                    "--row-type",
                    "phase",
                    "--iteration",
                    "0",
                    "--json-out",
                    str(report_path),
                    "--details-csv",
                    str(details_path),
                    "--explain-phase-candidates",
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("residual_row_set_diff:", result.stdout)
            self.assertIn("base_only_phase_candidate_explanations:", result.stdout)
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(report["base_only_rows"], 1)
            self.assertEqual(report["candidate_only_rows"], 1)
            with details_path.open(newline="", encoding="utf-8") as details_file:
                detail_rows = list(csv.DictReader(details_file))
            self.assertEqual([row["side"] for row in detail_rows], ["base_only", "candidate_only"])
            self.assertEqual(detail_rows[0]["sat"], "G02")
            self.assertEqual(detail_rows[1]["sat"], "G03")


if __name__ == "__main__":
    unittest.main()
