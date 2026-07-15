#!/usr/bin/env python3

from __future__ import annotations

import csv
import importlib.util
import math
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "analysis" / "clas_filter_state_diff.py"
spec = importlib.util.spec_from_file_location("clas_filter_state_diff", SCRIPT)
assert spec is not None and spec.loader is not None
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


FIELDS = (
    "schema", "week", "tow", "solution_state", "state_kind", "state_key",
    "state_index", "float_value", "fixed_value", "variance", "process_noise",
)


def write_rows(path: Path, rows: list[tuple[object, ...]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(FIELDS)
        writer.writerows(rows)


def row(tow: float, kind: str, key: str, value: float, *, index: int = 0) -> tuple[object, ...]:
    return (
        module.ROW_SCHEMA, 2068, tow, "fix", kind, key, index,
        value, value, 1e-4, 1e-6,
    )


class ClasFilterStateDiffTest(unittest.TestCase):
    def test_state_index_is_not_part_of_canonical_key(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp) / "base.csv"
            candidate = Path(temp) / "candidate.csv"
            write_rows(base, [
                row(230720.0, "position", "x", 10.0),
                row(230720.0, "ionosphere", "E27", 0.01, index=61),
            ])
            write_rows(candidate, [
                row(230720.0, "position", "x", 10.0),
                row(230720.0, "ionosphere", "E27", 0.011, index=88),
            ])

            report, details = module.compare(base, candidate, start_tow=230720.0)

        self.assertEqual(report["common_rows"], 2)
        self.assertEqual(report["status"], "passed")
        ionosphere = next(row for row in details if row["state_kind"] == "ionosphere")
        self.assertEqual(ionosphere["base_state_index"], 61)
        self.assertEqual(ionosphere["candidate_state_index"], 88)

    def test_position_gate_and_surface_are_explicit(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp) / "base.csv"
            candidate = Path(temp) / "candidate.csv"
            write_rows(base, [row(1.0, "position", "x", 10.0), row(1.0, "ionosphere", "G14", 0.0)])
            write_rows(candidate, [row(1.0, "position", "x", 10.1)])

            report, _ = module.compare(
                base,
                candidate,
                position_rms_threshold=0.02,
                require_complete_surface=True,
            )

        self.assertIn("position_float_rms", report["failed_gates"])
        self.assertIn("ionosphere_float_rms", report["failed_gates"])
        self.assertIn("row_surface", report["failed_gates"])

    def test_matching_nan_fixed_values_do_not_create_presence_failure(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp) / "base.csv"
            candidate = Path(temp) / "candidate.csv"
            float_row = list(row(1.0, "position", "x", 10.0))
            float_row[3] = "float"
            float_row[8] = math.nan
            write_rows(base, [tuple(float_row), row(1.0, "ionosphere", "G14", 0.0)])
            write_rows(candidate, [tuple(float_row), row(1.0, "ionosphere", "G14", 0.0)])

            report, _ = module.compare(base, candidate)

        self.assertEqual(report["fixed_presence_mismatches"], 0)
        self.assertEqual(report["status"], "passed")

    def test_start_week_and_tow_filter_across_rollover(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp) / "base.csv"
            candidate = Path(temp) / "candidate.csv"
            rows = [
                row(604799.0, "position", "x", 1.0),
                row(604799.0, "ionosphere", "G14", 0.0),
                row(271.0, "position", "x", 2.0),
                row(271.0, "ionosphere", "G14", 0.0),
            ]
            first_week = [list(value) for value in rows[:2]]
            second_week = [list(value) for value in rows[2:]]
            for value in first_week:
                value[1] = 2028
            for value in second_week:
                value[1] = 2029
            write_rows(base, [tuple(value) for value in (*first_week, *second_week)])
            write_rows(candidate, [tuple(value) for value in (*first_week, *second_week)])

            report, _ = module.compare(
                base, candidate, start_week=2029, start_tow=271.0
            )

        self.assertEqual(report["base_rows"], 2)
        self.assertEqual(report["common_rows"], 2)
        self.assertEqual(report["status"], "passed")


if __name__ == "__main__":
    unittest.main()
