#!/usr/bin/env python3

from __future__ import annotations

import csv
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "analysis" / "clas_dd_measurement_diff.py"
spec = importlib.util.spec_from_file_location("clas_dd_measurement_diff", SCRIPT)
assert spec is not None and spec.loader is not None
diff = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = diff
spec.loader.exec_module(diff)


FIELDS = (
    "schema", "week", "tow", "stage", "system_group", "frequency_index",
    "measurement_type", "reference_satellite", "target_satellite", "raw_dd_m", "residual_m",
    "linearization_x_m", "linearization_y_m", "linearization_z_m",
    "position_coeff_x", "position_coeff_y", "position_coeff_z",
)


def write_rows(path: Path, rows: list[tuple[object, ...]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(FIELDS)
        writer.writerows(rows)


def row(kind: str, target: str, residual: float, x_m: float = 0.0) -> tuple[object, ...]:
    return (
        "clas_dd_measurement.v3", 2068, 230420, "prefit", 0, 0, kind,
        "G01", target, residual, residual, x_m, 0.0, 0.0, 1.0, 0.0, 0.0,
    )


class ClasDdMeasurementDiffTest(unittest.TestCase):
    def test_exact_surface_and_rms_gates_pass(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory) / "base.csv"
            candidate = Path(directory) / "candidate.csv"
            write_rows(base, [row("phase", "G02", 1.0), row("code", "G02", 2.0)])
            write_rows(candidate, [row("phase", "G02", 1.004), row("code", "G02", 2.019)])
            report, details = diff.compare(base, candidate, 0.005, 0.020)

        self.assertEqual(report["status"], "passed")
        self.assertEqual(report["coverage_pct"], 100.0)
        self.assertAlmostEqual(report["metrics"]["phase"]["rms_delta_m"], 0.004)
        self.assertAlmostEqual(report["metrics"]["code"]["rms_delta_m"], 0.019)
        self.assertEqual(len(details), 2)

    def test_missing_row_and_threshold_failure_are_explicit(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory) / "base.csv"
            candidate = Path(directory) / "candidate.csv"
            write_rows(base, [row("phase", "G02", 1.0), row("code", "G02", 2.0)])
            write_rows(candidate, [row("phase", "G02", 1.006)])
            report, _ = diff.compare(base, candidate, 0.005, 0.020)

        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["base_only_rows"], 1)
        self.assertIn("row_surface", report["failed_gates"])
        self.assertIn("phase_rms", report["failed_gates"])
        self.assertIn("code_rms", report["failed_gates"])

    def test_duplicate_keys_fail_without_silent_averaging(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory) / "base.csv"
            candidate = Path(directory) / "candidate.csv"
            write_rows(base, [row("phase", "G02", 1.0), row("phase", "G02", 1.1), row("code", "G02", 2.0)])
            write_rows(candidate, [row("phase", "G02", 1.0), row("code", "G02", 2.0)])
            report, _ = diff.compare(base, candidate, 0.005, 0.020)

        self.assertEqual(report["base_duplicate_keys"], 1)
        self.assertIn("duplicate_keys", report["failed_gates"])

    def test_candidate_residual_can_be_moved_to_base_linearization_position(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory) / "base.csv"
            candidate = Path(directory) / "candidate.csv"
            write_rows(base, [row("phase", "G02", 1.0), row("code", "G02", 1.0)])
            write_rows(candidate, [row("phase", "G02", 0.0, 1.0), row("code", "G02", 0.0, 1.0)])
            report, _ = diff.compare(
                base, candidate, 0.005, 0.020, "residual_m", True
            )

        self.assertEqual(report["status"], "passed")
        self.assertTrue(report["candidate_position_normalized_to_base"])
        self.assertAlmostEqual(report["metrics"]["phase"]["rms_delta_m"], 0.0)

    def test_start_tow_excludes_declared_warmup_from_both_surfaces(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory) / "base.csv"
            candidate = Path(directory) / "candidate.csv"
            warmup = list(row("phase", "G02", 9.0))
            warmup[2] = 230419
            write_rows(base, [tuple(warmup), row("phase", "G02", 1.0), row("code", "G02", 1.0)])
            write_rows(candidate, [row("phase", "G02", 1.0), row("code", "G02", 1.0)])
            report, _ = diff.compare(
                base, candidate, 0.005, 0.020, "residual_m", False, 230420.0
            )

        self.assertEqual(report["status"], "passed")
        self.assertEqual(report["base_only_rows"], 0)
        self.assertEqual(report["start_tow"], 230420.0)


if __name__ == "__main__":
    unittest.main()
