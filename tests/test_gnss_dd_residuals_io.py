"""Unit tests for ``apps/gnss_dd_residuals_io``."""

from __future__ import annotations

import csv
import math
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"
sys.path.insert(0, str(APPS_DIR))

import gnss_dd_residuals_io as dd_io  # noqa: E402


_REQUIRED_COLUMNS = (
    "epoch_index",
    "gps_week",
    "tow",
    "status",
    "kind",
    "frequency_index",
    "reference_satellite",
    "satellite",
    "residual_m",
    "abs_residual_m",
    "suppressed_by_outlier_threshold",
)


def _write_csv(path: Path, columns: list[str], rows: list[dict]) -> None:
    with path.open("w", encoding="ascii", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


class TestOptionalFloat(unittest.TestCase):
    def test_parses_finite_string(self):
        self.assertAlmostEqual(dd_io.optional_float("1.25"), 1.25)

    def test_none_for_empty_or_missing(self):
        self.assertIsNone(dd_io.optional_float(None))
        self.assertIsNone(dd_io.optional_float(""))

    def test_none_for_nonfinite(self):
        self.assertIsNone(dd_io.optional_float("nan"))
        self.assertIsNone(dd_io.optional_float("inf"))


class TestReadRecords(unittest.TestCase):
    def test_parses_minimal_row(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = Path(tmpdir) / "dd.csv"
            _write_csv(
                csv_path,
                list(_REQUIRED_COLUMNS) + ["reference_variance_m2", "satellite_variance_m2"],
                [
                    {
                        "epoch_index": "1",
                        "gps_week": "2200",
                        "tow": "120.0",
                        "status": "4",
                        "kind": "phase",
                        "frequency_index": "0",
                        "reference_satellite": "G01",
                        "satellite": "G05",
                        "residual_m": "0.012",
                        "abs_residual_m": "0.012",
                        "suppressed_by_outlier_threshold": "0",
                        "reference_variance_m2": "0.0004",
                        "satellite_variance_m2": "0.0004",
                    }
                ],
            )
            records = dd_io.read_records(csv_path)
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]["kind"], "phase")
        self.assertAlmostEqual(records[0]["residual_m"], 0.012)
        self.assertAlmostEqual(records[0]["reference_variance_m2"], 0.0004)

    def test_rejects_missing_required_columns(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = Path(tmpdir) / "dd.csv"
            _write_csv(csv_path, ["epoch_index", "gps_week"], [])
            with self.assertRaises(SystemExit) as caught:
                dd_io.read_records(csv_path)
            self.assertIn("Missing DD residual columns", str(caught.exception))

    def test_drops_nonfinite_residual_rows(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = Path(tmpdir) / "dd.csv"
            _write_csv(
                csv_path,
                list(_REQUIRED_COLUMNS),
                [
                    {
                        "epoch_index": "1",
                        "gps_week": "2200",
                        "tow": "120.0",
                        "status": "4",
                        "kind": "phase",
                        "frequency_index": "0",
                        "reference_satellite": "G01",
                        "satellite": "G05",
                        "residual_m": "nan",
                        "abs_residual_m": "nan",
                        "suppressed_by_outlier_threshold": "0",
                    },
                    {
                        "epoch_index": "1",
                        "gps_week": "2200",
                        "tow": "120.0",
                        "status": "4",
                        "kind": "phase",
                        "frequency_index": "0",
                        "reference_satellite": "G01",
                        "satellite": "G05",
                        "residual_m": "0.05",
                        "abs_residual_m": "0.05",
                        "suppressed_by_outlier_threshold": "0",
                    },
                ],
            )
            records = dd_io.read_records(csv_path)
        self.assertEqual(len(records), 1)
        self.assertTrue(math.isfinite(records[0]["residual_m"]))

    def test_optional_variance_columns_default_to_none(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = Path(tmpdir) / "dd.csv"
            _write_csv(
                csv_path,
                list(_REQUIRED_COLUMNS),
                [
                    {
                        "epoch_index": "1",
                        "gps_week": "2200",
                        "tow": "120.0",
                        "status": "4",
                        "kind": "code",
                        "frequency_index": "1",
                        "reference_satellite": "G01",
                        "satellite": "G05",
                        "residual_m": "0.5",
                        "abs_residual_m": "0.5",
                        "suppressed_by_outlier_threshold": "0",
                    }
                ],
            )
            records = dd_io.read_records(csv_path)
        self.assertIsNone(records[0]["reference_variance_m2"])
        self.assertIsNone(records[0]["satellite_variance_m2"])


class TestWriteTopPairsCsv(unittest.TestCase):
    def test_emits_ranked_rows_with_header(self):
        rows = [
            {
                "kind": "phase",
                "frequency_index": 0,
                "frequency_label": "freq0 primary (L1/E1/B1)",
                "reference_satellite": "G01",
                "satellite": "G05",
                "pair": "G01-G05",
                "rows": 100,
                "epochs": 50,
                "suppressed_rows": 0,
                "rms_residual_m": 0.012,
                "p95_abs_residual_m": 0.025,
                "max_abs_residual_m": 0.040,
                "rms_normalized_residual": 0.9,
                "p95_abs_normalized_residual": 1.6,
                "max_abs_normalized_residual": 2.4,
            },
            {
                "kind": "code",
                "frequency_index": 0,
                "frequency_label": "freq0 primary (L1/E1/B1)",
                "reference_satellite": "G01",
                "satellite": "G07",
                "pair": "G01-G07",
                "rows": 90,
                "epochs": 48,
                "suppressed_rows": 1,
                "rms_residual_m": 0.4,
                "p95_abs_residual_m": 0.8,
                "max_abs_residual_m": 1.2,
                "rms_normalized_residual": 1.1,
                "p95_abs_normalized_residual": 2.0,
                "max_abs_normalized_residual": 3.5,
            },
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            out_path = Path(tmpdir) / "subdir" / "top_pairs.csv"
            dd_io.write_top_pairs_csv(out_path, rows)
            self.assertTrue(out_path.exists())
            with out_path.open(encoding="ascii", newline="") as handle:
                reader = csv.DictReader(handle)
                written = list(reader)
        self.assertEqual(written[0]["rank"], "1")
        self.assertEqual(written[1]["rank"], "2")
        self.assertEqual(written[0]["pair"], "G01-G05")
        self.assertEqual(written[1]["pair"], "G01-G07")


if __name__ == "__main__":
    unittest.main()
