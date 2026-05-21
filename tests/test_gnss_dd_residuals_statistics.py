"""Unit tests for ``apps/gnss_dd_residuals_statistics``."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"
sys.path.insert(0, str(APPS_DIR))

import gnss_dd_residuals_statistics as stats  # noqa: E402


def _row(
    *,
    epoch_index: int = 1,
    gps_week: int = 2200,
    tow: float = 120.0,
    status: int = 4,
    kind: str = "phase",
    frequency_index: int = 0,
    reference_satellite: str = "G01",
    satellite: str = "G05",
    residual_m: float = 0.01,
    reference_variance: float | None = 0.0001,
    satellite_variance: float | None = 0.0001,
    suppressed: int = 0,
) -> dict:
    return {
        "epoch_index": epoch_index,
        "gps_week": gps_week,
        "tow": tow,
        "status": status,
        "kind": kind,
        "frequency_index": frequency_index,
        "reference_satellite": reference_satellite,
        "satellite": satellite,
        "residual_m": residual_m,
        "abs_residual_m": abs(residual_m),
        "reference_variance_m2": reference_variance,
        "satellite_variance_m2": satellite_variance,
        "suppressed_by_outlier_threshold": suppressed,
    }


class TestPercentile(unittest.TestCase):
    def test_returns_none_for_empty(self):
        self.assertIsNone(stats.percentile([], 0.5))

    def test_single_value_returns_value(self):
        self.assertEqual(stats.percentile([3.0], 0.5), 3.0)

    def test_median_of_evenly_spaced(self):
        self.assertAlmostEqual(stats.percentile([1.0, 2.0, 3.0, 4.0, 5.0], 0.5), 3.0)

    def test_p95_interpolates_between_neighbours(self):
        values = list(range(1, 21))  # 1..20
        # numpy default: 19th element = 20 → p95 = 19 + 0.05 * (20 - 19) = 19.05
        self.assertAlmostEqual(stats.percentile(values, 0.95), 19.05)


class TestGroupRecords(unittest.TestCase):
    def test_groups_by_single_key(self):
        rows = [_row(kind="phase"), _row(kind="phase"), _row(kind="code")]
        groups = stats.group_records(rows, ("kind",))
        self.assertEqual(len(groups[("phase",)]), 2)
        self.assertEqual(len(groups[("code",)]), 1)

    def test_groups_by_multiple_keys(self):
        rows = [
            _row(kind="phase", frequency_index=0),
            _row(kind="phase", frequency_index=1),
            _row(kind="phase", frequency_index=0),
        ]
        groups = stats.group_records(rows, ("kind", "frequency_index"))
        self.assertEqual(len(groups[("phase", 0)]), 2)
        self.assertEqual(len(groups[("phase", 1)]), 1)


class TestResidualStats(unittest.TestCase):
    def test_empty_records_produce_none_aggregates(self):
        out = stats.residual_stats([])
        self.assertEqual(out["rows"], 0)
        self.assertIsNone(out["rms_residual_m"])
        self.assertIsNone(out["worst_row"])

    def test_rms_matches_manual_computation(self):
        rows = [_row(residual_m=value) for value in (0.01, -0.02, 0.03)]
        out = stats.residual_stats(rows)
        expected_rms = math.sqrt((0.0001 + 0.0004 + 0.0009) / 3)
        self.assertAlmostEqual(out["rms_residual_m"], round(expected_rms, 6))

    def test_worst_row_picks_largest_abs_residual(self):
        rows = [_row(residual_m=0.01), _row(residual_m=-0.05), _row(residual_m=0.02)]
        out = stats.residual_stats(rows)
        self.assertAlmostEqual(out["worst_row"]["abs_residual_m"], 0.05)

    def test_suppressed_rows_sum(self):
        rows = [
            _row(suppressed=0),
            _row(suppressed=1),
            _row(suppressed=1),
        ]
        out = stats.residual_stats(rows)
        self.assertEqual(out["suppressed_rows"], 2)


class TestCoverageStats(unittest.TestCase):
    def test_counts_unique_epochs_and_pairs(self):
        rows = [
            _row(epoch_index=1, tow=10.0, reference_satellite="G01", satellite="G05"),
            _row(epoch_index=2, tow=11.0, reference_satellite="G01", satellite="G05"),
            _row(epoch_index=2, tow=11.0, reference_satellite="G01", satellite="G07"),
        ]
        out = stats.coverage_stats(rows)
        self.assertEqual(out["rows"], 3)
        self.assertEqual(out["epochs"], 2)
        self.assertEqual(out["satellite_pairs"], 2)

    def test_first_and_last_epoch_indices(self):
        rows = [
            _row(epoch_index=5, tow=50.0),
            _row(epoch_index=2, tow=20.0),
            _row(epoch_index=3, tow=30.0),
        ]
        out = stats.coverage_stats(rows)
        self.assertEqual(out["first_epoch_index"], 2)
        self.assertEqual(out["last_epoch_index"], 5)


if __name__ == "__main__":
    unittest.main()
