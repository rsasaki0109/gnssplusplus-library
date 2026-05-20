"""Unit tests for ``apps/gnss_dd_residuals_summary``."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"
sys.path.insert(0, str(APPS_DIR))

import gnss_dd_residuals_summary as summary_mod  # noqa: E402


def _row(
    *,
    epoch_index: int = 1,
    kind: str = "phase",
    freq: int = 0,
    ref: str = "G01",
    sat: str = "G05",
    residual_m: float = 0.01,
    suppressed: int = 0,
) -> dict:
    return {
        "epoch_index": epoch_index,
        "gps_week": 2200,
        "tow": float(epoch_index) * 1.0,
        "status": 4,
        "kind": kind,
        "frequency_index": freq,
        "reference_satellite": ref,
        "satellite": sat,
        "residual_m": residual_m,
        "abs_residual_m": abs(residual_m),
        "reference_variance_m2": 0.0001,
        "satellite_variance_m2": 0.0001,
        "suppressed_by_outlier_threshold": suppressed,
    }


class TestSummarizeGroups(unittest.TestCase):
    def test_decorates_groups_with_pair_label(self):
        rows = [_row(ref="G01", sat="G05"), _row(ref="G01", sat="G05")]
        groups = summary_mod.summarize_groups(
            rows, ("kind", "frequency_index", "reference_satellite", "satellite")
        )
        self.assertEqual(groups[0]["pair"], "G01-G05")
        self.assertEqual(groups[0]["frequency_label"], "freq0 primary (L1/E1/B1)")

    def test_top_n_truncates_after_descending_sort(self):
        rows = [
            _row(sat=f"G{index:02d}", residual_m=index * 0.01)
            for index in range(2, 12)  # G02..G11, residuals 0.02..0.11
        ]
        top = summary_mod.summarize_groups(
            rows,
            ("kind", "frequency_index", "reference_satellite", "satellite"),
            top_n=3,
        )
        self.assertEqual(len(top), 3)
        # Sorted by descending p95 — largest residual at index 0.
        self.assertEqual(top[0]["satellite"], "G11")


class TestSummarizeTimeSeries(unittest.TestCase):
    def test_one_row_per_unique_epoch_status(self):
        rows = [
            _row(epoch_index=1, kind="phase", residual_m=0.01),
            _row(epoch_index=1, kind="code", residual_m=0.30),
            _row(epoch_index=2, kind="phase", residual_m=0.03),
        ]
        series = summary_mod.summarize_time_series(rows)
        self.assertEqual(len(series), 2)
        self.assertEqual(series[0]["epoch_index"], 1)
        self.assertEqual(series[1]["epoch_index"], 2)

    def test_phase_and_code_aggregates_split(self):
        rows = [
            _row(epoch_index=1, kind="phase", residual_m=0.01),
            _row(epoch_index=1, kind="code", residual_m=0.50),
        ]
        series = summary_mod.summarize_time_series(rows)
        self.assertAlmostEqual(series[0]["phase_max_abs_residual_m"], 0.01)
        self.assertAlmostEqual(series[0]["code_max_abs_residual_m"], 0.50)


class TestCompactResidualPoints(unittest.TestCase):
    def test_sorted_by_epoch_then_kind_then_pair(self):
        rows = [
            _row(epoch_index=2, kind="code", sat="G10", residual_m=0.5),
            _row(epoch_index=1, kind="phase", sat="G05", residual_m=0.01),
            _row(epoch_index=1, kind="phase", sat="G03", residual_m=0.02),
        ]
        points = summary_mod.compact_residual_points(rows)
        self.assertEqual(
            [(point["epoch_index"], point["kind"], point["satellite"]) for point in points],
            [(1, "phase", "G03"), (1, "phase", "G05"), (2, "code", "G10")],
        )


class TestSummarizeRecords(unittest.TestCase):
    def test_returns_canonical_top_level_keys(self):
        rows = [_row(kind="phase"), _row(kind="code", residual_m=0.5)]
        out = summary_mod.summarize_records(rows, top_n=5)
        for key in (
            "rows",
            "coverage",
            "overall",
            "by_kind",
            "by_frequency",
            "pair_tracks",
            "top_pairs",
            "top_normalized_pairs",
            "time_series",
            "residual_points",
        ):
            self.assertIn(key, out)
        self.assertEqual(out["rows"], 2)
        self.assertIn("phase", out["by_kind"])
        self.assertIn("code", out["by_kind"])

    def test_top_pairs_capped_at_top_n(self):
        rows = [
            _row(sat=f"G{index:02d}", residual_m=index * 0.01) for index in range(2, 8)
        ]
        out = summary_mod.summarize_records(rows, top_n=3)
        self.assertLessEqual(len(out["top_pairs"]), 3)


if __name__ == "__main__":
    unittest.main()
