"""Unit tests for ``apps/gnss_dd_residuals_records``."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"
sys.path.insert(0, str(APPS_DIR))

import gnss_dd_residuals_records as records  # noqa: E402


def _row(
    *,
    residual_m: float = 0.01,
    reference_variance: float | None = 0.0004,
    satellite_variance: float | None = 0.0004,
    suppressed: int = 0,
) -> dict:
    return {
        "epoch_index": 1,
        "gps_week": 2200,
        "tow": 123.0,
        "status": 4,
        "kind": "phase",
        "frequency_index": 0,
        "reference_satellite": "G01",
        "satellite": "G05",
        "residual_m": residual_m,
        "abs_residual_m": abs(residual_m),
        "reference_variance_m2": reference_variance,
        "satellite_variance_m2": satellite_variance,
        "suppressed_by_outlier_threshold": suppressed,
    }


class TestFrequencyLabel(unittest.TestCase):
    def test_known_indexes(self):
        self.assertEqual(records.frequency_label(0), "freq0 primary (L1/E1/B1)")
        self.assertEqual(records.frequency_label(1), "freq1 secondary (L2/E5/B2)")
        self.assertEqual(records.frequency_label(2), "freq2 L5-class (L5/E5a/B2a)")

    def test_unknown_index_falls_back_to_generic_label(self):
        self.assertEqual(records.frequency_label(7), "freq7")


class TestPairLabel(unittest.TestCase):
    def test_concatenates_with_hyphen(self):
        self.assertEqual(records.pair_label("G01", "G05"), "G01-G05")

    def test_accepts_non_string_inputs(self):
        self.assertEqual(records.pair_label(1, 5), "1-5")


class TestResidualSigma(unittest.TestCase):
    def test_returns_sqrt_of_sum(self):
        sigma = records.residual_sigma_m(_row(reference_variance=0.04, satellite_variance=0.09))
        self.assertAlmostEqual(sigma, math.sqrt(0.13), places=9)

    def test_returns_none_when_either_variance_missing(self):
        self.assertIsNone(records.residual_sigma_m(_row(reference_variance=None)))
        self.assertIsNone(records.residual_sigma_m(_row(satellite_variance=None)))

    def test_returns_none_for_nonpositive_total_variance(self):
        self.assertIsNone(
            records.residual_sigma_m(_row(reference_variance=0.0, satellite_variance=0.0))
        )
        self.assertIsNone(
            records.residual_sigma_m(_row(reference_variance=-0.1, satellite_variance=0.05))
        )

    def test_returns_none_for_nonfinite_inputs(self):
        self.assertIsNone(
            records.residual_sigma_m(
                _row(reference_variance=float("nan"), satellite_variance=0.04)
            )
        )


class TestNormalizedResidual(unittest.TestCase):
    def test_divides_residual_by_sigma(self):
        row = _row(residual_m=0.03, reference_variance=0.0009, satellite_variance=0.0)
        # sigma = sqrt(0.0009) = 0.03 → normalized = 1.0
        self.assertAlmostEqual(records.normalized_residual(row), 1.0, places=9)

    def test_returns_none_when_sigma_undefined(self):
        self.assertIsNone(records.normalized_residual(_row(reference_variance=None)))


class TestAbsNormalizedResidual(unittest.TestCase):
    def test_returns_magnitude(self):
        row = _row(residual_m=-0.03, reference_variance=0.0009, satellite_variance=0.0)
        self.assertAlmostEqual(records.abs_normalized_residual(row), 1.0, places=9)

    def test_propagates_none(self):
        self.assertIsNone(records.abs_normalized_residual(_row(reference_variance=None)))


class TestRounded(unittest.TestCase):
    def test_rounds_to_six_decimals(self):
        self.assertAlmostEqual(records.rounded(1.0 / 3.0), 0.333333)

    def test_passes_through_none(self):
        self.assertIsNone(records.rounded(None))


class TestCompactRow(unittest.TestCase):
    def test_adds_pair_and_label_annotations(self):
        compact = records.compact_row(_row())
        self.assertEqual(compact["pair"], "G01-G05")
        self.assertEqual(compact["frequency_label"], "freq0 primary (L1/E1/B1)")

    def test_carries_normalized_residual_when_sigma_finite(self):
        compact = records.compact_row(
            _row(residual_m=0.03, reference_variance=0.0009, satellite_variance=0.0)
        )
        self.assertAlmostEqual(compact["normalized_residual"], 1.0, places=6)
        self.assertAlmostEqual(compact["abs_normalized_residual"], 1.0, places=6)

    def test_returns_none_for_normalized_when_sigma_missing(self):
        compact = records.compact_row(_row(reference_variance=None))
        self.assertIsNone(compact["normalized_residual"])
        self.assertIsNone(compact["abs_normalized_residual"])

    def test_drops_raw_variance_fields(self):
        compact = records.compact_row(_row())
        self.assertNotIn("reference_variance_m2", compact)
        self.assertNotIn("satellite_variance_m2", compact)


if __name__ == "__main__":
    unittest.main()
