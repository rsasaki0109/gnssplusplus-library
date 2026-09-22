"""Independent examples for the covariance regression report's metrics."""
from __future__ import annotations

import importlib.util
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "covariance_comparison", ROOT / "scripts/analysis/compare_fixed_lag_covariance.py")
assert SPEC is not None and SPEC.loader is not None
comparison = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(comparison)


def row(tow: int, error: float, status: str = "FIXED", trace: str = "0.000001") -> dict:
    return dict(tow=str(tow), status=status, x_ecef_m="1", y_ecef_m="2", z_ecef_m="3",
                e_err_m=str(error), n_err_m="0", u_err_m="0", position_covariance_trace_m2=trace)


class CovarianceComparisonTest(unittest.TestCase):
    def test_missing_epoch_denominator_uses_inputs_before_filtering(self):
        log = "  total solutions: 3\n  exact base epochs: 2\n  interpolated base epochs: 7\n  skipped rover epochs: 1\n"
        self.assertEqual(comparison.native_input_epochs(log, "rtk"), 10)
        fgo = "  rover=E:/data/rover.obs (10 epochs capped)\nlag=5 epochs=8\n"
        self.assertEqual(comparison.native_input_epochs(fgo, "fgo"), 10)
        for kind in ("rtk", "fgo"):
            with self.assertRaises(ValueError):
                comparison.native_input_epochs("total solutions: 3\n", kind)

    def test_wrong_fix_recovery_and_censoring_have_explicit_denominators(self):
        rows = [row(0, 3), row(1, 4), row(2, 0.1, "FLOAT"),
                row(3, 0.1), row(4, 5), row(5, 0.1, "FLOAT")]
        report = comparison.summarize(rows, wrong_m=2.0, expected=8)
        self.assertEqual(report["fixed_epochs"], 4)
        self.assertEqual(report["wrong_fix_epochs"], 3)
        self.assertEqual(report["wrong_fix_pct_fixed"], 75)
        self.assertEqual(report["correct_fix_epochs"], 1)
        self.assertEqual(report["wrong_fix_events"], 2)
        self.assertEqual(report["recovered_events"], 1)
        self.assertEqual(report["unrecovered_events"], 1)
        self.assertEqual(report["recovery_delay_p50_s"], 2)
        self.assertEqual(report["missing_or_unmatched_pct"], 25)
        self.assertEqual(report["positive_covariance_rows"], 6)
        self.assertAlmostEqual(report["covariance_trace_p50_m2"], 1e-6)

    def test_missing_covariance_and_zero_fix_denominator_are_not_zero_accuracy(self):
        rows = [row(0, 1, "FLOAT", ""), row(1, 3, "FLOAT", "nan"),
                row(2, 2, "FLOAT", "-1"), row(3, 4, "FLOAT", "0")]
        report = comparison.summarize(rows, wrong_m=2.0, expected=None)
        self.assertIsNone(report["wrong_fix_pct_fixed"])
        self.assertIsNone(report["covariance_trace_p50_m2"])
        self.assertIsNone(report["missing_or_unmatched_epochs"])
        self.assertIsNone(report["recovery_delay_p50_s"])
        self.assertEqual(report["positive_covariance_rows"], 0)
        self.assertAlmostEqual(report["horizontal_p95_m"], 3.85)
        with self.assertRaises(ValueError):
            comparison.summarize(rows, 2.0, expected=3)

    def test_loader_rejects_duplicate_times_instead_of_silently_joining_them(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "epochs.csv"
            path.write_text("tow,status,x_ecef_m,y_ecef_m,z_ecef_m,e_err_m,n_err_m,u_err_m\n"
                            "1,FIXED,1,2,3,0,0,0\n1,FIXED,1,2,3,0,0,0\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "strictly increasing"):
                comparison.load(path)


if __name__ == "__main__":
    unittest.main()
