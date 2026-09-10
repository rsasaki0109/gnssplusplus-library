from __future__ import annotations

import csv
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_observable_error_correction as correction  # noqa: E402
import gnss_smartphone_observable_error_correction_eval as evaluation  # noqa: E402


FIELDS = ["MessageType", *correction.RAW_FEATURE_FIELDS]


class SmartphoneObservableCorrectionTests(unittest.TestCase):
    @staticmethod
    def _row(timestamp: int, svid: int, *, x: str = "-2704229.0") -> dict[str, str]:
        row = {field: "" for field in FIELDS}
        row.update(
            {
                "MessageType": "Raw",
                "utcTimeMillis": str(timestamp),
                "HardwareClockDiscontinuityCount": "0",
                "Svid": str(svid),
                "State": "47",
                "Cn0DbHz": str(20.0 + svid),
                "PseudorangeRateUncertaintyMetersPerSecond": "0.5",
                "AccumulatedDeltaRangeState": "18",
                "ConstellationType": "1",
                "SignalType": "GPS_L1_CA",
                "SvPositionXEcefMeters": str(20_000_000.0 + svid),
                "SvPositionYEcefMeters": str(-10_000_000.0 + svid),
                "SvPositionZEcefMeters": str(15_000_000.0 + svid),
                "WlsPositionXEcefMeters": x,
                "WlsPositionYEcefMeters": "-4288800.0",
                "WlsPositionZEcefMeters": "3856689.0",
            }
        )
        return row

    def _write(self, path: Path, *, invalid: bool = False) -> None:
        rows = [
            self._row(1_700_000_000_000, 1),
            self._row(1_700_000_000_000, 2),
            self._row(1_700_000_001_000, 1, x="-2704228.0"),
            self._row(1_700_000_001_000, 2, x="-2704228.0"),
        ]
        if invalid:
            rows[2]["Cn0DbHz"] = "nan"
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=FIELDS, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)

    def test_extracts_fixed_finite_feature_vector(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "device_gnss.csv"
            self._write(path)
            extraction = correction.extract_features(path, skip_epochs=0)
            self.assertEqual(extraction.selected_epochs, 2)
            self.assertEqual(extraction.rows[0].features.shape, (len(correction.FEATURE_NAMES),))
            self.assertTrue(np.isfinite(extraction.rows[0].features).all())
            self.assertEqual(extraction.rows[0].features[7], 2.0)
            self.assertEqual(extraction.rows[0].features[8], 2.0)
            self.assertEqual(extraction.rows[0].features[15], 2.0)

    def test_fit_and_apply_zero_residual_is_identity(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "device_gnss.csv"
            self._write(path)
            extraction = correction.extract_features(path, skip_epochs=0)
            matrix = np.vstack([row.features for row in extraction.rows])
            model = correction.fit_ridge_model(matrix, np.zeros((len(matrix), 3)))
            corrected, counts = correction.apply_model(extraction.rows, model)
            np.testing.assert_allclose(corrected, [row.ecef for row in extraction.rows], atol=1.0e-9)
            self.assertEqual(counts["base_fallback_rows"], 0)

    def test_model_contract_rejects_feature_reordering_and_wrong_alpha(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "device_gnss.csv"
            self._write(path)
            extraction = correction.extract_features(path, skip_epochs=0)
            matrix = np.vstack([row.features for row in extraction.rows])
            model = correction.fit_ridge_model(matrix, np.zeros((len(matrix), 3)))
            reordered = dict(model)
            reordered["feature_names"] = list(reversed(correction.FEATURE_NAMES))
            with self.assertRaises(correction.ObservableCorrectionError):
                correction.apply_model(extraction.rows, reordered)
            wrong_alpha = dict(model)
            wrong_alpha["alpha"] = 1.0
            with self.assertRaises(correction.ObservableCorrectionError):
                correction.apply_model(extraction.rows, wrong_alpha)

    def test_out_of_earth_range_prediction_fails_closed_to_base(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "device_gnss.csv"
            self._write(path)
            extraction = correction.extract_features(path, skip_epochs=0)
            matrix = np.vstack([row.features for row in extraction.rows])
            model = correction.fit_ridge_model(matrix, np.zeros((len(matrix), 3)))
            model["intercept_ecef_residual_m"] = [100_000_000.0, 0.0, 0.0]
            corrected, counts = correction.apply_model(extraction.rows, model)
            np.testing.assert_allclose(corrected, [row.ecef for row in extraction.rows])
            self.assertEqual(counts["base_fallback_rows"], len(extraction.rows))

    def test_feature_and_correction_artifacts_publish_atomically(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            input_path = root / "device_gnss.csv"
            self._write(input_path)
            extraction = correction.extract_features(input_path, skip_epochs=0)
            feature_manifest = correction.write_feature_artifact(
                extraction,
                root / "features.csv",
                dataset_id="fixture/phone",
                device_gnss=input_path,
            )
            model = correction.fit_ridge_model(
                np.vstack([row.features for row in extraction.rows]),
                np.zeros((len(extraction.rows), 3)),
            )
            corrected, counts = correction.apply_model(extraction.rows, model)
            manifest = correction.write_correction_outputs(
                extraction,
                corrected,
                root / "output",
                device_gnss=input_path,
                dataset_id="fixture/phone",
                model=model,
                fallback_counts=counts,
            )
            self.assertTrue((root / "features.csv").is_file())
            self.assertEqual(feature_manifest["artifact"]["sha256"], correction._sha256(root / "features.csv"))
            self.assertTrue((root / "output" / "corrected.pos").is_file())
            self.assertEqual(manifest["truth_used"], False)
            self.assertEqual(list((root / "output").glob("*.tmp")), [])

    @staticmethod
    def _metrics(h50: float, h95: float, v95: float, diag: float) -> dict:
        variants = {key: diag for key in evaluation.DIAGNOSTIC_KEYS}
        return {
            "availability_ratio": 1.0,
            "truth_coverage_ratio": 1.0,
            "horizontal_wgs84_m": {"p50_m": h50, "p95_m": h95},
            "vertical_p95_abs_m": v95,
            "kaggle_diagnostic_score_variants_m": variants,
        }

    def test_train_gate_requires_strict_aggregate_improvement(self) -> None:
        baseline = self._metrics(2.0, 4.0, 8.0, 3.0)
        same = {"baseline": baseline, "candidate": baseline}
        gate = evaluation._gate({"route/phone": same})
        self.assertFalse(gate["passed"])
        self.assertIn("aggregate_h_p95_not_strictly_improved", gate["strict_failures"])
        self.assertIn("aggregate_diagnostic_mean_not_strictly_improved", gate["strict_failures"])

    def test_train_gate_reports_non_regression_failures(self) -> None:
        baseline = self._metrics(2.0, 4.0, 8.0, 3.0)
        candidate = self._metrics(2.2, 4.1, 8.1, 3.1)
        failures = evaluation._non_regression(candidate, baseline)
        self.assertIn("h_p50_regression", failures)
        self.assertIn("h_p95_regression", failures)
        self.assertIn("v_p95_regression", failures)
        self.assertTrue(any(item.endswith("regression") for item in failures))


if __name__ == "__main__":
    unittest.main()
