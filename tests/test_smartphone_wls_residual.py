from __future__ import annotations

import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_wls as wls  # noqa: E402
import gnss_smartphone_wls_residual as residual  # noqa: E402
import gnss_smartphone_wls_residual_eval as evaluation  # noqa: E402


FIELDS = (
    "MessageType",
    "utcTimeMillis",
    "HardwareClockDiscontinuityCount",
    "Svid",
    "WlsPositionXEcefMeters",
    "WlsPositionYEcefMeters",
    "WlsPositionZEcefMeters",
)


class SmartphoneWlsResidualTests(unittest.TestCase):
    def _write_device(self, path: Path) -> None:
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=FIELDS, lineterminator="\n")
            writer.writeheader()
            for index, timestamp in enumerate((1_609_797_015_000, 1_609_797_016_000, 1_609_797_017_000)):
                writer.writerow(
                    {
                        "MessageType": "Raw",
                        "utcTimeMillis": timestamp,
                        "HardwareClockDiscontinuityCount": 0,
                        "Svid": index + 1,
                        "WlsPositionXEcefMeters": -2704229.0 + index,
                        "WlsPositionYEcefMeters": -4288800.0,
                        "WlsPositionZEcefMeters": 3856689.0,
                    }
                )

    def test_candidate_set_is_fixed_to_nine_truth_free_variants(self) -> None:
        self.assertEqual(len(residual.PREDECLARED_CANDIDATES), 9)
        self.assertEqual(
            {candidate.candidate_id for candidate in residual.PREDECLARED_CANDIDATES},
            {
                "raw_shift_m1",
                "raw_shift_0",
                "raw_shift_p1",
                "median3_shift_m1",
                "median3_shift_0",
                "median3_shift_p1",
                "median5_shift_m1",
                "median5_shift_0",
                "median5_shift_p1",
            },
        )
        self.assertEqual(residual.candidate_spec("median3_shift_0").along_shift_m, 0.0)
        with self.assertRaises(residual.WlsResidualCandidateError):
            residual.candidate_spec("truth_tuned")

    def test_candidate_publishes_finite_truth_free_atomic_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            device = root / "device_gnss.csv"
            self._write_device(device)
            source_dir = root / "source"
            wls.extract_to_directory(device, source_dir, skip_epochs=0)
            output = root / "candidate.pos"
            manifest = root / "candidate.manifest.json"
            payload = residual.write_candidate(
                source_dir / "wls.pos",
                output,
                "median3_shift_0",
                manifest_path=manifest,
            )
            self.assertTrue(output.is_file())
            self.assertTrue(manifest.is_file())
            loaded = json.loads(manifest.read_text(encoding="utf-8"))
            self.assertTrue(loaded["truth_free"])
            self.assertFalse(loaded["truth_used"])
            self.assertFalse(loaded["device_model_used"])
            self.assertEqual(payload["output"]["sha256"], residual._sha256(output))
            self.assertEqual(payload["manifest"]["sha256"], residual._sha256(manifest))
            rows = residual.smoother._read_positions(output, 18)
            self.assertTrue(all(row.ecef.shape == (3,) for row in rows))
            self.assertTrue(all(float(value) == float(value) for row in rows for value in row.ecef))

    def test_selection_record_proves_next_holdout_and_old_holdout_are_sealed(self) -> None:
        record = evaluation._load_selection_record(evaluation.DEFAULT_SELECTION_RECORD)
        self.assertEqual(record["new_validation"]["dataset_id"], evaluation.NEW_VALIDATION_ID)
        self.assertEqual(record["next_holdout"]["dataset_id"], evaluation.NEXT_HOLDOUT_ID)
        self.assertTrue(record["next_holdout"]["materialization_forbidden"])
        self.assertTrue(record["next_holdout"]["truth_open_forbidden"])
        self.assertFalse(record["candidate_source_and_selection_policy"]["old_holdout_run2_metrics_used_for_ranking"])
        self.assertFalse(record["archive"]["member_content_read_at_selection"])

    def test_horizontal_gate_requires_each_diagnostic_variant(self) -> None:
        reference = {
            "horizontal_wgs84_m": {"p50_m": 3.0, "p95_m": 6.0},
            "kaggle_diagnostic_score_variants_m": {key: 4.0 for key in evaluation.DIAGNOSTIC_KEYS},
        }
        candidate = {
            "horizontal_wgs84_m": {"p50_m": 2.0, "p95_m": 5.0},
            "kaggle_diagnostic_score_variants_m": {key: 3.0 for key in evaluation.DIAGNOSTIC_KEYS},
        }
        self.assertTrue(evaluation._strict_horizontal_better(candidate, reference))
        candidate["kaggle_diagnostic_score_variants_m"][evaluation.DIAGNOSTIC_KEYS[-1]] = 4.0
        self.assertFalse(evaluation._strict_horizontal_better(candidate, reference))

    def test_aggregate_gate_uses_mean_metric_names(self) -> None:
        reference = {
            "mean_availability_ratio": 1.0,
            "mean_horizontal_wgs84_p50_m": 3.0,
            "mean_horizontal_wgs84_p95_m": 6.0,
            "mean_vertical_p95_abs_m": 12.0,
            "mean_kaggle_diagnostic_score_variants_m": {
                key: 4.0 for key in evaluation.DIAGNOSTIC_KEYS
            },
        }
        candidate = {
            "mean_availability_ratio": 1.0,
            "mean_horizontal_wgs84_p50_m": 2.0,
            "mean_horizontal_wgs84_p95_m": 5.0,
            "mean_vertical_p95_abs_m": 11.0,
            "mean_kaggle_diagnostic_score_variants_m": {
                key: 3.0 for key in evaluation.DIAGNOSTIC_KEYS
            },
        }
        self.assertTrue(evaluation._non_regression(candidate, reference)[0])
        self.assertTrue(evaluation._strict_horizontal_better(candidate, reference))


if __name__ == "__main__":
    unittest.main()
