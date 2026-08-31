from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_wls_multi_phone_ensemble_eval as ensemble  # noqa: E402
import gnss_smartphone_wls_multi_phone_ensemble_holdout_eval as holdout  # noqa: E402


class SmartphoneWlsMultiPhoneEnsembleHoldoutTests(unittest.TestCase):
    def test_frozen_contract_is_truth_free_and_atomic(self) -> None:
        contract = holdout.frozen_contract()
        self.assertEqual(contract["route"], holdout.HOLDOUT_ROUTE)
        self.assertEqual(contract["selected_method"], "coordinate_wise_ecef_median")
        self.assertEqual(
            contract["algorithm_parameter_hash"],
            holdout.wls.MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH,
        )
        self.assertEqual(
            contract["algorithm_core_hash"],
            holdout.wls.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH,
        )
        self.assertEqual(contract["alignment_tolerance_ms"], 10)
        self.assertEqual(
            holdout.FREEZE_SCHEMA_VERSION,
            "smartphone-r5-wls-multi-phone-ensemble-holdout-freeze.v1.4",
        )
        self.assertEqual(
            holdout.wls.MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_4,
            holdout.FREEZE_SCHEMA_VERSION,
        )
        self.assertEqual(contract["minimum_aligned_phone_count"], 1)
        self.assertEqual(contract["wls_integrity"]["timestamp_gap_threshold_ms"], 1500)
        self.assertTrue(contract["wls_integrity"]["allow_missing_wls_epochs"])
        self.assertTrue(contract["wls_integrity"]["allow_timestamp_gaps_as_diagnostic"])
        self.assertEqual(contract["wls_integrity"]["extrapolation_policy"], "forbidden")
        self.assertEqual(
            contract["wls_integrity"]["partial_coordinate_triplet_policy"],
            "fail-closed",
        )
        self.assertEqual(
            tuple(contract["atomic_key_contract"]["submission_fields"]),
            ensemble.kaggle.SUBMISSION_FIELDS,
        )
        self.assertEqual(tuple(contract["official_diagnostics"]), ensemble.DIAGNOSTIC_KEYS)
        self.assertTrue(contract["no_post_holdout_tuning"])
        self.assertTrue(contract["truth_free_before_truth"])
        self.assertEqual(contract["truth_evaluation_pass_count"], 1)

    def test_central_metadata_fixture_preflight_does_not_need_payload_hashes(self) -> None:
        route = holdout.HOLDOUT_ROUTE
        dataset_id = f"{route}/sm-a205u"
        names = ensemble._member_names(dataset_id)
        with tempfile.TemporaryDirectory() as temporary:
            archive_path = Path(temporary) / "fixture.zip"
            with zipfile.ZipFile(archive_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
                archive.writestr(names["device_gnss"], b"device fixture")
                archive.writestr(names["ground_truth"], b"truth fixture")
                archive.writestr(f"dataset_2023/train/{route}/brdc.nav", b"nav fixture")
            metadata = lambda member: ensemble._central_metadata(archive_path, member)
            record = {
                "dataset_id": dataset_id,
                "device_gnss": metadata(names["device_gnss"]),
                "ground_truth": metadata(names["ground_truth"]),
            }
            freeze = {
                "holdout": {
                    "central_directory_broadcast_nav": metadata(
                        f"dataset_2023/train/{route}/brdc.nav"
                    ),
                    "records": [record],
                }
            }
            holdout._verify_holdout_central_metadata(archive_path, freeze)
            record["device_gnss"] = dict(record["device_gnss"])
            record["device_gnss"]["file_size"] += 1
            with self.assertRaises(holdout.HoldoutEnsembleError):
                holdout._verify_holdout_central_metadata(archive_path, freeze)

    def test_wls_integrity_fixture_is_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            summary = {
                "schema_version": "smartphone-wls-position.v1",
                "truth_free": True,
                "truth_used": False,
                "populations": {
                    "timestamp_gap_count_gt_1500ms": 0,
                    "max_timestamp_gap_s": 1.0,
                    "selected_epochs": 2,
                },
                "validation": {
                    "all_epoch_rows_consistent": True,
                    "all_selected_rows_finite": True,
                    "allow_missing_wls_epochs": True,
                    "classification_counts": {
                        "timestamp_gap": 0,
                        "nonfinite_wls_ecef": 0,
                    },
                },
            }
            manifest = {
                "truth_free": True,
                "truth_used": False,
                "contract": {"allow_missing_wls_epochs": True},
            }
            summary_path = root / "summary.json"
            manifest_path = root / "manifest.json"
            summary_path.write_text(json.dumps(summary), encoding="utf-8")
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            artifacts = {
                "phone": {
                    "summary": {"path": str(summary_path)},
                    "manifest": {"path": str(manifest_path)},
                }
            }
            result = holdout._verify_wls_integrity(artifacts)
            self.assertEqual(result["phone"]["timestamp_gap_count_gt_1500ms"], 0)
            summary["validation"]["classification_counts"]["timestamp_gap"] = 1
            summary_path.write_text(json.dumps(summary), encoding="utf-8")
            with self.assertRaises(holdout.HoldoutEnsembleError):
                holdout._verify_wls_integrity(artifacts)

    def test_wls_integrity_accepts_gap_only_with_v1_3_diagnostic_contract(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            summary = {
                "schema_version": "smartphone-wls-position.v1",
                "truth_free": True,
                "truth_used": False,
                "populations": {
                    "timestamp_gap_count_gt_1500ms": 1,
                    "max_timestamp_gap_s": 2.001,
                    "selected_epochs": 2,
                },
                "validation": {
                    "all_epoch_rows_consistent": True,
                    "all_selected_rows_finite": True,
                    "allow_missing_wls_epochs": True,
                    "timestamp_gap_allowed_as_diagnostic": True,
                    "classification_counts": {
                        "timestamp_gap": 1,
                        "blank_wls_row": 0,
                        "sparse_omitted_epoch": 0,
                    },
                },
            }
            manifest = {
                "truth_free": True,
                "truth_used": False,
                "contract": {
                    "allow_missing_wls_epochs": True,
                    "timestamp_gap_allowed_as_diagnostic": True,
                },
            }
            summary_path = root / "summary.json"
            manifest_path = root / "manifest.json"
            summary_path.write_text(json.dumps(summary), encoding="utf-8")
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            artifacts = {
                "phone": {
                    "summary": {"path": str(summary_path)},
                    "manifest": {"path": str(manifest_path)},
                }
            }
            result = holdout._verify_wls_integrity(artifacts)
            self.assertTrue(result["phone"]["timestamp_gap_allowed_as_diagnostic"])
            self.assertEqual(result["phone"]["timestamp_gap_count_gt_1500ms"], 1)


if __name__ == "__main__":
    unittest.main()
