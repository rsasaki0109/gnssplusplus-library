#!/usr/bin/env python3
"""Contract tests for the immutable v2.1 confirmation No-Go record."""

import hashlib
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v2_1_confirmation_freeze.json"
FREEZE_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v2_1_confirmation_freeze_manifest.json"
RESULT = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v2_1_confirmation_result.json"
RESULT_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v2_1_confirmation_result_manifest.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class SmartphoneNativeFgoV21ConfirmationContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        cls.freeze_manifest = json.loads(FREEZE_MANIFEST.read_text(encoding="utf-8"))
        cls.result = json.loads(RESULT.read_text(encoding="utf-8"))
        cls.result_manifest = json.loads(RESULT_MANIFEST.read_text(encoding="utf-8"))

    def test_selection_is_pre_materialization_and_unused(self):
        self.assertEqual(
            self.freeze["status"],
            "frozen-before-confirmation-payload-materialization",
        )
        selected = self.freeze["split_inventory"]["selected_confirmation_set"]
        self.assertEqual(
            selected,
            [
                "2021-03-10-23-13-us-ca-mtv-h/pixel5",
                "2022-04-01-18-22-us-ca-lax-t/pixel5",
            ],
        )
        self.assertTrue(self.freeze["split_inventory"]["selected_identities_are_in_inventory_unused_metadata_candidates"])
        self.assertTrue(self.freeze["split_inventory"]["selected_identities_never_scored_or_truth_opened_per_inventory"])
        self.assertFalse(self.freeze["truth_policy"]["confirmation_payload_materialized_before_this_record"])
        self.assertFalse(self.freeze["truth_policy"]["confirmation_truth_opened_before_this_record"])
        for row in self.freeze["central_directory_metadata"].values():
            truth = row["ground_truth.csv"]
            self.assertGreater(truth["file_size"], 0)
            self.assertNotEqual(truth["crc32_hex"], "metadata-only-not-read")

    def test_algorithm_identity_and_gate_are_frozen(self):
        self.assertEqual(
            self.freeze["frozen_v2_1_recipe"]["source_record_sha256"],
            "6c33400e97345cc7f966c3ff5ff7a946b4fb6f89bb4c35cf56bc95654f663eae",
        )
        self.assertEqual(
            self.freeze["immutable_source_hashes"]["build/apps/gnss_fgo_imu_v21"],
            "8b978458735996ea571baa9975be70c37287f84fec029ced187b9f30fbb6d483",
        )
        self.assertTrue(self.freeze["predeclared_confirmation_gate"]["per_route"]["all_four_diagnostic_variants_strict_improvement"])
        self.assertEqual(
            self.freeze["predeclared_confirmation_gate"]["per_route"]["vertical_p95_catastrophic_margin_m"],
            100.0,
        )

    def test_result_is_fail_closed_without_truth(self):
        self.assertEqual(self.result["status"], "final-no-go-structural-heading-gate")
        self.assertFalse(self.result["decision"]["passed"])
        self.assertFalse(self.result["decision"]["fresh_validation_allowed"])
        self.assertEqual(self.result["truth_policy"]["confirmation_truth_open_count"], 0)
        self.assertFalse(self.result["truth_policy"]["confirmation_truth_materialized"])
        for route in self.result["route_results"]:
            self.assertEqual(route["candidate"]["return_code"], 1)
            self.assertFalse(route["candidate"]["output_published"])
            self.assertFalse(route["structural_gate"]["passed"])

    def test_manifests_bind_records_and_artifacts(self):
        self.assertTrue(FREEZE_MANIFEST.exists())
        self.assertTrue(RESULT_MANIFEST.exists())
        self.assertEqual(sha256(FREEZE), self.freeze_manifest["freeze_record_sha256"])
        self.assertEqual(sha256(RESULT), self.result_manifest["result_record_sha256"])
        self.assertEqual(sha256(FREEZE_MANIFEST), self.result_manifest["freeze_manifest_sha256"])
        for route in self.result["route_results"]:
            route_dir = ROOT / "output/smartphone-r5/native-fgo-v2-1-confirmation-v1/routes" / route["dataset_id"].replace("/", "__")
            self.assertFalse((route_dir / "inputs/ground_truth.csv").exists())
            self.assertFalse((route_dir / "candidate-v21/submission.csv").exists())


if __name__ == "__main__":
    unittest.main()
