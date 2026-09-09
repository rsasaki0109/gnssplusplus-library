#!/usr/bin/env python3
"""Contract tests for the sealed Phase 38 native-state diagnosis No-Go."""

import hashlib
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase38_mtvh_failure_diagnosis_freeze_v1.json"
RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase38_mtvh_failure_diagnosis_result_v1.json"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
NATIVE_SOURCE = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class SmartphonePhase38MtvhFailureDiagnosisTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        cls.result = json.loads(RESULT.read_text(encoding="utf-8"))

    def test_result_is_sealed_native_state_no_go(self):
        self.assertEqual(self.result["status"], "sealed-diagnosis-no-go")
        self.assertFalse(self.result["decision"]["passed"])
        self.assertEqual(
            self.result["decision"]["classification"],
            "native_solver_state_failure",
        )
        self.assertEqual(self.result["authority"]["freeze_commit"], "2f9c7dd1ca8088aa9724a836f8199a377c8d28a2")
        self.assertEqual(
            sha256(FREEZE), self.result["authority"]["freeze_record_sha256"]
        )

    def test_runtime_and_native_state_evidence_are_preserved(self):
        runs = self.result["native_baseline"]["runs"]
        self.assertEqual([run["return_code"] for run in runs], [127, 1, 1])
        self.assertEqual(
            runs[0]["classification"], "environment_only_loader_failure"
        )
        self.assertEqual(
            runs[2]["command_environment"], "LD_LIBRARY_PATH=/home/sasaki/.local/lib"
        )
        gnss_first = self.result["native_state_evidence"]["gnss_first"]
        self.assertTrue(gnss_first["converged"])
        primary_scan = gnss_first["primary_gdb_scan"]
        self.assertEqual(primary_scan["invalid_states"], 611)
        self.assertEqual(primary_scan["first_invalid_index"], 0)
        self.assertAlmostEqual(
            primary_scan["first_invalid_ecef_norm_m"], 7432673.743891388
        )
        seeds = self.result["native_state_evidence"][
            "original_raw_only_seeds_before_handoff"
        ]
        self.assertEqual(seeds["states"], 1325)
        self.assertEqual(seeds["invalid_states"], 0)
        self.assertTrue(seeds["all_epochs_earth_valid"])

    def test_candidate_is_fail_closed_and_matrix_is_not_promoted(self):
        candidate = self.result["candidate_probe"]
        self.assertFalse(candidate["implementation_committed"])
        self.assertTrue(candidate["source_after_probe_reverted_to_freeze"])
        self.assertFalse(candidate["matrix_executed"])
        self.assertEqual(
            [run["return_code"] for run in candidate["exploratory_runs"]], [1, 1]
        )
        for run in candidate["exploratory_runs"]:
            self.assertIn("built=2085 inserted=0", run["failure"])
        self.assertEqual(
            self.result["structural_matrix"]["executed"], False
        )
        self.assertEqual(self.result["truth_accounting"]["allowed_truth_reads"], 0)
        self.assertEqual(self.result["truth_accounting"]["mat_inputs_or_outputs"], 0)
        self.assertEqual(self.result["truth_accounting"]["validation_truth_reads"], 0)
        self.assertEqual(self.result["truth_accounting"]["holdout_truth_reads"], 0)
        self.assertEqual(self.result["truth_accounting"]["kaggle_or_token_access"], 0)

    def test_flag_off_identity_is_explicitly_preserved(self):
        identity = self.result["candidate_probe"]["flag_off_byte_identity"]
        self.assertEqual(identity["status"], "preserved-no-candidate-commit")
        self.assertFalse(identity["source_flag_materialized"])
        self.assertFalse(identity["production_default_changed"])
        self.assertEqual(
            identity["frozen_binary_sha256"],
            self.result["native_baseline"]["binary_sha256"],
        )
        self.assertNotIn("--native-mtvh-failure-recovery", NATIVE_SOURCE.read_text(encoding="utf-8"))
        if BINARY.exists():
            self.assertEqual(
                sha256(BINARY), self.result["native_baseline"]["binary_sha256"]
            )


if __name__ == "__main__":
    unittest.main()
