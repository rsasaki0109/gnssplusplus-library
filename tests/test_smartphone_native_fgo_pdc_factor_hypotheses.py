#!/usr/bin/env python3
"""Regression contract for the sealed truth-free PDC hypothesis audit."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
RECORD = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_factor_hypothesis_audit_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_factor_hypothesis_audit_v1_manifest.json"
AUDIT = ROOT / "output/smartphone-r5/native-fgo-pdc-factor-audit-v1/factor_audit.json"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class NativeFgoPdcFactorHypothesisTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.record = json.loads(RECORD.read_text(encoding="utf-8"))
        cls.manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        cls.audit = json.loads(AUDIT.read_text(encoding="utf-8"))

    def test_record_is_sealed_without_truth_or_candidate_mutation(self) -> None:
        self.assertEqual(self.record["status"], "truth-free-hypothesis-audit-sealed")
        policy = self.record["policy"]
        for key in ("truth_opened", "validation_opened", "holdout_opened", "test_data_used", "kaggle_or_token_access", "external_mutation"):
            self.assertFalse(policy[key])
        self.assertFalse(policy["candidate_or_parameter_changed"])
        self.assertFalse(policy["production_default_changed"])
        self.assertFalse(policy["v5_artifact_touched"])
        self.assertIsNone(self.record["structural_decision"]["first_structural_failure"])

    def test_hash_chain_and_source_pins_match(self) -> None:
        self.assertEqual(sha256(RECORD), self.manifest["record_sha256"])
        self.assertEqual(sha256(AUDIT), self.manifest["audit_artifact_sha256"])
        self.assertEqual(self.record["sealed_inputs"]["audit_artifact_sha256"], self.manifest["audit_artifact_sha256"])
        for relative_path, expected in self.manifest["source_hashes"].items():
            accepted = {expected}
            compatibility = self.manifest.get("post_phase9_compatible_source_hashes", {})
            if relative_path in compatibility:
                accepted.add(compatibility[relative_path])
            phase10_compatibility = self.manifest.get(
                "post_phase10_compatible_source_hashes", {}
            )
            if relative_path in phase10_compatibility:
                accepted.add(phase10_compatibility[relative_path])
            self.assertIn(sha256(ROOT / relative_path), accepted, relative_path)
        current_binary_sha256 = sha256(ROOT / "build/apps/gnss_fgo")
        accepted_binary_sha256 = {
            self.manifest["release_binary_sha256"],
            self.manifest.get("post_phase5_compatible_release_binary_sha256"),
            self.manifest.get("post_phase7_compatible_release_binary_sha256"),
            self.manifest.get("post_phase9_compatible_release_binary_sha256"),
            self.manifest.get("post_phase10_compatible_release_binary_sha256"),
        }
        self.assertIn(current_binary_sha256, accepted_binary_sha256)

    def test_stage_matrix_and_first_degradation_are_preserved(self) -> None:
        stages = self.record["diagnostic_matrix"]["stages"]
        self.assertEqual(len(stages), 10)
        self.assertEqual([stage["id"] for stage in stages], self.manifest["stage_ids"])
        self.assertTrue(all(stage["global_cost"]["final"] <= stage["global_cost"]["initial"] for stage in stages))
        self.assertTrue(all(stage["termination"]["converged"] for stage in stages if stage["id"].startswith(("D_", "E_", "F_"))))
        d30 = next(stage for stage in stages if stage["id"] == "D_P_D_velocity_motion_30")
        e30 = next(stage for stage in stages if stage["id"] == "E_P_D_TDCP_30")
        self.assertAlmostEqual(d30["rms"]["D_mps"], 0.136764830452978)
        self.assertGreater(e30["rms"]["D_mps"], d30["rms"]["D_mps"])
        self.assertEqual(e30["factors"]["TDCP"], 337)

    def test_hypothesis_verdicts_are_explicit_and_parity_is_not_claimed(self) -> None:
        hypotheses = {item["id"]: item for item in self.record["hypotheses"]}
        for expected in ("H1_absolute_ecef_derivative_conditioning", "H2_clock_bias_seconds_vs_metres", "H3_clock_drift_dt_coupling", "H4_velocity_frame_or_los_sign", "H5_satellite_term_or_sagnac", "H6_motion_sigma_or_temporal_gauge", "H7_duplicate_or_conflicting_tdcp", "H8_clock_bias_inter_system_gauge", "H9_taroz_numeric_parity"):
            self.assertIn(expected, hypotheses)
        self.assertEqual(hypotheses["H9_taroz_numeric_parity"]["verdict"], "unavailable")
        self.assertFalse(self.record["sealed_inputs"]["route_directory"].endswith("ground_truth"))
        self.assertEqual(self.record["diagnostic_matrix"]["family_initial_cost_contract"]["status"], "partially-available")


if __name__ == "__main__":
    unittest.main()
