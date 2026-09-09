"""Launch-free Phase117 raw structural contract tests.

The tests inspect only source and sealed JSON metadata.  They never stat or
open raw phone/base members, truth, MAT, candidate coordinates, or solutions,
and never launch the native solver.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase117_tdcp_weighting.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_manifest_v1.json"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_source_parity_freeze_v1.json"


def load_contract():
    spec = importlib.util.spec_from_file_location("phase117_contract_test", CONTRACT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase117ExecutionContractTests(unittest.TestCase):
    def test_freeze_and_manifest_are_default_off_before_authorization(self) -> None:
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(freeze["phase"], 117)
        self.assertTrue(freeze["candidate"]["default_off"])
        self.assertFalse(freeze["decision"]["raw_execution_authorized"])
        self.assertFalse(manifest["freeze"]["raw_execution_authorized_before_manifest"])
        self.assertFalse(manifest["candidate"]["truth_evaluation"])
        self.assertFalse(manifest["matrix"]["solution_rows_authorized"])

    def test_exact_command_contains_phase117_once_and_forbids_truth_lanes(self) -> None:
        contract = load_contract()
        for route in contract.ROUTES:
            command = contract.command_template(route)
            self.assertEqual(command.count(contract.PHASE117_SELECTOR), 1)
            self.assertEqual(command.count(contract.PHASE116_SELECTOR), 1)
            self.assertEqual(command.count("--native-base-rinex"), 1)
            self.assertEqual(command.count("--native-base-rinex-sha256"), 1)
            self.assertIn(contract.PHASE93_SELECTOR, command)
            self.assertIn(contract.VECTOR_SELECTOR, command)
            self.assertIn(contract.QR_SELECTOR, command)
            rendered = " ".join(command).lower()
            self.assertNotIn("truth", rendered)
            self.assertNotIn(".mat", rendered)
            self.assertNotIn("precomputed", rendered)

    def test_pre_raw_verifier_reports_zero_payload_activity(self) -> None:
        contract = load_contract()
        pre = contract.verify_pre_raw()
        for key in (
            "raw_phone_gnss_reads", "raw_phone_imu_reads", "broadcast_navigation_reads",
            "raw_base_rinex_reads", "raw_base_hash_reads", "native_solver_invocations",
            "truth_reads", "mat_reads_or_generated", "phone_coordinate_reads",
            "precomputed_coordinate_reads", "pdc_reads", "accuracy_calculations",
        ):
            self.assertEqual(pre[key], 0, key)
        self.assertFalse(pre["raw_execution_authorized"])

    def test_manifest_freezes_official_sigma_contract_and_gates(self) -> None:
        contract = load_contract()
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(manifest["candidate"]["selector"], contract.PHASE117_SELECTOR)
        self.assertEqual(manifest["candidate"]["official_snr_percentile"], 85)
        self.assertEqual(manifest["candidate"]["official_snr_denominator"], 20)
        self.assertEqual(manifest["candidate"]["official_l_sn_ratio"], 0.0025)
        self.assertEqual(manifest["candidate"]["fixed_legacy_tdcp_sigma_m"], 0.03)
        self.assertEqual(manifest["candidate"]["sigma_units"], "source L cycles * retained wavelength_m = native metres")
        for gate in (
            "official_sigma_all_finite_source_derived", "valid_metadata_factor_count_unchanged",
            "missing_metadata_fail_closed_no_legacy_fallback", "tdcp_residual_keys_reject_predicate_unchanged",
            "gnss_first_progress_strict_cost_decrease", "gnss_first_full_finite_c7_d_exact_handoff",
            "main_qr_progress_strict_cost_decrease", "base_correction_exactly_once",
            "offset_exactly_once_final_boundary", "finite_expected_output_coverage",
        ):
            self.assertTrue(manifest["structural_gates"][gate], gate)


if __name__ == "__main__":
    unittest.main()
