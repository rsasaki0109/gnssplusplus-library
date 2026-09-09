"""Launch-free Phase112 raw execution contract tests.

These tests read only contract/source/metadata artifacts.  They do not stat or
open raw phone inputs, base RINEX, truth, MAT, candidate coordinates, or
Kaggle resources, and they never launch the native solver.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase112_main_output_offset.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_manifest_v1.json"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_fgo_source_parity_freeze_v1.json"
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"


def load_contract():
    spec = importlib.util.spec_from_file_location("phase112_contract_test", CONTRACT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase112ExecutionContractTests(unittest.TestCase):
    def test_freeze_and_manifest_are_default_off_before_authorization(self) -> None:
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(freeze["phase"], 112)
        self.assertTrue(freeze["candidate"]["default_off"])
        self.assertFalse(freeze["authorization_boundary"]["structural_raw_execution_authorized"])
        self.assertFalse(manifest["freeze"]["raw_execution_authorized_before_manifest"])
        self.assertFalse(manifest["candidate"]["truth_evaluation"])
        self.assertFalse(manifest["matrix"]["solution_rows_authorized"])

    def test_exact_composed_command_contains_offset_once_and_no_truth_lane(self) -> None:
        contract = load_contract()
        for route in contract.ROUTES:
            command = contract.command_template(route)
            self.assertEqual(command.count(contract.OFFSET_SELECTOR), 1)
            self.assertEqual(command.count("--native-base-rinex"), 1)
            self.assertEqual(command.count("--native-base-rinex-sha256"), 1)
            self.assertIn(contract.PHASE93_SELECTOR, command)
            self.assertIn(contract.VECTOR_SELECTOR, command)
            self.assertIn(contract.QR_SELECTOR, command)
            self.assertNotIn("truth", " ".join(command).lower())
            self.assertNotIn(".mat", " ".join(command).lower())
            self.assertNotIn("precomputed", " ".join(command).lower())

    def test_offset_application_is_single_final_output_site(self) -> None:
        source = APP.read_text(encoding="utf-8")
        self.assertEqual(source.count("offsetFromRpy("), 1)
        self.assertEqual(source.count("position_offset_application.claim()"), 1)
        self.assertIn("position offset application attempted more than once", source)
        self.assertIn('position_offset_report.phone != "pixel5"', source)

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

    def test_structural_gate_contract_mentions_offset_and_handoff(self) -> None:
        contract = load_contract()
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(manifest["candidate"]["selector"], contract.OFFSET_SELECTOR)
        for gate in (
            "base_factors_active_exactly_once", "gnss_first_full_finite_c7_d_exact_handoff",
            "main_qr_progress_strict_cost_decrease", "offset_pixel5_exactly_once_final_output",
            "finite_earth_valid_expected_output_coverage",
        ):
            self.assertTrue(manifest["structural_gates"][gate])


if __name__ == "__main__":
    unittest.main()
