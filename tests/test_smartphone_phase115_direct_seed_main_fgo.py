"""Launch-free Phase115 recipe-correction contract tests."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase115_direct_seed_main_fgo.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase115_direct_seed_main_fgo_manifest_v1.json"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase115_recipe_correction_freeze_v1.json"
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"


def load_contract():
    spec = importlib.util.spec_from_file_location("phase115_contract_test", CONTRACT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase115DirectSeedContractTests(unittest.TestCase):
    def test_freeze_is_single_default_off_recipe_correction(self) -> None:
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(freeze["phase"], 115)
        self.assertEqual(freeze["decision"]["candidate_count"], 1)
        self.assertTrue(freeze["candidate"]["opt_in"])
        self.assertTrue(freeze["candidate"]["default_off"])
        self.assertEqual(freeze["candidate"]["base_selectors_exact"], [
            "--native-base-pseudorange-compensation",
            "--native-base-pseudorange-source-miss-mask",
        ])
        self.assertEqual(freeze["candidate"]["removed_selector"], "--native-base-pseudorange-preserve-additional-frequency-bands")
        self.assertFalse(freeze["decision"]["raw_execution_authorized"])

    def test_exact_command_omits_only_additional_band_selector(self) -> None:
        contract = load_contract()
        for route in contract.ROUTES:
            command = contract.command_template(route)
            self.assertEqual(command.count(contract.DIRECT_SELECTOR), 1)
            self.assertEqual(command.count(contract.VECTOR_SELECTOR), 1)
            self.assertEqual(command.count(contract.QR_SELECTOR), 1)
            self.assertEqual(command.count(contract.OFFSET_SELECTOR), 1)
            self.assertEqual(command.count(contract.REMOVED_SELECTOR), 0)
            self.assertEqual(command.count("--native-base-pseudorange-compensation"), 1)
            self.assertEqual(command.count("--native-base-pseudorange-source-miss-mask"), 1)
            self.assertNotIn("gnss-first-meter-state-handoff", " ".join(command))
            joined = " ".join(command).lower()
            self.assertNotIn("truth", joined)
            self.assertNotIn(".mat", joined)
            self.assertNotIn("precomputed", joined)

    def test_source_and_binary_pins_are_phase114_unchanged(self) -> None:
        contract = load_contract()
        contract.verify_implementation()
        self.assertIn("native_direct_wls_ephemeral_c7d_main_seed", APP.read_text(encoding="utf-8"))

    def test_pre_raw_activity_is_zero(self) -> None:
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

    def test_structural_gates_preserve_direct_seed_and_phase107_base(self) -> None:
        contract = load_contract()
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(manifest["candidate"]["removed_selector"], contract.REMOVED_SELECTOR)
        self.assertEqual(manifest["base_contract"]["preserve_additional_frequency_bands"], False)
        for gate in (
            "direct_seed_full_finite_exact_position_c7_d_velocity_keys",
            "main_qr_accepted_iteration_strict_cost_decrease",
            "main_meter_c0d_units_sigma_factor",
            "phase107_base_correction_active_exactly_once",
            "final_pixel5_offset_exactly_once",
        ):
            self.assertTrue(manifest["structural_gates"][gate])


if __name__ == "__main__":
    unittest.main()
