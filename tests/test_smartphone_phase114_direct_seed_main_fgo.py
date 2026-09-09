"""Launch-free Phase114 direct-seed contract tests.

The tests inspect only source/contract/sealed metadata.  They do not stat or
open raw phone/navigation members, base RINEX, truth, MAT, candidate
coordinates, or Kaggle resources, and they never launch the native solver.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase114_direct_seed_main_fgo.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase114_direct_seed_main_fgo_manifest_v1.json"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase114_direct_seed_main_fgo_freeze_v1.json"
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"


def load_contract():
    spec = importlib.util.spec_from_file_location("phase114_contract_test", CONTRACT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase114DirectSeedContractTests(unittest.TestCase):
    def test_freeze_and_manifest_are_default_off_before_authorization(self) -> None:
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(freeze["phase"], 114)
        self.assertTrue(freeze["candidate"]["default_off"])
        self.assertTrue(freeze["candidate"]["bypass_gnss_first_for_candidate_only"])
        self.assertFalse(freeze["decision"]["raw_execution_authorized"])
        self.assertFalse(manifest["freeze"]["raw_execution_authorized_before_manifest"])
        self.assertTrue(manifest["candidate"]["same_run_ephemeral_seed"])
        self.assertFalse(manifest["matrix"]["solution_rows_authorized"])

    def test_exact_direct_command_has_no_gnss_first_or_forbidden_lane(self) -> None:
        contract = load_contract()
        for route in contract.ROUTES:
            command = contract.command_template(route)
            self.assertEqual(command.count(contract.DIRECT_SELECTOR), 1)
            self.assertEqual(command.count(contract.VECTOR_SELECTOR), 1)
            self.assertEqual(command.count(contract.QR_SELECTOR), 1)
            self.assertEqual(command.count(contract.OFFSET_SELECTOR), 1)
            self.assertNotIn("--native-source-clock-c0d-gnss-first-meter-state-handoff", command)
            self.assertNotIn("--native-direct-doppler-wls-handoff", command)
            joined = " ".join(command).lower()
            self.assertNotIn("truth", joined)
            self.assertNotIn(".mat", joined)
            self.assertNotIn("precomputed", joined)
            self.assertIn("__PHASE114_RAW_DEVICE_GNSS__", command)
            self.assertIn("__PHASE114_RAW_DEVICE_IMU__", command)
            self.assertIn("__PHASE114_RAW_BROADCAST_NAV__", command)
            self.assertIn("__PHASE114_RAW_BASE_RINEX__", command)

    def test_direct_adapter_and_result_export_markers_are_pinned(self) -> None:
        contract = load_contract()
        contract.verify_implementation()
        source = APP.read_text(encoding="utf-8")
        self.assertIn("validateDirectWlsEphemeralMainSeed", source)
        self.assertIn("main-direct-wls-ephemeral-c7d-seed", source)
        self.assertIn("epoch_clock_drift_mps", source)
        self.assertIn("gnss_first_stage_run", source)

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

    def test_structural_gate_contract_separates_direct_seed_from_stage(self) -> None:
        contract = load_contract()
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(manifest["candidate"]["selector"], contract.DIRECT_SELECTOR)
        self.assertEqual(manifest["candidate"]["gnss_first_stage"], "bypassed by design")
        for gate in (
            "direct_seed_full_finite_exact_key_handoff",
            "main_qr_progress_strict_cost_decrease",
            "main_meter_c0d_units_sigma_factor_gate",
            "base_factors_active_exactly_once",
            "final_pixel5_offset_exactly_once",
            "finite_earth_valid_expected_output_coverage",
        ):
            self.assertTrue(manifest["structural_gates"][gate])


if __name__ == "__main__":
    unittest.main()
