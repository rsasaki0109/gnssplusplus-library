"""Launch-free tests for the Phase103 truth-only evaluator correction."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase103_phase102_truth_only_evaluator_correction.py"


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise AssertionError(f"unable to import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase103TruthOnlyCorrectionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract = load_module(CONTRACT_PATH, "phase103_truth_only_correction_contract_test")

    def test_freeze_pins_phase102_failed_result_without_native_rerun(self):
        freeze = self.contract.verify_freeze()
        self.assertEqual(freeze["phase"], 103)
        self.assertEqual(freeze["correction"]["id"], self.contract.CORRECTION_ID)
        self.assertTrue(freeze["correction"]["native_solution_reuse"])
        self.assertEqual(freeze["correction"]["native_solver_invocations"], 0)
        self.assertFalse(freeze["correction"]["native_rerun"])
        self.assertEqual(freeze["metric_and_gates"]["strict_macro_gate_m"], 0.782)

    def test_optional_mat_used_absence_is_accepted_without_synthesis(self):
        route = self.contract.ROUTES[0]
        summary = {
            "dataset_id": route,
            "truth_used": False,
            "production_default_changed": False,
            "base_factors": False,
            "native_pdc_state_bridge": False,
            "native_source_clock_c0d_factor_enabled": True,
            "native_source_clock_c0d_meter_state_parity_enabled": True,
            "native_source_clock_c0d_gnss_first_meter_state_handoff_enabled": True,
            "native_source_clock_c0d_epoch_vector_parity_enabled": True,
            "native_source_clock_c0d_phase99_main_multifrontal_qr_solver_enabled": True,
            "native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected": True,
            "selected_linear_solver_type": "MULTIFRONTAL_QR",
            "selected_solver_branch": "multifrontal",
            "selected_elimination_function": "EliminateQR",
        }
        import json

        accepted, _ = self.contract._validate_corrected_summary(json.dumps(summary).encode(), route)
        self.assertNotIn("mat_used", accepted)

    def test_present_mat_used_true_is_rejected(self):
        route = self.contract.ROUTES[0]
        summary = {
            "dataset_id": route,
            "truth_used": False,
            "production_default_changed": False,
            "base_factors": False,
            "native_pdc_state_bridge": False,
            "native_source_clock_c0d_factor_enabled": True,
            "native_source_clock_c0d_meter_state_parity_enabled": True,
            "native_source_clock_c0d_gnss_first_meter_state_handoff_enabled": True,
            "native_source_clock_c0d_epoch_vector_parity_enabled": True,
            "native_source_clock_c0d_phase99_main_multifrontal_qr_solver_enabled": True,
            "native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected": True,
            "selected_linear_solver_type": "MULTIFRONTAL_QR",
            "selected_solver_branch": "multifrontal",
            "selected_elimination_function": "EliminateQR",
            "mat_used": True,
        }
        import json

        with self.assertRaises(Exception):
            self.contract._validate_corrected_summary(json.dumps(summary).encode(), route)

    def test_pre_truth_is_zero_and_native_process_is_not_available(self):
        pre = self.contract.verify_pre_truth()
        self.assertEqual(pre["native_solver_invocations"], 0)
        self.assertEqual(pre["raw_reads"], 0)
        self.assertEqual(pre["truth_reads"], 0)
        self.assertFalse(pre["native_rerun"])
        source = CONTRACT_PATH.read_text(encoding="utf-8")
        self.assertNotIn("subprocess.run", source)

    def test_lineage_rejects_forbidden_selector_and_nonzero_wrapper_accounting(self):
        base = self.contract._load_phase102()
        route = self.contract.ROUTES[0]
        command = ["build/apps/gnss_fgo_imu_no_base", "--dataset-id", route, "--obs", "bad"]
        run = {"command": command}
        with self.assertRaises(Exception):
            self.contract._validate_sealed_lineage(run, {}, route, base)


if __name__ == "__main__":
    unittest.main()
