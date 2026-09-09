"""Launch-free checks for the Phase99 main-only QR execution contract."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
EVALUATOR_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase99_main_multifrontal_qr.py"
WRAPPER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase99_main_multifrontal_qr_execute.py"


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise AssertionError(f"unable to import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase99MainMultifrontalQrContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.evaluator = load_module(EVALUATOR_PATH, "phase99_contract_test_module")
        cls.wrapper = load_module(WRAPPER_PATH, "phase99_wrapper_test_module")

    def test_freeze_and_manifest_pin_exact_two_routes_without_launch(self):
        freeze = self.evaluator.verify_freeze()
        manifest = self.evaluator.verify_manifest()
        self.assertEqual(freeze["phase"], 99)
        self.assertEqual([item["dataset_id"] for item in manifest["routes"]], list(self.evaluator.ROUTES))
        self.assertEqual(manifest["matrix"]["native_invocations_planned"], 2)
        self.assertFalse(manifest["execution_authorization"]["raw_execution_authorized"])

    def test_pre_raw_accounting_is_zero_and_accuracy_is_disabled(self):
        pre_raw = self.evaluator.verify_pre_raw()
        self.assertEqual(pre_raw["raw_reads"], 0)
        self.assertEqual(pre_raw["native_solver_invocations"], 0)
        self.assertFalse(pre_raw["accuracy_scored"])
        self.assertFalse(pre_raw["cholesky_comparison_rerun"])

    def test_phase95_path_map_is_metadata_only(self):
        paths = self.evaluator.phase95_paths()
        self.assertEqual(list(paths), list(self.evaluator.ROUTES))
        self.assertIn("phase25-raw-clock-eval-v1", paths[self.evaluator.ROUTES[0]]["device_gnss.csv"])
        self.assertIn("phase37-pixel5-repeatability-v1", paths[self.evaluator.ROUTES[1]]["device_gnss.csv"])
        source = EVALUATOR_PATH.read_text(encoding="utf-8")
        self.assertIn("raw-file hash is forbidden", source)

    def test_commands_have_exact_raw_roles_and_qr_selector(self):
        manifest = self.evaluator.verify_manifest()
        for record in manifest["routes"]:
            command = record["command"]
            self.assertEqual(command.count("--android-gnss"), 1)
            self.assertEqual(command.count("--android-imu"), 1)
            self.assertEqual(command.count("--nav"), 1)
            self.assertEqual(command.count(self.evaluator.PHASE93_SELECTOR), 1)
            self.assertEqual(command.count(self.evaluator.PHASE96_SELECTOR), 1)
            self.assertEqual(command.count(self.evaluator.PHASE98_SELECTOR), 1)
            self.assertEqual(command.count(self.evaluator.SELECTOR), 1)
            self.assertNotIn("--native-source-clock-c0d-phase94-stage-diagnostics", command)
            self.assertNotIn("--native-source-clock-c0d-phase97-singular-system-diagnostics", command)
            self.assertNotIn("--native-pdc-state-bridge", command)

    def test_solver_candidate_is_main_only_and_default_off(self):
        self.evaluator.verify_implementation()
        manifest = self.evaluator.verify_manifest()
        candidate = manifest["candidate"]
        self.assertTrue(candidate["opt_in"])
        self.assertTrue(candidate["default_off_outside_this_command"])
        self.assertEqual(candidate["solver_branch"]["selected_main_solver_type"], "MULTIFRONTAL_QR")
        self.assertEqual(candidate["solver_branch"]["gnss_first_solver_type"], "MULTIFRONTAL_CHOLESKY")
        self.assertTrue(candidate["no_solver_filter_or_lm_change"])

    def test_wrapper_preserves_withheld_solution_and_no_copy_policy(self):
        source = WRAPPER_PATH.read_text(encoding="utf-8")
        self.assertIn("subprocess.run", source)
        self.assertIn("withheld_solution_output", source)
        self.assertIn("runner_read_raw_bytes", source)
        self.assertIn("raw_content_copied_or_transformed", source)
        self.assertIn("solution_output_published", source)
        self.assertNotIn("read_bytes", source)

    def test_structural_gate_names_cover_progress_handoff_qr_and_coverage(self):
        gates = self.evaluator.verify_manifest()["structural_gates"]
        for key in (
            "gnss_first_strict_progress",
            "gnss_first_full_finite_d_exact_handoff",
            "main_selected_multifrontal_qr",
            "main_accepted_outer_iterations",
            "main_strict_cost_decrease",
            "main_finite_expected_coverage",
            "no_fallback_or_solution_publication",
        ):
            self.assertTrue(gates[key], key)


if __name__ == "__main__":
    unittest.main()
