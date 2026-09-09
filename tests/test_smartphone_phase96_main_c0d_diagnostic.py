"""Launch-free checks for the Phase96 two-route diagnostic contract."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
EVALUATOR_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase96_main_c0d_diagnostic.py"
)
WRAPPER_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase96_main_c0d_diagnostic_execute.py"
)


def load_evaluator():
    spec = importlib.util.spec_from_file_location("phase96_contract_test_module", EVALUATOR_PATH)
    if spec is None or spec.loader is None:
        raise AssertionError("unable to import Phase96 evaluator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase96MainC0DDiagnosticContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.evaluator = load_evaluator()

    def test_freeze_and_manifest_are_launch_free_and_exactly_two_routes(self):
        manifest = self.evaluator.verify_manifest()
        self.assertEqual(
            [item["dataset_id"] for item in manifest["routes"]],
            [
                "2021-03-16-18-59-us-ca-mtv-a/pixel5",
                "2022-04-01-18-22-us-ca-lax-t/pixel5",
            ],
        )
        self.assertEqual(manifest["matrix"]["native_invocations_planned"], 2)
        self.assertFalse(manifest["execution_authorization"]["native_route_rerun_performed"])

    def test_pre_raw_accounting_has_no_native_or_raw_reads(self):
        pre_raw = self.evaluator.verify_pre_raw()
        self.assertEqual(pre_raw["raw_reads"], 0)
        self.assertEqual(pre_raw["native_solver_invocations"], 0)
        self.assertFalse(pre_raw["accuracy_scored"])

    def test_each_command_has_exact_raw_roles_and_phase96_selector(self):
        manifest = self.evaluator.verify_manifest()
        for record in manifest["routes"]:
            command = record["command"]
            self.assertEqual(command.count("--android-gnss"), 1)
            self.assertEqual(command.count("--android-imu"), 1)
            self.assertEqual(command.count("--nav"), 1)
            self.assertEqual(command.count(self.evaluator.SELECTOR), 1)
            self.assertEqual(command.count(self.evaluator.PHASE93_SELECTOR), 1)
            self.assertNotIn("--native-pdc-state-bridge", command)
            self.assertNotIn("--native-source-clock-c0d-gnss-first-raw-drift-d-initializer", command)

    def test_archive_record_is_non_destructive(self):
        manifest = self.evaluator.verify_manifest()
        archive = manifest["workspace_archive_record"]
        self.assertFalse(archive["user_data_deleted"])
        self.assertEqual(len(archive["original_paths"]), 2)
        self.assertEqual(len(archive["current_paths"]), 2)

    def test_phase96_implementation_is_opt_in_and_source_aligned(self):
        self.evaluator.verify_implementation()
        source = WRAPPER_PATH.read_text(encoding="utf-8")
        self.assertIn("read_by_runner", source)
        self.assertIn("copied_or_transformed", source)
        self.assertIn("MAX_TRIALS", source)
        self.assertNotIn("sha256_file(raw", source)

    def test_phase96_result_projection_excludes_solution_fields(self):
        source = WRAPPER_PATH.read_text(encoding="utf-8")
        self.assertIn("factor_families", source)
        self.assertIn("variable_family_norms", source)
        self.assertIn("lm_trials", source)
        self.assertIn("observed_solution_rows", source)
        self.assertIn("solution_output_published", source)


if __name__ == "__main__":
    unittest.main()
