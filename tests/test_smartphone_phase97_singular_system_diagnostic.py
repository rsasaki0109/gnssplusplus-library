"""Launch-free checks for the Phase97 two-route diagnostic contract."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
EVALUATOR_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase97_singular_system_diagnostic.py"
WRAPPER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase97_singular_system_diagnostic_execute.py"


def load_evaluator():
    spec = importlib.util.spec_from_file_location("phase97_contract_test_module", EVALUATOR_PATH)
    if spec is None or spec.loader is None:
        raise AssertionError("unable to import Phase97 evaluator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase97SingularSystemDiagnosticContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.evaluator = load_evaluator()

    def test_freeze_and_manifest_are_launch_free_and_exactly_two_routes(self):
        manifest = self.evaluator.verify_manifest()
        self.assertEqual([item["dataset_id"] for item in manifest["routes"]], list(self.evaluator.ROUTES))
        self.assertEqual(manifest["matrix"]["native_invocations_planned"], 2)
        self.assertFalse(manifest["execution_authorization"]["raw_execution_authorized"])
        self.assertFalse(manifest["execution_authorization"]["native_route_rerun_performed"])

    def test_pre_raw_accounting_has_no_native_or_raw_reads(self):
        pre_raw = self.evaluator.verify_pre_raw()
        self.assertEqual(pre_raw["raw_reads"], 0)
        self.assertEqual(pre_raw["native_solver_invocations"], 0)
        self.assertFalse(pre_raw["accuracy_scored"])
        self.assertFalse(pre_raw["raw_execution_authorized"])

    def test_phase95_paths_are_pinned_without_hashing_raw_bytes(self):
        paths = self.evaluator.phase95_paths()
        self.assertEqual(list(paths), list(self.evaluator.ROUTES))
        self.assertIn("phase25-raw-clock-eval-v1", paths[self.evaluator.ROUTES[0]]["device_gnss.csv"])
        self.assertIn("phase37-pixel5-repeatability-v1", paths[self.evaluator.ROUTES[1]]["device_gnss.csv"])
        source = EVALUATOR_PATH.read_text(encoding="utf-8")
        self.assertIn("raw-file hash is forbidden", source)

    def test_commands_have_exact_raw_roles_and_only_phase97_selector(self):
        manifest = self.evaluator.verify_manifest()
        for record in manifest["routes"]:
            command = record["command"]
            self.assertEqual(command.count("--android-gnss"), 1)
            self.assertEqual(command.count("--android-imu"), 1)
            self.assertEqual(command.count("--nav"), 1)
            self.assertEqual(command.count(self.evaluator.SELECTOR), 1)
            self.assertEqual(command.count(self.evaluator.PHASE93_SELECTOR), 1)
            self.assertNotIn("--native-source-clock-c0d-phase94-stage-diagnostics", command)
            self.assertNotIn("--native-source-clock-c0d-phase96-main-diagnostics", command)
            self.assertNotIn("--native-pdc-state-bridge", command)

    def test_phase97_implementation_is_opt_in_and_diagnostic_only(self):
        self.evaluator.verify_implementation()
        source = WRAPPER_PATH.read_text(encoding="utf-8")
        self.assertIn("subprocess.run", source)
        self.assertIn("runner_read_raw_bytes", source)
        self.assertIn("raw_content_copied_or_transformed", source)
        self.assertIn("rank_decompositions", source)
        self.assertIn("variable_family_summary", source)
        self.assertIn("withheld_solution_output", source)
        self.assertNotIn("sha256_file(raw", source)

    def test_structural_gate_scope_excludes_solution_truth_and_accuracy(self):
        manifest = self.evaluator.verify_manifest()
        candidate = manifest["candidate"]
        self.assertTrue(candidate["diagnostic_only"])
        self.assertTrue(candidate["raw_only"])
        self.assertFalse(candidate["accuracy_scoring"])
        self.assertFalse(candidate["truth_evaluation"])
        self.assertFalse(candidate["submission_release"])
        self.assertTrue(candidate["no_solution_output_publication"])
        self.assertTrue(manifest["structural_gates"]["rank_or_nullspace_attribution"])
        self.assertTrue(manifest["structural_gates"]["factor_incidence_and_components"])


if __name__ == "__main__":
    unittest.main()
