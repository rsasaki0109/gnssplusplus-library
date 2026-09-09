"""Launch-free checks for the Phase98 compact diagnostic contract."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
EVALUATOR_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase98_solver_rank_diagnostic.py"
WRAPPER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase98_solver_rank_diagnostic_execute.py"


def load_evaluator():
    spec = importlib.util.spec_from_file_location("phase98_contract_test_module", EVALUATOR_PATH)
    if spec is None or spec.loader is None:
        raise AssertionError("unable to import Phase98 evaluator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_wrapper():
    spec = importlib.util.spec_from_file_location("phase98_wrapper_test_module", WRAPPER_PATH)
    if spec is None or spec.loader is None:
        raise AssertionError("unable to import Phase98 wrapper")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase98SolverRankDiagnosticContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.evaluator = load_evaluator()
        cls.wrapper = load_wrapper()

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
        self.assertEqual(pre_raw["raw_execution_authorized"], self.evaluator.AUTHORIZATION.is_file())

    def test_phase95_paths_are_pinned_without_hashing_raw_bytes(self):
        paths = self.evaluator.phase95_paths()
        self.assertEqual(list(paths), list(self.evaluator.ROUTES))
        self.assertIn("phase25-raw-clock-eval-v1", paths[self.evaluator.ROUTES[0]]["device_gnss.csv"])
        self.assertIn("phase37-pixel5-repeatability-v1", paths[self.evaluator.ROUTES[1]]["device_gnss.csv"])
        source = EVALUATOR_PATH.read_text(encoding="utf-8")
        self.assertIn("raw-file hash is forbidden", source)

    def test_commands_have_exact_raw_roles_and_phase96_phase98_only(self):
        manifest = self.evaluator.verify_manifest()
        for record in manifest["routes"]:
            command = record["command"]
            self.assertEqual(command.count("--android-gnss"), 1)
            self.assertEqual(command.count("--android-imu"), 1)
            self.assertEqual(command.count("--nav"), 1)
            self.assertEqual(command.count(self.evaluator.PHASE96_SELECTOR), 1)
            self.assertEqual(command.count(self.evaluator.SELECTOR), 1)
            self.assertEqual(command.count(self.evaluator.PHASE93_SELECTOR), 1)
            self.assertNotIn("--native-source-clock-c0d-phase97-singular-system-diagnostics", command)
            self.assertNotIn("--native-source-clock-c0d-phase94-stage-diagnostics", command)
            self.assertNotIn("--native-pdc-state-bridge", command)

    def test_phase98_implementation_is_opt_in_and_compact(self):
        self.evaluator.verify_implementation()
        source = WRAPPER_PATH.read_text(encoding="utf-8")
        self.assertIn("subprocess.run", source)
        self.assertIn("runner_read_raw_bytes", source)
        self.assertIn("raw_content_copied_or_transformed", source)
        self.assertIn("exception_text_digest", source)
        self.assertIn("nearby_variable_unavailable", source)
        self.assertIn("reused_without_copy", source)
        self.assertNotIn("rank_decompositions", source)
        self.assertNotIn("phase97_main", source)

    def test_compact_trial_preserves_exact_key_or_explicit_unavailable(self):
        solver = {
            "solver_type": "MULTIFRONTAL_CHOLESKY",
            "solver_branch": "multifrontal",
            "elimination_function": "EliminatePreferCholesky",
            "ordering_type": "COLAMD",
            "ordering_size": 3,
            "ordering_digest": "fnv1a64:0000000000000000",
            "diagonal_damping": True,
        }
        phase96 = {"trial_trace_complete": True, "lm_trials": [{"trial_index": i, "lambda": 10.0 ** (i - 5), "rejection_reason": "indeterminate_linear_system"} for i in range(10)], "exceptions": [{"message": "pinned LM trial did not solve the damped linear system", "count": 10, "stage": "lm_trial", "classification": "indeterminate_linear_system", "type": "IndeterminantLinearSystemException"}]}
        report = self.wrapper._compact_phase96(phase96, solver)
        self.assertEqual(len(report["lm_trials"]), 10)
        self.assertTrue(all(item["nearby_variable"] is None for item in report["lm_trials"]))
        self.assertTrue(all(item["nearby_variable_status"] == "nearby_variable_unavailable" for item in report["lm_trials"]))
        self.assertEqual(len(report["exception_text_digests"]), 1)

    def test_typed_exception_keeps_symbol_index_and_digest_only(self):
        report = self.wrapper._compact_phase98({"enabled": True, "attempted": True, "exception_captured": True, "solver_type": "MULTIFRONTAL_CHOLESKY", "solver_branch": "multifrontal", "elimination_function": "EliminatePreferCholesky", "ordering_type": "COLAMD", "ordering_size": 1, "ordering_digest": "fnv1a64:abcd", "diagonal_damping": True, "indeterminate_exceptions": [{"stage": "lm_optimize", "lambda": 1.0e-5, "solver_type": "MULTIFRONTAL_CHOLESKY", "solver_branch": "multifrontal", "elimination_function": "EliminatePreferCholesky", "ordering_type": "COLAMD", "ordering_size": 1, "ordering_digest": "fnv1a64:abcd", "diagonal_damping": True, "nearby_variable_available": True, "nearby_variable": {"numeric_key": 778, "symbol_character": "n", "symbol_index": 10}, "nearby_variable_status": "captured_from_exception", "exception_type": "gtsam::IndeterminantLinearSystemException", "exception_message": "nearbyVariable n10"}]} )
        item = report["indeterminate_exceptions"][0]
        self.assertEqual(item["nearby_variable"], {"numeric_key": 778, "symbol_character": "n", "symbol_index": 10})
        self.assertTrue(item["exception_text_digest"].startswith("sha256:"))
        self.assertNotIn("exception_message", item)

    def test_publication_policy_references_phase97_without_copy(self):
        reference = self.wrapper._phase97_reference()
        self.assertTrue(reference["reused_without_copy"])
        self.assertFalse(reference["nearby_key_present"])
        self.assertIsNone(reference["nearby_key_family"])
        manifest = self.evaluator.verify_manifest()
        self.assertTrue(manifest["candidate"]["compact_sidecar_only"])
        self.assertFalse(manifest["candidate"]["phase97_incidence_reemission"])


if __name__ == "__main__":
    unittest.main()
