"""Launch-free Phase118 structural contract tests.

These tests inspect only the pinned source and sealed metadata.  They never
stat or open raw phone/base members, truth, MAT, coordinates, or solution
rows, and never launch the native solver.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
VALIDATOR_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase118_tdcp_robust_k.py"
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase118_tdcp_robust_k_structural_execute.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_structural_manifest_v1.json"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_structural_contract_freeze_v1.json"


def load_validator():
    spec = importlib.util.spec_from_file_location("phase118_contract_test", VALIDATOR_PATH)
    if spec is None or spec.loader is None:
        raise AssertionError(f"unable to load validator: {VALIDATOR_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase118StructuralExecutionContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.validator = load_validator()
        cls.manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))

    def test_freeze_and_manifest_are_closed_before_authorization(self) -> None:
        self.assertEqual(self.freeze["phase"], 118)
        self.assertIs(self.freeze["execution_policy"]["raw_execution_authorized"], False)
        self.assertTrue(self.freeze["candidate"]["default_off"])
        self.assertEqual(self.manifest["phase"], 118)
        self.assertEqual(self.manifest["status"], "sealed-before-phase118-raw-execution")
        self.assertEqual(self.manifest["freeze"]["commit"], self.validator.FREEZE_COMMIT)
        self.assertEqual(self.manifest["implementation"]["commit"], self.validator.IMPLEMENTATION_COMMIT)

    def test_exact_two_route_commands_use_only_phase118_selector(self) -> None:
        routes = self.manifest["routes"]
        self.assertEqual([item["dataset_id"] for item in routes], list(self.validator.ROUTES))
        for item in routes:
            route = item["dataset_id"]
            self.assertEqual(item["command"], self.validator.command_template(route))
            command = item["command"]
            self.assertEqual(command.count(self.validator.SELECTOR), 1)
            self.assertEqual(command.count(self.validator.PHASE117_SELECTOR), 0)
            for flag in self.validator.BASE_SELECTORS:
                self.assertEqual(command.count(flag), 1)
            rendered = " ".join(command).lower()
            self.assertNotIn("truth", rendered)
            self.assertNotIn(".mat", rendered)
            self.assertNotIn("precomputed", rendered)
            self.assertNotIn("coordinate", rendered)
            self.assertNotIn("kaggle", rendered)

    def test_type_k_sigma_and_count_contract(self) -> None:
        candidate = self.manifest["candidate"]
        self.assertEqual(candidate["selector"], self.validator.SELECTOR)
        self.assertEqual(candidate["fixed_tdcp_sigma_m"], 0.03)
        self.assertFalse(candidate["phase117_dynamic_sigma"])
        self.assertEqual(candidate["official_highway_k"], 0.5)
        self.assertEqual(candidate["official_street_k"], 0.2)
        self.assertEqual(candidate["official_mix_k"], 0.2)
        self.assertTrue(candidate["factor_count_invariance"])
        for route in self.manifest["routes"]:
            self.assertEqual(route["official_setting_type"], "Highway")
            self.assertEqual(route["expected_tdcp_huber_k"], 0.5)
        for route, expected in self.manifest["baseline_tdcp_counts"].items():
            if route == "source_record":
                continue
            self.assertGreater(expected["factors_built"], 0)
            self.assertEqual(expected["factors_built"], expected["factors_inserted"])
            self.assertEqual(expected["factors_built"], expected["finite_residuals"])

    def test_validator_and_runner_are_launch_free(self) -> None:
        validator_source = VALIDATOR_PATH.read_text(encoding="utf-8")
        runner_source = RUNNER_PATH.read_text(encoding="utf-8")
        self.assertNotIn("subprocess", validator_source)
        self.assertNotIn("subprocess", runner_source)
        self.assertIn("no raw or solver execution is available", runner_source)
        self.assertIn("must not", validator_source)

    def test_pre_raw_read_accounting_is_zero(self) -> None:
        result = self.validator.verify_pre_raw()
        for key in (
            "raw_phone_gnss_reads",
            "raw_phone_imu_reads",
            "broadcast_navigation_reads",
            "raw_base_rinex_reads",
            "raw_base_hash_reads",
            "native_solver_invocations",
            "solution_rows_opened",
            "truth_reads",
            "mat_reads_or_generated",
            "phone_coordinate_reads",
            "precomputed_coordinate_reads",
            "pdc_reads",
            "accuracy_calculations",
            "kaggle_or_token_access",
            "route_reruns",
            "fallbacks",
        ):
            self.assertEqual(result[key], 0, key)
        self.assertFalse(result["raw_execution_authorized"])
        self.assertFalse(result["solution_output_published"])

    def test_all_structural_gates_are_explicitly_true(self) -> None:
        gates = self.manifest["structural_gates"]
        self.assertGreaterEqual(len(gates), 20)
        for key, value in gates.items():
            self.assertIs(value, True, key)


if __name__ == "__main__":
    unittest.main()
