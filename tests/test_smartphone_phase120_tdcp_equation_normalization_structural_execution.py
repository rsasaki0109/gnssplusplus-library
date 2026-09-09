"""Launch-free Phase120 structural contract tests.

These tests inspect only tracked source and sealed metadata.  They deliberately
do not stat or open raw phone/base members, truth, MAT, coordinate, or solution
payloads and never launch the native solver.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
VALIDATOR_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase120_tdcp_equation_normalization.py"
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase120_tdcp_equation_normalization_structural_execute.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase120_tdcp_equation_normalization_structural_manifest_v1.json"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase120_tdcp_equation_normalization_structural_contract_freeze_v1.json"


def load_validator():
    spec = importlib.util.spec_from_file_location("phase120_structural_contract_test", VALIDATOR_PATH)
    if spec is None or spec.loader is None:
        raise AssertionError(f"unable to load validator: {VALIDATOR_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase120StructuralExecutionContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.validator = load_validator()
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        cls.manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))

    def test_freeze_and_manifest_are_closed_before_authorization(self) -> None:
        self.assertEqual(self.freeze["phase"], 120)
        self.assertEqual(
            self.freeze["status"], "sealed-before-phase120-raw-execution"
        )
        self.assertFalse(self.freeze["execution_policy"]["raw_execution_authorized"])
        self.assertFalse(self.freeze["execution_policy"]["truth_evaluation_authorized"])
        self.assertEqual(self.freeze["candidate"]["candidate_count"], 1)
        self.assertTrue(self.freeze["candidate"]["default_off"])
        self.assertEqual(self.manifest["phase"], 120)
        self.assertEqual(
            self.manifest["freeze"]["commit"], self.validator.FREEZE_COMMIT
        )
        self.assertEqual(
            self.manifest["implementation"]["commit"],
            self.validator.IMPLEMENTATION_COMMIT,
        )

    def test_exact_route_commands_use_phase120_composed_recipe(self) -> None:
        routes = self.manifest["routes"]
        self.assertEqual(
            [item["dataset_id"] for item in routes], list(self.validator.ROUTES)
        )
        for item in routes:
            route = item["dataset_id"]
            command = item["command"]
            self.assertEqual(command, self.validator.command_template(route))
            self.assertEqual(command.count(self.validator.SELECTOR), 1)
            self.assertEqual(command.count(self.validator.PHASE118_SELECTOR), 1)
            self.assertEqual(command.count(self.validator.PHASE117_SELECTOR), 0)
            for flag in self.validator.BASE_SELECTORS:
                self.assertEqual(command.count(flag), 1)
            rendered = " ".join(command).lower()
            for forbidden in ("truth", ".mat", "precomputed", "coordinate", "kaggle"):
                self.assertNotIn(forbidden, rendered)

    def test_measurement_gate_and_tdcp_population_contract(self) -> None:
        candidate = self.manifest["candidate"]
        self.assertEqual(
            candidate["source_measurement_m"],
            "carrier_phase_cycles * retained_wavelength_m + satellite_clock_m",
        )
        self.assertEqual(
            candidate["legacy_measurement_m"],
            "carrier_phase_cycles * wavelength_m + satellite_clock_m - troposphere_delay_m + ionosphere_delay_m",
        )
        self.assertEqual(
            candidate["pair_gate_input"], "historical corrected_carrier_m delta"
        )
        self.assertEqual(candidate["fixed_tdcp_sigma_m"], 0.03)
        self.assertEqual(candidate["official_huber_k"], 0.5)
        self.assertFalse(candidate["phase117_dynamic_sigma"])
        self.assertTrue(candidate["factor_count_and_reject_invariance"])
        for route in self.manifest["routes"]:
            self.assertEqual(route["official_setting_type"], "Highway")
            self.assertEqual(route["expected_tdcp_huber_k"], 0.5)
            baseline = self.manifest["baseline_tdcp_counts"][route["dataset_id"]]
            self.assertEqual(baseline["factors_built"], baseline["factors_inserted"])
            self.assertEqual(baseline["factors_built"], baseline["finite_residuals"])

    def test_validator_and_runner_are_launch_free(self) -> None:
        validator_source = VALIDATOR_PATH.read_text(encoding="utf-8")
        runner_source = RUNNER_PATH.read_text(encoding="utf-8")
        self.assertNotIn("subprocess", validator_source)
        self.assertNotIn("subprocess", runner_source)
        self.assertIn("no raw or solver execution is available", runner_source)
        self.assertIn("payload/solution hash forbidden before authorization", validator_source)

    def test_validator_pre_raw_verification_reports_all_zero_payload_activity(self) -> None:
        result = self.validator.verify_pre_raw()
        for key in (
            "raw_phone_gnss_reads",
            "raw_phone_imu_reads",
            "broadcast_navigation_reads",
            "raw_base_rinex_reads",
            "raw_base_hash_reads",
            "native_solver_invocations",
            "solution_rows_opened",
            "solution_coordinate_interpretations",
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

    def test_sealed_phase118_provenance_and_route_shape_are_pinned(self) -> None:
        provenance = self.manifest["sealed_phase118_provenance"]
        self.assertEqual(
            provenance["structural_manifest"]["sha256"],
            self.validator.PHASE118_MANIFEST_SHA256,
        )
        self.assertEqual(
            provenance["structural_result"]["sha256"],
            self.validator.PHASE118_RESULT_SHA256,
        )
        self.assertEqual(
            [item["dataset_id"] for item in self.manifest["routes"]],
            list(self.validator.ROUTES),
        )
        for route in self.validator.ROUTES:
            item = next(record for record in self.manifest["routes"] if record["dataset_id"] == route)
            self.assertEqual(item["domain_rows"], self.validator.DOMAIN_ROWS[route])
            self.assertEqual(item["expected_problem_epochs"], self.validator.PROBLEM_EPOCHS[route])


if __name__ == "__main__":
    unittest.main()
