"""Launch-free tests for the Phase130 keyed base-support contract."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))
import gnss_smartphone_phase130_shared_ledger_structural as contract  # noqa: E402


FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase130_shared_ledger_join_semantics_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase130_shared_ledger_structural_manifest_v1.json"
VALIDATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase130_shared_ledger_structural.py"
RUNNER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase130_shared_ledger_structural_execute.py"


def passing_inventory(route: str) -> dict:
    """A deliberately unequal rover/base synthetic inventory."""
    rover = {
        "input_rows": 8,
        "certified_rows": 5,
        "explicit_local_miss_rows": 3,
        "header_primary_certified_rows": 1,
        "broadcast_geph_certified_rows": 4,
        "reason_counts": {"missing-fcn": 1, "query-time-gap": 2},
        "all_rows_classified": True,
        "no_raw_uncorrected": True,
        "no_zero_correction": True,
        "no_fallback": True,
        "no_extrapolation": True,
        "finite_certified_wavelength": True,
    }
    base = {
        "input_rows": 13,
        "certified_rows": 9,
        "explicit_local_miss_rows": 4,
        "header_primary_certified_rows": 3,
        "broadcast_geph_certified_rows": 6,
        "reason_counts": {"query-time-gap": 4},
        "all_rows_classified": True,
        "no_raw_uncorrected": True,
        "no_zero_correction": True,
        "no_fallback": True,
        "no_extrapolation": True,
        "finite_certified_wavelength": True,
    }
    return {
        "route": route,
        "stage": "post-authorization-pre-solver",
        "solver_invocations": 0,
        "solver_may_start": True,
        "rover": rover,
        "base": base,
        "shared_ledger": {
            "side_local_conservation": True,
            "retained_rover_exact_key_support": True,
            "exact_endpoint_or_adjacent_two_point_bracket": True,
            "unmatched_rover_explicit_factor_miss": True,
            "unused_base_rows_accounted": True,
            "whole_ledger_equality_required": False,
            "cross_side_count_equality_required": False,
            "cross_side_reason_map_equality_required": False,
            "no_raw_uncorrected": True,
            "no_zero_correction": True,
            "no_fallback": True,
            "no_extrapolation": True,
        },
        "base_streams": {
            "finite_samples": 6,
            "used_rows": 4,
            "unused_rows": 2,
            "unused_streams": 1,
            "all_samples_accounted": True,
            "duplicate_nonmonotonic_rejected": True,
        },
        "correction": {
            "factor_input_rows": 7,
            "retained_corrected_rows": 4,
            "missing_exact_stream": 2,
            "out_of_domain": 1,
            "nonfinite": 0,
            "support": {
                "exact_endpoint_rows": 1,
                "adjacent_two_point_bracket_rows": 2,
                "exact_interior_sample_rows": 1,
            },
            "retained_factors_have_exact_key_support": True,
            "one_support_result_per_retained_factor": True,
            "unused_base_rows_accounted": True,
            "application_passes": 1,
            "exactly_once": True,
            "no_duplicate_application": True,
            "no_raw_fallback": True,
            "no_zero_fallback": True,
            "no_nearest_fill": True,
            "no_endpoint_hold": True,
            "no_extrapolation": True,
            "finite_corrected_rows": True,
        },
        "phase130": {
            "enabled": True,
            "side_local_conservation": True,
            "whole_ledger_equality_required": False,
            "retained_rover_exact_key_support": True,
            "unmatched_rover_explicit_factor_miss": True,
            "unused_base_rows_accounted": True,
            "no_raw_uncorrected": True,
            "no_zero_correction": True,
            "no_fallback": True,
            "no_extrapolation": True,
            "source_complete_a_b_c": True,
        },
    }


def passing_summary(route: str) -> dict:
    return {
        "route": route,
        "solution_content_read": False,
        "fallback_used": False,
        "rerun_count": 0,
        "gnss_first": {"accepted_iterations": 2, "initial_cost": 10.0, "final_cost": 7.0},
        "main": {"accepted_iterations": 3, "initial_cost": 9.0, "final_cost": 6.0,
                 "solver_branch": "MULTIFRONTAL_QR"},
        "phase130": {
            "enabled": True,
            "whole_ledger_equality_required": False,
            "side_local_conservation": True,
            "retained_rover_exact_key_support": True,
            "unused_base_rows_accounted": True,
            "no_fallback": True,
        },
        "gates": {"local_join": True, "factor_support": True},
        "opaque_solution": {"sha256": "a" * 64, "rows": 10},
    }


class Phase130SharedLedgerStructuralTests(unittest.TestCase):
    def test_mismatched_cadence_and_counts_are_admitted(self) -> None:
        report = contract.verify_inventory_record(
            contract.ROUTES[0], passing_inventory(contract.ROUTES[0]))
        self.assertTrue(report["inventory_passed"])
        self.assertEqual(report["rover_rows"], 8)
        self.assertEqual(report["base_rows"], 13)
        self.assertEqual(report["base_unused_rows"], 2)

    def test_exact_endpoint_bracket_and_out_of_domain(self) -> None:
        samples = [
            {"time": 10.0, "value": 100.0},
            {"time": 20.0, "value": 120.0},
            {"time": 30.0, "value": 140.0},
        ]
        endpoint = contract.classify_support(samples, 10.0)
        self.assertTrue(endpoint["supported"])
        self.assertEqual(endpoint["support_kind"], "exact_endpoint")
        bracket = contract.classify_support(samples, 15.0)
        self.assertTrue(bracket["supported"])
        self.assertEqual(bracket["support_kind"], "adjacent_two_point_bracket")
        self.assertEqual(bracket["value"], 110.0)
        outside = contract.classify_support(samples, 9.0)
        self.assertFalse(outside["supported"])
        self.assertEqual(outside["reason"], "out_of_domain")

    def test_missing_key_is_explicit_factor_miss(self) -> None:
        inventory = passing_inventory(contract.ROUTES[0])
        self.assertGreater(inventory["correction"]["missing_exact_stream"], 0)
        report = contract.verify_inventory_record(contract.ROUTES[0], inventory)
        self.assertTrue(report["inventory_passed"])
        inventory["shared_ledger"]["unmatched_rover_explicit_factor_miss"] = False
        with self.assertRaises(contract.Phase130ContractError):
            contract.verify_inventory_record(contract.ROUTES[0], inventory)

    def test_unused_base_rows_are_accounted_without_global_equality(self) -> None:
        inventory = passing_inventory(contract.ROUTES[1])
        self.assertNotEqual(inventory["rover"]["input_rows"], inventory["base"]["input_rows"])
        self.assertFalse(inventory["shared_ledger"]["whole_ledger_equality_required"])
        self.assertEqual(
            inventory["base_streams"]["finite_samples"],
            inventory["base_streams"]["used_rows"] + inventory["base_streams"]["unused_rows"],
        )
        self.assertTrue(contract.verify_inventory_record(contract.ROUTES[1], inventory)["inventory_passed"])

    def test_duplicate_or_nonmonotonic_stream_fails_closed(self) -> None:
        with self.assertRaises(contract.Phase130ContractError):
            contract.validate_stream_samples(
                [{"time": 10.0, "value": 1.0}, {"time": 10.0, "value": 2.0}])
        with self.assertRaises(contract.Phase130ContractError):
            contract.validate_stream_samples(
                [{"time": 20.0, "value": 1.0}, {"time": 10.0, "value": 2.0}])

    def test_global_source_failure_stays_abort(self) -> None:
        inventory = passing_inventory(contract.ROUTES[0])
        inventory["phase130"]["source_complete_a_b_c"] = False
        with self.assertRaises(contract.Phase130ContractError):
            contract.verify_inventory_record(contract.ROUTES[0], inventory)

    def test_whole_ledger_equality_cannot_be_reintroduced(self) -> None:
        inventory = passing_inventory(contract.ROUTES[0])
        inventory["shared_ledger"]["whole_ledger_equality_required"] = True
        with self.assertRaises(contract.Phase130ContractError):
            contract.verify_inventory_record(contract.ROUTES[0], inventory)

    def test_structural_summary_remains_opaque_and_progressing(self) -> None:
        report = contract.verify_structural_summary(
            contract.ROUTES[0], passing_summary(contract.ROUTES[0]))
        self.assertTrue(report["structural_gate_passed"])
        summary = passing_summary(contract.ROUTES[0])
        summary["main"]["final_cost"] = 9.0
        with self.assertRaises(contract.Phase130ContractError):
            contract.verify_structural_summary(contract.ROUTES[0], summary)

    def test_manifest_command_selector_shape(self) -> None:
        command = contract.command_template(contract.ROUTES[0])
        self.assertEqual(command.count(contract.PHASE130_SELECTOR), 1)
        for flag in (contract.PHASE126_SELECTOR, contract.PHASE127_SELECTOR,
                     contract.PHASE128_SELECTOR, contract.PHASE129_SELECTOR,
                     contract.PHASE118_SELECTOR):
            self.assertEqual(command.count(flag), 1)
        for flag in (contract.PHASE117_SELECTOR, contract.PHASE120_SELECTOR,
                     contract.ADDITIONAL_SELECTOR):
            self.assertEqual(command.count(flag), 0)
        self.assertIn("__PHASE130_RAW_DEVICE_GNSS__", command)
        self.assertIn("__PHASE130_RAW_BASE_RINEX__", command)

    def test_launch_free_sources_have_no_process_or_payload_probe(self) -> None:
        combined = VALIDATOR.read_text(encoding="utf-8") + RUNNER.read_text(encoding="utf-8")
        for token in ("subprocess", "Popen", "os.system", "os.popen", "check_output"):
            self.assertNotIn(token, combined)
        self.assertIn("verify_pre_raw_static", combined)

    def test_pins_and_manifest_when_contract_is_present(self) -> None:
        self.assertTrue(FREEZE.is_file())
        self.assertTrue(MANIFEST.is_file())
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(freeze["phase"], 130)
        self.assertEqual(freeze["decision"]["candidate_id"], contract.CANDIDATE_ID)
        self.assertFalse(freeze["decision"]["implementation_authorized"])
        self.assertFalse(freeze["decision"]["raw_materialization_authorized"])
        self.assertEqual(contract.verify_freeze()["phase"], 130)
        self.assertEqual(contract.verify_manifest()["phase"], 130)

    def test_pre_raw_contract_is_zero_activity(self) -> None:
        result = contract.verify_pre_raw_static()
        self.assertEqual(result["raw_phone_gnss_reads"], 0)
        self.assertEqual(result["raw_base_rinex_reads"], 0)
        self.assertEqual(result["native_solver_invocations"], 0)
        self.assertEqual(result["truth_reads"], 0)
        self.assertFalse(result["raw_execution_authorized"])


if __name__ == "__main__":
    unittest.main()
