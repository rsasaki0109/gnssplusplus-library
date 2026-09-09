"""Launch-free tests for the Phase129 inventory-first structural contract."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))
import gnss_smartphone_phase129_glonass_local_miss_structural as contract  # noqa: E402


FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase129_glonass_local_miss_structural_contract_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase129_glonass_local_miss_structural_manifest_v1.json"
VALIDATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase129_glonass_local_miss_structural.py"
RUNNER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase129_glonass_local_miss_structural_execute.py"


def passing_inventory(route: str) -> dict:
    side = {
        "input_rows": 8,
        "certified_rows": 6,
        "explicit_local_miss_rows": 2,
        "header_primary_certified_rows": 3,
        "broadcast_geph_certified_rows": 3,
        "reason_counts": {"missing-fcn": 1, "query-time-gap": 1},
        "all_rows_classified": True,
        "no_raw_uncorrected": True,
        "no_zero_correction": True,
        "no_fallback": True,
        "no_extrapolation": True,
        "finite_certified_wavelength": True,
        "non_glonass": {"input_rows": 12, "retained_rows": 12},
    }
    return {
        "route": route,
        "stage": "post-authorization-pre-solver",
        "solver_invocations": 0,
        "solver_may_start": True,
        "rover": json.loads(json.dumps(side)),
        "base": json.loads(json.dumps(side)),
        "shared_ledger": {
            "base_factor_ledger_equal": True,
            "exact_key_decision_equal": True,
            "certified_and_miss_input_conservation": True,
            "same_local_miss_reasons": True,
        },
        "correction": {
            "input_rows": 20,
            "retained_corrected_rows": 17,
            "explicit_provenance_miss": 2,
            "missing_stream": 1,
            "out_of_domain": 0,
            "nonfinite": 0,
            "application_passes": 1,
            "exactly_once": True,
            "no_duplicate_application": True,
            "no_raw_fallback": True,
            "no_zero_fallback": True,
            "finite_corrected_rows": True,
        },
        "usable_finite_base_streams": 2,
        "retained_corrected_factor_rows": 17,
        "all_miss_route": False,
        "empty_route": False,
        "phase129": {
            "enabled": True,
            "all_glonass_rows_certified_or_local_miss": True,
            "shared_base_factor_ledger_equal": True,
            "non_glonass_retained": True,
            "no_raw_uncorrected": True,
            "no_zero_correction": True,
            "no_fallback": True,
            "no_extrapolation": True,
            "source_complete_a_b_c": True,
        },
    }


class Phase129InventoryStructuralTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        cls.manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        cls.validator_source = VALIDATOR.read_text(encoding="utf-8")
        cls.runner_source = RUNNER.read_text(encoding="utf-8")

    def test_single_candidate_and_authorization_boundary(self) -> None:
        self.assertEqual(self.freeze["phase"], 129)
        self.assertEqual(self.freeze["candidate"]["candidate_count"], 1)
        self.assertEqual(self.freeze["candidate"]["candidate_id"], contract.CANDIDATE_ID)
        self.assertTrue(self.freeze["candidate"]["implementation_authorized"])
        self.assertFalse(self.freeze["candidate"]["raw_execution_authorized"])
        self.assertFalse(self.freeze["authorization_boundary"]["inventory_reads_authorized"])

    def test_selector_composition_and_off_lanes(self) -> None:
        implementation = self.manifest["implementation"]
        self.assertEqual(
            implementation["required_selectors"],
            [contract.PHASE126_SELECTOR, contract.PHASE127_SELECTOR,
             contract.PHASE128_SELECTOR, contract.PHASE129_SELECTOR,
             contract.PHASE118_SELECTOR],
        )
        self.assertEqual(implementation["main_solver"], "MULTIFRONTAL_QR")
        self.assertEqual(implementation["fixed_tdcp_sigma_m"], 0.03)
        self.assertEqual(implementation["official_tdcp_huber_k"], 0.5)
        self.assertFalse(implementation["phase117_dynamic_sigma"])
        self.assertFalse(implementation["phase120_selector"])
        self.assertFalse(implementation["additional_frequency_selector"])

    def test_route_order_and_one_shot_matrix(self) -> None:
        self.assertEqual([item["target"] for item in self.manifest["routes"]], ["MTV-A", "LAX-T"])
        self.assertEqual([item["dataset_id"] for item in self.manifest["routes"]], list(contract.ROUTES))
        self.assertEqual(self.manifest["matrix"]["runs_per_route"], 1)
        self.assertEqual(self.manifest["matrix"]["controls"], 0)
        self.assertEqual(self.manifest["matrix"]["reruns"], 0)
        self.assertEqual(self.manifest["matrix"]["fallbacks"], 0)

    def test_command_has_phase129_exactly_once_and_no_forbidden_lanes(self) -> None:
        for route in contract.ROUTES:
            record = next(item for item in self.manifest["routes"] if item["dataset_id"] == route)
            command = record["command"]
            self.assertEqual(command, contract.command_template(route))
            for flag in (contract.PHASE126_SELECTOR, contract.PHASE127_SELECTOR,
                         contract.PHASE128_SELECTOR, contract.PHASE129_SELECTOR,
                         contract.PHASE118_SELECTOR):
                self.assertEqual(command.count(flag), 1)
            for flag in (contract.PHASE117_SELECTOR, contract.PHASE120_SELECTOR,
                         contract.ADDITIONAL_SELECTOR):
                self.assertEqual(command.count(flag), 0)
            self.assertIn("__PHASE129_RAW_DEVICE_GNSS__", command)
            self.assertIn("__PHASE129_RAW_DEVICE_IMU__", command)
            self.assertIn("__PHASE129_RAW_BROADCAST_NAV__", command)
            self.assertIn("__PHASE129_RAW_BASE_RINEX__", command)

    def test_complete_inventory_is_admitted(self) -> None:
        report = contract.verify_inventory_record(contract.ROUTES[0], passing_inventory(contract.ROUTES[0]))
        self.assertTrue(report["inventory_passed"])
        self.assertEqual(report["solver_invocations"], 0)
        self.assertTrue(report["solver_may_start"])
        self.assertEqual(report["glonass_local_miss_rows"], 2)

    def test_partition_shared_ledger_and_empty_route_fail_closed(self) -> None:
        inventory = passing_inventory(contract.ROUTES[0])
        inventory["rover"]["certified_rows"] = 5
        with self.assertRaises(contract.Phase129ContractError):
            contract.verify_inventory_record(contract.ROUTES[0], inventory)
        inventory = passing_inventory(contract.ROUTES[0])
        inventory["shared_ledger"]["base_factor_ledger_equal"] = False
        with self.assertRaises(contract.Phase129ContractError):
            contract.verify_inventory_record(contract.ROUTES[0], inventory)
        inventory = passing_inventory(contract.ROUTES[0])
        inventory["usable_finite_base_streams"] = 0
        with self.assertRaises(contract.Phase129ContractError):
            contract.verify_inventory_record(contract.ROUTES[0], inventory)

    def test_reason_conservation_and_non_glonass_retention(self) -> None:
        inventory = passing_inventory(contract.ROUTES[1])
        inventory["base"]["reason_counts"]["query-time-gap"] = 2
        with self.assertRaises(contract.Phase129ContractError):
            contract.verify_inventory_record(contract.ROUTES[1], inventory)
        inventory = passing_inventory(contract.ROUTES[1])
        inventory["base"]["non_glonass"]["retained_rows"] = 11
        with self.assertRaises(contract.Phase129ContractError):
            contract.verify_inventory_record(contract.ROUTES[1], inventory)

    def test_launch_free_sources_have_no_process_or_payload_probe(self) -> None:
        combined = self.validator_source + self.runner_source
        for token in ("subprocess", "Popen", "os.system", "os.popen", "check_output"):
            self.assertNotIn(token, combined)
        self.assertIn("verify_pre_raw_static", self.validator_source)
        self.assertIn("materialize_after_authorization", json.dumps(self.manifest))

    def test_pre_raw_manifest_accounting_is_zero(self) -> None:
        accounting = self.manifest["read_accounting_before_authorization"]
        for key, value in accounting.items():
            if key.endswith("copied_or_transformed"):
                self.assertFalse(value)
            else:
                self.assertEqual(value, 0, key)

    def test_static_validator_checks_all_pins_without_payload_activity(self) -> None:
        contract.verify_freeze()
        contract.verify_manifest()
        report = contract.verify_pre_raw_static()
        self.assertEqual(report["raw_base_rinex_reads"], 0)
        self.assertEqual(report["native_solver_invocations"], 0)
        self.assertFalse(report["raw_execution_authorized"])


if __name__ == "__main__":
    unittest.main()
