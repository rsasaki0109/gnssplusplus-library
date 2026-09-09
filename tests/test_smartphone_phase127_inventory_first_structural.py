"""Launch-free tests for the Phase127 inventory-first contract.

The fixtures are in-memory metadata only.  No route payload, solution row,
truth record, or native application is opened by this test module.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))
import gnss_smartphone_phase127_inventory_first_structural as contract  # noqa: E402


FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase127_inventory_first_structural_contract_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase127_inventory_first_structural_manifest_v1.json"
VALIDATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase127_inventory_first_structural.py"
RUNNER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase127_inventory_first_structural_execute.py"


def passing_inventory(route: str) -> dict:
    return {
        "route": route,
        "stage": "post-authorization-pre-solver",
        "solver_invocations": 0,
        "solver_may_start": True,
        "header": {
            "entries_seen": 2,
            "malformed_entries": 0,
            "conflict_entries": 0,
            "invalid_entries": 0,
            "unresolved_entries": 0,
            "primary_or_nav_fallback_accounted": True,
            "selected_geph_fcn_matches": True,
            "exact_satellite_keys": True,
            "exact_query_times": True,
        },
        "geph": {
            "query_rows": 4,
            "selected_rows": 4,
            "valid_rows": 4,
            "finite_fcns": 4,
            "in_domain_fcns": 4,
            "max_age_s": 1800.0,
            "query_times_exact": True,
            "coverage_gaps": 0,
            "different_fcn_ties": 0,
            "conflicts": 0,
            "missing_fcn": 0,
            "out_of_range": 0,
            "duplicate_entries": 0,
        },
        "coverage": {
            "rover_glonass_rows": 4,
            "rover_certified_rows": 4,
            "rover_unresolved_rows": 0,
            "rover_full": True,
            "base_glonass_rows": 4,
            "base_certified_rows": 4,
            "base_unresolved_rows": 0,
            "base_full": True,
            "all_finite_positive_frequency_wavelength": True,
        },
        "phase126": {
            "source_complete": True,
            "base_correction_exactly_once": True,
        },
    }


class Phase127InventoryFirstStructuralTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        cls.manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        cls.validator_source = VALIDATOR.read_text(encoding="utf-8")
        cls.runner_source = RUNNER.read_text(encoding="utf-8")

    def test_single_candidate_and_authorization_boundary(self) -> None:
        self.assertEqual(self.freeze["phase"], 127)
        self.assertEqual(self.freeze["candidate"]["candidate_count"], 1)
        self.assertEqual(self.freeze["candidate"]["candidate_id"], contract.CANDIDATE_ID)
        self.assertTrue(self.freeze["candidate"]["default_off"])
        self.assertTrue(self.freeze["candidate"]["legacy_default_unchanged"])
        self.assertFalse(self.freeze["candidate"]["raw_execution_authorized"])
        self.assertFalse(self.freeze["authorization_boundary"]["raw_inventory_reads_authorized"])

    def test_recipe_selector_isolation(self) -> None:
        implementation = self.manifest["implementation"]
        self.assertEqual(implementation["phase126_selector"], contract.PHASE126_SELECTOR)
        self.assertEqual(implementation["phase127_selector"], contract.SELECTOR)
        self.assertEqual(implementation["phase118_selector"], contract.PHASE118_SELECTOR)
        self.assertEqual(implementation["main_solver"], "MULTIFRONTAL_QR")
        self.assertEqual(implementation["fixed_tdcp_sigma_m"], 0.03)
        self.assertEqual(implementation["official_tdcp_huber_k"], 0.5)
        self.assertFalse(implementation["phase117_dynamic_sigma"])
        self.assertFalse(implementation["phase120_selector"])
        self.assertFalse(implementation["additional_frequency_selector"])

    def test_route_order_and_exact_one_shot(self) -> None:
        self.assertEqual([item["target"] for item in self.manifest["routes"]], ["MTV-A", "LAX-T"])
        self.assertEqual([item["dataset_id"] for item in self.manifest["routes"]], list(contract.ROUTES))
        self.assertEqual(self.manifest["matrix"]["runs_per_route"], 1)
        self.assertEqual(self.manifest["matrix"]["controls"], 0)
        self.assertEqual(self.manifest["matrix"]["reruns"], 0)
        self.assertEqual(self.manifest["matrix"]["fallbacks"], 0)

    def test_commands_have_exact_selectors_and_only_placeholders(self) -> None:
        for route in contract.ROUTES:
            record = next(item for item in self.manifest["routes"] if item["dataset_id"] == route)
            command = record["command"]
            self.assertEqual(command, contract.command_template(route))
            for flag in (contract.PHASE126_SELECTOR, contract.SELECTOR, contract.PHASE118_SELECTOR):
                self.assertEqual(command.count(flag), 1)
            self.assertEqual(command.count(contract.PHASE117_SELECTOR), 0)
            self.assertEqual(command.count(contract.PHASE120_SELECTOR), 0)
            self.assertEqual(command.count(contract.ADDITIONAL_BAND_SELECTOR), 0)
            self.assertIn("__PHASE127_RAW_DEVICE_GNSS__", command)
            self.assertIn("__PHASE127_RAW_DEVICE_IMU__", command)
            self.assertIn("__PHASE127_RAW_BROADCAST_NAV__", command)
            self.assertIn("__PHASE127_RAW_BASE_RINEX__", command)

    def test_complete_inventory_is_admitted(self) -> None:
        report = contract.verify_inventory_record(contract.ROUTES[0], passing_inventory(contract.ROUTES[0]))
        self.assertTrue(report["inventory_passed"])
        self.assertEqual(report["solver_invocations"], 0)
        self.assertTrue(report["solver_may_start"])

    def test_header_geph_mismatch_fails_closed(self) -> None:
        inventory = passing_inventory(contract.ROUTES[0])
        inventory["header"]["selected_geph_fcn_matches"] = False
        with self.assertRaises(contract.Phase127ContractError):
            contract.verify_inventory_record(contract.ROUTES[0], inventory)

    def test_query_gap_and_domain_fail_closed(self) -> None:
        inventory = passing_inventory(contract.ROUTES[1])
        inventory["geph"]["coverage_gaps"] = 1
        with self.assertRaises(contract.Phase127ContractError):
            contract.verify_inventory_record(contract.ROUTES[1], inventory)
        inventory = passing_inventory(contract.ROUTES[1])
        inventory["geph"]["max_age_s"] = 1800.0001
        with self.assertRaises(contract.Phase127ContractError):
            contract.verify_inventory_record(contract.ROUTES[1], inventory)

    def test_partial_coverage_fails_closed_and_solver_stays_zero(self) -> None:
        inventory = passing_inventory(contract.ROUTES[0])
        inventory["coverage"]["base_certified_rows"] = 3
        inventory["solver_may_start"] = False
        self.assertEqual(inventory["solver_invocations"], 0)
        with self.assertRaises(contract.Phase127ContractError):
            contract.verify_inventory_record(contract.ROUTES[0], inventory)

    def test_launch_free_sources_have_no_child_process_or_payload_probe(self) -> None:
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

    def test_static_validator_checks_sealed_artifacts(self) -> None:
        contract.verify_freeze()
        contract.verify_manifest()
        report = contract.verify_pre_raw_static()
        self.assertEqual(report["raw_base_rinex_reads"], 0)
        self.assertEqual(report["native_solver_invocations"], 0)
        self.assertEqual(report["historical_phase126_source_hash_check"], "mismatch-retained")
        self.assertFalse(report["raw_execution_authorized"])


if __name__ == "__main__":
    unittest.main()
