"""Launch-free Phase126 contract tests.

Only tracked source and sealed metadata are inspected.  No raw phone/base
payload, truth, MAT, coordinate row, solver, or solution content is opened.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))
import gnss_smartphone_phase126_raw_base_compound_structural as contract  # noqa: E402


FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase126_raw_base_compound_structural_contract_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase126_raw_base_compound_structural_manifest_v1.json"
VALIDATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase126_raw_base_compound_structural.py"
RUNNER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase126_raw_base_compound_structural_execute.py"


class Phase126StructuralContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        cls.manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        cls.validator_source = VALIDATOR.read_text(encoding="utf-8")
        cls.runner_source = RUNNER.read_text(encoding="utf-8")

    def test_single_default_off_candidate_and_authorization_boundary(self) -> None:
        self.assertEqual(self.freeze["phase"], 126)
        self.assertEqual(self.freeze["implementation"]["commit"], contract.IMPLEMENTATION_COMMIT)
        self.assertEqual(self.freeze["implementation"]["selector"], contract.SELECTOR)
        self.assertTrue(self.freeze["implementation"]["default_off"])
        self.assertFalse(self.freeze["implementation"]["partial_selectors_allowed"])
        self.assertFalse(self.freeze["authorization"]["raw_execution_authorized"])
        self.assertFalse(self.freeze["authorization"]["truth_evaluation_authorized"])

    def test_phase118_recipe_and_phase126_selector_isolation(self) -> None:
        recipe = self.freeze["fixed_phase118_recipe"]
        self.assertEqual(recipe["main_solver"], "MULTIFRONTAL_QR")
        self.assertEqual(recipe["fixed_tdcp_sigma_m"], 0.03)
        self.assertEqual(recipe["official_tdcp_huber_k"], 0.5)
        self.assertFalse(recipe["phase117_dynamic_sigma"])
        self.assertFalse(recipe["phase120_atmosphere_selector"])
        self.assertFalse(recipe["phase109_additional_frequency_selector"])
        self.assertIn(contract.SELECTOR, self.validator_source)

    def test_atomic_source_contract_and_equations_are_frozen(self) -> None:
        source = self.freeze["phase126_contract"]
        self.assertEqual(source["moving_mean_samples"], {"MTV-A": 151, "LAX-T": 11})
        self.assertIn("P_base_m + satellite_clock_m - geometric_range_m", source["resPc_m"])
        self.assertIn("interpolated_smoothed_base_resPc_m", source["application_m"])
        self.assertTrue(source["transactional_no_partial_application"])
        self.assertTrue(self.freeze["structural_gates"]["stream_miss_conservation"])
        self.assertTrue(self.freeze["structural_gates"]["atomic_a_b_c"])

    def test_manifest_route_order_and_one_shot_matrix(self) -> None:
        self.assertEqual(self.manifest["routes"][0]["target"], "MTV-A")
        self.assertEqual(self.manifest["routes"][1]["target"], "LAX-T")
        self.assertEqual([x["dataset_id"] for x in self.manifest["routes"]], list(contract.ROUTES))
        self.assertEqual(self.manifest["matrix"]["runs_per_route"], 1)
        self.assertEqual(self.manifest["matrix"]["controls"], 0)
        self.assertEqual(self.manifest["matrix"]["reruns"], 0)
        self.assertEqual(self.manifest["matrix"]["fallbacks"], 0)

    def test_commands_have_only_raw_placeholders_and_exact_selector_counts(self) -> None:
        for route in contract.ROUTES:
            record = next(item for item in self.manifest["routes"] if item["dataset_id"] == route)
            command = record["command"]
            self.assertEqual(command, contract.command_template(route))
            self.assertEqual(command.count(contract.SELECTOR), 1)
            self.assertEqual(command.count(contract.PHASE118_SELECTOR), 1)
            self.assertEqual(command.count(contract.PHASE117_SELECTOR), 0)
            self.assertEqual(command.count(contract.PHASE120_SELECTOR), 0)
            self.assertEqual(command.count(contract.ADDITIONAL_BAND_SELECTOR), 0)
            self.assertIn("__PHASE126_RAW_DEVICE_GNSS__", command)
            self.assertIn("__PHASE126_RAW_DEVICE_IMU__", command)
            self.assertIn("__PHASE126_RAW_BROADCAST_NAV__", command)
            self.assertIn("__PHASE126_RAW_BASE_RINEX__", command)

    def test_static_runner_has_no_launch_or_payload_probe(self) -> None:
        combined = self.validator_source + self.runner_source
        for token in ("subprocess", "Popen", "os.system", "os.popen", "check_output"):
            self.assertNotIn(token, combined)
        self.assertIn("verify_pre_raw_static", self.validator_source)
        self.assertIn("materialize_after_authorization", json.dumps(self.manifest))

    def test_base_inventory_is_post_authorization_and_fail_closed(self) -> None:
        for record in self.manifest["routes"]:
            base = record["base_input"]
            self.assertTrue(base["raw_rinex_only"])
            self.assertTrue(base["header_inventory_after_authorization"])
            self.assertTrue(base["signal_inventory_complete"])
            self.assertTrue(base["glonass_channel_frequency_if_present"])
            self.assertTrue(base["state_atmosphere_finite_in_domain"])
            self.assertFalse(base["payload_read_before_authorization"])
            self.assertFalse(base["hash_read_before_authorization"])

    def test_pre_authorization_accounting_is_zero(self) -> None:
        accounting = self.manifest["read_accounting_before_authorization"]
        for key, value in accounting.items():
            if key.endswith("copied_or_transformed"):
                self.assertFalse(value)
            else:
                self.assertEqual(value, 0, key)

    def test_launch_free_validator_passes_sealed_files(self) -> None:
        contract.verify_freeze()
        contract.verify_manifest()
        report = contract.verify_pre_raw_static()
        self.assertEqual(report["raw_base_rinex_reads"], 0)
        self.assertEqual(report["native_solver_invocations"], 0)
        self.assertFalse(report["raw_execution_authorized"])


if __name__ == "__main__":
    unittest.main()
