"""Launch-free Phase116 ordinary-TDCP diagnostic contract tests.

The tests read only tracked source/manifest/freeze metadata.  They never stat
or open raw phone/base members, solution CSVs, truth, MAT, coordinates, or
Kaggle resources, and they never launch the native solver.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase116_carrier_tdcp_incidence_diagnostic.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase116_carrier_tdcp_incidence_diagnostic_manifest_v1.json"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase116_carrier_tdcp_source_parity_freeze_v1.json"
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"


def load_contract():
    spec = importlib.util.spec_from_file_location("phase116_contract_test", CONTRACT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase116ExecutionContractTests(unittest.TestCase):
    def test_freeze_and_manifest_are_default_off_before_authorization(self) -> None:
        contract = load_contract()
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(freeze["phase"], 116)
        self.assertEqual(freeze["candidate"]["id"], contract.CANDIDATE_ID)
        self.assertTrue(freeze["candidate"]["default_off"])
        self.assertFalse(freeze["decision"]["raw_execution_authorized"])
        self.assertFalse(manifest["freeze"]["raw_execution_authorized_before_manifest"])
        self.assertTrue(manifest["candidate"]["solution_withheld"])
        self.assertFalse(manifest["matrix"]["solution_rows_authorized"])

    def test_exact_command_enables_only_ordinary_tdcp_diagnostic(self) -> None:
        contract = load_contract()
        for route in contract.ROUTES:
            command = contract.command_template(route)
            self.assertEqual(command.count(contract.DIAGNOSTIC_SELECTOR), 1)
            self.assertEqual(command.count("--native-base-rinex"), 1)
            self.assertEqual(command.count("--native-base-rinex-sha256"), 1)
            self.assertIn(contract.PHASE93_SELECTOR, command)
            self.assertIn(contract.VECTOR_SELECTOR, command)
            self.assertIn(contract.QR_SELECTOR, command)
            self.assertNotIn("truth", " ".join(command).lower())
            self.assertNotIn(".mat", " ".join(command).lower())
            self.assertNotIn("precomputed", " ".join(command).lower())

    def test_implementation_has_read_only_signal_counters(self) -> None:
        contract = load_contract()
        contract.verify_implementation()
        source = APP.read_text(encoding="utf-8")
        self.assertIn("config.use_carrier_tdcp_incidence_diagnostic = true", source)
        self.assertIn("Phase116CarrierTdcpReport", source)
        self.assertIn("factors_inserted_exact", source)

    def test_pre_raw_verifier_reports_zero_payload_activity(self) -> None:
        contract = load_contract()
        pre = contract.verify_pre_raw()
        for key in (
            "raw_phone_gnss_reads", "raw_phone_imu_reads", "broadcast_navigation_reads",
            "raw_base_rinex_reads", "raw_base_hash_reads", "native_solver_invocations",
            "truth_reads", "mat_reads_or_generated", "phone_coordinate_reads",
            "precomputed_coordinate_reads", "pdc_reads", "accuracy_calculations",
        ):
            if key in pre:
                self.assertEqual(pre[key], 0, key)
        self.assertFalse(pre["raw_execution_authorized"])
        self.assertFalse(pre["solution_output_published"])

    def test_manifest_schema_seals_costs_and_connected_keys(self) -> None:
        contract = load_contract()
        manifest = contract.verify_manifest()
        self.assertEqual(manifest["diagnostic_contract"]["costs"],
                         ["robust", "unwhitened", "whitened"])
        self.assertTrue(manifest["diagnostic_contract"]["connected_keys"])
        self.assertEqual(manifest["matrix"]["route_count"], 2)
        self.assertEqual(manifest["matrix"]["runs_per_route"], 1)


if __name__ == "__main__":
    unittest.main()
