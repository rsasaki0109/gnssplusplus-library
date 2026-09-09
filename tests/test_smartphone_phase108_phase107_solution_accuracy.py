"""Launch-free tests for the Phase108 truth-only accuracy contract.

These tests verify only sealed metadata and source.  They do not open either
withheld candidate CSV, official truth CSV, raw input, or base RINEX member,
and they do not launch a solver or evaluator subprocess.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase108_phase107_solution_accuracy.py"


def load_contract():
    spec = importlib.util.spec_from_file_location("phase108_solution_accuracy_contract_test", CONTRACT_PATH)
    if spec is None or spec.loader is None:
        raise AssertionError(f"unable to import {CONTRACT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase108SolutionAccuracyContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract = load_contract()

    def test_freeze_pins_phase107_structural_go_and_opaque_solution_seals(self):
        freeze = self.contract.verify_freeze()
        self.assertEqual(freeze["candidate"]["id"], self.contract.CANDIDATE_ID)
        self.assertTrue(freeze["candidate"]["metadata_hash_seal"]["opaque_bytes_only"])
        self.assertFalse(freeze["candidate"]["metadata_hash_seal"]["coordinate_interpretation"])
        self.assertFalse(freeze["candidate"]["metadata_hash_seal"]["csv_parse"])
        self.assertEqual(freeze["candidate"]["routes_metadata"][self.contract.ROUTES[0]]["sha256"], "b401156e76d5a1a1cfc948aa666d69d292fe4ba43ea3e3f4580cde7e5f0339f8")
        self.assertEqual(freeze["candidate"]["routes_metadata"][self.contract.ROUTES[1]]["sha256"], "6e85865825a250628db4d6aa842fc5992ebdc63aa51ffaf7f1dd513c1fe600d8")

    def test_pre_truth_is_zero_and_truth_boundary_is_evaluator_only(self):
        pre = self.contract.verify_pre_truth()
        self.assertEqual(pre["native_solver_invocations"], 0)
        self.assertEqual(pre["raw_reads"], 0)
        self.assertEqual(pre["base_reads"], 0)
        self.assertEqual(pre["truth_reads"], 0)
        self.assertEqual(pre["accuracy_calculations"], 0)
        self.assertFalse(pre["native_rerun"])
        manifest = self.contract.verify_manifest()
        self.assertEqual(manifest["truth_evaluator_boundary"]["truth_reads_per_route"], 1)
        self.assertTrue(manifest["truth_evaluator_boundary"]["candidate_parse_precedes_truth"])

    def test_metric_contract_and_strict_gate_are_unchanged(self):
        freeze = self.contract.verify_freeze()
        metric = freeze["metric_contract"]
        self.assertEqual(metric["earth_radius_m"], 6371008.8)
        self.assertEqual(metric["route_scalar"], "(P50 + P95) / 2 in metres")
        self.assertEqual(metric["macro"], "unweighted arithmetic mean over exactly MTV-A then LAX-T")
        gates = freeze["promotion_gates"]
        self.assertEqual(gates["candidate_macro_score_strict_max_m"], 0.782)
        self.assertTrue(gates["strict_0_782_is_explicit_and_gate"])
        self.assertTrue(gates["all_gates_anded"])

    def test_phase107_result_is_accuracy_free_and_not_rerun(self):
        result = self.contract._verify_pinned_json(self.contract.PHASE107_RESULT, self.contract.PHASE107_RESULT_SHA, "Phase107 result")
        self.assertEqual(result["status"], "go-phase107-raw-base-structural")
        self.assertFalse(result["accuracy_scored"])
        self.assertFalse(result["solution_output_published"])
        self.assertEqual(result["read_accounting"]["truth_reads"], 0)
        self.assertEqual(result["read_accounting"]["route_reruns"], 0)

    def test_evaluator_has_no_native_subprocess_or_raw_read_api(self):
        source = CONTRACT_PATH.read_text(encoding="utf-8")
        self.assertNotIn("import subprocess", source)
        self.assertNotIn("subprocess.run", source)
        self.assertNotIn("os.system", source)
        self.assertNotIn("device_gnss.csv", source)
        self.assertNotIn("device_imu.csv", source)
        self.assertNotIn("brdc.nav", source)

    def test_forbidden_solution_rows_are_not_result_fields(self):
        source = CONTRACT_PATH.read_text(encoding="utf-8")
        self.assertIn("coordinate_rows_omitted", source)
        self.assertIn("solution_rows_in_result", source)
        self.assertIn("solution_output_published", source)


if __name__ == "__main__":
    unittest.main()
