"""Launch-free tests for the Phase117 truth-only accuracy contract."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase117_tdcp_weighting_accuracy.py"


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise AssertionError(f"unable to import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase117TruthOnlyAccuracyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract = load_module(CONTRACT, "phase117_truth_only_accuracy_contract_test")

    def test_pre_truth_is_zero_and_does_not_launch_native(self):
        pre = self.contract.verify_pre_truth()
        self.assertEqual(pre["native_solver_invocations"], 0)
        self.assertEqual(pre["raw_reads"], 0)
        self.assertEqual(pre["base_reads"], 0)
        self.assertEqual(pre["truth_reads"], 0)
        self.assertEqual(pre["accuracy_calculations"], 0)
        self.assertFalse(pre["native_rerun"])
        source = CONTRACT.read_text(encoding="utf-8")
        self.assertNotIn("subprocess.run", source)
        self.assertNotIn("Popen", source)

    def test_freeze_and_structural_result_are_opaque_and_go(self):
        freeze = self.contract.verify_freeze()
        structural = self.contract.verify_structural_result()
        self.assertEqual(freeze["phase"], 117)
        self.assertEqual(freeze["candidate"]["candidate_count"], 1)
        self.assertFalse(freeze["candidate"]["solution_publication"])
        self.assertEqual(structural["status"], "go-phase117-tdcp-weighting-structural")
        self.assertFalse(structural["accuracy_scored"])
        self.assertEqual(structural["read_accounting"]["truth_reads"], 0)

    def test_manifest_has_exact_two_routes_and_frozen_metric(self):
        manifest = self.contract.verify_manifest()
        self.assertEqual(manifest["routes"], list(self.contract.ROUTES))
        self.assertEqual(manifest["candidate"]["candidate_count"], 1)
        self.assertFalse(manifest["candidate"]["solver_rerun"])
        self.assertFalse(manifest["candidate"]["tdcp_rerun"])
        self.assertEqual(manifest["metric_contract"]["strict_macro_gate_m"], 0.782)
        self.assertEqual(manifest["read_accounting_before_authorization"]["truth_reads"], 0)

    def test_authorization_is_truth_only_and_does_not_rerun_solver(self):
        authorization = self.contract.verify_authorization()
        self.assertEqual(authorization["candidate"]["candidate_count"], 1)
        self.assertFalse(authorization["candidate"]["solver_rerun"])
        self.assertFalse(authorization["candidate"]["raw_or_base_rerun"])
        self.assertEqual(authorization["execution_policy"]["truth_reads_per_route"], 1)
        self.assertFalse(authorization["execution_policy"]["native_after_truth"])

    def test_truth_paths_are_metadata_only_and_candidate_rows_are_omitted(self):
        freeze = self.contract.verify_freeze()
        for route in self.contract.ROUTES:
            truth = freeze["truth_cohort"]["routes"][route]
            self.assertIn("/truth/", truth["path"])
            self.assertEqual(len(truth["sha256"]), 64)
            candidate = freeze["candidate"]["routes_metadata"][route]
            self.assertTrue(candidate["opaque_metadata_seal"])
            self.assertEqual(candidate["expected_prediction_rows"], self.contract.DOMAIN_ROWS[route])
        source = CONTRACT.read_text(encoding="utf-8")
        self.assertIn("coordinate_rows_omitted", source)
        self.assertIn("truth_reads_per_route", source)


if __name__ == "__main__":
    unittest.main()
