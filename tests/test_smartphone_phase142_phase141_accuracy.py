"""Launch-free qualification for the Phase142 truth-only accuracy lane.

These tests inspect only pinned source and sealed structural/contract metadata.
They never materialize candidate or truth payloads, read raw/native output, or
launch a solver/evaluator.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase142_phase141_accuracy.py"


def load_contract():
    spec = importlib.util.spec_from_file_location("phase142_accuracy_contract_test", CONTRACT)
    if spec is None or spec.loader is None:
        raise AssertionError(f"unable to import {CONTRACT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase142AccuracyContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.contract = load_contract()

    def test_launch_free_source_has_no_solver_or_payload_reader(self) -> None:
        source = CONTRACT.read_text(encoding="utf-8")
        self.assertNotIn("import subprocess", source)
        self.assertNotIn("subprocess.", source)
        self.assertNotIn("Popen", source)

    def test_pre_truth_accounting_is_zero(self) -> None:
        # The pre-truth seal is tracked metadata; its contents contain no
        # candidate/truth rows and its verifier does not open deferred paths.
        pre = self.contract.verify_pre_truth()
        for key in (
            "candidate_paths_materialized", "truth_paths_materialized",
            "candidate_solution_reads", "candidate_coordinate_interpretations",
            "truth_reads", "accuracy_calculations", "native_solver_invocations",
            "raw_gnss_imu_navigation_reads", "raw_base_rinex_reads",
            "mat_pdc_precomputed_coordinate_reads", "kaggle_or_token_access",
            "reruns", "fallbacks", "repairs",
        ):
            self.assertEqual(pre["read_accounting"][key], 0, key)
        self.assertFalse(pre["read_accounting"]["truth_only_authorized"])
        self.assertFalse(pre["read_accounting"]["raw_execution_or_solver_authorized"])
        self.assertFalse(pre["read_accounting"]["solution_output_published"])

    def test_freeze_and_structural_candidate_are_opaque(self) -> None:
        freeze = self.contract.verify_freeze()
        structural = self.contract.verify_structural_result()
        self.assertEqual(freeze["phase"], 142)
        self.assertEqual(freeze["candidate"]["candidate_count"], 1)
        self.assertFalse(freeze["candidate"]["solution_publication"])
        self.assertTrue(freeze["candidate"]["phase141_pixel5_offset_boundary"]["already_applied_in_structural_run"])
        self.assertTrue(freeze["candidate"]["phase141_pixel5_offset_boundary"]["evaluator_must_not_apply_again"])
        self.assertEqual(structural["phase"], 141)
        self.assertEqual(structural["status"], "sealed-structural-raw-result-no-accuracy")
        self.assertEqual(structural["read_accounting"]["truth_reads"], 0)
        self.assertFalse(structural["policy"]["solution_coordinate_interpretation"])
        for route in self.contract.ROUTES:
            candidate = freeze["candidate"]["routes_metadata"][route]
            self.assertTrue(candidate["opaque_metadata_seal"])
            self.assertFalse(candidate["path_materialization_before_authorization"])
            structural_route = self.contract.STRUCTURAL_LABELS[route]
            self.assertEqual(candidate["sha256"], structural["routes"][0 if route == self.contract.ROUTES[0] else 1]["opaque_solution"]["sha256"])

    def test_metric_manifest_and_route_order_are_pinned(self) -> None:
        manifest = self.contract.verify_manifest()
        self.assertEqual(manifest["routes"], list(self.contract.ROUTES))
        self.assertEqual(manifest["candidate_reference"]["candidate_count"], 1)
        self.assertEqual(manifest["metric_contract"], self.contract.metric_contract())
        self.assertEqual(manifest["metric_contract"]["key"], "(phone, UnixTimeMillis)")
        self.assertEqual(manifest["metric_contract"]["earth_radius_m"], 6371008.8)
        self.assertEqual(manifest["metric_contract"]["route_scalar"], "(P50 + P95) / 2 in metres")
        self.assertEqual(manifest["metric_contract"]["macro"], "unweighted arithmetic mean over exactly MTV-A then LAX-T")
        self.assertEqual(manifest["metric_contract"]["strict_promotion_comparator"], "candidate_macro_score_m < 0.782")
        self.assertEqual(manifest["read_accounting_before_authorization"]["truth_reads"], 0)

    def test_payload_paths_require_future_independent_authorization(self) -> None:
        freeze = self.contract.verify_freeze()
        for route in self.contract.ROUTES:
            with self.assertRaises(self.contract.Phase142AccuracyError):
                self.contract.materialize_candidate_path(freeze, route, authorized=False)
            with self.assertRaises(self.contract.Phase142AccuracyError):
                self.contract.materialize_truth_path(freeze, route, authorized=False)

    def test_strict_gate_and_planned_read_boundary(self) -> None:
        self.assertTrue(self.contract.strict_macro_gate(0.781999999))
        self.assertFalse(self.contract.strict_macro_gate(0.782))
        self.assertFalse(self.contract.strict_macro_gate(0.782000001))
        self.assertFalse(self.contract.strict_macro_gate(float("nan")))
        freeze = self.contract.verify_freeze()
        boundary = freeze["truth_evaluator_boundary"]
        self.assertTrue(boundary["candidate_metadata_hash_seal_precedes_truth"])
        self.assertTrue(boundary["candidate_parse_precedes_truth"])
        self.assertFalse(boundary["truth_path_or_bytes_in_native"])
        planned = freeze["read_accounting"]["planned_truth_evaluation"]
        self.assertEqual(planned["candidate_solution_reads_for_hash_and_parse"], 2)
        self.assertEqual(planned["truth_reads"], 2)
        self.assertEqual(planned["accuracy_calculations"], 2)

    def test_truth_metadata_is_deferred_and_not_candidate_rows(self) -> None:
        freeze = self.contract.verify_freeze()
        self.assertFalse(freeze["truth_cohort"]["read_by_audit"])
        self.assertFalse(freeze["truth_cohort"]["read_by_freeze"])
        for route in self.contract.ROUTES:
            candidate = freeze["candidate"]["routes_metadata"][route]
            truth = freeze["truth_cohort"]["routes"][route]
            self.assertEqual(candidate["expected_prediction_rows"], self.contract.DOMAIN_ROWS[route])
            self.assertEqual(candidate["expected_problem_epochs"], self.contract.PROBLEM_EPOCHS[route])
            self.assertEqual(truth["rows"], self.contract.TRUTH_ROWS[route])
            self.assertIn("/truth/", truth["path"])
            self.assertNotIn("coordinate_rows", candidate)


if __name__ == "__main__":
    unittest.main()
