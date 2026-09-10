"""Launch-free qualification for the Phase137 isolated accuracy lane.

Only tracked evaluator source and sealed metadata are read.  The tests do not
materialize or open candidate/truth payloads, raw inputs, native output, MAT,
PDC, precomputed coordinates, or Kaggle artifacts.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase137_phase136_accuracy.py"


def load_contract():
    spec = importlib.util.spec_from_file_location("phase137_accuracy_contract_test", CONTRACT)
    if spec is None or spec.loader is None:
        raise AssertionError(f"unable to import {CONTRACT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase137AccuracyContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.contract = load_contract()

    def test_pre_truth_is_zero_and_no_solver_path_exists(self) -> None:
        pre = self.contract.verify_pre_truth()
        for key in (
            "candidate_paths_materialized", "truth_paths_materialized",
            "candidate_solution_reads", "candidate_coordinate_interpretations",
            "truth_reads", "accuracy_calculations", "native_solver_invocations",
            "raw_gnss_imu_navigation_reads", "raw_base_rinex_reads",
            "mat_precomputed_phone_coordinate_pdc_reads", "kaggle_or_token_access",
            "reruns", "fallbacks",
        ):
            self.assertEqual(pre[key], 0, key)
        self.assertFalse(pre["truth_only_authorized"])
        self.assertFalse(pre["raw_execution_or_solver_authorized"])
        self.assertFalse(pre["solution_output_published"])
        source = CONTRACT.read_text(encoding="utf-8")
        self.assertNotIn("import subprocess", source)
        self.assertNotIn("subprocess.", source)
        self.assertNotIn("Popen", source)

    def test_freeze_and_structural_candidate_are_opaque(self) -> None:
        freeze = self.contract.verify_freeze()
        structural = self.contract.verify_structural_result()
        self.assertEqual(freeze["phase"], 137)
        self.assertEqual(freeze["candidate"]["candidate_count"], 1)
        self.assertFalse(freeze["candidate"]["solution_publication"])
        self.assertTrue(freeze["candidate"]["phase136_pixel5_offset_boundary"]["already_applied_in_structural_run"])
        self.assertTrue(freeze["candidate"]["phase136_pixel5_offset_boundary"]["evaluator_must_not_apply_again"])
        self.assertEqual(structural["phase"], 136)
        self.assertEqual(structural["status"], "sealed-structural-raw-result-no-accuracy")
        self.assertEqual(structural["read_accounting"]["truth_reads"], 0)
        self.assertFalse(structural["policy"]["solution_coordinate_interpretation"])
        for route in self.contract.ROUTES:
            candidate = freeze["candidate"]["routes_metadata"][route]
            self.assertTrue(candidate["opaque_metadata_seal"])
            self.assertFalse(candidate["bytes_and_rows_probed_before_authorization"])
            structural_route = self.contract.STRUCTURAL_LABELS[route]
            self.assertEqual(candidate["sha256"], structural["route_results"][structural_route]["opaque_solution"]["sha256"])

    def test_manifest_reuses_pinned_metric_and_order(self) -> None:
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

    def test_payload_paths_cannot_materialize_before_authorization(self) -> None:
        freeze = self.contract.verify_freeze()
        for route in self.contract.ROUTES:
            with self.assertRaises(self.contract.Phase137AccuracyError):
                self.contract.materialize_candidate_path(freeze, route, authorized=False)
            with self.assertRaises(self.contract.Phase137AccuracyError):
                self.contract.materialize_truth_path(freeze, route, authorized=False)

    def test_strict_gate_and_planned_reads(self) -> None:
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

    def test_candidate_and_truth_metadata_have_no_payload_rows(self) -> None:
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
            self.assertFalse(candidate["bytes_and_rows_probed_before_authorization"])


if __name__ == "__main__":
    unittest.main()
