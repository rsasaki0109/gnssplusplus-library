"""Launch-free tests for the Phase118 truth-only accuracy contract."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase118_tdcp_robust_k_accuracy.py"


def load_contract():
    spec = importlib.util.spec_from_file_location("phase118_accuracy_contract_test", CONTRACT)
    if spec is None or spec.loader is None:
        raise AssertionError(f"unable to import {CONTRACT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase118TruthOnlyAccuracyContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.contract = load_contract()

    def test_pre_truth_is_zero_and_launch_free(self) -> None:
        pre = self.contract.verify_pre_truth()
        for key in (
            "candidate_paths_materialized",
            "truth_paths_materialized",
            "native_solver_invocations",
            "raw_gnss_imu_navigation_reads",
            "raw_base_rinex_reads",
            "truth_reads",
            "candidate_solution_reads",
            "candidate_coordinate_interpretations",
            "accuracy_calculations",
            "mat_precomputed_phone_coordinate_pdc_reads",
            "kaggle_or_token_access",
            "reruns",
            "fallbacks",
        ):
            self.assertEqual(pre[key], 0, key)
        self.assertFalse(pre["truth_only_authorized"])
        self.assertFalse(pre["solution_output_published"])
        source = CONTRACT.read_text(encoding="utf-8")
        self.assertNotIn("import subprocess", source)
        self.assertNotIn("subprocess.", source)
        self.assertNotIn("Popen", source)

    def test_freeze_and_structural_result_are_opaque_and_go(self) -> None:
        freeze = self.contract.verify_freeze()
        structural = self.contract.verify_structural_result(freeze)
        self.assertEqual(freeze["phase"], 118)
        self.assertEqual(freeze["candidate"]["candidate_count"], 1)
        self.assertFalse(freeze["candidate"]["solution_publication"])
        self.assertTrue(freeze["candidate"]["phase118_offset_boundary"]["required_already_applied_in_structural_run"])
        self.assertTrue(freeze["candidate"]["phase118_offset_boundary"]["evaluator_must_not_apply_again"])
        self.assertEqual(structural["status"], "go-phase118-official-tdcp-huber-k-structural")
        self.assertFalse(structural["accuracy_scored"])
        self.assertEqual(structural["read_accounting"]["truth_reads"], 0)

    def test_manifest_reuses_metric_and_has_exact_route_order(self) -> None:
        manifest = self.contract.verify_manifest()
        self.assertEqual(manifest["routes"], list(self.contract.ROUTES) if "routes" in manifest else list(self.contract.ROUTES))
        self.assertEqual(manifest["candidate"]["candidate_count"], 1)
        self.assertFalse(manifest["candidate"]["solver_rerun"])
        self.assertFalse(manifest["candidate"]["tdcp_rerun"])
        self.assertEqual(manifest["metric_contract"]["key"], "(phone, UnixTimeMillis)")
        self.assertEqual(manifest["metric_contract"]["earth_radius_m"], 6371008.8)
        self.assertEqual(manifest["metric_contract"]["route_scalar"], "(P50 + P95) / 2 in metres")
        self.assertEqual(manifest["metric_contract"]["macro"], "unweighted arithmetic mean over exactly MTV-A then LAX-T")
        self.assertEqual(manifest["metric_contract"]["strict_promotion_comparator"], "candidate_macro_score_m < 0.782")
        self.assertEqual(manifest["read_accounting_before_authorization"]["truth_reads"], 0)

    def test_payload_paths_cannot_materialize_before_authorization(self) -> None:
        freeze = self.contract.verify_freeze()
        structural = self.contract.verify_structural_result(freeze)
        with self.assertRaises(self.contract.Phase118AccuracyError):
            self.contract.materialize_candidate_path(structural, self.contract.ROUTES[0], authorized=False)
        with self.assertRaises(self.contract.Phase118AccuracyError):
            self.contract.materialize_truth_path(freeze, self.contract.ROUTES[0], authorized=False)

    def test_strict_gate_is_strict_and_macro_order_is_fixed(self) -> None:
        self.assertTrue(self.contract.strict_macro_gate(0.781999999))
        self.assertFalse(self.contract.strict_macro_gate(0.782))
        self.assertFalse(self.contract.strict_macro_gate(0.782000001))
        self.assertFalse(self.contract.strict_macro_gate(float("nan")))

    def test_synthetic_rows_use_pinned_parser_and_metric_without_files(self) -> None:
        route = self.contract.ROUTES[1]
        payload = (
            b"phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
            + f"{route},1000,37.000000,-122.000000\n".encode()
            + f"{route},2000,37.000001,-122.000001\n".encode()
        )
        p82 = self.contract.load_phase82()
        ordered, mapping = p82.P76.P74._parse_submission(payload, route)
        truth = p82.P76._parse_truth_dictreader(payload, route)
        score = p82.P76._score_prediction(mapping, truth, None, route, ordered)
        self.assertEqual(len(ordered), 2)
        self.assertEqual(score["prediction_domain_coverage"], 1.0)
        self.assertTrue(score["finite"])
        self.assertEqual(score["over_70_mps_count"], 0)
        self.assertEqual(score["score_m"], 0.0)

    def test_synthetic_duplicate_key_fails_closed(self) -> None:
        route = self.contract.ROUTES[0]
        duplicate = (
            b"phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
            + f"{route},1000,37.0,-122.0\n".encode()
            + f"{route},1000,37.0,-122.0\n".encode()
        )
        p82 = self.contract.load_phase82()
        with self.assertRaises(Exception):
            p82.P76.P74._parse_submission(duplicate, route)

    def test_candidate_and_truth_metadata_never_claim_payload_reads(self) -> None:
        freeze = self.contract.verify_freeze()
        for route in self.contract.ROUTES:
            candidate = freeze["candidate"]["routes_metadata"][route]
            truth = freeze["truth_cohort"]["routes"][route]
            self.assertTrue(candidate["opaque_metadata_seal"])
            self.assertEqual(candidate["expected_prediction_rows"], self.contract.DOMAIN_ROWS[route])
            self.assertIn("/truth/", truth["path"])
            self.assertEqual(len(truth["sha256"]), 64)
            self.assertFalse(freeze["truth_cohort"]["read_by_freeze"])
            self.assertFalse(freeze["truth_cohort"]["read_by_audit"])
        source = CONTRACT.read_text(encoding="utf-8")
        self.assertIn("coordinate_rows_omitted", source)
        self.assertIn("candidate_parse_precedes_truth", source)
        self.assertIn("truth_reads_per_route", source)


if __name__ == "__main__":
    unittest.main()
