"""Launch-free tests for the Phase112 truth-only accuracy boundary."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase112_main_output_offset_accuracy.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_accuracy_manifest_v1.json"


def load_contract():
    spec = importlib.util.spec_from_file_location("phase112_accuracy_contract_test", CONTRACT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase112AccuracyContractTests(unittest.TestCase):
    def test_pre_truth_has_no_solver_raw_or_truth_activity(self) -> None:
        contract = load_contract()
        pre = contract.verify_pre_truth()
        for key in (
            "native_solver_invocations", "raw_reads", "base_reads", "truth_reads",
            "candidate_solution_reads", "candidate_coordinate_interpretations",
            "accuracy_calculations", "mat_precomputed_phone_coordinate_pdc_reads",
            "kaggle_or_token_access",
        ):
            self.assertEqual(pre[key], 0, key)
        self.assertFalse(pre["native_rerun"])

    def test_truth_manifest_pins_phase112_structural_solution_seals(self) -> None:
        contract = load_contract()
        manifest = contract.verify_manifest()
        self.assertEqual(manifest["boundary"]["structural_commit"], contract.STRUCTURAL_COMMIT)
        self.assertTrue(manifest["candidate"]["metadata_hash_seal_precedes_truth"])
        self.assertFalse(manifest["candidate"]["solution_publication"])
        self.assertEqual(manifest["truth_evaluator_boundary"]["truth_reads_per_route"], 1)

    def test_metric_and_strict_gate_are_phase108_compatible(self) -> None:
        contract = load_contract()
        freeze = contract.verify_phase108_freeze()
        metric = freeze["metric_contract"]
        self.assertEqual(metric["earth_radius_m"], 6371008.8)
        self.assertEqual(metric["route_scalar"], "(P50 + P95) / 2 in metres")
        self.assertEqual(metric["macro"], "unweighted arithmetic mean over exactly MTV-A then LAX-T")
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(manifest["metric_contract"]["strict_0_782_threshold_m"], 0.782)

    def test_solution_and_truth_paths_are_metadata_only_until_evaluator(self) -> None:
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        self.assertFalse(manifest["read_accounting_before_authorization"]["truth_reads"])
        self.assertFalse(manifest["read_accounting_before_authorization"]["candidate_solution_reads"])
        self.assertFalse(manifest["truth_cohort"]["read_by_solver"])
        self.assertFalse(manifest["truth_cohort"]["read_by_manifest"])
        self.assertTrue(manifest["truth_cohort"]["read_by_evaluator_only"])


if __name__ == "__main__":
    unittest.main()
