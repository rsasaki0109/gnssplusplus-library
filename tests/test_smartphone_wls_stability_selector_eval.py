from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_wls_stability_selector_eval as EVAL  # noqa: E402


class SmartphoneWlsStabilitySelectorEvalTests(unittest.TestCase):
    def test_selection_is_frozen_and_excludes_all_used_routes(self) -> None:
        record = EVAL._load_selection_record(EVAL.DEFAULT_SELECTION_RECORD)
        inventory = EVAL._load_frozen_inventory(EVAL.DEFAULT_INVENTORY, record)
        candidate = EVAL._verify_candidate(inventory, record)
        self.assertEqual(candidate["dataset_id"], EVAL.NEW_VALIDATION_ID)
        self.assertTrue(set(EVAL.USED_IDS).issubset(record["previously_used_ids_excluded"]))
        self.assertFalse(record["sealed_data_policy"]["holdout_content_opened"])
        self.assertFalse(record["sealed_data_policy"]["holdout_truth_opened"])

    def test_completed_promotion_is_hash_linked_and_truth_free_at_runtime(self) -> None:
        root = ROOT / "output" / "smartphone-r5" / "wls-stability-selector-v1"
        report_path = root / "wls_stability_selector_report.json"
        manifest_path = root / "wls_stability_selector_manifest.json"
        record = EVAL._load_selection_record(EVAL.DEFAULT_SELECTION_RECORD)
        post = record["post_evaluation"]
        report = json.loads(report_path.read_text(encoding="utf-8"))
        self.assertEqual(post["promotion_decision"], "promote-development-only-recommended-selector")
        self.assertEqual(report["gates"]["promotion_decision"], post["promotion_decision"])
        self.assertTrue(report["truth_free_contract"]["new_route_truth_parsed_once_after_generation"])
        self.assertFalse(report["truth_free_contract"]["truth_dependent_runtime_selection"])
        self.assertFalse(report["truth_free_contract"]["selector_runtime_uses_device_model"])
        self.assertEqual(
            post["report"]["sha256"], hashlib.sha256(report_path.read_bytes()).hexdigest()
        )
        self.assertEqual(
            post["evaluation_manifest"]["sha256"],
            hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        )

    def test_known_seven_selector_is_strictly_better_horizontally(self) -> None:
        report = json.loads(
            (
                ROOT
                / "output"
                / "smartphone-r5"
                / "wls-stability-selector-v1"
                / "wls_stability_selector_report.json"
            ).read_text(encoding="utf-8")
        )
        aggregates = report["known_seven_route_aggregates"]
        selected = aggregates["selector"]
        for reference_name in ("native_only", "wls_only"):
            reference = aggregates[reference_name]
            self.assertLess(
                selected["mean_horizontal_wgs84_p50_m"],
                reference["mean_horizontal_wgs84_p50_m"],
            )
            self.assertLess(
                selected["mean_horizontal_wgs84_p95_m"],
                reference["mean_horizontal_wgs84_p95_m"],
            )
            for key in EVAL.DIAGNOSTIC_KEYS:
                self.assertLess(
                    selected["mean_kaggle_diagnostic_score_variants_m"][key],
                    reference["mean_kaggle_diagnostic_score_variants_m"][key],
                )
        self.assertTrue(report["gates"]["known_seven_route"]["passed"])

    def test_new_route_selects_wls_when_native_segment_is_unstable(self) -> None:
        report = json.loads(
            (
                ROOT
                / "output"
                / "smartphone-r5"
                / "wls-stability-selector-v1"
                / "wls_stability_selector_report.json"
            ).read_text(encoding="utf-8")
        )
        self.assertEqual(report["new_route_scores"]["selected_lane"], "wls_raw")
        self.assertFalse(report["route"]["native"]["stability"]["population"]["stable_segment_count"])
        self.assertTrue(report["gates"]["new_validation"]["passed"])
        self.assertTrue(report["gates"]["known_seven_route"]["passed"])
        self.assertFalse(report["truth_free_contract"]["holdout_content_opened"])


if __name__ == "__main__":
    unittest.main()
