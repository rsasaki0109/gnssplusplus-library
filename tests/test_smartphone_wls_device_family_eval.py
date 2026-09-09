from __future__ import annotations

import hashlib
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_wls_device_family_eval as EVAL  # noqa: E402


class SmartphoneWlsDeviceFamilyEvalTests(unittest.TestCase):
    def test_frozen_candidate_and_inventory_contract(self) -> None:
        record = EVAL._load_selection_record(EVAL.DEFAULT_SELECTION_RECORD)
        inventory = EVAL._load_frozen_inventory(
            ROOT / "output" / "smartphone-r5" / "generalization-v6" / "archive_inventory.json",
            record,
        )
        candidate = EVAL._verify_central_candidate(inventory, record)
        self.assertEqual(candidate["dataset_id"], EVAL.CANDIDATE_ID)
        self.assertEqual(candidate["phone"], "pixel7pro")
        self.assertFalse(record["sealed_data_policy"]["holdout_content_opened"])
        self.assertFalse(record["sealed_data_policy"]["holdout_truth_opened"])

    def test_selector_uses_device_metadata_only(self) -> None:
        profile = EVAL._selector_profile()
        self.assertEqual(EVAL._selector_lane("pixel7pro", profile), "native_segment_stability")
        self.assertEqual(EVAL._selector_lane("pixel5", profile), "wls_raw")
        self.assertEqual(EVAL._selector_lane("unseen-model", profile), "wls_raw")

    def test_completed_no_go_is_hash_linked_and_does_not_publish_selector(self) -> None:
        record = EVAL._load_selection_record(EVAL.DEFAULT_SELECTION_RECORD)
        post = record["post_evaluation"]
        self.assertEqual(post["promotion_decision"], "no-go-selector")
        self.assertFalse(post["gates"]["selector_profile_written"])
        self.assertFalse(
            (ROOT / "output" / "smartphone-r5" / "wls-device-family-v1" / "device_lane_selector_profile.json").exists()
        )
        report = ROOT / "output" / "smartphone-r5" / "wls-device-family-v1" / "wls_device_family_report.json"
        manifest = ROOT / "output" / "smartphone-r5" / "wls-device-family-v1" / "wls_device_family_manifest.json"
        self.assertEqual(post["report"]["sha256"], hashlib.sha256(report.read_bytes()).hexdigest())
        self.assertEqual(
            post["evaluation_manifest"]["sha256"],
            hashlib.sha256(manifest.read_bytes()).hexdigest(),
        )

    def test_aggregate_gate_uses_mean_metric_schema(self) -> None:
        def aggregate(h_p95: float, diagnostic: float) -> dict[str, object]:
            return {
                "route_count": 7,
                "mean_availability_ratio": 1.0,
                "mean_horizontal_wgs84_p50_m": 2.0,
                "mean_horizontal_wgs84_p95_m": h_p95,
                "mean_vertical_p95_abs_m": 4.0,
                "mean_kaggle_diagnostic_score_variants_m": {
                    key: diagnostic for key in EVAL.DIAGNOSTIC_KEYS
                },
            }

        candidate = aggregate(4.0, 2.0)
        reference = aggregate(5.0, 3.0)
        result = EVAL._aggregate_non_regression(candidate, reference)
        self.assertTrue(result["non_regression_passed"])

    def test_selector_gate_rejects_native_regression(self) -> None:
        def route(h_p95: float, diagnostic: float) -> dict[str, object]:
            return {
                "availability_ratio": 1.0,
                "horizontal_wgs84_m": {"p50_m": 2.0, "p95_m": h_p95},
                "vertical_p95_abs_m": 4.0,
                "kaggle_diagnostic_score_variants_m": {
                    key: diagnostic for key in EVAL.DIAGNOSTIC_KEYS
                },
            }

        def aggregate(h_p95: float, diagnostic: float) -> dict[str, object]:
            return {
                "route_count": 7,
                "mean_availability_ratio": 1.0,
                "mean_horizontal_wgs84_p50_m": 2.0,
                "mean_horizontal_wgs84_p95_m": h_p95,
                "mean_vertical_p95_abs_m": 4.0,
                "mean_kaggle_diagnostic_score_variants_m": {
                    key: diagnostic for key in EVAL.DIAGNOSTIC_KEYS
                },
            }

        gate = EVAL._selector_gate(
            {
                "selector": aggregate(6.0, 4.0),
                "wls_raw": aggregate(5.0, 3.0),
                "existing_six_selector": aggregate(5.0, 3.0),
                "existing_six_wls_raw": aggregate(5.0, 3.0),
            },
            {
                "native_segment_stability": route(6.0, 4.0),
                "wls_raw": route(5.0, 3.0),
            },
        )
        self.assertEqual(gate["promotion_decision"], "no-go-selector")
        self.assertFalse(gate["new_pixel7pro_reproduction"]["non_regression_passed"])


if __name__ == "__main__":
    unittest.main()
