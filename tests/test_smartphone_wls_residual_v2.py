from __future__ import annotations

import copy
import json
import sys
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_wls_residual_v2_eval as evaluation  # noqa: E402


class SmartphoneWlsResidualV2Tests(unittest.TestCase):
    def test_selection_freezes_median5_and_fresh_non_pixel_validation(self) -> None:
        record = evaluation._load_selection(evaluation.DEFAULT_SELECTION_RECORD)
        self.assertEqual(record["fixed_candidate"]["candidate_id"], "median5_shift_0")
        self.assertFalse(record["fixed_candidate"]["candidate_search_after_freeze"])
        selected = record["metadata_selection"]["selected_validation"]
        self.assertEqual(selected["dataset_id"], evaluation.NEW_VALIDATION_ID)
        self.assertEqual(selected["phone"], "sm-a505g")
        self.assertTrue(record["archive"]["central_directory_only"])
        self.assertFalse(record["archive"]["member_content_read_at_selection"])
        self.assertTrue(record["sealed_data_policy"]["next_holdout_materialization_forbidden"])

    def test_inventory_member_fixture_maps_route_nav_without_payload_access(self) -> None:
        route_members = {"device_gnss.csv": {"name": "device_gnss.csv"}}
        inventory_record = {
            "central_directory_files": route_members,
            "central_directory_broadcast_nav": {"name": "brdc.nav"},
        }
        self.assertEqual(
            evaluation._central_directory_members(inventory_record),
            {
                "device_gnss.csv": {"name": "device_gnss.csv"},
                "brdc.nav": {"name": "brdc.nav"},
            },
        )

    def _scores(self) -> tuple[dict[str, object], dict[str, object]]:
        diagnostics = {key: 3.0 for key in evaluation.DIAGNOSTIC_KEYS}
        baseline = {
            "horizontal_wgs84_m": {"p50_m": 3.0, "p95_m": 8.0},
            "vertical_p95_abs_m": 12.0,
            "availability_ratio": 1.0,
            "truth_coverage_ratio": 1.0,
            "kaggle_diagnostic_score_variants_m": diagnostics,
        }
        candidate = copy.deepcopy(baseline)
        candidate["horizontal_wgs84_m"] = {"p50_m": 3.005, "p95_m": 7.9}
        candidate["vertical_p95_abs_m"] = 14.0
        candidate["kaggle_diagnostic_score_variants_m"] = {
            key: 3.0000005 for key in evaluation.DIAGNOSTIC_KEYS
        }
        return candidate, baseline

    def _gate_record(self) -> dict[str, object]:
        return {
            "diagnostics": {"maximum_allowed_regression_m": 1.0e-6},
            "horizontal_p50": {"maximum_allowed_regression_m": 0.01},
            "vertical_p95": {"major_regression_tolerance_m": 5.0},
        }

    def test_public_gate_allows_frozen_tolerances(self) -> None:
        candidate, baseline = self._scores()
        result = evaluation._v2_gate(
            candidate,
            baseline,
            {"thresholds": {"vertical_p95_max": 45.0}},
            self._gate_record(),
        )
        self.assertTrue(result["passed"])
        self.assertTrue(all(item["passed"] for item in result["diagnostics"].values()))
        self.assertTrue(result["horizontal"]["h_p95_strict_improvement"])
        self.assertTrue(result["horizontal"]["h_p50_passed"])

    def test_public_gate_rejects_diagnostic_and_horizontal_regressions(self) -> None:
        candidate, baseline = self._scores()
        candidate["kaggle_diagnostic_score_variants_m"][evaluation.DIAGNOSTIC_KEYS[0]] = 3.000002
        candidate["horizontal_wgs84_m"] = {"p50_m": 3.011, "p95_m": 8.0}
        result = evaluation._v2_gate(
            candidate,
            baseline,
            {"thresholds": {"vertical_p95_max": 45.0}},
            self._gate_record(),
        )
        self.assertFalse(result["passed"])
        self.assertIn(f"{evaluation.DIAGNOSTIC_KEYS[0]}_regression", result["failures"])
        self.assertIn("h_p95_not_strictly_improved", result["failures"])
        self.assertIn("h_p50_regression_over_v2_tolerance", result["failures"])

    def test_public_gate_rejects_vertical_profile_or_major_regression(self) -> None:
        candidate, baseline = self._scores()
        candidate["vertical_p95_abs_m"] = 50.0
        result = evaluation._v2_gate(
            candidate,
            baseline,
            {"thresholds": {"vertical_p95_max": 45.0}},
            self._gate_record(),
        )
        self.assertFalse(result["passed"])
        self.assertIn("v_p95_profile_threshold", result["failures"])
        self.assertIn("v_p95_major_regression", result["failures"])

    def test_v21_profile_promotes_only_development_wls_postprocess(self) -> None:
        profile = json.loads(
            (ROOT / "configs" / "benchmarks" / "smartphone_r5_gsdc2023.json").read_text(
                encoding="utf-8"
            )
        )
        lane = profile["development_only_submission_lane"]["wls_residual_postprocess"]
        self.assertEqual(lane["status"], "promoted-development-only")
        self.assertEqual(lane["candidate"], "median5_shift_0")
        self.assertFalse(lane["production_default"])
        release = profile["wls_residual_release_research_v2_1"]
        self.assertEqual(
            release["promotion_decision"],
            "promote-development-only-wls-branch-postprocess",
        )
        self.assertFalse(release["next_holdout_materialized"])


if __name__ == "__main__":
    unittest.main()
