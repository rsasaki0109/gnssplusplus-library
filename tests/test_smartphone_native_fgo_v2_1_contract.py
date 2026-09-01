from __future__ import annotations

import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class NativeFgoV21ContractTests(unittest.TestCase):
    def test_freeze_is_truth_free_and_preserves_v1_factor_contract(self) -> None:
        record = json.loads(
            (ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v2_1_preserve_v1_freeze.json").read_text()
        )
        self.assertEqual(record["status"], "frozen-before-v2-1-truth-scoring")
        self.assertFalse(record["truth_policy"]["validation_materialization_or_truth_open"])
        self.assertFalse(record["truth_policy"]["holdout_materialization_or_truth_open"])
        self.assertEqual(
            record["graph_contract"]["preserved_v1_factors"],
            [
                "undifferenced pseudorange",
                "ordinary time-differenced carrier phase",
                "position motion between consecutive epochs",
                "clock motion between consecutive epochs",
                "existing Huber robust loss",
            ],
        )
        self.assertEqual(record["numeric_parameters"]["max_iterations"], 8)
        self.assertEqual(record["observable_offset_estimator"]["grid_seconds"]["step"], 0.01)

    def test_backend_and_entrypoint_have_no_base_and_insert_all_required_classes(self) -> None:
        backend = (ROOT / "src/algorithms/fgo_gtsam_backend.cpp").read_text()
        internal = (ROOT / "src/algorithms/fgo_gtsam_internal.hpp").read_text()
        entrypoint = (ROOT / "apps/native/gnss_fgo_imu_v21.cpp").read_text()
        cmake = (ROOT / "apps/CMakeLists.txt").read_text()
        self.assertIn("TimeDifferencedCarrierFactorArm", internal)
        self.assertIn("ordinary_tdcp_inserted", backend)
        self.assertIn("config.tdcp_huber_threshold_sigma", backend)
        self.assertIn("clock_motion_sigma_m", backend)
        self.assertIn("gtsam::gnss::C_LIGHT", backend)
        self.assertIn("gtsam::CombinedImuFactor", backend)
        self.assertIn("config.use_double_difference_factors = false", entrypoint)
        self.assertIn("config.use_tdcp_factors = true", entrypoint)
        self.assertIn("config.use_motion_factors = true", entrypoint)
        self.assertIn("estimateOffset", entrypoint)
        self.assertIn("gnss_fgo_imu_v21", cmake)


if __name__ == "__main__":
    unittest.main()
