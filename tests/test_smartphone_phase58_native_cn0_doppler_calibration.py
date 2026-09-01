#!/usr/bin/env python3
"""Static/focused checks for the Phase58 native C/N0 calibration contract.

This module reads only source and the pre-code implementation freeze.  It does
not open any smartphone raw file, navigation, truth, or Phase58 raw audit
payload.
"""

from __future__ import annotations

import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase58_native_cn0_doppler_calibration_freeze_v1.json"
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
PROBLEMS = ROOT / "src/algorithms/fgo_problems.cpp"
HELPER = ROOT / "include/libgnss++/algorithms/cn0_doppler_calibration.hpp"
FGO = ROOT / "include/libgnss++/algorithms/fgo.hpp"


class Phase58NativeCn0DopplerCalibrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        cls.app = APP.read_text(encoding="utf-8")
        cls.config = CONFIG.read_text(encoding="utf-8")
        cls.problems = PROBLEMS.read_text(encoding="utf-8")
        cls.helper = HELPER.read_text(encoding="utf-8")
        cls.fgo = FGO.read_text(encoding="utf-8")

    def test_freeze_precedes_code_and_pins_audit_and_alpha(self) -> None:
        self.assertEqual(self.freeze["phase"], 58)
        self.assertEqual(
            self.freeze["status"],
            "frozen-before-native-code-and-structural-matrix",
        )
        self.assertEqual(self.freeze["authority"]["base_commit"], "8da76f0")
        self.assertEqual(
            self.freeze["fixed_model"]["alpha_mps_at_reference"],
            0.7586783350728457,
        )
        self.assertEqual(self.freeze["cohort"]["route_order"].__len__(), 4)
        self.assertEqual(self.freeze["structural_matrix"]["candidate_runs_per_route"], 2)
        self.assertEqual(self.freeze["structural_matrix"]["control_runs_per_route"], 1)

    def test_formula_is_fixed_and_does_not_reimplement_p85_path(self) -> None:
        self.assertIn("kAlphaMpsAtReference = 0.7586783350728457", self.helper)
        self.assertIn("kReferenceCn0DbHz = 40.0", self.helper)
        self.assertIn("std::pow", self.helper)
        self.assertIn("max(existing_doppler_sigma_mps,model_sigma_mps)", self.app)
        self.assertIn("existing_p85_over_12_path_untouched", self.app)
        self.assertIn("p85 scale", self.helper)
        self.assertIn("/12", self.helper)
        self.assertIn("use_native_cn0_doppler_calibration", self.config)
        self.assertIn("cn0_doppler_calibration::sigmaWithFloor", self.problems)

    def test_scope_is_fgo_doppler_only_and_flag_is_opt_in(self) -> None:
        flag = "--native-cn0-doppler-calibration"
        self.assertIn(flag, self.app)
        self.assertIn("requires Android raw GNSS/IMU input", self.app)
        self.assertIn("spp_applied", self.app)
        self.assertIn("<< \"    \\\"spp_applied\\\": false", self.app)
        self.assertIn("tdcp_applied", self.app)
        self.assertIn("single_difference_doppler_applied", self.app)
        self.assertIn("candidate_rows", self.app)
        self.assertIn("model_sigma_p95_mps", self.app)
        self.assertIn("native_cn0_doppler_calibration_factors_affected", self.fgo)

    def test_missing_cn0_falls_back_and_no_cap_is_declared(self) -> None:
        self.assertIn("modelSigmaMps(cn0_dbhz)", self.helper)
        self.assertIn("return existing_sigma_mps;", self.helper)
        self.assertIn("No upper clip or coefficient is applied", self.helper)
        self.assertFalse(self.freeze["fixed_model"]["upper_clip"])
        self.assertFalse(self.freeze["fixed_model"]["spp_applied"])
        self.assertFalse(self.freeze["fixed_model"]["tdcp_applied"])

    def test_four_route_raw_hashes_and_zero_truth_stage(self) -> None:
        routes = self.freeze["cohort"]["route_order"]
        self.assertEqual(set(routes), set(self.freeze["cohort"]["input_hashes"]))
        for route in routes:
            for key in ("device_gnss.csv", "device_imu.csv", "brdc.nav"):
                self.assertRegex(
                    self.freeze["cohort"]["input_hashes"][route][key],
                    r"^[0-9a-f]{64}$",
                )
        policy = self.freeze["input_policy"]
        self.assertEqual(policy["truth_reads"], 0)
        self.assertEqual(policy["mat_reads_or_generated"], 0)
        self.assertEqual(policy["kaggle_or_token_access"], 0)


if __name__ == "__main__":
    unittest.main()
