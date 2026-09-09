"""Launch-free Phase118 TDCP robust-k implementation contract tests.

Only tracked source and the sealed Phase118 audit/freeze records are read.
This test never opens raw phone/base payloads, truth, MAT, or solution rows and
never launches the native solver.
"""

from __future__ import annotations

import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
FGO_HEADER = ROOT / "include/libgnss++/algorithms/fgo.hpp"
EIGEN = ROOT / "src/algorithms/fgo.cpp"
GTSAM = ROOT / "src/algorithms/fgo_gtsam_backend.cpp"
FGO_TEST = ROOT / "tests/test_fgo.cpp"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_source_parity_freeze_v1.json"


class Phase118TdcpRobustKTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = APP.read_text(encoding="utf-8")
        cls.config = CONFIG.read_text(encoding="utf-8")
        cls.fgo_header = FGO_HEADER.read_text(encoding="utf-8")
        cls.eigen = EIGEN.read_text(encoding="utf-8")
        cls.gtsam = GTSAM.read_text(encoding="utf-8")
        cls.fgo_test = FGO_TEST.read_text(encoding="utf-8")
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))

    def test_freeze_is_single_default_off_fixed_sigma_candidate(self) -> None:
        self.assertEqual(self.freeze["phase"], 118)
        candidate = self.freeze["candidate"]
        self.assertEqual(candidate["selector"], "--native-phase118-official-tdcp-huber-k")
        self.assertTrue(candidate["default_off"])
        self.assertTrue(candidate["fail_closed"])
        self.assertEqual(candidate["native_current_contract"]["tdcp_sigma_m"], 0.03)
        self.assertEqual(candidate["native_current_contract"]["tdcp_huber_threshold_sigma"], 4.0)
        self.assertFalse(candidate["native_current_contract"]["use_official_tdcp_snr_type_sigma"])
        official = candidate["official_huber_contract"]
        self.assertEqual(official["street_k"], 0.2)
        self.assertEqual(official["mix_k"], 0.2)
        self.assertEqual(official["other_type_k"], 0.5)
        self.assertFalse(self.freeze["decision"]["raw_execution_authorized"])

    def test_cli_and_config_are_default_off_and_type_keyed(self) -> None:
        selector = "--native-phase118-official-tdcp-huber-k"
        self.assertIn(selector, self.app)
        self.assertIn("native_phase118_official_tdcp_huber_k = false", self.app)
        self.assertIn("options.native_phase118_official_tdcp_huber_k = true", self.app)
        self.assertIn("config.use_official_tdcp_huber_k = true", self.app)
        self.assertIn("config.official_tdcp_setting_type", self.app)
        self.assertIn("use_official_tdcp_huber_k = false", self.config)
        self.assertIn("std::string official_tdcp_setting_type", self.config)
        self.assertIn("setting_type == \"Street\" || setting_type == \"Mix\"", self.config)
        self.assertIn('setting_type == "Highway"', self.config)
        self.assertIn("return false", self.config)

    def test_only_ordinary_tdcp_noise_call_uses_resolved_threshold(self) -> None:
        self.assertIn("ordinary_tdcp_huber_threshold_sigma", self.eigen)
        self.assertIn("ordinary_tdcp_huber_threshold_sigma", self.gtsam)
        self.assertIn("makeNoise(\n                factor.sigma_m, config.use_robust_loss,\n                ordinary_tdcp_huber_threshold_sigma)", self.gtsam)
        self.assertIn("robust_scale(raw_residual / sigma,\n                                 ordinary_tdcp_huber_threshold_sigma)", self.eigen)
        # The candidate does not mutate the fixed sigma or reuse the selector
        # for the non-ordinary single-difference Doppler/TDCP paths.
        self.assertIn("config_.tdcp_huber_threshold_sigma);", self.eigen)
        self.assertIn("config.tdcp_huber_threshold_sigma", self.gtsam)
        self.assertIn("config.tdcp_sigma_m = kNativeTdcpSigmaM", self.app)
        self.assertIn("config.use_official_tdcp_snr_type_sigma = false", self.app)

    def test_result_metadata_and_focused_mapping_regression_exist(self) -> None:
        self.assertIn("native_phase118_official_tdcp_huber_k", self.app)
        self.assertIn("official_huber_k_mapping", self.app)
        self.assertIn("fixed_sigma_m", self.app)
        self.assertIn("official_tdcp_huber_k_enabled", self.fgo_header)
        self.assertIn("OfficialTypeMappingIsOptInAndFailClosed", self.fgo_test)
        self.assertIn("use_official_tdcp_snr_type_sigma", self.eigen)
        self.assertIn("cannot be combined", self.eigen)


if __name__ == "__main__":
    unittest.main()
