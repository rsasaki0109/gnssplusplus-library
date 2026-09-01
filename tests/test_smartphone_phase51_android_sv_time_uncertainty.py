#!/usr/bin/env python3
"""Static/focused checks for the Phase51 native uncertainty-floor contract.

This test intentionally opens no smartphone raw route and no truth.  It checks
the predeclared freeze, the opt-in CLI/source wiring, and the fail-closed
implementation markers that are exercised by the C++ synthetic tests.
"""

from __future__ import annotations

import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase51_android_sv_time_uncertainty_sigma_floor_freeze_v1.json"
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
LOADER = ROOT / "src/io/android_raw_gnss.cpp"
PROBLEMS = ROOT / "src/algorithms/fgo_problems.cpp"
OBSERVATION = ROOT / "include/libgnss++/core/observation.hpp"
CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
HELPER = ROOT / "include/libgnss++/algorithms/android_sv_time_uncertainty.hpp"


class Phase51AndroidSvTimeUncertaintyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        cls.app = APP.read_text(encoding="utf-8")
        cls.loader = LOADER.read_text(encoding="utf-8")
        cls.problems = PROBLEMS.read_text(encoding="utf-8")
        cls.observation = OBSERVATION.read_text(encoding="utf-8")
        cls.config = CONFIG.read_text(encoding="utf-8")
        cls.helper = HELPER.read_text(encoding="utf-8")

    def test_freeze_is_pre_code_and_pre_truth_with_four_routes(self) -> None:
        self.assertEqual(self.freeze["phase"], 51)
        self.assertEqual(self.freeze["status"], "frozen-before-code-and-truth")
        self.assertEqual(self.freeze["authority"]["base_commit"], "7c4bbfa")
        routes = self.freeze["cohort"]["route_order"]
        self.assertEqual(len(routes), 4)
        self.assertEqual(self.freeze["truth_policy"]["phase51_truth_reads_before_freeze"], 0)
        self.assertEqual(self.freeze["truth_policy"]["phase51_truth_reads_before_evaluator_manifest_seal"], 0)
        self.assertEqual(self.freeze["accuracy_gates"]["route_improvement_min_m"], 0.05)
        self.assertEqual(self.freeze["accuracy_gates"]["macro_improvement_min_m"], 0.10)
        self.assertFalse(self.freeze["accuracy_gates"]["post_truth_tuning"])
        self.assertEqual(self.freeze["objective"]["opt_in_flag"], "--native-android-sv-time-uncertainty-sigma-floor")

    def test_phase48_is_policy_pin_not_metric_input(self) -> None:
        phase48 = self.freeze["authority"]["phase48_raw_evidence_result"]
        self.assertFalse(phase48["metric_input"])
        self.assertTrue(phase48["policy_input_only"])
        self.assertIn("continuous sigma floor", self.freeze["objective"]["distinct_from_phase48"])

    def test_source_path_is_optional_and_fail_closed(self) -> None:
        self.assertIn("received_sv_time_uncertainty_m", self.observation)
        self.assertIn("has_received_sv_time_uncertainty_m", self.observation)
        self.assertIn("ReceivedSvTimeUncertaintyNanos", self.loader)
        self.assertIn("metersFromNanoseconds", self.loader)
        self.assertIn("optional field", self.loader)
        self.assertIn("negative values", self.loader)
        self.assertIn("max(existing_sigma_m, uncertainty_m)", self.helper)
        self.assertIn("uncertainty_m <= 0.0", self.helper)
        self.assertIn("use_native_android_sv_time_uncertainty_sigma_floor", self.config)
        self.assertIn("sigmaWithFloor", self.problems)
        self.assertIn("native_android_sv_time_uncertainty_factors_affected", self.problems)

    def test_cli_is_opt_in_and_spp_is_explicitly_fgo_only(self) -> None:
        flag = "--native-android-sv-time-uncertainty-sigma-floor"
        self.assertIn(flag, self.app)
        self.assertIn("requires Android raw GNSS/IMU input", self.app)
        self.assertIn('\\"spp_applied\\": false', self.app)
        # The candidate section is conditional, so legacy summaries and the
        # flag-off graph remain byte-identical apart from no new JSON field.
        self.assertIn("if (options.native_android_sv_time_uncertainty_sigma_floor)", self.app)

    def test_freeze_route_hashes_are_well_formed(self) -> None:
        route_inputs = self.freeze["raw_inputs"]["routes"]
        self.assertEqual(set(route_inputs), set(self.freeze["cohort"]["route_order"]))
        for route in self.freeze["cohort"]["route_order"]:
            entry = route_inputs[route]
            self.assertRegex(entry["device_gnss_sha256"], r"^[0-9a-f]{64}$")
            self.assertGreater(entry["device_gnss_bytes"], 0)


if __name__ == "__main__":
    unittest.main()
