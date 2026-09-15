"""Static contract tests for the Phase91 implementation (no raw execution)."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import unittest

from frozen_contract import require_source_marker


ROOT = Path(__file__).resolve().parents[1]
FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase91_source_clock_c0d_gnss_first_in_memory_raw_drift_d_initializer_freeze_v1.json"
)
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
BACKEND = ROOT / "src/algorithms/fgo_gtsam_backend.cpp"
CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
INITIALIZER = ROOT / "include/libgnss++/algorithms/source_clock_c0d_initializer.hpp"


class Phase91ImplementationTests(unittest.TestCase):
    def test_authority_freeze_is_unchanged_and_execution_is_not_authorized(self) -> None:
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(freeze["phase"], 91)
        self.assertEqual(freeze["status"], "frozen-before-implementation")
        self.assertFalse(freeze["decision_boundary"]["execution_authorized"])
        self.assertEqual(freeze["decision_boundary"]["candidate_count"], 1)
        self.assertEqual(
            hashlib.sha256(FREEZE.read_bytes()).hexdigest(),
            "e218864172b789d67ce5961ebfe67457c267960be8da5f27f5b6c017ebba906b",
        )

    def test_opt_in_selector_and_exact_dependencies_are_present(self) -> None:
        source = APP.read_text(encoding="utf-8")
        selector = "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer"
        require_source_marker(
            "Phase91 frozen source marker",
            source,
            "gnss_first_config.use_native_source_clock_c0d_raw_drift_d_initializer =\n"
            "            false;",
        )
        self.assertIn(selector, source)
        self.assertIn("validatePhase91GnssFirstHandoff", source)
        self.assertIn("gnss_first_problem = problem", source)
        self.assertIn("android_gnss.epoch_utc_time_millis", source)
        self.assertIn("--native-gnss-first-velocity-only-handoff", source)
        self.assertIn("--native-direct-doppler-wls-handoff", source)

    def test_initializer_is_main_graph_only_and_fail_closed(self) -> None:
        backend = BACKEND.read_text(encoding="utf-8")
        config = CONFIG.read_text(encoding="utf-8")
        helper = INITIALIZER.read_text(encoding="utf-8")
        self.assertIn("use_native_source_clock_c0d_raw_drift_d_initializer", config)
        self.assertIn("epoch.receiver_clock_drift_mps", backend)
        self.assertIn("validateAndCopyRawDriftD", backend)
        self.assertIn("result.diagnostics.converged = false", backend)
        self.assertIn("No value is inferred, held, interpolated", helper)
        self.assertIn("strictEpochTimeEqual", helper)
        self.assertIn("raw UTC epoch keys are not strictly ordered and unique", helper)
        self.assertIn(
            "native_source_clock_c0d_raw_drift_d_initializer_failure", backend
        )

    def test_source_c0d_and_direct_quality_contracts_are_not_rewritten(self) -> None:
        source = APP.read_text(encoding="utf-8")
        backend = BACKEND.read_text(encoding="utf-8")
        self.assertIn("(c2-c1)-((d1+d2)*dt/(2*C_LIGHT))", source)
        self.assertIn("[-1,+1,-dt/(2*C_LIGHT),-dt/(2*C_LIGHT)]", source)
        self.assertIn("SourceClockC0DFactor", backend)
        self.assertIn("0.1 / libgnss::constants::SPEED_OF_LIGHT", source)
        self.assertIn(
            "config.use_native_source_clock_c0d_raw_drift_d_initializer", backend
        )
        self.assertIn("raw_drift_d_initializer[i]", backend)


if __name__ == "__main__":
    unittest.main()
