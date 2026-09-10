"""Launch-free Phase120 TDCP equation-normalization contract tests.

Only tracked source and the sealed Phase120 audit/freeze records are read.
This test does not open raw phone/base payloads, truth, MAT, coordinates, or
solutions and never launches the native solver.
"""

from __future__ import annotations

import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
FGO_HEADER = ROOT / "include/libgnss++/algorithms/fgo.hpp"
CONTRACT = ROOT / "include/libgnss++/algorithms/tdcp_contract.hpp"
INTERNAL = ROOT / "src/algorithms/fgo_internal.hpp"
PROBLEMS = ROOT / "src/algorithms/fgo_problems.cpp"
EIGEN = ROOT / "src/algorithms/fgo.cpp"
GTSAM = ROOT / "src/algorithms/fgo_gtsam_backend.cpp"
FGO_TEST = ROOT / "tests/test_fgo.cpp"
FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase120_tdcp_equation_normalization_freeze_v1.json"
)


class Phase120TdcpEquationNormalizationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = APP.read_text(encoding="utf-8")
        cls.config = CONFIG.read_text(encoding="utf-8")
        cls.fgo_header = FGO_HEADER.read_text(encoding="utf-8")
        cls.contract = CONTRACT.read_text(encoding="utf-8")
        cls.internal = INTERNAL.read_text(encoding="utf-8")
        cls.problems = PROBLEMS.read_text(encoding="utf-8")
        cls.eigen = EIGEN.read_text(encoding="utf-8")
        cls.gtsam = GTSAM.read_text(encoding="utf-8")
        cls.fgo_test = FGO_TEST.read_text(encoding="utf-8")
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))

    def test_freeze_is_single_default_off_candidate(self) -> None:
        self.assertEqual(self.freeze["phase"], 120)
        self.assertEqual(self.freeze["candidate"]["candidate_count"], 1)
        self.assertEqual(
            self.freeze["candidate"]["id"],
            "phase120-official-tdcp-resl-atmosphere-cancellation-v1",
        )
        self.assertTrue(self.freeze["candidate"]["default_off"])
        self.assertFalse(self.freeze["authorization"]["raw_execution_authorized"])
        self.assertFalse(self.freeze["authorization"]["truth_evaluation_authorized"])
        self.assertEqual(
            self.freeze["candidate"]["measurement_contract"][
                "source_parity_measurement_m"
            ],
            "carrier_phase_cycles * retained_wavelength_m + satellite_clock_m",
        )

    def test_cli_and_config_are_default_off_and_phase118_composable(self) -> None:
        selector = "--native-phase120-official-tdcp-resl-atmosphere-cancellation"
        self.assertIn(selector, self.app)
        self.assertIn(
            "native_phase120_official_tdcp_resl_atmosphere_cancellation = false",
            self.app,
        )
        self.assertIn(
            "options.native_phase120_official_tdcp_resl_atmosphere_cancellation = true",
            self.app,
        )
        self.assertIn(
            "use_official_tdcp_resl_atmosphere_cancellation = false",
            self.config,
        )
        self.assertIn(
            "config.use_official_tdcp_resl_atmosphere_cancellation = true",
            self.app,
        )
        self.assertIn(
            "requires --native-phase118-official-tdcp-huber-k",
            self.app,
        )
        self.assertIn("config.use_official_tdcp_huber_k = true", self.app)
        self.assertIn("config.use_official_tdcp_resl_atmosphere_cancellation = true", self.app)

    def test_only_ordinary_tdcp_gets_separate_source_measurement(self) -> None:
        self.assertIn("double tdcp_carrier_m = 0.0", self.internal)
        self.assertIn("carrier.tdcp_carrier_m = tdcp_carrier", self.problems)
        self.assertIn(
            "current.tdcp_carrier_m - previous.tdcp_carrier_m", self.problems
        )
        self.assertIn(
            "legacy_gate_delta_carrier_m =", self.problems
        )
        self.assertIn(
            "legacy_gate_delta_carrier_m, delta_code_m, max_gap",
            self.problems,
        )
        self.assertIn("carrier.corrected_carrier_m = corrected_carrier", self.problems)
        self.assertIn(
            "ordinaryTdcpCarrierMeters(",
            self.contract,
        )
        self.assertIn("config_.use_official_tdcp_resl_atmosphere_cancellation", self.problems)
        self.assertIn("config.use_official_tdcp_resl_atmosphere_cancellation", self.internal)
        # DD and standalone carrier consumers continue to use corrected_carrier_m.
        self.assertIn("satellite->corrected_carrier_m", self.problems)
        self.assertIn("carrier_observation.corrected_carrier_m", self.problems)

    def test_core_terms_fail_closed_and_atmosphere_is_ignored_only_when_on(self) -> None:
        self.assertIn("!std::isfinite(raw_carrier_m)", self.contract)
        self.assertIn("!std::isfinite(satellite_clock_m)", self.contract)
        self.assertIn("const double measurement_m = raw_carrier_m + satellite_clock_m", self.contract)
        self.assertIn("std::isfinite(measurement_m)", self.contract)
        self.assertIn("!std::isfinite(ionosphere_delay_m)", self.contract)
        self.assertIn("EXPECT_DOUBLE_EQ(source_parity, raw_carrier_m + satellite_clock_m)", self.fgo_test)
        self.assertIn("DynamicSigmaCompositionFailsClosedAtOptimizerBoundary", self.fgo_test)
        self.assertIn("Phase120 official resL TDCP normalization", self.eigen)
        self.assertIn("Phase120 official resL TDCP normalization", self.gtsam)
        self.assertIn("Phase117 dynamic TDCP sigma", self.eigen)
        self.assertIn("Phase117 dynamic TDCP sigma", self.gtsam)

    def test_diagnostic_metadata_exposes_selector_without_solution_lane(self) -> None:
        self.assertIn(
            "official_tdcp_resl_atmosphere_cancellation_enabled", self.fgo_header
        )
        self.assertIn(
            "official_resl_atmosphere_cancellation_enabled", self.app
        )
        self.assertIn("ordinary_measurement_m", self.app)
        self.assertIn('\\"truth_used\\": false', self.app)
        self.assertIn('\\"base_factors\\": false', self.app)


if __name__ == "__main__":
    unittest.main()
