"""Launch-free Phase138 affine-TDCP anchor-range contract tests.

Only tracked source and the sealed Phase138 design record are read.  These
tests do not materialize raw observations, navigation, base, truth, MAT/PDC,
precomputed coordinates, or solution rows, and never launch a solver.
"""

from __future__ import annotations

import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
RESULT = ROOT / "include/libgnss++/algorithms/fgo.hpp"
TDCP_CONTRACT = ROOT / "include/libgnss++/algorithms/tdcp_contract.hpp"
EIGEN = ROOT / "src/algorithms/fgo.cpp"
INTERNAL = ROOT / "src/algorithms/fgo_gtsam_internal.hpp"
BACKEND = ROOT / "src/algorithms/fgo_gtsam_backend.cpp"
TESTS_CMAKE = ROOT / "tests/CMakeLists.txt"
FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase138_phase135_affine_catastrophic_error_forensic_freeze_v1.json"
)


class Phase138AffineTdcpAnchorRangeConstantTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = APP.read_text(encoding="utf-8")
        cls.config = CONFIG.read_text(encoding="utf-8")
        cls.result = RESULT.read_text(encoding="utf-8")
        cls.tdcp_contract = TDCP_CONTRACT.read_text(encoding="utf-8")
        cls.eigen = EIGEN.read_text(encoding="utf-8")
        cls.internal = INTERNAL.read_text(encoding="utf-8")
        cls.backend = BACKEND.read_text(encoding="utf-8")
        cls.tests_cmake = TESTS_CMAKE.read_text(encoding="utf-8")
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))

    def test_freeze_is_one_default_off_candidate(self) -> None:
        self.assertEqual(self.freeze["phase"], 138)
        decision = self.freeze["decision"]
        self.assertEqual(decision["candidate_count"], 1)
        self.assertEqual(
            decision["candidate_id"],
            "phase138-affine-tdcp-anchor-range-constant-v1",
        )
        candidate = self.freeze["candidate"]
        self.assertEqual(
            candidate["selector"],
            "--native-phase138-affine-tdcp-anchor-range-constant",
        )
        self.assertTrue(candidate["default_off"])
        self.assertTrue(candidate["requires_phase135_affine"])
        self.assertFalse(decision["implementation_authorized"])
        self.assertFalse(decision["raw_execution_authorized"])
        self.assertFalse(decision["truth_evaluation_authorized"])

    def test_measurement_equation_and_fixed_invariants_are_pinned(self) -> None:
        contract = self.freeze["candidate"]["measurement_contract"]
        self.assertEqual(
            contract["new_input"],
            "tdcp_phase138_i = tdcp_native_i - (rho_{i+1}^0-rho_i^0)",
        )
        self.assertIn("source satellite state", contract["rho_definition"])
        fixed = self.freeze["candidate"]["fixed"]
        self.assertEqual(fixed["tdcp_sigma_m"], 0.03)
        self.assertEqual(fixed["tdcp_huber_k"], 0.5)
        self.assertIn("same-signal adjacent pair", fixed["pairing_and_gates"])
        self.assertIn("no second correction", fixed["sagnac"])

    def test_helper_is_finite_fail_closed_and_backend_uses_both_endpoints(self) -> None:
        self.assertIn(
            "applyPhase138AffineTdcpAnchorRangeConstant", self.tdcp_contract
        )
        self.assertIn(
            "tdcp_native - (rho_current_initial-rho_previous_initial)",
            self.tdcp_contract,
        )
        self.assertIn("!std::isfinite(previous_initial_range_m)", self.tdcp_contract)
        self.assertIn("!std::isfinite(current_initial_range_m)", self.tdcp_contract)
        self.assertIn("phase138_tdcp_range_constants_validated", self.backend)
        self.assertIn("phase138_tdcp_measurements_adjusted", self.backend)
        self.assertIn("factor.current_source_satellite_position_ecef", self.backend)
        self.assertIn("tdcp_measurement_m", self.backend)
        self.assertIn("phase138_adjusted_exactly_once", self.result)
        self.assertIn("phase138_factor_count_unchanged", self.result)

    def test_cli_config_and_summary_are_default_off_and_dependency_guarded(self) -> None:
        selector = "--native-phase138-affine-tdcp-anchor-range-constant"
        self.assertIn(selector, self.app)
        self.assertIn(
            "native_phase138_affine_tdcp_anchor_range_constant = false", self.app
        )
        self.assertIn(
            "options.native_phase138_affine_tdcp_anchor_range_constant = true",
            self.app,
        )
        self.assertIn(
            "use_native_phase138_affine_tdcp_anchor_range_constant = false",
            self.config,
        )
        self.assertIn(
            "config.use_native_phase138_affine_tdcp_anchor_range_constant = true",
            self.app,
        )
        self.assertIn(
            '\\"phase138_affine_tdcp_anchor_range_constant\\": {', self.app
        )
        self.assertIn(
            "requires --native-phase135-official-affine-",
            self.app,
        )
        self.assertIn("measurement-family", self.app)
        self.assertIn(
            "Phase138 affine TDCP anchor-range correction requires the ", self.eigen
        )
        self.assertIn("phase138_requested", self.backend)

    def test_selector_off_preserves_phase135_and_is_registered(self) -> None:
        self.assertIn(
            "phase138_affine_tdcp_anchor_range_constant_enabled", self.result
        )
        self.assertIn(
            'std::string phase138_measurement_equation = "disabled"', self.result
        )
        self.assertIn("Phase135TdcpAffinePointFactor", self.internal)
        self.assertIn("Phase135TdcpAffineVectorFactor", self.internal)
        self.assertIn(
            "python_smartphone_phase138_affine_tdcp_anchor_range_constant_tests",
            self.tests_cmake,
        )


if __name__ == "__main__":
    unittest.main()
