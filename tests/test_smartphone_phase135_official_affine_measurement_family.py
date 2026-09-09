"""Launch-free Phase135 official affine measurement-family contract tests.

This module reads only tracked source and the sealed Phase135 design record.
It does not open raw GNSS/IMU/navigation/base payloads, solution rows, truth,
MAT/PDC/precomputed coordinates, or Kaggle artifacts, and it never launches
the native solver.
"""

from __future__ import annotations

import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
RESULT = ROOT / "include/libgnss++/algorithms/fgo.hpp"
EIGEN = ROOT / "src/algorithms/fgo.cpp"
INTERNAL = ROOT / "src/algorithms/fgo_gtsam_internal.hpp"
BACKEND = ROOT / "src/algorithms/fgo_gtsam_backend.cpp"
PROBLEMS = ROOT / "src/algorithms/fgo_problems.cpp"
TESTS_CMAKE = ROOT / "tests/CMakeLists.txt"
FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase135_post_base_compound_port_triage_freeze_v1.json"
)


class Phase135OfficialAffineMeasurementFamilyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = APP.read_text(encoding="utf-8")
        cls.config = CONFIG.read_text(encoding="utf-8")
        cls.result = RESULT.read_text(encoding="utf-8")
        cls.eigen = EIGEN.read_text(encoding="utf-8")
        cls.internal = INTERNAL.read_text(encoding="utf-8")
        cls.backend = BACKEND.read_text(encoding="utf-8")
        cls.problems = PROBLEMS.read_text(encoding="utf-8")
        cls.tests_cmake = TESTS_CMAKE.read_text(encoding="utf-8")
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))

    def test_freeze_is_one_default_off_implementation_candidate(self) -> None:
        candidate = self.freeze["candidate"]
        self.assertEqual(self.freeze["phase"], 135)
        self.assertEqual(candidate["count"], 1)
        self.assertEqual(
            candidate["id"],
            "phase135-official-affine-measurement-family-sagnac-key-order-transactional-v1",
        )
        self.assertEqual(
            candidate["selector"],
            "--native-phase135-official-affine-measurement-family",
        )
        self.assertTrue(candidate["source_backed"])
        self.assertTrue(candidate["default_off"])
        self.assertFalse(candidate["partial_selector_or_family_allowed"])
        self.assertFalse(candidate["raw_execution_authorized"])
        self.assertFalse(candidate["truth_evaluation_authorized"])

    def test_equations_and_key_order_are_source_locked(self) -> None:
        equations = self.freeze["candidate"]["equations"]
        self.assertEqual(equations["pseudorange"]["keys"], ["X_i", "C_i"])
        self.assertEqual(equations["doppler"]["keys"], ["V_i", "D_i"])
        self.assertEqual(
            equations["ordinary_tdcp_xxcc"]["keys"],
            ["X_i", "X_i+1", "C_i", "C_i+1"],
        )
        self.assertIn("Phase135PseudorangeAffinePointFactor", self.internal)
        self.assertIn("Phase135DopplerAffineFactor", self.internal)
        self.assertIn("Phase135TdcpAffinePointFactor", self.internal)
        self.assertIn("Phase135Pose3Point3FactorPX", self.internal)
        self.assertIn("sourceClockComponentJacobian", self.internal)
        self.assertIn("affinePositionKey", self.internal)
        self.assertIn("clockKey", self.backend)
        self.assertIn("velocityKey", self.backend)
        self.assertIn("dopplerClockDriftKey", self.backend)
        self.assertIn("constants::OMEGA_E /", self.internal)

    def test_cli_config_and_summary_are_default_off_and_fail_closed(self) -> None:
        selector = "--native-phase135-official-affine-measurement-family"
        self.assertIn(selector, self.app)
        self.assertIn(
            "native_phase135_official_affine_measurement_family = false", self.app
        )
        self.assertIn(
            "options.native_phase135_official_affine_measurement_family = true",
            self.app,
        )
        self.assertIn(
            "use_native_phase135_official_affine_measurement_family = false",
            self.config,
        )
        self.assertIn(
            "config.use_native_phase135_official_affine_measurement_family = true",
            self.app,
        )
        self.assertIn(
            '\\"phase135_official_affine_measurement_family\\": {', self.app
        )
        self.assertIn("phase135_configuration_valid", self.result)
        self.assertIn("Phase135 requires the complete source C7/D meter graph", self.eigen)
        self.assertIn("failPhase135", self.backend)
        self.assertIn("phase135_geometry_rows_validated", self.result)

    def test_transactional_family_and_provenance_are_not_mixed(self) -> None:
        for field in (
            "use_official_tdcp_huber_k",
            "use_native_phase126_raw_base_source_complete",
            "use_native_phase127_glonass_channel_provenance",
            "use_native_phase128_glonass_provenance_parser_admission",
            "use_native_phase129_glonass_local_miss_mask",
            "use_native_phase131_canonical_correction_band_key",
        ):
            self.assertIn("!config_." + field, self.eigen)
        self.assertIn("phase135_geometry_representation", self.result)
        self.assertIn("phase135_pseudorange_factors_inserted", self.backend)
        self.assertIn("phase135_doppler_factors_inserted", self.backend)
        self.assertIn("phase135_tdcp_factors_inserted", self.backend)
        self.assertIn(
            "Phase135 requires nonempty pseudorange, Doppler, and TDCP families",
            self.backend,
        )
        self.assertIn("phase135", self.tests_cmake)


if __name__ == "__main__":
    unittest.main()
