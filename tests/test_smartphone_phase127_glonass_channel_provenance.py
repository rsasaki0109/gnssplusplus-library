"""Launch-free Phase127 CLI/config wiring and provenance-contract checks.

This test reads only tracked source and the sealed Phase127 design.  It never
opens route data, navigation payloads, truth, solutions, MAT files, or starts
the native solver.
"""

from __future__ import annotations

import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase127_glonass_channel_provenance_freeze_v1.json"
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
FGO_CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
BASE_HEADER = ROOT / "include/libgnss++/algorithms/base_pseudorange_compensation.hpp"
BASE_SOURCE = ROOT / "src/algorithms/base_pseudorange_compensation.cpp"
HELPER_HEADER = ROOT / "include/libgnss++/algorithms/phase127_glonass_channel_provenance.hpp"
HELPER_SOURCE = ROOT / "src/algorithms/phase127_glonass_channel_provenance.cpp"
RINEX_HEADER = ROOT / "include/libgnss++/io/rinex.hpp"
RINEX_SOURCE = ROOT / "src/io/rinex.cpp"
CMAKE = ROOT / "CMakeLists.txt"
TEST_CMAKE = ROOT / "tests/CMakeLists.txt"


class Phase127CliContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        cls.app = APP.read_text(encoding="utf-8")
        cls.fgo_config = FGO_CONFIG.read_text(encoding="utf-8")
        cls.base_header = BASE_HEADER.read_text(encoding="utf-8")
        cls.base_source = BASE_SOURCE.read_text(encoding="utf-8")
        cls.helper_header = HELPER_HEADER.read_text(encoding="utf-8")
        cls.helper_source = HELPER_SOURCE.read_text(encoding="utf-8")
        cls.rinex_header = RINEX_HEADER.read_text(encoding="utf-8")
        cls.rinex_source = RINEX_SOURCE.read_text(encoding="utf-8")

    def test_freeze_selector_is_single_default_off_composed_candidate(self) -> None:
        decision = self.freeze["decision"]
        candidate = self.freeze["candidate"]
        self.assertEqual(decision["selector"], "--native-phase127-glonass-channel-provenance")
        self.assertEqual(candidate["selector"], decision["selector"])
        self.assertEqual(candidate["requires_selectors"], [
            "--native-phase126-raw-base-source-complete",
        ])
        self.assertTrue(decision["default_off"])
        self.assertFalse(decision["implementation_authorized"])

    def test_cli_and_library_defaults_are_off_and_composition_is_explicit(self) -> None:
        selector = "--native-phase127-glonass-channel-provenance"
        self.assertIn(selector, self.app)
        self.assertIn("native_phase127_glonass_channel_provenance = false", self.app)
        self.assertIn("use_native_phase127_glonass_channel_provenance = false", self.fgo_config)
        self.assertIn("use_phase127_glonass_channel_provenance = false", self.base_header)
        self.assertIn("if (!options.native_phase126_raw_base_source_complete)", self.app)
        self.assertIn("base_config.use_phase127_glonass_channel_provenance", self.app)

    def test_header_ledger_and_timed_broadcast_helper_are_wired(self) -> None:
        self.assertIn("glonass_frequency_channel_entries", self.rinex_header)
        self.assertIn("glonass_frequency_channel_malformed_entries", self.rinex_header)
        self.assertIn("glonass_frequency_channel_entries.emplace_back", self.rinex_source)
        self.assertIn("glonass_frequency_channel_present", self.helper_source)
        self.assertIn("navigation.getEphemeris(satellite, query_time)", self.helper_source)
        self.assertIn("ephemeris.isValid(query_time)", self.helper_source)
        self.assertIn("kTieToleranceSeconds", self.helper_source)
        self.assertIn("header-geph-fcn-mismatch", self.helper_source)
        self.assertIn("query-time-coverage-gap", self.helper_source)
        self.assertIn("phase127_failure_counts", self.base_header)
        self.assertIn("phase127_failure_counts", self.base_source)

    def test_fcn_domain_and_forbidden_fallbacks_are_source_locked(self) -> None:
        self.assertIn("kMinFrequencyChannel = -7", self.helper_header)
        self.assertIn("kMaxFrequencyChannel = 6", self.helper_header)
        self.assertIn("GLO_L1_BASE_FREQ", self.helper_source)
        self.assertIn("GLO_L2_BASE_FREQ", self.helper_source)
        self.assertIn("header-fcn-invalid", self.helper_source)
        self.assertIn("ephemeris-fcn-missing", self.helper_source)
        self.assertIn("ephemeris-tie-fcn-conflict", self.helper_source)
        self.assertNotIn("CarrierFrequencyHz", self.helper_source)
        self.assertNotIn("fixed_channel", self.helper_source)

    def test_build_and_focused_test_are_registered(self) -> None:
        self.assertIn("phase127_glonass_channel_provenance.cpp", CMAKE.read_text(encoding="utf-8"))
        self.assertIn("test_phase127_glonass_channel_provenance.cpp", TEST_CMAKE.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
