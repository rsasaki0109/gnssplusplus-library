"""Launch-free Phase131 canonical correction-key contract checks.

These tests inspect only source and sealed contract metadata.  They do not
materialize or read GNSS/IMU/navigation/base payloads, invoke the solver, or
open truth/MAT/Kaggle artifacts.
"""

from __future__ import annotations

import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase131_signal_key_canonicalization_freeze_v1.json"
)
KEY_HEADER = ROOT / "include/libgnss++/algorithms/phase131_canonical_correction_key.hpp"
BASE_HEADER = ROOT / "include/libgnss++/algorithms/base_pseudorange_compensation.hpp"
BASE_SOURCE = ROOT / "src/algorithms/base_pseudorange_compensation.cpp"
MISS_HEADER = ROOT / "include/libgnss++/algorithms/source_pseudorange_miss_mask.hpp"
MISS_SOURCE = ROOT / "src/algorithms/source_pseudorange_miss_mask.cpp"
FGO_HEADER = ROOT / "include/libgnss++/algorithms/fgo.hpp"
FGO_CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
FGO_PROBLEMS = ROOT / "src/algorithms/fgo_problems.cpp"
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"


class Phase131CanonicalCorrectionKeyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        cls.sources = {
            path: path.read_text(encoding="utf-8")
            for path in (
                KEY_HEADER,
                BASE_HEADER,
                BASE_SOURCE,
                MISS_HEADER,
                MISS_SOURCE,
                FGO_HEADER,
                FGO_CONFIG,
                FGO_PROBLEMS,
                APP,
            )
        }

    def test_freeze_is_single_default_off_candidate(self) -> None:
        decision = self.freeze["decision"]
        self.assertEqual(decision["candidate_count"], 1)
        self.assertEqual(decision["candidate_id"],
                         "phase131-canonical-correction-band-key-v1")
        self.assertEqual(decision["selector"],
                         "--native-phase131-canonical-correction-band-key")
        self.assertTrue(decision["source_backed"])
        for key in (
            "default_off",
            "implementation_authorized",
            "raw_materialization_authorized",
            "solver_execution_authorized",
            "truth_evaluation_authorized",
            "accuracy_authorized",
            "solution_publication_authorized",
            "kaggle_authorized",
        ):
            self.assertEqual(decision[key], key == "default_off")

    def test_typed_family_and_provenance_contract(self) -> None:
        text = self.sources[KEY_HEADER]
        for token in (
            "PhysicalFrequencyFamily",
            "GLO_L1CA",
            "GLO_L1P",
            "GPS_L5",
            "familyForRinexObservationType",
            "familyForAndroidSignalType",
            "glonass-fcn-missing",
            "glonass-fcn-out-of-range",
            "non-glonass-fcn-present",
            "-7",
            "6",
        ):
            self.assertIn(token, text)
        self.assertIn("signal_policy::trySignalForObservationType", text)
        self.assertIn("physical-frequency-family", text)

    def test_canonical_lookup_is_composed_and_default_off(self) -> None:
        for path in (BASE_HEADER, FGO_CONFIG):
            text = self.sources[path]
            self.assertIn("phase131_canonical_correction_band_key", text)
            self.assertIn("= false", text)
        self.assertIn("canonical_streams_", self.sources[BASE_SOURCE])
        self.assertIn("hasCanonicalStream", self.sources[BASE_HEADER])
        self.assertIn("correctionAtCanonical", self.sources[BASE_HEADER])
        self.assertIn("Phase126/127/128/129", self.sources[BASE_SOURCE])
        self.assertIn("no finite canonical correction stream",
                      self.sources[BASE_SOURCE])

    def test_factor_and_miss_mask_preserve_topology(self) -> None:
        factor = self.sources[FGO_HEADER]
        miss_header = self.sources[MISS_HEADER]
        miss_source = self.sources[MISS_SOURCE]
        for token in (
            "has_glonass_frequency_channel",
            "glonass_frequency_channel",
            "phase131_canonical_correction_band_key_enabled",
        ):
            self.assertIn(token, factor)
        self.assertIn("applyCanonical", miss_header)
        self.assertIn("CanonicalHasStream", miss_header)
        self.assertIn("CanonicalCorrectionAt", miss_header)
        self.assertIn("canonical_key_mode", miss_source)
        self.assertIn("native_base_pseudorange_correction_applied", miss_source)
        self.assertIn("factors.swap(retained)", miss_source)

    def test_cli_wiring_and_fail_closed_composition(self) -> None:
        app = self.sources[APP]
        flag = "--native-phase131-canonical-correction-band-key"
        self.assertGreaterEqual(app.count(flag), 3)  # usage, parser, validation
        self.assertIn("native_phase131_canonical_correction_band_key = false", app)
        self.assertIn("use_native_phase131_canonical_correction_band_key = true", app)
        self.assertIn("use_phase131_canonical_correction_band_key =", app)
        self.assertIn("applyCanonical", app)
        self.assertIn("(GNSSSystem,PRN,physical-frequency-family", app)
        for token in (
            "requires ",
            "composed Phase126/127/128/129 selectors",
            "forbids ",
            "alternate/diagnostic candidate selectors",
            "factor_topology_changed",
        ):
            self.assertIn(token, app)

    def test_fgo_builder_carries_certified_channel_only(self) -> None:
        text = self.sources[FGO_PROBLEMS]
        self.assertIn("annotatePhase127GlonassObservation", text)
        self.assertIn("factor.has_glonass_frequency_channel", text)
        self.assertIn("frequency_observation.glonass_frequency_channel", text)
        self.assertIn("Phase131 canonical correction key requires", text)


if __name__ == "__main__":
    unittest.main()
