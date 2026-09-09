"""Launch-free Phase112 output-offset contract checks.

Only native/official source text and the sealed Phase112 freeze are read.  The
test never opens or hashes raw GNSS/IMU/navigation/base, truth, MAT, solver,
or Kaggle artifacts.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
HELPER = ROOT / "include/libgnss++/algorithms/upstream_position_offset.hpp"
OFFICIAL = ROOT / (
    "output/reproducibility-cache/gsdc2023/functions/add_position_offset.m"
)
FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase112_main_fgo_source_parity_freeze_v1.json"
)


class Phase112MainOutputOffsetTests(unittest.TestCase):
    def test_freeze_is_one_default_off_unexecuted_candidate(self) -> None:
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(freeze["phase"], 112)
        self.assertEqual(freeze["candidate"]["candidate_count"], 1)
        self.assertEqual(
            freeze["candidate"]["id"],
            "phase112-main-output-upstream-position-offset-v1",
        )
        self.assertTrue(freeze["candidate"]["default_off"])
        self.assertFalse(freeze["authorization_boundary"]["implementation_authorized"])
        self.assertFalse(
            freeze["authorization_boundary"]["structural_raw_execution_authorized"]
        )
        self.assertFalse(freeze["authorization_boundary"]["truth_evaluation_authorized"])

    def test_official_pixel5_vector_and_existing_helper_are_pinned(self) -> None:
        official = OFFICIAL.read_text(encoding="utf-8")
        self.assertIn("offsetRL = -0.10;", official)
        self.assertIn("offsetUD = -0.30;", official)
        self.assertIn("R = eul2rotm(rpyest-[0 0 pi]);", official)
        self.assertIn("[offsetUD offsetRL 0]", official)

        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        expected = freeze["source_pins"]["native_existing_offset_helper"]["sha256"]
        self.assertEqual(hashlib.sha256(HELPER.read_bytes()).hexdigest(), expected)

    def test_exact_pixel5_gate_is_only_phase101_output_admission(self) -> None:
        source = APP.read_text(encoding="utf-8")
        selector_start = source.index(
            "bool phase112MainOutputPositionOffsetSelectors(const Options& options)"
        )
        selector_end = source.index(
            "struct DirectObservableQualitySettings", selector_start
        )
        selector = source[selector_start:selector_end]
        for term in (
            "native_source_clock_c0d_gnss_first_meter_state_handoff",
            "native_source_clock_c0d_epoch_vector_parity",
            "native_source_clock_c0d_phase99_main_multifrontal_qr_solver",
            "!options.native_source_clock_c0d_phase94_stage_diagnostics",
            "!options.native_source_clock_c0d_phase96_main_diagnostics",
            "!options.native_source_clock_c0d_phase97_singular_system_diagnostics",
            "!options.native_source_clock_c0d_phase98_solver_rank_diagnostic",
            "!options.native_phase104_stage_main_accuracy_attribution",
        ):
            self.assertIn(term, selector)

        handoff_start = source.index(
            "if (options.native_source_clock_c0d_gnss_first_meter_state_handoff) {"
        )
        handoff_end = source.index(
            "if (options.native_source_clock_c0d_epoch_vector_parity)", handoff_start
        )
        handoff = source[handoff_start:handoff_end]
        self.assertIn("phoneFromDatasetId(options.dataset_id) != \"pixel5\"", handoff)
        self.assertIn(
            "--native-upstream-position-offset with the Phase101", handoff
        )
        generic_start = handoff.index("if (options.fgo_imu_sparse_recovery")
        self.assertNotIn("native_upstream_position_offset", handoff[generic_start:])

    def test_application_is_main_only_single_pass_before_raw_key_alignment(self) -> None:
        source = APP.read_text(encoding="utf-8")
        offset_site = source.index(
            "UpstreamPositionOffsetReport position_offset_report;"
        )
        phase104_call = source.rfind("writePhase104MainDisplacementStats(")
        raw_alignment = source.index("RawUtcOutputReport raw_utc_report;", offset_site)
        self.assertLess(phase104_call, offset_site)
        self.assertLess(offset_site, raw_alignment)
        self.assertEqual(source.count("offsetFromRpy("), 1)
        self.assertEqual(source.count("position_offset_application.claim()"), 1)
        self.assertIn(
            "position offset application attempted more than once", source
        )

        # No stage or handoff path may call the output helper.
        handoff_start = source.index(
            "if (options.native_source_clock_c0d_gnss_first_meter_state_handoff) {"
        )
        handoff_end = source.index(
            "if (options.native_source_clock_c0d_epoch_vector_parity)", handoff_start
        )
        self.assertNotIn("offsetFromRpy(", source[handoff_start:handoff_end])

    def test_legacy_phone_branches_and_selector_default_remain_unchanged(self) -> None:
        helper = HELPER.read_text(encoding="utf-8")
        for term in (
            'phone.find("pixel7")',
            'phone.find("pixel4")',
            'phone.find("pixel5")',
            'offset = {-0.10, -0.20}',
            'offset = {-0.10, -0.30}',
        ):
            self.assertIn(term, helper)
        self.assertIn("bool native_upstream_position_offset = false;", APP.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
