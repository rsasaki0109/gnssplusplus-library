"""Launch-free Phase129 selector and admission-contract regression tests."""

from __future__ import annotations

import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase129_glonass_local_miss_admission_freeze_v1.json"
APP = (ROOT / "apps/native/gnss_fgo_imu_no_base.cpp").read_text(encoding="utf-8")
FGO_CONFIG = (ROOT / "include/libgnss++/algorithms/fgo_config.hpp").read_text(encoding="utf-8")
FGO_HEADER = (ROOT / "include/libgnss++/algorithms/fgo.hpp").read_text(encoding="utf-8")
BASE_HEADER = (ROOT / "include/libgnss++/algorithms/base_pseudorange_compensation.hpp").read_text(encoding="utf-8")
BASE_SOURCE = (ROOT / "src/algorithms/base_pseudorange_compensation.cpp").read_text(encoding="utf-8")
FGO_INTERNAL = (ROOT / "src/algorithms/fgo_internal.hpp").read_text(encoding="utf-8")
FGO_PROBLEMS = (ROOT / "src/algorithms/fgo_problems.cpp").read_text(encoding="utf-8")
POLICY = (ROOT / "include/libgnss++/algorithms/phase129_glonass_local_miss.hpp").read_text(encoding="utf-8")


class Phase129LocalMissContractTests(unittest.TestCase):
    def test_freeze_is_one_default_off_composed_candidate(self) -> None:
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        decision = freeze["decision"]
        candidate = freeze["candidate"]
        self.assertEqual(freeze["phase"], 129)
        self.assertEqual(decision["candidate_count"], 1)
        self.assertEqual(candidate["selector"], "--native-phase129-glonass-local-miss-mask")
        self.assertTrue(candidate["default_off"])
        self.assertTrue(candidate["partial_selector"] is False)
        self.assertEqual(
            candidate["requires_selectors"],
            [
                "--native-phase126-raw-base-source-complete",
                "--native-phase127-glonass-channel-provenance",
                "--native-phase128-glonass-provenance-parser-admission",
            ],
        )

    def test_selector_wiring_and_fail_closed_composition(self) -> None:
        selector = "--native-phase129-glonass-local-miss-mask"
        self.assertIn(selector, APP)
        self.assertIn("native_phase129_glonass_local_miss_mask = false", APP)
        self.assertIn("use_native_phase129_glonass_local_miss_mask = false", FGO_CONFIG)
        self.assertIn("use_phase129_glonass_local_miss_mask = false", BASE_HEADER)
        self.assertIn("--native-phase129-glonass-local-miss-mask requires ", APP)
        self.assertIn("the composed Phase126/127/128 selectors", APP)
        self.assertIn("phase129_configuration_valid", FGO_HEADER)
        self.assertIn("phase129_configuration_valid", BASE_HEADER)

    def test_shared_reason_ledger_and_local_drop_are_explicit(self) -> None:
        self.assertIn("phase129_glonass_local_miss.hpp", FGO_INTERNAL)
        self.assertIn("phase129_glonass_local_miss.hpp", BASE_SOURCE)
        self.assertIn("phase129_glonass_local_miss_counts", FGO_INTERNAL)
        self.assertIn("phase129_glonass_local_miss_counts", BASE_SOURCE)
        self.assertIn("rowLedgerConsistent", POLICY)
        self.assertIn("continue;", BASE_SOURCE)
        self.assertIn("continue;", FGO_PROBLEMS)
        self.assertIn("phase129_glonass_row_count_consistent", FGO_PROBLEMS)
        self.assertIn("phase129_glonass_row_count_consistent", BASE_SOURCE)

    def test_forbidden_fallbacks_and_global_failure_boundary_remain(self) -> None:
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        candidate = freeze["candidate"]
        self.assertFalse(candidate["fallback"])
        self.assertFalse(candidate["raw_uncorrected_retention"])
        self.assertFalse(candidate["zero_correction_fallback"])
        self.assertFalse(candidate["changes_c7_d_ccdd"])
        self.assertFalse(candidate["changes_solver_or_lm"])
        self.assertIn("source-complete geometry/Sagnac is non-finite", BASE_SOURCE)
        self.assertIn("source-complete atmosphere model is non-finite", BASE_SOURCE)
        self.assertIn("source-complete base residual is non-finite", BASE_SOURCE)
        self.assertIn("Phase129 GLONASS row ledger is inconsistent", BASE_SOURCE)

    def test_no_runtime_lanes_are_started_by_contract_tests(self) -> None:
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertFalse(freeze["decision"]["raw_materialization_authorized"])
        self.assertFalse(freeze["decision"]["solver_execution_authorized"])
        self.assertFalse(freeze["decision"]["truth_evaluation_authorized"])
        self.assertFalse(freeze["decision"]["kaggle_authorized"])


if __name__ == "__main__":
    unittest.main()
