"""Focused tests for the Phase73 source-exact pseudorange miss mask."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[1]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase73_source_exact_pseudorange_miss_mask_freeze_v1.json"
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
HELPER_H = ROOT / "include/libgnss++/algorithms/source_pseudorange_miss_mask.hpp"
HELPER_CPP = ROOT / "src/algorithms/source_pseudorange_miss_mask.cpp"
EXPECTED_FREEZE_SHA256 = "cd38928123f75e0ff71a42c1486e5bb5998bd506f0ccee3b830aa60c2d696b33"
FLAG = "--native-base-pseudorange-source-miss-mask"


class Phase73SourceExactMissMaskTests(unittest.TestCase):
    def test_freeze_is_unchanged_and_truth_free(self) -> None:
        digest = hashlib.sha256(FREEZE.read_bytes()).hexdigest()
        self.assertEqual(digest, EXPECTED_FREEZE_SHA256)
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(freeze["status"], "source-only-frozen-before-raw-read")
        self.assertEqual(freeze["candidate"]["new_opt_in_flag"], FLAG)
        self.assertFalse(freeze["candidate"]["preserve_additional_frequency_bands"])
        self.assertEqual(freeze["read_accounting_at_freeze"]["truth"], 0)
        self.assertEqual(freeze["read_accounting_at_freeze"]["solver_invocation"], 0)

    def test_helper_contract_uses_retained_vector_and_preserves_epoch_indices(self) -> None:
        source = HELPER_CPP.read_text(encoding="utf-8")
        self.assertIn("std::vector<FGOProcessor::PseudorangeFactor> retained", source)
        self.assertIn("factors.swap(retained)", source)
        self.assertIn("factor.epoch_index >= epochs.size()", source)
        self.assertIn("dropped_missing_exact_stream_rows", source)
        self.assertIn("dropped_out_of_domain_rows", source)
        self.assertIn("dropped_nonfinite_correction_rows", source)
        header = HELPER_H.read_text(encoding="utf-8")
        self.assertIn("factor's original epoch_index", header)
        self.assertIn("no file or navigation access", header)

    def test_cli_requires_base_compensation_and_rejects_band_combination(self) -> None:
        source = APP.read_text(encoding="utf-8")
        self.assertIn(FLAG, source)
        self.assertIn(FLAG + " requires", source)
        self.assertIn("--native-base-pseudorange-compensation", source)
        self.assertIn(FLAG + " cannot be combined", source)
        self.assertIn(
            "--native-base-pseudorange-preserve-additional-frequency-bands",
            source,
        )

    def test_summary_telemetry_and_factor_invariants_are_explicit(self) -> None:
        source = APP.read_text(encoding="utf-8")
        for field in (
            "original_adopted_pseudorange_rows",
            "retained_finite_pc_pseudorange_rows",
            "dropped_missing_exact_stream_rows",
            "dropped_out_of_domain_rows",
            "dropped_nonfinite_correction_rows",
            "retained_finite_pc_fraction",
            "retained_over_original_fraction",
            "pseudorange_factor_count_consistent",
            "correction_abs_p50_m",
            "correction_abs_p95_m",
            "correction_abs_max_m",
        ):
            self.assertIn(field, source)
        self.assertIn("tdcp_doppler_imu_spp_unchanged", source)
        self.assertIn("problem.pseudorange_factors.size()", source)

    def test_help_lists_new_flag_without_reading_inputs(self) -> None:
        binary = ROOT / "build/apps/gnss_fgo_imu_no_base"
        if not binary.is_file():
            self.skipTest(f"native binary not built: {binary}")
        env = os.environ.copy()
        local_library_dir = "/home/sasaki/.local/lib"
        env["LD_LIBRARY_PATH"] = local_library_dir + (
            ":" + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else ""
        )
        result = subprocess.run(
            [str(binary), "--help"],
            cwd=ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
            env=env,
        )
        self.assertIn(FLAG, result.stdout)


if __name__ == "__main__":
    unittest.main()
