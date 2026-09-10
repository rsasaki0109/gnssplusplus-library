"""Focused tests for the Phase64 scorer-only v2 recovery."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase64_base_preflight_policy_recovery_v2 as recovery  # noqa: E402


class SmartphonePhase64BasePreflightPolicyRecoveryV2Test(unittest.TestCase):
    def test_v2_manifest_and_freeze_verify_without_archive(self) -> None:
        contract = recovery._verify_freeze()
        self.assertEqual(contract["freeze"]["status"], "source-only-frozen-before-phase64-result-read")
        self.assertEqual(contract["manifest"]["archive_reopen_count"], 0)
        self.assertEqual(contract["manifest"]["base_file_reread_count"], 0)

    def test_corrected_nested_freeze_key_is_pinned(self) -> None:
        freeze = recovery._verify_freeze()["freeze"]
        self.assertIn("prior_phase62_phase63", freeze)
        self.assertNotIn("prior_phase62", freeze)

    def test_v2_source_has_no_archive_or_base_reader(self) -> None:
        source = recovery.SOURCE.read_text(encoding="utf-8")
        self.assertNotIn("zipfile", source)
        self.assertNotIn("device_gnss.csv", source)
        self.assertNotIn("ground_truth.csv", source)

    def test_v1_failure_is_disclosed(self) -> None:
        self.assertEqual(recovery.V1_SOURCE_SHA256, "6dc9681b878dcf23c3f56950b6ed12aa291270772936bcd88848aac230094f0f")


if __name__ == "__main__":
    unittest.main()
