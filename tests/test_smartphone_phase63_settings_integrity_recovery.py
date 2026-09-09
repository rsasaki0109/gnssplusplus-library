"""Focused pre-archive tests for Phase63 settings-integrity recovery."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase63_settings_integrity_recovery as recovery  # noqa: E402


class SmartphonePhase63SettingsIntegrityRecoveryTest(unittest.TestCase):
    def test_verify_freeze_is_archive_free(self) -> None:
        verified = recovery._verify_freeze()
        self.assertEqual(verified["freeze"]["status"], "source-only-frozen-before-phase63-archive-read")
        self.assertEqual(verified["manifest"]["read_contract"]["archive_open_count"], 1)

    def test_phase62_source_is_reused_without_mutation(self) -> None:
        self.assertEqual(recovery.REUSED_SOURCE_SHA256, "a392ddeb438399817b13345b6c4b69a30943b4f9d9111536715752f77621f4b5")
        self.assertNotEqual(recovery.SOURCE, recovery.REUSED_SOURCE)

    def test_recovery_settings_hash_is_observed_phase62_value(self) -> None:
        self.assertEqual(recovery.SETTINGS_SHA256, "3e6ae65388b2809088b16732b87744e673f860c24a1fe0f709ef903a87397f39")
        self.assertEqual(recovery.SETTINGS_MEMBER, "dataset_2023/settings_train.csv")

    def test_archive_and_payload_reads_are_zero_in_verify_mode(self) -> None:
        self.assertEqual(recovery.ARCHIVE.exists(), True)
        # Existence is metadata-only; --verify-freeze never opens this path.
        self.assertEqual(recovery._verify_freeze()["manifest"]["read_contract"]["device_gnss_read_count"], 0)


if __name__ == "__main__":
    unittest.main()
