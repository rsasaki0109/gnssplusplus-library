"""Focused no-reopen tests for Phase64 base policy recovery."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase64_base_preflight_policy_recovery as recovery  # noqa: E402


class SmartphonePhase64BasePreflightPolicyRecoveryTest(unittest.TestCase):
    def test_verify_freeze_reads_no_archive_or_base(self) -> None:
        contract = recovery._verify_freeze()
        self.assertEqual(contract["freeze"]["status"], "source-only-frozen-before-phase64-result-read")
        self.assertEqual(contract["manifest"]["archive_reopen_count"], 0)
        self.assertEqual(contract["manifest"]["base_file_reread_count"], 0)

    def test_policy_uses_obs_dt_and_ignores_header_marker(self) -> None:
        freeze = recovery._verify_freeze()["freeze"]
        self.assertIn("obsb.dt", freeze["source_semantics"]["correct_pseudorange_m"]["dt_selection"])
        self.assertIn("not read", freeze["source_semantics"]["correct_pseudorange_m"]["header_independence"])
        self.assertIn("may be blank", freeze["policy_recovery"]["station_provenance"])

    def test_finite_position_gate(self) -> None:
        self.assertTrue(recovery._finite_xyz([1.0, 2.0, 3.0]))
        self.assertFalse(recovery._finite_xyz([1.0, 2.0]))
        self.assertFalse(recovery._finite_xyz([1.0, 2.0, float("nan")]))

    def test_result_is_the_only_phase64_payload_input(self) -> None:
        self.assertEqual(recovery.RESULT_SHA256, "d6dc56c00c89ddd4e38624bbfe36567a76632aecae3f3f87f529ab0c8f93799b")
        source = recovery.SOURCE.read_text(encoding="utf-8")
        self.assertNotIn("zipfile", source)
        self.assertNotIn("device_gnss.csv", source)
        self.assertNotIn("ground_truth.csv", source)

    def test_route_order_is_four_and_no_accuracy(self) -> None:
        self.assertEqual(len(recovery.ROUTES), 4)
        self.assertFalse(recovery._verify_freeze()["freeze"]["accuracy_evaluated"])


if __name__ == "__main__":
    unittest.main()
