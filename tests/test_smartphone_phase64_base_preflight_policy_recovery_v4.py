"""Focused tests for the Phase64 scorer-only v4 erratum."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase64_base_preflight_policy_recovery_v4 as recovery  # noqa: E402


class SmartphonePhase64BasePreflightPolicyRecoveryV4Test(unittest.TestCase):
    def test_v4_supersedes_v3_without_archive_reopen(self) -> None:
        contract = recovery._verify_freeze()
        self.assertEqual(
            contract["manifest"]["supersedes_manifest"],
            "docs/use_cases/records/smartphone_r5_phase64_base_preflight_policy_recovery_manifest_v3.json",
        )
        self.assertEqual(contract["manifest"]["archive_reopen_count"], 0)

    def test_top_level_settings_and_flat_route_fixture(self) -> None:
        fixture = {
            "settings": {"sha256": recovery.PHASE63_SETTINGS_SHA256},
            "routes": {
                "route/pixel5": {
                    "materialized_sha256": "base-digest",
                    "materialized_bytes": 7,
                    "approx_position_xyz_m": [1.0, 2.0, 3.0],
                }
            },
        }
        self.assertEqual(fixture["settings"]["sha256"], recovery.PHASE63_SETTINGS_SHA256)
        self.assertEqual(recovery._flat_to_nested(fixture["routes"]["route/pixel5"])["base"]["bytes"], 7)

    def test_no_payload_reader_and_prior_v3_pin(self) -> None:
        self.assertEqual(len(recovery.V3_SOURCE_SHA256), 64)
        source = recovery.SOURCE.read_text(encoding="utf-8")
        self.assertNotIn("zipfile", source)
        self.assertNotIn("device_gnss.csv", source)
        self.assertNotIn("ground_truth.csv", source)


if __name__ == "__main__":
    unittest.main()
