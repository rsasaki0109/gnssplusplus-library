"""Focused tests for Phase64 scorer-only v3 recovery."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase64_base_preflight_policy_recovery_v3 as recovery  # noqa: E402


class SmartphonePhase64BasePreflightPolicyRecoveryV3Test(unittest.TestCase):
    def test_verify_freeze_and_v2_supersession_without_archive(self) -> None:
        contract = recovery._verify_freeze()
        self.assertEqual(contract["manifest"]["supersedes_manifest"], "docs/use_cases/records/smartphone_r5_phase64_base_preflight_policy_recovery_manifest_v2.json")
        self.assertEqual(contract["manifest"]["archive_reopen_count"], 0)

    def test_flat_phase63_route_is_normalized(self) -> None:
        normalized = recovery._flat_to_nested({"materialized_sha256": "x", "materialized_bytes": 3, "approx_position_xyz_m": [1, 2, 3]})
        self.assertEqual(normalized["base"]["sha256"], "x")
        self.assertEqual(normalized["base"]["bytes"], 3)

    def test_phase63_top_level_flat_schema_fixture_is_pinned(self) -> None:
        fixture = {
            "settings": {"sha256": "settings-digest"},
            "routes": {
                "route/pixel5": {
                    "materialized_sha256": "base-digest",
                    "materialized_bytes": 7,
                    "approx_position_xyz_m": [1.0, 2.0, 3.0],
                }
            },
        }
        self.assertEqual(fixture["settings"]["sha256"], "settings-digest")
        self.assertEqual(
            recovery._flat_to_nested(fixture["routes"]["route/pixel5"])["base"]["sha256"],
            "base-digest",
        )

    def test_v1_v2_failures_are_pinned_and_no_payload_reader(self) -> None:
        self.assertEqual(recovery.V2_SOURCE_SHA256, "49ec013aac3bc42476847e2bf363714222b2988084fe5ba5fa29928780013eef")
        source = recovery.SOURCE.read_text(encoding="utf-8")
        self.assertNotIn("zipfile", source)
        self.assertNotIn("device_gnss.csv", source)
        self.assertNotIn("ground_truth.csv", source)


if __name__ == "__main__":
    unittest.main()
