"""Focused source-only tests for Phase62 base-RINEX preflight."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase62_raw_base_preflight as preflight  # noqa: E402


class SmartphonePhase62RawBasePreflightTest(unittest.TestCase):
    def test_verify_freeze_does_not_open_archive(self) -> None:
        verified = preflight._verify_freeze()
        self.assertEqual(verified["freeze"]["status"], "source-only-frozen-before-raw-read")
        self.assertEqual(verified["manifest"]["read_contract"]["archive_open_count"], 1)

    def test_exact_source_member_formula(self) -> None:
        self.assertEqual(
            preflight._expected_member("2023-03-08-21-34-us-ca-mtv-u", "P221", "V2"),
            "dataset_2023/train/2023-03-08-21-34-us-ca-mtv-u/P221_rnx2.obs",
        )
        self.assertEqual(preflight._rnx_major("V2"), 2)
        self.assertEqual(preflight._rnx_major("v3"), 3)

    def test_course_window_is_deterministic_and_no_truth_based(self) -> None:
        first, last = preflight._parse_course_window("2023-03-08-21-34-us-ca-mtv-u", 3)
        self.assertAlmostEqual(last - first, 2.0)

    def test_rinex_epoch_parser_handles_v2_and_v3(self) -> None:
        self.assertIsNotNone(preflight._parse_epoch(" 23  3  8 21 34 00.0000000  0  1G01"))
        self.assertIsNotNone(preflight._parse_epoch("> 2023  3  8 21 34 00.0000000  0  1"))
        self.assertIsNone(preflight._parse_epoch("G01  123.0"))

    def test_forbidden_members_fail_closed(self) -> None:
        for member in ("x/phone_data.mat", "x/ground_truth.csv", "x/device_gnss.csv", "x/device_imu.csv"):
            with self.assertRaises(preflight.PreflightError):
                preflight._safe_member(member)

    def test_route_contract_covers_four_pixel5_routes(self) -> None:
        self.assertEqual(tuple(preflight.ROUTE_CONTRACT), preflight.ROUTES)
        self.assertEqual({item["rinex"] for item in preflight.ROUTE_CONTRACT.values()}, {"V2"})
        self.assertEqual(
            [preflight.ROUTE_CONTRACT[route]["base1"] for route in preflight.ROUTES],
            ["SLAC", "P221", "LBCH", "P221"],
        )


if __name__ == "__main__":
    unittest.main()
