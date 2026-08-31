#!/usr/bin/env python3
"""Truth-free fallback contracts for the upstream MAT test candidate."""

from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_gsdc2023_upstream_mat_fallback_v2 as fallback  # noqa: E402


def _position(
    trip_id: str,
    timestamp_ms: int,
    latitude: float = 35.0,
    longitude: float = 139.0,
    source: str = "fixture",
) -> fallback.Candidate:
    return fallback.Candidate(trip_id, timestamp_ms, latitude, longitude, 0.0, source)


class SmartphoneGsdC2023FallbackV2Tests(unittest.TestCase):
    def test_precedence_prefers_imu_exact_or_projection(self) -> None:
        result = fallback.resolve_one(
            "r/p",
            1500,
            (
                ("result_gnss_imu", [_position("r/p", 1000), _position("r/p", 2000, 35.1)]),
                ("result_gnss", [_position("r/p", 1500, 36.0)]),
            ),
        )
        self.assertIsNotNone(result)
        self.assertEqual(result.source, "result_gnss_imu_interpolated")
        self.assertAlmostEqual(result.latitude, 35.05)

    def test_gnss_is_used_after_long_imu_gap(self) -> None:
        result = fallback.resolve_one(
            "r/p",
            10_000,
            (
                ("result_gnss_imu", [_position("r/p", 0), _position("r/p", 20_001)]),
                ("result_gnss", [_position("r/p", 10_000, 35.2)]),
            ),
        )
        self.assertIsNotNone(result)
        self.assertEqual(result.source, "result_gnss_exact")
        self.assertAlmostEqual(result.latitude, 35.2)

    def test_frozen_v5_is_used_only_after_published_lanes(self) -> None:
        result = fallback.resolve_one(
            "r/p",
            10_000,
            (
                ("result_gnss_imu", []),
                ("result_gnss", []),
                ("v5", [_position("r/p", 10_000, 35.3, source="v5")]),
            ),
        )
        self.assertIsNotNone(result)
        self.assertEqual(result.source, "v5_exact")
        self.assertAlmostEqual(result.latitude, 35.3)

    def test_same_trip_only_and_no_long_gap_bridge(self) -> None:
        self.assertIsNone(
            fallback.resolve_one(
                "r/p",
                10_000,
                (("result_gnss_imu", [_position("other/p", 9_000), _position("other/p", 11_000)]),),
            )
        )
        self.assertIsNone(
            fallback.resolve_one(
                "r/p",
                10_000,
                (("result_gnss_imu", [_position("r/p", 0), _position("r/p", 20_001)]),),
            )
        )

    def test_edge_hold_boundary_is_bounded(self) -> None:
        self.assertEqual(
            fallback.project_from_source(
                [_position("r/p", 2_000)], "r/p", 1_000, "v5"
            ).source,
            "v5_edge_hold",
        )
        self.assertIsNone(
            fallback.project_from_source(
                [_position("r/p", 2_001)], "r/p", 1_000, "v5"
            )
        )

    def test_dummy_and_out_of_range_coordinates_fail_closed(self) -> None:
        with self.assertRaises(fallback.FallbackError):
            fallback.validate_candidate(
                _position("r/p", 1_000, *fallback.KNOWN_SAMPLE_DUMMY)
            )
        with self.assertRaises(fallback.FallbackError):
            fallback.validate_candidate(_position("r/p", 1_000, 91.0, 139.0))

    def test_atomic_output_keeps_sample_key_order_without_sample_values(self) -> None:
        rows = [_position("r/p", 1_000), _position("r/p", 2_000, 35.0001)]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "submission.csv"
            fallback._write_rows(output, rows)
            verification = fallback._verify_submission(output, [("r/p", 1_000), ("r/p", 2_000)])
            self.assertEqual(verification["row_count"], 2)
            self.assertEqual(output.read_text(encoding="ascii").splitlines()[0], ",".join(fallback.SAMPLE_FIELDS))


if __name__ == "__main__":
    unittest.main()
