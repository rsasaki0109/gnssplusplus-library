from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_native_fgo_test_submission_continuity as CONTINUITY  # noqa: E402
import gnss_smartphone_native_fgo_test_submission_recovery as RECOVERY  # noqa: E402


def _point(timestamp: int, offset_m: float, source: str = "native_fgo_exact") -> RECOVERY.PositionSample:
    return RECOVERY.PositionSample(
        timestamp_ms=timestamp,
        ecef=(6_370_000.0 + offset_m, 0.0, 0.0),
        latitude=0.0,
        longitude=0.0,
        height=0.0,
        source=source,
    )


class NativeFgoSubmissionContinuityTests(unittest.TestCase):
    def test_legitimate_70_mps_motion_is_preserved(self) -> None:
        points = [_point(0, 0.0), _point(1000, 70.0), _point(2000, 140.0)]
        processed, report = CONTINUITY._process_trip("route/pixel", points)
        self.assertEqual(report["repaired_points"], 0)
        self.assertEqual(processed, points)

    def test_isolated_two_sided_spike_is_repaired_in_ecef(self) -> None:
        points = [_point(0, 0.0), _point(1000, 100.0), _point(2000, 10.0)]
        processed, report = CONTINUITY._process_trip("route/pixel", points)
        self.assertEqual(report["candidate_spike_points"], 1)
        self.assertEqual(report["repaired_points"], 1)
        self.assertEqual(processed[1].source, "continuity_ecef_interpolation")
        self.assertAlmostEqual(processed[1].ecef[0], 6_370_005.0, places=6)
        self.assertEqual(report["post_isolated_two_sided_spike_points"], 0)

    def test_consecutive_outliers_are_fail_closed(self) -> None:
        points = [_point(0, 0.0), _point(1000, 100.0), _point(2000, 0.0), _point(3000, 100.0)]
        processed, report = CONTINUITY._process_trip("route/pixel", points)
        self.assertEqual(report["candidate_spike_points"], 2)
        self.assertEqual(report["blocked_adjacent_candidate_points"], 2)
        self.assertEqual(report["repaired_points"], 0)
        self.assertEqual(processed, points)

    def test_trip_boundaries_cannot_be_interpolated(self) -> None:
        points = [_point(1000, 0.0), _point(2000, 1000.0)]
        self.assertIsNone(CONTINUITY._candidate_details(points, 0))
        self.assertIsNone(CONTINUITY._candidate_details(points, 1))
        processed, report = CONTINUITY._process_trip("route/pixel", points)
        self.assertEqual(report["repaired_points"], 0)
        self.assertEqual(processed, points)

if __name__ == "__main__":
    unittest.main()
