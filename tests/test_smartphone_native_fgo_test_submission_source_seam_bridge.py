from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_native_fgo_test_submission_source_seam_bridge as BRIDGE  # noqa: E402
import gnss_smartphone_native_fgo_test_submission_recovery as RECOVERY  # noqa: E402


def _point(timestamp: int, offset_m: float, source: str) -> RECOVERY.PositionSample:
    return RECOVERY.PositionSample(
        timestamp_ms=timestamp,
        ecef=(6_370_000.0 + offset_m, 0.0, 0.0),
        latitude=0.0,
        longitude=0.0,
        height=0.0,
        source=source,
    )


class NativeFgoSourceSeamBridgeTests(unittest.TestCase):
    def test_contiguous_prior_run_uses_same_native_lane_endpoints(self) -> None:
        points = [
            _point(0, 0.0, "native_fgo_exact"),
            _point(1000, 300.0, "prior_wls_exact"),
            _point(2000, 300.0, "prior_wls_exact"),
            _point(3000, 40.0, "native_fgo_exact"),
        ]
        candidate = BRIDGE._candidate_run("route/pixel", points, 1, 3)
        self.assertIsNotNone(candidate)
        assert candidate is not None
        self.assertEqual(candidate.trusted_lane, "native_fgo")
        self.assertEqual(candidate.run_count if hasattr(candidate, "run_count") else candidate.end_index - candidate.start_index, 2)
        self.assertEqual(candidate.bracket_gap_ms, 3000)
        self.assertLessEqual(candidate.endpoint_speed_mps, 70.0)
        output, report = BRIDGE._bridge_trips({"route/pixel": points})
        self.assertEqual(report["eligible_run_count"], 1)
        self.assertEqual(report["replaced_row_count"], 2)
        self.assertEqual(output["route/pixel"][1].source, "source_seam_ecef_interpolation")
        self.assertAlmostEqual(output["route/pixel"][1].ecef[0], 6_370_013.333333, places=5)
        self.assertAlmostEqual(output["route/pixel"][2].ecef[0], 6_370_026.666667, places=5)

    def test_nontrigger_seam_is_preserved(self) -> None:
        points = [
            _point(0, 0.0, "native_fgo_exact"),
            _point(1000, 20.0, "prior_wls_exact"),
            _point(2000, 40.0, "native_fgo_exact"),
        ]
        output, report = BRIDGE._bridge_trips({"route/pixel": points})
        self.assertEqual(report["eligible_run_count"], 0)
        self.assertEqual(report["nontrigger_seams_preserved"], 1)
        self.assertEqual(output["route/pixel"], points)

    def test_bracket_longer_than_ten_seconds_fails_closed(self) -> None:
        points = [
            _point(0, 0.0, "native_fgo_exact"),
            _point(1000, 200.0, "prior_wls_exact"),
            _point(11000, 20.0, "native_fgo_exact"),
        ]
        output, report = BRIDGE._bridge_trips({"route/pixel": points})
        self.assertEqual(report["eligible_run_count"], 0)
        self.assertEqual(output["route/pixel"], points)
        rejected = report["high_speed_seam_audits"]
        self.assertEqual(len(rejected), 1)
        self.assertFalse(rejected[0]["eligible"])
        self.assertIn("bracket gap", rejected[0]["reason"])

    def test_endpoint_speed_over_bound_fails_closed(self) -> None:
        points = [
            _point(0, 0.0, "native_fgo_exact"),
            _point(1000, 200.0, "prior_wls_exact"),
            _point(2000, 400.0, "native_fgo_exact"),
        ]
        output, report = BRIDGE._bridge_trips({"route/pixel": points})
        self.assertEqual(report["eligible_run_count"], 0)
        self.assertEqual(output["route/pixel"], points)
        self.assertIn("endpoint speed", report["high_speed_seam_audits"][0]["reason"])


if __name__ == "__main__":
    unittest.main()
