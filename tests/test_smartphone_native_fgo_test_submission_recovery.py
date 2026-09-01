from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_native_fgo_test_submission_recovery as RECOVERY  # noqa: E402


def _position(timestamp: int, latitude: float = 37.0, longitude: float = -122.0, source: str = "wls") -> RECOVERY.PositionSample:
    return RECOVERY.PositionSample(
        timestamp_ms=timestamp,
        ecef=(6_370_000.0, -1_000_000.0, 3_700_000.0),
        latitude=latitude,
        longitude=longitude,
        height=10.0,
        source=source,
    )


class NativeFgoSubmissionRecoveryTests(unittest.TestCase):
    def test_known_300_km_sample_dummy_is_rejected(self) -> None:
        with self.assertRaises(RECOVERY.RecoveryError):
            RECOVERY._guard_coordinate(
                RECOVERY.KNOWN_DUMMY_LATITUDE,
                RECOVERY.KNOWN_DUMMY_LONGITUDE,
                ((37.3268, -121.9460),),
            )

    def test_known_dummy_is_rejected_even_without_estimate(self) -> None:
        with self.assertRaises(RECOVERY.RecoveryError):
            RECOVERY._guard_coordinate(
                RECOVERY.KNOWN_DUMMY_LATITUDE,
                RECOVERY.KNOWN_DUMMY_LONGITUDE,
            )

    def test_cross_trip_interpolation_is_impossible(self) -> None:
        sources = {
            "route-a/pixel5": {
                "wls": {1000: _position(1000), 2000: _position(2000)},
            }
        }
        with self.assertRaises(RECOVERY.RecoveryError):
            RECOVERY._select_position("route-b/pixel5", 1500, sources)

    def test_same_trip_interpolation_uses_ecef_and_is_bounded(self) -> None:
        sources = {
            "route-a/pixel5": {
                "wls": {1000: _position(1000), 2000: _position(2000)},
            }
        }
        selected = RECOVERY._select_position("route-a/pixel5", 1500, sources)
        self.assertEqual(selected.source, "same_trip_ecef_interpolation")
        self.assertEqual(selected.timestamp_ms, 1500)

        too_wide = {
            "route-a/pixel5": {
                "wls": {1000: _position(1000), 3501: _position(3501)},
            }
        }
        with self.assertRaises(RECOVERY.RecoveryError):
            RECOVERY._select_position("route-a/pixel5", 2000, too_wide)

    def test_edge_hold_boundary_is_fixed_and_no_extrapolation(self) -> None:
        sources = {
            "route-a/pixel5": {
                "wls": {2000: _position(2000), 3000: _position(3000)},
            }
        }
        selected = RECOVERY._select_position("route-a/pixel5", 500, sources)
        self.assertEqual(selected.source, "same_trip_edge_hold")
        with self.assertRaises(RECOVERY.RecoveryError):
            RECOVERY._select_position("route-a/pixel5", -501, sources)

    def test_exact_priority_is_native_then_wls_then_prior(self) -> None:
        sources = {
            "route-a/pixel5": {
                "native_fgo": {1000: _position(1000, source="native")},
                "wls": {1000: _position(1000, source="wls")},
                "prior_wls": {2000: _position(2000, source="prior")},
            }
        }
        self.assertEqual(
            RECOVERY._select_position("route-a/pixel5", 1000, sources).source,
            "native_fgo_exact",
        )
        self.assertEqual(
            RECOVERY._select_position("route-a/pixel5", 2000, sources).source,
            "prior_wls_exact",
        )


if __name__ == "__main__":
    unittest.main()
