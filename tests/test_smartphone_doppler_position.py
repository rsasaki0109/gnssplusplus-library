from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_doppler_position as DOPPLER  # noqa: E402
import gnss_smartphone_trajectory_smoother as SMOOTHER  # noqa: E402


class SmartphoneDopplerPositionTests(unittest.TestCase):
    @staticmethod
    def _observations(receiver_velocity: np.ndarray, clock_rate: float):
        receiver = np.array((6_378_137.0, 0.0, 0.0), dtype=float)
        directions = (
            np.array((0.0, 1.0, 0.0)),
            np.array((0.0, 0.0, 1.0)),
            np.array((1.0, 1.0, 0.0)) / np.sqrt(2.0),
            np.array((1.0, 0.0, 1.0)) / np.sqrt(2.0),
            np.array((0.0, 1.0, 1.0)) / np.sqrt(2.0),
            np.array((-1.0, 1.0, 1.0)) / np.sqrt(3.0),
        )
        observations = []
        for index, direction in enumerate(directions, start=1):
            satellite = receiver + 22_000_000.0 * direction
            satellite_velocity = np.array((40.0, -20.0, 15.0)) + index * np.array(
                (1.0, 2.0, -1.0)
            )
            rate = float(
                direction @ (satellite_velocity - receiver_velocity) + clock_rate
            )
            observations.append(
                DOPPLER._Observation(
                    ("1", str(index)),
                    35.0 + index,
                    rate,
                    0.15,
                    satellite,
                    satellite_velocity,
                )
            )
        return receiver, observations

    def test_weighted_doppler_recovers_receiver_velocity_and_clock(self) -> None:
        velocity = np.array((7.0, -3.0, 2.0), dtype=float)
        receiver, observations = self._observations(velocity, 4.0)
        estimate = DOPPLER.estimate_doppler(
            1000, receiver, 0, observations, DOPPLER.DopplerConfig()
        )
        self.assertTrue(estimate.valid)
        np.testing.assert_allclose(estimate.velocity_ecef_mps, velocity, atol=1e-9)
        self.assertAlmostEqual(estimate.clock_range_rate_mps or 0.0, 4.0, places=9)
        self.assertEqual(estimate.inlier_count, len(observations))

    def test_robust_screen_rejects_one_gross_rate(self) -> None:
        velocity = np.array((7.0, -3.0, 2.0), dtype=float)
        receiver, observations = self._observations(velocity, 4.0)
        bad = list(observations)
        bad[-1] = DOPPLER._Observation(
            bad[-1].key,
            bad[-1].cn0,
            bad[-1].rate_mps + 40.0,
            bad[-1].uncertainty_mps,
            bad[-1].satellite_ecef,
            bad[-1].satellite_velocity_ecef_mps,
        )
        estimate = DOPPLER.estimate_doppler(
            1000, receiver, 0, bad, DOPPLER.DopplerConfig()
        )
        self.assertTrue(estimate.valid)
        self.assertEqual(estimate.inlier_count, len(observations) - 1)
        np.testing.assert_allclose(estimate.velocity_ecef_mps, velocity, atol=1e-8)

    def test_invalid_doppler_falls_back_to_exact_wls(self) -> None:
        base = SMOOTHER.PositionRow(
            2200,
            10.0,
            1000,
            np.array((6_378_137.0, 0.0, 0.0), dtype=float),
            0.0,
            0.0,
            0.0,
            1,
            8,
            0.0,
            0.0,
            0,
            0,
            1,
        )
        receiver, observations = self._observations(np.array((1.0, 0.0, 0.0)), 0.0)
        raw = {1000: (0, tuple(observations)), 2000: (0, tuple(observations[:3]))}
        second = SMOOTHER.PositionRow(
            2200,
            11.0,
            2000,
            receiver + np.array((1.0, 0.0, 0.0)),
            0.0,
            0.0,
            0.0,
            1,
            8,
            0.0,
            0.0,
            0,
            0,
            2,
        )
        result = DOPPLER.build_candidate(
            [base, second], [1000, 2000], raw, DOPPLER.DopplerConfig()
        )
        self.assertEqual(result.sources[0], "wls_anchor")
        self.assertEqual(result.sources[1], "wls_doppler_fallback")
        np.testing.assert_allclose(result.rows[1].ecef, second.ecef)

    def test_config_rejects_unsafe_minimum_satellites(self) -> None:
        with self.assertRaises(DOPPLER.DopplerPositionError):
            DOPPLER.DopplerConfig(min_satellites=3).validate()


if __name__ == "__main__":
    unittest.main()
