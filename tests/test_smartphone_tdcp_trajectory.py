from __future__ import annotations

import csv
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_tdcp_trajectory as TDCP  # noqa: E402


class SmartphoneTdcpTrajectoryTests(unittest.TestCase):
    @staticmethod
    def _observations(delta: np.ndarray, clock: float, *, outlier: bool = False):
        base = np.array([1_000.0, 2_000.0, 3_000.0], dtype=float)
        directions = (
            np.array((1.0, 0.0, 0.0)),
            np.array((0.0, 1.0, 0.0)),
            np.array((0.0, 0.0, 1.0)),
            np.array((1.0, 1.0, 1.0)) / np.sqrt(3.0),
            np.array((1.0, -1.0, 1.0)) / np.sqrt(3.0),
            np.array((-1.0, 1.0, 1.0)) / np.sqrt(3.0),
            np.array((1.0, 1.0, -1.0)) / np.sqrt(3.0),
            np.array((-1.0, -1.0, 1.0)) / np.sqrt(3.0),
        )
        previous = {}
        current = {}
        for index, direction in enumerate(directions, start=1):
            satellite = base + 20_000_000.0 * direction
            los = direction
            measurement = -float(np.dot(los, delta)) + clock
            if outlier and index == len(directions):
                measurement += 20.0
            previous[(1, index)] = TDCP.RawObservation(
                1000,
                0,
                1,
                index,
                TDCP.GPS_SIGNAL,
                TDCP.ADR_VALID_BIT,
                100.0 + index,
                40.0,
                satellite,
            )
            current[(1, index)] = TDCP.RawObservation(
                2000,
                0,
                1,
                index,
                TDCP.GPS_SIGNAL,
                TDCP.ADR_VALID_BIT,
                100.0 + index + measurement,
                40.0,
                satellite,
            )
        return base, previous, current

    def test_solves_frozen_sign_and_clock_state(self) -> None:
        delta = np.array((1.0, -2.0, 0.5))
        base, previous, current = self._observations(delta, 3.0)
        result = TDCP.solve_tdcp(previous, current, 1000, 2000, base)
        self.assertEqual(result.reason, "accepted")
        np.testing.assert_allclose(result.delta_ecef, delta, atol=1e-9)
        self.assertEqual(result.pair_count, 8)

    def test_robust_gate_rejects_one_bad_carrier_pair(self) -> None:
        delta = np.array((1.0, -2.0, 0.5))
        base, previous, current = self._observations(delta, 3.0, outlier=True)
        result = TDCP.solve_tdcp(previous, current, 1000, 1100, base)
        # With only a small number of independent carrier pairs a gross
        # outlier can broaden the median scale.  The frozen RMS gate still
        # has to fail closed rather than publish a contaminated displacement.
        self.assertEqual(result.reason, "rms_gate")
        self.assertIsNone(result.delta_ecef)

    def test_gap_and_clock_transition_fail_closed(self) -> None:
        delta = np.array((1.0, -2.0, 0.5))
        base, previous, current = self._observations(delta, 3.0)
        self.assertEqual(
            TDCP.solve_tdcp(previous, current, 1000, 2501, base).reason,
            "pair_gap_or_order",
        )
        current = {
            key: TDCP.RawObservation(
                value.timestamp_ms,
                1,
                value.constellation,
                value.svid,
                value.signal,
                value.adr_state,
                value.adr_m,
                value.cn0_db_hz,
                value.satellite_ecef,
            )
            for key, value in current.items()
        }
        self.assertEqual(
            TDCP.solve_tdcp(previous, current, 1000, 1100, base).reason,
            "insufficient_pairs",
        )

    def test_truth_argument_is_not_accepted(self) -> None:
        with self.assertRaises(SystemExit):
            TDCP.parse_args(
                [
                    "--base-position",
                    "base.pos",
                    "--device-gnss",
                    "device.csv",
                    "--output-dir",
                    "out",
                    "--ground-truth",
                    "truth.csv",
                ]
            )

    def test_read_observations_selects_highest_cn0_and_omits_invalid_rows(self) -> None:
        fields = [
            "MessageType", "utcTimeMillis", "HardwareClockDiscontinuityCount",
            "Svid", "AccumulatedDeltaRangeState", "AccumulatedDeltaRangeMeters",
            "CarrierFrequencyHz", "ConstellationType", "SignalType", "Cn0DbHz",
            "SvPositionXEcefMeters", "SvPositionYEcefMeters", "SvPositionZEcefMeters",
        ]
        row = {
            field: "" for field in fields
        }
        row.update({
            "MessageType": "Raw", "utcTimeMillis": "1000",
            "HardwareClockDiscontinuityCount": "0", "Svid": "1",
            "AccumulatedDeltaRangeState": "1", "AccumulatedDeltaRangeMeters": "1",
            "CarrierFrequencyHz": "1575420000", "ConstellationType": "1",
            "SignalType": "GPS_L1_CA", "Cn0DbHz": "20",
            "SvPositionXEcefMeters": "20000000", "SvPositionYEcefMeters": "0",
            "SvPositionZEcefMeters": "0",
        })
        duplicate = dict(row, Cn0DbHz="40", AccumulatedDeltaRangeMeters="2")
        invalid = dict(row, Svid="2", AccumulatedDeltaRangeState="5")
        with tempfile.TemporaryDirectory() as name:
            path = Path(name) / "device.csv"
            with path.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields, lineterminator="\n")
                writer.writeheader()
                writer.writerows((row, duplicate, invalid))
            epochs, stats = TDCP.read_observations(path)
        self.assertEqual(epochs[1000][(1, 1)].adr_m, 2.0)
        self.assertEqual(stats["duplicate_lower_cn0_rows"], 1)
        self.assertEqual(stats["invalid_adr_state_rows"], 1)


if __name__ == "__main__":
    unittest.main()
