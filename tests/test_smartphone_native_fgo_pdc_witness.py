#!/usr/bin/env python3
"""Tests for the truth-free Android/RINEX Doppler witness."""

from __future__ import annotations

import csv
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_native_fgo_pdc_witness as witness  # noqa: E402


FIELDS = [
    "utcTimeMillis",
    "PseudorangeRateMetersPerSecond",
    "PseudorangeRateUncertaintyMetersPerSecond",
    "CarrierFrequencyHz",
    "SvPositionXEcefMeters",
    "SvPositionYEcefMeters",
    "SvPositionZEcefMeters",
    "SvVelocityXEcefMetersPerSecond",
    "SvVelocityYEcefMetersPerSecond",
    "SvVelocityZEcefMetersPerSecond",
    "SvClockDriftMetersPerSecond",
    "WlsPositionXEcefMeters",
    "WlsPositionYEcefMeters",
    "WlsPositionZEcefMeters",
]


def row(timestamp: int, rate: float, satellite_x: float, wls_x: float) -> dict[str, str]:
    values = {field: "0" for field in FIELDS}
    values.update(
        {
            "utcTimeMillis": str(timestamp),
            "PseudorangeRateMetersPerSecond": str(rate),
            "PseudorangeRateUncertaintyMetersPerSecond": "0.1",
            "CarrierFrequencyHz": str(witness.GPS_L1_HZ),
            "SvPositionXEcefMeters": str(satellite_x),
            "SvPositionYEcefMeters": "1000000",
            "SvPositionZEcefMeters": "2000000",
            "SvVelocityXEcefMetersPerSecond": "100",
            "SvVelocityYEcefMetersPerSecond": "-200",
            "SvVelocityZEcefMetersPerSecond": "50",
            "SvClockDriftMetersPerSecond": "0.2",
            "WlsPositionXEcefMeters": str(wls_x),
            "WlsPositionYEcefMeters": "3000000",
            "WlsPositionZEcefMeters": "4000000",
        }
    )
    return values


class DopplerWitnessTests(unittest.TestCase):
    def test_roundtrip_is_truth_free_and_v5_boundary_is_explicit(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "device_gnss.csv"
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=FIELDS)
                writer.writeheader()
                writer.writerow(row(1000, 12.0, 20_000_000, 6_000_000))
                writer.writerow(row(1000, -7.0, -20_000_000, 6_000_000))
                writer.writerow(row(2000, 12.0, 20_000_000, 6_000_000))
                writer.writerow(row(2000, -7.0, -20_000_000, 6_000_000))

            result = witness.run_witness(path, "fixture/phone")

        self.assertTrue(result["truth_free"])
        self.assertEqual(result["population"]["finite_rows_used"], 4)
        self.assertEqual(
            result["mapping_variants"]["android_spec_mps"]["samples"],
            result["mapping_variants"]["rinex_roundtrip_mps"]["samples"],
        )
        self.assertEqual(
            result["mapping_variants"]["android_spec_mps"]["raw_residual_p95_abs_mps"],
            result["mapping_variants"]["rinex_roundtrip_mps"]["raw_residual_p95_abs_mps"],
        )
        self.assertFalse(result["interpretation"]["selection_or_tuning"])
        self.assertEqual(
            result["interpretation"]["v5_ecef_witness"],
            "blocked_missing_per_route_ecef",
        )

    def test_missing_satellite_state_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "device_gnss.csv"
            path.write_text("utcTimeMillis\n1000\n", encoding="utf-8")
            with self.assertRaises(witness.WitnessError):
                witness.run_witness(path)


if __name__ == "__main__":
    unittest.main()
