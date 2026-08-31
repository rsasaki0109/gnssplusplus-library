#!/usr/bin/env python3
"""Unit tests for the truth-free raw-quality control lane."""

from __future__ import annotations

import csv
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_raw_quality_control as quality  # noqa: E402


RAW_HEADER = [
    "MessageType", "utcTimeMillis", "HardwareClockDiscontinuityCount", "Svid",
    "SignalType", "RawPseudorangeMeters", "RawPseudorangeUncertaintyMeters",
    "Cn0DbHz", "AccumulatedDeltaRangeState", "CarrierFrequencyHz",
    "SvPositionXEcefMeters", "SvPositionYEcefMeters", "SvPositionZEcefMeters",
    "WlsPositionXEcefMeters", "WlsPositionYEcefMeters", "WlsPositionZEcefMeters",
    "ReceivedSvTimeUncertaintyNanos", "PseudorangeRateUncertaintyMetersPerSecond",
    "AccumulatedDeltaRangeUncertaintyMeters",
]


def _raw_row(timestamp: int, svid: int, signal: str, satellite: tuple[float, float, float], code: float) -> dict[str, str]:
    row = {field: "" for field in RAW_HEADER}
    row.update(
        {
            "MessageType": "Raw",
            "utcTimeMillis": str(timestamp),
            "HardwareClockDiscontinuityCount": "0",
            "Svid": str(svid),
            "SignalType": signal,
            "RawPseudorangeMeters": str(code),
            "RawPseudorangeUncertaintyMeters": "2.0",
            "Cn0DbHz": "38.0",
            "AccumulatedDeltaRangeState": "1",
            "CarrierFrequencyHz": "1575420000",
            "SvPositionXEcefMeters": str(satellite[0]),
            "SvPositionYEcefMeters": str(satellite[1]),
            "SvPositionZEcefMeters": str(satellite[2]),
            "WlsPositionXEcefMeters": "6371000.0",
            "WlsPositionYEcefMeters": "0.0",
            "WlsPositionZEcefMeters": "0.0",
            "ReceivedSvTimeUncertaintyNanos": "20",
            "PseudorangeRateUncertaintyMetersPerSecond": "0.2",
            "AccumulatedDeltaRangeUncertaintyMeters": "0.01",
        }
    )
    return row


class RawQualityControlTests(unittest.TestCase):
    def test_observable_audit_is_truth_free_and_reports_geometry(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "device_gnss.csv"
            receiver = (6371000.0, 0.0, 0.0)
            satellites = (
                (20200000.0, 14000000.0, 21000000.0),
                (-21000000.0, 17000000.0, 19000000.0),
                (18000000.0, -22000000.0, 16000000.0),
                (16000000.0, 21000000.0, -19000000.0),
            )
            with path.open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=RAW_HEADER)
                writer.writeheader()
                for index, satellite in enumerate(satellites, start=1):
                    distance = sum((satellite[axis] - receiver[axis]) ** 2 for axis in range(3)) ** 0.5
                    writer.writerow(_raw_row(1000, index, "GPS_L1_CA", satellite, distance + 100.0))
                # A second supported signal tests signal/frequency accounting and
                # a documented large Android uncertainty sentinel.
                extra = _raw_row(1000, 8, "GAL_E1_C_P", satellites[0], 20000000.0)
                extra["RawPseudorangeUncertaintyMeters"] = "1000000000"
                writer.writerow(extra)
            report = quality.build_observable_audit(path)
            self.assertTrue(report["truth_free"])
            self.assertIsNone(report["truth_path"])
            self.assertEqual(report["populations"]["epochs"], 1)
            self.assertEqual(report["populations"]["supported_rows"], 5)
            feature = report["epoch_features"][0]
            self.assertEqual(feature["geometry_rank"], 4)
            self.assertEqual(feature["geometry_row_count"], 5)
            self.assertIsNotNone(feature["gdop_proxy"])
            self.assertEqual(feature["clock_discontinuity_count"], 0)
            self.assertEqual(report["distributions"]["raw_uncertainty_m"]["count"], 4)
            self.assertEqual(report["contract"]["gap_threshold_ms"], 1500)

    def test_merge_rewrites_nearby_native_timestamp_and_falls_back_missing(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fallback = root / "fallback.pos"
            robust = root / "robust.pos"
            fallback.write_text(
                "% test\n"
                "2139 165034.000000 -2704229.0 -4288800.0 3856689.0 37.0 -122.0 20.0 1 8 2.0 0 0 3\n"
                "2139 165035.000000 -2704215.0 -4288785.0 3856684.0 37.0 -122.0 20.0 1 8 2.0 0 0 3\n",
                encoding="ascii",
            )
            robust.write_text(
                "% robust\n"
                "2139 165034.000500 -2704228.0 -4288799.0 3856690.0 37.0 -122.0 20.0 1 9 2.0 0 0 3\n",
                encoding="ascii",
            )
            content, counts = quality._merge_positions(robust, fallback)
            text = content.decode("ascii")
            self.assertIn("-2704228.0", text)
            self.assertIn("-2704215.0", text)
            self.assertEqual(counts, {"robust": 1, "fallback": 1, "total": 2})
            self.assertIn("165034.000000", text)

    def test_merge_rejects_outside_tolerance(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fallback = root / "fallback.pos"
            robust = root / "robust.pos"
            line = "2139 165034.000000 -2704229.0 -4288800.0 3856689.0 37.0 -122.0 20.0 1 8 2.0 0 0 3\n"
            fallback.write_text(line, encoding="ascii")
            robust.write_text(
                line.replace("165034.000000", "165034.200000"), encoding="ascii"
            )
            with self.assertRaises(quality.QualityControlError):
                quality._merge_positions(robust, fallback, tolerance_ms=100)


if __name__ == "__main__":
    unittest.main()
