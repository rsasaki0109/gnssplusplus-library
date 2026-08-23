#!/usr/bin/env python3

from __future__ import annotations

import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
GNSS = ROOT / "apps" / "gnss.py"
DEVICE_FIELDS = (
    "MessageType",
    "utcTimeMillis",
    "TimeNanos",
    "FullBiasNanos",
    "HardwareClockDiscontinuityCount",
    "Svid",
    "State",
    "ReceivedSvTimeNanos",
    "ReceivedSvTimeUncertaintyNanos",
    "Cn0DbHz",
    "PseudorangeRateMetersPerSecond",
    "AccumulatedDeltaRangeState",
    "AccumulatedDeltaRangeMeters",
    "AccumulatedDeltaRangeUncertaintyMeters",
    "CarrierFrequencyHz",
    "ConstellationType",
    "SignalType",
    "ArrivalTimeNanosSinceGpsEpoch",
    "RawPseudorangeMeters",
    "RawPseudorangeUncertaintyMeters",
    "WlsPositionXEcefMeters",
    "WlsPositionYEcefMeters",
    "WlsPositionZEcefMeters",
    "ExtraField",
)
TRUTH_FIELDS = (
    "MessageType",
    "Provider",
    "LatitudeDegrees",
    "LongitudeDegrees",
    "AltitudeMeters",
    "UnixTimeMillis",
)


def device_row(timestamp: str, clock_count: str, signal: str = "GPS_L1_CA") -> dict[str, str]:
    values = (
        "Raw",
        timestamp,
        "1000000000",
        "-1300000000000000000",
        clock_count,
        "3",
        "9",
        "123456789",
        "10",
        "35.5",
        "-12.25",
        "1",
        "123.5",
        "0.02",
        "1575420000",
        "1",
        signal,
        "1378148416000000000",
        "22000000.25",
        "4.5",
        "-2700000.0",
        "-4300000.0",
        "3850000.0",
        "preserve-me",
    )
    return dict(zip(DEVICE_FIELDS, values))


class SmartphoneGnssAdapterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory(prefix="gnss_smartphone_adapter_")
        self.root = Path(self.temp.name)
        self.device = self.root / "device_gnss.csv"
        self.truth = self.root / "ground_truth.csv"
        self.output = self.root / "output"

    def tearDown(self) -> None:
        self.temp.cleanup()

    def write_device(self, rows: list[dict[str, str]]) -> None:
        with self.device.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=DEVICE_FIELDS)
            writer.writeheader()
            writer.writerows(rows)

    def write_truth(self, timestamps: list[str]) -> None:
        with self.truth.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=TRUTH_FIELDS)
            writer.writeheader()
            for timestamp in timestamps:
                writer.writerow(
                    {
                        "MessageType": "Fix",
                        "Provider": "GT",
                        "LatitudeDegrees": "37.0",
                        "LongitudeDegrees": "-122.0",
                        "AltitudeMeters": "10.0",
                        "UnixTimeMillis": timestamp,
                    }
                )

    def run_adapter(self, *extra: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            (
                sys.executable,
                str(GNSS),
                "smartphone-gnss-adapter",
                "--device-gnss",
                str(self.device),
                "--ground-truth",
                str(self.truth),
                "--output-dir",
                str(self.output),
                "--dataset-id",
                "fixture/phone",
                "--device-model",
                "test-phone",
                "--source-url",
                "https://example.test/dataset",
                "--source-terms",
                "test fixture only",
                "--role",
                "development",
                *extra,
            ),
            cwd=ROOT,
            text=True,
            capture_output=True,
            check=False,
        )

    def test_preserves_rows_and_emits_truth_scored_contract(self) -> None:
        rows = [
            device_row("1694113198000", "10"),
            device_row("1694113198000", "10", ""),
            device_row("1694113199000", "11"),
            device_row("1694113199000", "11", "GAL_E1_C_P"),
        ]
        self.write_device(rows)
        self.write_truth(["1694113198000", "1694113199000"])

        result = self.run_adapter()

        self.assertEqual(result.returncode, 0, result.stderr)
        payload = json.loads((self.output / "summary.json").read_text(encoding="utf-8"))
        self.assertEqual(payload["schema_version"], "smartphone-gnss-adapter.v1")
        self.assertEqual(payload["decision"], "adapter-pass")
        self.assertEqual(payload["observations"]["epochs"], 2)
        self.assertEqual(payload["observations"]["explicitly_skipped_epochs"], 0)
        self.assertEqual(payload["observations"]["hardware_clock_discontinuity_transitions"], 1)
        self.assertEqual(payload["observations"]["unmapped_signal_rows"], 1)
        self.assertEqual(payload["truth"]["coverage_ratio"], 1.0)
        self.assertEqual(payload["native_observation_adapter"]["epochs"], 2)
        rinex = (self.output / "rover.obs").read_text(encoding="ascii")
        self.assertIn("TIME SYSTEM ID", rinex)
        self.assertIn("G03", rinex)
        with (self.output / "observations.csv").open(encoding="utf-8", newline="") as handle:
            normalized = list(csv.DictReader(handle))
        self.assertEqual(normalized, rows)

    def test_rejects_malformed_timestamp(self) -> None:
        self.write_device([device_row("not-a-time", "10")])
        self.write_truth(["1694113198000"])
        result = self.run_adapter()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("invalid utcTimeMillis", result.stderr)

    def test_rejects_backwards_clock_discontinuity_count(self) -> None:
        self.write_device(
            [device_row("1694113198000", "10"), device_row("1694113199000", "9")]
        )
        self.write_truth(["1694113198000", "1694113199000"])
        result = self.run_adapter()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("discontinuity count moved backwards", result.stderr)

    def test_rejects_missing_truth_by_default(self) -> None:
        self.write_device(
            [device_row("1694113198000", "10"), device_row("1694113199000", "10")]
        )
        self.write_truth(["1694113198000"])
        result = self.run_adapter()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("missing truth for 1 selected epochs", result.stderr)

    def test_explicit_skip_preserves_truth_gate(self) -> None:
        self.write_device(
            [device_row("1694113198000", "10"), device_row("1694113199000", "10")]
        )
        self.write_truth(["1694113199000"])
        result = self.run_adapter("--skip-epochs", "1")
        self.assertEqual(result.returncode, 0, result.stderr)
        payload = json.loads((self.output / "summary.json").read_text(encoding="utf-8"))
        self.assertEqual(payload["observations"]["input_epochs"], 2)
        self.assertEqual(payload["observations"]["explicitly_skipped_epochs"], 1)
        self.assertEqual(payload["truth"]["coverage_ratio"], 1.0)


if __name__ == "__main__":
    unittest.main()
