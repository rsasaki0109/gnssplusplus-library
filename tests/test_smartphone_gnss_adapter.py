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
    "CodeType",
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


def device_row(
    timestamp: str,
    clock_count: str,
    signal: str = "GPS_L1_CA",
    *,
    constellation: str = "1",
    frequency: str = "1575420000",
    svid: str = "3",
    code_type: str = "",
) -> dict[str, str]:
    values = (
        "Raw",
        timestamp,
        "1000000000",
        "-1300000000000000000",
        clock_count,
        svid,
        "9",
        "123456789",
        "10",
        "35.5",
        "-12.25",
        "1",
        "123.5",
        "0.02",
        frequency,
        constellation,
        code_type,
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

    def run_adapter(
        self, *extra: str, role: str = "development"
    ) -> subprocess.CompletedProcess[str]:
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
                role,
                *extra,
            ),
            cwd=ROOT,
            text=True,
            capture_output=True,
            check=False,
        )

    def run_truth_free_adapter(self, *extra: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            (
                sys.executable,
                str(GNSS),
                "smartphone-gnss-adapter",
                "--device-gnss",
                str(self.device),
                "--truth-free",
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

    def test_truth_free_pipeline_does_not_require_or_open_ground_truth(self) -> None:
        rows = [device_row("1694113198000", "10")]
        self.write_device(rows)

        result = self.run_truth_free_adapter()

        self.assertEqual(result.returncode, 0, result.stderr)
        payload = json.loads((self.output / "summary.json").read_text(encoding="utf-8"))
        self.assertEqual(payload["decision"], "truth-free-pipeline")
        self.assertTrue(payload["truth_free"])
        self.assertFalse(payload["truth"]["used"])
        self.assertIsNone(payload["inputs"]["ground_truth"])
        self.assertTrue((self.output / "rover.obs").is_file())

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
        self.assertEqual(list(self.output.iterdir()), [])

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

    def test_development_galileo_e1_candidate_is_mixed_rinex_and_nav_checked(self) -> None:
        rows = [
            device_row("1694113198000", "10"),
            device_row(
                "1694113198000",
                "10",
                "GAL_E1_C_P",
                constellation="6",
                svid="2",
            ),
        ]
        self.write_device(rows)
        self.write_truth(["1694113198000"])
        nav = self.root / "brdc.nav"
        nav.write_text(
            "E02 2023 09 07 18 00 00 0.0 0.0 0.0\n",
            encoding="ascii",
        )

        result = self.run_adapter(
            "--experimental-galileo-e1",
            "--broadcast-nav",
            str(nav),
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        payload = json.loads((self.output / "summary.json").read_text(encoding="utf-8"))
        self.assertTrue(payload["experimental_signal_candidate"]["enabled"])
        self.assertEqual(payload["native_observation_adapter"]["file_system"], "M")
        self.assertEqual(payload["native_observation_adapter"]["signal_rows"]["GAL_E1_C_P"], 1)
        self.assertEqual(payload["navigation"]["route_observation_nav_coverage"], 1.0)
        rinex = (self.output / "rover.obs").read_text(encoding="ascii")
        self.assertIn("OBSERVATION DATA    M", rinex)
        self.assertIn("E    4 C1C L1C D1C S1C", rinex)
        self.assertIn("E02", rinex)

    def test_galileo_e1_candidate_rejects_wrong_frequency(self) -> None:
        self.write_device(
            [
                device_row(
                    "1694113198000",
                    "10",
                    "GAL_E1_C_P",
                    constellation="6",
                    frequency="1176450000",
                )
            ]
        )
        self.write_truth(["1694113198000"])
        nav = self.root / "brdc.nav"
        nav.write_text("E03 2023 09 07 18 00 00 0.0 0.0 0.0\n", encoding="ascii")
        result = self.run_adapter(
            "--experimental-galileo-e1",
            "--broadcast-nav",
            str(nav),
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("CarrierFrequencyHz", result.stderr)
        self.assertEqual(list(self.output.iterdir()), [])

    def test_galileo_hatch_smooths_code_and_reports_reset_contract(self) -> None:
        rows: list[dict[str, str]] = []
        for timestamp, pseudorange, adr, state in (
            ("1694113198000", "100.0", "0.0", "25"),
            ("1694113199000", "110.0", "10.0", "25"),
            ("1694113200000", "100.0", "20.0", "25"),
            ("1694113201000", "130.0", "30.0", "29"),
            ("1694113202000", "140.0", "40.0", "25"),
            ("1694113204000", "150.0", "50.0", "25"),
        ):
            row = device_row(
                timestamp,
                "10",
                "GAL_E1_C_P",
                constellation="6",
                svid="2",
            )
            row["RawPseudorangeMeters"] = pseudorange
            row["AccumulatedDeltaRangeMeters"] = adr
            row["AccumulatedDeltaRangeState"] = state
            rows.append(row)
        self.write_device(rows)
        self.write_truth([row["utcTimeMillis"] for row in rows])
        nav = self.root / "brdc.nav"
        nav.write_text(
            "E02 2023 09 07 18 00 00 0.0 0.0 0.0\n",
            encoding="ascii",
        )

        result = self.run_adapter(
            "--experimental-galileo-e1",
            "--experimental-galileo-e1-hatch-window-s",
            "10",
            "--broadcast-nav",
            str(nav),
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        payload = json.loads((self.output / "summary.json").read_text(encoding="utf-8"))
        hatch = payload["native_observation_adapter"]["hatch_carrier_smoothing"]
        self.assertTrue(hatch["enabled"])
        self.assertEqual(hatch["window_seconds"], 10)
        self.assertEqual(hatch["updates"], 2)
        self.assertEqual(hatch["reset_counts"]["cycle_slip"], 1)
        self.assertEqual(hatch["reset_counts"]["epoch_gap"], 1)
        self.assertEqual(hatch["max_continuous_samples"], 3)

        code_values = [
            float(line[3:17])
            for line in (self.output / "rover.obs").read_text(encoding="ascii").splitlines()
            if line.startswith("E02")
        ]
        self.assertEqual(code_values, [100.0, 110.0, 113.333, 130.0, 140.0, 150.0])
        self.assertIn("Hatch C1C 10s", (self.output / "rover.obs").read_text(encoding="ascii"))

    def test_hatch_requires_e1_and_is_sealed_from_holdout(self) -> None:
        self.write_device([device_row("1694113198000", "10")])
        self.write_truth(["1694113198000"])
        result = self.run_adapter("--experimental-galileo-e1-hatch-window-s", "10")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("requires --experimental-galileo-e1", result.stderr)

        nav = self.root / "brdc.nav"
        nav.write_text("E03 2023 09 07 18 00 00 0.0 0.0 0.0\n", encoding="ascii")
        result = self.run_adapter(
            "--experimental-galileo-e1",
            "--experimental-galileo-e1-hatch-window-s",
            "10",
            "--broadcast-nav",
            str(nav),
            role="holdout",
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("development-only", result.stderr)

    def test_galileo_candidate_cannot_run_on_holdout(self) -> None:
        result = self.run_adapter(
            "--experimental-galileo-e1",
            "--broadcast-nav",
            str(self.root / "brdc.nav"),
            role="holdout",
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("development-only", result.stderr)


if __name__ == "__main__":
    unittest.main()
