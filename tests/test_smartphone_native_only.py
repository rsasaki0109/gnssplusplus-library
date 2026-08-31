#!/usr/bin/env python3
"""Regression tests for the fail-closed native-only smartphone boundary."""

from __future__ import annotations

import csv
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_native_only as native_only  # noqa: E402


GNSS_HEADER = [
    "MessageType",
    "utcTimeMillis",
    "TimeNanos",
    "FullBiasNanos",
    "BiasNanos",
    "ReceivedSvTimeNanos",
    "Svid",
    "ConstellationType",
    "PseudorangeRateMetersPerSecond",
    "AccumulatedDeltaRangeState",
    "AccumulatedDeltaRangeMeters",
    "CarrierFrequencyHz",
    "Cn0DbHz",
    "SignalType",
]
IMU_HEADER = [
    "MessageType",
    "utcTimeMillis",
    "elapsedRealtimeNanos",
    "MeasurementX",
    "MeasurementY",
    "MeasurementZ",
]


class NativeOnlyContractTests(unittest.TestCase):
    def _write_inputs(self, root: Path) -> tuple[Path, Path, Path]:
        gnss = root / "device_gnss.csv"
        with gnss.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(GNSS_HEADER)
            writer.writerow(
                [
                    "Raw",
                    1_700_000_000_000,
                    30_660_000_070_000_000,
                    -1_300_000_000_000_000_000,
                    0,
                    100_000_000_000_000,
                    3,
                    1,
                    0,
                    1,
                    42,
                    1_575_420_000,
                    40,
                    "GPS_L1_CA",
                ]
            )

        imu = root / "device_imu.csv"
        with imu.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(IMU_HEADER)
            writer.writerow(["UncalAccel", 1_700_000_000_000, 1, 0, 0, 9.8])
            writer.writerow(["UncalGyro", 1_700_000_000_000, 2, 0, 0, 0])

        nav = root / "brdc.nav"
        nav.write_text("raw broadcast navigation fixture\n", encoding="ascii")
        return gnss, imu, nav

    def test_raw_manifest_hashes_only_approved_inputs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="native_only_contract_") as directory:
            root = Path(directory)
            gnss, imu, nav = self._write_inputs(root)
            manifest_path = root / "manifest.json"
            manifest = native_only.validate_native_inputs(
                device_gnss=gnss,
                device_imu=imu,
                broadcast_nav=nav,
                output_manifest=manifest_path,
            )
            self.assertTrue(manifest["contract"]["native_inference_only"])
            self.assertEqual(
                manifest["approved_inputs"]["gnss"]["timing_mode"],
                "raw_android_clock",
            )
            self.assertEqual(
                set(manifest["read_members"]), {str(gnss), str(imu), str(nav)}
            )
            self.assertFalse(manifest["provenance"]["result_gnss_imu_read"])
            self.assertFalse(manifest["provenance"]["ground_truth_read"])
            self.assertTrue(manifest_path.is_file())

    def test_poisoned_result_sibling_is_rejected_and_never_read(self) -> None:
        with tempfile.TemporaryDirectory(prefix="native_only_poison_") as directory:
            root = Path(directory)
            gnss, imu, nav = self._write_inputs(root)
            first = native_only.validate_native_inputs(
                device_gnss=gnss, device_imu=imu, broadcast_nav=nav
            )
            poison = root / "result_gnss_imu.mat"
            poison.write_bytes(b"not a MAT file and must never be opened")
            second = native_only.validate_native_inputs(
                device_gnss=gnss, device_imu=imu, broadcast_nav=nav
            )
            self.assertEqual(first["approved_inputs"], second["approved_inputs"])
            self.assertNotIn(str(poison), second["read_members"])
            with self.assertRaises(native_only.NativeOnlyInputError):
                native_only.inspect_phone_data_mat(poison)

    def test_coordinate_and_submission_inputs_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory(prefix="native_only_forbidden_") as directory:
            root = Path(directory)
            for name in (
                "ground_truth.csv",
                "sample_submission.csv",
                "native-fgo-test-v5.csv",
            ):
                forbidden = root / name
                forbidden.write_text("poison\n", encoding="ascii")
                with self.assertRaises(native_only.NativeOnlyInputError):
                    native_only.inspect_raw_gnss(forbidden)

    def test_enriched_arrival_timing_is_not_accepted_as_raw_clock(self) -> None:
        with tempfile.TemporaryDirectory(prefix="native_only_enriched_") as directory:
            source = Path(directory) / "device_gnss.csv"
            header = [
                field
                for field in GNSS_HEADER
                if field not in {"TimeNanos", "FullBiasNanos", "ReceivedSvTimeNanos"}
            ]
            header.extend(
                [
                    "ArrivalTimeNanosSinceGpsEpoch",
                    "RawPseudorangeMeters",
                    "ReceivedSvTimeNanosSinceGpsEpoch",
                ]
            )
            with source.open("w", encoding="utf-8", newline="") as handle:
                csv.writer(handle).writerows([header, ["Raw"] + [0] * (len(header) - 1)])
            with self.assertRaises(native_only.NativeOnlyInputError):
                native_only.inspect_raw_gnss(source)

    def test_raw_phone_mat_requires_sensor_and_observation_container(self) -> None:
        try:
            import numpy as np
            from scipy.io import savemat
        except ImportError as exc:  # pragma: no cover - optional dependency
            self.skipTest(f"SciPy unavailable: {exc}")
        with tempfile.TemporaryDirectory(prefix="native_only_mat_") as directory:
            source = Path(directory) / "phone_data.mat"
            savemat(
                source,
                {
                    "obs": np.array([1.0]),
                    "acc": np.array([[0.0, 0.0, 9.8]]),
                    "gyro": np.array([[0.0, 0.0, 0.0]]),
                },
            )
            report = native_only.inspect_phone_data_mat(source)
            self.assertEqual(report["raw_gnss_container"], "obs")
            self.assertEqual(set(report["raw_sensor_fields"]), {"acc", "gyro"})


if __name__ == "__main__":
    unittest.main()
