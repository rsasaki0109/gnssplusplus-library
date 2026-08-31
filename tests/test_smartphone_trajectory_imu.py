from __future__ import annotations

import csv
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
MODULE_PATH = ROOT / "apps" / "commands" / "benchmarks" / "gnss_smartphone_imu.py"
SPEC = importlib.util.spec_from_file_location("gnss_smartphone_imu", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
IMU = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = IMU
SPEC.loader.exec_module(IMU)


FIELDS = [
    "MessageType",
    "utcTimeMillis",
    "elapsedRealtimeNanos",
    "MeasurementX",
    "MeasurementY",
    "MeasurementZ",
    "BiasX",
    "BiasY",
    "BiasZ",
]


def _write_imu(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def _row(
    message_type: str,
    timestamp: int,
    elapsed: int,
    values: tuple[object, object, object],
    bias: tuple[object, object, object] = (0.0, 0.0, 0.0),
) -> dict[str, object]:
    return {
        "MessageType": message_type,
        "utcTimeMillis": timestamp,
        "elapsedRealtimeNanos": elapsed,
        "MeasurementX": values[0],
        "MeasurementY": values[1],
        "MeasurementZ": values[2],
        "BiasX": bias[0],
        "BiasY": bias[1],
        "BiasZ": bias[2],
    }


class SmartphoneTrajectoryImuTests(unittest.TestCase):
    def test_schema_audit_and_causal_motion_features(self) -> None:
        with tempfile.TemporaryDirectory(prefix="trajectory_imu_") as temp_dir:
            path = Path(temp_dir) / "device_imu.csv"
            rows: list[dict[str, object]] = []
            for timestamp, gyro_x in ((1000, 0.01), (1010, 0.01), (1020, 1.0)):
                rows.append(_row("UncalGyro", timestamp, timestamp * 1_000_000, (gyro_x, 0.0, 0.0)))
                rows.append(_row("UncalAccel", timestamp, timestamp * 1_000_000, (0.0, 0.0, 9.8)))
            rows.append(_row("UncalMag", 1000, 1_000_000_000, (1.0, 2.0, 3.0), (0.5, 0.0, 0.0)))
            _write_imu(path, rows)
            dataset = IMU.read_imu(path)
            self.assertEqual(dataset.audit["message_type_counts"]["UncalMag"], 1)
            self.assertFalse(dataset.audit["hardware_clock_discontinuity_count_field_present"])
            self.assertEqual(dataset.audit["unsupported_streams"]["UncalMag"]["bias_nonzero_components"], 1)
            profile = IMU.build_motion_profile(
                [1000, 1010, 1020],
                dataset,
                IMU.MotionConfig(
                    window_s=0.02,
                    max_sample_gap_s=0.25,
                    max_sample_age_s=0.25,
                    gyro_threshold=0.5,
                    accel_dynamic_threshold=100.0,
                    motion_q_multiplier=2.0,
                ),
            )
            # The high gyro sample at 1020 is in the future of epoch 1010 and
            # must not influence that epoch's causal feature.
            self.assertFalse(profile.epochs[1].dynamic)
            self.assertTrue(profile.epochs[2].dynamic)
            self.assertFalse(profile.summary["algorithm"]["future_samples_used"])

    def test_reset_clamps_causal_window_and_fail_safe_flags(self) -> None:
        with tempfile.TemporaryDirectory(prefix="trajectory_imu_") as temp_dir:
            path = Path(temp_dir) / "device_imu.csv"
            rows = [
                _row("UncalGyro", 1000, 1_000_000_000, (1.0, 0.0, 0.0)),
                _row("UncalAccel", 1000, 1_000_000_000, (0.0, 0.0, 9.8)),
                _row("UncalGyro", 2000, 2_000_000_000, (0.01, 0.0, 0.0)),
                _row("UncalAccel", 2000, 2_000_000_000, ("nan", 0.0, 9.8)),
                # A backward elapsed clock marks a discontinuity; the large
                # timestamp gap independently triggers the safe fallback.
                _row("UncalGyro", 2500, 1_500_000_000, (0.01, 0.0, 0.0)),
                _row("UncalAccel", 2500, 2_500_000_000, (0.0, 0.0, 9.8)),
            ]
            _write_imu(path, rows)
            dataset = IMU.read_imu(path)
            self.assertEqual(dataset.audit["supported_nonfinite_rows"], 1)
            self.assertGreaterEqual(
                dataset.audit["streams"]["UncalGyro"]["detected_clock_discontinuity_count"],
                1,
            )
            profile = IMU.build_motion_profile(
                [1000, 2000, 2500],
                dataset,
                IMU.MotionConfig(max_sample_age_s=0.25),
                reset_indices=(1,),
            )
            self.assertEqual(profile.summary["algorithm"]["reset_indices"], [1])
            self.assertTrue(profile.epochs[1].fallback_to_baseline)
            self.assertIn("nonfinite_sample", profile.epochs[1].fallback_reason or "")
            self.assertTrue(profile.epochs[2].fallback_to_baseline)
            self.assertIn("clock_discontinuity", profile.epochs[2].fallback_reason or "")
            self.assertEqual(profile.epochs[0].process_noise_multiplier, 2.0)
            self.assertTrue(
                all(
                    epoch.process_noise_multiplier == 1.0
                    for epoch in profile.epochs[1:]
                )
            )

    def test_motion_profile_rows_are_serializable(self) -> None:
        with tempfile.TemporaryDirectory(prefix="trajectory_imu_") as temp_dir:
            path = Path(temp_dir) / "device_imu.csv"
            _write_imu(
                path,
                [
                    _row("UncalGyro", 1000, 1_000_000_000, (0.01, 0.0, 0.0)),
                    _row("UncalAccel", 1000, 1_000_000_000, (0.0, 0.0, 9.8)),
                ],
            )
            profile = IMU.build_motion_profile(
                [1000], IMU.read_imu(path), IMU.MotionConfig()
            )
            rows = IMU.profile_rows(profile)
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["timestamp_ms"], 1000)
            self.assertIn("fallback_reason", rows[0])


if __name__ == "__main__":
    unittest.main()
