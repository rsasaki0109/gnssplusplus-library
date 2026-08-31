from __future__ import annotations

import csv
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
MODULE_PATH = ROOT / "apps" / "commands" / "benchmarks" / "gnss_smartphone_trajectory_smoother.py"
SPEC = importlib.util.spec_from_file_location("gnss_smartphone_trajectory_smoother", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
SMOOTHER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SMOOTHER
SPEC.loader.exec_module(SMOOTHER)


def _write_device(path: Path, timestamps: list[int]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["MessageType", "utcTimeMillis"])
        writer.writeheader()
        for timestamp in timestamps:
            writer.writerow({"MessageType": "Raw", "utcTimeMillis": timestamp})


def _write_pos(path: Path, rows: list[tuple[int, float, tuple[float, float, float]]]) -> None:
    lines = [
        "% fixture",
        "% Columns: GPS_Week GPS_TOW X Y Z Lat Lon Height Status Satellites PDOP Ratio Fixed Iterations",
    ]
    for week, tow, ecef in rows:
        latitude, longitude, height = SMOOTHER._wgs84_ecef_to_geodetic(
            SMOOTHER.np.array(ecef, dtype=float)
        )
        lines.append(
            f"{week} {tow:.3f} {ecef[0]:.6f} {ecef[1]:.6f} {ecef[2]:.6f} "
            f"{SMOOTHER.math.degrees(latitude):.9f} {SMOOTHER.math.degrees(longitude):.9f} {height:.6f} 1 8 2.0 0 0 3"
        )
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


class SmartphoneTrajectorySmootherTests(unittest.TestCase):
    def test_requires_explicit_experimental_flag(self) -> None:
        with tempfile.TemporaryDirectory(prefix="trajectory_smoother_") as temp_dir:
            root = Path(temp_dir)
            self.assertEqual(
                SMOOTHER.run(
                    [
                        "--position",
                        str(root / "missing.pos"),
                        "--device-gnss",
                        str(root / "missing.csv"),
                        "--output-dir",
                        str(root / "out"),
                    ]
                ),
                1,
            )

    def test_device_imu_requires_motion_adaptive_opt_in(self) -> None:
        with tempfile.TemporaryDirectory(prefix="trajectory_smoother_") as temp_dir:
            root = Path(temp_dir)
            self.assertEqual(
                SMOOTHER.run(
                    [
                        "--experimental-truth-free-kalman-rts",
                        "--position",
                        str(root / "missing.pos"),
                        "--device-gnss",
                        str(root / "missing.csv"),
                        "--device-imu",
                        str(root / "missing-imu.csv"),
                        "--output-dir",
                        str(root / "out"),
                    ]
                ),
                1,
            )

    def test_device_keys_fill_missing_epoch_and_generate_truth_free_submission(self) -> None:
        with tempfile.TemporaryDirectory(prefix="trajectory_smoother_") as temp_dir:
            root = Path(temp_dir)
            week = 2200
            base = SMOOTHER._wgs84_geodetic_to_ecef(
                SMOOTHER.math.radians(35.0), SMOOTHER.math.radians(139.0), 40.0
            )
            positions = [
                (week, 100.0, tuple(base)),
                (week, 102.0, tuple(base + SMOOTHER.np.array((2.0, 1.0, 0.5)))),
            ]
            position = root / "input.pos"
            _write_pos(position, positions)
            device = root / "device.csv"
            keys = [
                SMOOTHER._position_timestamp(str(week), f"{100 + index}.0", 18, index + 1)
                for index in range(3)
            ]
            _write_device(device, keys)
            output = root / "output"
            submission = root / "submission.csv"
            code = SMOOTHER.run(
                [
                    "--experimental-truth-free-kalman-rts",
                    "--position",
                    str(position),
                    "--device-gnss",
                    str(device),
                    "--output-dir",
                    str(output),
                    "--submission-output",
                    str(submission),
                    "--phone",
                    "fixture-phone",
                ]
            )
            self.assertEqual(code, 0)
            summary = json.loads((output / "smoother_summary.json").read_text())
            self.assertEqual(summary["populations"]["selected_device_epochs"], 3)
            self.assertEqual(summary["populations"]["input_position_epochs"], 2)
            self.assertEqual(summary["populations"]["synthesized_epochs"], 1)
            self.assertEqual(summary["populations"]["source_counts"]["interpolated"], 1)
            manifest = json.loads((output / "smoother_manifest.json").read_text())
            self.assertFalse(manifest["truth_used"])
            self.assertNotIn("ground_truth", manifest["inputs"])
            with submission.open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 3)
            self.assertEqual([int(row["UnixTimeMillis"]) for row in rows], keys)
            submission_manifest = json.loads(
                (root / "submission.csv.manifest.json").read_text()
            )
            self.assertFalse(submission_manifest["generator_contract"]["truth_used"])
            self.assertNotIn("ground_truth", submission_manifest["inputs"])

    def test_explicit_boundary_and_large_gap_reset_are_isolated(self) -> None:
        week = 2200
        timestamps = [
            SMOOTHER._position_timestamp(str(week), "100.0", 18, 1),
            SMOOTHER._position_timestamp(str(week), "101.0", 18, 2),
            SMOOTHER._position_timestamp(str(week), "120.0", 18, 3),
            SMOOTHER._position_timestamp(str(week), "121.0", 18, 4),
        ]
        base = SMOOTHER._wgs84_geodetic_to_ecef(
            SMOOTHER.math.radians(35.0), SMOOTHER.math.radians(139.0), 40.0
        )
        positions = [
            SMOOTHER.PositionRow(
                week,
                tow,
                timestamp,
                base + SMOOTHER.np.array((float(index), 0.0, 0.0)),
                0.0,
                0.0,
                0.0,
                1,
                8,
                2.0,
                0.0,
                0,
                3,
                index + 1,
            )
            for index, (tow, timestamp) in enumerate(
                [(100.0, timestamps[0]), (101.0, timestamps[1]), (120.0, timestamps[2]), (121.0, timestamps[3])]
            )
        ]
        result = SMOOTHER.smooth_positions(
            positions,
            timestamps,
            SMOOTHER.SmootherConfig(1.0, 1.0, segment_gap_s=10.0),
            reset_indices=(2,),
        )
        self.assertEqual(result.segment_count, 2)
        self.assertEqual(result.reset_indices, (2,))
        self.assertEqual(result.max_input_gap_s, 19.0)
        self.assertTrue(all(row.segment_id == 0 for row in result.rows[:2]))
        self.assertTrue(all(row.segment_id == 1 for row in result.rows[2:]))

    def test_bounded_reacquisition_resets_after_reject_run(self) -> None:
        week = 2200
        base = SMOOTHER._wgs84_geodetic_to_ecef(
            SMOOTHER.math.radians(35.0), SMOOTHER.math.radians(139.0), 40.0
        )
        timestamps = [
            SMOOTHER._position_timestamp(str(week), f"{100 + index}.0", 18, index + 1)
            for index in range(4)
        ]
        ecef_rows = [
            base,
            base + SMOOTHER.np.array((100.0, 0.0, 0.0)),
            base + SMOOTHER.np.array((100.0, 0.0, 0.0)),
            base + SMOOTHER.np.array((101.0, 0.0, 0.0)),
        ]
        positions = []
        for index, (timestamp, ecef) in enumerate(zip(timestamps, ecef_rows)):
            latitude, longitude, height = SMOOTHER._wgs84_ecef_to_geodetic(ecef)
            positions.append(
                SMOOTHER.PositionRow(
                    week,
                    100.0 + index,
                    timestamp,
                    ecef,
                    SMOOTHER.math.degrees(latitude),
                    SMOOTHER.math.degrees(longitude),
                    height,
                    1,
                    12,
                    1.5,
                    0.0,
                    0,
                    3,
                    index + 1,
                )
            )
        result = SMOOTHER.smooth_positions(
            positions,
            timestamps,
            SMOOTHER.SmootherConfig(
                1.0,
                1.0,
                5.0,
                10.0,
                2,
                2.0,
            ),
        )
        self.assertEqual(result.reacquisition_indices, (3,))
        self.assertEqual(result.max_consecutive_rejects, 2)
        self.assertEqual(result.segment_count, 2)
        self.assertEqual(result.rows[3].source, "reacquired")
        self.assertTrue(result.rows[3].measurement_used)
        self.assertFalse(result.rows[3].outlier_rejected)


if __name__ == "__main__":
    unittest.main()
