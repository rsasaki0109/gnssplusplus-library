#!/usr/bin/env python3
"""Contract tests for the urban RTK/IMU bridge scorer."""

from __future__ import annotations

import csv
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
DISPATCHER = ROOT_DIR / "apps" / "gnss.py"
sys.path.insert(0, str(ROOT_DIR / "scripts"))

import generate_driving_comparison as comparison  # noqa: E402


def write_reference(path: Path, rows: list[tuple[int, float, float, float, float]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ("gps_tow_s", "gps_week", "lat_deg", "lon_deg", "height_m", "ecef_x_m", "ecef_y_m", "ecef_z_m")
        )
        for week, tow, lat, lon, height in rows:
            x, y, z = comparison.llh_to_ecef(lat, lon, height)
            writer.writerow((tow, week, lat, lon, height, x, y, z))


def write_pos(
    path: Path,
    rows: list[tuple[int, float, float, float, float, int]],
) -> None:
    with path.open("w", encoding="utf-8") as handle:
        handle.write("% LibGNSS++ Position Solution\n")
        for week, tow, lat, lon, height, status in rows:
            x, y, z = comparison.llh_to_ecef(lat, lon, height)
            handle.write(
                f"{week} {tow:.3f} {x:.6f} {y:.6f} {z:.6f} "
                f"{lat:.10f} {lon:.10f} {height:.4f} {status} 12 1.0\n"
            )


class UrbanBridgeScoreTest(unittest.TestCase):
    def make_inputs(self, root: Path) -> tuple[Path, Path, Path]:
        week = 2200
        base_lat = 35.0
        base_lon = 139.0
        reference_rows = [
            (week, 100.0 + index, base_lat, base_lon + index * 0.00001, 40.0)
            for index in range(8)
        ]
        reference_path = root / "reference.csv"
        write_reference(reference_path, reference_rows)

        statuses = {0: 4, 1: 4, 2: 3, 3: 3, 5: 4, 6: 4, 7: 3}
        rtk_path = root / "rtk.pos"
        write_pos(
            rtk_path,
            [(*reference_rows[index], statuses[index]) for index in statuses],
        )

        north_errors_m = (0.10, 0.10, 1.0, 2.0, 3.0, 0.20, 0.20, 4.0)
        fused_path = root / "fused.pos"
        write_pos(
            fused_path,
            [
                (
                    week,
                    tow,
                    lat + north_errors_m[index] / 111_319.49,
                    lon,
                    height,
                    3,
                )
                for index, (week, tow, lat, lon, height) in enumerate(reference_rows)
            ],
        )
        return rtk_path, fused_path, reference_path

    def run_score(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [sys.executable, str(DISPATCHER), "urban-bridge-score", *args],
            cwd=ROOT_DIR,
            check=False,
            capture_output=True,
            text=True,
        )

    def test_scores_complete_and_trailing_bridge_with_manifest(self) -> None:
        with tempfile.TemporaryDirectory(prefix="urban_bridge_score_") as temp_dir:
            root = Path(temp_dir)
            rtk_path, fused_path, reference_path = self.make_inputs(root)
            summary_path = root / "score.json"
            segments_path = root / "segments.csv"
            result = self.run_score(
                "--rtk-pos", str(rtk_path),
                "--fused-pos", str(fused_path),
                "--reference-csv", str(reference_path),
                "--summary-json", str(summary_path),
                "--segments-csv", str(segments_path),
                "--require-complete-bridges-min", "1",
                "--require-fused-bridge-coverage-min", "100",
                "--require-max-bridge-error-max", "4.1",
                "--require-max-reacquisition-jump-max", "3.0",
                "--require-fixed-p95-regression-max", "0.3",
                "--require-fused-availability-at-least-rtk",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("degraded / complete bridges: 2 / 1", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["schema_version"], "libgnsspp.urban_bridge_score.v1")
            self.assertEqual(payload["coordinate_frame_contract"]["rtk_pos"], "antenna_ecef")
            self.assertEqual(payload["coordinate_frame_contract"]["fused_pos"], "antenna_ecef")
            self.assertEqual(
                payload["coordinate_frame_contract"]["internal_eskf_output"],
                "imu_origin_local_enu",
            )
            self.assertEqual(payload["gate"]["status"], "passed")
            self.assertTrue(payload["gate"]["thresholds"]["fused_availability_at_least_rtk"])
            aggregate = payload["aggregate"]
            self.assertEqual(aggregate["reference_epochs"], 8)
            self.assertEqual(aggregate["fused_nonfinite_epochs"], 0)
            self.assertEqual(aggregate["rtk_degraded_reference_epochs"], 4)
            self.assertEqual(aggregate["complete_bridges"], 1)
            self.assertEqual(aggregate["fused_bridge_coverage_pct"], 100.0)
            self.assertTrue(math.isclose(aggregate["max_bridge_horizontal_error_m"], 4.0, abs_tol=0.02))
            self.assertTrue(math.isclose(aggregate["max_reacquisition_error_step_m"], 2.8, abs_tol=0.02))
            self.assertEqual(len(payload["segments"]), 2)
            self.assertTrue(payload["segments"][0]["complete_bridge"])
            self.assertFalse(payload["segments"][1]["complete_bridge"])
            for record in payload["inputs"].values():
                self.assertEqual(len(record["sha256"]), 64)
                self.assertGreater(record["bytes"], 0)
            self.assertEqual(len(segments_path.read_text(encoding="utf-8").splitlines()), 3)

    def test_availability_gate_fails_when_fused_is_less_than_rtk(self) -> None:
        with tempfile.TemporaryDirectory(prefix="urban_bridge_availability_gate_") as temp_dir:
            root = Path(temp_dir)
            rtk_path, fused_path, reference_path = self.make_inputs(root)
            fused_lines = fused_path.read_text(encoding="utf-8").splitlines()
            # Keep only six of eight fused epochs while RTK still has seven;
            # this exercises the comparison against the RTK denominator rather
            # than an absolute availability threshold.
            fused_path.write_text("\n".join(fused_lines[:7]) + "\n", encoding="utf-8")
            summary_path = root / "availability_failed.json"
            result = self.run_score(
                "--rtk-pos", str(rtk_path),
                "--fused-pos", str(fused_path),
                "--reference-csv", str(reference_path),
                "--summary-json", str(summary_path),
                "--require-fused-availability-at-least-rtk",
            )

            self.assertEqual(result.returncode, 1)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["gate"]["status"], "failed")
            self.assertTrue(
                any("fused_availability_pct" in failure for failure in payload["gate"]["failures"])
            )

    def test_gate_failure_still_writes_evidence(self) -> None:
        with tempfile.TemporaryDirectory(prefix="urban_bridge_gate_") as temp_dir:
            root = Path(temp_dir)
            rtk_path, fused_path, reference_path = self.make_inputs(root)
            summary_path = root / "failed.json"
            result = self.run_score(
                "--rtk-pos", str(rtk_path),
                "--fused-pos", str(fused_path),
                "--reference-csv", str(reference_path),
                "--summary-json", str(summary_path),
                "--require-max-bridge-error-max", "1.0",
            )

            self.assertEqual(result.returncode, 1)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["gate"]["status"], "failed")
            self.assertIn("max_bridge_horizontal_error_m", payload["gate"]["failures"][0])
            self.assertIn("FAIL:", result.stderr)

    def test_nonfinite_fused_epoch_fails_closed_when_required(self) -> None:
        with tempfile.TemporaryDirectory(prefix="urban_bridge_nonfinite_") as temp_dir:
            root = Path(temp_dir)
            rtk_path, fused_path, reference_path = self.make_inputs(root)
            with fused_path.open("a", encoding="utf-8") as handle:
                handle.write("2200 103.000 nan nan nan 35 139 40 3 0 999.9\n")
            summary_path = root / "nonfinite.json"
            result = self.run_score(
                "--rtk-pos", str(rtk_path),
                "--fused-pos", str(fused_path),
                "--reference-csv", str(reference_path),
                "--summary-json", str(summary_path),
                "--require-no-nonfinite",
            )

            self.assertEqual(result.returncode, 1)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["aggregate"]["fused_nonfinite_epochs"], 1)
            self.assertIn("fused_nonfinite_epochs", payload["gate"]["failures"][0])


if __name__ == "__main__":
    unittest.main()
