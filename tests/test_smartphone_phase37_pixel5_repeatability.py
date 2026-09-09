#!/usr/bin/env python3
"""Unit tests for the truth-free Phase37 Pixel5 runner."""

from __future__ import annotations

from pathlib import Path
import tempfile
import unittest
import sys


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase37_pixel5_repeatability as phase37  # noqa: E402


class Phase37Pixel5RepeatabilityTest(unittest.TestCase):
    def test_v2_freeze_is_pre_materialization_and_numeric(self) -> None:
        v1, v2 = phase37.verify_freeze()
        self.assertEqual(v1["schema_version"], "smartphone-r5-phase37-pixel5-repeatability-freeze.v1")
        self.assertEqual(v2["schema_version"], "smartphone-r5-phase37-pixel5-repeatability-freeze.v2")
        self.assertEqual(v2["status"], "frozen-before-phase37-materialization-and-solver")
        conditions = {item["id"]: item for item in v2["identifiability"]["cross_route_agreement"]["conditions"]}
        self.assertEqual(conditions["component_route_median_mad"]["threshold_m"], {"east": 0.5, "north": 0.5})
        self.assertEqual(conditions["geometric_center_radius"]["threshold_m"], 1.0)
        self.assertEqual(conditions["nonzero_common_median"]["threshold_factor"], 2.0)
        self.assertEqual(conditions["route_prefix_tail_stability"]["threshold_floor_m"], 1.0)

    def test_native_command_has_raw_inputs_only_and_fixed_flags(self) -> None:
        paths = {
            "device_gnss": Path("/tmp/device_gnss.csv"),
            "device_imu": Path("/tmp/device_imu.csv"),
            "broadcast_nav": Path("/tmp/brdc.nav"),
        }
        command = phase37._native_command(paths, "route/pixel5", Path("/tmp/run1"))
        self.assertEqual(command[-len(phase37.FLAGS) :], list(phase37.FLAGS))
        self.assertNotIn("ground_truth.csv", " ".join(command))
        self.assertNotIn(".mat", " ".join(command).lower())

    def test_speed_report_rejects_nonpositive_time(self) -> None:
        with self.assertRaises(phase37.Phase37Error):
            phase37.speed_report([(1, 37.0, -122.0), (1, 37.0, -122.0)])

    def test_speed_report_detects_transition_bound(self) -> None:
        report = phase37.speed_report([(0, 0.0, 0.0), (1000, 0.0, 0.001)])
        self.assertEqual(report["transition_count"], 1)
        self.assertGreater(report["max_speed_mps"], 70.0)
        self.assertEqual(report["over_70_mps_count"], 1)

    def test_read_raw_epoch_keys_collapses_satellite_rows(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "device_gnss.csv"
            path.write_text(
                "utcTimeMillis,other\n1000,a\n1000,b\n2000,c\n",
                encoding="utf-8",
            )
            self.assertEqual(phase37.read_raw_epoch_keys(path), [1000, 2000])

    def test_mat_paths_are_rejected_before_read(self) -> None:
        with self.assertRaises(phase37.Phase37Error):
            phase37._reject_mat(Path("/tmp/forbidden.mat"))

    def test_summary_contract_rejects_truth_used(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "summary.json"
            path.write_text('{"dataset_id":"route/pixel5","truth_used":true}', encoding="utf-8")
            with self.assertRaises(phase37.Phase37Error):
                phase37.validate_summary(path, "route/pixel5")


if __name__ == "__main__":
    unittest.main()
