#!/usr/bin/env python3

from __future__ import annotations

import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "apps" / "commands" / "benchmarks" / "gnss_performance_report.py"
SPEC = importlib.util.spec_from_file_location("gnss_performance_report", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
REPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPORT)


class GnssPerformanceReportTests(unittest.TestCase):
    def test_groups_gpst_rows_and_calculates_interval_metrics(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_performance_report_") as temp_dir:
            timing_path = Path(temp_dir) / "timing.csv"
            with timing_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=[
                        "epoch_index",
                        "gps_week",
                        "gps_tow_s",
                        "elapsed_ms",
                        "status",
                        "valid",
                        "num_satellites",
                        "iterations",
                        "processor_time_ms",
                    ],
                )
                writer.writeheader()
                writer.writerows(
                    [
                        {"epoch_index": 0, "gps_week": 2200, "gps_tow_s": 1000, "elapsed_ms": 1, "processor_time_ms": 0.5, "status": "1", "valid": 1, "num_satellites": 8, "iterations": 3},
                        {"epoch_index": 1, "gps_week": 2200, "gps_tow_s": 1010, "elapsed_ms": 2, "processor_time_ms": 1.0, "status": "1", "valid": 1, "num_satellites": 10, "iterations": 4},
                        {"epoch_index": 2, "gps_week": 2200, "gps_tow_s": 1061, "elapsed_ms": 3, "processor_time_ms": 1.5, "status": "0", "valid": 0, "num_satellites": 0, "iterations": 0},
                    ]
                )

            report = REPORT.build_report(timing_path, interval_s=60.0)

            self.assertEqual(report["input"]["row_count"], 3)
            self.assertEqual(len(report["intervals"]), 2)
            self.assertEqual(report["intervals"][0]["epoch_count"], 2)
            self.assertEqual(report["intervals"][0]["valid_epoch_count"], 2)
            self.assertEqual(report["intervals"][1]["status_counts"], {"0": 1})
            self.assertEqual(report["overall"]["solver_wall_time_s"], 0.006)
            self.assertEqual(report["overall"]["valid_rate"], round(2 / 3, 6))
            self.assertEqual(report["overall"]["processor_time_coverage"], 1.0)
            self.assertEqual(report["overall"]["mean_processor_time_ms"], 1.0)
            self.assertTrue(report["overall"]["realtime_factor"] > 0.0)

    def test_rejects_negative_or_nonfinite_elapsed_time(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_performance_report_") as temp_dir:
            timing_path = Path(temp_dir) / "timing.csv"
            timing_path.write_text(
                "gps_week,gps_tow_s,elapsed_ms\n2200,1000,-1\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "non-negative"):
                REPORT.build_report(timing_path)

    def test_summarizes_optional_rtk_stage_timings(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_performance_report_") as temp_dir:
            timing_path = Path(temp_dir) / "timing.csv"
            timing_path.write_text(
                "gps_week,gps_tow_s,elapsed_ms,stage_spp_ms,stage_filter_update_ms\n"
                "2200,1000,10,6,2\n"
                "2200,1001,12,8,4\n",
                encoding="utf-8",
            )
            report = REPORT.build_report(timing_path)
            stages = report["overall"]["stage_timings"]
            self.assertEqual(stages["stage_spp_ms"]["coverage"], 1.0)
            self.assertEqual(stages["stage_spp_ms"]["mean_ms"], 7.0)
            self.assertEqual(stages["stage_spp_ms"]["total_ms"], 14.0)
            self.assertEqual(stages["stage_filter_update_ms"]["p95_ms"], 3.9)
            self.assertNotIn("stage_velocity_ms", stages)

    def test_rejects_negative_rtk_stage_timing(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_performance_report_") as temp_dir:
            timing_path = Path(temp_dir) / "timing.csv"
            timing_path.write_text(
                "elapsed_ms,stage_spp_ms\n1,-0.1\n", encoding="utf-8"
            )
            with self.assertRaisesRegex(ValueError, "stage_spp_ms must be non-negative"):
                REPORT.build_report(timing_path)

    def test_empty_timing_file_is_valid_but_has_no_intervals(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_performance_report_") as temp_dir:
            timing_path = Path(temp_dir) / "timing.csv"
            timing_path.write_text("elapsed_ms\n", encoding="utf-8")
            report = REPORT.build_report(timing_path)
            self.assertEqual(report["input"]["row_count"], 0)
            self.assertEqual(report["intervals"], [])
            self.assertEqual(report["overall"]["epoch_count"], 0)

    def test_without_gpst_keeps_one_aggregate_without_realtime_claim(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_performance_report_") as temp_dir:
            timing_path = Path(temp_dir) / "timing.csv"
            timing_path.write_text(
                "elapsed_ms,valid\n1,1\n2,0\n", encoding="utf-8"
            )
            report = REPORT.build_report(timing_path)
            self.assertEqual(len(report["intervals"]), 1)
            self.assertFalse(report["intervals"][0]["timestamp_available"])
            self.assertIsNone(report["intervals"][0]["observation_span_s"])
            self.assertEqual(report["overall"]["realtime_factor"], 0.0)


if __name__ == "__main__":
    unittest.main()
