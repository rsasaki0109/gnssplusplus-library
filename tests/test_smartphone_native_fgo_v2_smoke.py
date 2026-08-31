from __future__ import annotations

import csv
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_native_fgo_v2_smoke as V2  # noqa: E402


class NativeFgoV2SmokeTests(unittest.TestCase):
    def test_parity_parser_and_gate_require_real_imu_interval(self) -> None:
        stdout = (
            "FGOProblem: epochs=2 dd_pr_factors=3 dd_cp_factors=4 "
            "sd_doppler_factors=0 ambiguity_states=1 pr_factors=2\n"
            "IMU: loaded 20 samples\n"
            "  imu: 0.2 s, iters=2, imu_intervals=1, initial_cost=1.0, "
            "final_cost=0.5, graph_values=6, graph_factors=8\n"
            "  solve_ok=yes, nonfinite_epochs=0\n"
            "RESULT (milestone 2b): IMU-coupled solve GO\n"
        )
        parsed = V2.parse_parity_output(stdout)
        gate = V2.smoke_gate({"return_code": 0, "timed_out": False}, parsed)
        self.assertTrue(gate["passed"])
        self.assertEqual(parsed["imu_intervals"], 1)

        parsed["imu_intervals"] = 0
        self.assertFalse(V2.smoke_gate({"return_code": 0, "timed_out": False}, parsed)["passed"])

    def test_adapter_command_is_truth_free(self) -> None:
        command = V2._adapter_command(
            Path("inputs"),
            Path("adapter"),
            "route/phone",
            "phone",
            {"archive": {"url": "url", "source_terms": "terms"}},
        )
        self.assertIn("--truth-free", command)
        self.assertNotIn("--ground-truth", command)

    def test_android_schema_adapter_pairs_nearest_accel_and_converts_gyro(self) -> None:
        fields = [
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
        rows = [
            ("UncalAccel", 1_000, (0.0, 0.0, 9.8)),
            ("UncalAccel", 1_010, (0.0, 0.0, 9.8)),
            ("UncalGyro", 1_005, (1.0, 0.0, 0.0)),
            ("UncalGyro", 2_000, (0.1, 0.0, 0.0)),
        ]
        with tempfile.TemporaryDirectory(prefix="native_fgo_v2_test_") as directory:
            source = Path(directory) / "device_imu.csv"
            output = Path(directory) / "imu_processed.csv"
            with source.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields, lineterminator="\n")
                writer.writeheader()
                for message_type, timestamp, values in rows:
                    writer.writerow(
                        {
                            "MessageType": message_type,
                            "utcTimeMillis": timestamp,
                            "elapsedRealtimeNanos": timestamp * 1_000_000,
                            "MeasurementX": values[0],
                            "MeasurementY": values[1],
                            "MeasurementZ": values[2],
                            "BiasX": 0.0,
                            "BiasY": 0.0,
                            "BiasZ": 0.0,
                        }
                    )
            result = V2._convert_android_imu_to_loadable(source, output)
            self.assertEqual(result["paired_rows"], 1)
            self.assertEqual(result["omitted_no_nearest_or_outside_bound"], 1)
            self.assertIn("GPS TOW (s),GPS Week", output.read_text(encoding="ascii"))
            with output.open(encoding="ascii", newline="") as handle:
                processed = list(csv.DictReader(handle))
            self.assertAlmostEqual(float(processed[0]["Ang Rate X (deg/s)"]), 57.2957795, places=5)

    def test_offset_grid_is_fixed_and_bounded(self) -> None:
        gnss = {
            1_000_000: (0.0, 0.0, 0.0),
            1_200_000: (2.0, 0.0, 0.0),
            1_400_000: (2.0, 3.0, 0.0),
        }
        imu = [
            (0.0 + index * 0.01, float(index % 3))
            for index in range(100)
        ]
        report = V2.estimate_time_offset(gnss, imu)
        self.assertEqual(report["candidate_count"], 21)
        self.assertLessEqual(abs(float(report["estimated_offset_s"] or 0.0)), 0.1)
        self.assertTrue(report["solver_application"].startswith("not-applied"))


if __name__ == "__main__":
    unittest.main()
