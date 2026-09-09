"""Regression checks for the sealed Phase95 path-corrected result.

Only the structural result, route metadata, and preserved logs are read.
These tests never open or hash raw GNSS/IMU/navigation, truth, MAT,
coordinate, base, or Kaggle artifacts.
"""

from __future__ import annotations

import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
RESULT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase95_raw_input_path_corrected_structural_result_v1.json"
)
MARKDOWN = RESULT.with_suffix(".md")
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
DOMAIN_ROWS = {
    ROUTES[0]: 2158,
    ROUTES[1]: 3139,
    ROUTES[2]: 1465,
    ROUTES[3]: 1101,
}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
FORBIDDEN_KEYS = {
    "latitude",
    "longitude",
    "position_ecef",
    "receiver_clock_bias",
    "epoch_clock_drift_mps",
    "epoch_velocity_nav_mps",
    "solutions",
}


def _forbidden_keys(value: object, prefix: str = "result") -> list[str]:
    found: list[str] = []
    if isinstance(value, dict):
        for key, child in value.items():
            if key.lower() in FORBIDDEN_KEYS:
                found.append(f"{prefix}.{key}")
            found.extend(_forbidden_keys(child, f"{prefix}.{key}"))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            found.extend(_forbidden_keys(child, f"{prefix}[{index}]"))
    return found


class Phase95PathCorrectedResultTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result = json.loads(RESULT.read_text(encoding="utf-8"))

    def test_result_is_phase95_structural_only_and_fail_closed(self) -> None:
        self.assertEqual(
            self.result["schema_version"],
            "smartphone-r5-phase95-raw-input-path-corrected-structural-result.v1",
        )
        self.assertEqual(self.result["phase"], 95)
        self.assertEqual(
            self.result["status"],
            "no-go-phase95-path-corrected-diagnostic-structural-gates",
        )
        self.assertTrue(self.result["diagnostic_only"])
        self.assertTrue(self.result["truth_free"])
        self.assertFalse(self.result["accuracy_scored"])
        self.assertFalse(self.result["solution_output_published"])
        self.assertTrue(self.result["stop_before_truth_accuracy_submission"])

    def test_exact_four_route_one_shot_accounting(self) -> None:
        self.assertEqual(list(self.result["routes"]), list(ROUTES))
        self.assertEqual(self.result["matrix"]["candidate_count"], 1)
        self.assertEqual(self.result["matrix"]["routes"], 4)
        self.assertEqual(self.result["matrix"]["runs_per_route"], 1)
        self.assertEqual(self.result["matrix"]["native_solver_invocations"], 4)
        self.assertEqual(self.result["matrix"]["raw_device_gnss_reads"], 4)
        self.assertEqual(self.result["matrix"]["raw_device_imu_reads"], 4)
        self.assertEqual(self.result["matrix"]["broadcast_navigation_reads"], 4)
        self.assertEqual(self.result["matrix"]["path_materializations"], 4)
        self.assertEqual(self.result["matrix"]["reruns"], 0)
        self.assertEqual(self.result["matrix"]["fallbacks"], 0)
        accounting = self.result["read_accounting"]
        self.assertEqual(accounting["native_solver_invocations"], 4)
        self.assertEqual(accounting["raw_device_gnss_reads"], 4)
        self.assertEqual(accounting["raw_device_imu_reads"], 4)
        self.assertEqual(accounting["broadcast_navigation_reads"], 4)
        self.assertEqual(accounting["runner_raw_input_byte_reads"], 0)
        self.assertFalse(accounting["raw_content_copied_or_transformed"])
        for key in (
            "truth_reads",
            "mat_reads_or_generated",
            "precomputed_coordinate_reads",
            "base_rinex_reads",
            "kaggle_or_token_access",
            "accuracy_calculations",
        ):
            self.assertEqual(accounting[key], 0)

    def test_each_route_has_exact_resolved_paths_and_full_stage_telemetry(self) -> None:
        for route in ROUTES:
            item = self.result["routes"][route]
            self.assertEqual(item["run_number"], 1)
            self.assertEqual(item["return_code"], 1)
            self.assertTrue(item["native_process_completed"])
            self.assertTrue(item["diagnostic_return_code_fail_closed"])
            self.assertTrue(item["summary"]["present"])
            self.assertFalse(item["withheld_solution_output_present"])
            self.assertFalse(item["solution_output_published"])
            for name in RAW_NAMES:
                raw = item["raw_inputs"][name]
                self.assertTrue(raw["exists_before_launch"])
                self.assertFalse(raw["read_by_runner"])
                self.assertTrue(raw["sha256_available"])
                self.assertIsInstance(raw["bytes"], int)
                self.assertGreater(raw["bytes"], 0)
                self.assertNotIn("raw/phase93/", raw["path"])
                self.assertNotIn("truth", raw["path"].lower())
            command = item["command"]
            for flag, name in (
                ("--android-gnss", "device_gnss.csv"),
                ("--android-imu", "device_imu.csv"),
                ("--nav", "brdc.nav"),
            ):
                resolved = command[command.index(flag) + 1]
                self.assertEqual(resolved, item["raw_inputs"][name]["path"])
                self.assertNotIn("raw/phase93/", resolved)
            expected = item["expected_output"]
            self.assertEqual(expected["domain_rows"], DOMAIN_ROWS[route])
            self.assertEqual(expected["problem_epochs"], DOMAIN_ROWS[route] + 1)
            self.assertFalse(expected["published"])

            telemetry = item["telemetry"]
            preflight = telemetry["preflight"]
            self.assertTrue(preflight["required_fields_present"]["gnss_first"])
            self.assertTrue(preflight["required_fields_present"]["main"])
            self.assertIsInstance(preflight["gnss_first"]["guard_predicate"], str)
            gnss = telemetry["gnss_first"]
            main = telemetry["main"]
            for mapping in (gnss, main):
                self.assertIn("telemetry", mapping)
                self.assertIn("required_fields_present", mapping)
                self.assertTrue(mapping["required_fields_present"])
                self.assertIn("initial_cost", mapping)
                self.assertIn("final_cost", mapping)
                self.assertIn("accepted_outer_iterations", mapping)
            self.assertEqual(telemetry["c0d_unit_contract"]["clock_C_and_ISB"], "metres")
            self.assertEqual(telemetry["c0d_unit_contract"]["drift_D"], "metres_per_second")
            self.assertEqual(telemetry["c0d_unit_contract"]["ordinary_sigma_m"], 0.1)

    def test_logs_and_structural_result_never_publish_solution_values(self) -> None:
        for route in ROUTES:
            item = self.result["routes"][route]
            stdout = ROOT / item["stdout"]["path"]
            stderr = ROOT / item["stderr"]["path"]
            self.assertTrue(stdout.is_file())
            self.assertTrue(stderr.is_file())
            self.assertEqual(stdout.stat().st_size, 0)
            self.assertGreater(stderr.stat().st_size, 0)
        self.assertEqual(_forbidden_keys(self.result), [])
        self.assertIn("Phase95", MARKDOWN.read_text(encoding="utf-8"))
        self.assertIn("withheld", MARKDOWN.read_text(encoding="utf-8").lower())
        self.assertNotIn("accuracy score", MARKDOWN.read_text(encoding="utf-8").lower())


if __name__ == "__main__":
    unittest.main()
