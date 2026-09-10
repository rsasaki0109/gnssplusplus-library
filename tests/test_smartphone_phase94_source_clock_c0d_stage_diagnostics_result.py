"""Regression checks for the sealed Phase94 diagnostic raw result.

The test reads only the generated structural result, route metadata, and
native logs.  It never opens, hashes, or executes a raw GNSS/IMU/navigation,
truth, MAT, coordinate, base, or Kaggle artifact.
"""

from __future__ import annotations

import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
RESULT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase94_source_clock_c0d_stage_diagnostics_structural_result_v1.json"
)
MARKDOWN = RESULT.with_suffix(".md")
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
FORBIDDEN_VALUE_KEYS = {
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
            if key.lower() in FORBIDDEN_VALUE_KEYS:
                found.append(f"{prefix}.{key}")
            found.extend(_forbidden_keys(child, f"{prefix}.{key}"))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            found.extend(_forbidden_keys(child, f"{prefix}[{index}]"))
    return found


class Phase94StructuralResultTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result = json.loads(RESULT.read_text(encoding="utf-8"))

    def test_result_is_fail_closed_and_never_accuracy_scored(self) -> None:
        self.assertEqual(
            self.result["schema_version"],
            "smartphone-r5-phase94-source-clock-c0d-stage-diagnostics-structural-result.v1",
        )
        self.assertEqual(self.result["phase"], 94)
        self.assertEqual(self.result["status"], "no-go-phase94-diagnostic-structural-gates")
        self.assertTrue(self.result["diagnostic_only"])
        self.assertTrue(self.result["truth_free"])
        self.assertFalse(self.result["accuracy_scored"])
        self.assertFalse(self.result["solution_output_published"])
        self.assertTrue(self.result["stop_before_truth_accuracy_submission"])
        self.assertEqual(
            self.result["forbidden_lanes"],
            {
                "truth": False,
                "MAT": False,
                "precomputed_coordinates": False,
                "base": False,
                "Kaggle_or_token": False,
                "accuracy": False,
            },
        )

    def test_exact_four_route_one_run_matrix_is_preserved(self) -> None:
        self.assertEqual(
            list(self.result["routes"]),
            list(ROUTES),
        )
        self.assertEqual(
            self.result["matrix"],
            {
                "accuracy_calculations": 0,
                "base_rinex_reads": 0,
                "broadcast_navigation_route_arguments": 4,
                "candidate_count": 1,
                "candidate_runs_total": 4,
                "controls": 0,
                "fallbacks": 0,
                "kaggle_or_token_access": 0,
                "mat_reads_or_generated": 0,
                "native_solver_invocations": 4,
                "precomputed_coordinate_reads": 0,
                "raw_device_gnss_route_arguments": 4,
                "raw_device_imu_route_arguments": 4,
                "reruns": 0,
                "route_score_selection": False,
                "routes": 4,
                "runner_raw_byte_reads": 0,
                "runs_per_route": 1,
                "truth_reads": 0,
            },
        )
        self.assertTrue(self.result["gates"]["exactly_four_routes_one_run_each"])
        self.assertEqual(self.result["read_accounting"]["reruns"], 0)
        self.assertEqual(self.result["read_accounting"]["fallbacks"], 0)

    def test_each_route_failure_and_stage_telemetry_are_sealed(self) -> None:
        for route in ROUTES:
            item = self.result["routes"][route]
            self.assertEqual(item["run_number"], 1)
            self.assertEqual(item["return_code"], 1)
            self.assertTrue(item["native_process_completed"])
            self.assertTrue(item["diagnostic_return_code_fail_closed"])
            self.assertFalse(item["summary"]["present"])
            self.assertFalse(item["withheld_solution_output_present"])
            self.assertFalse(item["solution_output_published"])
            self.assertEqual(item["failure_stage"], "native-launch-or-pre-summary")
            self.assertIn("native summary was not produced", item["failure"])
            stages = item["stage_telemetry"]
            self.assertTrue(stages["input_conversion"]["reached"])
            self.assertEqual(stages["input_conversion"]["status"], "failed-closed")
            self.assertIn("failed to open raw Android GNSS CSV", stages["input_conversion"]["stderr_observation"])
            self.assertFalse(stages["problem_build"]["reached"])
            self.assertFalse(stages["gnss_first_preflight"]["reached"])
            self.assertIsNone(stages["gnss_first_preflight"]["retained_epoch_count"])
            self.assertIsNone(stages["gnss_first_preflight"]["eligible_c0d_factor_count"])
            self.assertIsNone(stages["gnss_first_preflight"]["guard_predicate"])
            self.assertFalse(stages["gnss_first_optimize"]["reached"])
            self.assertIsNone(stages["gnss_first_optimize"]["accepted_outer_iterations"])
            self.assertIsNone(stages["gnss_first_optimize"]["initial_cost"])
            self.assertFalse(stages["exact_key_handoff"]["reached"])
            self.assertFalse(stages["main_preflight"]["reached"])
            self.assertIsNone(stages["main_preflight"]["position_solution_size"])
            self.assertIsNone(stages["main_preflight"]["receiver_clock_solution_size"])
            self.assertIsNone(stages["main_preflight"]["optimized_d_size_matches_problem_epochs"])
            self.assertFalse(stages["main_optimize"]["reached"])
            self.assertIsNone(stages["main_optimize"]["accepted_outer_iterations"])
            self.assertIsNone(stages["main_optimize"]["initial_cost"])
            self.assertIsNone(stages["main_optimize"]["initial_lambda"])
            self.assertIsNone(stages["main_optimize"]["conditioning_proxy"])
            self.assertFalse(stages["coverage_contract"]["reached"])
            telemetry = item["telemetry"]
            self.assertEqual(telemetry["status"], "unavailable")
            self.assertIsNone(telemetry["preflight"]["gnss_first"])
            self.assertIsNone(telemetry["preflight"]["main"])
            self.assertIsNone(telemetry["gnss_first"]["telemetry"])
            self.assertIsNone(telemetry["main"]["telemetry"])
            self.assertEqual(telemetry["c0d_unit_contract"]["ordinary_sigma_m"], 0.1)
            self.assertEqual(telemetry["c0d_unit_contract"]["clock_C_and_ISB"], "metres")
            self.assertEqual(telemetry["c0d_unit_contract"]["drift_D"], "metres_per_second")
            self.assertFalse(item["gates"]["gnss_first_guard_predicate_and_counts"])
            self.assertFalse(item["gates"]["main_validation_predicates"])
            self.assertTrue(item["gates"]["no_solution_or_accuracy_publication"])

    def test_logs_are_preserved_without_reading_raw_inputs(self) -> None:
        for route in ROUTES:
            item = self.result["routes"][route]
            stderr = ROOT / item["stderr"]["path"]
            stdout = ROOT / item["stdout"]["path"]
            self.assertTrue(stderr.is_file())
            self.assertTrue(stdout.is_file())
            self.assertGreater(stderr.stat().st_size, 0)
            self.assertEqual(stdout.stat().st_size, 0)
            self.assertFalse(item["raw_inputs"]["device_gnss.csv"]["read_by_runner"])
            self.assertFalse(item["raw_inputs"]["device_imu.csv"]["read_by_runner"])
            self.assertFalse(item["raw_inputs"]["brdc.nav"]["read_by_runner"])
            self.assertEqual(
                self.result["read_accounting"]["runner_raw_input_byte_reads"],
                0,
            )

    def test_no_solution_values_are_embedded_in_structural_result(self) -> None:
        self.assertEqual(_forbidden_keys(self.result), [])
        self.assertIn("GNSS", MARKDOWN.read_text(encoding="utf-8"))
        self.assertIn("withheld", MARKDOWN.read_text(encoding="utf-8"))
        self.assertNotIn("accuracy score", MARKDOWN.read_text(encoding="utf-8").lower())


if __name__ == "__main__":
    unittest.main()
