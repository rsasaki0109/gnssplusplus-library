"""Read-only checks for the sealed Phase99 structural result.

This test reads JSON/log metadata only.  It never opens the withheld solution
CSV and performs no truth or accuracy operation.
"""

from __future__ import annotations

import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase99_main_multifrontal_qr_structural_result_v1.json"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)


class Phase99MainMultifrontalQrResultTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = json.loads(RESULT.read_text(encoding="utf-8"))

    def test_result_is_structural_go_and_exact_matrix(self):
        self.assertEqual(self.result["status"], "go-phase99-main-multifrontal-qr-structural")
        self.assertEqual(list(self.result["routes"]), list(ROUTES))
        self.assertEqual(self.result["matrix"]["routes"], 2)
        self.assertEqual(self.result["matrix"]["runs_per_route"], 1)
        self.assertEqual(self.result["matrix"]["native_solver_invocations"], 2)
        self.assertEqual(self.result["matrix"]["reruns"], 0)
        self.assertEqual(self.result["matrix"]["fallbacks"], 0)

    def test_all_top_level_structural_gates_pass(self):
        gates = self.result["gates"]
        self.assertTrue(gates["all_passed"])
        self.assertTrue(gates["all_structural_gates_passed"])
        for key, value in gates.items():
            if key not in ("all_passed", "all_structural_gates_passed"):
                self.assertTrue(value, key)

    def test_each_route_has_progress_qr_and_finite_exact_handoff(self):
        for route in ROUTES:
            item = self.result["routes"][route]
            self.assertEqual(item["run_number"], 1)
            self.assertEqual(item["return_code"], 1)
            self.assertTrue(item["summary_present"])
            self.assertFalse(item["withheld_solution_output_present"])
            self.assertTrue(item["gates"]["gnss_first_strict_progress"])
            self.assertTrue(item["gates"]["gnss_first_full_finite_d_exact_handoff"])
            self.assertTrue(item["gates"]["main_selected_multifrontal_qr"])
            self.assertTrue(item["gates"]["main_accepted_outer_iterations"])
            self.assertTrue(item["gates"]["main_strict_cost_decrease"])
            self.assertTrue(item["gates"]["main_finite_expected_coverage"])
            self.assertTrue(item["gates"]["no_fallback_or_solution_publication"])
            self.assertEqual(item["phase98_solver"]["solver_type"], "MULTIFRONTAL_QR")
            self.assertEqual(item["phase98_solver"]["solver_branch"], "multifrontal")
            self.assertEqual(item["phase98_solver"]["elimination_function"], "EliminateQR")

    def test_costs_and_epoch_counts_are_structural_metadata_only(self):
        expected_rows = {ROUTES[0]: 2158, ROUTES[1]: 1465}
        for route in ROUTES:
            item = self.result["routes"][route]
            gnss = item["gnss_first"]
            main = item["main"]
            expected_epochs = expected_rows[route] + 1
            self.assertEqual(gnss["epochs"], expected_epochs)
            self.assertLess(gnss["final_cost"], gnss["initial_cost"])
            self.assertEqual(gnss["optimized_d_epoch_count"], expected_epochs)
            self.assertEqual(gnss["optimized_d_finite_count"], expected_epochs)
            self.assertTrue(gnss["exact_retained_key_alignment"])
            self.assertEqual(main["problem_epoch_count"], expected_epochs)
            self.assertEqual(main["position_solution_size"], expected_epochs)
            self.assertEqual(main["optimized_d_epoch_count"], expected_epochs)
            self.assertEqual(main["velocity_epoch_count"], expected_epochs)
            self.assertLess(main["final_cost"], main["initial_cost"])

    def test_read_accounting_and_publication_boundary_are_closed(self):
        accounting = self.result["read_accounting"]
        for key in (
            "runner_raw_input_byte_reads", "raw_input_hash_reads", "truth_reads",
            "mat_reads_or_generated", "precomputed_coordinate_reads",
            "base_rinex_reads", "accuracy_calculations", "kaggle_or_token_access",
            "reruns", "fallbacks",
        ):
            self.assertEqual(accounting[key], 0, key)
        self.assertFalse(accounting["raw_content_copied_or_transformed"])
        self.assertFalse(self.result["solution_output_published"])
        self.assertFalse(self.result["accuracy_scored"])
        self.assertTrue(self.result["stop_before_truth_accuracy_submission"])
        self.assertFalse(self.result["forbidden_lanes"]["solution_rows"])

    def test_withheld_csv_is_never_read_or_committed_as_solution_data(self):
        for route in ROUTES:
            path = ROOT / self.result["routes"][route]["expected_output"]["withheld_solution_output"]
            self.assertFalse(path.is_file())


if __name__ == "__main__":
    unittest.main()
