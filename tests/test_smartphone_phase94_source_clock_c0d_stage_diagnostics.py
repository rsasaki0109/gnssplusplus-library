"""Synthetic/focused tests for the Phase94 stage-local diagnostic lane.

These tests inspect source and the sealed freeze only.  They do not open,
hash, or execute raw GNSS/IMU/navigation, truth, MAT, coordinate, or Kaggle
artifacts.
"""

from __future__ import annotations

import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase94_source_clock_c0d_stage_diagnostics_freeze_v1.json"
)


def _c0d_guard(backend_allowed: bool, d_rows: int, epochs: int) -> bool:
    """The frozen GNSS-first backend admission predicate, synthetically."""

    source_path = d_rows > 0
    return not (backend_allowed and source_path and epochs >= 2)


class Phase94StageDiagnosticsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = APP.read_text(encoding="utf-8")
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))

    def test_selector_is_default_off_and_candidate_is_diagnostic_only(self) -> None:
        self.assertIn(
            "native_source_clock_c0d_phase94_stage_diagnostics = false",
            self.app,
        )
        self.assertIn(
            "--native-source-clock-c0d-phase94-stage-diagnostics",
            self.app,
        )
        candidate = self.freeze["candidate"]
        self.assertTrue(candidate["exactly_one"])
        self.assertEqual(
            candidate["id"],
            "phase94_source_clock_c0d_stage_admission_and_failure_telemetry",
        )
        self.assertTrue(candidate["default_off"])
        self.assertTrue(candidate["raw_execution_authorized"] is False)
        self.assertIn("captured-solution-output-withheld", self.app)
        self.assertIn("solution/accuracy output withheld", self.app)

    def test_preflight_records_route2_guard_disjuncts(self) -> None:
        for term in (
            "retained_epoch_count",
            "retained_undifferenced_doppler_factor_count",
            "eligible_c0d_pair_count",
            "eligible_c0d_factor_count",
            "d_initializer_coverage_valid",
            "d_initializer_all_finite",
            "gnss_first_problem.undifferenced_doppler_factors.empty()",
            "gnss_first_problem.epochs.size() < 2",
            "backend_configuration_not_allowed",
            "source_clock_c0d_problem_path_unavailable",
            "guard_predicate",
        ):
            self.assertIn(term, self.app)

    def test_synthetic_guard_truth_table_is_fail_closed(self) -> None:
        self.assertFalse(_c0d_guard(True, 3, 10))
        self.assertTrue(_c0d_guard(True, 0, 10))
        self.assertTrue(_c0d_guard(True, 3, 1))
        self.assertTrue(_c0d_guard(False, 3, 10))

    def test_main_subpredicates_and_failure_artifact_are_structured(self) -> None:
        for term in (
            "position_size_matches_problem_epochs",
            "receiver_clock_solution_size",
            "receiver_clock_size_matches_problem_epochs",
            "all_positions_earth_valid",
            "all_receiver_clocks_finite",
            "optimized_d_size_matches_problem_epochs",
            "optimized_d_all_finite",
            "velocity_size_matches_problem_epochs",
            "all_velocities_finite",
            "exact_retained_key_alignment",
            "gnss_first_progress",
            "accepted_outer_iterations",
            "inner_lambda_attempts",
            "initial_cost",
            "final_cost_strictly_less_than_initial",
            "conditioning_proxy",
            "terminal_branch",
            "writePhase94StageDiagnostics",
            "status = \"failed-closed\"",
        ):
            self.assertIn(term, self.app)

    def test_c0d_equation_units_and_sigma_sources_are_untouched(self) -> None:
        backend = (ROOT / "src/algorithms/fgo_gtsam_backend.cpp").read_text(
            encoding="utf-8"
        )
        internal = (ROOT / "src/algorithms/fgo_gtsam_internal.hpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("kNativeSourceClockC0DSigmaM = 0.1", internal)
        self.assertIn("SourceClockC0DFactor", internal)
        self.assertIn("use_native_source_clock_c0d_meter_state", backend)
        self.assertIn("nativeSourceClockC0DEdgeDecision", backend)
        self.assertIn("native-source-clock-c0d-phase94-stage-diagnostics", self.app)

    def test_phase94_has_no_solution_coordinates_or_accuracy_lane(self) -> None:
        self.assertIn("raw_truth_mat_kaggle_forbidden", self.app)
        self.assertIn("solution_output_published = false", self.app)
        self.assertIn("accuracy_output_published = false", self.app)
        self.assertIn("truth_used = false", self.app)
        self.assertIn("mat_used = false", self.app)
        self.assertIn("kaggle_or_token_accessed = false", self.app)


if __name__ == "__main__":
    unittest.main()
