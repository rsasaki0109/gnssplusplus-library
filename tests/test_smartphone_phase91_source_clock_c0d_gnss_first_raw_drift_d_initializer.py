"""Focused pre-raw tests for the Phase91 execution freeze/evaluator."""

from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase91_source_clock_c0d_gnss_first_raw_drift_d_initializer.py"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase91_source_clock_c0d_gnss_first_raw_drift_d_initializer_execution_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase91_source_clock_c0d_gnss_first_raw_drift_d_initializer_execution_manifest_v1.json"

_SPEC = importlib.util.spec_from_file_location("phase91_raw_d_initializer", RUNNER_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load evaluator: {RUNNER_PATH}")
RUNNER = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(RUNNER)


def _command(route: str) -> list[str]:
    manifest = RUNNER._read_json(MANIFEST, "Phase91 manifest")
    return next(item["command"] for item in manifest["routes"] if item["dataset_id"] == route)


def _summary(route: str, *, iterations: int = 2, initial: float = 10.0, final: float = 5.0) -> dict:
    rows = RUNNER.DOMAIN_ROWS[route]
    expected_epochs = rows + 1
    reason = "small_cost_change" if iterations else "no_inner_iteration"
    attempts = 3 if iterations else 0
    return {
        "schema_version": "smartphone-r5-native-fgo-android-imu-no-base-run.v3",
        "dataset_id": route,
        "status": "imu-combined-factor",
        "truth_used": False,
        "production_default_changed": False,
        "native_pdc_state_bridge": False,
        "native_pdc_imu_tdcp": True,
        "native_pdc_imu_tdcp_no_bridge": True,
        "native_upstream_quality": False,
        "native_source_direct_observable_quality_enabled": True,
        "native_source_clock_c0d_factor_enabled": True,
        "native_source_clock_c0d_active_solve_diagnostic_enabled": True,
        "native_source_clock_c0d_gnss_first_raw_drift_d_initializer_enabled": True,
        "inputs": {
            "observation": None,
            "navigation": "raw/brdc.nav",
            "imu": "raw/device_imu.csv",
            "android_gnss": "raw/device_gnss.csv",
        },
        "android_gnss_diagnostics": {"no_device_wls_seed": True},
        "imu_initialization": {"input_format": "android-device_imu.csv"},
        "epochs": {"problem": expected_epochs, "output": expected_epochs, "pseudorange_factors": rows},
        "graph": {"imu_intervals": rows, "converged": True, "iterations": iterations, "initial_cost": initial, "final_cost": final},
        "tdcp_contract": {"enabled": True, "factors_built": rows, "factors_inserted": rows, "finite_residuals": rows, "nonfinite_residuals": 0},
        "raw_utc_key_contract": {
            "warmup_epoch_excluded": True,
            "raw_epoch_keys": expected_epochs,
            "target_epochs": rows,
            "exact_solution_epochs": rows,
            "interpolated_epochs": 0,
            "edge_hold_epochs": 0,
            "unresolved_epochs": 0,
            "device_wls_coordinates_used": False,
        },
        "native_source_direct_observable_quality": {"enabled": True, "direct_no_pdc": True, "pdc_bridge": False, "native_pdc_state_bridge": False},
        "gnss_first": {
            "attempted": True,
            "converged": True,
            "handoff_mode": "gnss-first-in-memory-position-clock-velocity",
            "velocity_handoff_source": "same-run-gnss-first-optimizer-result",
            "position_clock_handoff_source": "same-run-gnss-first-optimizer-result",
            "raw_drift_d_initializer": "main-graph-only",
            "epoch_identity_alignment_valid": True,
            "main_epoch_count": expected_epochs,
            "gnss_first_epoch_count": expected_epochs,
            "solution_epoch_count": expected_epochs,
            "raw_epoch_count": expected_epochs,
            "raw_utc_key_count": expected_epochs,
            "aligned_epoch_count": expected_epochs,
            "epoch_count_mismatch_count": 0,
            "nonfinite_time_count": 0,
            "gnss_first_time_mismatch_count": 0,
            "raw_time_mismatch_count": 0,
            "raw_utc_key_order_mismatch_count": 0,
            "duplicate_raw_utc_key_count": 0,
            "raw_utc_key_mismatch_count": 0,
            "nonfinite_solution_count": 0,
            "positions_clocks_copied": expected_epochs,
            "coordinates_source": "in-memory GNSS-first result only",
            "forbidden_coordinate_sources": ["PDC", "direct-WLS", "external", "precomputed"],
            "failure": "",
        },
        "native_source_clock_c0d_factor": {
            "clock_c0d_enabled": True,
            "clock_c0d_factor_count": rows,
            "raw_drift_d_initializer": {
                "enabled": True,
                "attempted": True,
                "coverage_valid": True,
                "epoch_count": expected_epochs,
                "finite_count": expected_epochs,
                "nonfinite_count": 0,
                "min_mps": 0.1,
                "max_mps": 0.2,
                "source_field": "EpochSeed.receiver_clock_drift_mps",
                "units": "metres_per_second",
                "fallback": "none",
                "failure": "",
            },
            "active_solve_diagnostic_enabled": True,
            "active_solve_attempted": True,
            "active_solve_initial_cost": initial,
            "active_solve_final_cost": final,
            "accepted_outer_iterations": iterations,
            "total_inner_lambda_attempts": attempts,
            "initial_lambda": 1.0,
            "maximum_lambda": 2.0,
            "final_lambda": 1.0,
            "indeterminate_linear_solve_count": 0,
            "unsuccessful_model_step_count": max(0, attempts - iterations),
            "small_cost_change_stop_count": 1 if iterations else 0,
            "maximum_lambda_stop_count": 0,
            "active_solve_finite_costs": True,
            "termination_trace_complete": True,
            "termination_branch_reason": reason,
            "max_whitened_clock_column_norm": 2.0,
            "max_whitened_drift_column_norm": 3.0,
            "conditioning_proxy": 0.6666666666666666,
            "clock_c0d_clock_jump_skips": 0,
            "clock_c0d_gap_skips": 0,
            "clock_c0d_invalid_dt_skips": 0,
            "clock_c0d_phone_exclusion_skips": 0,
            "clock_c0d_dt_min_s": 1.0,
            "clock_c0d_dt_max_s": 1.0,
            "clock_c0d_equation": "(c2-c1)-((d1+d2)*dt/(2*C_LIGHT))",
            "clock_c0d_jacobian_order": ["c1", "c2", "d1", "d2"],
            "clock_c0d_jacobian": "[-1,+1,-dt/(2*C_LIGHT),-dt/(2*C_LIGHT)]",
            "clock_c0d_units": {"clock": "seconds", "drift": "metres_per_second", "dt": "seconds", "residual": "seconds", "sigma": "seconds"},
            "speed_of_light_mps": RUNNER.SPEED_OF_LIGHT_MPS,
            "clock_c0d_sigma_seconds": RUNNER.C0D_SIGMA_SECONDS,
            "clock_jump_noise": "Inf (active C0 factor omitted)",
            "legacy_scalar_clock_between_factor_count": 0,
            "parity_scope": "C0/D active-row parity; not full seven-vector",
        },
    }


class Phase91ExecutionTests(unittest.TestCase):
    def test_sealed_freeze_manifest_and_exact_four_route_matrix(self) -> None:
        self.assertEqual(hashlib.sha256(FREEZE.read_bytes()).hexdigest(), RUNNER.FREEZE_SHA256)
        freeze = RUNNER.verify_freeze()
        self.assertEqual(freeze["status"], "frozen-before-phase91-raw-execution")
        manifest = RUNNER._read_json(MANIFEST, "Phase91 manifest")
        RUNNER._verify_manifest(manifest)
        self.assertEqual([item["dataset_id"] for item in manifest["routes"]], list(RUNNER.ROUTES))
        self.assertEqual([item["runs"] for item in manifest["routes"]], [1, 1, 1, 1])
        self.assertEqual(manifest["matrix"]["native_invocations"], 4)
        self.assertFalse(manifest["matrix"]["accuracy_scored"])
        self.assertEqual(manifest["read_accounting_at_manifest_creation"]["raw_device_gnss_reads"], 0)

    def test_commands_are_raw_only_and_new_selector_is_exactly_once(self) -> None:
        manifest = RUNNER._read_json(MANIFEST, "Phase91 manifest")
        for route in RUNNER.ROUTES:
            command = _command(route)
            self.assertEqual(command[0], "build/apps/gnss_fgo_imu_no_base")
            self.assertEqual(command.count(RUNNER.SELECTOR), 1)
            self.assertEqual(command.count(RUNNER.CLOCK_FLAG), 1)
            self.assertEqual(command.count(RUNNER.ACTIVE_FLAG), 1)
            self.assertNotIn("--native-gnss-first-velocity-only-handoff", command)
            self.assertNotIn("--native-direct-doppler-wls-handoff", command)
            self.assertNotIn("--native-pdc-state-bridge", command)
        self.assertEqual(len(manifest["routes"]), 4)

    def test_accepted_synthetic_summary_has_aligned_handoff_and_active_progress(self) -> None:
        route = RUNNER.ROUTES[0]
        diagnostics = RUNNER._validate_summary(_summary(route), route, RUNNER.DOMAIN_ROWS[route], _command(route))
        self.assertEqual(diagnostics["gnss_first"]["aligned_epoch_count"], RUNNER.DOMAIN_ROWS[route] + 1)
        self.assertEqual(diagnostics["raw_drift_d_initializer"]["source_field"], "EpochSeed.receiver_clock_drift_mps")
        self.assertGreaterEqual(diagnostics["graph"]["iterations"], 1)
        self.assertLess(diagnostics["graph"]["final_cost"], diagnostics["graph"]["initial_cost"])

    def test_zero_iterations_and_equal_cost_are_material_no_go_gates(self) -> None:
        route = RUNNER.ROUTES[0]
        summary = _summary(route, iterations=0, initial=10.0, final=10.0)
        diagnostics = RUNNER._validate_summary(summary, route, RUNNER.DOMAIN_ROWS[route], _command(route))
        self.assertEqual(diagnostics["graph"]["iterations"], 0)
        self.assertFalse(diagnostics["graph"]["iterations"] >= 1)
        self.assertFalse(diagnostics["graph"]["final_cost"] < diagnostics["graph"]["initial_cost"])


if __name__ == "__main__":
    unittest.main()
