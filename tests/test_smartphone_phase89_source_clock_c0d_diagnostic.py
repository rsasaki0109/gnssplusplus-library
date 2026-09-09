"""Focused synthetic tests for the Phase89 raw-free structural evaluator."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase89_source_clock_c0d_diagnostic.py"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase89_source_clock_c0d_diagnostic_execution_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase89_source_clock_c0d_diagnostic_execution_manifest_v1.json"

_SPEC = importlib.util.spec_from_file_location("phase89_c0d_diagnostic", RUNNER_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load runner: {RUNNER_PATH}")
RUNNER = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(RUNNER)


def _command(route: str) -> list[str]:
    return [
        "build/apps/gnss_fgo_imu_no_base",
        "--dataset-id", route,
        "--android-gnss", "raw/device_gnss.csv",
        "--android-imu", "raw/device_imu.csv",
        "--nav", "raw/brdc.nav",
        "--all-epochs",
        "--android-raw-utc-keys",
        "--native-pdc-imu-tdcp-no-bridge",
        "--native-source-direct-observable-quality",
        "--native-source-clock-c0d-factor",
        "--native-gnss-first-velocity-only-handoff",
        "--native-source-clock-c0d-active-solve-diagnostic",
    ]


def _summary(route: str, *, iterations: int = 2, initial: float = 10.0, final: float = 5.0) -> dict:
    rows = RUNNER.DOMAIN_ROWS[route]
    reason = "small_cost_change" if iterations else "no_inner_iteration"
    attempts = 3 if iterations else 0
    return {
        "schema_version": "smartphone-r5-native-fgo-android-imu-no-base-run.v3",
        "dataset_id": route,
        "status": "imu-combined-factor",
        "truth_used": False,
        "production_default_changed": False,
        "native_pdc_state_bridge": False,
        "native_pdc_imu_tdcp_no_bridge": True,
        "native_upstream_quality": False,
        "native_source_direct_observable_quality_enabled": True,
        "native_source_clock_c0d_factor_enabled": True,
        "native_source_clock_c0d_active_solve_diagnostic_enabled": True,
        "inputs": {
            "observation": None,
            "navigation": "raw/brdc.nav",
            "imu": "raw/device_imu.csv",
            "android_gnss": "raw/device_gnss.csv",
        },
        "android_gnss_diagnostics": {"no_device_wls_seed": True},
        "imu_initialization": {"input_format": "android-device_imu.csv"},
        "epochs": {
            "problem": rows + 1,
            "output": rows + 1,
            "pseudorange_factors": rows,
        },
        "graph": {
            "imu_intervals": rows,
            "converged": True,
            "iterations": iterations,
            "initial_cost": initial,
            "final_cost": final,
        },
        "tdcp_contract": {
            "enabled": True,
            "factors_built": rows,
            "factors_inserted": rows,
            "finite_residuals": rows,
            "nonfinite_residuals": 0,
        },
        "raw_utc_key_contract": {
            "warmup_epoch_excluded": True,
            "raw_epoch_keys": rows + 1,
            "target_epochs": rows,
            "exact_solution_epochs": rows,
            "interpolated_epochs": 0,
            "edge_hold_epochs": 0,
            "unresolved_epochs": 0,
            "device_wls_coordinates_used": False,
        },
        "native_source_direct_observable_quality": {
            "enabled": True,
            "direct_no_pdc": True,
            "pdc_bridge": False,
            "native_pdc_state_bridge": False,
        },
        "gnss_first": {
            "attempted": True,
            "handoff_mode": "velocity-only",
            "positions_clocks_copied": 0,
            "velocity_handoff_source": "gnss-first-optimizer-result",
            "velocity_valid_count": rows + 1,
            "velocity_nonfinite_count": 0,
            "velocity_over_70_mps_count": 0,
        },
        "native_source_clock_c0d_factor": {
            "clock_c0d_enabled": True,
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
            "clock_c0d_factor_count": rows,
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


def _write_submission(path: Path, route: str, rows: int) -> None:
    values = ["phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees"]
    values.extend(f"{route},{1000 + index * 1000},37.0,-122.0" for index in range(rows))
    path.write_text("\n".join(values) + "\n", encoding="utf-8")


class Phase89DiagnosticTests(unittest.TestCase):
    def test_freeze_and_manifest_are_sealed_and_exactly_four_routes(self) -> None:
        self.assertEqual(hashlib.sha256(FREEZE.read_bytes()).hexdigest(), RUNNER.FREEZE_SHA256)
        freeze = RUNNER.verify_freeze()
        self.assertEqual(freeze["phase"], 89)
        self.assertEqual(freeze["status"], "frozen-before-phase89-raw-execution")
        manifest = RUNNER._read_json(MANIFEST, "Phase89 manifest")
        RUNNER._verify_manifest(manifest)
        self.assertEqual([item["dataset_id"] for item in manifest["routes"]], list(RUNNER.ROUTES))
        self.assertEqual([item["runs"] for item in manifest["routes"]], [1, 1, 1, 1])
        self.assertEqual(manifest["matrix"]["native_invocations"], 4)
        self.assertEqual(manifest["matrix"]["truth_reads"], 0)

    def test_valid_synthetic_summary_reports_complete_active_solve(self) -> None:
        route = RUNNER.ROUTES[0]
        diagnostics = RUNNER._validate_summary(_summary(route), route, RUNNER.DOMAIN_ROWS[route], _command(route))
        self.assertEqual(diagnostics["graph"]["iterations"], 2)
        self.assertEqual(diagnostics["clock_c0d"]["termination_branch_reason"], "small_cost_change")

    def test_zero_iterations_and_equal_cost_fail_the_materiality_gates(self) -> None:
        route = RUNNER.ROUTES[0]
        summary = _summary(route, iterations=0, initial=10.0, final=10.0)
        diagnostics = RUNNER._validate_summary(summary, route, RUNNER.DOMAIN_ROWS[route], _command(route))
        self.assertEqual(diagnostics["graph"]["iterations"], 0)
        self.assertFalse(diagnostics["graph"]["final_cost"] < diagnostics["graph"]["initial_cost"])

    def test_route_report_fail_closes_iteration_and_cost_gates(self) -> None:
        route = RUNNER.ROUTES[0]
        manifest = RUNNER._read_json(MANIFEST, "Phase89 manifest")
        record = manifest["routes"][0]
        with tempfile.TemporaryDirectory(prefix="phase89-fixture-") as directory:
            output_root = Path(directory)
            relative_root = output_root / "2021-03-16-18-59-us-ca-mtv-a__pixel5"
            relative_root.mkdir(parents=True)
            _write_submission(relative_root / "submission.csv", route, RUNNER.DOMAIN_ROWS[route])
            (relative_root / "summary.json").write_text(json.dumps(_summary(route, iterations=0, initial=10.0, final=10.0)), encoding="utf-8")
            report = RUNNER._route_report(record, output_root)
            self.assertFalse(report["gates"]["graph_iterations_at_least_one"])
            self.assertFalse(report["gates"]["final_cost_strictly_less_than_initial_cost"])

    def test_coordinate_and_speed_gate_reads_no_truth(self) -> None:
        route = "fixture/route"
        with tempfile.TemporaryDirectory(prefix="phase89-coord-") as directory:
            path = Path(directory) / "submission.csv"
            _write_submission(path, route, 3)
            rows = RUNNER.read_prediction(path, route)
            speed = RUNNER.speed_report(rows)
            self.assertEqual(len(rows), 3)
            self.assertTrue(speed["finite"])
            self.assertEqual(speed["over_70_mps_count"], 0)

    def test_evaluator_never_launches_native_or_reads_accuracy_inputs(self) -> None:
        source = RUNNER_PATH.read_text(encoding="utf-8").lower()
        self.assertNotIn("subprocess", source)
        self.assertNotIn("accuracy_compute", source.split("def _read_submission", 1)[0])
        self.assertIn("accuracy_scored", source)
        self.assertIn("graph_iterations_at_least_one", source)
        self.assertIn("final_cost_strictly_less_than_initial_cost", source)


if __name__ == "__main__":
    unittest.main()
