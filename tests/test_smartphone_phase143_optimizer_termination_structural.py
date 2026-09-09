"""Launch-free synthetic tests for the Phase143 structural contract.

The retained Phase138 fixture is an in-memory structural summary.  No raw
payload, solution coordinate, truth/MAT/PDC artifact, or native process is
read or launched by these tests.
"""

from __future__ import annotations

import copy
import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase143_optimizer_termination_structural.py"
)
PHASE138_TEST_PATH = ROOT / (
    "tests/test_smartphone_phase138_affine_tdcp_structural.py"
)


def _load(path: Path, module_name: str):
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RUNNER = _load(RUNNER_PATH, "phase143_structural_runner_test")
PHASE138_FIXTURES = _load(PHASE138_TEST_PATH, "phase138_fixture_for_phase143")


def _termination(stage: str, *, branch: str = "outer_convergence_tolerance",
                 accepted: int = 2, attempted: int | None = None,
                 configured: int | None = None) -> dict:
    if attempted is None:
        attempted = accepted
    if configured is None:
        configured = 12 if stage == "main" else 1000
    return {
        "selector_enabled": True,
        "stage": stage,
        "configured_max_iterations": configured,
        "effective_max_iterations": 1000,
        "attempted": True,
        "attempted_outer_iterations": attempted,
        "accepted_outer_iterations": accepted,
        "rejected_outer_iterations": attempted - accepted,
        "total_inner_lambda_attempts": max(accepted, 2),
        "initial_cost": 20.0 if stage == "main" else 10.0,
        "final_cost": 8.0 if stage == "main" else 4.0,
        "costs_finite": True,
        "strict_cost_decrease": True,
        "termination_branch": branch,
        "relative_error_tolerance": 1e-8,
        "absolute_error_tolerance": 1e-10,
        "error_tolerance": 0.0,
        "initial_lambda": 1e-5,
        "final_lambda": 1e-5,
        "maximum_lambda": 100000.0,
        "lambda_factor": 10.0,
        "lambda_lower_bound": 0.0,
        "lambda_upper_bound": 100000.0,
        "min_model_fidelity": 0.001,
        "diagonal_damping": False,
        "use_fixed_lambda_factor": True,
        "linear_solver": ("MULTIFRONTAL_QR" if stage == "main"
                           else "MULTIFRONTAL_CHOLESKY"),
        "elimination": ("EliminateQR" if stage == "main"
                        else "EliminatePreferCholesky"),
        "ordering_type": "COLAMD",
        "explicit_ordering_present": False,
        "no_fallback": True,
        "termination_trace_complete": True,
        "configuration_valid": True,
    }


def _summary(route: str) -> dict:
    summary = PHASE138_FIXTURES._summary(route)
    summary["phase143_termination"] = {
        "schema_version": RUNNER.NATIVE_TERMINATION_SCHEMA,
        "authority": "FGOResult.diagnostics.native_phase143_termination",
        "main": _termination("main"),
        "gnss_first": _termination("gnss-first", accepted=3),
        "no_solution_or_accuracy_fields": True,
    }
    return summary


class Phase143OptimizerTerminationStructuralTests(unittest.TestCase):
    def test_freeze_and_manifest_are_schema_valid(self) -> None:
        freeze = RUNNER.read_json(RUNNER.FREEZE, "freeze")
        RUNNER.validate_freeze(freeze)
        manifest = RUNNER.read_json(RUNNER.MANIFEST, "manifest")
        RUNNER.validate_manifest(manifest)

    def test_placeholder_commands_are_exact_and_ordered(self) -> None:
        manifest = RUNNER.read_json(RUNNER.MANIFEST, "manifest")
        self.assertEqual(manifest["routes"], list(RUNNER.ROUTES))
        for route in RUNNER.ROUTES:
            command = manifest["command_snapshots"][route]
            RUNNER.validate_command(route, command)
            self.assertEqual(command, RUNNER.command_template(route))
            self.assertEqual(command.count(RUNNER.PHASE143), 1)
            self.assertEqual(command.count(RUNNER.PHASE117), 0)

    def test_complete_summary_passes_for_both_routes(self) -> None:
        for route in RUNNER.ROUTES:
            RUNNER.validate_structural_summary(route, _summary(route))

    def test_cap_branch_requires_effective_1000_not_twelve(self) -> None:
        summary = _summary(RUNNER.ROUTES[0])
        report = summary["phase143_termination"]["main"]
        report.update({
            "termination_branch": "maximum_outer_iterations",
            "attempted_outer_iterations": 1000,
            "accepted_outer_iterations": 1000,
            "rejected_outer_iterations": 0,
            "total_inner_lambda_attempts": 1000,
        })
        RUNNER.validate_structural_summary(RUNNER.ROUTES[0], summary)
        bad = copy.deepcopy(summary)
        bad["phase143_termination"]["main"]["accepted_outer_iterations"] = 12
        bad["phase143_termination"]["main"]["attempted_outer_iterations"] = 12
        bad["phase143_termination"]["main"]["total_inner_lambda_attempts"] = 12
        bad["phase143_termination"]["main"]["rejected_outer_iterations"] = 0
        with self.assertRaises(RUNNER.Phase143ContractError):
            RUNNER.validate_structural_summary(RUNNER.ROUTES[0], bad)

    def test_missing_or_generic_iteration_telemetry_fails_closed(self) -> None:
        bad = _summary(RUNNER.ROUTES[0])
        del bad["phase143_termination"]["main"]["accepted_outer_iterations"]
        bad["phase143_termination"]["main"]["iterations"] = 4
        with self.assertRaises(RUNNER.Phase143ContractError):
            RUNNER.validate_structural_summary(RUNNER.ROUTES[0], bad)

    def test_cost_and_parameter_drift_fails_closed(self) -> None:
        for field, value in (("final_cost", 20.0),
                             ("relative_error_tolerance", 1e-5),
                             ("linear_solver", "MULTIFRONTAL_CHOLESKY"),
                             ("termination_trace_complete", False)):
            bad = _summary(RUNNER.ROUTES[0])
            bad["phase143_termination"]["main"][field] = value
            with self.subTest(field=field), self.assertRaises(
                RUNNER.Phase143ContractError
            ):
                RUNNER.validate_structural_summary(RUNNER.ROUTES[0], bad)

    def test_stage_scope_and_selector_isolation_are_strict(self) -> None:
        bad = _summary(RUNNER.ROUTES[0])
        bad["phase143_termination"]["gnss_first"]["stage"] = "main"
        with self.assertRaises(RUNNER.Phase143ContractError):
            RUNNER.validate_structural_summary(RUNNER.ROUTES[0], bad)
        bad = _summary(RUNNER.ROUTES[0])
        bad["selectors"]["phase117_dynamic_tdcp_sigma"] = True
        with self.assertRaises(RUNNER.Phase143ContractError):
            RUNNER.validate_structural_summary(RUNNER.ROUTES[0], bad)

    def test_solution_fields_are_never_allowed_in_termination_sidecar(self) -> None:
        bad = _summary(RUNNER.ROUTES[0])
        bad["phase143_termination"]["main"]["solution_values"] = []
        with self.assertRaises(RUNNER.Phase143ContractError):
            RUNNER.validate_structural_summary(RUNNER.ROUTES[0], bad)


if __name__ == "__main__":
    unittest.main()
