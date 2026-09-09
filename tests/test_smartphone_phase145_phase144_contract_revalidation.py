"""Launch-free regression tests for the Phase145 Phase144 revalidation lane.

Fixtures are synthetic in-memory summaries.  These tests do not open route
payloads, solution rows, truth, MAT/PDC artifacts, or start a solver.
"""

from __future__ import annotations

import copy
import importlib.util
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase145_phase144_contract_revalidation.py"
)
PHASE144_TESTS_PATH = ROOT / "tests/test_smartphone_phase144_telemetry_serializer.py"


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RUNNER = _load(RUNNER_PATH, "phase145_revalidation_test_runner")
PHASE144_TESTS = _load(PHASE144_TESTS_PATH, "phase144_synthetic_fixtures")


def corrected_summary(route: str) -> dict:
    """Adapt the existing synthetic Phase144 fixture to native semantics."""
    summary = copy.deepcopy(PHASE144_TESTS.complete_summary(route))
    for key, report in summary["phase143_termination"].items():
        if key not in ("main", "gnss_first"):
            continue
        report["relative_error_tolerance"] = 1e-8
        report["absolute_error_tolerance"] = 1e-10
        report["error_tolerance"] = 0.0
        report["lambda_lower_bound"] = 0.0
        report["lambda_upper_bound"] = 100000.0
        report["diagonal_damping"] = False
        report["termination_branch"] = "outer_convergence_tolerance"
    summary["phase143_termination"]["main"]["configured_max_iterations"] = 12
    summary["phase143_termination"]["gnss_first"]["stage"] = "gnss-first"
    summary["phase143_termination"]["gnss_first"]["linear_solver"] = (
        "MULTIFRONTAL_CHOLESKY"
    )
    summary["phase143_termination"]["gnss_first"]["elimination"] = (
        "EliminatePreferCholesky"
    )
    return summary


class Phase145Phase144ContractTests(unittest.TestCase):
    def test_correct_native_configured_effective_split_is_accepted(self) -> None:
        route = RUNNER.ROUTES[0]
        summary = corrected_summary(route)
        result = RUNNER.validate_native_summary(
            route,
            summary,
            expected_main_accepted=2,
            expected_gnss_first_accepted=2,
        )
        self.assertEqual(result["main"]["configured_max_iterations"], 12)
        self.assertEqual(result["main"]["effective_max_iterations"], 1000)
        self.assertEqual(result["gnss-first"]["configured_max_iterations"], 1000)
        self.assertEqual(result["gnss-first"]["effective_max_iterations"], 1000)

    def test_effective_twelve_is_rejected(self) -> None:
        route = RUNNER.ROUTES[0]
        summary = corrected_summary(route)
        summary["phase143_termination"]["main"]["effective_max_iterations"] = 12
        with self.assertRaises(RUNNER.Phase145ContractError):
            RUNNER.validate_native_summary(route, summary,
                                           expected_main_accepted=2,
                                           expected_gnss_first_accepted=2)

    def test_missing_and_invalid_termination_fields_fail_closed(self) -> None:
        route = RUNNER.ROUTES[0]
        missing = corrected_summary(route)
        del missing["phase143_termination"]["main"]["accepted_outer_iterations"]
        with self.assertRaises(RUNNER.Phase145ContractError):
            RUNNER.validate_native_summary(route, missing,
                                           expected_main_accepted=2,
                                           expected_gnss_first_accepted=2)

        invalid = corrected_summary(route)
        invalid["phase143_termination"]["main"]["maximum_lambda"] = 100001.0
        with self.assertRaises(RUNNER.Phase145ContractError):
            RUNNER.validate_native_summary(route, invalid,
                                           expected_main_accepted=2,
                                           expected_gnss_first_accepted=2)

    def test_wrong_selector_fails_closed(self) -> None:
        route = RUNNER.ROUTES[0]
        summary = corrected_summary(route)
        summary["phase144_telemetry"]["selectors"][
            "phase143_official_main_lm_termination_budget"
        ] = False
        with self.assertRaises(RUNNER.Phase145ContractError):
            RUNNER.validate_native_summary(route, summary,
                                           expected_main_accepted=2,
                                           expected_gnss_first_accepted=2)

    def test_duplicate_json_is_rejected_recursively(self) -> None:
        duplicate = '{"phase143_termination":{"main":{"failure":1,"failure":2}}}'
        self.assertEqual(
            RUNNER.duplicate_json_paths(duplicate),
            ["/phase143_termination/main/failure"],
        )
        with tempfile.NamedTemporaryFile("w", suffix=".json", encoding="utf-8") as handle:
            handle.write(duplicate)
            handle.flush()
            with self.assertRaises(RUNNER.Phase145ContractError):
                RUNNER.read_object(Path(handle.name), "synthetic duplicate")


if __name__ == "__main__":
    unittest.main()
