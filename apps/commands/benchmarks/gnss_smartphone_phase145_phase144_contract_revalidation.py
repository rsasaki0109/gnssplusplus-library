#!/usr/bin/env python3
"""Launch-free Phase145 revalidation of the sealed Phase144 summaries.

This lane corrects only the Phase144 validator's termination-field
expectations.  It reads the sealed Phase144 result metadata and the two
native_summary.json files named by that metadata.  It never opens raw GNSS,
IMU, navigation, base, solution, truth, MAT/PDC, or Kaggle payloads and never
starts a solver.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[3]
HISTORICAL_RESULT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase144_telemetry_serializer_raw_result_v1.json"
)
PHASE144_RUNNER_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase144_telemetry_serializer_structural.py"
)
RUNNER = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase145_phase144_contract_revalidation.py"
)
AUDIT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase145_phase144_contract_revalidation_audit_v1.md"
)
FOCUSED_TESTS = ROOT / (
    "tests/test_smartphone_phase145_phase144_contract_revalidation.py"
)

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
NATIVE_TERMINATION_SCHEMA = "smartphone-r5-native-fgo-phase143-termination.v1"
PHASE144_RESULT_SCHEMA = "smartphone-r5-phase144-telemetry-serializer-raw-result.v1"
ALLOWED_TERMINATION_BRANCHES = {
    "maximum_outer_iterations",
    "outer_convergence_tolerance",
    "small_cost_change",
    "maximum_lambda",
    "no_inner_iteration",
    "exception",
    "no_progress_unclassified",
}
EXPECTED_MAIN_ACCEPTED = {
    ROUTES[0]: 25,
    ROUTES[1]: 31,
}
EXPECTED_GNSS_FIRST_ACCEPTED = {
    ROUTES[0]: 111,
    ROUTES[1]: 116,
}


class Phase145ContractError(ValueError):
    """A Phase145 contract violation which must fail closed."""


def fail(message: str) -> Phase145ContractError:
    return Phase145ContractError(message)


def load_phase144_runner() -> Any:
    spec = importlib.util.spec_from_file_location(
        "phase144_runner_for_phase145", PHASE144_RUNNER_PATH
    )
    if spec is None or spec.loader is None:
        raise fail("unable to load the immutable Phase144 validator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


PHASE144 = load_phase144_runner()


def duplicate_json_paths(text: str) -> list[str]:
    """Use the existing recursive duplicate detector without changing it."""
    return PHASE144.duplicate_json_paths(text)


def _reject_duplicate_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise fail(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def read_object(path: Path, label: str) -> dict[str, Any]:
    try:
        text = path.read_text(encoding="utf-8")
        duplicates = duplicate_json_paths(text)
        if duplicates:
            raise fail(f"{label}: duplicate paths {duplicates}")
        value = json.loads(text, object_pairs_hook=_reject_duplicate_pairs)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError,
            Phase145ContractError, ValueError) as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label}: expected object")
    return value


def sha256_file(path: Path, label: str) -> str:
    """Hash only metadata/source artifacts; reject payload-like targets."""
    lowered_name = path.name.lower()
    lowered_path = str(path).lower()
    if lowered_name.endswith((".csv", ".nav", ".obs", ".mat")):
        raise fail(f"payload hash forbidden for {label}")
    if any(term in lowered_path for term in (
        "truth", "ground_truth", "precomputed", "kaggle", ".pdc",
    )):
        raise fail(f"forbidden artifact hash for {label}")
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _required(mapping: Mapping[str, Any], key: str, label: str) -> Any:
    if key not in mapping:
        raise fail(f"{label}/{key}: missing")
    return mapping[key]


def _bool(mapping: Mapping[str, Any], key: str, label: str) -> bool:
    value = _required(mapping, key, label)
    if not isinstance(value, bool):
        raise fail(f"{label}/{key}: expected boolean")
    return value


def _true(mapping: Mapping[str, Any], key: str, label: str) -> None:
    if not _bool(mapping, key, label):
        raise fail(f"{label}/{key}: expected true")


def _count(mapping: Mapping[str, Any], key: str, label: str) -> int:
    value = _required(mapping, key, label)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise fail(f"{label}/{key}: expected nonnegative integer")
    return value


def _finite(mapping: Mapping[str, Any], key: str, label: str) -> float:
    value = _required(mapping, key, label)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise fail(f"{label}/{key}: expected finite number")
    result = float(value)
    if not math.isfinite(result):
        raise fail(f"{label}/{key}: expected finite number")
    return result


def _equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def validate_termination_stage(
    stage: str,
    report: Mapping[str, Any],
    *,
    expected_accepted: int | None = None,
) -> dict[str, Any]:
    """Validate the native Phase143 report with corrected field semantics."""
    label = f"phase143_termination/{stage}"
    required = (
        "selector_enabled", "stage", "configured_max_iterations",
        "effective_max_iterations", "attempted", "attempted_outer_iterations",
        "accepted_outer_iterations", "rejected_outer_iterations",
        "total_inner_lambda_attempts", "initial_cost", "final_cost", "costs_finite",
        "strict_cost_decrease", "termination_branch", "relative_error_tolerance",
        "absolute_error_tolerance", "error_tolerance", "initial_lambda", "final_lambda",
        "maximum_lambda", "lambda_factor", "lambda_lower_bound", "lambda_upper_bound",
        "min_model_fidelity", "diagonal_damping", "use_fixed_lambda_factor",
        "linear_solver", "elimination", "ordering_type", "explicit_ordering_present",
        "no_fallback", "termination_trace_complete", "configuration_valid",
        "configuration_failure",
    )
    for key in required:
        _required(report, key, label)
    _true(report, "selector_enabled", label)
    _equal(_required(report, "stage", label), stage, f"{label}/stage")
    configured = _count(report, "configured_max_iterations", label)
    effective = _count(report, "effective_max_iterations", label)
    expected_configured = 12 if stage == "main" else 1000
    _equal(configured, expected_configured, f"{label}/configured_max_iterations")
    _equal(effective, 1000, f"{label}/effective_max_iterations")
    _true(report, "attempted", label)
    attempted = _count(report, "attempted_outer_iterations", label)
    accepted = _count(report, "accepted_outer_iterations", label)
    rejected = _count(report, "rejected_outer_iterations", label)
    _equal(rejected, attempted - accepted, f"{label}/outer iteration conservation")
    if accepted <= 0 or accepted > effective:
        raise fail(f"{label}: accepted outer iterations exceed positive effective cap")
    if expected_accepted is not None:
        _equal(accepted, expected_accepted, f"{label}/accepted_outer_iterations")
    if _count(report, "total_inner_lambda_attempts", label) < accepted:
        raise fail(f"{label}/total_inner_lambda_attempts: below accepted count")

    initial = _finite(report, "initial_cost", label)
    final = _finite(report, "final_cost", label)
    _true(report, "costs_finite", label)
    _true(report, "strict_cost_decrease", label)
    if not final < initial:
        raise fail(f"{label}: final cost did not strictly decrease")

    for key in (
        "relative_error_tolerance", "absolute_error_tolerance", "error_tolerance",
        "initial_lambda", "final_lambda", "maximum_lambda", "lambda_factor",
        "lambda_lower_bound", "lambda_upper_bound", "min_model_fidelity",
    ):
        _finite(report, key, label)
    _equal(report["relative_error_tolerance"], 1e-8,
           f"{label}/relative_error_tolerance")
    _equal(report["absolute_error_tolerance"], 1e-10,
           f"{label}/absolute_error_tolerance")
    _equal(report["error_tolerance"], 0.0, f"{label}/error_tolerance")
    _equal(report["initial_lambda"], 1e-5, f"{label}/initial_lambda")
    _equal(report["lambda_factor"], 10.0, f"{label}/lambda_factor")
    _equal(report["lambda_lower_bound"], 0.0, f"{label}/lambda_lower_bound")
    _equal(report["lambda_upper_bound"], 100000.0, f"{label}/lambda_upper_bound")
    _equal(report["min_model_fidelity"], 0.001, f"{label}/min_model_fidelity")
    observed_max_lambda = float(report["maximum_lambda"])
    final_lambda = float(report["final_lambda"])
    if (observed_max_lambda < 0.0 or
            observed_max_lambda > float(report["lambda_upper_bound"]) or
            observed_max_lambda < max(float(report["initial_lambda"]), final_lambda)):
        raise fail(
            f"{label}/maximum_lambda: observed peak is outside the configured range"
        )

    for key in ("diagonal_damping", "use_fixed_lambda_factor",
                "explicit_ordering_present"):
        _bool(report, key, label)
    _equal(report["diagonal_damping"], False, f"{label}/diagonal_damping")
    _equal(report["use_fixed_lambda_factor"], True, f"{label}/use_fixed_lambda_factor")
    _equal(report["explicit_ordering_present"], False,
           f"{label}/explicit_ordering_present")
    if stage == "main":
        _equal(report["linear_solver"], "MULTIFRONTAL_QR", f"{label}/linear_solver")
        _equal(report["elimination"], "EliminateQR", f"{label}/elimination")
    elif stage == "gnss-first":
        _equal(report["linear_solver"], "MULTIFRONTAL_CHOLESKY",
               f"{label}/linear_solver")
        _equal(report["elimination"], "EliminatePreferCholesky",
               f"{label}/elimination")
    else:
        raise fail(f"{label}/stage: unsupported native stage")
    _equal(report["ordering_type"], "COLAMD", f"{label}/ordering_type")
    _true(report, "no_fallback", label)
    _true(report, "termination_trace_complete", label)
    _true(report, "configuration_valid", label)
    _equal(report["configuration_failure"], "", f"{label}/configuration_failure")
    branch = report["termination_branch"]
    if branch not in ALLOWED_TERMINATION_BRANCHES:
        raise fail(f"{label}/termination_branch: unsupported branch {branch!r}")
    if branch == "maximum_outer_iterations":
        _equal(accepted, effective, f"{label}/maximum_outer_iterations cap")
    elif branch == "outer_convergence_tolerance" and accepted >= effective:
        raise fail(f"{label}/outer_convergence_tolerance reached the effective cap")
    return {
        "configured_max_iterations": configured,
        "effective_max_iterations": effective,
        "attempted_outer_iterations": attempted,
        "accepted_outer_iterations": accepted,
        "rejected_outer_iterations": rejected,
        "termination_branch": branch,
        "maximum_lambda_observed": observed_max_lambda,
    }


def validate_phase143_sidecar(
    summary: Mapping[str, Any],
    *,
    expected_main_accepted: int | None = None,
    expected_gnss_first_accepted: int | None = None,
) -> dict[str, Any]:
    sidecar = _required(summary, "phase143_termination", "summary")
    if not isinstance(sidecar, Mapping):
        raise fail("summary/phase143_termination: expected object")
    _equal(_required(sidecar, "schema_version", "summary/phase143_termination"),
           NATIVE_TERMINATION_SCHEMA, "summary/phase143_termination/schema_version")
    _equal(_required(sidecar, "authority", "summary/phase143_termination"),
           "FGOResult.diagnostics.native_phase143_termination",
           "summary/phase143_termination/authority")
    _true(sidecar, "no_solution_or_accuracy_fields", "summary/phase143_termination")
    main = _required(sidecar, "main", "summary/phase143_termination")
    gnss_first = _required(sidecar, "gnss_first", "summary/phase143_termination")
    if not isinstance(main, Mapping) or not isinstance(gnss_first, Mapping):
        raise fail("summary/phase143_termination stages: expected objects")
    return {
        "main": validate_termination_stage(
            "main", main, expected_accepted=expected_main_accepted
        ),
        "gnss-first": validate_termination_stage(
            "gnss-first", gnss_first,
            expected_accepted=expected_gnss_first_accepted,
        ),
    }


def _phase144_compatible_copy(summary: Mapping[str, Any]) -> dict[str, Any]:
    """Adapt only known Phase144 validator naming assumptions in memory."""
    value = copy.deepcopy(dict(summary))
    sidecar = value["phase143_termination"]
    sidecar["main"]["configured_max_iterations"] = 1000
    sidecar["gnss_first"]["stage"] = "gnss_first"
    return value


def validate_native_summary(
    route: str,
    summary: Mapping[str, Any],
    *,
    expected_main_accepted: int | None = None,
    expected_gnss_first_accepted: int | None = None,
) -> dict[str, Any]:
    """Run the corrected sidecar checks and every unchanged Phase144 gate."""
    if route not in ROUTES:
        raise fail(f"unknown route {route!r}")
    sidecar = validate_phase143_sidecar(
        summary,
        expected_main_accepted=expected_main_accepted,
        expected_gnss_first_accepted=expected_gnss_first_accepted,
    )
    # The immutable Phase144 validator covers equation, factor-family, clock,
    # base, solver, bridge, offset, output, policy, and duplicate-free schema
    # gates.  Its two termination naming/value assumptions are corrected above
    # and adapted only in this detached copy; the source summary is untouched.
    try:
        PHASE144.validate_native_summary(route, _phase144_compatible_copy(summary))
    except Exception as exc:
        raise fail(f"retained Phase144 structural gate failed: {exc}") from exc
    return sidecar


def _summary_path_from_route(route_record: Mapping[str, Any], route: str) -> Path:
    execution = _required(route_record, "execution", f"result/routes/{route}")
    if not isinstance(execution, Mapping):
        raise fail(f"result/routes/{route}/execution: expected object")
    argv = _required(execution, "argv", f"result/routes/{route}/execution")
    if not isinstance(argv, list) or argv.count("--summary-json") != 1:
        raise fail(f"result/routes/{route}/execution/argv: summary option missing/duplicated")
    index = argv.index("--summary-json")
    if index + 1 >= len(argv) or not isinstance(argv[index + 1], str):
        raise fail(f"result/routes/{route}/execution/argv: summary path missing")
    path = Path(argv[index + 1])
    try:
        relative = path.relative_to(ROOT)
    except ValueError as exc:
        raise fail(f"result/routes/{route}/summary path escapes repository") from exc
    if relative.name != "native_summary.json":
        raise fail(f"result/routes/{route}/summary path is not native_summary.json")
    if any(term in str(relative).lower() for term in (
        "device_gnss", "device_imu", "brdc.nav", "base.obs", "truth", "precomputed",
    )):
        raise fail(f"result/routes/{route}/summary path resembles forbidden payload")
    return path


def validate_historical_result(result: Mapping[str, Any]) -> list[dict[str, Any]]:
    _equal(result.get("schema_version"), PHASE144_RESULT_SCHEMA,
           "historical_result/schema_version")
    _equal(result.get("phase"), 144, "historical_result/phase")
    _equal(result.get("execution_label"), "Luna Max", "historical_result/execution_label")
    _equal(result.get("status"), "sealed-structural-raw-result-no-accuracy",
           "historical_result/status")
    policy = _required(result, "policy", "historical_result")
    if not isinstance(policy, Mapping):
        raise fail("historical_result/policy: expected object")
    for key in ("accuracy_evaluation", "fallback", "kaggle_access", "mat_used",
                "pdc_used", "precomputed_coordinates_used", "repair", "rerun",
                "solution_coordinate_interpretation", "solution_publication", "truth_used"):
        _equal(policy.get(key), False, f"historical_result/policy/{key}")
    routes = _required(result, "routes", "historical_result")
    if not isinstance(routes, list) or len(routes) != len(ROUTES):
        raise fail("historical_result/routes: expected two route records")
    by_route: dict[str, Mapping[str, Any]] = {}
    for record in routes:
        if not isinstance(record, Mapping):
            raise fail("historical_result/routes: route record is not an object")
        route = _required(record, "route", "historical_result/routes")
        if route in by_route or route not in ROUTES:
            raise fail(f"historical_result/routes: unexpected/duplicate route {route!r}")
        by_route[route] = record
    if set(by_route) != set(ROUTES):
        raise fail("historical_result/routes: route set changed")
    output: list[dict[str, Any]] = []
    for route in ROUTES:
        record = by_route[route]
        summary_path = _summary_path_from_route(record, route)
        summary = read_object(summary_path, f"native summary {route}")
        expected_main = EXPECTED_MAIN_ACCEPTED[route]
        expected_gnss = EXPECTED_GNSS_FIRST_ACCEPTED[route]
        telemetry = validate_native_summary(
            route, summary,
            expected_main_accepted=expected_main,
            expected_gnss_first_accepted=expected_gnss,
        )
        output.append({
            "route": route,
            "summary_path": str(summary_path.relative_to(ROOT)),
            "summary_sha256": sha256_file(summary_path, f"native summary {route}"),
            "main": telemetry["main"],
            "gnss_first": telemetry["gnss-first"],
            "solution_content_read": False,
            "coordinate_rows_interpreted": False,
        })
    return output


def revalidate() -> dict[str, Any]:
    result = read_object(HISTORICAL_RESULT, "historical Phase144 result")
    route_results = validate_historical_result(result)
    return {
        "schema_version": "smartphone-r5-phase145-phase144-contract-revalidation-result.v1",
        "phase": 145,
        "execution_label": "Luna Max",
        "status": "revalidated-structural-only-go",
        "historical_phase144_result": {
            "path": str(HISTORICAL_RESULT.relative_to(ROOT)),
            "sha256": sha256_file(HISTORICAL_RESULT, "historical Phase144 result"),
            "status_preserved": True,
            "historical_status": result["status"],
        },
        "validator": {
            "path": str(RUNNER.relative_to(ROOT)),
            "phase144_runner_path": str(PHASE144_RUNNER_PATH.relative_to(ROOT)),
            "audit_path": str(AUDIT.relative_to(ROOT)),
            "focused_tests_path": str(FOCUSED_TESTS.relative_to(ROOT)),
            "corrected_expectations": {
                "main_configured_max_iterations": 12,
                "main_effective_max_iterations": 1000,
                "gnss_first_configured_max_iterations": 1000,
                "gnss_first_effective_max_iterations": 1000,
                "native_gnss_first_stage_label": "gnss-first",
                "maximum_lambda_semantics": "observed peak lambda; finite and within configured bounds",
            },
            "phase144_structural_gates_reused": True,
            "wrapper_inference": False,
            "summary_overwrite": False,
        },
        "routes": route_results,
        "read_accounting": {
            "historical_native_summary_reads": len(route_results),
            "raw_phone_gnss_reads": 0,
            "raw_phone_imu_reads": 0,
            "broadcast_navigation_reads": 0,
            "raw_base_rinex_reads": 0,
            "native_solver_invocations": 0,
            "truth_reads": 0,
            "solution_coordinate_reads": 0,
            "mat_pdc_precomputed_coordinate_reads": 0,
            "accuracy_calculations": 0,
            "kaggle_access": 0,
            "reruns_fallbacks_repairs_sweeps": 0,
        },
        "policy": {
            "raw_execution": False,
            "solver_execution": False,
            "truth_used": False,
            "solution_content_interpreted": False,
            "historical_phase144_artifacts_mutated": False,
        },
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--print-result", action="store_true",
                        help="print the complete metadata-only revalidation result")
    args = parser.parse_args(argv)
    try:
        result = revalidate()
        print(json.dumps(result if args.print_result else {
            "status": result["status"],
            "phase": result["phase"],
            "routes": result["routes"],
            "read_accounting": result["read_accounting"],
        }, sort_keys=True, indent=2))
    except Phase145ContractError as exc:
        print(f"PHASE145_FAIL_CLOSED: {exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
