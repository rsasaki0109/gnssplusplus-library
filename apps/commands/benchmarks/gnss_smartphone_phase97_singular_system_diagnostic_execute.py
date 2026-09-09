#!/usr/bin/env python3
"""Execute and seal the authorized Phase97 two-route diagnostic run.

The wrapper is intentionally a one-shot launcher.  It resolves the exact
Phase95 raw-input paths recorded in the sealed Phase95 result, performs only
metadata ``stat`` checks on those files, and substitutes those paths into the
frozen Phase97 commands.  It never hashes, opens, copies, or transforms raw
bytes.  The native command receives only Android raw GNSS, Android raw IMU,
and broadcast navigation.  Its required CSV path is withheld; only compact
graph/rank/LM diagnostic telemetry and logs are retained.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
PRE_RAW = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase97_singular_system_diagnostic.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase97_singular_system_diagnostic_execution_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase97_singular_system_diagnostic_raw_execution_authorization_v1.json"
PHASE95_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase95_raw_input_path_corrected_structural_result_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase97-singular-system-diagnostic-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase97_singular_system_diagnostic_structural_result_v1.json"
RESULT_MD = RESULT_JSON.with_suffix(".md")
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
RAW_FLAGS = (
    ("--android-gnss", "device_gnss.csv", "__PHASE95_RAW_DEVICE_GNSS__"),
    ("--android-imu", "device_imu.csv", "__PHASE95_RAW_DEVICE_IMU__"),
    ("--nav", "brdc.nav", "__PHASE95_RAW_BROADCAST_NAV__"),
)
MAX_TRIALS = 10
RESULT_SCHEMA = "smartphone-r5-phase97-singular-system-diagnostic-structural-result.v1"


class Phase97StructuralError(ValueError):
    """Raised when Phase97 execution or capture fails closed."""


def fail(message: str) -> Phase97StructuralError:
    return Phase97StructuralError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def sha256_file(path: Path, label: str) -> str:
    if path.name in RAW_NAMES:
        raise fail(f"raw-file hash is forbidden: {label}")
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(value, handle, indent=2, sort_keys=True, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def atomic_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(value)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def load_evaluator() -> Any:
    if not PRE_RAW.is_file():
        raise fail(f"missing Phase97 evaluator: {PRE_RAW}")
    spec = importlib.util.spec_from_file_location("phase97_pre_raw", PRE_RAW)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load Phase97 evaluator: {PRE_RAW}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_pinned_contract() -> tuple[Any, dict[str, Any], dict[str, Any], dict[str, Any]]:
    evaluator = load_evaluator()
    try:
        pre_raw = evaluator.verify_pre_raw()
        manifest = evaluator.verify_manifest()
        authorization = evaluator.verify_authorization(manifest)
    except Exception as exc:
        raise fail(f"Phase97 pinned contract failed: {exc}") from exc
    if not authorization.get("release_boundary", {}).get("raw_execution_authorized"):
        raise fail("Phase97 one-shot raw authorization is not active")
    if pre_raw.get("raw_reads") != 0 or pre_raw.get("native_solver_invocations") != 0:
        raise fail("pre-raw verifier reported activity")
    if pre_raw.get("accuracy_scored") is not False:
        raise fail("pre-raw verifier reported accuracy")
    return evaluator, pre_raw, manifest, authorization


def load_phase95_raw_paths(evaluator: Any) -> dict[str, dict[str, str]]:
    """Resolve Phase95 result metadata without opening any raw file."""

    paths = evaluator.phase95_paths()
    if list(paths) != list(ROUTES):
        raise fail("Phase95 raw route order changed")
    return paths


def stat_raw_inputs(raw_paths: dict[str, dict[str, str]]) -> dict[str, dict[str, dict[str, Any]]]:
    """Stat exact Phase95 paths; this function does not read raw bytes."""

    metadata: dict[str, dict[str, dict[str, Any]]] = {}
    for route in ROUTES:
        metadata[route] = {}
        for name in RAW_NAMES:
            path = ROOT / raw_paths[route][name]
            try:
                stat = path.stat()
            except OSError as exc:
                raise fail(f"missing Phase95 raw input: {route}/{name}: {path}: {exc}") from exc
            if not path.is_file():
                raise fail(f"Phase95 raw input is not a regular file: {route}/{name}: {path}")
            metadata[route][name] = {
                "path": raw_paths[route][name],
                "bytes": stat.st_size,
                "exists_before_launch": True,
                "read_by_runner": False,
                "sha256_read_by_runner": False,
                "source": "Phase95 sealed result raw_inputs.path",
                "copied_or_transformed": False,
            }
    return metadata


def materialize_command(record: dict[str, Any], raw_paths: dict[str, str], evaluator: Any) -> list[str]:
    route = record.get("dataset_id")
    if route not in ROUTES:
        raise fail(f"unknown Phase97 route: {route}")
    command = record.get("command")
    evaluator._validate_command(route, command, record, raw_paths)
    materialized = list(command)
    for flag, name, placeholder in RAW_FLAGS:
        index = materialized.index(flag) + 1
        if materialized[index] != placeholder:
            raise fail(f"Phase95 materialization placeholder changed: {route}/{name}")
        materialized[index] = raw_paths[name]
    return materialized


def execute_matrix(manifest: dict[str, Any], evaluator: Any) -> list[dict[str, Any]]:
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite one-shot Phase97 output root: {OUTPUT_ROOT}")
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    records = manifest.get("routes")
    if not isinstance(records, list) or [item.get("dataset_id") for item in records] != list(ROUTES):
        raise fail("Phase97 route order/count changed")
    raw_paths = load_phase95_raw_paths(evaluator)
    raw_metadata = stat_raw_inputs(raw_paths)
    environment = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = local_lib + (":" + environment["LD_LIBRARY_PATH"] if environment.get("LD_LIBRARY_PATH") else "")
    metadata: list[dict[str, Any]] = []
    for record in records:
        route = record["dataset_id"]
        planned = record.get("planned_output")
        if not isinstance(planned, dict):
            raise fail(f"planned output missing: {route}")
        summary_path = ROOT / planned["summary"]
        withheld_path = ROOT / planned["withheld_solution_output"]
        route_dir = summary_path.parent
        route_dir.mkdir(parents=True, exist_ok=False)
        command = materialize_command(record, raw_paths[route], evaluator)
        stdout_path = route_dir / "stdout.log"
        stderr_path = route_dir / "stderr.log"
        started = time.time()
        return_code: int | None = None
        interrupted = False
        launch_error = ""
        with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
            try:
                completed = subprocess.run(command, cwd=ROOT, env=environment, stdout=stdout, stderr=stderr, check=False)
                return_code = completed.returncode
            except KeyboardInterrupt:
                interrupted = True
                stderr.write(b"\nPhase97 runner interrupted; partial output preserved.\n")
            except OSError as exc:
                launch_error = str(exc)
                stderr.write(f"\nPhase97 native launch failed: {exc}\n".encode())
        metadata_record = {
            "schema_version": "smartphone-r5-phase97-singular-system-diagnostic-route-execution.v1",
            "dataset_id": route,
            "run_number": 1,
            "command": command,
            "raw_inputs": raw_metadata[route],
            "planned_output": {"summary": relative(summary_path), "withheld_solution_output": relative(withheld_path)},
            "summary_present_after_launch": summary_path.is_file(),
            "withheld_solution_output_present_after_launch": withheld_path.is_file(),
            "stdout": relative(stdout_path),
            "stderr": relative(stderr_path),
            "started_unix_s": started,
            "ended_unix_s": time.time(),
            "return_code": return_code,
            "interrupted": interrupted,
            "launch_error": launch_error,
            "runner_read_raw_bytes": False,
            "runner_read_truth_mat_base_coordinate_kaggle": False,
            "raw_content_copied_or_transformed": False,
        }
        atomic_json(route_dir / "run_metadata.json", metadata_record)
        metadata.append(metadata_record)
    return metadata


def _json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): _json_safe(child) for key, child in value.items()}
    if isinstance(value, list):
        return [_json_safe(child) for child in value]
    return value


def _pick(mapping: Any, keys: tuple[str, ...]) -> dict[str, Any]:
    if not isinstance(mapping, dict):
        return {}
    return {key: _json_safe(mapping.get(key)) for key in keys}


def _find_forbidden_solution_keys(value: Any, path: str = "summary") -> list[str]:
    forbidden = {"latitude", "longitude", "position_ecef", "receiver_clock_bias", "epoch_clock_drift_mps", "epoch_velocity_nav_mps", "solutions"}
    found: list[str] = []
    if isinstance(value, dict):
        for key, child in value.items():
            if str(key).lower() in forbidden:
                found.append(f"{path}.{key}")
            found.extend(_find_forbidden_solution_keys(child, f"{path}.{key}"))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            found.extend(_find_forbidden_solution_keys(child, f"{path}[{index}]"))
    return found


def _stat_pin(path: Path, label: str) -> dict[str, Any]:
    try:
        stat = path.stat()
    except OSError as exc:
        return {"path": relative(path), "present": False, "bytes": None, "sha256": None, "error": str(exc)}
    return {"path": relative(path), "present": path.is_file(), "bytes": stat.st_size, "sha256": sha256_file(path, label) if path.is_file() else None}


def _read_limited(path: Path, limit: int = 8192) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")[:limit]
    except OSError:
        return "diagnostic log unavailable"


def _compact_key(key: Any) -> dict[str, Any]:
    return _pick(key, ("key", "variable_bucket", "value_type", "value_dimension", "value_present", "factor_degree", "prior_factor_degree", "component_id", "linearized_contribution_count", "finite_linearized_contribution_count", "nonfinite_linearized_contribution_count", "gradient_l2_norm", "normal_diagonal_l2_norm", "normal_diagonal_min", "normal_diagonal_max", "exact_zero_normal_diagonal_count", "near_zero_normal_diagonal_count", "exact_zero_column", "near_zero_column", "family_degrees"))


def _compact_factor(factor: Any) -> dict[str, Any]:
    return _pick(factor, ("graph_index", "concrete_factor_type", "family", "factor_dimension", "finite_error", "is_prior_or_anchor", "keys"))


def _compact_phase97(phase97: Any) -> dict[str, Any]:
    if not isinstance(phase97, dict):
        return {}
    report = _pick(phase97, ("enabled", "attempted", "graph_observed", "initial_linearization_observed", "diagnostic_complete", "graph_factor_count", "graph_value_count", "graph_value_dimension", "graph_initial_cost", "initial_cost", "final_cost", "accepted_outer_iterations", "trial_limit", "trial_trace_complete", "terminal_branch", "missing_factor_key_count", "duplicate_key_reference_count", "value_type_mismatch_count", "empty_factor_count", "isolated_value_key_count", "connected_component_count", "exact_zero_column_count", "near_zero_column_count", "near_zero_threshold", "nearby_variable_capture_status", "ordering_context", "missing_factor_keys", "duplicate_key_reference_keys", "value_type_mismatch_keys", "nearby_variables", "exceptions"))
    keys = phase97.get("keys") if isinstance(phase97.get("keys"), list) else []
    factors = phase97.get("factors") if isinstance(phase97.get("factors"), list) else []
    components = phase97.get("components") if isinstance(phase97.get("components"), list) else []
    ranks = phase97.get("rank_decompositions") if isinstance(phase97.get("rank_decompositions"), list) else []
    trials = phase97.get("lm_trials") if isinstance(phase97.get("lm_trials"), list) else []
    report["keys"] = [_compact_key(item) for item in keys if isinstance(item, dict)]
    report["factors"] = [_compact_factor(item) for item in factors if isinstance(item, dict)]
    report["components"] = [_json_safe(item) for item in components if isinstance(item, dict)]
    report["rank_decompositions"] = [_json_safe(item) for item in ranks if isinstance(item, dict)]
    report["lm_trials"] = [_json_safe(item) for item in trials[:MAX_TRIALS] if isinstance(item, dict)]
    family_summary: dict[str, dict[str, Any]] = {}
    for item in report["keys"]:
        bucket = item.get("variable_bucket") or "unknown"
        entry = family_summary.setdefault(bucket, {"key_count": 0, "total_value_dimension": 0, "factor_degree": 0, "prior_factor_degree": 0, "exact_zero_column_count": 0, "near_zero_column_count": 0})
        entry["key_count"] += 1
        entry["total_value_dimension"] += int(item.get("value_dimension") or 0)
        entry["factor_degree"] += int(item.get("factor_degree") or 0)
        entry["prior_factor_degree"] += int(item.get("prior_factor_degree") or 0)
        entry["exact_zero_column_count"] += int(bool(item.get("exact_zero_column")))
        entry["near_zero_column_count"] += int(bool(item.get("near_zero_column")))
    factor_summary: dict[str, dict[str, Any]] = {}
    for item in report["factors"]:
        family = item.get("family") or "unknown"
        entry = factor_summary.setdefault(family, {"factor_count": 0, "total_dimension": 0, "finite_error_count": 0, "prior_or_anchor_count": 0})
        entry["factor_count"] += 1
        entry["total_dimension"] += int(item.get("factor_dimension") or 0)
        entry["finite_error_count"] += int(bool(item.get("finite_error")))
        entry["prior_or_anchor_count"] += int(bool(item.get("is_prior_or_anchor")))
    report["variable_family_summary"] = family_summary
    report["factor_family_summary"] = factor_summary
    return report


def compact_route_report(metadata: dict[str, Any]) -> dict[str, Any]:
    route = metadata["dataset_id"]
    planned = metadata["planned_output"]
    summary_path = ROOT / planned["summary"]
    withheld_path = ROOT / planned["withheld_solution_output"]
    stdout_path = ROOT / metadata["stdout"]
    stderr_path = ROOT / metadata["stderr"]
    report: dict[str, Any] = {
        "dataset_id": route,
        "run_number": metadata.get("run_number"),
        "return_code": metadata.get("return_code"),
        "expected_diagnostic_return_code": 1,
        "interrupted": metadata.get("interrupted"),
        "launch_error": metadata.get("launch_error", ""),
        "raw_inputs": metadata.get("raw_inputs"),
        "expected_output": {"domain_rows": DOMAIN_ROWS[route], "problem_epochs": DOMAIN_ROWS[route] + 1, "summary": relative(summary_path), "withheld_solution_output": relative(withheld_path), "solution_rows_published": False, "coverage": "expected epoch count only; solution rows withheld"},
        "solution_output_published": False,
        "accuracy_output_published": False,
        "truth_used": False,
        "mat_used": False,
        "kaggle_or_token_accessed": False,
        "runner_read_raw_bytes": False,
        "runner_read_truth_mat_base_coordinate_kaggle": False,
        "raw_content_copied_or_transformed": False,
        "logs": {"stdout": _stat_pin(stdout_path, f"Phase97 stdout {route}"), "stderr": _stat_pin(stderr_path, f"Phase97 stderr {route}")},
        "withheld_solution_output_present": withheld_path.is_file(),
    }
    if not summary_path.is_file():
        report.update({"summary_present": False, "summary_error": "native summary was not produced", "stderr_observation": _read_limited(stderr_path), "phase97_main": None, "main": None, "gnss_first": None, "capture_gates": {"native_process_completed": isinstance(metadata.get("return_code"), int) and not metadata.get("launch_error"), "expected_diagnostic_return_code": metadata.get("return_code") == 1, "summary_present": False, "no_solution_or_accuracy_publication": not withheld_path.is_file()}, "diagnostic_capture_complete": False})
        return report
    try:
        summary = read_json(summary_path, f"Phase97 native summary {route}")
    except Phase97StructuralError as exc:
        report.update({"summary_present": False, "summary_error": str(exc), "stderr_observation": _read_limited(stderr_path), "phase97_main": None, "main": None, "gnss_first": None, "capture_gates": {"native_process_completed": isinstance(metadata.get("return_code"), int) and not metadata.get("launch_error"), "expected_diagnostic_return_code": metadata.get("return_code") == 1, "summary_present": False, "no_solution_or_accuracy_publication": not withheld_path.is_file()}, "diagnostic_capture_complete": False})
        return report
    forbidden_keys = _find_forbidden_solution_keys(summary)
    phase97 = summary.get("phase97_main")
    compact_phase97 = _compact_phase97(phase97)
    gnss = summary.get("gnss_first")
    main = summary.get("main")
    report.update({
        "summary_present": True,
        "summary": _stat_pin(summary_path, f"Phase97 summary {route}"),
        "summary_schema": summary.get("schema_version"),
        "summary_status": summary.get("status"),
        "failure_stage": summary.get("failure_stage"),
        "failure_reason": summary.get("failure_reason"),
        "exception_type": summary.get("exception_type"),
        "exception_message": summary.get("exception_message"),
        "forbidden_summary_keys": forbidden_keys,
        "gnss_first": _pick(gnss, ("attempted", "result_returned", "iterations", "initial_cost", "final_cost", "c0d_factor_count", "c0d_accepted_outer_iterations", "c0d_inner_lambda_attempts", "c0d_active_solve_attempted", "c0d_active_solve_finite_costs", "strict_cost_progress", "optimized_d_epoch_count", "optimized_d_finite_count", "optimized_d_nonfinite_count", "optimized_d_coverage", "exact_retained_key_alignment", "terminal_branch")),
        "main": _pick(main, ("attempted", "result_returned", "problem_epoch_count", "position_solution_size", "position_size_matches_problem_epochs", "receiver_clock_solution_size", "receiver_clock_size_matches_problem_epochs", "all_positions_earth_valid", "all_receiver_clocks_finite", "optimized_d_epoch_count", "optimized_d_size_matches_problem_epochs", "optimized_d_finite_count", "optimized_d_all_finite", "velocity_epoch_count", "velocity_size_matches_problem_epochs", "all_velocities_finite", "exact_retained_key_alignment", "gnss_first_progress", "c0d_factor_count", "active_solve_attempted", "accepted_outer_iterations", "inner_lambda_attempts", "initial_cost", "final_cost", "active_solve_costs_finite", "final_cost_strictly_less_than_initial", "initial_lambda", "maximum_lambda", "final_lambda", "conditioning_proxy", "termination_trace_complete", "terminal_branch", "contract_passed")),
        "phase97_main": compact_phase97,
    })
    graph_observed = bool(compact_phase97.get("enabled") is True and compact_phase97.get("attempted") is True and compact_phase97.get("graph_observed") is True and compact_phase97.get("initial_linearization_observed") is True)
    ranks = compact_phase97.get("rank_decompositions", [])
    trials = compact_phase97.get("lm_trials", [])
    no_publication = bool(summary.get("solution_output_published") is False and summary.get("accuracy_output_published") is False and summary.get("truth_used") is False and summary.get("mat_used") is False and summary.get("kaggle_or_token_accessed") is False and summary.get("raw_truth_mat_kaggle_forbidden") is True and not forbidden_keys and not withheld_path.is_file())
    capture = {
        "native_process_completed": isinstance(metadata.get("return_code"), int) and not metadata.get("launch_error"),
        "expected_diagnostic_return_code": metadata.get("return_code") == 1,
        "summary_present": True,
        "phase97_graph_observed": graph_observed,
        "variable_family_counts_and_dimensions": graph_observed and bool(compact_phase97.get("variable_family_summary")),
        "factor_incidence_and_components": graph_observed and bool(compact_phase97.get("factors")) and bool(compact_phase97.get("components")),
        "anchor_and_prior_incidence": graph_observed and any(bool(item.get("is_prior_or_anchor")) for item in compact_phase97.get("factors", [])),
        "zero_and_near_zero_columns": graph_observed and isinstance(compact_phase97.get("exact_zero_column_count"), int) and isinstance(compact_phase97.get("near_zero_column_count"), int),
        "rank_or_nullspace_attribution": graph_observed and bool(ranks) and all(isinstance(item.get("status"), str) and bool(item.get("status")) for item in ranks if isinstance(item, dict)),
        "first_ten_lm_trials": graph_observed and len(trials) <= MAX_TRIALS and bool(trials) and compact_phase97.get("trial_trace_complete") is True,
        "nearby_variable_status": graph_observed and isinstance(compact_phase97.get("nearby_variable_capture_status"), str) and bool(compact_phase97.get("nearby_variable_capture_status")),
        "linear_solver_exception_telemetry": graph_observed and isinstance(compact_phase97.get("exceptions"), list),
        "no_solution_or_accuracy_publication": no_publication,
    }
    report["capture_gates"] = capture
    report["diagnostic_capture_complete"] = all(capture.values())
    return report


def build_result(metadata: list[dict[str, Any]], manifest: dict[str, Any], pre_raw: dict[str, Any], authorization: dict[str, Any], evaluator: Any) -> dict[str, Any]:
    reports = {item["dataset_id"]: compact_route_report(item) for item in metadata}
    exact_matrix = len(metadata) == 2 and [item.get("dataset_id") for item in metadata] == list(ROUTES) and all(item.get("run_number") == 1 for item in metadata)
    gates = {
        "implementation_and_binary_pins": manifest.get("implementation", {}).get("commit") == evaluator.IMPLEMENTATION_COMMIT,
        "exactly_two_routes_one_run_each": exact_matrix,
        "diagnostic_telemetry_captured": bool(reports) and all(item.get("diagnostic_capture_complete") is True for item in reports.values()),
        "variable_family_counts_and_dimensions": bool(reports) and all(item.get("capture_gates", {}).get("variable_family_counts_and_dimensions") is True for item in reports.values()),
        "factor_incidence_and_components": bool(reports) and all(item.get("capture_gates", {}).get("factor_incidence_and_components") is True for item in reports.values()),
        "anchor_and_prior_incidence": bool(reports) and all(item.get("capture_gates", {}).get("anchor_and_prior_incidence") is True for item in reports.values()),
        "zero_and_near_zero_columns": bool(reports) and all(item.get("capture_gates", {}).get("zero_and_near_zero_columns") is True for item in reports.values()),
        "rank_or_nullspace_attribution": bool(reports) and all(item.get("capture_gates", {}).get("rank_or_nullspace_attribution") is True for item in reports.values()),
        "first_ten_lm_trials": bool(reports) and all(item.get("capture_gates", {}).get("first_ten_lm_trials") is True for item in reports.values()),
        "nearby_variable_status": bool(reports) and all(item.get("capture_gates", {}).get("nearby_variable_status") is True for item in reports.values()),
        "no_solution_or_accuracy_publication": bool(reports) and all(item.get("capture_gates", {}).get("no_solution_or_accuracy_publication") is True for item in reports.values()),
        "truth_free": True,
        "accuracy_not_scored": True,
        "raw_only_command_policy": True,
    }
    gates["all_capture_gates_anded"] = all(gates.values())
    failed = [name for name, passed in gates.items() if passed is False]
    for route, report in reports.items():
        failed.extend(f"{route}:{name}" for name, passed in report.get("capture_gates", {}).items() if passed is False)
    return _json_safe({
        "schema_version": RESULT_SCHEMA,
        "phase": 97,
        "execution_label": "Luna Max",
        "status": "captured-phase97-singular-system-diagnostic" if gates["all_capture_gates_anded"] else "no-go-phase97-singular-system-diagnostic-capture",
        "decision": "Diagnostic telemetry captured; no solution, accuracy, or promotion claim is made." if gates["all_capture_gates_anded"] else "Diagnostic capture failed closed; no solution, accuracy, or promotion lane is available.",
        "diagnostic_only": True,
        "truth_free": True,
        "accuracy_scored": False,
        "solution_output_published": False,
        "promotion_authorized": False,
        "stop_before_truth_accuracy_submission": True,
        "freeze": {"path": evaluator.relative(evaluator.FREEZE), "sha256": evaluator.FREEZE_SHA},
        "authorization": {"path": evaluator.relative(evaluator.AUTHORIZATION), "sha256": sha256_file(AUTHORIZATION, "Phase97 authorization"), "status": authorization.get("status")},
        "pre_raw_verification": pre_raw,
        "evaluator": {"path": relative(PRE_RAW), "sha256": sha256_file(PRE_RAW, "Phase97 evaluator")},
        "manifest": {"path": relative(MANIFEST), "sha256": sha256_file(MANIFEST, "Phase97 manifest")},
        "execution_wrapper": {"path": relative(Path(__file__).resolve()), "sha256": sha256_file(Path(__file__).resolve(), "Phase97 wrapper")},
        "phase95_path_source": {"path": relative(PHASE95_RESULT), "sha256": evaluator.PHASE95_RESULT_SHA, "materialization": "exact retained Phase95 result raw_inputs.path; metadata-only stat; no copy/transform"},
        "implementation": manifest.get("implementation"),
        "matrix": {"candidate_count": 1, "routes": 2, "runs_per_route": len(metadata), "candidate_runs_total": len(metadata), "native_solver_invocations": len(metadata), "controls": 0, "reruns": 0, "fallbacks": 0, "raw_device_gnss_route_arguments": len(metadata), "raw_device_imu_route_arguments": len(metadata), "broadcast_navigation_route_arguments": len(metadata), "runner_raw_byte_reads": 0, "truth_reads": 0, "mat_reads_or_generated": 0, "precomputed_coordinate_reads": 0, "base_rinex_reads": 0, "accuracy_calculations": 0, "kaggle_or_token_access": 0, "route_score_selection": False},
        "routes": reports,
        "gates": {**gates, "all_passed": gates["all_capture_gates_anded"]},
        "failed_gates": failed,
        "read_accounting": {"native_solver_invocations": len(metadata), "raw_device_gnss_reads": len(metadata), "raw_device_imu_reads": len(metadata), "broadcast_navigation_reads": len(metadata), "runner_raw_input_byte_reads": 0, "raw_input_hash_reads": 0, "base_rinex_reads": 0, "truth_reads": 0, "mat_reads_or_generated": 0, "precomputed_coordinate_reads": 0, "accuracy_calculations": 0, "accuracy_scored": False, "kaggle_or_token_access": 0, "route_score_selection": False, "reruns": 0, "fallbacks": 0, "raw_content_copied_or_transformed": False, "logs_and_partial_results_preserved": True},
        "raw_input_provenance": {"names_exact": list(RAW_NAMES), "roles": {"device_gnss.csv": "Android raw GNSS only", "device_imu.csv": "Android raw IMU only", "brdc.nav": "broadcast navigation only"}, "path_source": "Phase95 sealed structural result route raw_inputs.path", "wrapper_path_resolution": "exact path substitution after metadata-only stat", "wrapper_raw_byte_reads": 0, "copy_or_transform": False},
        "forbidden_lanes": {"truth": False, "MAT": False, "precomputed_coordinates": False, "base": False, "Kaggle_or_token": False, "accuracy": False, "solution_rows": False},
    })


def result_markdown(result: dict[str, Any]) -> str:
    lines = ["# Phase97 singular-system diagnostic structural result", "", f"- Status: `{result['status']}`", f"- Decision: {result['decision']}", "- Diagnostic-only: **true**; solution rows and accuracy: **withheld**", "- Matrix: exactly MTV-A and LAX-T × one native invocation each, sequential, no rerun/fallback", "- Raw inputs: Phase95 exact device_gnss.csv, device_imu.csv, brdc.nav paths only; wrapper raw-byte reads: 0", "- Truth/MAT/base/precomputed-coordinate/Kaggle/token/accuracy activity: 0", "", "## Route diagnostic summary", "", "| Route | Return | Graph factors / values | Components / anchors | Zero / near columns | Rank / nullity | LM trials | Capture |", "|---|---:|---:|---|---|---|---:|---|"]
    for route in ROUTES:
        item = result["routes"].get(route, {})
        phase97 = item.get("phase97_main") or {}
        ranks = phase97.get("rank_decompositions") or []
        rank = ranks[0] if ranks else {}
        lines.append(f"| `{route}` | `{item.get('return_code')}` | `{phase97.get('graph_factor_count', 'n/a')} / {phase97.get('graph_value_count', 'n/a')}` | `{phase97.get('connected_component_count', 'n/a')}` / `{sum(1 for c in phase97.get('components', []) if isinstance(c, dict) and c.get('anchored'))}` | `{phase97.get('exact_zero_column_count', 'n/a')} / {phase97.get('near_zero_column_count', 'n/a')}` | `{rank.get('rank', 'n/a')} / {rank.get('nullity', 'n/a')}` | `{len(phase97.get('lm_trials', []))}` | `{item.get('diagnostic_capture_complete')}` |")
        lines.append(f"  - Variable families: `{phase97.get('variable_family_summary', {})}`")
        lines.append(f"  - Factor families: `{phase97.get('factor_family_summary', {})}`")
        lines.append(f"  - Nearby status: `{phase97.get('nearby_variable_capture_status', 'n/a')}`; terminal: `{phase97.get('terminal_branch', 'n/a')}`")
        lines.append(f"  - Capture gates: `{item.get('capture_gates', {})}`")
    lines.extend(["", "All reported graph/key/rank/LM data is diagnostic telemetry only. No solution coordinate, raw observation, truth value, or accuracy score is published.", ""])
    return "\n".join(lines)


def metadata_from_existing() -> list[dict[str, Any]]:
    return [read_json(OUTPUT_ROOT / route.replace("/", "__") / "run_metadata.json", f"Phase97 route metadata {route}") for route in ROUTES]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="execute exactly two authorized routes once")
    parser.add_argument("--validate", action="store_true", help="validate preserved route metadata without rerunning")
    parser.add_argument("--result-json", type=Path, default=RESULT_JSON)
    args = parser.parse_args(argv)
    if args.execute == args.validate:
        parser.error("choose exactly one of --execute or --validate")
    try:
        evaluator, pre_raw, manifest, authorization = load_pinned_contract()
        metadata = execute_matrix(manifest, evaluator) if args.execute else metadata_from_existing()
        result = build_result(metadata, manifest, pre_raw, authorization, evaluator)
        target = args.result_json.resolve()
        atomic_json(target, result)
        atomic_text(target.with_suffix(".md"), result_markdown(result))
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["gates"]["all_capture_gates_anded"] else 1
    except (Phase97StructuralError, OSError) as exc:
        print(f"phase97 structural runner: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
