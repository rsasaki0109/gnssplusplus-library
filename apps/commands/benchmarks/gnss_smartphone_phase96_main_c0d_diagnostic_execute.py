#!/usr/bin/env python3
"""Run and seal the authorized Phase96 two-route diagnostic execution.

The wrapper is intentionally narrow: it resolves the exact Phase91 raw paths
by route and filename, performs metadata-only ``stat`` checks, and lets the
native binary read the three raw inputs.  It never hashes, opens, copies, or
transforms raw bytes.  The native command's required CSV path is explicitly
withheld; only the native Phase96 diagnostic summary, logs, and structural
metrics are retained.
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
PRE_RAW = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase96_main_c0d_diagnostic.py"
)
MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase96_main_c0d_diagnostic_execution_manifest_v1.json"
)
AUTHORIZATION = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase96_main_c0d_diagnostic_raw_execution_authorization_v1.json"
)
PHASE91_MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase91_source_clock_c0d_gnss_first_raw_drift_d_initializer_execution_manifest_v1.json"
)
FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase96_main_c0d_diagnostic_telemetry_freeze_v1.json"
)
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase96-main-c0d-diagnostic-v1"
RESULT_JSON = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase96_main_c0d_diagnostic_structural_result_v1.json"
)
RESULT_MD = RESULT_JSON.with_suffix(".md")

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {
    ROUTES[0]: 2158,
    ROUTES[1]: 1465,
}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
RAW_FLAGS = (
    ("--android-gnss", "device_gnss.csv"),
    ("--android-imu", "device_imu.csv"),
    ("--nav", "brdc.nav"),
)
RESULT_SCHEMA = "smartphone-r5-phase96-main-c0d-diagnostic-structural-result.v1"
MAX_TRIALS = 10


class Phase96StructuralError(ValueError):
    """Raised when the diagnostic execution cannot proceed safely."""


def fail(message: str) -> Phase96StructuralError:
    return Phase96StructuralError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def sha256_file(path: Path, label: str) -> str:
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise fail(f"failed to hash {label}: {path}: {exc}") from exc
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
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
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
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
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


def _load_evaluator() -> Any:
    if not PRE_RAW.is_file():
        raise fail(f"missing Phase96 pre-raw evaluator: {PRE_RAW}")
    spec = importlib.util.spec_from_file_location("phase96_pre_raw", PRE_RAW)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load Phase96 evaluator: {PRE_RAW}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_pinned_contract() -> tuple[Any, dict[str, Any], dict[str, Any], dict[str, Any]]:
    evaluator = _load_evaluator()
    try:
        pre_raw = evaluator.verify_pre_raw()
        manifest = evaluator.verify_manifest()
    except Exception as exc:
        raise fail(f"Phase96 pinned pre-raw contract failed: {exc}") from exc
    if pre_raw.get("raw_reads") != 0 or pre_raw.get("native_solver_invocations") != 0:
        raise fail("Phase96 pre-raw verifier reported native/raw activity")
    if pre_raw.get("accuracy_scored") is not False:
        raise fail("Phase96 pre-raw verifier reported accuracy")
    execution = manifest.get("execution_authorization")
    if not isinstance(execution, dict) or execution.get("raw_execution_authorized") is not True:
        raise fail("Phase96 raw authorization is not pinned")
    try:
        authorization = evaluator.verify_authorization(manifest)
    except Exception as exc:
        raise fail(f"Phase96 raw authorization verification failed: {exc}") from exc
    return evaluator, pre_raw, manifest, authorization


def _safe_relative_path(path_text: str, route: str, name: str) -> Path:
    path = Path(path_text)
    if path.is_absolute() or ".." in path.parts or path.name != name:
        raise fail(f"unsafe inherited raw path: {route}/{name}")
    return ROOT / path


def load_inherited_raw_inputs() -> dict[str, dict[str, dict[str, Any]]]:
    """Resolve and stat Phase91 paths without opening any raw input."""

    inherited = read_json(PHASE91_MANIFEST, "Phase91 inherited raw manifest")
    routes = inherited.get("routes")
    if not isinstance(routes, list):
        raise fail("Phase91 inherited route list is missing")
    by_route = {item.get("dataset_id"): item for item in routes if isinstance(item, dict)}
    resolved: dict[str, dict[str, dict[str, Any]]] = {}
    for route in ROUTES:
        record = by_route.get(route)
        if not isinstance(record, dict):
            raise fail(f"Phase91 inherited route missing: {route}")
        raw = record.get("raw_inputs")
        if not isinstance(raw, dict) or tuple(raw) != RAW_NAMES:
            raise fail(f"Phase91 inherited raw-input set changed: {route}")
        resolved[route] = {}
        for name in RAW_NAMES:
            pin = raw.get(name)
            if not isinstance(pin, dict) or not isinstance(pin.get("path"), str):
                raise fail(f"Phase91 raw pin malformed: {route}/{name}")
            path_text = pin["path"]
            path = _safe_relative_path(path_text, route, name)
            historical_sha = pin.get("sha256")
            if not isinstance(historical_sha, str) or len(historical_sha) != 64:
                raise fail(f"Phase91 raw SHA pin malformed: {route}/{name}")
            try:
                stat = path.stat()
            except OSError as exc:
                raise fail(f"missing inherited Phase91 raw input: {path}: {exc}") from exc
            if not path.is_file():
                raise fail(f"inherited Phase91 raw input is not a regular file: {path}")
            # This is deliberately metadata-only.  The current file is not
            # hashed or opened; the native process owns its raw-byte read.
            resolved[route][name] = {
                "path": path_text,
                "bytes": stat.st_size,
                "historical_sha256_pin": historical_sha,
                "current_sha256_read": False,
                "exists_before_launch": True,
                "read_by_runner": False,
                "copied_or_transformed": False,
                "source_manifest": relative(PHASE91_MANIFEST),
            }
    return resolved


def materialize_command(
    record: dict[str, Any],
    raw: dict[str, dict[str, Any]],
    evaluator: Any,
) -> list[str]:
    route = record.get("dataset_id")
    if route not in ROUTES:
        raise fail(f"unknown Phase96 route: {route}")
    command = record.get("command")
    evaluator._validate_command(route, command, record)
    materialized = list(command)
    for flag, name in RAW_FLAGS:
        index = materialized.index(flag) + 1
        pin = raw.get(name)
        if not isinstance(pin, dict) or not isinstance(pin.get("path"), str):
            raise fail(f"resolved raw pin missing: {route}/{name}")
        _safe_relative_path(pin["path"], route, name)
        materialized[index] = pin["path"]
    if materialized.count("--native-source-clock-c0d-phase96-main-diagnostics") != 1:
        raise fail(f"Phase96 selector disappeared during materialization: {route}")
    return materialized


def archive_metadata() -> dict[str, Any]:
    originals = [
        Path("/tmp/phase38-debug.PhRaoH/libgnss_lib.a"),
        Path("/tmp/phase38-debug.PhRaoH/libgnss_lib_solvers.a"),
    ]
    current = [
        Path("/dev/shm/phase96-build-space/libgnss_lib.a"),
        Path("/dev/shm/phase96-build-space/libgnss_lib_solvers.a"),
    ]

    def one(path: Path) -> dict[str, Any]:
        try:
            stat = path.stat()
            return {"path": str(path), "present": path.is_file(), "bytes": stat.st_size}
        except OSError:
            return {"path": str(path), "present": False, "bytes": None}

    return {
        "original_paths": [one(path) for path in originals],
        "current_paths": [one(path) for path in current],
        "user_data_deleted": False,
        "operation": "reversible move of two old temporary build archives; no user/repo data deletion",
    }


def execute_matrix(
    manifest: dict[str, Any],
    evaluator: Any,
) -> list[dict[str, Any]]:
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite Phase96 output root: {OUTPUT_ROOT}")
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    records = manifest.get("routes")
    if not isinstance(records, list) or [item.get("dataset_id") for item in records] != list(ROUTES):
        raise fail("Phase96 route order/count changed")
    raw_inputs = load_inherited_raw_inputs()
    environment = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = local_lib + (
        ":" + environment["LD_LIBRARY_PATH"] if environment.get("LD_LIBRARY_PATH") else ""
    )
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
        command = materialize_command(record, raw_inputs[route], evaluator)
        stdout_path = route_dir / "stdout.log"
        stderr_path = route_dir / "stderr.log"
        started = time.time()
        return_code: int | None = None
        interrupted = False
        launch_error = ""
        with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
            try:
                completed = subprocess.run(
                    command,
                    cwd=ROOT,
                    env=environment,
                    stdout=stdout,
                    stderr=stderr,
                    check=False,
                )
                return_code = completed.returncode
            except KeyboardInterrupt:
                interrupted = True
                stderr.write(b"\nPhase96 runner interrupted; partial output preserved.\n")
            except OSError as exc:
                launch_error = str(exc)
                stderr.write(f"\nPhase96 native launch failed: {exc}\n".encode())
        route_metadata = {
            "schema_version": "smartphone-r5-phase96-main-c0d-diagnostic-route-execution.v1",
            "dataset_id": route,
            "run_number": 1,
            "command": command,
            "raw_inputs": raw_inputs[route],
            "planned_output": {
                "summary": relative(summary_path),
                "withheld_solution_output": relative(withheld_path),
            },
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
            "runner_read_truth_mat_coordinate_base_kaggle": False,
            "raw_content_copied_or_transformed": False,
        }
        atomic_json(route_dir / "run_metadata.json", route_metadata)
        metadata.append(route_metadata)
    return metadata


def _json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): _json_safe(child) for key, child in value.items()}
    if isinstance(value, list):
        return [_json_safe(child) for child in value]
    return value


def _find_forbidden_solution_keys(value: Any, path: str = "summary") -> list[str]:
    forbidden = {
        "latitude",
        "longitude",
        "position_ecef",
        "receiver_clock_bias",
        "epoch_clock_drift_mps",
        "epoch_velocity_nav_mps",
        "solutions",
    }
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


def _pick(mapping: Any, keys: tuple[str, ...]) -> dict[str, Any] | None:
    if not isinstance(mapping, dict):
        return None
    return {key: _json_safe(mapping.get(key)) for key in keys}


def _compact_families(value: Any) -> list[dict[str, Any]]:
    if not isinstance(value, list):
        return []
    keys = (
        "family",
        "factor_count",
        "finite_factor_count",
        "nonfinite_factor_count",
        "initial_cost",
    )
    return [_pick(item, keys) for item in value if isinstance(item, dict)]  # type: ignore[list-item]


def _compact_norms(value: Any) -> list[dict[str, Any]]:
    if not isinstance(value, list):
        return []
    keys = (
        "family",
        "variable_bucket",
        "contribution_count",
        "finite_contribution_count",
        "nonfinite_contribution_count",
        "gradient_l2_norm",
        "normal_diagonal_l2_norm",
        "normal_diagonal_min",
        "normal_diagonal_max",
    )
    return [_pick(item, keys) for item in value if isinstance(item, dict)]  # type: ignore[list-item]


def _compact_trials(value: Any) -> list[dict[str, Any]]:
    if not isinstance(value, list):
        return []
    keys = (
        "trial_index",
        "outer_iteration",
        "lambda",
        "old_linearized_cost",
        "new_linearized_cost",
        "predicted_reduction",
        "candidate_nonlinear_cost",
        "actual_reduction",
        "model_fidelity",
        "candidate_finite",
        "linear_system_solved",
        "linear_system_status",
        "rejection_reason",
    )
    return [_pick(item, keys) for item in value[:MAX_TRIALS] if isinstance(item, dict)]  # type: ignore[list-item]


def _read_limited(path: Path, limit: int = 4096) -> str:
    try:
        value = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return "diagnostic log unavailable"
    return value[:limit]


def _stat_pin(path: Path, label: str) -> dict[str, Any]:
    try:
        stat = path.stat()
    except OSError as exc:
        return {"path": relative(path), "present": False, "bytes": None, "sha256": None, "error": str(exc)}
    return {
        "path": relative(path),
        "present": path.is_file(),
        "bytes": stat.st_size,
        "sha256": sha256_file(path, label) if path.is_file() else None,
    }


def compact_route_report(
    metadata: dict[str, Any],
    record: dict[str, Any],
) -> dict[str, Any]:
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
        "expected_output": {
            "domain_rows": DOMAIN_ROWS[route],
            "problem_epochs": DOMAIN_ROWS[route] + 1,
            "summary": relative(summary_path),
            "withheld_solution_output": relative(withheld_path),
            "solution_rows_published": False,
            "observed_solution_rows": None,
            "coverage": "structural expected coverage only; solution rows withheld",
        },
        "solution_output_published": False,
        "accuracy_output_published": False,
        "truth_used": False,
        "mat_used": False,
        "kaggle_or_token_accessed": False,
        "runner_read_raw_bytes": False,
        "runner_read_truth_mat_coordinate_base_kaggle": False,
        "raw_content_copied_or_transformed": False,
        "logs": {
            "stdout": _stat_pin(stdout_path, f"Phase96 stdout {route}"),
            "stderr": _stat_pin(stderr_path, f"Phase96 stderr {route}"),
        },
        "withheld_solution_output_present": withheld_path.is_file(),
    }
    summary: dict[str, Any] | None = None
    summary_error = ""
    if summary_path.is_file():
        try:
            summary = read_json(summary_path, f"Phase96 native summary {route}")
        except Phase96StructuralError as exc:
            summary_error = str(exc)
    if summary is None:
        report.update({
            "summary_present": False,
            "summary_error": summary_error or "native summary was not produced",
            "stderr_observation": _read_limited(stderr_path),
            "phase96_main": None,
            "telemetry": None,
            "capture_gates": {
                "native_process_completed": isinstance(metadata.get("return_code"), int) and not metadata.get("launch_error"),
                "expected_diagnostic_return_code": metadata.get("return_code") == 1,
                "summary_present": False,
                "phase96_graph_observed": False,
                "factor_family_costs": False,
                "gradient_normal_norms": False,
                "first_ten_lm_trials": False,
                "no_solution_or_accuracy_publication": not withheld_path.is_file(),
            },
        })
        return report

    forbidden_keys = _find_forbidden_solution_keys(summary)
    phase96 = summary.get("phase96_main")
    gnss = summary.get("gnss_first")
    main = summary.get("main")
    gnss_preflight = summary.get("gnss_first_preflight")
    main_preflight = summary.get("main_preflight")
    report.update({
        "summary_present": True,
        "summary": _stat_pin(summary_path, f"Phase96 summary {route}"),
        "summary_schema": summary.get("schema_version"),
        "summary_status": summary.get("status"),
        "failure_stage": summary.get("failure_stage"),
        "failure_reason": summary.get("failure_reason"),
        "exception_type": summary.get("exception_type"),
        "exception_message": summary.get("exception_message"),
        "forbidden_summary_keys": forbidden_keys,
        "gnss_first_preflight": _json_safe(gnss_preflight),
        "main_preflight": _json_safe(main_preflight),
        "gnss_first": _pick(gnss, (
            "attempted", "result_returned", "iterations", "initial_cost", "final_cost",
            "c0d_factor_count", "c0d_accepted_outer_iterations",
            "c0d_inner_lambda_attempts", "c0d_active_solve_attempted",
            "c0d_active_solve_finite_costs", "strict_cost_progress",
            "optimized_d_epoch_count", "optimized_d_finite_count",
            "optimized_d_nonfinite_count", "optimized_d_coverage",
            "exact_retained_key_alignment", "terminal_branch",
        )),
        "main": _pick(main, (
            "attempted", "result_returned", "problem_epoch_count",
            "position_solution_size", "position_size_matches_problem_epochs",
            "receiver_clock_solution_size", "receiver_clock_size_matches_problem_epochs",
            "all_positions_earth_valid", "all_receiver_clocks_finite",
            "optimized_d_epoch_count", "optimized_d_size_matches_problem_epochs",
            "optimized_d_finite_count", "optimized_d_all_finite",
            "velocity_epoch_count", "velocity_size_matches_problem_epochs",
            "all_velocities_finite", "exact_retained_key_alignment",
            "gnss_first_progress", "c0d_factor_count", "active_solve_attempted",
            "accepted_outer_iterations", "inner_lambda_attempts", "initial_cost",
            "final_cost", "active_solve_costs_finite",
            "final_cost_strictly_less_than_initial", "initial_lambda",
            "maximum_lambda", "final_lambda", "conditioning_proxy",
            "termination_trace_complete", "terminal_branch", "contract_passed",
        )),
    })
    phase96_compact = None
    if isinstance(phase96, dict):
        phase96_compact = {
            "enabled": phase96.get("enabled"),
            "attempted": phase96.get("attempted"),
            "graph_observed": phase96.get("graph_observed"),
            "initial_linearization_observed": phase96.get("initial_linearization_observed"),
            "trial_trace_complete": phase96.get("trial_trace_complete"),
            "trial_limit": phase96.get("trial_limit"),
            "graph_factor_count": phase96.get("graph_factor_count"),
            "graph_value_count": phase96.get("graph_value_count"),
            "graph_initial_cost": phase96.get("graph_initial_cost"),
            "initial_cost": phase96.get("initial_cost"),
            "final_cost": phase96.get("final_cost"),
            "accepted_outer_iterations": phase96.get("accepted_outer_iterations"),
            "terminal_branch": phase96.get("terminal_branch"),
            "factor_family_cost_sum": phase96.get("factor_family_cost_sum"),
            "factor_families": _compact_families(phase96.get("factor_families")),
            "variable_family_norms": _compact_norms(phase96.get("variable_family_norms")),
            "lm_trials": _compact_trials(phase96.get("lm_trials")),
            "exceptions": _json_safe(phase96.get("exceptions", [])),
        }
    report["phase96_main"] = _json_safe(phase96_compact)
    graph_observed = bool(
        isinstance(phase96, dict)
        and phase96.get("enabled") is True
        and phase96.get("attempted") is True
        and phase96.get("graph_observed") is True
        and phase96.get("initial_linearization_observed") is True
    )
    families = phase96_compact.get("factor_families", []) if isinstance(phase96_compact, dict) else []
    norms = phase96_compact.get("variable_family_norms", []) if isinstance(phase96_compact, dict) else []
    trials = phase96_compact.get("lm_trials", []) if isinstance(phase96_compact, dict) else []
    trace_complete = bool(isinstance(phase96, dict) and phase96.get("trial_trace_complete") is True)
    no_publication = bool(
        summary.get("solution_output_published") is False
        and summary.get("accuracy_output_published") is False
        and summary.get("truth_used") is False
        and summary.get("mat_used") is False
        and summary.get("kaggle_or_token_accessed") is False
        and summary.get("raw_truth_mat_kaggle_forbidden") is True
        and not forbidden_keys
        and not withheld_path.is_file()
    )
    report["capture_gates"] = {
        "native_process_completed": isinstance(metadata.get("return_code"), int) and not metadata.get("launch_error"),
        "expected_diagnostic_return_code": metadata.get("return_code") == 1,
        "summary_present": True,
        "phase96_graph_observed": graph_observed,
        "factor_family_costs": graph_observed and bool(families),
        "gradient_normal_norms": graph_observed and bool(norms),
        "first_ten_lm_trials": graph_observed and len(trials) <= MAX_TRIALS and bool(trials) and trace_complete,
        "linear_solver_exception_telemetry": graph_observed and isinstance(phase96_compact.get("exceptions", []), list),
        "no_solution_or_accuracy_publication": no_publication,
    }
    report["diagnostic_capture_complete"] = all(report["capture_gates"].values())
    return report


def build_result(
    metadata: list[dict[str, Any]],
    manifest: dict[str, Any],
    pre_raw: dict[str, Any],
    authorization: dict[str, Any],
) -> dict[str, Any]:
    record_by_route = {item["dataset_id"]: item for item in manifest["routes"]}
    reports = {
        item["dataset_id"]: compact_route_report(item, record_by_route[item["dataset_id"]])
        for item in metadata
    }
    exact_matrix = (
        len(metadata) == len(ROUTES)
        and [item.get("dataset_id") for item in metadata] == list(ROUTES)
        and all(item.get("run_number") == 1 for item in metadata)
    )
    all_capture = bool(reports) and all(
        item.get("diagnostic_capture_complete") is True for item in reports.values()
    )
    no_solution = bool(reports) and all(
        item.get("capture_gates", {}).get("no_solution_or_accuracy_publication") is True
        for item in reports.values()
    )
    gates = {
        "implementation_and_binary_pins": manifest.get("implementation", {}).get("commit") == "bcdcd952943623650cdf77410c17e9ec87f45d7f",
        "exactly_two_routes_one_run_each": exact_matrix,
        "diagnostic_telemetry_captured": all_capture,
        "factor_family_costs_captured": bool(reports) and all(item.get("capture_gates", {}).get("factor_family_costs") is True for item in reports.values()),
        "gradient_normal_norms_captured": bool(reports) and all(item.get("capture_gates", {}).get("gradient_normal_norms") is True for item in reports.values()),
        "first_ten_lm_trials_captured": bool(reports) and all(item.get("capture_gates", {}).get("first_ten_lm_trials") is True for item in reports.values()),
        "linear_solver_exception_telemetry": bool(reports) and all(item.get("capture_gates", {}).get("linear_solver_exception_telemetry") is True for item in reports.values()),
        "no_solution_or_accuracy_publication": no_solution,
        "truth_free": True,
        "accuracy_not_scored": True,
        "raw_only_command_policy": True,
    }
    gates["all_capture_gates_anded"] = all(gates.values())
    failed = [name for name, passed in gates.items() if passed is False]
    for route, report in reports.items():
        failed.extend(
            f"{route}:{name}"
            for name, passed in report.get("capture_gates", {}).items()
            if passed is False
        )
    return {
        "schema_version": RESULT_SCHEMA,
        "phase": 96,
        "execution_label": "Luna Max",
        "status": "captured-phase96-main-c0d-diagnostic" if gates["all_capture_gates_anded"] else "no-go-phase96-main-c0d-diagnostic-capture",
        "decision": (
            "Diagnostic telemetry captured; no convergence, accuracy, solution, or promotion claim is made."
            if gates["all_capture_gates_anded"]
            else "Diagnostic capture failed closed; no solution, accuracy, or promotion lane is available."
        ),
        "diagnostic_only": True,
        "truth_free": True,
        "accuracy_scored": False,
        "solution_output_published": False,
        "promotion_authorized": False,
        "stop_before_truth_accuracy_submission": True,
        "freeze": {
            "path": relative(FREEZE),
            "sha256": sha256_file(FREEZE, "Phase96 freeze"),
        },
        "authorization": {
            "path": relative(AUTHORIZATION),
            "sha256": sha256_file(AUTHORIZATION, "Phase96 raw authorization"),
            "status": authorization.get("status"),
        },
        "pre_raw_verification": pre_raw,
        "evaluator": {"path": relative(PRE_RAW), "sha256": sha256_file(PRE_RAW, "Phase96 evaluator")},
        "manifest": {"path": relative(MANIFEST), "sha256": sha256_file(MANIFEST, "Phase96 manifest")},
        "execution_wrapper": {"path": relative(Path(__file__).resolve()), "sha256": sha256_file(Path(__file__).resolve(), "Phase96 wrapper")},
        "implementation": manifest.get("implementation"),
        "archive_record": archive_metadata(),
        "matrix": {
            "candidate_count": 1,
            "routes": len(ROUTES),
            "runs_per_route": 1,
            "candidate_runs_total": len(metadata),
            "native_solver_invocations": len(metadata),
            "controls": 0,
            "reruns": 0,
            "fallbacks": 0,
            "raw_device_gnss_route_arguments": len(metadata),
            "raw_device_imu_route_arguments": len(metadata),
            "broadcast_navigation_route_arguments": len(metadata),
            "runner_raw_byte_reads": 0,
            "truth_reads": 0,
            "mat_reads_or_generated": 0,
            "precomputed_coordinate_reads": 0,
            "base_rinex_reads": 0,
            "accuracy_calculations": 0,
            "kaggle_or_token_access": 0,
            "route_score_selection": False,
        },
        "routes": reports,
        "gates": {**gates, "all_passed": gates["all_capture_gates_anded"]},
        "failed_gates": failed,
        "read_accounting": {
            "native_solver_invocations": len(metadata),
            "raw_device_gnss_route_arguments": len(metadata),
            "raw_device_imu_route_arguments": len(metadata),
            "broadcast_navigation_route_arguments": len(metadata),
            "runner_raw_input_byte_reads": 0,
            "raw_input_hash_reads": 0,
            "base_rinex_reads": 0,
            "truth_reads": 0,
            "mat_reads_or_generated": 0,
            "precomputed_coordinate_reads": 0,
            "validation_holdout_reads": 0,
            "kaggle_or_token_access": 0,
            "accuracy_calculations": 0,
            "accuracy_scored": False,
            "route_score_selection": False,
            "reruns": 0,
            "fallbacks": 0,
            "logs_and_partial_results_preserved": True,
        },
        "raw_input_provenance": {
            "names_exact": list(RAW_NAMES),
            "roles": {
                "device_gnss.csv": "Android raw GNSS only",
                "device_imu.csv": "Android raw IMU only",
                "brdc.nav": "broadcast navigation only",
            },
            "wrapper_path_resolution": "exact retained Phase91 route/key path; metadata-only stat; no copy/transform",
            "wrapper_raw_byte_reads": 0,
        },
        "forbidden_lanes": {
            "truth": False,
            "MAT": False,
            "precomputed_coordinates": False,
            "base": False,
            "Kaggle_or_token": False,
            "accuracy": False,
            "solution_rows": False,
        },
    }


def result_markdown(result: dict[str, Any]) -> str:
    lines = [
        "# Phase96 main C0/D diagnostic structural result",
        "",
        f"- Status: `{result['status']}`",
        f"- Decision: {result['decision']}",
        "- Diagnostic-only: **true**; solution rows and accuracy: **withheld**",
        "- Matrix: exactly two primary routes × one native invocation, sequential, no rerun/fallback",
        "- Raw inputs: device_gnss.csv, device_imu.csv, brdc.nav only; wrapper raw-byte reads: 0",
        "- Truth/MAT/base/precomputed-coordinate/Kaggle/token/accuracy activity: 0",
        "",
        "## Route diagnostic summary",
        "",
        "| Route | Return | GNSS-first accepted / cost | Main accepted / cost | Factor families | Norm buckets | LM trials | Capture |",
        "|---|---:|---|---|---:|---:|---:|---|",
    ]
    for route in ROUTES:
        item = result["routes"].get(route, {})
        gnss = item.get("gnss_first") or {}
        main = item.get("main") or {}
        phase96 = item.get("phase96_main") or {}
        lines.append(
            "| `{route}` | `{ret}` | `{ga} / {gi} → {gf}` | `{ma} / {mi} → {mf}` | `{families}` | `{norms}` | `{trials}` | `{capture}` |".format(
                route=route,
                ret=item.get("return_code"),
                ga=gnss.get("c0d_accepted_outer_iterations", "n/a"),
                gi=gnss.get("initial_cost", "n/a"),
                gf=gnss.get("final_cost", "n/a"),
                ma=main.get("accepted_outer_iterations", "n/a"),
                mi=main.get("initial_cost", "n/a"),
                mf=main.get("final_cost", "n/a"),
                families=len(phase96.get("factor_families", [])),
                norms=len(phase96.get("variable_family_norms", [])),
                trials=len(phase96.get("lm_trials", [])),
                capture=item.get("diagnostic_capture_complete"),
            )
        )
        lines.append(f"  - Capture gates: `{item.get('capture_gates', {})}`")
        lines.append(f"  - Phase96 terminal branch: `{phase96.get('terminal_branch', 'n/a')}`")
    lines.extend([
        "",
        "The factor-family costs, variable/family gradient and normal-diagonal norms, "
        "and first-ten existing LM trials are diagnostic observations only. No "
        "solution coordinate, raw observation, truth value, or accuracy score is published.",
        "",
    ])
    return "\n".join(lines)


def _metadata_from_existing() -> list[dict[str, Any]]:
    values: list[dict[str, Any]] = []
    for route in ROUTES:
        path = OUTPUT_ROOT / route.replace("/", "__") / "run_metadata.json"
        values.append(read_json(path, f"Phase96 route metadata {route}"))
    return values


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
        metadata = execute_matrix(manifest, evaluator) if args.execute else _metadata_from_existing()
        result = build_result(metadata, manifest, pre_raw, authorization)
        target = args.result_json.resolve()
        atomic_json(target, result)
        atomic_text(target.with_suffix(".md"), result_markdown(result))
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["gates"]["all_capture_gates_anded"] else 1
    except (Phase96StructuralError, OSError) as exc:
        print(f"phase96 structural runner: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
