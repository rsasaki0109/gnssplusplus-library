#!/usr/bin/env python3
"""Execute and seal the authorized Phase98 compact diagnostic matrix.

This is a one-shot launcher.  It resolves the exact Phase95 raw-input paths
from the sealed Phase95 result, performs metadata-only ``stat`` checks, and
substitutes those paths into the frozen Phase98 commands.  It never hashes,
opens, copies, or transforms raw bytes.  The native command receives only raw
Android GNSS, raw Android IMU, and broadcast navigation.  The native summary
is reduced to the compact Phase96/98 telemetry surface; the required solution
CSV path is withheld and is never published or committed.
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
PRE_RAW = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase98_solver_rank_diagnostic.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase98_solver_rank_diagnostic_execution_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase98_solver_rank_diagnostic_raw_execution_authorization_v1.json"
PHASE95_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase95_raw_input_path_corrected_structural_result_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase98-solver-rank-diagnostic-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase98_solver_rank_diagnostic_structural_result_v1.json"
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
RESULT_SCHEMA = "smartphone-r5-phase98-solver-rank-diagnostic-structural-result.v1"
PHASE97_RESULT = "docs/use_cases/records/smartphone_r5_phase97_singular_system_diagnostic_structural_result_v1.json"
PHASE97_RESULT_SHA = "abf4027bec33d51b54fcfa2c3def55ff9ea22568fca7eeeb83049271cf2efecc"
PHASE97_RESULT_COMMIT = "dd32a32ecc2960322511ecf6753431bda64c603a"


class Phase98StructuralError(ValueError):
    """Raised when Phase98 execution or capture fails closed."""


def fail(message: str) -> Phase98StructuralError:
    return Phase98StructuralError(message)


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
        raise fail(f"missing Phase98 evaluator: {PRE_RAW}")
    spec = importlib.util.spec_from_file_location("phase98_pre_raw", PRE_RAW)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load Phase98 evaluator: {PRE_RAW}")
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
        raise fail(f"Phase98 pinned contract failed: {exc}") from exc
    if not authorization.get("release_boundary", {}).get("raw_execution_authorized"):
        raise fail("Phase98 one-shot raw authorization is not active")
    if pre_raw.get("raw_reads") != 0 or pre_raw.get("native_solver_invocations") != 0:
        raise fail("pre-raw verifier reported activity")
    if pre_raw.get("accuracy_scored") is not False:
        raise fail("pre-raw verifier reported accuracy")
    return evaluator, pre_raw, manifest, authorization


def load_phase95_raw_paths(evaluator: Any) -> dict[str, dict[str, str]]:
    paths = evaluator.phase95_paths()
    if list(paths) != list(ROUTES):
        raise fail("Phase95 raw route order changed")
    return paths


def stat_raw_inputs(raw_paths: dict[str, dict[str, str]]) -> dict[str, dict[str, dict[str, Any]]]:
    """Stat exact paths only; this function never reads raw file contents."""

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
        raise fail(f"unknown Phase98 route: {route}")
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
        raise fail(f"refusing to overwrite one-shot Phase98 output root: {OUTPUT_ROOT}")
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    records = manifest.get("routes")
    if not isinstance(records, list) or [item.get("dataset_id") for item in records] != list(ROUTES):
        raise fail("Phase98 route order/count changed")
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
                stderr.write(b"\nPhase98 runner interrupted; partial output preserved.\n")
            except OSError as exc:
                launch_error = str(exc)
                stderr.write(f"\nPhase98 native launch failed: {exc}\n".encode())
        metadata_record = {
            "schema_version": "smartphone-r5-phase98-solver-rank-diagnostic-route-execution.v1",
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


def _digest_text(value: Any) -> str | None:
    if not isinstance(value, str) or not value:
        return None
    return "sha256:" + hashlib.sha256(value.encode("utf-8")).hexdigest()


def _compact_key(value: Any) -> dict[str, Any] | None:
    if not isinstance(value, dict):
        return None
    return _pick(value, ("numeric_key", "symbol_character", "symbol_index"))


def _compact_trial(trial: Any, solver: dict[str, Any], exception_digest: str | None) -> dict[str, Any]:
    record = _pick(trial, ("trial_index", "outer_iteration", "lambda", "predicted_reduction", "actual_reduction", "model_fidelity", "candidate_finite", "linear_system_solved", "linear_system_status", "rejection_reason"))
    record.update({
        "solver_type": solver.get("solver_type"),
        "solver_branch": solver.get("solver_branch"),
        "elimination_function": solver.get("elimination_function"),
        "ordering_type": solver.get("ordering_type"),
        "ordering_size": solver.get("ordering_size"),
        "ordering_digest": solver.get("ordering_digest"),
        "diagonal_damping": solver.get("diagonal_damping"),
        # Phase96's pinned trial surface has no exception object.  Preserve
        # that fact explicitly instead of inventing a nearby key.
        "nearby_variable_available": False,
        "nearby_variable": None,
        "nearby_variable_status": "nearby_variable_unavailable",
        "exception_text_digest": exception_digest,
    })
    return record


def _compact_phase96(phase96: Any, solver: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(phase96, dict):
        return {"enabled": False, "attempted": False, "lm_trials": [], "exceptions": []}
    report = _pick(phase96, ("enabled", "attempted", "graph_observed", "initial_linearization_observed", "trial_trace_complete", "trial_limit", "graph_factor_count", "graph_value_count", "graph_initial_cost", "initial_cost", "final_cost", "accepted_outer_iterations", "terminal_branch", "factor_family_cost_sum"))
    families = phase96.get("factor_families") if isinstance(phase96.get("factor_families"), list) else []
    report["factor_families"] = [_pick(item, ("family", "factor_count", "finite_factor_count", "nonfinite_factor_count", "initial_cost")) for item in families if isinstance(item, dict)]
    norms = phase96.get("variable_family_norms") if isinstance(phase96.get("variable_family_norms"), list) else []
    report["variable_family_norms"] = [_pick(item, ("family", "variable_bucket", "contribution_count", "finite_contribution_count", "nonfinite_contribution_count", "gradient_l2_norm", "normal_diagonal_l2_norm", "normal_diagonal_min", "normal_diagonal_max")) for item in norms if isinstance(item, dict)]
    exceptions = phase96.get("exceptions") if isinstance(phase96.get("exceptions"), list) else []
    compact_exceptions = []
    digests: list[str] = []
    for item in exceptions:
        if not isinstance(item, dict):
            continue
        digest = _digest_text(item.get("message"))
        if digest:
            digests.append(digest)
        compact_exceptions.append({**_pick(item, ("stage", "classification", "type", "count")), "message_digest": digest})
    report["exceptions"] = compact_exceptions
    report["exception_text_digests"] = sorted(set(digests))
    trials = phase96.get("lm_trials") if isinstance(phase96.get("lm_trials"), list) else []
    report["lm_trials"] = [_compact_trial(item, solver, digests[0] if digests else None) for item in trials[:MAX_TRIALS] if isinstance(item, dict)]
    return report


def _compact_phase98(phase98: Any) -> dict[str, Any]:
    if not isinstance(phase98, dict):
        return {"enabled": False, "attempted": False, "exception_captured": False, "indeterminate_exceptions": []}
    report = _pick(phase98, ("enabled", "attempted", "exception_captured", "solver_type", "solver_branch", "elimination_function", "ordering_type", "explicit_ordering_present", "ordering_size", "ordering_digest", "diagonal_damping", "existing_lm_trial_limit"))
    exceptions = phase98.get("indeterminate_exceptions") if isinstance(phase98.get("indeterminate_exceptions"), list) else []
    compact = []
    for item in exceptions:
        if not isinstance(item, dict):
            continue
        compact.append({
            **_pick(item, ("stage", "lambda", "solver_type", "solver_branch", "elimination_function", "ordering_type", "explicit_ordering_present", "ordering_size", "ordering_digest", "diagonal_damping", "nearby_variable_available", "nearby_variable_status", "exception_type")),
            "nearby_variable": _compact_key(item.get("nearby_variable")),
            "exception_text_digest": _digest_text(item.get("exception_message")),
        })
    report["indeterminate_exceptions"] = compact
    return report


def _find_forbidden_solution_keys(value: Any, path: str = "summary") -> list[str]:
    forbidden = {"latitude", "longitude", "position_ecef", "receiver_clock_bias", "epoch_clock_drift_mps", "epoch_velocity_nav_mps", "epoch_velocities_ecef_mps", "solutions"}
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


def _stat(path: Path) -> dict[str, Any]:
    try:
        info = path.stat()
    except OSError as exc:
        return {"path": relative(path), "present": False, "bytes": None, "error": str(exc)}
    return {"path": relative(path), "present": path.is_file(), "bytes": info.st_size}


def _read_limited(path: Path, limit: int = 8192) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")[:limit]
    except OSError:
        return "diagnostic log unavailable"


def _phase97_reference() -> dict[str, Any]:
    # The large Phase97 artifact is referenced, never opened or copied here.
    # Its sealed route records contain no nearby key, so no family/incidence/
    # anchor attribution is made for an unavailable key.
    return {
        "path": PHASE97_RESULT,
        "sha256": PHASE97_RESULT_SHA,
        "commit": PHASE97_RESULT_COMMIT,
        "reused_without_copy": True,
        "nearby_variable_capture_status": "nearby_variable_unavailable",
        "nearby_key_present": False,
        "nearby_key_family": None,
        "nearby_key_factor_incidence": None,
        "nearby_key_anchor_or_prior_incidence": None,
        "interpretation": "Sealed Phase97 route records expose no nearby key; family, factor incidence, and anchor/prior attribution are therefore unavailable and are not inferred.",
    }


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
        "logs": {"stdout": _stat(stdout_path), "stderr": _stat(stderr_path)},
        "withheld_solution_output_present": withheld_path.is_file(),
        "phase97_reference": _phase97_reference(),
    }
    if not summary_path.is_file():
        report.update({"summary_present": False, "summary_error": "native summary was not produced", "stderr_observation": _read_limited(stderr_path), "gnss_first": None, "main": None, "phase96_main": None, "phase98_solver": None, "lm_trials": [], "capture_gates": {"native_process_completed": isinstance(metadata.get("return_code"), int) and not metadata.get("launch_error"), "summary_present": False, "phase98_boundary_attempted": False, "ten_trials": False, "nearby_key_or_explicit_unavailable": False, "exception_text_digest": False, "no_solution_or_accuracy_publication": not withheld_path.is_file()}, "diagnostic_capture_complete": False})
        return report
    try:
        summary = read_json(summary_path, f"Phase98 native summary {route}")
    except Phase98StructuralError as exc:
        report.update({"summary_present": False, "summary_error": str(exc), "stderr_observation": _read_limited(stderr_path), "gnss_first": None, "main": None, "phase96_main": None, "phase98_solver": None, "lm_trials": [], "capture_gates": {"native_process_completed": isinstance(metadata.get("return_code"), int) and not metadata.get("launch_error"), "summary_present": False, "phase98_boundary_attempted": False, "ten_trials": False, "nearby_key_or_explicit_unavailable": False, "exception_text_digest": False, "no_solution_or_accuracy_publication": not withheld_path.is_file()}, "diagnostic_capture_complete": False})
        return report
    forbidden_keys = _find_forbidden_solution_keys(summary)
    solver = _compact_phase98(summary.get("phase98_solver"))
    phase96 = _compact_phase96(summary.get("phase96_main"), solver)
    gnss = summary.get("gnss_first")
    main = summary.get("main")
    report.update({
        "summary_present": True,
        "summary": _stat(summary_path),
        "summary_schema": summary.get("schema_version"),
        "summary_status": summary.get("status"),
        "failure_stage": summary.get("failure_stage"),
        "failure_reason": summary.get("failure_reason"),
        "exception_type": summary.get("exception_type"),
        "forbidden_summary_keys": forbidden_keys,
        "gnss_first": _pick(gnss, ("attempted", "result_returned", "iterations", "initial_cost", "final_cost", "c0d_factor_count", "c0d_accepted_outer_iterations", "c0d_active_solve_finite_costs", "strict_cost_progress", "optimized_d_epoch_count", "optimized_d_finite_count", "optimized_d_coverage", "exact_retained_key_alignment", "terminal_branch")),
        "main": _pick(main, ("attempted", "result_returned", "problem_epoch_count", "position_solution_size", "position_size_matches_problem_epochs", "receiver_clock_solution_size", "receiver_clock_size_matches_problem_epochs", "all_positions_earth_valid", "all_receiver_clocks_finite", "optimized_d_epoch_count", "optimized_d_size_matches_problem_epochs", "optimized_d_finite_count", "optimized_d_all_finite", "velocity_epoch_count", "velocity_size_matches_problem_epochs", "all_velocities_finite", "exact_retained_key_alignment", "gnss_first_progress", "c0d_factor_count", "active_solve_attempted", "accepted_outer_iterations", "inner_lambda_attempts", "initial_cost", "final_cost", "active_solve_costs_finite", "final_cost_strictly_less_than_initial", "initial_lambda", "maximum_lambda", "final_lambda", "conditioning_proxy", "termination_trace_complete", "terminal_branch", "contract_passed")),
        "phase96_main": phase96,
        "phase98_solver": solver,
        "lm_trials": phase96.get("lm_trials", []),
    })
    phase98_attempted = solver.get("enabled") is True and solver.get("attempted") is True
    trials = report["lm_trials"]
    exception_digests = list(phase96.get("exception_text_digests", []))
    exception_digests.extend(item.get("exception_text_digest") for item in solver.get("indeterminate_exceptions", []) if item.get("exception_text_digest"))
    no_publication = bool(summary.get("solution_output_published") is False and summary.get("accuracy_output_published") is False and summary.get("truth_used") is False and summary.get("mat_used") is False and summary.get("kaggle_or_token_accessed") is False and summary.get("raw_truth_mat_kaggle_forbidden") is True and not forbidden_keys and not withheld_path.is_file())
    capture = {
        "native_process_completed": isinstance(metadata.get("return_code"), int) and not metadata.get("launch_error"),
        "summary_present": True,
        "phase98_boundary_attempted": phase98_attempted,
        "solver_ordering_elimination_damping": phase98_attempted and all(solver.get(key) not in (None, "") for key in ("solver_type", "solver_branch", "elimination_function", "ordering_type", "ordering_size", "ordering_digest", "diagonal_damping")),
        "ten_trials": phase98_attempted and len(trials) == MAX_TRIALS and phase96.get("trial_trace_complete") is True,
        "nearby_key_or_explicit_unavailable": phase98_attempted and len(trials) == MAX_TRIALS and all(isinstance(item.get("nearby_variable_available"), bool) and isinstance(item.get("nearby_variable_status"), str) and item.get("nearby_variable_status") for item in trials),
        "exception_text_digest": bool(exception_digests) or (phase98_attempted and not solver.get("indeterminate_exceptions")),
        "phase97_reference_without_copy": report["phase97_reference"].get("reused_without_copy") is True,
        "no_solution_or_accuracy_publication": no_publication,
    }
    report["exception_text_digests"] = sorted(set(exception_digests))
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
        "gnss_first_guard_and_progress_telemetry": bool(reports) and all(item.get("gnss_first") is not None for item in reports.values()),
        "main_validation_telemetry": bool(reports) and all(item.get("main") is not None for item in reports.values()),
        "phase96_factor_family_and_trial_telemetry": bool(reports) and all(item.get("capture_gates", {}).get("ten_trials") is True for item in reports.values()),
        "phase98_solver_boundary_telemetry": bool(reports) and all(item.get("capture_gates", {}).get("solver_ordering_elimination_damping") is True for item in reports.values()),
        "nearby_key_exact_or_explicit_unavailable": bool(reports) and all(item.get("capture_gates", {}).get("nearby_key_or_explicit_unavailable") is True for item in reports.values()),
        "phase97_family_incidence_anchor_reference_without_copy": bool(reports) and all(item.get("capture_gates", {}).get("phase97_reference_without_copy") is True for item in reports.values()),
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
        "phase": 98,
        "execution_label": "Luna Max",
        "status": "captured-phase98-compact-solver-diagnostic" if gates["all_capture_gates_anded"] else "no-go-phase98-compact-solver-diagnostic-capture",
        "decision": "Compact solver-boundary telemetry captured; no solution, accuracy, or promotion claim is made." if gates["all_capture_gates_anded"] else "Compact diagnostic capture failed closed; no solution, accuracy, or promotion lane is available.",
        "diagnostic_only": True,
        "truth_free": True,
        "accuracy_scored": False,
        "solution_output_published": False,
        "promotion_authorized": False,
        "stop_before_truth_accuracy_submission": True,
        "freeze": {"path": evaluator.relative(evaluator.FREEZE), "sha256": evaluator.FREEZE_SHA},
        "authorization": {"path": evaluator.relative(evaluator.AUTHORIZATION), "sha256": sha256_file(AUTHORIZATION, "Phase98 authorization"), "status": authorization.get("status")},
        "pre_raw_verification": pre_raw,
        "evaluator": {"path": relative(PRE_RAW), "sha256": sha256_file(PRE_RAW, "Phase98 evaluator")},
        "manifest": {"path": relative(MANIFEST), "sha256": sha256_file(MANIFEST, "Phase98 manifest")},
        "execution_wrapper": {"path": relative(Path(__file__).resolve()), "sha256": sha256_file(Path(__file__).resolve(), "Phase98 wrapper")},
        "phase95_path_source": {"path": relative(PHASE95_RESULT), "sha256": evaluator.PHASE95_RESULT_SHA, "materialization": "exact retained Phase95 result raw_inputs.path; metadata-only stat; no copy/transform"},
        "phase97_reference": _phase97_reference(),
        "implementation": manifest.get("implementation"),
        "matrix": {"candidate_count": 1, "routes": 2, "runs_per_route": len(metadata), "candidate_runs_total": len(metadata), "native_solver_invocations": len(metadata), "controls": 0, "reruns": 0, "fallbacks": 0, "raw_device_gnss_route_arguments": len(metadata), "raw_device_imu_route_arguments": len(metadata), "broadcast_navigation_route_arguments": len(metadata), "runner_raw_byte_reads": 0, "truth_reads": 0, "mat_reads_or_generated": 0, "precomputed_coordinate_reads": 0, "base_rinex_reads": 0, "accuracy_calculations": 0, "kaggle_or_token_access": 0, "route_score_selection": False, "large_phase97_artifact_reemission": False},
        "routes": reports,
        "gates": {**gates, "all_passed": gates["all_capture_gates_anded"]},
        "failed_gates": failed,
        "read_accounting": {"native_solver_invocations": len(metadata), "raw_device_gnss_reads": len(metadata), "raw_device_imu_reads": len(metadata), "broadcast_navigation_reads": len(metadata), "runner_raw_input_byte_reads": 0, "raw_input_hash_reads": 0, "base_rinex_reads": 0, "truth_reads": 0, "mat_reads_or_generated": 0, "precomputed_coordinate_reads": 0, "accuracy_calculations": 0, "accuracy_scored": False, "kaggle_or_token_access": 0, "route_score_selection": False, "reruns": 0, "fallbacks": 0, "raw_content_copied_or_transformed": False, "logs_and_partial_results_preserved": True},
        "raw_input_provenance": {"names_exact": list(RAW_NAMES), "roles": {"device_gnss.csv": "Android raw GNSS only", "device_imu.csv": "Android raw IMU only", "brdc.nav": "broadcast navigation only"}, "path_source": "Phase95 sealed structural result route raw_inputs.path", "wrapper_path_resolution": "exact path substitution after metadata-only stat", "wrapper_raw_byte_reads": 0, "copy_or_transform": False},
        "forbidden_lanes": {"truth": False, "MAT": False, "precomputed_coordinates": False, "base": False, "Kaggle_or_token": False, "accuracy": False, "solution_rows": False},
    })


def result_markdown(result: dict[str, Any]) -> str:
    lines = ["# Phase98 compact solver-rank diagnostic structural result", "", f"- Status: `{result['status']}`", f"- Decision: {result['decision']}", "- Diagnostic-only: **true**; solution rows and accuracy: **withheld**", "- Matrix: exactly MTV-A and LAX-T × one native invocation each, sequential, no rerun/fallback", "- Raw inputs: Phase95 exact device_gnss.csv, device_imu.csv, brdc.nav paths only; wrapper raw-byte reads: 0", "- Truth/MAT/base/precomputed-coordinate/Kaggle/token/accuracy activity: 0", "- Phase97 large incidence artifact: referenced by hash only; not opened, copied, or re-emitted", "", "## Route summary", "", "| Route | Return | Phase98 solver | Ordering | 10 trials | Nearby key | Capture |", "|---|---:|---|---|---:|---|---|"]
    for route in ROUTES:
        item = result["routes"].get(route, {})
        solver = item.get("phase98_solver") or {}
        trials = item.get("lm_trials") or []
        nearby = sum(1 for trial in trials if trial.get("nearby_variable_available"))
        lines.append(f"| `{route}` | `{item.get('return_code')}` | `{solver.get('solver_type', 'n/a')}` / `{solver.get('solver_branch', 'n/a')}` | `{solver.get('ordering_type', 'n/a')}` / `{solver.get('elimination_function', 'n/a')}` | `{len(trials)}` | `{nearby} exact / {len(trials) - nearby} unavailable` | `{item.get('diagnostic_capture_complete')}` |")
        lines.append(f"  - Phase96 costs/accepted: `{(item.get('phase96_main') or {}).get('initial_cost')}` → `{(item.get('phase96_main') or {}).get('final_cost')}`, `{(item.get('phase96_main') or {}).get('accepted_outer_iterations')}`; damping `{solver.get('diagonal_damping', 'n/a')}`; ordering digest `{solver.get('ordering_digest', 'n/a')}`")
        lines.append(f"  - Trial lambda/rejection/status: `{[(trial.get('lambda'), trial.get('rejection_reason'), trial.get('nearby_variable_status')) for trial in trials]}`")
        lines.append(f"  - Exception text digests: `{item.get('exception_text_digests', [])}`")
        lines.append(f"  - Phase97 nearby interpretation: `{item.get('phase97_reference', {}).get('interpretation')}`")
    lines.extend(["", "All graph/solver/LM data above is compact diagnostic telemetry only. No solution coordinate, raw observation, truth value, or accuracy score is published.", ""])
    return "\n".join(lines)


def metadata_from_existing() -> list[dict[str, Any]]:
    return [read_json(OUTPUT_ROOT / route.replace("/", "__") / "run_metadata.json", f"Phase98 route metadata {route}") for route in ROUTES]


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
    except (Phase98StructuralError, OSError) as exc:
        print(f"phase98 structural runner: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
