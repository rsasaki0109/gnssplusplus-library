#!/usr/bin/env python3
"""Run and seal the one-shot Phase99 raw structural matrix.

The wrapper resolves the exact raw paths through the pinned Phase95 path
materializer, which performs metadata-only existence/stat checks.  The native
program receives only raw device GNSS, raw device IMU, and broadcast
navigation.  Phase96/98 diagnostic selectors intentionally keep the native
solution CSV withheld; this wrapper never opens that path and never computes
accuracy.
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
PRE_RAW = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase99_main_multifrontal_qr.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase99_main_multifrontal_qr_solver_execution_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase99_main_multifrontal_qr_solver_raw_execution_authorization_v1.json"
PHASE95_WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase95_raw_input_path_corrected_execute.py"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase99-main-multifrontal-qr-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase99_main_multifrontal_qr_structural_result_v1.json"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
RAW_FLAGS = (
    ("--android-gnss", "device_gnss.csv"),
    ("--android-imu", "device_imu.csv"),
    ("--nav", "brdc.nav"),
)
RESULT_SCHEMA = "smartphone-r5-phase99-main-multifrontal-qr-structural-result.v1"
MAX_SPEED_MPS = 70.0


class Phase99StructuralError(ValueError):
    """Raised when execution or structural capture fails closed."""


def fail(message: str) -> Phase99StructuralError:
    return Phase99StructuralError(message)


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
        raise fail(f"missing Phase99 evaluator: {PRE_RAW}")
    spec = importlib.util.spec_from_file_location("phase99_contract", PRE_RAW)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load Phase99 evaluator: {PRE_RAW}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_pinned_contract() -> tuple[Any, dict[str, Any], dict[str, Any]]:
    evaluator = load_evaluator()
    try:
        pre_raw = evaluator.verify_pre_raw()
        manifest = evaluator.verify_manifest()
        authorization = evaluator.verify_authorization(manifest)
    except Exception as exc:
        raise fail(f"Phase99 pinned contract failed: {exc}") from exc
    if pre_raw.get("raw_reads") != 0 or pre_raw.get("native_solver_invocations") != 0:
        raise fail("Phase99 pre-raw verifier reported activity")
    if pre_raw.get("accuracy_scored") is not False:
        raise fail("Phase99 pre-raw verifier reported accuracy")
    if authorization.get("status") != "authorized-for-exact-two-route-phase99-main-multifrontal-qr-structural-execution":
        raise fail("Phase99 authorization status is not exact")
    return evaluator, manifest, authorization


def load_phase95_raw_inputs(evaluator: Any) -> dict[str, dict[str, dict[str, Any]]]:
    """Call the pinned Phase95 path resolver; it stats but does not read raw."""

    actual_hash = evaluator.sha256_file(PHASE95_WRAPPER, "Phase95 path wrapper")
    if actual_hash != evaluator.PHASE95_WRAPPER_SHA:
        raise fail("Phase95 path materializer hash changed")
    spec = importlib.util.spec_from_file_location("phase95_path_materializer", PHASE95_WRAPPER)
    if spec is None or spec.loader is None:
        raise fail("unable to load Phase95 path materializer")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    try:
        all_routes = module.load_inherited_raw_inputs()
    except Exception as exc:
        raise fail(f"Phase95 raw path materialization failed: {exc}") from exc
    if not isinstance(all_routes, dict):
        raise fail("Phase95 materializer did not return a route map")
    selected: dict[str, dict[str, dict[str, Any]]] = {}
    for route in ROUTES:
        record = all_routes.get(route)
        if not isinstance(record, dict) or set(record) != set(RAW_NAMES):
            raise fail(f"Phase95 materializer raw roles changed: {route}")
        selected[route] = {}
        for name in RAW_NAMES:
            pin = record[name]
            if not isinstance(pin, dict) or not isinstance(pin.get("path"), str):
                raise fail(f"Phase95 materializer pin malformed: {route}/{name}")
            actual = Path(pin["path"])
            if actual.is_absolute() or ".." in actual.parts or actual.name != name:
                raise fail(f"unsafe Phase95 materialized path: {route}/{name}")
            if not actual.is_file():
                raise fail(f"Phase95 materialized path is absent: {route}/{name}")
            if pin.get("read_by_runner") is not False:
                raise fail(f"Phase95 materializer read raw bytes: {route}/{name}")
            selected[route][name] = dict(pin)
    return selected


def materialize_command(record: dict[str, Any], raw: dict[str, dict[str, Any]], evaluator: Any) -> list[str]:
    route = record.get("dataset_id")
    if route not in ROUTES:
        raise fail(f"unknown Phase99 route: {route}")
    paths = {name: raw[name]["path"] for name in RAW_NAMES}
    evaluator._validate_command(route, record.get("command"), record, paths)
    command = list(record["command"])
    for flag, name in RAW_FLAGS:
        index = command.index(flag) + 1
        if not isinstance(raw.get(name), dict) or not isinstance(raw[name].get("path"), str):
            raise fail(f"materialized raw pin missing: {route}/{name}")
        command[index] = raw[name]["path"]
    return command


def execute_matrix(manifest: dict[str, Any], evaluator: Any) -> list[dict[str, Any]]:
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite Phase99 output root: {OUTPUT_ROOT}")
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    records = manifest.get("routes")
    if not isinstance(records, list) or [item.get("dataset_id") for item in records] != list(ROUTES):
        raise fail("Phase99 route order/count changed")
    raw_inputs = load_phase95_raw_inputs(evaluator)
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
        command = materialize_command(record, raw_inputs[route], evaluator)
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
                stderr.write(b"\nPhase99 wrapper interrupted; partial output preserved.\n")
            except OSError as exc:
                launch_error = str(exc)
                stderr.write(f"\nPhase99 native launch failed: {exc}\n".encode())
        metadata_record = {
            "schema_version": "smartphone-r5-phase99-main-multifrontal-qr-route-execution.v1",
            "dataset_id": route,
            "run_number": 1,
            "command": command,
            "raw_inputs": raw_inputs[route],
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
            "runner_read_raw_hashes": False,
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
        return {key: None for key in keys}
    return {key: _json_safe(mapping.get(key)) for key in keys}


def _finite_tree(value: Any) -> bool:
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, dict):
        return all(_finite_tree(child) for child in value.values())
    if isinstance(value, list):
        return all(_finite_tree(child) for child in value)
    return True


def _forbidden_solution_keys(value: Any, path: str = "summary") -> list[str]:
    forbidden = {
        "latitude", "longitude", "position_ecef", "receiver_clock_bias",
        "epoch_clock_drift_mps", "epoch_velocity_nav_mps", "epoch_velocities_ecef_mps",
        "solutions",
    }
    found: list[str] = []
    if isinstance(value, dict):
        for key, child in value.items():
            if str(key).lower() in forbidden:
                found.append(f"{path}.{key}")
            found.extend(_forbidden_solution_keys(child, f"{path}.{key}"))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            found.extend(_forbidden_solution_keys(child, f"{path}[{index}]"))
    return found


def _file_observation(path: Path) -> dict[str, Any]:
    try:
        info = path.stat()
    except OSError as exc:
        return {"path": relative(path), "present": False, "bytes": None, "sha256": None, "error": str(exc)}
    return {"path": relative(path), "present": path.is_file(), "bytes": info.st_size, "sha256": sha256_file(path, f"execution log {path.name}") if path.is_file() else None}


def _structural_route_report(metadata: dict[str, Any]) -> dict[str, Any]:
    route = metadata["dataset_id"]
    expected_problem_epochs = DOMAIN_ROWS[route] + 1
    planned = metadata["planned_output"]
    summary_path = ROOT / planned["summary"]
    withheld_path = ROOT / planned["withheld_solution_output"]
    report: dict[str, Any] = {
        "dataset_id": route,
        "run_number": metadata.get("run_number"),
        "return_code": metadata.get("return_code"),
        "diagnostic_return_code_expected": True,
        "interrupted": metadata.get("interrupted"),
        "launch_error": metadata.get("launch_error", ""),
        "expected_output": {
            "domain_rows": DOMAIN_ROWS[route],
            "problem_epochs": expected_problem_epochs,
            "summary": relative(summary_path),
            "withheld_solution_output": relative(withheld_path),
            "solution_rows_published": False,
            "coverage_basis": "native stage telemetry size/finite predicates; solution rows withheld",
        },
        "raw_inputs": metadata.get("raw_inputs"),
        "solution_output_published": False,
        "accuracy_output_published": False,
        "truth_used": False,
        "mat_used": False,
        "kaggle_or_token_accessed": False,
        "runner_read_raw_bytes": metadata.get("runner_read_raw_bytes"),
        "runner_read_raw_hashes": metadata.get("runner_read_raw_hashes"),
        "runner_read_truth_mat_base_coordinate_kaggle": metadata.get("runner_read_truth_mat_base_coordinate_kaggle"),
        "raw_content_copied_or_transformed": metadata.get("raw_content_copied_or_transformed"),
        "withheld_solution_output_present": withheld_path.is_file(),
        "logs": {"stdout": _file_observation(ROOT / metadata["stdout"]), "stderr": _file_observation(ROOT / metadata["stderr"])},
    }
    if not summary_path.is_file():
        report.update({
            "summary_present": False,
            "summary_sha256": None,
            "summary_error": "native diagnostic summary was not produced",
            "gnss_first_preflight": None,
            "main_preflight": None,
            "gnss_first": None,
            "main": None,
            "phase96_main": None,
            "phase98_solver": None,
            "gates": {
                "native_diagnostic_process_completed": isinstance(metadata.get("return_code"), int) and metadata.get("return_code") == 1 and not metadata.get("launch_error") and not metadata.get("interrupted"),
                "summary_present": False,
                "gnss_first_strict_progress": False,
                "gnss_first_full_finite_d_exact_handoff": False,
                "main_selected_multifrontal_qr": False,
                "main_accepted_outer_iterations": False,
                "main_strict_cost_decrease": False,
                "main_finite_expected_coverage": False,
                "no_fallback_or_solution_publication": not withheld_path.is_file(),
            },
        })
        return report
    try:
        summary = read_json(summary_path, f"Phase99 native diagnostic summary {route}")
        summary_hash = sha256_file(summary_path, f"Phase99 summary {route}")
    except (Phase99StructuralError, OSError) as exc:
        report.update({"summary_present": False, "summary_sha256": None, "summary_error": str(exc), "gnss_first_preflight": None, "main_preflight": None, "gnss_first": None, "main": None, "phase96_main": None, "phase98_solver": None, "gates": {"summary_present": False}})
        return report
    pre = summary.get("gnss_first_preflight")
    main_pre = summary.get("main_preflight")
    gnss = summary.get("gnss_first")
    main = summary.get("main")
    phase96 = summary.get("phase96_main")
    phase98 = summary.get("phase98_solver")
    finite_summary = _finite_tree(summary)
    forbidden = _forbidden_solution_keys(summary)
    gnss_guard = (
        isinstance(pre, dict)
        and pre.get("c0d_factor_enabled") is True
        and pre.get("meter_state_parity_enabled") is True
        and pre.get("raw_d_initializer_enabled") is True
        and pre.get("gnss_first_handoff_enabled") is True
        and pre.get("guard_rejected") is False
        and pre.get("guard_failed_predicates") == []
        and isinstance(pre.get("retained_epoch_count"), int)
        and pre.get("retained_epoch_count") == expected_problem_epochs
        and isinstance(pre.get("eligible_c0d_factor_count"), int)
        and pre.get("eligible_c0d_factor_count") > 0
        and pre.get("d_initializer_epoch_count") == expected_problem_epochs
        and pre.get("d_initializer_finite_count") == expected_problem_epochs
        and pre.get("d_initializer_nonfinite_count") == 0
        and pre.get("d_initializer_coverage_valid") is True
        and pre.get("d_initializer_all_finite") is True
    )
    gnss_progress = (
        isinstance(gnss, dict)
        and gnss.get("attempted") is True
        and gnss.get("result_returned") is True
        and gnss.get("converged") is True
        and gnss.get("epochs") == expected_problem_epochs
        and isinstance(gnss.get("iterations"), int)
        and gnss.get("iterations") >= 1
        and isinstance(gnss.get("c0d_factor_count"), int)
        and gnss.get("c0d_factor_count") > 0
        and gnss.get("c0d_active_solve_attempted") is True
        and isinstance(gnss.get("c0d_accepted_outer_iterations"), int)
        and gnss.get("c0d_accepted_outer_iterations") > 0
        and gnss.get("c0d_active_solve_finite_costs") is True
        and isinstance(gnss.get("initial_cost"), (int, float))
        and isinstance(gnss.get("final_cost"), (int, float))
        and math.isfinite(float(gnss["initial_cost"]))
        and math.isfinite(float(gnss["final_cost"]))
        and float(gnss["final_cost"]) < float(gnss["initial_cost"])
        and gnss.get("strict_cost_progress") is True
    )
    d_handoff = (
        isinstance(gnss, dict)
        and gnss.get("optimized_d_epoch_count") == expected_problem_epochs
        and gnss.get("optimized_d_finite_count") == expected_problem_epochs
        and gnss.get("optimized_d_nonfinite_count") == 0
        and gnss.get("optimized_d_coverage") is True
        and gnss.get("exact_retained_key_alignment") is True
        and gnss_guard
    )
    selected_qr = (
        isinstance(phase98, dict)
        and phase98.get("enabled") is True
        and phase98.get("attempted") is True
        and phase98.get("solver_type") == "MULTIFRONTAL_QR"
        and phase98.get("solver_branch") == "multifrontal"
        and phase98.get("elimination_function") == "EliminateQR"
        and phase98.get("ordering_type") == "COLAMD"
        and phase98.get("explicit_ordering_present") is False
        and phase98.get("diagonal_damping") is False
    )
    main_validation = (
        isinstance(main, dict)
        and main.get("attempted") is True
        and main.get("result_returned") is True
        and main.get("converged") is True
        and main.get("contract_passed") is True
    )
    main_coverage = (
        isinstance(main, dict)
        and main.get("problem_epoch_count") == expected_problem_epochs
        and main.get("position_solution_size") == expected_problem_epochs
        and main.get("position_size_matches_problem_epochs") is True
        and main.get("receiver_clock_solution_size") == expected_problem_epochs
        and main.get("receiver_clock_size_matches_problem_epochs") is True
        and main.get("all_positions_earth_valid") is True
        and main.get("all_receiver_clocks_finite") is True
        and main.get("optimized_d_epoch_count") == expected_problem_epochs
        and main.get("optimized_d_size_matches_problem_epochs") is True
        and main.get("optimized_d_finite_count") == expected_problem_epochs
        and main.get("optimized_d_all_finite") is True
        and main.get("velocity_epoch_count") == expected_problem_epochs
        and main.get("velocity_size_matches_problem_epochs") is True
        and main.get("all_velocities_finite") is True
        and main.get("exact_retained_key_alignment") is True
        and main.get("gnss_first_progress") is True
        and main_validation
    )
    main_progress = (
        isinstance(main, dict)
        and main.get("c0d_factor_enabled") is True
        and main.get("meter_state_parity_enabled") is True
        and isinstance(main.get("c0d_factor_count"), int)
        and main.get("c0d_factor_count") > 0
        and main.get("active_solve_attempted") is True
        and isinstance(main.get("accepted_outer_iterations"), int)
        and main.get("accepted_outer_iterations") > 0
        and main.get("active_solve_costs_finite") is True
        and main.get("final_cost_strictly_less_than_initial") is True
        and isinstance(main.get("initial_cost"), (int, float))
        and isinstance(main.get("final_cost"), (int, float))
        and math.isfinite(float(main["initial_cost"]))
        and math.isfinite(float(main["final_cost"]))
        and float(main["final_cost"]) < float(main["initial_cost"])
    )
    no_publication = (
        metadata.get("runner_read_raw_bytes") is False
        and metadata.get("runner_read_raw_hashes") is False
        and metadata.get("runner_read_truth_mat_base_coordinate_kaggle") is False
        and metadata.get("raw_content_copied_or_transformed") is False
        and withheld_path.is_file() is False
        and summary.get("solution_output_published") is False
        and summary.get("accuracy_output_published") is False
        and summary.get("truth_used") is False
        and summary.get("mat_used") is False
        and summary.get("kaggle_or_token_accessed") is False
        and summary.get("raw_truth_mat_kaggle_forbidden") is True
        and not forbidden
    )
    process_completed = metadata.get("return_code") == 1 and not metadata.get("launch_error") and not metadata.get("interrupted")
    gates = {
        "native_diagnostic_process_completed": process_completed,
        "summary_present": True,
        "finite_structural_summary": finite_summary,
        "gnss_first_strict_progress": gnss_progress,
        "gnss_first_full_finite_d_exact_handoff": d_handoff,
        "main_selected_multifrontal_qr": selected_qr,
        "main_accepted_outer_iterations": isinstance(main, dict) and isinstance(main.get("accepted_outer_iterations"), int) and main.get("accepted_outer_iterations") > 0,
        "main_strict_cost_decrease": main_progress,
        "main_finite_expected_coverage": main_coverage,
        "no_fallback_or_solution_publication": no_publication,
    }
    report.update({
        "summary_present": True,
        "summary_sha256": summary_hash,
        "summary_schema": summary.get("schema_version"),
        "summary_status": summary.get("status"),
        "failure_stage": summary.get("failure_stage"),
        "failure_reason": summary.get("failure_reason"),
        "exception_type": summary.get("exception_type"),
        "forbidden_summary_keys": forbidden,
        "gnss_first_preflight": _pick(pre, ("c0d_factor_enabled", "meter_state_parity_enabled", "raw_d_initializer_enabled", "gnss_first_handoff_enabled", "retained_epoch_count", "eligible_c0d_pair_count", "eligible_c0d_factor_count", "invalid_dt_skip_count", "gap_skip_count", "phone_exclusion_skip_count", "clock_jump_skip_count", "d_initializer_epoch_count", "d_initializer_finite_count", "d_initializer_nonfinite_count", "d_initializer_coverage_valid", "d_initializer_all_finite", "guard_rejected", "guard_predicate", "guard_failed_predicates")),
        "main_preflight": _pick(main_pre, ("c0d_factor_enabled", "meter_state_parity_enabled", "retained_epoch_count", "eligible_c0d_factor_count", "guard_rejected", "guard_failed_predicates")),
        "gnss_first": _pick(gnss, ("attempted", "result_returned", "converged", "epochs", "iterations", "initial_cost", "final_cost", "costs_finite", "c0d_factor_count", "c0d_accepted_outer_iterations", "c0d_active_solve_attempted", "c0d_active_solve_finite_costs", "strict_cost_progress", "optimized_d_epoch_count", "optimized_d_finite_count", "optimized_d_nonfinite_count", "optimized_d_coverage", "exact_retained_key_alignment", "terminal_branch")),
        "main": _pick(main, ("attempted", "result_returned", "converged", "problem_epoch_count", "position_solution_size", "position_size_matches_problem_epochs", "receiver_clock_solution_size", "receiver_clock_size_matches_problem_epochs", "all_positions_earth_valid", "all_receiver_clocks_finite", "optimized_d_epoch_count", "optimized_d_size_matches_problem_epochs", "optimized_d_finite_count", "optimized_d_nonfinite_count", "optimized_d_all_finite", "velocity_epoch_count", "velocity_size_matches_problem_epochs", "all_velocities_finite", "exact_retained_key_alignment", "gnss_first_progress", "c0d_factor_enabled", "meter_state_parity_enabled", "c0d_factor_count", "active_solve_attempted", "accepted_outer_iterations", "inner_lambda_attempts", "initial_cost", "final_cost", "active_solve_costs_finite", "final_cost_strictly_less_than_initial", "initial_lambda", "maximum_lambda", "final_lambda", "conditioning_proxy", "termination_trace_complete", "terminal_branch", "contract_passed")),
        "phase96_main": _pick(phase96, ("enabled", "attempted", "graph_observed", "initial_linearization_observed", "trial_trace_complete", "trial_limit", "graph_factor_count", "graph_value_count", "graph_initial_cost", "initial_cost", "final_cost", "accepted_outer_iterations", "terminal_branch")),
        "phase98_solver": _pick(phase98, ("enabled", "attempted", "exception_captured", "solver_type", "solver_branch", "elimination_function", "ordering_type", "explicit_ordering_present", "ordering_size", "ordering_digest", "diagonal_damping", "existing_lm_trial_limit")),
        "gates": gates,
    })
    return report


def build_result(metadata: list[dict[str, Any]], manifest: dict[str, Any], authorization: dict[str, Any], evaluator: Any) -> dict[str, Any]:
    reports = {item["dataset_id"]: _structural_route_report(item) for item in metadata}
    exact_matrix = len(metadata) == 2 and [item.get("dataset_id") for item in metadata] == list(ROUTES) and all(item.get("run_number") == 1 for item in metadata)
    route_gates = [item.get("gates", {}) for item in reports.values()]
    gates = {
        "implementation_and_binary_pins": manifest.get("implementation", {}).get("commit") == evaluator.IMPLEMENTATION_COMMIT,
        "exact_two_routes_one_run_each": exact_matrix,
        "gnss_first_strict_progress": bool(route_gates) and all(item.get("gnss_first_strict_progress") is True for item in route_gates),
        "gnss_first_full_finite_d_exact_handoff": bool(route_gates) and all(item.get("gnss_first_full_finite_d_exact_handoff") is True for item in route_gates),
        "main_selected_multifrontal_qr": bool(route_gates) and all(item.get("main_selected_multifrontal_qr") is True for item in route_gates),
        "main_accepted_outer_iterations": bool(route_gates) and all(item.get("main_accepted_outer_iterations") is True for item in route_gates),
        "main_strict_cost_decrease": bool(route_gates) and all(item.get("main_strict_cost_decrease") is True for item in route_gates),
        "main_finite_expected_coverage": bool(route_gates) and all(item.get("main_finite_expected_coverage") is True for item in route_gates),
        "no_fallback_or_solution_publication": bool(route_gates) and all(item.get("no_fallback_or_solution_publication") is True for item in route_gates),
        "truth_free": True,
        "accuracy_not_scored": True,
        "raw_only_command_policy": True,
        "cholesky_comparison_not_rerun": True,
    }
    gates["all_structural_gates_passed"] = all(gates.values())
    failed = [name for name, passed in gates.items() if passed is False]
    for route, report in reports.items():
        failed.extend(f"{route}:{name}" for name, passed in report.get("gates", {}).items() if passed is False)
    return _json_safe({
        "schema_version": RESULT_SCHEMA,
        "phase": 99,
        "execution_label": "Luna Max",
        "status": "go-phase99-main-multifrontal-qr-structural" if gates["all_structural_gates_passed"] else "no-go-phase99-main-multifrontal-qr-structural",
        "decision": "Structural gates passed; solution and accuracy lanes remain unauthorized." if gates["all_structural_gates_passed"] else "Structural gate failed closed; no solution, accuracy, fallback, or rerun lane is available.",
        "diagnostic_only": True,
        "truth_free": True,
        "accuracy_scored": False,
        "solution_output_published": False,
        "promotion_authorized": False,
        "stop_before_truth_accuracy_submission": True,
        "freeze": {"path": evaluator.relative(evaluator.FREEZE), "sha256": evaluator.FREEZE_SHA},
        "authorization": {"path": evaluator.relative(evaluator.AUTHORIZATION), "sha256": sha256_file(AUTHORIZATION, "Phase99 authorization"), "status": authorization.get("status")},
        "manifest": {"path": evaluator.relative(evaluator.MANIFEST), "sha256": sha256_file(evaluator.MANIFEST, "Phase99 manifest")},
        "evaluator": {"path": evaluator.relative(evaluator.EVALUATOR), "sha256": sha256_file(evaluator.EVALUATOR, "Phase99 evaluator")},
        "execution_wrapper": {"path": relative(Path(__file__).resolve()), "sha256": sha256_file(Path(__file__).resolve(), "Phase99 wrapper")},
        "implementation": manifest.get("implementation"),
        "matrix": {"candidate_count": 1, "routes": 2, "runs_per_route": len(metadata), "native_solver_invocations": len(metadata), "controls": 0, "reruns": 0, "fallbacks": 0, "runner_raw_byte_reads": 0, "raw_device_gnss_reads": len(metadata), "raw_device_imu_reads": len(metadata), "broadcast_navigation_reads": len(metadata), "truth_reads": 0, "mat_reads_or_generated": 0, "precomputed_coordinate_reads": 0, "base_rinex_reads": 0, "accuracy_calculations": 0, "route_score_selection": False, "solution_rows_published": False},
        "routes": reports,
        "gates": {**gates, "all_passed": gates["all_structural_gates_passed"]},
        "failed_gates": failed,
        "read_accounting": {"native_solver_invocations": len(metadata), "raw_device_gnss_reads": len(metadata), "raw_device_imu_reads": len(metadata), "broadcast_navigation_reads": len(metadata), "runner_raw_input_byte_reads": 0, "raw_input_hash_reads": 0, "truth_reads": 0, "mat_reads_or_generated": 0, "precomputed_coordinate_reads": 0, "base_rinex_reads": 0, "accuracy_calculations": 0, "accuracy_scored": False, "kaggle_or_token_access": 0, "route_score_selection": False, "reruns": 0, "fallbacks": 0, "raw_content_copied_or_transformed": False, "logs_and_partial_results_preserved": True},
        "raw_input_provenance": {"names_exact": list(RAW_NAMES), "roles": {"device_gnss.csv": "raw Android GNSS only", "device_imu.csv": "raw Android IMU only", "brdc.nav": "broadcast navigation only"}, "path_source": "Phase95 load_inherited_raw_inputs exact path metadata", "wrapper_path_resolution": "metadata-only stat then exact path substitution", "wrapper_raw_byte_reads": 0, "copy_or_transform": False},
        "forbidden_lanes": {"truth": False, "MAT": False, "base": False, "precomputed_coordinates": False, "Kaggle_or_token": False, "accuracy": False, "solution_rows": False, "Cholesky_comparison": False},
    })


def result_markdown(result: dict[str, Any]) -> str:
    lines = [
        "# Phase99 main multifrontal-QR structural result",
        "",
        f"- Status: `{result['status']}`",
        f"- Decision: {result['decision']}",
        "- Matrix: exactly MTV-A and LAX-T, one native diagnostic invocation each, sequential; no rerun/fallback",
        "- Candidate: Phase93 meter-state main graph only, `MULTIFRONTAL_QR` / `EliminateQR`; GNSS-first remains historical Cholesky",
        "- Raw inputs: Phase95 exact device_gnss.csv, device_imu.csv, brdc.nav paths only; wrapper raw-byte reads: 0",
        "- Truth/MAT/base/precomputed-coordinate/Kaggle/token/accuracy/solution-row publication: unauthorized and withheld",
        "",
        "## Route summary",
        "",
        "| Route | Native return | GNSS-first cost / accepted | Main solver | Main cost / accepted | D handoff | Coverage | Structural gates |",
        "|---|---:|---|---|---|---|---|---|",
    ]
    for route in ROUTES:
        item = result["routes"].get(route, {})
        gnss = item.get("gnss_first") or {}
        main = item.get("main") or {}
        solver = item.get("phase98_solver") or {}
        lines.append(
            f"| `{route}` | `{item.get('return_code')}` | `{gnss.get('initial_cost')} → {gnss.get('final_cost')} / {gnss.get('c0d_accepted_outer_iterations')}` | `{solver.get('solver_type')} / {solver.get('elimination_function')}` | `{main.get('initial_cost')} → {main.get('final_cost')} / {main.get('accepted_outer_iterations')}` | `{gnss.get('optimized_d_epoch_count')}/{gnss.get('optimized_d_finite_count')} exact={gnss.get('exact_retained_key_alignment')}` | `{main.get('problem_epoch_count')} epochs; earth={main.get('all_positions_earth_valid')}; finite={main.get('all_velocities_finite')}` | `{item.get('gates')}` |"
        )
    lines.extend(["", "The withheld CSV path is recorded as metadata only and is never opened, published, or committed. Structural GO does not authorize accuracy evaluation or submission.", ""])
    return "\n".join(lines)


def metadata_from_existing() -> list[dict[str, Any]]:
    return [read_json(OUTPUT_ROOT / route.replace("/", "__") / "run_metadata.json", f"Phase99 route metadata {route}") for route in ROUTES]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="execute exactly two authorized routes once")
    parser.add_argument("--validate", action="store_true", help="validate preserved Phase99 metadata without rerunning")
    parser.add_argument("--result-json", type=Path, default=RESULT_JSON)
    args = parser.parse_args(argv)
    if args.execute == args.validate:
        parser.error("choose exactly one of --execute or --validate")
    try:
        evaluator, manifest, authorization = load_pinned_contract()
        metadata = execute_matrix(manifest, evaluator) if args.execute else metadata_from_existing()
        result = build_result(metadata, manifest, authorization, evaluator)
        target = args.result_json.resolve()
        atomic_json(target, result)
        atomic_text(target.with_suffix(".md"), result_markdown(result))
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["gates"]["all_structural_gates_passed"] else 1
    except (Phase99StructuralError, OSError) as exc:
        print(f"phase99 structural runner: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
