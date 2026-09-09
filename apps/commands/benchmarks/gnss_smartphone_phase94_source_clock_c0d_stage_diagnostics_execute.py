#!/usr/bin/env python3
"""Execute and seal the authorized Phase94 diagnostic-only raw matrix.

The companion pre-raw verifier owns the freeze, authorization, and exact
command contract.  This process is intentionally small and fail-closed: it
launches the pinned native command once for each route in manifest order,
keeps stdout/stderr and any native summary, and publishes only structural
telemetry.  It never opens or hashes a raw input, truth, MAT, coordinate,
base, or Kaggle artifact.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
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
    "gnss_smartphone_phase94_source_clock_c0d_stage_diagnostics.py"
)
MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase94_source_clock_c0d_stage_diagnostics_execution_manifest_v1.json"
)
AUTHORIZATION = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase94_source_clock_c0d_stage_diagnostics_raw_execution_authorization_v1.json"
)
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
OUTPUT_ROOT = ROOT / (
    "output/smartphone-r5/"
    "phase94-source-clock-c0d-stage-diagnostics-v1"
)
RESULT_JSON = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase94_source_clock_c0d_stage_diagnostics_structural_result_v1.json"
)
RESULT_MD = RESULT_JSON.with_suffix(".md")

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
DOMAIN_ROWS = {
    ROUTES[0]: 2158,
    ROUTES[1]: 3139,
    ROUTES[2]: 1465,
    ROUTES[3]: 1101,
}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
SCHEMA = "smartphone-r5-phase94-source-clock-c0d-stage-diagnostics-structural-result.v1"
TELEMETRY_SCHEMA = "smartphone-r5-phase94-source-clock-c0d-stage-diagnostics.v1"
FREEZE_SHA = "9b0b61dee17be33b595ddccbf0edddaf7844b664de3defa8e1e7139e36f0b4b9"
IMPLEMENTATION_COMMIT = "24f3329841f29d18b0b92c5476887464d8bf8be6"
MAX_SPEED_MPS = 70.0


class StructuralError(ValueError):
    """Raised when the sealed structural contract cannot be preserved."""


def fail(message: str) -> StructuralError:
    return StructuralError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def sha256_file(path: Path, label: str) -> str:
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    return hashlib.sha256(payload).hexdigest()


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


def finite_tree(value: Any, label: str) -> None:
    if value is None or isinstance(value, (bool, str)):
        return
    if isinstance(value, (int, float)):
        if not math.isfinite(float(value)):
            raise fail(f"non-finite telemetry value: {label}")
        return
    if isinstance(value, dict):
        for key, child in value.items():
            finite_tree(child, f"{label}.{key}")
        return
    if isinstance(value, list):
        for index, child in enumerate(value):
            finite_tree(child, f"{label}[{index}]")


def _number(mapping: dict[str, Any], key: str) -> float | None:
    value = mapping.get(key)
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _count(mapping: dict[str, Any], key: str) -> int | None:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return None
    return value


def _strict_cost(mapping: dict[str, Any]) -> bool:
    initial = _number(mapping, "initial_cost")
    final = _number(mapping, "final_cost")
    return initial is not None and final is not None and final < initial


def _required_keys(mapping: Any, keys: tuple[str, ...]) -> bool:
    return isinstance(mapping, dict) and all(key in mapping for key in keys)


def load_pinned_contract() -> tuple[Any, dict[str, Any], dict[str, Any], dict[str, Any]]:
    if not PRE_RAW.is_file():
        raise fail(f"missing Phase94 evaluator: {PRE_RAW}")
    sys.path.insert(0, str(PRE_RAW.parent))
    try:
        evaluator = importlib.import_module(
            "gnss_smartphone_phase94_source_clock_c0d_stage_diagnostics"
        )
    except ImportError as exc:
        raise fail(f"unable to import Phase94 evaluator: {exc}") from exc
    try:
        pre_raw = evaluator.verify_pre_raw()
        manifest = evaluator.verify_manifest()
        authorization = evaluator.verify_authorization(manifest)
    except Exception as exc:
        raise fail(f"pinned Phase94 contract failed: {exc}") from exc
    if pre_raw.get("raw_reads") != 0 or pre_raw.get("native_solver_invocations") != 0:
        raise fail("pre-raw verifier reported native/raw activity")
    if pre_raw.get("accuracy_scored") is not False:
        raise fail("pre-raw verifier reported accuracy")
    if manifest.get("status") != "sealed-before-phase94-raw-execution":
        raise fail("manifest is not sealed before raw execution")
    if authorization.get("status") != "authorized-for-exact-four-route-phase94-diagnostic-execution":
        raise fail("Phase94 raw authorization status changed")
    return evaluator, pre_raw, manifest, authorization


def _raw_input_metadata(record: dict[str, Any]) -> dict[str, dict[str, Any]]:
    raw = record.get("raw_inputs")
    if not isinstance(raw, dict) or tuple(raw) != RAW_NAMES:
        raise fail(f"raw input set changed: {record.get('dataset_id')}")
    metadata: dict[str, dict[str, Any]] = {}
    for name in RAW_NAMES:
        item = raw.get(name)
        if not isinstance(item, dict) or not isinstance(item.get("path"), str):
            raise fail(f"raw input pin malformed: {record.get('dataset_id')}/{name}")
        path_text = item["path"]
        relative_path = Path(path_text)
        path = ROOT / relative_path
        if relative_path.is_absolute() or ".." in relative_path.parts or relative_path.name != name:
            raise fail(f"unsafe raw input path: {record.get('dataset_id')}/{name}")
        # This is an existence flag only.  The runner deliberately never
        # opens, reads, or hashes the raw file; native owns raw reads.
        metadata[name] = {
            "path": path_text,
            "sha256": None,
            "sha256_available": False,
            "exists_before_launch": path.is_file(),
            "read_by_runner": False,
        }
    return metadata


def materialize_command(record: dict[str, Any], evaluator: Any) -> list[str]:
    route = record.get("dataset_id")
    if route not in ROUTES:
        raise fail(f"unknown route in manifest: {route}")
    raw = _raw_input_metadata(record)
    command = record.get("command")
    if not isinstance(command, list) or not all(isinstance(item, str) for item in command):
        raise fail(f"malformed sealed command: {route}")
    # Reuse the launch-free evaluator's exact flag/path checks.  It examines
    # only the manifest strings and does not touch raw bytes.
    evaluator._validate_command(route, command, raw)
    if command[0] != "build/apps/gnss_fgo_imu_no_base":
        raise fail(f"binary changed in command: {route}")
    return list(command)


def execute_matrix(manifest: dict[str, Any], evaluator: Any) -> list[dict[str, Any]]:
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite existing Phase94 output root: {OUTPUT_ROOT}")
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    records = manifest.get("routes")
    if not isinstance(records, list) or [item.get("dataset_id") for item in records] != list(ROUTES):
        raise fail("sealed Phase94 route order/count changed")
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
        command = materialize_command(record, evaluator)
        stdout_path = route_dir / "stdout.log"
        stderr_path = route_dir / "stderr.log"
        raw_metadata = _raw_input_metadata(record)
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
                stderr.write(b"\nPhase94 runner interrupted; partial output preserved.\n")
            except OSError as exc:
                launch_error = str(exc)
                stderr.write(f"\nPhase94 native launch failed: {exc}\n".encode())
        route_metadata = {
            "schema_version": "smartphone-r5-phase94-source-clock-c0d-stage-diagnostics-route-execution.v1",
            "dataset_id": route,
            "run_number": 1,
            "command": command,
            "raw_inputs": raw_metadata,
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
        }
        atomic_json(route_dir / "run_metadata.json", route_metadata)
        metadata.append(route_metadata)
    return metadata


def _log_pin(path_text: str, label: str) -> dict[str, Any]:
    path = ROOT / path_text
    try:
        return {"path": path_text, "bytes": path.stat().st_size, "sha256": sha256_file(path, label)}
    except OSError as exc:
        return {"path": path_text, "bytes": None, "sha256": None, "error": str(exc)}


def _forbidden_solution_keys(value: Any, path: str = "summary") -> list[str]:
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
            if key.lower() in forbidden:
                found.append(f"{path}.{key}")
            found.extend(_forbidden_solution_keys(child, f"{path}.{key}"))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            found.extend(_forbidden_solution_keys(child, f"{path}[{index}]"))
    return found


def _telemetry_report(summary: dict[str, Any], route: str, command_ok: bool) -> dict[str, Any]:
    finite_tree(summary, f"summary[{route}]")
    forbidden_keys = _forbidden_solution_keys(summary)
    if forbidden_keys:
        raise fail(f"solution values appeared in Phase94 summary: {route}/{forbidden_keys}")
    pre = summary.get("gnss_first_preflight")
    main_pre = summary.get("main_preflight")
    gnss = summary.get("gnss_first")
    main = summary.get("main")
    preflight_keys = (
        "retained_epoch_count",
        "retained_undifferenced_doppler_factor_count",
        "eligible_c0d_pair_count",
        "eligible_c0d_factor_count",
        "gap_skip_count",
        "invalid_dt_skip_count",
        "clock_jump_skip_count",
        "d_initializer_epoch_count",
        "d_initializer_finite_count",
        "d_initializer_coverage_valid",
        "d_initializer_all_finite",
        "guard_predicate",
        "guard_failed_predicates",
    )
    gnss_keys = (
        "attempted", "result_returned", "iterations", "initial_cost", "final_cost",
        "c0d_factor_count", "c0d_accepted_outer_iterations",
        "c0d_inner_lambda_attempts", "c0d_active_solve_attempted",
        "c0d_active_solve_finite_costs", "strict_cost_progress",
        "optimized_d_epoch_count", "optimized_d_finite_count",
        "optimized_d_nonfinite_count", "optimized_d_coverage",
        "exact_retained_key_alignment", "terminal_branch",
    )
    main_keys = (
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
    )
    expected_problem_epochs = DOMAIN_ROWS[route] + 1
    preflight = {
        "gnss_first": pre if isinstance(pre, dict) else None,
        "main": main_pre if isinstance(main_pre, dict) else None,
        "required_fields_present": {
            "gnss_first": _required_keys(pre, preflight_keys),
            "main": _required_keys(main_pre, preflight_keys),
        },
    }
    gnss_report = {
        "telemetry": gnss if isinstance(gnss, dict) else None,
        "required_fields_present": _required_keys(gnss, gnss_keys),
        "iterations": _count(gnss, "iterations") if isinstance(gnss, dict) else None,
        "accepted_outer_iterations": _count(gnss, "c0d_accepted_outer_iterations") if isinstance(gnss, dict) else None,
        "initial_cost": _number(gnss, "initial_cost") if isinstance(gnss, dict) else None,
        "final_cost": _number(gnss, "final_cost") if isinstance(gnss, dict) else None,
        "strict_cost_decrease": _strict_cost(gnss) if isinstance(gnss, dict) else False,
        "optimized_d_full_finite": bool(isinstance(gnss, dict) and gnss.get("optimized_d_coverage") is True and gnss.get("optimized_d_nonfinite_count") == 0),
        "exact_key_alignment": bool(isinstance(gnss, dict) and gnss.get("exact_retained_key_alignment") is True),
    }
    main_report = {
        "telemetry": main if isinstance(main, dict) else None,
        "required_fields_present": _required_keys(main, main_keys),
        "accepted_outer_iterations": _count(main, "accepted_outer_iterations") if isinstance(main, dict) else None,
        "initial_cost": _number(main, "initial_cost") if isinstance(main, dict) else None,
        "final_cost": _number(main, "final_cost") if isinstance(main, dict) else None,
        "initial_lambda": _number(main, "initial_lambda") if isinstance(main, dict) else None,
        "maximum_lambda": _number(main, "maximum_lambda") if isinstance(main, dict) else None,
        "final_lambda": _number(main, "final_lambda") if isinstance(main, dict) else None,
        "conditioning_proxy": _number(main, "conditioning_proxy") if isinstance(main, dict) else None,
        "strict_cost_decrease": _strict_cost(main) if isinstance(main, dict) else False,
    }
    c0d_unit_contract = {
        "clock_C_and_ISB": "metres",
        "drift_D": "metres_per_second",
        "dt": "seconds",
        "residual": "metres",
        "official_equation": "(C2-C1)-((D1+D2)*dt/2)",
        "official_jacobian": "[-1,+1,-dt/2,-dt/2]",
        "ordinary_sigma_m": 0.1,
        "source": "sealed Phase93/94 contract; no tuning",
    }
    guard_counts_ok = bool(
        preflight["required_fields_present"]["gnss_first"]
        and isinstance(pre, dict)
        and all(_count(pre, key) is not None for key in (
            "retained_epoch_count", "retained_undifferenced_doppler_factor_count",
            "eligible_c0d_pair_count", "eligible_c0d_factor_count", "gap_skip_count",
            "invalid_dt_skip_count", "clock_jump_skip_count", "d_initializer_epoch_count",
            "d_initializer_finite_count",
        ))
        and isinstance(pre.get("guard_predicate"), str)
        and isinstance(pre.get("guard_failed_predicates"), list)
    )
    gnss_progress = bool(
        gnss_report["required_fields_present"]
        and gnss_report["iterations"] is not None
        and gnss_report["iterations"] >= 1
        and gnss_report["accepted_outer_iterations"] is not None
        and gnss_report["accepted_outer_iterations"] >= 1
        and gnss_report["strict_cost_decrease"]
        and isinstance(gnss, dict)
        and gnss.get("costs_finite") is True
    )
    main_validation = bool(
        main_report["required_fields_present"]
        and isinstance(main, dict)
        and main.get("position_size_matches_problem_epochs") is True
        and main.get("receiver_clock_size_matches_problem_epochs") is True
        and main.get("all_positions_earth_valid") is True
        and main.get("all_receiver_clocks_finite") is True
        and main.get("optimized_d_size_matches_problem_epochs") is True
        and main.get("optimized_d_all_finite") is True
        and main.get("velocity_size_matches_problem_epochs") is True
        and main.get("all_velocities_finite") is True
        and main.get("exact_retained_key_alignment") is True
    )
    lambda_telemetry = bool(
        main_report["initial_lambda"] is not None
        and main_report["maximum_lambda"] is not None
        and main_report["final_lambda"] is not None
        and main_report["conditioning_proxy"] is not None
        and isinstance(main, dict)
        and main.get("termination_trace_complete") is True
    )
    no_publication = bool(
        summary.get("solution_output_published") is False
        and summary.get("accuracy_output_published") is False
        and summary.get("truth_used") is False
        and summary.get("mat_used") is False
        and summary.get("kaggle_or_token_accessed") is False
        and summary.get("raw_truth_mat_kaggle_forbidden") is True
    )
    return {
        "summary_schema": summary.get("schema_version"),
        "status": summary.get("status"),
        "failure_stage": summary.get("failure_stage"),
        "failure_reason": summary.get("failure_reason"),
        "exception_type": summary.get("exception_type"),
        "exception_message": summary.get("exception_message"),
        "expected_problem_epochs": expected_problem_epochs,
        "observed_problem_epochs": _count(main_pre, "retained_epoch_count") if isinstance(main_pre, dict) else None,
        "preflight": preflight,
        "gnss_first": gnss_report,
        "main": main_report,
        "c0d_unit_contract": c0d_unit_contract,
        "gates": {
            "gnss_first_guard_predicate_and_counts": guard_counts_ok,
            "gnss_first_progress_active_cost_decrease": gnss_progress,
            "main_validation_predicates": main_validation,
            "accepted_cost_lambda_terminal_telemetry": lambda_telemetry,
            "finite_earth_valid_exact_handoff": main_validation and gnss_report["optimized_d_full_finite"],
            "official_c0d_units_sigma_and_skip_telemetry": guard_counts_ok,
            "no_solution_or_accuracy_publication": no_publication,
            "command_policy_no_pdc_external_precomputed": command_ok,
        },
        "forbidden_summary_keys": forbidden_keys,
    }


def _unavailable_stage_telemetry(route: str, stderr_path: Path) -> dict[str, Any]:
    """Make every frozen stage predicate explicit when input conversion stops first."""

    try:
        observation = stderr_path.read_text(encoding="utf-8").strip()
    except (OSError, UnicodeDecodeError):
        observation = "native stderr was unavailable"
    if len(observation) > 4096:
        observation = observation[:4096]
    expected_problem_epochs = DOMAIN_ROWS[route] + 1
    return {
        "input_conversion": {
            "reached": True,
            "status": "failed-closed",
            "stderr_observation": observation,
        },
        "problem_build": {"reached": False},
        "gnss_first_preflight": {
            "reached": False,
            "retained_epoch_count": None,
            "retained_undifferenced_doppler_factor_count": None,
            "eligible_c0d_pair_count": None,
            "eligible_c0d_factor_count": None,
            "gap_skip_count": None,
            "invalid_dt_skip_count": None,
            "clock_jump_skip_count": None,
            "d_initializer_epoch_count": None,
            "d_initializer_finite_count": None,
            "d_initializer_coverage_valid": None,
            "d_initializer_all_finite": None,
            "guard_predicate": None,
            "guard_failed_predicates": None,
        },
        "gnss_first_optimize": {
            "reached": False,
            "accepted_outer_iterations": None,
            "iterations": None,
            "initial_cost": None,
            "final_cost": None,
            "inner_lambda_attempts": None,
            "terminal_branch": None,
        },
        "exact_key_handoff": {
            "reached": False,
            "exact_retained_key_alignment": None,
            "optimized_d_epoch_count": None,
            "optimized_d_finite_count": None,
            "optimized_d_coverage": None,
        },
        "imu_build": {"reached": False},
        "main_preflight": {
            "reached": False,
            "position_solution_size": None,
            "position_size_matches_problem_epochs": None,
            "receiver_clock_solution_size": None,
            "receiver_clock_size_matches_problem_epochs": None,
            "all_positions_earth_valid": None,
            "all_receiver_clocks_finite": None,
            "optimized_d_epoch_count": None,
            "optimized_d_size_matches_problem_epochs": None,
            "optimized_d_all_finite": None,
            "velocity_epoch_count": None,
            "velocity_size_matches_problem_epochs": None,
            "all_velocities_finite": None,
            "exact_retained_key_alignment": None,
        },
        "main_optimize": {
            "reached": False,
            "accepted_outer_iterations": None,
            "inner_lambda_attempts": None,
            "initial_cost": None,
            "final_cost": None,
            "initial_lambda": None,
            "maximum_lambda": None,
            "final_lambda": None,
            "conditioning_proxy": None,
            "terminal_branch": None,
        },
        "coverage_contract": {
            "reached": False,
            "contract_passed": None,
            "expected_problem_epochs": expected_problem_epochs,
            "expected_output_rows": DOMAIN_ROWS[route],
            "observed_output_rows": None,
            "output": "withheld",
        },
    }


def route_report(metadata: dict[str, Any], record: dict[str, Any]) -> dict[str, Any]:
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
        "command": metadata.get("command"),
        "raw_inputs": metadata.get("raw_inputs"),
        "expected_output": {
            "domain_rows": DOMAIN_ROWS[route],
            "problem_epochs": DOMAIN_ROWS[route] + 1,
            "path": relative(withheld_path),
            "published": False,
            "observed_rows": None,
            "coverage": "withheld-by-diagnostic-contract",
        },
        "solution_output_published": False,
        "accuracy_output_published": False,
    }
    for name, path in (("stdout", stdout_path), ("stderr", stderr_path)):
        report[name] = _log_pin(relative(path), f"{name} {route}")
    summary: dict[str, Any] | None = None
    summary_error = ""
    if summary_path.is_file():
        try:
            summary = read_json(summary_path, f"native Phase94 summary {route}")
        except StructuralError as exc:
            summary_error = str(exc)
    report["summary"] = {
        "path": relative(summary_path),
        "present": summary is not None,
        "metadata": _log_pin(relative(summary_path), f"native summary {route}") if summary is not None else None,
    }
    report["withheld_solution_output_present"] = withheld_path.is_file()
    if summary is not None:
        try:
            telemetry = _telemetry_report(summary, route, command_ok=True)
            report["telemetry"] = telemetry
            report["failure"] = summary.get("failure_reason", "")
            report["failure_stage"] = summary.get("failure_stage", "not-reached")
            report["gates"] = telemetry["gates"]
        except StructuralError as exc:
            summary_error = str(exc)
    if summary is None or summary_error:
        report["stage_telemetry"] = _unavailable_stage_telemetry(route, stderr_path)
        report.setdefault("telemetry", {
            "status": "unavailable",
            "unavailable_reason": summary_error or "native summary was not produced",
            "preflight": {"gnss_first": None, "main": None},
            "gnss_first": {"telemetry": None},
            "main": {"telemetry": None},
            "c0d_unit_contract": {
                "clock_C_and_ISB": "metres",
                "drift_D": "metres_per_second",
                "dt": "seconds",
                "ordinary_sigma_m": 0.1,
            },
            "gates": {},
        })
        report["failure"] = summary_error or (
            "native summary was not produced; raw input or an earlier native "
            "stage failed closed"
        )
        report["failure_stage"] = "native-launch-or-pre-summary"
        report["gates"] = {
            "gnss_first_guard_predicate_and_counts": False,
            "gnss_first_progress_active_cost_decrease": False,
            "main_validation_predicates": False,
            "accepted_cost_lambda_terminal_telemetry": False,
            "finite_earth_valid_exact_handoff": False,
            "official_c0d_units_sigma_and_skip_telemetry": False,
            "no_solution_or_accuracy_publication": not withheld_path.is_file(),
            "command_policy_no_pdc_external_precomputed": True,
        }
    report["native_process_completed"] = isinstance(metadata.get("return_code"), int) and not metadata.get("launch_error")
    report["diagnostic_return_code_fail_closed"] = metadata.get("return_code") == 1 and not metadata.get("interrupted") and not metadata.get("launch_error")
    report["all_structural_gates"] = all(report["gates"].values()) and report["native_process_completed"]
    return report


def build_result(
    metadata: list[dict[str, Any]],
    manifest: dict[str, Any],
    pre_raw: dict[str, Any],
    authorization: dict[str, Any],
    evaluator: Any,
) -> dict[str, Any]:
    manifest_records = manifest["routes"]
    record_by_route = {item["dataset_id"]: item for item in manifest_records}
    reports = {
        item["dataset_id"]: route_report(item, record_by_route[item["dataset_id"]])
        for item in metadata
    }
    exact_matrix = (
        len(metadata) == 4
        and [item.get("dataset_id") for item in metadata] == list(ROUTES)
        and all(item.get("run_number") == 1 for item in metadata)
    )
    route_gate_names = (
        "gnss_first_guard_predicate_and_counts",
        "gnss_first_progress_active_cost_decrease",
        "main_validation_predicates",
        "accepted_cost_lambda_terminal_telemetry",
        "finite_earth_valid_exact_handoff",
        "official_c0d_units_sigma_and_skip_telemetry",
        "no_solution_or_accuracy_publication",
        "command_policy_no_pdc_external_precomputed",
    )
    gates: dict[str, Any] = {
        "implementation_and_binary_pins": manifest.get("implementation", {}).get("commit") == IMPLEMENTATION_COMMIT,
        "exactly_four_routes_one_run_each": exact_matrix,
        "diagnostic_return_code_fail_closed": all(item.get("diagnostic_return_code_fail_closed") for item in reports.values()),
        "native_process_completed": all(item.get("native_process_completed") for item in reports.values()),
    }
    for name in route_gate_names:
        gates[name] = bool(reports) and all(
            item.get("gates", {}).get(name) is True for item in reports.values()
        )
    gates.update({
        "truth_free": True,
        "accuracy_not_scored": True,
        "solution_output_withheld": all(not item.get("solution_output_published") for item in reports.values()),
    })
    gates["all_gates_anded"] = all(gates.values())
    failed = [name for name, passed in gates.items() if passed is False]
    for route, report in reports.items():
        failed.extend(
            f"{route}:{name}"
            for name, passed in report.get("gates", {}).items()
            if passed is False
        )
    auth_sha = sha256_file(AUTHORIZATION, "Phase94 raw authorization")
    manifest_sha = sha256_file(MANIFEST, "Phase94 execution manifest")
    wrapper_sha = sha256_file(Path(__file__).resolve(), "Phase94 execution wrapper")
    result = {
        "schema_version": SCHEMA,
        "phase": 94,
        "execution_label": "Luna Max",
        "status": "go-phase94-diagnostic-structural" if gates["all_gates_anded"] else "no-go-phase94-diagnostic-structural-gates",
        "decision": (
            "GO: all authorized diagnostic gates passed; stop before truth/accuracy."
            if gates["all_gates_anded"]
            else "NO-GO: one or more diagnostic gates failed; fail closed before truth/accuracy."
        ),
        "diagnostic_only": True,
        "truth_free": True,
        "accuracy_scored": False,
        "solution_output_published": False,
        "promotion_authorized": False,
        "stop_before_truth_accuracy_submission": True,
        "authorization": {"path": relative(AUTHORIZATION), "sha256": auth_sha, "status": authorization.get("status")},
        "pre_raw_verification": pre_raw,
        "evaluator": {"path": relative(PRE_RAW), "sha256": sha256_file(PRE_RAW, "Phase94 evaluator")},
        "manifest": {"path": relative(MANIFEST), "sha256": manifest_sha},
        "implementation": {
            "commit": IMPLEMENTATION_COMMIT,
            "binary": manifest.get("implementation", {}).get("binary"),
        },
        "execution_wrapper": {"path": relative(Path(__file__).resolve()), "sha256": wrapper_sha},
        "matrix": {
            "candidate_count": 1,
            "routes": 4,
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
        "gates": {**gates, "all_passed": gates["all_gates_anded"]},
        "failed_gates": failed,
        "read_accounting": {
            "native_solver_invocations": len(metadata),
            "raw_device_gnss_route_arguments": len(metadata),
            "raw_device_imu_route_arguments": len(metadata),
            "broadcast_navigation_route_arguments": len(metadata),
            "runner_raw_input_byte_reads": 0,
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
            "runner_hashes_or_reads": False,
        },
        "forbidden_lanes": {
            "truth": False,
            "MAT": False,
            "precomputed_coordinates": False,
            "base": False,
            "Kaggle_or_token": False,
            "accuracy": False,
        },
    }
    return result


def result_markdown(result: dict[str, Any]) -> str:
    lines = [
        "# Phase94 source-clock C0/D diagnostic structural result",
        "",
        f"- Status: `{result['status']}`",
        f"- Decision: {result['decision']}",
        "- Diagnostic-only: **true**; solution and accuracy output: **withheld**",
        "- Truth/MAT/precomputed-coordinate/base/Kaggle/token reads: **0**",
        "- Matrix: exactly four routes × one native invocation, sequential, no controls/reruns/fallbacks",
        "",
        "## Route telemetry",
        "",
        "| Route | Return | Raw files present | Failure stage | Retained epochs | D init finite/full | Guard failures | GNSS accepted / cost | Main accepted / cost | Lambda / conditioning | Output |",
        "|---|---:|---|---|---:|---|---|---|---|---|---|",
    ]
    for route in ROUTES:
        item = result["routes"].get(route, {})
        raw = item.get("raw_inputs", {})
        present = ",".join(name for name in RAW_NAMES if raw.get(name, {}).get("exists_before_launch")) or "none"
        telemetry = item.get("telemetry", {})
        pre = telemetry.get("preflight", {}).get("gnss_first") if isinstance(telemetry, dict) else None
        gnss = telemetry.get("gnss_first", {}) if isinstance(telemetry, dict) else {}
        main = telemetry.get("main", {}) if isinstance(telemetry, dict) else {}
        guard = pre.get("guard_failed_predicates", []) if isinstance(pre, dict) else []
        retained = pre.get("retained_epoch_count", "n/a") if isinstance(pre, dict) else "n/a"
        dfinite = (
            f"{pre.get('d_initializer_finite_count')}/{pre.get('d_initializer_epoch_count')}"
            if isinstance(pre, dict) else "n/a"
        )
        gcost = f"{gnss.get('initial_cost', 'n/a')} → {gnss.get('final_cost', 'n/a')}"
        mcost = f"{main.get('initial_cost', 'n/a')} → {main.get('final_cost', 'n/a')}"
        lamb = f"{main.get('initial_lambda', 'n/a')} → {main.get('final_lambda', 'n/a')} / {main.get('conditioning_proxy', 'n/a')}"
        lines.append(
            f"| `{route}` | `{item.get('return_code')}` | `{present}` | `{item.get('failure_stage', 'n/a')}` | `{retained}` | `{dfinite}` | `{guard or 'none'}` | `{gnss.get('accepted_outer_iterations', 'n/a')} / {gcost}` | `{main.get('accepted_outer_iterations', 'n/a')} / {mcost}` | `{lamb}` | `withheld` |"
        )
        if item.get("failure"):
            lines.append(f"  - Failure: `{item['failure']}`")
        lines.append(f"  - Telemetry gates: `{item.get('gates', {})}`")
    lines.extend([
        "",
        "## Gate summary",
        "",
        "```json",
        json.dumps(result["gates"], indent=2, sort_keys=True),
        "```",
        "",
        "Structural diagnostics only; no truth, accuracy, or submission activity is authorized.",
        "",
    ])
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="execute exactly four route commands once")
    parser.add_argument("--validate", action="store_true", help="validate a preserved completed output tree")
    parser.add_argument("--result-json", type=Path, default=RESULT_JSON)
    args = parser.parse_args(argv)
    if args.execute == args.validate:
        parser.error("choose exactly one of --execute or --validate")
    try:
        evaluator, pre_raw, manifest, authorization = load_pinned_contract()
        if args.execute:
            metadata = execute_matrix(manifest, evaluator)
        else:
            metadata = []
            for route in ROUTES:
                path = OUTPUT_ROOT / route.replace("/", "__") / "run_metadata.json"
                metadata.append(read_json(path, f"route metadata {route}"))
        result = build_result(metadata, manifest, pre_raw, authorization, evaluator)
        target = args.result_json.resolve()
        atomic_json(target, result)
        atomic_text(target.with_suffix(".md"), result_markdown(result))
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["gates"]["all_passed"] else 1
    except (StructuralError, OSError) as exc:
        print(f"phase94 structural runner: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
