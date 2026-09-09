#!/usr/bin/env python3
"""Run and seal the authorized Phase101 raw-only structural matrix.

Exactly two native invocations are permitted: MTV-A followed by LAX-T.  The
wrapper materializes only the three raw paths from the sealed Phase95 result,
records metadata-only file stats, and never opens or hashes those raw files.
The native output CSV is placed in an isolated withheld path; this wrapper
never opens, publishes, scores, or commits it.  No truth, MAT, base,
precomputed-coordinate, Kaggle, or accuracy lane exists here.  A failed first
route does not trigger a retry, fallback, or alternate command; the second
route is still attempted once unless the wrapper itself is interrupted.
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
PRE_RAW = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase101_epoch_clock_vector.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase101_epoch_clock_vector_structural_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase101_epoch_clock_vector_raw_execution_authorization_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase101-epoch-clock-vector-structural-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase101_epoch_clock_vector_structural_result_v1.json"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
RAW_FLAGS = (
    ("--android-gnss", "device_gnss.csv"),
    ("--android-imu", "device_imu.csv"),
    ("--nav", "brdc.nav"),
)


class Phase101ExecutionError(ValueError):
    """Raised when a pinned structural execution cannot proceed safely."""


def fail(message: str) -> Phase101ExecutionError:
    return Phase101ExecutionError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


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


def sha256_file(path: Path, label: str) -> str:
    if path.name in RAW_NAMES:
        raise fail(f"raw hash requested by structural wrapper: {label}")
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


def load_evaluator() -> Any:
    spec = importlib.util.spec_from_file_location("phase101_epoch_clock_vector_contract", PRE_RAW)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load Phase101 evaluator: {PRE_RAW}")
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
        raise fail(f"Phase101 pinned contract failed: {exc}") from exc
    if pre_raw.get("raw_reads") != 0 or pre_raw.get("native_solver_invocations") != 0:
        raise fail("Phase101 pre-raw verifier reported activity")
    if pre_raw.get("accuracy_scored") is not False or pre_raw.get("solution_output_published") is not False:
        raise fail("Phase101 pre-raw verifier opened a prohibited output lane")
    expected_status = "authorized-for-exact-two-route-phase101-epoch-clock-vector-structural-execution"
    if authorization.get("status") != expected_status:
        raise fail("Phase101 authorization status is not exact")
    return evaluator, manifest, authorization


def materialize_raw_inputs(evaluator: Any) -> dict[str, dict[str, dict[str, Any]]]:
    """Resolve sealed metadata and stat selected raw files, without reading bytes."""

    sealed = evaluator.phase95_paths()
    selected: dict[str, dict[str, dict[str, Any]]] = {}
    for route in ROUTES:
        route_map = sealed.get(route)
        if not isinstance(route_map, dict) or set(route_map) != set(RAW_NAMES):
            raise fail(f"Phase95 raw roles changed: {route}")
        selected[route] = {}
        for name in RAW_NAMES:
            pin = route_map[name]
            path_text = pin.get("path") if isinstance(pin, dict) else None
            if not isinstance(path_text, str):
                raise fail(f"Phase95 raw path missing: {route}/{name}")
            relative_path = Path(path_text)
            path = ROOT / relative_path
            if relative_path.is_absolute() or ".." in relative_path.parts or relative_path.name != name:
                raise fail(f"unsafe Phase95 raw path: {route}/{name}")
            if not path.is_file():
                raise fail(f"missing Phase95 raw path: {path}")
            # stat() is provenance only.  No open(), hash, copy, or transform
            # is performed by this wrapper for a raw input.
            size = path.stat().st_size
            selected[route][name] = {
                "path": path_text,
                "sha256": pin.get("sha256"),
                "bytes": size,
                "sealed_bytes": pin.get("bytes"),
                "exists_before_launch": True,
                "read_by_runner": False,
                "hash_read_by_runner": False,
                "content_copied_or_transformed": False,
            }
    return selected


def materialize_command(record: dict[str, Any], raw: dict[str, dict[str, Any]], evaluator: Any) -> list[str]:
    route = record.get("dataset_id")
    if route not in ROUTES:
        raise fail(f"unknown Phase101 route: {route}")
    paths = {name: raw[name] for name in RAW_NAMES}
    evaluator.validate_command(route, record.get("command"), record, paths)
    command = list(record["command"])
    for flag, name in RAW_FLAGS:
        index = command.index(flag) + 1
        command[index] = raw[name]["path"]
    return command


def safe_environment() -> dict[str, str]:
    # Do not inherit arbitrary truth/MAT/token paths.  The native process gets
    # only locale/time-zone and the known local shared-library search path.
    return {
        "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
        "LANG": "C",
        "LC_ALL": "C",
        "TZ": "UTC",
        "LD_LIBRARY_PATH": "/home/sasaki/.local/lib",
    }


def execute_matrix(manifest: dict[str, Any], evaluator: Any) -> list[dict[str, Any]]:
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite Phase101 output root: {OUTPUT_ROOT}")
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    records = manifest.get("routes")
    if not isinstance(records, list) or [item.get("dataset_id") for item in records] != list(ROUTES):
        raise fail("Phase101 route order/count changed")
    raw_inputs = materialize_raw_inputs(evaluator)
    environment = safe_environment()
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
                stderr.write(b"\nPhase101 wrapper interrupted; partial output preserved.\n")
            except OSError as exc:
                launch_error = str(exc)
                stderr.write(f"\nPhase101 native launch failed: {exc}\n".encode())
        metadata_record = {
            "schema_version": "smartphone-r5-phase101-epoch-clock-vector-route-execution.v1",
            "dataset_id": route,
            "run_number": 1,
            "command": command,
            "raw_inputs": raw_inputs[route],
            "planned_output": {
                "summary": relative(summary_path),
                "withheld_solution_output": relative(withheld_path),
            },
            "summary_present_after_launch": summary_path.is_file(),
            # Presence is filesystem metadata only; content is never opened.
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
            "solution_output_published": False,
            "accuracy_scored": False,
        }
        atomic_json(route_dir / "run_metadata.json", metadata_record)
        metadata.append(metadata_record)
        atomic_json(OUTPUT_ROOT / "partial_execution_metadata.json", {"routes_completed": metadata})
        if interrupted:
            break
    return metadata


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def _pick(mapping: Any, keys: tuple[str, ...]) -> dict[str, Any]:
    if not isinstance(mapping, dict):
        return {key: None for key in keys}
    return {key: mapping.get(key) for key in keys}


def _forbidden_summary_keys(value: Any, path: str = "summary") -> list[str]:
    forbidden = {
        "solutions", "position_ecef", "latitude", "longitude",
        "epoch_clock_bias_components_m", "epoch_clock_drift_mps",
        "epoch_velocity_nav_mps", "epoch_velocities_ecef_mps",
    }
    found: list[str] = []
    if isinstance(value, dict):
        for key, child in value.items():
            if str(key).lower() in forbidden:
                found.append(f"{path}.{key}")
            found.extend(_forbidden_summary_keys(child, f"{path}.{key}"))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            found.extend(_forbidden_summary_keys(child, f"{path}[{index}]"))
    return found


def _summary_report(metadata: dict[str, Any], evaluator: Any) -> dict[str, Any]:
    route = metadata["dataset_id"]
    expected = int(next(item["expected_problem_epochs"] for item in read_json(MANIFEST, "manifest")["routes"] if item["dataset_id"] == route))
    planned = metadata["planned_output"]
    summary_path = ROOT / planned["summary"]
    withheld_path = ROOT / planned["withheld_solution_output"]
    report: dict[str, Any] = {
        "dataset_id": route,
        "run_number": metadata.get("run_number"),
        "return_code": metadata.get("return_code"),
        "expected_output": {
            "domain_rows": expected - 1,
            "problem_epochs": expected,
            "summary": relative(summary_path),
            "withheld_solution_output": relative(withheld_path),
            "solution_rows_published": False,
            "coverage_basis": "native summary epoch counts and native output validation; withheld CSV never opened",
        },
        "summary_present": summary_path.is_file(),
        "withheld_solution_output_present": withheld_path.is_file(),
        "solution_output_published": False,
        "accuracy_output_published": False,
        "truth_used": False,
        "mat_used": False,
        "base_used": False,
        "precomputed_coordinates_used": False,
        "kaggle_or_token_accessed": False,
        "runner_read_raw_bytes": metadata.get("runner_read_raw_bytes"),
        "runner_read_raw_hashes": metadata.get("runner_read_raw_hashes"),
        "runner_read_truth_mat_base_coordinate_kaggle": metadata.get("runner_read_truth_mat_base_coordinate_kaggle"),
        "raw_content_copied_or_transformed": metadata.get("raw_content_copied_or_transformed"),
        "raw_inputs": metadata.get("raw_inputs"),
        "logs": {"stdout": relative(ROOT / metadata["stdout"]), "stderr": relative(ROOT / metadata["stderr"])},
    }
    if not summary_path.is_file():
        report.update({
            "summary_sha256": None,
            "summary_error": "native standard summary was not produced; failure is sealed fail-closed",
            "telemetry": None,
            "gates": {
                "native_process_completed": metadata.get("return_code") in (0, 1) and not metadata.get("launch_error") and not metadata.get("interrupted"),
                "summary_present": False,
                "gnss_first_accepted_iterations_and_strict_cost_decrease": False,
                "gnss_first_full_finite_c7_d_exact_handoff": False,
                "no_global_isb_double_state": False,
                "main_selected_multifrontal_qr": False,
                "main_accepted_iterations_and_strict_cost_decrease": False,
                "main_finite_earth_valid_expected_output_coverage": False,
                "no_fallback_or_solution_publication": metadata.get("runner_read_raw_bytes") is False and not withheld_path.is_file(),
            },
        })
        return report
    try:
        summary = read_json(summary_path, f"Phase101 native summary {route}")
        summary_hash = sha256_file(summary_path, f"Phase101 summary {route}")
    except (Phase101ExecutionError, OSError) as exc:
        report.update({"summary_sha256": None, "summary_error": str(exc), "telemetry": None, "gates": {"summary_present": False}})
        return report
    epochs = summary.get("epochs")
    gnss = summary.get("gnss_first")
    clock = summary.get("native_source_clock_c0d_factor")
    graph = summary.get("graph")
    output_contract = summary.get("output_contract")
    forbidden = _forbidden_summary_keys(summary)

    gnss_progress = (
        isinstance(gnss, dict)
        and gnss.get("attempted") is True
        and gnss.get("converged") is True
        and gnss.get("epochs") == expected
        and isinstance(gnss.get("iterations"), int)
        and gnss.get("iterations") >= 1
        and isinstance(gnss.get("c0d_factor_count"), int)
        and gnss.get("c0d_factor_count") > 0
        and isinstance(gnss.get("c0d_accepted_outer_iterations"), int)
        and gnss.get("c0d_accepted_outer_iterations") > 0
        and gnss.get("c0d_active_solve_finite_costs") is True
        and _finite(gnss.get("initial_cost"))
        and _finite(gnss.get("final_cost"))
        and float(gnss["final_cost"]) < float(gnss["initial_cost"])
    )
    c_handoff = (
        isinstance(gnss, dict)
        and gnss.get("optimized_c_vector_parity_enabled") is True
        and gnss.get("optimized_c_export_valid") is True
        and gnss.get("optimized_c_dimension") == 7
        and gnss.get("optimized_c_epoch_count") == expected
        and gnss.get("optimized_c_finite_component_count") == expected * 7
        and gnss.get("optimized_c_nonfinite_component_count") == 0
        and gnss.get("epoch_identity_alignment_valid") is True
    )
    d_handoff = (
        isinstance(gnss, dict)
        and gnss.get("optimized_d_export_valid") is True
        and gnss.get("optimized_d_epoch_count") == expected
        and gnss.get("optimized_d_finite_count") == expected
        and gnss.get("optimized_d_nonfinite_count") == 0
        and gnss.get("epoch_identity_alignment_valid") is True
    )
    qr_selected = (
        summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_enabled") is True
        and summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected") is True
        and summary.get("selected_linear_solver_type") == "MULTIFRONTAL_QR"
        and summary.get("selected_solver_branch") == "multifrontal"
        and summary.get("selected_elimination_function") == "EliminateQR"
    )
    main_progress = (
        isinstance(graph, dict)
        and graph.get("converged") is True
        and isinstance(graph.get("iterations"), int)
        and graph.get("iterations") >= 1
        and isinstance(clock, dict)
        and clock.get("clock_c0d_enabled") is True
        and clock.get("meter_state_parity_enabled") is True
        and clock.get("active_solve_attempted") is True
        and isinstance(clock.get("accepted_outer_iterations"), int)
        and clock.get("accepted_outer_iterations") > 0
        and clock.get("active_solve_finite_costs") is True
        and _finite(clock.get("active_solve_initial_cost"))
        and _finite(clock.get("active_solve_final_cost"))
        and float(clock["active_solve_final_cost"]) < float(clock["active_solve_initial_cost"])
    )
    output_coverage = (
        isinstance(epochs, dict)
        and epochs.get("problem") == expected
        and epochs.get("output") == expected
        and isinstance(output_contract, dict)
        and output_contract.get("finite_coordinates") is True
    )
    no_global_isb = (
        isinstance(clock, dict)
        and clock.get("epoch_vector_parity_enabled") is True
        and clock.get("epoch_vector_dimension") == 7
        and clock.get("epoch_vector_state_count") == expected
        and clock.get("epoch_vector_handoff_count") == expected
        and clock.get("global_isb_state_count") == 0
        and clock.get("internal_clock_state_unit") == "metres"
        and clock.get("internal_isb_state_unit") == "metres"
        and clock.get("internal_drift_state_unit") == "metres_per_second"
    )
    no_publication = (
        metadata.get("runner_read_raw_bytes") is False
        and metadata.get("runner_read_raw_hashes") is False
        and metadata.get("runner_read_truth_mat_base_coordinate_kaggle") is False
        and metadata.get("raw_content_copied_or_transformed") is False
        and report["solution_output_published"] is False
        and report["accuracy_output_published"] is False
        and summary.get("truth_used") is False
        and summary.get("base_factors") is False
        and summary.get("no_base_contract") is True
        and summary.get("production_default_changed") is False
        and not forbidden
    )
    no_fallback = summary.get("status") == "imu-combined-factor" and summary.get("native_pdc_state_bridge") is False
    gates = {
        "native_process_completed": metadata.get("return_code") == 0 and not metadata.get("launch_error") and not metadata.get("interrupted"),
        "summary_present": True,
        "finite_structural_summary": not forbidden,
        "gnss_first_accepted_iterations_and_strict_cost_decrease": gnss_progress,
        "gnss_first_full_finite_c7_d_exact_handoff": c_handoff and d_handoff,
        "no_global_isb_double_state": no_global_isb,
        "main_selected_multifrontal_qr": qr_selected,
        "main_accepted_iterations_and_strict_cost_decrease": main_progress,
        "main_finite_earth_valid_expected_output_coverage": output_coverage,
        "no_fallback_or_solution_publication": no_fallback and no_publication,
    }
    report.update({
        "summary_sha256": summary_hash,
        "summary_schema": summary.get("schema_version"),
        "summary_status": summary.get("status"),
        "forbidden_summary_keys": forbidden,
        "telemetry": {
            "gnss_first": _pick(gnss, ("attempted", "converged", "epochs", "iterations", "initial_cost", "final_cost", "c0d_factor_count", "c0d_accepted_outer_iterations", "c0d_active_solve_finite_costs", "optimized_c_vector_parity_enabled", "optimized_c_export_valid", "optimized_c_epoch_count", "optimized_c_finite_component_count", "optimized_c_nonfinite_component_count", "optimized_c_dimension", "optimized_d_export_valid", "optimized_d_epoch_count", "optimized_d_finite_count", "optimized_d_nonfinite_count", "epoch_identity_alignment_valid", "handoff_mode")),
            "main": {
                **_pick(graph, ("factors", "values", "imu_intervals", "iterations", "converged", "initial_cost", "final_cost")),
                **_pick(clock, ("clock_c0d_enabled", "meter_state_parity_enabled", "epoch_vector_parity_enabled", "epoch_vector_dimension", "epoch_vector_state_count", "epoch_vector_handoff_count", "global_isb_state_count", "internal_clock_state_unit", "internal_isb_state_unit", "internal_drift_state_unit", "clock_c0d_factor_count", "active_solve_attempted", "active_solve_initial_cost", "active_solve_final_cost", "accepted_outer_iterations", "total_inner_lambda_attempts", "initial_lambda", "maximum_lambda", "final_lambda", "active_solve_finite_costs", "termination_branch_reason", "conditioning_proxy")),
            },
            "output": _pick(epochs, ("problem", "output", "pseudorange_factors", "tdcp_factors_built", "double_difference_pseudorange_factors", "double_difference_carrier_factors")),
            "solver": _pick(summary, ("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_enabled", "native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected", "selected_linear_solver_type", "selected_solver_branch", "selected_elimination_function")),
            "units_and_policy": {"clock": "metres", "ISB": "metres", "drift_D": "metres_per_second", "dt": "seconds", "ccdd_sigma_m": 0.1, "raw_D_source": "retained EpochSeed.receiver_clock_drift_mps", "fallback": "none", "global_ISB_state_count": clock.get("global_isb_state_count") if isinstance(clock, dict) else None},
            "gates": {"gnss_first_progress": gnss_progress, "C7_handoff": c_handoff, "D_handoff": d_handoff, "no_global_ISB": no_global_isb, "main_progress": main_progress, "output_coverage": output_coverage, "no_publication": no_publication},
        },
        "gates": gates,
    })
    return report


def build_result(metadata: list[dict[str, Any]], manifest: dict[str, Any], authorization: dict[str, Any], evaluator: Any) -> dict[str, Any]:
    route_reports = [_summary_report(item, evaluator) for item in metadata]
    by_route = {item["dataset_id"]: item for item in route_reports}
    all_expected = len(route_reports) == 2 and [item["dataset_id"] for item in route_reports] == list(ROUTES)
    gate_names = (
        "gnss_first_accepted_iterations_and_strict_cost_decrease",
        "gnss_first_full_finite_c7_d_exact_handoff",
        "no_global_isb_double_state",
        "main_selected_multifrontal_qr",
        "main_accepted_iterations_and_strict_cost_decrease",
        "main_finite_earth_valid_expected_output_coverage",
        "no_fallback_or_solution_publication",
    )
    all_passed = all_expected and all(all(item.get("gates", {}).get(name) is True for name in gate_names) for item in route_reports)
    failed = {
        route: [name for name in gate_names if report.get("gates", {}).get(name) is not True]
        for route, report in by_route.items()
        if any(report.get("gates", {}).get(name) is not True for name in gate_names)
    }
    result = {
        "schema_version": "smartphone-r5-phase101-epoch-clock-vector-structural-result.v1",
        "phase": 101,
        "execution_label": "Luna Max",
        "status": "go-phase101-epoch-clock-vector-structural" if all_passed else "no-go-phase101-epoch-clock-vector-structural",
        "decision": "Structural gates passed; solution and accuracy lanes remain unauthorized." if all_passed else "Structural gate failed closed; no solution, accuracy, fallback, or rerun lane is available.",
        "candidate": {"id": evaluator.CANDIDATE_ID, "selectors": [evaluator.PHASE93_SELECTOR, evaluator.SELECTOR, evaluator.PHASE99_SELECTOR], "default_off": True, "diagnostic_only": True},
        "freeze": {"path": evaluator.relative(evaluator.FREEZE), "sha256": evaluator.FREEZE_SHA, "commit": evaluator.FREEZE_COMMIT},
        "authorization": {"path": evaluator.relative(evaluator.AUTHORIZATION), "sha256": sha256_file(evaluator.AUTHORIZATION, "Phase101 authorization"), "status": authorization.get("status")},
        "manifest": {"path": evaluator.relative(evaluator.MANIFEST), "sha256": sha256_file(evaluator.MANIFEST, "Phase101 manifest")},
        "evaluator": {"path": evaluator.relative(evaluator.EVALUATOR), "sha256": sha256_file(evaluator.EVALUATOR, "Phase101 evaluator")},
        "execution_wrapper": {"path": relative(Path(__file__).resolve()), "sha256": sha256_file(Path(__file__).resolve(), "Phase101 wrapper")},
        "implementation": manifest.get("implementation"),
        "routes": by_route,
        "matrix": {"candidate_count": 1, "routes": 2, "runs_per_route": len(metadata), "native_solver_invocations": len(metadata), "controls": 0, "reruns": 0, "fallbacks": 0, "raw_device_gnss_reads": len(metadata), "raw_device_imu_reads": len(metadata), "broadcast_navigation_reads": len(metadata), "runner_raw_byte_reads": 0, "raw_input_hash_reads": 0, "truth_reads": 0, "mat_reads_or_generated": 0, "base_rinex_reads": 0, "precomputed_coordinate_reads": 0, "accuracy_calculations": 0, "kaggle_or_token_access": 0, "solution_rows_published": False},
        "gates": {"all_structural_gates_passed": all_passed, "route_order_exact": all_expected, "failed_routes": failed},
        "failed_gates": failed,
        "read_accounting": {"native_solver_invocations": len(metadata), "raw_device_gnss_reads": len(metadata), "raw_device_imu_reads": len(metadata), "broadcast_navigation_reads": len(metadata), "runner_raw_input_byte_reads": 0, "runner_raw_input_hash_reads": 0, "truth_reads": 0, "mat_reads_or_generated": 0, "base_rinex_reads": 0, "precomputed_coordinate_reads": 0, "accuracy_calculations": 0, "kaggle_or_token_access": 0, "route_reruns": 0, "fallbacks": 0, "accuracy_scored": False, "solution_output_published": False, "raw_content_copied_or_transformed": False, "logs_and_partial_results_preserved": True},
        "raw_input_provenance": {"names_exact": list(RAW_NAMES), "roles": {"device_gnss.csv": "raw Android GNSS only", "device_imu.csv": "raw Android IMU only", "brdc.nav": "broadcast navigation only"}, "path_source": "Phase95 sealed result raw_inputs.path", "wrapper_path_resolution": "metadata-only stat then exact path substitution", "wrapper_raw_byte_reads": 0, "wrapper_raw_hash_reads": 0, "copy_or_transform": False},
        "forbidden_lanes": {"truth": False, "MAT": False, "base": False, "precomputed_coordinates": False, "Kaggle_or_token": False, "accuracy": False, "solution_rows": False, "Cholesky_comparison": False},
        "truth_free": True,
        "accuracy_scored": False,
        "solution_output_published": False,
        "promotion_authorized": False,
        "stop_before_truth_accuracy_submission": True,
    }
    return result


def result_markdown(result: dict[str, Any]) -> str:
    lines = [
        "# Phase101 epoch-local C/ISB structural result",
        "",
        f"- Status: `{result['status']}`",
        f"- Decision: {result['decision']}",
        "- Matrix: exactly MTV-A then LAX-T, one native invocation each; no rerun, fallback, Cholesky control, truth, accuracy, or solution publication.",
        "- Candidate: Phase101 epoch-local seven-vector C/ISB plus D handoff, Phase99 multifrontal QR main selector; legacy default remains off.",
        "",
        "## Route summary",
        "",
        "| Route | Return | GNSS-first | C7/D handoff | Main QR/progress | Output | Failed gates |",
        "|---|---:|---|---|---|---|---|",
    ]
    for route in ROUTES:
        item = result["routes"].get(route, {})
        telem = item.get("telemetry") or {}
        gnss = telem.get("gnss_first") or {}
        main = telem.get("main") or {}
        solver = telem.get("solver") or {}
        gates = item.get("gates") or {}
        lines.append(
            f"| `{route}` | `{item.get('return_code')}` | `{gnss.get('initial_cost')} → {gnss.get('final_cost')} / accepted={gnss.get('c0d_accepted_outer_iterations')}` | `C {gnss.get('optimized_c_epoch_count')}/{gnss.get('optimized_c_finite_component_count')}; D {gnss.get('optimized_d_epoch_count')}/{gnss.get('optimized_d_finite_count')}; exact={gnss.get('epoch_identity_alignment_valid')}` | `{solver.get('selected_linear_solver_type')} / {main.get('accepted_outer_iterations')} / {main.get('active_solve_initial_cost')} → {main.get('active_solve_final_cost')}` | `{(telem.get('output') or {}).get('problem')}→{(telem.get('output') or {}).get('output')}` | `{[name for name, passed in gates.items() if passed is not True]}` |"
        )
    lines.extend(["", "The withheld CSV paths are metadata only; they were never opened, published, scored, or committed. Structural GO, if any, does not authorize accuracy evaluation or submission.", ""])
    return "\n".join(lines)


def metadata_from_existing() -> list[dict[str, Any]]:
    if not OUTPUT_ROOT.is_dir():
        raise fail(f"missing preserved Phase101 output root: {OUTPUT_ROOT}")
    values = []
    for route in ROUTES:
        path = OUTPUT_ROOT / route.replace("/", "__") / "run_metadata.json"
        values.append(read_json(path, f"Phase101 route metadata {route}"))
    return values


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="execute exactly two authorized routes once")
    parser.add_argument("--validate", action="store_true", help="seal preserved metadata without rerunning")
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
    except (Phase101ExecutionError, OSError) as exc:
        print(f"phase101 structural runner: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
