#!/usr/bin/env python3
"""Execute and seal the authorized Phase113 remaining-route diagnostic.

This wrapper owns the post-authorization path materialization and launches
exactly one native raw+base process for MTV-H followed by MTV-U.  It records
only structural summaries and failure metadata.  The native solution path is
opaque: existence is recorded, but the file is never opened, hashed, parsed,
or published.  A missing native summary is a valid fail-closed outcome and is
sealed together with the preceding route's partial metadata.
"""

from __future__ import annotations

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
PRE_RAW = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase113_remaining_routes_diagnostic.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase113_remaining_routes_diagnostic_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase113_remaining_routes_diagnostic_authorization_v1.json"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase113_remaining_routes_diagnostic_result_v1.json"
RESULT_MD = ROOT / "docs/use_cases/records/smartphone_r5_phase113_remaining_routes_diagnostic_result_v1.md"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase113-remaining-routes-diagnostic-v1"
ROUTES = (
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")


class Phase113ExecutionError(ValueError):
    """Raised when the independent Phase113 execution contract fails closed."""


def fail(message: str) -> Phase113ExecutionError:
    return Phase113ExecutionError(message)


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
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def hash_base_once(path: Path, label: str) -> str:
    if path.name != "base.obs" or not path.is_file():
        raise fail(f"missing authorized raw base member: {label}: {path}")
    # This is the one wrapper payload read permitted by the authorization.
    return sha256_file(path, label)


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def load_contract() -> Any:
    spec = importlib.util.spec_from_file_location("phase113_remaining_routes_contract", PRE_RAW)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load Phase113 evaluator: {PRE_RAW}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def safe_member_path(path_text: Any, basename: str, route: str) -> Path:
    if not isinstance(path_text, str):
        raise fail(f"missing sealed {basename} path: {route}")
    relative_path = Path(path_text)
    if relative_path.is_absolute() or ".." in relative_path.parts or relative_path.name != basename:
        raise fail(f"unsafe sealed {basename} path: {route}: {path_text}")
    path = ROOT / relative_path
    if not path.is_file():
        raise fail(f"missing sealed {basename} member: {route}: {path}")
    return path


def materialize_inputs(contract: Any) -> dict[str, dict[str, Any]]:
    """Stat phone inputs and hash each sealed base member exactly once."""

    raw = contract.phase95_raw_metadata()
    base = contract.phase65_base_metadata()
    selected: dict[str, dict[str, Any]] = {}
    for route in ROUTES:
        selected[route] = {"raw": {}, "base": {}}
        for name in RAW_NAMES:
            pin = raw[route][name]
            path = safe_member_path(pin["path"], name, route)
            size = path.stat().st_size
            if pin.get("bytes") is not None and size != pin["bytes"]:
                raise fail(f"sealed raw byte count changed: {route}/{name}: {size} != {pin['bytes']}")
            # Phone GNSS/IMU/navigation payloads are passed through by path;
            # this wrapper never opens or hashes them.
            selected[route]["raw"][name] = {
                "path": pin["path"],
                "sha256": pin["sha256"],
                "sealed_bytes": pin.get("bytes"),
                "bytes": size,
                "exists_before_launch": True,
                "stat_read_by_wrapper": True,
                "payload_read_by_wrapper": False,
                "hash_read_by_wrapper": False,
                "content_copied_or_transformed": False,
            }
        base_pin = base[route]
        base_path = safe_member_path(base_pin["path"], "base.obs", route)
        base_size = base_path.stat().st_size
        if base_size != base_pin["bytes"]:
            raise fail(f"sealed base byte count changed: {route}: {base_size} != {base_pin['bytes']}")
        actual_hash = hash_base_once(base_path, f"base {route}")
        if actual_hash != base_pin["sha256"]:
            raise fail(f"sealed base SHA changed: {route}: {actual_hash} != {base_pin['sha256']}")
        selected[route]["base"] = {
            "path": base_pin["path"],
            "sha256": actual_hash,
            "sealed_sha256": base_pin["sha256"],
            "sealed_bytes": base_pin["bytes"],
            "bytes": base_size,
            "observed_dt_s": base_pin["observed_dt_s"],
            "moving_mean_samples": base_pin["moving_mean_samples"],
            "approx_position_xyz_m": base_pin["approx_position_xyz_m"],
            "coordinate_source": "raw RINEX header APPROX POSITION XYZ (native process)",
            "exists_before_launch": True,
            "stat_read_by_wrapper": True,
            "payload_read_by_wrapper": False,
            "hash_read_by_wrapper": True,
            "hash_verification_reads": 1,
            "native_process_reads_expected": 1,
            "content_copied_or_transformed": False,
        }
    return selected


def materialize_command(contract: Any, record: dict[str, Any], inputs: dict[str, Any]) -> list[str]:
    route = record.get("dataset_id")
    raw = inputs["raw"]
    base = inputs["base"]
    contract.validate_command(route, record.get("command"), record, raw, base)
    replacements = {
        "__PHASE113_RAW_DEVICE_GNSS__": raw["device_gnss.csv"]["path"],
        "__PHASE113_RAW_DEVICE_IMU__": raw["device_imu.csv"]["path"],
        "__PHASE113_RAW_BROADCAST_NAV__": raw["brdc.nav"]["path"],
        "__PHASE113_RAW_BASE_RINEX__": base["path"],
        "__PHASE113_RAW_BASE_SHA256__": base["sha256"],
    }
    return [replacements.get(token, token) for token in record["command"]]


def safe_environment() -> dict[str, str]:
    # Do not inherit truth/MAT/coordinate/token paths or credentials.  The
    # native binary uses the already trusted local GTSAM directory.
    return {
        "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
        "LANG": "C",
        "LC_ALL": "C",
        "TZ": "UTC",
        "LD_LIBRARY_PATH": "/home/sasaki/.local/lib",
    }


def execute_matrix(contract: Any, manifest: dict[str, Any]) -> list[dict[str, Any]]:
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite Phase113 output root: {OUTPUT_ROOT}")
    records = manifest.get("routes")
    if not isinstance(records, list) or [item.get("dataset_id") for item in records] != list(ROUTES):
        raise fail("Phase113 route order/count changed")
    inputs = materialize_inputs(contract)
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    environment = safe_environment()
    metadata: list[dict[str, Any]] = []
    for record in records:
        route = record["dataset_id"]
        route_dir = OUTPUT_ROOT / route.replace("/", "__")
        route_dir.mkdir(parents=True, exist_ok=False)
        command = materialize_command(contract, record, inputs[route])
        planned = record["planned_output"]
        summary_path = ROOT / planned["summary"]
        withheld_path = ROOT / planned["withheld_solution_output"]
        stdout_path = route_dir / "stdout.log"
        stderr_path = route_dir / "stderr.log"
        started = time.time()
        return_code: int | None = None
        interrupted = False
        timed_out = False
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
                    timeout=1800,
                )
                return_code = completed.returncode
            except subprocess.TimeoutExpired as exc:
                timed_out = True
                stderr.write(f"\nPhase113 native launch timed out: {exc}\n".encode())
            except KeyboardInterrupt:
                interrupted = True
                stderr.write(b"\nPhase113 wrapper interrupted; partial output preserved.\n")
            except OSError as exc:
                launch_error = str(exc)
                stderr.write(f"\nPhase113 native launch failed: {exc}\n".encode())
        route_metadata = {
            "schema_version": "smartphone-r5-phase113-remaining-route-execution.v1",
            "dataset_id": route,
            "run_number": 1,
            "command": command,
            "raw_inputs": inputs[route]["raw"],
            "base_input": inputs[route]["base"],
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
            "timed_out": timed_out,
            "launch_error": launch_error,
            "runner_read_raw_payloads": False,
            "runner_read_raw_hashes": False,
            "runner_read_base_payload_for_hash": True,
            "runner_read_truth_mat_phone_coordinate_precomputed_pdc_kaggle": False,
            "raw_content_copied_or_transformed": False,
            "solution_output_opened": False,
            "solution_output_published": False,
            "accuracy_scored": False,
        }
        atomic_json(route_dir / "run_metadata.json", route_metadata)
        metadata.append(route_metadata)
        # Persist after every route so an interruption cannot erase the first
        # route's record.  The result seal is written only after the matrix.
        atomic_json(OUTPUT_ROOT / "partial_execution_metadata.json", {"routes_completed": metadata})
        if interrupted or timed_out:
            break
    return metadata


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def pick(value: Any, keys: tuple[str, ...]) -> dict[str, Any]:
    if not isinstance(value, dict):
        return {}
    return {key: value[key] for key in keys if key in value}


def read_summary(path: Path, route: str) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    if not path.is_file():
        return None, {"path": relative(path), "present": False, "read_count": 0}
    try:
        payload = path.read_bytes()
        value = json.loads(payload.decode("utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        return None, {"path": relative(path), "present": True, "read_count": 1, "error": str(exc)}
    if not isinstance(value, dict):
        return None, {"path": relative(path), "present": True, "read_count": 1, "error": "summary is not an object"}
    return value, {
        "path": relative(path),
        "present": True,
        "bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
        "read_count": 1,
    }


def sealed_prior_reference(contract: Any, route: str) -> dict[str, Any]:
    # The evaluator reads only the already sealed Phase95 JSON.  This is
    # explicitly labelled historical evidence, never substituted for current
    # run telemetry.
    result = contract.read_json(contract.PHASE95_RESULT, "Phase95 result")
    record = result.get("routes", {}).get(route)
    if not isinstance(record, dict):
        return {"available": False, "reason": "sealed Phase95 route absent"}
    return {
        "available": isinstance(record.get("telemetry"), dict),
        "source": contract.relative(contract.PHASE95_RESULT),
        "source_sha256": contract.PHASE95_RESULT_SHA256,
        "historical_only": True,
        "telemetry": record.get("telemetry"),
        "return_code": record.get("return_code"),
        "failure_stage": record.get("failure_stage"),
    }


def summarize_current(summary: dict[str, Any], base_pin: dict[str, Any], contract: Any) -> dict[str, Any]:
    base = summary.get("native_base_pseudorange_compensation")
    miss = summary.get("native_base_pseudorange_source_miss_mask")
    gnss = summary.get("gnss_first")
    graph = summary.get("graph")
    clock = summary.get("native_source_clock_c0d_factor")
    epochs = summary.get("epochs")
    # The normal Phase101 recipe deliberately emits aggregate values/factors,
    # not Phase97 keyed incidence.  The latter selector is forbidden with
    # raw-base input.  Preserve that fact instead of inventing per-key counts.
    per_family = {
        "available": False,
        "reason": "native Phase101 standard summary exposes aggregate graph values/factors only; Phase97 incidence selector is outside the frozen recipe",
        "aggregate_values": graph.get("values") if isinstance(graph, dict) else None,
        "aggregate_factors": graph.get("factors") if isinstance(graph, dict) else None,
    }
    active_c0d = gnss.get("c0d_factor_count") if isinstance(gnss, dict) else None
    return {
        "summary_schema": summary.get("schema_version"),
        "summary_status": summary.get("status"),
        "stage": {
            "retained_epoch_count": gnss.get("epochs") if isinstance(gnss, dict) else None,
            "gnss_first_doppler_factor_count": summary.get("gnss_first", {}).get("undifferenced_doppler_factors") if isinstance(summary.get("gnss_first"), dict) else None,
            "source": "current native summary",
        },
        "per_family_key_incidence": per_family,
        "observability_admission": {
            "direct_quality_enabled": summary.get("native_source_direct_observable_quality_enabled"),
            "no_pdc_bridge": summary.get("native_pdc_state_bridge") is False and summary.get("native_pdc_imu_tdcp_no_bridge") is True,
            "phase101_handoff_enabled": summary.get("native_source_clock_c0d_gnss_first_meter_state_handoff_enabled"),
            "epoch_vector_parity_enabled": summary.get("native_source_clock_c0d_epoch_vector_parity_enabled"),
            "clock_dimension": clock.get("epoch_vector_dimension") if isinstance(clock, dict) else None,
            "global_isb_state_count": clock.get("global_isb_state_count") if isinstance(clock, dict) else None,
            "velocity_state_count": graph.get("velocity_states") if isinstance(graph, dict) else None,
            "position_state_count": graph.get("position_states") if isinstance(graph, dict) else None,
            "clock_state_count": graph.get("clock_states") if isinstance(graph, dict) else None,
            "d_state_count": gnss.get("optimized_d_epoch_count") if isinstance(gnss, dict) else None,
        },
        "eligible_vs_active_c0d": {
            "eligible_pair_count": None,
            "eligible_factor_count": None,
            "active_factor_count": active_c0d,
            "active_count_source": "current native GNSS-first summary",
            "eligibility_source": "not emitted by standard Phase101 raw-base summary; sealed Phase95 preflight is historical reference only",
        },
        "raw_base": {
            "compensation": pick(base, (
                "enabled", "built", "applied", "base_rinex", "base_rinex_sha256", "base_member_sha256",
                "base_rinex_bytes", "base_rinex_read_count", "base_coordinate_provenance", "base_coordinate_xyz_m",
                "observed_interval_s", "moving_mean_samples", "matching_key", "same_satellite_signal_only",
                "matched_base_rows", "finite_base_residual_rows", "interpolation_misses", "adopted_pseudorange_rows",
                "adopted_rows_corrected", "matched_factor_rows", "finite_correction_rows_among_matched",
                "spp_applied", "doppler_applied", "tdcp_applied", "no_extrapolation_or_endpoint_hold", "failure",
            )),
            "miss_mask": pick(miss, (
                "enabled", "original_adopted_pseudorange_rows", "retained_finite_pc_pseudorange_rows",
                "dropped_missing_exact_stream_rows", "dropped_out_of_domain_rows", "dropped_nonfinite_correction_rows",
                "corrected_rows", "signal_count_consistent", "retained_finite_pc_fraction", "retained_over_original_fraction",
                "pseudorange_factors_inserted", "pseudorange_factor_count_consistent", "correction_application_pass_count",
                "correction_applied_exactly_once", "duplicate_correction_rejected", "signal_taxonomy_by_signal",
                "retained_factor_epoch_indices_unchanged", "tdcp_doppler_imu_spp_unchanged", "no_extrapolation_or_endpoint_hold",
                "sign", "failure",
            )),
            "expected_base_path": base_pin.get("path"),
            "expected_base_sha256": base_pin.get("sha256"),
            "expected_base_bytes": base_pin.get("bytes"),
        },
        "gnss_first": pick(gnss, (
            "attempted", "converged", "epochs", "iterations", "initial_cost", "final_cost",
            "c0d_factor_count", "c0d_accepted_outer_iterations", "c0d_active_solve_finite_costs",
            "optimized_c_vector_parity_enabled", "optimized_c_export_valid", "optimized_c_epoch_count",
            "optimized_c_finite_component_count", "optimized_c_nonfinite_component_count", "optimized_c_dimension",
            "optimized_d_export_valid", "optimized_d_epoch_count", "optimized_d_finite_count", "optimized_d_nonfinite_count",
            "epoch_identity_alignment_valid", "handoff_mode",
        )),
        "main": {
            **pick(graph, ("factors", "values", "imu_intervals", "iterations", "converged", "initial_cost", "final_cost")),
            **pick(clock, (
                "clock_c0d_enabled", "meter_state_parity_enabled", "epoch_vector_parity_enabled", "epoch_vector_dimension",
                "epoch_vector_state_count", "epoch_vector_handoff_count", "global_isb_state_count",
                "internal_clock_state_unit", "internal_isb_state_unit", "internal_drift_state_unit",
                "clock_c0d_factor_count", "active_solve_attempted", "active_solve_initial_cost", "active_solve_final_cost",
                "accepted_outer_iterations", "total_inner_lambda_attempts", "initial_lambda", "maximum_lambda", "final_lambda",
                "active_solve_finite_costs", "termination_branch_reason", "conditioning_proxy",
            )),
            "position_solution_size": graph.get("position_solution_size") if isinstance(graph, dict) else None,
            "position_nonfinite_count": graph.get("position_nonfinite_count") if isinstance(graph, dict) else None,
            "position_out_of_earth_count": graph.get("position_out_of_earth_count") if isinstance(graph, dict) else None,
            "earth_valid_position_count": graph.get("earth_valid_position_count") if isinstance(graph, dict) else None,
            "velocity_size": graph.get("velocity_size") if isinstance(graph, dict) else None,
            "d_size": gnss.get("optimized_d_epoch_count") if isinstance(gnss, dict) else None,
            "coverage_source": "current native summary; coordinate values omitted",
        },
        "output": pick(epochs, ("problem", "output", "pseudorange_factors", "tdcp_factors_built", "double_difference_pseudorange_factors", "double_difference_carrier_factors")),
        "solver": pick(summary, (
            "native_source_clock_c0d_phase99_main_multifrontal_qr_solver_enabled",
            "native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected",
            "selected_linear_solver_type", "selected_solver_branch", "selected_elimination_function",
        )),
    }


def route_report(metadata: dict[str, Any], contract: Any, manifest: dict[str, Any]) -> dict[str, Any]:
    route = metadata["dataset_id"]
    record = next(item for item in manifest["routes"] if item["dataset_id"] == route)
    expected = int(record["expected_problem_epochs"])
    summary_path = ROOT / metadata["planned_output"]["summary"]
    withheld_path = ROOT / metadata["planned_output"]["withheld_solution_output"]
    summary, summary_meta = read_summary(summary_path, route)
    report: dict[str, Any] = {
        "dataset_id": route,
        "run_number": metadata.get("run_number"),
        "return_code": metadata.get("return_code"),
        "expected": {"domain_rows": expected - 1, "problem_epochs": expected, "output_epochs": expected},
        "summary": summary_meta,
        "withheld_solution": {
            "path": relative(withheld_path),
            "present": withheld_path.is_file(),
            "opened": False,
            "published": False,
            "content_read": False,
        },
        "raw_inputs": metadata.get("raw_inputs"),
        "base_input": metadata.get("base_input"),
        "truth_used": False,
        "mat_used": False,
        "phone_coordinates_used": False,
        "precomputed_coordinates_used": False,
        "pdc_used": False,
        "kaggle_or_token_accessed": False,
        "read_accounting": {
            "runner_raw_payload_reads": 0,
            "runner_raw_hash_reads": 0,
            "runner_base_hash_reads": 1,
            "native_raw_phone_gnss_reads": 1,
            "native_raw_phone_imu_reads": 1,
            "native_broadcast_navigation_reads": 1,
            "native_base_rinex_reads_declared": 1,
            "truth_reads": 0,
            "mat_reads_or_generated": 0,
            "phone_coordinate_reads": 0,
            "precomputed_coordinate_reads": 0,
            "pdc_reads": 0,
            "accuracy_calculations": 0,
            "solution_output_opened": 0,
            "raw_content_copied_or_transformed": False,
        },
        "logs": {"stdout": metadata.get("stdout"), "stderr": metadata.get("stderr")},
    }
    if summary is None:
        report["failure_stage"] = "native-summary-unavailable-fail-closed"
        report["failure_reason"] = summary_meta.get("error", "native summary was not produced")
        report["telemetry"] = None
        report["sealed_phase95_reference"] = sealed_prior_reference(contract, route)
        report["gates"] = {
            "native_process_completed": metadata.get("return_code") == 0 and not metadata.get("launch_error") and not metadata.get("interrupted") and not metadata.get("timed_out"),
            "summary_present": False,
            "stage_and_admission_telemetry": False,
            "eligible_vs_active_c0d_telemetry": False,
            "raw_base_corrected_miss_telemetry": False,
            "main_coverage_earth_valid_solver_progress": False,
            "finite_handoff_when_reached": False,
            "no_fallback_or_solution_publication": metadata.get("solution_output_opened") is False and metadata.get("solution_output_published") is False,
        }
        report["failure_reasons"] = [name for name, passed in report["gates"].items() if passed is not True]
        return report
    telemetry = summarize_current(summary, metadata["base_input"], contract)
    report["telemetry"] = telemetry
    report["failure_stage"] = "native-summary-observed"
    report["failure_reason"] = ""
    base = telemetry["raw_base"]["compensation"]
    miss = telemetry["raw_base"]["miss_mask"]
    gnss = telemetry["gnss_first"]
    main = telemetry["main"]
    solver = telemetry["solver"]
    gate_base = (
        base.get("enabled") is True and base.get("built") is True and base.get("applied") is True
        and base.get("base_rinex") == metadata["base_input"].get("path")
        and base.get("base_rinex_sha256") == metadata["base_input"].get("sha256")
        and base.get("base_member_sha256") == metadata["base_input"].get("sha256")
        and base.get("base_rinex_bytes") == metadata["base_input"].get("bytes")
        and base.get("base_rinex_read_count") == 1
    )
    gate_miss = (
        isinstance(miss, dict) and miss.get("enabled") is True
        and isinstance(miss.get("pseudorange_factors_inserted"), int)
        and miss.get("pseudorange_factor_count_consistent") is True
        and miss.get("correction_application_pass_count") == 1
        and miss.get("correction_applied_exactly_once") is True
        and miss.get("duplicate_correction_rejected") is False
    )
    gnss_progress = (
        gnss.get("attempted") is True and gnss.get("converged") is True
        and gnss.get("epochs") == expected and isinstance(gnss.get("iterations"), int) and gnss.get("iterations") >= 1
        and isinstance(gnss.get("c0d_factor_count"), int) and gnss.get("c0d_factor_count") > 0
        and isinstance(gnss.get("c0d_accepted_outer_iterations"), int) and gnss.get("c0d_accepted_outer_iterations") > 0
        and gnss.get("c0d_active_solve_finite_costs") is True
        and finite(gnss.get("initial_cost")) and finite(gnss.get("final_cost")) and gnss.get("final_cost") < gnss.get("initial_cost")
    )
    handoff = (
        gnss.get("optimized_c_vector_parity_enabled") is True and gnss.get("optimized_c_export_valid") is True
        and gnss.get("optimized_c_dimension") == 7 and gnss.get("optimized_c_epoch_count") == expected
        and gnss.get("optimized_c_finite_component_count") == expected * 7 and gnss.get("optimized_c_nonfinite_component_count") == 0
        and gnss.get("optimized_d_export_valid") is True and gnss.get("optimized_d_epoch_count") == expected
        and gnss.get("optimized_d_finite_count") == expected and gnss.get("optimized_d_nonfinite_count") == 0
        and gnss.get("epoch_identity_alignment_valid") is True
    )
    main_progress = (
        main.get("converged") is True and isinstance(main.get("iterations"), int) and main.get("iterations") >= 1
        and main.get("clock_c0d_enabled") is True and main.get("meter_state_parity_enabled") is True
        and main.get("active_solve_attempted") is True and isinstance(main.get("accepted_outer_iterations"), int)
        and main.get("accepted_outer_iterations") > 0 and main.get("active_solve_finite_costs") is True
        and finite(main.get("active_solve_initial_cost")) and finite(main.get("active_solve_final_cost"))
        and main.get("active_solve_final_cost") < main.get("active_solve_initial_cost")
    )
    coverage = (
        telemetry["output"].get("problem") == expected and telemetry["output"].get("output") == expected
        and main.get("position_nonfinite_count") == 0
        and main.get("earth_valid_position_count") == expected
    )
    no_fallback = summary.get("status") == "imu-combined-factor" and summary.get("native_pdc_state_bridge") is False
    report["gates"] = {
        "native_process_completed": metadata.get("return_code") == 0 and not metadata.get("launch_error") and not metadata.get("interrupted") and not metadata.get("timed_out"),
        "summary_present": True,
        "stage_and_admission_telemetry": telemetry["stage"]["retained_epoch_count"] is not None and telemetry["observability_admission"]["phase101_handoff_enabled"] is True,
        "eligible_vs_active_c0d_telemetry": telemetry["eligible_vs_active_c0d"]["active_factor_count"] is not None,
        "raw_base_corrected_miss_telemetry": gate_base and gate_miss,
        "main_coverage_earth_valid_solver_progress": main_progress and coverage and solver.get("selected_linear_solver_type") == "MULTIFRONTAL_QR",
        "finite_handoff_when_reached": handoff,
        "no_fallback_or_solution_publication": no_fallback and metadata.get("solution_output_opened") is False and metadata.get("solution_output_published") is False and summary.get("truth_used") is False,
    }
    report["failure_reasons"] = [name for name, passed in report["gates"].items() if passed is not True]
    return report


def build_result(metadata: list[dict[str, Any]], contract: Any, manifest: dict[str, Any], authorization: dict[str, Any]) -> dict[str, Any]:
    reports = [route_report(item, contract, manifest) for item in metadata]
    by_route = {item["dataset_id"]: item for item in reports}
    expected_names = (
        "native_process_completed", "summary_present", "stage_and_admission_telemetry",
        "eligible_vs_active_c0d_telemetry", "raw_base_corrected_miss_telemetry",
        "main_coverage_earth_valid_solver_progress", "finite_handoff_when_reached",
        "no_fallback_or_solution_publication",
    )
    all_passed = len(reports) == 2 and [item["dataset_id"] for item in reports] == list(ROUTES) and all(
        all(report.get("gates", {}).get(name) is True for name in expected_names) for report in reports
    )
    failed = {
        route: [name for name in expected_names if report.get("gates", {}).get(name) is not True]
        for route, report in by_route.items()
        if any(report.get("gates", {}).get(name) is not True for name in expected_names)
    }
    return {
        "schema_version": "smartphone-r5-phase113-remaining-routes-diagnostic-result.v1",
        "phase": 113,
        "execution_label": "Luna Max",
        "status": "go-phase113-remaining-routes-diagnostic" if all_passed else "no-go-phase113-remaining-routes-diagnostic",
        "decision": "Structural telemetry captured; no solution/truth/accuracy lane is authorized." if all_passed else "One or more structural telemetry gates failed closed; no retry, fallback, solution, truth, or accuracy lane is available.",
        "candidate": {"id": contract.CANDIDATE_ID, "diagnostic_only": True, "default_off": True, "algorithm_or_solver_change": False, "solution_output_published": False},
        "freeze": {"path": relative(contract.FREEZE), "sha256": contract.FREEZE_SHA256, "commit": contract.FREEZE_COMMIT},
        "manifest": {"path": relative(MANIFEST), "sha256": sha256_file(MANIFEST, "Phase113 manifest")},
        "authorization": {"path": relative(AUTHORIZATION), "sha256": sha256_file(AUTHORIZATION, "Phase113 authorization"), "status": authorization.get("status")},
        "evaluator": {"path": relative(PRE_RAW), "sha256": sha256_file(PRE_RAW, "Phase113 evaluator")},
        "execution_wrapper": {"path": relative(Path(__file__).resolve()), "sha256": sha256_file(Path(__file__).resolve(), "Phase113 wrapper")},
        "implementation": manifest.get("implementation"),
        "routes": by_route,
        "matrix": {
            "candidate_count": 1, "route_count": 2, "runs_per_route": 1,
            "native_solver_invocations": len(metadata), "controls": 0, "reruns": 0, "fallbacks": 0,
            "raw_phone_gnss_process_reads": len(metadata), "raw_phone_imu_process_reads": len(metadata),
            "broadcast_navigation_process_reads": len(metadata), "raw_base_wrapper_hash_reads": len(metadata),
            "raw_base_native_process_reads_declared": len(metadata), "truth_reads": 0, "mat_reads_or_generated": 0,
            "phone_coordinate_reads": 0, "precomputed_coordinate_reads": 0, "pdc_reads": 0,
            "accuracy_calculations": 0, "solution_rows_published": False,
        },
        "gates": {"all_structural_gates_passed": all_passed, "route_order_exact": len(reports) == 2 and [item["dataset_id"] for item in reports] == list(ROUTES), "failed_routes": failed},
        "failed_gates": failed,
        "read_accounting": {
            "native_solver_invocations": len(metadata), "raw_phone_gnss_process_reads": len(metadata),
            "raw_phone_imu_process_reads": len(metadata), "broadcast_navigation_process_reads": len(metadata),
            "raw_base_wrapper_hash_reads": len(metadata), "raw_base_native_process_reads_declared": len(metadata),
            "runner_raw_payload_reads": 0, "runner_raw_hash_reads": 0, "truth_reads": 0,
            "mat_reads_or_generated": 0, "phone_coordinate_reads": 0, "precomputed_coordinate_reads": 0,
            "pdc_reads": 0, "accuracy_calculations": 0, "kaggle_or_token_access": 0,
            "route_reruns": 0, "fallbacks": 0, "solution_output_opened": False,
            "solution_output_published": False, "raw_content_copied_or_transformed": False,
            "logs_and_partial_results_preserved": True,
        },
        "forbidden_lanes": {"truth": False, "MAT": False, "phone_coordinates": False, "precomputed_coordinates": False, "PDC": False, "Kaggle_or_token": False, "accuracy": False, "solution_rows": False},
        "truth_free": True, "accuracy_scored": False, "solution_output_published": False,
        "promotion_authorized": False, "stop_before_truth_accuracy_submission": True,
    }


def render_markdown(result: dict[str, Any]) -> str:
    lines = [
        "# Phase113 remaining-route diagnostic result", "",
        f"- status: `{result['status']}`",
        "- routes: exactly MTV-H then MTV-U, one native invocation each",
        "- raw phone/base/nav/IMU only; truth/MAT/PDC/precomputed coordinates/Kaggle/accuracy: not read",
        "- solution CSV: opaque, never opened or published",
        "", "| Route | Return | Summary | Structural gates |", "|---|---:|---|---:|",
    ]
    for route, report in result["routes"].items():
        gates = report.get("gates", {})
        passed = sum(value is True for value in gates.values())
        lines.append(f"| `{route}` | `{report.get('return_code')}` | `{report.get('summary', {}).get('present')}` | `{passed}/{len(gates)}` |")
        lines.append(f"  - failure stage: `{report.get('failure_stage')}`; failed: `{report.get('failure_reasons', [])}`")
    lines.extend(["", "Missing native summaries and pre-handoff failures remain sealed fail-closed; no rerun or fallback is available.", ""])
    return "\n".join(lines)


def main() -> int:
    contract = load_contract()
    try:
        pre_raw = contract.verify_pre_raw()
        if any(value != 0 for key, value in pre_raw.items() if key.endswith("_reads") or key in {
            "native_solver_invocations", "mat_reads_or_generated", "phone_coordinate_reads",
            "precomputed_coordinate_reads", "pdc_reads", "accuracy_calculations",
        }):
            raise fail("Phase113 pre-raw verifier reported prohibited activity")
        manifest = contract.verify_manifest()
        authorization = contract.verify_authorization(manifest)
        metadata = execute_matrix(contract, manifest)
        result = build_result(metadata, contract, manifest, authorization)
        atomic_json(RESULT_JSON, result)
        atomic_text(RESULT_MD, render_markdown(result))
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["gates"]["all_structural_gates_passed"] else 1
    except (Phase113ExecutionError, OSError) as exc:
        print(f"phase113 execution: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
