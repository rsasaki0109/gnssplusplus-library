#!/usr/bin/env python3
"""Execute exactly one authorized Phase114 A/LAX direct-seed matrix.

The wrapper is intentionally opaque to the native solution.  After the
independent authorization it stats raw members, passes them through without
copying or hashing, hashes each sealed base RINEX exactly once, and launches
the native child once for each route.  It reads only structural summary JSON
afterward; the solution CSV is never opened, interpreted, hashed, or
published.  Truth and accuracy are outside this authorization boundary.
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
CONTRACT_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase114_direct_seed_main_fgo.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase114_direct_seed_main_fgo_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase114_direct_seed_main_fgo_authorization_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase114-main-direct-wls-c7d-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase114_direct_seed_main_fgo_structural_result_v1.json"
RESULT_MD = ROOT / "docs/use_cases/records/smartphone_r5_phase114_direct_seed_main_fgo_structural_result_v1.md"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")


class Phase114ExecutionError(ValueError):
    """Raised when the one-shot Phase114 execution fails closed."""


def fail(message: str) -> Phase114ExecutionError:
    return Phase114ExecutionError(message)


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


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def load_contract() -> Any:
    spec = importlib.util.spec_from_file_location("phase114_direct_seed_contract", CONTRACT_PATH)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load Phase114 contract: {CONTRACT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def safe_member(path_text: Any, basename: str, route: str) -> Path:
    if not isinstance(path_text, str):
        raise fail(f"missing sealed {basename} path: {route}")
    member = Path(path_text)
    if member.is_absolute() or ".." in member.parts or member.name != basename:
        raise fail(f"unsafe sealed {basename} path: {route}: {path_text}")
    path = ROOT / member
    if not path.is_file():
        raise fail(f"missing sealed {basename} member: {route}: {path}")
    return path


def sha256_base_once(path: Path, label: str) -> str:
    if path.name != "base.obs" or not path.is_file():
        raise fail(f"missing authorized base RINEX: {label}: {path}")
    digest = hashlib.sha256()
    # This is the one and only wrapper payload read of this base member.
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def materialize_inputs(contract: Any) -> dict[str, dict[str, Any]]:
    raw_pins, base_pins = contract.sealed_input_metadata()
    selected: dict[str, dict[str, Any]] = {}
    for route in ROUTES:
        selected[route] = {"raw": {}, "base": {}}
        for name in RAW_NAMES:
            pin = raw_pins[route][name]
            path = safe_member(pin["path"], name, route)
            size = path.stat().st_size
            if pin.get("bytes") is not None and size != pin["bytes"]:
                raise fail(f"sealed raw byte count changed: {route}/{name}: {size} != {pin['bytes']}")
            # Raw phone/navigation payloads are passed through.  The wrapper
            # records metadata only; it never opens or hashes these members.
            selected[route]["raw"][name] = {
                "path": pin["path"], "sha256": pin["sha256"],
                "sealed_bytes": pin.get("bytes"), "bytes": size,
                "exists_before_launch": True, "stat_read_by_wrapper": True,
                "payload_read_by_wrapper": False, "hash_read_by_wrapper": False,
                "content_copied_or_transformed": False,
            }
        pin = base_pins[route]
        path = safe_member(pin["path"], "base.obs", route)
        size = path.stat().st_size
        if size != pin["bytes"]:
            raise fail(f"sealed base byte count changed: {route}: {size} != {pin['bytes']}")
        actual = sha256_base_once(path, f"base {route}")
        if actual != pin["sha256"]:
            raise fail(f"sealed base SHA changed: {route}")
        selected[route]["base"] = {
            "path": pin["path"], "sha256": actual, "sealed_sha256": pin["sha256"],
            "sealed_bytes": pin["bytes"], "bytes": size,
            "observed_dt_s": pin["observed_dt_s"],
            "moving_mean_samples": pin["moving_mean_samples"],
            "approx_position_xyz_m": pin["approx_position_xyz_m"],
            "coordinate_source": "raw RINEX header APPROX POSITION XYZ (native process)",
            "exists_before_launch": True, "stat_read_by_wrapper": True,
            "payload_read_by_wrapper": False, "hash_read_by_wrapper": True,
            "hash_verification_reads": 1, "native_process_reads_expected": 1,
            "content_copied_or_transformed": False,
        }
    return selected


def materialize_command(contract: Any, record: dict[str, Any], inputs: dict[str, Any]) -> list[str]:
    raw = {name: inputs["raw"][name] for name in RAW_NAMES}
    contract.validate_command(record["dataset_id"], record.get("command"), record, raw, inputs["base"])
    replacements = {
        "__PHASE114_RAW_DEVICE_GNSS__": inputs["raw"]["device_gnss.csv"]["path"],
        "__PHASE114_RAW_DEVICE_IMU__": inputs["raw"]["device_imu.csv"]["path"],
        "__PHASE114_RAW_BROADCAST_NAV__": inputs["raw"]["brdc.nav"]["path"],
        "__PHASE114_RAW_BASE_RINEX__": inputs["base"]["path"],
        "__PHASE114_RAW_BASE_SHA256__": inputs["base"]["sha256"],
    }
    command = [replacements.get(token, token) for token in record["command"]]
    for token in command:
        if token in contract.FORBIDDEN_FLAGS:
            raise fail(f"forbidden command flag after materialization: {record['dataset_id']}/{token}")
    return command


def safe_environment() -> dict[str, str]:
    # Do not inherit truth/MAT/coordinate/token variables or loader paths.
    return {
        "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
        "LANG": "C", "LC_ALL": "C", "TZ": "UTC",
        "LD_LIBRARY_PATH": "/home/sasaki/.local/lib",
    }


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def all_finite(values: Any) -> bool:
    if isinstance(values, (int, float)) and not isinstance(values, bool):
        return math.isfinite(float(values))
    if isinstance(values, list):
        return all(all_finite(item) for item in values)
    if isinstance(values, dict):
        return all(all_finite(item) for item in values.values())
    return True


def close(actual: Any, expected: Any, tolerance: float = 1e-6) -> bool:
    return finite(actual) and finite(expected) and abs(float(actual) - float(expected)) <= tolerance


def read_summary(path: Path) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    if not path.is_file():
        return None, {"path": relative(path), "present": False, "read_count": 0}
    try:
        payload = path.read_bytes()
        summary = json.loads(payload.decode("utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        return None, {"path": relative(path), "present": True, "read_count": 1, "error": str(exc)}
    if not isinstance(summary, dict):
        return None, {"path": relative(path), "present": True, "read_count": 1, "error": "summary is not an object"}
    return summary, {
        "path": relative(path), "present": True, "bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(), "read_count": 1,
    }


def solution_stat(path: Path) -> dict[str, Any]:
    # Metadata-only: is_file/stat do not open the withheld solution.
    if not path.is_file():
        return {"path": relative(path), "present": False, "opened": False, "published": False, "read_count": 0}
    return {
        "path": relative(path), "present": True, "opened": False,
        "published": False, "bytes": path.stat().st_size, "read_count": 0,
        "coordinates_interpreted": False, "rows_read": False,
    }


def selected(item: Any, keys: tuple[str, ...]) -> dict[str, Any]:
    if not isinstance(item, dict):
        return {}
    return {key: item.get(key) for key in keys if key in item}


def taxonomy_valid(miss: Any) -> bool:
    taxonomy = miss.get("signal_taxonomy_by_signal") if isinstance(miss, dict) else None
    if not isinstance(taxonomy, dict) or not taxonomy:
        return False
    integer_keys = (
        "original_adopted_rows", "retained_finite_pc_rows", "corrected_rows",
        "matched_exact_stream_rows", "finite_correction_rows_among_matched",
        "dropped_missing_exact_stream_rows", "dropped_out_of_domain_rows",
        "dropped_nonfinite_correction_rows",
    )
    for item in taxonomy.values():
        if not isinstance(item, dict) or not isinstance(item.get("frequency_band"), str):
            return False
        if any(not isinstance(item.get(key), int) or isinstance(item.get(key), bool) or item[key] < 0 for key in integer_keys):
            return False
        if item["original_adopted_rows"] != item["retained_finite_pc_rows"] + item["dropped_missing_exact_stream_rows"] + item["dropped_out_of_domain_rows"] + item["dropped_nonfinite_correction_rows"]:
            return False
        if item["corrected_rows"] != item["retained_finite_pc_rows"]:
            return False
        if item.get("factor_count_consistent") is not True:
            return False
    return True


def validate_route(metadata: dict[str, Any], contract: Any, manifest: dict[str, Any]) -> dict[str, Any]:
    route = metadata["dataset_id"]
    record = next(item for item in manifest["routes"] if item["dataset_id"] == route)
    expected = int(record["expected_problem_epochs"])
    domain = int(record["domain_rows"])
    summary_path = ROOT / metadata["planned_output"]["summary"]
    solution_path = ROOT / metadata["planned_output"]["withheld_solution_output"]
    summary, summary_meta = read_summary(summary_path)
    solution_meta = solution_stat(solution_path)
    gate_names = (
        "native_process_completed", "summary_present",
        "direct_seed_full_finite_exact_key_handoff",
        "main_qr_progress_strict_cost_decrease",
        "main_meter_c0d_units_sigma_factor_gate",
        "base_factors_active_exactly_once",
        "final_pixel5_offset_exactly_once",
        "finite_earth_valid_expected_output_coverage",
        "no_gnss_first_stage_or_solver_fallback",
    )
    report: dict[str, Any] = {
        "dataset_id": route, "return_code": metadata.get("return_code"),
        "expected": {"domain_rows": domain, "problem_epochs": expected, "output_epochs": expected},
        "summary": summary_meta, "solution_output": solution_meta,
        "raw_inputs": metadata.get("raw_inputs"), "base_input": metadata.get("base_input"),
        "truth_used": False, "mat_used": False, "phone_coordinates_used": False,
        "precomputed_coordinates_used": False, "pdc_used": False,
        "kaggle_or_token_accessed": False, "solution_output_published": False,
        "read_accounting": {
            "runner_raw_payload_reads": 0, "runner_raw_hash_reads": 0,
            "runner_base_hash_reads": 1, "native_raw_phone_gnss_reads": 1,
            "native_raw_phone_imu_reads": 1, "native_broadcast_navigation_reads": 1,
            "native_base_rinex_reads": 1, "truth_reads": 0, "mat_reads_or_generated": 0,
            "phone_coordinate_reads": 0, "precomputed_coordinate_reads": 0,
            "pdc_reads": 0, "accuracy_calculations": 0,
            "solution_output_reads": 0, "raw_content_copied_or_transformed": False,
        },
    }
    if summary is None:
        report["telemetry"] = None
        report["gates"] = {name: False for name in gate_names}
        report["failure_reasons"] = list(gate_names)
        if metadata.get("return_code") != 0:
            report["failure_reasons"].insert(0, "native_process_completed")
        return report

    gnss = summary.get("gnss_first")
    clock = summary.get("native_source_clock_c0d_factor")
    graph = summary.get("graph")
    epochs = summary.get("epochs")
    output_contract = summary.get("output_contract")
    raw_utc = summary.get("raw_utc_key_contract")
    base = summary.get("native_base_pseudorange_compensation")
    miss = summary.get("native_base_pseudorange_source_miss_mask")
    offset = summary.get("upstream_position_offset")

    direct_gate = (
        summary.get("native_direct_wls_ephemeral_c7d_main_seed_enabled") is True
        and isinstance(gnss, dict) and gnss.get("attempted") is False
        and gnss.get("gnss_first_stage_run") is False
        and gnss.get("handoff_mode") == "main-direct-wls-ephemeral-c7d-seed"
        and gnss.get("direct_seed_valid") is True
        and gnss.get("raw_epoch_count") == expected
        and gnss.get("retained_epoch_count") == expected
        and gnss.get("exact_key_count") == expected
        and gnss.get("finite_position_count") == expected
        and gnss.get("finite_clock_count") == expected
        and gnss.get("finite_drift_count") == expected
        and gnss.get("finite_velocity_count") == expected
        and gnss.get("raw_key_mismatch_count") == 0
        and gnss.get("raw_key_order_mismatch_count") == 0
        and gnss.get("raw_drift_mismatch_count") == 0
        and gnss.get("full_raw_coverage") is True
        and gnss.get("positions_clocks_copied") == 0
        and gnss.get("clock_drift_handoff_source") == "same-run-retained-raw-epoch-seed"
        and gnss.get("clock_drift_unit") == "metres_per_second"
        and gnss.get("c_vector_unit") == "metres"
        and gnss.get("c_vector_dimension") == 7
        and gnss.get("c_vector_component_order") == [
            "base_gps_l1", "glo_l1", "gal_l1", "bds_l1", "gps_l5", "gal_l5", "bds_l5"
        ]
        and gnss.get("failure") == ""
    )
    qr_selected = (
        summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_enabled") is True
        and summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected") is True
        and summary.get("selected_linear_solver_type") == "MULTIFRONTAL_QR"
        and summary.get("selected_solver_branch") == "multifrontal"
        and summary.get("selected_elimination_function") == "EliminateQR"
    )
    main_progress = (
        isinstance(graph, dict) and graph.get("converged") is True
        and isinstance(graph.get("iterations"), int) and graph.get("iterations") >= 1
        and finite(graph.get("initial_cost")) and finite(graph.get("final_cost"))
        and graph.get("final_cost") < graph.get("initial_cost")
        and isinstance(clock, dict) and clock.get("clock_c0d_enabled") is True
        and clock.get("meter_state_parity_enabled") is True
        and clock.get("epoch_vector_parity_enabled") is True
        and clock.get("epoch_vector_dimension") == 7
        and clock.get("epoch_vector_state_count") == expected
        and clock.get("epoch_vector_handoff_count") == expected
        and clock.get("global_isb_state_count") == 0
        and clock.get("active_solve_attempted") is True
        and isinstance(clock.get("accepted_outer_iterations"), int)
        and clock.get("accepted_outer_iterations") > 0
        and clock.get("active_solve_finite_costs") is True
        and finite(clock.get("active_solve_initial_cost"))
        and finite(clock.get("active_solve_final_cost"))
        and clock.get("active_solve_final_cost") < clock.get("active_solve_initial_cost")
    )
    units_factor = (
        isinstance(clock, dict)
        and clock.get("internal_clock_state_unit") == "metres"
        and clock.get("internal_isb_state_unit") == "metres"
        and clock.get("internal_drift_state_unit") == "metres_per_second"
        and clock.get("clock_c0d_sigma_m") == 0.1
        and isinstance(clock.get("clock_c0d_factor_count"), int)
        and clock.get("clock_c0d_factor_count") > 0
        and clock.get("clock_c0d_equation") == "(C2-C1)-((D1+D2)*dt/2)"
    )
    base_path = (
        isinstance(base, dict) and base.get("enabled") is True and base.get("built") is True
        and base.get("applied") is True and base.get("preserve_additional_frequency_bands") is True
        and base.get("base_rinex") == metadata["base_input"].get("path")
        and base.get("base_rinex_sha256") == metadata["base_input"].get("sha256")
        and base.get("base_member_sha256") == metadata["base_input"].get("sha256")
        and base.get("base_rinex_bytes") == metadata["base_input"].get("bytes")
        and base.get("base_rinex_read_count") == 1
        and base.get("base_coordinate_provenance") == "RINEX header APPROX POSITION XYZ"
        and isinstance(base.get("base_coordinate_xyz_m"), list)
        and len(base["base_coordinate_xyz_m"]) == 3
        and all(close(a, b) for a, b in zip(base["base_coordinate_xyz_m"], metadata["base_input"]["approx_position_xyz_m"]))
        and close(base.get("observed_interval_s"), metadata["base_input"].get("observed_dt_s"))
        and base.get("moving_mean_samples") == metadata["base_input"].get("moving_mean_samples")
        and base.get("same_satellite_signal_only") is True
    )
    base_exactly_once = (
        base_path and isinstance(base, dict) and base.get("source_model_build_count") == 1
        and base.get("correction_application_pass_count") == 1
        and base.get("correction_applied_exactly_once") is True
        and base.get("duplicate_correction_rejected") is False
        and isinstance(miss, dict) and miss.get("enabled") is True
        and miss.get("correction_application_pass_count") == 1
        and miss.get("correction_applied_exactly_once") is True
        and miss.get("duplicate_correction_rejected") is False
        and taxonomy_valid(miss)
        and isinstance(epochs, dict)
        and epochs.get("double_difference_pseudorange_factors") == 0
        and epochs.get("double_difference_carrier_factors") == 0
        and miss.get("pseudorange_factors_inserted") == epochs.get("pseudorange_factors")
        and isinstance(miss.get("pseudorange_factors_inserted"), int)
        and miss.get("pseudorange_factors_inserted") > 0
    )
    offset_gate = (
        isinstance(offset, dict) and offset.get("enabled") is True
        and offset.get("applied") is True and offset.get("phone") == "pixel5"
        and offset.get("corrected_epochs") == expected
        and close(offset.get("offset_rl_m"), -0.1)
        and close(offset.get("offset_ud_m"), -0.3)
        and close(offset.get("max_offset_enu_m"), 0.31622776601683794, 1e-9)
        and offset.get("rotation_contract") == "Rx*Ry*Rz(rpy-[0,0,pi])"
        and offset.get("failure") == ""
        and solution_meta.get("present") is True
        and solution_meta.get("opened") is False
    )
    output_gate = (
        isinstance(epochs, dict) and epochs.get("problem") == expected
        and epochs.get("output") == expected
        and isinstance(epochs.get("pseudorange_factors"), int)
        and epochs.get("pseudorange_factors") > 0
        and isinstance(epochs.get("tdcp_factors_built"), int)
        and epochs.get("tdcp_factors_built") > 0
        and isinstance(raw_utc, dict)
        and raw_utc.get("raw_epoch_keys") == expected
        and raw_utc.get("target_epochs") == expected - 1
        and raw_utc.get("exact_solution_epochs") == expected - 1
        and raw_utc.get("unresolved_epochs") == 0
        and isinstance(output_contract, dict)
        and output_contract.get("finite_coordinates") is True
        and output_contract.get("atomic_publish") is True
        and all_finite(output_contract)
    )
    no_fallback = (
        metadata.get("return_code") == 0
        and summary.get("status") == "imu-combined-factor"
        and summary.get("native_pdc_state_bridge") is False
        and summary.get("native_pdc_imu_tdcp_no_bridge") is True
        and summary.get("native_source_direct_observable_quality_enabled") is True
        and summary.get("native_source_clock_c0d_gnss_first_meter_state_handoff_enabled") is False
        and summary.get("native_direct_wls_ephemeral_c7d_main_seed_enabled") is True
        and summary.get("production_default_changed") is False
        and summary.get("truth_used") is False
        and metadata.get("runner_read_raw_payloads") is False
        and metadata.get("runner_read_raw_hashes") is False
        and metadata.get("runner_read_truth_mat_phone_coordinate_precomputed_pdc_kaggle") is False
        and metadata.get("raw_content_copied_or_transformed") is False
        and metadata.get("solution_output_published") is False
    )
    process_ok = metadata.get("return_code") == 0 and not metadata.get("launch_error") and not metadata.get("interrupted") and not metadata.get("timed_out")
    report["telemetry"] = {
        "direct_seed": selected(gnss, (
            "attempted", "converged", "handoff_mode", "velocity_handoff_source",
            "position_clock_handoff_source", "clock_drift_handoff_source",
            "direct_seed_valid", "raw_epoch_count", "retained_epoch_count",
            "exact_key_count", "finite_position_count", "finite_clock_count",
            "finite_drift_count", "finite_velocity_count", "raw_key_mismatch_count",
            "raw_key_order_mismatch_count", "raw_drift_mismatch_count",
            "full_raw_coverage", "positions_clocks_copied", "gnss_first_stage_run",
            "clock_drift_unit", "c_vector_unit", "c_vector_dimension", "failure",
        )),
        "main": {"graph": selected(graph, ("converged", "iterations", "initial_cost", "final_cost", "factors", "values", "imu_intervals")),
                 "clock": selected(clock, ("clock_c0d_enabled", "meter_state_parity_enabled", "epoch_vector_parity_enabled", "epoch_vector_dimension", "epoch_vector_state_count", "epoch_vector_handoff_count", "global_isb_state_count", "clock_c0d_factor_count", "clock_c0d_sigma_m", "active_solve_attempted", "active_solve_initial_cost", "active_solve_final_cost", "accepted_outer_iterations", "active_solve_finite_costs", "internal_clock_state_unit", "internal_isb_state_unit", "internal_drift_state_unit"))},
        "base": selected(base, ("enabled", "built", "applied", "preserve_additional_frequency_bands", "base_rinex", "base_rinex_sha256", "base_member_sha256", "base_rinex_bytes", "base_rinex_read_count", "source_model_build_count", "correction_application_pass_count", "correction_applied_exactly_once", "duplicate_correction_rejected", "adopted_pseudorange_rows", "adopted_rows_corrected", "in_domain_rows", "interpolation_misses", "failure")),
        "source_miss_mask": selected(miss, ("enabled", "original_adopted_pseudorange_rows", "retained_finite_pc_pseudorange_rows", "corrected_rows", "dropped_missing_exact_stream_rows", "dropped_out_of_domain_rows", "dropped_nonfinite_correction_rows", "pseudorange_factors_inserted", "pseudorange_factor_count_consistent", "signal_count_consistent", "correction_application_pass_count", "correction_applied_exactly_once", "duplicate_correction_rejected", "failure", "signal_taxonomy_by_signal")),
        "offset": selected(offset, ("enabled", "applied", "phone", "corrected_epochs", "offset_rl_m", "offset_ud_m", "max_offset_enu_m", "rotation_contract", "failure")),
        "output": {"epochs": selected(epochs, ("problem", "output", "pseudorange_factors", "tdcp_factors_built", "double_difference_pseudorange_factors", "double_difference_carrier_factors")), "contract": selected(output_contract, ("finite_coordinates", "atomic_publish", "header")), "raw_utc": selected(raw_utc, ("raw_epoch_keys", "target_epochs", "exact_solution_epochs", "unresolved_epochs"))},
        "solver": {"enabled": summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_enabled"), "selected": summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected"), "linear_solver_type": summary.get("selected_linear_solver_type"), "branch": summary.get("selected_solver_branch"), "elimination": summary.get("selected_elimination_function")},
    }
    report["gates"] = {
        "native_process_completed": process_ok,
        "summary_present": summary_meta.get("present") is True,
        "direct_seed_full_finite_exact_key_handoff": direct_gate,
        "main_qr_progress_strict_cost_decrease": qr_selected and main_progress,
        "main_meter_c0d_units_sigma_factor_gate": units_factor,
        "base_factors_active_exactly_once": base_exactly_once,
        "final_pixel5_offset_exactly_once": offset_gate,
        "finite_earth_valid_expected_output_coverage": output_gate,
        "no_gnss_first_stage_or_solver_fallback": no_fallback,
    }
    report["failure_reasons"] = [name for name, passed in report["gates"].items() if passed is not True]
    return report


def execute_matrix(contract: Any, manifest: dict[str, Any]) -> list[dict[str, Any]]:
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite Phase114 output root: {OUTPUT_ROOT}")
    records = manifest.get("routes")
    if not isinstance(records, list) or [item.get("dataset_id") for item in records] != list(ROUTES):
        raise fail("Phase114 route order/count changed")
    inputs = materialize_inputs(contract)
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    metadata: list[dict[str, Any]] = []
    for record in records:
        route = record["dataset_id"]
        route_dir = OUTPUT_ROOT / route.replace("/", "__")
        route_dir.mkdir(parents=True, exist_ok=False)
        command = materialize_command(contract, record, inputs[route])
        summary_path = ROOT / record["command"][record["command"].index("--summary-json") + 1]
        solution_path = ROOT / record["command"][record["command"].index("--out") + 1]
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
                    command, cwd=ROOT, env=safe_environment(), stdout=stdout, stderr=stderr,
                    check=False, timeout=1800,
                )
                return_code = completed.returncode
            except subprocess.TimeoutExpired as exc:
                timed_out = True
                stderr.write(f"\nPhase114 native launch timed out: {exc}\n".encode())
            except KeyboardInterrupt:
                interrupted = True
                stderr.write(b"\nPhase114 wrapper interrupted; partial output preserved.\n")
            except OSError as exc:
                launch_error = str(exc)
                stderr.write(f"\nPhase114 native launch failed: {exc}\n".encode())
        item = {
            "schema_version": "smartphone-r5-phase114-direct-seed-main-fgo-route-execution.v1",
            "dataset_id": route, "run_number": 1, "command": command,
            "raw_inputs": inputs[route]["raw"], "base_input": inputs[route]["base"],
            "planned_output": {"summary": relative(summary_path), "withheld_solution_output": relative(solution_path)},
            "summary_present_after_launch": summary_path.is_file(),
            "withheld_solution_output_present_after_launch": solution_path.is_file(),
            "stdout": relative(stdout_path), "stderr": relative(stderr_path),
            "started_unix_s": started, "ended_unix_s": time.time(),
            "return_code": return_code, "interrupted": interrupted, "timed_out": timed_out,
            "launch_error": launch_error,
            "runner_read_raw_payloads": False, "runner_read_raw_hashes": False,
            "runner_read_truth_mat_phone_coordinate_precomputed_pdc_kaggle": False,
            "runner_read_base_payload_for_hash": True,
            "raw_content_copied_or_transformed": False, "solution_output_opened": False,
            "solution_output_published": False, "accuracy_scored": False,
        }
        atomic_json(route_dir / "run_metadata.json", item)
        metadata.append(item)
        atomic_json(OUTPUT_ROOT / "partial_execution_metadata.json", {"routes_completed": metadata})
    return metadata


def build_result(metadata: list[dict[str, Any]], contract: Any, manifest: dict[str, Any], authorization: dict[str, Any]) -> dict[str, Any]:
    reports = [validate_route(item, contract, manifest) for item in metadata]
    by_route = {item["dataset_id"]: item for item in reports}
    gate_names = (
        "native_process_completed", "summary_present", "direct_seed_full_finite_exact_key_handoff",
        "main_qr_progress_strict_cost_decrease", "main_meter_c0d_units_sigma_factor_gate",
        "base_factors_active_exactly_once", "final_pixel5_offset_exactly_once",
        "finite_earth_valid_expected_output_coverage", "no_gnss_first_stage_or_solver_fallback",
    )
    ordered = len(reports) == 2 and [item["dataset_id"] for item in reports] == list(ROUTES)
    all_passed = ordered and all(all(report.get("gates", {}).get(name) is True for name in gate_names) for report in reports)
    failed = {
        route: [name for name in gate_names if report.get("gates", {}).get(name) is not True]
        for route, report in by_route.items()
        if any(report.get("gates", {}).get(name) is not True for name in gate_names)
    }
    return {
        "schema_version": "smartphone-r5-phase114-direct-seed-main-fgo-structural-result.v1",
        "phase": 114, "execution_label": "Luna Max",
        "status": "go-phase114-direct-seed-main-structural" if all_passed else "no-go-phase114-direct-seed-main-structural",
        "decision": "Structural gates passed; truth/accuracy and solution release remain separately unauthorized." if all_passed else "Structural gate failed closed; preserve outputs and do not retry, fallback, truth-score, or publish.",
        "candidate": {
            "id": contract.CANDIDATE_ID, "selector": contract.DIRECT_SELECTOR,
            "selectors": [contract.DIRECT_SELECTOR, contract.VECTOR_SELECTOR, contract.QR_SELECTOR, contract.OFFSET_SELECTOR],
            "base_selectors": list(contract.BASE_SELECTORS), "default_off": True,
            "same_run_ephemeral": True, "gnss_first_stage": "bypassed by design",
            "solution_output_published": False,
        },
        "freeze": {"path": relative(contract.FREEZE), "sha256": contract.FREEZE_SHA256, "commit": contract.FREEZE_COMMIT},
        "manifest": {"path": relative(MANIFEST), "sha256": _sha256_noninput(MANIFEST)},
        "authorization": {"path": relative(AUTHORIZATION), "sha256": _sha256_noninput(AUTHORIZATION), "status": authorization.get("status")},
        "implementation": {"commit": contract.IMPLEMENTATION_COMMIT, "source_sha256": contract.SOURCE_SHA256},
        "routes": by_route,
        "matrix": {
            "candidate_count": 1, "route_count": 2, "runs_per_route": 1,
            "native_solver_invocations": len(metadata), "controls": 0, "reruns": 0, "fallbacks": 0,
            "raw_phone_gnss_process_reads": len(metadata), "raw_phone_imu_process_reads": len(metadata),
            "broadcast_navigation_process_reads": len(metadata), "raw_base_wrapper_hash_reads": len(metadata),
            "raw_base_native_process_reads_declared": len(metadata), "truth_reads": 0,
            "mat_reads_or_generated": 0, "phone_coordinate_reads": 0, "precomputed_coordinate_reads": 0,
            "pdc_reads": 0, "accuracy_calculations": 0, "solution_rows_published": False,
        },
        "gates": {"all_structural_gates_passed": all_passed, "route_order_exact": ordered, "failed_routes": failed},
        "failed_gates": failed,
        "read_accounting": {
            "native_solver_invocations": len(metadata), "raw_phone_gnss_process_reads": len(metadata),
            "raw_phone_imu_process_reads": len(metadata), "broadcast_navigation_process_reads": len(metadata),
            "raw_base_wrapper_hash_reads": len(metadata), "raw_base_native_process_reads_declared": len(metadata),
            "runner_raw_payload_reads": 0, "runner_raw_input_hash_reads": 0, "truth_reads": 0,
            "mat_reads_or_generated": 0, "phone_coordinate_reads": 0, "precomputed_coordinate_reads": 0,
            "pdc_reads": 0, "accuracy_calculations": 0, "kaggle_or_token_access": 0,
            "route_reruns": 0, "fallbacks": 0, "solution_output_reads": 0,
            "solution_output_published": False, "raw_content_copied_or_transformed": False,
            "logs_and_partial_results_preserved": True,
        },
        "forbidden_lanes": {"truth": False, "MAT": False, "phone_coordinates": False, "precomputed_coordinates": False, "PDC": False, "Kaggle_or_token": False, "accuracy": False, "solution_rows": False},
        "truth_free": True, "accuracy_scored": False, "solution_output_published": False,
        "promotion_authorized": False, "stop_before_truth_accuracy_submission": True,
    }


def _sha256_noninput(path: Path) -> str:
    if path.name in RAW_NAMES or path.name == "base.obs":
        raise fail(f"input hash forbidden: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def render_markdown(result: dict[str, Any]) -> str:
    lines = [
        "# Phase114 direct WLS ephemeral C7/D seed raw/base structural result", "",
        f"- status: `{result['status']}`",
        "- recipe: same-run direct WLS raw seed + exact C7/D + Phase99 MULTIFRONTAL_QR + existing raw-base + final Pixel5 offset",
        "- GNSS-first staging: bypassed by design for this opt-in candidate",
        "- truth/MAT/phone-result-coordinate/PDC/Kaggle/accuracy lanes: not read",
        "- solution CSV: withheld; no bytes or rows were opened/interpreted/published", "",
        "## Route gates", "", "| Route | Return | Structural gates |", "|---|---:|---|",
    ]
    for route, report in result["routes"].items():
        passed = sum(value is True for value in report.get("gates", {}).values())
        total = len(report.get("gates", {}))
        lines.append(f"| `{route}` | `{report.get('return_code')}` | `{passed}/{total}` |")
    lines.extend(["", "A failed gate remains sealed fail-closed; no retry, fallback, truth score, or publication is available.", ""])
    return "\n".join(lines)


def main() -> int:
    contract = load_contract()
    try:
        pre = contract.verify_pre_raw()
        prohibited = (
            "raw_phone_gnss_reads", "raw_phone_imu_reads", "broadcast_navigation_reads",
            "raw_base_rinex_reads", "raw_base_hash_reads", "native_solver_invocations",
            "truth_reads", "mat_reads_or_generated", "phone_coordinate_reads",
            "precomputed_coordinate_reads", "pdc_reads", "accuracy_calculations",
        )
        if any(pre.get(key) != 0 for key in prohibited):
            raise fail("pre-raw verifier reported prohibited activity")
        manifest = contract.verify_manifest()
        authorization = contract.verify_authorization(manifest)
        expected_status = "authorized-for-exact-two-route-phase114-direct-seed-raw-structural-execution"
        if authorization.get("status") != expected_status:
            raise fail("Phase114 raw authorization status is not exact")
        metadata = execute_matrix(contract, manifest)
        result = build_result(metadata, contract, manifest, authorization)
        atomic_json(RESULT_JSON, result)
        atomic_text(RESULT_MD, render_markdown(result))
        return 0 if result["status"].startswith("go-") else 1
    except (Phase114ExecutionError, OSError) as exc:
        print(f"phase114 raw execution: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
