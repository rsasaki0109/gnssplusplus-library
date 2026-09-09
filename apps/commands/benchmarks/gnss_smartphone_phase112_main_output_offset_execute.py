#!/usr/bin/env python3
"""Run exactly one authorized Phase112 raw/base structural matrix.

The wrapper materializes the sealed Phase95 phone paths and Phase65 base paths
only after the independent raw authorization has been committed.  Phone raw
files are passed through without copying, hashing, or parsing.  Each base
member is SHA-verified once by this wrapper and is then read once by the
native process.  The native solution is opaque: its bytes are hashed and its
row count is sealed after exit, but its coordinates are never interpreted or
published.  Truth and accuracy are a separate, later evaluator boundary.
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
CONTRACT_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase112_main_output_offset.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_raw_authorization_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase112-main-output-offset-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_structural_result_v1.json"
RESULT_MD = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_structural_result_v1.md"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")


class Phase112ExecutionError(ValueError):
    """Raised when the one-shot Phase112 structural execution fails closed."""


def fail(message: str) -> Phase112ExecutionError:
    return Phase112ExecutionError(message)


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
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
    return digest.hexdigest()


def hash_base_once(path: Path, label: str) -> str:
    if path.name != "base.obs" or not path.is_file():
        raise fail(f"missing authorized raw base member: {label}: {path}")
    # This is the sole wrapper payload read of a base member.
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
    spec = importlib.util.spec_from_file_location("phase112_raw_contract", CONTRACT_PATH)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load Phase112 contract: {CONTRACT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def safe_member(path_text: Any, basename: str, route: str) -> Path:
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
            # Metadata-only: no open/read/hash of phone payloads here.
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
        actual = hash_base_once(path, f"base {route}")
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
    raw = {
        name: {"path": inputs["raw"][name]["path"], "sha256": inputs["raw"][name]["sha256"]}
        for name in RAW_NAMES
    }
    base = {
        "path": inputs["base"]["path"], "sha256": inputs["base"]["sha256"],
        "bytes": inputs["base"]["bytes"], "observed_dt_s": inputs["base"]["observed_dt_s"],
        "moving_mean_samples": inputs["base"]["moving_mean_samples"],
        "approx_position_xyz_m": inputs["base"]["approx_position_xyz_m"],
    }
    contract.validate_command(record["dataset_id"], record.get("command"), record, raw, base)
    replacements = {
        "__PHASE112_RAW_DEVICE_GNSS__": inputs["raw"]["device_gnss.csv"]["path"],
        "__PHASE112_RAW_DEVICE_IMU__": inputs["raw"]["device_imu.csv"]["path"],
        "__PHASE112_RAW_BROADCAST_NAV__": inputs["raw"]["brdc.nav"]["path"],
        "__PHASE112_RAW_BASE_RINEX__": inputs["base"]["path"],
        "__PHASE112_RAW_BASE_SHA256__": inputs["base"]["sha256"],
    }
    command = [replacements.get(token, token) for token in record["command"]]
    for token in command:
        if token in contract.FORBIDDEN_FLAGS:
            raise fail(f"forbidden command flag after materialization: {record['dataset_id']}/{token}")
    return command


def safe_environment() -> dict[str, str]:
    # The native child receives only deterministic locale/loader variables;
    # no truth, MAT, coordinate, token, or Kaggle path is inherited.
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
    return True


def close(actual: Any, expected: Any, tolerance: float = 1e-6) -> bool:
    return finite(actual) and finite(expected) and abs(float(actual) - float(expected)) <= tolerance


def read_summary(path: Path, route: str) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    if not path.is_file():
        return None, {"path": relative(path), "present": False}
    payload = path.read_bytes()
    try:
        summary = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        return None, {"path": relative(path), "present": True, "error": str(exc)}
    if not isinstance(summary, dict):
        return None, {"path": relative(path), "present": True, "error": "summary is not an object"}
    return summary, {"path": relative(path), "present": True, "bytes": len(payload), "sha256": hashlib.sha256(payload).hexdigest(), "read_count": 1}


def seal_solution(path: Path, route: str, expected_rows: int) -> dict[str, Any]:
    if not path.is_file():
        return {"path": relative(path), "present": False, "opened": False, "published": False}
    digest = hashlib.sha256()
    line_count = 0
    first_line = b""
    with path.open("rb") as handle:
        first_line = handle.readline()
        if first_line:
            line_count = 1
            digest.update(first_line)
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
            line_count += chunk.count(b"\n")
    rows = max(0, line_count - 1)
    return {
        "path": relative(path), "present": True, "opened": True, "published": False,
        "sha256": digest.hexdigest(), "bytes": path.stat().st_size,
        "header": first_line.decode("utf-8", errors="replace").rstrip("\r\n"),
        "rows": rows, "expected_domain_rows": expected_rows,
        "row_count_matches_domain": rows == expected_rows,
        "coordinate_rows_omitted": True, "read_count": 1,
    }


def validate_route(metadata: dict[str, Any], contract: Any, manifest: dict[str, Any]) -> dict[str, Any]:
    route = metadata["dataset_id"]
    record = next(item for item in manifest["routes"] if item["dataset_id"] == route)
    expected = int(record["expected_problem_epochs"])
    domain = int(record["domain_rows"])
    summary_path = ROOT / metadata["planned_output"]["summary"]
    solution_path = ROOT / metadata["planned_output"]["withheld_solution_output"]
    summary, summary_meta = read_summary(summary_path, route)
    solution_meta = seal_solution(solution_path, route, domain)
    report: dict[str, Any] = {
        "dataset_id": route,
        "return_code": metadata.get("return_code"),
        "expected": {"domain_rows": domain, "problem_epochs": expected, "output_epochs": expected},
        "summary": summary_meta,
        "solution_hash_seal": solution_meta,
        "raw_inputs": metadata.get("raw_inputs"),
        "base_input": metadata.get("base_input"),
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
            "solution_output_hash_reads": 1 if solution_meta.get("present") else 0,
            "raw_content_copied_or_transformed": False,
        },
    }
    names = (
        "native_process_completed", "summary_present", "base_path_hash_bytes_header_match",
        "base_factors_active_exactly_once", "gnss_first_progress_strict_cost_decrease",
        "gnss_first_full_finite_c7_d_exact_handoff", "main_qr_progress_strict_cost_decrease",
        "offset_pixel5_exactly_once_final_output", "finite_earth_valid_expected_output_coverage",
        "no_solver_fallback_or_solution_publication",
    )
    if summary is None:
        report["telemetry"] = None
        report["gates"] = {name: False for name in names}
        report["failure_reasons"] = [name for name in names if name != "native_process_completed"]
        if metadata.get("return_code") != 0:
            report["failure_reasons"].insert(0, "native_process_completed")
        return report

    base = summary.get("native_base_pseudorange_compensation")
    miss = summary.get("native_base_pseudorange_source_miss_mask")
    epochs = summary.get("epochs")
    gnss = summary.get("gnss_first")
    clock = summary.get("native_source_clock_c0d_factor")
    graph = summary.get("graph")
    raw_utc = summary.get("raw_utc_key_contract")
    output_contract = summary.get("output_contract")
    offset = summary.get("upstream_position_offset")
    base_pin = metadata["base_input"]

    base_path_ok = (
        isinstance(base, dict) and base.get("enabled") is True and base.get("built") is True
        and base.get("applied") is True and base.get("preserve_additional_frequency_bands") is True
        and base.get("base_rinex") == base_pin.get("path")
        and base.get("base_rinex_sha256") == base_pin.get("sha256")
        and base.get("base_member_sha256") == base_pin.get("sha256")
        and base.get("base_rinex_bytes") == base_pin.get("bytes")
        and base.get("base_rinex_read_count") == 1
        and base.get("base_coordinate_provenance") == "RINEX header APPROX POSITION XYZ"
        and isinstance(base.get("base_coordinate_xyz_m"), list)
        and len(base["base_coordinate_xyz_m"]) == 3
        and all(close(a, b) for a, b in zip(base["base_coordinate_xyz_m"], base_pin["approx_position_xyz_m"]))
        and close(base.get("observed_interval_s"), base_pin.get("observed_dt_s"))
        and base.get("moving_mean_samples") == base_pin.get("moving_mean_samples")
        and base.get("same_satellite_signal_only") is True
    )
    taxonomy = miss.get("signal_taxonomy_by_signal") if isinstance(miss, dict) else None
    taxonomy_ok = isinstance(taxonomy, dict) and bool(taxonomy) and all(
        isinstance(item, dict) and isinstance(item.get("frequency_band"), str)
        and all(isinstance(item.get(key), int) and not isinstance(item.get(key), bool) and item.get(key) >= 0 for key in (
            "original_adopted_rows", "retained_finite_pc_rows", "corrected_rows",
            "matched_exact_stream_rows", "finite_correction_rows_among_matched",
            "dropped_missing_exact_stream_rows", "dropped_out_of_domain_rows",
            "dropped_nonfinite_correction_rows"))
        and item.get("original_adopted_rows") == item.get("retained_finite_pc_rows") + item.get("dropped_missing_exact_stream_rows") + item.get("dropped_out_of_domain_rows") + item.get("dropped_nonfinite_correction_rows")
        and item.get("corrected_rows") == item.get("retained_finite_pc_rows")
        and item.get("factor_count_consistent") is True
        for item in taxonomy.values()
    )
    base_exactly_once = (
        isinstance(base, dict) and base.get("source_model_build_count") == 1
        and base.get("correction_application_pass_count") == 1
        and base.get("correction_applied_exactly_once") is True
        and base.get("duplicate_correction_rejected") is False
        and isinstance(miss, dict) and miss.get("correction_application_pass_count") == 1
        and miss.get("correction_applied_exactly_once") is True
        and miss.get("duplicate_correction_rejected") is False
        and isinstance(epochs, dict) and epochs.get("double_difference_pseudorange_factors") == 0
        and epochs.get("double_difference_carrier_factors") == 0
        and miss.get("pseudorange_factors_inserted") == epochs.get("pseudorange_factors")
        and isinstance(miss.get("pseudorange_factors_inserted"), int)
        and miss.get("pseudorange_factors_inserted") > 0
    )
    gnss_progress = (
        isinstance(gnss, dict) and gnss.get("attempted") is True and gnss.get("converged") is True
        and gnss.get("epochs") == expected and isinstance(gnss.get("iterations"), int)
        and gnss.get("iterations") >= 1 and isinstance(gnss.get("c0d_factor_count"), int)
        and gnss.get("c0d_factor_count") > 0 and isinstance(gnss.get("c0d_accepted_outer_iterations"), int)
        and gnss.get("c0d_accepted_outer_iterations") > 0 and gnss.get("c0d_active_solve_finite_costs") is True
        and finite(gnss.get("initial_cost")) and finite(gnss.get("final_cost"))
        and gnss.get("final_cost") < gnss.get("initial_cost")
    )
    handoff = (
        isinstance(gnss, dict) and gnss.get("optimized_c_vector_parity_enabled") is True
        and gnss.get("optimized_c_export_valid") is True and gnss.get("optimized_c_dimension") == 7
        and gnss.get("optimized_c_epoch_count") == expected
        and gnss.get("optimized_c_finite_component_count") == expected * 7
        and gnss.get("optimized_c_nonfinite_component_count") == 0
        and gnss.get("optimized_d_export_valid") is True and gnss.get("optimized_d_epoch_count") == expected
        and gnss.get("optimized_d_finite_count") == expected and gnss.get("optimized_d_nonfinite_count") == 0
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
        isinstance(graph, dict) and graph.get("converged") is True
        and isinstance(graph.get("iterations"), int) and graph.get("iterations") >= 1
        and finite(graph.get("initial_cost")) and finite(graph.get("final_cost"))
        and graph.get("final_cost") < graph.get("initial_cost")
        and isinstance(clock, dict) and clock.get("clock_c0d_enabled") is True
        and clock.get("meter_state_parity_enabled") is True and clock.get("epoch_vector_parity_enabled") is True
        and clock.get("epoch_vector_dimension") == 7 and clock.get("global_isb_state_count") == 0
        and clock.get("active_solve_attempted") is True
        and isinstance(clock.get("accepted_outer_iterations"), int) and clock.get("accepted_outer_iterations") > 0
        and clock.get("active_solve_finite_costs") is True
        and finite(clock.get("active_solve_initial_cost")) and finite(clock.get("active_solve_final_cost"))
        and clock.get("active_solve_final_cost") < clock.get("active_solve_initial_cost")
        and clock.get("internal_clock_state_unit") == "metres"
        and clock.get("internal_isb_state_unit") == "metres"
        and clock.get("internal_drift_state_unit") == "metres_per_second"
    )
    offset_ok = (
        isinstance(offset, dict) and offset.get("enabled") is True and offset.get("applied") is True
        and offset.get("phone") == "pixel5" and offset.get("corrected_epochs") == expected
        and close(offset.get("offset_rl_m"), -0.1) and close(offset.get("offset_ud_m"), -0.3)
        and close(offset.get("max_offset_enu_m"), 0.31622776601683794, 1e-9)
        and offset.get("rotation_contract") == "Rx*Ry*Rz(rpy-[0,0,pi])" and offset.get("failure") == ""
        and solution_meta.get("present") is True and solution_meta.get("row_count_matches_domain") is True
    )
    output_ok = (
        isinstance(epochs, dict) and epochs.get("problem") == expected and epochs.get("output") == expected
        and isinstance(epochs.get("pseudorange_factors"), int) and epochs.get("pseudorange_factors") > 0
        and isinstance(epochs.get("tdcp_factors_built"), int) and epochs.get("tdcp_factors_built") > 0
        and isinstance(raw_utc, dict) and raw_utc.get("raw_epoch_keys") == expected
        and raw_utc.get("target_epochs") == expected - 1 and raw_utc.get("exact_solution_epochs") == expected - 1
        and raw_utc.get("unresolved_epochs") == 0
        and isinstance(output_contract, dict) and output_contract.get("finite_coordinates") is True
        and all_finite(output_contract)
    )
    no_publication = (
        metadata.get("runner_read_raw_payloads") is False and metadata.get("runner_read_raw_hashes") is False
        and metadata.get("runner_read_truth_mat_phone_coordinate_precomputed_pdc_kaggle") is False
        and metadata.get("raw_content_copied_or_transformed") is False
        and metadata.get("solution_output_published") is False and summary.get("truth_used") is False
        and summary.get("production_default_changed") is False
    )
    no_solver_fallback = (
        summary.get("status") == "imu-combined-factor" and summary.get("native_pdc_state_bridge") is False
        and summary.get("native_pdc_imu_tdcp_no_bridge") is True
        and summary.get("native_source_direct_observable_quality_enabled") is True
    )
    process_ok = metadata.get("return_code") == 0 and not metadata.get("launch_error") and not metadata.get("interrupted") and not metadata.get("timed_out")
    gates = {
        "native_process_completed": process_ok,
        "summary_present": summary_meta.get("present") is True,
        "base_path_hash_bytes_header_match": base_path_ok,
        "base_factors_active_exactly_once": base_exactly_once and taxonomy_ok,
        "gnss_first_progress_strict_cost_decrease": gnss_progress,
        "gnss_first_full_finite_c7_d_exact_handoff": handoff,
        "main_qr_progress_strict_cost_decrease": qr_selected and main_progress,
        "offset_pixel5_exactly_once_final_output": offset_ok,
        "finite_earth_valid_expected_output_coverage": output_ok,
        "no_solver_fallback_or_solution_publication": no_solver_fallback and no_publication,
    }
    report["telemetry"] = {
        "base": base, "source_miss_mask": miss, "gnss_first": gnss,
        "main": {"graph": graph, "clock": clock}, "offset": offset,
        "raw_utc": raw_utc, "output": {"epochs": epochs, "contract": output_contract},
        "utc_wall_clock_fallback_applied": summary.get("android_utc_wall_clock_fallback_applied"),
        "solver": {
            "enabled": summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_enabled"),
            "selected": summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected"),
            "linear_solver_type": summary.get("selected_linear_solver_type"),
            "branch": summary.get("selected_solver_branch"),
            "elimination": summary.get("selected_elimination_function"),
        },
    }
    report["gates"] = gates
    report["failure_reasons"] = [name for name, passed in gates.items() if passed is not True]
    return report


def execute_matrix(contract: Any, manifest: dict[str, Any]) -> list[dict[str, Any]]:
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite Phase112 output root: {OUTPUT_ROOT}")
    records = manifest.get("routes")
    if not isinstance(records, list) or [item.get("dataset_id") for item in records] != list(ROUTES):
        raise fail("Phase112 route order/count changed")
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
                stderr.write(f"\nPhase112 native launch timed out: {exc}\n".encode())
            except KeyboardInterrupt:
                interrupted = True
                stderr.write(b"\nPhase112 wrapper interrupted; partial output preserved.\n")
            except OSError as exc:
                launch_error = str(exc)
                stderr.write(f"\nPhase112 native launch failed: {exc}\n".encode())
        item = {
            "schema_version": "smartphone-r5-phase112-main-output-offset-route-execution.v1",
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
        "native_process_completed", "summary_present", "base_path_hash_bytes_header_match",
        "base_factors_active_exactly_once", "gnss_first_progress_strict_cost_decrease",
        "gnss_first_full_finite_c7_d_exact_handoff", "main_qr_progress_strict_cost_decrease",
        "offset_pixel5_exactly_once_final_output", "finite_earth_valid_expected_output_coverage",
        "no_solver_fallback_or_solution_publication",
    )
    all_passed = len(reports) == 2 and [item["dataset_id"] for item in reports] == list(ROUTES) and all(
        all(report.get("gates", {}).get(name) is True for name in gate_names) for report in reports
    )
    failed = {
        route: [name for name in gate_names if report.get("gates", {}).get(name) is not True]
        for route, report in by_route.items()
        if any(report.get("gates", {}).get(name) is not True for name in gate_names)
    }
    return {
        "schema_version": "smartphone-r5-phase112-main-output-offset-structural-result.v1",
        "phase": 112, "execution_label": "Luna Max",
        "status": "go-phase112-main-output-offset-structural" if all_passed else "no-go-phase112-main-output-offset-structural",
        "decision": "Structural gates passed; truth/accuracy and solution release remain separately unauthorized." if all_passed else "Structural gate failed closed; preserve outputs and do not retry, fallback, truth-score, or publish.",
        "candidate": {
            "id": contract.CANDIDATE_ID, "selector": contract.OFFSET_SELECTOR,
            "selectors": [contract.PHASE93_SELECTOR, contract.VECTOR_SELECTOR, contract.QR_SELECTOR, contract.OFFSET_SELECTOR],
            "base_selectors": list(contract.BASE_SELECTORS), "default_off": True,
            "pixel5_only": True, "offset_applied_final_output_only": True,
            "solution_output_published": False,
        },
        "freeze": {"path": relative(contract.FREEZE), "sha256": contract.FREEZE_SHA256, "commit": contract.FREEZE_COMMIT},
        "manifest": {"path": relative(MANIFEST), "sha256": sha256_file(MANIFEST, "Phase112 manifest")},
        "authorization": {"path": relative(AUTHORIZATION), "sha256": sha256_file(AUTHORIZATION, "Phase112 raw authorization"), "status": authorization.get("status")},
        "implementation": {"commit": contract.IMPLEMENTATION_COMMIT, "source_sha256": contract.SOURCE_SHA256},
        "routes": by_route,
        "matrix": {
            "candidate_count": 1, "route_count": 2, "runs_per_route": 1,
            "native_solver_invocations": len(metadata), "controls": 0, "reruns": 0,
            "fallbacks": 0, "raw_phone_gnss_process_reads": len(metadata),
            "raw_phone_imu_process_reads": len(metadata), "broadcast_navigation_process_reads": len(metadata),
            "raw_base_wrapper_hash_reads": len(metadata), "raw_base_native_process_reads_declared": len(metadata),
            "truth_reads": 0, "mat_reads_or_generated": 0, "phone_coordinate_reads": 0,
            "precomputed_coordinate_reads": 0, "pdc_reads": 0, "accuracy_calculations": 0,
            "solution_rows_published": False,
        },
        "gates": {"all_structural_gates_passed": all_passed, "route_order_exact": len(reports) == 2 and [item["dataset_id"] for item in reports] == list(ROUTES), "failed_routes": failed},
        "failed_gates": failed,
        "read_accounting": {
            "native_solver_invocations": len(metadata), "raw_phone_gnss_process_reads": len(metadata),
            "raw_phone_imu_process_reads": len(metadata), "broadcast_navigation_process_reads": len(metadata),
            "raw_base_wrapper_hash_reads": len(metadata), "raw_base_native_process_reads_declared": len(metadata),
            "runner_raw_payload_reads": 0, "runner_raw_input_hash_reads": 0, "truth_reads": 0,
            "mat_reads_or_generated": 0, "phone_coordinate_reads": 0, "precomputed_coordinate_reads": 0,
            "pdc_reads": 0, "accuracy_calculations": 0, "kaggle_or_token_access": 0,
            "route_reruns": 0, "fallbacks": 0, "solution_output_hash_seal_reads": len(metadata),
            "solution_output_published": False, "raw_content_copied_or_transformed": False,
            "logs_and_partial_results_preserved": True,
        },
        "forbidden_lanes": {"truth": False, "MAT": False, "phone_coordinates": False, "precomputed_coordinates": False, "PDC": False, "Kaggle_or_token": False, "accuracy": False, "solution_rows": False},
        "truth_free": True, "accuracy_scored": False, "solution_output_published": False,
        "promotion_authorized": False, "stop_before_truth_accuracy_submission": True,
    }


def render_markdown(result: dict[str, Any]) -> str:
    lines = [
        "# Phase112 main output offset raw/base structural result", "",
        f"- status: `{result['status']}`",
        "- recipe: Phase109 raw/base + Phase101 C7/D handoff + Phase99 MULTIFRONTAL_QR + final Pixel5 offset",
        "- truth/MAT/phone-result-coordinate/PDC/Kaggle/accuracy lanes: not read",
        "- solution CSV: opaque hash/row seal only; coordinate rows omitted and unpublished", "",
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
        prohibited = ("raw_phone_gnss_reads", "raw_phone_imu_reads", "broadcast_navigation_reads", "raw_base_rinex_reads", "raw_base_hash_reads", "native_solver_invocations", "truth_reads", "mat_reads_or_generated", "phone_coordinate_reads", "precomputed_coordinate_reads", "pdc_reads", "accuracy_calculations")
        if any(pre.get(key) != 0 for key in prohibited):
            raise fail("pre-raw verifier reported prohibited activity")
        manifest = contract.verify_manifest()
        authorization = contract.verify_authorization(manifest)
        if authorization.get("status") != "authorized-for-exact-two-route-phase112-raw-structural-execution":
            raise fail("Phase112 raw authorization status is not exact")
        metadata = execute_matrix(contract, manifest)
        result = build_result(metadata, contract, manifest, authorization)
        atomic_json(RESULT_JSON, result)
        atomic_text(RESULT_MD, render_markdown(result))
        return 0 if result["status"].startswith("go-") else 1
    except (Phase112ExecutionError, OSError) as exc:
        print(f"phase112 raw execution: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
