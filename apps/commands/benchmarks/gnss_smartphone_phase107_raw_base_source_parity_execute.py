#!/usr/bin/env python3
"""Execute the independently authorized Phase107 raw-base structural matrix.

The pre-raw evaluator and the separate authorization record are checked before
any input path is materialized.  Exactly one native invocation is made for
each route, in MTV-A then LAX-T order.  The wrapper stats raw inputs but never
hashes, copies, or transforms them.  It hashes each sealed base RINEX member
once for the declared preflight, then the native process reads that member
once.  The native solution CSV is placed in a withheld path and is never
opened, hashed, scored, published, or committed.
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
EVALUATOR_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase107_raw_base_source_parity.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase107_raw_base_source_parity_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase107_raw_base_source_parity_authorization_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase107-raw-base-source-parity-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase107_raw_base_source_parity_result_v1.json"
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


class Phase107ExecutionError(ValueError):
    """Raised when the authorized raw-base execution cannot proceed safely."""


def fail(message: str) -> Phase107ExecutionError:
    return Phase107ExecutionError(message)


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
    if path.name in RAW_NAMES or path.name == "base.obs":
        raise fail(f"input-member hash requested outside authorized base preflight: {label}")
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def hash_base_member(path: Path, label: str) -> str:
    """Authorized post-auth base preflight; exactly one full member read."""

    if path.name != "base.obs" or not path.is_file():
        raise fail(f"missing authorized base RINEX member: {label}: {path}")
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
    spec = importlib.util.spec_from_file_location("phase107_raw_base_contract", EVALUATOR_PATH)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load Phase107 evaluator: {EVALUATOR_PATH}")
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
        raise fail(f"Phase107 pinned contract failed: {exc}") from exc
    if pre_raw.get("raw_reads") != 0 or pre_raw.get("base_payload_reads") != 0:
        raise fail("pre-raw verifier reported raw/base payload activity")
    if pre_raw.get("native_solver_invocations") != 0:
        raise fail("pre-raw verifier launched a native solver")
    if pre_raw.get("truth_reads") != 0 or pre_raw.get("mat_reads_or_generated") != 0:
        raise fail("pre-raw verifier opened a prohibited truth/MAT lane")
    if pre_raw.get("accuracy_scored") is not False or pre_raw.get("solution_output_published") is not False:
        raise fail("pre-raw verifier released a solution or accuracy lane")
    expected = "authorized-for-exact-two-route-phase107-raw-base-structural-execution"
    if authorization.get("status") != expected:
        raise fail("Phase107 authorization status is not exact")
    return evaluator, manifest, authorization


def _safe_input_path(path_text: Any, basename: str, route: str) -> Path:
    if not isinstance(path_text, str):
        raise fail(f"missing sealed {basename} path: {route}")
    relative_path = Path(path_text)
    if relative_path.is_absolute() or ".." in relative_path.parts or relative_path.name != basename:
        raise fail(f"unsafe sealed {basename} path: {route}: {path_text}")
    path = ROOT / relative_path
    if not path.is_file():
        raise fail(f"missing sealed {basename} member: {path}")
    return path


def materialize_inputs(evaluator: Any) -> dict[str, dict[str, Any]]:
    """After authorization, stat raw files and hash each base member once."""

    raw = evaluator.phase95_raw_metadata()
    base = evaluator.phase65_base_metadata()
    selected: dict[str, dict[str, Any]] = {}
    for route in ROUTES:
        selected[route] = {"raw": {}, "base": {}}
        for name in RAW_NAMES:
            pin = raw[route][name]
            path = _safe_input_path(pin["path"], name, route)
            size = path.stat().st_size
            if pin.get("bytes") is not None and size != pin["bytes"]:
                raise fail(f"sealed raw byte count changed: {route}/{name}: {size} != {pin['bytes']}")
            # Metadata-only stat.  No raw content is opened or hashed.
            selected[route]["raw"][name] = {
                "path": pin["path"],
                "sha256": pin.get("sha256"),
                "sealed_bytes": pin.get("bytes"),
                "bytes": size,
                "exists_before_launch": True,
                "stat_read_by_wrapper": True,
                "payload_read_by_wrapper": False,
                "hash_read_by_wrapper": False,
                "content_copied_or_transformed": False,
            }
        base_pin = base[route]
        base_path = _safe_input_path(base_pin["path"], "base.obs", route)
        base_size = base_path.stat().st_size
        if base_size != base_pin["bytes"]:
            raise fail(f"sealed base byte count changed: {route}: {base_size} != {base_pin['bytes']}")
        actual_hash = hash_base_member(base_path, f"base {route}")
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


def materialize_command(record: dict[str, Any], inputs: dict[str, Any], evaluator: Any) -> list[str]:
    route = record.get("dataset_id")
    raw = inputs["raw"]
    base = inputs["base"]
    evaluator.validate_command(route, record.get("command"), record, {
        name: {"path": raw[name]["path"], "sha256": raw[name]["sha256"]}
        for name in RAW_NAMES
    }, {
        "path": base["path"], "sha256": base["sha256"], "bytes": base["bytes"],
        "observed_dt_s": base["observed_dt_s"], "moving_mean_samples": base["moving_mean_samples"],
        "approx_position_xyz_m": base["approx_position_xyz_m"],
    })
    command = list(record["command"])
    for flag, name in RAW_FLAGS:
        command[command.index(flag) + 1] = raw[name]["path"]
    command[command.index("--native-base-rinex") + 1] = base["path"]
    command[command.index("--native-base-rinex-sha256") + 1] = base["sha256"]
    return command


def safe_environment() -> dict[str, str]:
    # The native child receives no inherited truth/MAT/token/coordinate paths.
    return {
        "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
        "LANG": "C",
        "LC_ALL": "C",
        "TZ": "UTC",
        "LD_LIBRARY_PATH": "/home/sasaki/.local/lib",
    }


def execute_matrix(manifest: dict[str, Any], evaluator: Any) -> list[dict[str, Any]]:
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite Phase107 output root: {OUTPUT_ROOT}")
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    records = manifest.get("routes")
    if not isinstance(records, list) or [item.get("dataset_id") for item in records] != list(ROUTES):
        raise fail("Phase107 route order/count changed")
    inputs = materialize_inputs(evaluator)
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
        command = materialize_command(record, inputs[route], evaluator)
        stdout_path = route_dir / "stdout.log"
        stderr_path = route_dir / "stderr.log"
        started = time.time()
        return_code: int | None = None
        interrupted = False
        launch_error = ""
        with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
            try:
                completed = subprocess.run(
                    command, cwd=ROOT, env=environment, stdout=stdout, stderr=stderr, check=False,
                )
                return_code = completed.returncode
            except KeyboardInterrupt:
                interrupted = True
                stderr.write(b"\nPhase107 wrapper interrupted; partial output preserved.\n")
            except OSError as exc:
                launch_error = str(exc)
                stderr.write(f"\nPhase107 native launch failed: {exc}\n".encode())
        metadata_record = {
            "schema_version": "smartphone-r5-phase107-raw-base-route-execution.v1",
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
            # Presence is metadata only; the withheld CSV is never opened.
            "withheld_solution_output_present_after_launch": withheld_path.is_file(),
            "stdout": relative(stdout_path),
            "stderr": relative(stderr_path),
            "started_unix_s": started,
            "ended_unix_s": time.time(),
            "return_code": return_code,
            "interrupted": interrupted,
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


def _summary_report(metadata: dict[str, Any], evaluator: Any, manifest: dict[str, Any]) -> dict[str, Any]:
    route = metadata["dataset_id"]
    record = next(item for item in manifest["routes"] if item["dataset_id"] == route)
    expected = int(record["expected_problem_epochs"])
    summary_path = ROOT / metadata["planned_output"]["summary"]
    withheld_path = ROOT / metadata["planned_output"]["withheld_solution_output"]
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
            "coverage_basis": "native summary epoch counts/output contract; withheld CSV never opened",
        },
        "summary_present": summary_path.is_file(),
        "withheld_solution_output_present": withheld_path.is_file(),
        "solution_output_opened": False,
        "solution_output_published": False,
        "accuracy_output_published": False,
        "truth_used": False,
        "mat_used": False,
        "phone_coordinates_used": False,
        "precomputed_coordinates_used": False,
        "pdc_used": False,
        "kaggle_or_token_accessed": False,
        "runner_read_raw_payloads": metadata.get("runner_read_raw_payloads"),
        "runner_read_raw_hashes": metadata.get("runner_read_raw_hashes"),
        "runner_read_base_payload_for_hash": metadata.get("runner_read_base_payload_for_hash"),
        "runner_read_forbidden_lanes": metadata.get("runner_read_truth_mat_phone_coordinate_precomputed_pdc_kaggle"),
        "raw_content_copied_or_transformed": metadata.get("raw_content_copied_or_transformed"),
        "raw_inputs": metadata.get("raw_inputs"),
        "base_input": metadata.get("base_input"),
        "base_read_accounting": {
            "wrapper_hash_verification_reads": 1,
            "native_process_reads_declared": 1,
            "native_summary_read_count": None,
            "base_payload_reads_before_auth": 0,
        },
        "logs": {"stdout": relative(ROOT / metadata["stdout"]), "stderr": relative(ROOT / metadata["stderr"])},
    }
    if not summary_path.is_file():
        report["summary_sha256"] = None
        report["summary_error"] = "native summary was not produced; failure is sealed fail-closed"
        report["telemetry"] = None
        report["gates"] = {
            "native_process_completed": metadata.get("return_code") == 0 and not metadata.get("launch_error") and not metadata.get("interrupted"),
            "summary_present": False,
            "base_factors_active_exactly_once": False,
            "base_path_hash_bytes_and_header_match": False,
            "base_exact_stream_miss_mask_telemetry": False,
            "gnss_first_accepted_iterations_and_strict_cost_decrease": False,
            "gnss_first_full_finite_c7_d_exact_handoff": False,
            "no_global_isb_double_state": False,
            "main_selected_multifrontal_qr": False,
            "main_accepted_iterations_and_strict_cost_decrease": False,
            "main_finite_earth_valid_expected_output_coverage": False,
            "no_fallback_or_solution_publication": False,
        }
        return report
    try:
        summary = read_json(summary_path, f"Phase107 native summary {route}")
        summary_hash = sha256_file(summary_path, f"Phase107 native summary {route}")
    except (Phase107ExecutionError, OSError) as exc:
        report.update({"summary_sha256": None, "summary_error": str(exc), "telemetry": None, "gates": {"summary_present": False}})
        return report
    base_pin = metadata["base_input"]
    base = summary.get("native_base_pseudorange_compensation")
    miss = summary.get("native_base_pseudorange_source_miss_mask")
    epochs = summary.get("epochs")
    gnss = summary.get("gnss_first")
    clock = summary.get("native_source_clock_c0d_factor")
    graph = summary.get("graph")
    output_contract = summary.get("output_contract")
    expected_base = evaluator.phase65_base_metadata()[route]

    base_path_hash_bytes = (
        isinstance(base, dict)
        and base.get("enabled") is True
        and base.get("built") is True
        and base.get("applied") is True
        and base.get("base_rinex") == base_pin.get("path")
        and base.get("base_rinex_sha256") == base_pin.get("sha256")
        and base.get("base_member_sha256") == base_pin.get("sha256")
        and base.get("base_rinex_bytes") == base_pin.get("bytes")
        and base.get("base_rinex_read_count") == 1
        and base.get("base_coordinate_provenance") == "RINEX header APPROX POSITION XYZ"
        and base.get("base_coordinate_xyz_m") == expected_base.get("approx_position_xyz_m")
        and base.get("observed_interval_s") == expected_base.get("observed_dt_s")
        and base.get("moving_mean_samples") == expected_base.get("moving_mean_samples")
        and base.get("same_satellite_signal_only") is True
    )
    base_exact_once = (
        isinstance(base, dict)
        and base.get("preserve_additional_frequency_bands") is False
        and base.get("spp_applied") is False
        and base.get("doppler_applied") is False
        and base.get("tdcp_applied") is False
        and base.get("no_extrapolation_or_endpoint_hold") is True
        and isinstance(epochs, dict)
        and epochs.get("double_difference_pseudorange_factors") == 0
        and epochs.get("double_difference_carrier_factors") == 0
        and isinstance(miss, dict)
        and miss.get("enabled") is True
        and miss.get("retained_factor_epoch_indices_unchanged") is True
        and miss.get("tdcp_doppler_imu_spp_unchanged") is True
        and isinstance(miss.get("pseudorange_factors_inserted"), int)
        and miss.get("pseudorange_factors_inserted") == epochs.get("pseudorange_factors")
        and miss.get("pseudorange_factors_inserted", 0) > 0
    )
    base_miss = (
        base_exact_once
        and isinstance(miss.get("original_adopted_pseudorange_rows"), int)
        and isinstance(miss.get("retained_finite_pc_pseudorange_rows"), int)
        and isinstance(miss.get("dropped_missing_exact_stream_rows"), int)
        and isinstance(miss.get("dropped_out_of_domain_rows"), int)
        and isinstance(miss.get("dropped_nonfinite_correction_rows"), int)
        and miss.get("retained_finite_pc_pseudorange_rows") > 0
        and miss.get("pseudorange_factor_count_consistent") is True
        and miss.get("no_extrapolation_or_endpoint_hold") is True
    )
    gnss_progress = (
        isinstance(gnss, dict)
        and gnss.get("attempted") is True
        and gnss.get("converged") is True
        and gnss.get("epochs") == expected
        and isinstance(gnss.get("iterations"), int) and gnss.get("iterations") >= 1
        and isinstance(gnss.get("c0d_factor_count"), int) and gnss.get("c0d_factor_count") > 0
        and isinstance(gnss.get("c0d_accepted_outer_iterations"), int) and gnss.get("c0d_accepted_outer_iterations") > 0
        and gnss.get("c0d_active_solve_finite_costs") is True
        and _finite(gnss.get("initial_cost")) and _finite(gnss.get("final_cost"))
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
        isinstance(graph, dict) and graph.get("converged") is True
        and isinstance(graph.get("iterations"), int) and graph.get("iterations") >= 1
        and isinstance(clock, dict) and clock.get("clock_c0d_enabled") is True
        and clock.get("meter_state_parity_enabled") is True
        and clock.get("active_solve_attempted") is True
        and isinstance(clock.get("accepted_outer_iterations"), int) and clock.get("accepted_outer_iterations") > 0
        and clock.get("active_solve_finite_costs") is True
        and _finite(clock.get("active_solve_initial_cost")) and _finite(clock.get("active_solve_final_cost"))
        and float(clock["active_solve_final_cost"]) < float(clock["active_solve_initial_cost"])
    )
    output_coverage = (
        isinstance(epochs, dict) and epochs.get("problem") == expected and epochs.get("output") == expected
        and isinstance(output_contract, dict) and output_contract.get("finite_coordinates") is True
    )
    no_global_isb = (
        isinstance(clock, dict) and clock.get("epoch_vector_parity_enabled") is True
        and clock.get("epoch_vector_dimension") == 7 and clock.get("epoch_vector_state_count") == expected
        and clock.get("epoch_vector_handoff_count") == expected and clock.get("global_isb_state_count") == 0
        and clock.get("internal_clock_state_unit") == "metres"
        and clock.get("internal_isb_state_unit") == "metres"
        and clock.get("internal_drift_state_unit") == "metres_per_second"
    )
    no_publication = (
        metadata.get("runner_read_raw_payloads") is False
        and metadata.get("runner_read_raw_hashes") is False
        and metadata.get("runner_read_forbidden_lanes") is False
        and metadata.get("raw_content_copied_or_transformed") is False
        and metadata.get("solution_output_opened") is False
        and report["solution_output_published"] is False
        and report["accuracy_output_published"] is False
        and summary.get("truth_used") is False
        and summary.get("base_factors") is False
        and summary.get("production_default_changed") is False
    )
    no_fallback = summary.get("status") == "imu-combined-factor" and summary.get("native_pdc_state_bridge") is False
    gates = {
        "native_process_completed": metadata.get("return_code") == 0 and not metadata.get("launch_error") and not metadata.get("interrupted"),
        "summary_present": True,
        "base_factors_active_exactly_once": base_exact_once,
        "base_path_hash_bytes_and_header_match": base_path_hash_bytes,
        "base_exact_stream_miss_mask_telemetry": base_miss,
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
        "base_summary_read_count": base.get("base_rinex_read_count") if isinstance(base, dict) else None,
        "telemetry": {
            "base": _pick(base, ("enabled", "built", "applied", "base_rinex", "base_rinex_bytes", "base_rinex_sha256", "base_member_sha256", "base_rinex_read_count", "base_coordinate_provenance", "base_coordinate_xyz_m", "header_version", "header_interval_s", "observed_interval_s", "moving_mean_samples", "matching_key", "same_satellite_signal_only", "matched_base_rows", "finite_base_residual_rows", "smoothed_rows", "interpolation_misses", "adopted_pseudorange_rows", "adopted_rows_corrected", "matched_factor_rows", "finite_correction_rows_among_matched", "spp_applied", "tdcp_applied", "doppler_applied", "no_extrapolation_or_endpoint_hold", "failure")),
            "base_source_miss_mask": _pick(miss, ("enabled", "original_adopted_pseudorange_rows", "retained_finite_pc_pseudorange_rows", "dropped_missing_exact_stream_rows", "dropped_out_of_domain_rows", "dropped_nonfinite_correction_rows", "retained_finite_pc_fraction", "retained_over_original_fraction", "pseudorange_factors_inserted", "pseudorange_factor_count_consistent", "retained_factor_epoch_indices_unchanged", "tdcp_doppler_imu_spp_unchanged", "no_extrapolation_or_endpoint_hold", "sign", "failure")),
            "gnss_first": _pick(gnss, ("attempted", "converged", "epochs", "iterations", "initial_cost", "final_cost", "c0d_factor_count", "c0d_accepted_outer_iterations", "c0d_active_solve_finite_costs", "optimized_c_vector_parity_enabled", "optimized_c_export_valid", "optimized_c_epoch_count", "optimized_c_finite_component_count", "optimized_c_nonfinite_component_count", "optimized_c_dimension", "optimized_d_export_valid", "optimized_d_epoch_count", "optimized_d_finite_count", "optimized_d_nonfinite_count", "epoch_identity_alignment_valid", "handoff_mode")),
            "main": {**_pick(graph, ("factors", "values", "imu_intervals", "iterations", "converged", "initial_cost", "final_cost")), **_pick(clock, ("clock_c0d_enabled", "meter_state_parity_enabled", "epoch_vector_parity_enabled", "epoch_vector_dimension", "epoch_vector_state_count", "epoch_vector_handoff_count", "global_isb_state_count", "internal_clock_state_unit", "internal_isb_state_unit", "internal_drift_state_unit", "clock_c0d_factor_count", "active_solve_attempted", "active_solve_initial_cost", "active_solve_final_cost", "accepted_outer_iterations", "total_inner_lambda_attempts", "initial_lambda", "maximum_lambda", "final_lambda", "active_solve_finite_costs", "termination_branch_reason", "conditioning_proxy"))},
            "output": _pick(epochs, ("problem", "output", "pseudorange_factors", "tdcp_factors_built", "double_difference_pseudorange_factors", "double_difference_carrier_factors")),
            "solver": _pick(summary, ("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_enabled", "native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected", "selected_linear_solver_type", "selected_solver_branch", "selected_elimination_function")),
            "gates": {"base_exactly_once": base_exact_once, "base_path_hash_bytes_header": base_path_hash_bytes, "base_miss_mask": base_miss, "gnss_first_progress": gnss_progress, "C7_handoff": c_handoff, "D_handoff": d_handoff, "no_global_ISB": no_global_isb, "main_QR": qr_selected, "main_progress": main_progress, "output_coverage": output_coverage, "no_publication": no_publication},
            "units_and_policy": {"clock": "metres", "ISB": "metres", "drift_D": "metres_per_second", "dt": "seconds", "ccdd_sigma_m": 0.1, "raw_D_source": "retained EpochSeed.receiver_clock_drift_mps", "base_correction": "existing raw code factor only; positive pc subtracted once", "base_coordinate_source": "raw RINEX header APPROX POSITION XYZ", "fallback": "none"},
        },
        "gates": gates,
    })
    return report


def build_result(metadata: list[dict[str, Any]], manifest: dict[str, Any], authorization: dict[str, Any], evaluator: Any) -> dict[str, Any]:
    reports = [_summary_report(item, evaluator, manifest) for item in metadata]
    by_route = {item["dataset_id"]: item for item in reports}
    all_expected = len(reports) == 2 and [item["dataset_id"] for item in reports] == list(ROUTES)
    gate_names = (
        "base_factors_active_exactly_once", "base_path_hash_bytes_and_header_match", "base_exact_stream_miss_mask_telemetry",
        "gnss_first_accepted_iterations_and_strict_cost_decrease", "gnss_first_full_finite_c7_d_exact_handoff",
        "no_global_isb_double_state", "main_selected_multifrontal_qr", "main_accepted_iterations_and_strict_cost_decrease",
        "main_finite_earth_valid_expected_output_coverage", "no_fallback_or_solution_publication",
    )
    all_passed = all_expected and all(all(item.get("gates", {}).get(name) is True for name in gate_names) for item in reports)
    failed = {
        route: [name for name in gate_names if report.get("gates", {}).get(name) is not True]
        for route, report in by_route.items()
        if any(report.get("gates", {}).get(name) is not True for name in gate_names)
    }
    base_process_reads = sum(1 for item in reports if item.get("base_summary_read_count") == 1)
    result = {
        "schema_version": "smartphone-r5-phase107-raw-base-source-parity-structural-result.v1",
        "phase": 107,
        "execution_label": "Luna Max",
        "status": "go-phase107-raw-base-structural" if all_passed else "no-go-phase107-raw-base-structural",
        "decision": "Structural gates passed; solution and truth/accuracy lanes remain unauthorized." if all_passed else "Structural gate failed closed; no solution, truth, accuracy, fallback, or rerun lane is available.",
        "candidate": {"id": evaluator.CANDIDATE_ID, "selectors": [evaluator.PHASE93_SELECTOR, evaluator.VECTOR_SELECTOR, evaluator.QR_SELECTOR], "base_flags": ["--native-base-pseudorange-compensation", "--native-base-rinex", "--native-base-rinex-sha256", "--native-base-pseudorange-source-miss-mask"], "default_off": True, "diagnostic_only": True},
        "freeze": {"path": evaluator.relative(evaluator.FREEZE), "sha256": evaluator.FREEZE_SHA, "commit": evaluator.FREEZE_COMMIT},
        "audit": {"path": evaluator.relative(evaluator.AUDIT), "sha256": evaluator.AUDIT_SHA, "commit": evaluator.AUDIT_COMMIT},
        "authorization": {"path": evaluator.relative(evaluator.AUTHORIZATION), "sha256": sha256_file(evaluator.AUTHORIZATION, "Phase107 authorization"), "status": authorization.get("status")},
        "manifest": {"path": evaluator.relative(evaluator.MANIFEST), "sha256": sha256_file(evaluator.MANIFEST, "Phase107 manifest")},
        "evaluator": {"path": evaluator.relative(evaluator.EVALUATOR), "sha256": sha256_file(evaluator.EVALUATOR, "Phase107 evaluator")},
        "execution_wrapper": {"path": relative(Path(__file__).resolve()), "sha256": sha256_file(Path(__file__).resolve(), "Phase107 wrapper")},
        "implementation": manifest.get("implementation"),
        "routes": by_route,
        "matrix": {"candidate_count": 1, "routes": 2, "runs_per_route": 1, "native_solver_invocations": len(metadata), "controls": 0, "reruns": 0, "fallbacks": 0, "raw_device_gnss_process_reads": len(metadata), "raw_device_imu_process_reads": len(metadata), "broadcast_navigation_process_reads": len(metadata), "base_rinex_wrapper_hash_verification_reads": len(metadata), "base_rinex_native_process_reads": base_process_reads, "base_rinex_payload_reads_total": len(metadata) + base_process_reads, "base_rinex_read_accounting": "one wrapper SHA verification read plus one native RINEX process read per route", "runner_raw_payload_reads": 0, "raw_input_hash_reads": 0, "truth_reads": 0, "mat_reads_or_generated": 0, "phone_coordinate_reads": 0, "precomputed_coordinate_reads": 0, "pdc_reads": 0, "accuracy_calculations": 0, "kaggle_or_token_access": 0, "solution_rows_published": False},
        "gates": {"all_structural_gates_passed": all_passed, "route_order_exact": all_expected, "failed_routes": failed},
        "failed_gates": failed,
        "read_accounting": {"native_solver_invocations": len(metadata), "raw_device_gnss_reads": len(metadata), "raw_device_imu_reads": len(metadata), "broadcast_navigation_reads": len(metadata), "runner_raw_payload_reads": 0, "runner_raw_input_hash_reads": 0, "base_rinex_wrapper_hash_verification_reads": len(metadata), "base_rinex_native_process_reads": base_process_reads, "base_rinex_payload_reads_total": len(metadata) + base_process_reads, "truth_reads": 0, "mat_reads_or_generated": 0, "phone_coordinate_reads": 0, "precomputed_coordinate_reads": 0, "pdc_reads": 0, "accuracy_calculations": 0, "kaggle_or_token_access": 0, "route_reruns": 0, "fallbacks": 0, "accuracy_scored": False, "solution_output_opened": False, "solution_output_published": False, "raw_content_copied_or_transformed": False, "logs_and_partial_results_preserved": True},
        "raw_input_provenance": {"names_exact": list(RAW_NAMES), "roles": {"device_gnss.csv": "raw Android GNSS only", "device_imu.csv": "raw Android IMU only", "brdc.nav": "broadcast navigation only"}, "path_source": "Phase95 sealed result raw_inputs.path", "wrapper_path_resolution": "post-auth metadata-only stat then exact path substitution", "wrapper_raw_payload_reads": 0, "wrapper_raw_hash_reads": 0, "copy_or_transform": False},
        "base_provenance": {"path_source": "Phase65 sealed base_inputs.path", "coordinate_source": "raw RINEX header APPROX POSITION XYZ only", "phone_coordinate_or_precomputed_source": False, "base_wrapper_hash_reads_per_route": 1, "base_native_process_reads_per_route": 1, "base_hash_and_bytes_checked_before_launch": True, "base_correction_active_once_contract": "native telemetry: enabled+built+applied, one base RINEX read, one source-exact correction/miss-mask pass on existing code factors, no new base factor family"},
        "forbidden_lanes": {"truth": False, "MAT": False, "phone_coordinates": False, "precomputed_coordinates": False, "PDC": False, "Kaggle_or_token": False, "accuracy": False, "solution_rows": False},
        "truth_free": True,
        "accuracy_scored": False,
        "solution_output_published": False,
        "promotion_authorized": False,
        "stop_before_truth_accuracy_submission": True,
    }
    return result


def result_markdown(result: dict[str, Any]) -> str:
    lines = [
        "# Phase107 raw-base source-parity structural result", "",
        f"- Status: `{result['status']}`",
        f"- Decision: {result['decision']}",
        "- Matrix: exactly MTV-A then LAX-T, one native invocation each; no rerun, fallback, truth, accuracy, or solution publication.",
        "- Base accounting: one wrapper SHA verification read plus one native RINEX process read per route; station coordinate source is the raw RINEX header APPROX POSITION XYZ.",
        "", "## Route summary", "", "| Route | Return | Base correction | GNSS-first | C7/D | Main QR/progress | Output | Failed gates |", "|---|---:|---|---|---|---|---|---|",
    ]
    for route in ROUTES:
        item = result["routes"].get(route, {})
        telem = item.get("telemetry") or {}
        base = telem.get("base") or {}
        miss = telem.get("base_source_miss_mask") or {}
        gnss = telem.get("gnss_first") or {}
        main = telem.get("main") or {}
        solver = telem.get("solver") or {}
        output = telem.get("output") or {}
        gates = item.get("gates") or {}
        lines.append(f"| `{route}` | `{item.get('return_code')}` | `enabled={base.get('enabled')}; built={base.get('built')}; applied={base.get('applied')}; retained={miss.get('retained_finite_pc_pseudorange_rows')}; miss={miss.get('dropped_missing_exact_stream_rows') + miss.get('dropped_out_of_domain_rows') + miss.get('dropped_nonfinite_correction_rows') if isinstance(miss.get('dropped_missing_exact_stream_rows'), int) and isinstance(miss.get('dropped_out_of_domain_rows'), int) and isinstance(miss.get('dropped_nonfinite_correction_rows'), int) else None}; native_reads={base.get('base_rinex_read_count')}` | `{gnss.get('initial_cost')} → {gnss.get('final_cost')} / accepted={gnss.get('c0d_accepted_outer_iterations')}` | `C {gnss.get('optimized_c_epoch_count')}/{gnss.get('optimized_c_finite_component_count')}; D {gnss.get('optimized_d_epoch_count')}/{gnss.get('optimized_d_finite_count')}; exact={gnss.get('epoch_identity_alignment_valid')}` | `{solver.get('selected_linear_solver_type')} / {main.get('accepted_outer_iterations')} / {main.get('active_solve_initial_cost')} → {main.get('active_solve_final_cost')}` | `{output.get('problem')}→{output.get('output')}` | `{[name for name, passed in gates.items() if passed is not True]}` |")
    lines.extend(["", "The withheld CSV paths are metadata only; they were never opened, published, scored, or committed. Structural GO does not authorize truth/accuracy evaluation or submission.", ""])
    return "\n".join(lines)


def metadata_from_existing() -> list[dict[str, Any]]:
    if not OUTPUT_ROOT.is_dir():
        raise fail(f"missing preserved Phase107 output root: {OUTPUT_ROOT}")
    return [read_json(OUTPUT_ROOT / route.replace("/", "__") / "run_metadata.json", f"Phase107 route metadata {route}") for route in ROUTES]


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
    except (Phase107ExecutionError, OSError) as exc:
        print(f"phase107 structural runner: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
