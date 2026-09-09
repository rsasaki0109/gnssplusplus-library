#!/usr/bin/env python3
"""Execute exactly one authorized Phase117 A/LAX raw/base matrix.

Only the two sealed phone raw routes, broadcast navigation, and sealed raw
base RINEX are materialized after independent authorization.  Phone members
are stat-only at the wrapper; each base member is hashed once for identity and
then read once by the native process.  The native solution is opaque: after a
successful launch this wrapper seals bytes/header/row count without parsing
coordinates.  Truth and accuracy are a separate later authorization.
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
CONTRACT_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase117_tdcp_weighting.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_raw_authorization_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase117-tdcp-weighting-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_structural_result_v1.json"
RESULT_MD = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_structural_result_v1.md"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")


class Phase117ExecutionError(ValueError):
    """Raised when the one-shot Phase117 execution fails closed."""


def fail(message: str) -> Phase117ExecutionError:
    return Phase117ExecutionError(message)


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
    # Exactly one wrapper payload read of each base member, after auth.
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
    spec = importlib.util.spec_from_file_location("phase117_tdcp_contract", CONTRACT_PATH)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load Phase117 contract: {CONTRACT_PATH}")
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
    # Called only after verify_authorization().  The contract metadata source
    # is the sealed Phase112 path manifest; no phone payload is opened here.
    source_routes = contract.sealed_phase112_routes()
    selected: dict[str, dict[str, Any]] = {}
    for route in ROUTES:
        source = source_routes[route]
        selected[route] = {"raw": {}, "base": {}}
        for name in RAW_NAMES:
            pin = source["raw_inputs"][name]
            path = safe_member(pin["path"], name, route)
            size = path.stat().st_size
            if size != pin["bytes"]:
                raise fail(f"sealed raw byte count changed: {route}/{name}: {size} != {pin['bytes']}")
            selected[route]["raw"][name] = {
                "path": pin["path"], "sha256": pin["sha256"],
                "sealed_bytes": pin["bytes"], "bytes": size,
                "exists_before_launch": True, "stat_read_by_wrapper": True,
                "payload_read_by_wrapper": False, "hash_read_by_wrapper": False,
                "content_copied_or_transformed": False,
            }
        base_pin = source["base_input"]
        base_path = safe_member(base_pin["path"], "base.obs", route)
        base_size = base_path.stat().st_size
        if base_size != base_pin["bytes"]:
            raise fail(f"sealed base byte count changed: {route}")
        actual = hash_base_once(base_path, f"base {route}")
        if actual != base_pin["sha256"]:
            raise fail(f"sealed base SHA changed: {route}")
        selected[route]["base"] = {
            "path": base_pin["path"], "sha256": actual,
            "sealed_sha256": base_pin["sha256"], "sealed_bytes": base_pin["bytes"],
            "bytes": base_size, "observed_dt_s": base_pin["observed_dt_s"],
            "moving_mean_samples": base_pin["moving_mean_samples"],
            "approx_position_xyz_m": base_pin["approx_position_xyz_m"],
            "coordinate_source": "raw RINEX header APPROX POSITION XYZ (native process)",
            "exists_before_launch": True, "stat_read_by_wrapper": True,
            "payload_read_by_wrapper": False, "hash_read_by_wrapper": True,
            "hash_verification_reads": 1, "native_process_reads_expected": 1,
            "content_copied_or_transformed": False,
        }
    return selected


def materialize_command(contract: Any, record: dict[str, Any], inputs: dict[str, Any]) -> list[str]:
    route = record["dataset_id"]
    source = contract.sealed_phase112_routes()[route]
    contract.validate_command(route, record.get("command"), source)
    replacements = {
        "__PHASE117_RAW_DEVICE_GNSS__": inputs["raw"]["device_gnss.csv"]["path"],
        "__PHASE117_RAW_DEVICE_IMU__": inputs["raw"]["device_imu.csv"]["path"],
        "__PHASE117_RAW_BROADCAST_NAV__": inputs["raw"]["brdc.nav"]["path"],
        "__PHASE117_RAW_BASE_RINEX__": inputs["base"]["path"],
        "__PHASE117_RAW_BASE_SHA256__": inputs["base"]["sha256"],
    }
    command = [replacements.get(token, token) for token in record["command"]]
    for token in command:
        if token in contract.FORBIDDEN_FLAGS:
            raise fail(f"forbidden flag after materialization: {route}/{token}")
    return command


def safe_environment() -> dict[str, str]:
    # Keep truth/MAT/coordinate/token paths out of the native child.
    return {
        "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
        "LANG": "C", "LC_ALL": "C", "TZ": "UTC",
        "LD_LIBRARY_PATH": "/home/sasaki/.local/lib",
    }


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def all_finite(value: Any) -> bool:
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return math.isfinite(float(value))
    if isinstance(value, list):
        return all(all_finite(item) for item in value)
    if isinstance(value, dict):
        return all(all_finite(item) for item in value.values())
    return True


def read_summary(path: Path) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    if not path.is_file():
        return None, {"path": relative(path), "present": False, "read_count": 0}
    payload = path.read_bytes()
    try:
        summary = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        return None, {"path": relative(path), "present": True, "bytes": len(payload), "error": str(exc), "read_count": 1}
    if not isinstance(summary, dict):
        return None, {"path": relative(path), "present": True, "bytes": len(payload), "error": "summary is not an object", "read_count": 1}
    return summary, {"path": relative(path), "present": True, "bytes": len(payload), "sha256": hashlib.sha256(payload).hexdigest(), "read_count": 1}


def seal_solution(path: Path, route: str, expected_rows: int) -> dict[str, Any]:
    if not path.is_file():
        return {"path": relative(path), "present": False, "opened": False, "published": False, "read_count": 0}
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


def expected_tdcp_counts(contract: Any) -> dict[str, int]:
    sealed = read_json(contract.PHASE116_RESULT, "sealed Phase116 result")
    if sha256_file(contract.PHASE116_RESULT, "sealed Phase116 result") != contract.PHASE116_RESULT_SHA256:
        raise fail("sealed Phase116 result hash changed")
    result: dict[str, int] = {}
    for route in ROUTES:
        telemetry = sealed.get("routes", {}).get(route, {}).get("telemetry", {})
        phase116 = telemetry.get("phase116", {}) if isinstance(telemetry, dict) else {}
        value = phase116.get("factors_built")
        if not isinstance(value, int) or value <= 0:
            raise fail(f"sealed Phase116 TDCP count missing: {route}")
        result[route] = value
    return result


def compact_signal(value: dict[str, Any]) -> dict[str, Any]:
    keep = (
        "signal", "frequency_band", "candidate_pairs", "accepted_pairs",
        "rejected_invalid_weight", "rejected_missing_previous", "rejected_code_phase_jump",
        "rejected_gap", "rejected_clock_discontinuity", "rejected_loss_of_lock",
        "rejected_nonfinite", "retained_carrier_rows", "initial_residual_count",
        "final_residual_count", "connected_epoch_count", "connected_key_count",
        "sigma_m", "initial_robust_cost", "final_robust_cost",
        "initial_unwhitened_cost", "final_unwhitened_cost",
        "initial_whitened_cost", "final_whitened_cost",
    )
    return {key: value[key] for key in keep if key in value}


def compact_phase116(value: Any) -> dict[str, Any] | None:
    if not isinstance(value, dict):
        return None
    result = {key: value.get(key) for key in (
        "enabled", "read_only", "ordinary_tdcp_only", "standalone_carrier_phase_factors",
        "double_difference_factors", "ambiguity_states", "pdc_state_bridge",
        "factors_built", "factors_inserted", "factors_inserted_exact",
        "sigma_m", "graph_initial_cost", "graph_final_cost",
    ) if key in value}
    signals = value.get("signals")
    if isinstance(signals, dict):
        result["signals"] = {name: compact_signal(item) for name, item in signals.items() if isinstance(item, dict)}
    return result


def compact_telemetry(summary: dict[str, Any]) -> dict[str, Any]:
    # Do not copy output positions or connected-key arrays into the sealed
    # result.  All fields below are structural/non-solution telemetry.
    return {
        "phase116": compact_phase116(summary.get("phase116_carrier_tdcp_incidence")),
        "epochs": summary.get("epochs"),
        "tdcp_contract": summary.get("tdcp_contract"),
        "gnss_first": summary.get("gnss_first"),
        "graph": summary.get("graph"),
        "clock": summary.get("native_source_clock_c0d_factor"),
        "solver": {
            "requested": summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_enabled"),
            "selected": summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected"),
            "linear_solver_type": summary.get("selected_linear_solver_type"),
            "branch": summary.get("selected_solver_branch"),
            "elimination": summary.get("selected_elimination_function"),
        },
        "base": summary.get("native_base_pseudorange_compensation"),
        "base_miss_mask": summary.get("native_base_pseudorange_source_miss_mask"),
        "offset": summary.get("upstream_position_offset"),
        "raw_utc": summary.get("raw_utc_key_contract"),
        "status": summary.get("status"),
        "fallback": summary.get("fallback"),
        "truth_used": summary.get("truth_used"),
        "production_default_changed": summary.get("production_default_changed"),
    }


def validate_route(metadata: dict[str, Any], contract: Any, manifest: dict[str, Any], expected_counts: dict[str, int]) -> dict[str, Any]:
    route = metadata["dataset_id"]
    record = next(item for item in manifest["routes"] if item["dataset_id"] == route)
    expected = int(record["expected_problem_epochs"])
    domain = int(record["domain_rows"])
    summary_path = ROOT / metadata["planned_output"]["summary"]
    solution_path = ROOT / metadata["planned_output"]["withheld_solution_output"]
    summary, summary_meta = read_summary(summary_path)
    solution_meta = seal_solution(solution_path, route, domain)
    report: dict[str, Any] = {
        "dataset_id": route, "return_code": metadata.get("return_code"),
        "expected": {"domain_rows": domain, "problem_epochs": expected, "output_epochs": expected},
        "summary": summary_meta, "solution_hash_seal": solution_meta,
        "raw_inputs": metadata.get("raw_inputs"), "base_input": metadata.get("base_input"),
        "truth_used": False, "mat_used": False, "phone_coordinates_used": False,
        "precomputed_coordinates_used": False, "pdc_used": False,
        "kaggle_or_token_accessed": False, "solution_output_published": False,
        "read_accounting": {
            "runner_raw_payload_reads": 0, "runner_raw_hash_reads": 0,
            "runner_base_hash_reads": 1, "native_raw_phone_gnss_reads": 1,
            "native_raw_phone_imu_reads": 1, "native_broadcast_navigation_reads": 1,
            "native_base_rinex_reads": 1, "truth_reads": 0,
            "mat_reads_or_generated": 0, "phone_coordinate_reads": 0,
            "precomputed_coordinate_reads": 0, "pdc_reads": 0,
            "accuracy_calculations": 0, "solution_output_hash_reads": 1 if solution_meta.get("present") else 0,
            "raw_content_copied_or_transformed": False,
        },
    }
    names = (
        "native_process_completed", "summary_present", "official_sigma_all_finite_source_derived",
        "valid_metadata_factor_count_unchanged", "missing_metadata_fail_closed_no_legacy_fallback",
        "tdcp_residual_keys_reject_predicate_unchanged", "gnss_first_progress_strict_cost_decrease",
        "gnss_first_full_finite_c7_d_exact_handoff", "main_qr_progress_strict_cost_decrease",
        "base_correction_exactly_once", "offset_exactly_once_final_boundary",
        "finite_expected_output_coverage", "no_solver_fallback_or_solution_publication",
    )
    if summary is None:
        report["telemetry"] = None
        report["gates"] = {name: False for name in names}
        report["failure_reasons"] = list(names)
        return report

    phase116 = summary.get("phase116_carrier_tdcp_incidence")
    tdcp = summary.get("tdcp_contract")
    gnss = summary.get("gnss_first")
    graph = summary.get("graph")
    clock = summary.get("native_source_clock_c0d_factor")
    base = summary.get("native_base_pseudorange_compensation")
    miss = summary.get("native_base_pseudorange_source_miss_mask")
    offset = summary.get("upstream_position_offset")
    raw_utc = summary.get("raw_utc_key_contract")
    phase117_enabled = (
        summary.get("native_phase117_tdcp_snr_type_sigma") is True and
        isinstance(tdcp, dict) and tdcp.get("official_snr_type_sigma_enabled") is True and
        tdcp.get("official_snr_percentile") == 85 and
        tdcp.get("official_carrier_sigma_units") == "source L cycles * retained wavelength_m = native metres" and
        tdcp.get("official_invalid_weight_fail_closed") is True
    )
    factors = phase116.get("factors_built") if isinstance(phase116, dict) else None
    inserted = phase116.get("factors_inserted") if isinstance(phase116, dict) else None
    factor_structure = (
        isinstance(phase116, dict) and phase116.get("enabled") is True and phase116.get("read_only") is True and
        phase116.get("ordinary_tdcp_only") is True and phase116.get("standalone_carrier_phase_factors") is False and
        phase116.get("double_difference_factors") is False and phase116.get("ambiguity_states") is False and
        phase116.get("pdc_state_bridge") is False and isinstance(factors, int) and factors > 0 and
        inserted == factors and phase116.get("factors_inserted_exact") is True and
        isinstance(tdcp, dict) and tdcp.get("factors_built") == factors and tdcp.get("factors_inserted") == inserted
    )
    signals = phase116.get("signals") if isinstance(phase116, dict) else None
    accepted_total = 0
    signal_valid = isinstance(signals, dict) and bool(signals)
    invalid_weight_total = 0
    if signal_valid:
        for value in signals.values():
            if not isinstance(value, dict):
                signal_valid = False
                continue
            accepted = value.get("accepted_pairs")
            candidate_pairs = value.get("candidate_pairs")
            sigma = value.get("sigma_m")
            if not isinstance(accepted, int) or not isinstance(candidate_pairs, int) or not (0 <= accepted <= candidate_pairs):
                signal_valid = False
            accepted_total += accepted if isinstance(accepted, int) else 0
            invalid = value.get("rejected_invalid_weight", 0)
            if not isinstance(invalid, int) or invalid < 0:
                signal_valid = False
            invalid_weight_total += invalid if isinstance(invalid, int) else 0
            if not finite(sigma) or float(sigma) <= 0.0:
                signal_valid = False
            for key in ("initial_residual_count", "final_residual_count", "connected_key_count"):
                if not isinstance(value.get(key), int) or value[key] < 0:
                    signal_valid = False
            for key in ("initial_robust_cost", "final_robust_cost", "initial_unwhitened_cost", "final_unwhitened_cost", "initial_whitened_cost", "final_whitened_cost"):
                if key in value and not finite(value[key]):
                    signal_valid = False
    official_sigma_ok = phase117_enabled and factor_structure and signal_valid and invalid_weight_total == 0 and accepted_total == factors
    valid_count_unchanged = factor_structure and factors == expected_counts[route] and inserted == expected_counts[route]
    fail_closed = phase117_enabled and tdcp.get("rejected_invalid_weight") == invalid_weight_total and metadata.get("runner_read_raw_payloads") is False
    reject_unchanged = factor_structure and tdcp.get("pair_key") == "(satellite,signal)" and tdcp.get("code_phase_jump_threshold_m") == 10 and tdcp.get("max_gap_s") == 2 and tdcp.get("standalone_carrier_ambiguity_factors") is False
    gnss_progress = (
        isinstance(gnss, dict) and gnss.get("attempted") is True and gnss.get("converged") is True and
        isinstance(gnss.get("iterations"), int) and gnss.get("iterations") >= 1 and
        finite(gnss.get("initial_cost")) and finite(gnss.get("final_cost")) and
        gnss.get("final_cost") < gnss.get("initial_cost")
    )
    handoff = (
        isinstance(gnss, dict) and gnss.get("optimized_c_vector_parity_enabled") is True and
        gnss.get("optimized_c_export_valid") is True and gnss.get("optimized_c_dimension") == 7 and
        gnss.get("optimized_c_epoch_count") == expected and gnss.get("optimized_c_finite_component_count") == expected * 7 and
        gnss.get("optimized_c_nonfinite_component_count") == 0 and gnss.get("optimized_d_export_valid") is True and
        gnss.get("optimized_d_epoch_count") == expected and gnss.get("optimized_d_finite_count") == expected and
        gnss.get("optimized_d_nonfinite_count") == 0 and gnss.get("epoch_identity_alignment_valid") is True
    )
    main_progress = (
        isinstance(graph, dict) and graph.get("converged") is True and isinstance(graph.get("iterations"), int) and
        graph.get("iterations") >= 1 and finite(graph.get("initial_cost")) and finite(graph.get("final_cost")) and
        graph.get("final_cost") < graph.get("initial_cost") and isinstance(clock, dict) and
        clock.get("active_solve_attempted") is True and isinstance(clock.get("accepted_outer_iterations"), int) and
        clock.get("accepted_outer_iterations") > 0 and finite(clock.get("active_solve_initial_cost")) and
        finite(clock.get("active_solve_final_cost")) and clock.get("active_solve_final_cost") < clock.get("active_solve_initial_cost")
    )
    qr_ok = (
        summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_enabled") is True and
        summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected") is True and
        summary.get("selected_linear_solver_type") == "MULTIFRONTAL_QR" and
        summary.get("selected_elimination_function") == "EliminateQR"
    )
    base_ok = (
        isinstance(base, dict) and base.get("enabled") is True and base.get("applied") is True and
        base.get("correction_application_pass_count") == 1 and base.get("correction_applied_exactly_once") is True and
        base.get("duplicate_correction_rejected") is False and isinstance(miss, dict) and
        miss.get("correction_application_pass_count") == 1 and miss.get("correction_applied_exactly_once") is True and
        miss.get("duplicate_correction_rejected") is False
    )
    offset_ok = (
        isinstance(offset, dict) and offset.get("enabled") is True and offset.get("applied") is True and
        offset.get("phone") == "pixel5" and offset.get("corrected_epochs") == expected and
        finite(offset.get("max_offset_enu_m")) and solution_meta.get("present") is True and
        solution_meta.get("row_count_matches_domain") is True
    )
    output_ok = (
        isinstance(summary.get("epochs"), dict) and summary["epochs"].get("problem") == expected and
        summary["epochs"].get("output") == expected and isinstance(raw_utc, dict) and
        raw_utc.get("raw_epoch_keys") == expected and raw_utc.get("unresolved_epochs") == 0 and
        all_finite(summary.get("output_contract", {}))
    )
    no_publication = (
        metadata.get("runner_read_raw_payloads") is False and metadata.get("runner_read_raw_hashes") is False and
        metadata.get("runner_read_truth_mat_phone_coordinate_precomputed_pdc_kaggle") is False and
        metadata.get("solution_output_published") is False and summary.get("truth_used") is False and
        summary.get("production_default_changed") is False
    )
    process_ok = metadata.get("return_code") == 0 and not metadata.get("launch_error") and not metadata.get("interrupted") and not metadata.get("timed_out")
    report["telemetry"] = compact_telemetry(summary)
    report["gates"] = {
        "native_process_completed": process_ok,
        "summary_present": summary_meta.get("present") is True,
        "official_sigma_all_finite_source_derived": official_sigma_ok,
        "valid_metadata_factor_count_unchanged": valid_count_unchanged,
        "missing_metadata_fail_closed_no_legacy_fallback": fail_closed,
        "tdcp_residual_keys_reject_predicate_unchanged": reject_unchanged,
        "gnss_first_progress_strict_cost_decrease": gnss_progress,
        "gnss_first_full_finite_c7_d_exact_handoff": handoff,
        "main_qr_progress_strict_cost_decrease": qr_ok and main_progress,
        "base_correction_exactly_once": base_ok,
        "offset_exactly_once_final_boundary": offset_ok,
        "finite_expected_output_coverage": output_ok,
        "no_solver_fallback_or_solution_publication": no_publication,
    }
    report["failure_reasons"] = [name for name, passed in report["gates"].items() if passed is not True]
    return report


def execute_matrix(contract: Any, manifest: dict[str, Any]) -> list[dict[str, Any]]:
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite Phase117 output root: {OUTPUT_ROOT}")
    records = manifest.get("routes")
    if not isinstance(records, list) or [item.get("dataset_id") for item in records] != list(ROUTES):
        raise fail("Phase117 route order/count changed")
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
                completed = subprocess.run(command, cwd=ROOT, env=safe_environment(), stdout=stdout, stderr=stderr, check=False, timeout=1800)
                return_code = completed.returncode
            except subprocess.TimeoutExpired as exc:
                timed_out = True
                stderr.write(f"\nPhase117 native launch timed out: {exc}\n".encode())
            except KeyboardInterrupt:
                interrupted = True
                stderr.write(b"\nPhase117 wrapper interrupted; partial output preserved.\n")
            except OSError as exc:
                launch_error = str(exc)
                stderr.write(f"\nPhase117 native launch failed: {exc}\n".encode())
        item = {
            "schema_version": "smartphone-r5-phase117-tdcp-weighting-route-execution.v1",
            "dataset_id": route, "run_number": 1, "command": command,
            "raw_inputs": inputs[route]["raw"], "base_input": inputs[route]["base"],
            "planned_output": {"summary": relative(summary_path), "withheld_solution_output": relative(solution_path)},
            "summary_present_after_launch": summary_path.is_file(),
            "withheld_solution_output_present_after_launch": solution_path.is_file(),
            "stdout": relative(stdout_path), "stderr": relative(stderr_path),
            "started_unix_s": started, "ended_unix_s": time.time(),
            "return_code": return_code, "interrupted": interrupted, "timed_out": timed_out,
            "launch_error": launch_error, "runner_read_raw_payloads": False,
            "runner_read_raw_hashes": False,
            "runner_read_truth_mat_phone_coordinate_precomputed_pdc_kaggle": False,
            "runner_read_base_payload_for_hash": True, "raw_content_copied_or_transformed": False,
            "solution_output_opened": False, "solution_output_published": False,
            "accuracy_scored": False,
        }
        atomic_json(route_dir / "run_metadata.json", item)
        metadata.append(item)
        atomic_json(OUTPUT_ROOT / "partial_execution_metadata.json", {"routes_completed": metadata})
    return metadata


def build_result(metadata: list[dict[str, Any]], contract: Any, manifest: dict[str, Any], authorization: dict[str, Any]) -> dict[str, Any]:
    expected_counts = expected_tdcp_counts(contract)
    reports = [validate_route(item, contract, manifest, expected_counts) for item in metadata]
    by_route = {item["dataset_id"]: item for item in reports}
    gate_names = (
        "native_process_completed summary_present official_sigma_all_finite_source_derived valid_metadata_factor_count_unchanged "
        "missing_metadata_fail_closed_no_legacy_fallback tdcp_residual_keys_reject_predicate_unchanged "
        "gnss_first_progress_strict_cost_decrease gnss_first_full_finite_c7_d_exact_handoff "
        "main_qr_progress_strict_cost_decrease base_correction_exactly_once offset_exactly_once_final_boundary "
        "finite_expected_output_coverage no_solver_fallback_or_solution_publication"
    ).split()
    all_passed = len(reports) == 2 and [item["dataset_id"] for item in reports] == list(ROUTES) and all(all(report.get("gates", {}).get(name) is True for name in gate_names) for report in reports)
    failed = {route: [name for name in gate_names if report.get("gates", {}).get(name) is not True] for route, report in by_route.items() if any(report.get("gates", {}).get(name) is not True for name in gate_names)}
    return {
        "schema_version": "smartphone-r5-phase117-tdcp-weighting-structural-result.v1",
        "phase": 117, "execution_label": "Luna Max",
        "status": "go-phase117-tdcp-weighting-structural" if all_passed else "no-go-phase117-tdcp-weighting-structural",
        "decision": "Structural gates passed; truth/accuracy and solution release remain separately unauthorized." if all_passed else "Structural gate failed closed; preserve partial artifacts and do not retry, fallback, truth-score, or publish.",
        "candidate": {
            "id": contract.CANDIDATE_ID, "candidate_count": 1,
            "selector": contract.PHASE117_SELECTOR,
            "selectors": [contract.PHASE93_SELECTOR, contract.VECTOR_SELECTOR, contract.QR_SELECTOR, contract.OFFSET_SELECTOR, contract.PHASE116_SELECTOR, contract.PHASE117_SELECTOR],
            "base_selectors": list(contract.BASE_SELECTORS), "official_snr_percentile": 85,
            "official_snr_denominator": 20, "official_l_sn_ratio": 0.0025,
            "sigma_units": "source L cycles * retained wavelength_m = native metres",
            "fixed_legacy_tdcp_sigma_m": 0.03, "default_off": True,
            "solution_output_published": False,
        },
        "freeze": {"path": relative(contract.FREEZE), "sha256": contract.FREEZE_SHA256, "commit": contract.FREEZE_COMMIT},
        "manifest": {"path": relative(MANIFEST), "sha256": sha256_file(MANIFEST, "Phase117 manifest")},
        "authorization": {"path": relative(AUTHORIZATION), "sha256": sha256_file(AUTHORIZATION, "Phase117 authorization"), "status": authorization.get("status")},
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
        "# Phase117 official TDCP SNR/type weighting structural result", "",
        f"- status: `{result['status']}`",
        "- recipe: Phase112 raw/base + Phase101 C7/D + Phase99 QR + Pixel5 offset + Phase116 ordinary-TDCP telemetry + Phase117 official sigma",
        "- official model: p85 SNR, denominator 20, L ratio 1/400, source cycles multiplied by retained wavelength to native metres",
        "- truth/MAT/phone-coordinate/precomputed/PDC/Kaggle/accuracy lanes: not read",
        "- solution CSV: opaque hash/header/row seal only; coordinates omitted and unpublished", "",
        "| Route | Return | Gates |", "|---|---:|---:|",
    ]
    for route, report in result["routes"].items():
        gates = report.get("gates", {})
        lines.append(f"| `{route}` | `{report.get('return_code')}` | `{sum(value is True for value in gates.values())}/{len(gates)}` |")
    lines.extend(["", "A failed gate remains sealed fail-closed; no retry, fallback, truth score, or solution release is permitted.", ""])
    return "\n".join(lines)


def main() -> int:
    contract = load_contract()
    manifest = contract.verify_manifest()
    authorization = contract.verify_authorization(manifest)
    metadata = execute_matrix(contract, manifest)
    result = build_result(metadata, contract, manifest, authorization)
    atomic_json(RESULT_JSON, result)
    atomic_text(RESULT_MD, render_markdown(result))
    return 0 if result["gates"]["all_structural_gates_passed"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (Phase117ExecutionError, OSError) as exc:
        print(f"phase117 raw execution: fail-closed: {exc}", file=sys.stderr)
        raise SystemExit(2)
