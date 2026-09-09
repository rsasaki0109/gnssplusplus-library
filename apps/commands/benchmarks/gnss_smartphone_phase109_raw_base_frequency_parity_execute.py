#!/usr/bin/env python3
"""Execute and seal the authorized Phase109 raw-base structural matrix.

The wrapper requires the independently committed Phase109 authorization.  It
then runs exactly one native candidate invocation for MTV-A followed by LAX-T.
Phone raw GNSS/IMU/navigation files are passed through by path and are only
metadata-statted by this wrapper; the native child reads them.  Each sealed
base RINEX member is hashed once by this wrapper and read once by the native
child.  The native solution CSV is placed under a withheld path and is never
opened, hashed, scored, published, or committed.
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
PRE_RAW = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase109_raw_base_frequency_parity.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase109_raw_base_frequency_parity_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase109_raw_base_frequency_parity_authorization_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase109-raw-base-frequency-parity-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase109_raw_base_frequency_parity_result_v1.json"
RESULT_MD = ROOT / "docs/use_cases/records/smartphone_r5_phase109_raw_base_frequency_parity_result_v1.md"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")


class Phase109ExecutionError(ValueError):
    """Raised when the independently authorized execution fails closed."""


def fail(message: str) -> Phase109ExecutionError:
    return Phase109ExecutionError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


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


def sha256_file(path: Path, label: str) -> str:
    if path.name in RAW_NAMES or path.name == "base.obs":
        raise fail(f"input payload hash forbidden here: {label}")
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def hash_base_member(path: Path, label: str) -> str:
    """Authorized one-time base-member hash read after the auth boundary."""

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


def load_contract_module() -> Any:
    spec = importlib.util.spec_from_file_location("phase109_raw_base_contract", PRE_RAW)
    if spec is None or spec.loader is None:
        raise fail(f"failed to load Phase109 evaluator: {PRE_RAW}")
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


def materialize_inputs(module: Any) -> dict[str, dict[str, Any]]:
    """Stat phone inputs and hash each authorized base member exactly once."""

    raw = module.phase95_raw_metadata()
    base = module.phase65_base_metadata()
    selected: dict[str, dict[str, Any]] = {}
    for route in ROUTES:
        selected[route] = {"raw": {}, "base": {}}
        for name in RAW_NAMES:
            pin = raw[route][name]
            path = safe_member_path(pin["path"], name, route)
            size = path.stat().st_size
            if pin.get("bytes") is not None and size != pin["bytes"]:
                raise fail(f"sealed raw byte count changed: {route}/{name}: {size} != {pin['bytes']}")
            # This is metadata-only: the wrapper never opens or hashes phone
            # GNSS, IMU, or navigation payloads.
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


def materialize_command(module: Any, record: dict[str, Any], inputs: dict[str, Any]) -> list[str]:
    route = record.get("dataset_id")
    # Validate the placeholder form before replacing any input path.
    module.validate_command(route, record.get("command"), record, {
        name: {"path": inputs["raw"][name]["path"], "sha256": inputs["raw"][name]["sha256"]}
        for name in RAW_NAMES
    }, {
        "path": inputs["base"]["path"], "sha256": inputs["base"]["sha256"],
        "bytes": inputs["base"]["bytes"],
        "observed_dt_s": inputs["base"]["observed_dt_s"],
        "moving_mean_samples": inputs["base"]["moving_mean_samples"],
        "approx_position_xyz_m": inputs["base"]["approx_position_xyz_m"],
    })
    command = list(record["command"])
    replacements = {
        "__PHASE109_RAW_DEVICE_GNSS__": inputs["raw"]["device_gnss.csv"]["path"],
        "__PHASE109_RAW_DEVICE_IMU__": inputs["raw"]["device_imu.csv"]["path"],
        "__PHASE109_RAW_BROADCAST_NAV__": inputs["raw"]["brdc.nav"]["path"],
        "__PHASE109_RAW_BASE_RINEX__": inputs["base"]["path"],
        "__PHASE109_RAW_BASE_SHA256__": inputs["base"]["sha256"],
    }
    command = [replacements.get(token, token) for token in command]
    for token in command:
        if token in module.FORBIDDEN_FLAGS:
            raise fail(f"forbidden command flag after materialization: {route}/{token}")
    return command


def safe_environment() -> dict[str, str]:
    # Do not pass arbitrary parent paths or credentials into the native child.
    return {
        "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
        "LANG": "C",
        "LC_ALL": "C",
        "TZ": "UTC",
        "LD_LIBRARY_PATH": "/home/sasaki/.local/lib",
    }


def execute_matrix(module: Any, manifest: dict[str, Any]) -> list[dict[str, Any]]:
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite Phase109 output root: {OUTPUT_ROOT}")
    records = manifest.get("routes")
    if not isinstance(records, list) or [item.get("dataset_id") for item in records] != list(ROUTES):
        raise fail("Phase109 route order/count changed")
    inputs = materialize_inputs(module)
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    environment = safe_environment()
    metadata: list[dict[str, Any]] = []
    for record in records:
        route = record["dataset_id"]
        route_dir = OUTPUT_ROOT / route.replace("/", "__")
        route_dir.mkdir(parents=True, exist_ok=False)
        command = materialize_command(module, record, inputs[route])
        summary_path = ROOT / record["planned_output"]["summary"] if "planned_output" in record else route_dir / "summary.json"
        withheld_path = ROOT / record["planned_output"]["withheld_solution_output"] if "planned_output" in record else route_dir / "withheld_solution_output.csv"
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
                stderr.write(f"\nPhase109 native launch timed out: {exc}\n".encode())
            except KeyboardInterrupt:
                interrupted = True
                stderr.write(b"\nPhase109 wrapper interrupted; partial output preserved.\n")
            except OSError as exc:
                launch_error = str(exc)
                stderr.write(f"\nPhase109 native launch failed: {exc}\n".encode())
        metadata_record = {
            "schema_version": "smartphone-r5-phase109-raw-base-frequency-route-execution.v1",
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
        atomic_json(route_dir / "run_metadata.json", metadata_record)
        metadata.append(metadata_record)
        atomic_json(OUTPUT_ROOT / "partial_execution_metadata.json", {"routes_completed": metadata})
        if interrupted:
            break
    return metadata


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def nonnegative_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def close_float(actual: Any, expected: Any, tolerance: float = 1e-6) -> bool:
    return finite(actual) and finite(expected) and abs(float(actual) - float(expected)) <= tolerance


def signal_taxonomy_gate(base: Any, miss: Any) -> tuple[bool, dict[str, Any]]:
    if not isinstance(base, dict) or not isinstance(miss, dict):
        return False, {"reason": "base or source miss-mask telemetry missing"}
    required_global = (
        "original_adopted_pseudorange_rows", "retained_finite_pc_pseudorange_rows",
        "corrected_rows", "dropped_missing_exact_stream_rows",
        "dropped_out_of_domain_rows", "dropped_nonfinite_correction_rows",
    )
    if not all(nonnegative_int(miss.get(key)) for key in required_global):
        return False, {"reason": "global signal taxonomy denominator missing"}
    global_counts = {key: miss[key] for key in required_global}
    taxonomy = miss.get("signal_taxonomy_by_signal")
    if not isinstance(taxonomy, dict) or not taxonomy:
        taxonomy = base.get("source_miss_taxonomy_by_signal")
    if not isinstance(taxonomy, dict) or not taxonomy:
        return False, {"reason": "per-signal taxonomy missing"}
    keys = (
        "original_adopted_rows", "retained_finite_pc_rows", "corrected_rows",
        "matched_exact_stream_rows", "finite_correction_rows_among_matched",
        "dropped_missing_exact_stream_rows", "dropped_out_of_domain_rows",
        "dropped_nonfinite_correction_rows",
    )
    sums = {key: 0 for key in keys}
    entries: dict[str, Any] = {}
    for signal, entry in taxonomy.items():
        if not isinstance(entry, dict) or not isinstance(entry.get("frequency_band"), str) or not entry["frequency_band"]:
            return False, {"reason": f"missing frequency band: {signal}"}
        if not all(nonnegative_int(entry.get(key)) for key in keys):
            return False, {"reason": f"non-integral signal counts: {signal}"}
        if entry["corrected_rows"] != entry["retained_finite_pc_rows"]:
            return False, {"reason": f"corrected/retained mismatch: {signal}"}
        if entry["matched_exact_stream_rows"] < entry["finite_correction_rows_among_matched"]:
            return False, {"reason": f"matched/finite mismatch: {signal}"}
        if entry["original_adopted_rows"] != (
            entry["retained_finite_pc_rows"]
            + entry["dropped_missing_exact_stream_rows"]
            + entry["dropped_out_of_domain_rows"]
            + entry["dropped_nonfinite_correction_rows"]
        ):
            return False, {"reason": f"per-signal conservation mismatch: {signal}"}
        if entry.get("factor_count_consistent") is not True:
            return False, {"reason": f"per-signal factor count inconsistent: {signal}"}
        entries[signal] = entry
        for key in keys:
            sums[key] += entry[key]
    expected_sums = {
        "original_adopted_rows": global_counts["original_adopted_pseudorange_rows"],
        "retained_finite_pc_rows": global_counts["retained_finite_pc_pseudorange_rows"],
        "corrected_rows": global_counts["corrected_rows"],
        "dropped_missing_exact_stream_rows": global_counts["dropped_missing_exact_stream_rows"],
        "dropped_out_of_domain_rows": global_counts["dropped_out_of_domain_rows"],
        "dropped_nonfinite_correction_rows": global_counts["dropped_nonfinite_correction_rows"],
    }
    for key, expected in expected_sums.items():
        if sums[key] != expected:
            return False, {"reason": f"global/per-signal conservation mismatch: {key}", "sums": sums}
    if global_counts["corrected_rows"] != global_counts["retained_finite_pc_rows"]:
        return False, {"reason": "global corrected/retained mismatch"}
    global_conserved = global_counts["original_adopted_pseudorange_rows"] == (
        global_counts["retained_finite_pc_pseudorange_rows"]
        + global_counts["dropped_missing_exact_stream_rows"]
        + global_counts["dropped_out_of_domain_rows"]
        + global_counts["dropped_nonfinite_correction_rows"]
    )
    if not global_conserved or miss.get("pseudorange_factor_count_consistent") is not True or miss.get("signal_count_consistent") is not True:
        return False, {"reason": "global conservation or consistency flag failed"}
    return True, {
        "global": global_counts,
        "per_signal": entries,
        "conservation": True,
    }


def validate_route(metadata: dict[str, Any], module: Any, manifest: dict[str, Any]) -> dict[str, Any]:
    route = metadata["dataset_id"]
    record = next(item for item in manifest["routes"] if item["dataset_id"] == route)
    expected = int(record["expected_problem_epochs"])
    summary_path = ROOT / metadata["planned_output"]["summary"]
    withheld_path = ROOT / metadata["planned_output"]["withheld_solution_output"]
    report: dict[str, Any] = {
        "dataset_id": route,
        "run_number": metadata.get("run_number"),
        "return_code": metadata.get("return_code"),
        "expected": {"domain_rows": expected - 1, "problem_epochs": expected, "output_epochs": expected},
        "summary": {"path": relative(summary_path), "present": summary_path.is_file()},
        "withheld_solution": {
            "path": relative(withheld_path),
            "present": withheld_path.is_file(),
            "opened": False,
            "published": False,
        },
        "truth_used": False,
        "mat_used": False,
        "phone_coordinates_used": False,
        "precomputed_coordinates_used": False,
        "pdc_used": False,
        "kaggle_or_token_accessed": False,
        "raw_inputs": metadata.get("raw_inputs"),
        "base_input": metadata.get("base_input"),
        "read_accounting": {
            "runner_raw_payload_reads": 0,
            "runner_raw_hash_reads": 0,
            "runner_base_hash_reads": 1,
            "native_raw_phone_gnss_reads": 1,
            "native_raw_phone_imu_reads": 1,
            "native_broadcast_navigation_reads": 1,
            "native_base_rinex_reads": 1,
            "truth_reads": 0,
            "mat_reads_or_generated": 0,
            "phone_coordinate_reads": 0,
            "precomputed_coordinate_reads": 0,
            "pdc_reads": 0,
            "accuracy_calculations": 0,
            "solution_output_opened": 0,
            "raw_content_copied_or_transformed": False,
        },
    }
    if not summary_path.is_file():
        report["summary"]["error"] = "native summary absent; route is fail-closed"
        report["telemetry"] = None
        report["gates"] = {name: False for name in (
            "native_process_completed", "summary_present", "base_path_hash_bytes_header_match",
            "base_preserve_reader_active", "base_factors_active_exactly_once",
            "base_signal_band_taxonomy_conserved", "gnss_first_progress_strict_cost_decrease",
            "gnss_first_full_finite_c7_d_exact_handoff", "main_qr_progress_strict_cost_decrease",
            "finite_earth_valid_expected_output_coverage", "no_fallback_or_solution_publication",
        )}
        return report
    try:
        summary = read_json(summary_path, f"Phase109 native summary {route}")
        report["summary"]["sha256"] = sha256_file(summary_path, f"Phase109 summary {route}")
        report["summary"]["bytes"] = summary_path.stat().st_size
    except (Phase109ExecutionError, OSError) as exc:
        report["summary"]["error"] = str(exc)
        report["telemetry"] = None
        report["gates"] = {"summary_present": False}
        return report

    base = summary.get("native_base_pseudorange_compensation")
    miss = summary.get("native_base_pseudorange_source_miss_mask")
    epochs = summary.get("epochs")
    gnss = summary.get("gnss_first")
    clock = summary.get("native_source_clock_c0d_factor")
    graph = summary.get("graph")
    output_contract = summary.get("output_contract")
    base_pin = metadata["base_input"]

    base_path_hash = (
        isinstance(base, dict)
        and base.get("enabled") is True
        and base.get("built") is True
        and base.get("applied") is True
        and base.get("preserve_additional_frequency_bands") is True
        and base.get("base_rinex") == base_pin.get("path")
        and base.get("base_rinex_sha256") == base_pin.get("sha256")
        and base.get("base_member_sha256") == base_pin.get("sha256")
        and base.get("base_rinex_bytes") == base_pin.get("bytes")
        and base.get("base_rinex_read_count") == 1
        and base.get("base_coordinate_provenance") == "RINEX header APPROX POSITION XYZ"
        and isinstance(base.get("base_coordinate_xyz_m"), list)
        and len(base["base_coordinate_xyz_m"]) == 3
        and all(close_float(a, b) for a, b in zip(base["base_coordinate_xyz_m"], base_pin["approx_position_xyz_m"]))
        and close_float(base.get("observed_interval_s"), base_pin.get("observed_dt_s"))
        and base.get("moving_mean_samples") == base_pin.get("moving_mean_samples")
        and base.get("same_satellite_signal_only") is True
    )
    taxonomy_ok, taxonomy = signal_taxonomy_gate(base, miss)
    exact_once = (
        isinstance(base, dict)
        and base.get("source_model_build_count") == 1
        and base.get("correction_application_pass_count") == 1
        and base.get("correction_applied_exactly_once") is True
        and base.get("duplicate_correction_rejected") is False
        and isinstance(miss, dict)
        and miss.get("correction_application_pass_count") == 1
        and miss.get("correction_applied_exactly_once") is True
        and miss.get("duplicate_correction_rejected") is False
        and isinstance(epochs, dict)
        and epochs.get("double_difference_pseudorange_factors") == 0
        and epochs.get("double_difference_carrier_factors") == 0
        and miss.get("pseudorange_factors_inserted") == epochs.get("pseudorange_factors")
        and nonnegative_int(miss.get("pseudorange_factors_inserted"))
        and miss.get("pseudorange_factors_inserted") > 0
    )
    selected_maps = (
        isinstance(base, dict)
        and nonnegative_int(base.get("selected_band_observation_rows"))
        and base.get("selected_band_observation_rows") > 0
        and nonnegative_int(base.get("selected_band_streams"))
        and base.get("selected_band_streams") > 0
        and isinstance(base.get("selected_band_observation_rows_by_signal"), dict)
        and isinstance(base.get("selected_band_streams_by_signal"), dict)
        and sum(base["selected_band_observation_rows_by_signal"].values()) == base.get("selected_band_observation_rows")
        and sum(base["selected_band_streams_by_signal"].values()) == base.get("selected_band_streams")
    )
    gnss_progress = (
        isinstance(gnss, dict)
        and gnss.get("attempted") is True
        and gnss.get("converged") is True
        and gnss.get("epochs") == expected
        and nonnegative_int(gnss.get("iterations")) and gnss.get("iterations") >= 1
        and nonnegative_int(gnss.get("c0d_factor_count")) and gnss.get("c0d_factor_count") > 0
        and nonnegative_int(gnss.get("c0d_accepted_outer_iterations")) and gnss.get("c0d_accepted_outer_iterations") > 0
        and gnss.get("c0d_active_solve_finite_costs") is True
        and finite(gnss.get("initial_cost")) and finite(gnss.get("final_cost"))
        and gnss.get("final_cost") < gnss.get("initial_cost")
    )
    handoff = (
        isinstance(gnss, dict)
        and gnss.get("optimized_c_vector_parity_enabled") is True
        and gnss.get("optimized_c_export_valid") is True
        and gnss.get("optimized_c_dimension") == 7
        and gnss.get("optimized_c_epoch_count") == expected
        and gnss.get("optimized_c_finite_component_count") == expected * 7
        and gnss.get("optimized_c_nonfinite_component_count") == 0
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
        and nonnegative_int(graph.get("iterations")) and graph.get("iterations") >= 1
        and finite(graph.get("initial_cost")) and finite(graph.get("final_cost"))
        and graph.get("final_cost") < graph.get("initial_cost")
        and isinstance(clock, dict)
        and clock.get("clock_c0d_enabled") is True
        and clock.get("meter_state_parity_enabled") is True
        and clock.get("epoch_vector_parity_enabled") is True
        and clock.get("epoch_vector_dimension") == 7
        and clock.get("global_isb_state_count") == 0
        and clock.get("active_solve_attempted") is True
        and nonnegative_int(clock.get("accepted_outer_iterations"))
        and clock.get("accepted_outer_iterations") > 0
        and clock.get("active_solve_finite_costs") is True
        and finite(clock.get("active_solve_initial_cost"))
        and finite(clock.get("active_solve_final_cost"))
        and clock.get("active_solve_final_cost") < clock.get("active_solve_initial_cost")
        and clock.get("internal_clock_state_unit") == "metres"
        and clock.get("internal_isb_state_unit") == "metres"
        and clock.get("internal_drift_state_unit") == "metres_per_second"
    )
    output_ok = (
        isinstance(epochs, dict)
        and epochs.get("problem") == expected
        and epochs.get("output") == expected
        and nonnegative_int(epochs.get("pseudorange_factors"))
        and epochs.get("pseudorange_factors") > 0
        and nonnegative_int(epochs.get("tdcp_factors_built"))
        and epochs.get("tdcp_factors_built") > 0
        and isinstance(output_contract, dict)
        and output_contract.get("finite_coordinates") is True
    )
    no_publication = (
        metadata.get("runner_read_raw_payloads") is False
        and metadata.get("runner_read_raw_hashes") is False
        and metadata.get("runner_read_truth_mat_phone_coordinate_precomputed_pdc_kaggle") is False
        and metadata.get("raw_content_copied_or_transformed") is False
        and metadata.get("solution_output_opened") is False
        and metadata.get("solution_output_published") is False
        and summary.get("truth_used") is False
        and summary.get("base_factors") is False
        and summary.get("production_default_changed") is False
    )
    no_fallback = (
        summary.get("status") == "imu-combined-factor"
        and summary.get("native_pdc_state_bridge") is False
        and summary.get("native_pdc_imu_tdcp_no_bridge") is True
        and summary.get("native_source_direct_observable_quality_enabled") is True
    )
    process_ok = metadata.get("return_code") == 0 and not metadata.get("launch_error") and not metadata.get("interrupted") and not metadata.get("timed_out")
    gates = {
        "native_process_completed": process_ok,
        "summary_present": True,
        "base_path_hash_bytes_header_match": base_path_hash,
        "base_preserve_reader_active": isinstance(base, dict) and base.get("preserve_additional_frequency_bands") is True and selected_maps,
        "base_factors_active_exactly_once": exact_once,
        "base_signal_band_taxonomy_conserved": taxonomy_ok,
        "gnss_first_progress_strict_cost_decrease": gnss_progress,
        "gnss_first_full_finite_c7_d_exact_handoff": handoff,
        "main_qr_progress_strict_cost_decrease": qr_selected and main_progress,
        "finite_earth_valid_expected_output_coverage": output_ok,
        "no_fallback_or_solution_publication": no_fallback and no_publication,
    }
    report["telemetry"] = {
        "base": base,
        "source_miss_mask": miss,
        "signal_taxonomy": taxonomy,
        "gnss_first": gnss,
        "main": {"graph": graph, "clock": clock},
        "solver": {
            "enabled": summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_enabled"),
            "selected": summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected"),
            "linear_solver_type": summary.get("selected_linear_solver_type"),
            "branch": summary.get("selected_solver_branch"),
            "elimination": summary.get("selected_elimination_function"),
        },
        "output": {"epochs": epochs, "contract": output_contract},
    }
    report["gates"] = gates
    report["failure_reasons"] = [name for name, passed in gates.items() if passed is not True]
    return report


def render_markdown(result: dict[str, Any]) -> str:
    lines = [
        "# Phase109 raw-base frequency-parity structural result",
        "",
        f"- status: `{result['status']}`",
        f"- candidate: `{result['candidate']['id']}`",
        f"- routes: `{', '.join(result['routes'].keys())}`",
        "- truth/MAT/phone-result-coordinate/PDC/Kaggle/accuracy lanes: not read",
        "- solution CSV: withheld and not opened",
        "",
        "## Route gates",
        "",
        "| Route | Return | Structural gates |",
        "|---|---:|---|",
    ]
    for route, report in result["routes"].items():
        passed = sum(value is True for value in report.get("gates", {}).values())
        total = len(report.get("gates", {}))
        lines.append(f"| `{route}` | `{report.get('return_code')}` | `{passed}/{total}` |")
    lines.extend([
        "",
        "The JSON artifact contains compact per-signal/band conservation, "
        "exactly-once correction, GNSS-first/main progress, C7/D handoff, QR, "
        "and output-coverage telemetry.  A failed gate remains sealed as "
        "fail-closed; no retry or fallback is available.",
        "",
    ])
    return "\n".join(lines)


def build_result(metadata: list[dict[str, Any]], module: Any, manifest: dict[str, Any], authorization: dict[str, Any]) -> dict[str, Any]:
    reports = [validate_route(item, module, manifest) for item in metadata]
    by_route = {item["dataset_id"]: item for item in reports}
    expected_gates = (
        "native_process_completed", "summary_present", "base_path_hash_bytes_header_match",
        "base_preserve_reader_active", "base_factors_active_exactly_once",
        "base_signal_band_taxonomy_conserved", "gnss_first_progress_strict_cost_decrease",
        "gnss_first_full_finite_c7_d_exact_handoff", "main_qr_progress_strict_cost_decrease",
        "finite_earth_valid_expected_output_coverage", "no_fallback_or_solution_publication",
    )
    all_passed = (
        len(reports) == len(ROUTES)
        and [item["dataset_id"] for item in reports] == list(ROUTES)
        and all(all(report.get("gates", {}).get(name) is True for name in expected_gates) for report in reports)
    )
    failed = {
        route: [name for name in expected_gates if report.get("gates", {}).get(name) is not True]
        for route, report in by_route.items()
        if any(report.get("gates", {}).get(name) is not True for name in expected_gates)
    }
    result = {
        "schema_version": "smartphone-r5-phase109-raw-base-frequency-parity-structural-result.v1",
        "phase": 109,
        "execution_label": "Luna Max",
        "status": "go-phase109-raw-base-frequency-parity-structural" if all_passed else "no-go-phase109-raw-base-frequency-parity-structural",
        "decision": "Structural gates passed; truth/accuracy and solution-release lanes remain unauthorized." if all_passed else "Structural gate failed closed; no retry, fallback, truth, accuracy, or solution-release lane is available.",
        "candidate": {
            "id": module.CANDIDATE_ID,
            "selectors": [module.PHASE93_SELECTOR, module.VECTOR_SELECTOR, module.QR_SELECTOR],
            "base_selectors": ["--native-base-pseudorange-compensation", "--native-base-pseudorange-source-miss-mask", module.PRESERVE_SELECTOR],
            "default_off": True,
            "diagnostic_only": True,
            "solution_output_published": False,
        },
        "freeze": {"path": relative(module.FREEZE), "sha256": module.FREEZE_SHA256, "commit": module.FREEZE_COMMIT},
        "manifest": {"path": relative(MANIFEST), "sha256": sha256_file(MANIFEST, "Phase109 manifest")},
        "authorization": {"path": relative(AUTHORIZATION), "sha256": sha256_file(AUTHORIZATION, "Phase109 authorization"), "status": authorization.get("status")},
        "evaluator": {"path": relative(PRE_RAW), "sha256": sha256_file(PRE_RAW, "Phase109 evaluator")},
        "execution_wrapper": {"path": relative(Path(__file__).resolve()), "sha256": sha256_file(Path(__file__).resolve(), "Phase109 wrapper")},
        "implementation": {"commit": module.IMPLEMENTATION_COMMIT, "source_sha256": module.SOURCE_SHA256},
        "routes": by_route,
        "matrix": {
            "candidate_count": 1,
            "route_count": 2,
            "runs_per_route": 1,
            "native_solver_invocations": len(metadata),
            "controls": 0,
            "reruns": 0,
            "fallbacks": 0,
            "raw_phone_gnss_process_reads": len(metadata),
            "raw_phone_imu_process_reads": len(metadata),
            "broadcast_navigation_process_reads": len(metadata),
            "raw_base_wrapper_hash_reads": len(metadata),
            "raw_base_native_process_reads_declared": len(metadata),
            "runner_raw_payload_reads": 0,
            "runner_raw_input_hash_reads": 0,
            "truth_reads": 0,
            "mat_reads_or_generated": 0,
            "phone_coordinate_reads": 0,
            "precomputed_coordinate_reads": 0,
            "pdc_reads": 0,
            "accuracy_calculations": 0,
            "solution_rows_published": False,
        },
        "gates": {"all_structural_gates_passed": all_passed, "route_order_exact": len(reports) == 2 and [item["dataset_id"] for item in reports] == list(ROUTES), "failed_routes": failed},
        "failed_gates": failed,
        "read_accounting": {
            "native_solver_invocations": len(metadata),
            "raw_phone_gnss_process_reads": len(metadata),
            "raw_phone_imu_process_reads": len(metadata),
            "broadcast_navigation_process_reads": len(metadata),
            "raw_base_wrapper_hash_reads": len(metadata),
            "raw_base_native_process_reads_declared": len(metadata),
            "runner_raw_payload_reads": 0,
            "runner_raw_input_hash_reads": 0,
            "truth_reads": 0,
            "mat_reads_or_generated": 0,
            "phone_coordinate_reads": 0,
            "precomputed_coordinate_reads": 0,
            "pdc_reads": 0,
            "accuracy_calculations": 0,
            "kaggle_or_token_access": 0,
            "route_reruns": 0,
            "fallbacks": 0,
            "solution_output_opened": False,
            "solution_output_published": False,
            "raw_content_copied_or_transformed": False,
            "logs_and_partial_results_preserved": True,
        },
        "forbidden_lanes": {"truth": False, "MAT": False, "phone_coordinates": False, "precomputed_coordinates": False, "PDC": False, "Kaggle_or_token": False, "accuracy": False, "solution_rows": False},
        "truth_free": True,
        "accuracy_scored": False,
        "solution_output_published": False,
        "promotion_authorized": False,
        "stop_before_truth_accuracy_submission": True,
    }
    return result


def main() -> int:
    module = load_contract_module()
    try:
        pre_raw = module.verify_pre_raw()
        if any(value != 0 for key, value in pre_raw.items() if key.endswith("_reads") or key in {
            "native_solver_invocations", "mat_reads_or_generated", "phone_coordinate_reads",
            "precomputed_coordinate_reads", "pdc_reads", "accuracy_calculations",
        }):
            raise fail("pre-raw verifier reported prohibited activity")
        manifest = module.verify_manifest()
        authorization = module.verify_authorization(manifest)
        if authorization.get("status") != "authorized-for-exact-two-route-phase109-raw-base-frequency-structural-execution":
            raise fail("Phase109 authorization status is not exact")
        metadata = execute_matrix(module, manifest)
        result = build_result(metadata, module, manifest, authorization)
        atomic_json(RESULT_JSON, result)
        atomic_text(RESULT_MD, render_markdown(result))
        return 0
    except (Phase109ExecutionError, OSError) as exc:
        print(f"phase109 execution: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
