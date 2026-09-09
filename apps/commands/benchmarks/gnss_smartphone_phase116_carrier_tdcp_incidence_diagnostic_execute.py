#!/usr/bin/env python3
"""Execute exactly one authorized Phase116 A/LAX raw/base diagnostic.

The wrapper materializes only sealed phone GNSS/IMU, broadcast navigation, and
raw base RINEX paths after the independent authorization.  Phone payloads are
metadata-statted, the base member is hashed once for its sealed identity, and
the native process reads each input once.  The native solution CSV remains
opaque: it is never opened, hashed, counted, interpreted, published, or
committed.  Only the non-solution summary telemetry is read and sealed.
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
CONTRACT_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase116_carrier_tdcp_incidence_diagnostic.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase116_carrier_tdcp_incidence_diagnostic_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase116_carrier_tdcp_incidence_diagnostic_authorization_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase116-carrier-tdcp-incidence-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase116_carrier_tdcp_incidence_structural_result_v1.json"
RESULT_MD = ROOT / "docs/use_cases/records/smartphone_r5_phase116_carrier_tdcp_incidence_structural_result_v1.md"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")


class Phase116ExecutionError(ValueError):
    """Raised when the one-shot Phase116 execution fails closed."""


def fail(message: str) -> Phase116ExecutionError:
    return Phase116ExecutionError(message)


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
    # The solution and input members are deliberately opaque to this wrapper.
    if path.name in RAW_NAMES or path.name == "base.obs" or path.name.endswith("withheld_solution_output.csv"):
        raise fail(f"payload hash forbidden by Phase116 wrapper: {label}")
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
    # This is the only wrapper payload read, and it occurs after auth.
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


def load_contract() -> Any:
    spec = importlib.util.spec_from_file_location("phase116_contract", CONTRACT_PATH)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load Phase116 contract: {CONTRACT_PATH}")
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
    # This function is called only after verify_authorization().  It reads
    # sealed metadata first, then stats phone files and hashes each base once.
    source_routes = contract.phase112_route_metadata()
    selected: dict[str, dict[str, Any]] = {}
    for route in ROUTES:
        source = source_routes[route]
        selected[route] = {"raw": {}, "base": {}}
        for name in RAW_NAMES:
            pin = source["raw_inputs"][name]
            path = safe_member(pin["path"], name, route)
            size = path.stat().st_size
            if size != pin["bytes"]:
                raise fail(f"sealed raw byte count changed: {route}/{name}")
            # Metadata only: do not open or hash phone GNSS/IMU/navigation.
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


def materialize_command(contract: Any, record: dict[str, Any],
                        inputs: dict[str, Any]) -> list[str]:
    source_routes = contract.phase112_route_metadata()
    route = record["dataset_id"]
    contract.validate_command(route, record.get("command"), record,
                              source_routes[route])
    replacements = {
        "__PHASE116_RAW_DEVICE_GNSS__": inputs["raw"]["device_gnss.csv"]["path"],
        "__PHASE116_RAW_DEVICE_IMU__": inputs["raw"]["device_imu.csv"]["path"],
        "__PHASE116_RAW_BROADCAST_NAV__": inputs["raw"]["brdc.nav"]["path"],
        "__PHASE116_RAW_BASE_RINEX__": inputs["base"]["path"],
        "__PHASE116_RAW_BASE_SHA256__": inputs["base"]["sha256"],
    }
    command = [replacements.get(token, token) for token in record["command"]]
    for token in command:
        if token in contract.FORBIDDEN_FLAGS:
            raise fail(f"forbidden flag after materialization: {route}/{token}")
    return command


def safe_environment() -> dict[str, str]:
    # No parent truth/MAT/coordinate/token environment is inherited.
    return {
        "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
        "LANG": "C", "LC_ALL": "C", "TZ": "UTC",
        "LD_LIBRARY_PATH": "/home/sasaki/.local/lib",
    }


def finite(value: Any) -> bool:
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(float(value)))


def read_summary(path: Path) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    if not path.is_file():
        return None, {"path": relative(path), "present": False, "read_count": 0}
    payload = path.read_bytes()
    try:
        summary = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        return None, {"path": relative(path), "present": True,
                      "bytes": len(payload), "error": str(exc), "read_count": 1}
    if not isinstance(summary, dict):
        return None, {"path": relative(path), "present": True,
                      "bytes": len(payload), "error": "summary is not an object",
                      "read_count": 1}
    return summary, {"path": relative(path), "present": True,
                     "bytes": len(payload),
                     "sha256": hashlib.sha256(payload).hexdigest(),
                     "read_count": 1}


def compact_telemetry(summary: dict[str, Any]) -> dict[str, Any]:
    # Copy only structural/non-solution fields.  In particular, no output CSV
    # row, position, latitude, longitude, truth, or accuracy field is read.
    return {
        "phase116": summary.get("phase116_carrier_tdcp_incidence"),
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


def validate_route(metadata: dict[str, Any], manifest: dict[str, Any]) -> dict[str, Any]:
    route = metadata["dataset_id"]
    record = next(item for item in manifest["routes"] if item["dataset_id"] == route)
    expected = int(record["expected_problem_epochs"])
    summary_path = ROOT / metadata["planned_output"]["summary"]
    summary, summary_meta = read_summary(summary_path)
    report: dict[str, Any] = {
        "dataset_id": route,
        "return_code": metadata.get("return_code"),
        "expected": {"domain_rows": int(record["domain_rows"]),
                      "problem_epochs": expected, "output_epochs": expected},
        "summary": summary_meta,
        "raw_inputs": metadata.get("raw_inputs"),
        "base_input": metadata.get("base_input"),
        "truth_used": False, "mat_used": False,
        "phone_coordinates_used": False, "precomputed_coordinates_used": False,
        "pdc_used": False, "kaggle_or_token_accessed": False,
        "solution_output_published": False, "solution_output_opened": False,
        "withheld_solution_output_present": metadata.get("withheld_solution_output_present", False),
        "read_accounting": {
            "runner_raw_payload_reads": 0, "runner_raw_hash_reads": 0,
            "runner_base_hash_reads": 1, "native_raw_phone_gnss_reads": 1,
            "native_raw_phone_imu_reads": 1, "native_broadcast_navigation_reads": 1,
            "native_base_rinex_reads": 1, "truth_reads": 0,
            "mat_reads_or_generated": 0, "phone_coordinate_reads": 0,
            "precomputed_coordinate_reads": 0, "pdc_reads": 0,
            "accuracy_calculations": 0, "solution_output_reads": 0,
            "raw_content_copied_or_transformed": False,
        },
    }
    gate_names = (
        "native_process_completed", "summary_present", "phase116_enabled_read_only",
        "ordinary_tdcp_only_no_dd_or_ambiguity", "tdcp_factor_insertion_exact",
        "per_signal_accounting_conserved", "finite_tdcp_costs_and_connected_keys",
        "gnss_first_and_main_progress", "qr_solver_selected",
        "base_correction_exactly_once", "offset_exactly_once_final_boundary",
        "finite_expected_output_coverage", "no_solution_or_accuracy_publication",
    )
    if summary is None:
        report["telemetry"] = None
        report["gates"] = {name: False for name in gate_names}
        report["failure_reasons"] = list(gate_names)
        return report

    phase116 = summary.get("phase116_carrier_tdcp_incidence")
    epochs = summary.get("epochs")
    tdcp = summary.get("tdcp_contract")
    gnss = summary.get("gnss_first")
    graph = summary.get("graph")
    clock = summary.get("native_source_clock_c0d_factor")
    base = summary.get("native_base_pseudorange_compensation")
    miss = summary.get("native_base_pseudorange_source_miss_mask")
    offset = summary.get("upstream_position_offset")
    raw_utc = summary.get("raw_utc_key_contract")
    phase116_ok = isinstance(phase116, dict) and phase116.get("enabled") is True and phase116.get("read_only") is True
    ordinary_only = (
        isinstance(phase116, dict) and phase116.get("ordinary_tdcp_only") is True and
        phase116.get("standalone_carrier_phase_factors") is False and
        phase116.get("double_difference_factors") is False and
        phase116.get("ambiguity_states") is False and
        phase116.get("pdc_state_bridge") is False
    )
    factors_built = phase116.get("factors_built") if isinstance(phase116, dict) else None
    factors_inserted = phase116.get("factors_inserted") if isinstance(phase116, dict) else None
    insertion_ok = (
        isinstance(factors_built, int) and factors_built > 0 and
        isinstance(factors_inserted, int) and factors_inserted == factors_built and
        phase116.get("factors_inserted_exact") is True and
        isinstance(tdcp, dict) and tdcp.get("factors_built") == factors_built and
        tdcp.get("factors_inserted") == factors_inserted
    )
    signals = phase116.get("signals") if isinstance(phase116, dict) else None
    signal_accounting = False
    finite_costs = False
    if isinstance(signals, dict) and signals:
        accepted_total = 0
        signal_accounting = True
        finite_costs = True
        for value in signals.values():
            if not isinstance(value, dict):
                signal_accounting = False
                finite_costs = False
                continue
            accepted = value.get("accepted_pairs")
            candidates = value.get("candidate_pairs")
            if not isinstance(accepted, int) or not isinstance(candidates, int) or accepted < 0 or accepted > candidates:
                signal_accounting = False
            accepted_total += accepted if isinstance(accepted, int) else 0
            for key in ("initial_residual_count", "final_residual_count", "connected_key_count"):
                if not isinstance(value.get(key), int) or value.get(key) < 0:
                    finite_costs = False
            keys = value.get("connected_keys")
            if not isinstance(keys, list) or value.get("connected_key_count") != len(keys):
                finite_costs = False
            for key in ("initial_robust_cost", "final_robust_cost",
                        "initial_unwhitened_cost", "final_unwhitened_cost",
                        "initial_whitened_cost", "final_whitened_cost"):
                if value.get(key) is not None and not finite(value.get(key)):
                    finite_costs = False
        signal_accounting = signal_accounting and accepted_total == factors_built
        finite_costs = finite_costs and isinstance(factors_built, int) and factors_built > 0
    gnss_progress = (
        isinstance(gnss, dict) and gnss.get("attempted") is True and
        gnss.get("converged") is True and isinstance(gnss.get("iterations"), int) and
        gnss.get("iterations") >= 1 and finite(gnss.get("initial_cost")) and
        finite(gnss.get("final_cost")) and gnss.get("final_cost") < gnss.get("initial_cost")
    )
    main_progress = (
        isinstance(graph, dict) and graph.get("converged") is True and
        isinstance(graph.get("iterations"), int) and graph.get("iterations") >= 1 and
        finite(graph.get("initial_cost")) and finite(graph.get("final_cost")) and
        graph.get("final_cost") < graph.get("initial_cost") and
        isinstance(clock, dict) and clock.get("active_solve_attempted") is True and
        isinstance(clock.get("accepted_outer_iterations"), int) and
        clock.get("accepted_outer_iterations") > 0 and
        finite(clock.get("active_solve_initial_cost")) and
        finite(clock.get("active_solve_final_cost")) and
        clock.get("active_solve_final_cost") < clock.get("active_solve_initial_cost")
    )
    qr_ok = (
        summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_enabled") is True and
        summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected") is True and
        summary.get("selected_linear_solver_type") == "MULTIFRONTAL_QR" and
        summary.get("selected_elimination_function") == "EliminateQR"
    )
    base_ok = (
        isinstance(base, dict) and base.get("enabled") is True and base.get("applied") is True and
        base.get("correction_application_pass_count") == 1 and
        base.get("correction_applied_exactly_once") is True and
        base.get("duplicate_correction_rejected") is False and
        isinstance(miss, dict) and miss.get("correction_application_pass_count") == 1 and
        miss.get("correction_applied_exactly_once") is True and
        miss.get("duplicate_correction_rejected") is False
    )
    offset_ok = (
        isinstance(offset, dict) and offset.get("enabled") is True and
        offset.get("applied") is True and offset.get("phone") == "pixel5" and
        offset.get("corrected_epochs") == expected and
        finite(offset.get("max_offset_enu_m"))
    )
    output_ok = (
        isinstance(epochs, dict) and epochs.get("problem") == expected and
        epochs.get("output") == expected and isinstance(raw_utc, dict) and
        raw_utc.get("raw_epoch_keys") == expected and
        raw_utc.get("unresolved_epochs") == 0
    )
    process_ok = (
        metadata.get("return_code") == 0 and not metadata.get("launch_error") and
        not metadata.get("interrupted") and not metadata.get("timed_out")
    )
    no_publication = (
        metadata.get("runner_read_raw_payloads") is False and
        metadata.get("runner_read_raw_hashes") is False and
        metadata.get("solution_output_opened") is False and
        metadata.get("solution_output_published") is False and
        summary.get("truth_used") is False and
        summary.get("production_default_changed") is False
    )
    report["telemetry"] = compact_telemetry(summary)
    report["gates"] = {
        "native_process_completed": process_ok,
        "summary_present": summary_meta.get("present") is True,
        "phase116_enabled_read_only": phase116_ok,
        "ordinary_tdcp_only_no_dd_or_ambiguity": ordinary_only,
        "tdcp_factor_insertion_exact": insertion_ok,
        "per_signal_accounting_conserved": signal_accounting,
        "finite_tdcp_costs_and_connected_keys": finite_costs,
        "gnss_first_and_main_progress": gnss_progress and main_progress,
        "qr_solver_selected": qr_ok,
        "base_correction_exactly_once": base_ok,
        "offset_exactly_once_final_boundary": offset_ok,
        "finite_expected_output_coverage": output_ok,
        "no_solution_or_accuracy_publication": no_publication,
    }
    report["failure_reasons"] = [name for name, passed in report["gates"].items() if passed is not True]
    return report


def execute_matrix(contract: Any, manifest: dict[str, Any]) -> list[dict[str, Any]]:
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite Phase116 output root: {OUTPUT_ROOT}")
    records = manifest.get("routes")
    if not isinstance(records, list) or [item.get("dataset_id") for item in records] != list(ROUTES):
        raise fail("Phase116 route order/count changed")
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
                    command, cwd=ROOT, env=safe_environment(), stdout=stdout,
                    stderr=stderr, check=False, timeout=1800,
                )
                return_code = completed.returncode
            except subprocess.TimeoutExpired as exc:
                timed_out = True
                stderr.write(f"\nPhase116 native launch timed out: {exc}\n".encode())
            except KeyboardInterrupt:
                interrupted = True
                stderr.write(b"\nPhase116 wrapper interrupted; partial output preserved.\n")
            except OSError as exc:
                launch_error = str(exc)
                stderr.write(f"\nPhase116 native launch failed: {exc}\n".encode())
        item = {
            "schema_version": "smartphone-r5-phase116-carrier-tdcp-route-execution.v1",
            "dataset_id": route, "run_number": 1, "command": command,
            "raw_inputs": inputs[route]["raw"], "base_input": inputs[route]["base"],
            "planned_output": {
                "summary": relative(summary_path),
                "withheld_solution_output": relative(solution_path),
            },
            "summary_present_after_launch": summary_path.is_file(),
            # Existence is metadata only; the solution bytes are never opened.
            "withheld_solution_output_present_after_launch": solution_path.is_file(),
            "stdout": relative(stdout_path), "stderr": relative(stderr_path),
            "started_unix_s": started, "ended_unix_s": time.time(),
            "return_code": return_code, "interrupted": interrupted,
            "timed_out": timed_out, "launch_error": launch_error,
            "runner_read_raw_payloads": False, "runner_read_raw_hashes": False,
            "runner_read_truth_mat_phone_coordinate_precomputed_pdc_kaggle": False,
            "runner_read_base_payload_for_hash": True,
            "raw_content_copied_or_transformed": False,
            "solution_output_opened": False, "solution_output_published": False,
            "accuracy_scored": False,
        }
        atomic_json(route_dir / "run_metadata.json", item)
        metadata.append(item)
        atomic_json(OUTPUT_ROOT / "partial_execution_metadata.json",
                    {"routes_completed": metadata})
    return metadata


def build_result(metadata: list[dict[str, Any]], contract: Any,
                 manifest: dict[str, Any], authorization: dict[str, Any]) -> dict[str, Any]:
    reports = [validate_route(item, manifest) for item in metadata]
    by_route = {item["dataset_id"]: item for item in reports}
    gate_names = tuple(
        "native_process_completed summary_present phase116_enabled_read_only "
        "ordinary_tdcp_only_no_dd_or_ambiguity tdcp_factor_insertion_exact "
        "per_signal_accounting_conserved finite_tdcp_costs_and_connected_keys "
        "gnss_first_and_main_progress qr_solver_selected base_correction_exactly_once "
        "offset_exactly_once_final_boundary finite_expected_output_coverage "
        "no_solution_or_accuracy_publication"
    .split())
    all_passed = (
        len(reports) == 2 and
        [item["dataset_id"] for item in reports] == list(ROUTES) and
        all(all(report.get("gates", {}).get(name) is True for name in gate_names)
            for report in reports)
    )
    failed = {
        route: [name for name in gate_names if report.get("gates", {}).get(name) is not True]
        for route, report in by_route.items()
        if any(report.get("gates", {}).get(name) is not True for name in gate_names)
    }
    return {
        "schema_version": "smartphone-r5-phase116-carrier-tdcp-structural-result.v1",
        "phase": 116, "execution_label": "Luna Max",
        "status": "go-phase116-carrier-tdcp-diagnostic" if all_passed else "no-go-phase116-carrier-tdcp-diagnostic",
        "decision": (
            "Diagnostic structural gates passed; solution/truth/accuracy lanes remain unauthorized."
            if all_passed else
            "Diagnostic gate failed closed; preserve partial artifacts and do not retry or fallback."
        ),
        "candidate": {
            "id": contract.CANDIDATE_ID, "candidate_count": 1,
            "selector": contract.DIAGNOSTIC_SELECTOR,
            "selectors": [contract.PHASE93_SELECTOR, contract.VECTOR_SELECTOR,
                          contract.QR_SELECTOR, contract.OFFSET_SELECTOR,
                          contract.DIAGNOSTIC_SELECTOR],
            "base_selectors": list(contract.BASE_SELECTORS),
            "ordinary_tdcp_only": True, "default_off": True,
            "diagnostic_only": True, "solution_withheld": True,
            "solution_output_published": False,
        },
        "freeze": {"path": relative(contract.FREEZE),
                   "sha256": contract.FREEZE_SHA256,
                   "commit": contract.FREEZE_COMMIT},
        "manifest": {"path": relative(MANIFEST),
                      "sha256": sha256_file(MANIFEST, "Phase116 manifest")},
        "authorization": {"path": relative(AUTHORIZATION),
                          "sha256": sha256_file(AUTHORIZATION, "Phase116 authorization"),
                          "status": authorization.get("status")},
        "implementation": {"commit": contract.IMPLEMENTATION_COMMIT,
                            "source_sha256": contract.SOURCE_SHA256},
        "routes": by_route,
        "matrix": {
            "candidate_count": 1, "route_count": 2, "runs_per_route": 1,
            "native_solver_invocations": len(metadata), "controls": 0,
            "reruns": 0, "fallbacks": 0,
            "raw_phone_gnss_process_reads": len(metadata),
            "raw_phone_imu_process_reads": len(metadata),
            "broadcast_navigation_process_reads": len(metadata),
            "raw_base_wrapper_hash_reads": len(metadata),
            "raw_base_native_process_reads_declared": len(metadata),
            "truth_reads": 0, "mat_reads_or_generated": 0,
            "phone_coordinate_reads": 0, "precomputed_coordinate_reads": 0,
            "pdc_reads": 0, "accuracy_calculations": 0,
            "solution_rows_published": False,
        },
        "gates": {"all_structural_gates_passed": all_passed,
                  "route_order_exact": len(reports) == 2 and
                  [item["dataset_id"] for item in reports] == list(ROUTES),
                  "failed_routes": failed},
        "failed_gates": failed,
        "read_accounting": {
            "native_solver_invocations": len(metadata),
            "raw_phone_gnss_process_reads": len(metadata),
            "raw_phone_imu_process_reads": len(metadata),
            "broadcast_navigation_process_reads": len(metadata),
            "raw_base_wrapper_hash_reads": len(metadata),
            "raw_base_native_process_reads_declared": len(metadata),
            "runner_raw_payload_reads": 0, "runner_raw_input_hash_reads": 0,
            "truth_reads": 0, "mat_reads_or_generated": 0,
            "phone_coordinate_reads": 0, "precomputed_coordinate_reads": 0,
            "pdc_reads": 0, "accuracy_calculations": 0,
            "kaggle_or_token_access": 0, "route_reruns": 0, "fallbacks": 0,
            "solution_output_reads": 0, "solution_output_published": False,
            "raw_content_copied_or_transformed": False,
            "logs_and_partial_results_preserved": True,
        },
        "forbidden_lanes": {
            "truth": False, "MAT": False, "phone_coordinates": False,
            "precomputed_coordinates": False, "PDC": False,
            "Kaggle_or_token": False, "accuracy": False,
            "solution_rows": False,
        },
        "truth_free": True, "accuracy_scored": False,
        "solution_output_published": False,
        "promotion_authorized": False,
        "stop_before_truth_accuracy_submission": True,
    }


def render_markdown(result: dict[str, Any]) -> str:
    lines = [
        "# Phase116 ordinary TDCP incidence diagnostic result", "",
        f"- status: `{result['status']}`",
        "- recipe: Phase112 raw/base + Phase101 C7/D + Phase99 QR + Pixel5 offset",
        "- diagnostic: ordinary same-satellite/same-signal TDCP only; no DD/ambiguity/PDC state",
        "- truth/MAT/phone-coordinate/precomputed/PDC/Kaggle/accuracy lanes: not read",
        "- solution CSV: withheld and never opened, hashed, interpreted, or published", "",
        "| Route | Return | Gates |", "|---|---:|---:|",
    ]
    for route, report in result["routes"].items():
        gates = report.get("gates", {})
        passed = sum(value is True for value in gates.values())
        lines.append(f"| `{route}` | `{report.get('return_code')}` | `{passed}/{len(gates)}` |")
    lines.extend(["", "A failed gate remains sealed fail-closed; no retry, fallback, truth score, or solution release is permitted.", ""])
    return "\n".join(lines)


def main() -> int:
    contract = load_contract()
    # Auth and preflight are read-only.  No raw member is touched until after
    # this independent authorization verification returns successfully.
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
    except (Phase116ExecutionError, OSError) as exc:
        print(f"phase116 execution: fail-closed: {exc}", file=sys.stderr)
        raise SystemExit(2)
