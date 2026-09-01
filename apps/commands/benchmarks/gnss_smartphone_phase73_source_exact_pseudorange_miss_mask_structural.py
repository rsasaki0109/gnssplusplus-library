#!/usr/bin/env python3
"""Truth-free structural matrix for the Phase73 source-exact miss mask.

The candidate retains an adopted pseudorange factor only when the source
compatible base correction is finite and in-domain.  This evaluator runs one
flag-off Phase43 control and two candidate repetitions per route.  It keeps
summary artifact metadata separate from the parsed native summary payload so
that an integrity failure cannot be hidden by a dictionary-key collision.
No development truth, MATLAB product, validation/holdout data, archive, WLS,
or solver process outside the twelve native invocations is opened.
"""

from __future__ import annotations

import argparse
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
PHASE65_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase65_native_base_pseudorange_compensation.py"
_SPEC = importlib.util.spec_from_file_location("phase65_structural_helpers", PHASE65_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load sealed Phase65 helper: {PHASE65_PATH}")
P65 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(P65)


FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase73_source_exact_pseudorange_miss_mask_structural_freeze_v1.json"
FREEZE_SHA256 = "9d97f1a97bb8559f5f0ede55fcecd109c590b8afb2cf10938b2884d8ffd6d1a0"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase73_source_exact_pseudorange_miss_mask_structural_manifest_v1.json"
EVALUATOR = Path(__file__).resolve()
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
BINARY_SHA256 = "941eec7044775dbc0b0c1f512a3afbc6821f1997ef299a56f3018ce0747c279d"
SOURCE_COMMIT = "fda746f79a7bb88d9c24c7637d75b228350af838"
SOURCE_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase73_source_exact_pseudorange_miss_mask_freeze_v1.json"
SOURCE_FREEZE_SHA256 = "cd38928123f75e0ff71a42c1486e5bb5998bd506f0ccee3b830aa60c2d696b33"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase73-source-exact-pseudorange-miss-mask-structural-v1"
OUTPUT_SCHEMA = "smartphone-r5-phase73-source-exact-pseudorange-miss-mask-structural-result.v1"
MISS_MASK_FLAG = "--native-base-pseudorange-source-miss-mask"
BASE_COMP_FLAG = "--native-base-pseudorange-compensation"
BASE_RINEX_FLAG = "--native-base-rinex"
BASE_SHA_FLAG = "--native-base-rinex-sha256"
ROUTES = P65.ROUTES
INPUTS = P65.INPUTS
BASE_INPUTS = P65.BASE_INPUTS
RAW_INPUT_HASHES = P65.RAW_INPUT_HASHES
BASE_INPUT_HASHES = P65.BASE_INPUT_HASHES
PHASE43_CONTROL = P65.PHASE43_CONTROL
BASE_FLAGS = P65.BASE_FLAGS
PHASE43_ROOT = P65.PHASE43_ROOT
EARTH_RADIUS_M = P65.EARTH_RADIUS_M
MAX_SPEED_MPS = P65.MAX_SPEED_MPS


class Phase73StructuralError(ValueError):
    """Raised when a frozen Phase73 structural contract fails."""


def fail(message: str) -> Phase73StructuralError:
    return Phase73StructuralError(message)


def reject_forbidden(path: Path | str) -> None:
    P65.reject_forbidden(path)


def sha256(path: Path) -> str:
    return P65.sha256(path)


def load_json(path: Path, label: str) -> dict[str, Any]:
    return P65.load_json(path, label)


def atomic_write(path: Path, payload: bytes) -> None:
    P65.atomic_write(path, payload)


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    P65.atomic_json(path, value)


def relative(path: Path) -> str:
    return P65.relative(path)


def verify_freeze() -> dict[str, Any]:
    if sha256(FREEZE) != FREEZE_SHA256:
        raise fail("Phase73 structural freeze hash changed")
    freeze = load_json(FREEZE, "Phase73 structural freeze")
    if freeze.get("status") != "frozen-before-phase73-structural-reads":
        raise fail("Phase73 structural freeze status changed")
    authority = freeze.get("authority", {})
    if authority.get("implementation_commit") != SOURCE_COMMIT:
        raise fail("Phase73 implementation commit pin changed")
    if authority.get("source_freeze_sha256") != SOURCE_FREEZE_SHA256:
        raise fail("Phase73 source freeze pin changed")
    if sha256(SOURCE_FREEZE) != SOURCE_FREEZE_SHA256:
        raise fail("Phase73 source freeze bytes changed")
    if authority.get("phase72_candidate_not_reused") is not True:
        raise fail("Phase72 partial candidate reuse contract changed")
    artifacts = freeze.get("implementation_artifacts", {})
    if artifacts.get("binary", {}).get("sha256") != BINARY_SHA256:
        raise fail("Phase73 binary pin changed")
    if sha256(BINARY) != BINARY_SHA256:
        raise fail("built Phase73 binary hash changed")
    candidate = freeze.get("candidate", {})
    if candidate.get("flag") != MISS_MASK_FLAG or candidate.get("required_flags") != [BASE_COMP_FLAG, BASE_RINEX_FLAG, BASE_SHA_FLAG]:
        raise fail("Phase73 candidate flag contract changed")
    if candidate.get("preserve_additional_frequency_bands") is not False:
        raise fail("additional-frequency-band option must remain off")
    if candidate.get("no_fill_or_extrapolation") is not True or candidate.get("no_new_rows_or_factors") is not True:
        raise fail("source miss-mask scope changed")
    if candidate.get("tdcp_doppler_imu_spp_unchanged") is not True or candidate.get("default_off") is not True:
        raise fail("unchanged-path contract changed")
    routes = freeze.get("routes_and_inputs", {}).get("route_order")
    if tuple(routes or ()) != ROUTES:
        raise fail("Phase73 route order changed")
    telemetry = freeze.get("telemetry_gates", {})
    required = {
        "original_adopted_pseudorange_rows",
        "retained_finite_pc_pseudorange_rows",
        "dropped_missing_exact_stream_rows",
        "dropped_out_of_domain_rows",
        "dropped_nonfinite_correction_rows",
        "retained_finite_pc_fraction",
        "retained_over_original_fraction",
        "pseudorange_factors_inserted",
        "pseudorange_factor_count_consistent",
        "correction_abs_p50_m",
        "correction_abs_p95_m",
        "correction_abs_max_m",
    }
    if set(telemetry.get("required_fields", ())) != required:
        raise fail("Phase73 telemetry field contract changed")
    gates = freeze.get("fixed_structural_gates", {})
    for key in (
        "raw_and_base_hash_bytes_exact",
        "all_core_values_finite",
        "factor_count_consistency_each_route",
        "tdcp_built_equals_inserted",
        "control_flag_off_phase43_identity",
        "candidate_repeat_identity",
        "no_non_pseudorange_population_change",
        "truth_free",
        "no_accuracy_truth_in_this_stage",
        "fail_closed_on_any_mismatch",
    ):
        if gates.get(key) is not True:
            raise fail(f"Phase73 structural gate declaration changed: {key}")
    if gates.get("candidate_retained_finite_pc_fraction_each_route") != 1.0:
        raise fail("Phase73 retained finite-pc gate declaration changed")
    if gates.get("prediction_domain_coverage_exact") != 1.0 or gates.get("over_70_mps_count") != 0:
        raise fail("Phase73 domain/speed gate declaration changed")
    matrix = freeze.get("matrix", {})
    expected_matrix = {
        "control_runs_per_route": 1,
        "candidate_runs_per_route": 2,
        "native_solver_invocations": 12,
        "raw_device_gnss_process_reads": 12,
        "raw_device_imu_process_reads": 12,
        "broadcast_nav_process_reads": 12,
        "base_rinex_process_reads": 12,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "validation_holdout_reads": 0,
        "kaggle_token_reads": 0,
        "archive_reopens": 0,
        "new_output_root": relative(DEFAULT_OUTPUT),
    }
    if any(matrix.get(key) != value for key, value in expected_matrix.items()):
        raise fail("Phase73 structural matrix/read contract changed")
    return freeze


def input_paths(route: str) -> dict[str, Path]:
    if route not in INPUTS or route not in BASE_INPUTS:
        raise fail(f"unknown route: {route}")
    return {name: ROOT / path for name, path in INPUTS[route].items()} | {"base.obs": ROOT / BASE_INPUTS[route]}


def verify_inputs(route: str) -> dict[str, Any]:
    reports: dict[str, Any] = {}
    for name, path in input_paths(route).items():
        digest = sha256(path)
        expected = BASE_INPUT_HASHES[route]["sha256"] if name == "base.obs" else RAW_INPUT_HASHES[route][name]
        if digest != expected:
            raise fail(f"input hash mismatch: {route}/{name}")
        expected_bytes = BASE_INPUT_HASHES[route]["bytes"] if name == "base.obs" else path.stat().st_size
        if path.stat().st_size != expected_bytes:
            raise fail(f"input byte-size mismatch: {route}/{name}")
        reports[name] = {"path": relative(path), "bytes": path.stat().st_size, "sha256": digest}
    return reports


def read_prediction(path: Path, dataset_id: str) -> list[tuple[int, float, float]]:
    return P65.read_prediction(path, dataset_id)


def speed_report(rows: list[tuple[int, float, float]]) -> dict[str, Any]:
    return P65.speed_report(rows)


def _assert_finite(value: Any, label: str) -> None:
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return
    if isinstance(value, (int, float)):
        if isinstance(value, float) and not math.isfinite(value):
            raise fail(f"nonfinite summary value: {label}")
        return
    if isinstance(value, list):
        for index, item in enumerate(value):
            _assert_finite(item, f"{label}[{index}]")
        return
    if isinstance(value, dict):
        for key, item in value.items():
            _assert_finite(item, f"{label}.{key}")


def _require_number(mapping: dict[str, Any], key: str, label: str) -> float:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise fail(f"missing/nonfinite summary number: {label}/{key}")
    return float(value)


def _validate_common_summary(summary: dict[str, Any], route: str) -> None:
    for key, expected in (
        ("dataset_id", route),
        ("truth_used", False),
        ("production_default_changed", False),
        ("native_quality_anchor", True),
        ("native_pdc_imu_tdcp_no_bridge", True),
        ("native_pdc_state_bridge", False),
    ):
        if summary.get(key) != expected:
            raise fail(f"summary contract mismatch {route}/{key}")
    graph = summary.get("graph")
    epochs = summary.get("epochs")
    raw = summary.get("raw_utc_key_contract")
    tdcp = summary.get("tdcp_contract")
    if not all(isinstance(item, dict) for item in (graph, epochs, raw, tdcp)):
        raise fail(f"summary diagnostics missing: {route}")
    assert isinstance(graph, dict) and isinstance(epochs, dict) and isinstance(raw, dict) and isinstance(tdcp, dict)
    if graph.get("converged") is not True or _require_number(epochs, "problem", route) <= 1.0 or _require_number(epochs, "output", route) <= 1.0:
        raise fail(f"graph/epoch gate failed: {route}")
    if _require_number(epochs, "pseudorange_factors", route) <= 0.0:
        raise fail(f"pseudorange factor gate failed: {route}")
    if _require_number(tdcp, "factors_built", route) <= 0.0 or int(_require_number(tdcp, "factors_inserted", route)) != int(_require_number(tdcp, "factors_built", route)) or int(_require_number(tdcp, "nonfinite_residuals", route)) != 0:
        raise fail(f"TDCP factor gate failed: {route}")
    if int(_require_number(raw, "unresolved_epochs", route)) != 0 or int(_require_number(raw, "target_epochs", route)) != int(_require_number(raw, "exact_solution_epochs", route)):
        raise fail(f"raw UTC alignment gate failed: {route}")
    _assert_finite(summary, f"summary[{route}]")


def _validate_base(summary: dict[str, Any], route: str) -> tuple[dict[str, Any], dict[str, Any]]:
    base = summary.get("native_base_pseudorange_compensation")
    miss = summary.get("native_base_pseudorange_source_miss_mask")
    if not isinstance(base, dict) or not isinstance(miss, dict):
        raise fail(f"Phase73 base/miss-mask telemetry missing: {route}")
    pin = BASE_INPUT_HASHES[route]
    if base.get("enabled") is not True or base.get("built") is not True or base.get("applied") is not True:
        raise fail(f"base compensation not applied: {route}")
    if base.get("preserve_additional_frequency_bands") is not False:
        raise fail(f"additional base bands unexpectedly enabled: {route}")
    if base.get("base_member_sha256") != pin["sha256"] or base.get("base_rinex_sha256") != pin["sha256"] or int(base.get("base_rinex_bytes", -1)) != pin["bytes"]:
        raise fail(f"base hash/bytes telemetry mismatch: {route}")
    xyz = base.get("base_coordinate_xyz_m")
    if not isinstance(xyz, list) or len(xyz) != 3 or any(abs(float(actual) - float(expected)) > 1e-6 for actual, expected in zip(xyz, pin["xyz"])):
        raise fail(f"base coordinate telemetry mismatch: {route}")
    if abs(float(base.get("observed_interval_s", math.nan)) - pin["dt_s"]) > 1e-6 or int(base.get("moving_mean_samples", -1)) != pin["window"]:
        raise fail(f"base interval/window telemetry mismatch: {route}")
    if base.get("same_satellite_signal_only") is not True or base.get("spp_applied") is not False or base.get("tdcp_applied") is not False or base.get("doppler_applied") is not False or base.get("no_extrapolation_or_endpoint_hold") is not True:
        raise fail(f"base scope contract failed: {route}")
    if miss.get("enabled") is not True or miss.get("retained_factor_epoch_indices_unchanged") is not True or miss.get("tdcp_doppler_imu_spp_unchanged") is not True or miss.get("no_extrapolation_or_endpoint_hold") is not True:
        raise fail(f"source miss-mask scope contract failed: {route}")
    original = int(miss.get("original_adopted_pseudorange_rows", -1))
    retained = int(miss.get("retained_finite_pc_pseudorange_rows", -1))
    drops = [int(miss.get(key, -1)) for key in ("dropped_missing_exact_stream_rows", "dropped_out_of_domain_rows", "dropped_nonfinite_correction_rows")]
    if original <= 0 or retained <= 0 or any(value < 0 for value in drops) or original != retained + sum(drops):
        raise fail(f"source miss-mask accounting mismatch: {route}")
    if float(miss.get("retained_finite_pc_fraction", math.nan)) != 1.0:
        raise fail(f"retained finite pc fraction is not exactly one: {route}")
    retained_fraction = float(miss.get("retained_over_original_fraction", math.nan))
    if not math.isfinite(retained_fraction) or abs(retained_fraction - retained / original) > 1e-12:
        raise fail(f"retained/original fraction mismatch: {route}")
    inserted = int(miss.get("pseudorange_factors_inserted", -1))
    if inserted != retained or miss.get("pseudorange_factor_count_consistent") is not True:
        raise fail(f"pseudorange factor count mismatch: {route}")
    for key in ("correction_abs_p50_m", "correction_abs_p95_m", "correction_abs_max_m"):
        value = float(miss.get(key, math.nan))
        if not math.isfinite(value) or value < 0.0:
            raise fail(f"retained correction statistic invalid: {route}/{key}")
    if int(epochs_value(summary, "pseudorange_factors")) != inserted:
        raise fail(f"summary pseudorange factor count differs from retained telemetry: {route}")
    if base.get("adopted_pseudorange_rows") != original or base.get("adopted_rows_corrected") != retained:
        raise fail(f"base and miss-mask adopted counts differ: {route}")
    return base, miss


def epochs_value(summary: dict[str, Any], key: str) -> int:
    epochs = summary.get("epochs")
    if not isinstance(epochs, dict):
        raise fail(f"epochs summary missing/{key}")
    return int(_require_number(epochs, key, str(summary.get("dataset_id"))))


def validate_summary(path: Path, route: str, candidate: bool) -> dict[str, Any]:
    summary = load_json(path, "native summary")
    _validate_common_summary(summary, route)
    if candidate:
        base, miss = _validate_base(summary, route)
        return {"summary": summary, "base": base, "source_miss_mask": miss}
    if "native_base_pseudorange_compensation" in summary or "native_base_pseudorange_source_miss_mask" in summary:
        raise fail(f"flag-off base telemetry leaked: {route}")
    return {"summary": summary}


def artifact_report(submission: Path, summary: Path, route: str, candidate: bool) -> dict[str, Any]:
    """Return disjoint metadata and payload keys for an output pair."""
    rows = read_prediction(submission, route)
    speed = speed_report(rows)
    if not speed["finite"] or speed["over_70_mps_count"] != 0:
        raise fail(f"speed gate failed: {route}")
    diagnostics = validate_summary(summary, route, candidate)
    report: dict[str, Any] = {
        "submission_artifact": {
            "path": relative(submission),
            "bytes": submission.stat().st_size,
            "sha256": sha256(submission),
            "rows": len(rows),
        },
        "summary_artifact": {
            "path": relative(summary),
            "bytes": summary.stat().st_size,
            "sha256": sha256(summary),
        },
        "summary_payload": diagnostics["summary"],
        "prediction_keys": [row[0] for row in rows],
        "speed": speed,
    }
    if candidate:
        report["base_telemetry"] = diagnostics["base"]
        report["source_miss_mask_telemetry"] = diagnostics["source_miss_mask"]
    return report


def native_command(route: str, run_dir: Path, candidate: bool) -> list[str]:
    paths = input_paths(route)
    command = [
        str(BINARY.relative_to(ROOT)),
        "--android-gnss", relative(paths["device_gnss.csv"]),
        "--android-imu", relative(paths["device_imu.csv"]),
        "--nav", relative(paths["brdc.nav"]),
        "--out", relative(run_dir / "submission.csv"),
        "--summary-json", relative(run_dir / "summary.json"),
        "--dataset-id", route,
        *BASE_FLAGS,
    ]
    if candidate:
        command.extend((BASE_COMP_FLAG, BASE_RINEX_FLAG, relative(paths["base.obs"]), BASE_SHA_FLAG, BASE_INPUT_HASHES[route]["sha256"], MISS_MASK_FLAG))
    for token in command:
        reject_forbidden(token)
    return command


def run_case(output_root: Path, route: str, variant: str, run_number: int, candidate: bool) -> dict[str, Any]:
    run_dir = output_root / route / variant / f"run{run_number}"
    if run_dir.exists():
        raise fail(f"refusing to overwrite structural output: {run_dir}")
    run_dir.mkdir(parents=True, exist_ok=True)
    command = native_command(route, run_dir, candidate)
    environment = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = local_lib + ((":" + environment["LD_LIBRARY_PATH"]) if environment.get("LD_LIBRARY_PATH") else "")
    started = time.perf_counter()
    try:
        process = subprocess.run(command, cwd=ROOT, env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False, timeout=1800)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise fail(f"native process failed: {route}/{variant}/run{run_number}: {exc}") from exc
    log = process.stdout or ""
    atomic_write(run_dir / "run.log", log.encode())
    report: dict[str, Any] = {
        "variant": variant,
        "run": run_number,
        "candidate": candidate,
        "return_code": process.returncode,
        "wall_seconds": time.perf_counter() - started,
        "command": command,
        "log": {"path": relative(run_dir / "run.log"), "sha256": sha256(run_dir / "run.log")},
    }
    if process.returncode != 0:
        raise fail(f"native process returned {process.returncode}: {route}/{variant}/run{run_number}")
    submission = run_dir / "submission.csv"
    summary = run_dir / "summary.json"
    if not submission.is_file() or not summary.is_file():
        raise fail(f"native artifacts missing: {route}/{variant}/run{run_number}")
    report.update(artifact_report(submission, summary, route, candidate))
    return report


def phase43_control(route: str) -> dict[str, Any]:
    base = PHASE43_ROOT / route / "run1"
    submission = base / "submission.csv"
    summary = base / "summary.json"
    if not submission.is_file() or not summary.is_file():
        raise fail(f"missing Phase43 control artifact: {route}")
    expected = PHASE43_CONTROL[route]
    actual = {"submission": sha256(submission), "summary": sha256(summary)}
    if actual != expected:
        raise fail(f"Phase43 control artifact hash changed: {route}")
    rows = read_prediction(submission, route)
    return {
        "path": relative(base),
        "submission_artifact": {"bytes": submission.stat().st_size, "sha256": actual["submission"], "rows": len(rows)},
        "summary_artifact": {"bytes": summary.stat().st_size, "sha256": actual["summary"]},
        "prediction_keys": [row[0] for row in rows],
    }


def _non_pseudorange_contract(summary: dict[str, Any]) -> dict[str, Any]:
    epochs = summary["epochs"]
    tdcp = summary["tdcp_contract"]
    graph = summary["graph"]
    return {
        "tdcp_factors_built": tdcp["factors_built"],
        "tdcp_factors_inserted": tdcp["factors_inserted"],
        "tdcp_nonfinite_residuals": tdcp["nonfinite_residuals"],
        "tdcp_candidate_pairs": tdcp["candidate_pairs"],
        "tdcp_rejected_gap": tdcp["rejected_gap"],
        "tdcp_rejected_clock_discontinuity": tdcp["rejected_clock_discontinuity"],
        "tdcp_rejected_loss_of_lock": tdcp["rejected_loss_of_lock"],
        "tdcp_rejected_invalid_measurement": tdcp["rejected_invalid_measurement"],
        "imu_intervals": graph["imu_intervals"],
        "receiver_signal_bias_factors": epochs["receiver_signal_bias_factors"],
        "receiver_signal_bias_states": epochs["receiver_signal_bias_states"],
        "residual_ionosphere_factors": epochs["residual_ionosphere_factors"],
        "residual_ionosphere_states": epochs["residual_ionosphere_states"],
        "double_difference_pseudorange_factors": epochs["double_difference_pseudorange_factors"],
        "double_difference_carrier_factors": epochs["double_difference_carrier_factors"],
    }


def _public_report(report: dict[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in report.items() if key != "prediction_keys"}


def run_matrix(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    verify_freeze()
    if not MANIFEST.is_file():
        raise fail("Phase73 structural manifest is missing")
    manifest = load_json(MANIFEST, "Phase73 structural manifest")
    if manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256 or manifest.get("binary", {}).get("sha256") != BINARY_SHA256 or manifest.get("source_commit") != SOURCE_COMMIT or manifest.get("routes") != list(ROUTES):
        raise fail("Phase73 structural manifest pin changed")
    if manifest.get("read_accounting", {}).get("truth_reads") != 0 or manifest.get("read_accounting", {}).get("native_solver_invocations") != 12:
        raise fail("Phase73 manifest read contract changed")
    if manifest.get("evaluator", {}).get("path") != relative(EVALUATOR) or manifest.get("evaluator", {}).get("sha256") != sha256(EVALUATOR):
        raise fail("Phase73 evaluator manifest pin changed")
    output_root = output_root.resolve()
    reject_forbidden(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty structural output: {output_root}")
    input_reports = {route: verify_inputs(route) for route in ROUTES}
    output_root.mkdir(parents=True, exist_ok=True)
    routes: dict[str, Any] = {}
    started = 0
    try:
        for route in ROUTES:
            control = run_case(output_root, route, "control", 1, False)
            started += 1
            control_reference = phase43_control(route)
            control_identity = (
                control["submission_artifact"]["sha256"] == control_reference["submission_artifact"]["sha256"]
                and control["summary_artifact"]["sha256"] == control_reference["summary_artifact"]["sha256"]
                and control["prediction_keys"] == control_reference["prediction_keys"]
            )
            if not control_identity:
                raise fail(f"Phase43 control identity failed: {route}")
            candidate_one = run_case(output_root, route, "candidate", 1, True)
            started += 1
            candidate_two = run_case(output_root, route, "candidate", 2, True)
            started += 1
            repeat_identity = (
                candidate_one["submission_artifact"]["sha256"] == candidate_two["submission_artifact"]["sha256"]
                and candidate_one["summary_artifact"]["sha256"] == candidate_two["summary_artifact"]["sha256"]
                and candidate_one["source_miss_mask_telemetry"] == candidate_two["source_miss_mask_telemetry"]
                and candidate_one["base_telemetry"] == candidate_two["base_telemetry"]
                and candidate_one["prediction_keys"] == candidate_two["prediction_keys"]
            )
            if not repeat_identity:
                raise fail(f"candidate repeat identity failed: {route}")
            candidate_payload = candidate_one["summary_payload"]
            control_payload = control["summary_payload"]
            non_pr_unchanged = _non_pseudorange_contract(candidate_payload) == _non_pseudorange_contract(control_payload)
            domain_exact = candidate_one["prediction_keys"] == control_reference["prediction_keys"] and candidate_two["prediction_keys"] == control_reference["prediction_keys"]
            speed_ok = candidate_one["speed"]["over_70_mps_count"] == 0 and candidate_two["speed"]["over_70_mps_count"] == 0
            route_gates = {
                "prediction_domain_coverage_exact": domain_exact,
                "base_hash_and_bytes_exact": True,
                "all_core_values_finite": True,
                "retained_finite_pc_fraction_exact_1": candidate_one["source_miss_mask_telemetry"]["retained_finite_pc_fraction"] == 1.0,
                "factor_count_consistency": candidate_one["source_miss_mask_telemetry"]["pseudorange_factor_count_consistent"] is True,
                "tdcp_built_equals_inserted": int(candidate_payload["tdcp_contract"]["factors_built"]) == int(candidate_payload["tdcp_contract"]["factors_inserted"]),
                "no_non_pseudorange_population_change": non_pr_unchanged,
                "candidate_repeat_identity": repeat_identity,
                "control_phase43_identity": control_identity,
                "over_70_mps_count_zero": speed_ok,
            }
            if not all(route_gates.values()):
                raise fail(f"Phase73 structural gate failed: {route}: {route_gates}")
            routes[route] = {
                "input": input_reports[route],
                "control": _public_report(control),
                "phase43_control_reference": _public_report(control_reference),
                "candidate_run1": _public_report(candidate_one),
                "candidate_run2": _public_report(candidate_two),
                "prediction_domain_coverage": 1.0,
                "non_pseudorange_contract": _non_pseudorange_contract(candidate_payload),
                "gates": route_gates,
            }
        result = {
            "schema_version": OUTPUT_SCHEMA,
            "phase": 73,
            "execution_label": "Luna Max",
            "status": "go-phase73-source-exact-pseudorange-miss-mask-structural",
            "truth_free": True,
            "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
            "manifest": {"path": relative(MANIFEST), "sha256": sha256(MANIFEST)},
            "evaluator": {"path": relative(EVALUATOR), "sha256": sha256(EVALUATOR)},
            "source_commit": SOURCE_COMMIT,
            "artifact_schema": {"summary_artifact": "path/bytes/sha256 metadata", "summary_payload": "parsed native summary JSON", "summary_key_collision": False},
            "candidate": {"flag": MISS_MASK_FLAG, "required_flags": [BASE_COMP_FLAG, BASE_RINEX_FLAG, BASE_SHA_FLAG], "scope": "adopted undifferenced FGO pseudorange factors only", "formula": "P_rover_corrected_m=P_rover_raw_m-pc_m", "finite_pc_rule": "retain only finite in-domain pc and finite corrected pseudorange", "no_extrapolation_or_endpoint_hold": True, "spp_applied": False, "tdcp_applied": False, "doppler_applied": False},
            "routes": routes,
            "gates": {"all_four_routes": True, "prediction_domain_coverage_exact": True, "raw_and_base_hash_bytes_exact": True, "all_core_values_finite": True, "retained_finite_pc_fraction_each_route": True, "factor_count_consistency_each_route": True, "tdcp_built_equals_inserted": True, "no_non_pseudorange_population_change": True, "candidate_repeat_identity": True, "control_phase43_identity": True, "over_70_mps_count_zero": True, "truth_free": True, "all_passed": True},
            "read_accounting": {"single_process_per_case": True, "routes": 4, "candidate_runs_per_route": 2, "control_runs_per_route": 1, "native_solver_invocations": 12, "raw_device_gnss_process_reads": 12, "raw_device_imu_process_reads": 12, "broadcast_nav_process_reads": 12, "base_rinex_process_reads": 12, "hash_verification_reads": {"device_gnss": 4, "device_imu": 4, "brdc.nav": 4, "base_rinex": 4}, "truth_reads": 0, "mat_reads_or_generated": 0, "validation_holdout_reads": 0, "kaggle_token_reads": 0, "archive_reopens": 0, "post_truth_tuning": False},
            "phase43_champion_preserved": True,
            "phase51_experimental_preserved": True,
            "phase58_experimental_preserved": True,
            "zero_point_782_claim": "not evaluated in truth-free structural phase; separate accuracy freeze required",
        }
        result_path = output_root / "phase73_source_exact_pseudorange_miss_mask_structural_result.json"
        atomic_json(result_path, result)
        output_manifest = {"schema_version": "smartphone-r5-phase73-source-exact-pseudorange-miss-mask-structural-output-manifest.v1", "phase": 73, "status": "sealed-truth-free-structural-matrix", "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "evaluator": {"path": relative(EVALUATOR), "sha256": sha256(EVALUATOR)}, "result": {"path": relative(result_path), "sha256": sha256(result_path), "bytes": result_path.stat().st_size}, "native_solver_invocations": 12, "truth_reads": 0, "all_gates_passed": True}
        atomic_json(output_root / "phase73_source_exact_pseudorange_miss_mask_structural_manifest.json", output_manifest)
        return result
    except Exception as exc:
        failure = {"schema_version": "smartphone-r5-phase73-source-exact-pseudorange-miss-mask-structural-failure.v1", "status": "fail-closed", "exception_type": type(exc).__name__, "error": str(exc), "truth_reads": 0, "native_solver_invocations_started": started, "partial_routes": routes, "phase72_partial_output_reused": False}
        atomic_json(output_root / "phase73_source_exact_pseudorange_miss_mask_structural_failure.json", failure)
        if isinstance(exc, Phase73StructuralError):
            raise
        raise fail(f"unexpected evaluator exception: {type(exc).__name__}: {exc}") from exc


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--run-matrix", action="store_true")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        if args.verify_freeze:
            verify_freeze()
        if args.run_matrix:
            result = run_matrix(args.output_root)
            print(json.dumps({"status": result["status"], "all_gates_passed": result["gates"]["all_passed"], "native_solver_invocations": result["read_accounting"]["native_solver_invocations"], "truth_reads": result["read_accounting"]["truth_reads"]}, sort_keys=True))
        elif not args.verify_freeze:
            parser.error("one of --verify-freeze or --run-matrix is required")
        return 0
    except Phase73StructuralError as exc:
        print(f"phase73 structural failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
