#!/usr/bin/env python3
"""Truth-free Phase80 direct observable-quality structural matrix.

The matrix is candidate-only: two byte-identical native reruns per frozen
Pixel5 route. It composes the Phase78 command recipe with the direct source
P+D quality flag and never opens accuracy truth or runs a control.
"""

from __future__ import annotations

import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
P77_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase77_phase73_signal_bias_composition_structural.py"
_SPEC = importlib.util.spec_from_file_location("phase77_helpers_for_phase80", P77_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load Phase77 helpers: {P77_PATH}")
P77 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(P77)

FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase80_source_exact_direct_p_quality_freeze_v1.json"
FREEZE_SHA256 = "9ebdde21fe7fd955029243d3bdbba3a9f9d3832388d67480760394177c8f685e"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase80_source_exact_direct_p_quality_structural_manifest_v1.json"
EVALUATOR = Path(__file__).resolve()
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
BINARY_SHA256 = "a9498b7aab4a1432af993cd1f3fe45873cf008cf165a6c245993ac77d0fe239c"
SOURCE_COMMIT = "f28aebaf1b41df3ee5e363a8ad94987247321048"
PHASE78_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase78_phase77_structural_reclassification_freeze_v1.json"
PHASE78_FREEZE_SHA256 = "0214783abba6584383085c00824a3c227d7e17fdc041e369fcdd369e48baf897"
PHASE78_RESULT = ROOT / "output/smartphone-r5/phase78-phase77-structural-reclassification-v1/phase78_phase77_structural_reclassification_result.json"
PHASE78_RESULT_SHA256 = "d3d190d8da951afc0a62abcf82d87498a6b8ff1da68cb2154966ca387e357114"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase80-source-exact-direct-observable-quality-structural-v2"
OUTPUT_SCHEMA = "smartphone-r5-phase80-source-exact-direct-observable-quality-structural-result.v1"
DIRECT_FLAG = "--native-source-direct-observable-quality"
ROUTES = P77.ROUTES
INPUTS, BASE_INPUTS = P77.INPUTS, P77.BASE_INPUTS
RAW_INPUT_HASHES, BASE_INPUT_HASHES = P77.RAW_INPUT_HASHES, P77.BASE_INPUT_HASHES
DIRECT_SETTINGS = {ROUTES[0]: ("Highway", 0.2, 0.8), ROUTES[1]: ("Street", 0.1, 0.4), ROUTES[2]: ("Highway", 0.2, 0.8), ROUTES[3]: ("Street", 0.1, 0.4)}
LOCAL_SOURCE_HASHES = {
    "apps/native/gnss_fgo_imu_no_base.cpp": "f75eaf3bb9b1b138396fbb550ef04925cfd2b68cd306f8e01a43147354028cfe",
    "src/algorithms/fgo_gtsam_backend.cpp": "c48c1452a4f31058965bf1e22d9ed41ee54e2f52c5ea5cf22cd14f1b2d8d4dfb",
    "src/algorithms/fgo_problems.cpp": "725167ccc21a62e61c4da9852726ed5cafbfc05ab794eaa76c4641676f576245",
    "include/libgnss++/algorithms/fgo.hpp": "90efe485b2c50eb56535a7eb0709a1b7daa29e4f7a9bd3878acf527dc624ab36",
    "include/libgnss++/algorithms/fgo_config.hpp": "5bf6a54be423a283e42256df2adbd50f4b30083c317bfc6b46cda6a54efd0423",
    "include/libgnss++/algorithms/signal_bias_contract.hpp": "cc54e29fe2c4cd14a243e9a16ec97c2767a9fb6c5ce96c57abebe7ee3b503af4",
    "include/libgnss++/core/signal_policy.hpp": "d3f8edbdd785f0292c2b8a1596248c00a2b5e9284c09de439e5c7340c1dbca99",
}


class Phase80StructuralError(ValueError):
    """Raised when a frozen Phase80 structural contract fails."""


def fail(message: str) -> Phase80StructuralError:
    return Phase80StructuralError(message)


reject_forbidden = P77.reject_forbidden
sha256 = P77.sha256
load_json = P77.load_json
atomic_write = P77.atomic_write
atomic_json = P77.atomic_json
relative = P77.relative
input_paths = P77.input_paths
read_prediction = P77.read_prediction
speed_report = P77.speed_report


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise fail(f"missing/nonfinite number: {label}")
    return float(value)


def _int(mapping: dict[str, Any], key: str, label: str, minimum: int = 0) -> int:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise fail(f"invalid count: {label}/{key}")
    return value


def verify_inputs(route: str) -> dict[str, Any]:
    return P77.verify_inputs(route)


def _validate_base(summary: dict[str, Any], route: str) -> tuple[dict[str, Any], dict[str, Any]]:
    P77.P73._validate_common_summary(summary, route)
    base, miss = summary.get("native_base_pseudorange_compensation"), summary.get("native_base_pseudorange_source_miss_mask")
    if not isinstance(base, dict) or not isinstance(miss, dict):
        raise fail(f"base/miss-mask telemetry missing: {route}")
    pin = BASE_INPUT_HASHES[route]
    if base.get("enabled") is not True or base.get("built") is not True or base.get("applied") is not True:
        raise fail(f"base compensation not applied: {route}")
    if base.get("preserve_additional_frequency_bands") is not False:
        raise fail(f"additional base bands are unexpectedly enabled: {route}")
    if base.get("base_member_sha256") != pin["sha256"] or base.get("base_rinex_sha256") != pin["sha256"] or base.get("base_rinex_bytes") != pin["bytes"]:
        raise fail(f"base hash/bytes mismatch: {route}")
    xyz = base.get("base_coordinate_xyz_m")
    if not isinstance(xyz, list) or len(xyz) != 3 or any(abs(float(a) - float(b)) > 1e-6 for a, b in zip(xyz, pin["xyz"])):
        raise fail(f"base coordinates mismatch: {route}")
    if abs(_number(base.get("observed_interval_s"), f"base/{route}") - pin["dt_s"]) > 1e-6 or base.get("moving_mean_samples") != pin["window"]:
        raise fail(f"base interval/window mismatch: {route}")
    if any(base.get(key) is not True for key in ("same_satellite_signal_only", "no_extrapolation_or_endpoint_hold")) or any(base.get(key) is not False for key in ("spp_applied", "tdcp_applied", "doppler_applied")):
        raise fail(f"base scope mismatch: {route}")
    if miss.get("enabled") is not True or miss.get("retained_factor_epoch_indices_unchanged") is not True or miss.get("tdcp_doppler_imu_spp_unchanged") is not True:
        raise fail(f"miss-mask scope mismatch: {route}")
    original, retained = _int(miss, "original_adopted_pseudorange_rows", route, 1), _int(miss, "retained_finite_pc_pseudorange_rows", route, 1)
    drops = [_int(miss, key, route) for key in ("dropped_missing_exact_stream_rows", "dropped_out_of_domain_rows", "dropped_nonfinite_correction_rows")]
    if original != retained + sum(drops) or miss.get("retained_finite_pc_fraction") != 1.0:
        raise fail(f"miss-mask accounting mismatch: {route}")
    if miss.get("pseudorange_factor_count_consistent") is not True or miss.get("pseudorange_factors_inserted") != retained:
        raise fail(f"miss-mask factor count mismatch: {route}")
    for key in ("retained_over_original_fraction", "correction_abs_p50_m", "correction_abs_p95_m", "correction_abs_max_m"):
        _number(miss.get(key), f"miss/{route}/{key}")
    if base.get("adopted_pseudorange_rows") != original or base.get("adopted_rows_corrected") != retained:
        raise fail(f"base/miss adopted counts mismatch: {route}")
    return base, miss


def _validate_signal_bias(summary: dict[str, Any], route: str) -> dict[str, Any]:
    if summary.get("native_signal_bias_states") is not True:
        raise fail(f"signal-bias flag telemetry missing: {route}")
    epochs = summary.get("epochs")
    if not isinstance(epochs, dict):
        raise fail(f"signal-bias epoch telemetry missing: {route}")
    factors, states = _int(epochs, "receiver_signal_bias_factors", route, 1), _int(epochs, "receiver_signal_bias_states", route, 1)
    if factors < states:
        raise fail(f"signal-bias factor/state materiality failed: {route}")
    estimates = summary.get("receiver_signal_bias_estimates_m")
    if not isinstance(estimates, dict) or len(estimates) != states:
        raise fail(f"signal-bias estimate count mismatch: {route}")
    for key, value in estimates.items():
        _number(value, f"signal-bias/{route}/{key}")
    return {"factors": factors, "states": states, "estimates_m": estimates}


def _validate_direct(summary: dict[str, Any], route: str) -> dict[str, Any]:
    if summary.get("native_source_direct_observable_quality_enabled") is not True or summary.get("native_upstream_quality") is not False:
        raise fail(f"direct quality top-level identity failed: {route}")
    if summary.get("native_pdc_state_bridge") is not False or summary.get("native_pdc_imu_tdcp_no_bridge") is not True:
        raise fail(f"PDC bridge identity failed: {route}")
    direct, upstream = summary.get("native_source_direct_observable_quality"), summary.get("upstream_observable_quality")
    if not isinstance(direct, dict) or not isinstance(upstream, dict):
        raise fail(f"direct quality object missing: {route}")
    env, p_sigma, d_sigma = DIRECT_SETTINGS[route]
    if any(direct.get(key) != value for key, value in (("enabled", True), ("direct_no_pdc", True), ("pdc_bridge", False), ("native_pdc_state_bridge", False), ("environment", env))):
        raise fail(f"direct quality/PDC object mismatch: {route}")
    huber = direct.get("p_and_d_huber")
    if not isinstance(huber, dict) or huber.get("pseudorange_sigma") != p_sigma or huber.get("doppler_sigma") != d_sigma:
        raise fail(f"direct P+D Huber mapping mismatch: {route}")
    config = direct.get("config")
    expected_config = {"use_upstream_observable_quality": True, "upstream_snr_percentile": 85.0, "upstream_min_snr_dbhz": 20.0, "upstream_min_elevation_deg": 5.0, "upstream_max_adjacent_gap_s": 1.5, "pseudorange_huber_threshold_sigma": p_sigma, "undifferenced_doppler_huber_threshold_sigma": d_sigma, "tdcp_sigma_m_unchanged": 0.03, "pseudorange_sigma_contract": "snr_scale*signal_type_factor", "doppler_sigma_contract": "snr_scale/12", "adjacent_mask_contract": "applyAdjacentMasks Pmask_dDP/Lmask_dDL unchanged", "pseudorange_residual_screen": "Pmask_res L1=20m/L5=15m", "doppler_residual_screen": "Dmask_res 3m/s; non-initializer"}
    if config != expected_config:
        raise fail(f"direct quality config mismatch: {route}")
    if any(upstream.get(key) != value for key, value in (("enabled", True), ("snr_percentile", 85.0), ("snr_denominator_db", 20.0), ("min_snr_dbhz", 20.0), ("min_elevation_deg", 5.0), ("max_adjacent_gap_s", 1.5), ("tdcp_sigma_m_unchanged", 0.03), ("pseudorange_sigma_contract", "snr_scale*signal_type_factor"), ("doppler_sigma_contract", "snr_scale/12"))):
        raise fail(f"upstream quality contract mismatch: {route}")
    counts = direct.get("counts")
    if not isinstance(counts, dict):
        raise fail(f"direct quality counts missing: {route}")
    keys = ("pseudorange_candidates", "pseudorange_factors", "doppler_candidates", "doppler_factors", "doppler_graph_factors", "pseudorange_residual_rejections", "doppler_residual_rejections")
    for key in keys:
        value = _int(counts, key, route)
        if value != _int(upstream, key, route):
            raise fail(f"direct/upstream count mismatch: {route}/{key}")
    if counts["pseudorange_candidates"] < counts["pseudorange_factors"] or counts["doppler_candidates"] < counts["doppler_factors"] or counts["pseudorange_factors"] < 1 or counts["doppler_graph_factors"] < 1:
        raise fail(f"direct quality count materiality failed: {route}")
    return {"object": direct, "upstream": upstream, "counts": counts}


def _population(summary: dict[str, Any]) -> dict[str, Any]:
    epochs, tdcp, graph, quality = (summary.get(key) for key in ("epochs", "tdcp_contract", "graph", "upstream_observable_quality"))
    if not all(isinstance(item, dict) for item in (epochs, tdcp, graph, quality)):
        raise fail("population telemetry missing")
    return {"pseudorange_factors": epochs["pseudorange_factors"], "tdcp_factors_built": tdcp["factors_built"], "tdcp_factors_inserted": tdcp["factors_inserted"], "undifferenced_doppler_factors_inserted": quality["doppler_graph_factors"], "imu_intervals": graph["imu_intervals"], "output_epochs": epochs["output"]}


def validate_summary(path: Path, route: str) -> dict[str, Any]:
    summary = load_json(path, "Phase80 native summary")
    base, miss = _validate_base(summary, route)
    signal, direct = _validate_signal_bias(summary, route), _validate_direct(summary, route)
    return {"summary": summary, "base": base, "source_miss_mask": miss, "signal_bias": signal, "direct_quality": direct, "population": _population(summary)}


def artifact_report(submission: Path, summary: Path, route: str) -> dict[str, Any]:
    rows = read_prediction(submission, route)
    speed = speed_report(rows)
    if not speed["finite"] or speed["over_70_mps_count"] != 0:
        raise fail(f"speed/finite gate failed: {route}")
    diagnostics = validate_summary(summary, route)
    return {"submission_artifact": {"path": relative(submission), "bytes": submission.stat().st_size, "sha256": sha256(submission), "rows": len(rows)}, "summary_artifact": {"path": relative(summary), "bytes": summary.stat().st_size, "sha256": sha256(summary)}, "summary_payload": diagnostics["summary"], "prediction_keys": [row[0] for row in rows], "speed": speed, "base_telemetry": diagnostics["base"], "source_miss_mask_telemetry": diagnostics["source_miss_mask"], "signal_bias": diagnostics["signal_bias"], "direct_quality": diagnostics["direct_quality"], "population": diagnostics["population"]}


def native_command(route: str, run_dir: Path) -> list[str]:
    command = P77.native_command(route, run_dir, True)
    command.append(DIRECT_FLAG)
    for token in command:
        reject_forbidden(token)
    return command


def verify_freeze() -> dict[str, Any]:
    if sha256(FREEZE) != FREEZE_SHA256:
        raise fail("Phase80 freeze hash changed")
    freeze = load_json(FREEZE, "Phase80 freeze")
    if freeze.get("schema_version") != "smartphone-r5-phase80-source-exact-direct-p-quality-freeze.v1" or freeze.get("status") != "frozen-before-phase80-raw-read":
        raise fail("Phase80 freeze schema/status changed")
    candidate = freeze.get("candidate", {})
    if any(candidate.get(key) != value for key, value in (("opt_in", True), ("default_off", True), ("direct_no_pdc", True), ("pdc_state_bridge", False), ("source_quality_flag", DIRECT_FLAG), ("direct_flag_requires", ["--native-pdc-imu-tdcp-no-bridge"]))):
        raise fail("Phase80 candidate contract changed")
    expected_flags = list(P77.BASE_FLAGS) + [P77.SIGNAL_BIAS_FLAG, P77.BASE_COMP_FLAG, P77.BASE_RINEX_FLAG, P77.BASE_SHA_FLAG, P77.MISS_MASK_FLAG]
    if candidate.get("phase78_flags_preserved") != expected_flags:
        raise fail("Phase78 flag recipe changed")
    if freeze.get("cohort", {}).get("route_order") != list(ROUTES):
        raise fail("Phase80 route order changed")
    source = freeze.get("source_quality_contract", {})
    if any(source.get(key) != value for key, value in (("doppler_model", "SNR"), ("D_sn_ratio", "1/12"), ("Dmask_res_mps", 3.0), ("doppler_residual_screen_applies_to_initialization", False), ("whole_contract_includes_p_and_d", True))):
        raise fail("Phase80 source quality contract changed")
    for route, (env, p_sigma, d_sigma) in DIRECT_SETTINGS.items():
        values = freeze.get("public_settings_huber", {}).get("by_route", {}).get(route, {})
        if (values.get("environment"), values.get("pseudorange_huber_threshold_sigma"), values.get("doppler_huber_threshold_sigma")) != (env, p_sigma, d_sigma):
            raise fail(f"Phase80 route Huber mapping changed: {route}")
    gates = freeze.get("truth_free_structural_contract", {})
    for key in ("truth_free", "candidate_repeat_submission_and_summary_byte_identical", "candidate_all_coordinates_finite_and_earth_valid", "candidate_converged", "source_hashes_exact", "phase78_composition_identity", "direct_no_pdc_identity", "p85_snr_sigma_times_signal_identity", "whole_p_and_d_quality_contract_identity", "doppler_snr_sigma_divide_12_identity", "doppler_Dmask_res_3mps_non_initializer_identity", "minimum_snr_and_elevation_identity", "pseudorange_residual_screen_identity", "existing_adjacent_p_d_mask_identity", "public_settings_huber_identity", "no_route_specific_selection", "all_gates_anded"):
        if gates.get(key) is not True:
            raise fail(f"Phase80 gate declaration changed: {key}")
    matrix = freeze.get("structural_matrix", {})
    expected = {"candidate_runs_per_route": 2, "candidate_runs_total": 8, "native_solver_invocations": 8, "raw_device_gnss_reads": 8, "raw_device_imu_reads": 8, "broadcast_nav_reads": 8, "truth_reads": 0, "mat_reads_or_generated": 0, "precomputed_coordinate_reads": 0, "kaggle_or_token_access": 0, "new_output_root_required": True}
    if any(matrix.get(key) != value for key, value in expected.items()):
        raise fail("Phase80 matrix/read contract changed")
    authority = freeze.get("authority", {})
    if authority.get("official_source", {}).get("commit") != "29923f9f370f09ebc00f96d8cca375007a18e7d5":
        raise fail("Phase80 official source pin changed")
    for path, digest in ((PHASE78_FREEZE, PHASE78_FREEZE_SHA256), (PHASE78_RESULT, PHASE78_RESULT_SHA256), (BINARY, BINARY_SHA256)):
        if sha256(path) != digest:
            raise fail(f"Phase80 pinned artifact changed: {path}")
    for path, digest in LOCAL_SOURCE_HASHES.items():
        if sha256(ROOT / path) != digest:
            raise fail(f"Phase80 implementation source changed: {path}")
    if not MANIFEST.is_file():
        raise fail("Phase80 structural manifest is missing")
    manifest = load_json(MANIFEST, "Phase80 structural manifest")
    if manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256 or manifest.get("source_commit") != SOURCE_COMMIT or manifest.get("routes") != list(ROUTES):
        raise fail("Phase80 manifest freeze/source/routes pin changed")
    if manifest.get("binary", {}).get("sha256") != BINARY_SHA256 or manifest.get("matrix", {}).get("native_invocations") != 8 or manifest.get("matrix", {}).get("truth_reads") != 0 or manifest.get("matrix", {}).get("new_output_root") != relative(DEFAULT_OUTPUT):
        raise fail("Phase80 manifest matrix/binary pin changed")
    if manifest.get("evaluator", {}).get("path") != relative(EVALUATOR) or manifest.get("evaluator", {}).get("sha256") != sha256(EVALUATOR):
        raise fail("Phase80 evaluator manifest pin changed")
    return freeze


def run_case(output_root: Path, route: str, run_number: int) -> dict[str, Any]:
    run_dir = output_root / route / "candidate" / f"run{run_number}"
    if run_dir.exists():
        raise fail(f"refusing to overwrite output: {run_dir}")
    run_dir.mkdir(parents=True, exist_ok=True)
    command = native_command(route, run_dir)
    environment = os.environ.copy()
    environment["LD_LIBRARY_PATH"] = "/home/sasaki/.local/lib" + ((":" + environment["LD_LIBRARY_PATH"]) if environment.get("LD_LIBRARY_PATH") else "")
    started = time.perf_counter()
    try:
        process = subprocess.run(command, cwd=ROOT, env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False, timeout=1800)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise fail(f"native process failed: {route}/run{run_number}: {exc}") from exc
    atomic_write(run_dir / "run.log", (process.stdout or "").encode())
    report = {"candidate": True, "run": run_number, "return_code": process.returncode, "wall_seconds": time.perf_counter() - started, "command": command, "log": {"path": relative(run_dir / "run.log"), "sha256": sha256(run_dir / "run.log")}}
    if process.returncode != 0:
        raise fail(f"native process returned {process.returncode}: {route}/run{run_number}")
    submission, summary = run_dir / "submission.csv", run_dir / "summary.json"
    if not submission.is_file() or not summary.is_file():
        raise fail(f"native artifacts missing: {route}/run{run_number}")
    report.update(artifact_report(submission, summary, route))
    return report


def _public(report: dict[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in report.items() if key != "prediction_keys"}


def run_matrix(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    verify_freeze()
    output_root = output_root.resolve()
    reject_forbidden(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty output: {output_root}")
    inputs = {route: verify_inputs(route) for route in ROUTES}
    output_root.mkdir(parents=True, exist_ok=True)
    routes, errors, started = {}, [], 0
    try:
        for route in ROUTES:
            cases = {}
            for run_number in (1, 2):
                started += 1
                try:
                    cases[f"candidate_run{run_number}"] = run_case(output_root, route, run_number)
                except Exception as exc:
                    cases[f"candidate_run{run_number}"] = {"run": run_number, "candidate": True, "error_type": type(exc).__name__, "error": str(exc)}
                    errors.append({"route": route, **cases[f"candidate_run{run_number}"]})
            record = {"input": inputs[route], "cases": {key: _public(value) for key, value in cases.items()}}
            if any("error" in value for value in cases.values()):
                record["gates"] = {"case_execution": False, "all_route_gates": False}
                routes[route] = record
                continue
            first, second = cases["candidate_run1"], cases["candidate_run2"]
            repeat = first["submission_artifact"]["sha256"] == second["submission_artifact"]["sha256"] and first["summary_artifact"]["sha256"] == second["summary_artifact"]["sha256"] and first["prediction_keys"] == second["prediction_keys"] and first["direct_quality"] == second["direct_quality"] and first["base_telemetry"] == second["base_telemetry"] and first["source_miss_mask_telemetry"] == second["source_miss_mask_telemetry"] and first["signal_bias"] == second["signal_bias"]
            domain = first["submission_artifact"]["rows"] + 1 == first["population"]["output_epochs"] == len(first["prediction_keys"]) + 1 and second["submission_artifact"]["rows"] + 1 == second["population"]["output_epochs"]
            populations = first["population"] == second["population"]
            tdcp = first["population"]["tdcp_factors_built"] == first["population"]["tdcp_factors_inserted"] and first["summary_payload"]["tdcp_contract"]["nonfinite_residuals"] == 0
            gates = {"prediction_domain_coverage_exact": domain, "candidate_direct_quality_identity": True, "candidate_no_pdc": True, "base_phase78_frequency_policy": True, "miss_mask_contract": True, "signal_bias_material": True, "candidate_converged": first["summary_payload"]["graph"]["converged"] is True, "candidate_imu_epoch_invariants": populations, "candidate_tdcp_built_equals_inserted": tdcp, "candidate_repeat_identity": repeat, "candidate_speed_finite": first["speed"]["finite"] and second["speed"]["finite"], "over_70_mps_count_zero": first["speed"]["over_70_mps_count"] == 0 and second["speed"]["over_70_mps_count"] == 0}
            gates["all_route_gates"] = all(gates.values())
            record.update({"candidate_run1": _public(first), "candidate_run2": _public(second), "prediction_domain_coverage": 1.0 if domain else 0.0, "population_contract": first["population"], "gates": gates})
            routes[route] = record
        all_passed = not errors and bool(routes) and all(value.get("gates", {}).get("all_route_gates") is True for value in routes.values())
        if not all_passed:
            failure = {"schema_version": OUTPUT_SCHEMA.replace("result", "failure"), "status": "fail-closed", "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "manifest": {"path": relative(MANIFEST), "sha256": sha256(MANIFEST)}, "evaluator": {"path": relative(EVALUATOR), "sha256": sha256(EVALUATOR)}, "routes": routes, "errors": errors, "truth_reads": 0, "native_solver_invocations_started": started, "raw_device_gnss_process_reads": started, "raw_device_imu_process_reads": started, "broadcast_nav_process_reads": started, "base_rinex_process_reads": started}
            atomic_json(output_root / "phase80_direct_observable_quality_structural_failure.json", failure)
            raise fail(f"Phase80 structural gates failed; output={relative(output_root)}")
        result = {"schema_version": OUTPUT_SCHEMA, "phase": 80, "execution_label": "Luna Max", "status": "go-phase80-direct-observable-quality-structural", "truth_free": True, "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "manifest": {"path": relative(MANIFEST), "sha256": sha256(MANIFEST)}, "evaluator": {"path": relative(EVALUATOR), "sha256": sha256(EVALUATOR)}, "source_commit": SOURCE_COMMIT, "candidate": {"flag": DIRECT_FLAG, "phase78_flags_preserved": list(P77.BASE_FLAGS) + [P77.SIGNAL_BIAS_FLAG, P77.BASE_COMP_FLAG, P77.BASE_RINEX_FLAG, P77.BASE_SHA_FLAG, P77.MISS_MASK_FLAG], "runs_per_route": 2, "controls": 0, "direct_no_pdc": True, "preserve_additional_frequency_bands": False}, "routes": routes, "gates": {"all_four_routes": True, "prediction_domain_coverage_exact": True, "direct_quality_identity": True, "no_pdc": True, "base_phase78_frequency_policy": True, "miss_mask_contract": True, "signal_bias_material": True, "candidate_converged": True, "imu_epoch_invariants": True, "tdcp_built_equals_inserted": True, "candidate_repeat_identity": True, "speed_finite": True, "over_70_mps_count_zero": True, "truth_free": True, "all_passed": True}, "read_accounting": {"candidate_runs_per_route": 2, "control_runs_per_route": 0, "native_solver_invocations": 8, "raw_device_gnss_process_reads": 8, "raw_device_imu_process_reads": 8, "broadcast_nav_reads": 8, "base_rinex_reads": 8, "truth_reads": 0, "mat_reads_or_generated": 0, "precomputed_coordinate_reads": 0, "kaggle_or_token_access": 0}}
        result_path = output_root / "phase80_direct_observable_quality_structural_result.json"
        atomic_json(result_path, result)
        atomic_json(output_root / "phase80_direct_observable_quality_structural_manifest.json", {"schema_version": "smartphone-r5-phase80-direct-observable-quality-structural-output-manifest.v1", "phase": 80, "status": "sealed-truth-free-structural-matrix", "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "evaluator": {"path": relative(EVALUATOR), "sha256": sha256(EVALUATOR)}, "result": {"path": relative(result_path), "sha256": sha256(result_path), "bytes": result_path.stat().st_size}, "native_solver_invocations": 8, "truth_reads": 0, "all_gates_passed": True})
        return result
    except Phase80StructuralError:
        raise
    except Exception as exc:
        atomic_json(output_root / "phase80_direct_observable_quality_structural_exception.json", {"schema_version": OUTPUT_SCHEMA.replace("result", "exception"), "status": "fail-closed", "exception_type": type(exc).__name__, "error": str(exc), "routes": routes, "truth_reads": 0, "native_solver_invocations_started": started})
        raise fail(f"unexpected Phase80 evaluator exception: {type(exc).__name__}: {exc}") from exc


def main(argv: list[str] | None = None) -> int:
    import argparse
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
            print(json.dumps({"status": result["status"], "all_gates_passed": result["gates"]["all_passed"], "native_solver_invocations": 8, "truth_reads": 0}, sort_keys=True))
        elif not args.verify_freeze:
            parser.error("one of --verify-freeze or --run-matrix is required")
        return 0
    except Phase80StructuralError as exc:
        print(f"phase80 structural failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
