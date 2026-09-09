#!/usr/bin/env python3
"""Truth-free Phase85 structural matrix for the source-exact C0/D clock row.

The candidate is the frozen Phase80 direct/no-PDC recipe plus one opt-in
clock-factor flag.  It runs two native candidate repetitions for each of the
four pinned Pixel5 routes, with no control lane and no accuracy calculation.
All source, inherited-contract, command, telemetry, domain, repeatability,
finite-earth, convergence, and read-accounting gates are fail-closed.
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
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
PHASE80_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase80_direct_observable_quality_structural.py"
_SPEC = importlib.util.spec_from_file_location("phase80_helpers_for_phase85", PHASE80_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load Phase80 helper: {PHASE80_PATH}")
P80 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(P80)


PHASE84_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase84_source_exact_clock_c0d_freeze_v1.json"
PHASE84_FREEZE_SHA256 = "c20d8849257cf10afdcea9e57d7299ed595c2c045a1b555241a40430e077cca4"
PHASE83_AUDIT = ROOT / "docs/use_cases/records/smartphone_r5_phase83_native_clock_factor_gap_audit_v1.md"
PHASE83_AUDIT_SHA256 = "291ee545f5d022f56cfb357ec97c2c62aaec72df6d39c0c3cc82ce41459299fd"
PHASE83_AUDIT_COMMIT = "7f48778b86c810b0a1cbfea710c12456a691e8ca"
PHASE80_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase80_source_exact_direct_p_quality_freeze_v1.json"
PHASE80_FREEZE_SHA256 = "9ebdde21fe7fd955029243d3bdbba3a9f9d3832388d67480760394177c8f685e"
PHASE80_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase80_source_exact_direct_p_quality_structural_manifest_v1.json"
PHASE80_MANIFEST_SHA256 = "6d20541560b726ed2409cd05902e5105874ac72e113af0c3812f029130df310f"
PHASE78_RESULT = ROOT / "output/smartphone-r5/phase78-phase77-structural-reclassification-v1/phase78_phase77_structural_reclassification_result.json"
PHASE78_RESULT_SHA256 = "d3d190d8da951afc0a62abcf82d87498a6b8ff1da68cb2154966ca387e357114"
PHASE85_V1_FAILURE = ROOT / "output/smartphone-r5/phase85-source-clock-c0d-structural-v1/phase85_source_clock_c0d_structural_failure.json"
PHASE85_V1_FAILURE_SHA256 = "c22014dc4b9291f44c0a7ee073cd62212ab6cce366bf1aa3e71e784d97bd7b27"
PHASE85_V1_FAILURE_BYTES = 10331
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase85_source_clock_c0d_structural_manifest_v1.json"
EVALUATOR = Path(__file__).resolve()
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase85_source_clock_c0d_structural.py"
TESTS_CMAKE = ROOT / "tests/CMakeLists.txt"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
BINARY_SHA256 = "0a2f70f2debdc34b47f9e185939cdb02c839b4133c40634c33e6eb77b2c4e3d5"
SOURCE_COMMIT = "f77f0086378b9b5bce0d51a543529343a4e11cb5"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase85-source-clock-c0d-structural-v2"
OUTPUT_SCHEMA = "smartphone-r5-phase85-source-clock-c0d-structural-result.v1"
CLOCK_FLAG = "--native-source-clock-c0d-factor"
DIRECT_FLAG = "--native-source-direct-observable-quality"
NO_BRIDGE_FLAG = "--native-pdc-imu-tdcp-no-bridge"
LEGACY_QUALITY_FLAG = "--native-upstream-quality"
PRESERVE_BANDS_FLAG = "--native-base-pseudorange-preserve-additional-frequency-bands"
SPEED_OF_LIGHT_MPS = 299792458.0
C0D_SIGMA_SECONDS = 0.1 / SPEED_OF_LIGHT_MPS

ROUTES = P80.ROUTES
INPUTS, BASE_INPUTS = P80.INPUTS, P80.BASE_INPUTS
RAW_INPUT_HASHES, BASE_INPUT_HASHES = P80.RAW_INPUT_HASHES, P80.BASE_INPUT_HASHES
DOMAIN_ROWS = {
    ROUTES[0]: 2158,
    ROUTES[1]: 3139,
    ROUTES[2]: 1465,
    ROUTES[3]: 1101,
}

PHASE84_SOURCE_HASHES = {
    "apps/native/gnss_fgo_imu_no_base.cpp": "89688fc5ce16402c66a15c6f373185fbcc3e4a1b1c6d9b7e7b691bf95b051189",
    "include/libgnss++/algorithms/fgo.hpp": "79048bba9da2edac466a06241df3ac05835b06a39513f394271163183b13425f",
    "include/libgnss++/algorithms/fgo_config.hpp": "4b66fd4b63b4864f126d0c7d15316b60f957007bae4c4fe123b2b442cf0dc468",
    "src/algorithms/fgo.cpp": "dbf2e9e08cb160f5f650d2fc78f2e99771878266d356d2d7147c6ce1afa091f5",
    "src/algorithms/fgo_gtsam_backend.cpp": "4075156d10189f7f16df8e3f570106c73e0635fa2a99ef084f34295fda88afbd",
    "src/algorithms/fgo_gtsam_internal.hpp": "90db67fe9dd8d032c3d28d37de6c161966eb3cd5ca8fb873d98a47fa6181859b",
    "tests/test_fgo_gtsam_backend.cpp": "3472805572b42c8f3bd65ed32c822bd80c142a8c6100a4e6dd99e5e67d727f35",
}

OFFICIAL_SOURCE_PINS = {
    "fgo_gnss_imu_m": {
        "path": "output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m",
        "sha256": "c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3",
        "lines": ["102-114", "169-186", "189-218", "252-277"],
    },
    "parameters_m": {
        "path": "output/reproducibility-cache/gsdc2023/parameters.m",
        "sha256": "518925e9c75c7a14fceb5cd99432fe883311e0d64c72b94396744ac765120f52",
        "lines": ["46-47", "130-133"],
    },
    "clock_factor_ccdd_h": {
        "path": "output/reproducibility-cache/gtsam_gnss/src/ClockFactor_CCDD.h",
        "sha256": "7c174217218aa0059155e1e5a7182c69d56a3cd389d0ebd95c242d6d8c65b9fc",
        "lines": ["16-20", "32-61"],
    },
}


class Phase85StructuralError(ValueError):
    """Raised when a frozen Phase85 structural contract fails."""


def fail(message: str) -> Phase85StructuralError:
    return Phase85StructuralError(message)


def reject_forbidden(path: Path | str) -> None:
    P80.reject_forbidden(path)


def sha256(path: Path) -> str:
    return P80.sha256(path)


def load_json(path: Path, label: str) -> dict[str, Any]:
    return P80.load_json(path, label)


def atomic_write(path: Path, payload: bytes) -> None:
    P80.atomic_write(path, payload)


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    P80.atomic_json(path, value)


def relative(path: Path) -> str:
    return P80.relative(path)


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise fail(f"missing/nonfinite number: {label}")
    return float(value)


def _int(mapping: dict[str, Any], key: str, label: str, minimum: int = 0) -> int:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise fail(f"invalid count: {label}/{key}")
    return value


def _pinned_json(path: Path, expected: str, label: str) -> dict[str, Any]:
    if sha256(path) != expected:
        raise fail(f"{label} hash changed")
    return load_json(path, label)


def _check_file_hash(path: Path, expected: str, label: str) -> None:
    if sha256(path) != expected:
        raise fail(f"{label} hash changed: {relative(path)}")


def _phase78_domain_rows() -> dict[str, int]:
    result = _pinned_json(PHASE78_RESULT, PHASE78_RESULT_SHA256, "Phase78 structural result")
    if result.get("truth_free") is not True or result.get("read_accounting", {}).get("truth_reads") != 0:
        raise fail("Phase78 provenance is not truth-free")
    rows: dict[str, int] = {}
    for route in ROUTES:
        value = result.get("routes", {}).get(route, {}).get("prediction_domain", {}).get("rows")
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise fail(f"Phase78 route row count missing: {route}")
        rows[route] = value
    if rows != DOMAIN_ROWS:
        raise fail("Phase78 domain rows differ from the frozen Phase80/78 pin")
    return rows


def _verify_phase80_contract() -> None:
    freeze = _pinned_json(PHASE80_FREEZE, PHASE80_FREEZE_SHA256, "Phase80 freeze")
    if freeze.get("schema_version") != "smartphone-r5-phase80-source-exact-direct-p-quality-freeze.v1" or freeze.get("status") != "frozen-before-phase80-raw-read":
        raise fail("Phase80 freeze schema/status changed")
    expected_flags = list(P80.P77.BASE_FLAGS) + [P80.P77.SIGNAL_BIAS_FLAG, P80.P77.BASE_COMP_FLAG, P80.P77.BASE_RINEX_FLAG, P80.P77.BASE_SHA_FLAG, P80.P77.MISS_MASK_FLAG]
    candidate = freeze.get("candidate", {})
    if candidate.get("source_quality_flag") != DIRECT_FLAG or candidate.get("default_off") is not True or candidate.get("direct_no_pdc") is not True or candidate.get("pdc_state_bridge") is not False or candidate.get("phase78_flags_preserved") != expected_flags:
        raise fail("Phase80 direct/no-PDC recipe changed")
    matrix = freeze.get("structural_matrix", {})
    for key, value in (("candidate_runs_per_route", 2), ("candidate_runs_total", 8), ("native_solver_invocations", 8), ("truth_reads", 0), ("mat_reads_or_generated", 0), ("precomputed_coordinate_reads", 0), ("kaggle_or_token_access", 0)):
        if matrix.get(key) != value:
            raise fail(f"Phase80 matrix pin changed: {key}")
    manifest = _pinned_json(PHASE80_MANIFEST, PHASE80_MANIFEST_SHA256, "Phase80 structural manifest")
    if manifest.get("freeze", {}).get("sha256") != PHASE80_FREEZE_SHA256 or manifest.get("routes") != list(ROUTES) or manifest.get("matrix", {}).get("native_invocations") != 8 or manifest.get("matrix", {}).get("truth_reads") != 0:
        raise fail("Phase80 structural manifest contract changed")
    if manifest.get("candidate", {}).get("phase78_flags_preserved") != expected_flags or manifest.get("candidate", {}).get("controls") != 0 or manifest.get("candidate", {}).get("preserve_additional_frequency_bands") is not False:
        raise fail("Phase80 structural candidate recipe changed")


def verify_freeze() -> dict[str, Any]:
    freeze = _pinned_json(PHASE84_FREEZE, PHASE84_FREEZE_SHA256, "Phase84 freeze")
    if freeze.get("schema_version") != "smartphone-r5-phase84-source-exact-clock-c0d-freeze.v1" or freeze.get("status") != "implementation-freeze-before-code-or-raw-execution":
        raise fail("Phase84 freeze schema/status changed")
    authority = freeze.get("authority", {})
    if authority.get("phase83_audit", {}).get("sha256") != PHASE83_AUDIT_SHA256 or authority.get("phase83_audit", {}).get("git_commit") != PHASE83_AUDIT_COMMIT:
        raise fail("Phase83 audit pin changed")
    _check_file_hash(PHASE83_AUDIT, PHASE83_AUDIT_SHA256, "Phase83 audit")
    if authority.get("phase80_freeze", {}).get("sha256") != PHASE80_FREEZE_SHA256 or authority.get("phase80_structural_manifest", {}).get("sha256") != PHASE80_MANIFEST_SHA256:
        raise fail("Phase80 authority pin changed")
    if authority.get("official_commit") != "29923f9f370f09ebc00f96d8cca375007a18e7d5" or authority.get("official_repository") != "taroz/gsdc2023":
        raise fail("official repository pin changed")

    official = freeze.get("official_source_pins", {})
    for name, expected in OFFICIAL_SOURCE_PINS.items():
        actual = official.get(name, {})
        if any(actual.get(key) != expected[key] for key in ("path", "sha256", "lines")):
            raise fail(f"official source pin changed: {name}")
        _check_file_hash(ROOT / expected["path"], expected["sha256"], f"official source {name}")

    candidate = freeze.get("candidate_contract", {})
    if candidate.get("flag") != CLOCK_FLAG or candidate.get("default_off") is not True or candidate.get("implementation_status_at_freeze") != "not implemented":
        raise fail("Phase84 candidate identity changed")
    if candidate.get("requires") != [DIRECT_FLAG, NO_BRIDGE_FLAG] or candidate.get("backend_scope", {}).get("selected") != "GTSAM/Pose3/IMU path only":
        raise fail("Phase84 dependency/backend contract changed")
    if candidate.get("edge_gate", {}).get("dt_condition") != "0 < dt < 1.5 seconds" or candidate.get("edge_gate", {}).get("source_excluded_phones") != ["sm-a205u", "sm-a505u", "samsunga325g"] or candidate.get("edge_gate", {}).get("pixel5_result") != "Pixel5 is not source-excluded, so eligible non-jump edges are active":
        raise fail("Phase84 edge gate changed")
    factor = candidate.get("factor", {})
    expected_factor = {
        "parity_scope": "C0/D active-row parity, not full seven-vector parity",
        "clock_state_units": "c1,c2 in seconds",
        "drift_state_units": "d1,d2 in metres per second",
        "dt_units": "seconds",
        "residual_units": "seconds",
        "speed_of_light_mps": 299792458.0,
        "residual_equation": "(c2-c1)-((d1+d2)*dt/(2*C_LIGHT))",
        "jacobian_order": ["c1", "c2", "d1", "d2"],
        "jacobian": "[-1,+1,-dt/(2*C_LIGHT),-dt/(2*C_LIGHT)]",
        "regular_sigma_seconds": "0.1/C_LIGHT",
        "jump_sigma_seconds": "Inf (implemented by omission)",
    }
    if any(factor.get(key) != value for key, value in expected_factor.items()):
        raise fail("Phase84 C0/D factor contract changed")
    plan = freeze.get("structural_plan", {})
    if plan.get("implementation_authorized_after_freeze") is not True or plan.get("route_execution_authorized_by_this_freeze") is not False or plan.get("raw_execution_requires_new_structural_manifest_or_freeze") is not True:
        raise fail("Phase84 authorization boundary changed")
    route_matrix = plan.get("route_matrix", {})
    if route_matrix.get("route_order_frozen") is not True or route_matrix.get("phone_model") != "pixel5" or route_matrix.get("runs_per_route") != 2 or route_matrix.get("total_runs") != 8:
        raise fail("Phase84 route matrix changed")

    _verify_phase80_contract()
    _phase78_domain_rows()
    _check_file_hash(BINARY, BINARY_SHA256, "Phase84 implementation binary")
    for path_text, expected in PHASE84_SOURCE_HASHES.items():
        _check_file_hash(ROOT / path_text, expected, "Phase84 implementation source")
    if freeze.get("read_accounting_at_freeze", {}).get("ground_truth_reads") != 0 or freeze.get("read_accounting_at_freeze", {}).get("native_solver_invocations") != 0:
        raise fail("Phase84 pre-execution accounting changed")

    if not MANIFEST.is_file():
        raise fail("Phase85 structural manifest is missing")
    manifest = load_json(MANIFEST, "Phase85 structural manifest")
    if manifest.get("schema_version") != "smartphone-r5-phase85-source-clock-c0d-structural-manifest.v1" or manifest.get("status") != "frozen-before-phase85-v2-raw-read":
        raise fail("Phase85 manifest schema/status changed")
    manifest_authority = manifest.get("authority", {})
    if manifest_authority.get("phase84_freeze", {}).get("sha256") != PHASE84_FREEZE_SHA256 or manifest_authority.get("phase80_freeze", {}).get("sha256") != PHASE80_FREEZE_SHA256 or manifest_authority.get("phase80_structural_manifest", {}).get("sha256") != PHASE80_MANIFEST_SHA256 or manifest_authority.get("phase78_structural_result", {}).get("sha256") != PHASE78_RESULT_SHA256:
        raise fail("Phase85 inherited hash chain changed")
    if manifest.get("implementation", {}).get("commit") != SOURCE_COMMIT or manifest.get("implementation", {}).get("binary", {}).get("sha256") != BINARY_SHA256 or manifest.get("implementation", {}).get("source_hashes") != PHASE84_SOURCE_HASHES:
        raise fail("Phase85 implementation pin changed")
    if manifest.get("official_source_pins") != OFFICIAL_SOURCE_PINS or manifest.get("routes") != list(ROUTES) or manifest.get("phase80_phase78_domain_rows") != DOMAIN_ROWS:
        raise fail("Phase85 official/routes/domain pins changed")
    candidate_manifest = manifest.get("candidate", {})
    if candidate_manifest.get("flag") != CLOCK_FLAG or candidate_manifest.get("direct_flag") != DIRECT_FLAG or candidate_manifest.get("controls") != 0 or candidate_manifest.get("runs_per_route") != 2 or candidate_manifest.get("preserve_additional_frequency_bands") is not False:
        raise fail("Phase85 candidate manifest contract changed")
    matrix = manifest.get("matrix", {})
    expected_matrix = {"candidate_runs_per_route": 2, "candidate_runs_total": 8, "control_runs_per_route": 0, "native_invocations": 8, "raw_device_gnss_reads": 8, "raw_device_imu_reads": 8, "broadcast_nav_reads": 8, "base_rinex_reads": 8, "truth_reads": 0, "mat_reads_or_generated": 0, "precomputed_coordinate_reads": 0, "kaggle_or_token_access": 0, "new_output_root": relative(DEFAULT_OUTPUT)}
    if any(matrix.get(key) != value for key, value in expected_matrix.items()):
        raise fail(f"Phase85 matrix pin changed: {key}")
    if manifest.get("output_policy", {}).get("refuse_nonempty_output_root") is not True or manifest.get("output_policy", {}).get("accuracy_scoring") is not False:
        raise fail("Phase85 output policy changed")
    prior = manifest.get("retry", {}).get("prior_v1_failure", {})
    if prior.get("path") != relative(PHASE85_V1_FAILURE) or prior.get("sha256") != PHASE85_V1_FAILURE_SHA256 or prior.get("bytes") != PHASE85_V1_FAILURE_BYTES:
        raise fail("Phase85 v1 fail-closed provenance pin changed")
    if sha256(PHASE85_V1_FAILURE) != PHASE85_V1_FAILURE_SHA256 or PHASE85_V1_FAILURE.stat().st_size != PHASE85_V1_FAILURE_BYTES:
        raise fail("Phase85 v1 failure output was overwritten")
    for item, label in ((manifest.get("evaluator", {}), "evaluator"), (manifest.get("focused_tests", {}), "focused tests"), (manifest.get("cmake", {}), "CMake")):
        if item.get("path") != relative({"evaluator": EVALUATOR, "focused tests": FOCUSED_TESTS, "CMake": TESTS_CMAKE}[label]) or item.get("sha256") != sha256({"evaluator": EVALUATOR, "focused tests": FOCUSED_TESTS, "CMake": TESTS_CMAKE}[label]):
            raise fail(f"Phase85 {label} hash pin changed")
    return freeze


def native_command(route: str, run_dir: Path) -> list[str]:
    phase80_command = P80.native_command(route, run_dir)
    command = phase80_command + [CLOCK_FLAG]
    for token in command:
        reject_forbidden(token)
    if command[:-1] != phase80_command or command[-1] != CLOCK_FLAG:
        raise fail("candidate command is not Phase80 exact flags plus C0/D")
    if command.count(CLOCK_FLAG) != 1 or command.count(DIRECT_FLAG) != 1 or command.count(NO_BRIDGE_FLAG) != 1:
        raise fail(f"candidate flag multiplicity failed: {route}")
    if LEGACY_QUALITY_FLAG in command or PRESERVE_BANDS_FLAG in command or command.count("--native-signal-bias-states") != 1:
        raise fail(f"candidate command contains a forbidden/incomplete inherited option: {route}")
    return command


def verify_inputs(route: str) -> dict[str, Any]:
    return P80.verify_inputs(route)


def _validate_clock_telemetry(summary: dict[str, Any], route: str, expected_rows: int) -> dict[str, Any]:
    telemetry = summary.get("native_source_clock_c0d_factor")
    if not isinstance(telemetry, dict) or telemetry.get("clock_c0d_enabled") is not True:
        raise fail(f"C0/D telemetry is not enabled: {route}")
    factor_count = _int(telemetry, "clock_c0d_factor_count", route, 1)
    skip_keys = ("clock_c0d_clock_jump_skips", "clock_c0d_gap_skips", "clock_c0d_invalid_dt_skips", "clock_c0d_phone_exclusion_skips")
    skips = {key: _int(telemetry, key, route) for key in skip_keys}
    if skips["clock_c0d_phone_exclusion_skips"] != 0:
        raise fail(f"Pixel5 phone exclusion is nonzero: {route}")
    epochs = summary.get("epochs")
    if not isinstance(epochs, dict) or _int(epochs, "output", route, 2) != expected_rows + 1:
        raise fail(f"C0/D edge domain is not the pinned epoch domain: {route}")
    if factor_count + sum(skips.values()) != expected_rows:
        raise fail(f"C0/D factor/skip accounting mismatch: {route}")
    dt_min = _number(telemetry.get("clock_c0d_dt_min_s"), f"clock-c0d/{route}/dt_min")
    dt_max = _number(telemetry.get("clock_c0d_dt_max_s"), f"clock-c0d/{route}/dt_max")
    if not (dt_min > 0.0 and dt_min <= dt_max):
        raise fail(f"C0/D dt range invalid: {route}")
    units = {"clock": "seconds", "drift": "metres_per_second", "dt": "seconds", "residual": "seconds", "sigma": "seconds"}
    expected = {
        "clock_c0d_equation": "(c2-c1)-((d1+d2)*dt/(2*C_LIGHT))",
        "clock_c0d_jacobian_order": ["c1", "c2", "d1", "d2"],
        "clock_c0d_jacobian": "[-1,+1,-dt/(2*C_LIGHT),-dt/(2*C_LIGHT)]",
        "clock_c0d_units": units,
        "clock_jump_noise": "Inf (active C0 factor omitted)",
        "parity_scope": "C0/D active-row parity; not full seven-vector",
        "legacy_scalar_clock_between_factor_count": 0,
    }
    for key, value in expected.items():
        if telemetry.get(key) != value:
            raise fail(f"C0/D telemetry contract mismatch: {route}/{key}")
    if not math.isclose(_number(telemetry.get("speed_of_light_mps"), f"clock-c0d/{route}/c"), SPEED_OF_LIGHT_MPS, rel_tol=0.0, abs_tol=1e-9) or not math.isclose(_number(telemetry.get("clock_c0d_sigma_seconds"), f"clock-c0d/{route}/sigma"), C0D_SIGMA_SECONDS, rel_tol=1e-12, abs_tol=1e-18):
        raise fail(f"C0/D physical constants mismatch: {route}")
    return {"factor_count": factor_count, **skips, "dt_min_s": dt_min, "dt_max_s": dt_max, "equation": telemetry["clock_c0d_equation"], "jacobian_order": telemetry["clock_c0d_jacobian_order"], "jacobian": telemetry["clock_c0d_jacobian"], "units": units, "sigma_seconds": telemetry["clock_c0d_sigma_seconds"], "legacy_scalar_clock_between_factor_count": 0}


def validate_summary(path: Path, route: str, expected_rows: int) -> dict[str, Any]:
    try:
        diagnostics = P80.validate_summary(path, route)
    except Exception as exc:
        raise fail(f"Phase80 invariant failed for {route}: {exc}") from exc
    summary = diagnostics["summary"]
    epochs = summary.get("epochs")
    graph = summary.get("graph")
    raw = summary.get("raw_utc_key_contract")
    if not all(isinstance(value, dict) for value in (epochs, graph, raw)):
        raise fail(f"Phase80 epoch/graph telemetry missing: {route}")
    assert isinstance(epochs, dict) and isinstance(graph, dict) and isinstance(raw, dict)
    if epochs.get("output") != expected_rows + 1 or epochs.get("problem") != epochs.get("output") or graph.get("imu_intervals") != expected_rows:
        raise fail(f"Phase80 epoch/IMU invariant failed: {route}")
    if raw.get("warmup_epoch_excluded") is not True or raw.get("raw_epoch_keys") != expected_rows + 1 or raw.get("target_epochs") != expected_rows or raw.get("exact_solution_epochs") != expected_rows or raw.get("interpolated_epochs") != 0 or raw.get("edge_hold_epochs") != 0 or raw.get("unresolved_epochs") != 0:
        raise fail(f"Phase80 raw UTC domain invariant failed: {route}")
    clock = _validate_clock_telemetry(summary, route, expected_rows)
    diagnostics["clock_c0d"] = clock
    return diagnostics


def artifact_report(submission: Path, summary: Path, route: str, expected_rows: int) -> dict[str, Any]:
    rows = P80.read_prediction(submission, route)
    speed = P80.speed_report(rows)
    if len(rows) != expected_rows or not speed["finite"] or speed["over_70_mps_count"] != 0:
        raise fail(f"prediction finite/earth/domain/speed gate failed: {route}")
    diagnostics = validate_summary(summary, route, expected_rows)
    return {
        "submission_artifact": {"path": relative(submission), "bytes": submission.stat().st_size, "sha256": sha256(submission), "rows": len(rows)},
        "summary_artifact": {"path": relative(summary), "bytes": summary.stat().st_size, "sha256": sha256(summary)},
        "summary_payload": diagnostics["summary"],
        "prediction_keys": [row[0] for row in rows],
        "speed": speed,
        "base_telemetry": diagnostics["base"],
        "source_miss_mask_telemetry": diagnostics["source_miss_mask"],
        "signal_bias": diagnostics["signal_bias"],
        "direct_quality": diagnostics["direct_quality"],
        "population": diagnostics["population"],
        "clock_c0d": diagnostics["clock_c0d"],
    }


def run_case(output_root: Path, route: str, run_number: int, expected_rows: int) -> dict[str, Any]:
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
    report: dict[str, Any] = {"candidate": True, "run": run_number, "return_code": process.returncode, "wall_seconds": time.perf_counter() - started, "command": command, "log": {"path": relative(run_dir / "run.log"), "sha256": sha256(run_dir / "run.log")}}
    if process.returncode != 0:
        raise fail(f"native process returned {process.returncode}: {route}/run{run_number}")
    submission, summary = run_dir / "submission.csv", run_dir / "summary.json"
    if not submission.is_file() or not summary.is_file():
        raise fail(f"native artifacts missing: {route}/run{run_number}")
    report.update(artifact_report(submission, summary, route, expected_rows))
    return report


def _public(report: dict[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in report.items() if key != "prediction_keys"}


def _repeat_identity(first: dict[str, Any], second: dict[str, Any]) -> bool:
    for artifact in ("submission_artifact", "summary_artifact"):
        if first[artifact].get("sha256") != second[artifact].get("sha256") or first[artifact].get("bytes") != second[artifact].get("bytes"):
            return False
    return all(first[key] == second[key] for key in ("prediction_keys", "population", "base_telemetry", "source_miss_mask_telemetry", "signal_bias", "direct_quality", "clock_c0d"))


def _failure(output_root: Path, routes: dict[str, Any], errors: list[dict[str, Any]], started: int, error: str) -> dict[str, Any]:
    return {"schema_version": OUTPUT_SCHEMA.replace("result", "failure"), "phase": 85, "execution_label": "Luna Max", "status": "fail-closed", "error": error, "truth_free": True, "freeze": {"path": relative(PHASE84_FREEZE), "sha256": PHASE84_FREEZE_SHA256}, "manifest": {"path": relative(MANIFEST), "sha256": sha256(MANIFEST) if MANIFEST.is_file() else None}, "evaluator": {"path": relative(EVALUATOR), "sha256": sha256(EVALUATOR)}, "routes": routes, "errors": errors, "native_solver_invocations_started": started, "native_solver_invocations_successful": sum(1 for record in routes.values() for case in record.get("cases", {}).values() if case.get("return_code") == 0), "raw_device_gnss_process_reads": started, "raw_device_imu_process_reads": started, "broadcast_nav_process_reads": started, "base_rinex_process_reads": started, "truth_reads": 0, "mat_reads_or_generated": 0, "precomputed_coordinate_reads": 0, "kaggle_or_token_access": 0, "accuracy_scored": False}


def run_matrix(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    verify_freeze()
    output_root = output_root.resolve()
    reject_forbidden(output_root)
    if output_root.exists() and (not output_root.is_dir() or any(output_root.iterdir())):
        raise fail(f"refusing to overwrite nonempty output: {output_root}")
    expected_rows = _phase78_domain_rows()
    output_root.mkdir(parents=True, exist_ok=True)
    routes: dict[str, Any] = {}
    errors: list[dict[str, Any]] = []
    started = 0
    try:
        input_reports = {route: verify_inputs(route) for route in ROUTES}
        for route in ROUTES:
            cases: dict[str, Any] = {}
            for run_number in (1, 2):
                started += 1
                try:
                    cases[f"candidate_run{run_number}"] = run_case(output_root, route, run_number, expected_rows[route])
                except Exception as exc:
                    case_error = {"candidate": True, "run": run_number, "error_type": type(exc).__name__, "error": str(exc)}
                    cases[f"candidate_run{run_number}"] = case_error
                    errors.append({"route": route, **case_error})
            route_record: dict[str, Any] = {"input": input_reports[route], "cases": {key: _public(value) for key, value in cases.items()}}
            if any("error" in value for value in cases.values()):
                route_record["gates"] = {"case_execution": False, "all_route_gates": False}
                routes[route] = route_record
                continue
            first, second = cases["candidate_run1"], cases["candidate_run2"]
            repeat = _repeat_identity(first, second)
            domain = all(case["submission_artifact"]["rows"] == expected_rows[route] and case["submission_artifact"]["rows"] + 1 == case["population"]["output_epochs"] for case in (first, second))
            clock = first["clock_c0d"]
            gates = {
                "case_execution": first["return_code"] == 0 and second["return_code"] == 0,
                "exact_four_route_matrix": route in ROUTES,
                "prediction_domain_coverage_exact": domain,
                "rows_plus_warmup_equals_summary_epochs_output": domain,
                "candidate_repeat_submission_and_summary_byte_identical": repeat,
                "candidate_coordinates_finite_and_earth_valid": first["speed"]["finite"] and second["speed"]["finite"],
                "candidate_speed_over_70_zero": first["speed"]["over_70_mps_count"] == 0 and second["speed"]["over_70_mps_count"] == 0,
                "candidate_converged": first["summary_payload"]["graph"]["converged"] is True and second["summary_payload"]["graph"]["converged"] is True,
                "phase80_direct_no_pdc_base_miss_signal_bias_pd_tdcp_invariants": True,
                "clock_c0d_enabled": clock["factor_count"] > 0,
                "clock_c0d_phone_exclusion_zero": clock["clock_c0d_phone_exclusion_skips"] == 0,
                "clock_c0d_legacy_scalar_between_zero": clock["legacy_scalar_clock_between_factor_count"] == 0,
                "clock_c0d_factor_skip_accounting": clock["factor_count"] + clock["clock_c0d_clock_jump_skips"] + clock["clock_c0d_gap_skips"] + clock["clock_c0d_invalid_dt_skips"] + clock["clock_c0d_phone_exclusion_skips"] == expected_rows[route],
                "clock_c0d_dt_range_positive": 0.0 < clock["dt_min_s"] <= clock["dt_max_s"],
                "candidate_flag_dependency_contract": all(case["command"].count(CLOCK_FLAG) == 1 and case["command"].count(DIRECT_FLAG) == 1 and case["command"].count(NO_BRIDGE_FLAG) == 1 and LEGACY_QUALITY_FLAG not in case["command"] and PRESERVE_BANDS_FLAG not in case["command"] for case in (first, second)),
            }
            gates["all_route_gates"] = all(gates.values())
            route_record.update({"candidate_run1": _public(first), "candidate_run2": _public(second), "prediction_domain": {"phase80_phase78_pinned_rows": expected_rows[route], "run1_rows": first["submission_artifact"]["rows"], "run2_rows": second["submission_artifact"]["rows"], "exact": domain}, "clock_c0d": clock, "repeat_identity": repeat, "gates": gates})
            routes[route] = route_record
        all_passed = set(routes) == set(ROUTES) and not errors and all(record.get("gates", {}).get("all_route_gates") is True for record in routes.values())
        if not all_passed:
            raise fail("Phase85 structural gates failed")
        result = {"schema_version": OUTPUT_SCHEMA, "phase": 85, "execution_label": "Luna Max", "status": "go-phase85-source-clock-c0d-structural", "truth_free": True, "accuracy_scored": False, "freeze": {"path": relative(PHASE84_FREEZE), "sha256": PHASE84_FREEZE_SHA256}, "manifest": {"path": relative(MANIFEST), "sha256": sha256(MANIFEST)}, "evaluator": {"path": relative(EVALUATOR), "sha256": sha256(EVALUATOR)}, "implementation": {"commit": SOURCE_COMMIT, "binary": {"path": relative(BINARY), "sha256": BINARY_SHA256}, "source_hashes": PHASE84_SOURCE_HASHES}, "candidate": {"flag": CLOCK_FLAG, "direct_flag": DIRECT_FLAG, "requires": [DIRECT_FLAG, NO_BRIDGE_FLAG], "phase80_flags_preserved": list(P80.P77.BASE_FLAGS) + [P80.P77.SIGNAL_BIAS_FLAG, P80.P77.BASE_COMP_FLAG, P80.P77.BASE_RINEX_FLAG, P80.P77.BASE_SHA_FLAG, P80.P77.MISS_MASK_FLAG], "controls": 0, "runs_per_route": 2, "preserve_additional_frequency_bands": False, "parity_scope": "C0/D active-row parity; not full seven-vector"}, "phase80_phase78_domain_rows": expected_rows, "routes": routes, "gates": {"all_four_routes": True, "native_solver_invocations": 8, "candidate_only": True, "direct_no_pdc": True, "phase80_invariants": True, "clock_c0d_telemetry": True, "clock_c0d_factor_count_positive": True, "clock_c0d_phone_exclusion_zero": True, "clock_c0d_legacy_scalar_between_zero": True, "clock_c0d_skip_accounting": True, "clock_c0d_dt_range": True, "candidate_repeat_identity": True, "coordinates_finite_earth_valid": True, "converged": True, "speed_over_70_zero": True, "truth_free": True, "accuracy_not_scored": True, "all_passed": True}, "read_accounting": {"candidate_runs_per_route": 2, "control_runs_per_route": 0, "native_solver_invocations": 8, "raw_device_gnss_process_reads": 8, "raw_device_imu_process_reads": 8, "broadcast_nav_reads": 8, "base_rinex_reads": 8, "truth_reads": 0, "mat_reads_or_generated": 0, "precomputed_coordinate_reads": 0, "kaggle_or_token_access": 0, "accuracy_scored": False}}
        result_path = output_root / "phase85_source_clock_c0d_structural_result.json"
        atomic_json(result_path, result)
        atomic_json(output_root / "phase85_source_clock_c0d_structural_manifest.json", {"schema_version": "smartphone-r5-phase85-source-clock-c0d-structural-output-manifest.v1", "phase": 85, "status": "sealed-truth-free-structural-matrix", "freeze": {"path": relative(PHASE84_FREEZE), "sha256": PHASE84_FREEZE_SHA256}, "evaluator": {"path": relative(EVALUATOR), "sha256": sha256(EVALUATOR)}, "result": {"path": relative(result_path), "bytes": result_path.stat().st_size, "sha256": sha256(result_path)}, "candidate_runs": 8, "control_runs": 0, "truth_reads": 0, "accuracy_scored": False, "all_gates_passed": True})
        return result
    except Phase85StructuralError as exc:
        failure = _failure(output_root, routes, errors, started, str(exc))
        atomic_json(output_root / "phase85_source_clock_c0d_structural_failure.json", failure)
        raise
    except Exception as exc:
        failure = _failure(output_root, routes, errors, started, f"unexpected {type(exc).__name__}: {exc}")
        atomic_json(output_root / "phase85_source_clock_c0d_structural_exception.json", failure)
        raise fail(f"unexpected Phase85 evaluator exception: {type(exc).__name__}: {exc}") from exc


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
            print(json.dumps({"status": result["status"], "all_gates_passed": result["gates"]["all_passed"], "native_solver_invocations": result["read_accounting"]["native_solver_invocations"], "truth_reads": result["read_accounting"]["truth_reads"], "accuracy_scored": result["accuracy_scored"]}, sort_keys=True))
        elif not args.verify_freeze:
            parser.error("one of --verify-freeze or --run-matrix is required")
        return 0
    except Phase85StructuralError as exc:
        print(f"phase85 structural failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
