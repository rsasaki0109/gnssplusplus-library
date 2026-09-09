#!/usr/bin/env python3
"""Fail-closed, pre-raw evaluator for the Phase92 Luna Max candidate.

This evaluator seals exactly one candidate, one run per each of the four
Phase80/85 routes, and the raw-input *plan* (GNSS + IMU + broadcast nav only).
It intentionally has no native-process launch path and never opens or hashes a
raw input.  It verifies only the implementation, inherited authority, planned
commands, and zero-read execution boundary before any raw run is authorized.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]

PHASE92_AUDIT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase92_phase91_alignment_and_clock_isb_unit_audit_v1.md"
)
PHASE92_AUDIT_SHA256 = (
    "45b15439063eaa823f786df15155c83b92e65939812e1d18698cf1812d0f5271"
)
IMPLEMENTATION_FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase92_source_meter_clock_state_parity_freeze_v1.json"
)
IMPLEMENTATION_FREEZE_SHA256 = (
    "998582a3edec859e576283c836395d27b9476dcca9da25c9d9aafb3a88eb1102"
)
PHASE85_MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase85_source_clock_c0d_structural_manifest_v1.json"
)
PHASE85_MANIFEST_SHA256 = (
    "20adc2e3388b92d045580739d025106c18aa068fcf26f8d117d48d3d2f5f7058"
)
PHASE80_FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase80_source_exact_direct_p_quality_freeze_v1.json"
)
PHASE80_FREEZE_SHA256 = (
    "9ebdde21fe7fd955029243d3bdbba3a9f9d3832388d67480760394177c8f685e"
)
EXECUTION_FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase92_source_meter_clock_state_parity_execution_freeze_v1.json"
)
# Digest of the separately sealed pre-raw execution freeze.
EXECUTION_FREEZE_SHA256 = "c07ac587ebcfde6391de0b8a824687316de854442166a93daadb416d4e9b74d8"
MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase92_source_meter_clock_state_parity_execution_manifest_v1.json"
)
EVALUATOR = Path(__file__).resolve()
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase92_source_meter_clock_state_parity.py"
TESTS_CMAKE = ROOT / "tests/CMakeLists.txt"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

IMPLEMENTATION_COMMIT = "a1852bf45030d681e178fb9da9979c9a0dee737d"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
DOMAIN_ROWS = {
    ROUTES[0]: 2158,
    ROUTES[1]: 3139,
    ROUTES[2]: 1465,
    ROUTES[3]: 1101,
}

SELECTOR = "--native-source-clock-c0d-meter-state-parity"
CLOCK_FLAG = "--native-source-clock-c0d-factor"
ACTIVE_FLAG = "--native-source-clock-c0d-active-solve-diagnostic"
RAW_DRIFT_FLAG = "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer"
DIRECT_FLAG = "--native-source-direct-observable-quality"
NO_BRIDGE_FLAG = "--native-pdc-imu-tdcp-no-bridge"
RAW_UTC_FLAG = "--android-raw-utc-keys"
COMPATIBILITY_FLAGS = (
    "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback",
)
RAW_INPUT_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
FORBIDDEN_FLAGS = (
    "--obs",
    "--native-gnss-first-velocity-only-handoff",
    "--native-direct-doppler-wls-handoff",
    "--native-pdc-state-bridge",
    "--native-upstream-quality",
    "--native-base-rinex",
    "--native-base-pseudorange-compensation",
    "--native-base-pseudorange-source-miss-mask",
    "--native-base-pseudorange-preserve-additional-frequency-bands",
)
FORBIDDEN_PATH_TERMS = (
    ".mat",
    "ground_truth",
    "validation",
    "holdout",
    "kaggle",
    "token",
    "phase82",
    "base.rinex",
    "coordinate",
    "truth",
)
REQUIRED_FLAGS = (
    "--android-gnss <frozen raw device_gnss.csv>",
    "--android-imu <frozen raw device_imu.csv>",
    "--nav <frozen broadcast brdc.nav>",
    "--dataset-id <one of the four frozen route IDs>",
    "--all-epochs",
    RAW_UTC_FLAG,
    *COMPATIBILITY_FLAGS,
    NO_BRIDGE_FLAG,
    DIRECT_FLAG,
    CLOCK_FLAG,
    ACTIVE_FLAG,
    RAW_DRIFT_FLAG,
    SELECTOR,
)

SOURCE_PINS = {
    "apps/native/gnss_fgo_imu_no_base.cpp": "3ef5d9ce8dcc0b6e2ef72a019dd0ede67b9403730f5566310ea5d77d32acfc1e",
    "include/libgnss++/algorithms/fgo.hpp": "eb75b5ca5cbf41845721efc5cb97a2cb71c827828c7fc6a0526ef952cb52c2b3",
    "include/libgnss++/algorithms/fgo_config.hpp": "b0e516d6c400fbd6eef6b506c81ac3040284a97059bdca1cc6192ff0468c0c9a",
    "include/libgnss++/algorithms/source_clock_c0d_initializer.hpp": "43a4be7e553407fe9417a8548b33f74870cd339c496239836fa589febe490237",
    "include/libgnss++/core/observation.hpp": "7a81b4f1c7d36b4af2ed8a11c0f1edcbb998591a22a4d1406d75aeb43641f11b",
    "src/algorithms/fgo.cpp": "d89e466572589a4cf068abd46532747d8faa00881ee95150c02da355d5591932",
    "src/algorithms/fgo_gtsam_backend.cpp": "0f97a40a6e863f068facc5113481b921c4050dc29afc7a820e4ad003187615bb",
    "src/algorithms/fgo_gtsam_internal.hpp": "71513cc8f27d5e8833c8b0833f693c5dd1cac01293a6731ea00a3ada22cd8a44",
    "src/algorithms/fgo_problems.cpp": "e7607ce8f0fe27fd382a01b5b6f147b027b7bf51453e920eae458c86ddac0c17",
    "src/io/android_raw_gnss.cpp": "a020ce900db6a9d3a994cc3adb3dcebbe84cbf67c1ff9793991b7a75039529c5",
    "tests/test_android_raw_gnss.cpp": "40a77cbf37468f39fb9b1ef30b8ab94a05900cfc6ab789b7b66fcdae947f6bd2",
    "tests/test_fgo.cpp": "60ede7bdd22ddc6e0db3f494edea523b5dbe48f1e6119afda1ba65046e805b2d",
    "tests/test_fgo_gtsam_backend.cpp": "2cb19e77ebbbe4ff5cddb4041374a2afff1c763d59a9289dc0e6407ceec71c48",
}


class Phase92PreRawError(ValueError):
    """Raised when the sealed Phase92 pre-raw contract fails."""


def fail(message: str) -> Phase92PreRawError:
    return Phase92PreRawError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _reject_forbidden_path(path: Path | str) -> None:
    lowered = str(path).lower()
    if any(term in lowered for term in FORBIDDEN_PATH_TERMS):
        raise fail(f"forbidden artifact path: {path}")


def _read_bytes(path: Path, label: str, expected_sha256: str | None = None) -> bytes:
    _reject_forbidden_path(path)
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    if expected_sha256 is not None and hashlib.sha256(payload).hexdigest() != expected_sha256:
        raise fail(f"{label} SHA-256 mismatch: {path}")
    return payload


def _sha256(path: Path, label: str) -> str:
    return hashlib.sha256(_read_bytes(path, label)).hexdigest()


def _read_json(path: Path, label: str, expected_sha256: str | None = None) -> dict[str, Any]:
    try:
        value = json.loads(_read_bytes(path, label, expected_sha256).decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"invalid {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def _assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label} changed: expected {expected!r}, got {actual!r}")


def _assert_zero_accounting(accounting: dict[str, Any], label: str) -> None:
    zero_keys = (
        "raw_device_gnss_reads",
        "raw_device_imu_reads",
        "broadcast_navigation_reads",
        "base_rinex_reads",
        "native_solver_invocations",
        "native_reruns",
        "truth_reads",
        "mat_reads_or_generated",
        "precomputed_coordinate_reads",
        "validation_holdout_reads",
        "kaggle_or_token_access",
    )
    for key in zero_keys:
        _assert_equal(accounting.get(key), 0, f"{label}/{key}")
    _assert_equal(accounting.get("accuracy_scored"), False, f"{label}/accuracy_scored")
    _assert_equal(accounting.get("route_score_selection"), False, f"{label}/route_score_selection")


def verify_inherited_authority() -> None:
    _read_bytes(PHASE92_AUDIT, "Phase92 audit", PHASE92_AUDIT_SHA256)
    implementation_freeze = _read_json(
        IMPLEMENTATION_FREEZE,
        "Phase92 implementation freeze",
        IMPLEMENTATION_FREEZE_SHA256,
    )
    _assert_equal(implementation_freeze.get("phase"), 92, "Phase92 implementation freeze/phase")
    _assert_equal(
        implementation_freeze.get("status"),
        "frozen-before-implementation",
        "Phase92 implementation freeze/status",
    )
    _assert_equal(
        implementation_freeze.get("execution_label"),
        "Luna Max",
        "Phase92 implementation freeze/execution_label",
    )
    boundary = implementation_freeze.get("decision_boundary", {})
    _assert_equal(boundary.get("candidate_count"), 1, "Phase92 implementation freeze/candidate_count")
    _assert_equal(boundary.get("execution_authorized"), False, "Phase92 implementation freeze/execution_authorized")
    candidate = implementation_freeze.get("candidate", {})
    _assert_equal(candidate.get("selector"), SELECTOR, "Phase92 implementation freeze/selector")
    _assert_equal(candidate.get("default_off"), True, "Phase92 implementation freeze/default_off")

    phase85 = _read_json(PHASE85_MANIFEST, "Phase85 manifest", PHASE85_MANIFEST_SHA256)
    _assert_equal(phase85.get("phase"), 85, "Phase85 manifest/phase")
    _assert_equal(phase85.get("status"), "frozen-before-phase85-v2-raw-read", "Phase85 manifest/status")
    _assert_equal(phase85.get("execution_label"), "Luna Max", "Phase85 manifest/execution_label")
    _assert_equal(phase85.get("routes"), list(ROUTES), "Phase85 manifest/routes")
    _assert_equal(phase85.get("phase80_phase78_domain_rows"), DOMAIN_ROWS, "Phase85 manifest/domain_rows")
    matrix = phase85.get("matrix", {})
    for key in ("truth_reads", "mat_reads_or_generated", "precomputed_coordinate_reads", "kaggle_or_token_access"):
        _assert_equal(matrix.get(key), 0, f"Phase85 manifest/matrix/{key}")

    phase80 = _read_json(PHASE80_FREEZE, "Phase80 freeze", PHASE80_FREEZE_SHA256)
    _assert_equal(phase80.get("phase"), 80, "Phase80 freeze/phase")
    _assert_equal(phase80.get("execution_label"), "Luna Max", "Phase80 freeze/execution_label")
    phase80_candidate = phase80.get("candidate", {})
    _assert_equal(phase80_candidate.get("source_quality_flag"), DIRECT_FLAG, "Phase80 freeze/direct quality flag")
    _assert_equal(phase80_candidate.get("default_off"), True, "Phase80 freeze/default_off")
    _assert_equal(phase80_candidate.get("direct_no_pdc"), True, "Phase80 freeze/direct_no_pdc")


def verify_implementation_freeze() -> dict[str, Any]:
    freeze = _read_json(EXECUTION_FREEZE, "Phase92 execution freeze", EXECUTION_FREEZE_SHA256)
    verify_inherited_authority()
    _assert_equal(
        freeze.get("schema_version"),
        "smartphone-r5-phase92-source-meter-clock-state-parity-execution-freeze.v1",
        "Phase92 execution freeze/schema_version",
    )
    _assert_equal(freeze.get("phase"), 92, "Phase92 execution freeze/phase")
    _assert_equal(freeze.get("status"), "frozen-before-phase92-raw-execution", "Phase92 execution freeze/status")
    _assert_equal(freeze.get("execution_label"), "Luna Max", "Phase92 execution freeze/execution_label")

    authority = freeze.get("authority", {})
    _assert_equal(authority.get("phase92_audit", {}).get("sha256"), PHASE92_AUDIT_SHA256, "Phase92 execution freeze/audit pin")
    _assert_equal(authority.get("phase92_implementation_freeze", {}).get("sha256"), IMPLEMENTATION_FREEZE_SHA256, "Phase92 execution freeze/implementation freeze pin")
    _assert_equal(authority.get("phase85_manifest", {}).get("sha256"), PHASE85_MANIFEST_SHA256, "Phase92 execution freeze/Phase85 pin")
    _assert_equal(authority.get("phase80_freeze", {}).get("sha256"), PHASE80_FREEZE_SHA256, "Phase92 execution freeze/Phase80 pin")
    _assert_equal(freeze.get("implementation", {}).get("commit"), IMPLEMENTATION_COMMIT, "Phase92 execution freeze/implementation commit")

    boundary = freeze.get("decision_boundary", {})
    expected_boundary = {
        "candidate_count": 1,
        "route_count": 4,
        "repetitions_per_route": 1,
        "controls": 0,
        "implementation_authorized": True,
        "execution_authorized": False,
        "before_raw_execution": True,
        "raw_execution_before_this_freeze": False,
        "accuracy_scoring": False,
        "route_score_selection": False,
        "promotion_or_accuracy_authorization": False,
        "fail_closed_if_missing_meter_telemetry": True,
        "fail_closed_if_alignment_or_unit_invariant_failure": True,
        "stop_before_accuracy_or_submission": True,
    }
    for key, value in expected_boundary.items():
        _assert_equal(boundary.get(key), value, f"Phase92 execution freeze/decision_boundary/{key}")

    candidate = freeze.get("candidate", {})
    for key, value in {
        "id": "phase92_source_meter_clock_state_parity_with_retained_raw_d_alignment",
        "selector": SELECTOR,
        "source_aligned": True,
        "raw_only": True,
        "diagnostic_only": True,
        "default_off": True,
        "exactly_one_candidate": True,
    }.items():
        _assert_equal(candidate.get(key), value, f"Phase92 execution freeze/candidate/{key}")
    _assert_equal(candidate.get("required_flags"), list(REQUIRED_FLAGS), "Phase92 execution freeze/required_flags")
    _assert_equal(
        candidate.get("forbidden_flags"),
        [*FORBIDDEN_FLAGS, "any truth, MAT, Kaggle/token, base, result-file, or precomputed-coordinate input"],
        "Phase92 execution freeze/forbidden_flags",
    )
    raw_contract = candidate.get("raw_input_contract", {})
    _assert_equal(raw_contract.get("input_names"), list(RAW_INPUT_NAMES), "Phase92 execution freeze/raw input names")
    _assert_equal(raw_contract.get("raw_gnss_only"), True, "Phase92 execution freeze/raw_gnss_only")
    _assert_equal(raw_contract.get("raw_imu_only"), True, "Phase92 execution freeze/raw_imu_only")
    _assert_equal(raw_contract.get("broadcast_navigation_only"), True, "Phase92 execution freeze/broadcast_navigation_only")
    _assert_equal(raw_contract.get("truth_mat_kaggle_base_coordinate_accuracy"), False, "Phase92 execution freeze/forbidden artifacts")

    routes = freeze.get("routes_and_domain", {})
    for key, value in {
        "route_order": list(ROUTES),
        "domain_rows": DOMAIN_ROWS,
        "exactly_one_run_each": True,
        "controls": 0,
        "route_score_selection": False,
    }.items():
        _assert_equal(routes.get(key), value, f"Phase92 execution freeze/routes_and_domain/{key}")

    authorization = freeze.get("execution_authorization", {})
    for key, value in {
        "no_raw_execution_performed_at_freeze": True,
        "raw_execution_authorized": False,
        "no_accuracy_or_submission_release": True,
        "accuracy_scored": False,
        "route_score_selection": False,
        "base_rinex_reads": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "precomputed_coordinate_reads": 0,
        "validation_holdout_reads": 0,
        "kaggle_or_token_access": 0,
    }.items():
        _assert_equal(authorization.get(key), value, f"Phase92 execution freeze/execution_authorization/{key}")
    for key in ("raw_device_gnss_reads", "raw_device_imu_reads", "broadcast_navigation_reads", "native_solver_invocations"):
        _assert_equal(authorization.get(key), 0, f"Phase92 execution freeze/execution_authorization/{key}")
    _assert_zero_accounting(freeze.get("read_accounting_at_freeze", {}), "Phase92 execution freeze/read_accounting_at_freeze")
    return freeze


def _verify_source_pins(manifest: dict[str, Any]) -> None:
    implementation = manifest.get("implementation", {})
    _assert_equal(implementation.get("commit"), IMPLEMENTATION_COMMIT, "manifest/implementation/commit")
    _assert_equal(implementation.get("source_hashes"), SOURCE_PINS, "manifest/implementation/source_hashes")
    for path_text, expected in SOURCE_PINS.items():
        path = ROOT / path_text
        if _sha256(path, f"implementation source {path_text}") != expected:
            raise fail(f"implementation source hash changed: {path_text}")
    binary = implementation.get("binary", {})
    _assert_equal(binary.get("path"), relative(BINARY), "manifest/implementation/binary/path")
    expected_binary_sha = binary.get("sha256")
    if not isinstance(expected_binary_sha, str) or len(expected_binary_sha) != 64:
        raise fail("manifest/implementation/binary/sha256 is not sealed")
    if _sha256(BINARY, "implementation binary") != expected_binary_sha:
        raise fail("implementation binary hash changed")


def _raw_input_path(route: str, name: str) -> str:
    return f"raw/phase92/{route}/{name}"


def _validate_raw_input_plan(route: str, raw_inputs: Any) -> None:
    if not isinstance(raw_inputs, dict) or tuple(raw_inputs) != RAW_INPUT_NAMES:
        raise fail(f"raw input set is not exactly GNSS+IMU+nav: {route}")
    for name in RAW_INPUT_NAMES:
        item = raw_inputs.get(name)
        if not isinstance(item, dict):
            raise fail(f"raw input pin is not an object: {route}/{name}")
        path_text = item.get("path")
        if not isinstance(path_text, str) or not path_text.startswith("raw/phase92/"):
            raise fail(f"raw input path is not a pre-raw placeholder: {route}/{name}")
        if Path(path_text).is_absolute() or Path(path_text).name != name:
            raise fail(f"raw input path/name mismatch: {route}/{name}")
        if path_text != _raw_input_path(route, name):
            raise fail(f"raw input route path changed: {route}/{name}")
        _reject_forbidden_path(path_text)
        if item.get("sha256") is not None or item.get("sha256_available") is not False:
            raise fail(f"raw input was hashed/read before the freeze: {route}/{name}")
        if item.get("read_at_manifest_creation") is not False:
            raise fail(f"raw input read accounting changed: {route}/{name}")


def _validate_command(route: str, command: Any, raw_inputs: dict[str, Any]) -> None:
    if not isinstance(command, list) or not command or command[0] != relative(BINARY):
        raise fail(f"native command binary changed: {route}")
    if any(not isinstance(token, str) for token in command):
        raise fail(f"native command contains a non-string token: {route}")
    for token in command:
        lowered = token.lower()
        if any(term in lowered for term in FORBIDDEN_PATH_TERMS):
            raise fail(f"native command contains forbidden artifact token: {route}: {token}")
        if token in FORBIDDEN_FLAGS:
            raise fail(f"native command contains forbidden flag: {route}: {token}")
    expected_flags = (
        RAW_UTC_FLAG,
        *COMPATIBILITY_FLAGS,
        NO_BRIDGE_FLAG,
        DIRECT_FLAG,
        CLOCK_FLAG,
        ACTIVE_FLAG,
        RAW_DRIFT_FLAG,
        SELECTOR,
    )
    for flag in expected_flags:
        if command.count(flag) != 1:
            raise fail(f"native command flag multiplicity failed: {route}/{flag}")
    if command.count("--dataset-id") != 1 or command[command.index("--dataset-id") + 1] != route:
        raise fail(f"native command dataset identity changed: {route}")
    input_flag_names = (("--android-gnss", "device_gnss.csv"), ("--android-imu", "device_imu.csv"), ("--nav", "brdc.nav"))
    for flag, name in input_flag_names:
        if command.count(flag) != 1:
            raise fail(f"native command input flag multiplicity failed: {route}/{flag}")
        value = command[command.index(flag) + 1]
        if value != raw_inputs[name]["path"]:
            raise fail(f"native command raw path does not match manifest: {route}/{name}")
    if "--out" not in command or "--summary-json" not in command:
        raise fail(f"native command output paths are incomplete: {route}")
    for flag in ("--out", "--summary-json"):
        value = command[command.index(flag) + 1]
        if not value.startswith("output/smartphone-r5/phase92-source-meter-clock-state-parity-v1/"):
            raise fail(f"native command output path escaped the Phase92 root: {route}")


def verify_manifest() -> dict[str, Any]:
    verify_implementation_freeze()
    manifest = _read_json(MANIFEST, "Phase92 execution manifest")
    _assert_equal(manifest.get("schema_version"), "smartphone-r5-phase92-source-meter-clock-state-parity-execution-manifest.v1", "manifest/schema_version")
    _assert_equal(manifest.get("phase"), 92, "manifest/phase")
    _assert_equal(manifest.get("status"), "sealed-before-phase92-raw-execution", "manifest/status")
    _assert_equal(manifest.get("execution_label"), "Luna Max", "manifest/execution_label")
    freeze_ref = manifest.get("freeze", {})
    _assert_equal(freeze_ref.get("path"), relative(EXECUTION_FREEZE), "manifest/freeze/path")
    _assert_equal(freeze_ref.get("sha256"), EXECUTION_FREEZE_SHA256, "manifest/freeze/sha256")
    _verify_source_pins(manifest)

    evaluator_pin = manifest.get("evaluator", {})
    _assert_equal(evaluator_pin.get("path"), relative(EVALUATOR), "manifest/evaluator/path")
    _assert_equal(evaluator_pin.get("sha256"), _sha256(EVALUATOR, "pre-raw evaluator"), "manifest/evaluator/sha256")
    focused_pin = manifest.get("focused_tests", {})
    _assert_equal(focused_pin.get("path"), relative(FOCUSED_TESTS), "manifest/focused_tests/path")
    _assert_equal(focused_pin.get("sha256"), _sha256(FOCUSED_TESTS, "focused pre-raw tests"), "manifest/focused_tests/sha256")
    cmake_pin = manifest.get("cmake", {})
    _assert_equal(cmake_pin.get("path"), relative(TESTS_CMAKE), "manifest/cmake/path")
    _assert_equal(cmake_pin.get("sha256"), _sha256(TESTS_CMAKE, "tests CMake"), "manifest/cmake/sha256")

    candidate = manifest.get("candidate", {})
    for key, value in {
        "id": "phase92_source_meter_clock_state_parity_with_retained_raw_d_alignment",
        "selector": SELECTOR,
        "candidate_count": 1,
        "raw_only": True,
        "source_aligned": True,
        "diagnostic_only": True,
        "default_off": True,
        "controls": 0,
        "runs_per_route": 1,
        "accuracy_scoring": False,
        "route_score_selection": False,
    }.items():
        _assert_equal(candidate.get(key), value, f"manifest/candidate/{key}")
    _assert_equal(candidate.get("required_flags"), list(REQUIRED_FLAGS), "manifest/candidate/required_flags")
    _assert_equal(candidate.get("forbidden_flags"), [*FORBIDDEN_FLAGS, "any truth, MAT, Kaggle/token, base, result-file, or precomputed-coordinate input"], "manifest/candidate/forbidden_flags")

    routes = manifest.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("manifest route order/count changed")
    for item in routes:
        route = item.get("dataset_id")
        if route not in DOMAIN_ROWS:
            raise fail(f"manifest has an unknown route: {route}")
        _assert_equal(item.get("runs"), 1, f"manifest/route/{route}/runs")
        _assert_equal(item.get("domain_rows"), DOMAIN_ROWS[route], f"manifest/route/{route}/domain_rows")
        _assert_equal(item.get("diagnostic_only"), True, f"manifest/route/{route}/diagnostic_only")
        _validate_raw_input_plan(route, item.get("raw_inputs"))
        _validate_command(route, item.get("command"), item["raw_inputs"])
        outputs = item.get("planned_output", {})
        for key in ("submission", "summary"):
            path_text = outputs.get(key)
            if not isinstance(path_text, str) or not path_text.startswith("output/smartphone-r5/phase92-source-meter-clock-state-parity-v1/"):
                raise fail(f"manifest planned output escaped the Phase92 root: {route}/{key}")

    matrix = manifest.get("matrix", {})
    for key, value in {
        "candidate_count": 1,
        "routes": 4,
        "runs_per_route": 1,
        "candidate_runs_total": 4,
        "control_runs_per_route": 0,
        "native_invocations": 4,
        "raw_device_gnss_reads_planned": 4,
        "raw_device_imu_reads_planned": 4,
        "broadcast_nav_reads_planned": 4,
        "base_rinex_reads_planned": 0,
        "truth_reads_planned": 0,
        "mat_reads_or_generated_planned": 0,
        "precomputed_coordinate_reads_planned": 0,
        "accuracy_scored": False,
        "route_score_selection": False,
    }.items():
        _assert_equal(matrix.get(key), value, f"manifest/matrix/{key}")
    authorization = manifest.get("execution_authorization", {})
    _assert_equal(authorization.get("before_raw_execution"), True, "manifest/execution_authorization/before_raw_execution")
    _assert_equal(authorization.get("raw_execution_authorized"), False, "manifest/execution_authorization/raw_execution_authorized")
    _assert_equal(authorization.get("native_route_rerun_performed"), False, "manifest/execution_authorization/native_route_rerun_performed")
    _assert_equal(authorization.get("stop_before_accuracy_or_submission"), True, "manifest/execution_authorization/stop_before_accuracy_or_submission")
    _assert_equal(authorization.get("accuracy_or_submission_release"), False, "manifest/execution_authorization/accuracy_or_submission_release")
    _assert_zero_accounting(manifest.get("read_accounting_at_manifest_creation", {}), "manifest/read_accounting_at_manifest_creation")
    return manifest


def verify_pre_raw() -> dict[str, Any]:
    freeze = verify_implementation_freeze()
    manifest = verify_manifest()
    # This source has no process-launch/import path by construction.  Keep the
    # assertion local so a future edit cannot accidentally turn this pre-raw
    # verifier into an execution runner.
    source = _read_bytes(EVALUATOR, "pre-raw evaluator source").decode("utf-8")
    for forbidden in (
        "import " + "subprocess",
        "sub" + "process.",
        "Popen" + "(",
        "run" + "(",
    ):
        if forbidden in source:
            raise fail(f"pre-raw evaluator contains a process-launch token: {forbidden}")
    return {
        "status": "pre-raw-verified",
        "phase": 92,
        "execution_label": "Luna Max",
        "candidate_count": 1,
        "routes": 4,
        "runs_per_route": 1,
        "raw_execution_authorized": False,
        "raw_reads": 0,
        "native_solver_invocations": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "precomputed_coordinate_reads": 0,
        "accuracy_scored": False,
        "route_score_selection": False,
        "freeze_sha256": EXECUTION_FREEZE_SHA256,
        "manifest_sha256": _sha256(MANIFEST, "Phase92 execution manifest"),
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "freeze_status": freeze["status"],
        "manifest_status": manifest["status"],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--verify-manifest", action="store_true")
    parser.add_argument("--verify-pre-raw", action="store_true")
    args = parser.parse_args(argv)
    if not any((args.verify_freeze, args.verify_manifest, args.verify_pre_raw)):
        parser.error("one of --verify-freeze, --verify-manifest, or --verify-pre-raw is required")
    try:
        if args.verify_freeze:
            verify_implementation_freeze()
        if args.verify_manifest:
            verify_manifest()
        if args.verify_pre_raw:
            print(json.dumps(verify_pre_raw(), sort_keys=True))
        return 0
    except Phase92PreRawError as exc:
        print(f"phase92 pre-raw failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
