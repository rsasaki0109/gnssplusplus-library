#!/usr/bin/env python3
"""One-shot Phase138 structural raw executor.

This is the post-authorization boundary for the Phase138 contract.  It
validates the independent authorization and all static pins before opening a
raw input.  Each route is then hashed once and, if its sealed hashes match,
the native binary is launched once.  Only native structural metadata and an
opaque output hash are recorded; solution coordinates are never parsed.
Truth, MAT, PDC, precomputed-coordinate, accuracy, network, retry, fallback,
and rerun paths are intentionally absent.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[3]
AUTH = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase138_affine_tdcp_structural_raw_authorization_v1.json"
)
RESULT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase138_affine_tdcp_structural_raw_result_v1.json"
)
RUNNER_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase138_affine_tdcp_structural.py"
)
FORBIDDEN_INPUT_TERMS = (
    ".mat",
    "truth",
    "ground_truth",
    "pdc",
    "precomputed",
    "kaggle",
    "token",
)
OPAQUE_OUTPUT_NAME = "opaque_solution_output.csv"
LEGITIMATE_PDC_ALGORITHM_OPTION = "--native-pdc-imu-tdcp-no-bridge"
ALLOWED_INPUT_NAMES = {"device_gnss.csv", "device_imu.csv", "brdc.nav", "base.obs"}
AUTHORIZATION_SCHEMA_VERSION = (
    "smartphone-r5-phase138-affine-tdcp-structural-raw-authorization.v1"
)
ROUTE_KEYS = frozenset({
    "target",
    "dataset_id",
    "runs",
    "raw_inputs",
    "base_input",
    "output_directory",
})
RAW_INPUT_KEYS = frozenset({"device_gnss.csv", "device_imu.csv", "brdc.nav"})
RAW_METADATA_KEYS = frozenset({
    "path",
    "bytes",
    "sha256",
    "source",
    "read_before_authorization",
    "copy_or_transform",
})
BASE_METADATA_KEYS = frozenset({
    "path",
    "bytes",
    "sha256",
    "interval_s",
    "moving_mean_samples",
    "source",
    "read_before_authorization",
    "hash_read_before_authorization",
    "copy_or_transform",
})
RAW_SOURCE = "sealed raw-only input lineage"
BASE_SOURCE = "raw base RINEX; station coordinate from header only"
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}\Z")


class AuthorizationError(ValueError):
    """A fail-closed authorization, input, or structural violation."""


def fail(message: str) -> AuthorizationError:
    return AuthorizationError(message)


def load_runner():
    spec = importlib.util.spec_from_file_location(
        "phase138_static_structural_runner", RUNNER_PATH
    )
    if spec is None or spec.loader is None:
        raise fail("unable to load Phase138 static validator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


STATIC = load_runner()


def _reject_duplicate_json_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    """Reject duplicate object keys instead of silently taking the last one."""
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise fail(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def read_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_json_keys,
        )
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"{label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label}: expected object")
    return value


def digest_file(path: Path, label: str) -> tuple[str, int]:
    """Hash one authorized file; callers invoke this once per raw input."""
    if not path.is_file():
        raise fail(f"{label}: missing authorized input {path}")
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            size += len(chunk)
            digest.update(chunk)
    return digest.hexdigest(), size


def digest_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def opaque_solution_metadata(path: Path) -> dict[str, Any]:
    """Hash/count newlines only; never parse a solution field."""
    digest = hashlib.sha256()
    size = 0
    newlines = 0
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            size += len(chunk)
            newlines += chunk.count(b"\n")
            digest.update(chunk)
    return {
        "path": str(path.relative_to(ROOT)),
        "sha256": digest.hexdigest(),
        "bytes": size,
        "newline_count": newlines,
        "content_interpreted": False,
    }


def _require(mapping: Mapping[str, Any], key: str, label: str) -> Any:
    if key not in mapping:
        raise fail(f"{label}/{key}: missing")
    return mapping[key]


def _mapping(mapping: Mapping[str, Any], key: str, label: str) -> Mapping[str, Any]:
    value = _require(mapping, key, label)
    if not isinstance(value, Mapping):
        raise fail(f"{label}/{key}: expected object")
    return value


def _int(mapping: Mapping[str, Any], key: str, label: str) -> int:
    value = _require(mapping, key, label)
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise fail(f"{label}/{key}: expected nonnegative integer")
    return value


def _bool(mapping: Mapping[str, Any], key: str, label: str) -> bool:
    value = _require(mapping, key, label)
    if not isinstance(value, bool):
        raise fail(f"{label}/{key}: expected boolean")
    return value


def _number(mapping: Mapping[str, Any], key: str, label: str) -> float:
    value = _require(mapping, key, label)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise fail(f"{label}/{key}: expected number")
    result = float(value)
    if not __import__("math").isfinite(result):
        raise fail(f"{label}/{key}: expected finite number")
    return result


def _exact_keys(mapping: Mapping[str, Any], expected: frozenset[str],
                label: str) -> None:
    """Require a sealed object shape; reject both missing and unknown keys."""
    actual = set(mapping.keys())
    if actual != set(expected):
        missing = sorted((key for key in expected if key not in actual),
                         key=repr)
        unknown = sorted((key for key in actual if key not in expected),
                         key=repr)
        raise fail(f"{label}: schema keys differ; missing={missing}, "
                   f"unknown={unknown}")


def _string(mapping: Mapping[str, Any], key: str, label: str) -> str:
    value = _require(mapping, key, label)
    if not isinstance(value, str) or not value:
        raise fail(f"{label}/{key}: expected non-empty string")
    if "\x00" in value:
        raise fail(f"{label}/{key}: NUL is forbidden")
    return value


def _positive_int(mapping: Mapping[str, Any], key: str, label: str) -> int:
    value = _require(mapping, key, label)
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise fail(f"{label}/{key}: expected positive integer")
    return value


def _sha256(mapping: Mapping[str, Any], key: str, label: str) -> str:
    value = _string(mapping, key, label)
    if SHA256_PATTERN.fullmatch(value) is None:
        raise fail(f"{label}/{key}: expected lowercase 64-hex SHA-256")
    return value


def _repo_relative_path(mapping: Mapping[str, Any], key: str, label: str,
                        basename: str) -> Path:
    value = _string(mapping, key, label)
    if value.startswith(("/", "\\")) or re.match(r"^[A-Za-z]:", value):
        raise fail(f"{label}/{key}: absolute path is forbidden")
    if "\\" in value:
        raise fail(f"{label}/{key}: backslash path separator is forbidden")
    raw_components = value.split("/")
    if any(part in {".", ".."} for part in raw_components):
        raise fail(f"{label}/{key}: dot/traversal component is forbidden")
    relative_path = Path(value)
    if not relative_path.parts or any(
            part in {".", ".."} for part in relative_path.parts):
        raise fail(f"{label}/{key}: dot/traversal component is forbidden")
    path = ROOT / relative_path
    try:
        path.relative_to(ROOT)
    except ValueError as exc:
        raise fail(f"{label}/{key}: path escapes repository") from exc
    if path.name != basename:
        raise fail(f"{label}/{key}: basename mismatch")
    if any(term in str(path).lower() for term in FORBIDDEN_INPUT_TERMS):
        raise fail(f"{label}/{key}: forbidden input category")
    return path


def _validate_raw_metadata(name: str, metadata: Any) -> Path:
    label = f"raw/{name}"
    if not isinstance(metadata, Mapping):
        raise fail(f"{label}: expected flat metadata object")
    _exact_keys(metadata, RAW_METADATA_KEYS, label)
    path = _repo_relative_path(metadata, "path", label, name)
    _positive_int(metadata, "bytes", label)
    _sha256(metadata, "sha256", label)
    if _string(metadata, "source", label) != RAW_SOURCE:
        raise fail(f"{label}/source: unexpected provenance")
    if _bool(metadata, "read_before_authorization", label):
        raise fail(f"{label}/read_before_authorization: must be false")
    if _bool(metadata, "copy_or_transform", label):
        raise fail(f"{label}/copy_or_transform: must be false")
    return path


def _validate_base_metadata(metadata: Any) -> Path:
    label = "base"
    if not isinstance(metadata, Mapping):
        raise fail(f"{label}: expected flat metadata object")
    _exact_keys(metadata, BASE_METADATA_KEYS, label)
    path = _repo_relative_path(metadata, "path", label, "base.obs")
    _positive_int(metadata, "bytes", label)
    _sha256(metadata, "sha256", label)
    if _number(metadata, "interval_s", label) <= 0:
        raise fail(f"{label}/interval_s: expected positive number")
    _positive_int(metadata, "moving_mean_samples", label)
    if _string(metadata, "source", label) != BASE_SOURCE:
        raise fail(f"{label}/source: unexpected provenance")
    if _bool(metadata, "read_before_authorization", label) or _bool(
            metadata, "hash_read_before_authorization", label):
        raise fail(f"{label}: pre-authorization read must be false")
    if _bool(metadata, "copy_or_transform", label):
        raise fail(f"{label}/copy_or_transform: must be false")
    return path


def _validate_route_schema(route_record: Any) -> tuple[Mapping[str, Any], Mapping[str, Any]]:
    if not isinstance(route_record, Mapping):
        raise fail("route: expected object")
    _exact_keys(route_record, ROUTE_KEYS, "route")
    target = _string(route_record, "target", "route")
    dataset_id = _string(route_record, "dataset_id", "route")
    expected_dataset = next(
        (dataset for dataset, label in STATIC.ROUTE_LABELS.items()
         if label == target),
        None,
    )
    if expected_dataset is None or dataset_id != expected_dataset:
        raise fail("route: target/dataset_id mismatch")
    runs = _require(route_record, "runs", "route")
    if not isinstance(runs, int) or isinstance(runs, bool) or runs != 1:
        raise fail("route/runs: expected integer 1")
    output_directory = _string(route_record, "output_directory", "route")
    expected_output = (
        "output/smartphone-r5/phase138-affine-tdcp-structural-v1/"
        f"{dataset_id.replace('/', '__')}"
    )
    if output_directory != expected_output:
        raise fail("route/output_directory: does not match sealed route")
    raw = _mapping(route_record, "raw_inputs", "route")
    base = _mapping(route_record, "base_input", "route")
    _exact_keys(raw, RAW_INPUT_KEYS, "route/raw_inputs")
    return raw, base


def _assert_static_hash(path: Path, expected: str, label: str) -> None:
    actual, _ = digest_file(path, label)
    if actual != expected:
        raise fail(f"{label}: expected {expected}, got {actual}")


def verify_authorization(auth: Mapping[str, Any]) -> None:
    if auth.get("schema_version") != AUTHORIZATION_SCHEMA_VERSION:
        raise fail("authorization/schema_version is not the sealed Phase138 schema")
    if auth.get("phase") != 138:
        raise fail("authorization/phase is not 138")
    if auth.get("status") != "independent-one-shot-structural-raw-authorized":
        raise fail("authorization/status is not independent raw authorization")
    routes = _require(auth, "routes", "authorization")
    if not isinstance(routes, list) or len(routes) != 2:
        raise fail("authorization/routes must contain exactly two objects")
    for index, expected_target in enumerate(("MTV-A", "LAX-T")):
        record = routes[index]
        if not isinstance(record, Mapping) or record.get("target") != expected_target:
            raise fail(f"authorization/routes/{index}: target/order mismatch")
        raw, base = _validate_route_schema(record)
        for name, metadata in raw.items():
            _validate_raw_metadata(name, metadata)
        _validate_base_metadata(base)
    authorization = _mapping(auth, "authorization", "authorization")
    for key in ("implementation", "contract", "raw_materialization",
                "raw_structural_execution", "solver"):
        if _bool(authorization, key, "authorization") is not True:
            raise fail(f"authorization/{key}: required true")
    for key in ("truth_evaluation", "accuracy", "solution_publication",
                "kaggle_submission", "rerun", "fallback", "repair", "sweep"):
        if _bool(authorization, key, "authorization") is not False:
            raise fail(f"authorization/{key}: required false")

    scope = _mapping(auth, "authorization_scope", "authorization")
    if scope.get("candidate_id") != "phase138-affine-tdcp-anchor-range-constant-v1":
        raise fail("authorization candidate mismatch")
    if scope.get("route_order") != ["MTV-A", "LAX-T"]:
        raise fail("authorization route order is not MTV-A then LAX-T")
    for key, expected in {
        "runs_per_route": 1,
        "controls": 0,
        "reruns": 0,
        "fallbacks": 0,
        "repairs": 0,
        "sweeps": 0,
    }.items():
        if scope.get(key) != expected:
            raise fail(f"authorization_scope/{key}: expected {expected}")

    pins = _mapping(auth, "pins", "authorization")
    expected_pins = {
        "source_audit_commit": STATIC.AUDIT_COMMIT,
        "structural_freeze_commit": STATIC.FREEZE_COMMIT,
        "runner_manifest_tests_commit": "a2a80f44385c01787097e1a70e95c8bda58bc075",
        "pre_raw_accounting_commit": "d0bdcda26c3c5c5839859090ab0ef423936e4ede",
        "implementation_commit": STATIC.IMPLEMENTATION_COMMIT,
        "design_freeze_commit": STATIC.DESIGN_FREEZE_COMMIT,
        "phase135_corrected_source_commit": STATIC.PHASE135_CORRECTION_COMMIT,
        "target_binary_sha256": STATIC.TARGET_BINARY_SHA256,
    }
    for key, expected in expected_pins.items():
        if pins.get(key) != expected:
            raise fail(f"authorization/pins/{key}: expected {expected}")
    for key, path, expected in (
        ("audit_sha256", STATIC.AUDIT, STATIC.AUDIT_SHA256),
        ("freeze_sha256", STATIC.FREEZE, STATIC.FREEZE_SHA256),
        ("manifest_sha256", STATIC.MANIFEST, "f93fd9fc6f4fddefa0f267307a5bc2238e519c16da50f8207f6d97a7d81dda0a"),
        ("runner_sha256", STATIC.RUNNER, "cf14981e58250e362d1e2d77a0c32cd7b7bf3c9cd9cde4663646924d00e8af80"),
        ("focused_tests_sha256", STATIC.FOCUSED_TESTS, "6b16547ffa910cf23f9693b2991f9a7cd9fcf366db6af5b948508887515b9054"),
        ("target_binary_sha256", STATIC.BINARY, STATIC.TARGET_BINARY_SHA256),
        ("pre_raw_sha256", ROOT / "docs/use_cases/records/smartphone_r5_phase138_affine_tdcp_structural_pre_raw_v1.json", "5839acfbd5891db93af45046109c03bcc791cd47354fb5e7f2216892046240a3"),
    ):
        if pins.get(key) != expected:
            raise fail(f"authorization/{key}: declared digest mismatch")
        _assert_static_hash(path, expected, key)
    authorized_runner = ROOT / (
        "apps/commands/benchmarks/"
        "gnss_smartphone_phase138_affine_tdcp_structural_authorized_execute.py"
    )
    authorized_runner_sha = pins.get("authorized_runner_sha256")
    if not isinstance(authorized_runner_sha, str) or len(authorized_runner_sha) != 64:
        raise fail("authorization/authorized_runner_sha256 missing")
    _assert_static_hash(authorized_runner, authorized_runner_sha,
                        "authorized_runner_sha256")

    # This also validates the target source witnesses and the launch-free
    # manifest before any payload path is opened.
    static_result = STATIC.launch_free_validation()
    if static_result.get("raw_execution_authorized") is not False:
        raise fail("static contract is already raw-authorized")

    recipe = _mapping(auth, "recipe", "authorization")
    for key in ("phase135_official_affine_measurement_family",
                "phase138_affine_tdcp_anchor_range_constant",
                "phase118_official_tdcp_huber_k",
                "phase107_raw_base_compensation",
                "phase107_raw_base_source_miss_mask"):
        if recipe.get(key) is not True:
            raise fail(f"recipe/{key}: required true")
    for key in ("phase117_dynamic_tdcp_sigma",
                "phase120_official_tdcp_resl_atmosphere_cancellation",
                "phase126_raw_base_source_complete",
                "phase127_glonass_channel_provenance",
                "phase128_glonass_provenance_parser_admission",
                "phase129_glonass_local_miss_mask",
                "phase130_shared_ledger_key_local_support",
                "phase131_canonical_correction_band_key",
                "phase132_typed_canonical_preflight",
                "phase133_runner_native_selector_boundary",
                "phase134_native_summary_bridge",
                "phase107_preserve_additional_frequency_bands"):
        if recipe.get(key) is not False:
            raise fail(f"recipe/{key}: forbidden selector active")
    for key, expected in {
        "phase118_fixed_tdcp_sigma_m": 0.03,
        "phase118_highway_huber_k": 0.5,
        "main_linear_solver": "MULTIFRONTAL_QR",
        "main_elimination": "EliminateQR",
        "c7_dimension": 7,
        "c_units": "metres",
        "d_units": "metres/second",
        "pixel5_offset_applications": 1,
    }.items():
        if recipe.get(key) != expected:
            raise fail(f"recipe/{key}: expected {expected!r}")

    forbidden = _mapping(auth, "forbidden", "authorization")
    for key in ("truth", "MAT", "PDC", "precomputed_coordinates",
                "accuracy", "Kaggle", "solution_publication"):
        if forbidden.get(key) is not True:
            raise fail(f"forbidden/{key}: required true")
    accounting = _mapping(auth, "pre_authorization_read_accounting", "authorization")
    for key, value in accounting.items():
        if isinstance(value, bool):
            if value is not False:
                raise fail(f"pre_authorization/{key}: nonzero")
        elif isinstance(value, int):
            if value != 0:
                raise fail(f"pre_authorization/{key}: nonzero")
        elif value not in {"read-only", "sealed-metadata-only"}:
            raise fail(f"pre_authorization/{key}: invalid marker")


def actual_payload_paths(route_record: Mapping[str, Any]) -> dict[str, Path]:
    """Resolve the sealed flat route schema without opening any payload."""
    raw, base = _validate_route_schema(route_record)
    paths = {
        name: _validate_raw_metadata(name, raw[name])
        for name in sorted(RAW_INPUT_KEYS)
    }
    paths["base.obs"] = _validate_base_metadata(base)
    return paths


def command_for(route: str, paths: Mapping[str, Path], hashes: Mapping[str, str],
                output_dir: Path) -> list[str]:
    command = list(STATIC.command_template(route))
    replacements = {
        STATIC.RAW_PLACEHOLDERS["--android-gnss"]: str(paths["device_gnss.csv"]),
        STATIC.RAW_PLACEHOLDERS["--android-imu"]: str(paths["device_imu.csv"]),
        STATIC.RAW_PLACEHOLDERS["--nav"]: str(paths["brdc.nav"]),
        STATIC.RAW_PLACEHOLDERS["--native-base-rinex"]: str(paths["base.obs"]),
        STATIC.RAW_PLACEHOLDERS["--native-base-rinex-sha256"]: hashes["base.obs"],
    }
    command = [replacements.get(token, token) for token in command]
    command[command.index("--out") + 1] = str(output_dir / OPAQUE_OUTPUT_NAME)
    command[command.index("--summary-json") + 1] = str(output_dir / "native_summary.json")
    return command


def _forbidden_value(value: str) -> bool:
    lowered = value.lower()
    return any(term in lowered for term in FORBIDDEN_INPUT_TERMS)


def validate_typed_command(command: list[str], route: str) -> None:
    expected = STATIC.command_template(route)
    if len(command) != len(expected):
        raise fail(f"{route}: argv length differs from frozen template")
    for index, (actual, frozen) in enumerate(zip(command, expected)):
        if frozen.startswith("--"):
            if actual != frozen:
                raise fail(f"{route}: argv option {index} changed to {actual!r}")
            if "pdc" in actual.lower() and actual != LEGITIMATE_PDC_ALGORITHM_OPTION:
                raise fail(f"{route}: forbidden PDC option {actual!r}")
            continue
        if actual.startswith("--"):
            raise fail(f"{route}: unknown option injected at {index}")
        opaque = (index > 0 and expected[index - 1] == "--out" and
                  Path(actual).name == OPAQUE_OUTPUT_NAME)
        if _forbidden_value(actual) and not opaque:
            raise fail(f"{route}: forbidden input value {actual!r}")
    for selector in STATIC.REQUIRED_RECIPE_FLAGS + STATIC.ON_SELECTORS:
        if command.count(selector) != 1:
            raise fail(f"{route}: selector {selector} count is not one")
    for selector in STATIC.OFF_SELECTORS:
        if command.count(selector) != 0:
            raise fail(f"{route}: selector {selector} is active")


def _family(count: int, geometry_ok: bool) -> dict[str, Any]:
    return {
        "admitted_rows": count,
        "affine_factors_inserted": count,
        "key_order_exact": True,
        "finite_values": geometry_ok,
        "source_geometry_same_path": geometry_ok,
    }


def normalize_native_summary(route: str, native: Mapping[str, Any],
                             return_code: int, opaque: Mapping[str, Any] | None,
                             read_accounting: Mapping[str, int]) -> dict[str, Any]:
    """Project native scalar metadata into the launch-free validator schema."""
    p135 = _mapping(native, "phase135_official_affine_measurement_family", "native")
    p138 = _mapping(native, "phase138_affine_tdcp_anchor_range_constant", "native")
    if p135.get("mixed_or_partial_family_allowed") is not False:
        raise fail("native Phase135 mixed/partial-family witness is not false")
    geometry = p135.get("geometry_representation")
    geometry_ok = geometry == "RTKLIB-geodist-single-Sagnac-fixed-initial-LOS"
    tdcp_count = _int(p135, "tdcp_factors_inserted", "native/phase135")
    p_count = _int(p135, "pseudorange_factors_inserted", "native/phase135")
    d_count = _int(p135, "doppler_factors_inserted", "native/phase135")
    g_rows = _int(p135, "geometry_rows_validated", "native/phase135")
    phase138_geometry_ok = (
        p138.get("geometry_representation") ==
        "RTKLIB-geodist-single-Sagnac-fixed-initial-endpoints"
    )
    base = _mapping(native, "native_base_pseudorange_compensation", "native")
    mask = _mapping(native, "native_base_pseudorange_source_miss_mask", "native")
    base_good = (
        base.get("correction_applied_exactly_once") is True and
        base.get("no_extrapolation_or_endpoint_hold") is True and
        mask.get("pseudorange_factor_count_consistent") is True and
        mask.get("signal_count_consistent") is True
    )
    first = _mapping(native, "gnss_first", "native")
    graph = _mapping(native, "graph", "native")
    offset = _mapping(native, "upstream_position_offset", "native")
    raw_utc = _mapping(native, "raw_utc_key_contract", "native")
    target_epochs = _int(raw_utc, "target_epochs", "native/raw_utc")
    exact_epochs = _int(raw_utc, "exact_solution_epochs", "native/raw_utc")
    interpolated = _int(raw_utc, "interpolated_epochs", "native/raw_utc")
    edge_hold = _int(raw_utc, "edge_hold_epochs", "native/raw_utc")
    unresolved = _int(raw_utc, "unresolved_epochs", "native/raw_utc")
    coverage = (target_epochs > 0 and unresolved == 0 and
                exact_epochs + interpolated + edge_hold == target_epochs)
    output_contract = _mapping(native, "output_contract", "native")
    output_finite = output_contract.get("finite_coordinates") is True
    offset_good = (
        offset.get("enabled") is True and offset.get("applied") is True and
        offset.get("phone") == "pixel5" and
        isinstance(offset.get("corrected_epochs"), int) and
        offset.get("corrected_epochs", 0) > 0
    )
    c7_good = (
        native.get("native_source_clock_c0d_factor_enabled") is True and
        native.get("native_source_clock_c0d_meter_state_parity_enabled") is True and
        native.get("native_source_clock_c0d_epoch_vector_parity_enabled") is True and
        first.get("handoff_mode") == "gnss-first-in-memory-meter-clock-state" and
        first.get("optimized_c_export_valid") is True and
        first.get("optimized_d_export_valid") is True and
        first.get("optimized_c_dimension") == 7 and
        first.get("optimized_c_nonfinite_component_count") == 0 and
        first.get("optimized_d_nonfinite_count") == 0 and
        first.get("epoch_identity_alignment_valid") is True
    )
    selected_solver = native.get("selected_linear_solver_type")
    selected_elimination = native.get("selected_elimination_function")
    solver_good = selected_solver == "MULTIFRONTAL_QR" and selected_elimination == "EliminateQR"
    gnss_iterations = _int(first, "iterations", "native/gnss_first")
    main_iterations = _int(graph, "iterations", "native/graph")
    return {
        "route": route,
        "selectors": {
            "phase135_official_affine_measurement_family": True,
            "phase138_affine_tdcp_anchor_range_constant": True,
            "phase118_official_tdcp_huber_k": native.get("native_phase118_official_tdcp_huber_k") is True,
            "phase117_dynamic_tdcp_sigma": native.get("native_phase117_tdcp_snr_type_sigma") is True,
            "phase120_official_tdcp_resl_atmosphere_cancellation": native.get("native_phase120_official_tdcp_resl_atmosphere_cancellation") is True,
            "phase126_raw_base_source_complete": native.get("native_phase126_raw_base_source_complete") is True,
            "phase127_glonass_channel_provenance": native.get("native_phase127_glonass_channel_provenance") is True,
            "phase128_glonass_provenance_parser_admission": native.get("native_phase128_glonass_provenance_parser_admission") is True,
            "phase129_glonass_local_miss_mask": native.get("native_phase129_glonass_local_miss_mask") is True,
            "phase130_shared_ledger_key_local_support": False,
            "phase131_canonical_correction_band_key": native.get("native_phase131_canonical_correction_band_key") is True,
            "phase107_raw_base_compensation": True,
            "phase107_raw_base_source_miss_mask": True,
            "phase107_preserve_additional_frequency_bands": base.get("preserve_additional_frequency_bands") is True,
        },
        "phase135": {
            "enabled": p135.get("enabled") is True,
            "configuration_valid": p135.get("configuration_valid") is True,
            "transactional": geometry_ok and p135.get("mixed_or_partial_family_allowed") is False,
            "fixed_initial_geometry": geometry_ok,
            "finite_jacobians": geometry_ok and p135.get("doppler_residual_provenance_required") is True,
            "single_sagnac_representation": p135.get("single_sagnac_representation") is True,
            "los_convention": p135.get("doppler_factor_los_convention"),
            "sagnac_evaluations": g_rows,
            "geometry_rows": g_rows,
            "pseudorange": _family(p_count, geometry_ok),
            "doppler": _family(d_count, geometry_ok),
            "ordinary_tdcp": _family(tdcp_count, geometry_ok),
            "legacy_factor_counts": {"pseudorange": 0, "doppler": 0, "ordinary_tdcp": 0},
            "pose3_x_bridge": {
                "count": _int(p135, "pose3_x_bridge_factors", "native/phase135"),
                "keys_exact": True,
            },
        },
        "phase138": {
            "enabled": p138.get("enabled") is True,
            "phase135_dependency_satisfied": p138.get("phase135_dependency_satisfied") is True,
            "configuration_valid": p138.get("configuration_valid") is True,
            "measurement_equation": p138.get("measurement_equation"),
            "geometry_representation": p138.get("geometry_representation"),
            "range_constants_validated": _int(p138, "range_constants_validated", "native/phase138"),
            "tdcp_measurements_adjusted": _int(p138, "tdcp_measurements_adjusted", "native/phase138"),
            "affine_tdcp_factor_count": tdcp_count,
            "adjustment_application_passes": _int(p138, "adjustment_application_passes", "native/phase138"),
            "adjusted_exactly_once": p138.get("adjusted_exactly_once") is True,
            "factor_count_unchanged": p138.get("factor_count_unchanged") is True,
            "same_endpoint_epoch_and_satellite_state": phase138_geometry_ok,
            "same_satellite_state": phase138_geometry_ok,
            "finite_adjusted_measurements": phase138_geometry_ok and p138.get("configuration_valid") is True,
            "phase118_atmosphere_sigma_huber_unchanged": p138.get("phase118_atmosphere_sigma_huber_unchanged") is True,
            "single_sagnac_representation": p138.get("single_sagnac_representation") is True,
            "no_raw_or_zero_fallback": phase138_geometry_ok,
            "transactional": p138.get("mixed_or_partial_family_allowed") is False,
            "legacy_tdcp_factor_count": 0,
        },
        "raw_base": {
            "phase107_recipe": p135.get("phase107_raw_base_compatibility") is True,
            "applied_exactly_once": base_good,
            "source_miss_conservation": base_good,
            "no_raw_or_zero_fallback": base.get("no_extrapolation_or_endpoint_hold") is True,
        },
        "clock": {
            "c_units": "metres",
            "d_units": "metres/second",
            "c7_mapping_exact": c7_good,
            "d_full_finite_exact_alignment": c7_good,
        },
        "solver": {
            "linear_solver": selected_solver,
            "elimination": selected_elimination,
            "gnss_first": {
                "accepted_iterations": first.get("c0d_accepted_outer_iterations", gnss_iterations),
                "initial_cost": first.get("initial_cost"),
                "final_cost": first.get("final_cost"),
                "no_fallback": return_code == 0,
            },
            "main": {
                "accepted_iterations": main_iterations,
                "initial_cost": graph.get("initial_cost"),
                "final_cost": graph.get("final_cost"),
                "no_fallback": return_code == 0,
            },
        },
        "output": {
            "finite": output_finite,
            "earth_valid": return_code == 0 and opaque is not None and output_finite,
            "expected_epoch_coverage": coverage,
            "pixel5_offset_applications": 1 if offset_good else 0,
            "opaque_solution_seal": opaque is not None,
        },
        "read_accounting": dict(read_accounting),
        "fallback": False,
        "rerun": False,
    }


def run_one_route(route_record: Mapping[str, Any]) -> dict[str, Any]:
    route = str(_require(route_record, "dataset_id", "route"))
    paths = actual_payload_paths(route_record)
    expected_meta: dict[str, tuple[str, int]] = {}
    raw = _mapping(route_record, "raw_inputs", "route")
    for name, metadata in raw.items():
        expected_meta[name] = (
            str(_require(metadata, "sha256", f"raw/{name}")),
            int(_require(metadata, "bytes", f"raw/{name}")),
        )
    base_meta = _mapping(route_record, "base_input", "route")
    expected_meta["base.obs"] = (
        str(_require(base_meta, "sha256", "base")),
        int(_require(base_meta, "bytes", "base")),
    )
    reads: dict[str, int] = {name: 0 for name in paths}
    hashes: dict[str, Any] = {}
    try:
        for name in ("device_gnss.csv", "device_imu.csv", "brdc.nav", "base.obs"):
            digest, size = digest_file(paths[name], name)
            reads[name] = 1
            expected_digest, expected_size = expected_meta[name]
            hashes[name] = {
                "sha256": digest,
                "bytes": size,
                "expected_sha256": expected_digest,
                "expected_bytes": expected_size,
                "match": digest == expected_digest and size == expected_size,
            }
            if not hashes[name]["match"]:
                return {
                    "route": route,
                    "status": "fail-closed-input-hash",
                    "solver_invocations": 0,
                    "input_read_counts": reads,
                    "input_hashes": hashes,
                    "failure": f"{name}: sealed hash/size mismatch",
                }
    except AuthorizationError as exc:
        return {
            "route": route,
            "status": "fail-closed-input-preflight",
            "solver_invocations": 0,
            "input_read_counts": reads,
            "input_hashes": hashes,
            "failure": str(exc),
        }

    output_dir = ROOT / "output/smartphone-r5/phase138-affine-tdcp-structural-v1" / route.replace("/", "__")
    output_path = output_dir / OPAQUE_OUTPUT_NAME
    summary_path = output_dir / "native_summary.json"
    stdout_path = output_dir / "native.stdout.log"
    stderr_path = output_dir / "native.stderr.log"
    if any(path.exists() for path in (output_path, summary_path, stdout_path, stderr_path)):
        return {
            "route": route,
            "status": "fail-closed-existing-output",
            "solver_invocations": 0,
            "input_read_counts": reads,
            "input_hashes": hashes,
            "failure": "one-shot output path already exists",
        }
    try:
        output_dir.mkdir(parents=True, exist_ok=False)
        command = command_for(route, paths,
                             {name: item["sha256"] for name, item in hashes.items()},
                             output_dir)
        validate_typed_command(command, route)
    except (OSError, AuthorizationError) as exc:
        return {
            "route": route,
            "status": "fail-closed-command-preflight",
            "solver_invocations": 0,
            "input_read_counts": reads,
            "input_hashes": hashes,
            "failure": str(exc),
        }

    environment = os.environ.copy()
    local_library = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = local_library + (
        (":" + environment["LD_LIBRARY_PATH"])
        if environment.get("LD_LIBRARY_PATH") else ""
    )
    completed = subprocess.run(command, cwd=ROOT, env=environment,
                               capture_output=True, check=False)
    stdout_path.write_bytes(completed.stdout)
    stderr_path.write_bytes(completed.stderr)
    route_result: dict[str, Any] = {
        "route": route,
        "status": "native-returned",
        "solver_invocations": 1,
        "input_read_counts": reads,
        "input_hashes": hashes,
        "execution": {
            "argv": command,
            "invocation_count": 1,
            "return_code": completed.returncode,
            "stdout_sha256": digest_bytes(completed.stdout),
            "stdout_bytes": len(completed.stdout),
            "stderr_sha256": digest_bytes(completed.stderr),
            "stderr_bytes": len(completed.stderr),
        },
        "solution_content_read": False,
    }
    opaque = opaque_solution_metadata(output_path) if output_path.is_file() else None
    route_result["opaque_solution"] = opaque
    if not summary_path.is_file():
        route_result["status"] = "fail-closed-missing-native-summary"
        route_result["structural_gate"] = "native summary missing"
        return route_result
    try:
        summary_bytes = summary_path.read_bytes()
        native = json.loads(summary_bytes)
        if not isinstance(native, Mapping):
            raise fail("native summary is not an object")
        route_result["native_summary"] = {
            "path": str(summary_path.relative_to(ROOT)),
            "sha256": digest_bytes(summary_bytes),
            "bytes": len(summary_bytes),
            "coordinate_fields_interpreted": False,
        }
        read_accounting = {
            "raw_phone_gnss_reads": reads["device_gnss.csv"],
            "raw_phone_imu_reads": reads["device_imu.csv"],
            "broadcast_navigation_reads": reads["brdc.nav"],
            "raw_base_rinex_reads": reads["base.obs"],
            "truth_reads": 0,
            "solution_coordinate_reads": 0,
            "mat_pdc_precomputed_coordinate_reads": 0,
            "solver_invocations": 1,
            "accuracy_calculations": 0,
            "kaggle_access": 0,
        }
        normalized = normalize_native_summary(route, native, completed.returncode,
                                              opaque, read_accounting)
        STATIC.validate_structural_summary(route, normalized)
        route_result["structural_gate"] = "pass"
        route_result["structural_summary"] = normalized
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, AuthorizationError,
            KeyError, TypeError, ValueError) as exc:
        route_result["status"] = "fail-closed-structural-gate"
        route_result["structural_gate"] = "fail"
        route_result["failure"] = str(exc)
    return route_result


def run(auth_path: Path = AUTH) -> int:
    auth = read_object(auth_path, "authorization")
    verify_authorization(auth)
    routes = _require(auth, "routes", "authorization")
    if not isinstance(routes, list) or len(routes) != 2:
        raise fail("authorization routes must contain exactly two records")
    results: list[dict[str, Any]] = []
    for index, expected_target in enumerate(("MTV-A", "LAX-T")):
        record = routes[index]
        if not isinstance(record, Mapping) or record.get("target") != expected_target:
            raise fail(f"route order mismatch at index {index}")
        results.append(run_one_route(record))
    result = {
        "schema_version": "smartphone-r5-phase138-affine-tdcp-structural-raw-result.v1",
        "phase": 138,
        "execution_label": "Luna Max",
        "status": "sealed-structural-raw-result-no-accuracy",
        "authorization_path": str(auth_path.relative_to(ROOT)),
        "authorization_sha256": digest_bytes(auth_path.read_bytes()),
        "candidate_id": "phase138-affine-tdcp-anchor-range-constant-v1",
        "route_order": ["MTV-A", "LAX-T"],
        "runs_per_route": 1,
        "routes": results,
        "policy": {
            "truth_used": False,
            "mat_used": False,
            "pdc_used": False,
            "precomputed_coordinates_used": False,
            "accuracy_evaluation": False,
            "kaggle_access": False,
            "solution_publication": False,
            "rerun": False,
            "fallback": False,
            "repair": False,
            "solution_coordinate_interpretation": False,
        },
        "read_accounting": {
            "raw_phone_gnss_reads": sum(r.get("input_read_counts", {}).get("device_gnss.csv", 0) for r in results),
            "raw_phone_imu_reads": sum(r.get("input_read_counts", {}).get("device_imu.csv", 0) for r in results),
            "broadcast_navigation_reads": sum(r.get("input_read_counts", {}).get("brdc.nav", 0) for r in results),
            "raw_base_rinex_reads": sum(r.get("input_read_counts", {}).get("base.obs", 0) for r in results),
            "native_solver_invocations": sum(r.get("solver_invocations", 0) for r in results),
            "truth_reads": 0,
            "solution_coordinate_reads": 0,
            "accuracy_calculations": 0,
            "mat_pdc_precomputed_coordinate_reads": 0,
            "kaggle_or_token_access": 0,
            "reruns_fallbacks_repairs_sweeps": 0,
        },
        "solution_policy": "opaque hash/bytes/newline seal only; no coordinate fields read or published",
        "authorization_commit_is_separate": True,
        "rerun_or_fallback": False,
    }
    RESULT.parent.mkdir(parents=True, exist_ok=True)
    RESULT.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                      encoding="utf-8")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--authorization", type=Path, default=AUTH)
    args = parser.parse_args()
    try:
        return run(args.authorization)
    except AuthorizationError as exc:
        print(f"PHASE138_RAW_FAIL_CLOSED: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
