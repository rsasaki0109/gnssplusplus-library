#!/usr/bin/env python3
"""Independent one-shot Phase141 structural raw executor.

The Phase141 launch-free contract proves the native telemetry schema without
opening a route payload.  This module is the separate authorization boundary:
it verifies every sealed pin first, then hashes each authorized raw input once
and invokes the pinned native binary at most once for each route, in the
sealed MTV-A -> LAX-T order.  Native telemetry is validated in place; it is
never normalized, inferred, or overwritten.  The output solution is sealed
only by opaque bytes/newline metadata and no coordinate field is interpreted.

Truth, MAT, PDC input, precomputed-coordinate, accuracy, Kaggle, retry,
fallback, repair, and rerun paths are intentionally absent.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import subprocess
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[3]
AUTH = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase141_telemetry_schema_raw_authorization_v1.json"
)
RESULT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase141_telemetry_schema_raw_result_v1.json"
)
STATIC_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase138_affine_tdcp_structural.py"
)
PHASE141_CONTRACT_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase141_telemetry_schema_structural.py"
)
PHASE138_AUTH_HELPERS_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase138_affine_tdcp_structural_authorized_execute.py"
)
AUTHORIZED_RUNNER = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase141_telemetry_schema_authorized_execute.py"
)
PHASE141_MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase141_telemetry_schema_manifest_v1.json"
)
PHASE141_PRE_RAW = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase141_telemetry_schema_pre_raw_v1.json"
)
OPAQUE_OUTPUT_NAME = "opaque_solution_output.csv"
OUTPUT_ROOT = "output/smartphone-r5/phase141-telemetry-schema-structural-v1"
PHASE141_SELECTOR = "--native-phase141-telemetry-schema"
LEGITIMATE_PDC_ALGORITHM_OPTION = "--native-pdc-imu-tdcp-no-bridge"
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}\Z")
AUTHORIZATION_SCHEMA_VERSION = (
    "smartphone-r5-phase141-telemetry-schema-raw-authorization.v1"
)
NATIVE_SCHEMA = "smartphone-r5-native-fgo-phase141-telemetry.v1"
EQUATION_ID = "phase138-affine-tdcp-anchor-range-constant-v1"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
TARGETS = ("MTV-A", "LAX-T")
ROUTE_TARGETS = dict(zip(ROUTES, TARGETS))
RAW_INPUT_KEYS = frozenset({"device_gnss.csv", "device_imu.csv", "brdc.nav"})
ROUTE_KEYS = frozenset({
    "target", "dataset_id", "runs", "raw_inputs", "base_input",
    "output_directory",
})
RAW_SOURCE = "sealed raw-only input lineage"
BASE_SOURCE = "raw base RINEX; station coordinate from header only"
FORBIDDEN_INPUT_TERMS = (
    ".mat", "truth", "ground_truth", "precomputed", "kaggle", "token",
)


def _load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load {name}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


STATIC = _load_module(STATIC_PATH, "phase141_static_phase138_contract")
HELPERS = _load_module(PHASE138_AUTH_HELPERS_PATH,
                       "phase141_phase138_authorization_helpers")
CONTRACT = _load_module(PHASE141_CONTRACT_PATH,
                        "phase141_launch_free_contract")


class AuthorizationError(ValueError):
    """A fail-closed authorization, input, or structural violation."""


def fail(message: str) -> AuthorizationError:
    return AuthorizationError(message)


def _reject_duplicate_json_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
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
    except (OSError, UnicodeDecodeError, json.JSONDecodeError,
            AuthorizationError) as exc:
        raise fail(f"{label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label}: expected object")
    return value


def digest_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def digest_static(path: Path, label: str) -> str:
    lowered = path.name.lower()
    if lowered.endswith((".csv", ".nav", ".obs", ".mat")):
        raise fail(f"static payload hash forbidden for {label}")
    if any(term in lowered for term in ("truth", "ground_truth", "precomputed")):
        raise fail(f"forbidden artifact hash for {label}")
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def digest_file(path: Path, label: str) -> tuple[str, int]:
    """Read and hash one authorized payload; callers invoke this once."""
    if not path.is_file():
        raise fail(f"{label}: missing authorized input {path}")
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            size += len(chunk)
            digest.update(chunk)
    return digest.hexdigest(), size


def opaque_solution_metadata(path: Path) -> dict[str, Any]:
    """Hash bytes and newline count only; never parse a solution field."""
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


def _equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def _bool(mapping: Mapping[str, Any], key: str, label: str) -> bool:
    value = _require(mapping, key, label)
    if not isinstance(value, bool):
        raise fail(f"{label}/{key}: expected boolean")
    return value


def _zero_accounting(value: Any, label: str) -> None:
    if not isinstance(value, Mapping):
        raise fail(f"{label}: expected object")
    account = value
    for key, item in account.items():
        if isinstance(item, bool):
            _equal(item, False, f"{label}/{key}")
        elif isinstance(item, int):
            _equal(item, 0, f"{label}/{key}")
        elif item not in {"read-only", "sealed-metadata-only"}:
            raise fail(f"{label}/{key}: invalid pre-raw marker {item!r}")


def _sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA256_PATTERN.fullmatch(value) is None:
        raise fail(f"{label}: expected lowercase SHA-256")
    return value


def _verify_raw_route(record: Mapping[str, Any], index: int) -> None:
    label = f"authorization/routes/{index}"
    if set(record) != set(ROUTE_KEYS):
        raise fail(f"{label}: route schema keys differ")
    target = _require(record, "target", label)
    dataset_id = _require(record, "dataset_id", label)
    _equal(target, TARGETS[index], f"{label}/target")
    _equal(dataset_id, ROUTES[index], f"{label}/dataset_id")
    _equal(_require(record, "runs", label), 1, f"{label}/runs")
    expected_output = f"{OUTPUT_ROOT}/{dataset_id.replace('/', '__')}"
    _equal(_require(record, "output_directory", label), expected_output,
           f"{label}/output_directory")
    raw = _mapping(record, "raw_inputs", label)
    _equal(set(raw), set(RAW_INPUT_KEYS), f"{label}/raw_inputs/keys")
    for name in sorted(RAW_INPUT_KEYS):
        metadata = raw[name]
        # These helpers validate the flat schema, safe repository-relative
        # path, provenance, positive size, and sealed digest without opening
        # the payload.
        HELPERS._validate_raw_metadata(name, metadata)
    HELPERS._validate_base_metadata(_require(record, "base_input", label))


def _verify_pin_file(pins: Mapping[str, Any], key: str, path: Path,
                     expected: str, label: str) -> None:
    _equal(pins.get(key), expected, f"authorization/pins/{key}")
    _equal(digest_static(path, label), expected, f"{label}/sha256")


def verify_authorization(auth: Mapping[str, Any]) -> None:
    """Verify all static pins and route metadata before any payload read."""
    _equal(auth.get("schema_version"), AUTHORIZATION_SCHEMA_VERSION,
           "authorization/schema_version")
    _equal(auth.get("phase"), 141, "authorization/phase")
    _equal(auth.get("status"), "independent-one-shot-structural-raw-authorized",
           "authorization/status")

    authorization = _mapping(auth, "authorization", "authorization")
    for key in ("implementation", "contract", "raw_materialization",
                "raw_structural_execution", "solver"):
        _equal(_bool(authorization, key, "authorization"), True,
               f"authorization/{key}")
    for key in ("truth_evaluation", "accuracy", "solution_publication",
                "kaggle_submission", "rerun", "fallback", "repair", "sweep"):
        _equal(_bool(authorization, key, "authorization"), False,
               f"authorization/{key}")

    scope = _mapping(auth, "authorization_scope", "authorization")
    _equal(scope.get("candidate_id"),
           "phase141-native-authoritative-telemetry-schema-v1",
           "authorization_scope/candidate_id")
    _equal(scope.get("route_order"), list(TARGETS),
           "authorization_scope/route_order")
    for key in ("runs_per_route", "controls", "reruns", "fallbacks",
                "repairs", "sweeps"):
        expected = 1 if key == "runs_per_route" else 0
        _equal(scope.get(key), expected, f"authorization_scope/{key}")

    routes = _require(auth, "routes", "authorization")
    if not isinstance(routes, list) or len(routes) != 2:
        raise fail("authorization/routes must contain exactly two objects")
    for index, record in enumerate(routes):
        if not isinstance(record, Mapping):
            raise fail(f"authorization/routes/{index}: expected object")
        _verify_raw_route(record, index)

    pins = _mapping(auth, "pins", "authorization")
    full_commit_pins = {
        "phase141_audit_commit": "de71c394cafc955fe5b2296a2d91afb382e80c50",
        "phase141_freeze_commit": "d105e0a826c967ec6ccb978a2860d3a1557f54ff",
        "phase141_implementation_base_commit": "0cbc988006a9a52817af74fd8add141914563119",
        "phase141_equation_fix_commit": "cbb1f4e48c439270264ec18904b0ef23ce8b8111",
        "phase141_authority_fix_commit": "6c752c2264c7444da6eb4833ddbbb28ee43ce62a",
        "phase141_final_runner_manifest_commit": "b629ef164f35a7783f2acc1902921d6e76136c70",
        "phase141_pre_raw_commit": "6d55e2b9a963e9b2b598ac77d8861aeb5b7e0bb0",
    }
    for key, expected in full_commit_pins.items():
        _equal(pins.get(key), expected, f"authorization/pins/{key}")

    # The launch-free contract rechecks the audit/freeze/manifest/source and
    # binary pins before this function can authorize a payload read.
    qualified = CONTRACT.launch_free_validation()
    _equal(qualified.get("raw_execution_authorized"), False,
           "Phase141 launch-free contract/raw_execution_authorized")
    _verify_pin_file(pins, "phase141_audit_sha256", CONTRACT.AUDIT,
                     CONTRACT.AUDIT_SHA256, "Phase141 audit")
    _verify_pin_file(pins, "phase141_freeze_sha256", CONTRACT.FREEZE,
                     CONTRACT.FREEZE_SHA256, "Phase141 freeze")
    manifest_sha = digest_static(PHASE141_MANIFEST, "Phase141 manifest")
    _verify_pin_file(pins, "phase141_manifest_sha256", PHASE141_MANIFEST,
                     manifest_sha, "Phase141 manifest")
    pre_raw_sha = digest_static(PHASE141_PRE_RAW, "Phase141 pre-raw")
    _verify_pin_file(pins, "phase141_pre_raw_sha256", PHASE141_PRE_RAW,
                     pre_raw_sha, "Phase141 pre-raw")
    _verify_pin_file(pins, "authorized_runner_sha256", AUTHORIZED_RUNNER,
                     _sha(pins.get("authorized_runner_sha256"),
                          "authorization/pins/authorized_runner_sha256"),
                     "Phase141 authorized runner")
    _verify_pin_file(pins, "focused_tests_sha256", CONTRACT.FOCUSED_TESTS,
                     digest_static(CONTRACT.FOCUSED_TESTS,
                                   "Phase141 focused tests"),
                     "Phase141 focused tests")
    _verify_pin_file(pins, "target_binary_sha256", CONTRACT.BINARY,
                     CONTRACT.TARGET_BINARY_SHA256, "Phase141 target binary")
    _equal(pins.get("authorized_runner_path"),
           str(AUTHORIZED_RUNNER.relative_to(ROOT)),
           "authorization/pins/authorized_runner_path")
    _equal(pins.get("target_binary_path"),
           str(CONTRACT.BINARY.relative_to(ROOT)),
           "authorization/pins/target_binary_path")

    source_pins = _mapping(auth, "source_pins", "authorization")
    _equal(source_pins, CONTRACT.validate_static_sources(),
           "authorization/source_pins")

    recipe = _mapping(auth, "recipe", "authorization")
    required_true = (
        "phase135_official_affine_measurement_family",
        "phase138_affine_tdcp_anchor_range_constant",
        "phase118_official_tdcp_huber_k",
        "phase107_raw_base_compensation",
        "phase107_raw_base_source_miss_mask",
        "phase141_telemetry_schema",
    )
    for key in required_true:
        _equal(recipe.get(key), True, f"recipe/{key}")
    forbidden_selectors = (
        "phase117_dynamic_tdcp_sigma",
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
        "phase107_preserve_additional_frequency_bands",
    )
    for key in forbidden_selectors:
        _equal(recipe.get(key), False, f"recipe/{key}")
    for key, expected in {
        "phase118_fixed_tdcp_sigma_m": 0.03,
        "phase118_highway_huber_k": 0.5,
        "main_linear_solver": "MULTIFRONTAL_QR",
        "main_elimination": "EliminateQR",
        "c7_dimension": 7,
        "c_units": "metres",
        "d_units": "metres/second",
        "pixel5_offset_applications": 1,
        "pdc_input": False,
        "truth_input": False,
        "mat_input": False,
        "precomputed_coordinate_input": False,
    }.items():
        _equal(recipe.get(key), expected, f"recipe/{key}")

    forbidden = _mapping(auth, "forbidden", "authorization")
    for key in ("truth", "MAT", "PDC", "precomputed_coordinates",
                "accuracy", "Kaggle", "solution_publication"):
        _equal(forbidden.get(key), True, f"forbidden/{key}")
    _zero_accounting(auth.get("pre_authorization_read_accounting"),
                     "authorization/pre_authorization_read_accounting")


def actual_payload_paths(record: Mapping[str, Any]) -> dict[str, Path]:
    """Validate flat sealed metadata and resolve paths without opening them."""
    _verify_raw_route(record, TARGETS.index(record.get("target")))
    raw = _mapping(record, "raw_inputs", "route")
    paths = {
        name: HELPERS._validate_raw_metadata(name, raw[name])
        for name in sorted(RAW_INPUT_KEYS)
    }
    paths["base.obs"] = HELPERS._validate_base_metadata(record["base_input"])
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
    command.insert(command.index(STATIC.PHASE118_SELECTOR) + 1,
                   PHASE141_SELECTOR)
    command[command.index("--out") + 1] = str(output_dir / OPAQUE_OUTPUT_NAME)
    command[command.index("--summary-json") + 1] = str(
        output_dir / "native_summary.json"
    )
    return command


def validate_command(route: str, command: list[str],
                     paths: Mapping[str, Path], hashes: Mapping[str, str],
                     output_dir: Path) -> None:
    expected = list(STATIC.command_template(route))
    replacements = {
        STATIC.RAW_PLACEHOLDERS["--android-gnss"]: str(paths["device_gnss.csv"]),
        STATIC.RAW_PLACEHOLDERS["--android-imu"]: str(paths["device_imu.csv"]),
        STATIC.RAW_PLACEHOLDERS["--nav"]: str(paths["brdc.nav"]),
        STATIC.RAW_PLACEHOLDERS["--native-base-rinex"]: str(paths["base.obs"]),
        STATIC.RAW_PLACEHOLDERS["--native-base-rinex-sha256"]: hashes["base.obs"],
    }
    expected = [replacements.get(token, token) for token in expected]
    expected.insert(expected.index(STATIC.PHASE118_SELECTOR) + 1,
                    PHASE141_SELECTOR)
    expected[expected.index("--out") + 1] = str(output_dir / OPAQUE_OUTPUT_NAME)
    expected[expected.index("--summary-json") + 1] = str(
        output_dir / "native_summary.json"
    )
    _equal(command, expected, f"command/{route}")
    for selector in STATIC.REQUIRED_RECIPE_FLAGS + STATIC.ON_SELECTORS + (
            PHASE141_SELECTOR,):
        _equal(command.count(selector), 1, f"command/{route}/{selector}")
    for selector in STATIC.OFF_SELECTORS:
        _equal(command.count(selector), 0, f"command/{route}/{selector}")
    for option in ("--android-gnss", "--android-imu", "--nav",
                   "--native-base-rinex", "--native-base-rinex-sha256"):
        if option not in command:
            raise fail(f"command/{route}: missing input option {option}")
    # The native PDC-looking token is an existing algorithm selector required
    # by the frozen recipe.  No PDC path/value is admitted.
    for index, token in enumerate(command):
        if token.startswith("--") and "pdc" in token.lower() and token != LEGITIMATE_PDC_ALGORITHM_OPTION:
            raise fail(f"command/{route}: forbidden PDC option {token!r}")
        if not token.startswith("--") and any(term in token.lower()
                                               for term in FORBIDDEN_INPUT_TERMS):
            if token not in {str(paths[name]) for name in paths} and \
                    token != str(output_dir / OPAQUE_OUTPUT_NAME) and \
                    token != str(output_dir / "native_summary.json"):
                raise fail(f"command/{route}: forbidden value {token!r}")
        if token == "--native-base-rinex-sha256":
            _equal(command[index + 1], hashes["base.obs"],
                   f"command/{route}/base digest")


def _copy_equation(equation: Mapping[str, Any]) -> dict[str, Any]:
    # The equation AST/token tuple are semantic metadata, not solution data.
    return json.loads(json.dumps(equation, sort_keys=True))


def compact_authority_metrics(native: Mapping[str, Any],
                              opaque: Mapping[str, Any] | None,
                              return_code: int) -> dict[str, Any]:
    """Return scalar structural evidence without copying native summary."""
    telemetry = native["phase141_telemetry"]
    p135 = telemetry["phase135"]
    p138 = telemetry["phase138"]
    solver = telemetry["solver"]
    clock = telemetry["clock"]
    factor_counts = telemetry["factor_counts"]
    bridge = telemetry["bridge"]
    output = telemetry["output"]
    stage_metrics = {
        stage: {
            key: solver[stage][key] for key in (
                "attempted", "accepted_iterations", "initial_cost",
                "final_cost", "costs_finite", "strict_cost_decrease",
                "terminal_branch", "no_fallback",
            )
        }
        for stage in ("gnss_first", "main")
    }
    return {
        "equation": _copy_equation(telemetry["equation"]),
        "phase135": {
            "geometry_rows": p135["geometry_rows"],
            "sagnac_evaluations": p135["sagnac_evaluations"],
            "los_convention": p135["los_convention"],
            "pseudorange": {
                "admitted_rows": p135["pseudorange"]["admitted_rows"],
                "affine_factors_inserted": p135["pseudorange"]["affine_factors_inserted"],
            },
            "doppler": {
                "admitted_rows": p135["doppler"]["admitted_rows"],
                "affine_factors_inserted": p135["doppler"]["affine_factors_inserted"],
            },
            "ordinary_tdcp": {
                "admitted_rows": p135["ordinary_tdcp"]["admitted_rows"],
                "affine_factors_inserted": p135["ordinary_tdcp"]["affine_factors_inserted"],
            },
            "legacy_factor_counts": dict(p135["legacy_factor_counts"]),
            "pose3_x_bridge": dict(p135["pose3_x_bridge"]),
        },
        "phase138": {
            "range_constants_validated": p138["range_constants_validated"],
            "tdcp_measurements_adjusted": p138["tdcp_measurements_adjusted"],
            "affine_tdcp_factor_count": p138["affine_tdcp_factor_count"],
            "adjustment_application_passes": p138["adjustment_application_passes"],
            "adjusted_exactly_once": p138["adjusted_exactly_once"],
            "factor_count_unchanged": p138["factor_count_unchanged"],
            "same_endpoint_epoch_and_satellite_state": p138["same_endpoint_epoch_and_satellite_state"],
            "single_sagnac_representation": p138["single_sagnac_representation"],
        },
        "raw_base": dict(telemetry["raw_base"]),
        "clock": {
            key: clock[key] for key in (
                "c_units", "d_units", "c7_dimension", "c_epoch_count",
                "c_finite_count", "d_epoch_count", "d_finite_count",
                "c7_state_count", "c7_handoff_count",
            )
        },
        "solver": {
            "linear_solver": solver["linear_solver"],
            "elimination": solver["elimination"],
            **stage_metrics,
        },
        "factor_counts": dict(factor_counts),
        "bridge": dict(bridge),
        "offset": dict(telemetry["offset"]),
        "output": dict(output),
        "return_code": return_code,
        "opaque_solution_seal": opaque,
    }


def _read_native_summary(path: Path) -> Mapping[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"native summary: {exc}") from exc
    if not isinstance(value, Mapping):
        raise fail("native summary: expected object")
    return value


def run_one_route(record: Mapping[str, Any]) -> dict[str, Any]:
    route = str(record["dataset_id"])
    paths = actual_payload_paths(record)
    raw = _mapping(record, "raw_inputs", "route")
    base = _mapping(record, "base_input", "route")
    expected: dict[str, tuple[str, int]] = {
        name: (str(raw[name]["sha256"]), int(raw[name]["bytes"]))
        for name in RAW_INPUT_KEYS
    }
    expected["base.obs"] = (str(base["sha256"]), int(base["bytes"]))
    reads = {name: 0 for name in paths}
    hashes: dict[str, Any] = {}
    try:
        for name in ("device_gnss.csv", "device_imu.csv", "brdc.nav", "base.obs"):
            actual_sha, actual_bytes = digest_file(paths[name], name)
            reads[name] = 1
            declared_sha, declared_bytes = expected[name]
            hashes[name] = {
                "sha256": actual_sha,
                "bytes": actual_bytes,
                "expected_sha256": declared_sha,
                "expected_bytes": declared_bytes,
                "match": actual_sha == declared_sha and actual_bytes == declared_bytes,
            }
            if not hashes[name]["match"]:
                return {
                    "route": route,
                    "target": ROUTE_TARGETS[route],
                    "status": "fail-closed-input-hash",
                    "solver_invocations": 0,
                    "input_read_counts": reads,
                    "input_hashes": hashes,
                    "failure": f"{name}: sealed hash/size mismatch",
                }
    except AuthorizationError as exc:
        return {
            "route": route,
            "target": ROUTE_TARGETS[route],
            "status": "fail-closed-input-preflight",
            "solver_invocations": 0,
            "input_read_counts": reads,
            "input_hashes": hashes,
            "failure": str(exc),
        }

    output_dir = ROOT / str(record["output_directory"])
    output_path = output_dir / OPAQUE_OUTPUT_NAME
    summary_path = output_dir / "native_summary.json"
    stdout_path = output_dir / "native.stdout.log"
    stderr_path = output_dir / "native.stderr.log"
    if any(path.exists() for path in (output_dir, output_path, summary_path,
                                      stdout_path, stderr_path)):
        return {
            "route": route,
            "target": ROUTE_TARGETS[route],
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
        validate_command(route, command, paths,
                         {name: item["sha256"] for name, item in hashes.items()},
                         output_dir)
    except (OSError, AuthorizationError) as exc:
        return {
            "route": route,
            "target": ROUTE_TARGETS[route],
            "status": "fail-closed-command-preflight",
            "solver_invocations": 0,
            "input_read_counts": reads,
            "input_hashes": hashes,
            "failure": str(exc),
        }

    environment = os.environ.copy()
    trusted_library = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = trusted_library + (
        (":" + environment["LD_LIBRARY_PATH"])
        if environment.get("LD_LIBRARY_PATH") else ""
    )
    completed = subprocess.run(command, cwd=ROOT, env=environment,
                               capture_output=True, check=False)
    stdout_path.write_bytes(completed.stdout)
    stderr_path.write_bytes(completed.stderr)
    route_result: dict[str, Any] = {
        "route": route,
        "target": ROUTE_TARGETS[route],
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
        native = _read_native_summary(summary_path)
        route_result["native_summary"] = {
            "path": str(summary_path.relative_to(ROOT)),
            "sha256": digest_bytes(summary_bytes),
            "bytes": len(summary_bytes),
            "coordinate_fields_interpreted": False,
        }
        if completed.returncode != 0:
            raise fail(f"native return code {completed.returncode}")
        CONTRACT.validate_native_summary(route, native)
        route_result["structural_gate"] = "pass"
        route_result["authority_metrics"] = compact_authority_metrics(
            native, opaque, completed.returncode
        )
    except (OSError, UnicodeDecodeError, json.JSONDecodeError,
            AuthorizationError, KeyError, TypeError, ValueError) as exc:
        route_result["status"] = "fail-closed-structural-gate"
        route_result["structural_gate"] = "fail"
        route_result["failure"] = str(exc)
    return route_result


def _result_read_accounting(results: list[Mapping[str, Any]]) -> dict[str, int]:
    return {
        "raw_phone_gnss_reads": sum(
            int(item.get("input_read_counts", {}).get("device_gnss.csv", 0))
            for item in results
        ),
        "raw_phone_imu_reads": sum(
            int(item.get("input_read_counts", {}).get("device_imu.csv", 0))
            for item in results
        ),
        "broadcast_navigation_reads": sum(
            int(item.get("input_read_counts", {}).get("brdc.nav", 0))
            for item in results
        ),
        "raw_base_rinex_reads": sum(
            int(item.get("input_read_counts", {}).get("base.obs", 0))
            for item in results
        ),
        "native_solver_invocations": sum(
            int(item.get("solver_invocations", 0)) for item in results
        ),
        "truth_reads": 0,
        "solution_coordinate_reads": 0,
        "accuracy_calculations": 0,
        "mat_pdc_precomputed_coordinate_reads": 0,
        "kaggle_or_token_access": 0,
        "reruns_fallbacks_repairs_sweeps": 0,
    }


def run(auth_path: Path = AUTH) -> int:
    auth = read_object(auth_path, "Phase141 authorization")
    verify_authorization(auth)
    routes = _require(auth, "routes", "authorization")
    results: list[dict[str, Any]] = []
    for index, target in enumerate(TARGETS):
        record = routes[index]
        if not isinstance(record, Mapping) or record.get("target") != target:
            raise fail(f"route order mismatch at index {index}")
        # Each route gets exactly one hash preflight and at most one native
        # launch.  A failure is sealed; no route is retried or repaired.
        results.append(run_one_route(record))
    all_pass = all(item.get("structural_gate") == "pass" for item in results)
    result = {
        "schema_version": "smartphone-r5-phase141-telemetry-schema-raw-result.v1",
        "phase": 141,
        "execution_label": "Luna Max",
        "status": "sealed-structural-raw-result-no-accuracy",
        "authorization_path": str(auth_path.relative_to(ROOT)),
        "authorization_sha256": digest_bytes(auth_path.read_bytes()),
        "candidate_id": "phase141-native-authoritative-telemetry-schema-v1",
        "route_order": list(TARGETS),
        "runs_per_route": 1,
        "structural_gate": "GO" if all_pass else "NO-GO",
        "routes": results,
        "policy": {
            "truth_used": False,
            "mat_used": False,
            "pdc_input_used": False,
            "precomputed_coordinates_used": False,
            "accuracy_evaluation": False,
            "kaggle_access": False,
            "solution_publication": False,
            "rerun": False,
            "fallback": False,
            "repair": False,
            "solution_coordinate_interpretation": False,
        },
        "read_accounting": _result_read_accounting(results),
        "solution_policy": "opaque SHA-256/bytes/newline seal only; no coordinate fields read or published",
        "authorization_commit_is_separate": True,
        "no_rerun_or_fallback": True,
    }
    RESULT.parent.mkdir(parents=True, exist_ok=True)
    RESULT.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                      encoding="utf-8")
    return 0 if all_pass else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--authorization", type=Path, default=AUTH)
    parser.add_argument("--verify-authorization", action="store_true",
                        help="verify all pins without materializing raw input")
    args = parser.parse_args()
    try:
        auth = read_object(args.authorization, "Phase141 authorization")
        verify_authorization(auth)
        if args.verify_authorization:
            print("PHASE141_AUTHORIZATION_VERIFIED: raw reads=0 solver=0")
            return 0
        return run(args.authorization)
    except AuthorizationError as exc:
        print(f"PHASE141_RAW_FAIL_CLOSED: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
