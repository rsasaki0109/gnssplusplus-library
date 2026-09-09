#!/usr/bin/env python3
"""Launch-free Phase141 native telemetry schema contract.

This module validates only tracked source, the Phase141 audit/freeze, the
static manifest, and synthetic/in-memory native-summary-shaped objects.  It
never opens raw payloads, solution rows, truth/MAT/PDC artifacts, and never
starts the native binary.  A future raw runner must obtain a new independent
authorization after this launch-free qualification.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[3]
AUDIT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase141_telemetry_schema_audit_v1.md"
)
FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase141_telemetry_schema_freeze_v1.json"
)
MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase141_telemetry_schema_manifest_v1.json"
)
RUNNER = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase141_telemetry_schema_structural.py"
)
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase141_telemetry_schema.py"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"


AUDIT_COMMIT = "de71c39"
AUDIT_SHA256 = "a040f2076da254f2f11e8041edbc3c23112f65bdb12b8f0665e8de927212f092"
FREEZE_COMMIT = "d105e0a"
FREEZE_SHA256 = "c6a8b18f52e1469ae746e1b0b3295fea067d58fa41e1b43e1cfd2797d6823793"
IMPLEMENTATION_COMMIT = "6c752c2264c7444da6eb4833ddbbb28ee43ce62a"
TARGET_BINARY_SHA256 = "aa2180cfe4a36018b98307c3bdf87bfabd229d32a1dacaaada817c257a9280a6"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
SCHEMA = "smartphone-r5-phase141-telemetry-schema-manifest.v1"
NATIVE_SCHEMA = "smartphone-r5-native-fgo-phase141-telemetry.v1"
EQUATION_ID = "phase138-affine-tdcp-anchor-range-constant-v1"
EQUATION_DISPLAY = "tdcp_native-(rho_current_initial-rho_previous_initial)"
EQUATION_FULL_TOKENS = [
    "ASSIGN", "tdcp_phase138", "SUB", "GROUP_OPEN", "tdcp_native", "SUB",
    "GROUP_OPEN", "rho_current_initial", "SUB", "rho_previous_initial",
    "GROUP_CLOSE", "GROUP_CLOSE",
]
EQUATION_RHS_TOKENS = [
    "tdcp_native", "SUB", "GROUP_OPEN", "rho_current_initial", "SUB",
    "rho_previous_initial", "GROUP_CLOSE",
]


class Phase141ContractError(ValueError):
    """A Phase141 schema violation which must fail closed."""


def fail(message: str) -> Phase141ContractError:
    return Phase141ContractError(message)


def _reject_duplicate_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise fail(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"), object_pairs_hook=_reject_duplicate_pairs
        )
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, Phase141ContractError) as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} must be an object")
    return value


def relative(path: Path) -> str:
    return str(path.relative_to(ROOT))


def static_sha256(path: Path, label: str) -> str:
    lowered = path.name.lower()
    if lowered.endswith((".csv", ".nav", ".obs", ".mat")):
        raise fail(f"payload hash forbidden for {label}")
    if any(term in lowered for term in ("truth", "ground_truth", "precomputed")):
        raise fail(f"forbidden artifact hash for {label}")
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _required(mapping: Mapping[str, Any], key: str, label: str) -> Any:
    if key not in mapping:
        raise fail(f"{label}/{key}: missing")
    return mapping[key]


def _bool(mapping: Mapping[str, Any], key: str, label: str) -> bool:
    value = _required(mapping, key, label)
    if not isinstance(value, bool):
        raise fail(f"{label}/{key}: expected boolean")
    return value


def _true(mapping: Mapping[str, Any], key: str, label: str) -> None:
    if not _bool(mapping, key, label):
        raise fail(f"{label}/{key}: expected true")


def _count(mapping: Mapping[str, Any], key: str, label: str) -> int:
    value = _required(mapping, key, label)
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise fail(f"{label}/{key}: expected nonnegative integer")
    return value


def _finite(mapping: Mapping[str, Any], key: str, label: str) -> float:
    value = _required(mapping, key, label)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise fail(f"{label}/{key}: expected finite number")
    value = float(value)
    if not math.isfinite(value):
        raise fail(f"{label}/{key}: expected finite number")
    return value


def _equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def validate_equation(equation: Mapping[str, Any], label: str) -> None:
    _equal(_required(equation, "semantic_id", label), EQUATION_ID, f"{label}/semantic_id")
    representation = _required(equation, "representation", label)
    if representation not in {"full-assignment", "rhs-only-native-diagnostic"}:
        raise fail(f"{label}/representation: unsupported {representation!r}")
    _equal(_required(equation, "display_expression", label),
           EQUATION_DISPLAY, f"{label}/display_expression")
    ast = _required(equation, "ast", label)
    if not isinstance(ast, Mapping):
        raise fail(f"{label}/ast: expected object")
    _equal(ast, {
        "kind": "assign",
        "lhs": "tdcp_phase138",
        "rhs": {
            "kind": "sub",
            "left": "tdcp_native",
            "right": {
                "kind": "sub",
                "left": "rho_current_initial",
                "right": "rho_previous_initial",
            },
        },
    }, f"{label}/ast")
    _equal(_required(equation, "token_tuple", label), EQUATION_FULL_TOKENS,
           f"{label}/token_tuple")
    _equal(_required(equation, "rhs_only_token_tuple", label), EQUATION_RHS_TOKENS,
           f"{label}/rhs_only_token_tuple")
    _equal(_required(equation, "whitespace_normalization_only", label), True,
           f"{label}/whitespace_normalization_only")


def _validate_family(family: Mapping[str, Any], label: str) -> int:
    admitted = _count(family, "admitted_rows", label)
    inserted = _count(family, "affine_factors_inserted", label)
    if admitted <= 0:
        raise fail(f"{label}/admitted_rows must be positive")
    _equal(inserted, admitted, f"{label}/admission conservation")
    _true(family, "key_order_exact", label)
    _true(family, "finite_values", label)
    _true(family, "source_geometry_same_path", label)
    _required(family, "source_report", label)
    return admitted


def validate_native_summary(route: str, summary: Mapping[str, Any]) -> None:
    """Validate the native Phase141 object without normalizing or inferring."""
    if route not in ROUTES:
        raise fail(f"unknown route {route!r}")
    _equal(summary.get("dataset_id"), route, f"summary/{route}/dataset_id")
    _equal(summary.get("truth_used"), False, f"summary/{route}/truth_used")
    _equal(summary.get("no_base_contract"), True, f"summary/{route}/no_base_contract")
    _equal(summary.get("native_phase141_telemetry_schema"), True,
           f"summary/{route}/native_phase141_telemetry_schema")
    telemetry = _required(summary, "phase141_telemetry", f"summary/{route}")
    if not isinstance(telemetry, Mapping):
        raise fail(f"summary/{route}/phase141_telemetry: expected object")
    _equal(telemetry.get("schema_version"), NATIVE_SCHEMA,
           f"summary/{route}/phase141_telemetry/schema_version")
    _equal(_count(telemetry, "sync_count", f"summary/{route}/phase141_telemetry"),
           1, f"summary/{route}/phase141_telemetry/sync_count")

    authority = _required(telemetry, "authority", f"summary/{route}/phase141_telemetry")
    if not isinstance(authority, Mapping):
        raise fail(f"summary/{route}/phase141_telemetry/authority: expected object")
    for key in ("gnss_first", "main", "raw_base", "offset", "output",
                "wrapper_read_accounting"):
        value = _required(authority, key, f"summary/{route}/authority")
        if not isinstance(value, str) or not value:
            raise fail(f"summary/{route}/authority/{key}: expected source label")

    selectors = _required(telemetry, "selectors", f"summary/{route}/phase141_telemetry")
    if not isinstance(selectors, Mapping):
        raise fail(f"summary/{route}/phase141_telemetry/selectors: expected object")
    expected_selectors = {
        "phase135_official_affine_measurement_family": True,
        "phase138_affine_tdcp_anchor_range_constant": True,
        "phase118_official_tdcp_huber_k": True,
        "phase117_dynamic_tdcp_sigma": False,
        "phase120_official_tdcp_resl_atmosphere_cancellation": False,
        "phase126_raw_base_source_complete": False,
        "phase127_glonass_channel_provenance": False,
        "phase128_glonass_provenance_parser_admission": False,
        "phase129_glonass_local_miss_mask": False,
        "phase130_shared_ledger_key_local_support": False,
        "phase131_canonical_correction_band_key": False,
        "phase107_raw_base_compensation": True,
        "phase107_raw_base_source_miss_mask": True,
        "phase107_preserve_additional_frequency_bands": False,
        "phase141_telemetry_schema": True,
    }
    for key, expected in expected_selectors.items():
        _equal(selectors.get(key), expected,
               f"summary/{route}/selectors/{key}")

    equation = _required(telemetry, "equation", f"summary/{route}/phase141_telemetry")
    if not isinstance(equation, Mapping):
        raise fail(f"summary/{route}/equation: expected object")
    validate_equation(equation, f"summary/{route}/equation")

    phase135 = _required(telemetry, "phase135", f"summary/{route}/phase141_telemetry")
    if not isinstance(phase135, Mapping):
        raise fail(f"summary/{route}/phase135: expected object")
    for key in ("enabled", "configuration_valid", "transactional",
                "fixed_initial_geometry", "finite_jacobians",
                "single_sagnac_representation"):
        _true(phase135, key, f"summary/{route}/phase135")
    _equal(_required(phase135, "los_convention", f"summary/{route}/phase135"),
           "-e=(receiver-satellite)/range", f"summary/{route}/phase135/los_convention")
    geometry_rows = _count(phase135, "geometry_rows", f"summary/{route}/phase135")
    _equal(_count(phase135, "sagnac_evaluations", f"summary/{route}/phase135"),
           geometry_rows, f"summary/{route}/phase135/sagnac_evaluations")
    families = {
        key: _validate_family(
            _required(phase135, key, f"summary/{route}/phase135"),
            f"summary/{route}/phase135/{key}",
        )
        for key in ("pseudorange", "doppler", "ordinary_tdcp")
    }
    legacy = _required(phase135, "legacy_factor_counts", f"summary/{route}/phase135")
    if not isinstance(legacy, Mapping):
        raise fail(f"summary/{route}/phase135/legacy_factor_counts: expected object")
    for key in ("pseudorange", "doppler", "ordinary_tdcp"):
        _equal(_count(legacy, key, f"summary/{route}/phase135/legacy_factor_counts"),
               0, f"summary/{route}/phase135/legacy/{key}")
    bridge = _required(phase135, "pose3_x_bridge", f"summary/{route}/phase135")
    if not isinstance(bridge, Mapping):
        raise fail(f"summary/{route}/phase135/pose3_x_bridge: expected object")
    if _count(bridge, "count", f"summary/{route}/phase135/pose3_x_bridge") <= 0:
        raise fail(f"summary/{route}/phase135/pose3_x_bridge/count: empty")
    _true(bridge, "keys_exact", f"summary/{route}/phase135/pose3_x_bridge")

    phase138 = _required(telemetry, "phase138", f"summary/{route}/phase141_telemetry")
    if not isinstance(phase138, Mapping):
        raise fail(f"summary/{route}/phase138: expected object")
    for key in ("enabled", "phase135_dependency_satisfied", "configuration_valid",
                "transactional", "adjusted_exactly_once", "factor_count_unchanged",
                "same_endpoint_epoch_and_satellite_state", "same_satellite_state",
                "finite_adjusted_measurements", "no_raw_or_zero_fallback",
                "phase118_atmosphere_sigma_huber_unchanged",
                "single_sagnac_representation"):
        _true(phase138, key, f"summary/{route}/phase138")
    _equal(_required(phase138, "measurement_equation", f"summary/{route}/phase138"),
           EQUATION_DISPLAY, f"summary/{route}/phase138/measurement_equation")
    _equal(_required(phase138, "geometry_representation", f"summary/{route}/phase138"),
           "RTKLIB-geodist-single-Sagnac-fixed-initial-endpoints",
           f"summary/{route}/phase138/geometry_representation")
    for key in ("range_constants_validated", "tdcp_measurements_adjusted",
                "affine_tdcp_factor_count"):
        _equal(_count(phase138, key, f"summary/{route}/phase138"), families["ordinary_tdcp"],
               f"summary/{route}/phase138/{key}")
    _equal(_count(phase138, "adjustment_application_passes", f"summary/{route}/phase138"),
           1, f"summary/{route}/phase138/adjustment_application_passes")
    _equal(_count(phase138, "legacy_tdcp_factor_count", f"summary/{route}/phase138"),
           0, f"summary/{route}/phase138/legacy_tdcp_factor_count")
    nested_equation = _required(phase138, "equation", f"summary/{route}/phase138")
    if not isinstance(nested_equation, Mapping):
        raise fail(f"summary/{route}/phase138/equation: expected object")
    validate_equation(nested_equation, f"summary/{route}/phase138/equation")

    base = _required(telemetry, "raw_base", f"summary/{route}/phase141_telemetry")
    if not isinstance(base, Mapping):
        raise fail(f"summary/{route}/raw_base: expected object")
    for key in ("phase107_recipe", "applied_exactly_once",
                "source_miss_conservation", "no_raw_or_zero_fallback"):
        _true(base, key, f"summary/{route}/raw_base")

    clock = _required(telemetry, "clock", f"summary/{route}/phase141_telemetry")
    if not isinstance(clock, Mapping):
        raise fail(f"summary/{route}/clock: expected object")
    _equal(_required(clock, "c_units", f"summary/{route}/clock"), "metres",
           f"summary/{route}/clock/c_units")
    _equal(_required(clock, "d_units", f"summary/{route}/clock"), "metres/second",
           f"summary/{route}/clock/d_units")
    for key in ("c7_mapping_exact", "d_full_finite_exact_alignment"):
        _true(clock, key, f"summary/{route}/clock")
    _equal(_count(clock, "c7_dimension", f"summary/{route}/clock"), 7,
           f"summary/{route}/clock/c7_dimension")
    for key in ("c_epoch_count", "c_finite_count", "d_epoch_count", "d_finite_count",
                "c7_state_count", "c7_handoff_count"):
        _count(clock, key, f"summary/{route}/clock")

    solver = _required(telemetry, "solver", f"summary/{route}/phase141_telemetry")
    if not isinstance(solver, Mapping):
        raise fail(f"summary/{route}/solver: expected object")
    _equal(_required(solver, "linear_solver", f"summary/{route}/solver"),
           "MULTIFRONTAL_QR", f"summary/{route}/solver/linear_solver")
    _equal(_required(solver, "elimination", f"summary/{route}/solver"),
           "EliminateQR", f"summary/{route}/solver/elimination")
    for stage in ("gnss_first", "main"):
        data = _required(solver, stage, f"summary/{route}/solver")
        if not isinstance(data, Mapping):
            raise fail(f"summary/{route}/solver/{stage}: expected object")
        _true(data, "attempted", f"summary/{route}/solver/{stage}")
        if _count(data, "accepted_iterations", f"summary/{route}/solver/{stage}") <= 0:
            raise fail(f"summary/{route}/solver/{stage}/accepted_iterations: zero")
        initial = _finite(data, "initial_cost", f"summary/{route}/solver/{stage}")
        final = _finite(data, "final_cost", f"summary/{route}/solver/{stage}")
        _true(data, "costs_finite", f"summary/{route}/solver/{stage}")
        _true(data, "strict_cost_decrease", f"summary/{route}/solver/{stage}")
        if not final < initial:
            raise fail(f"summary/{route}/solver/{stage}: costs are not decreasing")
        _true(data, "no_fallback", f"summary/{route}/solver/{stage}")
        _required(data, "terminal_branch", f"summary/{route}/solver/{stage}")

    factor_counts = _required(telemetry, "factor_counts", f"summary/{route}/phase141_telemetry")
    if not isinstance(factor_counts, Mapping):
        raise fail(f"summary/{route}/factor_counts: expected object")
    for key, family in (("pseudorange", "pseudorange"), ("doppler", "doppler"),
                        ("ordinary_tdcp", "ordinary_tdcp")):
        _equal(_count(factor_counts, key, f"summary/{route}/factor_counts"), families[family],
               f"summary/{route}/factor_counts/{key}")
    for key in ("legacy_pseudorange", "legacy_doppler", "legacy_tdcp"):
        _equal(_count(factor_counts, key, f"summary/{route}/factor_counts"), 0,
               f"summary/{route}/factor_counts/{key}")

    bridge_report = _required(telemetry, "bridge", f"summary/{route}/phase141_telemetry")
    if not isinstance(bridge_report, Mapping):
        raise fail(f"summary/{route}/bridge: expected object")
    phase131_sync_count = _count(
        bridge_report, "phase131_sync_count", f"summary/{route}/bridge"
    )
    expected_phase131_sync = 1 if selectors["phase131_canonical_correction_band_key"] else 0
    _equal(phase131_sync_count, expected_phase131_sync,
           f"summary/{route}/bridge/phase131_sync_count")
    _true(bridge_report, "pose3_x_keys_exact", f"summary/{route}/bridge")

    offset = _required(telemetry, "offset", f"summary/{route}/phase141_telemetry")
    if not isinstance(offset, Mapping):
        raise fail(f"summary/{route}/offset: expected object")
    _true(offset, "enabled", f"summary/{route}/offset")
    _true(offset, "applied", f"summary/{route}/offset")
    _equal(_count(offset, "application_passes", f"summary/{route}/offset"), 1,
           f"summary/{route}/offset/application_passes")

    output = _required(telemetry, "output", f"summary/{route}/phase141_telemetry")
    if not isinstance(output, Mapping):
        raise fail(f"summary/{route}/output: expected object")
    for key in ("finite", "earth_valid", "expected_epoch_coverage",
                "opaque_solution_seal"):
        _true(output, key, f"summary/{route}/output")
    _equal(_required(output, "coordinate_rows_interpreted", f"summary/{route}/output"),
           False, f"summary/{route}/output/coordinate_rows_interpreted")
    _equal(_count(output, "pixel5_offset_applications", f"summary/{route}/output"), 1,
           f"summary/{route}/output/pixel5_offset_applications")
    if any(key in output for key in ("coordinate_rows", "solution_rows", "solution_values")):
        raise fail(f"summary/{route}/output contains solution content")

    policy = _required(telemetry, "policy", f"summary/{route}/phase141_telemetry")
    if not isinstance(policy, Mapping):
        raise fail(f"summary/{route}/policy: expected object")
    for key in ("truth_used", "coordinate_rows_interpreted",
                "solution_publication_authorized", "fallback", "rerun"):
        _equal(_required(policy, key, f"summary/{route}/policy"), False,
               f"summary/{route}/policy/{key}")

    if "coordinate_rows" in summary or "solution_rows" in summary:
        raise fail(f"summary/{route} contains top-level solution content")


def validate_freeze(freeze: Mapping[str, Any]) -> None:
    _equal(freeze.get("schema_version"),
           "smartphone-r5-phase141-telemetry-schema-freeze.v1", "freeze/schema_version")
    _equal(freeze.get("phase"), 141, "freeze/phase")
    _equal(freeze.get("execution_label"), "Luna Max", "freeze/execution_label")
    _equal(freeze.get("status"), "frozen-before-implementation-and-raw-execution",
           "freeze/status")
    audit = _required(freeze, "audit", "freeze")
    if not isinstance(audit, Mapping):
        raise fail("freeze/audit: expected object")
    _equal(audit.get("path"), relative(AUDIT), "freeze/audit/path")
    _equal(audit.get("commit"), AUDIT_COMMIT, "freeze/audit/commit")
    _equal(audit.get("sha256"), AUDIT_SHA256, "freeze/audit/sha256")
    candidate = _required(freeze, "candidate", "freeze")
    if not isinstance(candidate, Mapping):
        raise fail("freeze/candidate: expected object")
    for key, expected in {
        "count": 1,
        "id": "phase141-native-authoritative-telemetry-schema-v1",
        "selector": "--native-phase141-telemetry-schema",
        "default_off": True,
        "diagnostic_only": True,
        "native_summary_immutable": True,
        "wrapper_may_validate_only": True,
        "wrapper_may_infer_or_overwrite": False,
        "partial_schema_allowed": False,
        "raw_execution_authorized": False,
        "solver_execution_authorized": False,
        "truth_evaluation_authorized": False,
        "solution_publication_authorized": False,
    }.items():
        _equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    native = _required(freeze, "native_schema", "freeze")
    if not isinstance(native, Mapping):
        raise fail("freeze/native_schema: expected object")
    _equal(native.get("schema_version"), NATIVE_SCHEMA, "freeze/native_schema/schema_version")
    _equal(native.get("object_path"), "phase141_telemetry", "freeze/native_schema/object_path")
    _equal(native.get("sync_count"), 1, "freeze/native_schema/sync_count")
    qualification = _required(freeze, "qualification", "freeze")
    if not isinstance(qualification, Mapping):
        raise fail("freeze/qualification: expected object")
    if qualification.get("full_cpp_suite_required") is not True:
        raise fail("freeze/qualification/full_cpp_suite_required: expected true")
    boundary = _required(freeze, "authorization_boundary", "freeze")
    if not isinstance(boundary, Mapping):
        raise fail("freeze/authorization_boundary: expected object")
    for key in ("implementation_commit_before_runner_pin",
                "launch_free_runner_manifest_pre_raw_seal_separate",
                "new_independent_authorization_required_before_any_raw_or_solver",
                "historical_phase139_artifacts_immutable"):
        _equal(boundary.get(key), True, f"freeze/authorization_boundary/{key}")


def validate_manifest(manifest: Mapping[str, Any]) -> None:
    for key, expected in {
        "schema_version": SCHEMA,
        "phase": 141,
        "status": "launch-free",
        "audit_commit": AUDIT_COMMIT,
        "freeze_commit": FREEZE_COMMIT,
        "freeze_sha256": FREEZE_SHA256,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "target_binary_sha256": TARGET_BINARY_SHA256,
        "native_schema_version": NATIVE_SCHEMA,
        "routes": list(ROUTES),
    }.items():
        _equal(manifest.get(key), expected, f"manifest/{key}")
    _equal(manifest.get("audit_path"), relative(AUDIT), "manifest/audit_path")
    _equal(manifest.get("freeze_path"), relative(FREEZE), "manifest/freeze_path")
    _equal(manifest.get("pre_raw_path"),
           "docs/use_cases/records/smartphone_r5_phase141_telemetry_schema_pre_raw_v1.json",
           "manifest/pre_raw_path")
    recipe = _required(manifest, "recipe", "manifest")
    if not isinstance(recipe, Mapping):
        raise fail("manifest/recipe: expected object")
    for key, expected in {
        "native_phase141_selector": "--native-phase141-telemetry-schema",
        "native_phase135_selector": "--native-phase135-official-affine-measurement-family",
        "native_phase138_selector": "--native-phase138-affine-tdcp-anchor-range-constant",
        "native_phase118_selector": "--native-phase118-official-tdcp-huber-k",
        "native_phase99_solver": "MULTIFRONTAL_QR",
        "native_phase99_elimination": "EliminateQR",
        "phase141_default_off": True,
        "wrapper_inference": False,
        "native_summary_overwrite": False,
    }.items():
        _equal(recipe.get(key), expected, f"manifest/recipe/{key}")
    policy = _required(manifest, "policy", "manifest")
    if not isinstance(policy, Mapping):
        raise fail("manifest/policy: expected object")
    for key in ("raw_execution_authorized", "solver_execution_authorized",
                "truth_used", "solution_coordinate_reads", "mat_pdc_precomputed_reads",
                "accuracy_evaluation", "kaggle_access", "rerun", "fallback", "repair"):
        _equal(policy.get(key), False, f"manifest/policy/{key}")
    artifacts = _required(manifest, "artifacts", "manifest")
    if not isinstance(artifacts, Mapping):
        raise fail("manifest/artifacts: expected object")
    for key, path in (("audit", AUDIT), ("freeze", FREEZE),
                      ("runner", RUNNER), ("focused_tests", FOCUSED_TESTS)):
        value = artifacts.get(key)
        if not isinstance(value, Mapping):
            raise fail(f"manifest/artifacts/{key}: expected object")
        _equal(value.get("path"), relative(path), f"manifest/artifacts/{key}/path")
        _equal(value.get("sha256"), static_sha256(path, f"manifest artifact {key}"),
               f"manifest/artifacts/{key}/sha256")
    read = _required(manifest, "pre_raw_read_accounting", "manifest")
    if not isinstance(read, Mapping):
        raise fail("manifest/pre_raw_read_accounting: expected object")
    for key, value in read.items():
        if isinstance(value, bool):
            _equal(value, False, f"manifest/pre_raw_read_accounting/{key}")
        elif isinstance(value, int):
            _equal(value, 0, f"manifest/pre_raw_read_accounting/{key}")
        else:
            raise fail(f"manifest/pre_raw_read_accounting/{key}: expected zero")


def validate_static_sources() -> dict[str, str]:
    source_paths = (
        "apps/native/gnss_fgo_imu_no_base.cpp",
        "include/libgnss++/algorithms/fgo.hpp",
        "src/algorithms/fgo_gtsam_backend.cpp",
    )
    result = {path: static_sha256(ROOT / path, path) for path in source_paths}
    source = (ROOT / "apps/native/gnss_fgo_imu_no_base.cpp").read_text(encoding="utf-8")
    for needle in (
        "--native-phase141-telemetry-schema",
        "phase141_telemetry",
        "smartphone-r5-native-fgo-phase141-telemetry.v1",
        "phase135_transactional",
        "phase138_same_endpoint_epoch_and_satellite_state",
    ):
        if needle not in source:
            raise fail(f"native Phase141 witness missing: {needle}")
    result[relative(BINARY)] = static_sha256(BINARY, "Phase141 target binary")
    _equal(result[relative(BINARY)], TARGET_BINARY_SHA256, "target binary sha256")
    return result


def launch_free_validation() -> dict[str, Any]:
    freeze = read_json(FREEZE, "Phase141 freeze")
    manifest = read_json(MANIFEST, "Phase141 manifest")
    _equal(static_sha256(AUDIT, "Phase141 audit"), AUDIT_SHA256, "audit sha256")
    _equal(static_sha256(FREEZE, "Phase141 freeze"), FREEZE_SHA256, "freeze sha256")
    validate_freeze(freeze)
    validate_manifest(manifest)
    source_hashes = validate_static_sources()
    _equal(manifest.get("source_pins"), source_hashes, "manifest/source_pins")
    return {
        "status": "launch-free-qualified",
        "phase": 141,
        "routes": list(ROUTES),
        "raw_execution_authorized": False,
        "solver_invocations": 0,
        "raw_phone_gnss_reads": 0,
        "raw_phone_imu_reads": 0,
        "broadcast_navigation_reads": 0,
        "raw_base_rinex_reads": 0,
        "truth_reads": 0,
        "solution_coordinate_reads": 0,
        "mat_pdc_precomputed_coordinate_reads": 0,
        "accuracy_calculations": 0,
        "kaggle_access": 0,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--print-pins", action="store_true",
                        help="print static pins; never materialize or launch inputs")
    args = parser.parse_args(argv)
    try:
        result = launch_free_validation()
        if args.print_pins:
            result["source_pins"] = validate_static_sources()
        print(json.dumps(result, sort_keys=True))
    except Phase141ContractError as exc:
        print(f"PHASE141_FAIL_CLOSED: {exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
