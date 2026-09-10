#!/usr/bin/env python3
"""Launch-free Phase144 telemetry serializer qualification.

Only tracked source, audit/freeze/manifest metadata, and in-memory synthetic
summary objects are inspected here.  The validator never opens a raw payload,
solution, truth, MAT/PDC artifact and never starts the native solver.  A new
independent authorization is required before any later structural run.
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
    "smartphone_r5_phase144_telemetry_serializer_audit_v1.md"
)
FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase144_telemetry_serializer_freeze_v1.json"
)
MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase144_telemetry_serializer_manifest_v1.json"
)
PRE_RAW = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase144_telemetry_serializer_pre_raw_v1.json"
)
RUNNER = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase144_telemetry_serializer_structural.py"
)
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase144_telemetry_serializer.py"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
NATIVE_SOURCE = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"


AUDIT_COMMIT = "0f49224d2ca79f2f24b8b28b20abb0937879753f"
AUDIT_SHA256 = "135f57eff3afaa27eb55e7f26cdf42f5983b279236bfad0f259a5b5baf616a87"
FREEZE_COMMIT = "84742b216c262f6c22e22a668810712dc7427f8b"
FREEZE_SHA256 = "e9ec77b84e541085dd11eb6b06cf68dda052a199830a8cda8b8f9a91aa28b6dd"
IMPLEMENTATION_COMMIT = "9a8226c7db46baa503c1ba886450fe1a77775d02"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
SCHEMA = "smartphone-r5-phase144-telemetry-serializer-manifest.v1"
NATIVE_SCHEMA = "smartphone-r5-native-fgo-phase144-telemetry.v1"
EQUATION_ID = "phase138-affine-tdcp-anchor-range-constant-v1"
EQUATION_DISPLAY = (
    "tdcp_phase138 = tdcp_native - "
    "(rho_current_initial - rho_previous_initial)"
)
EQUATION_FULL_TOKENS = [
    "ASSIGN", "tdcp_phase138", "SUB", "GROUP_OPEN", "tdcp_native", "SUB",
    "GROUP_OPEN", "rho_current_initial", "SUB", "rho_previous_initial",
    "GROUP_CLOSE", "GROUP_CLOSE",
]
EQUATION_RHS_TOKENS = [
    "tdcp_native", "SUB", "GROUP_OPEN", "rho_current_initial", "SUB",
    "rho_previous_initial", "GROUP_CLOSE",
]


class Phase144ContractError(ValueError):
    """A Phase144 contract violation which must fail closed."""


def fail(message: str) -> Phase144ContractError:
    return Phase144ContractError(message)


def _json_pointer(parts: Sequence[str | int]) -> str:
    if not parts:
        return ""
    escaped = []
    for part in parts:
        text = str(part).replace("~", "~0").replace("/", "~1")
        escaped.append(text)
    return "/" + "/".join(escaped)


class _DuplicateScanner:
    """Small JSON scanner retaining object paths while accepting all values."""

    def __init__(self, text: str) -> None:
        self.text = text
        self.length = len(text)
        self.index = 0
        self.decoder = json.JSONDecoder()
        self.duplicates: list[str] = []

    def skip_space(self) -> None:
        while self.index < self.length and self.text[self.index] in " \t\r\n":
            self.index += 1

    def parse_string(self) -> str:
        self.skip_space()
        if self.index >= self.length or self.text[self.index] != '"':
            raise ValueError(f"expected string at byte {self.index}")
        value, end = self.decoder.raw_decode(self.text, self.index)
        if not isinstance(value, str):
            raise ValueError(f"expected string at byte {self.index}")
        self.index = end
        return value

    def parse_value(self, path: list[str | int]) -> None:
        self.skip_space()
        if self.index >= self.length:
            raise ValueError("unexpected end of JSON")
        token = self.text[self.index]
        if token == "{":
            self.parse_object(path)
        elif token == "[":
            self.parse_array(path)
        else:
            _, end = self.decoder.raw_decode(self.text, self.index)
            self.index = end

    def parse_object(self, path: list[str | int]) -> None:
        self.index += 1
        self.skip_space()
        seen: set[str] = set()
        if self.index < self.length and self.text[self.index] == "}":
            self.index += 1
            return
        while True:
            key = self.parse_string()
            key_path = path + [key]
            if key in seen:
                self.duplicates.append(_json_pointer(key_path))
            seen.add(key)
            self.skip_space()
            if self.index >= self.length or self.text[self.index] != ":":
                raise ValueError(f"expected ':' at byte {self.index}")
            self.index += 1
            self.parse_value(key_path)
            self.skip_space()
            if self.index >= self.length:
                raise ValueError("unterminated object")
            if self.text[self.index] == "}":
                self.index += 1
                return
            if self.text[self.index] != ",":
                raise ValueError(f"expected ',' at byte {self.index}")
            self.index += 1

    def parse_array(self, path: list[str | int]) -> None:
        self.index += 1
        self.skip_space()
        item = 0
        if self.index < self.length and self.text[self.index] == "]":
            self.index += 1
            return
        while True:
            self.parse_value(path + [item])
            item += 1
            self.skip_space()
            if self.index >= self.length:
                raise ValueError("unterminated array")
            if self.text[self.index] == "]":
                self.index += 1
                return
            if self.text[self.index] != ",":
                raise ValueError(f"expected ',' at byte {self.index}")
            self.index += 1

    def scan(self) -> list[str]:
        self.parse_value([])
        self.skip_space()
        if self.index != self.length:
            raise ValueError(f"trailing JSON at byte {self.index}")
        return list(self.duplicates)


def duplicate_json_paths(text: str) -> list[str]:
    """Return every repeated object-key path, including nested array objects."""
    try:
        return _DuplicateScanner(text).scan()
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        raise fail(f"invalid JSON while scanning duplicate paths: {exc}") from exc


def _reject_duplicate_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise fail(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        text = path.read_text(encoding="utf-8")
        duplicates = duplicate_json_paths(text)
        if duplicates:
            raise fail(f"{label}: duplicate paths {duplicates}")
        value = json.loads(text, object_pairs_hook=_reject_duplicate_pairs)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError,
            Phase144ContractError, ValueError) as exc:
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


def equation_object() -> dict[str, Any]:
    return {
        "semantic_id": EQUATION_ID,
        "representation": "full-assignment",
        "display_expression": EQUATION_DISPLAY,
        "ast": {
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
        },
        "token_tuple": list(EQUATION_FULL_TOKENS),
        "rhs_only_token_tuple": list(EQUATION_RHS_TOKENS),
        "whitespace_normalization_only": True,
    }


def validate_equation(equation: Mapping[str, Any], label: str) -> None:
    _equal(dict(equation), equation_object(), label)


def _validate_family(family: Mapping[str, Any], label: str) -> int:
    admitted = _count(family, "admitted_rows", label)
    inserted = _count(family, "affine_factors_inserted", label)
    if admitted <= 0:
        raise fail(f"{label}/admitted_rows must be positive")
    _equal(inserted, admitted, f"{label}/admission conservation")
    for key in ("key_order_exact", "finite_values", "source_geometry_same_path"):
        _true(family, key, label)
    source_report = _required(family, "source_report", label)
    if not isinstance(source_report, str) or not source_report:
        raise fail(f"{label}/source_report: expected nonempty string")
    return admitted


def _validate_stage(stage: Mapping[str, Any], label: str) -> None:
    _true(stage, "attempted", label)
    if _count(stage, "accepted_iterations", label) <= 0:
        raise fail(f"{label}/accepted_iterations: zero")
    initial = _finite(stage, "initial_cost", label)
    final = _finite(stage, "final_cost", label)
    _true(stage, "costs_finite", label)
    _true(stage, "strict_cost_decrease", label)
    if not final < initial:
        raise fail(f"{label}: costs are not decreasing")
    _true(stage, "no_fallback", label)
    terminal = _required(stage, "terminal_branch", label)
    if not isinstance(terminal, str) or not terminal:
        raise fail(f"{label}/terminal_branch: expected nonempty string")


def _validate_termination(report: Mapping[str, Any], stage: str) -> None:
    label = f"summary/phase143_termination/{stage}"
    _true(report, "selector_enabled", label)
    _equal(_required(report, "stage", label), stage, f"{label}/stage")
    _equal(_count(report, "configured_max_iterations", label), 1000,
           f"{label}/configured_max_iterations")
    _equal(_count(report, "effective_max_iterations", label), 1000,
           f"{label}/effective_max_iterations")
    _true(report, "attempted", label)
    attempted = _count(report, "attempted_outer_iterations", label)
    accepted = _count(report, "accepted_outer_iterations", label)
    rejected = _count(report, "rejected_outer_iterations", label)
    if accepted <= 0 or attempted < accepted or attempted < rejected:
        raise fail(f"{label}: inconsistent iteration counts")
    initial = _finite(report, "initial_cost", label)
    final = _finite(report, "final_cost", label)
    _true(report, "costs_finite", label)
    _true(report, "strict_cost_decrease", label)
    if not final < initial:
        raise fail(f"{label}: costs are not decreasing")
    for key in ("termination_branch", "linear_solver", "elimination", "ordering_type"):
        value = _required(report, key, label)
        if not isinstance(value, str) or not value:
            raise fail(f"{label}/{key}: expected nonempty string")
    for key in ("relative_error_tolerance", "absolute_error_tolerance",
                "error_tolerance", "initial_lambda", "final_lambda",
                "maximum_lambda", "lambda_factor", "lambda_lower_bound",
                "lambda_upper_bound", "min_model_fidelity"):
        _finite(report, key, label)
    for key in ("diagonal_damping", "use_fixed_lambda_factor",
                "explicit_ordering_present"):
        _bool(report, key, label)
    for key in ("no_fallback", "termination_trace_complete", "configuration_valid"):
        _true(report, key, label)
    failure = _required(report, "configuration_failure", label)
    _equal(failure, "", f"{label}/configuration_failure")


def validate_phase143_sidecar(summary: Mapping[str, Any]) -> None:
    sidecar = _required(summary, "phase143_termination", "summary")
    if not isinstance(sidecar, Mapping):
        raise fail("summary/phase143_termination: expected object")
    _equal(_required(sidecar, "schema_version", "summary/phase143_termination"),
           "smartphone-r5-native-fgo-phase143-termination.v1",
           "summary/phase143_termination/schema_version")
    _equal(_required(sidecar, "authority", "summary/phase143_termination"),
           "FGOResult.diagnostics.native_phase143_termination",
           "summary/phase143_termination/authority")
    _validate_termination(_required(sidecar, "main", "summary/phase143_termination"),
                          "main")
    _validate_termination(
        _required(sidecar, "gnss_first", "summary/phase143_termination"),
        "gnss_first",
    )
    _equal(_required(sidecar, "no_solution_or_accuracy_fields",
                     "summary/phase143_termination"), True,
           "summary/phase143_termination/no_solution_or_accuracy_fields")


def validate_native_summary(route: str, summary: Mapping[str, Any]) -> None:
    """Validate the native Phase144 object without normalizing or inferring."""
    if route not in ROUTES:
        raise fail(f"unknown route {route!r}")
    _equal(summary.get("dataset_id"), route, f"summary/{route}/dataset_id")
    _equal(summary.get("truth_used"), False, f"summary/{route}/truth_used")
    _equal(summary.get("no_base_contract"), True, f"summary/{route}/no_base_contract")
    _equal(summary.get("native_phase144_telemetry_schema"), True,
           f"summary/{route}/native_phase144_telemetry_schema")
    for forbidden in ("coordinate_rows", "solution_rows", "solution_values"):
        if forbidden in summary:
            raise fail(f"summary/{route} contains solution content: {forbidden}")
    telemetry = _required(summary, "phase144_telemetry", f"summary/{route}")
    if not isinstance(telemetry, Mapping):
        raise fail(f"summary/{route}/phase144_telemetry: expected object")
    _equal(telemetry.get("schema_version"), NATIVE_SCHEMA,
           f"summary/{route}/phase144_telemetry/schema_version")
    _equal(_count(telemetry, "sync_count", f"summary/{route}/phase144_telemetry"),
           1, f"summary/{route}/phase144_telemetry/sync_count")

    authority = _required(telemetry, "authority", f"summary/{route}/phase144_telemetry")
    if not isinstance(authority, Mapping):
        raise fail(f"summary/{route}/authority: expected object")
    for key in ("gnss_first", "main", "raw_base", "offset", "output",
                "wrapper_read_accounting"):
        value = _required(authority, key, f"summary/{route}/authority")
        if not isinstance(value, str) or not value:
            raise fail(f"summary/{route}/authority/{key}: expected source label")

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
        "phase143_official_main_lm_termination_budget": True,
        "phase144_telemetry_schema": True,
    }
    selectors = _required(telemetry, "selectors", f"summary/{route}/phase144_telemetry")
    if not isinstance(selectors, Mapping):
        raise fail(f"summary/{route}/selectors: expected object")
    _equal(set(selectors), set(expected_selectors), f"summary/{route}/selectors/keys")
    for key, expected in expected_selectors.items():
        _equal(selectors.get(key), expected, f"summary/{route}/selectors/{key}")

    validate_equation(_required(telemetry, "equation", f"summary/{route}/telemetry"),
                      f"summary/{route}/telemetry/equation")

    phase135 = _required(telemetry, "phase135", f"summary/{route}/telemetry")
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

    phase138 = _required(telemetry, "phase138", f"summary/{route}/telemetry")
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
    validate_equation(_required(phase138, "equation", f"summary/{route}/phase138"),
                      f"summary/{route}/phase138/equation")

    base = _required(telemetry, "raw_base", f"summary/{route}/telemetry")
    if not isinstance(base, Mapping):
        raise fail(f"summary/{route}/raw_base: expected object")
    for key in ("phase107_recipe", "applied_exactly_once",
                "source_miss_conservation", "no_raw_or_zero_fallback"):
        _true(base, key, f"summary/{route}/raw_base")

    clock = _required(telemetry, "clock", f"summary/{route}/telemetry")
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

    solver = _required(telemetry, "solver", f"summary/{route}/telemetry")
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
        _validate_stage(data, f"summary/{route}/solver/{stage}")

    factor_counts = _required(telemetry, "factor_counts", f"summary/{route}/telemetry")
    if not isinstance(factor_counts, Mapping):
        raise fail(f"summary/{route}/factor_counts: expected object")
    for key in ("pseudorange", "doppler", "ordinary_tdcp"):
        _equal(_count(factor_counts, key, f"summary/{route}/factor_counts"),
               families[key], f"summary/{route}/factor_counts/{key}")
    for key in ("legacy_pseudorange", "legacy_doppler", "legacy_tdcp"):
        _equal(_count(factor_counts, key, f"summary/{route}/factor_counts"), 0,
               f"summary/{route}/factor_counts/{key}")

    bridge_report = _required(telemetry, "bridge", f"summary/{route}/telemetry")
    if not isinstance(bridge_report, Mapping):
        raise fail(f"summary/{route}/bridge: expected object")
    _true(bridge_report, "pose3_x_keys_exact", f"summary/{route}/bridge")
    _equal(_count(bridge_report, "phase131_sync_count", f"summary/{route}/bridge"),
           0, f"summary/{route}/bridge/phase131_sync_count")

    offset = _required(telemetry, "offset", f"summary/{route}/telemetry")
    if not isinstance(offset, Mapping):
        raise fail(f"summary/{route}/offset: expected object")
    for key in ("enabled", "applied"):
        _true(offset, key, f"summary/{route}/offset")
    _equal(_count(offset, "application_passes", f"summary/{route}/offset"), 1,
           f"summary/{route}/offset/application_passes")

    output = _required(telemetry, "output", f"summary/{route}/telemetry")
    if not isinstance(output, Mapping):
        raise fail(f"summary/{route}/output: expected object")
    for key in ("finite", "earth_valid", "expected_epoch_coverage",
                "opaque_solution_seal"):
        _true(output, key, f"summary/{route}/output")
    _equal(_required(output, "coordinate_rows_interpreted", f"summary/{route}/output"),
           False, f"summary/{route}/output/coordinate_rows_interpreted")
    _equal(_count(output, "pixel5_offset_applications", f"summary/{route}/output"), 1,
           f"summary/{route}/output/pixel5_offset_applications")
    for key in ("coordinate_rows", "solution_rows", "solution_values"):
        if key in output:
            raise fail(f"summary/{route}/output contains solution content: {key}")

    policy = _required(telemetry, "policy", f"summary/{route}/telemetry")
    if not isinstance(policy, Mapping):
        raise fail(f"summary/{route}/policy: expected object")
    for key in ("truth_used", "coordinate_rows_interpreted",
                "solution_publication_authorized", "fallback", "rerun"):
        _equal(_required(policy, key, f"summary/{route}/policy"), False,
               f"summary/{route}/policy/{key}")

    top_phase138 = _required(summary, "phase138_affine_tdcp_anchor_range_constant",
                             f"summary/{route}")
    if not isinstance(top_phase138, Mapping):
        raise fail(f"summary/{route}/phase138_affine...: expected object")
    _equal(_required(top_phase138, "measurement_equation", "summary/phase138"),
           EQUATION_DISPLAY, "summary/phase138/measurement_equation")
    validate_equation(_required(top_phase138, "equation", "summary/phase138"),
                      "summary/phase138/equation")
    validate_phase143_sidecar(summary)


def validate_freeze(freeze: Mapping[str, Any]) -> None:
    _equal(freeze.get("schema_version"),
           "smartphone-r5-phase144-telemetry-serializer-freeze.v1",
           "freeze/schema_version")
    _equal(freeze.get("phase"), 144, "freeze/phase")
    _equal(freeze.get("execution_label"), "Luna Max", "freeze/execution_label")
    candidate = _required(freeze, "candidate", "freeze")
    if not isinstance(candidate, Mapping):
        raise fail("freeze/candidate: expected object")
    for key, expected in {
        "count": 1,
        "id": "phase144-telemetry-serializer-canonical-summary-v1",
        "selector": "--native-phase144-telemetry-schema",
        "default_off": True,
        "partial_selector_set_allowed": False,
        "telemetry_only": True,
        "algorithm_changed": False,
        "graph_changed": False,
        "factor_topology_changed": False,
        "equation_evaluation_changed": False,
        "unit_sigma_filter_lm_changed": False,
        "initialization_changed": False,
        "output_solution_rows_changed": False,
        "solution_or_accuracy_fields": False,
        "historical_result_mutated": False,
    }.items():
        _equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    audit = _required(freeze, "audit", "freeze")
    if not isinstance(audit, Mapping):
        raise fail("freeze/audit: expected object")
    _equal(audit.get("path"), relative(AUDIT), "freeze/audit/path")
    _equal(audit.get("commit"), AUDIT_COMMIT, "freeze/audit/commit")
    _equal(audit.get("sha256"), AUDIT_SHA256, "freeze/audit/sha256")
    equation = _required(freeze, "canonical_equation", "freeze")
    if not isinstance(equation, Mapping):
        raise fail("freeze/canonical_equation: expected object")
    validate_equation(equation, "freeze/canonical_equation")
    boundary = _required(freeze, "authorization_boundary", "freeze")
    if not isinstance(boundary, Mapping):
        raise fail("freeze/authorization_boundary: expected object")
    for key in ("implementation_commit_before_runner",
                "launch_free_qualification_required",
                "new_independent_raw_authorization_required",
                "raw_reads_before_authorization", "solver_invocations_before_authorization",
                "truth_reads", "mat_pdc_precomputed_reads", "accuracy_evaluation",
                "kaggle_access", "rerun_fallback_repair"):
        if key not in boundary:
            raise fail(f"freeze/authorization_boundary/{key}: missing")


def validate_manifest(manifest: Mapping[str, Any]) -> None:
    for key, expected in {
        "schema_version": SCHEMA,
        "phase": 144,
        "status": "launch-free",
        "audit_commit": AUDIT_COMMIT,
        "freeze_commit": FREEZE_COMMIT,
        "freeze_sha256": FREEZE_SHA256,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "native_schema_version": NATIVE_SCHEMA,
        "routes": list(ROUTES),
    }.items():
        _equal(manifest.get(key), expected, f"manifest/{key}")
    for key, path in (("audit_path", AUDIT), ("freeze_path", FREEZE),
                      ("pre_raw_path", PRE_RAW)):
        _equal(manifest.get(key), relative(path), f"manifest/{key}")
    recipe = _required(manifest, "recipe", "manifest")
    if not isinstance(recipe, Mapping):
        raise fail("manifest/recipe: expected object")
    for key, expected in {
        "native_phase144_selector": "--native-phase144-telemetry-schema",
        "native_phase143_selector": "--native-phase143-official-main-lm-termination-budget",
        "native_phase135_selector": "--native-phase135-official-affine-measurement-family",
        "native_phase138_selector": "--native-phase138-affine-tdcp-anchor-range-constant",
        "native_phase118_selector": "--native-phase118-official-tdcp-huber-k",
        "native_phase99_solver": "MULTIFRONTAL_QR",
        "native_phase99_elimination": "EliminateQR",
        "phase144_default_off": True,
        "wrapper_inference": False,
        "native_summary_overwrite": False,
        "duplicate_json_keys_fail_closed": True,
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
    source = NATIVE_SOURCE.read_text(encoding="utf-8")
    for needle in (
        "--native-phase144-telemetry-schema",
        "phase144_telemetry",
        NATIVE_SCHEMA,
        "handoff_failure",
        "kPhase138EquationDisplay",
        "phase143_official_main_lm_termination_budget",
    ):
        if needle not in source:
            raise fail(f"native Phase144 witness missing: {needle}")
    result[relative(BINARY)] = static_sha256(BINARY, "Phase144 target binary")
    return result


def launch_free_validation() -> dict[str, Any]:
    freeze = read_json(FREEZE, "Phase144 freeze")
    manifest = read_json(MANIFEST, "Phase144 manifest")
    _equal(static_sha256(AUDIT, "Phase144 audit"), AUDIT_SHA256,
           "audit sha256")
    _equal(static_sha256(FREEZE, "Phase144 freeze"), FREEZE_SHA256,
           "freeze sha256")
    validate_freeze(freeze)
    validate_manifest(manifest)
    source_hashes = validate_static_sources()
    _equal(manifest.get("source_pins"), source_hashes, "manifest/source_pins")
    if PRE_RAW.is_file():
        pre_raw = read_json(PRE_RAW, "Phase144 pre-raw")
        _equal(pre_raw.get("manifest_commit"), manifest.get("manifest_commit"),
               "pre-raw/manifest_commit")
    return {
        "status": "launch-free-qualified",
        "phase": 144,
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
    except Phase144ContractError as exc:
        print(f"PHASE144_FAIL_CLOSED: {exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
