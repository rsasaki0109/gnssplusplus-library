#!/usr/bin/env python3
"""Sealed-artifact-only Phase86 reclassification of the Phase85-v2 matrix.

Phase85's native structural evaluator declared a converged candidate matrix,
but its eight summaries report zero graph iterations and unchanged graph
costs.  This evaluator rereads only the Phase86 freeze, the Phase86 contract
manifest, the immutable Phase85-v2 aggregate and output manifest, and the
eight pinned candidate submission/summary pairs.  It never launches a native
process and never opens raw GNSS/IMU/navigation, truth, MAT, validation,
Kaggle, token, or Phase82 coordinate artifacts.

The result is deliberately a formal NO-GO when the structural artifacts are
otherwise valid but the active-solve materiality gates fail.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase86_phase85_source_clock_c0d_reclassification_freeze_v1.json"
FREEZE_SHA256 = "9b13d6a5118c734521e69d350c9478f0844c5bbf9047c5e057bf4f5a427ace46"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase86_phase85_source_clock_c0d_reclassification_manifest_v1.json"
EVALUATOR = Path(__file__).resolve()
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase86_phase85_source_clock_c0d_reclassification.py"
TESTS_CMAKE = ROOT / "tests/CMakeLists.txt"

PHASE85_ROOT = ROOT / "output/smartphone-r5/phase85-source-clock-c0d-structural-v2"
PHASE85_RESULT = PHASE85_ROOT / "phase85_source_clock_c0d_structural_result.json"
# The Phase86 freeze contains this historical declaration verbatim.  It is
# one character short of the digest in the immutable Phase85 output manifest;
# both values are retained so the hash inconsistency is reported as a gate,
# never silently repaired.
PHASE85_RESULT_FREEZE_DECLARED_SHA256 = "2ac9919e57bb18e52ccb81bc73471af017af77fe3dad65d5750f919d5b1cc4a"
PHASE85_RESULT_SHA256 = "2ac9919e57bb18e52ccb81bc73471af017af77fe3dad65d5750f919d5b1cc4a6"
PHASE85_RESULT_BYTES = 641315
PHASE85_OUTPUT_MANIFEST = PHASE85_ROOT / "phase85_source_clock_c0d_structural_manifest.json"
PHASE85_OUTPUT_MANIFEST_SHA256 = "5bb7bdaeb0f1fb2c00d7fe1354cd0cda1c289b2b664840b4b39135380574c53c"
PHASE85_OUTPUT_MANIFEST_BYTES = 921
PHASE85_RETRY_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase85_source_clock_c0d_structural_manifest_v1.json"
PHASE85_RETRY_MANIFEST_SHA256 = "20adc2e3388b92d045580739d025106c18aa068fcf26f8d117d48d3d2f5f7058"
PHASE85_RETRY_MANIFEST_BYTES = 10529

OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase86-phase85-source-clock-c0d-reclassification-v1"
OUTPUT_SCHEMA = "smartphone-r5-phase86-phase85-source-clock-c0d-reclassification-result.v1"
OUTPUT_MANIFEST_SCHEMA = "smartphone-r5-phase86-phase85-source-clock-c0d-reclassification-output-manifest.v1"

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

CLOCK_FLAG = "--native-source-clock-c0d-factor"
DIRECT_FLAG = "--native-source-direct-observable-quality"
NO_BRIDGE_FLAG = "--native-pdc-imu-tdcp-no-bridge"
SIGNAL_BIAS_FLAG = "--native-signal-bias-states"
UPSTREAM_FLAG = "--native-upstream-quality"
PRESERVE_BANDS_FLAG = "--native-base-pseudorange-preserve-additional-frequency-bands"
SPEED_OF_LIGHT_MPS = 299792458.0
C0D_SIGMA_SECONDS = 0.1 / SPEED_OF_LIGHT_MPS
EARTH_RADIUS_M = 6_371_000.0
MAX_SPEED_MPS = 70.0

FIXED_GATE_NAMES = (
    "sealed_phase85_v2_artifact_hashes_exact",
    "candidate_only_exact_four_routes",
    "rows_equal_pinned_phase80_phase78_domain",
    "rows_plus_one_equals_summary_epochs_output",
    "repeat_submission_and_summary_byte_identical",
    "converged",
    "finite_coordinates_and_earth_valid",
    "speed_over_70_zero",
    "finite_initial_and_final_cost",
    "graph_iterations_at_least_one",
    "final_cost_strictly_less_than_initial_cost",
    "phase80_direct_no_pdc_base_miss_signal_bias_pd_tdcp_invariants",
    "c0d_telemetry_exact_equation_jacobian_units_sigma",
    "c0d_factor_count_positive",
    "c0d_phone_exclusion_zero",
    "c0d_legacy_scalar_between_zero",
    "c0d_factor_plus_all_skips_equals_epochs_minus_one",
    "c0d_dt_range_positive",
    "candidate_flag_once_and_dependencies",
    "truth_free",
    "accuracy_not_scored",
    "all_gates_anded",
)


class Phase86Error(ValueError):
    """Raised when a sealed Phase86 contract is malformed or changed."""


def fail(message: str) -> Phase86Error:
    return Phase86Error(message)


def reject_forbidden(path: Path | str) -> None:
    """Reject paths outside the sealed Phase85 artifact boundary."""

    token = str(path).lower()
    if token.endswith(".mat") or ".mat/" in token or ".mat\\" in token:
        raise fail(f"MAT path is forbidden: {path}")
    for term in ("ground_truth", "validation", "holdout", "kaggle", "token"):
        if term in token:
            raise fail(f"forbidden artifact path: {path}")


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _read_bytes(
    path: Path,
    label: str,
    *,
    expected_sha256: str | None = None,
    expected_bytes: int | None = None,
    counters: dict[str, int] | None = None,
    counter_key: str | None = None,
) -> bytes:
    reject_forbidden(path)
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    if expected_bytes is not None and len(payload) != expected_bytes:
        raise fail(f"{label} byte-size mismatch: {path}")
    digest = hashlib.sha256(payload).hexdigest()
    if expected_sha256 is not None and digest != expected_sha256:
        raise fail(f"{label} SHA-256 mismatch: {path}")
    if counters is not None and counter_key is not None:
        counters[counter_key] = counters.get(counter_key, 0) + 1
    return payload


def _hash_file(path: Path, label: str) -> str:
    return hashlib.sha256(_read_bytes(path, label)).hexdigest()


def _json_payload(payload: bytes, label: str) -> dict[str, Any]:
    try:
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"invalid {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def _read_json(
    path: Path,
    label: str,
    *,
    expected_sha256: str | None = None,
    expected_bytes: int | None = None,
    counters: dict[str, int] | None = None,
    counter_key: str | None = None,
) -> tuple[dict[str, Any], bytes]:
    payload = _read_bytes(
        path,
        label,
        expected_sha256=expected_sha256,
        expected_bytes=expected_bytes,
        counters=counters,
        counter_key=counter_key,
    )
    return _json_payload(payload, label), payload


def _atomic_write(path: Path, payload: bytes) -> None:
    reject_forbidden(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with open(descriptor, "wb", closefd=True) as handle:
            handle.write(payload)
            handle.flush()
        Path(temporary).replace(path)
        temporary = ""
    finally:
        if temporary:
            try:
                Path(temporary).unlink()
            except FileNotFoundError:
                pass


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    _atomic_write(path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def _finite_tree(value: Any, label: str) -> None:
    if isinstance(value, bool):
        return
    if isinstance(value, (int, float)) and not math.isfinite(float(value)):
        raise fail(f"nonfinite value: {label}")
    if isinstance(value, dict):
        for key, item in value.items():
            _finite_tree(item, f"{label}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _finite_tree(item, f"{label}[{index}]")


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise fail(f"missing/nonfinite number: {label}")
    return float(value)


def _count(mapping: dict[str, Any], key: str, label: str, minimum: int = 0) -> int:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise fail(f"invalid count: {label}/{key}")
    return value


def _expected_manifest_pins(freeze: dict[str, Any]) -> dict[str, Any]:
    pins = freeze.get("candidate_artifact_pins")
    if not isinstance(pins, dict) or tuple(pins) != ROUTES:
        raise fail("Phase86 candidate artifact routes changed")
    for route in ROUTES:
        route_pins = pins.get(route)
        if not isinstance(route_pins, dict) or set(route_pins) != {"candidate_run1", "candidate_run2"}:
            raise fail(f"Phase86 candidate run pins incomplete: {route}")
        for run in ("candidate_run1", "candidate_run2"):
            run_pins = route_pins[run]
            if not isinstance(run_pins, dict) or set(run_pins) != {"submission", "summary"}:
                raise fail(f"Phase86 artifact kinds incomplete: {route}/{run}")
            for kind in ("submission", "summary"):
                pin = run_pins[kind]
                if not isinstance(pin, dict) or not isinstance(pin.get("path"), str) or not isinstance(pin.get("sha256"), str) or len(pin["sha256"]) != 64 or not isinstance(pin.get("bytes"), int) or pin["bytes"] <= 0:
                    raise fail(f"Phase86 artifact pin malformed: {route}/{run}/{kind}")
                path = Path(pin["path"])
                if path.is_absolute() or path.parts[: len(PHASE85_ROOT.relative_to(ROOT).parts)] != PHASE85_ROOT.relative_to(ROOT).parts:
                    raise fail(f"Phase86 artifact leaves Phase85-v2 root: {route}/{run}/{kind}")
                if kind == "submission" and not path.name.endswith("submission.csv"):
                    raise fail(f"Phase86 submission pin has unexpected name: {path}")
                if kind == "summary" and path.name != "summary.json":
                    raise fail(f"Phase86 summary pin has unexpected name: {path}")
    return pins


def _verify_phase86_manifest(freeze: dict[str, Any]) -> dict[str, Any]:
    manifest, _ = _read_json(MANIFEST, "Phase86 manifest")
    if manifest.get("schema_version") != "smartphone-r5-phase86-phase85-source-clock-c0d-reclassification-manifest.v1" or manifest.get("phase") != 86 or manifest.get("status") != "frozen-before-phase86-sealed-artifact-read" or manifest.get("execution_label") != "Luna Max":
        raise fail("Phase86 manifest schema/status changed")
    if manifest.get("freeze") != {"path": relative(FREEZE), "sha256": FREEZE_SHA256}:
        raise fail("Phase86 manifest freeze pin changed")
    authority = manifest.get("authority", {})
    if authority.get("phase85_retry_manifest") != {"path": relative(PHASE85_RETRY_MANIFEST), "sha256": PHASE85_RETRY_MANIFEST_SHA256, "bytes": PHASE85_RETRY_MANIFEST_BYTES}:
        raise fail("Phase86 manifest Phase85 retry-manifest pin changed")
    if authority.get("phase85_v2_result") != {"path": relative(PHASE85_RESULT), "sha256": PHASE85_RESULT_FREEZE_DECLARED_SHA256, "verified_sha256": PHASE85_RESULT_SHA256, "bytes": PHASE85_RESULT_BYTES}:
        raise fail("Phase86 manifest aggregate pin changed")
    if authority.get("phase85_v2_output_manifest") != {"path": relative(PHASE85_OUTPUT_MANIFEST), "sha256": PHASE85_OUTPUT_MANIFEST_SHA256, "bytes": PHASE85_OUTPUT_MANIFEST_BYTES}:
        raise fail("Phase86 manifest Phase85 output-manifest pin changed")
    if manifest.get("routes") != list(ROUTES) or manifest.get("phase80_phase78_domain_rows") != DOMAIN_ROWS:
        raise fail("Phase86 manifest route/domain pin changed")
    if manifest.get("candidate_artifact_pins") != _expected_manifest_pins(freeze):
        raise fail("Phase86 manifest candidate artifact pins changed")
    evaluator = manifest.get("evaluator", {})
    tests = manifest.get("focused_tests", {})
    cmake = manifest.get("cmake", {})
    if evaluator.get("path") != relative(EVALUATOR) or evaluator.get("sha256") != _hash_file(EVALUATOR, "Phase86 evaluator"):
        raise fail("Phase86 evaluator hash pin changed")
    if tests.get("path") != relative(FOCUSED_TESTS) or tests.get("sha256") != _hash_file(FOCUSED_TESTS, "Phase86 focused tests"):
        raise fail("Phase86 focused-test hash pin changed")
    if cmake.get("path") != relative(TESTS_CMAKE) or cmake.get("sha256") != _hash_file(TESTS_CMAKE, "tests CMake"):
        raise fail("Phase86 CMake hash pin changed")
    policy = manifest.get("read_policy", {})
    expected_policy = {
        "sealed_phase85_v2_artifacts_only": True,
        "native_reruns": 0,
        "raw_device_gnss_reads": 0,
        "raw_device_imu_reads": 0,
        "broadcast_navigation_reads": 0,
        "base_rinex_reads": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "phase82_candidate_coordinate_reads": 0,
        "precomputed_coordinate_reads": 0,
        "kaggle_or_token_access": 0,
        "accuracy_scored": False,
        "route_score_selection": False,
    }
    if policy != expected_policy:
        raise fail("Phase86 manifest read policy changed")
    if manifest.get("fixed_reclassification_gates") != {key: True for key in FIXED_GATE_NAMES}:
        raise fail("Phase86 manifest fixed-gate declaration changed")
    return manifest


def verify_freeze() -> dict[str, Any]:
    """Verify the Phase86 freeze and its local contract manifest only."""

    freeze, _ = _read_json(FREEZE, "Phase86 freeze", expected_sha256=FREEZE_SHA256)
    if freeze.get("schema_version") != "smartphone-r5-phase86-phase85-source-clock-c0d-reclassification-freeze.v1" or freeze.get("phase") != 86 or freeze.get("status") != "frozen-before-phase86-sealed-artifact-read" or freeze.get("execution_label") != "Luna Max":
        raise fail("Phase86 freeze schema/status changed")
    boundary = freeze.get("decision_boundary", {})
    if boundary.get("phase85_v2_go_is_legacy_diagnostic_only") is not True or boundary.get("phase86_promotion_requires") != "every pinned candidate run has finite costs, graph iterations >= 1, and final_cost < initial_cost" or boundary.get("accuracy_scoring") is not False:
        raise fail("Phase86 decision boundary changed")
    scope = freeze.get("scope", {})
    expected_zero = ("native_reruns", "raw_device_gnss_reads", "raw_device_imu_reads", "broadcast_navigation_reads", "base_rinex_reads", "truth_reads", "mat_reads_or_generated", "phase82_candidate_coordinate_reads", "precomputed_coordinate_reads", "kaggle_or_token_access")
    if scope.get("sealed_phase85_artifacts_only") is not True or scope.get("accuracy_scored") is not False or scope.get("route_score_selection") is not False or any(scope.get(key) != 0 for key in expected_zero):
        raise fail("Phase86 scope/read declaration changed")
    if freeze.get("routes") != list(ROUTES) or freeze.get("phase80_phase78_domain_rows") != DOMAIN_ROWS:
        raise fail("Phase86 route/domain declaration changed")
    if freeze.get("fixed_reclassification_gates") != {key: True for key in FIXED_GATE_NAMES}:
        raise fail("Phase86 fixed-gate declaration changed")
    authority = freeze.get("authority", {})
    aggregate_pin = authority.get("phase85_v2_result", {})
    output_manifest_pin = authority.get("phase85_v2_output_manifest", {})
    if aggregate_pin != {"path": relative(PHASE85_RESULT), "sha256": PHASE85_RESULT_FREEZE_DECLARED_SHA256, "bytes": PHASE85_RESULT_BYTES, "status": "legacy-evaluator-go-superseded-by-phase86"}:
        raise fail("Phase85-v2 aggregate authority pin changed")
    if output_manifest_pin != {"path": relative(PHASE85_OUTPUT_MANIFEST), "sha256": PHASE85_OUTPUT_MANIFEST_SHA256, "bytes": PHASE85_OUTPUT_MANIFEST_BYTES}:
        raise fail("Phase85-v2 output-manifest authority pin changed")
    _expected_manifest_pins(freeze)
    _verify_phase86_manifest(freeze)
    return freeze


def _read_submission(payload: bytes, route: str) -> list[tuple[int, float, float]]:
    try:
        text = payload.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise fail(f"submission is not UTF-8: {route}") from exc
    rows = list(csv.reader(io.StringIO(text)))
    header = ["phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"]
    if not rows or rows[0] != header:
        raise fail(f"submission header mismatch: {route}")
    parsed: list[tuple[int, float, float]] = []
    previous: int | None = None
    for line_number, fields in enumerate(rows[1:], start=2):
        if len(fields) != 4 or fields[0] != route:
            raise fail(f"submission row key mismatch: {route}:{line_number}")
        try:
            timestamp = int(fields[1])
            latitude = float(fields[2])
            longitude = float(fields[3])
        except ValueError as exc:
            raise fail(f"non-numeric submission row: {route}:{line_number}") from exc
        if previous is not None and timestamp <= previous:
            raise fail(f"submission timestamps are not increasing: {route}")
        if not all(math.isfinite(value) for value in (latitude, longitude)) or not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise fail(f"invalid earth coordinate: {route}:{line_number}")
        parsed.append((timestamp, latitude, longitude))
        previous = timestamp
    if not parsed:
        raise fail(f"empty submission: {route}")
    return parsed


def _speed_report(rows: list[tuple[int, float, float]]) -> dict[str, Any]:
    speeds: list[float] = []
    for previous, current in zip(rows, rows[1:]):
        dt = (current[0] - previous[0]) / 1000.0
        if dt <= 0.0:
            raise fail("non-positive submission interval")
        lat1, lon1, lat2, lon2 = map(math.radians, (previous[1], previous[2], current[1], current[2]))
        dlat, dlon = lat2 - lat1, lon2 - lon1
        hav = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
        distance = EARTH_RADIUS_M * 2.0 * math.asin(math.sqrt(min(1.0, max(0.0, hav))))
        speeds.append(distance / dt)
    return {
        "finite": all(math.isfinite(speed) for speed in speeds),
        "max_speed_mps": max(speeds, default=0.0),
        "over_70_mps_count": sum(speed > MAX_SPEED_MPS for speed in speeds),
        "transition_count": len(speeds),
    }


def read_prediction(path: Path, route: str) -> list[tuple[int, float, float]]:
    """Public alias used by focused tests and downstream audit tooling."""

    return _read_submission(_read_bytes(path, f"submission {route}"), route)


def speed_report(rows: list[tuple[int, float, float]]) -> dict[str, Any]:
    return _speed_report(rows)


def _validate_base_and_miss(summary: dict[str, Any], route: str) -> tuple[dict[str, Any], dict[str, Any]]:
    base = summary.get("native_base_pseudorange_compensation")
    miss = summary.get("native_base_pseudorange_source_miss_mask")
    if not isinstance(base, dict) or not isinstance(miss, dict):
        raise fail(f"base/miss-mask telemetry missing: {route}")
    if any(base.get(key) is not True for key in ("enabled", "built", "applied", "same_satellite_signal_only", "no_extrapolation_or_endpoint_hold")):
        raise fail(f"base contract failed: {route}")
    if any(base.get(key) is not False for key in ("preserve_additional_frequency_bands", "spp_applied", "tdcp_applied", "doppler_applied")):
        raise fail(f"base frequency policy failed: {route}")
    if base.get("base_member_sha256") != base.get("base_rinex_sha256") or not isinstance(base.get("base_rinex_sha256"), str) or len(base["base_rinex_sha256"]) != 64 or _count(base, "base_rinex_bytes", route, 1) <= 0 or _count(base, "base_rinex_read_count", route, 1) != 1:
        raise fail(f"base artifact identity failed: {route}")
    if base.get("matching_key") != "(satellite,signal)" or base.get("scope") != "adopted undifferenced FGO pseudorange factors only":
        raise fail(f"base matching/scope contract failed: {route}")
    if miss.get("enabled") is not True or miss.get("retained_factor_epoch_indices_unchanged") is not True or miss.get("tdcp_doppler_imu_spp_unchanged") is not True or miss.get("no_extrapolation_or_endpoint_hold") is not True:
        raise fail(f"miss-mask scope failed: {route}")
    original = _count(miss, "original_adopted_pseudorange_rows", route, 1)
    retained = _count(miss, "retained_finite_pc_pseudorange_rows", route, 1)
    drops = [_count(miss, key, route) for key in ("dropped_missing_exact_stream_rows", "dropped_out_of_domain_rows", "dropped_nonfinite_correction_rows")]
    if original != retained + sum(drops) or miss.get("retained_finite_pc_fraction") != 1 or miss.get("pseudorange_factor_count_consistent") is not True or miss.get("pseudorange_factors_inserted") != retained:
        raise fail(f"miss-mask factor accounting failed: {route}")
    if base.get("adopted_pseudorange_rows") != original or base.get("adopted_rows_corrected") != retained:
        raise fail(f"base/miss adopted-count identity failed: {route}")
    for key in ("retained_over_original_fraction", "correction_abs_p50_m", "correction_abs_p95_m", "correction_abs_max_m"):
        _number(miss.get(key), f"miss/{route}/{key}")
    return base, miss


def _validate_direct(summary: dict[str, Any], route: str) -> tuple[dict[str, Any], dict[str, Any]]:
    if summary.get("native_source_direct_observable_quality_enabled") is not True or summary.get("native_upstream_quality") is not False or summary.get("native_pdc_state_bridge") is not False or summary.get("native_pdc_imu_tdcp_no_bridge") is not True:
        raise fail(f"direct/no-PDC top-level identity failed: {route}")
    direct = summary.get("native_source_direct_observable_quality")
    upstream = summary.get("upstream_observable_quality")
    if not isinstance(direct, dict) or not isinstance(upstream, dict):
        raise fail(f"direct/upstream quality telemetry missing: {route}")
    settings = {
        ROUTES[0]: ("Highway", 0.2, 0.8),
        ROUTES[1]: ("Street", 0.1, 0.4),
        ROUTES[2]: ("Highway", 0.2, 0.8),
        ROUTES[3]: ("Street", 0.1, 0.4),
    }
    environment, p_sigma, d_sigma = settings[route]
    if any(direct.get(key) != value for key, value in (("enabled", True), ("direct_no_pdc", True), ("pdc_bridge", False), ("native_pdc_state_bridge", False), ("environment", environment))):
        raise fail(f"direct quality identity failed: {route}")
    if direct.get("p_and_d_huber") != {"pseudorange_sigma": p_sigma, "doppler_sigma": d_sigma}:
        raise fail(f"direct Huber identity failed: {route}")
    expected_config = {
        "use_upstream_observable_quality": True,
        "upstream_snr_percentile": 85.0,
        "upstream_min_snr_dbhz": 20.0,
        "upstream_min_elevation_deg": 5.0,
        "upstream_max_adjacent_gap_s": 1.5,
        "pseudorange_huber_threshold_sigma": p_sigma,
        "undifferenced_doppler_huber_threshold_sigma": d_sigma,
        "tdcp_sigma_m_unchanged": 0.03,
        "pseudorange_sigma_contract": "snr_scale*signal_type_factor",
        "doppler_sigma_contract": "snr_scale/12",
        "adjacent_mask_contract": "applyAdjacentMasks Pmask_dDP/Lmask_dDL unchanged",
        "pseudorange_residual_screen": "Pmask_res L1=20m/L5=15m",
        "doppler_residual_screen": "Dmask_res 3m/s; non-initializer",
    }
    if direct.get("config") != expected_config:
        raise fail(f"direct quality config failed: {route}")
    expected_upstream = {
        "enabled": True,
        "snr_percentile": 85.0,
        "snr_denominator_db": 20.0,
        "min_snr_dbhz": 20.0,
        "min_elevation_deg": 5.0,
        "max_adjacent_gap_s": 1.5,
        "tdcp_sigma_m_unchanged": 0.03,
        "pseudorange_sigma_contract": "snr_scale*signal_type_factor",
        "doppler_sigma_contract": "snr_scale/12",
    }
    if any(upstream.get(key) != value for key, value in expected_upstream.items()):
        raise fail(f"upstream quality contract failed: {route}")
    counts = direct.get("counts")
    if not isinstance(counts, dict):
        raise fail(f"direct quality counts missing: {route}")
    count_keys = ("pseudorange_candidates", "pseudorange_factors", "doppler_candidates", "doppler_factors", "doppler_graph_factors", "pseudorange_residual_rejections", "doppler_residual_rejections")
    for key in count_keys:
        if _count(counts, key, route) != _count(upstream, key, route):
            raise fail(f"direct/upstream count mismatch: {route}/{key}")
    if counts["pseudorange_candidates"] < counts["pseudorange_factors"] or counts["doppler_candidates"] < counts["doppler_factors"] or counts["pseudorange_factors"] < 1 or counts["doppler_graph_factors"] < 1:
        raise fail(f"direct quality materiality failed: {route}")
    return direct, upstream


def _validate_signal_bias(summary: dict[str, Any], route: str) -> dict[str, Any]:
    if summary.get("native_signal_bias_states") is not True:
        raise fail(f"signal-bias flag telemetry missing: {route}")
    epochs = summary.get("epochs")
    if not isinstance(epochs, dict):
        raise fail(f"signal-bias epoch telemetry missing: {route}")
    factors = _count(epochs, "receiver_signal_bias_factors", route, 1)
    states = _count(epochs, "receiver_signal_bias_states", route, 1)
    if factors < states:
        raise fail(f"signal-bias factor/state materiality failed: {route}")
    estimates = summary.get("receiver_signal_bias_estimates_m")
    if not isinstance(estimates, dict) or len(estimates) != states:
        raise fail(f"signal-bias estimate count mismatch: {route}")
    for key, value in estimates.items():
        _number(value, f"signal-bias/{route}/{key}")
    return {"factors": factors, "states": states, "estimates_m": estimates}


def _validate_clock(summary: dict[str, Any], route: str, expected_rows: int) -> dict[str, Any]:
    telemetry = summary.get("native_source_clock_c0d_factor")
    if summary.get("native_source_clock_c0d_factor_enabled") is not True or not isinstance(telemetry, dict) or telemetry.get("clock_c0d_enabled") is not True:
        raise fail(f"C0/D telemetry is not enabled: {route}")
    factor_count = _count(telemetry, "clock_c0d_factor_count", route, 1)
    skip_keys = ("clock_c0d_clock_jump_skips", "clock_c0d_gap_skips", "clock_c0d_invalid_dt_skips", "clock_c0d_phone_exclusion_skips")
    skips = {key: _count(telemetry, key, route) for key in skip_keys}
    if skips["clock_c0d_phone_exclusion_skips"] != 0:
        raise fail(f"Pixel5 phone exclusion is nonzero: {route}")
    if factor_count + sum(skips.values()) != expected_rows:
        raise fail(f"C0/D factor/skip accounting mismatch: {route}")
    dt_min = _number(telemetry.get("clock_c0d_dt_min_s"), f"clock-c0d/{route}/dt_min")
    dt_max = _number(telemetry.get("clock_c0d_dt_max_s"), f"clock-c0d/{route}/dt_max")
    if not (0.0 < dt_min <= dt_max < 1.5):
        raise fail(f"C0/D dt range invalid: {route}")
    expected = {
        "clock_c0d_equation": "(c2-c1)-((d1+d2)*dt/(2*C_LIGHT))",
        "clock_c0d_jacobian_order": ["c1", "c2", "d1", "d2"],
        "clock_c0d_jacobian": "[-1,+1,-dt/(2*C_LIGHT),-dt/(2*C_LIGHT)]",
        "clock_c0d_units": {"clock": "seconds", "drift": "metres_per_second", "dt": "seconds", "residual": "seconds", "sigma": "seconds"},
        "clock_jump_noise": "Inf (active C0 factor omitted)",
        "parity_scope": "C0/D active-row parity; not full seven-vector",
        "legacy_scalar_clock_between_factor_count": 0,
    }
    if any(telemetry.get(key) != value for key, value in expected.items()):
        raise fail(f"C0/D telemetry equation/Jacobian/units contract failed: {route}")
    if not math.isclose(_number(telemetry.get("speed_of_light_mps"), f"clock-c0d/{route}/c"), SPEED_OF_LIGHT_MPS, rel_tol=0.0, abs_tol=1e-9) or not math.isclose(_number(telemetry.get("clock_c0d_sigma_seconds"), f"clock-c0d/{route}/sigma"), C0D_SIGMA_SECONDS, rel_tol=1e-12, abs_tol=1e-18):
        raise fail(f"C0/D physical constants mismatch: {route}")
    return {"factor_count": factor_count, **skips, "dt_min_s": dt_min, "dt_max_s": dt_max, "equation": telemetry["clock_c0d_equation"], "jacobian_order": telemetry["clock_c0d_jacobian_order"], "jacobian": telemetry["clock_c0d_jacobian"], "units": telemetry["clock_c0d_units"], "sigma_seconds": telemetry["clock_c0d_sigma_seconds"], "legacy_scalar_clock_between_factor_count": 0}


def _validate_summary(summary: dict[str, Any], route: str, expected_rows: int) -> dict[str, Any]:
    _finite_tree(summary, f"summary[{route}]")
    if summary.get("dataset_id") != route or summary.get("truth_used") is not False or summary.get("status") != "imu-combined-factor" or summary.get("production_default_changed") is not False:
        raise fail(f"summary identity/truth/default gate failed: {route}")
    base, miss = _validate_base_and_miss(summary, route)
    direct, upstream = _validate_direct(summary, route)
    signal_bias = _validate_signal_bias(summary, route)
    epochs = summary.get("epochs")
    graph = summary.get("graph")
    tdcp = summary.get("tdcp_contract")
    utc = summary.get("raw_utc_key_contract")
    if not all(isinstance(item, dict) for item in (epochs, graph, tdcp, utc)):
        raise fail(f"population telemetry missing: {route}")
    assert isinstance(epochs, dict) and isinstance(graph, dict) and isinstance(tdcp, dict) and isinstance(utc, dict)
    if epochs.get("problem") != expected_rows + 1 or epochs.get("output") != expected_rows + 1 or graph.get("imu_intervals") != expected_rows or graph.get("converged") is not True:
        raise fail(f"epoch/IMU/convergence invariant failed: {route}")
    if epochs.get("pseudorange_factors") != miss["retained_finite_pc_pseudorange_rows"]:
        raise fail(f"pseudorange factor/miss-mask identity failed: {route}")
    if tdcp.get("enabled") is not True or tdcp.get("factors_built") != tdcp.get("factors_inserted") or _count(tdcp, "factors_built", route, 1) <= 0 or tdcp.get("finite_residuals") != tdcp.get("factors_inserted") or tdcp.get("nonfinite_residuals") != 0:
        raise fail(f"TDCP built/inserted/finite invariant failed: {route}")
    if not isinstance(utc, dict) or utc.get("warmup_epoch_excluded") is not True or utc.get("raw_epoch_keys") != expected_rows + 1 or utc.get("target_epochs") != expected_rows or utc.get("exact_solution_epochs") != expected_rows or utc.get("interpolated_epochs") != 0 or utc.get("edge_hold_epochs") != 0 or utc.get("unresolved_epochs") != 0 or utc.get("device_wls_coordinates_used") is not False:
        raise fail(f"raw UTC warmup/domain invariant failed: {route}")
    clock = _validate_clock(summary, route, expected_rows)
    initial = _number(graph.get("initial_cost"), f"graph/{route}/initial_cost")
    final = _number(graph.get("final_cost"), f"graph/{route}/final_cost")
    iterations = _count(graph, "iterations", route, 0)
    return {
        "population": {
            "imu_intervals": graph["imu_intervals"],
            "output_epochs": epochs["output"],
            "pseudorange_factors": epochs["pseudorange_factors"],
            "tdcp_factors_built": tdcp["factors_built"],
            "tdcp_factors_inserted": tdcp["factors_inserted"],
            "undifferenced_doppler_factors_inserted": upstream["doppler_graph_factors"],
        },
        "base": {
            "enabled": base["enabled"],
            "built": base["built"],
            "applied": base["applied"],
            "preserve_additional_frequency_bands": base["preserve_additional_frequency_bands"],
            "adopted_pseudorange_rows": base["adopted_pseudorange_rows"],
            "adopted_rows_corrected": base["adopted_rows_corrected"],
        },
        "source_miss_mask": {
            "original_adopted_pseudorange_rows": miss["original_adopted_pseudorange_rows"],
            "retained_finite_pc_pseudorange_rows": miss["retained_finite_pc_pseudorange_rows"],
            "dropped_missing_exact_stream_rows": miss["dropped_missing_exact_stream_rows"],
            "dropped_out_of_domain_rows": miss["dropped_out_of_domain_rows"],
            "dropped_nonfinite_correction_rows": miss["dropped_nonfinite_correction_rows"],
            "retained_finite_pc_fraction": miss["retained_finite_pc_fraction"],
            "pseudorange_factors_inserted": miss["pseudorange_factors_inserted"],
        },
        "direct_quality": {"environment": direct["environment"], "p_and_d_huber": direct["p_and_d_huber"], "counts": direct["counts"]},
        "signal_bias": signal_bias,
        "clock_c0d": clock,
        "raw_utc_key_contract": {key: utc.get(key) for key in ("warmup_epoch_excluded", "raw_epoch_keys", "target_epochs", "exact_solution_epochs", "interpolated_epochs", "edge_hold_epochs", "unresolved_epochs")},
        "graph": {"initial_cost": initial, "final_cost": final, "iterations": iterations, "converged": graph["converged"]},
    }


def validate_summary(path: Path, route: str, expected_rows: int | None = None) -> dict[str, Any]:
    """Validate a summary file when used by focused tests or audits."""

    if expected_rows is None:
        expected_rows = DOMAIN_ROWS.get(route, 0)
    payload = _read_bytes(path, f"summary {route}")
    summary = _json_payload(payload, f"summary {route}")
    return _validate_summary(summary, route, expected_rows)


def _phase85_output_manifest(freeze: dict[str, Any], counters: dict[str, int]) -> dict[str, Any]:
    manifest, _ = _read_json(PHASE85_OUTPUT_MANIFEST, "Phase85-v2 output manifest", expected_sha256=PHASE85_OUTPUT_MANIFEST_SHA256, expected_bytes=PHASE85_OUTPUT_MANIFEST_BYTES, counters=counters, counter_key="phase85_v2_output_manifest")
    if manifest.get("schema_version") != "smartphone-r5-phase85-source-clock-c0d-structural-output-manifest.v1" or manifest.get("phase") != 85 or manifest.get("status") != "sealed-truth-free-structural-matrix" or manifest.get("candidate_runs") != 8 or manifest.get("control_runs") != 0 or manifest.get("truth_reads") != 0 or manifest.get("accuracy_scored") is not False or manifest.get("all_gates_passed") is not True:
        raise fail("Phase85-v2 output manifest legacy contract changed")
    result_claim = manifest.get("result", {})
    if result_claim != {"path": relative(PHASE85_RESULT), "bytes": PHASE85_RESULT_BYTES, "sha256": PHASE85_RESULT_SHA256}:
        raise fail("Phase85-v2 output manifest aggregate claim changed")
    return manifest


def _phase85_retry_manifest(freeze: dict[str, Any], counters: dict[str, int]) -> dict[str, Any]:
    manifest, _ = _read_json(PHASE85_RETRY_MANIFEST, "Phase85 retry manifest", expected_sha256=PHASE85_RETRY_MANIFEST_SHA256, expected_bytes=PHASE85_RETRY_MANIFEST_BYTES, counters=counters, counter_key="phase85_retry_manifest")
    if manifest.get("schema_version") != "smartphone-r5-phase85-source-clock-c0d-structural-manifest.v1" or manifest.get("phase") != 85 or manifest.get("status") != "frozen-before-phase85-v2-raw-read" or manifest.get("execution_label") != "Luna Max":
        raise fail("Phase85 retry manifest identity changed")
    if manifest.get("routes") != list(ROUTES) or manifest.get("phase80_phase78_domain_rows") != DOMAIN_ROWS:
        raise fail("Phase85 retry manifest route/domain contract changed")
    candidate = manifest.get("candidate", {})
    if candidate.get("flag") != CLOCK_FLAG or candidate.get("direct_flag") != DIRECT_FLAG or candidate.get("controls") != 0 or candidate.get("runs_per_route") != 2 or candidate.get("preserve_additional_frequency_bands") is not False or candidate.get("no_control") is not True or candidate.get("no_route_selector") is not True or candidate.get("requires") != [DIRECT_FLAG, NO_BRIDGE_FLAG]:
        raise fail("Phase85 retry manifest candidate contract changed")
    matrix = manifest.get("matrix", {})
    expected_matrix = {"candidate_runs_per_route": 2, "candidate_runs_total": 8, "control_runs_per_route": 0, "native_invocations": 8, "raw_device_gnss_reads": 8, "raw_device_imu_reads": 8, "broadcast_nav_reads": 8, "base_rinex_reads": 8, "truth_reads": 0, "mat_reads_or_generated": 0, "precomputed_coordinate_reads": 0, "kaggle_or_token_access": 0}
    if any(matrix.get(key) != value for key, value in expected_matrix.items()):
        raise fail("Phase85 retry manifest matrix contract changed")
    structural_gates = manifest.get("structural_gates", {})
    expected_structural_gates = ("candidate_only_exact_four_routes", "rows_equal_pinned_phase80_phase78_domain", "rows_plus_one_equals_summary_epochs_output", "repeat_submission_and_summary_byte_identical", "converged_finite_earth_valid_speed_le_70", "phase80_direct_no_pdc_base_miss_signal_bias_pd_tdcp_invariants", "c0d_telemetry_exact_equation_jacobian_units_sigma", "c0d_factor_count_positive", "c0d_phone_exclusion_zero", "c0d_legacy_scalar_between_zero", "c0d_factor_plus_all_skips_equals_epochs_minus_one", "c0d_dt_range_positive", "candidate_flag_once_and_dependencies", "truth_free", "accuracy_not_scored", "all_gates_anded")
    if any(structural_gates.get(key) is not True for key in expected_structural_gates):
        raise fail("Phase85 retry manifest structural gate declaration changed")
    output_policy = manifest.get("output_policy", {})
    if output_policy.get("refuse_nonempty_output_root") is not True or output_policy.get("accuracy_scoring") is not False or output_policy.get("route_score_selection") is not False:
        raise fail("Phase85 retry manifest output policy changed")
    return manifest


def _check_case_claim(case: dict[str, Any], pin: dict[str, Any], route: str, run: int) -> None:
    if case.get("candidate") is not True or case.get("run") != run or case.get("return_code") != 0:
        raise fail(f"Phase85 candidate case execution claim changed: {route}/run{run}")
    for kind in ("submission", "summary"):
        claim = case.get(f"{kind}_artifact")
        if not isinstance(claim, dict) or claim.get("path") != pin[kind]["path"] or claim.get("sha256") != pin[kind]["sha256"] or claim.get("bytes") != pin[kind]["bytes"]:
            raise fail(f"Phase85 candidate {kind} claim changed: {route}/run{run}")
    if case.get("summary_payload") is not None and not isinstance(case.get("summary_payload"), dict):
        raise fail(f"Phase85 candidate summary payload malformed: {route}/run{run}")


def _check_command(case: dict[str, Any], route: str, run: int) -> bool:
    command = case.get("command")
    if not isinstance(command, list) or not all(isinstance(token, str) for token in command):
        raise fail(f"Phase85 candidate command missing: {route}/run{run}")
    for token in command:
        reject_forbidden(token)
    required = (CLOCK_FLAG, DIRECT_FLAG, NO_BRIDGE_FLAG, SIGNAL_BIAS_FLAG)
    if any(command.count(flag) != 1 for flag in required) or UPSTREAM_FLAG in command or PRESERVE_BANDS_FLAG in command:
        raise fail(f"Phase85 candidate flag/dependency contract changed: {route}/run{run}")
    return True


def _artifact_case(
    case: dict[str, Any],
    pin: dict[str, Any],
    route: str,
    run: int,
    expected_rows: int,
    counters: dict[str, int],
) -> dict[str, Any]:
    _check_case_claim(case, pin, route, run)
    submission_path = ROOT / pin["submission"]["path"]
    summary_path = ROOT / pin["summary"]["path"]
    submission_payload = _read_bytes(submission_path, f"candidate submission {route}/run{run}", expected_sha256=pin["submission"]["sha256"], expected_bytes=pin["submission"]["bytes"], counters=counters, counter_key="candidate_submission")
    summary_payload = _read_bytes(summary_path, f"candidate summary {route}/run{run}", expected_sha256=pin["summary"]["sha256"], expected_bytes=pin["summary"]["bytes"], counters=counters, counter_key="candidate_summary")
    rows = _read_submission(submission_payload, route)
    if len(rows) != expected_rows:
        raise fail(f"candidate row count changed: {route}/run{run}")
    speed = _speed_report(rows)
    summary = _json_payload(summary_payload, f"candidate summary {route}/run{run}")
    diagnostics = _validate_summary(summary, route, expected_rows)
    if case.get("summary_payload") != summary:
        raise fail(f"Phase85 aggregate summary payload differs from pinned summary: {route}/run{run}")
    if case.get("submission_artifact", {}).get("rows") != len(rows):
        raise fail(f"Phase85 aggregate submission row claim changed: {route}/run{run}")
    if len(rows) + 1 != diagnostics["population"]["output_epochs"]:
        raise fail(f"rows plus warmup does not equal summary output epochs: {route}/run{run}")
    return {
        "run": run,
        "candidate": True,
        "submission_artifact": {"path": pin["submission"]["path"], "bytes": len(submission_payload), "sha256": pin["submission"]["sha256"], "rows": len(rows)},
        "summary_artifact": {"path": pin["summary"]["path"], "bytes": len(summary_payload), "sha256": pin["summary"]["sha256"]},
        "prediction_keys": [row[0] for row in rows],
        "speed": speed,
        "summary": summary,
        "summary_diagnostics": diagnostics,
        "flag_contract": _check_command(case, route, run),
    }


def _verify_phase85_aggregate(freeze: dict[str, Any], counters: dict[str, int]) -> tuple[dict[str, Any], dict[str, Any]]:
    aggregate, _ = _read_json(PHASE85_RESULT, "Phase85-v2 aggregate", expected_sha256=PHASE85_RESULT_SHA256, expected_bytes=PHASE85_RESULT_BYTES, counters=counters, counter_key="phase85_v2_aggregate")
    if aggregate.get("schema_version") != "smartphone-r5-phase85-source-clock-c0d-structural-result.v1" or aggregate.get("phase") != 85 or aggregate.get("status") != "go-phase85-source-clock-c0d-structural" or aggregate.get("truth_free") is not True or aggregate.get("accuracy_scored") is not False:
        raise fail("Phase85-v2 aggregate legacy identity changed")
    if aggregate.get("phase80_phase78_domain_rows") != DOMAIN_ROWS:
        raise fail("Phase85-v2 aggregate domain rows changed")
    candidate = aggregate.get("candidate", {})
    expected_candidate = {
        "flag": CLOCK_FLAG,
        "direct_flag": DIRECT_FLAG,
        "requires": [DIRECT_FLAG, NO_BRIDGE_FLAG],
        "controls": 0,
        "runs_per_route": 2,
        "preserve_additional_frequency_bands": False,
    }
    if any(candidate.get(key) != value for key, value in expected_candidate.items()):
        raise fail("Phase85-v2 aggregate candidate contract changed")
    accounting = aggregate.get("read_accounting", {})
    expected_accounting = {
        "candidate_runs_per_route": 2,
        "control_runs_per_route": 0,
        "native_solver_invocations": 8,
        "raw_device_gnss_process_reads": 8,
        "raw_device_imu_process_reads": 8,
        "broadcast_nav_reads": 8,
        "base_rinex_reads": 8,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "precomputed_coordinate_reads": 0,
        "kaggle_or_token_access": 0,
        "accuracy_scored": False,
    }
    if any(accounting.get(key) != value for key, value in expected_accounting.items()):
        raise fail("Phase85-v2 aggregate legacy read accounting changed")
    old_gates = aggregate.get("gates", {})
    if old_gates.get("all_passed") is not True or old_gates.get("all_four_routes") is not True or old_gates.get("candidate_only") is not True or old_gates.get("truth_free") is not True or old_gates.get("accuracy_not_scored") is not True:
        raise fail("Phase85-v2 aggregate legacy gates changed")
    if aggregate.get("manifest") != {"path": relative(PHASE85_RETRY_MANIFEST), "sha256": PHASE85_RETRY_MANIFEST_SHA256}:
        raise fail("Phase85-v2 aggregate retry-manifest provenance changed")
    routes = aggregate.get("routes")
    if not isinstance(routes, dict) or tuple(routes) != ROUTES:
        raise fail("Phase85-v2 aggregate route set/order changed")
    pins = _expected_manifest_pins(freeze)
    route_records: dict[str, Any] = {}
    for route in ROUTES:
        record = routes[route]
        if not isinstance(record, dict) or set(record.get("cases", {})) != {"candidate_run1", "candidate_run2"} or record.get("repeat_identity") is not True:
            raise fail(f"Phase85-v2 candidate-only route record changed: {route}")
        route_gates = record.get("gates", {})
        if route_gates.get("all_route_gates") is not True or any(value is not True for key, value in route_gates.items() if key != "all_route_gates"):
            raise fail(f"Phase85-v2 legacy route gates changed: {route}")
        route_records[route] = record
    return aggregate, route_records


def _repeat_identity(first: dict[str, Any], second: dict[str, Any]) -> bool:
    return first["submission_artifact"]["sha256"] == second["submission_artifact"]["sha256"] and first["summary_artifact"]["sha256"] == second["summary_artifact"]["sha256"] and first["submission_artifact"]["bytes"] == second["submission_artifact"]["bytes"] and first["summary_artifact"]["bytes"] == second["summary_artifact"]["bytes"] and first["prediction_keys"] == second["prediction_keys"] and first["summary"] == second["summary"]


def _route_result(route: str, first: dict[str, Any], second: dict[str, Any], expected_rows: int, first_payload: bytes | None = None, second_payload: bytes | None = None, first_summary_payload: bytes | None = None, second_summary_payload: bytes | None = None) -> tuple[dict[str, Any], dict[str, Any]]:
    first_graph = first["summary_diagnostics"]["graph"]
    second_graph = second["summary_diagnostics"]["graph"]
    repeat = _repeat_identity(first, second)
    if first_payload is not None and second_payload is not None:
        repeat = repeat and first_payload == second_payload
    if first_summary_payload is not None and second_summary_payload is not None:
        repeat = repeat and first_summary_payload == second_summary_payload
    active_runs = (f"{route}/candidate_run1", f"{route}/candidate_run2")
    finite_costs = all(math.isfinite(value) for value in (first_graph["initial_cost"], first_graph["final_cost"], second_graph["initial_cost"], second_graph["final_cost"]))
    iterations_pass = first_graph["iterations"] >= 1 and second_graph["iterations"] >= 1
    decrease_pass = first_graph["final_cost"] < first_graph["initial_cost"] and second_graph["final_cost"] < second_graph["initial_cost"]
    gates = {
        "rows_equal_pinned_phase80_phase78_domain": first["submission_artifact"]["rows"] == expected_rows and second["submission_artifact"]["rows"] == expected_rows,
        "rows_plus_one_equals_summary_epochs_output": first["submission_artifact"]["rows"] + 1 == first["summary_diagnostics"]["population"]["output_epochs"] and second["submission_artifact"]["rows"] + 1 == second["summary_diagnostics"]["population"]["output_epochs"],
        "repeat_submission_and_summary_byte_identical": repeat,
        "converged": first_graph["converged"] is True and second_graph["converged"] is True,
        "finite_coordinates_and_earth_valid": first["speed"]["finite"] and second["speed"]["finite"],
        "speed_over_70_zero": first["speed"]["over_70_mps_count"] == 0 and second["speed"]["over_70_mps_count"] == 0,
        "finite_initial_and_final_cost": finite_costs,
        "graph_iterations_at_least_one": iterations_pass,
        "final_cost_strictly_less_than_initial_cost": decrease_pass,
        "phase80_direct_no_pdc_base_miss_signal_bias_pd_tdcp_invariants": True,
        "c0d_telemetry_exact_equation_jacobian_units_sigma": True,
        "c0d_factor_count_positive": first["summary_diagnostics"]["clock_c0d"]["factor_count"] > 0 and second["summary_diagnostics"]["clock_c0d"]["factor_count"] > 0,
        "c0d_phone_exclusion_zero": first["summary_diagnostics"]["clock_c0d"]["clock_c0d_phone_exclusion_skips"] == 0 and second["summary_diagnostics"]["clock_c0d"]["clock_c0d_phone_exclusion_skips"] == 0,
        "c0d_legacy_scalar_between_zero": first["summary_diagnostics"]["clock_c0d"]["legacy_scalar_clock_between_factor_count"] == 0 and second["summary_diagnostics"]["clock_c0d"]["legacy_scalar_clock_between_factor_count"] == 0,
        "c0d_factor_plus_all_skips_equals_epochs_minus_one": all(case["summary_diagnostics"]["clock_c0d"]["factor_count"] + sum(case["summary_diagnostics"]["clock_c0d"][key] for key in ("clock_c0d_clock_jump_skips", "clock_c0d_gap_skips", "clock_c0d_invalid_dt_skips", "clock_c0d_phone_exclusion_skips")) == expected_rows for case in (first, second)),
        "c0d_dt_range_positive": first["summary_diagnostics"]["clock_c0d"]["dt_min_s"] > 0 and first["summary_diagnostics"]["clock_c0d"]["dt_min_s"] <= first["summary_diagnostics"]["clock_c0d"]["dt_max_s"] and second["summary_diagnostics"]["clock_c0d"]["dt_min_s"] > 0 and second["summary_diagnostics"]["clock_c0d"]["dt_min_s"] <= second["summary_diagnostics"]["clock_c0d"]["dt_max_s"],
        "candidate_flag_once_and_dependencies": first["flag_contract"] and second["flag_contract"],
    }
    gates["all_route_gates"] = all(gates.values())
    observations = {
        active_runs[0]: {"iterations": first_graph["iterations"], "initial_cost": first_graph["initial_cost"], "final_cost": first_graph["final_cost"], "finite_costs": finite_costs, "final_cost_less_than_initial": first_graph["final_cost"] < first_graph["initial_cost"]},
        active_runs[1]: {"iterations": second_graph["iterations"], "initial_cost": second_graph["initial_cost"], "final_cost": second_graph["final_cost"], "finite_costs": finite_costs, "final_cost_less_than_initial": second_graph["final_cost"] < second_graph["initial_cost"]},
    }
    report = {
        "candidate_run1": {key: value for key, value in first.items() if key not in ("summary", "summary_diagnostics", "prediction_keys")},
        "candidate_run2": {key: value for key, value in second.items() if key not in ("summary", "summary_diagnostics", "prediction_keys")},
        "prediction_domain": {"phase80_phase78_pinned_rows": expected_rows, "run1_rows": first["submission_artifact"]["rows"], "run2_rows": second["submission_artifact"]["rows"], "exact": gates["rows_equal_pinned_phase80_phase78_domain"]},
        "repeat_identity": repeat,
        "graph_materiality": {"run1": observations[active_runs[0]], "run2": observations[active_runs[1]]},
        "gates": gates,
    }
    return report, observations


def _failed_gate_details(route_reports: dict[str, Any], gates: dict[str, bool]) -> dict[str, Any]:
    names = [name for name in FIXED_GATE_NAMES if gates.get(name) is False]
    instances: list[str] = []
    for route in ROUTES:
        route_gates = route_reports[route]["gates"]
        for key, value in route_gates.items():
            if key != "all_route_gates" and value is False:
                instances.extend(f"{route}/candidate_run{run}:{key}" for run in (1, 2))
    return {"names": names, "instances": instances}


def run_reclassification(output_root: Path = OUTPUT_ROOT) -> dict[str, Any]:
    freeze = verify_freeze()
    output_root = output_root.resolve()
    reject_forbidden(output_root)
    if output_root.exists() and (not output_root.is_dir() or any(output_root.iterdir())):
        raise fail(f"refusing to overwrite nonempty Phase86 output: {output_root}")
    counters: dict[str, int] = {"phase86_freeze": 1, "phase86_manifest": 1, "phase85_retry_manifest": 0, "phase85_v2_aggregate": 0, "phase85_v2_output_manifest": 0, "candidate_submission": 0, "candidate_summary": 0}
    # The freeze and Phase86 manifest were read by verify_freeze.  Their
    # one-read counts are explicit above; all Phase85 artifacts below are read
    # exactly once, with hashing and parsing performed from those bytes.
    _phase85_retry_manifest(freeze, counters)
    _phase85_output_manifest(freeze, counters)
    aggregate, route_records = _verify_phase85_aggregate(freeze, counters)
    pins = _expected_manifest_pins(freeze)
    route_reports: dict[str, Any] = {}
    observations: dict[str, dict[str, Any]] = {}
    all_case_bytes: dict[str, bytes] = {}
    all_summary_bytes: dict[str, bytes] = {}
    for route in ROUTES:
        record = route_records[route]
        cases = record["cases"]
        first_pin, second_pin = pins[route]["candidate_run1"], pins[route]["candidate_run2"]
        # Read the immutable files once here.  The byte payloads are retained
        # solely to establish repeat identity and are never used as inputs to
        # any solver or accuracy computation.
        first_submission_bytes = _read_bytes(ROOT / first_pin["submission"]["path"], f"candidate submission {route}/run1", expected_sha256=first_pin["submission"]["sha256"], expected_bytes=first_pin["submission"]["bytes"], counters=counters, counter_key="candidate_submission")
        first_summary_bytes = _read_bytes(ROOT / first_pin["summary"]["path"], f"candidate summary {route}/run1", expected_sha256=first_pin["summary"]["sha256"], expected_bytes=first_pin["summary"]["bytes"], counters=counters, counter_key="candidate_summary")
        second_submission_bytes = _read_bytes(ROOT / second_pin["submission"]["path"], f"candidate submission {route}/run2", expected_sha256=second_pin["submission"]["sha256"], expected_bytes=second_pin["submission"]["bytes"], counters=counters, counter_key="candidate_submission")
        second_summary_bytes = _read_bytes(ROOT / second_pin["summary"]["path"], f"candidate summary {route}/run2", expected_sha256=second_pin["summary"]["sha256"], expected_bytes=second_pin["summary"]["bytes"], counters=counters, counter_key="candidate_summary")
        first_summary = _json_payload(first_summary_bytes, f"candidate summary {route}/run1")
        second_summary = _json_payload(second_summary_bytes, f"candidate summary {route}/run2")
        first_case = _artifact_case_from_payload(cases["candidate_run1"], first_pin, route, 1, DOMAIN_ROWS[route], first_submission_bytes, first_summary_bytes, first_summary)
        second_case = _artifact_case_from_payload(cases["candidate_run2"], second_pin, route, 2, DOMAIN_ROWS[route], second_submission_bytes, second_summary_bytes, second_summary)
        report, route_observations = _route_result(route, first_case, second_case, DOMAIN_ROWS[route], first_submission_bytes, second_submission_bytes, first_summary_bytes, second_summary_bytes)
        route_reports[route] = report
        observations.update(route_observations)
        all_case_bytes[f"{route}/candidate_run1"] = first_submission_bytes
        all_case_bytes[f"{route}/candidate_run2"] = second_submission_bytes
        all_summary_bytes[f"{route}/candidate_run1"] = first_summary_bytes
        all_summary_bytes[f"{route}/candidate_run2"] = second_summary_bytes

    freeze_result_hash_matches_verified_output = freeze.get("authority", {}).get("phase85_v2_result", {}).get("sha256") == PHASE85_RESULT_SHA256
    global_gates: dict[str, bool] = {
        "sealed_phase85_v2_artifact_hashes_exact": freeze_result_hash_matches_verified_output and counters["phase85_v2_aggregate"] == 1 and counters["phase85_v2_output_manifest"] == 1 and counters["candidate_submission"] == 8 and counters["candidate_summary"] == 8,
        "candidate_only_exact_four_routes": tuple(aggregate["routes"]) == ROUTES and aggregate["candidate"]["controls"] == 0 and aggregate["read_accounting"]["control_runs_per_route"] == 0,
        "rows_equal_pinned_phase80_phase78_domain": all(route_reports[route]["gates"]["rows_equal_pinned_phase80_phase78_domain"] for route in ROUTES),
        "rows_plus_one_equals_summary_epochs_output": all(route_reports[route]["gates"]["rows_plus_one_equals_summary_epochs_output"] for route in ROUTES),
        "repeat_submission_and_summary_byte_identical": all(route_reports[route]["gates"]["repeat_submission_and_summary_byte_identical"] for route in ROUTES),
        "converged": all(route_reports[route]["gates"]["converged"] for route in ROUTES),
        "finite_coordinates_and_earth_valid": all(route_reports[route]["gates"]["finite_coordinates_and_earth_valid"] for route in ROUTES),
        "speed_over_70_zero": all(route_reports[route]["gates"]["speed_over_70_zero"] for route in ROUTES),
        "finite_initial_and_final_cost": all(route_reports[route]["gates"]["finite_initial_and_final_cost"] for route in ROUTES),
        "graph_iterations_at_least_one": all(route_reports[route]["gates"]["graph_iterations_at_least_one"] for route in ROUTES),
        "final_cost_strictly_less_than_initial_cost": all(route_reports[route]["gates"]["final_cost_strictly_less_than_initial_cost"] for route in ROUTES),
        "phase80_direct_no_pdc_base_miss_signal_bias_pd_tdcp_invariants": all(route_reports[route]["gates"]["phase80_direct_no_pdc_base_miss_signal_bias_pd_tdcp_invariants"] for route in ROUTES),
        "c0d_telemetry_exact_equation_jacobian_units_sigma": all(route_reports[route]["gates"]["c0d_telemetry_exact_equation_jacobian_units_sigma"] for route in ROUTES),
        "c0d_factor_count_positive": all(route_reports[route]["gates"]["c0d_factor_count_positive"] for route in ROUTES),
        "c0d_phone_exclusion_zero": all(route_reports[route]["gates"]["c0d_phone_exclusion_zero"] for route in ROUTES),
        "c0d_legacy_scalar_between_zero": all(route_reports[route]["gates"]["c0d_legacy_scalar_between_zero"] for route in ROUTES),
        "c0d_factor_plus_all_skips_equals_epochs_minus_one": all(route_reports[route]["gates"]["c0d_factor_plus_all_skips_equals_epochs_minus_one"] for route in ROUTES),
        "c0d_dt_range_positive": all(route_reports[route]["gates"]["c0d_dt_range_positive"] for route in ROUTES),
        "candidate_flag_once_and_dependencies": all(route_reports[route]["gates"]["candidate_flag_once_and_dependencies"] for route in ROUTES),
        "truth_free": aggregate.get("truth_free") is True and aggregate.get("read_accounting", {}).get("truth_reads") == 0 and all(report["candidate_run1"]["candidate"] and report["candidate_run2"]["candidate"] for report in route_reports.values()),
        "accuracy_not_scored": aggregate.get("accuracy_scored") is False and aggregate.get("read_accounting", {}).get("accuracy_scored") is False,
    }
    global_gates["all_gates_anded"] = all(global_gates.values())
    failed = _failed_gate_details(route_reports, global_gates)
    status = "go-phase86-phase85-sealed-artifact-reclassification" if global_gates["all_gates_anded"] else "no-go-phase86-phase85-source-clock-c0d-active-solve-gates"
    result = {
        "schema_version": OUTPUT_SCHEMA,
        "phase": 86,
        "execution_label": "Luna Max",
        "status": status,
        "decision": "GO" if global_gates["all_gates_anded"] else "NO-GO: Phase85-v2 candidate artifacts are legacy converged-only diagnostics; promotion requires finite costs, graph iterations >= 1, and final_cost < initial_cost for every pinned candidate run.",
        "truth_free": True,
        "accuracy_scored": False,
        "promotion_authorized": global_gates["all_gates_anded"],
        "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
        "manifest": {"path": relative(MANIFEST), "sha256": _hash_file(MANIFEST, "Phase86 manifest")},
        "evaluator": {"path": relative(EVALUATOR), "sha256": _hash_file(EVALUATOR, "Phase86 evaluator")},
        "phase85_v2_legacy_diagnostic": {"path": relative(PHASE85_RESULT), "sha256": PHASE85_RESULT_SHA256, "freeze_declared_sha256": freeze["authority"]["phase85_v2_result"]["sha256"], "verified_sha256": PHASE85_RESULT_SHA256, "bytes": PHASE85_RESULT_BYTES, "status": aggregate["status"], "rewritten": False, "freeze_hash_matches_verified_output": freeze_result_hash_matches_verified_output},
        "phase85_v2_output_manifest": {"path": relative(PHASE85_OUTPUT_MANIFEST), "sha256": PHASE85_OUTPUT_MANIFEST_SHA256, "bytes": PHASE85_OUTPUT_MANIFEST_BYTES},
        "phase80_phase78_domain_rows": DOMAIN_ROWS,
        "routes": route_reports,
        "active_solve": {
            "required": {"finite_initial_and_final_cost": True, "graph_iterations_at_least_one": True, "final_cost_strictly_less_than_initial_cost": True},
            "runs_evaluated": len(observations),
            "finite_cost_runs": [key for key, value in observations.items() if value["finite_costs"]],
            "iterations_at_least_one_runs": [key for key, value in observations.items() if value["iterations"] >= 1],
            "strictly_decreasing_cost_runs": [key for key, value in observations.items() if value["final_cost_less_than_initial"]],
            "all_iterations_zero": all(value["iterations"] == 0 for value in observations.values()),
            "all_initial_cost_equals_final_cost": all(value["initial_cost"] == value["final_cost"] for value in observations.values()),
            "observations": observations,
        },
        "gates": {**global_gates, "all_passed": global_gates["all_gates_anded"]},
        "failed_gates": failed,
        "read_accounting": {
            "phase86_freeze_reads": counters["phase86_freeze"],
            "phase86_manifest_reads": counters["phase86_manifest"],
            "phase85_v2_aggregate_reads": counters["phase85_v2_aggregate"],
            "phase85_v2_output_manifest_reads": counters["phase85_v2_output_manifest"],
            "phase85_retry_manifest_reads": counters["phase85_retry_manifest"],
            "candidate_submission_reads": counters["candidate_submission"],
            "candidate_summary_reads": counters["candidate_summary"],
            "candidate_artifact_reads_total": counters["candidate_submission"] + counters["candidate_summary"],
            "phase85_candidate_artifact_reads": counters["candidate_submission"] + counters["candidate_summary"],
            "native_solver_invocations": 0,
            "native_reruns": 0,
            "raw_reads": 0,
            "raw_device_gnss_reads": 0,
            "raw_device_imu_reads": 0,
            "broadcast_navigation_reads": 0,
            "base_rinex_reads": 0,
            "truth_reads": 0,
            "mat_reads_or_generated": 0,
            "phase82_candidate_coordinate_reads": 0,
            "precomputed_coordinate_reads": 0,
            "kaggle_or_token_access": 0,
            "validation_holdout_reads": 0,
            "archive_reopens": 0,
            "accuracy_scored": False,
            "route_score_selection": False,
            "sealed_phase85_artifacts_only": True,
        },
    }
    output_root.mkdir(parents=True, exist_ok=True)
    result_path = output_root / "phase86_phase85_source_clock_c0d_reclassification_result.json"
    atomic_json(result_path, result)
    output_manifest = {
        "schema_version": OUTPUT_MANIFEST_SCHEMA,
        "phase": 86,
        "status": status,
        "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
        "evaluator": {"path": relative(EVALUATOR), "sha256": result["evaluator"]["sha256"]},
        "phase85_retry_manifest": {"path": relative(PHASE85_RETRY_MANIFEST), "sha256": PHASE85_RETRY_MANIFEST_SHA256, "bytes": PHASE85_RETRY_MANIFEST_BYTES},
        "phase85_v2_aggregate": {"path": relative(PHASE85_RESULT), "sha256": PHASE85_RESULT_SHA256, "freeze_declared_sha256": freeze["authority"]["phase85_v2_result"]["sha256"], "verified_sha256": PHASE85_RESULT_SHA256, "bytes": PHASE85_RESULT_BYTES, "freeze_hash_matches_verified_output": freeze_result_hash_matches_verified_output},
        "phase85_v2_output_manifest": {"path": relative(PHASE85_OUTPUT_MANIFEST), "sha256": PHASE85_OUTPUT_MANIFEST_SHA256, "bytes": PHASE85_OUTPUT_MANIFEST_BYTES},
        "result": {"path": relative(result_path), "sha256": _hash_file(result_path, "Phase86 result"), "bytes": result_path.stat().st_size},
        "candidate_runs": 8,
        "control_runs": 0,
        "native_reruns": 0,
        "truth_reads": 0,
        "accuracy_scored": False,
        "all_gates_passed": global_gates["all_gates_anded"],
        "failed_gates": failed,
        "read_accounting": result["read_accounting"],
    }
    atomic_json(output_root / "phase86_phase85_source_clock_c0d_reclassification_manifest.json", output_manifest)
    return result


def _artifact_case_from_payload(case: dict[str, Any], pin: dict[str, Any], route: str, run: int, expected_rows: int, submission_payload: bytes, summary_payload: bytes, summary: dict[str, Any]) -> dict[str, Any]:
    _check_case_claim(case, pin, route, run)
    rows = _read_submission(submission_payload, route)
    if len(rows) != expected_rows:
        raise fail(f"candidate row count changed: {route}/run{run}")
    speed = _speed_report(rows)
    diagnostics = _validate_summary(summary, route, expected_rows)
    if case.get("summary_payload") != summary or case.get("submission_artifact", {}).get("rows") != len(rows):
        raise fail(f"Phase85 aggregate artifact claim changed: {route}/run{run}")
    return {"run": run, "candidate": True, "submission_artifact": {"path": pin["submission"]["path"], "bytes": len(submission_payload), "sha256": pin["submission"]["sha256"], "rows": len(rows)}, "summary_artifact": {"path": pin["summary"]["path"], "bytes": len(summary_payload), "sha256": pin["summary"]["sha256"]}, "prediction_keys": [row[0] for row in rows], "speed": speed, "summary": summary, "summary_diagnostics": diagnostics, "flag_contract": _check_command(case, route, run)}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--run-reclassification", action="store_true")
    parser.add_argument("--output-root", type=Path, default=OUTPUT_ROOT)
    args = parser.parse_args(argv)
    try:
        if args.verify_freeze:
            verify_freeze()
        if args.run_reclassification:
            result = run_reclassification(args.output_root)
            print(json.dumps({"status": result["status"], "all_passed": result["gates"]["all_passed"], "failed_gates": result["failed_gates"]["names"], "candidate_summary_reads": result["read_accounting"]["candidate_summary_reads"], "candidate_submission_reads": result["read_accounting"]["candidate_submission_reads"], "native_reruns": result["read_accounting"]["native_reruns"], "truth_reads": result["read_accounting"]["truth_reads"]}, sort_keys=True))
        elif not args.verify_freeze:
            parser.error("one of --verify-freeze or --run-reclassification is required")
        return 0
    except Phase86Error as exc:
        print(f"phase86 sealed-artifact reclassification failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
