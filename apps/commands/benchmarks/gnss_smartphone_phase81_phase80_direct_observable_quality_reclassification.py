#!/usr/bin/env python3
"""Truth-free reclassification of the immutable Phase80 v2 artifacts.

The Phase80 v2 native matrix is retained as a fail-closed diagnostic because
its evaluator compared CSV rows with the pre-warmup solver-state count.  This
sealed-artifact evaluator independently rereads only the pinned Phase80
failure, its eight candidate CSV/summary artifacts, and the Phase78 result's
route row counts.  It never invokes native code and never opens raw, truth,
MAT, validation, or Kaggle data.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase81_phase80_direct_observable_quality_reclassification_freeze_v1.json"
FREEZE_SHA256 = "c648ec5f49f9023239360a70a1d77ee4b546e2b4d6f07cdf076f00694a2aeb6f"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase81_phase80_direct_observable_quality_reclassification_manifest_v1.json"
EVALUATOR = Path(__file__).resolve()
V2_FAILURE = ROOT / "output/smartphone-r5/phase80-source-exact-direct-observable-quality-structural-v2/phase80_direct_observable_quality_structural_failure.json"
V2_FAILURE_SHA256 = "767ef5f7089e535063ec6cbe4e9dc3e6d061bdab62eb521f794c3fab0b171903"
V2_FAILURE_BYTES = 600598
PHASE78_RESULT = ROOT / "output/smartphone-r5/phase78-phase77-structural-reclassification-v1/phase78_phase77_structural_reclassification_result.json"
PHASE78_RESULT_SHA256 = "d3d190d8da951afc0a62abcf82d87498a6b8ff1da68cb2154966ca387e357114"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase81-phase80-direct-observable-quality-reclassification-v1"
OUTPUT_SCHEMA = "smartphone-r5-phase81-phase80-direct-observable-quality-reclassification-result.v1"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
DIRECT_SETTINGS = {
    ROUTES[0]: ("Highway", 0.2, 0.8),
    ROUTES[1]: ("Street", 0.1, 0.4),
    ROUTES[2]: ("Highway", 0.2, 0.8),
    ROUTES[3]: ("Street", 0.1, 0.4),
}


class Phase81Error(ValueError):
    """Raised when an immutable Phase80 artifact fails reclassification."""


def fail(message: str) -> Phase81Error:
    return Phase81Error(message)


def reject_forbidden(path: Path | str) -> None:
    token = str(path).lower()
    if token.endswith(".mat") or ".mat/" in token or ".mat\\" in token:
        raise fail(f"MAT path is forbidden: {path}")
    if any(term in token for term in ("ground_truth", "validation", "holdout", "kaggle", "token")):
        raise fail(f"forbidden path: {path}")


def sha256(path: Path) -> str:
    reject_forbidden(path)
    if not path.is_file():
        raise fail(f"missing artifact: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path, label: str) -> dict[str, Any]:
    reject_forbidden(path)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise fail(f"invalid {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object: {path}")
    return value


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    reject_forbidden(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _finite(value: Any, label: str) -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise fail(f"nonfinite value: {label}")
    if isinstance(value, dict):
        for key, item in value.items():
            _finite(item, f"{label}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _finite(item, f"{label}[{index}]")


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise fail(f"missing/nonfinite number: {label}")
    return float(value)


def _count(mapping: dict[str, Any], key: str, label: str, minimum: int = 0) -> int:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise fail(f"invalid count: {label}/{key}")
    return value


def _read_prediction(path: Path, route: str) -> list[tuple[int, float, float]]:
    reject_forbidden(path)
    lines = path.read_text(encoding="utf-8-sig").splitlines()
    if not lines or lines[0] != "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees":
        raise fail(f"submission header mismatch: {path}")
    rows: list[tuple[int, float, float]] = []
    previous: int | None = None
    for line_number, line in enumerate(lines[1:], start=2):
        fields = line.split(",")
        if len(fields) != 4 or fields[0] != route:
            raise fail(f"submission key mismatch: {path}:{line_number}")
        try:
            timestamp = int(fields[1])
            latitude = float(fields[2])
            longitude = float(fields[3])
        except ValueError as exc:
            raise fail(f"non-numeric submission row: {path}:{line_number}") from exc
        if previous is not None and timestamp <= previous:
            raise fail(f"submission timestamps are not increasing: {path}")
        if not all(math.isfinite(item) for item in (latitude, longitude)) or not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise fail(f"invalid earth coordinate: {path}:{line_number}")
        rows.append((timestamp, latitude, longitude))
        previous = timestamp
    if not rows:
        raise fail(f"empty submission: {path}")
    return rows


def _speed_report(rows: list[tuple[int, float, float]]) -> dict[str, Any]:
    radius = 6_371_000.0
    speeds: list[float] = []
    for previous, current in zip(rows, rows[1:]):
        dt = (current[0] - previous[0]) / 1000.0
        if dt <= 0.0:
            raise fail("non-positive submission interval")
        lat1, lon1, lat2, lon2 = map(math.radians, (previous[1], previous[2], current[1], current[2]))
        dlat, dlon = lat2 - lat1, lon2 - lon1
        hav = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
        distance = radius * 2.0 * math.asin(math.sqrt(min(1.0, max(0.0, hav))))
        speeds.append(distance / dt)
    return {
        "finite": all(math.isfinite(speed) for speed in speeds),
        "max_mps": max(speeds, default=0.0),
        "over_70_mps_count": sum(speed > 70.0 for speed in speeds),
        "count": len(speeds),
    }


def _artifact(path: Path, pin: dict[str, Any], label: str) -> dict[str, Any]:
    actual = sha256(path)
    if actual != pin.get("sha256") or path.stat().st_size != pin.get("bytes"):
        raise fail(f"{label} hash/bytes changed")
    return {"path": relative(path), "bytes": path.stat().st_size, "sha256": actual}


def _phase78_rows() -> dict[str, int]:
    result = load_json(PHASE78_RESULT, "Phase78 result")
    rows: dict[str, int] = {}
    for route in ROUTES:
        domain = result.get("routes", {}).get(route, {}).get("prediction_domain", {})
        value = domain.get("rows")
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise fail(f"Phase78 route row count missing: {route}")
        rows[route] = value
    return rows


def verify_freeze() -> dict[str, Any]:
    if sha256(FREEZE) != FREEZE_SHA256:
        raise fail("Phase81 freeze hash changed")
    freeze = load_json(FREEZE, "Phase81 freeze")
    if freeze.get("schema_version") != "smartphone-r5-phase81-phase80-direct-observable-quality-reclassification-freeze.v1" or freeze.get("status") != "frozen-before-phase81-sealed-artifact-read":
        raise fail("Phase81 freeze schema/status changed")
    if tuple(freeze.get("scope", {}).get("routes", ())) != ROUTES:
        raise fail("Phase81 route order changed")
    scope = freeze.get("scope", {})
    if any(scope.get(key) is not True for key in ("no_native_rerun", "no_raw_input_read", "no_accuracy_truth", "no_mat", "no_kaggle", "candidate_csvs_are_current_artifacts_only", "phase78_input_is_route_row_counts_only")):
        raise fail("Phase81 read scope changed")
    authority = freeze.get("authority", {})
    failure_pin = authority.get("phase80_v2_failure", {})
    if failure_pin.get("sha256") != V2_FAILURE_SHA256 or failure_pin.get("bytes") != V2_FAILURE_BYTES or sha256(V2_FAILURE) != V2_FAILURE_SHA256 or V2_FAILURE.stat().st_size != V2_FAILURE_BYTES:
        raise fail("Phase80 v2 failure pin changed")
    phase78_pin = authority.get("phase78_result", {})
    if phase78_pin.get("sha256") != PHASE78_RESULT_SHA256 or sha256(PHASE78_RESULT) != PHASE78_RESULT_SHA256:
        raise fail("Phase78 result pin changed")
    expected_rows = _phase78_rows()
    if phase78_pin.get("route_rows") != expected_rows:
        raise fail("Phase78 route row-count pin changed")
    pins = freeze.get("candidate_artifact_pins", {})
    if tuple(pins) != ROUTES:
        raise fail("Phase81 candidate artifact routes changed")
    for route in ROUTES:
        if set(pins[route]) != {"candidate_run1", "candidate_run2"}:
            raise fail(f"Phase81 candidate run pins incomplete: {route}")
        for run in ("candidate_run1", "candidate_run2"):
            for kind in ("submission", "summary"):
                item = pins[route][run].get(kind, {})
                if len(item.get("sha256", "")) != 64 or not isinstance(item.get("bytes"), int) or not item.get("path"):
                    raise fail(f"Phase81 artifact pin malformed: {route}/{run}/{kind}")
    gates = freeze.get("fixed_reclassification_gates", {})
    if gates.get("candidate_prediction_domain_coverage") != 1.0 or any(gates.get(key) is not True for key in ("legacy_failure_is_immutable_diagnostic", "legacy_only_false_gate_is_prediction_domain", "candidate_artifact_hashes_exact", "candidate_repeat_submission_and_summary_hash_identity", "rows_plus_warmup_equals_summary_output_epochs", "candidate_coordinates_finite_and_earth_valid", "candidate_speed_finite_and_over_70_zero", "candidate_converged", "direct_quality_object_and_counts", "direct_no_pdc", "base_phase78_frequency_policy", "miss_mask_contract", "signal_bias_material_and_finite", "tdcp_built_equals_inserted_and_finite", "raw_utc_warmup_contract", "truth_free", "all_gates_anded")):
        raise fail("Phase81 gate declaration changed")
    accounting = freeze.get("read_accounting_at_freeze", {})
    if any(accounting.get(key) != 0 for key in ("native_solver_invocations", "raw_device_gnss", "raw_device_imu", "broadcast_nav", "base_rinex", "precomputed_coordinates", "truth", "mat", "validation_holdout", "kaggle", "token", "archive_reopens")):
        raise fail("Phase81 zero-read declaration changed")
    if not MANIFEST.is_file():
        raise fail("Phase81 manifest is missing")
    manifest = load_json(MANIFEST, "Phase81 manifest")
    if manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256 or manifest.get("routes") != list(ROUTES):
        raise fail("Phase81 manifest freeze/routes pin changed")
    if manifest.get("phase80_v2_failure", {}).get("sha256") != V2_FAILURE_SHA256 or manifest.get("phase78_result", {}).get("sha256") != PHASE78_RESULT_SHA256:
        raise fail("Phase81 manifest source pin changed")
    if manifest.get("evaluator", {}).get("path") != relative(EVALUATOR) or manifest.get("evaluator", {}).get("sha256") != sha256(EVALUATOR):
        raise fail("Phase81 evaluator manifest pin changed")
    return freeze


def _legacy_diagnostic(failure: dict[str, Any], freeze: dict[str, Any]) -> dict[str, Any]:
    if failure.get("status") != "fail-closed" or failure.get("errors") != []:
        raise fail("Phase80 v2 failure is not the pinned clean diagnostic")
    if failure.get("native_solver_invocations_started") != 8 or failure.get("truth_reads") != 0:
        raise fail("Phase80 v2 invocation/truth accounting changed")
    if any(failure.get(key) != 8 for key in ("raw_device_gnss_process_reads", "raw_device_imu_process_reads", "broadcast_nav_process_reads", "base_rinex_process_reads")):
        raise fail("Phase80 v2 process-read accounting changed")
    authority = freeze["authority"]["phase80_v2_failure"]
    if failure.get("evaluator") != authority.get("legacy_evaluator"):
        raise fail("Phase80 legacy evaluator provenance changed")
    diagnostic_routes: dict[str, Any] = {}
    for route in ROUTES:
        record = failure.get("routes", {}).get(route)
        if not isinstance(record, dict):
            raise fail(f"Phase80 v2 route missing: {route}")
        gates = record.get("gates")
        if not isinstance(gates, dict) or gates.get("all_route_gates") is not False or gates.get("prediction_domain_coverage_exact") is not False:
            raise fail(f"Phase80 v2 legacy aggregate/domain diagnostic changed: {route}")
        leaf = {key: value for key, value in gates.items() if key not in ("all_route_gates", "prediction_domain_coverage_exact")}
        if set(key for key, value in leaf.items() if value is False) != set() or not all(value is True for value in leaf.values()):
            raise fail(f"Phase80 v2 has a false non-domain gate: {route}")
        diagnostic_routes[route] = {"legacy_false_leaf_gate": "prediction_domain_coverage_exact", "other_leaf_gates_all_true": True}
    return {"errors": [], "native_solver_invocations_started": 8, "routes": diagnostic_routes}


def _validate_summary(summary: dict[str, Any], route: str, expected: dict[str, Any], expected_rows: int) -> dict[str, Any]:
    _finite(summary, f"summary[{route}]")
    if summary.get("dataset_id") != route or summary.get("truth_used") is not False:
        raise fail(f"summary identity/truth gate failed: {route}")
    if summary.get("native_source_direct_observable_quality_enabled") is not True or summary.get("native_upstream_quality") is not False:
        raise fail(f"direct quality top-level identity failed: {route}")
    if summary.get("native_pdc_state_bridge") is not False or summary.get("native_pdc_imu_tdcp_no_bridge") is not True:
        raise fail(f"PDC bridge identity failed: {route}")

    base = summary.get("native_base_pseudorange_compensation")
    miss = summary.get("native_base_pseudorange_source_miss_mask")
    if not isinstance(base, dict) or not isinstance(miss, dict):
        raise fail(f"base/miss-mask telemetry missing: {route}")
    if any(base.get(key) is not True for key in ("enabled", "built", "applied", "same_satellite_signal_only", "no_extrapolation_or_endpoint_hold")):
        raise fail(f"base scope failed: {route}")
    if base.get("preserve_additional_frequency_bands") is not False or any(base.get(key) is not False for key in ("spp_applied", "tdcp_applied", "doppler_applied")):
        raise fail(f"base Phase78 frequency policy failed: {route}")
    if base.get("base_member_sha256") != base.get("base_rinex_sha256") or not isinstance(base.get("base_rinex_bytes"), int) or base.get("base_rinex_bytes", 0) <= 0:
        raise fail(f"base artifact identity failed: {route}")
    if miss.get("enabled") is not True or miss.get("retained_factor_epoch_indices_unchanged") is not True or miss.get("tdcp_doppler_imu_spp_unchanged") is not True or miss.get("no_extrapolation_or_endpoint_hold") is not True:
        raise fail(f"miss-mask scope failed: {route}")
    original = _count(miss, "original_adopted_pseudorange_rows", route, 1)
    retained = _count(miss, "retained_finite_pc_pseudorange_rows", route, 1)
    drops = [_count(miss, key, route) for key in ("dropped_missing_exact_stream_rows", "dropped_out_of_domain_rows", "dropped_nonfinite_correction_rows")]
    if original != retained + sum(drops) or miss.get("retained_finite_pc_fraction") != 1.0 or miss.get("pseudorange_factors_inserted") != retained or miss.get("pseudorange_factor_count_consistent") is not True:
        raise fail(f"miss-mask accounting failed: {route}")
    if base.get("adopted_pseudorange_rows") != original or base.get("adopted_rows_corrected") != retained:
        raise fail(f"base/miss count identity failed: {route}")

    epochs = summary.get("epochs")
    graph = summary.get("graph")
    tdcp = summary.get("tdcp_contract")
    upstream = summary.get("upstream_observable_quality")
    direct = summary.get("native_source_direct_observable_quality")
    if not all(isinstance(item, dict) for item in (epochs, graph, tdcp, upstream, direct)):
        raise fail(f"quality/population telemetry missing: {route}")
    assert isinstance(epochs, dict) and isinstance(graph, dict) and isinstance(tdcp, dict) and isinstance(upstream, dict) and isinstance(direct, dict)
    env, p_sigma, d_sigma = DIRECT_SETTINGS[route]
    if any(direct.get(key) != value for key, value in (("enabled", True), ("direct_no_pdc", True), ("pdc_bridge", False), ("native_pdc_state_bridge", False), ("environment", env))):
        raise fail(f"direct object/PDC contract failed: {route}")
    huber = direct.get("p_and_d_huber")
    if not isinstance(huber, dict) or huber.get("pseudorange_sigma") != p_sigma or huber.get("doppler_sigma") != d_sigma:
        raise fail(f"direct Huber mapping failed: {route}")
    expected_config = {"use_upstream_observable_quality": True, "upstream_snr_percentile": 85.0, "upstream_min_snr_dbhz": 20.0, "upstream_min_elevation_deg": 5.0, "upstream_max_adjacent_gap_s": 1.5, "pseudorange_huber_threshold_sigma": p_sigma, "undifferenced_doppler_huber_threshold_sigma": d_sigma, "tdcp_sigma_m_unchanged": 0.03, "pseudorange_sigma_contract": "snr_scale*signal_type_factor", "doppler_sigma_contract": "snr_scale/12", "adjacent_mask_contract": "applyAdjacentMasks Pmask_dDP/Lmask_dDL unchanged", "pseudorange_residual_screen": "Pmask_res L1=20m/L5=15m", "doppler_residual_screen": "Dmask_res 3m/s; non-initializer"}
    if direct.get("config") != expected_config:
        raise fail(f"direct quality config failed: {route}")
    for key, value in (("enabled", True), ("snr_percentile", 85.0), ("snr_denominator_db", 20.0), ("min_snr_dbhz", 20.0), ("min_elevation_deg", 5.0), ("max_adjacent_gap_s", 1.5), ("tdcp_sigma_m_unchanged", 0.03), ("pseudorange_sigma_contract", "snr_scale*signal_type_factor"), ("doppler_sigma_contract", "snr_scale/12")):
        if upstream.get(key) != value:
            raise fail(f"upstream quality contract failed: {route}/{key}")
    counts = direct.get("counts")
    if not isinstance(counts, dict):
        raise fail(f"direct quality counts missing: {route}")
    count_keys = ("pseudorange_candidates", "pseudorange_factors", "doppler_candidates", "doppler_factors", "doppler_graph_factors", "pseudorange_residual_rejections", "doppler_residual_rejections")
    for key in count_keys:
        value = _count(counts, key, route)
        if value != _count(upstream, key, route):
            raise fail(f"direct/upstream count mismatch: {route}/{key}")
    if counts["pseudorange_candidates"] < counts["pseudorange_factors"] or counts["doppler_candidates"] < counts["doppler_factors"] or counts["pseudorange_factors"] < 1 or counts["doppler_graph_factors"] < 1:
        raise fail(f"direct quality count materiality failed: {route}")

    if graph.get("converged") is not True or graph.get("imu_intervals") != expected["imu_intervals"] or epochs.get("output") != expected["output_epochs"] or epochs.get("pseudorange_factors") != expected["pseudorange_factors"] or upstream.get("doppler_graph_factors") != expected["undifferenced_doppler_factors_inserted"]:
        raise fail(f"convergence/population contract failed: {route}")
    if tdcp.get("factors_built") != expected["tdcp_factors_built"] or tdcp.get("factors_inserted") != expected["tdcp_factors_inserted"] or tdcp.get("factors_built") != tdcp.get("factors_inserted") or tdcp.get("nonfinite_residuals") != 0:
        raise fail(f"TDCP contract failed: {route}")
    states = _count(epochs, "receiver_signal_bias_states", route, 1)
    factors = _count(epochs, "receiver_signal_bias_factors", route, 1)
    estimates = summary.get("receiver_signal_bias_estimates_m")
    if factors < states or not isinstance(estimates, dict) or len(estimates) != states:
        raise fail(f"signal-bias materiality failed: {route}")
    for key, value in estimates.items():
        _number(value, f"signal-bias/{route}/{key}")
    utc = summary.get("raw_utc_key_contract")
    if not isinstance(utc, dict) or utc.get("warmup_epoch_excluded") is not True or utc.get("raw_epoch_keys") != epochs.get("output") or utc.get("target_epochs") != expected_rows or utc.get("exact_solution_epochs") != expected_rows or utc.get("interpolated_epochs") != 0 or utc.get("edge_hold_epochs") != 0 or utc.get("unresolved_epochs") != 0:
        raise fail(f"raw UTC warmup/domain contract failed: {route}")
    return {
        "population": {"imu_intervals": graph["imu_intervals"], "output_epochs": epochs["output"], "pseudorange_factors": epochs["pseudorange_factors"], "tdcp_factors_built": tdcp["factors_built"], "tdcp_factors_inserted": tdcp["factors_inserted"], "undifferenced_doppler_factors_inserted": upstream["doppler_graph_factors"]},
        "direct_quality": {"environment": env, "p_and_d_huber": huber, "counts": counts},
        "signal_bias": {"states": states, "factors": factors, "estimates_m": estimates},
        "miss_mask": {key: miss.get(key) for key in ("original_adopted_pseudorange_rows", "retained_finite_pc_pseudorange_rows", "dropped_missing_exact_stream_rows", "dropped_out_of_domain_rows", "dropped_nonfinite_correction_rows", "retained_finite_pc_fraction", "pseudorange_factors_inserted")},
        "raw_utc_key_contract": {key: utc.get(key) for key in ("warmup_epoch_excluded", "raw_epoch_keys", "target_epochs", "exact_solution_epochs", "interpolated_epochs", "edge_hold_epochs", "unresolved_epochs")},
    }


def _case(failure_case: dict[str, Any], pins: dict[str, Any], route: str, run: int, expected_rows: int) -> dict[str, Any]:
    case_name = f"candidate_run{run}"
    if failure_case.get("candidate") is not True or failure_case.get("return_code") != 0:
        raise fail(f"Phase80 candidate case execution changed: {route}/{run}")
    pin = pins[case_name]
    submission = ROOT / pin["submission"]["path"]
    summary_path = ROOT / pin["summary"]["path"]
    if failure_case.get("submission_artifact", {}).get("sha256") != pin["submission"]["sha256"] or failure_case.get("summary_artifact", {}).get("sha256") != pin["summary"]["sha256"]:
        raise fail(f"Phase80 failure artifact claim changed: {route}/{run}")
    submission_artifact = _artifact(submission, pin["submission"], f"submission {route}/run{run}")
    summary_artifact = _artifact(summary_path, pin["summary"], f"summary {route}/run{run}")
    rows = _read_prediction(submission, route)
    speed = _speed_report(rows)
    if len(rows) != expected_rows or not speed["finite"] or speed["over_70_mps_count"] != 0:
        raise fail(f"prediction row/finite/speed gate failed: {route}/run{run}")
    summary = load_json(summary_path, f"Phase80 candidate summary {route}/run{run}")
    expected = failure_case.get("population", {})
    contract = _validate_summary(summary, route, expected, expected_rows)
    if len(rows) + 1 != contract["population"]["output_epochs"]:
        raise fail(f"rows plus warmup does not equal output epochs: {route}/run{run}")
    return {"run": run, "submission_artifact": {**submission_artifact, "rows": len(rows)}, "summary_artifact": summary_artifact, "prediction_keys": [row[0] for row in rows], "speed": speed, **contract}


def _public(value: dict[str, Any]) -> dict[str, Any]:
    return {key: item for key, item in value.items() if key != "prediction_keys"}


def run_reclassification(output_root: Path = OUTPUT_ROOT) -> dict[str, Any]:
    freeze = verify_freeze()
    output_root = output_root.resolve()
    reject_forbidden(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty Phase81 output: {output_root}")
    failure = load_json(V2_FAILURE, "Phase80 v2 failure")
    expected_rows = _phase78_rows()
    legacy = _legacy_diagnostic(failure, freeze)
    output_root.mkdir(parents=True, exist_ok=True)
    route_records: dict[str, Any] = {}
    for route in ROUTES:
        record = failure["routes"][route]
        first = _case(record["candidate_run1"], freeze["candidate_artifact_pins"][route], route, 1, expected_rows[route])
        second = _case(record["candidate_run2"], freeze["candidate_artifact_pins"][route], route, 2, expected_rows[route])
        repeat = first["submission_artifact"]["sha256"] == second["submission_artifact"]["sha256"] and first["summary_artifact"]["sha256"] == second["summary_artifact"]["sha256"] and first["prediction_keys"] == second["prediction_keys"] and first["population"] == second["population"] and first["direct_quality"] == second["direct_quality"] and first["miss_mask"] == second["miss_mask"] and first["signal_bias"] == second["signal_bias"]
        if not repeat:
            raise fail(f"candidate repeat identity failed: {route}")
        gates = {"candidate_artifact_hashes_exact": True, "candidate_repeat_submission_and_summary_hash_identity": True, "candidate_prediction_domain_coverage": first["submission_artifact"]["rows"] == expected_rows[route] and second["submission_artifact"]["rows"] == expected_rows[route] and first["prediction_keys"] == second["prediction_keys"], "rows_plus_warmup_equals_summary_output_epochs": first["submission_artifact"]["rows"] + 1 == first["population"]["output_epochs"] and second["submission_artifact"]["rows"] + 1 == second["population"]["output_epochs"], "candidate_coordinates_finite_and_earth_valid": True, "candidate_speed_finite_and_over_70_zero": first["speed"]["finite"] and second["speed"]["finite"] and first["speed"]["over_70_mps_count"] == 0 and second["speed"]["over_70_mps_count"] == 0, "candidate_converged": True, "direct_quality_object_and_counts": True, "direct_no_pdc": True, "base_phase78_frequency_policy": True, "miss_mask_contract": True, "signal_bias_material_and_finite": True, "tdcp_built_equals_inserted_and_finite": True, "raw_utc_warmup_contract": True}
        gates["all_route_gates"] = all(gates.values())
        if not gates["all_route_gates"]:
            raise fail(f"Phase81 route gates failed: {route}")
        route_records[route] = {"candidate_run1": _public(first), "candidate_run2": _public(second), "prediction_domain": {"phase78_pinned_rows": expected_rows[route], "run1_rows": first["submission_artifact"]["rows"], "run2_rows": second["submission_artifact"]["rows"], "exact": True}, "repeat_identity": True, "gates": gates}
    result = {"schema_version": OUTPUT_SCHEMA, "phase": 81, "execution_label": "Luna Max", "status": "go-phase81-phase80-sealed-artifact-reclassification", "decision": "All Phase80 direct-quality structural contracts pass on immutable v2 artifacts after correcting the warmup row-count interpretation; the original Phase80 v2 failure remains unchanged.", "truth_free": True, "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "manifest": {"path": relative(MANIFEST), "sha256": sha256(MANIFEST)}, "evaluator": {"path": relative(EVALUATOR), "sha256": sha256(EVALUATOR)}, "phase80_v2_failure": {"path": relative(V2_FAILURE), "sha256": V2_FAILURE_SHA256, "bytes": V2_FAILURE_BYTES}, "phase78_result_row_counts": {"path": relative(PHASE78_RESULT), "sha256": PHASE78_RESULT_SHA256, "rows": expected_rows}, "legacy_phase80_diagnostic": legacy, "routes": route_records, "gates": {"all_four_routes": True, "legacy_only_false_gate_was_prediction_domain": True, "candidate_artifact_hashes_exact": True, "candidate_repeat_identity": True, "candidate_prediction_domain_coverage": 1.0, "rows_plus_warmup_equals_summary_output_epochs": True, "candidate_coordinates_finite_and_earth_valid": True, "candidate_speed_finite_and_over_70_zero": True, "candidate_converged": True, "direct_quality_object_and_counts": True, "direct_no_pdc": True, "base_phase78_frequency_policy": True, "miss_mask_contract": True, "signal_bias_material_and_finite": True, "tdcp_built_equals_inserted_and_finite": True, "raw_utc_warmup_contract": True, "truth_free": True, "all_passed": True}, "read_accounting": {"native_solver_invocations": 0, "raw_device_gnss": 0, "raw_device_imu": 0, "broadcast_nav": 0, "base_rinex": 0, "precomputed_coordinates": 0, "truth": 0, "mat": 0, "validation_holdout": 0, "kaggle": 0, "token": 0, "archive_reopens": 0}}
    result_path = output_root / "phase81_phase80_direct_observable_quality_reclassification_result.json"
    atomic_json(result_path, result)
    atomic_json(output_root / "phase81_phase80_direct_observable_quality_reclassification_manifest.json", {"schema_version": "smartphone-r5-phase81-phase80-direct-observable-quality-reclassification-output-manifest.v1", "phase": 81, "status": "sealed-truth-free-artifact-reclassification", "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "evaluator": {"path": relative(EVALUATOR), "sha256": sha256(EVALUATOR)}, "phase80_v2_failure": {"path": relative(V2_FAILURE), "sha256": V2_FAILURE_SHA256, "bytes": V2_FAILURE_BYTES}, "result": {"path": relative(result_path), "sha256": sha256(result_path), "bytes": result_path.stat().st_size}, "native_solver_invocations": 0, "truth_reads": 0, "all_gates_passed": True})
    return result


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
            print(json.dumps({"status": result["status"], "all_passed": result["gates"]["all_passed"], "native_solver_invocations": 0, "truth_reads": 0}, sort_keys=True))
        elif not args.verify_freeze:
            parser.error("one of --verify-freeze or --run-reclassification is required")
        return 0
    except Phase81Error as exc:
        print(f"phase81 structural reclassification failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
