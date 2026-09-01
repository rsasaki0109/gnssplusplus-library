#!/usr/bin/env python3
"""Score immutable Phase73 outputs against the four Phase44 truths.

This is a scorer-only Phase74 process.  It never launches native code or a
solver and never opens raw GNSS/IMU/navigation inputs.  Each Phase73
candidate/control submission and summary is read once, and each declared
development truth is read once.  Phase51 values are sealed historical context
only, never a scoring or tuning input.
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
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase74_phase73_miss_mask_accuracy_freeze_v1.json"
FREEZE_SHA256 = "8308f15a491d1ef1daae0542efd1d7ac96562738e51e6c90ea04fd9f81aba8d9"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase74_phase73_miss_mask_accuracy_manifest_v1.json"
EVALUATOR = Path(__file__).resolve()
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase74-phase73-miss-mask-accuracy-v1"
OUTPUT_SCHEMA = "smartphone-r5-phase74-phase73-source-exact-miss-mask-accuracy-result.v1"
EARTH_RADIUS_M = 6_371_008.8
MAX_SPEED_MPS = 70.0


class Phase74AccuracyError(ValueError):
    """Raised when the frozen Phase74 scoring contract fails."""


def fail(message: str) -> Phase74AccuracyError:
    return Phase74AccuracyError(message)


def reject_forbidden(path: Path | str) -> None:
    token = str(path).lower()
    if token.endswith(".mat") or ".mat/" in token or ".mat\\" in token:
        raise fail(f"MAT path is forbidden: {path}")
    for term in ("validation", "holdout", "kaggle", "token", "device_wls", "svposition", "svelevation"):
        if term in token:
            raise fail(f"forbidden accuracy path: {path}")


def sha256_file(path: Path) -> str:
    reject_forbidden(path)
    if not path.is_file():
        raise fail(f"missing file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise fail(f"failed to hash {path}: {exc}") from exc
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


def atomic_write(path: Path, payload: bytes) -> None:
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
    atomic_write(path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode())


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def _read_once(path: Path, expected_sha256: str, expected_bytes: int, label: str) -> bytes:
    reject_forbidden(path)
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    try:
        with path.open("rb") as handle:
            payload = handle.read()
    except OSError as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    if len(payload) != expected_bytes:
        raise fail(f"{label} byte-size mismatch: {path}")
    if hashlib.sha256(payload).hexdigest() != expected_sha256:
        raise fail(f"{label} SHA-256 mismatch: {path}")
    return payload


def _require_freeze_route_map(freeze: dict[str, Any]) -> tuple[str, ...]:
    routes = tuple(freeze.get("cohort", {}).get("route_order", ()))
    expected = (
        "2021-03-16-18-59-us-ca-mtv-a/pixel5",
        "2021-08-24-20-32-us-ca-mtv-h/pixel5",
        "2022-04-01-18-22-us-ca-lax-t/pixel5",
        "2023-03-08-21-34-us-ca-mtv-u/pixel5",
    )
    if routes != expected:
        raise fail("Phase74 route order changed")
    if set(freeze.get("cohort", {}).get("truths", {})) != set(expected):
        raise fail("Phase74 truth route set changed")
    if set(freeze.get("sealed_phase73_artifacts", {}).get("routes", {})) != set(expected):
        raise fail("Phase74 artifact route set changed")
    return expected


def verify_freeze() -> dict[str, Any]:
    if sha256_file(FREEZE) != FREEZE_SHA256:
        raise fail("Phase74 accuracy freeze hash changed")
    freeze = load_json(FREEZE, "Phase74 accuracy freeze")
    if freeze.get("status") != "frozen-before-phase74-truth-read":
        raise fail("Phase74 accuracy freeze status changed")
    if freeze.get("authority", {}).get("phase73_structural_result_record", {}).get("sha256") != "8493d16b3d2c5ce8b09b65ab0dada6903fcbac01257fb8db7977cb41c85577fd":
        raise fail("Phase73 structural result record pin changed")
    structural_record = ROOT / "docs/use_cases/records/smartphone_r5_phase73_source_exact_pseudorange_miss_mask_structural_result_v1.json"
    if sha256_file(structural_record) != "8493d16b3d2c5ce8b09b65ab0dada6903fcbac01257fb8db7977cb41c85577fd":
        raise fail("Phase73 structural result record bytes changed")
    structural_output = freeze.get("authority", {}).get("phase73_structural_output", {})
    for key in ("result_path", "manifest_path"):
        path = ROOT / str(structural_output.get(key, ""))
        expected = structural_output.get(key.replace("_path", "_sha256"))
        if not expected or sha256_file(path) != expected:
            raise fail(f"Phase73 structural output pin changed: {key}")
    if freeze.get("authority", {}).get("phase44_freeze", {}).get("sha256") != "95c0990af0015b7cb5fcf736aefbcff6fc97356093edcf03094b31b4083b28bc":
        raise fail("Phase44 freeze pin changed")
    if freeze.get("authority", {}).get("phase44_result", {}).get("sha256") != "9e441c78f7c2bf8b3cf2cf9a7c9fe7e447fb2ff8eb6324fa5ef138c3b419e48a":
        raise fail("Phase44 result pin changed")
    if freeze.get("authority", {}).get("phase51_historical_reference", {}).get("scoring_input") is not False:
        raise fail("Phase51 scoring-input policy changed")
    if freeze.get("authority", {}).get("phase52_historical_recovery_reference", {}).get("scoring_input") is not False:
        raise fail("Phase52 scoring-input policy changed")
    metric = freeze.get("metric_contract", {})
    if metric.get("key") != "(phone, UnixTimeMillis)" or metric.get("earth_radius_m") != EARTH_RADIUS_M or metric.get("percentile") != "linear interpolation at rank (n - 1) * q" or metric.get("score") != "(P50 + P95) / 2":
        raise fail("Phase74 metric contract changed")
    gates = freeze.get("accuracy_gates", {})
    expected_gates = {
        "candidate_improves_each_route_by_at_least_m": 0.05,
        "candidate_no_route_regression": True,
        "candidate_macro_improvement_at_least_m": 0.1,
        "candidate_mtv_h_improvement_at_least_m": 0.1,
        "candidate_prediction_domain_coverage": 1.0,
        "candidate_macro_score_max_m": 2.0,
        "candidate_route_score_max_m": 3.0,
        "candidate_mtv_h_p95_max_m": 5.0,
        "candidate_all_finite": True,
        "candidate_over_70_mps_count": 0,
        "declared_before_truth": True,
    }
    if any(gates.get(key) != value for key, value in expected_gates.items()):
        raise fail("Phase74 accuracy gates changed")
    policy = freeze.get("read_policy", {})
    if policy.get("truth_reads_before_freeze") != 0 or policy.get("truth_reads_before_manifest") != 0 or policy.get("truth_reads_after_manifest") != 4 or policy.get("truth_reads_per_route") != 1 or policy.get("native_or_solver_subprocess") is not False or policy.get("post_truth_tuning") is not False:
        raise fail("Phase74 truth/read policy changed")
    _require_freeze_route_map(freeze)
    return freeze


def _parse_submission(payload: bytes, route: str) -> tuple[list[tuple[int, float, float]], dict[int, tuple[float, float]]]:
    try:
        text = payload.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise fail(f"submission is not UTF-8: {route}") from exc
    rows = list(csv.reader(io.StringIO(text)))
    header = ["phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"]
    if not rows or rows[0] != header:
        raise fail(f"submission header mismatch: {route}")
    ordered: list[tuple[int, float, float]] = []
    mapping: dict[int, tuple[float, float]] = {}
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
        if timestamp in mapping:
            raise fail(f"duplicate prediction key: {route}:{timestamp}")
        if previous is not None and timestamp <= previous:
            raise fail(f"prediction timestamps are not strictly increasing: {route}")
        if not all(_finite(value) for value in (latitude, longitude)) or not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise fail(f"invalid prediction coordinate: {route}:{line_number}")
        ordered.append((timestamp, latitude, longitude))
        mapping[timestamp] = (latitude, longitude)
        previous = timestamp
    if not ordered:
        raise fail(f"empty submission: {route}")
    return ordered, mapping


def _parse_truth(payload: bytes, route: str) -> dict[int, tuple[float, float]]:
    try:
        text = payload.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise fail(f"truth is not UTF-8: {route}") from exc
    rows = list(csv.reader(io.StringIO(text)))
    if not rows:
        raise fail(f"empty truth: {route}")
    header = rows[0]
    expected = ["UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"]
    if header != expected:
        raise fail(f"truth header mismatch: {route}")
    mapping: dict[int, tuple[float, float]] = {}
    for line_number, fields in enumerate(rows[1:], start=2):
        if len(fields) != 3:
            raise fail(f"truth column mismatch: {route}:{line_number}")
        try:
            timestamp = int(fields[0])
            latitude = float(fields[1])
            longitude = float(fields[2])
        except ValueError as exc:
            raise fail(f"non-numeric truth row: {route}:{line_number}") from exc
        if timestamp in mapping:
            raise fail(f"duplicate truth key: {route}:{timestamp}")
        if not all(_finite(value) for value in (latitude, longitude)) or not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise fail(f"invalid truth coordinate: {route}:{line_number}")
        mapping[timestamp] = (latitude, longitude)
    if not mapping:
        raise fail(f"empty truth rows: {route}")
    return mapping


def _haversine_m(first: tuple[float, float], second: tuple[float, float]) -> float:
    lat0, lon0 = math.radians(first[0]), math.radians(first[1])
    lat1, lon1 = math.radians(second[0]), math.radians(second[1])
    dlat = lat1 - lat0
    dlon = lon1 - lon0
    h = math.sin(dlat / 2.0) ** 2 + math.cos(lat0) * math.cos(lat1) * math.sin(dlon / 2.0) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(max(0.0, h))))


def _percentile(values: list[float], quantile: float) -> float:
    if not values:
        raise fail("cannot percentile an empty error vector")
    ordered = sorted(values)
    rank = (len(ordered) - 1) * quantile
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (rank - lower) * (ordered[upper] - ordered[lower])


def _speed_report(rows: list[tuple[int, float, float]]) -> dict[str, Any]:
    speeds: list[float] = []
    for previous, current in zip(rows, rows[1:]):
        dt = (current[0] - previous[0]) / 1000.0
        if dt <= 0.0:
            raise fail("prediction time interval is not positive")
        speeds.append(_haversine_m((previous[1], previous[2]), (current[1], current[2])) / dt)
    return {
        "transition_count": len(speeds),
        "max_speed_mps": max(speeds, default=0.0),
        "over_70_mps_count": sum(speed > MAX_SPEED_MPS for speed in speeds),
        "finite": all(_finite(speed) for speed in speeds),
    }


def _score_prediction(prediction: dict[int, tuple[float, float]], truth: dict[int, tuple[float, float]], expected_missing: list[Any] | None, route: str, ordered: list[tuple[int, float, float]]) -> dict[str, Any]:
    prediction_keys = set(prediction)
    truth_keys = set(truth)
    extras = sorted(prediction_keys - truth_keys)
    missing = sorted(truth_keys - prediction_keys)
    expected_key = None if expected_missing is None else int(expected_missing[1])
    allowed_missing = [] if expected_key is None else [expected_key]
    if extras:
        raise fail(f"prediction extras are not allowed: {route}: {extras[:3]}")
    if missing != allowed_missing:
        raise fail(f"truth missing-key contract mismatch: {route}: {missing}")
    errors = [_haversine_m(prediction[key], truth[key]) for key in sorted(prediction_keys & truth_keys)]
    speed = _speed_report(ordered)
    if not errors or not all(_finite(value) for value in errors) or not speed["finite"]:
        raise fail(f"nonfinite accuracy vector: {route}")
    p50 = _percentile(errors, 0.50)
    p95 = _percentile(errors, 0.95)
    return {
        "prediction_rows": len(prediction),
        "truth_rows": len(truth),
        "matched_rows": len(errors),
        "missing_truth_rows": len(missing),
        "missing_truth_keys": [[route, key] for key in missing],
        "prediction_domain_coverage": len(prediction_keys & truth_keys) / len(prediction_keys),
        "truth_row_coverage": len(errors) / len(truth_keys),
        "mean_m": sum(errors) / len(errors),
        "p50_m": p50,
        "p95_m": p95,
        "max_m": max(errors),
        "score_m": (p50 + p95) / 2.0,
        "max_speed_mps": speed["max_speed_mps"],
        "over_70_mps_count": speed["over_70_mps_count"],
        "finite": True,
    }


def _validate_summary(payload: bytes, route: str, candidate: bool) -> dict[str, Any]:
    try:
        summary = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"invalid native summary: {route}") from exc
    if not isinstance(summary, dict) or summary.get("dataset_id") != route or summary.get("truth_used") is not False or summary.get("production_default_changed") is not False:
        raise fail(f"summary identity/policy mismatch: {route}")
    if candidate:
        miss = summary.get("native_base_pseudorange_source_miss_mask")
        base = summary.get("native_base_pseudorange_compensation")
        if not isinstance(miss, dict) or miss.get("enabled") is not True or not isinstance(base, dict) or base.get("enabled") is not True or base.get("preserve_additional_frequency_bands") is not False:
            raise fail(f"Phase73 candidate summary contract mismatch: {route}")
        if miss.get("pseudorange_factor_count_consistent") is not True or miss.get("retained_finite_pc_fraction") != 1.0 or miss.get("tdcp_doppler_imu_spp_unchanged") is not True:
            raise fail(f"Phase73 candidate telemetry mismatch: {route}")
        return {"summary": summary, "miss_mask": miss, "base": base}
    if "native_base_pseudorange_source_miss_mask" in summary or "native_base_pseudorange_compensation" in summary:
        raise fail(f"control unexpectedly carries base telemetry: {route}")
    return {"summary": summary}


def _artifact_paths(freeze: dict[str, Any], route: str) -> tuple[dict[str, Any], dict[str, Any]]:
    route_artifacts = freeze["sealed_phase73_artifacts"]["routes"][route]
    return route_artifacts["candidate"], route_artifacts["control"]


def score_route(freeze: dict[str, Any], route: str, accounting: dict[str, int]) -> dict[str, Any]:
    candidate_pin, control_pin = _artifact_paths(freeze, route)
    candidate_submission_path = ROOT / candidate_pin["submission_path"]
    candidate_summary_path = ROOT / candidate_pin["summary_path"]
    control_submission_path = ROOT / control_pin["submission_path"]
    control_summary_path = ROOT / control_pin["summary_path"]
    accounting["candidate_artifact_reads"] += 1
    candidate_submission = _read_once(candidate_submission_path, candidate_pin["submission_sha256"], int(candidate_pin["submission_bytes"]), "candidate submission")
    accounting["candidate_artifact_reads"] += 1
    candidate_summary = _read_once(candidate_summary_path, candidate_pin["summary_sha256"], int(candidate_pin["summary_bytes"]), "candidate summary") if "summary_bytes" in candidate_pin else _read_once(candidate_summary_path, candidate_pin["summary_sha256"], candidate_summary_path.stat().st_size, "candidate summary")
    accounting["control_artifact_reads"] += 1
    control_submission = _read_once(control_submission_path, control_pin["submission_sha256"], int(control_pin["submission_bytes"]), "control submission")
    accounting["control_artifact_reads"] += 1
    control_summary = _read_once(control_summary_path, control_pin["summary_sha256"], int(control_pin["summary_bytes"]), "control summary") if "summary_bytes" in control_pin else _read_once(control_summary_path, control_pin["summary_sha256"], control_summary_path.stat().st_size, "control summary")
    candidate_rows, candidate_map = _parse_submission(candidate_submission, route)
    control_rows, control_map = _parse_submission(control_submission, route)
    candidate_diag = _validate_summary(candidate_summary, route, True)
    control_diag = _validate_summary(control_summary, route, False)
    if candidate_map.keys() != control_map.keys():
        raise fail(f"candidate/control prediction domain differs: {route}")
    truth_pin = freeze["cohort"]["truths"][route]
    truth_path = ROOT / truth_pin["path"]
    accounting["truth_reads"] += 1
    truth_payload = _read_once(truth_path, truth_pin["sha256"], int(truth_pin["bytes"]), "development truth")
    truth_map = _parse_truth(truth_payload, route)
    candidate_score = _score_prediction(candidate_map, truth_map, truth_pin.get("expected_missing_key"), route, candidate_rows)
    control_score = _score_prediction(control_map, truth_map, truth_pin.get("expected_missing_key"), route, control_rows)
    improvement = control_score["score_m"] - candidate_score["score_m"]
    return {
        "truth": {"path": relative(truth_path), "sha256": truth_pin["sha256"], "bytes": len(truth_payload), "rows": len(truth_map), "read_count": 1, "expected_missing_truth_rows": truth_pin["expected_missing_truth_rows"], "expected_missing_key": truth_pin["expected_missing_key"]},
        "artifact_hashes": {"candidate_submission": candidate_pin["submission_sha256"], "candidate_summary": candidate_pin["summary_sha256"], "control_submission": control_pin["submission_sha256"], "control_summary": control_pin["summary_sha256"]},
        "candidate": candidate_score,
        "control": control_score,
        "improvement_m": improvement,
        "candidate_phase73_telemetry": {key: candidate_diag["miss_mask"][key] for key in ("original_adopted_pseudorange_rows", "retained_finite_pc_pseudorange_rows", "dropped_missing_exact_stream_rows", "dropped_out_of_domain_rows", "dropped_nonfinite_correction_rows", "retained_over_original_fraction", "correction_abs_p50_m", "correction_abs_p95_m", "correction_abs_max_m")},
        "control_phase43_identity": control_pin["submission_sha256"] in {"d4d7652e5d12389466e586fe2d8e85d34977a7e11036cee2f591a89293c5426c", "3cfe97750927fa268f71bbe7393754ce7a1eb39d937e085b177c50a71eec10ae", "4eb2b566708a87db4903610a41cd648c7ff065d35710d358894a78eaa8e116e3", "524769cdf67aa857eefaafdadf943dd76586dda6a53e8b6d1df4a707d7699f71"},
    }


def run_score(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    freeze = verify_freeze()
    if not MANIFEST.is_file():
        raise fail("Phase74 accuracy evaluator manifest is missing")
    manifest = load_json(MANIFEST, "Phase74 accuracy evaluator manifest")
    if manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256 or manifest.get("evaluator", {}).get("path") != relative(EVALUATOR) or manifest.get("evaluator", {}).get("sha256") != sha256_file(EVALUATOR) or manifest.get("read_policy", {}).get("truth_reads") != 4 or manifest.get("read_policy", {}).get("native_or_solver_subprocess") is not False:
        raise fail("Phase74 evaluator manifest pin/read contract changed")
    output_root = output_root.resolve()
    reject_forbidden(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty Phase74 output: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    accounting = {"candidate_artifact_reads": 0, "control_artifact_reads": 0, "truth_reads": 0}
    routes: dict[str, Any] = {}
    try:
        for route in _require_freeze_route_map(freeze):
            report = score_route(freeze, route, accounting)
            candidate = report["candidate"]
            improvement = report["improvement_m"]
            route_gate = {
                "candidate_improvement_at_least_0_05m": improvement >= 0.05,
                "candidate_no_route_regression": improvement >= 0.0,
                "candidate_prediction_domain_coverage_exact": candidate["prediction_domain_coverage"] == 1.0,
                "candidate_finite": candidate["finite"],
                "candidate_over_70_mps_count_zero": candidate["over_70_mps_count"] == 0,
                "candidate_route_score_at_most_3m": candidate["score_m"] <= 3.0,
                "control_phase43_identity": report["control_phase43_identity"],
            }
            if not all(route_gate.values()):
                report["gate"] = {"passed": False, "failures": [key for key, value in route_gate.items() if not value]}
            else:
                report["gate"] = {"passed": True, "failures": []}
            routes[route] = report
        candidate_scores = [routes[route]["candidate"]["score_m"] for route in _require_freeze_route_map(freeze)]
        control_scores = [routes[route]["control"]["score_m"] for route in _require_freeze_route_map(freeze)]
        candidate_macro = sum(candidate_scores) / len(candidate_scores)
        control_macro = sum(control_scores) / len(control_scores)
        macro_improvement = control_macro - candidate_macro
        target_route = "2021-08-24-20-32-us-ca-mtv-h/pixel5"
        gates = {
            "all_four_routes": len(routes) == 4,
            "candidate_each_route_improves_0_05m": all(routes[route]["gate"]["passed"] for route in routes),
            "candidate_macro_improvement_at_least_0_10m": macro_improvement >= 0.1,
            "candidate_mtv_h_improvement_at_least_0_10m": routes[target_route]["improvement_m"] >= 0.1,
            "candidate_macro_score_at_most_2m": candidate_macro <= 2.0,
            "candidate_each_route_score_at_most_3m": all(routes[route]["candidate"]["score_m"] <= 3.0 for route in routes),
            "candidate_mtv_h_p95_at_most_5m": routes[target_route]["candidate"]["p95_m"] <= 5.0,
            "candidate_prediction_domain_coverage_exact": all(routes[route]["candidate"]["prediction_domain_coverage"] == 1.0 for route in routes),
            "candidate_over_70_mps_count_zero": all(routes[route]["candidate"]["over_70_mps_count"] == 0 for route in routes),
            "all_finite": all(routes[route]["candidate"]["finite"] and routes[route]["control"]["finite"] for route in routes),
            "control_phase43_identity": all(routes[route]["control_phase43_identity"] for route in routes),
        }
        all_passed = all(gates.values())
        result = {
            "schema_version": OUTPUT_SCHEMA,
            "phase": 74,
            "execution_label": "Luna Max",
            "status": "go-phase74-accuracy-gates" if all_passed else "no-go-phase74-accuracy-gates",
            "decision": "candidate may proceed to a separately frozen validation phase" if all_passed else "preserve-phase43-champion; keep-phase73-experimental; no-validation",
            "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
            "manifest": {"path": relative(MANIFEST), "sha256": sha256_file(MANIFEST)},
            "evaluator": {"path": relative(EVALUATOR), "sha256": sha256_file(EVALUATOR), "single_process": True, "native_or_solver_subprocess": False, "post_truth_tuning": False},
            "phase73_structural_input": {"result_record": {"path": "docs/use_cases/records/smartphone_r5_phase73_source_exact_pseudorange_miss_mask_structural_result_v1.json", "sha256": "8493d16b3d2c5ce8b09b65ab0dada6903fcbac01257fb8db7977cb41c85577fd"}, "candidate/control_source": "Phase73 structural run1 only; control is exact Phase43 identity"},
            "metric_contract": freeze["metric_contract"],
            "routes": routes,
            "aggregate": {"candidate_macro_score_m": candidate_macro, "control_phase43_macro_score_m": control_macro, "macro_improvement_m": macro_improvement, "candidate_route_count": len(candidate_scores), "control_route_count": len(control_scores)},
            "historical_phase51_reference": freeze["authority"]["phase52_historical_recovery_reference"],
            "phase44_baseline_reference": freeze["authority"]["phase44_baseline_reference"],
            "accuracy_gates": gates | {"all_passed": all_passed},
            "stretch_0_782": {"target_score_m": 0.782, "candidate_macro_score_m": candidate_macro, "met": candidate_macro <= 0.782, "report_only": True},
            "read_accounting": {"single_scorer_process": True, "candidate_artifact_reads": accounting["candidate_artifact_reads"], "control_artifact_reads": accounting["control_artifact_reads"], "truth_reads": accounting["truth_reads"], "truth_reads_per_route": 1, "raw_gnss_reads": 0, "raw_imu_reads": 0, "navigation_reads": 0, "native_or_solver_subprocess": 0, "validation_holdout_reads": 0, "mat_reads_or_generated": 0, "device_wls_or_precomputed_coordinates": 0, "kaggle_or_token_access": 0, "archive_reopen_or_rematerialize": False, "post_truth_tuning": False},
            "phase43_champion_preserved": True,
            "phase51_experimental_preserved": True,
            "phase58_experimental_preserved": True,
            "fresh_validation": "not run in Phase74; separate freeze required" if all_passed else "not authorized",
        }
        result_path = output_root / "phase74_phase73_miss_mask_accuracy_result.json"
        atomic_json(result_path, result)
        output_manifest = {"schema_version": "smartphone-r5-phase74-phase73-miss-mask-accuracy-output-manifest.v1", "phase": 74, "status": result["status"], "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "evaluator": {"path": relative(EVALUATOR), "sha256": sha256_file(EVALUATOR)}, "result": {"path": relative(result_path), "sha256": sha256_file(result_path), "bytes": result_path.stat().st_size}, "truth_reads": accounting["truth_reads"], "candidate_artifact_reads": accounting["candidate_artifact_reads"], "control_artifact_reads": accounting["control_artifact_reads"], "native_or_solver_subprocess": 0, "all_gates_passed": all_passed}
        atomic_json(output_root / "phase74_phase73_miss_mask_accuracy_manifest.json", output_manifest)
        return result
    except Exception as exc:
        failure = {"schema_version": "smartphone-r5-phase74-phase73-miss-mask-accuracy-failure.v1", "status": "fail-closed", "exception_type": type(exc).__name__, "error": str(exc), "truth_reads": accounting["truth_reads"], "candidate_artifact_reads": accounting["candidate_artifact_reads"], "control_artifact_reads": accounting["control_artifact_reads"], "routes_completed": sorted(routes), "native_or_solver_subprocess": 0, "post_truth_tuning": False}
        atomic_json(output_root / "phase74_phase73_miss_mask_accuracy_failure.json", failure)
        if isinstance(exc, Phase74AccuracyError):
            raise
        raise fail(f"unexpected scorer exception: {type(exc).__name__}: {exc}") from exc


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--run-score", action="store_true")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        if args.verify_freeze:
            verify_freeze()
        if args.run_score:
            result = run_score(args.output_root)
            print(json.dumps({"status": result["status"], "all_gates_passed": result["accuracy_gates"]["all_passed"], "truth_reads": result["read_accounting"]["truth_reads"]}, sort_keys=True))
        elif not args.verify_freeze:
            parser.error("one of --verify-freeze or --run-score is required")
        return 0
    except Phase74AccuracyError as exc:
        print(f"phase74 accuracy failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
