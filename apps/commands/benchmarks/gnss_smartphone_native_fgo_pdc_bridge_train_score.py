#!/usr/bin/env python3
"""Score sealed bridge/baseline artifacts on development truth exactly once.

This evaluator is deliberately not a solver.  It verifies that all route
artifacts were sealed before reading any development truth, reads each
historical development ``ground_truth.csv`` once, and publishes a signed
route/aggregate report.  Validation, holdout, test, leaderboard and token
paths are rejected by construction.
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
import time
from typing import Any


_COMMANDS_DIR = Path(__file__).resolve().parents[1]
if str(_COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(_COMMANDS_DIR))
from support.gnss_runtime import application_root  # noqa: E402


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_kaggle as kaggle  # noqa: E402
import gnss_smartphone_native_fgo_eval as native_eval  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402


GATE_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-pdc-bridge-train-gate.v1"
REPORT_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-pdc-bridge-train-score.v1"
DEFAULT_OUTPUT_ROOT = ROOT / "output/smartphone-r5/native-fgo-pdc-bridge-v1"
TRAIN_IDS = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-07-27-19-49-us-ca-mtv-b/pixel4",
    "2022-02-24-18-29-us-ca-lax-o/pixel5",
)
DIAGNOSTIC_KEYS = tuple(native_eval.DIAGNOSTIC_KEYS)
LEAP_SECONDS = 18
TOLERANCE = 1e-12
SPEED_BOUND_MPS = 70.0


class ScoreError(ValueError):
    """Raised when the sealed scoring contract is invalid."""


def _resolve(path: str | Path) -> Path:
    candidate = Path(path)
    return candidate if candidate.is_absolute() else ROOT / candidate


def _sha256_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise ScoreError(f"missing file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ScoreError(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise ScoreError(f"{label} must be a JSON object")
    return payload


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    content = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_bytes(content)
    temporary.replace(path)


def _load_gate(path: Path) -> dict[str, Any]:
    gate = _load_json(path, "train gate record")
    if gate.get("schema_version") != GATE_SCHEMA:
        raise ScoreError("train gate schema mismatch")
    if gate.get("status") != "frozen-before-development-train-score":
        raise ScoreError("train gate is not frozen before scoring")
    policy = gate.get("truth_policy")
    if not isinstance(policy, dict):
        raise ScoreError("train gate truth policy is missing")
    for key, expected in (
        ("truth_free_candidate_artifacts_sealed", True),
        ("truth_free_baseline_artifacts_sealed", True),
        ("development_truth_only", True),
        ("fresh_validation_opened", False),
        ("holdout_opened", False),
        ("test_opened", False),
        ("leaderboard_used_for_tuning", False),
        ("token_access", False),
        ("external_mutation", False),
        ("no_post_score_parameter_tuning", True),
    ):
        if policy.get(key) is not expected:
            raise ScoreError(f"train gate policy mismatch: {key}")
    inventory = gate.get("authoritative_inventory")
    if not isinstance(inventory, dict):
        raise ScoreError("authoritative inventory is missing")
    inventory_path = _resolve(str(inventory["record"]))
    if _sha256(inventory_path) != inventory.get("sha256_at_freeze"):
        raise ScoreError("authoritative inventory hash mismatch")
    routes = gate.get("fixed_routes")
    if not isinstance(routes, dict) or tuple(routes) != TRAIN_IDS:
        raise ScoreError("train route order mismatch")
    for dataset_id in TRAIN_IDS:
        route = routes[dataset_id]
        if not isinstance(route, dict):
            raise ScoreError(f"invalid route gate: {dataset_id}")
        for role in ("candidate", "baseline"):
            manifest_path = _resolve(str(route[f"{role}_manifest"]))
            expected_hash = route[f"{role}_manifest_sha256"]
            if _sha256(manifest_path) != expected_hash:
                raise ScoreError(f"{role} manifest hash mismatch: {dataset_id}")
            manifest = _load_json(manifest_path, f"{role} route manifest")
            if manifest.get("truth_opened") is not False:
                raise ScoreError(f"{role} artifact was not truth-free: {dataset_id}")
            artifacts = manifest.get("artifacts")
            if not isinstance(artifacts, list):
                raise ScoreError(f"{role} artifact list is missing: {dataset_id}")
            for artifact in artifacts:
                if not isinstance(artifact, dict):
                    raise ScoreError(f"invalid {role} artifact entry: {dataset_id}")
                artifact_path = _resolve(str(manifest_path.parent / str(artifact["path"])))
                if _sha256(artifact_path) != artifact.get("sha256"):
                    raise ScoreError(f"{role} artifact hash mismatch: {dataset_id}/{artifact['path']}")
        truth = _resolve(str(route["truth"]))
        if not truth.is_file():
            raise ScoreError(f"historical development truth is missing: {dataset_id}")
    return gate


def _read_truth_once(path: Path, expected_hash: str) -> tuple[dict[int, tuple[float, float, float]], str]:
    """Read/hash one truth file in one byte read, then parse that buffer."""

    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise ScoreError(f"failed to read development truth: {path}") from exc
    observed = _sha256_bytes(raw)
    if observed != expected_hash:
        raise ScoreError(f"development truth hash mismatch: {path}")
    try:
        text = raw.decode("utf-8-sig")
        reader = csv.DictReader(io.StringIO(text, newline=""))
        required = {"UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees", "AltitudeMeters"}
        if not required.issubset(set(reader.fieldnames or ())):
            raise ScoreError(f"development truth schema mismatch: {path}")
        truth: dict[int, tuple[float, float, float]] = {}
        for number, row in enumerate(reader, start=2):
            try:
                timestamp = int(row.get("UnixTimeMillis", ""))
                values = (
                    float(row.get("LatitudeDegrees", "")),
                    float(row.get("LongitudeDegrees", "")),
                    float(row.get("AltitudeMeters", "")),
                )
            except (TypeError, ValueError) as exc:
                raise ScoreError(f"invalid truth row {number}: {path}") from exc
            if timestamp < 0 or not all(math.isfinite(value) for value in values) or timestamp in truth:
                raise ScoreError(f"invalid/duplicate truth row {number}: {path}")
            truth[timestamp] = values
    except UnicodeDecodeError as exc:
        raise ScoreError(f"development truth is not UTF-8: {path}") from exc
    if not truth:
        raise ScoreError(f"development truth is empty: {path}")
    return truth, observed


def _score_position(path: Path, device_path: Path, truth: dict[int, tuple[float, float, float]]) -> dict[str, Any]:
    try:
        positions = smoother._read_positions(path, LEAP_SECONDS)
        epochs = kaggle._read_device_epochs(device_path, 0)
    except Exception as exc:  # noqa: BLE001 - normalize parser failure
        raise ScoreError(f"invalid score input: {path}") from exc
    if any(row.timestamp_ms not in set(epochs) for row in positions):
        raise ScoreError(f"position contains non-device timestamp: {path}")
    raw_rows = native_eval._raw_rows(positions)
    return smoother_eval._score_rows(
        raw_rows,
        {row.timestamp_ms: row for row in positions},
        truth,
        0,
        len(epochs),
        match_tolerance_ms=100,
    )


def _metric(metrics: dict[str, Any], path: tuple[str, ...]) -> float:
    value: Any = metrics
    for key in path:
        value = value[key]
    if value is None or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        return math.inf
    return float(value)


def _aggregate(metrics: list[dict[str, Any]]) -> dict[str, Any]:
    if not metrics:
        raise ScoreError("cannot aggregate an empty route set")
    variants = {
        key: sum(_metric(row, ("kaggle_diagnostic_score_variants_m", key)) for row in metrics) / len(metrics)
        for key in DIAGNOSTIC_KEYS
    }
    result: dict[str, Any] = {
        "route_count": len(metrics),
        "mean_availability_ratio": sum(_metric(row, ("availability_ratio",)) for row in metrics) / len(metrics),
        "mean_truth_coverage_ratio": sum(_metric(row, ("truth_coverage_ratio",)) for row in metrics) / len(metrics),
        "mean_horizontal_wgs84_p50_m": sum(_metric(row, ("horizontal_wgs84_m", "p50_m")) for row in metrics) / len(metrics),
        "mean_horizontal_wgs84_p95_m": sum(_metric(row, ("horizontal_wgs84_m", "p95_m")) for row in metrics) / len(metrics),
        "mean_vertical_p95_abs_m": sum(_metric(row, ("vertical_p95_abs_m",)) for row in metrics) / len(metrics),
        "mean_kaggle_diagnostic_score_variants_m": variants,
    }
    result["mean_kaggle_diagnostic_m"] = sum(variants.values()) / len(variants)
    return result


def _route_speed_stats(path: Path) -> dict[str, Any]:
    positions = smoother._read_positions(path, LEAP_SECONDS)
    speeds: list[float] = []
    for previous, current in zip(positions, positions[1:]):
        dt = (current.timestamp_ms - previous.timestamp_ms) / 1000.0
        if dt <= 0.0:
            raise ScoreError(f"non-increasing position timestamps: {path}")
        speed = float((current.ecef - previous.ecef).dot(current.ecef - previous.ecef) ** 0.5) / dt
        if not math.isfinite(speed):
            raise ScoreError(f"non-finite transition speed: {path}")
        speeds.append(speed)
    return {
        "transition_count": len(speeds),
        "max_speed_mps": max(speeds, default=0.0),
        "over_70mps_count": sum(speed > SPEED_BOUND_MPS for speed in speeds),
    }


def _route_gate(candidate: dict[str, Any], baseline: dict[str, Any], candidate_speed: dict[str, Any], baseline_speed: dict[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    if _metric(candidate, ("availability_ratio",)) + TOLERANCE < _metric(baseline, ("availability_ratio",)):
        failures.append("availability_regression")
    if _metric(candidate, ("truth_coverage_ratio",)) + TOLERANCE < _metric(baseline, ("truth_coverage_ratio",)):
        failures.append("truth_coverage_regression")
    for path, label in (
        (("horizontal_wgs84_m", "p50_m"), "h_p50_regression"),
        (("horizontal_wgs84_m", "p95_m"), "h_p95_regression"),
        (("vertical_p95_abs_m",), "v_p95_regression"),
    ):
        if _metric(candidate, path) > _metric(baseline, path) + TOLERANCE:
            failures.append(label)
    for key in DIAGNOSTIC_KEYS:
        if _metric(candidate, ("kaggle_diagnostic_score_variants_m", key)) >= _metric(baseline, ("kaggle_diagnostic_score_variants_m", key)) - TOLERANCE:
            failures.append(f"{key}_not_strictly_better")
    if candidate_speed["over_70mps_count"] > baseline_speed["over_70mps_count"]:
        failures.append("new_over_70mps_transitions")
    return {"passed": not failures, "failures": failures}


def _aggregate_gate(candidate: dict[str, Any], baseline: dict[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    if candidate["mean_availability_ratio"] + TOLERANCE < baseline["mean_availability_ratio"]:
        failures.append("availability_regression")
    if candidate["mean_truth_coverage_ratio"] + TOLERANCE < baseline["mean_truth_coverage_ratio"]:
        failures.append("truth_coverage_regression")
    for key, label in (
        ("mean_horizontal_wgs84_p50_m", "h_p50_regression"),
        ("mean_horizontal_wgs84_p95_m", "h_p95_regression"),
        ("mean_vertical_p95_abs_m", "v_p95_regression"),
    ):
        if candidate[key] > baseline[key] + TOLERANCE:
            failures.append(label)
    for key in DIAGNOSTIC_KEYS:
        if candidate["mean_kaggle_diagnostic_score_variants_m"][key] >= baseline["mean_kaggle_diagnostic_score_variants_m"][key] - TOLERANCE:
            failures.append(f"{key}_not_strictly_better")
    return {"passed": not failures, "failures": failures}


def score(gate_path: Path, output_root: Path) -> dict[str, Any]:
    started = time.perf_counter()
    gate = _load_gate(gate_path)
    route_reports: dict[str, Any] = {}
    baseline_metrics: list[dict[str, Any]] = []
    candidate_metrics: list[dict[str, Any]] = []
    truth_hashes: dict[str, str] = {}
    truth_read_count = 0
    for dataset_id in TRAIN_IDS:
        route = gate["fixed_routes"][dataset_id]
        candidate_manifest_path = _resolve(route["candidate_manifest"])
        baseline_manifest_path = _resolve(route["baseline_manifest"])
        candidate_manifest = _load_json(candidate_manifest_path, "candidate manifest")
        baseline_manifest = _load_json(baseline_manifest_path, "baseline manifest")
        candidate_pos = candidate_manifest_path.parent / "fgo.pos"
        baseline_pos = baseline_manifest_path.parent / "fgo.pos"
        device_path = _resolve(
            "output/smartphone-r5/native-fgo-v2-processed/routes/"
            + dataset_id.replace("/", "__")
            + "/inputs/device_gnss.csv"
        )
        truth, truth_hash = _read_truth_once(
            _resolve(route["truth"]), route["truth_sha256_from_prior_record"]
        )
        truth_read_count += 1
        truth_hashes[dataset_id] = truth_hash
        baseline = _score_position(baseline_pos, device_path, truth)
        candidate = _score_position(candidate_pos, device_path, truth)
        baseline_metrics.append(baseline)
        candidate_metrics.append(candidate)
        route_reports[dataset_id] = {
            "baseline_native_fgo_v1": baseline,
            "candidate_native_fgo_pdc": candidate,
            "continuity": {
                "baseline": _route_speed_stats(baseline_pos),
                "candidate": _route_speed_stats(candidate_pos),
            },
            "gate": _route_gate(
                candidate,
                baseline,
                _route_speed_stats(candidate_pos),
                _route_speed_stats(baseline_pos),
            ),
            "truth": {"path": route["truth"], "sha256": truth_hash, "read_count": 1},
            "truth_opened": True,
        }
    baseline_aggregate = _aggregate(baseline_metrics)
    candidate_aggregate = _aggregate(candidate_metrics)
    aggregate_gate = _aggregate_gate(candidate_aggregate, baseline_aggregate)
    route_gate = all(report["gate"]["passed"] for report in route_reports.values())
    train_passed = route_gate and aggregate_gate["passed"]
    report: dict[str, Any] = {
        "schema_version": REPORT_SCHEMA,
        "status": "train-pass" if train_passed else "no-go-train-gate",
        "candidate_id": "native-fgo-pdc-bridge-v1",
        "gate_record": str(gate_path.relative_to(ROOT)),
        "gate_record_sha256": _sha256(gate_path),
        "truth_free_artifacts_verified_before_truth": True,
        "truth_policy": {
            "truth_read_count": truth_read_count,
            "truth_hashes": truth_hashes,
            "development_only": True,
            "fresh_validation_truth_read_count": 0,
            "holdout_truth_read_count": 0,
            "test_truth_read_count": 0,
            "leaderboard_used_for_tuning": False,
            "token_access": False,
            "external_mutation": False,
        },
        "routes": route_reports,
        "aggregate": {
            "baseline_native_fgo_v1": baseline_aggregate,
            "candidate_native_fgo_pdc": candidate_aggregate,
            "gate": aggregate_gate,
        },
        "train_gate": {
            "route_level_passed": route_gate,
            "aggregate_passed": aggregate_gate["passed"],
            "passed": train_passed,
        },
        "validation_policy": {
            "opened": False,
            "reason": "A failed train gate seals fresh validation and all holdout/test identities.",
            "fresh_validation_id": "2023-05-09-21-32-us-ca-mtv-pe1/pixel5",
            "future_holdout_id": "2023-05-16-19-54-us-ca-mtv-xe1/pixel5",
        },
        "runtime": {"wall_seconds": time.perf_counter() - started},
        "no_post_score_tuning": True,
    }
    output_root.mkdir(parents=True, exist_ok=True)
    report_path = output_root / "train_evaluation.json"
    _atomic_json(report_path, report)
    manifest = {
        "schema_version": "smartphone-r5-gsdc2023-native-fgo-pdc-bridge-train-score-manifest.v1",
        "status": report["status"],
        "report": {"path": str(report_path.relative_to(ROOT)), "sha256": _sha256(report_path), "bytes": report_path.stat().st_size},
        "gate_record": {"path": str(gate_path.relative_to(ROOT)), "sha256": _sha256(gate_path)},
        "route_manifest_sha256": {
            dataset_id: {
                "candidate": gate["fixed_routes"][dataset_id]["candidate_manifest_sha256"],
                "baseline": gate["fixed_routes"][dataset_id]["baseline_manifest_sha256"],
            }
            for dataset_id in TRAIN_IDS
        },
        "truth_read_count": truth_read_count,
        "fresh_validation_truth_read_count": 0,
        "holdout_truth_read_count": 0,
    }
    manifest_path = output_root / "train_evaluation.manifest.json"
    _atomic_json(manifest_path, manifest)
    _atomic_json(output_root / "train_evaluation.manifest.sha256.json", {"sha256": _sha256(manifest_path), "path": manifest_path.name})
    report["report_path"] = str(report_path.relative_to(ROOT))
    report["report_sha256"] = _sha256(report_path)
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gate-record", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    args = parser.parse_args(argv)
    try:
        result = score(_resolve(args.gate_record), _resolve(args.output_root))
    except ScoreError as exc:
        print(f"score error: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
