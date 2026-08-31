#!/usr/bin/env python3
"""Score one sealed raw-only native GNSS/PDC route with fixed time alignment.

The native Android clock is an epoch clock while the development truth export
can be sampled on the adjacent millisecond phase.  This recovery evaluator
uses a predeclared, one-to-one nearest timestamp match with a fixed 1000 ms
bound.  It is an evaluation adapter only: it never changes coordinates,
reruns a solver, or selects parameters.  A result/truth/sample MAT path is
rejected before any file is opened; MAT data is not an accepted input.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
import tempfile
from typing import Any


BENCHMARK_DIR = Path(__file__).resolve().parent
ROOT = BENCHMARK_DIR.parents[2]
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_kaggle as metric  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-native-gnss-pdc-train-score.v1"
RECOVERY_SCHEMA_VERSION = "smartphone-r5-native-gnss-pdc-train-score-recovery.v1"
TIMESTAMP_TOLERANCE_MS = 1000
SUBMISSION_FIELDS = metric.SUBMISSION_FIELDS


class NativeGnssPdcScoreError(ValueError):
    """Raised when the sealed evaluation contract cannot be met."""


def _reject_mat_path(path: Path, label: str) -> None:
    lowered = "/".join(path.parts).lower()
    if ".mat" in lowered:
        raise NativeGnssPdcScoreError(
            f"{label} is forbidden: MATLAB data paths are never opened"
        )


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise NativeGnssPdcScoreError(f"missing regular file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        content = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
        with open(descriptor, "wb", closefd=True) as handle:
            handle.write(content)
            handle.flush()
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def _load_json(path: Path, label: str) -> dict[str, Any]:
    _reject_mat_path(path, label)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise NativeGnssPdcScoreError(f"invalid {label}: {exc}") from exc
    if not isinstance(payload, dict):
        raise NativeGnssPdcScoreError(f"{label} must be a JSON object")
    return payload


def _validate_recovery(
    recovery_path: Path,
    candidate_path: Path,
    run_manifest_path: Path,
    phone: str,
) -> dict[str, Any]:
    recovery = _load_json(recovery_path, "score-recovery authorization")
    if recovery.get("schema_version") != RECOVERY_SCHEMA_VERSION:
        raise NativeGnssPdcScoreError("score-recovery authorization schema mismatch")
    if recovery.get("status") not in {
        "authorized-evaluation-recovery",
        "authorized-evaluation-recovery-v2",
    }:
        raise NativeGnssPdcScoreError("score-recovery authorization is not active")
    if recovery.get("candidate", {}).get("route_device") != phone:
        raise NativeGnssPdcScoreError("score-recovery route/device does not match --phone")
    expected_candidate = recovery.get("candidate", {}).get("keyed_output_sha256")
    if expected_candidate != _sha256(candidate_path):
        raise NativeGnssPdcScoreError("candidate hash does not match recovery authorization")
    expected_manifest = recovery.get("candidate", {}).get("run_manifest_sha256")
    if expected_manifest != _sha256(run_manifest_path):
        raise NativeGnssPdcScoreError("run manifest hash does not match recovery authorization")
    if recovery.get("contract", {}).get("candidate_unchanged") is not True:
        raise NativeGnssPdcScoreError("recovery contract does not freeze candidate")
    if recovery.get("contract", {}).get("parameter_selection") is not False:
        raise NativeGnssPdcScoreError("recovery contract permits parameter selection")
    return recovery


def _validate_run_manifest(
    path: Path, candidate_path: Path, phone: str
) -> dict[str, Any]:
    manifest = _load_json(path, "native PDC run manifest")
    if manifest.get("schema_version") != "smartphone-r5-native-gnss-pdc-run-manifest.v1":
        raise NativeGnssPdcScoreError("native PDC run manifest schema mismatch")
    if manifest.get("status") != "truth-free-artifacts-sealed" or manifest.get("truth_free") is not True:
        raise NativeGnssPdcScoreError("native PDC run manifest is not truth-free and sealed")
    if manifest.get("trip_id") != phone:
        raise NativeGnssPdcScoreError("native PDC run manifest route/device mismatch")
    policy = manifest.get("forbidden_input_policy")
    if not isinstance(policy, dict):
        raise NativeGnssPdcScoreError("native PDC run manifest has no forbidden-input policy")
    required_false = (
        "result_coordinates_read",
        "ground_truth_read",
        "sample_coordinates_read",
        "v5_output_read",
        "python_coordinate_or_rinex_stage",
        "base_or_double_difference_factors",
    )
    if manifest.get("forbidden_input_policy", {}).get("mat_paths_rejected_before_open") is not True:
        raise NativeGnssPdcScoreError("native PDC manifest does not reject MAT paths")
    if any(policy.get(name) is not False for name in required_false):
        raise NativeGnssPdcScoreError("native PDC manifest violates forbidden-input policy")
    artifact = manifest.get("artifacts", {}).get("keyed.csv")
    if not isinstance(artifact, dict) or artifact.get("sha256") != _sha256(candidate_path):
        raise NativeGnssPdcScoreError("keyed output hash does not match native PDC manifest")
    companion = path.with_name("run_manifest.sha256")
    if companion.is_file():
        token = companion.read_text(encoding="ascii").strip().split()
        if len(token) < 1 or token[0] != _sha256(path):
            raise NativeGnssPdcScoreError("run manifest companion hash mismatch")
    return manifest


def _read_truth_once(path: Path, default_phone: str) -> tuple[list[metric.CoordinateRow], str]:
    """Read and hash one truth file through one file descriptor."""

    _reject_mat_path(path, "ground truth")
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise NativeGnssPdcScoreError(f"failed to read ground truth: {exc}") from exc
    digest = hashlib.sha256(raw).hexdigest()
    try:
        text = raw.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise NativeGnssPdcScoreError(f"ground truth is not UTF-8: {exc}") from exc
    reader = csv.DictReader(text.splitlines())
    fields = list(reader.fieldnames or ())
    missing = [field for field in metric.GROUND_TRUTH_FIELDS if field not in fields]
    if missing:
        raise NativeGnssPdcScoreError(
            f"ground truth missing fields: {', '.join(missing)}"
        )
    has_phone = "phone" in fields
    parsed_default = metric._parse_phone(default_phone, "--phone", 0)
    rows: list[metric.CoordinateRow] = []
    seen: set[tuple[str, int]] = set()
    for row_number, raw_row in enumerate(reader, start=2):
        if None in raw_row:
            raise NativeGnssPdcScoreError(
                f"ground-truth row {row_number}: more columns than header"
            )
        row = {key: (value or "") for key, value in raw_row.items()}
        phone = (
            metric._parse_phone(row.get("phone"), "phone", row_number)
            if has_phone
            else parsed_default
        )
        timestamp = metric._parse_integer(
            row.get("UnixTimeMillis"), "UnixTimeMillis", row_number
        )
        if timestamp < 0:
            raise NativeGnssPdcScoreError(
                f"ground-truth row {row_number}: negative timestamp"
            )
        latitude = metric._coordinate(
            row.get("LatitudeDegrees"), "LatitudeDegrees", row_number, -90.0, 90.0
        )
        longitude = metric._coordinate(
            row.get("LongitudeDegrees"), "LongitudeDegrees", row_number, -180.0, 180.0
        )
        key = (phone, timestamp)
        if key in seen:
            raise NativeGnssPdcScoreError(
                f"ground-truth row {row_number}: duplicate key {key!r}"
            )
        seen.add(key)
        rows.append(metric.CoordinateRow(phone, timestamp, latitude, longitude, row_number))
    if not rows:
        raise NativeGnssPdcScoreError("ground truth is empty")
    return rows, digest


def _nearest_one_to_one(
    predictions: list[metric.CoordinateRow],
    truth: list[metric.CoordinateRow],
) -> tuple[list[tuple[metric.CoordinateRow, metric.CoordinateRow, int]], dict[str, Any]]:
    truth_by_phone: dict[str, list[metric.CoordinateRow]] = {}
    for row in truth:
        truth_by_phone.setdefault(row.phone, []).append(row)
    for rows in truth_by_phone.values():
        rows.sort(key=lambda row: row.timestamp)
    used: dict[str, set[int]] = {phone: set() for phone in truth_by_phone}
    matches: list[tuple[metric.CoordinateRow, metric.CoordinateRow, int]] = []
    unmatched_predictions: list[metric.CoordinateRow] = []
    for prediction in sorted(predictions, key=lambda row: (row.phone, row.timestamp, row.source_line)):
        candidates = truth_by_phone.get(prediction.phone, [])
        times = [row.timestamp for row in candidates]
        lower = bisect.bisect_left(times, prediction.timestamp - TIMESTAMP_TOLERANCE_MS)
        upper = bisect.bisect_right(times, prediction.timestamp + TIMESTAMP_TOLERANCE_MS)
        available = [index for index in range(lower, upper) if index not in used[prediction.phone]]
        if not available:
            # A raw route can contain one more epoch than its exported truth
            # file (the known train route has a trailing native epoch).  Keep
            # that row explicit in coverage instead of turning a scoring
            # schema mismatch into a candidate failure.
            unmatched_predictions.append(prediction)
            continue
        distances = [abs(candidates[index].timestamp - prediction.timestamp) for index in available]
        minimum = min(distances)
        winners = [index for index, distance in zip(available, distances) if distance == minimum]
        if len(winners) != 1:
            raise NativeGnssPdcScoreError(
                f"ambiguous nearest truth timestamp for {prediction.phone!r}/{prediction.timestamp}"
            )
        index = winners[0]
        used[prediction.phone].add(index)
        reference = candidates[index]
        matches.append((prediction, reference, minimum))
    deltas = [delta for _, _, delta in matches]
    if not matches:
        raise NativeGnssPdcScoreError("no timestamps were aligned")
    return matches, {
        "prediction_rows": len(predictions),
        "truth_rows": len(truth),
        "matched_rows": len(matches),
        "unmatched_prediction_rows": len(predictions) - len(matches),
        "unmatched_prediction_examples": [
            {"phone": row.phone, "timestamp": row.timestamp}
            for row in unmatched_predictions[:10]
        ],
        "unmatched_truth_rows": len(truth) - len({(row.phone, row.timestamp) for _, row, _ in matches}),
        "tolerance_ms": TIMESTAMP_TOLERANCE_MS,
        "delta_ms": {
            "min": min(deltas),
            "median": statistics.median(deltas),
            "max": max(deltas),
        },
    }


def _score(matches: list[tuple[metric.CoordinateRow, metric.CoordinateRow, int]]) -> dict[str, Any]:
    distance_functions = {
        "wgs84_vincenty": metric._wgs84_horizontal_distance_m,
        "haversine_sphere": metric._haversine_horizontal_distance_m,
    }
    percentile_functions = {
        "linear_n_minus_1": metric._percentile_linear_n_minus_1,
        "nearest_rank_ceiling": metric._percentile_nearest_rank_ceiling,
    }
    by_phone: dict[str, dict[str, list[float]]] = {}
    for prediction, reference, _ in matches:
        phone_distances = by_phone.setdefault(
            prediction.phone, {name: [] for name in distance_functions}
        )
        for name, function in distance_functions.items():
            phone_distances[name].append(
                function(
                    prediction.latitude,
                    prediction.longitude,
                    reference.latitude,
                    reference.longitude,
                )
            )
    variants: dict[str, list[float]] = {}
    phones: dict[str, Any] = {}
    for phone in sorted(by_phone):
        phone_variants: dict[str, Any] = {}
        for distance_name, distances in by_phone[phone].items():
            for percentile_name, function in percentile_functions.items():
                p50 = function(distances, 0.50)
                p95 = function(distances, 0.95)
                variant = metric._score_variant_id(distance_name, percentile_name)
                phone_score = (p50 + p95) / 2.0
                variants.setdefault(variant, []).append(phone_score)
                phone_variants[variant] = {
                    "distance_variant": distance_name,
                    "percentile_variant": percentile_name,
                    "p50_m": p50,
                    "p95_m": p95,
                    "phone_score_m": phone_score,
                }
        phones[phone] = {
            "prediction_rows": len(next(iter(by_phone[phone].values()))),
            "score_variants": phone_variants,
        }
    return {
        "official_metric_m": None,
        "official_metric_status": metric.PRIMARY_SCORE_STATUS,
        "score_variants_m": {
            variant: sum(values) / len(values) for variant, values in sorted(variants.items())
        },
        "phone_count_scored": len(phones),
        "phones": phones,
    }


def evaluate(
    candidate_path: Path,
    run_manifest_path: Path,
    truth_path: Path,
    recovery_path: Path,
    output_path: Path,
    phone: str,
) -> dict[str, Any]:
    for path, label in (
        (candidate_path, "native keyed output"),
        (run_manifest_path, "native run manifest"),
        (truth_path, "ground truth"),
        (recovery_path, "score-recovery authorization"),
        (output_path, "score output"),
    ):
        _reject_mat_path(path, label)
    if output_path.exists() and output_path.resolve() not in {
        candidate_path.resolve(), run_manifest_path.resolve(), truth_path.resolve(), recovery_path.resolve()
    }:
        raise NativeGnssPdcScoreError(f"refusing to overwrite existing score output: {output_path}")
    recovery = _validate_recovery(recovery_path, candidate_path, run_manifest_path, phone)
    manifest = _validate_run_manifest(run_manifest_path, candidate_path, phone)
    try:
        predictions = metric._read_submission(candidate_path)
    except (OSError, ValueError) as exc:
        raise NativeGnssPdcScoreError(f"invalid native keyed output: {exc}") from exc
    truth, truth_sha256 = _read_truth_once(truth_path, phone)
    matches, alignment = _nearest_one_to_one(predictions, truth)
    report: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "status": "scored-evaluation-recovery",
        "candidate": {
            "route_device": phone,
            "keyed_output": {"path": str(candidate_path), "sha256": _sha256(candidate_path)},
            "run_manifest": {"path": str(run_manifest_path), "sha256": _sha256(run_manifest_path)},
            "run_manifest_schema": manifest["schema_version"],
        },
        "truth": {
            "path": str(truth_path),
            "sha256": truth_sha256,
            "role": "development-train",
            "opened_after_truth_free_seal": True,
            "read_method": "single-open-bytes-parse-and-hash",
        },
        "recovery": {
            "authorization": {"path": str(recovery_path), "sha256": _sha256(recovery_path)},
            "failed_exact_attempt_recorded": True,
            "exact_failure": recovery["exact_failure"],
            "candidate_regenerated": False,
            "solver_rerun": False,
            "parameter_selection": False,
        },
        "timestamp_alignment": {
            "contract": "deterministic one-to-one nearest truth timestamp, absolute delta <= 1000 ms",
            **alignment,
        },
        "metrics": _score(matches),
        "factor_telemetry": manifest.get("summary", {}),
        "policy": {
            "mat_inputs_used": False,
            "validation_truth_read_count": 0,
            "holdout_truth_read_count": 0,
            "test_truth_read_count": 0,
            "leaderboard_used_for_tuning": False,
            "external_mutation": False,
        },
    }
    _atomic_json(output_path, report)
    return report


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--run-manifest", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--recovery-authorization", type=Path, required=True)
    parser.add_argument("--phone", required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    try:
        report = evaluate(
            args.candidate,
            args.run_manifest,
            args.ground_truth,
            args.recovery_authorization,
            args.output_json,
            args.phone,
        )
    except (NativeGnssPdcScoreError, OSError, ValueError) as exc:
        print(f"native raw GNSS/PDC score rejected: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(report, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
