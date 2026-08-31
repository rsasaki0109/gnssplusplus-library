#!/usr/bin/env python3
"""Truth-scoring gate for the frozen Doppler position-update lane.

The candidate itself never sees truth.  This evaluator reads truth only after
the caller has generated and hash-sealed the truth-free candidate artifacts.
It supports the three fixed train routes and an explicitly supplied fresh
validation route; no holdout route is accepted.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_doppler_position as doppler  # noqa: E402
import gnss_smartphone_kaggle as kaggle  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402


SCHEMA_VERSION = "smartphone-doppler-position-evaluation.v1"
SELECTION_SCHEMA = "smartphone-r5-gsdc2023-top-solution-feasibility-audit.v1"
SKIP_EPOCHS = 1
MATCH_TOLERANCE_MS = 100
TRAIN_IDS = (
    "2021-01-04-21-50-us-ca-e1highway280driveroutea/mi8",
    "2022-08-04-20-07-us-ca-sjc-q/pixel5",
    "2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro",
)
FRESH_VALIDATION_ID = "2023-09-07-18-59-us-ca/pixel5"
FUTURE_HOLDOUT_ID = "2023-09-06-00-01-us-ca-routen/pixel6pro"
FORBIDDEN_IDS = {
    "2023-09-06-22-49-us-ca-routebb1/pixel7pro",
    "2023-09-06-00-01-us-ca-routen/pixel6pro",
}
DIAGNOSTIC_KEYS = (
    "haversine_sphere__linear_n_minus_1",
    "haversine_sphere__nearest_rank_ceiling",
    "wgs84_vincenty__linear_n_minus_1",
    "wgs84_vincenty__nearest_rank_ceiling",
)


class DopplerEvaluationError(ValueError):
    """Raised when the frozen evaluation contract is not satisfied."""


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise DopplerEvaluationError(f"missing artifact: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    doppler._atomic_write(
        path, (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
    )


def _load_selection(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DopplerEvaluationError(f"invalid selection record: {path}") from exc
    if not isinstance(payload, dict) or payload.get("schema_version") != SELECTION_SCHEMA:
        raise DopplerEvaluationError("selection record schema is invalid")
    if payload.get("status") != "selection-frozen-before-new-implementation":
        raise DopplerEvaluationError("selection record is not frozen")
    candidate = payload.get("frozen_candidate")
    if not isinstance(candidate, dict) or candidate.get("id") != "doppler_position_update_v1":
        raise DopplerEvaluationError("candidate differs from frozen Doppler candidate")
    split = payload.get("frozen_split")
    if not isinstance(split, dict) or split.get("future_holdout") in (None, ""):
        raise DopplerEvaluationError("selection record has no future holdout")
    if split.get("future_holdout") != FUTURE_HOLDOUT_ID:
        raise DopplerEvaluationError("future holdout differs from frozen contract")
    if split.get("fresh_validation") != FRESH_VALIDATION_ID:
        raise DopplerEvaluationError("fresh validation differs from frozen contract")
    return payload


def _parse_route_spec(raw: str) -> tuple[str, Path, Path, Path, Path]:
    parts = raw.split("|", 4)
    if len(parts) != 5 or not all(parts):
        raise DopplerEvaluationError(
            "route spec must be dataset_id|baseline.pos|candidate.pos|device_gnss.csv|ground_truth.csv"
        )
    dataset_id, baseline, candidate, device, truth = parts
    if dataset_id in FORBIDDEN_IDS or dataset_id.endswith("/pixel6pro") and dataset_id.startswith("2023-09-06-00-01"):
        raise DopplerEvaluationError(f"holdout route is forbidden: {dataset_id}")
    return dataset_id, Path(baseline), Path(candidate), Path(device), Path(truth)


def _match_truth(
    timestamp: int,
    truth: dict[int, tuple[float, float, float]],
    tolerance_ms: int,
) -> tuple[float, float, float] | None:
    if timestamp in truth:
        return truth[timestamp]
    nearest = min(truth, key=lambda candidate: abs(candidate - timestamp))
    if abs(nearest - timestamp) <= tolerance_ms:
        return truth[nearest]
    return None


def _percentile(values: list[float], fraction: float) -> float | None:
    return (
        kaggle._percentile_linear_n_minus_1(values, fraction) if values else None
    )


def _metrics(
    position_path: Path,
    device_path: Path,
    truth_path: Path,
    *,
    truth_cache: dict[str, dict[int, tuple[float, float, float]]],
) -> dict[str, Any]:
    positions = smoother._read_positions(position_path, smoother.DEFAULT_GPS_UTC_LEAP_SECONDS)
    epochs = smoother._read_device_epochs(device_path, SKIP_EPOCHS)
    if not positions or not epochs:
        raise DopplerEvaluationError("position/device input is empty")
    position_by_timestamp = {row.timestamp_ms: row for row in positions}
    if set(position_by_timestamp) - set(epochs):
        raise DopplerEvaluationError("candidate position has a non-device timestamp")
    truth_key = str(truth_path)
    truth = truth_cache.setdefault(truth_key, smoother_eval._read_truth(truth_path))
    horizontal_wgs84: list[float] = []
    horizontal_haversine: list[float] = []
    vertical: list[float] = []
    for timestamp in epochs:
        row = position_by_timestamp.get(timestamp)
        if row is None:
            continue
        reference = _match_truth(timestamp, truth, MATCH_TOLERANCE_MS)
        if reference is None:
            continue
        latitude, longitude, height = reference
        horizontal_wgs84.append(
            kaggle._wgs84_horizontal_distance_m(
                row.latitude, row.longitude, latitude, longitude
            )
        )
        horizontal_haversine.append(
            kaggle._haversine_horizontal_distance_m(
                row.latitude, row.longitude, latitude, longitude
            )
        )
        vertical.append(abs(row.height - height))
    if not horizontal_wgs84:
        raise DopplerEvaluationError("no candidate epochs matched ground truth")
    wgs84_p50 = _percentile(horizontal_wgs84, 0.50)
    wgs84_p95 = _percentile(horizontal_wgs84, 0.95)
    hav_p50 = _percentile(horizontal_haversine, 0.50)
    hav_p95 = _percentile(horizontal_haversine, 0.95)
    diagnostics = {
        "wgs84_vincenty__linear_n_minus_1": (wgs84_p50 + wgs84_p95) / 2.0,
        "wgs84_vincenty__nearest_rank_ceiling": (
            kaggle._percentile_nearest_rank_ceiling(horizontal_wgs84, 0.50)
            + kaggle._percentile_nearest_rank_ceiling(horizontal_wgs84, 0.95)
        )
        / 2.0,
        "haversine_sphere__linear_n_minus_1": (hav_p50 + hav_p95) / 2.0,
        "haversine_sphere__nearest_rank_ceiling": (
            kaggle._percentile_nearest_rank_ceiling(horizontal_haversine, 0.50)
            + kaggle._percentile_nearest_rank_ceiling(horizontal_haversine, 0.95)
        )
        / 2.0,
    }
    return {
        "device_epochs": len(epochs),
        "position_epochs": len(positions),
        "matched_epochs": len(horizontal_wgs84),
        "availability_ratio": len(positions) / len(epochs),
        "truth_coverage_ratio": len(horizontal_wgs84) / len(epochs),
        "horizontal_wgs84_m": {"p50_m": wgs84_p50, "p95_m": wgs84_p95},
        "horizontal_haversine_m": {"p50_m": hav_p50, "p95_m": hav_p95},
        "vertical_p95_abs_m": _percentile(vertical, 0.95),
        "kaggle_diagnostic_score_variants_m": diagnostics,
        "kaggle_diagnostic_mean_m": sum(diagnostics.values()) / len(diagnostics),
    }


def _flatten_metrics(
    route_values: list[tuple[dict[str, Any], dict[str, Any]]],
    truth_cache: dict[str, dict[int, tuple[float, float, float]]],
) -> dict[str, Any]:
    """Aggregate route metrics by replaying distance arrays from saved truth."""

    # Aggregate gates use a sample-weighted error population.  Re-read the
    # route inputs only; this is still the same single truth read per route.
    del truth_cache
    all_wgs84: list[float] = []
    all_haversine: list[float] = []
    all_vertical: list[float] = []
    total_epochs = total_candidate = total_matched = 0
    for baseline, candidate in route_values:
        # The caller stores arrays privately under the internal keys.
        all_wgs84.extend(candidate.pop("_wgs84_errors"))
        all_haversine.extend(candidate.pop("_haversine_errors"))
        all_vertical.extend(candidate.pop("_vertical_errors"))
        total_epochs += int(candidate["device_epochs"])
        total_candidate += int(candidate["position_epochs"])
        total_matched += int(candidate["matched_epochs"])
        del baseline
    wgs84_p50 = _percentile(all_wgs84, 0.50)
    wgs84_p95 = _percentile(all_wgs84, 0.95)
    hav_p50 = _percentile(all_haversine, 0.50)
    hav_p95 = _percentile(all_haversine, 0.95)
    diagnostics = {
        "wgs84_vincenty__linear_n_minus_1": (wgs84_p50 + wgs84_p95) / 2.0,
        "wgs84_vincenty__nearest_rank_ceiling": (
            kaggle._percentile_nearest_rank_ceiling(all_wgs84, 0.50)
            + kaggle._percentile_nearest_rank_ceiling(all_wgs84, 0.95)
        )
        / 2.0,
        "haversine_sphere__linear_n_minus_1": (hav_p50 + hav_p95) / 2.0,
        "haversine_sphere__nearest_rank_ceiling": (
            kaggle._percentile_nearest_rank_ceiling(all_haversine, 0.50)
            + kaggle._percentile_nearest_rank_ceiling(all_haversine, 0.95)
        )
        / 2.0,
    }
    return {
        "device_epochs": total_epochs,
        "position_epochs": total_candidate,
        "matched_epochs": total_matched,
        "availability_ratio": total_candidate / total_epochs,
        "truth_coverage_ratio": total_matched / total_epochs,
        "horizontal_wgs84_m": {"p50_m": wgs84_p50, "p95_m": wgs84_p95},
        "horizontal_haversine_m": {"p50_m": hav_p50, "p95_m": hav_p95},
        "vertical_p95_abs_m": _percentile(all_vertical, 0.95),
        "kaggle_diagnostic_score_variants_m": diagnostics,
        "kaggle_diagnostic_mean_m": sum(diagnostics.values()) / len(diagnostics),
    }


def _distance_arrays(
    position_path: Path,
    device_path: Path,
    truth_path: Path,
    truth_cache: dict[str, dict[int, tuple[float, float, float]]],
) -> tuple[list[float], list[float], list[float]]:
    positions = smoother._read_positions(position_path, smoother.DEFAULT_GPS_UTC_LEAP_SECONDS)
    epochs = smoother._read_device_epochs(device_path, SKIP_EPOCHS)
    truth = truth_cache.setdefault(str(truth_path), smoother_eval._read_truth(truth_path))
    by_timestamp = {row.timestamp_ms: row for row in positions}
    wgs84: list[float] = []
    haversine: list[float] = []
    vertical: list[float] = []
    for timestamp in epochs:
        row = by_timestamp.get(timestamp)
        reference = _match_truth(timestamp, truth, MATCH_TOLERANCE_MS)
        if row is None or reference is None:
            continue
        lat, lon, height = reference
        wgs84.append(kaggle._wgs84_horizontal_distance_m(row.latitude, row.longitude, lat, lon))
        haversine.append(kaggle._haversine_horizontal_distance_m(row.latitude, row.longitude, lat, lon))
        vertical.append(abs(row.height - height))
    return wgs84, haversine, vertical


def _with_arrays(
    metrics: dict[str, Any],
    arrays: tuple[list[float], list[float], list[float]],
) -> dict[str, Any]:
    output = dict(metrics)
    output["_wgs84_errors"], output["_haversine_errors"], output["_vertical_errors"] = arrays
    return output


def _non_regression(candidate: dict[str, Any], baseline: dict[str, Any]) -> list[str]:
    failures: list[str] = []
    comparisons = (
        (("availability_ratio",), "availability_regression"),
        (("truth_coverage_ratio",), "coverage_regression"),
        (("horizontal_wgs84_m", "p50_m"), "h_p50_regression"),
        (("horizontal_wgs84_m", "p95_m"), "h_p95_regression"),
        (("vertical_p95_abs_m",), "v_p95_regression"),
    )
    for path, label in comparisons:
        left: Any = candidate
        right: Any = baseline
        for key in path:
            left = left[key]
            right = right[key]
        if float(left) > float(right) and not math.isclose(
            float(left), float(right), rel_tol=0.0, abs_tol=1.0e-9
        ):
            failures.append(label)
    for key in DIAGNOSTIC_KEYS:
        if candidate["kaggle_diagnostic_score_variants_m"][key] > baseline["kaggle_diagnostic_score_variants_m"][key] and not math.isclose(
            candidate["kaggle_diagnostic_score_variants_m"][key],
            baseline["kaggle_diagnostic_score_variants_m"][key],
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            failures.append(key + "_regression")
    return failures


def _strict_train_gate(candidate: dict[str, Any], baseline: dict[str, Any]) -> list[str]:
    failures = _non_regression(candidate, baseline)
    if not candidate["horizontal_wgs84_m"]["p95_m"] < baseline["horizontal_wgs84_m"]["p95_m"] - 1.0e-9:
        failures.append("aggregate_h_p95_not_strictly_improved")
    if not candidate["kaggle_diagnostic_mean_m"] < baseline["kaggle_diagnostic_mean_m"] - 1.0e-9:
        failures.append("aggregate_diagnostic_mean_not_strictly_improved")
    return sorted(set(failures))


def evaluate_route(
    dataset_id: str,
    baseline_path: Path,
    candidate_path: Path,
    device_path: Path,
    truth_path: Path,
    truth_cache: dict[str, dict[int, tuple[float, float, float]]],
) -> dict[str, Any]:
    baseline = _metrics(baseline_path, device_path, truth_path, truth_cache=truth_cache)
    candidate = _metrics(candidate_path, device_path, truth_path, truth_cache=truth_cache)
    candidate_arrays = _distance_arrays(candidate_path, device_path, truth_path, truth_cache)
    baseline_arrays = _distance_arrays(baseline_path, device_path, truth_path, truth_cache)
    return {
        "dataset_id": dataset_id,
        "baseline": _with_arrays(baseline, baseline_arrays),
        "candidate": _with_arrays(candidate, candidate_arrays),
        "gate_failures": _non_regression(candidate, baseline),
        "inputs": {
            "baseline": {"path": str(baseline_path), "sha256": _sha256(baseline_path)},
            "candidate": {"path": str(candidate_path), "sha256": _sha256(candidate_path)},
            "device_gnss": {"path": str(device_path), "sha256": _sha256(device_path)},
            "ground_truth": {"path": str(truth_path), "sha256": _sha256(truth_path)},
        },
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="gnss smartphone-doppler-position-eval",
        description="Score frozen truth-free Doppler candidate artifacts on train or fresh validation routes.",
    )
    parser.add_argument("--selection-record", type=Path, required=True)
    parser.add_argument("--route", action="append", required=True, help="dataset|baseline.pos|candidate.pos|device.csv|truth.csv")
    parser.add_argument("--role", choices=("train", "fresh-validation"), required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    started = time.perf_counter()
    selection = _load_selection(args.selection_record)
    specs = [_parse_route_spec(raw) for raw in args.route]
    ids = tuple(spec[0] for spec in specs)
    expected = TRAIN_IDS if args.role == "train" else (FRESH_VALIDATION_ID,)
    if ids != expected:
        raise SystemExit(f"route IDs must exactly match frozen {args.role} role: {expected}")
    truth_cache: dict[str, dict[int, tuple[float, float, float]]] = {}
    routes: list[dict[str, Any]] = []
    for dataset_id, baseline, candidate, device, truth in specs:
        routes.append(evaluate_route(dataset_id, baseline, candidate, device, truth, truth_cache))
    # Build aggregate error populations from the already-opened per-route truth
    # arrays.  No second truth read occurs because ``truth_cache`` is shared.
    baseline_arrays = ([], [], [])
    candidate_arrays = ([], [], [])
    for route in routes:
        for target, holder in ((route["baseline"], baseline_arrays), (route["candidate"], candidate_arrays)):
            holder[0].extend(target.pop("_wgs84_errors"))
            holder[1].extend(target.pop("_haversine_errors"))
            holder[2].extend(target.pop("_vertical_errors"))
    def aggregate_from_arrays(source: tuple[list[float], list[float], list[float]]) -> dict[str, Any]:
        wgs, hav, vert = source
        wp50, wp95 = _percentile(wgs, .5), _percentile(wgs, .95)
        hp50, hp95 = _percentile(hav, .5), _percentile(hav, .95)
        diagnostics = {
            "wgs84_vincenty__linear_n_minus_1": (wp50 + wp95) / 2.0,
            "wgs84_vincenty__nearest_rank_ceiling": (kaggle._percentile_nearest_rank_ceiling(wgs,.5)+kaggle._percentile_nearest_rank_ceiling(wgs,.95))/2.0,
            "haversine_sphere__linear_n_minus_1": (hp50 + hp95) / 2.0,
            "haversine_sphere__nearest_rank_ceiling": (kaggle._percentile_nearest_rank_ceiling(hav,.5)+kaggle._percentile_nearest_rank_ceiling(hav,.95))/2.0,
        }
        total_epochs = sum(r["candidate"]["device_epochs"] for r in routes)
        total_positions = sum(r["candidate"]["position_epochs"] for r in routes)
        total_matched = len(wgs)
        return {
            "device_epochs": total_epochs,
            "position_epochs": total_positions,
            "matched_epochs": total_matched,
            "availability_ratio": total_positions / total_epochs,
            "truth_coverage_ratio": total_matched / total_epochs,
            "horizontal_wgs84_m": {"p50_m": wp50, "p95_m": wp95},
            "horizontal_haversine_m": {"p50_m": hp50, "p95_m": hp95},
            "vertical_p95_abs_m": _percentile(vert,.95),
            "kaggle_diagnostic_score_variants_m": diagnostics,
            "kaggle_diagnostic_mean_m": sum(diagnostics.values())/len(diagnostics),
        }
    # This role is single-phone per route; recompute candidate position counts
    # independently when building aggregate to avoid candidate/baseline aliasing.
    baseline_aggregate = aggregate_from_arrays(baseline_arrays)
    candidate_aggregate = aggregate_from_arrays(candidate_arrays)
    if args.role == "train":
        aggregate_failures = _strict_train_gate(candidate_aggregate, baseline_aggregate)
    else:
        aggregate_failures = _non_regression(candidate_aggregate, baseline_aggregate)
        if not candidate_aggregate["horizontal_wgs84_m"]["p95_m"] < baseline_aggregate["horizontal_wgs84_m"]["p95_m"] - 1.0e-9:
            aggregate_failures.append("aggregate_h_p95_not_strictly_improved")
    for route in routes:
        route["gate_failures"] = sorted(set(route["gate_failures"]))
    payload = {
        "schema_version": SCHEMA_VERSION,
        "role": args.role,
        "selection_record": {"path": str(args.selection_record), "sha256": _sha256(args.selection_record)},
        "candidate_id": selection["frozen_candidate"]["id"],
        "truth_open_count": len(truth_cache),
        "truth_free_artifacts_preceded_truth": True,
        "public_private_scores_used_for_tuning": False,
        "routes": routes,
        "aggregate": {"baseline": baseline_aggregate, "candidate": candidate_aggregate, "gate_failures": sorted(set(aggregate_failures))},
        "passed": not aggregate_failures and all(not route["gate_failures"] for route in routes),
        "timing": {"wall_time_s": time.perf_counter() - started},
        "holdout_opened": False,
        "production_default_changed": False,
    }
    _atomic_json(args.output_json, payload)
    manifest = args.output_json.with_name(args.output_json.stem + "_manifest.json")
    _atomic_json(
        manifest,
        {
            "schema_version": SCHEMA_VERSION + ".manifest",
            "record": {"path": str(args.output_json), "sha256": _sha256(args.output_json)},
            "selection_record_sha256": _sha256(args.selection_record),
            "truth_open_count": len(truth_cache),
            "passed": payload["passed"],
            "holdout_opened": False,
        },
    )
    print(f"Doppler evaluation: {args.output_json}")
    print(f"Doppler evaluation manifest: {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
