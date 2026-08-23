#!/usr/bin/env python3
"""Truth-score a libgnss++ smartphone POS and enforce R5 gates."""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
import os
from pathlib import Path
from statistics import median


SCHEMA_VERSION = "smartphone-gnss-signoff.v1"
GPS_EPOCH_UNIX_SECONDS = 315_964_800.0


def fail(message: str) -> "NoReturn":
    raise SystemExit(message)


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - index) + ordered[upper] * (index - lower)


def horizontal_error_m(lat: float, lon: float, truth_lat: float, truth_lon: float) -> float:
    radius = 6378137.0
    dlat = math.radians(lat - truth_lat)
    dlon = math.radians(lon - truth_lon)
    mean_lat = math.radians((lat + truth_lat) / 2.0)
    return radius * math.hypot(dlat, math.cos(mean_lat) * dlon)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--position", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--adapter-summary", type=Path, required=True)
    parser.add_argument("--solver-summary", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--gps-utc-leap-seconds", type=int, default=18)
    parser.add_argument("--match-tolerance-s", type=float, default=0.05)
    parser.add_argument("--require-availability-min", type=float)
    parser.add_argument("--require-truth-coverage-min", type=float)
    parser.add_argument("--require-horizontal-median-max", type=float)
    parser.add_argument("--require-horizontal-p95-max", type=float)
    parser.add_argument("--require-vertical-p95-max", type=float)
    parser.add_argument("--require-max-gap-s", type=float)
    return parser.parse_args()


def load_json(path: Path, label: str) -> dict[str, object]:
    if not path.is_file():
        fail(f"missing {label}: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        fail(f"invalid {label}: {exc}")
    if not isinstance(payload, dict):
        fail(f"{label} must contain a JSON object")
    return payload


def read_truth(path: Path) -> list[tuple[int, float, float, float]]:
    if not path.is_file():
        fail(f"missing ground truth: {path}")
    rows: list[tuple[int, float, float, float]] = []
    with path.open(encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees", "AltitudeMeters"}
        missing = required - set(reader.fieldnames or ())
        if missing:
            fail(f"ground truth missing fields: {', '.join(sorted(missing))}")
        for row_number, row in enumerate(reader, start=2):
            try:
                timestamp = int(row["UnixTimeMillis"])
                lat = float(row["LatitudeDegrees"])
                lon = float(row["LongitudeDegrees"])
                height = float(row["AltitudeMeters"])
            except (TypeError, ValueError) as exc:
                fail(f"truth row {row_number}: invalid numeric value: {exc}")
            if not all(math.isfinite(value) for value in (lat, lon, height)):
                fail(f"truth row {row_number}: non-finite position")
            if rows and timestamp <= rows[-1][0]:
                fail(f"truth row {row_number}: timestamps must be strictly increasing")
            rows.append((timestamp, lat, lon, height))
    if not rows:
        fail("ground truth is empty")
    return rows


def read_position(path: Path, leap_seconds: int) -> list[dict[str, object]]:
    if not path.is_file():
        fail(f"missing position file: {path}")
    epochs: list[dict[str, object]] = []
    for line_number, line in enumerate(path.read_text(encoding="ascii").splitlines(), start=1):
        if not line.strip() or line.lstrip().startswith("%"):
            continue
        fields = line.split()
        if len(fields) < 11:
            fail(f"position line {line_number}: expected at least 11 columns")
        try:
            week = int(fields[0])
            tow = float(fields[1])
            lat = float(fields[5])
            lon = float(fields[6])
            height = float(fields[7])
            status = int(fields[8])
            satellites = int(fields[9])
        except ValueError as exc:
            fail(f"position line {line_number}: invalid numeric value: {exc}")
        if not all(math.isfinite(value) for value in (tow, lat, lon, height)):
            fail(f"position line {line_number}: non-finite position or time")
        unix_ms = int(round((GPS_EPOCH_UNIX_SECONDS + week * 604800.0 + tow - leap_seconds) * 1000.0))
        if epochs and unix_ms <= int(epochs[-1]["unix_time_millis"]):
            fail(f"position line {line_number}: timestamps must be strictly increasing")
        epochs.append(
            {
                "week": week,
                "tow": tow,
                "unix_time_millis": unix_ms,
                "latitude_deg": lat,
                "longitude_deg": lon,
                "height_m": height,
                "status": status,
                "satellites": satellites,
            }
        )
    if not epochs:
        fail("position file contains no solution epochs")
    return epochs


def check_gate(name: str, actual: float | None, threshold: float | None, minimum: bool) -> dict[str, object] | None:
    if threshold is None:
        return None
    passed = actual is not None and (actual >= threshold if minimum else actual <= threshold)
    return {
        "name": name,
        "actual": actual,
        "operator": ">=" if minimum else "<=",
        "threshold": threshold,
        "passed": passed,
    }


def main() -> int:
    args = parse_args()
    if args.match_tolerance_s <= 0.0:
        fail("--match-tolerance-s must be positive")
    adapter = load_json(args.adapter_summary, "adapter summary")
    solver = load_json(args.solver_summary, "solver summary")
    if adapter.get("schema_version") != "smartphone-gnss-adapter.v1":
        fail("adapter summary schema is not smartphone-gnss-adapter.v1")
    selected_epochs = int(dict(adapter.get("observations", {})).get("epochs", 0))
    if selected_epochs <= 0:
        fail("adapter summary has no selected epochs")

    profile: dict[str, object] | None = None
    if args.profile is not None:
        profile = load_json(args.profile, "R5 profile")
        if profile.get("schema_version") != "smartphone-r5-profile.v1":
            fail("R5 profile schema is not smartphone-r5-profile.v1")
        explicit_thresholds = (
            args.require_availability_min,
            args.require_truth_coverage_min,
            args.require_horizontal_median_max,
            args.require_horizontal_p95_max,
            args.require_vertical_p95_max,
            args.require_max_gap_s,
        )
        if any(value is not None for value in explicit_thresholds):
            fail("do not override frozen thresholds when --profile is used")
        thresholds = dict(profile.get("thresholds", {}))
        try:
            args.require_availability_min = float(thresholds["availability_min"])
            args.require_truth_coverage_min = float(thresholds["truth_coverage_min"])
            args.require_horizontal_median_max = float(thresholds["horizontal_median_max"])
            args.require_horizontal_p95_max = float(thresholds["horizontal_p95_max"])
            args.require_vertical_p95_max = float(thresholds["vertical_p95_max"])
            args.require_max_gap_s = float(thresholds["max_gap_s"])
        except (KeyError, TypeError, ValueError) as exc:
            fail(f"R5 profile has invalid thresholds: {exc}")
        adapter_dataset = dict(adapter.get("dataset", {}))
        role = str(adapter_dataset.get("role", ""))
        datasets = dict(profile.get("datasets", {}))
        expected = dict(datasets.get(role, {}))
        if not expected or adapter_dataset.get("id") != expected.get("id"):
            fail(f"adapter dataset does not match frozen {role or 'unknown'} profile")
        adapter_inputs = dict(adapter.get("inputs", {}))
        device_input = dict(adapter_inputs.get("device_gnss", {}))
        truth_input = dict(adapter_inputs.get("ground_truth", {}))
        if device_input.get("sha256") != expected.get("device_gnss_sha256"):
            fail("adapter device_gnss hash does not match frozen profile")
        if truth_input.get("sha256") != expected.get("ground_truth_sha256"):
            fail("adapter ground_truth hash does not match frozen profile")

    truth = read_truth(args.ground_truth)
    truth_times = [row[0] for row in truth]
    positions = read_position(args.position, args.gps_utc_leap_seconds)
    tolerance_ms = args.match_tolerance_s * 1000.0
    horizontal: list[float] = []
    vertical_abs: list[float] = []
    matches: list[dict[str, object]] = []
    used_truth_indices: set[int] = set()
    for position in positions:
        timestamp = int(position["unix_time_millis"])
        insertion = bisect.bisect_left(truth_times, timestamp)
        candidates = [index for index in (insertion - 1, insertion) if 0 <= index < len(truth)]
        if not candidates:
            continue
        index = min(candidates, key=lambda candidate: abs(truth_times[candidate] - timestamp))
        delta_ms = abs(truth_times[index] - timestamp)
        if delta_ms > tolerance_ms or index in used_truth_indices:
            continue
        used_truth_indices.add(index)
        _, truth_lat, truth_lon, truth_height = truth[index]
        h_error = horizontal_error_m(
            float(position["latitude_deg"]), float(position["longitude_deg"]), truth_lat, truth_lon
        )
        v_error = float(position["height_m"]) - truth_height
        horizontal.append(h_error)
        vertical_abs.append(abs(v_error))
        matches.append(
            {
                "solution_unix_time_millis": timestamp,
                "truth_unix_time_millis": truth_times[index],
                "time_delta_ms": delta_ms,
                "horizontal_error_m": h_error,
                "vertical_error_m": v_error,
                "status": position["status"],
                "satellites": position["satellites"],
            }
        )
    if not matches:
        fail("no position epochs matched ground truth")

    position_times = [int(position["unix_time_millis"]) for position in positions]
    gaps = [
        (current - previous) / 1000.0
        for previous, current in zip(position_times, position_times[1:])
    ]
    availability = len(positions) / selected_epochs
    truth_coverage = len(matches) / selected_epochs
    metrics = {
        "selected_epochs": selected_epochs,
        "solution_epochs": len(positions),
        "truth_matched_epochs": len(matches),
        "solution_availability_ratio": availability,
        "truth_scored_coverage_ratio": truth_coverage,
        "solution_truth_match_ratio": len(matches) / len(positions),
        "horizontal_median_m": median(horizontal),
        "horizontal_p95_m": percentile(horizontal, 0.95),
        "horizontal_max_m": max(horizontal),
        "vertical_median_abs_m": median(vertical_abs),
        "vertical_p95_abs_m": percentile(vertical_abs, 0.95),
        "vertical_max_abs_m": max(vertical_abs),
        "max_solution_gap_s": max(gaps) if gaps else 0.0,
        "status_population": {
            str(status): sum(1 for position in positions if int(position["status"]) == status)
            for status in sorted({int(position["status"]) for position in positions})
        },
    }
    gates = [
        gate
        for gate in (
            check_gate("solution_availability_ratio", availability, args.require_availability_min, True),
            check_gate("truth_scored_coverage_ratio", truth_coverage, args.require_truth_coverage_min, True),
            check_gate("horizontal_median_m", metrics["horizontal_median_m"], args.require_horizontal_median_max, False),
            check_gate("horizontal_p95_m", metrics["horizontal_p95_m"], args.require_horizontal_p95_max, False),
            check_gate("vertical_p95_abs_m", metrics["vertical_p95_abs_m"], args.require_vertical_p95_max, False),
            check_gate("max_solution_gap_s", metrics["max_solution_gap_s"], args.require_max_gap_s, False),
        )
        if gate is not None
    ]
    decision = "pass" if gates and all(bool(gate["passed"]) for gate in gates) else "fail" if gates else "ungated"

    args.output_dir.mkdir(parents=True, exist_ok=True)
    matches_path = args.output_dir / "matches.csv"
    with matches_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(matches[0]))
        writer.writeheader()
        writer.writerows(matches)
    summary = {
        "schema_version": SCHEMA_VERSION,
        "decision": decision,
        "metrics": metrics,
        "gates": gates,
        "time_contract": {
            "position": "GPST week/tow",
            "truth": "Unix UTC milliseconds",
            "gps_utc_leap_seconds": args.gps_utc_leap_seconds,
            "match_tolerance_s": args.match_tolerance_s,
        },
        "inputs": {
            "position": {"path": str(args.position), "sha256": file_sha256(args.position)},
            "ground_truth": {"path": str(args.ground_truth), "sha256": file_sha256(args.ground_truth)},
            "adapter_summary": {"path": str(args.adapter_summary), "sha256": file_sha256(args.adapter_summary)},
            "solver_summary": {"path": str(args.solver_summary), "sha256": file_sha256(args.solver_summary)},
            "profile": (
                {"path": str(args.profile), "sha256": file_sha256(args.profile)}
                if args.profile is not None
                else None
            ),
        },
        "profile_id": profile.get("profile_id") if profile is not None else None,
        "solver_summary": solver,
        "artifacts": {"matches_csv": str(matches_path)},
    }
    summary_path = args.output_dir / "signoff_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Smartphone GNSS sign-off {decision}: {summary_path}")
    return 0 if decision != "fail" else 1


if __name__ == "__main__":
    raise SystemExit(main())
