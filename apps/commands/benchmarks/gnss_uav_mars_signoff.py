#!/usr/bin/env python3
"""Truth-score an R6 UAV flight and publish separate consumer decisions."""

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


SCHEMA_VERSION = "uav-mars-signoff.v1"
GPS_EPOCH_UNIX_SECONDS = 315_964_800.0


def fail(message: str) -> "NoReturn":
    raise SystemExit(message)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path, label: str) -> dict[str, object]:
    if not path.is_file():
        fail(f"missing {label}: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        fail(f"invalid {label}: {exc}")
    if not isinstance(value, dict):
        fail(f"{label} must contain a JSON object")
    return value


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower, upper = math.floor(index), math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - index) + ordered[upper] * (index - lower)


def horizontal_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    radius = 6_378_137.0
    mean_lat = math.radians((lat1 + lat2) / 2.0)
    return radius * math.hypot(
        math.radians(lat1 - lat2), math.cos(mean_lat) * math.radians(lon1 - lon2)
    )


def wrapped_error_deg(first: float, second: float) -> float:
    return abs((first - second + 180.0) % 360.0 - 180.0)


def quaternion_clockwise_enu_yaw_deg(row: dict[str, str]) -> float:
    x, y, z, w = (float(row[key]) for key in ("qx", "qy", "qz", "qw"))
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or abs(norm - 1.0) > 1e-3:
        fail("attitude contains a non-unit quaternion")
    yaw_ccw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return (-yaw_ccw) % 360.0


def read_csv(path: Path, fields: set[str], label: str) -> list[dict[str, str]]:
    if not path.is_file():
        fail(f"missing {label}: {path}")
    with path.open(encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        missing = fields - set(reader.fieldnames or ())
        if missing:
            fail(f"{label} missing fields: {', '.join(sorted(missing))}")
        rows = list(reader)
    if not rows:
        fail(f"{label} is empty")
    return rows


def read_position(path: Path, leap_seconds: int) -> list[dict[str, float | int]]:
    rows: list[dict[str, float | int]] = []
    for number, line in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
        if not line.strip() or line.lstrip().startswith("%"):
            continue
        fields = line.split()
        if len(fields) < 11:
            fail(f"position line {number}: expected at least 11 columns")
        try:
            week, tow = int(fields[0]), float(fields[1])
            lat, lon, height = float(fields[5]), float(fields[6]), float(fields[7])
            status, satellites = int(fields[8]), int(fields[9])
        except ValueError as exc:
            fail(f"position line {number}: invalid numeric value: {exc}")
        unix_s = GPS_EPOCH_UNIX_SECONDS + week * 604_800.0 + tow - leap_seconds
        if rows and unix_s <= float(rows[-1]["unix_s"]):
            fail(f"position line {number}: timestamps are not strictly increasing")
        rows.append({"unix_s": unix_s, "lat": lat, "lon": lon, "height": height, "status": status, "satellites": satellites})
    if not rows:
        fail("position file contains no solutions")
    return rows


def motion(values: list[tuple[float, float, float, float]]) -> list[dict[str, float]]:
    result: list[dict[str, float]] = []
    for index, (stamp, lat, lon, height) in enumerate(values):
        before = max(0, index - 1)
        after = min(len(values) - 1, index + 1)
        duration = values[after][0] - values[before][0]
        if duration <= 0.0:
            fail("truth timestamps are not strictly increasing")
        horizontal = horizontal_m(values[after][1], values[after][2], values[before][1], values[before][2]) / duration
        vertical = (values[after][3] - values[before][3]) / duration
        result.append({"horizontal_speed_mps": horizontal, "vertical_speed_mps": vertical})
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--position", type=Path, required=True)
    parser.add_argument("--truth", type=Path, required=True)
    parser.add_argument("--attitude", type=Path, required=True)
    parser.add_argument("--rtk-yaw", type=Path, required=True)
    parser.add_argument("--rtk-status", type=Path, required=True)
    parser.add_argument("--adapter-summary", type=Path, required=True)
    parser.add_argument("--solver-summary", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    profile = load_json(args.profile, "R6 profile")
    adapter = load_json(args.adapter_summary, "adapter summary")
    solver = load_json(args.solver_summary, "solver summary")
    if profile.get("schema_version") != "uav-r6-profile.v1" or adapter.get("schema_version") != "uav-mars-adapter.v1":
        fail("profile or adapter schema mismatch")
    if dict(adapter.get("profile", {})).get("sha256") != sha256(args.profile):
        fail("adapter was not generated with this frozen profile")
    selected_epochs = int(dict(adapter.get("observations", {})).get("epochs", 0))
    if selected_epochs <= 0 or int(solver.get("processed_epochs", -1)) != selected_epochs:
        fail("solver epoch count does not match adapter")
    role = str(dict(adapter.get("dataset", {})).get("role", ""))
    expected = dict(dict(profile.get("datasets", {})).get(role, {}))
    if not expected or expected.get("id") != dict(adapter.get("dataset", {})).get("id"):
        fail("adapter dataset does not match the profile")

    truth_rows = read_csv(args.truth, {"unix_time_ns", "latitude_deg", "longitude_deg", "ellipsoidal_height_m"}, "truth")
    truth = [(int(row["unix_time_ns"]) / 1e9, float(row["latitude_deg"]), float(row["longitude_deg"]), float(row["ellipsoidal_height_m"])) for row in truth_rows]
    truth_times = [row[0] for row in truth]
    truth_motion = motion(truth)
    positions = read_position(args.position, int(dict(profile["time_contract"]).get("gps_utc_leap_seconds", 18)))
    yaw_rows = read_csv(args.rtk_yaw, {"unix_time_ns", "yaw_degrees"}, "RTK yaw")
    yaw_times = [int(row["unix_time_ns"]) / 1e9 for row in yaw_rows]
    yaw_values = [float(row["yaw_degrees"]) for row in yaw_rows]
    yaw_rates: list[float] = []
    for index, value in enumerate(yaw_values):
        before, after = max(0, index - 1), min(len(yaw_values) - 1, index + 1)
        duration = yaw_times[after] - yaw_times[before]
        if duration <= 0.0:
            fail("RTK yaw timestamps are not strictly increasing")
        signed_delta = (yaw_values[after] - yaw_values[before] + 180.0) % 360.0 - 180.0
        yaw_rates.append(signed_delta / duration)
    segmentation = dict(profile.get("segmentation", {}))
    hover_speed = float(segmentation.get("hover_horizontal_speed_max_mps", 1.0))
    cruise_speed = float(segmentation.get("cruise_horizontal_speed_min_mps", 3.0))
    vertical_speed = float(segmentation.get("climb_descent_vertical_speed_min_mps", 1.0))
    turn_rate = float(segmentation.get("turn_yaw_rate_min_degps", 10.0))
    landing_s = float(segmentation.get("landing_tail_duration_s", 30.0))
    max_span = float(segmentation.get("truth_interpolation_span_max_s", 0.25))
    errors: list[dict[str, object]] = []
    for solution in positions:
        stamp = float(solution["unix_s"])
        index = bisect.bisect_right(truth_times, stamp) - 1
        if index < 0 or index + 1 >= len(truth):
            continue
        left, right = truth[index], truth[index + 1]
        span = right[0] - left[0]
        if span <= 0.0 or span > max_span:
            continue
        fraction = (stamp - left[0]) / span
        interpolated = tuple(left[column] + fraction * (right[column] - left[column]) for column in (1, 2, 3))
        h_error = horizontal_m(float(solution["lat"]), float(solution["lon"]), interpolated[0], interpolated[1])
        v_error = float(solution["height"]) - interpolated[2]
        horizontal_speed = truth_motion[index]["horizontal_speed_mps"] + fraction * (truth_motion[index + 1]["horizontal_speed_mps"] - truth_motion[index]["horizontal_speed_mps"])
        vertical = truth_motion[index]["vertical_speed_mps"] + fraction * (truth_motion[index + 1]["vertical_speed_mps"] - truth_motion[index]["vertical_speed_mps"])
        yaw_index = min(
            range(max(0, bisect.bisect_left(yaw_times, stamp) - 1), min(len(yaw_times), bisect.bisect_left(yaw_times, stamp) + 1)),
            key=lambda item: abs(yaw_times[item] - stamp),
        )
        populations = ["whole_route"]
        if horizontal_speed <= hover_speed and abs(vertical) < vertical_speed:
            populations.append("hover")
        if horizontal_speed >= cruise_speed and abs(vertical) < vertical_speed:
            populations.append("cruise")
        if abs(vertical) >= vertical_speed:
            populations.append("climb_descent")
        if abs(yaw_rates[yaw_index]) >= turn_rate:
            populations.append("turn")
        if stamp >= truth[-1][0] - landing_s:
            populations.append("landing")
        errors.append({"unix_time_s": stamp, "horizontal_error_m": h_error, "vertical_error_m": v_error, "populations": populations})
    if not errors:
        fail("no solutions fall inside continuous truth intervals")

    attitude = read_csv(args.attitude, {"unix_time_ns", "frame_id", "qx", "qy", "qz", "qw"}, "attitude")
    attitude_times = [int(row["unix_time_ns"]) for row in attitude]
    yaw_errors: list[float] = []
    yaw_time_deltas: list[float] = []
    for row in yaw_rows:
        stamp = int(row["unix_time_ns"])
        insertion = bisect.bisect_left(attitude_times, stamp)
        candidates = [index for index in (insertion - 1, insertion) if 0 <= index < len(attitude)]
        index = min(candidates, key=lambda item: abs(attitude_times[item] - stamp))
        yaw_time_deltas.append(abs(attitude_times[index] - stamp) / 1e9)
        yaw_errors.append(wrapped_error_deg(quaternion_clockwise_enu_yaw_deg(attitude[index]), float(row["yaw_degrees"])))

    status_rows = read_csv(args.rtk_status, {"kind", "unix_time_ns", "status"}, "RTK status")
    status_ratio = {}
    for kind in ("position", "yaw"):
        values = [int(row["status"]) for row in status_rows if row["kind"] == kind]
        if not values:
            fail(f"RTK status contains no {kind} rows")
        status_ratio[kind] = sum(value == 50 for value in values) / len(values)

    population_metrics = {}
    for name in ("whole_route", "cruise", "turn", "climb_descent", "hover", "landing"):
        rows = [row for row in errors if name in row["populations"]]
        horizontal = [float(row["horizontal_error_m"]) for row in rows]
        vertical = [abs(float(row["vertical_error_m"])) for row in rows]
        population_metrics[name] = {"epochs": len(rows), "horizontal_median_m": median(horizontal) if horizontal else None, "horizontal_p95_m": percentile(horizontal, 0.95), "vertical_p95_abs_m": percentile(vertical, 0.95)}
    if any(value["epochs"] == 0 for value in population_metrics.values()):
        fail("one or more required UAV motion populations are empty")

    stamps = [float(row["unix_s"]) for row in positions]
    gaps = [current - previous for previous, current in zip(stamps, stamps[1:])]
    jumps = [horizontal_m(float(current["lat"]), float(current["lon"]), float(previous["lat"]), float(previous["lon"])) for previous, current in zip(positions, positions[1:])]
    metrics = {
        "selected_epochs": selected_epochs,
        "solution_epochs": len(positions),
        "truth_interpolated_epochs": len(errors),
        "solution_availability_ratio": len(positions) / selected_epochs,
        "truth_coverage_ratio": len(errors) / selected_epochs,
        "max_solution_gap_s": max(gaps) if gaps else 0.0,
        "max_horizontal_jump_m": max(jumps) if jumps else 0.0,
        "attitude_rtk_yaw_median_abs_deg": median(yaw_errors),
        "attitude_rtk_yaw_p95_abs_deg": percentile(yaw_errors, 0.95),
        "attitude_rtk_yaw_max_match_delta_s": max(yaw_time_deltas),
        "rtk_fixed_ratio": status_ratio,
        "reference_point_mismatch_bound_m": float(dict(profile["frame_contract"])["raw_to_truth_antenna_distance_m"]),
        "populations": population_metrics,
    }
    thresholds = dict(profile.get("thresholds", {}))
    gates: list[dict[str, object]] = []
    gate_specs = (
        ("solution_availability_ratio", metrics["solution_availability_ratio"], "availability_min", True),
        ("truth_coverage_ratio", metrics["truth_coverage_ratio"], "truth_coverage_min", True),
        ("whole_route_horizontal_p95_m", population_metrics["whole_route"]["horizontal_p95_m"], "horizontal_p95_max_m", False),
        ("whole_route_vertical_p95_abs_m", population_metrics["whole_route"]["vertical_p95_abs_m"], "vertical_p95_max_m", False),
        ("max_solution_gap_s", metrics["max_solution_gap_s"], "max_solution_gap_s", False),
        ("max_horizontal_jump_m", metrics["max_horizontal_jump_m"], "max_horizontal_jump_m", False),
        ("attitude_rtk_yaw_p95_abs_deg", metrics["attitude_rtk_yaw_p95_abs_deg"], "attitude_yaw_p95_max_deg", False),
        ("rtk_position_fixed_ratio", status_ratio["position"], "rtk_fixed_ratio_min", True),
        ("rtk_yaw_fixed_ratio", status_ratio["yaw"], "rtk_fixed_ratio_min", True),
        ("landing_horizontal_p95_m", population_metrics["landing"]["horizontal_p95_m"], "landing_horizontal_p95_max_m", False),
    )
    if thresholds:
        for name, actual, key, minimum in gate_specs:
            if key not in thresholds:
                fail(f"frozen profile is missing threshold {key}")
            threshold = float(thresholds[key])
            gates.append({"name": name, "actual": actual, "operator": ">=" if minimum else "<=", "threshold": threshold, "passed": actual >= threshold if minimum else actual <= threshold})
        for population_name, values in population_metrics.items():
            for suffix, actual, key, minimum in (
                ("epochs", values["epochs"], "population_epochs_min", True),
                ("horizontal_p95_m", values["horizontal_p95_m"], "population_horizontal_p95_max_m", False),
                ("vertical_p95_abs_m", values["vertical_p95_abs_m"], "population_vertical_p95_max_m", False),
            ):
                if key not in thresholds:
                    fail(f"frozen profile is missing threshold {key}")
                threshold = float(thresholds[key])
                gates.append({
                    "name": f"{population_name}_{suffix}",
                    "actual": actual,
                    "operator": ">=" if minimum else "<=",
                    "threshold": threshold,
                    "passed": actual >= threshold if minimum else actual <= threshold,
                })
    navigation = "pass" if gates and all(bool(gate["passed"]) for gate in gates) else "fail" if gates else "development_ungated"
    mapping = "no_go" if dict(profile["frame_contract"]).get("lidar_to_dji_body_rotation_status") != "resolved" else navigation
    visualization = "pass" if metrics["truth_coverage_ratio"] >= 0.99 and metrics["max_solution_gap_s"] <= 0.2 else "fail"

    args.output_dir.mkdir(parents=True, exist_ok=True)
    matches_path = args.output_dir / "uav_matches.csv"
    with matches_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("unix_time_s", "horizontal_error_m", "vertical_error_m", "populations"))
        for row in errors:
            writer.writerow((row["unix_time_s"], row["horizontal_error_m"], row["vertical_error_m"], ";".join(row["populations"])))
    payload = {
        "schema_version": SCHEMA_VERSION,
        "dataset": {"id": expected["id"], "role": role},
        "metrics": metrics,
        "gates": gates,
        "decisions": {"navigation": navigation, "mapping": mapping, "visualization": visualization},
        "contracts": {"vehicle_nhc": False, "attitude_yaw_formula": "(-yaw_ccw_of_body_FLU_to_ground_ENU) mod 360; no fitted offset", "truth_interpolation": "linear WGS84 geodetic/ellipsoidal-height interpolation only across profile-bounded adjacent RTK samples", "reference_point": dict(profile["frame_contract"])["navigation_rule"]},
        "inputs": {name: {"path": str(path), "sha256": sha256(path)} for name, path in (("position", args.position), ("truth", args.truth), ("attitude", args.attitude), ("rtk_yaw", args.rtk_yaw), ("rtk_status", args.rtk_status), ("adapter_summary", args.adapter_summary), ("solver_summary", args.solver_summary), ("profile", args.profile))},
        "artifacts": {"matches": {"path": str(matches_path), "sha256": sha256(matches_path), "rows": len(errors)}},
    }
    summary = args.output_dir / "uav_signoff_summary.json"
    summary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if navigation in ("pass", "development_ungated") else 1


if __name__ == "__main__":
    raise SystemExit(main())
