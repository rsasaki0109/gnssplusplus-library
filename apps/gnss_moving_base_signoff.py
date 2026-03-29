#!/usr/bin/env python3
"""Run and validate real moving-base replay/live datasets."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import time

from gnss_runtime import ensure_input_exists, parse_summary_metrics, resolve_gnss_command


ROOT_DIR = Path(__file__).resolve().parent.parent
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--solver", choices=("replay", "live"), default="replay")
    parser.add_argument("--rover-rinex", type=Path, default=None)
    parser.add_argument("--rover-ubx", type=Path, default=None)
    parser.add_argument("--rover-rtcm", type=Path, default=None)
    parser.add_argument("--base-rinex", type=Path, default=None)
    parser.add_argument("--base-ubx", type=Path, default=None)
    parser.add_argument("--base-rtcm", type=Path, default=None)
    parser.add_argument("--nav-rinex", type=Path, default=None)
    parser.add_argument("--reference-csv", type=Path, required=True)
    parser.add_argument(
        "--out",
        type=Path,
        default=ROOT_DIR / "output/moving_base_solution.pos",
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=ROOT_DIR / "output/moving_base_summary.json",
    )
    parser.add_argument("--plot-png", type=Path, default=None)
    parser.add_argument("--plot-title", default="Moving-base sign-off")
    parser.add_argument("--log-out", type=Path, default=None)
    parser.add_argument("--max-epochs", type=int, default=120)
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--use-existing-solution", action="store_true")
    parser.add_argument("--solver-wall-time-s", type=float, default=None)

    parser.add_argument("--require-valid-epochs-min", type=int, default=None)
    parser.add_argument("--require-matched-epochs-min", type=int, default=None)
    parser.add_argument("--require-fix-rate-min", type=float, default=None)
    parser.add_argument("--require-median-baseline-error-max", type=float, default=None)
    parser.add_argument("--require-p95-baseline-error-max", type=float, default=None)
    parser.add_argument("--require-max-baseline-error-max", type=float, default=None)
    parser.add_argument("--require-p95-heading-error-max", type=float, default=None)
    parser.add_argument("--require-heading-samples-min", type=int, default=None)
    parser.add_argument("--require-solver-wall-time-max", type=float, default=None)
    parser.add_argument("--require-realtime-factor-min", type=float, default=None)
    parser.add_argument("--require-effective-epoch-rate-min", type=float, default=None)
    parser.add_argument("--require-termination", default=None)
    parser.add_argument("--require-rover-decoder-errors-max", type=int, default=None)
    parser.add_argument("--require-base-decoder-errors-max", type=int, default=None)
    return parser.parse_args()


def normalize_field(name: str) -> str:
    return "".join(ch for ch in name.strip().lower() if ch.isalnum())


def get_token(row: dict[str, str], *aliases: str) -> str | None:
    normalized = {normalize_field(key): value for key, value in row.items()}
    for alias in aliases:
        value = normalized.get(normalize_field(alias))
        if value is not None and value != "":
            return value
    return None


def geodetic_to_ecef(lat_deg: float, lon_deg: float, height_m: float) -> tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + height_m) * cos_lat * cos_lon
    y = (n + height_m) * cos_lat * sin_lon
    z = (n * (1.0 - WGS84_E2) + height_m) * sin_lat
    return x, y, z


def ecef_to_geodetic(x: float, y: float, z: float) -> tuple[float, float, float]:
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))
    height = 0.0
    for _ in range(8):
        sin_lat = math.sin(lat)
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        height = p / max(math.cos(lat), 1e-12) - n
        lat = math.atan2(z, p * (1.0 - WGS84_E2 * n / (n + height)))
    return math.degrees(lat), math.degrees(lon), height


def ecef_delta_to_enu(delta: tuple[float, float, float], ref_lat_deg: float, ref_lon_deg: float) -> tuple[float, float, float]:
    lat = math.radians(ref_lat_deg)
    lon = math.radians(ref_lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    dx, dy, dz = delta
    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return east, north, up


def percentile(values: list[float], p: float) -> float:
    if not values:
        raise ValueError("percentile on empty list")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * p
    low = int(math.floor(rank))
    high = int(math.ceil(rank))
    if low == high:
        return ordered[low]
    weight = rank - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def read_pos_records(path: Path) -> list[dict[str, float | int]]:
    records: list[dict[str, float | int]] = []
    with path.open(encoding="ascii") as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 10:
                continue
            records.append(
                {
                    "week": int(float(parts[0])),
                    "tow": float(parts[1]),
                    "x": float(parts[2]),
                    "y": float(parts[3]),
                    "z": float(parts[4]),
                    "status": int(parts[8]),
                    "satellites": int(parts[9]),
                }
            )
    return records


def read_reference_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open(encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            week_token = get_token(row, "gps_week", "week")
            tow_token = get_token(row, "gps_tow_s", "tow_s", "tow", "gpstow")
            if week_token is None or tow_token is None:
                raise SystemExit(
                    f"{path} must contain gps_week/week and gps_tow_s/tow columns"
                )

            def parse_ecef(prefix: str) -> tuple[float, float, float]:
                x_token = get_token(
                    row,
                    f"{prefix}_ecef_x_m",
                    f"{prefix}_x_m",
                    f"{prefix}_x",
                )
                y_token = get_token(
                    row,
                    f"{prefix}_ecef_y_m",
                    f"{prefix}_y_m",
                    f"{prefix}_y",
                )
                z_token = get_token(
                    row,
                    f"{prefix}_ecef_z_m",
                    f"{prefix}_z_m",
                    f"{prefix}_z",
                )
                if x_token is not None and y_token is not None and z_token is not None:
                    return float(x_token), float(y_token), float(z_token)

                lat_token = get_token(row, f"{prefix}_lat_deg", f"{prefix}_lat")
                lon_token = get_token(row, f"{prefix}_lon_deg", f"{prefix}_lon")
                h_token = get_token(row, f"{prefix}_height_m", f"{prefix}_h_m", f"{prefix}_height")
                if lat_token is not None and lon_token is not None and h_token is not None:
                    return geodetic_to_ecef(float(lat_token), float(lon_token), float(h_token))

                raise SystemExit(
                    f"{path} must contain either {prefix}_ecef_[xyz]_m or {prefix}_lat/lon/height columns"
                )

            base_x, base_y, base_z = parse_ecef("base")
            rover_x, rover_y, rover_z = parse_ecef("rover")
            rows.append(
                {
                    "week": float(week_token),
                    "tow": float(tow_token),
                    "base_x": base_x,
                    "base_y": base_y,
                    "base_z": base_z,
                    "rover_x": rover_x,
                    "rover_y": rover_y,
                    "rover_z": rover_z,
                }
            )

    rows.sort(key=lambda item: (item["week"], item["tow"]))
    return rows


def baseline_heading_deg(east: float, north: float) -> float:
    return (math.degrees(math.atan2(east, north)) + 360.0) % 360.0


def angle_error_deg(a_deg: float, b_deg: float) -> float:
    delta = abs(a_deg - b_deg) % 360.0
    return min(delta, 360.0 - delta)


def match_solution_to_reference(
    solution_records: list[dict[str, float | int]],
    reference_rows: list[dict[str, float]],
    tolerance_s: float,
) -> list[dict[str, float]]:
    matches: list[dict[str, float]] = []
    ref_index = 0
    for record in solution_records:
        record_week = int(record["week"])
        record_tow = float(record["tow"])
        while ref_index < len(reference_rows):
            ref = reference_rows[ref_index]
            if int(ref["week"]) < record_week:
                ref_index += 1
                continue
            if int(ref["week"]) == record_week and ref["tow"] < record_tow - tolerance_s:
                ref_index += 1
                continue
            break
        if ref_index >= len(reference_rows):
            break
        ref = reference_rows[ref_index]
        if int(ref["week"]) != record_week:
            continue
        if abs(ref["tow"] - record_tow) > tolerance_s:
            continue

        base_x = ref["base_x"]
        base_y = ref["base_y"]
        base_z = ref["base_z"]
        rover_x = ref["rover_x"]
        rover_y = ref["rover_y"]
        rover_z = ref["rover_z"]

        ref_lat_deg, ref_lon_deg, _ = ecef_to_geodetic(base_x, base_y, base_z)
        ref_delta = (rover_x - base_x, rover_y - base_y, rover_z - base_z)
        sol_delta = (
            float(record["x"]) - base_x,
            float(record["y"]) - base_y,
            float(record["z"]) - base_z,
        )
        ref_enu = ecef_delta_to_enu(ref_delta, ref_lat_deg, ref_lon_deg)
        sol_enu = ecef_delta_to_enu(sol_delta, ref_lat_deg, ref_lon_deg)
        error_enu = (
            sol_enu[0] - ref_enu[0],
            sol_enu[1] - ref_enu[1],
            sol_enu[2] - ref_enu[2],
        )
        baseline_error = math.sqrt(
            error_enu[0] * error_enu[0]
            + error_enu[1] * error_enu[1]
            + error_enu[2] * error_enu[2]
        )
        horizontal_ref = math.hypot(ref_enu[0], ref_enu[1])
        horizontal_sol = math.hypot(sol_enu[0], sol_enu[1])
        heading_error = None
        if horizontal_ref >= 0.5 and horizontal_sol >= 0.5:
            heading_error = angle_error_deg(
                baseline_heading_deg(sol_enu[0], sol_enu[1]),
                baseline_heading_deg(ref_enu[0], ref_enu[1]),
            )
        matches.append(
            {
                "week": float(record_week),
                "tow": record_tow,
                "baseline_error_m": baseline_error,
                "baseline_length_m": math.sqrt(
                    ref_delta[0] * ref_delta[0]
                    + ref_delta[1] * ref_delta[1]
                    + ref_delta[2] * ref_delta[2]
                ),
                "heading_error_deg": heading_error,
                "status": float(record["status"]),
                "satellites": float(record["satellites"]),
            }
        )
    return matches


def rounded(value: float | None) -> float | None:
    if value is None:
        return None
    return round(float(value), 6)


def build_solver_command(args: argparse.Namespace) -> list[str]:
    command = [*resolve_gnss_command(ROOT_DIR), args.solver]
    if args.solver == "replay":
        if bool(args.rover_rinex) == bool(args.rover_ubx):
            raise SystemExit("replay sign-off requires exactly one of --rover-rinex or --rover-ubx")
        if sum(
            int(value is not None)
            for value in (args.base_rinex, args.base_ubx, args.base_rtcm)
        ) != 1:
            raise SystemExit(
                "replay sign-off requires exactly one of --base-rinex, --base-ubx, or --base-rtcm"
            )
        if args.rover_rinex is not None:
            ensure_input_exists(args.rover_rinex, "moving-base rover RINEX", ROOT_DIR)
            command += ["--rover-rinex", str(args.rover_rinex)]
        if args.rover_ubx is not None:
            ensure_input_exists(args.rover_ubx, "moving-base rover UBX", ROOT_DIR)
            command += ["--rover-ubx", str(args.rover_ubx)]
        if args.base_rinex is not None:
            ensure_input_exists(args.base_rinex, "moving-base base RINEX", ROOT_DIR)
            command += ["--base-rinex", str(args.base_rinex)]
        if args.base_ubx is not None:
            ensure_input_exists(args.base_ubx, "moving-base base UBX", ROOT_DIR)
            command += ["--base-ubx", str(args.base_ubx)]
        if args.base_rtcm is not None:
            ensure_input_exists(args.base_rtcm, "moving-base base RTCM", ROOT_DIR)
            command += ["--base-rtcm", str(args.base_rtcm)]
        if args.nav_rinex is not None:
            ensure_input_exists(args.nav_rinex, "moving-base navigation RINEX", ROOT_DIR)
            command += ["--nav-rinex", str(args.nav_rinex)]
    else:
        if bool(args.rover_rtcm) == bool(args.rover_ubx):
            raise SystemExit("live sign-off requires exactly one of --rover-rtcm or --rover-ubx")
        if args.base_rtcm is None:
            raise SystemExit("live sign-off requires --base-rtcm")
        if args.rover_rtcm is not None:
            ensure_input_exists(args.rover_rtcm, "moving-base rover RTCM", ROOT_DIR)
            command += ["--rover-rtcm", str(args.rover_rtcm)]
        if args.rover_ubx is not None:
            ensure_input_exists(args.rover_ubx, "moving-base rover UBX", ROOT_DIR)
            command += ["--rover-ubx", str(args.rover_ubx)]
        ensure_input_exists(args.base_rtcm, "moving-base base RTCM", ROOT_DIR)
        command += ["--base-rtcm", str(args.base_rtcm)]
        if args.nav_rinex is not None:
            ensure_input_exists(args.nav_rinex, "moving-base navigation RINEX", ROOT_DIR)
            command += ["--nav-rinex", str(args.nav_rinex)]

    command += [
        "--mode",
        "moving-base",
        "--out",
        str(args.out),
        "--max-epochs",
        str(args.max_epochs),
        "--quiet",
    ]
    return command


def build_summary_payload(
    args: argparse.Namespace,
    solution_records: list[dict[str, float | int]],
    matches: list[dict[str, float]],
    solver_wall_time_s: float | None,
    solver_metrics: dict[str, object] | None,
    stdout_text: str,
    stderr_text: str,
    exit_code: int | None,
) -> dict[str, object]:
    heading_errors = [
        float(match["heading_error_deg"])
        for match in matches
        if match["heading_error_deg"] is not None
    ]
    baseline_errors = [float(match["baseline_error_m"]) for match in matches]
    baseline_lengths = [float(match["baseline_length_m"]) for match in matches]
    valid_epochs = len(solution_records)
    fixed_epochs = sum(1 for record in solution_records if int(record["status"]) == 4)
    solution_span_s = 0.0
    if solution_records:
        first = solution_records[0]
        last = solution_records[-1]
        solution_span_s = (
            (int(last["week"]) - int(first["week"])) * 604800.0
            + float(last["tow"]) - float(first["tow"])
        )
    effective_wall_time_s = (
        float(solver_metrics["solver_wall_time_s"])
        if solver_metrics is not None and "solver_wall_time_s" in solver_metrics
        else solver_wall_time_s
    )
    realtime_factor = (
        solution_span_s / effective_wall_time_s
        if effective_wall_time_s is not None and effective_wall_time_s > 0.0
        else None
    )
    effective_epoch_rate_hz = (
        valid_epochs / effective_wall_time_s
        if effective_wall_time_s is not None and effective_wall_time_s > 0.0
        else None
    )

    payload: dict[str, object] = {
        "dataset": "custom moving-base",
        "solver": args.solver,
        "reference_csv": str(args.reference_csv),
        "solution_pos": str(args.out),
        "summary_json": str(args.summary_json),
        "log_out": str(args.log_out) if args.log_out is not None else None,
        "use_existing_solution": bool(args.use_existing_solution),
        "match_tolerance_s": rounded(args.match_tolerance_s),
        "epochs": valid_epochs,
        "valid_epochs": valid_epochs,
        "matched_epochs": len(matches),
        "fixed_epochs": fixed_epochs,
        "fix_rate_pct": rounded(100.0 * fixed_epochs / valid_epochs) if valid_epochs else 0.0,
        "median_baseline_error_m": rounded(percentile(baseline_errors, 0.5)) if baseline_errors else None,
        "p95_baseline_error_m": rounded(percentile(baseline_errors, 0.95)) if baseline_errors else None,
        "max_baseline_error_m": rounded(max(baseline_errors)) if baseline_errors else None,
        "mean_baseline_length_m": rounded(sum(baseline_lengths) / len(baseline_lengths)) if baseline_lengths else None,
        "heading_samples": len(heading_errors),
        "median_heading_error_deg": rounded(percentile(heading_errors, 0.5)) if heading_errors else None,
        "p95_heading_error_deg": rounded(percentile(heading_errors, 0.95)) if heading_errors else None,
        "max_heading_error_deg": rounded(max(heading_errors)) if heading_errors else None,
        "solution_span_s": rounded(solution_span_s),
        "solver_wall_time_s": rounded(effective_wall_time_s),
        "realtime_factor": rounded(realtime_factor),
        "effective_epoch_rate_hz": rounded(effective_epoch_rate_hz),
        "stdout": stdout_text,
        "stderr": stderr_text,
        "exit_code": exit_code,
        "plot_png": str(args.plot_png) if args.plot_png is not None else None,
    }
    if solver_metrics is not None:
        payload["solver_metrics"] = solver_metrics
        for key in (
            "termination",
            "rover_decoder_errors",
            "base_decoder_errors",
            "aligned_epochs",
            "written_solutions",
            "fixed_solutions",
        ):
            if key in solver_metrics:
                payload[key] = solver_metrics[key]

    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload


def enforce_requirements(payload: dict[str, object], args: argparse.Namespace) -> None:
    failures: list[str] = []

    def require_min(key: str, threshold: float | int | None) -> None:
        if threshold is None:
            return
        value = payload.get(key)
        if value is None:
            failures.append(f"missing `{key}`")
        elif float(value) < float(threshold):
            failures.append(f"{key} {float(value):.6f} < {float(threshold):.6f}")

    def require_max(key: str, threshold: float | int | None) -> None:
        if threshold is None:
            return
        value = payload.get(key)
        if value is None:
            failures.append(f"missing `{key}`")
        elif float(value) > float(threshold):
            failures.append(f"{key} {float(value):.6f} > {float(threshold):.6f}")

    if args.require_termination is not None:
        termination = payload.get("termination")
        if termination != args.require_termination:
            failures.append(f"termination `{termination}` != `{args.require_termination}`")

    require_min("valid_epochs", args.require_valid_epochs_min)
    require_min("matched_epochs", args.require_matched_epochs_min)
    require_min("fix_rate_pct", args.require_fix_rate_min)
    require_min("heading_samples", args.require_heading_samples_min)
    require_min("realtime_factor", args.require_realtime_factor_min)
    require_min("effective_epoch_rate_hz", args.require_effective_epoch_rate_min)
    require_max("median_baseline_error_m", args.require_median_baseline_error_max)
    require_max("p95_baseline_error_m", args.require_p95_baseline_error_max)
    require_max("max_baseline_error_m", args.require_max_baseline_error_max)
    require_max("p95_heading_error_deg", args.require_p95_heading_error_max)
    require_max("solver_wall_time_s", args.require_solver_wall_time_max)
    require_max("rover_decoder_errors", args.require_rover_decoder_errors_max)
    require_max("base_decoder_errors", args.require_base_decoder_errors_max)

    if failures:
        raise SystemExit(
            "Moving-base sign-off checks failed:\n" + "\n".join(f"  - {failure}" for failure in failures)
        )


def main() -> int:
    args = parse_args()
    ensure_input_exists(args.reference_csv, "moving-base reference CSV", ROOT_DIR)
    if args.use_existing_solution:
        ensure_input_exists(args.out, "existing moving-base solution", ROOT_DIR)

    stdout_text = ""
    stderr_text = ""
    exit_code: int | None = None
    solver_metrics: dict[str, object] | None = None
    solver_wall_time_s = args.solver_wall_time_s

    if not args.use_existing_solution:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        command = build_solver_command(args)
        print("+", " ".join(command))
        started = time.perf_counter()
        completed = subprocess.run(command, cwd=ROOT_DIR, capture_output=True, text=True, check=False)
        finished = time.perf_counter()
        stdout_text = completed.stdout
        stderr_text = completed.stderr
        exit_code = completed.returncode
        if args.log_out is not None:
            args.log_out.parent.mkdir(parents=True, exist_ok=True)
            args.log_out.write_text(stdout_text + stderr_text, encoding="utf-8")
        if completed.returncode != 0:
            raise SystemExit(
                f"gnss {args.solver} failed with exit code {completed.returncode}\n"
                f"{stdout_text}{stderr_text}"
            )
        solver_wall_time_s = finished - started
        summary_line = ""
        for line in reversed((stdout_text + "\n" + stderr_text).splitlines()):
            if line.startswith("summary:"):
                summary_line = line.strip()
                break
        if summary_line:
            solver_metrics = parse_summary_metrics(summary_line)

    solution_records = read_pos_records(args.out)
    if not solution_records:
        raise SystemExit(f"No solution epochs found in {args.out}")
    reference_rows = read_reference_rows(args.reference_csv)
    if not reference_rows:
        raise SystemExit(f"No reference rows found in {args.reference_csv}")
    matches = match_solution_to_reference(solution_records, reference_rows, args.match_tolerance_s)
    payload = build_summary_payload(
        args,
        solution_records,
        matches,
        solver_wall_time_s,
        solver_metrics,
        stdout_text,
        stderr_text,
        exit_code,
    )
    if args.plot_png is not None:
        from gnss_moving_base_plot import render_matches_plot

        render_matches_plot(matches, args.plot_png, args.plot_title)
    enforce_requirements(payload, args)

    print("Finished moving-base sign-off.")
    print(f"  solver: {args.solver}")
    print(f"  solution: {args.out}")
    print(f"  summary: {args.summary_json}")
    if args.plot_png is not None:
        print(f"  plot: {args.plot_png}")
    print(f"  matched_epochs: {payload['matched_epochs']}")
    print(f"  fix_rate_pct: {payload['fix_rate_pct']}")
    print(f"  p95_baseline_error_m: {payload['p95_baseline_error_m']}")
    print(f"  p95_heading_error_deg: {payload['p95_heading_error_deg']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
