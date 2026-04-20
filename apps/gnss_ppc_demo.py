#!/usr/bin/env python3
"""Run libgnss++ against an external PPC-Dataset run and summarize the result."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timedelta
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

from gnss_runtime import ensure_input_exists, resolve_gnss_command


ROOT_DIR = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = ROOT_DIR / "scripts"
GPS_EPOCH = datetime(1980, 1, 6)
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3

sys.path.insert(0, str(SCRIPTS_DIR))

import generate_driving_comparison as comparison  # noqa: E402
import gnss_moving_base_signoff as moving_base_signoff  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=None,
        help="Root of an extracted PPC-Dataset tree.",
    )
    parser.add_argument(
        "--city",
        choices=("tokyo", "nagoya"),
        default=None,
        help="City directory under --dataset-root.",
    )
    parser.add_argument(
        "--run",
        default="run1",
        help="Run directory under --dataset-root/<city> (default: run1).",
    )
    parser.add_argument(
        "--run-dir",
        type=Path,
        default=None,
        help="Explicit PPC run directory containing rover.obs, base.obs, base.nav, and reference.csv.",
    )
    parser.add_argument(
        "--solver",
        choices=("rtk", "ppp"),
        default="rtk",
        help="libgnss++ solver path to run against the PPC data.",
    )
    parser.add_argument("--rover", type=Path, default=None, help="Override rover.obs path.")
    parser.add_argument("--base", type=Path, default=None, help="Override base.obs path for RTK.")
    parser.add_argument("--nav", type=Path, default=None, help="Override base.nav path.")
    parser.add_argument(
        "--reference-csv",
        type=Path,
        default=None,
        help="Override reference.csv path.",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Output .pos path (default: output/ppc_<city>_<run>_<solver>.pos).",
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=None,
        help="Output summary JSON path (default: output/ppc_<city>_<run>_<solver>_summary.json).",
    )
    parser.add_argument(
        "--max-epochs",
        type=int,
        default=120,
        help="Stop after N epochs (default: 120). Use -1 for the full run.",
    )
    parser.add_argument(
        "--match-tolerance-s",
        type=float,
        default=0.25,
        help="Reference matching tolerance in seconds (default: 0.25).",
    )
    parser.add_argument(
        "--use-existing-solution",
        action="store_true",
        help="Do not rerun the solver; only summarize an existing --out file.",
    )
    parser.add_argument(
        "--solver-wall-time-s",
        type=float,
        default=None,
        help="Optional solver wall time in seconds. If omitted, actual runtime is recorded when the solver is executed.",
    )
    parser.add_argument("--sp3", type=Path, default=None, help="Optional SP3 precise orbit file for PPP.")
    parser.add_argument("--clk", type=Path, default=None, help="Optional CLK precise clock file for PPP.")
    parser.add_argument("--ionex", type=Path, default=None, help="Optional IONEX TEC map file for PPP.")
    parser.add_argument("--dcb", type=Path, default=None, help="Optional DCB / Bias-SINEX file for PPP.")
    parser.add_argument("--antex", type=Path, default=None, help="Optional ANTEX file for PPP.")
    parser.add_argument("--blq", type=Path, default=None, help="Optional BLQ loading coefficients for PPP.")
    parser.add_argument(
        "--ppp-summary-json",
        type=Path,
        default=None,
        help="Optional gnss ppp summary JSON output path when --solver ppp is executed.",
    )
    parser.add_argument(
        "--preset",
        choices=("survey", "low-cost", "moving-base"),
        default=None,
        help="Optional RTK tuning preset passed through to gnss solve when --solver rtk.",
    )
    parser.add_argument("--arfilter", dest="arfilter", action="store_true", help="Enable AR filter for RTK.")
    parser.add_argument("--no-arfilter", dest="arfilter", action="store_false", help="Disable AR filter for RTK.")
    parser.set_defaults(arfilter=None)
    parser.add_argument("--arfilter-margin", type=float, default=None, help="Optional AR filter margin for RTK.")
    parser.add_argument("--min-hold-count", type=int, default=None, help="Optional min hold count for RTK.")
    parser.add_argument(
        "--hold-ratio-threshold",
        type=float,
        default=None,
        help="Optional ratio threshold used while hold is active for RTK.",
    )
    parser.add_argument(
        "--rtklib-bin",
        type=Path,
        default=None,
        help="Optional RTKLIB rnx2rtkp binary for side-by-side comparison.",
    )
    parser.add_argument(
        "--rtklib-config",
        type=Path,
        default=ROOT_DIR / "scripts/rtklib_odaiba.conf",
        help="RTKLIB configuration file used when --rtklib-bin is set.",
    )
    parser.add_argument(
        "--rtklib-pos",
        type=Path,
        default=None,
        help="Optional RTKLIB .pos path. If omitted, one is generated next to the summary when --rtklib-bin is used.",
    )
    parser.add_argument(
        "--use-existing-rtklib-solution",
        action="store_true",
        help="Do not rerun RTKLIB; only summarize an existing --rtklib-pos file.",
    )
    parser.add_argument(
        "--rtklib-solver-wall-time-s",
        type=float,
        default=None,
        help="Optional RTKLIB wall time in seconds. If omitted, actual runtime is recorded when RTKLIB is executed.",
    )
    parser.add_argument(
        "--commercial-pos",
        type=Path,
        default=None,
        help=(
            "Optional commercial receiver solution, as libgnss .pos or normalized CSV, "
            "to summarize against the PPC reference."
        ),
    )
    parser.add_argument(
        "--commercial-format",
        choices=("auto", "pos", "csv"),
        default="auto",
        help="Commercial receiver solution format (default: auto from suffix).",
    )
    parser.add_argument(
        "--commercial-label",
        default="commercial_receiver",
        help="Label stored in the commercial receiver summary.",
    )
    parser.add_argument(
        "--commercial-matched-csv",
        type=Path,
        default=None,
        help="Optional CSV of commercial receiver epochs matched to the PPC reference.",
    )
    parser.add_argument(
        "--commercial-solver-wall-time-s",
        type=float,
        default=None,
        help="Optional commercial receiver wall time in seconds for realtime metrics.",
    )
    parser.add_argument(
        "--enable-ar",
        action="store_true",
        help="Enable PPP ambiguity resolution.",
    )
    parser.add_argument(
        "--low-dynamics",
        action="store_true",
        help="Use the quasi-static low-dynamics PPP profile.",
    )
    parser.add_argument("--require-valid-epochs-min", type=int, default=None)
    parser.add_argument("--require-matched-epochs-min", type=int, default=None)
    parser.add_argument("--require-fix-rate-min", type=float, default=None)
    parser.add_argument("--require-median-h-max", type=float, default=None)
    parser.add_argument("--require-p95-h-max", type=float, default=None)
    parser.add_argument("--require-max-h-max", type=float, default=None)
    parser.add_argument("--require-p95-up-max", type=float, default=None)
    parser.add_argument("--require-mean-sats-min", type=float, default=None)
    parser.add_argument("--require-solver-wall-time-max", type=float, default=None)
    parser.add_argument("--require-realtime-factor-min", type=float, default=None)
    parser.add_argument("--require-effective-epoch-rate-min", type=float, default=None)
    parser.add_argument("--require-lib-fix-rate-vs-rtklib-min-delta", type=float, default=None)
    parser.add_argument("--require-lib-median-h-vs-rtklib-max-delta", type=float, default=None)
    parser.add_argument("--require-lib-p95-h-vs-rtklib-max-delta", type=float, default=None)
    return parser.parse_args()


def run_command(command: list[str]) -> None:
    print("+", " ".join(command))
    subprocess.run(command, check=True)


def rounded(value: float) -> float:
    return round(value, 6)


def week_tow_to_seconds(week: int, tow: float) -> float:
    return week * 604800.0 + tow


def solution_span_seconds(epochs: list[comparison.SolutionEpoch]) -> float:
    if len(epochs) < 2:
        return 0.0
    first = week_tow_to_seconds(epochs[0].week, epochs[0].tow)
    last = week_tow_to_seconds(epochs[-1].week, epochs[-1].tow)
    return max(0.0, last - first)


def reference_span_seconds(reference: list[comparison.ReferenceEpoch]) -> float:
    if len(reference) < 2:
        return 0.0
    first = week_tow_to_seconds(reference[0].week, reference[0].tow)
    last = week_tow_to_seconds(reference[-1].week, reference[-1].tow)
    return max(0.0, last - first)


def normalize_header(name: str) -> str:
    normalized = name.strip().lower()
    for old, new in (
        (" ", "_"),
        ("-", "_"),
        ("/", "_"),
        ("(", ""),
        (")", ""),
        ("[", ""),
        ("]", ""),
    ):
        normalized = normalized.replace(old, new)
    while "__" in normalized:
        normalized = normalized.replace("__", "_")
    return normalized


def gps_datetime_to_week_tow(stamp: datetime) -> tuple[int, float]:
    delta = stamp - GPS_EPOCH
    total_seconds = delta.total_seconds()
    week = int(total_seconds // 604800)
    tow = total_seconds - week * 604800
    return week, tow


def parse_reference_timestamp(text: str) -> tuple[int, float]:
    candidates = (
        "%Y/%m/%d %H:%M:%S.%f",
        "%Y/%m/%d %H:%M:%S",
        "%Y-%m-%d %H:%M:%S.%f",
        "%Y-%m-%d %H:%M:%S",
        "%Y-%m-%dT%H:%M:%S.%f",
        "%Y-%m-%dT%H:%M:%S",
    )
    stripped = text.strip()
    for pattern in candidates:
        try:
            return gps_datetime_to_week_tow(datetime.strptime(stripped, pattern))
        except ValueError:
            continue
    raise SystemExit(f"Unsupported PPC reference timestamp format: {text}")


def llh_from_ecef(ecef_x_m: float, ecef_y_m: float, ecef_z_m: float) -> tuple[float, float, float]:
    longitude = math.atan2(ecef_y_m, ecef_x_m)
    p = math.hypot(ecef_x_m, ecef_y_m)
    latitude = math.atan2(ecef_z_m, p * (1.0 - WGS84_E2))
    for _ in range(6):
        sin_lat = math.sin(latitude)
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        height = p / max(math.cos(latitude), 1e-12) - n
        latitude = math.atan2(ecef_z_m, p * (1.0 - WGS84_E2 * n / (n + height)))
    sin_lat = math.sin(latitude)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    height = p / max(math.cos(latitude), 1e-12) - n
    return math.degrees(latitude), math.degrees(longitude), height


def read_flexible_reference_csv(path: Path) -> list[comparison.ReferenceEpoch]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            return comparison.read_reference_csv(path)

        normalized_fields = {normalize_header(name): name for name in reader.fieldnames if name}
        if "gps_week" not in normalized_fields and "week" not in normalized_fields:
            if "gps_tow_s" not in normalized_fields and "tow" not in normalized_fields:
                return comparison.read_reference_csv(path)

        def get(row: dict[str, str], *names: str) -> str | None:
            for name in names:
                original = normalized_fields.get(name)
                if original is None:
                    continue
                value = row.get(original)
                if value is not None and value.strip():
                    return value.strip()
            return None

        rows: list[comparison.ReferenceEpoch] = []
        for row in reader:
            week_token = get(row, "gps_week", "week")
            tow_token = get(row, "gps_tow_s", "gps_tow", "tow_s", "tow", "gpstow")
            if week_token is not None and tow_token is not None:
                week = int(float(week_token))
                tow = float(tow_token)
            else:
                timestamp_token = get(row, "gpst", "gps_time", "timestamp")
                if timestamp_token is None:
                    date_token = get(row, "date")
                    time_token = get(row, "time")
                    if date_token is None or time_token is None:
                        raise SystemExit(
                            f"Could not infer GPS week/tow from {path}; supported columns are gps_week/gps_tow_s or timestamp/date+time"
                        )
                    timestamp_token = f"{date_token} {time_token}"
                week, tow = parse_reference_timestamp(timestamp_token)

            lat_token = get(row, "lat_deg", "latitude_deg", "lat", "latitude")
            lon_token = get(row, "lon_deg", "longitude_deg", "lon", "longitude")
            height_token = get(
                row,
                "height_m",
                "altitude_m",
                "alt_m",
                "height",
                "altitude",
            )
            ecef_x_token = get(row, "ecef_x_m", "x_m", "ecef_x", "x")
            ecef_y_token = get(row, "ecef_y_m", "y_m", "ecef_y", "y")
            ecef_z_token = get(row, "ecef_z_m", "z_m", "ecef_z", "z")

            if lat_token is not None and lon_token is not None and height_token is not None:
                lat_deg = float(lat_token)
                lon_deg = float(lon_token)
                height_m = float(height_token)
                ecef = comparison.llh_to_ecef(lat_deg, lon_deg, height_m)
            elif ecef_x_token is not None and ecef_y_token is not None and ecef_z_token is not None:
                ecef_x_m = float(ecef_x_token)
                ecef_y_m = float(ecef_y_token)
                ecef_z_m = float(ecef_z_token)
                lat_deg, lon_deg, height_m = llh_from_ecef(ecef_x_m, ecef_y_m, ecef_z_m)
                ecef = comparison.llh_to_ecef(lat_deg, lon_deg, height_m)
            else:
                raise SystemExit(
                    f"Could not infer reference coordinates from {path}; expected lat/lon/height or ECEF XYZ columns"
                )

            rows.append(
                comparison.ReferenceEpoch(
                    week=week,
                    tow=tow,
                    lat_deg=lat_deg,
                    lon_deg=lon_deg,
                    height_m=height_m,
                    ecef=ecef,
                )
            )
        return rows


def gps_week_tow_to_datetime_strings(week: int, tow: float) -> tuple[str, str]:
    stamp = GPS_EPOCH + timedelta(weeks=week, seconds=tow)
    return stamp.strftime("%Y/%m/%d"), stamp.strftime("%H:%M:%S.%f")[:-3]


def rtklib_config_text(config_path: Path, solver: str) -> str:
    text = config_path.read_text(encoding="utf-8")
    replacements = {
        "pos1-navsys        =1": "pos1-navsys        =61",
    }
    if solver == "rtk":
        replacements["pos1-posmode       =kinematic"] = "pos1-posmode       =kinematic"
    else:
        replacements["pos1-posmode       =kinematic"] = "pos1-posmode       =ppp-kine"
        replacements["pos2-armode        =continuous"] = "pos2-armode        =off"
    for old, new in replacements.items():
        text = text.replace(old, new)
    return text


def run_rtklib_solver(
    args: argparse.Namespace,
    rover: Path,
    base: Path | None,
    nav: Path,
    reference: list[comparison.ReferenceEpoch],
    rtklib_pos: Path,
) -> float:
    if args.solver != "rtk":
        raise SystemExit("RTKLIB comparison in ppc-demo is currently supported for --solver rtk only")
    if base is None:
        raise SystemExit("RTKLIB comparison requires a base observation file")
    if not reference:
        raise SystemExit("RTKLIB comparison requires at least one reference epoch")

    rtklib_bin = Path(args.rtklib_bin)
    command = [str(rtklib_bin)]
    with tempfile.TemporaryDirectory(prefix="ppc_rtklib_conf_") as temp_dir:
        temp_dir_path = Path(temp_dir)
        config_path = temp_dir_path / "ppc_rtklib.conf"
        config_path.write_text(rtklib_config_text(Path(args.rtklib_config), args.solver), encoding="utf-8")
        command.extend(["-k", str(config_path), "-o", str(rtklib_pos)])
        window_reference = reference
        if args.max_epochs > 0:
            window_reference = reference[:args.max_epochs]
        start_day, start_time = gps_week_tow_to_datetime_strings(
            window_reference[0].week, window_reference[0].tow
        )
        end_day, end_time = gps_week_tow_to_datetime_strings(
            window_reference[-1].week, window_reference[-1].tow
        )
        command.extend(["-ts", start_day, start_time, "-te", end_day, end_time])
        command.extend([str(rover), str(base), str(nav)])
        start = time.perf_counter()
        run_command(command)
        return time.perf_counter() - start


def summarize_solution_epochs(
    reference: list[comparison.ReferenceEpoch],
    solution_epochs: list[comparison.SolutionEpoch],
    fixed_status: int,
    label: str,
    match_tolerance_s: float,
    solver_wall_time_s: float | None,
) -> dict[str, object]:
    if not solution_epochs:
        raise SystemExit(f"No solution epochs found for {label}")
    matched = comparison.match_to_reference(solution_epochs, reference, match_tolerance_s)
    if not matched:
        raise SystemExit(f"No PPC epochs matched reference for {label}")

    summary = comparison.summarize(matched, fixed_status, label)
    mean_h_m = sum(epoch.horiz_error_m for epoch in matched) / len(matched)
    matched_fixed_epochs = sum(1 for epoch in matched if epoch.status == fixed_status)
    mean_satellites = sum(epoch.num_satellites for epoch in solution_epochs) / len(solution_epochs)
    valid_span_s = solution_span_seconds(solution_epochs)

    payload = {
        "valid_epochs": len(solution_epochs),
        "matched_epochs": len(matched),
        "fixed_epochs": matched_fixed_epochs,
        "fix_rate_pct": rounded(float(summary["fix_rate_pct"])),
        "mean_h_m": rounded(mean_h_m),
        "median_h_m": rounded(float(summary["median_h_m"])),
        "p95_h_m": rounded(float(summary["p95_h_m"])),
        "max_h_m": rounded(float(summary["max_h_m"])),
        "median_abs_up_m": rounded(float(summary["median_abs_up_m"])),
        "p95_abs_up_m": rounded(float(summary["p95_abs_up_m"])),
        "mean_up_m": rounded(float(summary["mean_up_m"])),
        "mean_satellites": rounded(mean_satellites),
        "solution_span_s": rounded(valid_span_s),
        "solver_wall_time_s": rounded(solver_wall_time_s) if solver_wall_time_s is not None else None,
        "realtime_factor": None,
        "effective_epoch_rate_hz": None,
    }
    if solver_wall_time_s is not None and solver_wall_time_s > 0.0:
        payload["effective_epoch_rate_hz"] = rounded(len(solution_epochs) / solver_wall_time_s)
        if valid_span_s > 0.0:
            payload["realtime_factor"] = rounded(valid_span_s / solver_wall_time_s)
    return payload


def read_commercial_solution_epochs(
    path: Path,
    requested_format: str,
) -> tuple[list[comparison.SolutionEpoch], str]:
    records, resolved_format = moving_base_signoff.read_commercial_solution_records(
        path,
        requested_format,
    )
    epochs: list[comparison.SolutionEpoch] = []
    for record in records:
        x = float(record["x"])
        y = float(record["y"])
        z = float(record["z"])
        lat_deg, lon_deg, height_m = llh_from_ecef(x, y, z)
        ecef = comparison.llh_to_ecef(lat_deg, lon_deg, height_m)
        epochs.append(
            comparison.SolutionEpoch(
                week=int(record["week"]),
                tow=float(record["tow"]),
                lat_deg=lat_deg,
                lon_deg=lon_deg,
                height_m=height_m,
                ecef=ecef,
                status=int(record["status"]),
                num_satellites=int(record["satellites"]),
            )
        )
    return epochs, resolved_format


def write_reference_matches_csv(
    path: Path,
    matches: list[comparison.MatchedEpoch],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "gps_tow_s",
                "traj_east_m",
                "traj_north_m",
                "traj_up_m",
                "east_error_m",
                "north_error_m",
                "up_error_m",
                "horizontal_error_m",
                "status",
            ]
        )
        for match in matches:
            writer.writerow(
                [
                    f"{match.tow:.3f}",
                    f"{match.traj_east_m:.6f}",
                    f"{match.traj_north_m:.6f}",
                    f"{match.traj_up_m:.6f}",
                    f"{match.east_m:.6f}",
                    f"{match.north_m:.6f}",
                    f"{match.up_m:.6f}",
                    f"{match.horiz_error_m:.6f}",
                    int(match.status),
                ]
            )


def solution_metric_delta(
    left: dict[str, object],
    right: dict[str, object],
) -> dict[str, object]:
    def optional_delta(key: str) -> float | None:
        left_value = left.get(key)
        right_value = right.get(key)
        if left_value is None or right_value is None:
            return None
        return rounded(float(left_value) - float(right_value))

    return {
        "valid_epochs": int(left["valid_epochs"]) - int(right["valid_epochs"]),
        "matched_epochs": int(left["matched_epochs"]) - int(right["matched_epochs"]),
        "fixed_epochs": int(left["fixed_epochs"]) - int(right["fixed_epochs"]),
        "fix_rate_pct": optional_delta("fix_rate_pct"),
        "mean_h_m": optional_delta("mean_h_m"),
        "median_h_m": optional_delta("median_h_m"),
        "p95_h_m": optional_delta("p95_h_m"),
        "max_h_m": optional_delta("max_h_m"),
        "median_abs_up_m": optional_delta("median_abs_up_m"),
        "p95_abs_up_m": optional_delta("p95_abs_up_m"),
        "solver_wall_time_s": optional_delta("solver_wall_time_s"),
        "realtime_factor": optional_delta("realtime_factor"),
    }


def resolve_run_dir(args: argparse.Namespace) -> Path:
    if args.run_dir is not None:
        return args.run_dir
    if args.dataset_root is None or args.city is None:
        raise SystemExit("Provide either --run-dir or both --dataset-root and --city")
    return args.dataset_root / args.city / args.run


def dataset_label(args: argparse.Namespace, run_dir: Path) -> tuple[str, str, str]:
    city = args.city or run_dir.parent.name
    run_name = args.run if args.run_dir is None else run_dir.name
    slug = f"{city}_{run_name}".replace("-", "_")
    return city, run_name, slug


def solver_fixed_status(solver: str) -> int:
    if solver == "ppp":
        return 6
    return 4


def resolve_paths(args: argparse.Namespace) -> tuple[Path, Path | None, Path, Path, Path, Path]:
    run_dir = resolve_run_dir(args)
    city, run_name, slug = dataset_label(args, run_dir)
    rover = args.rover or (run_dir / "rover.obs")
    base = args.base or (run_dir / "base.obs")
    nav = args.nav or (run_dir / "base.nav")
    reference_csv = args.reference_csv or (run_dir / "reference.csv")
    out = args.out or (ROOT_DIR / "output" / f"ppc_{slug}_{args.solver}.pos")
    summary_json = args.summary_json or (ROOT_DIR / "output" / f"ppc_{slug}_{args.solver}_summary.json")
    args._dataset_city = city
    args._dataset_run = run_name
    return rover, base if args.solver == "rtk" else None, nav, reference_csv, out, summary_json


def run_solver(
    args: argparse.Namespace,
    rover: Path,
    base: Path | None,
    nav: Path,
    out: Path,
) -> float:
    gnss_command = resolve_gnss_command(ROOT_DIR)
    if args.solver == "rtk":
        assert base is not None
        command = [
            *gnss_command,
            "solve",
            "--rover",
            str(rover),
            "--base",
            str(base),
            "--nav",
            str(nav),
            "--out",
            str(out),
            "--mode",
            "kinematic",
            "--no-kml",
        ]
        if args.preset is not None:
            command.extend(["--preset", args.preset])
        if args.arfilter is True:
            command.append("--arfilter")
        elif args.arfilter is False:
            command.append("--no-arfilter")
        if args.arfilter_margin is not None:
            command.extend(["--arfilter-margin", str(args.arfilter_margin)])
        if args.min_hold_count is not None:
            command.extend(["--min-hold-count", str(args.min_hold_count)])
        if args.hold_ratio_threshold is not None:
            command.extend(["--hold-ratio-threshold", str(args.hold_ratio_threshold)])
    else:
        command = [
            *gnss_command,
            "ppp",
            "--kinematic",
            "--obs",
            str(rover),
            "--nav",
            str(nav),
            "--out",
            str(out),
        ]
        if args.low_dynamics:
            command.append("--low-dynamics")
        if args.sp3 is not None:
            command.extend(["--sp3", str(args.sp3)])
        if args.clk is not None:
            command.extend(["--clk", str(args.clk)])
        if args.ionex is not None:
            command.extend(["--ionex", str(args.ionex)])
        if args.dcb is not None:
            command.extend(["--dcb", str(args.dcb)])
        if args.antex is not None:
            command.extend(["--antex", str(args.antex)])
        if args.blq is not None:
            command.extend(["--blq", str(args.blq)])
        if args.enable_ar:
            command.append("--enable-ar")
        if args.ppp_summary_json is not None:
            command.extend(["--summary-json", str(args.ppp_summary_json)])
    if args.max_epochs > 0:
        command.extend(["--max-epochs", str(args.max_epochs)])
    start = time.perf_counter()
    run_command(command)
    return time.perf_counter() - start


def build_summary_payload(
    args: argparse.Namespace,
    run_dir: Path,
    rover: Path,
    base: Path | None,
    nav: Path,
    reference_csv: Path,
    out: Path,
    summary_json: Path,
    solver_wall_time_s: float | None = None,
) -> dict[str, object]:
    reference = read_flexible_reference_csv(reference_csv)
    solution_epochs = comparison.read_libgnss_pos(out)
    lib_metrics = summarize_solution_epochs(
        reference,
        solution_epochs,
        solver_fixed_status(args.solver),
        args.solver,
        args.match_tolerance_s,
        solver_wall_time_s,
    )

    payload = {
        "dataset": f"PPC-Dataset {args._dataset_city} {args._dataset_run}",
        "solver": args.solver,
        "run_dir": str(run_dir),
        "rover": str(rover),
        "base": str(base) if base is not None else None,
        "nav": str(nav),
        "reference_csv": str(reference_csv),
        "solution_pos": str(out),
        "summary_json": str(summary_json),
        "generated_solution": not args.use_existing_solution,
        "valid_epochs": lib_metrics["valid_epochs"],
        "reference_epochs": len(reference),
        "matched_epochs": lib_metrics["matched_epochs"],
        "fixed_epochs": lib_metrics["fixed_epochs"],
        "fix_rate_pct": lib_metrics["fix_rate_pct"],
        "mean_h_m": lib_metrics["mean_h_m"],
        "median_h_m": lib_metrics["median_h_m"],
        "p95_h_m": lib_metrics["p95_h_m"],
        "max_h_m": lib_metrics["max_h_m"],
        "median_abs_up_m": lib_metrics["median_abs_up_m"],
        "p95_abs_up_m": lib_metrics["p95_abs_up_m"],
        "mean_up_m": lib_metrics["mean_up_m"],
        "mean_satellites": lib_metrics["mean_satellites"],
        "match_tolerance_s": rounded(args.match_tolerance_s),
        "solution_span_s": lib_metrics["solution_span_s"],
        "solver_wall_time_s": lib_metrics["solver_wall_time_s"],
        "realtime_factor": lib_metrics["realtime_factor"],
        "effective_epoch_rate_hz": lib_metrics["effective_epoch_rate_hz"],
    }

    if args.solver == "ppp" and args.ppp_summary_json is not None and args.ppp_summary_json.exists():
        ppp_run_summary = json.loads(args.ppp_summary_json.read_text(encoding="utf-8"))
        if isinstance(ppp_run_summary, dict):
            payload.update(
                {
                    "ppp_run_summary": ppp_run_summary,
                    "ppp_converged": bool(ppp_run_summary.get("converged", False)),
                    "ppp_convergence_time_s": rounded(
                        float(ppp_run_summary.get("convergence_time_s", 0.0))
                    ),
                    "ppp_solution_rate_pct": rounded(
                        float(ppp_run_summary.get("solution_rate_pct", 0.0))
                    ),
                    "ionex_corrections": int(ppp_run_summary.get("ionex_corrections", 0)),
                    "ionex_meters": rounded(float(ppp_run_summary.get("ionex_meters", 0.0))),
                    "dcb_corrections": int(ppp_run_summary.get("dcb_corrections", 0)),
                    "dcb_meters": rounded(float(ppp_run_summary.get("dcb_meters", 0.0))),
                }
            )

    rtklib_pos = getattr(args, "rtklib_pos", None)
    if rtklib_pos is not None and Path(rtklib_pos).exists():
        rtklib_metrics = summarize_solution_epochs(
            reference,
            comparison.read_rtklib_pos(Path(rtklib_pos)),
            1,
            "RTKLIB",
            args.match_tolerance_s,
            getattr(args, "rtklib_solver_wall_time_s", None),
        )
        payload["rtklib_pos"] = str(rtklib_pos)
        payload["rtklib_generated_solution"] = not getattr(args, "use_existing_rtklib_solution", False)
        payload["rtklib"] = rtklib_metrics
        payload["delta_vs_rtklib"] = {
            "fix_rate_pct": rounded(float(payload["fix_rate_pct"]) - float(rtklib_metrics["fix_rate_pct"])),
            "median_h_m": rounded(float(payload["median_h_m"]) - float(rtklib_metrics["median_h_m"])),
            "p95_h_m": rounded(float(payload["p95_h_m"]) - float(rtklib_metrics["p95_h_m"])),
            "max_h_m": rounded(float(payload["max_h_m"]) - float(rtklib_metrics["max_h_m"])),
            "solver_wall_time_s": (
                rounded(float(payload["solver_wall_time_s"]) - float(rtklib_metrics["solver_wall_time_s"]))
                if payload["solver_wall_time_s"] is not None and rtklib_metrics["solver_wall_time_s"] is not None
                else None
            ),
            "realtime_factor": (
                rounded(float(payload["realtime_factor"]) - float(rtklib_metrics["realtime_factor"]))
                if payload["realtime_factor"] is not None and rtklib_metrics["realtime_factor"] is not None
                else None
            ),
        }

    commercial_pos = getattr(args, "commercial_pos", None)
    if commercial_pos is not None and Path(commercial_pos).exists():
        commercial_epochs, commercial_format = read_commercial_solution_epochs(
            Path(commercial_pos),
            getattr(args, "commercial_format", "auto"),
        )
        commercial_metrics = summarize_solution_epochs(
            reference,
            commercial_epochs,
            4,
            getattr(args, "commercial_label", "commercial_receiver"),
            args.match_tolerance_s,
            getattr(args, "commercial_solver_wall_time_s", None),
        )
        commercial_matched_csv = getattr(args, "commercial_matched_csv", None)
        if commercial_matched_csv is not None:
            commercial_matches = comparison.match_to_reference(
                commercial_epochs,
                reference,
                args.match_tolerance_s,
            )
            write_reference_matches_csv(Path(commercial_matched_csv), commercial_matches)
        commercial_metrics.update(
            {
                "label": getattr(args, "commercial_label", "commercial_receiver"),
                "solution_pos": str(commercial_pos),
                "format": commercial_format,
                "matched_csv": (
                    str(commercial_matched_csv) if commercial_matched_csv is not None else None
                ),
            }
        )
        payload["commercial_receiver"] = commercial_metrics
        payload["delta_vs_commercial_receiver"] = solution_metric_delta(
            payload,
            commercial_metrics,
        )

    summary_json.parent.mkdir(parents=True, exist_ok=True)
    summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload


def enforce_summary_requirements(payload: dict[str, object], args: argparse.Namespace) -> None:
    failures: list[str] = []
    if (
        args.require_valid_epochs_min is not None
        and int(payload["valid_epochs"]) < args.require_valid_epochs_min
    ):
        failures.append(
            f"valid epochs {int(payload['valid_epochs'])} < {args.require_valid_epochs_min}"
        )
    if (
        args.require_matched_epochs_min is not None
        and int(payload["matched_epochs"]) < args.require_matched_epochs_min
    ):
        failures.append(
            f"matched epochs {int(payload['matched_epochs'])} < {args.require_matched_epochs_min}"
        )
    if (
        args.require_fix_rate_min is not None
        and float(payload["fix_rate_pct"]) < args.require_fix_rate_min
    ):
        failures.append(
            f"fix rate {float(payload['fix_rate_pct']):.6f}% < {args.require_fix_rate_min:.6f}%"
        )
    if (
        args.require_median_h_max is not None
        and float(payload["median_h_m"]) > args.require_median_h_max
    ):
        failures.append(
            f"median horizontal error {float(payload['median_h_m']):.6f} m > {args.require_median_h_max:.6f} m"
        )
    if (
        args.require_p95_h_max is not None
        and float(payload["p95_h_m"]) > args.require_p95_h_max
    ):
        failures.append(
            f"p95 horizontal error {float(payload['p95_h_m']):.6f} m > {args.require_p95_h_max:.6f} m"
        )
    if (
        args.require_max_h_max is not None
        and float(payload["max_h_m"]) > args.require_max_h_max
    ):
        failures.append(
            f"max horizontal error {float(payload['max_h_m']):.6f} m > {args.require_max_h_max:.6f} m"
        )
    if (
        args.require_p95_up_max is not None
        and float(payload["p95_abs_up_m"]) > args.require_p95_up_max
    ):
        failures.append(
            f"p95 absolute up error {float(payload['p95_abs_up_m']):.6f} m > {args.require_p95_up_max:.6f} m"
        )
    if (
        args.require_mean_sats_min is not None
        and float(payload["mean_satellites"]) < args.require_mean_sats_min
    ):
        failures.append(
            f"mean satellites {float(payload['mean_satellites']):.6f} < {args.require_mean_sats_min:.6f}"
        )
    if args.require_solver_wall_time_max is not None:
        solver_wall_time_s = payload["solver_wall_time_s"]
        if solver_wall_time_s is None:
            failures.append("solver wall time is unavailable")
        elif float(solver_wall_time_s) > args.require_solver_wall_time_max:
            failures.append(
                f"solver wall time {float(solver_wall_time_s):.6f} s > {args.require_solver_wall_time_max:.6f} s"
            )
    if args.require_realtime_factor_min is not None:
        realtime_factor = payload["realtime_factor"]
        if realtime_factor is None:
            failures.append("realtime factor is unavailable")
        elif float(realtime_factor) < args.require_realtime_factor_min:
            failures.append(
                f"realtime factor {float(realtime_factor):.6f} < {args.require_realtime_factor_min:.6f}"
            )
    if args.require_effective_epoch_rate_min is not None:
        effective_epoch_rate_hz = payload["effective_epoch_rate_hz"]
        if effective_epoch_rate_hz is None:
            failures.append("effective epoch rate is unavailable")
        elif float(effective_epoch_rate_hz) < args.require_effective_epoch_rate_min:
            failures.append(
                f"effective epoch rate {float(effective_epoch_rate_hz):.6f} Hz < {args.require_effective_epoch_rate_min:.6f} Hz"
            )

    if (
        args.require_lib_fix_rate_vs_rtklib_min_delta is not None
        or args.require_lib_median_h_vs_rtklib_max_delta is not None
        or args.require_lib_p95_h_vs_rtklib_max_delta is not None
    ):
        if "delta_vs_rtklib" not in payload:
            failures.append("RTKLIB comparison summary is unavailable")
        else:
            deltas = payload["delta_vs_rtklib"]
            if args.require_lib_fix_rate_vs_rtklib_min_delta is not None:
                if float(deltas["fix_rate_pct"]) < args.require_lib_fix_rate_vs_rtklib_min_delta:
                    failures.append(
                        "lib fix rate vs RTKLIB delta "
                        f"{float(deltas['fix_rate_pct']):.6f}% < "
                        f"{args.require_lib_fix_rate_vs_rtklib_min_delta:.6f}%"
                    )
            if args.require_lib_median_h_vs_rtklib_max_delta is not None:
                if float(deltas["median_h_m"]) > args.require_lib_median_h_vs_rtklib_max_delta:
                    failures.append(
                        "lib median horizontal vs RTKLIB delta "
                        f"{float(deltas['median_h_m']):.6f} m > "
                        f"{args.require_lib_median_h_vs_rtklib_max_delta:.6f} m"
                    )
            if args.require_lib_p95_h_vs_rtklib_max_delta is not None:
                if float(deltas["p95_h_m"]) > args.require_lib_p95_h_vs_rtklib_max_delta:
                    failures.append(
                        "lib p95 horizontal vs RTKLIB delta "
                        f"{float(deltas['p95_h_m']):.6f} m > "
                        f"{args.require_lib_p95_h_vs_rtklib_max_delta:.6f} m"
                    )

    if failures:
        raise SystemExit(
            "PPC demo checks failed:\n" + "\n".join(f"  - {failure}" for failure in failures)
        )


def main() -> int:
    args = parse_args()
    rover, base, nav, reference_csv, out, summary_json = resolve_paths(args)
    run_dir = resolve_run_dir(args)
    rtklib_pos = args.rtklib_pos
    if rtklib_pos is None and args.rtklib_bin is not None:
        rtklib_pos = summary_json.with_name(summary_json.stem.replace("_summary", "_rtklib") + ".pos")
        args.rtklib_pos = rtklib_pos

    ensure_input_exists(reference_csv, "PPC reference CSV", ROOT_DIR)
    if args.commercial_pos is not None:
        ensure_input_exists(args.commercial_pos, "commercial receiver solution", ROOT_DIR)
    elif args.commercial_matched_csv is not None:
        raise SystemExit("--commercial-matched-csv requires --commercial-pos")
    if args.max_epochs == 0:
        raise SystemExit("--max-epochs must be positive or -1")

    if args.use_existing_solution:
        ensure_input_exists(out, "existing solution file", ROOT_DIR)
        if args.solver == "ppp" and args.ppp_summary_json is not None:
            ensure_input_exists(args.ppp_summary_json, "existing PPP summary JSON", ROOT_DIR)
    else:
        ensure_input_exists(rover, "PPC rover observation file", ROOT_DIR)
        ensure_input_exists(nav, "PPC navigation file", ROOT_DIR)
        if base is not None:
            ensure_input_exists(base, "PPC base observation file", ROOT_DIR)
        if args.sp3 is not None:
            ensure_input_exists(args.sp3, "PPP SP3 file", ROOT_DIR)
        if args.clk is not None:
            ensure_input_exists(args.clk, "PPP CLK file", ROOT_DIR)
        if args.ionex is not None:
            ensure_input_exists(args.ionex, "PPP IONEX file", ROOT_DIR)
        if args.dcb is not None:
            ensure_input_exists(args.dcb, "PPP DCB file", ROOT_DIR)
        if args.antex is not None:
            ensure_input_exists(args.antex, "PPP ANTEX file", ROOT_DIR)
        if args.blq is not None:
            ensure_input_exists(args.blq, "PPP BLQ file", ROOT_DIR)
        if args.ppp_summary_json is not None:
            args.ppp_summary_json.parent.mkdir(parents=True, exist_ok=True)

        out.parent.mkdir(parents=True, exist_ok=True)
        measured_wall_time_s = run_solver(args, rover, base, nav, out)
        if args.solver_wall_time_s is None:
            args.solver_wall_time_s = measured_wall_time_s

    if args.rtklib_bin is not None or args.use_existing_rtklib_solution:
        if args.solver != "rtk":
            raise SystemExit("PPC RTKLIB comparison is currently supported only for --solver rtk")
        if args.rtklib_bin is not None:
            ensure_input_exists(args.rtklib_bin, "RTKLIB binary", ROOT_DIR)
            ensure_input_exists(args.rtklib_config, "RTKLIB config", ROOT_DIR)
        if args.use_existing_rtklib_solution:
            if rtklib_pos is None:
                raise SystemExit("--use-existing-rtklib-solution requires --rtklib-pos")
            ensure_input_exists(rtklib_pos, "existing RTKLIB solution file", ROOT_DIR)
        else:
            reference = read_flexible_reference_csv(reference_csv)
            if rtklib_pos is None:
                raise SystemExit("missing RTKLIB output path")
            rtklib_pos.parent.mkdir(parents=True, exist_ok=True)
            measured_rtklib_wall_time_s = run_rtklib_solver(args, rover, base, nav, reference, rtklib_pos)
            if args.rtklib_solver_wall_time_s is None:
                args.rtklib_solver_wall_time_s = measured_rtklib_wall_time_s

    payload = build_summary_payload(
        args,
        run_dir,
        rover,
        base,
        nav,
        reference_csv,
        out,
        summary_json,
        solver_wall_time_s=args.solver_wall_time_s,
    )
    enforce_summary_requirements(payload, args)

    print("Finished PPC-Dataset demo.")
    print(f"  dataset: {payload['dataset']}")
    print(f"  solver: {args.solver}")
    print(f"  solution: {out}")
    print(f"  summary: {summary_json}")
    if payload["solver_wall_time_s"] is not None:
        print(
            "  performance:"
            f" wall={payload['solver_wall_time_s']} s"
            f", span={payload['solution_span_s']} s"
            f", rtf={payload['realtime_factor']}"
            f", rate={payload['effective_epoch_rate_hz']} Hz"
        )
    if "rtklib" in payload:
        rtklib = payload["rtklib"]
        print(f"  rtklib: {payload['rtklib_pos']}")
        print(
            "  rtklib performance:"
            f" wall={rtklib['solver_wall_time_s']} s"
            f", span={rtklib['solution_span_s']} s"
            f", rtf={rtklib['realtime_factor']}"
            f", rate={rtklib['effective_epoch_rate_hz']} Hz"
        )
        print(
            "  vs rtklib:"
            f" fix={payload['delta_vs_rtklib']['fix_rate_pct']} %"
            f", p95_h={payload['delta_vs_rtklib']['p95_h_m']} m"
            f", wall={payload['delta_vs_rtklib']['solver_wall_time_s']} s"
        )
    if "commercial_receiver" in payload:
        commercial = payload["commercial_receiver"]
        print(f"  commercial_receiver: {commercial['solution_pos']}")
        print(
            "  vs commercial_receiver:"
            f" fix={payload['delta_vs_commercial_receiver']['fix_rate_pct']} %"
            f", p95_h={payload['delta_vs_commercial_receiver']['p95_h_m']} m"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
