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

from gnss_runtime import ensure_input_exists, resolve_gnss_command


ROOT_DIR = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = ROOT_DIR / "scripts"
GPS_EPOCH = datetime(1980, 1, 6)
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3

sys.path.insert(0, str(SCRIPTS_DIR))

import generate_driving_comparison as comparison  # noqa: E402


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
    parser.add_argument("--sp3", type=Path, default=None, help="Optional SP3 precise orbit file for PPP.")
    parser.add_argument("--clk", type=Path, default=None, help="Optional CLK precise clock file for PPP.")
    parser.add_argument("--antex", type=Path, default=None, help="Optional ANTEX file for PPP.")
    parser.add_argument("--blq", type=Path, default=None, help="Optional BLQ loading coefficients for PPP.")
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
    return parser.parse_args()


def run_command(command: list[str]) -> None:
    print("+", " ".join(command))
    subprocess.run(command, check=True)


def rounded(value: float) -> float:
    return round(value, 6)


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
) -> None:
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
        if args.antex is not None:
            command.extend(["--antex", str(args.antex)])
        if args.blq is not None:
            command.extend(["--blq", str(args.blq)])
        if args.enable_ar:
            command.append("--enable-ar")
    if args.max_epochs > 0:
        command.extend(["--max-epochs", str(args.max_epochs)])
    run_command(command)


def build_summary_payload(
    args: argparse.Namespace,
    run_dir: Path,
    rover: Path,
    base: Path | None,
    nav: Path,
    reference_csv: Path,
    out: Path,
    summary_json: Path,
) -> dict[str, object]:
    reference = read_flexible_reference_csv(reference_csv)
    solution_epochs = comparison.read_libgnss_pos(out)
    if not solution_epochs:
        raise SystemExit(f"No solution epochs found in {out}")
    matched = comparison.match_to_reference(solution_epochs, reference, args.match_tolerance_s)
    if not matched:
        raise SystemExit(f"No PPC epochs matched {reference_csv} for {out}")

    summary = comparison.summarize(matched, solver_fixed_status(args.solver), args.solver)
    matched_fixed_epochs = sum(
        1 for epoch in matched if epoch.status == solver_fixed_status(args.solver)
    )
    mean_satellites = sum(epoch.num_satellites for epoch in solution_epochs) / len(solution_epochs)

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
        "valid_epochs": len(solution_epochs),
        "reference_epochs": len(reference),
        "matched_epochs": len(matched),
        "fixed_epochs": matched_fixed_epochs,
        "fix_rate_pct": rounded(float(summary["fix_rate_pct"])),
        "median_h_m": rounded(float(summary["median_h_m"])),
        "p95_h_m": rounded(float(summary["p95_h_m"])),
        "max_h_m": rounded(float(summary["max_h_m"])),
        "median_abs_up_m": rounded(float(summary["median_abs_up_m"])),
        "p95_abs_up_m": rounded(float(summary["p95_abs_up_m"])),
        "mean_up_m": rounded(float(summary["mean_up_m"])),
        "mean_satellites": rounded(mean_satellites),
        "match_tolerance_s": rounded(args.match_tolerance_s),
    }
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

    if failures:
        raise SystemExit(
            "PPC demo checks failed:\n" + "\n".join(f"  - {failure}" for failure in failures)
        )


def main() -> int:
    args = parse_args()
    rover, base, nav, reference_csv, out, summary_json = resolve_paths(args)
    run_dir = resolve_run_dir(args)

    ensure_input_exists(reference_csv, "PPC reference CSV", ROOT_DIR)
    if args.max_epochs == 0:
        raise SystemExit("--max-epochs must be positive or -1")

    if args.use_existing_solution:
        ensure_input_exists(out, "existing solution file", ROOT_DIR)
    else:
        ensure_input_exists(rover, "PPC rover observation file", ROOT_DIR)
        ensure_input_exists(nav, "PPC navigation file", ROOT_DIR)
        if base is not None:
            ensure_input_exists(base, "PPC base observation file", ROOT_DIR)
        if args.sp3 is not None:
            ensure_input_exists(args.sp3, "PPP SP3 file", ROOT_DIR)
        if args.clk is not None:
            ensure_input_exists(args.clk, "PPP CLK file", ROOT_DIR)
        if args.antex is not None:
            ensure_input_exists(args.antex, "PPP ANTEX file", ROOT_DIR)
        if args.blq is not None:
            ensure_input_exists(args.blq, "PPP BLQ file", ROOT_DIR)

        out.parent.mkdir(parents=True, exist_ok=True)
        run_solver(args, rover, base, nav, out)

    payload = build_summary_payload(args, run_dir, rover, base, nav, reference_csv, out, summary_json)
    enforce_summary_requirements(payload, args)

    print("Finished PPC-Dataset demo.")
    print(f"  dataset: {payload['dataset']}")
    print(f"  solver: {args.solver}")
    print(f"  solution: {out}")
    print(f"  summary: {summary_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
