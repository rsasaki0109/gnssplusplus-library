#!/usr/bin/env python3
"""Run and validate a mixed-GNSS short-baseline static sign-off."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys

from gnss_runtime import resolve_gnss_command


ROOT_DIR = Path(__file__).resolve().parent.parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--rover",
        type=Path,
        default=ROOT_DIR / "data/short_baseline/TSK200JPN_R_20240010000_01D_30S_MO.rnx",
        help="Rover RINEX observation file.",
    )
    parser.add_argument(
        "--base",
        type=Path,
        default=ROOT_DIR / "data/short_baseline/TSKB00JPN_R_20240010000_01D_30S_MO.rnx",
        help="Base RINEX observation file.",
    )
    parser.add_argument(
        "--nav",
        type=Path,
        default=ROOT_DIR / "data/short_baseline/BRDC00IGS_R_20240010000_01D_MN.rnx",
        help="Navigation RINEX file.",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=ROOT_DIR / "output/short_baseline_solution.pos",
        help="Output .pos path for the libgnss++ solution.",
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=ROOT_DIR / "output/short_baseline_summary.json",
        help="Output path for a machine-readable sign-off summary JSON.",
    )
    parser.add_argument(
        "--max-epochs",
        type=int,
        default=-1,
        help="Stop after N rover epochs (default: full dataset).",
    )
    parser.add_argument(
        "--require-fix-rate-min",
        type=float,
        default=None,
        help="Fail if fixed solution rate is below this percentage.",
    )
    parser.add_argument(
        "--require-mean-error-max",
        type=float,
        default=None,
        help="Fail if mean 3D position error vs rover header exceeds this value in meters.",
    )
    parser.add_argument(
        "--require-max-error-max",
        type=float,
        default=None,
        help="Fail if max 3D position error vs rover header exceeds this value in meters.",
    )
    parser.add_argument(
        "--require-mean-sats-min",
        type=float,
        default=None,
        help="Fail if mean used-satellite count is below this value.",
    )
    return parser.parse_args()


def ensure_exists(path: Path, description: str) -> None:
    if not path.exists():
        raise SystemExit(f"Missing {description}: {path}")


def run_command(command: list[str]) -> None:
    print("+", " ".join(command))
    subprocess.run(command, check=True)


def read_approximate_position(path: Path) -> tuple[float, float, float]:
    with path.open(encoding="ascii", errors="ignore") as handle:
        for line in handle:
            if "APPROX POSITION XYZ" in line:
                fields = line[:60].split()
                if len(fields) < 3:
                    break
                return float(fields[0]), float(fields[1]), float(fields[2])
            if "END OF HEADER" in line:
                break
    raise SystemExit(f"Failed to read APPROX POSITION XYZ from {path}")


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


def rounded(value: float) -> float:
    return round(value, 6)


def build_summary_payload(args: argparse.Namespace) -> dict[str, object]:
    rover_position = read_approximate_position(args.rover)
    base_position = read_approximate_position(args.base)
    records = read_pos_records(args.out)
    if not records:
        raise SystemExit(f"No solution epochs found in {args.out}")

    errors = [
        math.dist((float(record["x"]), float(record["y"]), float(record["z"])), rover_position)
        for record in records
    ]
    fixed_count = sum(1 for record in records if int(record["status"]) == 4)
    mean_satellites = sum(int(record["satellites"]) for record in records) / len(records)

    payload = {
        "dataset": "Tsukuba short_baseline",
        "rover": str(args.rover),
        "base": str(args.base),
        "nav": str(args.nav),
        "solution_pos": str(args.out),
        "epochs": len(records),
        "fixed_epochs": fixed_count,
        "fix_rate_pct": rounded(100.0 * fixed_count / len(records)),
        "mean_position_error_m": rounded(sum(errors) / len(errors)),
        "max_position_error_m": rounded(max(errors)),
        "mean_satellites": rounded(mean_satellites),
        "rover_header_position_ecef_m": [rounded(value) for value in rover_position],
        "base_header_position_ecef_m": [rounded(value) for value in base_position],
        "header_baseline_m": rounded(math.dist(rover_position, base_position)),
    }
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload


def enforce_summary_requirements(payload: dict[str, object], args: argparse.Namespace) -> None:
    failures: list[str] = []

    if (
        args.require_fix_rate_min is not None
        and float(payload["fix_rate_pct"]) < args.require_fix_rate_min
    ):
        failures.append(
            f"fix rate {float(payload['fix_rate_pct']):.6f}% < {args.require_fix_rate_min:.6f}%"
        )
    if (
        args.require_mean_error_max is not None
        and float(payload["mean_position_error_m"]) > args.require_mean_error_max
    ):
        failures.append(
            f"mean position error {float(payload['mean_position_error_m']):.6f} m > "
            f"{args.require_mean_error_max:.6f} m"
        )
    if (
        args.require_max_error_max is not None
        and float(payload["max_position_error_m"]) > args.require_max_error_max
    ):
        failures.append(
            f"max position error {float(payload['max_position_error_m']):.6f} m > "
            f"{args.require_max_error_max:.6f} m"
        )
    if (
        args.require_mean_sats_min is not None
        and float(payload["mean_satellites"]) < args.require_mean_sats_min
    ):
        failures.append(
            f"mean satellites {float(payload['mean_satellites']):.6f} < "
            f"{args.require_mean_sats_min:.6f}"
        )

    if failures:
        message = "Short-baseline sign-off checks failed:\n" + "\n".join(
            f"  - {failure}" for failure in failures
        )
        raise SystemExit(message)


def main() -> int:
    args = parse_args()
    gnss_command = resolve_gnss_command(ROOT_DIR)

    ensure_exists(args.rover, "rover observation file")
    ensure_exists(args.base, "base observation file")
    ensure_exists(args.nav, "navigation file")
    if args.max_epochs == 0:
        raise SystemExit("--max-epochs must be positive or -1")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)

    command = [
        *gnss_command,
        "solve",
        "--rover",
        str(args.rover),
        "--base",
        str(args.base),
        "--nav",
        str(args.nav),
        "--out",
        str(args.out),
        "--mode",
        "static",
        "--no-kml",
    ]
    if args.max_epochs > 0:
        command.extend(["--max-epochs", str(args.max_epochs)])
    run_command(command)

    payload = build_summary_payload(args)
    enforce_summary_requirements(payload, args)

    print("Finished short-baseline sign-off.")
    print(f"  solution: {args.out}")
    print(f"  summary: {args.summary_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
