#!/usr/bin/env python3
"""Scan UrbanNav Odaiba in fixed-size windows and report reference-based metrics."""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

from gnss_runtime import resolve_gnss_command


ROOT_DIR = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = ROOT_DIR / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from generate_driving_comparison import match_to_reference, read_libgnss_pos, read_reference_csv, read_rtklib_pos, summarize


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--rover",
        type=Path,
        default=ROOT_DIR / "data/driving/Tokyo_Data/Odaiba/rover_trimble.obs",
        help="Rover RINEX observation file.",
    )
    parser.add_argument(
        "--base",
        type=Path,
        default=ROOT_DIR / "data/driving/Tokyo_Data/Odaiba/base_trimble.obs",
        help="Base RINEX observation file.",
    )
    parser.add_argument(
        "--nav",
        type=Path,
        default=ROOT_DIR / "data/driving/Tokyo_Data/Odaiba/base.nav",
        help="Navigation RINEX file.",
    )
    parser.add_argument(
        "--reference-csv",
        type=Path,
        default=ROOT_DIR / "data/driving/Tokyo_Data/Odaiba/reference.csv",
        help="Ground-truth CSV from UrbanNav.",
    )
    parser.add_argument(
        "--rtklib-pos",
        type=Path,
        default=ROOT_DIR / "output/driving_rtklib_rtk.pos",
        help="Optional RTKLIB .pos file for side-by-side window stats.",
    )
    parser.add_argument(
        "--mode",
        default="kinematic",
        choices=("auto", "kinematic", "static"),
        help="Mode passed through to gnss solve.",
    )
    parser.add_argument(
        "--glonass-ar",
        default="off",
        choices=("off", "on", "autocal"),
        help="GLONASS ambiguity resolution mode passed through to gnss solve.",
    )
    parser.add_argument(
        "--window-size",
        type=int,
        default=1000,
        help="Epoch count per scan window.",
    )
    parser.add_argument(
        "--step",
        type=int,
        default=1000,
        help="Epoch step between windows.",
    )
    parser.add_argument(
        "--start-epoch",
        type=int,
        default=0,
        help="First rover epoch index to scan.",
    )
    parser.add_argument(
        "--end-epoch",
        type=int,
        default=12000,
        help="Stop when skip-epochs reaches this value.",
    )
    parser.add_argument(
        "--match-tolerance",
        type=float,
        default=0.15,
        help="Reference matching tolerance in seconds.",
    )
    parser.add_argument(
        "--output-csv",
        type=Path,
        default=ROOT_DIR / "output/odaiba_window_scan.csv",
        help="CSV output for per-window metrics.",
    )
    return parser.parse_args()


def ensure_exists(path: Path, description: str) -> None:
    if not path.exists():
        raise SystemExit(f"Missing {description}: {path}")


def filter_window_by_tow(matched: list, tow_start: float, tow_end: float) -> list:
    return [epoch for epoch in matched if tow_start <= epoch.tow <= tow_end]


def run_window(
    gnss_command: list[str],
    rover: Path,
    base: Path,
    nav: Path,
    mode: str,
    glonass_ar: str,
    skip_epochs: int,
    max_epochs: int,
    out_path: Path,
) -> tuple[float, str]:
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
        str(out_path),
        "--mode",
        mode,
        "--glonass-ar",
        glonass_ar,
        "--no-kml",
        "--skip-epochs",
        str(skip_epochs),
        "--max-epochs",
        str(max_epochs),
    ]
    started = time.perf_counter()
    completed = subprocess.run(command, capture_output=True, text=True, check=True)
    elapsed_s = time.perf_counter() - started
    return elapsed_s, completed.stdout


def main() -> int:
    args = parse_args()
    gnss_command = resolve_gnss_command(ROOT_DIR)

    ensure_exists(args.rover, "rover observation file")
    ensure_exists(args.base, "base observation file")
    ensure_exists(args.nav, "navigation file")
    ensure_exists(args.reference_csv, "reference csv")
    if args.window_size <= 0:
        raise SystemExit("--window-size must be > 0")
    if args.step <= 0:
        raise SystemExit("--step must be > 0")
    if args.start_epoch < 0:
        raise SystemExit("--start-epoch must be >= 0")
    if args.end_epoch <= args.start_epoch:
        raise SystemExit("--end-epoch must be > --start-epoch")

    reference = read_reference_csv(args.reference_csv)
    rtklib_matched = None
    if args.rtklib_pos.exists():
        rtklib_matched = match_to_reference(
            read_rtklib_pos(args.rtklib_pos),
            reference,
            args.match_tolerance,
        )

    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, float | int | str]] = []

    with tempfile.TemporaryDirectory(prefix="odaiba_scan_") as temp_dir:
        temp_root = Path(temp_dir)
        for skip_epochs in range(args.start_epoch, args.end_epoch, args.step):
            out_path = temp_root / f"scan_{skip_epochs:06d}.pos"
            elapsed_s, stdout = run_window(
                gnss_command,
                args.rover,
                args.base,
                args.nav,
                args.mode,
                args.glonass_ar,
                skip_epochs,
                args.window_size,
                out_path,
            )

            lib_matched = match_to_reference(
                read_libgnss_pos(out_path),
                reference,
                args.match_tolerance,
            )
            lib_summary = summarize(lib_matched, fixed_status=4, label="libgnss++")
            tow_start = lib_matched[0].tow
            tow_end = lib_matched[-1].tow
            row: dict[str, float | int | str] = {
                "skip_epochs": skip_epochs,
                "max_epochs": args.window_size,
                "tow_start": tow_start,
                "tow_end": tow_end,
                "wall_time_s": elapsed_s,
                "lib_epochs": int(lib_summary["epochs"]),
                "lib_fix_rate_pct": lib_summary["fix_rate_pct"],
                "lib_median_h_m": lib_summary["median_h_m"],
                "lib_p95_h_m": lib_summary["p95_h_m"],
                "lib_max_h_m": lib_summary["max_h_m"],
                "lib_p95_abs_up_m": lib_summary["p95_abs_up_m"],
            }
            if rtklib_matched is not None:
                rt_window = filter_window_by_tow(rtklib_matched, tow_start, tow_end)
                if rt_window:
                    rt_summary = summarize(rt_window, fixed_status=1, label="RTKLIB")
                    row.update(
                        {
                            "rtklib_epochs": int(rt_summary["epochs"]),
                            "rtklib_fix_rate_pct": rt_summary["fix_rate_pct"],
                            "rtklib_median_h_m": rt_summary["median_h_m"],
                            "rtklib_p95_h_m": rt_summary["p95_h_m"],
                            "rtklib_max_h_m": rt_summary["max_h_m"],
                            "rtklib_p95_abs_up_m": rt_summary["p95_abs_up_m"],
                            "delta_fix_rate_pct": lib_summary["fix_rate_pct"] - rt_summary["fix_rate_pct"],
                            "delta_p95_h_m": lib_summary["p95_h_m"] - rt_summary["p95_h_m"],
                            "delta_max_h_m": lib_summary["max_h_m"] - rt_summary["max_h_m"],
                        }
                    )

            rows.append(row)
            summary_bits = [
                f"skip={skip_epochs}",
                f"fix={lib_summary['fix_rate_pct']:.1f}%",
                f"median={lib_summary['median_h_m']:.3f}m",
                f"p95={lib_summary['p95_h_m']:.2f}m",
                f"max={lib_summary['max_h_m']:.2f}m",
                f"time={elapsed_s:.2f}s",
            ]
            if "rtklib_p95_h_m" in row:
                summary_bits.append(f"vs_rtklib_p95={row['delta_p95_h_m']:+.2f}m")
            print("  ".join(summary_bits))
            if os.environ.get("GNSS_SCAN_VERBOSE") == "1":
                print(stdout, end="")

    with args.output_csv.open("w", newline="") as handle:
        fieldnames = sorted({key for row in rows for key in row.keys()})
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    best_p95 = min(rows, key=lambda row: float(row["lib_p95_h_m"]))
    worst_p95 = max(rows, key=lambda row: float(row["lib_p95_h_m"]))
    worst_runtime = max(rows, key=lambda row: float(row["wall_time_s"]))
    print(f"Saved: {args.output_csv}")
    print(
        "Best window:"
        f" skip={best_p95['skip_epochs']} p95={best_p95['lib_p95_h_m']:.2f}m"
        f" fix={best_p95['lib_fix_rate_pct']:.1f}%"
    )
    print(
        "Worst window:"
        f" skip={worst_p95['skip_epochs']} p95={worst_p95['lib_p95_h_m']:.2f}m"
        f" fix={worst_p95['lib_fix_rate_pct']:.1f}%"
    )
    print(
        "Slowest window:"
        f" skip={worst_runtime['skip_epochs']} time={worst_runtime['wall_time_s']:.2f}s"
        f" p95={worst_runtime['lib_p95_h_m']:.2f}m"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
