#!/usr/bin/env python3
"""navi.776 C: offline GNSS-IMU constant time-offset search.

Runs gnss_fuse with --tc-ins-time-update (or --tc-closed-loop) over a grid of
candidate --imu-time-offset values on a bounded epoch prefix, parses the
imu_time_offset_score line from stdout/log, and picks the offset minimizing
J(dt) = mean squared Kalman position correction over INS-time-update epochs
(Motooka 2026, NAVIGATION navi.776).

Default is a coarse->fine search: 100 ms steps over [--offset-min,
--offset-max], then 20 ms steps over +/-120 ms around the coarse argmin.
Pass --coarse-fine off for the paper-faithful exhaustive 20 ms sweep.

Sanity guards:
- a candidate whose applied_updates/correction coverage is below
  --min-coverage of the max across candidates is rejected as meaningless;
- a warning is printed when the argmin sits on the search boundary;
- rerun on a second disjoint window (different --max-epochs, or another run
  of the same receiver family) and accept dt only if the argmins agree
  within ~2 fine steps.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import os
import re
import subprocess
import sys
from pathlib import Path

SCORE_RE = re.compile(
    r"imu_time_offset_score:\s*offset_s=(?P<offset>[-0-9.eE+]+)"
    r"\s+J_mean_sq_correction_m2=(?P<j>[-0-9.eE+na]+)"
    r"\s+correction_epochs=(?P<epochs>\d+)"
    r"\s+applied_updates=(?P<applied>\d+)"
)


def build_candidates(args: argparse.Namespace) -> list[float]:
    def frange(lo: float, hi: float, step: float) -> list[float]:
        values = []
        v = lo
        while v <= hi + 1e-9:
            values.append(round(v, 6))
            v += step
        return values

    if args.coarse_fine == "off":
        return frange(args.offset_min, args.offset_max, args.offset_step)
    return frange(args.offset_min, args.offset_max, args.coarse_step)


def run_candidate(args: argparse.Namespace, offset: float, out_dir: Path) -> dict:
    tag = f"dt{offset:+.3f}".replace("+", "p").replace("-", "m").replace(".", "_")
    log_path = out_dir / f"cand_{tag}.log"
    command = [
        # Windows CreateProcess rejects forward-slash relative paths.
        os.path.abspath(args.gnss_fuse),
        "--data-dir", args.data_dir,
        "--lever-arm", args.lever_arm,
        "--preset", args.preset,
        "--ratio", str(args.ratio),
        "--max-subset-ar-drop-steps", str(args.max_subset_ar_drop_steps),
        "--rtk-snr-weighting",
        "--no-arfilter",
        "--max-epochs", str(args.max_epochs),
        "--imu-time-offset", str(offset),
        "--out", str(out_dir / f"cand_{tag}_fused.pos"),
    ]
    command.extend(args.coupling_flags.split())
    result = subprocess.run(command, capture_output=True, text=True)
    log_path.write_text(result.stdout + "\n--- stderr ---\n" + result.stderr,
                        encoding="utf-8")
    row = {"offset_s": offset, "j_m2": float("nan"), "correction_epochs": 0,
           "applied_updates": 0, "exit_code": result.returncode}
    match = SCORE_RE.search(result.stdout)
    if match:
        row["j_m2"] = float(match.group("j"))
        row["correction_epochs"] = int(match.group("epochs"))
        row["applied_updates"] = int(match.group("applied"))
    return row


def sweep(args: argparse.Namespace, candidates: list[float], out_dir: Path) -> list[dict]:
    rows: list[dict] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = {pool.submit(run_candidate, args, c, out_dir): c for c in candidates}
        for future in concurrent.futures.as_completed(futures):
            row = future.result()
            rows.append(row)
            print(f"  dt={row['offset_s']:+.3f}s J={row['j_m2']:.6f} m^2 "
                  f"corr_epochs={row['correction_epochs']} "
                  f"applied={row['applied_updates']} exit={row['exit_code']}")
    rows.sort(key=lambda r: r["offset_s"])
    return rows


def pick_argmin(rows: list[dict], min_coverage: float) -> dict | None:
    valid = [r for r in rows if r["exit_code"] == 0 and r["correction_epochs"] > 0
             and r["j_m2"] == r["j_m2"]]  # NaN check
    if not valid:
        return None
    max_epochs = max(r["correction_epochs"] for r in valid)
    covered = [r for r in valid
               if r["correction_epochs"] >= min_coverage * max_epochs]
    if not covered:
        return None
    return min(covered, key=lambda r: r["j_m2"])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", required=True)
    parser.add_argument("--lever-arm", required=True)
    parser.add_argument("--gnss-fuse",
                        default="build/clang-ninja/apps/gnss_fuse.exe")
    parser.add_argument("--preset", default="low-cost")
    parser.add_argument("--ratio", type=float, default=2.4)
    parser.add_argument("--max-subset-ar-drop-steps", type=int, default=18)
    parser.add_argument("--coupling-flags", default="--tc-ins-time-update",
                        help="INS coupling flags; J needs the INS time update "
                             "active (e.g. '--tc-closed-loop "
                             "--tc-velocity-states').")
    parser.add_argument("--offset-min", type=float, default=-1.0)
    parser.add_argument("--offset-max", type=float, default=1.0)
    parser.add_argument("--offset-step", type=float, default=0.02)
    parser.add_argument("--coarse-step", type=float, default=0.10)
    parser.add_argument("--fine-half-window", type=float, default=0.12)
    parser.add_argument("--coarse-fine", choices=["on", "off"], default="on")
    parser.add_argument("--max-epochs", type=int, default=3000)
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--min-coverage", type=float, default=0.8)
    parser.add_argument("--out-dir", default="output/imu_offset_search")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    candidates = build_candidates(args)
    print(f"Coarse sweep: {len(candidates)} candidates "
          f"[{args.offset_min}, {args.offset_max}]")
    rows = sweep(args, candidates, out_dir)

    best = pick_argmin(rows, args.min_coverage)
    if best is None:
        print("ERROR: no candidate produced a usable J "
              "(check that the INS time update is active).", file=sys.stderr)
        return 1

    if args.coarse_fine == "on":
        lo = max(args.offset_min, best["offset_s"] - args.fine_half_window)
        hi = min(args.offset_max, best["offset_s"] + args.fine_half_window)
        fine = [round(lo + i * args.offset_step, 6)
                for i in range(int(round((hi - lo) / args.offset_step)) + 1)]
        fine = [c for c in fine if not any(abs(c - r["offset_s"]) < 1e-9 for r in rows)]
        print(f"Fine sweep: {len(fine)} candidates [{lo}, {hi}]")
        rows.extend(sweep(args, fine, out_dir))
        rows.sort(key=lambda r: r["offset_s"])
        best = pick_argmin(rows, args.min_coverage)

    csv_path = out_dir / "imu_offset_search.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["offset_s", "j_m2",
                                               "correction_epochs",
                                               "applied_updates", "exit_code"])
        writer.writeheader()
        writer.writerows(rows)
    print(f"CSV written: {csv_path}")

    assert best is not None
    print(f"ARGMIN: dt={best['offset_s']:+.3f} s "
          f"J={best['j_m2']:.6f} m^2 "
          f"correction_epochs={best['correction_epochs']}")
    span = 1e-9
    if (abs(best["offset_s"] - args.offset_min) < span or
            abs(best["offset_s"] - args.offset_max) < span):
        print("WARNING: argmin sits on the search boundary -- widen the "
              "range; this result is not trustworthy.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
