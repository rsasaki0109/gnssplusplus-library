#!/usr/bin/env python3
"""rtkplot-style comparison figures for GNSS/IMU FGO parity per-epoch dumps.

Reads the per-epoch CSV produced by `gnss_fgo_parity --dump-csv <path>`
(columns: tow,status,e_err_m,n_err_m,u_err_m,horiz_err_m,e_pos_m,n_pos_m,
ref_e_pos_m,ref_n_pos_m -- see apps/native/gnss_fgo_parity.cpp's dumpEpochCsv()) and
renders one 2-panel PNG per run:

- Top: 2D ground track (East vs North, meters, equal aspect). The
  reference.csv trajectory (thin gray line) plus our solution epochs as dots,
  colored green for FIXED and orange for FLOAT.
- Bottom: horizontal error vs time [min from run start], log-scale y, same
  FIX/FLOAT colors, with a dashed line at 0.5 m.

The title reports fix-rate, <50cm fraction, and fixed-only RMS computed
directly from the CSV, so it always matches what gnss_fgo_parity printed for
that run (same alignment, same epochs).

Usage:
    python scripts/plot_fgo_parity_runs.py RUN1.csv RUN2.csv RUN3.csv OUT_DIR \\
        --labels "Tokyo run1" "Tokyo run2" "Tokyo run3"
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

COLOR_FIXED = "#2ca02c"
COLOR_FLOAT = "#ff7f0e"
COLOR_REFERENCE = "#999999"


class RunData:
    __slots__ = ("tow", "status", "horiz", "e_pos", "n_pos", "ref_e_pos", "ref_n_pos")

    def __init__(self) -> None:
        self.tow: list[float] = []
        self.status: list[str] = []
        self.horiz: list[float] = []
        self.e_pos: list[float] = []
        self.n_pos: list[float] = []
        self.ref_e_pos: list[float] = []
        self.ref_n_pos: list[float] = []


def load_dump_csv(path: Path) -> RunData:
    data = RunData()
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            data.tow.append(float(row["tow"]))
            data.status.append(row["status"])
            data.horiz.append(float(row["horiz_err_m"]))
            data.e_pos.append(float(row["e_pos_m"]))
            data.n_pos.append(float(row["n_pos_m"]))
            data.ref_e_pos.append(float(row["ref_e_pos_m"]))
            data.ref_n_pos.append(float(row["ref_n_pos_m"]))
    return data


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csvs", nargs=3, type=Path, help="three gnss_fgo_parity --dump-csv outputs, one per run")
    parser.add_argument("out_dir", type=Path, help="directory to write the PNGs into")
    parser.add_argument(
        "--labels",
        nargs=3,
        default=["Tokyo run1", "Tokyo run2", "Tokyo run3"],
        help="figure title prefixes, one per CSV",
    )
    parser.add_argument(
        "--names",
        nargs=3,
        default=[
            "gnss_imu_fgo_tokyo_run1.png",
            "gnss_imu_fgo_tokyo_run2.png",
            "gnss_imu_fgo_tokyo_run3.png",
        ],
        help="output PNG filenames (written under out_dir), one per CSV",
    )
    parser.add_argument("--dpi", type=int, default=100, help="output DPI (figure is 14x10 in => dpi*14 x dpi*10 px)")
    return parser.parse_args()


def plot_run(data: RunData, title: str, out_path: Path, dpi: int) -> None:
    n = len(data.tow)
    is_fixed = [s == "FIXED" for s in data.status]
    n_fixed = sum(is_fixed)
    n_float = n - n_fixed
    fix_rate_pct = 100.0 * n_fixed / n if n else 0.0
    under50_pct = 100.0 * sum(1 for h in data.horiz if h < 0.5) / n if n else 0.0
    fixed_sq = [h * h for h, f in zip(data.horiz, is_fixed) if f]
    fixed_rms = math.sqrt(sum(fixed_sq) / len(fixed_sq)) if fixed_sq else float("nan")

    t0 = data.tow[0] if data.tow else 0.0
    t_min = [(t - t0) / 60.0 for t in data.tow]

    e_fix = [e for e, f in zip(data.e_pos, is_fixed) if f]
    n_fix = [v for v, f in zip(data.n_pos, is_fixed) if f]
    e_flt = [e for e, f in zip(data.e_pos, is_fixed) if not f]
    n_flt = [v for v, f in zip(data.n_pos, is_fixed) if not f]
    t_fix = [t for t, f in zip(t_min, is_fixed) if f]
    h_fix = [h for h, f in zip(data.horiz, is_fixed) if f]
    t_flt = [t for t, f in zip(t_min, is_fixed) if not f]
    h_flt = [h for h, f in zip(data.horiz, is_fixed) if not f]

    fig, (ax_track, ax_err) = plt.subplots(2, 1, figsize=(14, 10))
    fig.patch.set_facecolor("white")

    ax_track.plot(data.ref_e_pos, data.ref_n_pos, color=COLOR_REFERENCE, linewidth=1.0, zorder=1, label="Reference")
    ax_track.scatter(e_flt, n_flt, s=5, c=COLOR_FLOAT, linewidths=0, zorder=2, rasterized=True, label=f"FLOAT (n={n_float})")
    ax_track.scatter(e_fix, n_fix, s=5, c=COLOR_FIXED, linewidths=0, zorder=3, rasterized=True, label=f"FIXED (n={n_fixed})")
    ax_track.set_xlabel("East [m]")
    ax_track.set_ylabel("North [m]")
    ax_track.set_title("Ground track (East vs North)")
    ax_track.set_aspect("equal", adjustable="datalim")
    ax_track.grid(True, color="#e9e9e9", linewidth=0.6)
    ax_track.legend(loc="best", fontsize=9, markerscale=2)

    ax_err.scatter(t_flt, h_flt, s=5, c=COLOR_FLOAT, linewidths=0, zorder=2, rasterized=True, label=f"FLOAT (n={n_float})")
    ax_err.scatter(t_fix, h_fix, s=5, c=COLOR_FIXED, linewidths=0, zorder=3, rasterized=True, label=f"FIXED (n={n_fixed})")
    ax_err.axhline(0.5, color="black", linestyle="--", linewidth=1.0, zorder=1)
    ax_err.text(0.995, 0.5, "0.5 m", transform=ax_err.get_yaxis_transform(), ha="right", va="bottom", fontsize=8)
    ax_err.set_yscale("log")
    ax_err.set_xlabel("Time [min]")
    ax_err.set_ylabel("Horizontal error [m]")
    ax_err.grid(True, which="both", color="#e9e9e9", linewidth=0.6)
    ax_err.legend(loc="best", fontsize=9, markerscale=2)

    fig.suptitle(
        f"{title} — fix {fix_rate_pct:.1f}%, <50cm {under50_pct:.1f}%, fixed RMS {fixed_rms:.2f} m",
        fontsize=14,
    )
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)
    size_kb = out_path.stat().st_size / 1024.0
    print(
        f"{out_path}: n={n} fix={fix_rate_pct:.2f}% under50={under50_pct:.2f}% "
        f"fixedRMS={fixed_rms:.4f} m size={size_kb:.1f} KiB"
    )


def main() -> int:
    args = parse_args()
    for csv_path, label, name in zip(args.csvs, args.labels, args.names):
        data = load_dump_csv(csv_path)
        plot_run(data, label, args.out_dir / name, args.dpi)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
