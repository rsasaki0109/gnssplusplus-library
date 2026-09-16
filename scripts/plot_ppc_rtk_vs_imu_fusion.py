#!/usr/bin/env python3
"""PPC RTK-only vs tightly-coupled GNSS/IMU fusion comparison figure.

Reads one RTK-only `.pos`, one GNSS/IMU fusion `.pos`, and the run's
`reference.csv`, then draws a tweet-friendly four-panel comparison:

  (a) ENU trajectory: reference, RTK-only, RTK+GNSS/IMU fusion
  (b) horizontal-error CDF with P50/P95 annotations
  (c) P50/P95 horizontal error bars
  (d) 3D-<50cm fraction and PPC official score

Metric definitions are the shared PPC helpers, so the numbers agree with
`apps/gnss.py ppc-coverage-matrix` and the README tables.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

SCRIPTS_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPTS_DIR.parent
COMMANDS_DIR = ROOT_DIR / "apps" / "commands"
for path in (SCRIPTS_DIR, COMMANDS_DIR, COMMANDS_DIR / "benchmarks"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import generate_driving_comparison as comparison  # noqa: E402
import gnss_ppc_metrics as metrics  # noqa: E402

RTK_COLOR = "#1f77b4"
FUSION_COLOR = "#d62728"
REF_COLOR = "#7f7f7f"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--rtk-pos", type=Path, required=True)
    parser.add_argument("--fusion-pos", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--title", default="PPC: RTK only vs RTK + GNSS/IMU fusion")
    parser.add_argument("--rtk-label", default="RTK only")
    parser.add_argument("--fusion-label", default="RTK + GNSS/IMU (tight FGO)")
    parser.add_argument("--match-tolerance-s", type=float, default=0.11)
    parser.add_argument("--dpi", type=int, default=160)
    return parser.parse_args()


def summarize(path: Path, reference, label: str, tol: float) -> dict:
    solution = comparison.read_libgnss_pos(path)
    summary = metrics.summarize_solution_epochs(
        reference, solution, 4, label, tol, solver_wall_time_s=None
    )
    matched = comparison.match_to_reference(solution, reference, tol)
    return {"solution": solution, "summary": summary, "matched": matched}


def metric_row(summary: dict) -> dict:
    return {
        "p50": float(summary["median_h_m"]),
        "p95": float(summary["p95_h_m"]),
        "h50": float(summary["ppc_score_3d_50cm_matched_pct"]),
        "official": float(summary["ppc_official_score_pct"]),
        "fix": float(summary["fix_rate_pct"]),
        "wrong_fix": float(summary["wrong_fix_rate_pct"]),
    }


def main() -> int:
    args = parse_args()
    reference = comparison.read_reference_csv(args.reference)
    rtk = summarize(args.rtk_pos, reference, args.rtk_label, args.match_tolerance_s)
    fusion = summarize(args.fusion_pos, reference, args.fusion_label, args.match_tolerance_s)
    rtk_m, fusion_m = metric_row(rtk["summary"]), metric_row(fusion["summary"])

    origin = reference[0]
    ref_enu = comparison.trajectory_enu(reference, origin)
    rtk_enu = comparison.trajectory_enu(rtk["solution"], origin)
    fusion_enu = comparison.trajectory_enu(fusion["solution"], origin)

    thresholds = (0.5, 1.0, 2.0, 5.0, 10.0)

    def coverage(matched):
        horiz = np.array([e.horiz_error_m for e in matched])
        up = np.array([e.up_m for e in matched])
        err3d = np.hypot(horiz, up)
        return [100.0 * float(np.mean(err3d <= t)) for t in thresholds]

    rtk_cov = coverage(rtk["matched"])
    fusion_cov = coverage(fusion["matched"])
    rtk_max = max(e.horiz_error_m for e in rtk["matched"])
    fusion_max = max(e.horiz_error_m for e in fusion["matched"])
    x = np.arange(2)
    width = 0.36

    fig, axes = plt.subplots(2, 2, figsize=(14, 11))
    fig.suptitle(args.title, fontsize=20, fontweight="bold")

    # (a) trajectory
    ax = axes[0, 0]
    ax.plot(ref_enu[:, 0], ref_enu[:, 1], color=REF_COLOR, lw=4, alpha=0.7, label="Reference")
    ax.plot(rtk_enu[:, 0], rtk_enu[:, 1], color=RTK_COLOR, lw=1.4, alpha=0.9, label=f"{args.rtk_label} (P95 {rtk_m['p95']:.2f} m)")
    ax.plot(fusion_enu[:, 0], fusion_enu[:, 1], color=FUSION_COLOR, lw=1.4, alpha=0.9, label=f"{args.fusion_label} (P95 {fusion_m['p95']:.2f} m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_title("Trajectory (ENU)")
    ax.legend(fontsize=11, loc="best")
    ax.grid(alpha=0.3)

    # (b) CDF
    ax = axes[0, 1]
    rtk_cdf = comparison.cdf_xy(rtk["matched"])
    fusion_cdf = comparison.cdf_xy(fusion["matched"])
    ax.plot(rtk_cdf[0], rtk_cdf[1], color=RTK_COLOR, lw=2.5, label=args.rtk_label)
    ax.plot(fusion_cdf[0], fusion_cdf[1], color=FUSION_COLOR, lw=2.5, label=args.fusion_label)
    for value, color in ((rtk_m["p95"], RTK_COLOR), (fusion_m["p95"], FUSION_COLOR)):
        ax.axvline(value, color=color, ls="--", lw=1.2, alpha=0.8)
    ax.set_xlim(left=0, right=min(20.0, max(rtk_m["p95"], fusion_m["p95"]) * 2.5 + 1))
    ax.set_xlabel("Horizontal error [m]")
    ax.set_ylabel("CDF [%]")
    ax.set_title("Horizontal error CDF (dashed = P95)")
    ax.legend(fontsize=12, loc="lower right")
    ax.text(0.02, 0.96, f"max  {rtk_max:.0f} → {fusion_max:.0f} m",
            transform=ax.transAxes, fontsize=12, va="top",
            bbox=dict(boxstyle="round", fc="white", ec="#999999"))
    ax.grid(alpha=0.3)

    # (c) 3D-error coverage at thresholds
    ax = axes[1, 0]
    xc = np.arange(len(thresholds))
    rtk_bars = ax.bar(xc - width / 2, rtk_cov, width, color=RTK_COLOR, label=args.rtk_label)
    fusion_bars = ax.bar(xc + width / 2, fusion_cov, width, color=FUSION_COLOR, label=args.fusion_label)
    for bars in (rtk_bars, fusion_bars):
        for bar in bars:
            ax.annotate(f"{bar.get_height():.1f}", (bar.get_x() + bar.get_width() / 2, bar.get_height()),
                        ha="center", va="bottom", fontsize=11, fontweight="bold")
    ax.set_xticks(xc, [f"≤ {t:g} m" for t in thresholds])
    ax.set_ylim(0, 100)
    ax.set_xlabel("3D error threshold")
    ax.set_ylabel("Epochs within threshold [%]")
    ax.set_title("3D-error coverage")
    ax.legend(fontsize=12)
    ax.grid(alpha=0.3, axis="y")

    # (d) FIX rate and official score
    ax = axes[1, 1]
    rtk_vals = [rtk_m["fix"], rtk_m["official"]]
    fusion_vals = [fusion_m["fix"], fusion_m["official"]]
    rtk_bars = ax.bar(x - width / 2, rtk_vals, width, color=RTK_COLOR, label=args.rtk_label)
    fusion_bars = ax.bar(x + width / 2, fusion_vals, width, color=FUSION_COLOR, label=args.fusion_label)
    for bars in (rtk_bars, fusion_bars):
        for bar in bars:
            ax.annotate(f"{bar.get_height():.1f}", (bar.get_x() + bar.get_width() / 2, bar.get_height()),
                        ha="center", va="bottom", fontsize=13, fontweight="bold")
    ax.set_xticks(x, ["FIX rate [%]", "Official score [%]"])
    ax.set_ylim(0, 100)
    ax.set_ylabel("[%]")
    ax.set_title("FIX rate and official PPC score")
    ax.legend(fontsize=12)
    ax.grid(alpha=0.3, axis="y")

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=args.dpi)
    print(f"wrote {args.output}")
    print(f"{args.rtk_label}: fix {rtk_m['fix']:.2f}% P50 {rtk_m['p50']:.3f} P95 {rtk_m['p95']:.3f} "
          f"3D<50cm {rtk_m['h50']:.2f}% official {rtk_m['official']:.2f}% wrong-fix {rtk_m['wrong_fix']:.2f}%")
    print(f"{args.fusion_label}: fix {fusion_m['fix']:.2f}% P50 {fusion_m['p50']:.3f} P95 {fusion_m['p95']:.3f} "
          f"3D<50cm {fusion_m['h50']:.2f}% official {fusion_m['official']:.2f}% wrong-fix {fusion_m['wrong_fix']:.2f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
