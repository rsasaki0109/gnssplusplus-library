#!/usr/bin/env python3
"""Score and plot the complete six-run PPC moving-CLAS benchmark."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import generate_driving_comparison as comparison
import generate_ppc_clas_scorecard as scorecard


RUNS = (
    ("tokyo_run1", "Tokyo 1"),
    ("tokyo_run2", "Tokyo 2"),
    ("tokyo_run3", "Tokyo 3"),
    ("nagoya_run1", "Nagoya 1"),
    ("nagoya_run2", "Nagoya 2"),
    ("nagoya_run3", "Nagoya 3"),
)
MIN_INTERVAL_COVERAGE = 0.99


def rounded(value: float) -> float:
    return round(float(value), 6)


def git_revision(path: Path) -> str | None:
    completed = subprocess.run(
        ["git", "-C", str(path), "rev-parse", "HEAD"],
        check=False,
        capture_output=True,
        text=True,
    )
    revision = completed.stdout.strip()
    return revision if completed.returncode == 0 and revision else None


def validate_coverage(
    key: str,
    *,
    reference_duration: float,
    matched_duration: float,
    matched_epochs: int,
    observation_epochs: int,
    min_epoch_coverage: float = MIN_INTERVAL_COVERAGE,
) -> tuple[float, float]:
    interval_coverage = (
        matched_duration / reference_duration if reference_duration > 0.0 else 0.0
    )
    epoch_coverage = (
        matched_epochs / observation_epochs if observation_epochs > 0 else 0.0
    )
    if (
        interval_coverage < MIN_INTERVAL_COVERAGE
        or epoch_coverage < min_epoch_coverage
    ):
        raise SystemExit(
            f"Incomplete interval for {key}: matched duration {matched_duration:.1f}s "
            f"of {reference_duration:.1f}s ({100.0 * interval_coverage:.2f}%); "
            f"matched epochs {matched_epochs} of {observation_epochs} "
            f"({100.0 * epoch_coverage:.2f}%)"
        )
    return interval_coverage, epoch_coverage


def load_run(
    dataset_root: Path,
    results_dir: Path,
    key: str,
    *,
    min_epoch_coverage: float = MIN_INTERVAL_COVERAGE,
) -> dict[str, Any]:
    city, run = key.split("_", 1)
    run_dir = dataset_root / city / run
    observation_epochs = scorecard.parse_rinex_observation_epochs(
        run_dir / "rover.obs"
    )
    reference = scorecard.read_reference_csv(
        run_dir / "reference.csv", apply_lever_arm=True, city=city
    )
    pos_path = results_dir / f"{key}.pos"
    if not pos_path.exists():
        pos_path = results_dir / key / "pos" / f"{key}_parity.pos"
    solutions = scorecard.read_ppp_pos(pos_path)
    matched = comparison.match_to_reference(
        solutions, reference, scorecard.MATCH_TOLERANCE_S
    )
    reference_duration = reference[-1].tow - reference[0].tow
    matched_duration = matched[-1].tow - matched[0].tow if matched else 0.0
    interval_coverage, epoch_coverage = validate_coverage(
        key,
        reference_duration=reference_duration,
        matched_duration=matched_duration,
        matched_epochs=len(matched),
        observation_epochs=len(observation_epochs),
        min_epoch_coverage=min_epoch_coverage,
    )
    scored = matched[scorecard.ARTICLE_SKIP_EPOCHS :]
    if not scored:
        raise SystemExit(f"No scored epochs for {key}")

    all_error = np.asarray([epoch.horiz_error_m for epoch in scored])
    fixed = [epoch for epoch in scored if epoch.status == scorecard.PPP_FIXED_STATUS]
    fixed_error = np.asarray([epoch.horiz_error_m for epoch in fixed])
    target = scorecard.MRTKLIB_TARGETS[key]
    first_tow = matched[0].tow
    metrics = {
        "duration_s": rounded(matched[-1].tow - first_tow),
        "reference_duration_s": rounded(reference_duration),
        "interval_coverage_pct": rounded(100.0 * interval_coverage),
        "epoch_coverage_pct": rounded(100.0 * epoch_coverage),
        "observation_epochs": len(observation_epochs),
        "solution_epochs": len(solutions),
        "matched_epochs": len(matched),
        "scored_epochs": len(scored),
        "fixed_epochs": len(fixed),
        "fix_pct": rounded(100.0 * len(fixed) / len(scored)),
        "rms2d_fixed_m": rounded(np.sqrt(np.mean(fixed_error**2))) if len(fixed) else None,
        "p68_fixed_m": rounded(np.percentile(fixed_error, 68)) if len(fixed) else None,
        "p95_fixed_m": rounded(np.percentile(fixed_error, 95)) if len(fixed) else None,
        "max_fixed_m": rounded(np.max(fixed_error)) if len(fixed) else None,
        "fixed_over_1m": int(np.count_nonzero(fixed_error > 1.0)),
        "rms2d_all_m": rounded(np.sqrt(np.mean(all_error**2))),
        "p95_all_m": rounded(np.percentile(all_error, 95)),
        "ttff_30_s": scorecard.compute_ttff_s(
            scored, scorecard.PPP_FIXED_STATUS
        ),
    }
    return {
        "key": key,
        "reference": reference,
        "matched": matched,
        "scored": scored,
        "all_error": all_error,
        "fixed_error": fixed_error,
        "metrics": metrics,
        "mrtklib_v0_4_2": target,
    }


def write_trajectory_figure(runs: list[dict[str, Any]], output: Path) -> None:
    plt.style.use("seaborn-v0_8-whitegrid")
    figure, axes = plt.subplots(2, 3, figsize=(14.4, 9.0))
    for ax, run, (_, label) in zip(axes.flat, runs, RUNS, strict=True):
        reference = run["reference"]
        truth = comparison.trajectory_enu(reference, reference[0])
        matched = run["matched"]
        solution_xy = np.asarray(
            [[epoch.traj_east_m, epoch.traj_north_m] for epoch in matched]
        )
        fixed = np.asarray(
            [epoch.status == scorecard.PPP_FIXED_STATUS for epoch in matched]
        )
        ax.plot(
            truth[:, 0],
            truth[:, 1],
            color="#111827",
            linewidth=1.7,
            label="PPC antenna truth",
        )
        ax.plot(solution_xy[:, 0], solution_xy[:, 1], color="#d97706", linewidth=0.75,
                alpha=0.85, label="libgnss++")
        ax.scatter(solution_xy[fixed, 0], solution_xy[fixed, 1], s=3.5,
                   color="#16a34a", alpha=0.7, label="FIX", zorder=3)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_title(label)
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
    handles, labels = axes.flat[0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="lower center", ncol=3)
    figure.suptitle("PPC moving CLAS — complete trajectories", fontsize=15, fontweight="bold")
    figure.tight_layout(rect=(0, 0.055, 1, 0.96))
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(figure)


def write_metric_figure(runs: list[dict[str, Any]], output: Path) -> None:
    plt.style.use("seaborn-v0_8-whitegrid")
    labels = [label for _, label in RUNS]
    x = np.arange(len(runs))
    width = 0.36
    fields = (
        ("fix_pct", "fix_pct", "FIX rate (%)"),
        ("rms2d_fixed_m", "rms2d_m", "FIX RMS2D (m)"),
        ("p68_fixed_m", "sigma2d_m", "FIX 68th percentile (m)"),
    )
    figure, axes = plt.subplots(1, 3, figsize=(15.2, 4.8))
    for ax, (local_field, mrtk_field, title) in zip(axes, fields, strict=True):
        local = [run["metrics"][local_field] or 0.0 for run in runs]
        mrtk = [run["mrtklib_v0_4_2"][mrtk_field] for run in runs]
        ax.bar(x - width / 2, local, width, color="#d97706", label="libgnss++")
        ax.bar(x + width / 2, mrtk, width, color="#2563eb", label="MRTKLIB v0.4.2")
        ax.set_title(title)
        ax.set_xticks(x, labels, rotation=35, ha="right")
        ax.set_axisbelow(True)
    handles, legend_labels = axes[0].get_legend_handles_labels()
    figure.legend(handles, legend_labels, loc="lower center", ncol=2)
    figure.suptitle("PPC full-run CLAS comparison", fontsize=15, fontweight="bold")
    figure.text(
        0.5,
        0.015,
        "Precision reference note: libgnss++ uses vehicle→antenna lever-arm correction; "
        "the published MRTKLIB values do not.",
        ha="center",
        fontsize=8.5,
        color="#4b5563",
    )
    figure.tight_layout(rect=(0, 0.12, 1, 0.94))
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(figure)


def write_error_figure(runs: list[dict[str, Any]], output: Path) -> None:
    plt.style.use("seaborn-v0_8-whitegrid")
    figure, axes = plt.subplots(2, 3, figsize=(14.4, 8.2), sharey=True)
    for ax, run, (_, label) in zip(axes.flat, runs, RUNS, strict=True):
        scored = run["scored"]
        elapsed = np.asarray([epoch.tow - scored[0].tow for epoch in scored])
        errors = np.maximum(
            np.asarray([epoch.horiz_error_m for epoch in scored]), 1e-4
        )
        fixed = np.asarray(
            [epoch.status == scorecard.PPP_FIXED_STATUS for epoch in scored]
        )
        ax.plot(elapsed, errors, color="#d97706", linewidth=0.55, alpha=0.75,
                label="All solutions")
        ax.scatter(elapsed[fixed], errors[fixed], s=4.0, color="#16a34a",
                   alpha=0.75, label="FIX")
        ax.axhline(1.0, color="#dc2626", linewidth=0.8, linestyle="--", label="1 m")
        ax.set_yscale("log")
        ax.set_title(label)
        ax.set_xlabel("Elapsed scored time (s)")
        ax.set_ylabel("Horizontal error (m)")
    handles, labels = axes.flat[0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="lower center", ncol=3)
    figure.suptitle(
        "PPC moving CLAS — horizontal error after common warm-up",
        fontsize=15,
        fontweight="bold",
    )
    figure.tight_layout(rect=(0, 0.055, 1, 0.96))
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(figure)


def markdown_value(value: float | None, suffix: str = "") -> str:
    if value is None:
        return "—"
    return f"{value:.3f}{suffix}"


def write_markdown_table(
    runs: list[dict[str, Any]], aggregate: dict[str, Any], output: Path
) -> None:
    lines = [
        "| Run | Coverage (time / epochs) | libgnss++ FIX | MRTKLIB FIX | libgnss++ FIX RMS2D* | MRTKLIB RMS2D† | libgnss++ FIX p68* | MRTKLIB p68† | libgnss++ TTFF | MRTKLIB TTFF |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for run, (_, label) in zip(runs, RUNS, strict=True):
        local = run["metrics"]
        mrtk = run["mrtklib_v0_4_2"]
        lines.append(
            "| " + " | ".join(
                [
                    label,
                    f"{local['interval_coverage_pct']:.3f}% / {local['epoch_coverage_pct']:.3f}%",
                    markdown_value(local["fix_pct"], "%"),
                    markdown_value(mrtk["fix_pct"], "%"),
                    markdown_value(local["rms2d_fixed_m"], " m"),
                    markdown_value(mrtk["rms2d_m"], " m"),
                    markdown_value(local["p68_fixed_m"], " m"),
                    markdown_value(mrtk["sigma2d_m"], " m"),
                    markdown_value(local["ttff_30_s"], " s"),
                    markdown_value(mrtk["ttff_s"], " s"),
                ]
            ) + " |"
        )
    lines.append(
        "| **Six-run aggregate** | — | "
        f"**{markdown_value(aggregate['fix_pct'], '%')}** | — | "
        f"**{markdown_value(aggregate['rms2d_fixed_m'], ' m')}** | — | "
        f"**{markdown_value(aggregate['p68_fixed_m'], ' m')}** | — | — | — |"
    )
    lines.extend(
        [
            "",
            "\\* libgnss++ precision uses PPC vehicle truth transformed to the antenna phase center.",
            "† Published MRTKLIB v0.4.2 precision uses the unmodified PPC reference point; precision columns are contextual rather than reference-identical.",
        ]
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, default=Path("data/PPC-Dataset"))
    parser.add_argument("--results-dir", type=Path, required=True)
    parser.add_argument("--metrics", type=Path, required=True)
    parser.add_argument("--markdown", type=Path, required=True)
    parser.add_argument("--trajectory-figure", type=Path, required=True)
    parser.add_argument("--error-figure", type=Path, required=True)
    parser.add_argument("--metric-figure", type=Path, required=True)
    parser.add_argument(
        "--min-epoch-coverage",
        type=float,
        default=MIN_INTERVAL_COVERAGE,
        help=(
            "Override the epoch-coverage acceptance threshold only; interval "
            "coverage still requires MIN_INTERVAL_COVERAGE "
            f"({MIN_INTERVAL_COVERAGE})."
        ),
    )
    args = parser.parse_args()

    runs = [
        load_run(
            args.dataset_root,
            args.results_dir,
            key,
            min_epoch_coverage=args.min_epoch_coverage,
        )
        for key, _ in RUNS
    ]
    all_errors = np.concatenate([run["all_error"] for run in runs])
    fixed_errors = np.concatenate([run["fixed_error"] for run in runs])
    total_scored = sum(run["metrics"]["scored_epochs"] for run in runs)
    total_fixed = sum(run["metrics"]["fixed_epochs"] for run in runs)
    payload = {
        "scoring": {
            "scope": "complete available interval of all six PPC runs",
            "dataset_revision": git_revision(args.dataset_root),
            "solver_profile": [
                "--kinematic",
                "--use-dynamics-model",
                "--no-ionosphere-free",
                "--estimate-ionosphere",
                "--estimate-troposphere",
                "--enable-ar",
                "--ar-method wlnl",
                "--ar-ratio-threshold 3.0",
                "--clas-osr",
                "--emit-epoch-time",
            ],
            "parity_environment": scorecard.PARITY_ENV,
            "reference": "PPC vehicle reference transformed to the antenna phase center",
            "lever_arm_body_m": {
                city: scorecard.lever_arm_for_city(city).tolist()
                for city in ("tokyo", "nagoya")
            },
            "warmup": f"first {scorecard.ARTICLE_SKIP_EPOCHS} matched epochs discarded per run",
            "match_tolerance_s": scorecard.MATCH_TOLERANCE_S,
            "fixed_status": scorecard.PPP_FIXED_STATUS,
            "ttff": f"first run of {scorecard.ARTICLE_TTFF_CONSECUTIVE_FIX_EPOCHS} consecutive FIX epochs",
            "comparison_source": "https://zenn.dev/hatognss/articles/7a54dd82606faf",
            "comparison_caveat": "MRTKLIB article values use the published reference without this lever-arm transform",
        },
        "runs": {
            run["key"]: {
                "libgnssplusplus": run["metrics"],
                "mrtklib_v0_4_2_article": run["mrtklib_v0_4_2"],
            }
            for run in runs
        },
        "aggregate": {
            "scored_epochs": total_scored,
            "fixed_epochs": total_fixed,
            "fix_pct": rounded(100.0 * total_fixed / total_scored),
            "rms2d_fixed_m": rounded(np.sqrt(np.mean(fixed_errors**2)))
            if len(fixed_errors) else None,
            "p68_fixed_m": rounded(np.percentile(fixed_errors, 68))
            if len(fixed_errors) else None,
            "p95_fixed_m": rounded(np.percentile(fixed_errors, 95))
            if len(fixed_errors) else None,
            "max_fixed_m": rounded(np.max(fixed_errors))
            if len(fixed_errors) else None,
            "fixed_over_1m": int(np.count_nonzero(fixed_errors > 1.0)),
            "rms2d_all_m": rounded(np.sqrt(np.mean(all_errors**2))),
            "p95_all_m": rounded(np.percentile(all_errors, 95)),
        },
    }
    args.metrics.parent.mkdir(parents=True, exist_ok=True)
    args.metrics.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    write_markdown_table(runs, payload["aggregate"], args.markdown)
    write_trajectory_figure(runs, args.trajectory_figure)
    write_error_figure(runs, args.error_figure)
    write_metric_figure(runs, args.metric_figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
