#!/usr/bin/env python3
"""Plot PPC 2D trajectories by GNSS status with optional wrong-FIX marking."""

from __future__ import annotations

import argparse
import csv
from collections import Counter
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


RUNS: tuple[tuple[str, str, str], ...] = (
    ("tokyo_run1", "Tokyo run1", "tokyo/run1"),
    ("tokyo_run2", "Tokyo run2", "tokyo/run2"),
    ("tokyo_run3", "Tokyo run3", "tokyo/run3"),
    ("nagoya_run1", "Nagoya run1", "nagoya/run1"),
    ("nagoya_run2", "Nagoya run2", "nagoya/run2"),
    ("nagoya_run3", "Nagoya run3", "nagoya/run3"),
)
STATUS_NAMES = {
    1: "SPP",
    3: "FLOAT",
    4: "FIXED",
    5: "BRIDGE",
}
PLOT_ORDER = ("SPP", "FLOAT", "FIXED", "WRONG_FIX", "BRIDGE")
COLORS = {
    "SPP": "#8f969f",
    "FLOAT": "#df9418",
    "FIXED": "#1b9e77",
    "WRONG_FIX": "#d62728",
    "BRIDGE": "#2b6cb0",
}


@dataclass(frozen=True)
class Position:
    week: int
    tow_s: float
    ecef: tuple[float, float, float]
    status: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--solution-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--title", required=True)
    parser.add_argument("--subtitle", default="")
    parser.add_argument(
        "--wrong-fix-threshold-m",
        type=float,
        default=0.10,
        help="Classify status=FIXED epochs above this 3D truth error as WRONG_FIX.",
    )
    return parser.parse_args()


def load_reference(path: Path) -> dict[tuple[int, float], tuple[float, float, float]]:
    records: dict[tuple[int, float], tuple[float, float, float]] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                week = int(row["GPS Week"])
                tow_s = round(float(row["GPS TOW (s)"]), 3)
                records[(week, tow_s)] = (
                    float(row["ECEF X (m)"]),
                    float(row["ECEF Y (m)"]),
                    float(row["ECEF Z (m)"]),
                )
            except (KeyError, ValueError):
                continue
    return records


def load_solution(path: Path) -> list[Position]:
    records: list[Position] = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 9:
                continue
            try:
                records.append(
                    Position(
                        week=int(parts[0]),
                        tow_s=round(float(parts[1]), 3),
                        ecef=(float(parts[2]), float(parts[3]), float(parts[4])),
                        status=int(parts[8]),
                    )
                )
            except ValueError:
                continue
    return records


def ecef_to_geodetic_rad(ecef: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = ecef
    a = 6378137.0
    e2 = 6.69437999014e-3
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - e2))
    for _ in range(8):
        sin_lat = math.sin(lat)
        n = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
        lat = math.atan2(z + e2 * n * sin_lat, p)
    sin_lat = math.sin(lat)
    n = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    height = p / math.cos(lat) - n
    return lat, lon, height


def ecef_to_enu(
    ecef: tuple[float, float, float],
    origin_ecef: tuple[float, float, float],
    origin_lat_rad: float,
    origin_lon_rad: float,
) -> tuple[float, float, float]:
    dx = ecef[0] - origin_ecef[0]
    dy = ecef[1] - origin_ecef[1]
    dz = ecef[2] - origin_ecef[2]
    sin_lat = math.sin(origin_lat_rad)
    cos_lat = math.cos(origin_lat_rad)
    sin_lon = math.sin(origin_lon_rad)
    cos_lon = math.cos(origin_lon_rad)
    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return east, north, up


def error_3d_m(
    lhs: tuple[float, float, float],
    rhs: tuple[float, float, float],
) -> float:
    return math.sqrt(
        (lhs[0] - rhs[0]) ** 2
        + (lhs[1] - rhs[1]) ** 2
        + (lhs[2] - rhs[2]) ** 2
    )


def classify(
    position: Position,
    reference: dict[tuple[int, float], tuple[float, float, float]],
    wrong_fix_threshold_m: float,
) -> str:
    name = STATUS_NAMES.get(position.status, f"STATUS_{position.status}")
    if position.status != 4:
        return name
    truth = reference.get((position.week, position.tow_s))
    if truth is None:
        return name
    return "WRONG_FIX" if error_3d_m(position.ecef, truth) > wrong_fix_threshold_m else "FIXED"


def sorted_labels(counter: Counter[str]) -> list[str]:
    labels = [label for label in PLOT_ORDER if counter.get(label, 0)]
    labels.extend(sorted(label for label in counter if label not in PLOT_ORDER))
    return labels


def label_counts_text(counter: Counter[str]) -> str:
    return ", ".join(f"{label}={counter[label]}" for label in sorted_labels(counter))


def scatter_label(ax: plt.Axes, points: list[tuple[float, float]], label: str) -> None:
    if not points:
        return
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    marker = "x" if label == "WRONG_FIX" else "."
    size = 18 if label == "WRONG_FIX" else 8
    ax.scatter(
        xs,
        ys,
        s=size,
        marker=marker,
        c=COLORS.get(label, "#444444"),
        linewidths=0.8 if label == "WRONG_FIX" else 0.0,
        label=label,
        alpha=0.95,
        zorder=4 if label == "WRONG_FIX" else 3,
    )


def plot_run(
    ax: plt.Axes,
    dataset_root: Path,
    solution_dir: Path,
    run_key: str,
    run_title: str,
    run_relpath: str,
    wrong_fix_threshold_m: float,
) -> Counter[str]:
    reference = load_reference(dataset_root / run_relpath / "reference.csv")
    solution = load_solution(solution_dir / f"{run_key}.pos")
    if not reference or not solution:
        ax.set_title(f"{run_title}\nmissing data", fontsize=12)
        ax.axis("off")
        return Counter()
    origin_ecef = next(iter(reference.values()))
    origin_lat, origin_lon, _ = ecef_to_geodetic_rad(origin_ecef)
    by_label: dict[str, list[tuple[float, float]]] = {}
    counts: Counter[str] = Counter()
    for position in solution:
        label = classify(position, reference, wrong_fix_threshold_m)
        east, north, _ = ecef_to_enu(position.ecef, origin_ecef, origin_lat, origin_lon)
        by_label.setdefault(label, []).append((east, north))
        counts[label] += 1
    ax.plot(
        [ecef_to_enu(ecef, origin_ecef, origin_lat, origin_lon)[0] for ecef in reference.values()],
        [ecef_to_enu(ecef, origin_ecef, origin_lat, origin_lon)[1] for ecef in reference.values()],
        color="#c8c8c8",
        linewidth=0.8,
        zorder=1,
    )
    for label in sorted_labels(counts):
        scatter_label(ax, by_label.get(label, []), label)
    ax.set_title(f"{run_title}\n{label_counts_text(counts)}", fontsize=12)
    ax.set_xlabel("East offset (m)")
    ax.set_ylabel("North offset (m)")
    ax.grid(True, color="#e9e9e9", linewidth=0.6)
    ax.set_aspect("equal", adjustable="datalim")
    return counts


def build_legend(fig: plt.Figure, labels: Iterable[str]) -> None:
    handles = []
    for label in labels:
        marker = "x" if label == "WRONG_FIX" else "."
        size = 8 if label == "WRONG_FIX" else 10
        handles.append(
            plt.Line2D(
                [0],
                [0],
                marker=marker,
                color="none",
                markeredgecolor=COLORS.get(label, "#444444"),
                markerfacecolor=COLORS.get(label, "#444444"),
                markersize=size,
                linestyle="None",
                label=label,
            )
        )
    fig.legend(handles=handles, loc="upper center", ncol=len(handles), frameon=False, bbox_to_anchor=(0.5, 0.925))


def main() -> int:
    args = parse_args()
    fig, axes = plt.subplots(2, 3, figsize=(19, 10), constrained_layout=False)
    total_counts: Counter[str] = Counter()
    for ax, run in zip(axes.flat, RUNS, strict=True):
        total_counts.update(
            plot_run(
                ax,
                args.dataset_root,
                args.solution_dir,
                run[0],
                run[1],
                run[2],
                args.wrong_fix_threshold_m,
            )
        )
    title = args.title
    if args.subtitle:
        title = f"{title}\n{args.subtitle}"
    fig.suptitle(title, fontsize=20, y=0.99)
    build_legend(fig, sorted_labels(total_counts))
    fig.tight_layout(rect=(0.02, 0.02, 0.98, 0.90))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180)
    print(f"wrote {args.output}")
    print(label_counts_text(total_counts))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
