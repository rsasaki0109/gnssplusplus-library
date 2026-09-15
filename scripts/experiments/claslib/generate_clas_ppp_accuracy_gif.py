#!/usr/bin/env python3
"""Render an animated GIF showing CLAS PPP below 10 cm without a base station.

The demo uses the public QZSS-Strategy-Office/claslib 2019-08-27 sample
(rover observations + broadcast navigation + raw QZSS L6) and the reference
`rnx2rtkp` CLAS PPP solver that ships with that repository. No local base
station is involved: the corrections arrive over the QZSS L6 signal.

The script fetches the pinned public sample, runs the reference solver, scores
the NMEA GGA stream against the published ECEF reference, and writes an
animated GIF plus a metrics JSON.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timedelta
import json
import math
import os
from pathlib import Path
import shutil
import subprocess

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle


DEFAULT_CLASLIB_REPO = "https://github.com/QZSS-Strategy-Office/claslib.git"
DEFAULT_CLASLIB_REF = "23cfd363a2db6d8d8144e292c82e9d97ca2d3015"
DEFAULT_START_DATE = "2019/08/27"
DEFAULT_START_TIME = "16:00:00"
DEFAULT_GPS_WEEK = 2068
DEFAULT_MAX_EPOCHS = 3600
DEFAULT_OBSERVATION = "0627239Q.obs"
DEFAULT_NAVIGATION = "sept_2019239.nav"
DEFAULT_L6 = "2019239Q.l6"
DEFAULT_REFERENCE_ECEF = (-3957235.3717, 3310368.2257, 3737529.7179)
DEFAULT_REFERENCE_LABEL = (
    "Published CLASLIB 2019-08-27 antenna reference "
    "(-3957235.3717, 3310368.2257, 3737529.7179)"
)

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3
FIX_QUALITY = 4
FLOAT_QUALITY = 5
TARGET_HORIZONTAL_M = 0.10
COLOR_FIX = "#16a34a"
COLOR_FLOAT = "#d97706"
COLOR_OTHER = "#2563eb"
COLOR_TARGET = "#dc2626"


@dataclass(frozen=True)
class Sample:
    week: int
    tow: float
    ecef: np.ndarray
    quality: int


@dataclass(frozen=True)
class ScoredSolution:
    samples: list[Sample]
    horizontal_error_m: np.ndarray
    east_m: np.ndarray
    north_m: np.ndarray
    elapsed_s: np.ndarray
    vertical_error_m: np.ndarray


def repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[3]


def ecef_llh(ecef: np.ndarray) -> tuple[float, float]:
    x, y, z = (float(value) for value in ecef)
    lon = math.atan2(y, x)
    radius = math.hypot(x, y)
    lat = math.atan2(z, radius * (1.0 - WGS84_E2))
    for _ in range(8):
        sin_lat = math.sin(lat)
        prime_vertical = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        lat = math.atan2(z + WGS84_E2 * prime_vertical * sin_lat, radius)
    return lat, lon


def enu_from_delta(
    delta: np.ndarray, lat: float, lon: float
) -> tuple[float, float, float]:
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    east = -sin_lon * delta[0] + cos_lon * delta[1]
    north = (
        -sin_lat * cos_lon * delta[0]
        - sin_lat * sin_lon * delta[1]
        + cos_lat * delta[2]
    )
    up = (
        cos_lat * cos_lon * delta[0] + cos_lat * sin_lon * delta[1] + sin_lat * delta[2]
    )
    return east, north, up


def dm_to_degrees(value: str, hemisphere: str) -> float:
    raw = float(value)
    degrees = int(raw // 100)
    result = degrees + (raw - degrees * 100) / 60.0
    return -result if hemisphere in ("S", "W") else result


def llh_to_ecef(lat_deg: float, lon_deg: float, height_m: float) -> np.ndarray:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    prime_vertical = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return np.array(
        [
            (prime_vertical + height_m) * cos_lat * math.cos(lon),
            (prime_vertical + height_m) * cos_lat * math.sin(lon),
            (prime_vertical * (1.0 - WGS84_E2) + height_m) * sin_lat,
        ]
    )


def run_logged(
    command: list[str], *, cwd: Path, log_path: Path
) -> subprocess.CompletedProcess[str]:
    with log_path.open("a", encoding="utf-8") as handle:
        handle.write("$ " + " ".join(command) + "\n")
    completed = subprocess.run(
        command,
        cwd=cwd,
        text=True,
        capture_output=True,
        check=False,
    )
    with log_path.open("a", encoding="utf-8") as handle:
        if completed.stdout:
            handle.write(completed.stdout)
        if completed.stderr:
            handle.write(completed.stderr)
        handle.write(f"# exit={completed.returncode}\n\n")
    return completed


def fetch_claslib(root: Path, repo: str, ref: str, log_path: Path) -> None:
    if root.exists():
        shutil.rmtree(root)
    root.parent.mkdir(parents=True, exist_ok=True)
    commands = [
        ["git", "init", str(root)],
        ["git", "-C", str(root), "remote", "add", "origin", repo],
        ["git", "-C", str(root), "fetch", "--depth", "1", "origin", ref],
        ["git", "-C", str(root), "checkout", "--detach", "FETCH_HEAD"],
    ]
    for command in commands:
        completed = run_logged(command, cwd=root.parent, log_path=log_path)
        if completed.returncode != 0:
            raise SystemExit(f"failed to fetch CLASLIB: {' '.join(command)}")


def data_root(claslib_root: Path) -> Path:
    candidate = claslib_root / "data"
    if not candidate.is_dir():
        raise SystemExit(f"missing CLASLIB data directory: {candidate}")
    return candidate


def write_config(claslib_root: Path, work_dir: Path) -> Path:
    source = claslib_root / "util" / "rnx2rtkp" / "static.conf"
    text = source.read_text(encoding="utf-8")
    prefix = str(data_root(claslib_root)) + os.sep
    text = text.replace("..\\..\\data\\", prefix)
    text = text.replace("../../data/", prefix)
    config = work_dir / "static.conf"
    config.write_text(text, encoding="utf-8")
    return config


def resolve_rnx2rtkp(claslib_root: Path, log_path: Path) -> Path:
    tool_dir = claslib_root / "util" / "rnx2rtkp"
    if os.name == "nt":
        binary = tool_dir / "rnx2rtkp.exe"
        if binary.is_file():
            return binary
        raise SystemExit(f"missing prebuilt Windows solver: {binary}")
    binary = tool_dir / "rnx2rtkp"
    if not binary.is_file():
        completed = run_logged(
            ["make", "-C", str(tool_dir)], cwd=tool_dir, log_path=log_path
        )
        if completed.returncode != 0 or not binary.is_file():
            raise SystemExit("failed to build CLASLIB rnx2rtkp")
    return binary


def end_date_time(start_date: str, start_time: str, max_epochs: int) -> tuple[str, str]:
    start = datetime.strptime(f"{start_date} {start_time}", "%Y/%m/%d %H:%M:%S")
    end = start + timedelta(seconds=max(max_epochs - 1, 0))
    return end.strftime("%Y/%m/%d"), end.strftime("%H:%M:%S")


def run_solver(
    binary: Path,
    claslib_root: Path,
    config: Path,
    output: Path,
    *,
    start_date: str,
    start_time: str,
    gps_week: int,
    max_epochs: int,
    log_path: Path,
) -> None:
    root = data_root(claslib_root)
    end_date, end_clock = end_date_time(start_date, start_time, max_epochs)
    command = [
        str(binary),
        "-ti",
        "1",
        "-ts",
        start_date,
        start_time,
        "-te",
        end_date,
        end_clock,
        "-l6w",
        str(gps_week),
        "-x",
        "0",
        "-k",
        str(config),
        "-o",
        str(output),
        str(root / DEFAULT_OBSERVATION),
        str(root / DEFAULT_NAVIGATION),
        str(root / DEFAULT_L6),
    ]
    completed = run_logged(command, cwd=binary.parent, log_path=log_path)
    if completed.returncode != 0 or not output.is_file():
        raise SystemExit(f"CLASLIB solver failed; see {log_path}")


def parse_gga(path: Path) -> list[Sample]:
    samples: list[Sample] = []
    for raw in path.read_text(encoding="ascii", errors="ignore").splitlines():
        line = raw.strip()
        if not line.startswith(("$GPGGA", "$GNGGA", "$GAGGA", "$GJGGA")):
            continue
        fields = line.split(",")
        if len(fields) < 12:
            continue
        try:
            quality = int(fields[6])
            if quality == 0:
                continue
            stamp = fields[1]
            sod = int(stamp[0:2]) * 3600.0 + int(stamp[2:4]) * 60.0 + float(stamp[4:])
            lat = dm_to_degrees(fields[2], fields[3])
            lon = dm_to_degrees(fields[4], fields[5])
            height = float(fields[9]) + float(fields[11] or 0.0)
        except (ValueError, IndexError):
            continue
        samples.append(
            Sample(
                week=0,
                tow=sod,
                ecef=llh_to_ecef(lat, lon, height),
                quality=quality,
            )
        )
    samples.sort(key=lambda item: item.tow)
    return samples


def score(samples: list[Sample], reference_ecef: np.ndarray) -> ScoredSolution:
    lat, lon = ecef_llh(reference_ecef)
    east, north, up = [], [], []
    for sample in samples:
        e, n, u = enu_from_delta(sample.ecef - reference_ecef, lat, lon)
        east.append(e)
        north.append(n)
        up.append(u)
    east_arr = np.asarray(east)
    north_arr = np.asarray(north)
    up_arr = np.asarray(up)
    elapsed = np.asarray([sample.tow - samples[0].tow for sample in samples])
    return ScoredSolution(
        samples=samples,
        horizontal_error_m=np.hypot(east_arr, north_arr),
        east_m=east_arr,
        north_m=north_arr,
        elapsed_s=elapsed,
        vertical_error_m=up_arr,
    )


def metrics_payload(
    scored: ScoredSolution,
    *,
    reference_label: str,
    claslib_ref: str,
    dataset_window: str,
) -> dict[str, object]:
    qualities = np.asarray([sample.quality for sample in scored.samples])
    horizontal = scored.horizontal_error_m
    quality_stats: dict[str, object] = {}
    for quality in sorted(set(int(q) for q in qualities)):
        mask = qualities == quality
        values = horizontal[mask]
        quality_stats[str(quality)] = {
            "epochs": int(values.size),
            "median_horizontal_m": round(float(np.median(values)), 6),
            "p95_horizontal_m": round(float(np.percentile(values, 95)), 6),
            "max_horizontal_m": round(float(np.max(values)), 6),
        }
    fixed_mask = qualities == FIX_QUALITY
    fixed_horizontal = horizontal[fixed_mask]
    return {
        "schema": "clas_ppp_accuracy_gif.v1",
        "dataset": {
            "source": "QZSS-Strategy-Office/claslib public data",
            "ref": claslib_ref,
            "window": dataset_window,
            "inputs": [DEFAULT_OBSERVATION, DEFAULT_NAVIGATION, DEFAULT_L6],
            "base_station": None,
        },
        "reference": reference_label,
        "target_horizontal_m": TARGET_HORIZONTAL_M,
        "epochs": len(scored.samples),
        "quality_counts": {
            "fix": int(np.count_nonzero(qualities == FIX_QUALITY)),
            "float": int(np.count_nonzero(qualities == FLOAT_QUALITY)),
        },
        "overall": {
            "median_horizontal_m": round(float(np.median(horizontal)), 6),
            "p95_horizontal_m": round(float(np.percentile(horizontal, 95)), 6),
            "max_horizontal_m": round(float(np.max(horizontal)), 6),
            "median_vertical_m": round(
                float(np.median(np.abs(scored.vertical_error_m))), 6
            ),
        },
        "fix": {
            "median_horizontal_m": round(float(np.median(fixed_horizontal)), 6),
            "p95_horizontal_m": round(float(np.percentile(fixed_horizontal, 95)), 6),
            "max_horizontal_m": round(float(np.max(fixed_horizontal)), 6),
            "below_target_ratio": round(
                float(np.count_nonzero(fixed_horizontal < TARGET_HORIZONTAL_M))
                / fixed_horizontal.size,
                6,
            ),
        },
        "quality_stats": quality_stats,
    }


def render_gif(
    scored: ScoredSolution,
    output: Path,
    *,
    frames: int,
    fps: int,
    dpi: int,
) -> None:
    qualities = np.asarray([sample.quality for sample in scored.samples])
    fixed = qualities == FIX_QUALITY
    floating = qualities == FLOAT_QUALITY
    other = ~(fixed | floating)
    elapsed_min = scored.elapsed_s / 60.0
    horizontal = scored.horizontal_error_m

    total = len(scored.samples)
    frame_counts = sorted(
        set(int(value) for value in np.linspace(1, total, min(frames, total)))
    )

    fig, (ax_map, ax_err) = plt.subplots(1, 2, figsize=(12.4, 5.2))
    fig.suptitle(
        "CLAS PPP (QZSS L6) — no base station — horizontal error stays below 10 cm",
        fontsize=13,
        fontweight="bold",
    )

    limit = max(0.12, float(np.max(np.abs(scored.horizontal_error_m))) * 1.15)
    ax_map.set_xlim(-limit, limit)
    ax_map.set_ylim(-limit, limit)
    ax_map.set_aspect("equal", adjustable="box")
    ax_map.set_xlabel("East error (m)")
    ax_map.set_ylabel("North error (m)")
    ax_map.set_title("Horizontal scatter vs. the published reference")
    ax_map.add_patch(
        Circle(
            (0, 0),
            TARGET_HORIZONTAL_M,
            color=COLOR_TARGET,
            fill=False,
            linestyle="--",
            linewidth=1.4,
            label="10 cm",
        )
    )
    ax_map.add_patch(
        Circle(
            (0, 0),
            0.05,
            color="#6b7280",
            fill=False,
            linestyle=":",
            linewidth=1.1,
            label="5 cm",
        )
    )
    ax_map.scatter([], [], s=10, color=COLOR_FIX, label="FIX", zorder=3)
    ax_map.scatter([], [], s=10, color=COLOR_FLOAT, label="FLOAT", zorder=3)
    if np.any(other):
        ax_map.scatter([], [], s=10, color=COLOR_OTHER, label="Other", zorder=3)
    ax_map.legend(loc="upper right", fontsize=8)

    error_floor = 1e-4
    ax_err.set_yscale("log")
    ax_err.set_ylim(error_floor, 1.0)
    ax_err.set_xlim(0.0, max(elapsed_min[-1], 1.0))
    ax_err.set_xlabel("Elapsed time (min)")
    ax_err.set_ylabel("Horizontal error (m)")
    ax_err.set_title("Horizontal error vs. time")
    ax_err.axhline(
        TARGET_HORIZONTAL_M,
        color=COLOR_TARGET,
        linestyle="--",
        linewidth=1.2,
        label="10 cm target",
    )
    (line_all,) = ax_err.plot(
        [], [], color="#9ca3af", linewidth=0.7, alpha=0.7, label="All solutions"
    )
    (line_median,) = ax_err.plot(
        [], [], color=COLOR_FIX, linewidth=1.8, label="Running median"
    )
    ax_err.legend(loc="upper right", fontsize=8)
    status = ax_err.text(
        0.02,
        0.06,
        "",
        transform=ax_err.transAxes,
        fontsize=8.5,
        color="#374151",
        verticalalignment="bottom",
    )

    def update(count: int):
        ax_map.collections[0].set_offsets(
            np.column_stack(
                (
                    scored.east_m[:count][fixed[:count]],
                    scored.north_m[:count][fixed[:count]],
                )
            )
        )
        ax_map.collections[1].set_offsets(
            np.column_stack(
                (
                    scored.east_m[:count][floating[:count]],
                    scored.north_m[:count][floating[:count]],
                )
            )
        )
        if np.any(other[:count]):
            if len(ax_map.collections) > 2:
                ax_map.collections[2].set_offsets(
                    np.column_stack(
                        (
                            scored.east_m[:count][other[:count]],
                            scored.north_m[:count][other[:count]],
                        )
                    )
                )
        line_all.set_data(
            elapsed_min[:count], np.maximum(horizontal[:count], error_floor)
        )
        running = np.median(horizontal[:count])
        line_median.set_data([0.0, elapsed_min[count - 1]], [running, running])
        fix_ratio = float(np.count_nonzero(fixed[:count])) / count
        status.set_text(
            f"epochs: {count:,}/{total:,}\n"
            f"FIX rate: {100.0 * fix_ratio:5.1f}%\n"
            f"running median: {1000.0 * running:5.2f} mm"
        )
        return (*ax_map.collections, line_all, line_median, status)

    animation = FuncAnimation(
        fig,
        update,
        frames=frame_counts,
        interval=1000 // max(fps, 1),
        blit=False,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    animation.save(str(output), writer=PillowWriter(fps=fps), dpi=dpi)
    plt.close(fig)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    root = repo_root_from_script()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--claslib-root",
        type=Path,
        default=None,
        help="Existing CLASLIB checkout; skips fetching when present.",
    )
    parser.add_argument("--claslib-repo", default=DEFAULT_CLASLIB_REPO)
    parser.add_argument("--claslib-ref", default=DEFAULT_CLASLIB_REF)
    parser.add_argument("--work-dir", type=Path, default=None)
    parser.add_argument(
        "--output-gif",
        type=Path,
        default=root / "docs" / "clas_ppp_no_base_accuracy.gif",
    )
    parser.add_argument(
        "--output-metrics",
        type=Path,
        default=root / "docs" / "clas_ppp_no_base_accuracy.json",
    )
    parser.add_argument(
        "--nmea",
        type=Path,
        default=None,
        help="Reuse an existing NMEA GGA file instead of running the solver.",
    )
    parser.add_argument("--max-epochs", type=int, default=DEFAULT_MAX_EPOCHS)
    parser.add_argument("--start-date", default=DEFAULT_START_DATE)
    parser.add_argument("--start-time", default=DEFAULT_START_TIME)
    parser.add_argument("--gps-week", type=int, default=DEFAULT_GPS_WEEK)
    parser.add_argument("--frames", type=int, default=180)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--dpi", type=int, default=110)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    root = repo_root_from_script()
    work_dir = (args.work_dir or (root / "output" / "clas_ppp_accuracy")).resolve()
    work_dir.mkdir(parents=True, exist_ok=True)
    log_path = work_dir / "run.log"
    log_path.write_text("", encoding="utf-8")

    if args.nmea is not None:
        nmea = args.nmea
        if not nmea.is_file():
            raise SystemExit(f"missing NMEA file: {nmea}")
    else:
        claslib_root = args.claslib_root
        if claslib_root is None:
            claslib_root = work_dir / "claslib"
            fetch_claslib(claslib_root, args.claslib_repo, args.claslib_ref, log_path)
        else:
            claslib_root = claslib_root.resolve()
        binary = resolve_rnx2rtkp(claslib_root, log_path)
        config = write_config(claslib_root, work_dir)
        nmea = work_dir / "claslib.nmea"
        run_solver(
            binary,
            claslib_root,
            config,
            nmea,
            start_date=args.start_date,
            start_time=args.start_time,
            gps_week=args.gps_week,
            max_epochs=args.max_epochs,
            log_path=log_path,
        )

    samples = parse_gga(nmea)
    if not samples:
        raise SystemExit(f"no valid GGA epochs in {nmea}")

    reference_ecef = np.asarray(DEFAULT_REFERENCE_ECEF)
    scored = score(samples, reference_ecef)
    window = f"{args.start_date} {args.start_time} GPST, {len(samples)} epochs"
    payload = metrics_payload(
        scored,
        reference_label=DEFAULT_REFERENCE_LABEL,
        claslib_ref=args.claslib_ref,
        dataset_window=window,
    )
    args.output_metrics.parent.mkdir(parents=True, exist_ok=True)
    args.output_metrics.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    render_gif(
        scored,
        args.output_gif,
        frames=args.frames,
        fps=args.fps,
        dpi=args.dpi,
    )
    print(f"GIF: {args.output_gif}")
    print(f"metrics: {args.output_metrics}")
    print(
        "median horizontal error: "
        f"{1000.0 * payload['overall']['median_horizontal_m']:.2f} mm "
        f"(FIX {100.0 * payload['fix']['below_target_ratio']:.1f}% of "
        f"{payload['quality_counts']['fix']} epochs below 10 cm)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
