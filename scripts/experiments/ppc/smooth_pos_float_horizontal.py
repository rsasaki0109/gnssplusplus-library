#!/usr/bin/env python3
"""Locally smooth FLOAT horizontal positions without changing FIX or Up.

The smoother is reference-free.  It fits a local polynomial to the input
trajectory in a fixed ENU frame, applies only the horizontal correction, and
rejects corrections beyond a caller-provided displacement gate.  Status and
all solution telemetry are preserved.
"""

from __future__ import annotations

from _paths import ANALYSIS_DIR, APPS_DIR, COMMANDS_DIR, PPC_DIR, ROOT_DIR, SCRIPTS_DIR

import argparse
import dataclasses
import json
import math
from pathlib import Path
import sys

import numpy as np


for command_path in (
    SCRIPTS_DIR,
    COMMANDS_DIR,
    *(COMMANDS_DIR / group for group in ("benchmarks", "positioning", "products", "receivers")),
):
    if str(command_path) not in sys.path:
        sys.path.insert(0, str(command_path))

import apply_ppc_dual_profile_selector as pos_writer  # noqa: E402
import generate_driving_comparison as comparison  # noqa: E402
import gnss_ppc_metrics as ppc_metrics  # noqa: E402


def ecef_to_enu_rotation(epoch: comparison.SolutionEpoch) -> np.ndarray:
    lat = math.radians(epoch.lat_deg)
    lon = math.radians(epoch.lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    return np.asarray(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ]
    )


def local_polynomial_values(
    times_s: np.ndarray,
    values: np.ndarray,
    window_epochs: int,
    degree: int,
) -> np.ndarray:
    """Evaluate a sliding least-squares polynomial at every input epoch."""
    count = len(times_s)
    if count < window_epochs:
        raise ValueError("trajectory is shorter than window_epochs")
    half = window_epochs // 2
    smoothed = np.empty(count, dtype=float)
    for index in range(count):
        begin = max(0, min(index - half, count - window_epochs))
        end = begin + window_epochs
        relative_times = times_s[begin:end] - times_s[index]
        design = np.vander(relative_times, N=degree + 1, increasing=True)
        coefficients, *_ = np.linalg.lstsq(design, values[begin:end], rcond=None)
        smoothed[index] = coefficients[0]
    return smoothed


def smooth_float_horizontal(
    epochs: list[comparison.SolutionEpoch],
    window_epochs: int,
    degree: int,
    max_displacement_m: float,
) -> tuple[list[comparison.SolutionEpoch], dict[str, object]]:
    if not epochs:
        raise ValueError("input trajectory is empty")
    if window_epochs < 3 or window_epochs % 2 == 0:
        raise ValueError("window_epochs must be an odd integer >= 3")
    if degree < 1 or degree >= window_epochs:
        raise ValueError("degree must be >= 1 and less than window_epochs")
    if max_displacement_m <= 0.0:
        raise ValueError("max_displacement_m must be positive")

    ordered = sorted(epochs, key=lambda epoch: (epoch.week, epoch.tow))
    origin = ordered[0]
    rotation = ecef_to_enu_rotation(origin)
    origin_ecef = np.asarray(origin.ecef, dtype=float)
    positions_ecef = np.asarray([epoch.ecef for epoch in ordered], dtype=float)
    positions_enu = (rotation @ (positions_ecef - origin_ecef).T).T
    times_s = np.asarray(
        [(epoch.week - origin.week) * 604800.0 + epoch.tow - origin.tow for epoch in ordered],
        dtype=float,
    )
    smooth_east = local_polynomial_values(
        times_s, positions_enu[:, 0], window_epochs, degree
    )
    smooth_north = local_polynomial_values(
        times_s, positions_enu[:, 1], window_epochs, degree
    )

    output: list[comparison.SolutionEpoch] = []
    replaced = 0
    rejected = 0
    for index, epoch in enumerate(ordered):
        if epoch.status == 4:
            output.append(epoch)
            continue
        correction_enu = np.asarray(
            [smooth_east[index] - positions_enu[index, 0],
             smooth_north[index] - positions_enu[index, 1],
             0.0]
        )
        displacement_m = float(np.linalg.norm(correction_enu))
        if displacement_m > max_displacement_m:
            output.append(epoch)
            rejected += 1
            continue
        candidate_ecef = np.asarray(epoch.ecef) + rotation.transpose() @ correction_enu
        lat_deg, lon_deg, height_m = ppc_metrics.llh_from_ecef(*candidate_ecef)
        output.append(
            dataclasses.replace(
                epoch,
                ecef=candidate_ecef,
                lat_deg=lat_deg,
                lon_deg=lon_deg,
                height_m=height_m,
            )
        )
        replaced += 1

    summary: dict[str, object] = {
        "reference_truth_used": False,
        "input_epochs": len(ordered),
        "output_epochs": len(output),
        "preserved_fixed_epochs": sum(epoch.status == 4 for epoch in ordered),
        "replaced_float_epochs": replaced,
        "displacement_rejected_epochs": rejected,
        "window_epochs": window_epochs,
        "degree": degree,
        "max_displacement_m": max_displacement_m,
        "preserved_status": True,
        "preserved_up": True,
    }
    return output, summary


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-pos", type=Path, required=True)
    parser.add_argument("--output-pos", type=Path, required=True)
    parser.add_argument("--summary-json", type=Path)
    parser.add_argument("--window-epochs", type=int, default=31)
    parser.add_argument("--degree", type=int, default=2)
    parser.add_argument("--max-displacement-m", type=float, default=0.5)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    epochs = comparison.read_libgnss_pos(args.baseline_pos)
    smoothed, summary = smooth_float_horizontal(
        epochs,
        args.window_epochs,
        args.degree,
        args.max_displacement_m,
    )
    pos_writer.write_pos(args.output_pos, smoothed)
    payload = json.dumps(summary, indent=2, sort_keys=True) + "\n"
    if args.summary_json is None:
        print(payload, end="")
    else:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(payload, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
