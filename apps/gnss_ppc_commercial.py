#!/usr/bin/env python3
"""Commercial receiver comparison helpers for PPC-style sign-offs."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys
import time
from typing import Callable


ROOT_DIR = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = ROOT_DIR / "scripts"

if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import generate_driving_comparison as comparison  # noqa: E402
import gnss_moving_base_signoff as moving_base_signoff  # noqa: E402
import gnss_ppc_metrics as ppc_metrics  # noqa: E402


RunCommand = Callable[[list[str]], None]


@dataclass(frozen=True)
class CommercialRoverTuning:
    preset: str | None = None
    arfilter: bool | None = None
    arfilter_margin: float | None = None
    min_hold_count: int | None = None
    hold_ratio_threshold: float | None = None


@dataclass(frozen=True)
class CommercialRoverSolve:
    rover: Path
    base: Path
    nav: Path
    out: Path
    max_epochs: int
    tuning: CommercialRoverTuning


def default_commercial_out(summary_json: Path) -> Path:
    stem = summary_json.stem
    if stem.endswith("_summary"):
        stem = stem[: -len("_summary")]
    return summary_json.with_name(f"{stem}_commercial_receiver.pos")


def read_commercial_solution_epochs(
    path: Path,
    requested_format: str,
) -> tuple[list[comparison.SolutionEpoch], str]:
    records, resolved_format = moving_base_signoff.read_commercial_solution_records(
        path,
        requested_format,
    )
    epochs: list[comparison.SolutionEpoch] = []
    for record in records:
        x = float(record["x"])
        y = float(record["y"])
        z = float(record["z"])
        lat_deg, lon_deg, height_m = ppc_metrics.llh_from_ecef(x, y, z)
        ecef = comparison.llh_to_ecef(lat_deg, lon_deg, height_m)
        epochs.append(
            comparison.SolutionEpoch(
                week=int(record["week"]),
                tow=float(record["tow"]),
                lat_deg=lat_deg,
                lon_deg=lon_deg,
                height_m=height_m,
                ecef=ecef,
                status=int(record["status"]),
                num_satellites=int(record["satellites"]),
            )
        )
    return epochs, resolved_format


def summarize_receiver_epochs(
    *,
    reference: list[comparison.ReferenceEpoch],
    epochs: list[comparison.SolutionEpoch],
    label: str,
    source: str,
    solution_pos: Path,
    solution_format: str,
    matched_csv: Path | None,
    match_tolerance_s: float,
    solver_wall_time_s: float | None,
    generated_solution: bool | None = None,
    rover: Path | None = None,
    base: Path | None = None,
    nav: Path | None = None,
) -> dict[str, object]:
    metrics = ppc_metrics.summarize_solution_epochs(
        reference,
        epochs,
        4,
        label,
        match_tolerance_s,
        solver_wall_time_s,
    )
    if matched_csv is not None:
        matches = comparison.match_to_reference(epochs, reference, match_tolerance_s)
        ppc_metrics.write_reference_matches_csv(matched_csv, matches)
    metrics.update(
        {
            "label": label,
            "source": source,
            "solution_pos": str(solution_pos),
            "format": solution_format,
            "matched_csv": str(matched_csv) if matched_csv is not None else None,
        }
    )
    if generated_solution is not None:
        metrics["generated_solution"] = generated_solution
    if rover is not None:
        metrics["rover"] = str(rover)
    if base is not None:
        metrics["base"] = str(base)
    if nav is not None:
        metrics["nav"] = str(nav)
    return metrics


def summarize_existing_receiver_solution(
    *,
    reference: list[comparison.ReferenceEpoch],
    solution_pos: Path,
    requested_format: str,
    label: str,
    matched_csv: Path | None,
    match_tolerance_s: float,
    solver_wall_time_s: float | None,
) -> dict[str, object]:
    epochs, resolved_format = read_commercial_solution_epochs(solution_pos, requested_format)
    return summarize_receiver_epochs(
        reference=reference,
        epochs=epochs,
        label=label,
        source="provided_solution",
        solution_pos=solution_pos,
        solution_format=resolved_format,
        matched_csv=matched_csv,
        match_tolerance_s=match_tolerance_s,
        solver_wall_time_s=solver_wall_time_s,
    )


def summarize_solved_receiver_observations(
    *,
    reference: list[comparison.ReferenceEpoch],
    solution_pos: Path,
    label: str,
    matched_csv: Path | None,
    match_tolerance_s: float,
    solver_wall_time_s: float | None,
    generated_solution: bool,
    rover: Path,
    base: Path,
    nav: Path,
) -> dict[str, object]:
    epochs = comparison.read_libgnss_pos(solution_pos)
    return summarize_receiver_epochs(
        reference=reference,
        epochs=epochs,
        label=label,
        source="libgnss_solved_receiver_observations",
        solution_pos=solution_pos,
        solution_format="pos",
        matched_csv=matched_csv,
        match_tolerance_s=match_tolerance_s,
        solver_wall_time_s=solver_wall_time_s,
        generated_solution=generated_solution,
        rover=rover,
        base=base,
        nav=nav,
    )


def build_commercial_rover_command(
    gnss_command: list[str],
    solve: CommercialRoverSolve,
) -> list[str]:
    command = [
        *gnss_command,
        "solve",
        "--rover",
        str(solve.rover),
        "--base",
        str(solve.base),
        "--nav",
        str(solve.nav),
        "--out",
        str(solve.out),
        "--mode",
        "kinematic",
        "--no-kml",
    ]
    tuning = solve.tuning
    if tuning.preset is not None:
        command.extend(["--preset", tuning.preset])
    if tuning.arfilter is True:
        command.append("--arfilter")
    elif tuning.arfilter is False:
        command.append("--no-arfilter")
    if tuning.arfilter_margin is not None:
        command.extend(["--arfilter-margin", str(tuning.arfilter_margin)])
    if tuning.min_hold_count is not None:
        command.extend(["--min-hold-count", str(tuning.min_hold_count)])
    if tuning.hold_ratio_threshold is not None:
        command.extend(["--hold-ratio-threshold", str(tuning.hold_ratio_threshold)])
    if solve.max_epochs > 0:
        command.extend(["--max-epochs", str(solve.max_epochs)])
    return command


def run_commercial_rover_solver(
    gnss_command: list[str],
    solve: CommercialRoverSolve,
    run_command: RunCommand,
) -> float:
    command = build_commercial_rover_command(gnss_command, solve)
    start = time.perf_counter()
    run_command(command)
    return time.perf_counter() - start
