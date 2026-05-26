#!/usr/bin/env python3
"""Run and validate a mixed-GNSS short-baseline static sign-off."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import sys

from gnss_runtime import ensure_input_exists, resolve_gnss_command
import gnss_dd_residuals


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
        "--diagnostics-csv",
        type=Path,
        default=None,
        help="Optional per-epoch RTK candidate diagnostics CSV written by gnss solve.",
    )
    parser.add_argument(
        "--debug-epoch-log",
        type=Path,
        default=None,
        help="Optional per-epoch AR/slip/debug telemetry CSV written by gnss solve.",
    )
    parser.add_argument(
        "--dd-residuals-csv",
        type=Path,
        default=None,
        help="Optional per-DD RTK prefit residual diagnostics CSV written by gnss solve.",
    )
    parser.add_argument(
        "--dd-top-pairs-csv",
        type=Path,
        default=None,
        help="Optional worst DD residual pair summary CSV.",
    )
    parser.add_argument(
        "--dd-html-report",
        type=Path,
        default=None,
        help="Optional DD residual HTML report path.",
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
    parser.add_argument(
        "--require-mean-baseline-error-max",
        type=float,
        default=None,
        help="Fail if mean absolute solution-baseline length error exceeds this value in meters.",
    )
    parser.add_argument(
        "--require-mean-rtk-phase-obs-min",
        type=float,
        default=None,
        help="Fail if mean RTK carrier-phase update observation count is below this value.",
    )
    parser.add_argument(
        "--require-mean-rtk-postfit-rms-max",
        type=float,
        default=None,
        help="Fail if mean RTK post-suppression residual RMS exceeds this value in meters.",
    )
    parser.add_argument(
        "--require-mean-postfix-phase-rms-max",
        type=float,
        default=None,
        help="Fail if mean post-fix carrier-phase residual RMS exceeds this value in meters.",
    )
    parser.add_argument(
        "--require-dd-phase-p95-max",
        type=float,
        default=None,
        help="Fail if DD phase p95 absolute residual exceeds this value in meters.",
    )
    parser.add_argument(
        "--require-dd-code-p95-max",
        type=float,
        default=None,
        help="Fail if DD code p95 absolute residual exceeds this value in meters.",
    )
    parser.add_argument(
        "--require-dd-max-abs-residual-max",
        type=float,
        default=None,
        help="Fail if DD max absolute residual exceeds this value in meters.",
    )
    parser.add_argument(
        "--require-dd-suppressed-rows-max",
        type=int,
        default=None,
        help="Fail if DD outlier-threshold suppressed row count exceeds this value.",
    )
    return parser.parse_args()


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


def maybe_float(parts: list[str], index: int) -> float | None:
    if index >= len(parts):
        return None
    value = float(parts[index])
    return value if math.isfinite(value) else None


def maybe_int(parts: list[str], index: int) -> int | None:
    value = maybe_float(parts, index)
    return None if value is None else int(value)


def read_pos_records(path: Path) -> list[dict[str, float | int | None]]:
    records: list[dict[str, float | int | None]] = []
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
                    "pdop": maybe_float(parts, 10),
                    "ratio": maybe_float(parts, 11),
                    "baseline_length_m": maybe_float(parts, 12),
                    "rtk_iterations": maybe_int(parts, 13),
                    "rtk_update_observations": maybe_int(parts, 14),
                    "rtk_update_phase_observations": maybe_int(parts, 15),
                    "rtk_update_code_observations": maybe_int(parts, 16),
                    "rtk_update_suppressed_outliers": maybe_int(parts, 17),
                    "rtk_update_prefit_residual_rms_m": maybe_float(parts, 18),
                    "rtk_update_prefit_residual_max_m": maybe_float(parts, 19),
                    "rtk_update_post_suppression_residual_rms_m": maybe_float(parts, 20),
                    "rtk_update_post_suppression_residual_max_m": maybe_float(parts, 21),
                    "rtk_update_normalized_innovation_squared": maybe_float(parts, 22),
                    "rtk_update_normalized_innovation_squared_per_observation": maybe_float(parts, 23),
                    "rtk_update_rejected_by_innovation_gate": maybe_int(parts, 24),
                }
            )
    return records


def rounded(value: float) -> float:
    return round(value, 6)


def optional_rounded(value: float | None) -> float | None:
    return None if value is None else rounded(value)


def finite_record_values(records: list[dict[str, float | int | None]], key: str) -> list[float]:
    values: list[float] = []
    for record in records:
        raw_value = record.get(key)
        if raw_value is None:
            continue
        value = float(raw_value)
        if math.isfinite(value):
            values.append(value)
    return values


def mean_or_none(values: list[float]) -> float | None:
    if not values:
        return None
    return sum(values) / len(values)


def read_debug_postfix_phase_residuals(path: Path | None) -> list[float]:
    if path is None or not path.exists():
        return []
    values: list[float] = []
    with path.open(encoding="ascii", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            raw_value = row.get("postfix_residual_rms")
            if raw_value is None or raw_value == "":
                continue
            value = float(raw_value)
            if math.isfinite(value):
                values.append(value)
    return values


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
    header_baseline_m = math.dist(rover_position, base_position)
    solution_baselines = []
    for record in records:
        baseline = record.get("baseline_length_m")
        if baseline is None or not math.isfinite(float(baseline)) or float(baseline) <= 0.0:
            baseline = math.dist(
                (float(record["x"]), float(record["y"]), float(record["z"])),
                base_position,
            )
        solution_baselines.append(float(baseline))
    baseline_length_errors = [abs(value - header_baseline_m) for value in solution_baselines]
    phase_obs = finite_record_values(records, "rtk_update_phase_observations")
    code_obs = finite_record_values(records, "rtk_update_code_observations")
    prefit_rms = finite_record_values(records, "rtk_update_prefit_residual_rms_m")
    postfit_rms = finite_record_values(records, "rtk_update_post_suppression_residual_rms_m")
    postfit_abs_max = finite_record_values(records, "rtk_update_post_suppression_residual_max_m")
    nis_per_obs = finite_record_values(
        records, "rtk_update_normalized_innovation_squared_per_observation"
    )
    postfix_phase_rms = read_debug_postfix_phase_residuals(args.debug_epoch_log)
    dd_residual_summary = None
    if args.dd_residuals_csv is not None and args.dd_residuals_csv.exists():
        dd_residual_records = gnss_dd_residuals.read_records(args.dd_residuals_csv)
        if dd_residual_records:
            dd_residual_summary = gnss_dd_residuals.summarize_records(
                dd_residual_records,
                top_n=5,
            )
            if args.dd_top_pairs_csv is not None:
                gnss_dd_residuals.write_top_pairs_csv(
                    args.dd_top_pairs_csv,
                    list(dd_residual_summary["top_pairs"]),
                )
            if args.dd_html_report is not None:
                gnss_dd_residuals.write_html_report(
                    args.dd_html_report,
                    dd_residual_summary,
                    title="Short-Baseline DD Residual Report",
                )
    fixed_count = sum(1 for record in records if int(record["status"]) == 4)
    mean_satellites = sum(int(record["satellites"]) for record in records) / len(records)
    innovation_rejections = sum(
        1
        for record in records
        if int(record.get("rtk_update_rejected_by_innovation_gate") or 0) != 0
    )

    payload = {
        "dataset": "Tsukuba short_baseline",
        "rover": str(args.rover),
        "base": str(args.base),
        "nav": str(args.nav),
        "solution_pos": str(args.out),
        "diagnostics_csv": str(args.diagnostics_csv) if args.diagnostics_csv else None,
        "debug_epoch_log": str(args.debug_epoch_log) if args.debug_epoch_log else None,
        "dd_residuals_csv": str(args.dd_residuals_csv) if args.dd_residuals_csv else None,
        "dd_top_pairs_csv": str(args.dd_top_pairs_csv) if args.dd_top_pairs_csv else None,
        "dd_html_report": str(args.dd_html_report) if args.dd_html_report else None,
        "epochs": len(records),
        "fixed_epochs": fixed_count,
        "fix_rate_pct": rounded(100.0 * fixed_count / len(records)),
        "mean_position_error_m": rounded(sum(errors) / len(errors)),
        "max_position_error_m": rounded(max(errors)),
        "mean_satellites": rounded(mean_satellites),
        "rover_header_position_ecef_m": [rounded(value) for value in rover_position],
        "base_header_position_ecef_m": [rounded(value) for value in base_position],
        "header_baseline_m": rounded(header_baseline_m),
        "mean_solution_baseline_m": rounded(sum(solution_baselines) / len(solution_baselines)),
        "mean_baseline_length_error_m": rounded(
            sum(baseline_length_errors) / len(baseline_length_errors)
        ),
        "max_baseline_length_error_m": rounded(max(baseline_length_errors)),
        "mean_rtk_update_phase_observations": optional_rounded(mean_or_none(phase_obs)),
        "mean_rtk_update_code_observations": optional_rounded(mean_or_none(code_obs)),
        "mean_rtk_update_prefit_residual_rms_m": optional_rounded(mean_or_none(prefit_rms)),
        "mean_rtk_update_post_suppression_residual_rms_m": optional_rounded(
            mean_or_none(postfit_rms)
        ),
        "max_rtk_update_post_suppression_residual_abs_m": optional_rounded(
            max(postfit_abs_max) if postfit_abs_max else None
        ),
        "mean_rtk_update_nis_per_observation": optional_rounded(mean_or_none(nis_per_obs)),
        "rtk_update_innovation_gate_rejected_epochs": innovation_rejections,
        "mean_postfix_phase_residual_rms_m": optional_rounded(mean_or_none(postfix_phase_rms)),
        "max_postfix_phase_residual_rms_m": optional_rounded(
            max(postfix_phase_rms) if postfix_phase_rms else None
        ),
        "dd_residual_summary": dd_residual_summary,
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
    if (
        getattr(args, "require_mean_baseline_error_max", None) is not None
        and float(payload["mean_baseline_length_error_m"]) > args.require_mean_baseline_error_max
    ):
        failures.append(
            f"mean baseline length error {float(payload['mean_baseline_length_error_m']):.6f} m > "
            f"{args.require_mean_baseline_error_max:.6f} m"
        )
    mean_phase_obs = payload.get("mean_rtk_update_phase_observations")
    if getattr(args, "require_mean_rtk_phase_obs_min", None) is not None:
        if mean_phase_obs is None or float(mean_phase_obs) < args.require_mean_rtk_phase_obs_min:
            observed = "n/a" if mean_phase_obs is None else f"{float(mean_phase_obs):.6f}"
            failures.append(
                f"mean RTK phase observations {observed} < "
                f"{args.require_mean_rtk_phase_obs_min:.6f}"
            )
    mean_postfit_rms = payload.get("mean_rtk_update_post_suppression_residual_rms_m")
    if getattr(args, "require_mean_rtk_postfit_rms_max", None) is not None:
        if mean_postfit_rms is None or float(mean_postfit_rms) > args.require_mean_rtk_postfit_rms_max:
            observed = "n/a" if mean_postfit_rms is None else f"{float(mean_postfit_rms):.6f}"
            failures.append(
                f"mean RTK postfit residual RMS {observed} m > "
                f"{args.require_mean_rtk_postfit_rms_max:.6f} m"
            )
    mean_postfix_phase_rms = payload.get("mean_postfix_phase_residual_rms_m")
    if getattr(args, "require_mean_postfix_phase_rms_max", None) is not None:
        if (
            mean_postfix_phase_rms is None
            or float(mean_postfix_phase_rms) > args.require_mean_postfix_phase_rms_max
        ):
            observed = (
                "n/a"
                if mean_postfix_phase_rms is None
                else f"{float(mean_postfix_phase_rms):.6f}"
            )
            failures.append(
                f"mean post-fix carrier-phase residual RMS {observed} m > "
                f"{args.require_mean_postfix_phase_rms_max:.6f} m"
            )
    dd_requirement_args = argparse.Namespace(
        require_phase_p95_max=getattr(args, "require_dd_phase_p95_max", None),
        require_code_p95_max=getattr(args, "require_dd_code_p95_max", None),
        require_max_abs_residual_max=getattr(args, "require_dd_max_abs_residual_max", None),
        require_suppressed_rows_max=getattr(args, "require_dd_suppressed_rows_max", None),
    )
    has_dd_requirements = any(
        value is not None for value in vars(dd_requirement_args).values()
    )
    if has_dd_requirements:
        dd_summary = payload.get("dd_residual_summary")
        if not isinstance(dd_summary, dict):
            failures.append("DD residual summary is unavailable")
        else:
            try:
                gnss_dd_residuals.enforce_requirements(dd_summary, dd_requirement_args)
            except SystemExit as exc:
                message = str(exc)
                for line in message.splitlines():
                    stripped = line.strip()
                    if stripped.startswith("- "):
                        failures.append(f"DD {stripped[2:]}")

    if failures:
        message = "Short-baseline sign-off checks failed:\n" + "\n".join(
            f"  - {failure}" for failure in failures
        )
        raise SystemExit(message)


def main() -> int:
    args = parse_args()
    gnss_command = resolve_gnss_command(ROOT_DIR)

    ensure_input_exists(args.rover, "rover observation file", ROOT_DIR)
    ensure_input_exists(args.base, "base observation file", ROOT_DIR)
    ensure_input_exists(args.nav, "navigation file", ROOT_DIR)
    if args.max_epochs == 0:
        raise SystemExit("--max-epochs must be positive or -1")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    if args.diagnostics_csv is not None:
        args.diagnostics_csv.parent.mkdir(parents=True, exist_ok=True)
    if args.debug_epoch_log is not None:
        args.debug_epoch_log.parent.mkdir(parents=True, exist_ok=True)
    if args.dd_residuals_csv is not None:
        args.dd_residuals_csv.parent.mkdir(parents=True, exist_ok=True)
    if args.dd_top_pairs_csv is not None:
        args.dd_top_pairs_csv.parent.mkdir(parents=True, exist_ok=True)
    if args.dd_html_report is not None:
        args.dd_html_report.parent.mkdir(parents=True, exist_ok=True)

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
    if args.diagnostics_csv is not None:
        command.extend(["--diagnostics-csv", str(args.diagnostics_csv)])
    if args.debug_epoch_log is not None:
        command.extend(["--debug-epoch-log", str(args.debug_epoch_log)])
    if args.dd_residuals_csv is not None:
        command.extend(["--dd-residuals-csv", str(args.dd_residuals_csv)])
    run_command(command)

    payload = build_summary_payload(args)
    enforce_summary_requirements(payload, args)

    print("Finished short-baseline sign-off.")
    print(f"  solution: {args.out}")
    print(f"  summary: {args.summary_json}")
    if args.diagnostics_csv is not None:
        print(f"  diagnostics_csv: {args.diagnostics_csv}")
    if args.debug_epoch_log is not None:
        print(f"  debug_epoch_log: {args.debug_epoch_log}")
    if args.dd_residuals_csv is not None:
        print(f"  dd_residuals_csv: {args.dd_residuals_csv}")
    if args.dd_top_pairs_csv is not None:
        print(f"  dd_top_pairs_csv: {args.dd_top_pairs_csv}")
    if args.dd_html_report is not None:
        print(f"  dd_html_report: {args.dd_html_report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
