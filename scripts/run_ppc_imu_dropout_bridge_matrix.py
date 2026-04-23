#!/usr/bin/env python3
"""Apply a causal IMU-aided dropout bridge to PPC runs and score it."""

from __future__ import annotations

import argparse
import bisect
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = Path(__file__).resolve().parent
APPS_DIR = ROOT_DIR / "apps"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
if str(APPS_DIR) not in sys.path:
    sys.path.insert(0, str(APPS_DIR))

import apply_ppc_dual_profile_selector as pos_writer  # noqa: E402
import generate_driving_comparison as comparison  # noqa: E402
import gnss_ppc_metrics as ppc_metrics  # noqa: E402
import run_ppc_cv_dropout_bridge_matrix as cv_bridge  # noqa: E402


TIME_CANDIDATES = ("gpstows", "gpstow", "tow", "tows", "time", "times")
WEEK_CANDIDATES = ("gpsweek", "week")
ACCEL_CANDIDATES = {
    "x": ("accxms2", "accx", "accelx", "ax"),
    "y": ("accyms2", "accy", "accely", "ay"),
    "z": ("acczms2", "accz", "accelz", "az"),
}


@dataclass(frozen=True)
class ImuSample:
    week: int
    tow: float
    acc_x_mps2: float
    acc_y_mps2: float
    acc_z_mps2: float


@dataclass(frozen=True)
class IMUBridgeConfig:
    max_gap_s: float
    max_anchor_age_s: float
    max_velocity_baseline_s: float
    bridge_status: int
    bridge_num_satellites: int
    match_tolerance_s: float
    threshold_m: float
    bias_window_s: float
    min_heading_speed_mps: float
    max_horizontal_accel_mps2: float
    forward_axis: str
    lateral_axis: str
    forward_sign: float
    lateral_sign: float
    anchor_mode: str = "scored"
    anchor_statuses: tuple[int, ...] = ()
    anchor_min_ratio: float | None = None
    anchor_max_prefit_rms_m: float | None = None
    anchor_max_post_rms_m: float | None = None
    anchor_max_suppressed_outliers: int | None = None


def normalize_header(name: str) -> str:
    return "".join(ch for ch in name.strip().lower() if ch.isalnum())


def field_lookup(fieldnames: list[str] | None) -> dict[str, str]:
    return {normalize_header(name): name for name in fieldnames or []}


def find_column(lookup: dict[str, str], candidates: tuple[str, ...], context: str) -> str:
    for candidate in candidates:
        if candidate in lookup:
            return lookup[candidate]
    raise SystemExit(f"{context}: missing any of {', '.join(candidates)}")


def read_imu_csv(path: Path) -> list[ImuSample]:
    if not path.exists():
        raise SystemExit(f"missing IMU CSV: {path}")
    samples: list[ImuSample] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        lookup = field_lookup(reader.fieldnames)
        time_column = find_column(lookup, TIME_CANDIDATES, str(path))
        week_column = find_column(lookup, WEEK_CANDIDATES, str(path))
        accel_columns = {
            axis: find_column(lookup, candidates, f"{path}: acc {axis}")
            for axis, candidates in ACCEL_CANDIDATES.items()
        }
        for row in reader:
            samples.append(
                ImuSample(
                    week=int(float(row[week_column])),
                    tow=float(row[time_column]),
                    acc_x_mps2=float(row[accel_columns["x"]]),
                    acc_y_mps2=float(row[accel_columns["y"]]),
                    acc_z_mps2=float(row[accel_columns["z"]]),
                )
            )
    samples.sort(key=lambda sample: cv_bridge.gps_seconds(sample.week, sample.tow))
    return samples


def sample_seconds(sample: ImuSample) -> float:
    return cv_bridge.gps_seconds(sample.week, sample.tow)


def samples_between(
    samples: list[ImuSample],
    seconds: list[float],
    start_s: float,
    end_s: float,
) -> list[ImuSample]:
    start_index = bisect.bisect_left(seconds, start_s)
    end_index = bisect.bisect_right(seconds, end_s)
    return samples[start_index:end_index]


def accel_component(sample: ImuSample, axis: str) -> float:
    if axis == "x":
        return sample.acc_x_mps2
    if axis == "y":
        return sample.acc_y_mps2
    if axis == "z":
        return sample.acc_z_mps2
    raise ValueError(f"unsupported axis: {axis}")


def enu_to_ecef_delta(enu: np.ndarray, ref_lat_deg: float, ref_lon_deg: float) -> np.ndarray:
    lat = math.radians(ref_lat_deg)
    lon = math.radians(ref_lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    rot = np.array(
        [
            [-sin_lon, -sin_lat * cos_lon, cos_lat * cos_lon],
            [cos_lon, -sin_lat * sin_lon, cos_lat * sin_lon],
            [0.0, cos_lat, sin_lat],
        ]
    )
    return rot @ enu


def horizontal_accel_enu(
    sample: ImuSample,
    forward_unit: np.ndarray,
    lateral_unit: np.ndarray,
    forward_bias_mps2: float,
    lateral_bias_mps2: float,
    config: IMUBridgeConfig,
) -> np.ndarray:
    forward_accel = config.forward_sign * (
        accel_component(sample, config.forward_axis) - forward_bias_mps2
    )
    lateral_accel = config.lateral_sign * (
        accel_component(sample, config.lateral_axis) - lateral_bias_mps2
    )
    horizontal = forward_unit * forward_accel + lateral_unit * lateral_accel
    norm = float(np.linalg.norm(horizontal))
    if norm > config.max_horizontal_accel_mps2 > 0.0:
        horizontal *= config.max_horizontal_accel_mps2 / norm
    return np.array([horizontal[0], horizontal[1], 0.0])


def propagated_imu_epochs(
    reference: list[comparison.ReferenceEpoch],
    span: cv_bridge.DropoutSpan,
    previous_anchor: comparison.SolutionEpoch,
    last_anchor: comparison.SolutionEpoch,
    samples: list[ImuSample],
    sample_times_s: list[float],
    config: IMUBridgeConfig,
) -> list[comparison.SolutionEpoch] | None:
    previous_s = cv_bridge.gps_seconds(previous_anchor.week, previous_anchor.tow)
    last_s = cv_bridge.gps_seconds(last_anchor.week, last_anchor.tow)
    if last_s <= previous_s:
        return None

    origin_lat = last_anchor.lat_deg
    origin_lon = last_anchor.lon_deg
    origin_ecef = last_anchor.ecef
    previous_enu = comparison.ecef_to_enu(previous_anchor.ecef - origin_ecef, origin_lat, origin_lon)
    velocity_enu = -previous_enu / (last_s - previous_s)
    horizontal_speed = float(np.linalg.norm(velocity_enu[:2]))
    if horizontal_speed < config.min_heading_speed_mps:
        return None

    forward_unit = velocity_enu[:2] / horizontal_speed
    lateral_unit = np.array([-forward_unit[1], forward_unit[0]])
    bias_samples = samples_between(
        samples,
        sample_times_s,
        max(previous_s, last_s - config.bias_window_s),
        last_s,
    )
    forward_bias = (
        sum(accel_component(sample, config.forward_axis) for sample in bias_samples)
        / len(bias_samples)
        if bias_samples
        else 0.0
    )
    lateral_bias = (
        sum(accel_component(sample, config.lateral_axis) for sample in bias_samples)
        / len(bias_samples)
        if bias_samples
        else 0.0
    )

    span_start_s = cv_bridge.gps_seconds(reference[span.start_reference_index].week, span.start_tow_s)
    span_end_s = cv_bridge.gps_seconds(reference[span.end_reference_index].week, span.end_tow_s)
    imu_samples = samples_between(samples, sample_times_s, last_s, span_end_s)
    if not imu_samples:
        return None

    position_enu = np.zeros(3)
    velocity = np.array(velocity_enu, dtype=float)
    sample_index = 0
    current_s = last_s
    predicted: list[comparison.SolutionEpoch] = []

    for ref_index in range(span.start_reference_index, span.end_reference_index + 1):
        ref = reference[ref_index]
        target_s = cv_bridge.gps_seconds(ref.week, ref.tow)
        while sample_index < len(imu_samples) and sample_seconds(imu_samples[sample_index]) <= target_s:
            next_s = min(sample_seconds(imu_samples[sample_index]), target_s)
            dt = max(0.0, next_s - current_s)
            if dt > 0.0:
                accel_enu = horizontal_accel_enu(
                    imu_samples[sample_index],
                    forward_unit,
                    lateral_unit,
                    forward_bias,
                    lateral_bias,
                    config,
                )
                position_enu += velocity * dt + 0.5 * accel_enu * dt * dt
                velocity += accel_enu * dt
                current_s = next_s
            sample_index += 1
        if current_s < target_s:
            dt = target_s - current_s
            position_enu += velocity * dt
            current_s = target_s
        ecef = origin_ecef + enu_to_ecef_delta(position_enu, origin_lat, origin_lon)
        lat_deg, lon_deg, height_m = ppc_metrics.llh_from_ecef(
            float(ecef[0]),
            float(ecef[1]),
            float(ecef[2]),
        )
        predicted.append(
            comparison.SolutionEpoch(
                ref.week,
                ref.tow,
                lat_deg,
                lon_deg,
                height_m,
                ecef,
                config.bridge_status,
                config.bridge_num_satellites,
            )
        )
    return predicted


def bridge_dropout_spans(
    reference: list[comparison.ReferenceEpoch],
    baseline_epochs: list[comparison.SolutionEpoch],
    baseline_records: list[dict[str, object]],
    imu_samples: list[ImuSample],
    config: IMUBridgeConfig,
) -> tuple[list[comparison.SolutionEpoch], list[dict[str, object]]]:
    anchors = cv_bridge.trusted_anchor_epochs(
        reference,
        baseline_epochs,
        config,  # type: ignore[arg-type]
    )
    spans = cv_bridge.dropout_spans(baseline_records)
    selected_by_key = cv_bridge.solution_by_key(baseline_epochs)
    sample_times_s = [sample_seconds(sample) for sample in imu_samples]
    selected_rows: list[dict[str, object]] = []

    for span in spans:
        row: dict[str, object] = {
            "start_reference_index": span.start_reference_index,
            "end_reference_index": span.end_reference_index,
            "start_tow_s": cv_bridge.rounded(span.start_tow_s),
            "end_tow_s": cv_bridge.rounded(span.end_tow_s),
            "duration_s": cv_bridge.rounded(span.duration_s),
            "distance_m": cv_bridge.rounded(span.distance_m),
            "segment_count": span.segment_count,
            "bridge_applied": False,
            "generated_epochs": 0,
            "reject_reason": None,
        }
        if span.duration_s > config.max_gap_s:
            row["reject_reason"] = "span_too_long"
            selected_rows.append(row)
            continue
        if span.end_reference_index >= len(reference):
            row["reject_reason"] = "reference_index_out_of_range"
            selected_rows.append(row)
            continue
        anchors_pair = cv_bridge.select_velocity_anchors(
            anchors,
            span,
            reference[span.start_reference_index].week,
            config,
        )
        if anchors_pair is None:
            row["reject_reason"] = "missing_recent_trusted_anchors"
            selected_rows.append(row)
            continue
        previous_anchor, last_anchor = anchors_pair
        predicted = propagated_imu_epochs(
            reference,
            span,
            previous_anchor,
            last_anchor,
            imu_samples,
            sample_times_s,
            config,
        )
        if predicted is None:
            row["reject_reason"] = "imu_propagation_unavailable"
            selected_rows.append(row)
            continue
        for epoch in predicted:
            selected_by_key[cv_bridge.epoch_key(epoch)] = epoch
        row.update(
            {
                "bridge_applied": True,
                "generated_epochs": len(predicted),
                "previous_anchor_tow_s": cv_bridge.rounded(previous_anchor.tow),
                "last_anchor_tow_s": cv_bridge.rounded(last_anchor.tow),
                "anchor_age_s": cv_bridge.rounded(span.start_tow_s - last_anchor.tow),
                "velocity_baseline_s": cv_bridge.rounded(last_anchor.tow - previous_anchor.tow),
            }
        )
        selected_rows.append(row)

    return [selected_by_key[key] for key in sorted(selected_by_key)], selected_rows


def run_output_path(template: str, run: cv_bridge.RunSpec) -> Path:
    return Path(template.format(city=run.city, run=run.run, key=run.key))


def summarize_run(
    dataset_root: Path,
    run: cv_bridge.RunSpec,
    baseline_pos_template: str,
    output_pos_template: str,
    run_summary_template: str | None,
    config: IMUBridgeConfig,
) -> dict[str, object]:
    reference_csv = dataset_root / run.city / run.run / "reference.csv"
    imu_csv = dataset_root / run.city / run.run / "imu.csv"
    baseline_pos = run_output_path(baseline_pos_template, run)
    output_pos = run_output_path(output_pos_template, run)
    reference = comparison.read_reference_csv(reference_csv)
    baseline_epochs = comparison.read_libgnss_pos(baseline_pos)
    imu_samples = read_imu_csv(imu_csv)
    baseline_records = ppc_metrics.ppc_official_segment_records(
        reference,
        baseline_epochs,
        config.match_tolerance_s,
        config.threshold_m,
    )
    bridged_epochs, bridge_rows = bridge_dropout_spans(
        reference,
        baseline_epochs,
        baseline_records,
        imu_samples,
        config,
    )
    bridged_records = ppc_metrics.ppc_official_segment_records(
        reference,
        bridged_epochs,
        config.match_tolerance_s,
        config.threshold_m,
    )
    cv_bridge.enrich_bridge_rows(bridge_rows, baseline_records, bridged_records)
    pos_writer.write_pos(output_pos, bridged_epochs)

    baseline_metrics = ppc_metrics.summarize_solution_epochs(
        reference,
        baseline_epochs,
        fixed_status=4,
        label=f"{run.key} baseline",
        match_tolerance_s=config.match_tolerance_s,
        solver_wall_time_s=None,
    )
    bridged_metrics = ppc_metrics.summarize_solution_epochs(
        reference,
        bridged_epochs,
        fixed_status=4,
        label=f"{run.key} imu bridge",
        match_tolerance_s=config.match_tolerance_s,
        solver_wall_time_s=None,
    )
    payload = {
        "key": run.key,
        "city": run.city,
        "run": run.run,
        "reference_csv": str(reference_csv),
        "imu_csv": str(imu_csv),
        "baseline_pos": str(baseline_pos),
        "output_pos": str(output_pos),
        "config": config.__dict__,
        "selection": cv_bridge.bridge_selection_summary(bridge_rows),
        "baseline": baseline_metrics,
        "metrics": bridged_metrics,
        "delta_vs_baseline": ppc_metrics.solution_metric_delta(bridged_metrics, baseline_metrics),
        "bridge_spans": bridge_rows,
    }
    if run_summary_template:
        summary_path = run_output_path(run_summary_template, run)
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        summary_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload


def build_matrix_payload(runs: list[dict[str, object]], title: str, config: IMUBridgeConfig) -> dict[str, object]:
    payload = cv_bridge.build_matrix_payload(runs, title, config)  # type: ignore[arg-type]
    payload["bridge_model"] = "imu_horizontal_acceleration"
    payload["bridge_label"] = "IMU bridge"
    return payload


def render_markdown(payload: dict[str, object]) -> str:
    return cv_bridge.render_markdown(payload)


def render_png(payload: dict[str, object], output: Path) -> None:
    cv_bridge.render_png(payload, output)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument(
        "--ppc-run",
        action="append",
        type=cv_bridge.parse_run,
        dest="runs",
        help="Run to include as CITY/RUN. Defaults to all six PPC runs.",
    )
    parser.add_argument(
        "--baseline-pos-template",
        default="output/ppc_coverage_matrix_floatreset10/{key}.pos",
    )
    parser.add_argument("--run-output-template", required=True)
    parser.add_argument("--run-summary-template")
    parser.add_argument("--max-gap-s", type=float, default=10.0)
    parser.add_argument("--max-anchor-age-s", type=float, default=2.0)
    parser.add_argument("--max-velocity-baseline-s", type=float, default=1.0)
    parser.add_argument("--bridge-status", type=int, default=3)
    parser.add_argument("--bridge-num-satellites", type=int, default=0)
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--threshold-m", type=float, default=0.50)
    parser.add_argument("--bias-window-s", type=float, default=1.0)
    parser.add_argument("--min-heading-speed-mps", type=float, default=0.5)
    parser.add_argument("--max-horizontal-accel-mps2", type=float, default=3.0)
    parser.add_argument("--forward-axis", choices=("x", "y"), default="x")
    parser.add_argument("--lateral-axis", choices=("x", "y"), default="y")
    parser.add_argument("--forward-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--lateral-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--anchor-mode", choices=("scored", "telemetry"), default="scored")
    parser.add_argument(
        "--anchor-statuses",
        help="Comma-separated libgnss++ statuses for --anchor-mode telemetry, e.g. FIXED,FLOAT or 4,3.",
    )
    parser.add_argument("--anchor-min-ratio", type=float)
    parser.add_argument("--anchor-max-prefit-rms-m", type=float)
    parser.add_argument("--anchor-max-post-rms-m", type=float)
    parser.add_argument("--anchor-max-suppressed-outliers", type=int)
    parser.add_argument("--summary-json", type=Path, required=True)
    parser.add_argument("--markdown-output", type=Path)
    parser.add_argument("--output-png", type=Path)
    parser.add_argument("--title", default="PPC causal IMU dropout bridge")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.forward_axis == args.lateral_axis:
        raise SystemExit("--forward-axis and --lateral-axis must differ")
    config = IMUBridgeConfig(
        max_gap_s=args.max_gap_s,
        max_anchor_age_s=args.max_anchor_age_s,
        max_velocity_baseline_s=args.max_velocity_baseline_s,
        bridge_status=args.bridge_status,
        bridge_num_satellites=args.bridge_num_satellites,
        match_tolerance_s=args.match_tolerance_s,
        threshold_m=args.threshold_m,
        bias_window_s=args.bias_window_s,
        min_heading_speed_mps=args.min_heading_speed_mps,
        max_horizontal_accel_mps2=args.max_horizontal_accel_mps2,
        forward_axis=args.forward_axis,
        lateral_axis=args.lateral_axis,
        forward_sign=args.forward_sign,
        lateral_sign=args.lateral_sign,
        anchor_mode=args.anchor_mode,
        anchor_statuses=cv_bridge.parse_statuses(args.anchor_statuses),
        anchor_min_ratio=args.anchor_min_ratio,
        anchor_max_prefit_rms_m=args.anchor_max_prefit_rms_m,
        anchor_max_post_rms_m=args.anchor_max_post_rms_m,
        anchor_max_suppressed_outliers=args.anchor_max_suppressed_outliers,
    )
    runs = [
        summarize_run(
            args.dataset_root,
            run,
            args.baseline_pos_template,
            args.run_output_template,
            args.run_summary_template,
            config,
        )
        for run in (args.runs or cv_bridge.default_run_specs())
    ]
    payload = build_matrix_payload(runs, args.title, config)
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.markdown_output:
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(render_markdown(payload), encoding="utf-8")
    if args.output_png:
        render_png(payload, args.output_png)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
