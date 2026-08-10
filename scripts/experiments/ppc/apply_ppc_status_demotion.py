#!/usr/bin/env python3
"""Apply deployable POS-only FIX-to-FLOAT status demotion rules."""

from __future__ import annotations

from _paths import ANALYSIS_DIR, APPS_DIR, COMMANDS_DIR, PPC_DIR, ROOT_DIR, SCRIPTS_DIR

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path



@dataclass(frozen=True)
class DemotionRule:
    max_ratio: float | None
    min_baseline_m: float | None
    max_baseline_m: float | None
    max_nis_per_obs: float | None
    max_post_rms_m: float | None
    min_satellites: int | None = None
    low_satellite_ceiling: int | None = None
    low_satellite_max_ratio: float | None = None
    exonerate_min_satellites: int | None = None
    exonerate_max_prefit_rms_m: float | None = None
    exonerate_max_nis_per_obs: float | None = None
    kinematic_max_jump_m: float | None = None
    kinematic_min_acceleration_mps2: float | None = None
    kinematic_hold_epochs: int = 0
    kinematic_plateau_max_jump_m: float | None = None
    kinematic_max_hold_epochs: int = 0
    kinematic_secondary_min_jump_m: float | None = None
    kinematic_secondary_min_acceleration_mps2: float | None = None
    kinematic_secondary_min_prefit_rms_m: float | None = None
    kinematic_secondary_max_ratio: float | None = None
    kinematic_secondary_min_outliers: int | None = None
    kinematic_secondary_max_satellites: int | None = None
    residual_streak_min_prefit_rms_m: float | None = None
    residual_streak_max_ratio: float | None = None
    residual_streak_min_outliers: int | None = None
    residual_streak_min_outlier_fraction: float | None = None
    residual_streak_epochs: int = 0
    residual_streak_buffer_prefix: bool = False
    residual_spike_min_prefit_rms_m: float | None = None
    residual_spike_max_satellites: int | None = None


@dataclass(frozen=True)
class DemotionSummary:
    input_path: Path
    output_path: Path
    fixed_epochs: int
    demoted_epochs: int
    exonerated_epochs: int = 0
    kinematic_demoted_epochs: int = 0
    residual_streak_demoted_epochs: int = 0
    residual_spike_demoted_epochs: int = 0
    residual_overlap_epochs: int = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--input-dir", type=Path)
    source.add_argument(
        "--metrics-json",
        type=Path,
        help="goal-metrics JSON whose runs contain libgnss.pos paths",
    )
    source.add_argument(
        "--pos",
        action="append",
        metavar="RUN_KEY=PATH",
        help="selected POS input; repeat for each output run key",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--min-satellites", type=int, default=None)
    parser.add_argument("--low-satellite-ceiling", type=int, default=None)
    parser.add_argument("--low-satellite-max-ratio", type=float, default=None)
    parser.add_argument("--max-ratio", type=float, default=None)
    parser.add_argument("--min-baseline-m", type=float, default=None)
    parser.add_argument("--max-baseline-m", type=float, default=None)
    parser.add_argument("--max-nis-per-obs", type=float, default=None)
    parser.add_argument("--max-post-rms-m", type=float, default=None)
    parser.add_argument("--exonerate-min-satellites", type=int)
    parser.add_argument("--exonerate-max-prefit-rms-m", type=float)
    parser.add_argument("--exonerate-max-nis-per-obs", type=float)
    parser.add_argument("--kinematic-max-jump-m", type=float)
    parser.add_argument("--kinematic-min-acceleration-mps2", type=float)
    parser.add_argument("--kinematic-hold-epochs", type=int, default=0)
    parser.add_argument("--kinematic-plateau-max-jump-m", type=float)
    parser.add_argument("--kinematic-max-hold-epochs", type=int, default=0)
    parser.add_argument("--kinematic-secondary-min-jump-m", type=float)
    parser.add_argument("--kinematic-secondary-min-acceleration-mps2", type=float)
    parser.add_argument("--kinematic-secondary-min-prefit-rms-m", type=float)
    parser.add_argument("--kinematic-secondary-max-ratio", type=float)
    parser.add_argument("--kinematic-secondary-min-outliers", type=int)
    parser.add_argument("--kinematic-secondary-max-satellites", type=int)
    parser.add_argument("--residual-streak-min-prefit-rms-m", type=float)
    parser.add_argument("--residual-streak-max-ratio", type=float)
    parser.add_argument("--residual-streak-min-outliers", type=int)
    parser.add_argument("--residual-streak-min-outlier-fraction", type=float)
    parser.add_argument("--residual-streak-epochs", type=int, default=0)
    parser.add_argument(
        "--residual-streak-buffer-prefix",
        action="store_true",
        help=(
            "Buffer the bounded suspect prefix and demote it when the configured "
            "streak is confirmed. Adds (streak-1) epochs of output latency."
        ),
    )
    parser.add_argument("--residual-spike-min-prefit-rms-m", type=float)
    parser.add_argument("--residual-spike-max-satellites", type=int)
    return parser.parse_args()


def optional_float(parts: list[str], index: int) -> float | None:
    if index >= len(parts):
        return None
    try:
        return float(parts[index])
    except ValueError:
        return None


def should_demote(parts: list[str], rule: DemotionRule) -> bool:
    if len(parts) < 24:
        return False
    try:
        status = int(parts[8])
    except ValueError:
        return False
    if status != 4:
        return False

    ratio = optional_float(parts, 11)
    satellites = optional_float(parts, 9)
    baseline_m = optional_float(parts, 12)
    post_rms_m = optional_float(parts, 20)
    nis_per_obs = optional_float(parts, 23)

    if rule.min_satellites is not None and (
        satellites is None or satellites < rule.min_satellites
    ):
        return True
    if (
        rule.low_satellite_ceiling is not None
        and rule.low_satellite_max_ratio is not None
        and satellites is not None
        and ratio is not None
        and satellites <= rule.low_satellite_ceiling
        and ratio <= rule.low_satellite_max_ratio
    ):
        return True
    if rule.max_ratio is not None and (ratio is None or ratio > rule.max_ratio):
        return False
    if rule.min_baseline_m is not None and (
        baseline_m is None or baseline_m < rule.min_baseline_m
    ):
        return False
    if rule.max_baseline_m is not None and (
        baseline_m is None or baseline_m > rule.max_baseline_m
    ):
        return False

    fails_nis = (
        rule.max_nis_per_obs is not None
        and nis_per_obs is not None
        and nis_per_obs > rule.max_nis_per_obs
    )
    fails_post_rms = (
        rule.max_post_rms_m is not None
        and post_rms_m is not None
        and post_rms_m > rule.max_post_rms_m
    )
    return fails_nis or fails_post_rms


def should_exonerate(parts: list[str], rule: DemotionRule) -> bool:
    """Return whether strong runtime telemetry may override the base gate."""
    configured = (
        rule.exonerate_min_satellites,
        rule.exonerate_max_prefit_rms_m,
        rule.exonerate_max_nis_per_obs,
    )
    if any(value is None for value in configured):
        return False
    satellites = optional_float(parts, 9)
    prefit_rms_m = optional_float(parts, 18)
    nis_per_obs = optional_float(parts, 23)
    return (
        satellites is not None
        and satellites >= rule.exonerate_min_satellites
        and prefit_rms_m is not None
        and prefit_rms_m <= rule.exonerate_max_prefit_rms_m
        and nis_per_obs is not None
        and nis_per_obs <= rule.exonerate_max_nis_per_obs
    )


def position_time(parts: list[str]) -> tuple[int, float, tuple[float, float, float]] | None:
    if len(parts) < 5:
        return None
    try:
        return (
            int(parts[0]),
            float(parts[1]),
            (float(parts[2]), float(parts[3]), float(parts[4])),
        )
    except ValueError:
        return None


def apply_file(input_path: Path, output_path: Path, rule: DemotionRule) -> DemotionSummary:
    fixed_epochs = 0
    demoted_epochs = 0
    exonerated_epochs = 0
    kinematic_demoted_epochs = 0
    residual_streak_selected_indices: set[int] = set()
    residual_spike_selected_indices: set[int] = set()
    residual_streak_count = 0
    residual_streak_output_indices: list[int] = []
    previous: tuple[int, float, tuple[float, float, float]] | None = None
    previous_velocity: tuple[float, float, float] | None = None
    kinematic_hold_remaining = 0
    kinematic_quarantine_age = 0
    kinematic_quarantine_active = False
    output_lines: list[str] = []
    with input_path.open(encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.rstrip("\n")
            if not line.strip() or line.startswith("%"):
                output_lines.append(line)
                continue
            parts = line.split()
            try:
                is_fixed = len(parts) >= 9 and int(parts[8]) == 4
            except ValueError:
                is_fixed = False
            if is_fixed:
                fixed_epochs += 1
            current = position_time(parts)
            kinematic_trigger = False
            jump_m: float | None = None
            acceleration_mps2: float | None = None
            if current is not None and previous is not None and current[0] == previous[0]:
                dt = current[1] - previous[1]
                if 0.0 < dt <= 1.0:
                    velocity = tuple(
                        (current[2][axis] - previous[2][axis]) / dt for axis in range(3)
                    )
                    jump_m = math.sqrt(
                        sum(
                            (current[2][axis] - previous[2][axis]) ** 2
                            for axis in range(3)
                        )
                    )
                    acceleration_mps2 = (
                        math.sqrt(
                            sum(
                                (velocity[axis] - previous_velocity[axis]) ** 2
                                for axis in range(3)
                            )
                        )
                        / dt
                        if previous_velocity is not None
                        else None
                    )
                    kinematic_trigger = (
                        rule.kinematic_max_jump_m is not None
                        and rule.kinematic_min_acceleration_mps2 is not None
                        and jump_m > rule.kinematic_max_jump_m
                        and acceleration_mps2 is not None
                        and acceleration_mps2 > rule.kinematic_min_acceleration_mps2
                    )
                    previous_velocity = velocity
                else:
                    previous_velocity = None
            else:
                previous_velocity = None
            if current is not None:
                previous = current

            secondary_values = (
                rule.kinematic_secondary_min_jump_m,
                rule.kinematic_secondary_min_acceleration_mps2,
                rule.kinematic_secondary_min_prefit_rms_m,
                rule.kinematic_secondary_max_ratio,
                rule.kinematic_secondary_min_outliers,
                rule.kinematic_secondary_max_satellites,
            )
            secondary_configured = all(value is not None for value in secondary_values)
            prefit_rms_m = optional_float(parts, 18)
            ratio = optional_float(parts, 11)
            outliers = optional_float(parts, 17)
            observations = optional_float(parts, 14)
            satellites = optional_float(parts, 9)
            secondary_trigger = (
                secondary_configured
                and jump_m is not None
                and jump_m > rule.kinematic_secondary_min_jump_m
                and acceleration_mps2 is not None
                and acceleration_mps2
                > rule.kinematic_secondary_min_acceleration_mps2
                and prefit_rms_m is not None
                and prefit_rms_m > rule.kinematic_secondary_min_prefit_rms_m
                and ratio is not None
                and ratio <= rule.kinematic_secondary_max_ratio
                and outliers is not None
                and outliers >= rule.kinematic_secondary_min_outliers
                and satellites is not None
                and satellites <= rule.kinematic_secondary_max_satellites
            )
            residual_values = (
                rule.residual_streak_min_prefit_rms_m,
                rule.residual_streak_max_ratio,
                rule.residual_streak_min_outliers,
            )
            residual_configured = all(value is not None for value in residual_values)
            residual_matches = (
                is_fixed
                and residual_configured
                and rule.residual_streak_epochs > 0
                and prefit_rms_m is not None
                and prefit_rms_m > rule.residual_streak_min_prefit_rms_m
                and ratio is not None
                and ratio <= rule.residual_streak_max_ratio
                and outliers is not None
                and outliers >= rule.residual_streak_min_outliers
                and (
                    rule.residual_streak_min_outlier_fraction is None
                    or (
                        observations is not None
                        and observations > 0.0
                        and outliers / observations
                        >= rule.residual_streak_min_outlier_fraction
                    )
                )
            )
            residual_streak_count = residual_streak_count + 1 if residual_matches else 0
            if residual_matches:
                residual_streak_output_indices.append(len(output_lines))
            else:
                residual_streak_output_indices.clear()
            residual_streak_demote = (
                residual_matches
                and residual_streak_count >= rule.residual_streak_epochs
            )
            residual_spike_configured = (
                rule.residual_spike_min_prefit_rms_m is not None
                and rule.residual_spike_max_satellites is not None
            )
            residual_spike_demote = (
                is_fixed
                and residual_spike_configured
                and prefit_rms_m is not None
                and prefit_rms_m >= rule.residual_spike_min_prefit_rms_m
                and satellites is not None
                and satellites <= rule.residual_spike_max_satellites
            )
            if (
                residual_streak_demote
                and rule.residual_streak_buffer_prefix
                and residual_streak_count == rule.residual_streak_epochs
            ):
                residual_streak_selected_indices.update(residual_streak_output_indices)
                for output_index in residual_streak_output_indices[:-1]:
                    buffered_parts = output_lines[output_index].split()
                    if len(buffered_parts) >= 9 and buffered_parts[8] == "4":
                        buffered_parts[8] = "3"
                        output_lines[output_index] = " ".join(buffered_parts)
                        demoted_epochs += 1
            elif residual_streak_demote:
                residual_streak_selected_indices.add(len(output_lines))
            if residual_spike_demote:
                residual_spike_selected_indices.add(len(output_lines))
            if kinematic_trigger or secondary_trigger:
                kinematic_hold_remaining = max(
                    kinematic_hold_remaining, rule.kinematic_hold_epochs
                )
                kinematic_quarantine_age = 0
                kinematic_quarantine_active = True
            keep_quarantine = False
            if kinematic_quarantine_active:
                keep_quarantine = kinematic_hold_remaining > 0
                if (
                    not keep_quarantine
                    and rule.kinematic_plateau_max_jump_m is not None
                    and rule.kinematic_max_hold_epochs > 0
                    and kinematic_quarantine_age < rule.kinematic_max_hold_epochs
                    and jump_m is not None
                    and jump_m <= rule.kinematic_plateau_max_jump_m
                ):
                    keep_quarantine = True
                if keep_quarantine:
                    kinematic_quarantine_age += 1
                    kinematic_hold_remaining = max(0, kinematic_hold_remaining - 1)
                else:
                    kinematic_quarantine_active = False
            kinematic_demote = is_fixed and keep_quarantine

            base_demote = should_demote(parts, rule)
            exonerated = base_demote and should_exonerate(parts, rule)
            if exonerated:
                exonerated_epochs += 1
            if kinematic_demote:
                kinematic_demoted_epochs += 1
            if (
                (base_demote and not exonerated)
                or kinematic_demote
                or residual_streak_demote
                or residual_spike_demote
            ):
                parts[8] = "3"
                line = " ".join(parts)
                demoted_epochs += 1
            output_lines.append(line)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(output_lines) + "\n", encoding="utf-8")
    return DemotionSummary(
        input_path=input_path,
        output_path=output_path,
        fixed_epochs=fixed_epochs,
        demoted_epochs=demoted_epochs,
        exonerated_epochs=exonerated_epochs,
        kinematic_demoted_epochs=kinematic_demoted_epochs,
        residual_streak_demoted_epochs=len(residual_streak_selected_indices),
        residual_spike_demoted_epochs=len(residual_spike_selected_indices),
        residual_overlap_epochs=len(
            residual_streak_selected_indices & residual_spike_selected_indices
        ),
    )


def apply_directory(input_dir: Path, output_dir: Path, rule: DemotionRule) -> list[DemotionSummary]:
    pos_files = sorted(input_dir.glob("*.pos"))
    if not pos_files:
        raise SystemExit(f"no .pos files found in {input_dir}")
    return [
        apply_file(input_path, output_dir / input_path.name, rule)
        for input_path in pos_files
    ]


def apply_metrics(
    metrics_json: Path,
    output_dir: Path,
    rule: DemotionRule,
) -> list[DemotionSummary]:
    payload = json.loads(metrics_json.read_text(encoding="utf-8"))
    runs = payload.get("runs")
    if not isinstance(runs, list):
        raise SystemExit(f"{metrics_json}: missing runs list")
    summaries: list[DemotionSummary] = []
    for row in runs:
        if not isinstance(row, dict):
            continue
        key = row.get("key")
        libgnss = row.get("libgnss")
        if not isinstance(key, str) or not isinstance(libgnss, dict):
            continue
        path_text = libgnss.get("pos")
        if not isinstance(path_text, str) or not path_text:
            continue
        input_path = Path(path_text)
        if not input_path.is_absolute():
            input_path = ROOT_DIR / input_path
        summaries.append(apply_file(input_path, output_dir / f"{key}.pos", rule))
    if not summaries:
        raise SystemExit(f"{metrics_json}: no libgnss POS paths found")
    return summaries


def apply_specs(
    specs: list[str],
    output_dir: Path,
    rule: DemotionRule,
) -> list[DemotionSummary]:
    summaries: list[DemotionSummary] = []
    seen: set[str] = set()
    for spec in specs:
        try:
            key, path_text = spec.split("=", 1)
        except ValueError as error:
            raise SystemExit(f"invalid --pos value {spec!r}; expected RUN_KEY=PATH") from error
        if not key or key in seen:
            raise SystemExit(f"invalid or duplicate --pos run key: {key!r}")
        seen.add(key)
        input_path = Path(path_text)
        if not input_path.is_absolute():
            input_path = ROOT_DIR / input_path
        summaries.append(apply_file(input_path, output_dir / f"{key}.pos", rule))
    return summaries


def main() -> int:
    args = parse_args()
    if (args.low_satellite_ceiling is None) != (args.low_satellite_max_ratio is None):
        raise SystemExit(
            "--low-satellite-ceiling and --low-satellite-max-ratio must be used together"
        )
    exoneration_values = (
        args.exonerate_min_satellites,
        args.exonerate_max_prefit_rms_m,
        args.exonerate_max_nis_per_obs,
    )
    if any(value is not None for value in exoneration_values) and any(
        value is None for value in exoneration_values
    ):
        raise SystemExit("all three --exonerate-* options must be used together")
    kinematic_values = (
        args.kinematic_max_jump_m,
        args.kinematic_min_acceleration_mps2,
    )
    if any(value is not None for value in kinematic_values) != all(
        value is not None for value in kinematic_values
    ):
        raise SystemExit("both kinematic thresholds must be used together")
    if args.kinematic_hold_epochs < 0:
        raise SystemExit("--kinematic-hold-epochs must be >= 0")
    if args.kinematic_max_hold_epochs < 0:
        raise SystemExit("--kinematic-max-hold-epochs must be >= 0")
    secondary_values = (
        args.kinematic_secondary_min_jump_m,
        args.kinematic_secondary_min_acceleration_mps2,
        args.kinematic_secondary_min_prefit_rms_m,
        args.kinematic_secondary_max_ratio,
        args.kinematic_secondary_min_outliers,
        args.kinematic_secondary_max_satellites,
    )
    if any(value is not None for value in secondary_values) and any(
        value is None for value in secondary_values
    ):
        raise SystemExit("all six --kinematic-secondary-* options must be used together")
    residual_values = (
        args.residual_streak_min_prefit_rms_m,
        args.residual_streak_max_ratio,
        args.residual_streak_min_outliers,
    )
    if any(value is not None for value in residual_values) and any(
        value is None for value in residual_values
    ):
        raise SystemExit("all three --residual-streak-* telemetry options must be used together")
    if (
        args.residual_streak_min_outlier_fraction is not None
        and not all(value is not None for value in residual_values)
    ):
        raise SystemExit(
            "--residual-streak-min-outlier-fraction requires all three "
            "--residual-streak-* telemetry options"
        )
    if args.residual_streak_epochs < 0:
        raise SystemExit("--residual-streak-epochs must be >= 0")
    if (
        args.residual_streak_min_outlier_fraction is not None
        and not 0.0 <= args.residual_streak_min_outlier_fraction <= 1.0
    ):
        raise SystemExit("--residual-streak-min-outlier-fraction must be in [0, 1]")
    if all(value is not None for value in residual_values) != (
        args.residual_streak_epochs > 0
    ):
        raise SystemExit(
            "residual-streak telemetry options and a positive --residual-streak-epochs "
            "must be used together"
        )
    if args.residual_streak_buffer_prefix and args.residual_streak_epochs <= 1:
        raise SystemExit(
            "--residual-streak-buffer-prefix requires --residual-streak-epochs > 1"
        )
    spike_values = (
        args.residual_spike_min_prefit_rms_m,
        args.residual_spike_max_satellites,
    )
    if any(value is not None for value in spike_values) and any(
        value is None for value in spike_values
    ):
        raise SystemExit("both --residual-spike-* options must be used together")
    rule = DemotionRule(
        min_satellites=args.min_satellites,
        low_satellite_ceiling=args.low_satellite_ceiling,
        low_satellite_max_ratio=args.low_satellite_max_ratio,
        max_ratio=args.max_ratio,
        min_baseline_m=args.min_baseline_m,
        max_baseline_m=args.max_baseline_m,
        max_nis_per_obs=args.max_nis_per_obs,
        max_post_rms_m=args.max_post_rms_m,
        exonerate_min_satellites=args.exonerate_min_satellites,
        exonerate_max_prefit_rms_m=args.exonerate_max_prefit_rms_m,
        exonerate_max_nis_per_obs=args.exonerate_max_nis_per_obs,
        kinematic_max_jump_m=args.kinematic_max_jump_m,
        kinematic_min_acceleration_mps2=args.kinematic_min_acceleration_mps2,
        kinematic_hold_epochs=args.kinematic_hold_epochs,
        kinematic_plateau_max_jump_m=args.kinematic_plateau_max_jump_m,
        kinematic_max_hold_epochs=args.kinematic_max_hold_epochs,
        kinematic_secondary_min_jump_m=args.kinematic_secondary_min_jump_m,
        kinematic_secondary_min_acceleration_mps2=(
            args.kinematic_secondary_min_acceleration_mps2
        ),
        kinematic_secondary_min_prefit_rms_m=(
            args.kinematic_secondary_min_prefit_rms_m
        ),
        kinematic_secondary_max_ratio=args.kinematic_secondary_max_ratio,
        kinematic_secondary_min_outliers=args.kinematic_secondary_min_outliers,
        kinematic_secondary_max_satellites=(
            args.kinematic_secondary_max_satellites
        ),
        residual_streak_min_prefit_rms_m=args.residual_streak_min_prefit_rms_m,
        residual_streak_max_ratio=args.residual_streak_max_ratio,
        residual_streak_min_outliers=args.residual_streak_min_outliers,
        residual_streak_min_outlier_fraction=(
            args.residual_streak_min_outlier_fraction
        ),
        residual_streak_epochs=args.residual_streak_epochs,
        residual_streak_buffer_prefix=args.residual_streak_buffer_prefix,
        residual_spike_min_prefit_rms_m=args.residual_spike_min_prefit_rms_m,
        residual_spike_max_satellites=args.residual_spike_max_satellites,
    )
    if args.pos is not None:
        summaries = apply_specs(args.pos, args.output_dir, rule)
    elif args.metrics_json is not None:
        summaries = apply_metrics(args.metrics_json, args.output_dir, rule)
    else:
        assert args.input_dir is not None
        summaries = apply_directory(args.input_dir, args.output_dir, rule)
    total_fixed = sum(summary.fixed_epochs for summary in summaries)
    total_demoted = sum(summary.demoted_epochs for summary in summaries)
    print(f"wrote {args.output_dir}")
    print(f"demoted {total_demoted} of {total_fixed} FIXED epochs")
    print(
        "exonerated "
        f"{sum(summary.exonerated_epochs for summary in summaries)} base-gate epochs; "
        "kinematic demotions "
        f"{sum(summary.kinematic_demoted_epochs for summary in summaries)}"
    )
    print(
        "residual-streak selections "
        f"{sum(summary.residual_streak_demoted_epochs for summary in summaries)}"
    )
    print(
        "residual-spike selections "
        f"{sum(summary.residual_spike_demoted_epochs for summary in summaries)}"
    )
    print(
        "residual rule overlap "
        f"{sum(summary.residual_overlap_epochs for summary in summaries)}"
    )
    for summary in summaries:
        print(f"{summary.input_path.name}: {summary.demoted_epochs}/{summary.fixed_epochs}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
