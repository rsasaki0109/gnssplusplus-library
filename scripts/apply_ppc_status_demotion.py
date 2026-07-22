#!/usr/bin/env python3
"""Apply deployable POS-only FIX-to-FLOAT status demotion rules."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class DemotionRule:
    max_ratio: float | None
    min_baseline_m: float | None
    max_baseline_m: float | None
    max_nis_per_obs: float | None
    max_post_rms_m: float | None
    min_satellites: int | None = None


@dataclass(frozen=True)
class DemotionSummary:
    input_path: Path
    output_path: Path
    fixed_epochs: int
    demoted_epochs: int


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
    parser.add_argument("--max-ratio", type=float, default=None)
    parser.add_argument("--min-baseline-m", type=float, default=None)
    parser.add_argument("--max-baseline-m", type=float, default=None)
    parser.add_argument("--max-nis-per-obs", type=float, default=None)
    parser.add_argument("--max-post-rms-m", type=float, default=None)
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


def apply_file(input_path: Path, output_path: Path, rule: DemotionRule) -> DemotionSummary:
    fixed_epochs = 0
    demoted_epochs = 0
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
            if should_demote(parts, rule):
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
    rule = DemotionRule(
        min_satellites=args.min_satellites,
        max_ratio=args.max_ratio,
        min_baseline_m=args.min_baseline_m,
        max_baseline_m=args.max_baseline_m,
        max_nis_per_obs=args.max_nis_per_obs,
        max_post_rms_m=args.max_post_rms_m,
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
    for summary in summaries:
        print(f"{summary.input_path.name}: {summary.demoted_epochs}/{summary.fixed_epochs}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
