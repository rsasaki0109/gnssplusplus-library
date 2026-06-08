#!/usr/bin/env python3
"""Apply deployable POS-only FIX-to-FLOAT status demotion rules."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class DemotionRule:
    max_ratio: float | None
    min_baseline_m: float | None
    max_baseline_m: float | None
    max_nis_per_obs: float | None
    max_post_rms_m: float | None


@dataclass(frozen=True)
class DemotionSummary:
    input_path: Path
    output_path: Path
    fixed_epochs: int
    demoted_epochs: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
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
    baseline_m = optional_float(parts, 12)
    post_rms_m = optional_float(parts, 20)
    nis_per_obs = optional_float(parts, 23)

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


def main() -> int:
    args = parse_args()
    rule = DemotionRule(
        max_ratio=args.max_ratio,
        min_baseline_m=args.min_baseline_m,
        max_baseline_m=args.max_baseline_m,
        max_nis_per_obs=args.max_nis_per_obs,
        max_post_rms_m=args.max_post_rms_m,
    )
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
