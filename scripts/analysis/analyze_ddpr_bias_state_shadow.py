#!/usr/bin/env python3
"""Score causal DDPR bias-state shadow rows against offline truth labels."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path


KEY_FIELDS = ("current_epoch", "target", "reference", "signal")


def key(row: dict[str, str]) -> tuple[str, ...]:
    return tuple(row[name] for name in KEY_FIELDS)


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def ratio(numerator: float | None, denominator: float | None) -> float | None:
    if numerator is None or denominator is None or denominator == 0.0:
        return None
    return numerator / denominator


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bias-csv", required=True, type=Path)
    parser.add_argument("--truth-quality-csv", required=True, type=Path)
    parser.add_argument("--json", type=Path)
    parser.add_argument("--gross-threshold-m", type=float, default=4.0)
    parser.add_argument("--clean-threshold-m", type=float, default=1.0)
    args = parser.parse_args()

    with args.truth_quality_csv.open(newline="", encoding="utf-8") as stream:
        truth = {
            key(row): abs(float(row["current_predicted_ddpr_residual_m"]))
            for row in csv.DictReader(stream)
            if row["imu_geometry_evaluated"] == "1"
        }

    joined: list[dict[str, float | str]] = []
    epoch_biases: dict[str, list[float]] = defaultdict(list)
    total_rows = 0
    usable_rows = 0
    with args.bias_csv.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            total_rows += 1
            if row["prediction_usable"] != "1":
                continue
            usable_rows += 1
            truth_abs = truth.get(key(row))
            if truth_abs is None:
                continue
            raw = float(row["raw_residual_m"])
            corrected = float(row["corrected_residual_m"])
            prior_bias = float(row["prior_bias_m"])
            if not all(math.isfinite(value) for value in
                       (truth_abs, raw, corrected, prior_bias)):
                continue
            epoch_biases[row["current_epoch"]].append(prior_bias)
            joined.append({
                "epoch": row["current_epoch"],
                "truth_abs": truth_abs,
                "raw_abs": abs(raw),
                "corrected_abs": abs(corrected),
                "raw": raw,
                "prior_bias": prior_bias,
            })

    epoch_median = {
        epoch: statistics.median(values)
        for epoch, values in epoch_biases.items()
    }
    gross = [
        row for row in joined
        if row["truth_abs"] > args.gross_threshold_m
    ]
    clean = [
        row for row in joined
        if row["truth_abs"] < args.clean_threshold_m
    ]

    gross_raw = [float(row["raw_abs"]) for row in gross]
    gross_corrected = [float(row["corrected_abs"]) for row in gross]
    clean_raw = [float(row["raw_abs"]) for row in clean]
    clean_corrected = [float(row["corrected_abs"]) for row in clean]
    common_corrected = [
        abs(float(row["raw"]) - epoch_median[str(row["epoch"])])
        for row in gross
    ]

    gross_raw_median = percentile(gross_raw, 0.5)
    gross_corrected_median = percentile(gross_corrected, 0.5)
    clean_raw_p95 = percentile(clean_raw, 0.95)
    clean_corrected_p95 = percentile(clean_corrected, 0.95)
    common_corrected_median = percentile(common_corrected, 0.5)
    improvement = None
    if gross_raw_median:
        improvement = 1.0 - float(gross_corrected_median) / gross_raw_median

    summary = {
        "inputs": {
            "bias_csv": str(args.bias_csv),
            "truth_quality_csv": str(args.truth_quality_csv),
            "gross_threshold_m": args.gross_threshold_m,
            "clean_threshold_m": args.clean_threshold_m,
        },
        "support": {
            "total_bias_rows": total_rows,
            "usable_bias_rows": usable_rows,
            "truth_joined_rows": len(joined),
            "gross_rows": len(gross),
            "clean_rows": len(clean),
        },
        "gross": {
            "raw_abs_median_m": gross_raw_median,
            "corrected_abs_median_m": gross_corrected_median,
            "median_improvement_fraction": improvement,
            "same_epoch_common_bias_corrected_abs_median_m":
                common_corrected_median,
            "pair_specific_vs_common_ratio":
                ratio(gross_corrected_median, common_corrected_median),
        },
        "clean": {
            "raw_abs_p95_m": clean_raw_p95,
            "corrected_abs_p95_m": clean_corrected_p95,
            "p95_ratio": ratio(clean_corrected_p95, clean_raw_p95),
        },
    }
    summary["gate1"] = {
        "support_pass": len(gross) >= 100 and len(clean) >= 100,
        "gross_improvement_pass":
            improvement is not None and improvement >= 0.25,
        "clean_p95_pass":
            summary["clean"]["p95_ratio"] is not None and
            float(summary["clean"]["p95_ratio"]) <= 1.05,
    }
    summary["gate1"]["pass"] = all(summary["gate1"].values())

    rendered = json.dumps(summary, indent=2, sort_keys=True)
    print(rendered)
    if args.json:
        args.json.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
