#!/usr/bin/env python3
"""Summarize PPC SPP jump-gate policy sweep outputs across runs."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
from typing import Any


REPORT_FIELDS = [
    "label",
    "sweep_json",
    "policy_enabled",
    "policy_candidate_count",
    "selected_rate_mps",
    "selected_min_jump_m",
    "baseline_valid_epochs",
    "policy_valid_epochs",
    "valid_epoch_delta",
    "baseline_matched_epochs",
    "policy_matched_epochs",
    "matched_epoch_delta",
    "baseline_positioning_rate_pct",
    "policy_positioning_rate_pct",
    "positioning_drop_pct",
    "baseline_median_h_m",
    "policy_median_h_m",
    "median_h_delta_m",
    "baseline_p95_h_m",
    "policy_p95_h_m",
    "p95_h_delta_m",
    "baseline_max_h_m",
    "policy_max_h_m",
    "max_h_delta_m",
    "baseline_p95_abs_up_m",
    "policy_p95_abs_up_m",
    "p95_abs_up_delta_m",
    "jump_gate_rejected_epochs",
    "jump_gate_rejected_groups",
    "bridge_inserted_epochs",
    "max_positioning_drop_pct",
    "min_positioning_rate_pct",
]


@dataclass(frozen=True)
class SweepSpec:
    label: str
    path: Path


def parse_sweep_spec(text: str) -> SweepSpec:
    if "=" in text:
        label, path_text = text.split("=", 1)
        label = label.strip()
        path_text = path_text.strip()
        if not label or not path_text:
            raise argparse.ArgumentTypeError("expected --sweep label=path")
        return SweepSpec(label, Path(path_text))
    path = Path(text)
    return SweepSpec(path.stem, path)


def finite_float(value: Any, key: str) -> float:
    if value is None:
        raise SystemExit(f"Missing numeric value for {key}")
    result = float(value)
    if not math.isfinite(result):
        raise SystemExit(f"Non-finite numeric value for {key}: {value}")
    return result


def optional_float(value: Any) -> float | None:
    if value is None:
        return None
    result = float(value)
    if not math.isfinite(result):
        return None
    return result


def optional_int(value: Any) -> int | None:
    if value is None:
        return None
    return int(value)


def rounded(value: float) -> float:
    return round(value, 6)


def delta(policy: dict[str, Any], baseline: dict[str, Any], key: str) -> float:
    return rounded(finite_float(policy.get(key), key) - finite_float(baseline.get(key), key))


def find_baseline(results: list[Any], label: str) -> dict[str, Any]:
    for row in results:
        if (
            isinstance(row, dict)
            and row.get("max_position_jump_rate_mps") is None
            and row.get("max_position_jump_min_m") is None
        ):
            return row
    raise SystemExit(f"No baseline row found in sweep summary for {label}")


def load_policy_row(spec: SweepSpec) -> tuple[dict[str, Any], dict[str, Any]]:
    if not spec.path.exists():
        raise SystemExit(f"Sweep summary not found for {spec.label}: {spec.path}")
    payload = json.loads(spec.path.read_text(encoding="utf-8"))
    results = payload.get("results")
    if not isinstance(results, list):
        raise SystemExit(f"Sweep summary for {spec.label} has no results list")
    baseline = find_baseline(results, spec.label)

    policy = payload.get("policy")
    if not isinstance(policy, dict):
        raise SystemExit(f"Sweep summary for {spec.label} has no policy object")
    policy_best = policy.get("best_by_p95_h_m")
    if not isinstance(policy_best, dict):
        raise SystemExit(f"Sweep summary for {spec.label} has no policy best row")

    report_row: dict[str, Any] = {
        "label": spec.label,
        "sweep_json": str(spec.path),
        "policy_enabled": bool(policy.get("enabled", False)),
        "policy_candidate_count": optional_int(policy.get("candidate_count")),
        "selected_rate_mps": optional_float(policy_best.get("max_position_jump_rate_mps")),
        "selected_min_jump_m": optional_float(policy_best.get("max_position_jump_min_m")),
        "baseline_valid_epochs": optional_int(baseline.get("valid_epochs")),
        "policy_valid_epochs": optional_int(policy_best.get("valid_epochs")),
        "baseline_matched_epochs": optional_int(baseline.get("matched_epochs")),
        "policy_matched_epochs": optional_int(policy_best.get("matched_epochs")),
        "baseline_positioning_rate_pct": finite_float(
            baseline.get("positioning_rate_pct"), "positioning_rate_pct"
        ),
        "policy_positioning_rate_pct": finite_float(
            policy_best.get("positioning_rate_pct"), "positioning_rate_pct"
        ),
        "baseline_median_h_m": finite_float(baseline.get("median_h_m"), "median_h_m"),
        "policy_median_h_m": finite_float(policy_best.get("median_h_m"), "median_h_m"),
        "baseline_p95_h_m": finite_float(baseline.get("p95_h_m"), "p95_h_m"),
        "policy_p95_h_m": finite_float(policy_best.get("p95_h_m"), "p95_h_m"),
        "baseline_max_h_m": finite_float(baseline.get("max_h_m"), "max_h_m"),
        "policy_max_h_m": finite_float(policy_best.get("max_h_m"), "max_h_m"),
        "baseline_p95_abs_up_m": finite_float(
            baseline.get("p95_abs_up_m"), "p95_abs_up_m"
        ),
        "policy_p95_abs_up_m": finite_float(
            policy_best.get("p95_abs_up_m"), "p95_abs_up_m"
        ),
        "jump_gate_rejected_epochs": optional_int(
            policy_best.get("jump_gate_rejected_epochs")
        ),
        "jump_gate_rejected_groups": optional_int(
            policy_best.get("jump_gate_rejected_groups")
        ),
        "bridge_inserted_epochs": optional_int(policy_best.get("bridge_inserted_epochs")),
        "max_positioning_drop_pct": optional_float(policy.get("max_positioning_drop_pct")),
        "min_positioning_rate_pct": optional_float(policy.get("min_positioning_rate_pct")),
    }
    report_row.update(
        {
            "valid_epoch_delta": (
                report_row["policy_valid_epochs"] - report_row["baseline_valid_epochs"]
                if report_row["policy_valid_epochs"] is not None
                and report_row["baseline_valid_epochs"] is not None
                else None
            ),
            "matched_epoch_delta": (
                report_row["policy_matched_epochs"]
                - report_row["baseline_matched_epochs"]
                if report_row["policy_matched_epochs"] is not None
                and report_row["baseline_matched_epochs"] is not None
                else None
            ),
            "positioning_drop_pct": rounded(
                report_row["baseline_positioning_rate_pct"]
                - report_row["policy_positioning_rate_pct"]
            ),
            "median_h_delta_m": delta(policy_best, baseline, "median_h_m"),
            "p95_h_delta_m": delta(policy_best, baseline, "p95_h_m"),
            "max_h_delta_m": delta(policy_best, baseline, "max_h_m"),
            "p95_abs_up_delta_m": delta(policy_best, baseline, "p95_abs_up_m"),
        }
    )
    return report_row, payload


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=REPORT_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in REPORT_FIELDS})


def check_rows(
    rows: list[dict[str, Any]],
    max_p95_delta_m: float | None,
    max_positioning_drop_pct: float | None,
) -> list[str]:
    failures: list[str] = []
    for row in rows:
        label = row["label"]
        p95_delta = finite_float(row["p95_h_delta_m"], "p95_h_delta_m")
        positioning_drop = finite_float(row["positioning_drop_pct"], "positioning_drop_pct")
        if max_p95_delta_m is not None and p95_delta > max_p95_delta_m + 1e-9:
            failures.append(
                f"{label}: p95 delta {p95_delta:.6f} m exceeds {max_p95_delta_m:.6f} m"
            )
        if (
            max_positioning_drop_pct is not None
            and positioning_drop > max_positioning_drop_pct + 1e-9
        ):
            failures.append(
                f"{label}: positioning drop {positioning_drop:.6f}% exceeds "
                f"{max_positioning_drop_pct:.6f}%"
            )
    return failures


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sweep",
        action="append",
        type=parse_sweep_spec,
        required=True,
        help="Jump-sweep summary JSON as label=path. Repeat for each PPC run.",
    )
    parser.add_argument("--summary-json", type=Path, required=True)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument(
        "--max-p95-delta-m",
        type=float,
        help="Fail when selected policy p95 H exceeds baseline by more than this value",
    )
    parser.add_argument(
        "--max-positioning-drop-pct",
        type=float,
        help="Fail when selected policy positioning rate drops by more than this many percentage points",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    labels = [spec.label for spec in args.sweep]
    if len(set(labels)) != len(labels):
        raise SystemExit("--sweep labels must be unique")
    if args.max_p95_delta_m is not None and not math.isfinite(args.max_p95_delta_m):
        raise SystemExit("--max-p95-delta-m must be finite")
    if (
        args.max_positioning_drop_pct is not None
        and (
            not math.isfinite(args.max_positioning_drop_pct)
            or args.max_positioning_drop_pct < 0.0
        )
    ):
        raise SystemExit("--max-positioning-drop-pct must be finite and >= 0")

    rows: list[dict[str, Any]] = []
    source_payloads: dict[str, Any] = {}
    for spec in args.sweep:
        row, payload = load_policy_row(spec)
        rows.append(row)
        source_payloads[spec.label] = {
            "reference_csv": payload.get("reference_csv"),
            "pos": payload.get("pos"),
            "bridge_max_gap_s": payload.get("bridge_max_gap_s"),
            "bridge_max_anchor_speed_mps": payload.get("bridge_max_anchor_speed_mps"),
        }

    failures = check_rows(rows, args.max_p95_delta_m, args.max_positioning_drop_pct)
    payload = {
        "sweep_count": len(rows),
        "checks": {
            "max_p95_delta_m": args.max_p95_delta_m,
            "max_positioning_drop_pct": args.max_positioning_drop_pct,
            "passed": not failures,
            "failures": failures,
        },
        "sources": source_payloads,
        "runs": rows,
    }

    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    write_csv(args.csv, rows)

    print("PPC SPP policy report")
    print(f"  sweeps: {len(rows)}")
    print(f"  summary: {args.summary_json}")
    print(f"  csv: {args.csv}")
    for row in rows:
        rate = row["selected_rate_mps"]
        min_jump = row["selected_min_jump_m"]
        label = "baseline" if rate is None else f"rate={rate:g} min={min_jump:g}"
        print(
            f"    {row['label']}: {label} "
            f"p95_h={float(row['policy_p95_h_m']):.3f} "
            f"delta_p95={float(row['p95_h_delta_m']):.3f} "
            f"positioning={float(row['policy_positioning_rate_pct']):.3f}% "
            f"drop={float(row['positioning_drop_pct']):.3f}%"
        )

    if failures:
        print("  check failures:", file=sys.stderr)
        for failure in failures:
            print(f"    {failure}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
