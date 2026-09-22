#!/usr/bin/env python3
"""Compare matched-reference gnss_fgo_parity CSVs without changing inference.

The native CSV exporter matches reference timestamps within 0.11 s. Metrics
here describe those matched rows, not a new independent ground-truth dataset.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
from pathlib import Path
import statistics


def native_input_epochs(log: str, kind: str) -> int:
    """Count admitted reader inputs, never the post-filter solution count."""
    if kind == "fgo":
        matches = re.findall(r"^\s*rover=.*\((\d+) epochs", log, re.MULTILINE)
        if len(matches) != 1:
            raise ValueError("missing or ambiguous native FGO rover input count")
        return int(matches[0])
    if kind == "rtk":
        counts = []
        for name in ("exact base epochs", "interpolated base epochs", "skipped rover epochs"):
            matches = re.findall(r"^\s*" + name + r":\s*(\d+)\s*$", log, re.MULTILINE)
            if len(matches) != 1:
                raise ValueError(f"missing or ambiguous native RTK counter: {name}")
            counts.append(int(matches[0]))
        return sum(counts)
    raise ValueError(f"unsupported native log kind: {kind}")


def quantile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = fraction * (len(ordered) - 1)
    low = int(index)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (index - low) * (ordered[high] - ordered[low])


def finite(value: str | None) -> float | None:
    try:
        parsed = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def load(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"empty native epoch CSV: {path}")
    previous = -math.inf
    for row in rows:
        tow = finite(row.get("tow"))
        if tow is None or tow <= previous:
            raise ValueError("comparison requires one strictly increasing GPS-week segment")
        previous = tow
        for key in ("x_ecef_m", "y_ecef_m", "z_ecef_m", "e_err_m", "n_err_m", "u_err_m"):
            if finite(row.get(key)) is None:
                raise ValueError(f"missing/nonfinite {key} at {tow}")
    return rows


def summarize(rows: list[dict[str, str]], wrong_m: float, expected: int | None) -> dict:
    if expected is not None and expected < len(rows):
        raise ValueError("expected epochs cannot be smaller than exported rows")
    errors = [math.sqrt(sum(float(row[k]) ** 2 for k in
                           ("e_err_m", "n_err_m", "u_err_m"))) for row in rows]
    horizontal = [math.hypot(float(r["e_err_m"]), float(r["n_err_m"])) for r in rows]
    fixed = [row["status"] == "FIXED" for row in rows]
    wrong = [f and e > wrong_m for f, e in zip(fixed, errors)]
    times = [float(row["tow"]) for row in rows]
    cadence = statistics.median(b - a for a, b in zip(times, times[1:])) if len(times) > 1 else 0.0
    event_ends = [i for i, value in enumerate(wrong) if value and (
        i + 1 == len(rows) or not wrong[i + 1] or times[i + 1] - times[i] > 1.5 * cadence)]
    recovery_delays = []
    censored = 0
    for end in event_ends:
        recovered = next((j for j in range(end + 1, len(rows))
                          if fixed[j] and errors[j] <= wrong_m), None)
        if recovered is None:
            censored += 1
        else:
            recovery_delays.append(times[recovered] - times[end])
    traces = [finite(row.get("position_covariance_trace_m2")) for row in rows]
    valid_cov = [value is not None and value > 0.0 for value in traces]
    fixed_count = sum(fixed)
    return {
        "matched_rows": len(rows), "expected_input_epochs": expected,
        "missing_or_unmatched_epochs": None if expected is None else expected - len(rows),
        "missing_or_unmatched_pct": None if not expected else 100 * (expected - len(rows)) / expected,
        "fixed_epochs": fixed_count, "fix_pct_matched": 100 * fixed_count / len(rows),
        "wrong_fix_threshold_3d_m": wrong_m, "wrong_fix_epochs": sum(wrong),
        "wrong_fix_pct_fixed": 100 * sum(wrong) / fixed_count if fixed_count else None,
        "correct_fix_epochs": fixed_count - sum(wrong),
        "within_50cm_pct_matched": 100 * sum(e < 0.5 for e in errors) / len(rows),
        "horizontal_p50_m": quantile(horizontal, 0.5),
        "horizontal_p95_m": quantile(horizontal, 0.95),
        "error_3d_p95_m": quantile(errors, 0.95),
        "positive_covariance_rows": sum(valid_cov),
        "positive_covariance_fixed_rows": sum(f and c for f, c in zip(fixed, valid_cov)),
        "covariance_trace_p50_m2": quantile([v for v in traces if v is not None and v > 0], 0.5),
        "wrong_fix_events": len(event_ends),
        "recovery_delay_definition": "last wrong FIX in event to next FIX with 3D error <= threshold",
        "recovered_events": len(recovery_delays), "unrecovered_events": censored,
        "recovery_delay_p50_s": quantile(recovery_delays, 0.5),
        "recovery_delay_p95_s": quantile(recovery_delays, 0.95),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--candidate", required=True, type=Path)
    parser.add_argument("--output-json", required=True, type=Path)
    parser.add_argument("--expected-epochs", type=int)
    parser.add_argument("--wrong-fix-threshold-m", type=float, default=2.0)
    parser.add_argument("--baseline-wall-s", type=float)
    parser.add_argument("--candidate-wall-s", type=float)
    args = parser.parse_args()
    if args.wrong_fix_threshold_m <= 0 or not math.isfinite(args.wrong_fix_threshold_m):
        parser.error("wrong FIX threshold must be finite and positive")
    baseline, candidate = load(args.baseline), load(args.candidate)
    columns = ("tow", "status", "x_ecef_m", "y_ecef_m", "z_ecef_m", "ratio", "nfixed")
    unchanged = len(baseline) == len(candidate) and all(
        all(a.get(key) == b.get(key) for key in columns) for a, b in zip(baseline, candidate))
    payload = {
        "schema_version": 1,
        "comparison_scope": "existing PPC development data, matched native CSV rows",
        "position_status_ratio_count_text_identical": unchanged,
        "baseline": summarize(baseline, args.wrong_fix_threshold_m, args.expected_epochs),
        "candidate": summarize(candidate, args.wrong_fix_threshold_m, args.expected_epochs),
        "baseline_csv_sha256": hashlib.sha256(args.baseline.read_bytes()).hexdigest(),
        "candidate_csv_sha256": hashlib.sha256(args.candidate.read_bytes()).hexdigest(),
        "baseline_wall_s": args.baseline_wall_s, "candidate_wall_s": args.candidate_wall_s,
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"unchanged_solution_columns": unchanged, "output": str(args.output_json)}))


if __name__ == "__main__":
    main()
