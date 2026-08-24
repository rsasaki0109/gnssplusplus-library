#!/usr/bin/env python3
"""Score SPP receiver-clock telemetry against an independent IGS CLK series."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import sys
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


SCHEMA_VERSION = "libgnsspp.timing_holdover.v1"
GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
HORIZONS_S = (300, 600, 900, 1800)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME", "gnss timing-holdover-signoff"))
    parser.add_argument("--clock-csv", type=Path, required=True)
    parser.add_argument("--reference-clk", type=Path, required=True)
    parser.add_argument("--station", default="BRUX")
    parser.add_argument("--role", choices=("development", "sealed_holdout"), required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--profile", type=Path)
    return parser.parse_args(argv)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def record(path: Path) -> dict[str, Any]:
    return {"path": str(path.resolve()), "bytes": path.stat().st_size, "sha256": sha256_file(path)}


def gps_key(week: int, tow_s: float) -> int:
    return round((week * 604800.0 + tow_s) * 10)


def calendar_gps_key(year: int, month: int, day: int, hour: int, minute: int, second: float) -> int:
    whole = int(second); fraction = second - whole
    instant = datetime(year, month, day, hour, minute, whole, tzinfo=timezone.utc)
    return round(((instant - GPS_EPOCH).total_seconds() + fraction) * 10)


def read_clock_csv(path: Path) -> dict[int, dict[str, float | int]]:
    rows: dict[int, dict[str, float | int]] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            week = int(row["gps_week"]); tow = float(row["gps_tow_s"])
            values: dict[str, float | int] = {
                "week": week, "tow_s": tow,
                "bias_s": float(row["receiver_clock_bias_s"]),
                "drift_sps": float(row["receiver_clock_drift_sps"]),
                "status": int(row["status"]), "satellites": int(row["satellites"]),
                "pdop": float(row["pdop"]),
            }
            if not all(math.isfinite(float(value)) for value in values.values()):
                raise ValueError("non-finite SPP clock telemetry")
            key = gps_key(week, tow)
            if key in rows:
                raise ValueError("duplicate SPP clock epoch")
            rows[key] = values
    if not rows:
        raise ValueError("empty SPP clock telemetry")
    return rows


def read_reference_clk(path: Path, station: str) -> dict[int, dict[str, float]]:
    rows: dict[int, dict[str, float]] = {}
    prefix = f"AR {station.upper()} "
    with path.open(encoding="ascii", errors="strict") as handle:
        for line in handle:
            if not line.startswith(prefix):
                continue
            parts = line.split()
            if len(parts) < 10:
                raise ValueError("malformed station row in IGS CLK")
            year, month, day, hour, minute = (int(parts[index]) for index in range(2, 7))
            second = float(parts[7]); count = int(parts[8]); bias = float(parts[9])
            sigma = float(parts[10]) if count >= 2 and len(parts) > 10 else math.nan
            key = calendar_gps_key(year, month, day, hour, minute, second)
            if key in rows or not math.isfinite(bias):
                raise ValueError("duplicate or non-finite station clock reference")
            rows[key] = {"bias_s": bias, "sigma_s": sigma}
    if not rows:
        raise ValueError(f"station {station} absent from independent IGS CLK")
    return rows


def percentile(values: list[float], q: float) -> float | None:
    return float(np.percentile(values, q)) if values else None


def analyse(clock: dict[int, dict[str, float | int]], reference: dict[int, dict[str, float]]) -> dict[str, Any]:
    clock_start, clock_end = min(clock), max(clock)
    eligible_reference_keys = sorted(key for key in reference if clock_start <= key <= clock_end)
    matched_keys = sorted(set(clock) & set(eligible_reference_keys))
    if len(matched_keys) < 12:
        raise ValueError("fewer than 12 matched independent clock epochs")
    phase_errors_s = [float(clock[key]["bias_s"]) - reference[key]["bias_s"] for key in matched_keys]
    frequency_errors = [
        (phase_errors_s[index] - phase_errors_s[index - 1]) /
        ((matched_keys[index] - matched_keys[index - 1]) / 10.0)
        for index in range(1, len(matched_keys))
    ]
    steps_s = [abs(phase_errors_s[index] - phase_errors_s[index - 1]) for index in range(1, len(phase_errors_s))]
    holdover: dict[str, Any] = {}
    clock_keys = sorted(clock)
    for horizon in HORIZONS_S:
        errors = []
        for start_key in matched_keys:
            target_key = start_key + horizon * 10
            if target_key not in reference:
                continue
            history = [key for key in clock_keys if start_key - 1800 * 10 <= key <= start_key]
            if len(history) < 30:
                continue
            times = np.array([(key - start_key) / 10.0 for key in history], dtype=float)
            biases = np.array([float(clock[key]["bias_s"]) for key in history], dtype=float)
            slope, intercept = np.polyfit(times, biases, 1)
            prediction = float(intercept + slope * horizon)
            errors.append((prediction - reference[target_key]["bias_s"]) * 1e9)
        absolute = [abs(value) for value in errors]
        holdover[str(horizon)] = {
            "episodes": len(errors), "mean_error_ns": statistics.fmean(errors) if errors else None,
            "p95_abs_error_ns": percentile(absolute, 95), "max_abs_error_ns": max(absolute) if absolute else None,
            "model": "linear fit over preceding 1800 s of GNSS-disciplined SPP clock bias",
        }
    return {
        "reference_epochs": len(eligible_reference_keys), "matched_lock_epochs": len(matched_keys),
        "lock_coverage_pct": 100.0 * len(matched_keys) / len(eligible_reference_keys),
        "lock_phase_error": {
            "mean_ns": statistics.fmean(phase_errors_s) * 1e9,
            "median_ns": statistics.median(phase_errors_s) * 1e9,
            "p95_abs_ns": percentile([abs(value) * 1e9 for value in phase_errors_s], 95),
            "max_abs_ns": max(abs(value) * 1e9 for value in phase_errors_s),
        },
        "relative_frequency_error": {
            "samples": len(frequency_errors), "mean": statistics.fmean(frequency_errors),
            "p95_abs": percentile([abs(value) for value in frequency_errors], 95),
            "max_abs": max(abs(value) for value in frequency_errors),
        },
        "maximum_lock_step_ns": max(steps_s) * 1e9,
        "simulated_holdover": holdover,
        "matched_rows": [
            {"gps_time_key": key, "spp_bias_s": float(clock[key]["bias_s"]),
             "igs_bias_s": reference[key]["bias_s"], "phase_error_ns": error * 1e9}
            for key, error in zip(matched_keys, phase_errors_s)
        ],
    }


def build_candidate_profile(result: dict[str, Any]) -> dict[str, Any]:
    holdover_limits = {
        horizon: math.ceil(float(metrics["p95_abs_error_ns"]) * 1.5)
        for horizon, metrics in result["simulated_holdover"].items()
    }
    return {
        "schema_version": "libgnsspp.timing_holdover.profile.v1", "release_state": "candidate",
        "minimum_lock_coverage_pct": 95.0,
        "maximum_lock_phase_p95_abs_ns": math.ceil(float(result["lock_phase_error"]["p95_abs_ns"]) * 1.5),
        "maximum_lock_step_ns": math.ceil(float(result["maximum_lock_step_ns"]) * 1.5),
        "maximum_holdover_p95_abs_ns": holdover_limits,
        "maximum_holdover_horizon_s": max(HORIZONS_S),
        "service_decision": "no_go_missing_physical_pps_and_real_outage_test",
        "claim_boundary": "Offline SPP-vs-IGS-CLK and simulated linear holdover only; no physical PPS/NTP/PTP service evidence.",
    }


def enforce(result: dict[str, Any], profile: dict[str, Any]) -> list[str]:
    failures = []
    if result["lock_coverage_pct"] < float(profile["minimum_lock_coverage_pct"]):
        failures.append("lock_reference_coverage_below_minimum")
    if result["lock_phase_error"]["p95_abs_ns"] > float(profile["maximum_lock_phase_p95_abs_ns"]):
        failures.append("lock_phase_p95_above_maximum")
    if result["maximum_lock_step_ns"] > float(profile["maximum_lock_step_ns"]):
        failures.append("lock_step_above_maximum")
    for horizon, limit in profile["maximum_holdover_p95_abs_ns"].items():
        metrics = result["simulated_holdover"].get(horizon)
        if not metrics or metrics["p95_abs_error_ns"] is None:
            failures.append(f"holdover_{horizon}s_population_missing")
        elif metrics["p95_abs_error_ns"] > float(limit):
            failures.append(f"holdover_{horizon}s_p95_above_maximum")
    return failures


def write_artifacts(output: Path, result: dict[str, Any]) -> None:
    output.mkdir(parents=True, exist_ok=True)
    with (output / "clock_comparison.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(result["matched_rows"][0]))
        writer.writeheader(); writer.writerows(result["matched_rows"])
    rows = result["matched_rows"]
    fig, ax = plt.subplots(figsize=(10, 3.5))
    ax.plot([row["phase_error_ns"] for row in rows], linewidth=0.8)
    ax.set_xlabel("IGS CLK matched epoch"); ax.set_ylabel("SPP - IGS clock (ns)")
    ax.grid(True, alpha=0.25); fig.tight_layout(); fig.savefig(output / "clock_scorecard.png", dpi=150); plt.close(fig)


def run(args: argparse.Namespace) -> int:
    output = args.output_dir.resolve()
    try:
        clock = read_clock_csv(args.clock_csv)
        reference = read_reference_clk(args.reference_clk, args.station)
        result = analyse(clock, reference)
        if args.role == "development":
            profile = build_candidate_profile(result)
        else:
            if args.profile is None:
                raise ValueError("sealed_holdout requires --profile")
            profile = json.loads(args.profile.read_text(encoding="utf-8"))
            if profile.get("release_state") != "sealed":
                raise ValueError("holdout profile is not sealed")
        failures = enforce(result, profile)
        write_artifacts(output, result)
        result.pop("matched_rows")
        payload = {
            "schema_version": SCHEMA_VERSION, "role": args.role,
            "status": "passed" if not failures else "failed",
            "assessment_decision": "usable" if not failures else "degraded",
            "service_decision": "no_go",
            "failures": failures, "metrics": result, "profile": profile,
            "inputs": {"clock_csv": record(args.clock_csv), "independent_igs_clk": record(args.reference_clk)},
            "observability": {
                "lock": "SPP code-derived receiver clock compared with independent IGS final station clock",
                "holdover": "simulated open-loop extrapolation; receiver remained GNSS-connected in source data",
                "unobserved": ["physical PPS edge", "system-clock/NTP/PTP offset", "real oscillator outage behavior", "temperature and power transients"],
            },
        }
    except Exception as exc:
        payload = {
            "schema_version": SCHEMA_VERSION, "role": args.role, "status": "failed",
            "assessment_decision": "unusable", "service_decision": "no_go", "failures": [str(exc)],
        }
    output.mkdir(parents=True, exist_ok=True)
    (output / "summary.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    artifacts = {}
    for name in ("summary.json", "clock_comparison.csv", "clock_scorecard.png"):
        path = output / name
        artifacts[name] = record(path) if path.is_file() else {"path": str(path), "exists": False}
    (output / "manifest.json").write_text(json.dumps({"schema_version": SCHEMA_VERSION, "status": payload["status"], "artifacts": artifacts}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Timing/holdover assessment: {payload['assessment_decision']} (service: No-Go)")
    print(f"Summary: {output / 'summary.json'}")
    return 0 if payload["status"] == "passed" else 1


def main(argv: list[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
