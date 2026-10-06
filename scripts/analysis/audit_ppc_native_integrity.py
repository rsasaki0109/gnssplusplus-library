#!/usr/bin/env python3
"""Offline wrong-FIX labels and runtime-feature classes for native PPC replays.

Reference coordinates are consumed only here, after solver outputs exist.
Runtime feature classes are descriptive witnesses, not integrity guarantees.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_ppc_wrong_fix_residuals import load_reference, load_solution, error_3d_m
from compare_fixed_lag_covariance import native_input_epochs, quantile

RUNS = [f"{city}/run{number}" for city in ("tokyo", "nagoya") for number in (1, 2, 3)]


def runtime_classes(epoch) -> list[str]:
    """Only source output diagnostics; never a truth coordinate or label."""
    labels = []
    if epoch.nsat is not None and epoch.nsat < 8:
        labels.append("low_satellites")
    if epoch.ratio is not None and epoch.ratio < 6.0:
        labels.append("low_ratio")
    if epoch.prefit_rms_m is not None and epoch.prefit_rms_m > 10.0:
        labels.append("large_prefit")
    if epoch.post_rms_m is not None and epoch.post_rms_m > 4.0:
        labels.append("large_post_suppression_residual")
    if epoch.nis_per_obs is not None and epoch.nis_per_obs > 20.0:
        labels.append("large_update_nis")
    if epoch.observations and epoch.outliers is not None and epoch.outliers / epoch.observations > 0.3:
        labels.append("heavy_outlier_suppression")
    if any(value is None for value in (epoch.nsat, epoch.ratio, epoch.prefit_rms_m,
                                       epoch.post_rms_m, epoch.nis_per_obs, epoch.outliers, epoch.observations)):
        labels.append("incomplete_runtime_diagnostics")
    return labels or ["no_named_runtime_warning"]


def labeled_rows(epochs, reference):
    rows = []
    seen = set()
    previous = -math.inf
    for epoch in epochs:
        stamp = epoch.week * 604800.0 + epoch.tow_s
        key = (epoch.week, epoch.tow_s)
        if key in seen or stamp <= previous:
            raise ValueError("solution epoch keys must be unique and increasing")
        if not all(math.isfinite(v) for v in epoch.ecef):
            raise ValueError("nonfinite position in POS file")
        seen.add(key)
        previous = stamp
        truth = reference.get(key)
        if truth is None:
            continue
        error = error_3d_m(epoch.ecef, truth)
        rows.append({"key": key, "stamp": stamp, "fixed": epoch.status == 4,
                     "error_m": error,
                     "runtime_classes": runtime_classes(epoch)})
    return rows


def recovery_events(rows, threshold):
    cadence = statistics.median(b["stamp"] - a["stamp"] for a, b in zip(rows, rows[1:])) if len(rows) > 1 else 0.2
    wrong = [row["fixed"] and row["error_m"] > threshold for row in rows]
    events = []
    begin = None
    for index, is_wrong in enumerate(wrong):
        if is_wrong and begin is None:
            begin = index
        end = is_wrong and (index == len(rows) - 1 or not wrong[index + 1] or
                           rows[index + 1]["stamp"] - rows[index]["stamp"] > 1.5 * cadence)
        if end:
            recovered = next((row for row in rows[index + 1:] if row["fixed"] and row["error_m"] <= threshold), None)
            span = rows[begin:index + 1]
            events.append({"start": list(span[0]["key"]), "end": list(span[-1]["key"]),
                           "epochs": len(span), "elapsed_s": span[-1]["stamp"] - span[0]["stamp"],
                           "max_error_m": max(row["error_m"] for row in span),
                           "runtime_classes": dict(Counter(label for row in span for label in row["runtime_classes"])),
                           "recovery_delay_s": recovered["stamp"] - rows[index]["stamp"] if recovered else None,
                           "right_censored": recovered is None})
            begin = None
    return events


def summarize(rows, input_epochs):
    if len(rows) > input_epochs:
        raise ValueError("matched solutions exceed admitted rover inputs")
    result = {"admitted_rover_epochs": input_epochs, "exact_reference_matched_epochs": len(rows),
              "missing_or_unmatched_epochs": input_epochs - len(rows),
              "fixed_epochs": sum(row["fixed"] for row in rows),
              "error_3d_p95_m": quantile([row["error_m"] for row in rows], 0.95),
              "thresholds": {}}
    for threshold in (0.5, 2.0):
        wrong = [row for row in rows if row["fixed"] and row["error_m"] > threshold]
        correct = [row for row in rows if row["fixed"] and row["error_m"] <= threshold]
        events = recovery_events(rows, threshold)
        delays = [event["recovery_delay_s"] for event in events if not event["right_censored"]]
        result["thresholds"][str(threshold)] = {
            "wrong_fixed_epochs": len(wrong), "correct_fixed_epochs": len(correct),
            "wrong_fixed_runtime_classes": dict(Counter(label for row in wrong for label in row["runtime_classes"])),
            "correct_fixed_runtime_classes": dict(Counter(label for row in correct for label in row["runtime_classes"])),
            "wrong_fixed_events": len(events), "right_censored_events": sum(event["right_censored"] for event in events),
            "recovery_delay_p50_s": quantile(delays, 0.5), "recovery_delay_p95_s": quantile(delays, 0.95),
            "events": events}
    return result


def compare(baseline_rows, candidate_rows):
    candidate = {row["key"]: row for row in candidate_rows}
    result = {}
    for threshold in (0.5, 2.0):
        correct = [row for row in baseline_rows if row["fixed"] and row["error_m"] <= threshold]
        wrong = [row for row in baseline_rows if row["fixed"] and row["error_m"] > threshold]
        result[str(threshold)] = {
            "baseline_correct_fixed_lost": sum(row["key"] not in candidate or not candidate[row["key"]]["fixed"] or
                                                candidate[row["key"]]["error_m"] > threshold for row in correct),
            "baseline_wrong_fixed_removed": sum(row["key"] not in candidate or not candidate[row["key"]]["fixed"] or
                                                 candidate[row["key"]]["error_m"] <= threshold for row in wrong),
            "baseline_wrong_epochs_now_missing": sum(row["key"] not in candidate for row in wrong),
            "baseline_wrong_epochs_now_accurate": sum(row["key"] in candidate and
                                                       candidate[row["key"]]["error_m"] <= threshold for row in wrong)}
    return result


def file_record(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1048576), b""):
            digest.update(block)
    return {"path": str(path.resolve()), "sha256": digest.hexdigest(), "bytes": path.stat().st_size}


def audit(args):
    result = {"schema": "ppc_native_integrity_audit.v1", "evaluation_runs": args.runs,
              "reference_role": "offline labels only; no detector input", "runs": {},
              "matching": "exact GPS week and millisecond-rounded TOW; unmatched outputs excluded",
              "recovery_contract": "time after last wrong FIX until next threshold-correct FIX, including intervening missing/float epochs",
              "horizontal_contract": "official geodetic horizontal P95 retained from replay scorer",
              "input_artifacts": []}
    for run in args.runs:
        reference = load_reference(args.dataset_root / run / "reference.csv")
        directory = args.replay_dir / run
        baseline = labeled_rows(load_solution(directory / "rtk.pos"), reference)
        expected = native_input_epochs((directory / "rtk.log").read_text(encoding="utf-8"), "rtk")
        summary = json.loads((directory / "rtk_summary.json").read_text(encoding="utf-8"))
        result["input_artifacts"].extend(file_record(path) for path in
            (args.dataset_root / run / "reference.csv", directory / "rtk.pos", directory / "rtk.log", directory / "rtk_summary.json"))
        report = summarize(baseline, expected)
        report.update(native_wall_s=summary["solver_wall_time_s"], official_geodetic_horizontal_p95_m=summary["p95_h_m"])
        if args.candidate_dir:
            candidate_dir = args.candidate_dir / run
            candidate = labeled_rows(load_solution(candidate_dir / "rtk.pos"), reference)
            candidate_expected = native_input_epochs((candidate_dir / "rtk.log").read_text(encoding="utf-8"), "rtk")
            if candidate_expected != expected:
                raise ValueError(f"admitted input populations differ for {run}")
            report["candidate"] = summarize(candidate, expected)
            candidate_summary = json.loads((candidate_dir / "rtk_summary.json").read_text(encoding="utf-8"))
            report["candidate"].update(native_wall_s=candidate_summary["solver_wall_time_s"],
                official_geodetic_horizontal_p95_m=candidate_summary["p95_h_m"])
            result["input_artifacts"].extend(file_record(path) for path in
                (candidate_dir / "rtk.pos", candidate_dir / "rtk.log", candidate_dir / "rtk_summary.json"))
            report["comparison"] = compare(baseline, candidate)
        result["runs"][run] = report
    for directory, name in ((args.replay_dir, "baseline"), (args.candidate_dir, "candidate")):
        if directory is not None and (directory / "manifest.json").is_file():
            manifest_path = directory / "manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            result[name + "_replay_manifest"] = {**file_record(manifest_path), "state": manifest.get("state")}
    for record in result["input_artifacts"]:
        if file_record(Path(record["path"])) != record:
            raise ValueError("audit input changed while labeling")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--replay-dir", type=Path, required=True)
    parser.add_argument("--candidate-dir", type=Path)
    parser.add_argument("--runs", nargs="+", choices=RUNS, default=RUNS)
    parser.add_argument("--output-json", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    for run, report in result["runs"].items():
        print(run, "wrong FIX:", {key: value["wrong_fixed_epochs"] for key, value in report["thresholds"].items()},
              "missing/unmatched:", report["missing_or_unmatched_epochs"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
