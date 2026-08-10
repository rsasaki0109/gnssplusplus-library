#!/usr/bin/env python3
"""Build an offline, machine-readable ledger of contiguous PPC wrong-FIX events.

Reference truth labels events only after solution generation. Runtime fingerprints
are derived exclusively from POS telemetry and optional ``--debug-log`` CSVs.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable

ANALYSIS_DIR = Path(__file__).resolve().parent / "analysis"
if str(ANALYSIS_DIR) not in sys.path:
    sys.path.insert(0, str(ANALYSIS_DIR))

import analyze_ppc_wrong_fix_residuals as audit


RUN_PATHS = dict(audit.RUNS)
DEBUG_NUMERIC_FIELDS = (
    "full_ratio",
    "selected_ratio",
    "selected_pair_count",
    "selected_fixed_ambiguities",
    "prior_held_integer_count",
    "prior_held_pair_count",
    "prior_consecutive_fix_count",
    "prior_tracked_ambiguity_count",
    "float_update_prefit_residual_rms_m",
    "float_update_post_suppression_residual_rms_m",
    "float_update_nis_per_observation",
    "float_update_suppressed_outliers",
    "float_position_covariance_trace_m2",
    "fixed_candidate_doppler_consensus_distance_m",
)


@dataclass(frozen=True)
class LabeledEpoch:
    solution: audit.SolutionEpoch
    error_m: float


def parse_keyed_paths(specs: Iterable[str], option: str) -> dict[str, Path]:
    paths: dict[str, Path] = {}
    for spec in specs:
        key, separator, value = spec.partition("=")
        if not separator or key not in RUN_PATHS or key in paths or not value:
            raise SystemExit(f"invalid or duplicate {option} value: {spec!r}")
        paths[key] = Path(value)
    return paths


def optional_float(value: object) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def optional_int(value: object) -> int | None:
    parsed = optional_float(value)
    return int(parsed) if parsed is not None else None


def load_debug_log(path: Path) -> dict[tuple[int, float], dict[str, str]]:
    rows: dict[tuple[int, float], dict[str, str]] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            week = optional_int(row.get("gps_week"))
            tow = optional_float(row.get("tow", row.get("tow_s")))
            if week is not None and tow is not None:
                rows[(week, round(tow, 3))] = row
    return rows


def split_events(
    labeled: list[LabeledEpoch], max_gap_s: float = 0.25
) -> list[list[LabeledEpoch]]:
    events: list[list[LabeledEpoch]] = []
    current: list[LabeledEpoch] = []
    for row in sorted(labeled, key=lambda item: (item.solution.week, item.solution.tow_s)):
        if current:
            previous = current[-1].solution
            epoch = row.solution
            if epoch.week != previous.week or epoch.tow_s - previous.tow_s > max_gap_s:
                events.append(current)
                current = []
        current.append(row)
    if current:
        events.append(current)
    return events


def finite_values(values: Iterable[object]) -> list[float]:
    return [parsed for value in values if (parsed := optional_float(value)) is not None]


def numeric_summary(values: Iterable[object]) -> dict[str, float | None]:
    finite = finite_values(values)
    return {
        "min": round(min(finite), 6) if finite else None,
        "p50": audit.percentile(finite, 50.0),
        "max": round(max(finite), 6) if finite else None,
    }


def event_fingerprints(
    event: list[LabeledEpoch], debug_rows: list[dict[str, str]]
) -> list[str]:
    errors = [row.error_m for row in event]
    prefit = finite_values(row.solution.prefit_rms_m for row in event)
    outliers = finite_values(row.solution.outliers for row in event)
    ratios = finite_values(row.solution.ratio for row in event)
    fingerprints: list[str] = []
    if max(errors) > 50.0:
        fingerprints.append("catastrophic_gt50m")
    if prefit and float(audit.percentile(prefit, 50.0) or 0.0) >= 8.0:
        fingerprints.append("high_prefit_basin")
    if outliers and max(outliers) >= 35.0:
        fingerprints.append("outlier_suppression_storm")
    if ratios and float(audit.percentile(ratios, 50.0) or math.inf) <= 5.0:
        fingerprints.append("low_ar_margin")
    if any(optional_int(row.get("selected_used_subset")) == 1 for row in debug_rows):
        fingerprints.append("subset_ar")
    if any((optional_int(row.get("prior_held_integer_count")) or 0) > 0 for row in debug_rows):
        fingerprints.append("inherited_held_integers")
    references = {
        row.get("selected_reference_satellites", "")
        for row in debug_rows
        if row.get("selected_reference_satellites")
    }
    if len(references) > 1:
        fingerprints.append("reference_satellite_change")
    return fingerprints or ["unclassified"]


def build_event_payload(
    run_key: str,
    index: int,
    event: list[LabeledEpoch],
    context: list[audit.SolutionEpoch],
    debug: dict[tuple[int, float], dict[str, str]],
) -> dict[str, Any]:
    epochs = [row.solution for row in event]
    errors = [row.error_m for row in event]
    start = epochs[0].tow_s
    end = epochs[-1].tow_s
    debug_rows = [
        debug[(epoch.week, epoch.tow_s)]
        for epoch in context
        if (epoch.week, epoch.tow_s) in debug
    ]
    event_debug_rows = [
        debug[(epoch.week, epoch.tow_s)]
        for epoch in epochs
        if (epoch.week, epoch.tow_s) in debug
    ]
    references = sorted(
        {
            row.get("selected_reference_satellites", "")
            for row in debug_rows
            if row.get("selected_reference_satellites")
        }
    )
    return {
        "event_id": f"{run_key}-{epochs[0].week}-{start:.3f}-{index:03d}",
        "run": run_key,
        "gps_week": epochs[0].week,
        "start_tow_s": start,
        "end_tow_s": end,
        "duration_s": round(end - start + 0.2, 3),
        "epochs": len(event),
        "max_error_m": round(max(errors), 6),
        "p95_error_m": audit.percentile(errors, 95.0),
        "severity": audit.severity_counts(errors),
        "fingerprints": event_fingerprints(event, event_debug_rows),
        "pos_telemetry": {
            "satellites": numeric_summary(epoch.nsat for epoch in epochs),
            "ratio": numeric_summary(epoch.ratio for epoch in epochs),
            "prefit_rms_m": numeric_summary(epoch.prefit_rms_m for epoch in epochs),
            "prefit_max_m": numeric_summary(epoch.prefit_max_m for epoch in epochs),
            "suppressed_outliers": numeric_summary(epoch.outliers for epoch in epochs),
            "post_rms_m": numeric_summary(epoch.post_rms_m for epoch in epochs),
            "nis_per_observation": numeric_summary(epoch.nis_per_obs for epoch in epochs),
        },
        "debug_context_available": bool(debug_rows),
        "debug_event_epochs": len(event_debug_rows),
        "debug_context": {
            field: numeric_summary(row.get(field) for row in debug_rows)
            for field in DEBUG_NUMERIC_FIELDS
        },
        "selected_reference_satellites": references,
        "epoch_errors": [
            {"tow_s": row.solution.tow_s, "error_m": round(row.error_m, 6)}
            for row in event
        ],
    }


def build_ledger(
    dataset_root: Path,
    positions: dict[str, Path],
    debug_logs: dict[str, Path],
    wrong_fix_threshold_m: float,
    context_before_s: float,
    max_gap_s: float,
) -> dict[str, Any]:
    all_events: list[dict[str, Any]] = []
    runs: list[dict[str, Any]] = []
    for run_key, pos_path in positions.items():
        solution = audit.load_solution(pos_path)
        reference = audit.load_reference(dataset_root / RUN_PATHS[run_key] / "reference.csv")
        labeled = [
            LabeledEpoch(epoch, audit.error_3d_m(epoch.ecef, reference[(epoch.week, epoch.tow_s)]))
            for epoch in solution
            if epoch.status == 4
            and (epoch.week, epoch.tow_s) in reference
            and audit.error_3d_m(epoch.ecef, reference[(epoch.week, epoch.tow_s)])
            > wrong_fix_threshold_m
        ]
        debug = load_debug_log(debug_logs[run_key]) if run_key in debug_logs else {}
        run_events = []
        for index, event in enumerate(split_events(labeled, max_gap_s), start=1):
            start = event[0].solution.tow_s
            end = event[-1].solution.tow_s
            context = [
                epoch
                for epoch in solution
                if epoch.week == event[0].solution.week
                and start - context_before_s <= epoch.tow_s <= end
            ]
            run_events.append(build_event_payload(run_key, index, event, context, debug))
        all_events.extend(run_events)
        runs.append(
            {
                "run": run_key,
                "pos": str(pos_path),
                "debug_log": str(debug_logs[run_key]) if run_key in debug_logs else None,
                "wrong_fix_epochs": len(labeled),
                "events": len(run_events),
                "events_above_10m": sum(
                    event["severity"]["above_10m"] > 0 for event in run_events
                ),
            }
        )
    all_events.sort(key=lambda event: float(event["max_error_m"]), reverse=True)
    return {
        "schema_version": 1,
        "reference_truth_role": "offline_event_labeling_only",
        "runtime_truth_used": False,
        "wrong_fix_threshold_m": wrong_fix_threshold_m,
        "event_max_gap_s": max_gap_s,
        "debug_context_before_s": context_before_s,
        "runs": runs,
        "summary": {
            "wrong_fix_epochs": sum(run["wrong_fix_epochs"] for run in runs),
            "events": len(all_events),
            "events_above_10m": sum(
                event["severity"]["above_10m"] > 0 for event in all_events
            ),
        },
        "events": all_events,
    }


def render_markdown(payload: dict[str, Any]) -> str:
    summary = payload["summary"]
    lines = [
        "# PPC wrong-FIX event ledger",
        "",
        "Reference truth labels events offline only; fingerprints use runtime POS/debug telemetry.",
        "",
        f"Wrong FIX: **{summary['wrong_fix_epochs']} epochs / {summary['events']} events**; "
        f"events containing >10 m error: **{summary['events_above_10m']}**.",
        "",
        "| Event | Run | TOW span | Epochs | Max error | >10 m | Fingerprints | Debug |",
        "|---|---|---:|---:|---:|---:|---|---|",
    ]
    for event in payload["events"]:
        lines.append(
            f"| `{event['event_id']}` | {event['run']} | "
            f"{event['start_tow_s']:.3f}-{event['end_tow_s']:.3f} | {event['epochs']} | "
            f"{event['max_error_m']:.3f} m | {event['severity']['above_10m']} | "
            f"{', '.join(event['fingerprints'])} | "
            f"{'yes' if event['debug_context_available'] else 'no'} |"
        )
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--pos", action="append", required=True, metavar="RUN_KEY=PATH")
    parser.add_argument("--debug-log", action="append", default=[], metavar="RUN_KEY=CSV")
    parser.add_argument("--wrong-fix-threshold-m", type=float, default=0.5)
    parser.add_argument("--context-before-s", type=float, default=10.0)
    parser.add_argument("--max-gap-s", type=float, default=0.25)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--markdown-output", type=Path, required=True)
    args = parser.parse_args()
    if args.wrong_fix_threshold_m < 0.0 or args.context_before_s < 0.0 or args.max_gap_s <= 0.0:
        raise SystemExit("threshold/context must be non-negative and max gap must be positive")
    return args


def main() -> int:
    args = parse_args()
    positions = parse_keyed_paths(args.pos, "--pos")
    debug_logs = parse_keyed_paths(args.debug_log, "--debug-log")
    if not set(debug_logs).issubset(positions):
        raise SystemExit("--debug-log run keys must also be present in --pos")
    payload = build_ledger(
        args.dataset_root,
        positions,
        debug_logs,
        args.wrong_fix_threshold_m,
        args.context_before_s,
        args.max_gap_s,
    )
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    args.markdown_output.write_text(render_markdown(payload), encoding="utf-8")
    print(f"wrote {args.output_json}")
    print(f"wrote {args.markdown_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
