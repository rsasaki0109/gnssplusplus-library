#!/usr/bin/env python3
"""Compare PPC official-score segments between RTK profile candidates."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import sys


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = Path(__file__).resolve().parent
APPS_DIR = ROOT_DIR / "apps"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
if str(APPS_DIR) not in sys.path:
    sys.path.insert(0, str(APPS_DIR))

import generate_driving_comparison as comparison  # noqa: E402
import gnss_ppc_metrics as ppc_metrics  # noqa: E402


STATUS_NAMES = {
    4: "FIXED",
    3: "FLOAT",
    2: "DGPS",
    1: "SPP",
}
SEGMENT_FIELDNAMES = [
    "candidate_label",
    "reference_index",
    "start_tow_s",
    "end_tow_s",
    "segment_distance_m",
    "score_delta_distance_m",
    "baseline_score_distance_m",
    "candidate_score_distance_m",
    "score_transition",
    "baseline_score_state",
    "candidate_score_state",
    "baseline_scored",
    "candidate_scored",
    "status_transition",
    "baseline_status",
    "baseline_status_name",
    "candidate_status",
    "candidate_status_name",
    "baseline_error_3d_m",
    "candidate_error_3d_m",
    "error_delta_3d_m",
    "baseline_horiz_error_m",
    "candidate_horiz_error_m",
    "baseline_up_error_m",
    "candidate_up_error_m",
    "baseline_num_satellites",
    "candidate_num_satellites",
    "baseline_ratio",
    "candidate_ratio",
    "baseline_baseline_m",
    "candidate_baseline_m",
    "baseline_rtk_iterations",
    "candidate_rtk_iterations",
    "baseline_rtk_update_observations",
    "candidate_rtk_update_observations",
    "baseline_rtk_update_phase_observations",
    "candidate_rtk_update_phase_observations",
    "baseline_rtk_update_code_observations",
    "candidate_rtk_update_code_observations",
    "baseline_rtk_update_suppressed_outliers",
    "candidate_rtk_update_suppressed_outliers",
    "baseline_rtk_update_prefit_residual_rms_m",
    "candidate_rtk_update_prefit_residual_rms_m",
    "baseline_rtk_update_prefit_residual_max_m",
    "candidate_rtk_update_prefit_residual_max_m",
    "baseline_rtk_update_post_suppression_residual_rms_m",
    "candidate_rtk_update_post_suppression_residual_rms_m",
    "baseline_rtk_update_post_suppression_residual_max_m",
    "candidate_rtk_update_post_suppression_residual_max_m",
    "baseline_rtk_update_normalized_innovation_squared",
    "candidate_rtk_update_normalized_innovation_squared",
    "baseline_rtk_update_normalized_innovation_squared_per_observation",
    "candidate_rtk_update_normalized_innovation_squared_per_observation",
    "baseline_rtk_update_rejected_by_innovation_gate",
    "candidate_rtk_update_rejected_by_innovation_gate",
]
DIAGNOSTIC_KEYS = (
    "ratio",
    "error_3d_m",
    "horiz_error_m",
    "rtk_update_observations",
    "rtk_update_prefit_residual_rms_m",
    "rtk_update_prefit_residual_max_m",
    "rtk_update_post_suppression_residual_rms_m",
    "rtk_update_post_suppression_residual_max_m",
    "rtk_update_normalized_innovation_squared",
    "rtk_update_normalized_innovation_squared_per_observation",
    "rtk_update_rejected_by_innovation_gate",
)


@dataclass(frozen=True)
class CandidateSpec:
    label: str
    pos_path: Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--reference-csv",
        type=Path,
        required=True,
        help="PPC reference.csv used for official segment scoring.",
    )
    parser.add_argument(
        "--baseline-pos",
        type=Path,
        required=True,
        help="Baseline gnssplusplus .pos file.",
    )
    parser.add_argument(
        "--baseline-label",
        default="baseline",
        help="Baseline label used in reports (default: baseline).",
    )
    parser.add_argument(
        "--candidate",
        action="append",
        default=[],
        metavar="LABEL=POS",
        help="Candidate gnssplusplus .pos file. Repeat for multiple profiles.",
    )
    parser.add_argument(
        "--match-tolerance-s",
        type=float,
        default=0.25,
        help="Solution/reference time match tolerance in seconds (default: 0.25).",
    )
    parser.add_argument(
        "--threshold-m",
        type=float,
        default=0.50,
        help="PPC official 3D scoring threshold in meters (default: 0.50).",
    )
    parser.add_argument(
        "--top-segments",
        type=int,
        default=12,
        help="Number of gain/loss segments to include in summary outputs (default: 12).",
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=None,
        help="Optional output JSON path.",
    )
    parser.add_argument(
        "--markdown-output",
        type=Path,
        default=None,
        help="Optional Markdown report path.",
    )
    parser.add_argument(
        "--segments-csv",
        type=Path,
        default=None,
        help="Optional CSV path for all changed official segments.",
    )
    parser.add_argument(
        "--write-all-segments",
        action="store_true",
        help=(
            "When --segments-csv is set, write every official segment instead of only "
            "segments whose score delta changed."
        ),
    )
    return parser.parse_args()


def parse_candidate(value: str) -> CandidateSpec:
    label, sep, path_text = value.partition("=")
    if not sep or not label.strip() or not path_text.strip():
        raise SystemExit("--candidate must use LABEL=POS")
    return CandidateSpec(label=label.strip(), pos_path=Path(path_text.strip()))


def rounded(value: float | None) -> float | None:
    return None if value is None else round(float(value), 6)


def status_name(status: object) -> str:
    if status is None:
        return "NO_SOLUTION"
    return STATUS_NAMES.get(int(status), f"status_{int(status)}")


def optional_float(record: dict[str, object], key: str) -> float | None:
    value = record.get(key)
    if value is None:
        return None
    numeric = float(value)
    return numeric if math.isfinite(numeric) else None


def optional_int(record: dict[str, object], key: str) -> int | None:
    value = record.get(key)
    return None if value is None else int(value)


def score_distance(record: dict[str, object]) -> float:
    return float(record["segment_distance_m"]) if bool(record["scored"]) else 0.0


def records_by_reference(records: list[dict[str, object]], label: str) -> dict[int, dict[str, object]]:
    by_index: dict[int, dict[str, object]] = {}
    for record in records:
        reference_index = int(record["reference_index"])
        if reference_index in by_index:
            raise SystemExit(f"{label}: duplicate reference index {reference_index}")
        by_index[reference_index] = record
    return by_index


def segment_row(
    baseline_record: dict[str, object],
    candidate_record: dict[str, object],
    candidate_label: str,
) -> dict[str, object]:
    baseline_score_distance = score_distance(baseline_record)
    candidate_score_distance = score_distance(candidate_record)
    baseline_status_name = status_name(baseline_record.get("status"))
    candidate_status_name = status_name(candidate_record.get("status"))
    baseline_error_3d = optional_float(baseline_record, "error_3d_m")
    candidate_error_3d = optional_float(candidate_record, "error_3d_m")
    return {
        "candidate_label": candidate_label,
        "reference_index": int(baseline_record["reference_index"]),
        "start_tow_s": rounded(float(baseline_record["start_tow_s"])),
        "end_tow_s": rounded(float(baseline_record["end_tow_s"])),
        "segment_distance_m": rounded(float(baseline_record["segment_distance_m"])),
        "score_delta_distance_m": rounded(candidate_score_distance - baseline_score_distance),
        "baseline_score_distance_m": rounded(baseline_score_distance),
        "candidate_score_distance_m": rounded(candidate_score_distance),
        "score_transition": (
            f"{baseline_record['score_state']}->{candidate_record['score_state']}"
        ),
        "baseline_score_state": baseline_record["score_state"],
        "candidate_score_state": candidate_record["score_state"],
        "baseline_scored": bool(baseline_record["scored"]),
        "candidate_scored": bool(candidate_record["scored"]),
        "status_transition": f"{baseline_status_name}->{candidate_status_name}",
        "baseline_status": optional_int(baseline_record, "status"),
        "baseline_status_name": baseline_status_name,
        "candidate_status": optional_int(candidate_record, "status"),
        "candidate_status_name": candidate_status_name,
        "baseline_error_3d_m": rounded(baseline_error_3d),
        "candidate_error_3d_m": rounded(candidate_error_3d),
        "error_delta_3d_m": rounded(
            candidate_error_3d - baseline_error_3d
            if candidate_error_3d is not None and baseline_error_3d is not None
            else None
        ),
        "baseline_horiz_error_m": rounded(optional_float(baseline_record, "horiz_error_m")),
        "candidate_horiz_error_m": rounded(optional_float(candidate_record, "horiz_error_m")),
        "baseline_up_error_m": rounded(optional_float(baseline_record, "up_error_m")),
        "candidate_up_error_m": rounded(optional_float(candidate_record, "up_error_m")),
        "baseline_num_satellites": optional_int(baseline_record, "num_satellites"),
        "candidate_num_satellites": optional_int(candidate_record, "num_satellites"),
        "baseline_ratio": rounded(optional_float(baseline_record, "ratio")),
        "candidate_ratio": rounded(optional_float(candidate_record, "ratio")),
        "baseline_baseline_m": rounded(optional_float(baseline_record, "baseline_m")),
        "candidate_baseline_m": rounded(optional_float(candidate_record, "baseline_m")),
        "baseline_rtk_iterations": optional_int(baseline_record, "rtk_iterations"),
        "candidate_rtk_iterations": optional_int(candidate_record, "rtk_iterations"),
        "baseline_rtk_update_observations": optional_int(
            baseline_record, "rtk_update_observations"
        ),
        "candidate_rtk_update_observations": optional_int(
            candidate_record, "rtk_update_observations"
        ),
        "baseline_rtk_update_phase_observations": optional_int(
            baseline_record, "rtk_update_phase_observations"
        ),
        "candidate_rtk_update_phase_observations": optional_int(
            candidate_record, "rtk_update_phase_observations"
        ),
        "baseline_rtk_update_code_observations": optional_int(
            baseline_record, "rtk_update_code_observations"
        ),
        "candidate_rtk_update_code_observations": optional_int(
            candidate_record, "rtk_update_code_observations"
        ),
        "baseline_rtk_update_suppressed_outliers": optional_int(
            baseline_record, "rtk_update_suppressed_outliers"
        ),
        "candidate_rtk_update_suppressed_outliers": optional_int(
            candidate_record, "rtk_update_suppressed_outliers"
        ),
        "baseline_rtk_update_prefit_residual_rms_m": rounded(
            optional_float(baseline_record, "rtk_update_prefit_residual_rms_m")
        ),
        "candidate_rtk_update_prefit_residual_rms_m": rounded(
            optional_float(candidate_record, "rtk_update_prefit_residual_rms_m")
        ),
        "baseline_rtk_update_prefit_residual_max_m": rounded(
            optional_float(baseline_record, "rtk_update_prefit_residual_max_m")
        ),
        "candidate_rtk_update_prefit_residual_max_m": rounded(
            optional_float(candidate_record, "rtk_update_prefit_residual_max_m")
        ),
        "baseline_rtk_update_post_suppression_residual_rms_m": rounded(
            optional_float(baseline_record, "rtk_update_post_suppression_residual_rms_m")
        ),
        "candidate_rtk_update_post_suppression_residual_rms_m": rounded(
            optional_float(candidate_record, "rtk_update_post_suppression_residual_rms_m")
        ),
        "baseline_rtk_update_post_suppression_residual_max_m": rounded(
            optional_float(baseline_record, "rtk_update_post_suppression_residual_max_m")
        ),
        "candidate_rtk_update_post_suppression_residual_max_m": rounded(
            optional_float(candidate_record, "rtk_update_post_suppression_residual_max_m")
        ),
        "baseline_rtk_update_normalized_innovation_squared": rounded(
            optional_float(baseline_record, "rtk_update_normalized_innovation_squared")
        ),
        "candidate_rtk_update_normalized_innovation_squared": rounded(
            optional_float(candidate_record, "rtk_update_normalized_innovation_squared")
        ),
        "baseline_rtk_update_normalized_innovation_squared_per_observation": rounded(
            optional_float(
                baseline_record,
                "rtk_update_normalized_innovation_squared_per_observation",
            )
        ),
        "candidate_rtk_update_normalized_innovation_squared_per_observation": rounded(
            optional_float(
                candidate_record,
                "rtk_update_normalized_innovation_squared_per_observation",
            )
        ),
        "baseline_rtk_update_rejected_by_innovation_gate": optional_int(
            baseline_record,
            "rtk_update_rejected_by_innovation_gate",
        ),
        "candidate_rtk_update_rejected_by_innovation_gate": optional_int(
            candidate_record,
            "rtk_update_rejected_by_innovation_gate",
        ),
    }


def median(values: list[float]) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    rank = (len(ordered) - 1) * pct / 100.0
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[int(rank)]
    return ordered[lower] * (upper - rank) + ordered[upper] * (rank - lower)


def diagnostic_summary(rows: list[dict[str, object]], prefix: str) -> dict[str, object]:
    summary: dict[str, object] = {
        "segments": len(rows),
        "distance_m": rounded(sum(float(row["segment_distance_m"]) for row in rows)),
    }
    for key in DIAGNOSTIC_KEYS:
        values = [
            float(row[f"{prefix}_{key}"])
            for row in rows
            if row.get(f"{prefix}_{key}") is not None
        ]
        summary[f"median_{key}"] = rounded(median(values))
        summary[f"p95_{key}"] = rounded(percentile(values, 95.0))
    return summary


def grouped_delta(rows: list[dict[str, object]], key: str) -> list[dict[str, object]]:
    grouped: dict[str, dict[str, object]] = {}
    for row in rows:
        name = str(row[key])
        bucket = grouped.setdefault(
            name,
            {
                key: name,
                "segments": 0,
                "distance_m": 0.0,
                "score_delta_distance_m": 0.0,
            },
        )
        bucket["segments"] = int(bucket["segments"]) + 1
        bucket["distance_m"] = float(bucket["distance_m"]) + float(row["segment_distance_m"])
        bucket["score_delta_distance_m"] = float(bucket["score_delta_distance_m"]) + float(
            row["score_delta_distance_m"]
        )
    result = list(grouped.values())
    for bucket in result:
        bucket["distance_m"] = rounded(float(bucket["distance_m"]))
        bucket["score_delta_distance_m"] = rounded(float(bucket["score_delta_distance_m"]))
    return sorted(
        result,
        key=lambda bucket: (
            -abs(float(bucket["score_delta_distance_m"])),
            str(bucket[key]),
        ),
    )


def compare_segment_records(
    baseline_records: list[dict[str, object]],
    candidate_records: list[dict[str, object]],
    candidate_label: str,
    top_segments: int = 12,
) -> dict[str, object]:
    baseline_by_index = records_by_reference(baseline_records, "baseline")
    candidate_by_index = records_by_reference(candidate_records, candidate_label)
    if set(baseline_by_index) != set(candidate_by_index):
        missing = sorted(set(baseline_by_index) - set(candidate_by_index))
        extra = sorted(set(candidate_by_index) - set(baseline_by_index))
        details: list[str] = []
        if missing:
            details.append("missing " + ", ".join(map(str, missing[:8])))
        if extra:
            details.append("extra " + ", ".join(map(str, extra[:8])))
        raise SystemExit(f"{candidate_label}: reference segment mismatch: {'; '.join(details)}")

    rows = [
        segment_row(
            baseline_by_index[reference_index],
            candidate_by_index[reference_index],
            candidate_label,
        )
        for reference_index in sorted(baseline_by_index)
    ]
    changed_rows = [row for row in rows if abs(float(row["score_delta_distance_m"])) > 1e-9]
    gain_rows = [row for row in changed_rows if float(row["score_delta_distance_m"]) > 0.0]
    loss_rows = [row for row in changed_rows if float(row["score_delta_distance_m"]) < 0.0]
    total_distance = sum(float(record["segment_distance_m"]) for record in baseline_records)
    if total_distance <= 0.0:
        raise SystemExit(f"{candidate_label}: non-positive official segment distance")
    baseline_score = sum(score_distance(record) for record in baseline_records)
    candidate_score = sum(score_distance(record) for record in candidate_records)
    net_delta = candidate_score - baseline_score

    return {
        "label": candidate_label,
        "segments": len(rows),
        "total_distance_m": rounded(total_distance),
        "baseline_score_distance_m": rounded(baseline_score),
        "candidate_score_distance_m": rounded(candidate_score),
        "baseline_official_score_pct": rounded(100.0 * baseline_score / total_distance),
        "candidate_official_score_pct": rounded(100.0 * candidate_score / total_distance),
        "delta_vs_baseline_score_distance_m": rounded(net_delta),
        "delta_vs_baseline_score_pct": rounded(100.0 * net_delta / total_distance),
        "changed_segments": len(changed_rows),
        "gain_segments": len(gain_rows),
        "loss_segments": len(loss_rows),
        "gain_distance_m": rounded(sum(float(row["score_delta_distance_m"]) for row in gain_rows)),
        "loss_distance_m": rounded(sum(float(row["score_delta_distance_m"]) for row in loss_rows)),
        "score_transitions": grouped_delta(changed_rows, "score_transition"),
        "status_transitions": grouped_delta(changed_rows, "status_transition"),
        "candidate_gain_diagnostics": diagnostic_summary(gain_rows, "candidate"),
        "candidate_loss_diagnostics": diagnostic_summary(loss_rows, "candidate"),
        "top_gains": sorted(
            gain_rows,
            key=lambda row: float(row["score_delta_distance_m"]),
            reverse=True,
        )[:top_segments],
        "top_losses": sorted(loss_rows, key=lambda row: float(row["score_delta_distance_m"]))[
            :top_segments
        ],
        "_all_rows": rows,
        "_changed_rows": changed_rows,
    }


def load_official_records(
    reference_csv: Path,
    pos_path: Path,
    match_tolerance_s: float,
    threshold_m: float,
    label: str,
) -> list[dict[str, object]]:
    if not reference_csv.exists():
        raise SystemExit(f"Missing reference CSV: {reference_csv}")
    if not pos_path.exists():
        raise SystemExit(f"{label}: missing POS file: {pos_path}")
    reference = comparison.read_reference_csv(reference_csv)
    solution = comparison.read_libgnss_pos(pos_path)
    if not solution:
        raise SystemExit(f"{label}: no solution epochs in {pos_path}")
    return ppc_metrics.ppc_official_segment_records(
        reference,
        solution,
        match_tolerance_s,
        threshold_m,
    )


def public_summary(summary: dict[str, object]) -> dict[str, object]:
    return {key: value for key, value in summary.items() if not key.startswith("_")}


def write_segments_csv(
    path: Path,
    candidate_summaries: list[dict[str, object]],
    *,
    changed_only: bool = True,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=SEGMENT_FIELDNAMES, lineterminator="\n")
        writer.writeheader()
        for summary in candidate_summaries:
            row_key = "_changed_rows" if changed_only else "_all_rows"
            for row in summary[row_key]:
                serialized = dict(row)
                for key, value in list(serialized.items()):
                    if value is None:
                        serialized[key] = ""
                    elif isinstance(value, float):
                        serialized[key] = rounded(value)
                writer.writerow(serialized)


def fmt(value: object, unit: str = "") -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.3f}{unit}"
    return f"{value}{unit}"


def markdown_transition_table(rows: list[dict[str, object]], key: str) -> list[str]:
    if not rows:
        return ["No changed segments."]
    lines = [
        f"| {key.replace('_', ' ')} | segments | distance m | score delta m |",
        "|---|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"| {row[key]} | {row['segments']} | {fmt(row['distance_m'])} | "
            f"{fmt(row['score_delta_distance_m'])} |"
        )
    return lines


def markdown_segment_table(rows: list[dict[str, object]]) -> list[str]:
    if not rows:
        return ["No segments."]
    lines = [
        "| ref | tow s | dist m | delta m | score | status | candidate 3D m | ratio | prefit RMS m |",
        "|---:|---:|---:|---:|---|---|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"| {row['reference_index']} | {fmt(row['end_tow_s'])} | "
            f"{fmt(row['segment_distance_m'])} | {fmt(row['score_delta_distance_m'])} | "
            f"{row['score_transition']} | {row['status_transition']} | "
            f"{fmt(row['candidate_error_3d_m'])} | {fmt(row['candidate_ratio'])} | "
            f"{fmt(row['candidate_rtk_update_prefit_residual_rms_m'])} |"
        )
    return lines


def render_markdown(
    baseline_label: str,
    baseline_pos: Path,
    reference_csv: Path,
    candidate_summaries: list[dict[str, object]],
) -> str:
    lines = [
        "# PPC Profile Segment Delta",
        "",
        f"Baseline: `{baseline_label}` from `{baseline_pos}`",
        f"Reference: `{reference_csv}`",
        "",
        "## Summary",
        "",
        "| Candidate | official % | delta pp | delta m | gain m | loss m | changed | gained | lost |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for summary in candidate_summaries:
        lines.append(
            f"| {summary['label']} | {fmt(summary['candidate_official_score_pct'])} | "
            f"{fmt(summary['delta_vs_baseline_score_pct'])} | "
            f"{fmt(summary['delta_vs_baseline_score_distance_m'])} | "
            f"{fmt(summary['gain_distance_m'])} | {fmt(summary['loss_distance_m'])} | "
            f"{summary['changed_segments']} | {summary['gain_segments']} | "
            f"{summary['loss_segments']} |"
        )

    for summary in candidate_summaries:
        lines.extend(["", f"## {summary['label']}", "", "### Score Transitions", ""])
        lines.extend(markdown_transition_table(summary["score_transitions"], "score_transition"))
        lines.extend(["", "### Status Transitions", ""])
        lines.extend(markdown_transition_table(summary["status_transitions"], "status_transition"))
        lines.extend(["", "### Top Gains", ""])
        lines.extend(markdown_segment_table(summary["top_gains"]))
        lines.extend(["", "### Top Losses", ""])
        lines.extend(markdown_segment_table(summary["top_losses"]))
    return "\n".join(lines) + "\n"


def main() -> None:
    args = parse_args()
    if not args.candidate:
        raise SystemExit("At least one --candidate LABEL=POS is required")
    candidates = [parse_candidate(value) for value in args.candidate]
    baseline_records = load_official_records(
        args.reference_csv,
        args.baseline_pos,
        args.match_tolerance_s,
        args.threshold_m,
        args.baseline_label,
    )
    candidate_summaries: list[dict[str, object]] = []
    for candidate in candidates:
        candidate_records = load_official_records(
            args.reference_csv,
            candidate.pos_path,
            args.match_tolerance_s,
            args.threshold_m,
            candidate.label,
        )
        candidate_summaries.append(
            compare_segment_records(
                baseline_records,
                candidate_records,
                candidate.label,
                args.top_segments,
            )
        )

    payload = {
        "reference_csv": str(args.reference_csv),
        "baseline_pos": str(args.baseline_pos),
        "baseline_label": args.baseline_label,
        "match_tolerance_s": args.match_tolerance_s,
        "threshold_m": args.threshold_m,
        "candidates": [public_summary(summary) for summary in candidate_summaries],
    }
    if args.summary_json:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    else:
        print(json.dumps(payload, indent=2, sort_keys=True))
    if args.markdown_output:
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(
            render_markdown(
                args.baseline_label,
                args.baseline_pos,
                args.reference_csv,
                candidate_summaries,
            ),
            encoding="utf-8",
        )
    if args.segments_csv:
        write_segments_csv(
            args.segments_csv,
            candidate_summaries,
            changed_only=not args.write_all_segments,
        )


if __name__ == "__main__":
    main()
