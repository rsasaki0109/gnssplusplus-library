#!/usr/bin/env python3
"""Apply a PPC dual-profile segment selector and write a combined POS file."""

from __future__ import annotations

import argparse
import csv
import json
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

import analyze_ppc_profile_segment_delta as profile_delta  # noqa: E402
import analyze_ppc_segment_selector_sweep as selector_sweep  # noqa: E402
import generate_driving_comparison as comparison  # noqa: E402
import gnss_ppc_metrics as ppc_metrics  # noqa: E402


SEGMENT_FIELDNAMES = [
    "reference_index",
    "start_tow_s",
    "end_tow_s",
    "selected_profile",
    "rule_matched",
    "score_delta_distance_m",
    "segment_distance_m",
    "score_transition",
    "status_transition",
    "baseline_score_state",
    "candidate_score_state",
    "baseline_status_name",
    "candidate_status_name",
    "baseline_solution_tow_s",
    "candidate_solution_tow_s",
    "selected_solution_tow_s",
    "candidate_rtk_update_post_suppression_residual_rms_m",
    "candidate_rtk_update_prefit_residual_rms_m",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--reference-csv", type=Path, required=True)
    parser.add_argument("--baseline-pos", type=Path, required=True)
    parser.add_argument("--candidate-pos", type=Path, required=True)
    parser.add_argument("--baseline-label", default="baseline")
    parser.add_argument("--candidate-label", default="candidate")
    parser.add_argument(
        "--rule",
        default=None,
        help=(
            "Selector rule, e.g. `candidate_status_name == FIXED AND "
            "candidate_num_satellites >= 8 AND "
            "candidate_rtk_update_post_suppression_residual_rms_m <= 3.1545`."
        ),
    )
    parser.add_argument(
        "--selector-summary-json",
        type=Path,
        default=None,
        help="Optional selector sweep JSON whose top rule should be applied.",
    )
    parser.add_argument(
        "--rule-index",
        type=int,
        default=1,
        help="1-based top_rules index when --selector-summary-json is used (default: 1).",
    )
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--threshold-m", type=float, default=0.50)
    parser.add_argument("--out-pos", type=Path, required=True)
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--segments-csv", type=Path, default=None)
    return parser.parse_args()


def load_rule_text(rule_text: str | None, selector_summary_json: Path | None, rule_index: int) -> str:
    if rule_text and selector_summary_json is not None:
        raise SystemExit("Use either --rule or --selector-summary-json, not both")
    if rule_text:
        return rule_text.strip()
    if selector_summary_json is None:
        raise SystemExit("Either --rule or --selector-summary-json is required")
    payload = json.loads(selector_summary_json.read_text(encoding="utf-8"))
    top_rules = payload.get("top_rules")
    if not isinstance(top_rules, list) or not top_rules:
        raise SystemExit(f"{selector_summary_json}: missing non-empty top_rules")
    if rule_index < 1 or rule_index > len(top_rules):
        raise SystemExit(f"--rule-index must be within 1..{len(top_rules)}")
    rule = top_rules[rule_index - 1]
    if not isinstance(rule, dict) or not isinstance(rule.get("rule"), str):
        raise SystemExit(f"{selector_summary_json}: selected top rule is malformed")
    return str(rule["rule"])


def parse_rule(rule_text: str) -> selector_sweep.RuleSpec:
    text = rule_text.strip().strip("`")
    if text == "candidate_all":
        return selector_sweep.RuleSpec()
    categorical: list[selector_sweep.CategoricalCondition] = []
    numeric: list[selector_sweep.NumericCondition] = []
    parts = [part.strip().strip("`") for part in text.split(" AND ") if part.strip()]
    if not parts:
        raise SystemExit("Selector rule is empty")
    for part in parts:
        if "==" in part:
            feature, value = [item.strip().strip("`") for item in part.split("==", 1)]
            if not feature or not value:
                raise SystemExit(f"Malformed categorical selector condition: {part}")
            categorical.append(selector_sweep.CategoricalCondition(feature, value))
            continue
        operator = "<=" if "<=" in part else ">=" if ">=" in part else None
        if operator is None:
            raise SystemExit(f"Unsupported selector condition: {part}")
        feature, value = [item.strip().strip("`") for item in part.split(operator, 1)]
        if not feature or not value:
            raise SystemExit(f"Malformed numeric selector condition: {part}")
        numeric.append(selector_sweep.NumericCondition(feature, operator, float(value)))
    return selector_sweep.RuleSpec(tuple(categorical), tuple(numeric))


def epoch_key(epoch: comparison.SolutionEpoch) -> tuple[int, float]:
    return epoch.week, round(epoch.tow, 6)


def epoch_map(epochs: list[comparison.SolutionEpoch]) -> dict[tuple[int, float], comparison.SolutionEpoch]:
    return {epoch_key(epoch): epoch for epoch in epochs}


def best_epoch_for_reference(
    reference: comparison.ReferenceEpoch,
    epochs: list[comparison.SolutionEpoch],
    tolerance_s: float,
) -> comparison.SolutionEpoch | None:
    candidates = [
        epoch
        for epoch in epochs
        if epoch.week == reference.week and abs(epoch.tow - reference.tow) <= tolerance_s
    ]
    if not candidates:
        return None
    return min(candidates, key=lambda epoch: abs(epoch.tow - reference.tow))


def all_segment_rows(
    baseline_records: list[dict[str, object]],
    candidate_records: list[dict[str, object]],
    candidate_label: str,
) -> list[dict[str, object]]:
    baseline_by_index = profile_delta.records_by_reference(baseline_records, "baseline")
    candidate_by_index = profile_delta.records_by_reference(candidate_records, candidate_label)
    if set(baseline_by_index) != set(candidate_by_index):
        raise SystemExit("baseline and candidate official segment sets differ")
    return [
        profile_delta.segment_row(
            baseline_by_index[reference_index],
            candidate_by_index[reference_index],
            candidate_label,
        )
        for reference_index in sorted(baseline_by_index)
    ]


def solution_tow_key(record: dict[str, object], prefix: str, reference_week: int) -> tuple[int, float] | None:
    value = record.get(f"{prefix}_solution_tow_s")
    if value is None:
        return None
    return reference_week, round(float(value), 6)


def augment_solution_tows(
    rows: list[dict[str, object]],
    baseline_records: list[dict[str, object]],
    candidate_records: list[dict[str, object]],
) -> None:
    baseline_by_index = profile_delta.records_by_reference(baseline_records, "baseline")
    candidate_by_index = profile_delta.records_by_reference(candidate_records, "candidate")
    for row in rows:
        reference_index = int(row["reference_index"])
        row["baseline_solution_tow_s"] = baseline_by_index[reference_index].get("solution_tow_s")
        row["candidate_solution_tow_s"] = candidate_by_index[reference_index].get("solution_tow_s")


def selected_solution_epochs(
    reference: list[comparison.ReferenceEpoch],
    baseline_epochs: list[comparison.SolutionEpoch],
    candidate_epochs: list[comparison.SolutionEpoch],
    segment_rows: list[dict[str, object]],
    rule: selector_sweep.RuleSpec,
    match_tolerance_s: float,
) -> tuple[list[comparison.SolutionEpoch], list[dict[str, object]]]:
    baseline_by_tow = epoch_map(baseline_epochs)
    candidate_by_tow = epoch_map(candidate_epochs)
    selected_by_key: dict[tuple[int, float], comparison.SolutionEpoch] = {}
    selected_rows: list[dict[str, object]] = []

    if reference:
        first_epoch = best_epoch_for_reference(reference[0], baseline_epochs, match_tolerance_s)
        if first_epoch is None:
            first_epoch = best_epoch_for_reference(reference[0], candidate_epochs, match_tolerance_s)
        if first_epoch is not None:
            selected_by_key[epoch_key(first_epoch)] = first_epoch

    for row in segment_rows:
        ref_index = int(row["reference_index"])
        if ref_index >= len(reference):
            continue
        reference_week = reference[ref_index].week
        rule_matched = selector_sweep.matches_rule(row, rule)
        selected_profile = "candidate" if rule_matched else "baseline"
        key = solution_tow_key(row, selected_profile, reference_week)
        source_map = candidate_by_tow if selected_profile == "candidate" else baseline_by_tow
        epoch = source_map.get(key) if key is not None else None
        if epoch is None and selected_profile == "candidate":
            selected_profile = "baseline"
            key = solution_tow_key(row, "baseline", reference_week)
            epoch = baseline_by_tow.get(key) if key is not None else None
        if epoch is not None:
            selected_by_key[epoch_key(epoch)] = epoch
        selected_row = dict(row)
        selected_row["rule_matched"] = rule_matched
        selected_row["selected_profile"] = selected_profile
        selected_row["selected_solution_tow_s"] = epoch.tow if epoch is not None else None
        selected_rows.append(selected_row)

    return (
        [selected_by_key[key] for key in sorted(selected_by_key)],
        selected_rows,
    )


def format_float(value: float, decimals: int) -> str:
    return f"{value:.{decimals}f}"


def optional_tokens(epoch: comparison.SolutionEpoch) -> list[object | None]:
    return [
        epoch.ratio,
        epoch.baseline_m,
        epoch.rtk_iterations,
        epoch.rtk_update_observations,
        epoch.rtk_update_phase_observations,
        epoch.rtk_update_code_observations,
        epoch.rtk_update_suppressed_outliers,
        epoch.rtk_update_prefit_residual_rms_m,
        epoch.rtk_update_prefit_residual_max_m,
        epoch.rtk_update_post_suppression_residual_rms_m,
        epoch.rtk_update_post_suppression_residual_max_m,
    ]


def write_pos(path: Path, epochs: list[comparison.SolutionEpoch]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as handle:
        handle.write("% Dual-profile PPC selector position solution\n")
        handle.write(
            "% GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) "
            "Status NumSat PDOP Ratio Baseline(m) RtkIter RtkObs RtkPhaseObs "
            "RtkCodeObs RtkOutliers RtkPrefitRms(m) RtkPrefitMax(m) "
            "RtkPostRms(m) RtkPostMax(m)\n"
        )
        for epoch in epochs:
            tokens: list[str] = [
                str(epoch.week),
                format_float(epoch.tow, 3),
                format_float(float(epoch.ecef[0]), 4),
                format_float(float(epoch.ecef[1]), 4),
                format_float(float(epoch.ecef[2]), 4),
                format_float(epoch.lat_deg, 9),
                format_float(epoch.lon_deg, 9),
                format_float(epoch.height_m, 4),
                str(epoch.status),
                str(epoch.num_satellites),
                "2.00",
            ]
            optional = optional_tokens(epoch)
            while optional and optional[-1] is None:
                optional.pop()
            for value in optional:
                if value is None:
                    tokens.append("0")
                elif isinstance(value, int):
                    tokens.append(str(value))
                else:
                    tokens.append(format_float(float(value), 4))
            handle.write(" ".join(tokens) + "\n")


def write_segments_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=SEGMENT_FIELDNAMES, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    key: "" if row.get(key) is None else row.get(key)
                    for key in SEGMENT_FIELDNAMES
                }
            )


def selection_summary(rows: list[dict[str, object]]) -> dict[str, object]:
    candidate_rows = [row for row in rows if row["selected_profile"] == "candidate"]
    candidate_delta = sum(float(row["score_delta_distance_m"]) for row in candidate_rows)
    return {
        "segments": len(rows),
        "candidate_selected_segments": len(candidate_rows),
        "baseline_selected_segments": len(rows) - len(candidate_rows),
        "candidate_selected_score_delta_distance_m": profile_delta.rounded(candidate_delta),
        "candidate_selected_gain_distance_m": profile_delta.rounded(
            sum(max(0.0, float(row["score_delta_distance_m"])) for row in candidate_rows)
        ),
        "candidate_selected_loss_distance_m": profile_delta.rounded(
            sum(min(0.0, float(row["score_delta_distance_m"])) for row in candidate_rows)
        ),
    }


def build_payload(
    reference: list[comparison.ReferenceEpoch],
    baseline_epochs: list[comparison.SolutionEpoch],
    candidate_epochs: list[comparison.SolutionEpoch],
    selected_epochs: list[comparison.SolutionEpoch],
    selected_rows: list[dict[str, object]],
    rule_text: str,
    match_tolerance_s: float,
    out_pos: Path,
) -> dict[str, object]:
    baseline_metrics = ppc_metrics.summarize_solution_epochs(
        reference,
        baseline_epochs,
        fixed_status=4,
        label="baseline",
        match_tolerance_s=match_tolerance_s,
        solver_wall_time_s=None,
    )
    candidate_metrics = ppc_metrics.summarize_solution_epochs(
        reference,
        candidate_epochs,
        fixed_status=4,
        label="candidate",
        match_tolerance_s=match_tolerance_s,
        solver_wall_time_s=None,
    )
    selected_metrics = ppc_metrics.summarize_solution_epochs(
        reference,
        selected_epochs,
        fixed_status=4,
        label="dual-profile selector",
        match_tolerance_s=match_tolerance_s,
        solver_wall_time_s=None,
    )
    return {
        "rule": rule_text,
        "out_pos": str(out_pos),
        "selection": selection_summary(selected_rows),
        "metrics": selected_metrics,
        "baseline": baseline_metrics,
        "candidate": candidate_metrics,
        "delta_vs_baseline": ppc_metrics.solution_metric_delta(selected_metrics, baseline_metrics),
        "delta_vs_candidate": ppc_metrics.solution_metric_delta(selected_metrics, candidate_metrics),
    }


def main() -> None:
    args = parse_args()
    rule_text = load_rule_text(args.rule, args.selector_summary_json, args.rule_index)
    rule = parse_rule(rule_text)
    reference = comparison.read_reference_csv(args.reference_csv)
    baseline_epochs = comparison.read_libgnss_pos(args.baseline_pos)
    candidate_epochs = comparison.read_libgnss_pos(args.candidate_pos)
    baseline_records = ppc_metrics.ppc_official_segment_records(
        reference,
        baseline_epochs,
        args.match_tolerance_s,
        args.threshold_m,
    )
    candidate_records = ppc_metrics.ppc_official_segment_records(
        reference,
        candidate_epochs,
        args.match_tolerance_s,
        args.threshold_m,
    )
    segment_rows = all_segment_rows(baseline_records, candidate_records, args.candidate_label)
    augment_solution_tows(segment_rows, baseline_records, candidate_records)
    selected_epochs, selected_rows = selected_solution_epochs(
        reference,
        baseline_epochs,
        candidate_epochs,
        segment_rows,
        rule,
        args.match_tolerance_s,
    )
    if not selected_epochs:
        raise SystemExit("Selector produced no solution epochs")
    write_pos(args.out_pos, selected_epochs)
    payload = build_payload(
        reference,
        baseline_epochs,
        candidate_epochs,
        selected_epochs,
        selected_rows,
        rule_text,
        args.match_tolerance_s,
        args.out_pos,
    )
    if args.summary_json is not None:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    else:
        print(json.dumps(payload, indent=2, sort_keys=True))
    if args.segments_csv is not None:
        write_segments_csv(args.segments_csv, selected_rows)


if __name__ == "__main__":
    main()
