#!/usr/bin/env python3
"""Apply a PPC multi-candidate segment selector and write a combined POS file.

Algorithm (case B: simultaneous per-segment selection):

1. Load baseline + N candidate POS files; build epoch dicts keyed by (week, tow).
2. Compute ppc_official_segment_records for each candidate against reference CSV.
3. For each segment, evaluate all candidates whose rule matches; select the one
   with the highest score_delta_distance_m.  Tie-break by --priority-order.
4. Per-run non-regression gate: if any candidate's *net* contribution (sum of
   score_delta across segments it was selected for) is negative, drop that
   candidate and retry selection without it.
5. Merge selected epochs into a single POS file and write summary JSON /
   segments CSV.
"""

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


# ---------------------------------------------------------------------------
# Segment CSV columns written to --segments-csv
# ---------------------------------------------------------------------------
SEGMENT_FIELDNAMES = [
    "reference_index",
    "start_tow_s",
    "end_tow_s",
    "selected_candidate",
    "rule_matched",
    "score_delta_distance_m",
    "segment_distance_m",
    "score_transition",
    "status_transition",
    "baseline_status_name",
    "selected_status_name",
    "selected_solution_tow_s",
]


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_candidate(text: str) -> tuple[str, Path]:
    """Parse 'LABEL=path/to/file.pos' into (label, Path)."""
    if "=" not in text:
        raise SystemExit(f"--candidate must use LABEL=PATH, got: {text!r}")
    label, path_text = text.split("=", 1)
    label = label.strip()
    if not label:
        raise SystemExit(f"--candidate label is empty in: {text!r}")
    return label, Path(path_text.strip())


def _parse_candidate_rule(text: str) -> tuple[str, str]:
    """Parse 'LABEL=rule text' into (label, rule_text)."""
    if "=" not in text:
        raise SystemExit(f"--candidate-rule must use LABEL=RULE, got: {text!r}")
    label, rule_text = text.split("=", 1)
    return label.strip(), rule_text.strip()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description="Multi-candidate PPC segment selector.",
    )
    parser.add_argument("--reference-csv", type=Path, required=True)
    parser.add_argument("--baseline-pos", type=Path, required=True)
    parser.add_argument(
        "--candidate",
        dest="candidates",
        action="append",
        default=[],
        metavar="LABEL=PATH",
        help="Repeat for each candidate POS (e.g. nis5=output/.../<run>.pos).",
    )
    parser.add_argument(
        "--candidate-rule",
        dest="candidate_rules",
        action="append",
        default=[],
        metavar="LABEL=RULE",
        help="Selector rule per candidate label.",
    )
    parser.add_argument(
        "--priority-order",
        default="",
        help="Comma-separated candidate labels; earlier = higher priority on tie.",
    )
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--threshold-m", type=float, default=0.50)
    parser.add_argument("--out-pos", type=Path, required=True)
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--segments-csv", type=Path, default=None)
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Rule parsing (reused from apply_ppc_dual_profile_selector)
# ---------------------------------------------------------------------------

def parse_rule(rule_text: str) -> selector_sweep.RuleSpec:
    text = rule_text.strip().strip("`")
    if not text or text == "candidate_all":
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


# ---------------------------------------------------------------------------
# Epoch helpers (reused from apply_ppc_dual_profile_selector)
# ---------------------------------------------------------------------------

def epoch_key(epoch: comparison.SolutionEpoch) -> tuple[int, float]:
    return epoch.week, round(epoch.tow, 6)


def epoch_map(
    epochs: list[comparison.SolutionEpoch],
) -> dict[tuple[int, float], comparison.SolutionEpoch]:
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


def solution_tow_key(
    record: dict[str, object], prefix: str, reference_week: int
) -> tuple[int, float] | None:
    value = record.get(f"{prefix}_solution_tow_s")
    if value is None:
        return None
    return reference_week, round(float(value), 6)


# ---------------------------------------------------------------------------
# Segment rows (multi-candidate version)
# ---------------------------------------------------------------------------

def build_candidate_segment_rows(
    baseline_records: list[dict[str, object]],
    candidate_records: list[dict[str, object]],
    candidate_label: str,
) -> list[dict[str, object]]:
    """Return segment rows for one (baseline, candidate) pair."""
    baseline_by_index = profile_delta.records_by_reference(baseline_records, "baseline")
    candidate_by_index = profile_delta.records_by_reference(
        candidate_records, candidate_label
    )
    if set(baseline_by_index) != set(candidate_by_index):
        raise SystemExit(
            f"baseline and candidate '{candidate_label}' official segment sets differ"
        )
    rows = []
    for reference_index in sorted(baseline_by_index):
        row = profile_delta.segment_row(
            baseline_by_index[reference_index],
            candidate_by_index[reference_index],
            candidate_label,
        )
        row["candidate_solution_tow_s"] = candidate_by_index[reference_index].get(
            "solution_tow_s"
        )
        row["baseline_solution_tow_s"] = baseline_by_index[reference_index].get(
            "solution_tow_s"
        )
        rows.append(row)
    return rows


# ---------------------------------------------------------------------------
# Core selection logic
# ---------------------------------------------------------------------------

def _priority_index(label: str, priority_order: list[str]) -> int:
    """Return position of *label* in priority list; unlisted = appended last."""
    try:
        return priority_order.index(label)
    except ValueError:
        return len(priority_order)


def select_segments(
    reference: list[comparison.ReferenceEpoch],
    baseline_epochs: list[comparison.SolutionEpoch],
    candidate_specs: list[tuple[str, list[comparison.SolutionEpoch], selector_sweep.RuleSpec]],
    candidate_rows_by_label: dict[str, list[dict[str, object]]],
    priority_order: list[str],
    match_tolerance_s: float,
    baseline_records: list[dict[str, object]],
    candidate_records_by_label: dict[str, list[dict[str, object]]],
) -> tuple[list[comparison.SolutionEpoch], list[dict[str, object]]]:
    """Select epochs segment-by-segment from all candidates simultaneously.

    Returns (selected_epochs, segment_result_rows).
    """
    active_labels = [label for label, _, _ in candidate_specs]

    # Map epochs for fast lookup
    baseline_by_tow = epoch_map(baseline_epochs)
    candidate_epoch_maps: dict[str, dict[tuple[int, float], comparison.SolutionEpoch]] = {
        label: epoch_map(epochs) for label, epochs, _ in candidate_specs
    }

    # Map segment rows by label -> reference_index
    rows_by_label_index: dict[str, dict[int, dict[str, object]]] = {}
    for label in active_labels:
        rows_by_label_index[label] = {
            int(row["reference_index"]): row
            for row in candidate_rows_by_label[label]
        }

    selected_by_key: dict[tuple[int, float], comparison.SolutionEpoch] = {}
    result_rows: list[dict[str, object]] = []

    # Ensure first reference epoch is anchored in baseline
    if reference:
        first_epoch = best_epoch_for_reference(
            reference[0], baseline_epochs, match_tolerance_s
        )
        if first_epoch is not None:
            selected_by_key[epoch_key(first_epoch)] = first_epoch

    # Iterate over segments (use baseline reference indices, sorted)
    baseline_by_index = profile_delta.records_by_reference(baseline_records, "baseline")

    for reference_index in sorted(baseline_by_index):
        if reference_index >= len(reference):
            continue
        reference_week = reference[reference_index].week

        # Collect rule-matching candidates for this segment
        matching: list[tuple[str, float]] = []  # (label, score_delta)
        for label, _, rule in candidate_specs:
            row = rows_by_label_index[label].get(reference_index)
            if row is None:
                continue
            if selector_sweep.matches_rule(row, rule):
                delta = float(row.get("score_delta_distance_m") or 0.0)
                matching.append((label, delta))

        # Choose best candidate: max delta, tie-break by priority_order
        selected_label: str | None = None
        selected_delta: float = 0.0  # baseline = 0 reference

        for label, delta in matching:
            if delta > selected_delta or (
                delta == selected_delta
                and selected_label is not None
                and _priority_index(label, priority_order)
                < _priority_index(selected_label, priority_order)
            ):
                selected_label = label
                selected_delta = delta

        # Resolve epoch from selected candidate (or baseline)
        if selected_label is not None:
            cand_row = rows_by_label_index[selected_label][reference_index]
            key = solution_tow_key(cand_row, "candidate", reference_week)
            epoch = candidate_epoch_maps[selected_label].get(key) if key is not None else None
            if epoch is None:
                # fallback to baseline
                selected_label = None
        if selected_label is None:
            bl_row = baseline_by_index[reference_index]
            bl_tow = bl_row.get("solution_tow_s")
            key = (reference_week, round(float(bl_tow), 6)) if bl_tow is not None else None
            epoch = baseline_by_tow.get(key) if key is not None else None

        if epoch is not None:
            selected_by_key[epoch_key(epoch)] = epoch

        # Build result row for CSV
        best_row = (
            rows_by_label_index[selected_label][reference_index]
            if selected_label is not None
            else None
        )
        bl_row_data = baseline_by_index[reference_index]
        result_row: dict[str, object] = {
            "reference_index": reference_index,
            "start_tow_s": bl_row_data.get("start_tow_s"),
            "end_tow_s": bl_row_data.get("end_tow_s"),
            "selected_candidate": selected_label if selected_label is not None else "baseline",
            "rule_matched": selected_label is not None,
            "score_delta_distance_m": selected_delta if selected_label is not None else 0.0,
            "segment_distance_m": bl_row_data.get("segment_distance_m"),
            "score_transition": best_row.get("score_transition") if best_row else "",
            "status_transition": best_row.get("status_transition") if best_row else "",
            "baseline_status_name": best_row.get("baseline_status_name") if best_row else "",
            "selected_status_name": (
                best_row.get("candidate_status_name") if best_row else ""
            ),
            "selected_solution_tow_s": epoch.tow if epoch is not None else None,
        }
        result_rows.append(result_row)

    return (
        [selected_by_key[key] for key in sorted(selected_by_key)],
        result_rows,
    )


# ---------------------------------------------------------------------------
# Per-run non-regression gate
# ---------------------------------------------------------------------------

def drop_negative_candidates(
    candidate_specs: list[tuple[str, list[comparison.SolutionEpoch], selector_sweep.RuleSpec]],
    candidate_rows_by_label: dict[str, list[dict[str, object]]],
    result_rows: list[dict[str, object]],
) -> tuple[
    list[tuple[str, list[comparison.SolutionEpoch], selector_sweep.RuleSpec]],
    bool,
]:
    """Drop any candidate whose selected net contribution is negative.

    Returns (updated_candidate_specs, any_dropped).
    """
    # Sum score_delta for each selected candidate
    net_by_label: dict[str, float] = {}
    for row in result_rows:
        label = str(row["selected_candidate"])
        if label == "baseline":
            continue
        delta = float(row["score_delta_distance_m"])
        net_by_label[label] = net_by_label.get(label, 0.0) + delta

    dropped = [label for label, net in net_by_label.items() if net < 0.0]
    if not dropped:
        return candidate_specs, False

    filtered = [spec for spec in candidate_specs if spec[0] not in dropped]
    return filtered, True


def run_selection_with_nonneg_constraint(
    reference: list[comparison.ReferenceEpoch],
    baseline_epochs: list[comparison.SolutionEpoch],
    candidate_specs: list[tuple[str, list[comparison.SolutionEpoch], selector_sweep.RuleSpec]],
    candidate_rows_by_label: dict[str, list[dict[str, object]]],
    priority_order: list[str],
    match_tolerance_s: float,
    baseline_records: list[dict[str, object]],
    candidate_records_by_label: dict[str, list[dict[str, object]]],
) -> tuple[list[comparison.SolutionEpoch], list[dict[str, object]], list[str]]:
    """Iteratively drop negative-contributing candidates and re-select.

    Returns (selected_epochs, result_rows, dropped_labels).
    """
    active_specs = list(candidate_specs)
    all_dropped: list[str] = []
    max_iterations = len(candidate_specs) + 2

    for _ in range(max_iterations):
        selected_epochs, result_rows = select_segments(
            reference,
            baseline_epochs,
            active_specs,
            candidate_rows_by_label,
            priority_order,
            match_tolerance_s,
            baseline_records,
            candidate_records_by_label,
        )
        active_specs, any_dropped = drop_negative_candidates(
            active_specs, candidate_rows_by_label, result_rows
        )
        if not any_dropped:
            break
        for spec in candidate_specs:
            if spec[0] not in [s[0] for s in active_specs] and spec[0] not in all_dropped:
                all_dropped.append(spec[0])

    return selected_epochs, result_rows, all_dropped


# ---------------------------------------------------------------------------
# POS writer (mirrors apply_ppc_dual_profile_selector.write_pos)
# ---------------------------------------------------------------------------

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
        epoch.rtk_update_normalized_innovation_squared,
        epoch.rtk_update_normalized_innovation_squared_per_observation,
        epoch.rtk_update_rejected_by_innovation_gate,
    ]


def write_pos(path: Path, epochs: list[comparison.SolutionEpoch]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as handle:
        handle.write("% Multi-candidate PPC selector position solution\n")
        handle.write(
            "% GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) "
            "Status NumSat PDOP Ratio Baseline(m) RtkIter RtkObs RtkPhaseObs "
            "RtkCodeObs RtkOutliers RtkPrefitRms(m) RtkPrefitMax(m) "
            "RtkPostRms(m) RtkPostMax(m) RtkUpdateNIS RtkUpdateNISPerObs "
            "RtkUpdateNISRejected\n"
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
        writer = csv.DictWriter(
            handle, fieldnames=SEGMENT_FIELDNAMES, lineterminator="\n"
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    key: "" if row.get(key) is None else row.get(key)
                    for key in SEGMENT_FIELDNAMES
                }
            )


# ---------------------------------------------------------------------------
# Summary helpers
# ---------------------------------------------------------------------------

def selection_summary(
    result_rows: list[dict[str, object]],
    candidate_labels: list[str],
) -> dict[str, object]:
    candidate_rows = [
        row for row in result_rows if str(row["selected_candidate"]) != "baseline"
    ]
    per_candidate: dict[str, dict[str, object]] = {}
    for label in candidate_labels:
        label_rows = [
            row for row in candidate_rows if str(row["selected_candidate"]) == label
        ]
        per_candidate[label] = {
            "selected_segments": len(label_rows),
            "score_delta_distance_m": profile_delta.rounded(
                sum(float(row["score_delta_distance_m"]) for row in label_rows)
            ),
        }
    return {
        "segments": len(result_rows),
        "candidate_selected_segments": len(candidate_rows),
        "baseline_selected_segments": len(result_rows) - len(candidate_rows),
        "total_score_delta_distance_m": profile_delta.rounded(
            sum(float(row["score_delta_distance_m"]) for row in candidate_rows)
        ),
        "per_candidate": per_candidate,
    }


def build_payload(
    reference: list[comparison.ReferenceEpoch],
    baseline_epochs: list[comparison.SolutionEpoch],
    candidate_specs: list[tuple[str, list[comparison.SolutionEpoch], selector_sweep.RuleSpec]],
    selected_epochs: list[comparison.SolutionEpoch],
    result_rows: list[dict[str, object]],
    priority_order: list[str],
    dropped_labels: list[str],
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
    selected_metrics = ppc_metrics.summarize_solution_epochs(
        reference,
        selected_epochs,
        fixed_status=4,
        label="multi-candidate selector",
        match_tolerance_s=match_tolerance_s,
        solver_wall_time_s=None,
    )
    candidate_labels = [label for label, _, _ in candidate_specs]
    return {
        "out_pos": str(out_pos),
        "priority_order": priority_order,
        "dropped_candidates": dropped_labels,
        "active_candidates": candidate_labels,
        "rules": {label: rule.expression() for label, _, rule in candidate_specs},
        "selection": selection_summary(result_rows, candidate_labels),
        "metrics": selected_metrics,
        "baseline": baseline_metrics,
        "delta_vs_baseline": ppc_metrics.solution_metric_delta(
            selected_metrics, baseline_metrics
        ),
    }


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main() -> None:
    args = parse_args()

    # Parse candidate specs
    candidate_paths: dict[str, Path] = {}
    for text in args.candidates:
        label, path = _parse_candidate(text)
        candidate_paths[label] = path

    candidate_rule_texts: dict[str, str] = {}
    for text in args.candidate_rules:
        label, rule_text = _parse_candidate_rule(text)
        candidate_rule_texts[label] = rule_text

    if not candidate_paths:
        raise SystemExit("At least one --candidate is required")

    priority_order = (
        [s.strip() for s in args.priority_order.split(",") if s.strip()]
        if args.priority_order
        else list(candidate_paths.keys())
    )

    # Load reference + baseline
    reference = comparison.read_reference_csv(args.reference_csv)
    baseline_epochs = comparison.read_libgnss_pos(args.baseline_pos)
    baseline_records = ppc_metrics.ppc_official_segment_records(
        reference,
        baseline_epochs,
        args.match_tolerance_s,
        args.threshold_m,
    )

    # Load each candidate
    candidate_specs: list[
        tuple[str, list[comparison.SolutionEpoch], selector_sweep.RuleSpec]
    ] = []
    candidate_rows_by_label: dict[str, list[dict[str, object]]] = {}
    candidate_records_by_label: dict[str, list[dict[str, object]]] = {}

    for label, pos_path in candidate_paths.items():
        rule_text = candidate_rule_texts.get(label, "candidate_all")
        rule = parse_rule(rule_text)
        epochs = comparison.read_libgnss_pos(pos_path)
        records = ppc_metrics.ppc_official_segment_records(
            reference,
            epochs,
            args.match_tolerance_s,
            args.threshold_m,
        )
        rows = build_candidate_segment_rows(baseline_records, records, label)
        candidate_specs.append((label, epochs, rule))
        candidate_rows_by_label[label] = rows
        candidate_records_by_label[label] = records

    # Run selection with non-regression gate
    selected_epochs, result_rows, dropped_labels = run_selection_with_nonneg_constraint(
        reference,
        baseline_epochs,
        candidate_specs,
        candidate_rows_by_label,
        priority_order,
        args.match_tolerance_s,
        baseline_records,
        candidate_records_by_label,
    )

    if not selected_epochs:
        raise SystemExit("Selector produced no solution epochs")

    # Write outputs
    write_pos(args.out_pos, selected_epochs)

    payload = build_payload(
        reference,
        baseline_epochs,
        candidate_specs,
        selected_epochs,
        result_rows,
        priority_order,
        dropped_labels,
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
        write_segments_csv(args.segments_csv, result_rows)


if __name__ == "__main__":
    main()
