#!/usr/bin/env python3
"""Leave-one-run-out validation for PPC segment selector sweeps."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys


SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import analyze_ppc_segment_selector_sweep as selector_sweep  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--segment-csv",
        action="append",
        default=[],
        metavar="LABEL=CSV",
        help="Segment-delta CSV from analyze_ppc_profile_segment_delta.py. Repeat per run.",
    )
    parser.add_argument(
        "--top-rules",
        type=int,
        default=8,
        help="Number of ranked selector rules to keep per training fold (default: 8).",
    )
    parser.add_argument(
        "--max-thresholds",
        type=int,
        default=64,
        help="Maximum numeric thresholds to test per feature/category (default: 64).",
    )
    parser.add_argument(
        "--max-numeric-conditions",
        type=int,
        choices=(1, 2, 3),
        default=3,
        help="Maximum numeric conditions per rule (default: 3).",
    )
    parser.add_argument(
        "--numeric-refinement-beam",
        type=int,
        default=12,
        help="Top rules to refine at each numeric-condition level (default: 12).",
    )
    parser.add_argument(
        "--numeric-threshold-refinement-beam",
        type=int,
        default=32,
        help="Top ranked rules whose thresholds are refined exactly (default: 32).",
    )
    parser.add_argument(
        "--min-selected-distance-m",
        type=float,
        default=0.0,
        help="Drop rules that select less changed-segment distance than this threshold.",
    )
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--markdown-output", type=Path, default=None)
    return parser.parse_args()


def parse_rule(rule_text: str) -> selector_sweep.RuleSpec:
    text = rule_text.strip().strip("`")
    if text == "candidate_all":
        return selector_sweep.RuleSpec()
    categorical: list[selector_sweep.CategoricalCondition] = []
    numeric: list[selector_sweep.NumericCondition] = []
    for part in [item.strip().strip("`") for item in text.split(" AND ") if item.strip()]:
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
    return selector_sweep.normalize_rule(selector_sweep.RuleSpec(tuple(categorical), tuple(numeric)))


def rounded(value: float | None) -> float | None:
    return selector_sweep.rounded(value)


def fold_rows(
    rows: list[dict[str, object]],
    holdout_run: str,
) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    train_rows = [row for row in rows if str(row["run_label"]) != holdout_run]
    holdout_rows = [row for row in rows if str(row["run_label"]) == holdout_run]
    return train_rows, holdout_rows


def build_fold(
    rows: list[dict[str, object]],
    holdout_run: str,
    *,
    top_rules: int,
    max_thresholds: int,
    max_numeric_conditions: int,
    numeric_refinement_beam: int,
    numeric_threshold_refinement_beam: int,
    min_selected_distance_m: float,
) -> dict[str, object]:
    train_rows, holdout_rows = fold_rows(rows, holdout_run)
    if not train_rows or not holdout_rows:
        raise SystemExit(f"{holdout_run}: empty train or holdout split")
    train_payload = selector_sweep.build_payload(
        train_rows,
        top_rules=top_rules,
        max_thresholds=max_thresholds,
        max_numeric_conditions=max_numeric_conditions,
        numeric_refinement_beam=numeric_refinement_beam,
        numeric_threshold_refinement_beam=numeric_threshold_refinement_beam,
        min_selected_distance_m=min_selected_distance_m,
    )
    best_rule = dict(train_payload["top_rules"][0])
    rule = parse_rule(str(best_rule["rule"]))
    holdout_totals = selector_sweep.dataset_gain_loss(holdout_rows)
    holdout_score = selector_sweep.score_rule(
        holdout_rows,
        rule,
        totals=holdout_totals,
        include_by_run=False,
    )
    candidate_all = selector_sweep.score_rule(
        holdout_rows,
        selector_sweep.RuleSpec(),
        totals=holdout_totals,
        include_by_run=False,
    )
    return {
        "holdout_run": holdout_run,
        "learned_rule": best_rule["rule"],
        "train_changed_segments": train_payload["changed_segments"],
        "train_evaluated_rules": train_payload["evaluated_rules"],
        "train_selected_score_delta_distance_m": best_rule["selected_score_delta_distance_m"],
        "train_selected_gain_distance_m": best_rule["selected_gain_distance_m"],
        "train_selected_loss_distance_m": best_rule["selected_loss_distance_m"],
        "holdout_changed_segments": len(holdout_rows),
        "holdout_candidate_all_delta_distance_m": candidate_all[
            "selected_score_delta_distance_m"
        ],
        "holdout_selected_score_delta_distance_m": holdout_score[
            "selected_score_delta_distance_m"
        ],
        "holdout_selected_gain_distance_m": holdout_score["selected_gain_distance_m"],
        "holdout_selected_loss_distance_m": holdout_score["selected_loss_distance_m"],
        "holdout_selected_segments": holdout_score["selected_segments"],
        "holdout_distance_precision_pct": holdout_score["distance_precision_pct"],
        "holdout_gain_recall_pct": holdout_score["gain_recall_pct"],
        "holdout_loss_exposure_pct": holdout_score["loss_exposure_pct"],
    }


def aggregate_folds(folds: list[dict[str, object]]) -> dict[str, object]:
    holdout_net = sum(float(fold["holdout_selected_score_delta_distance_m"]) for fold in folds)
    holdout_gain = sum(float(fold["holdout_selected_gain_distance_m"]) for fold in folds)
    holdout_loss = sum(float(fold["holdout_selected_loss_distance_m"]) for fold in folds)
    candidate_all = sum(float(fold["holdout_candidate_all_delta_distance_m"]) for fold in folds)
    negative_runs = [
        fold["holdout_run"]
        for fold in folds
        if float(fold["holdout_selected_score_delta_distance_m"]) < 0.0
    ]
    selected_abs = holdout_gain + abs(holdout_loss)
    return {
        "fold_count": len(folds),
        "holdout_selected_score_delta_distance_m": rounded(holdout_net),
        "holdout_selected_gain_distance_m": rounded(holdout_gain),
        "holdout_selected_loss_distance_m": rounded(holdout_loss),
        "holdout_candidate_all_delta_distance_m": rounded(candidate_all),
        "holdout_selector_vs_candidate_all_delta_m": rounded(holdout_net - candidate_all),
        "holdout_distance_precision_pct": rounded(
            100.0 * holdout_gain / selected_abs if selected_abs > 0.0 else None
        ),
        "negative_holdout_runs": negative_runs,
        "nonnegative_holdout_runs": len(folds) - len(negative_runs),
        "min_holdout_delta_m": rounded(
            min(float(fold["holdout_selected_score_delta_distance_m"]) for fold in folds)
        ),
        "max_holdout_delta_m": rounded(
            max(float(fold["holdout_selected_score_delta_distance_m"]) for fold in folds)
        ),
    }


def build_payload(
    rows: list[dict[str, object]],
    *,
    top_rules: int,
    max_thresholds: int,
    max_numeric_conditions: int,
    numeric_refinement_beam: int,
    numeric_threshold_refinement_beam: int,
    min_selected_distance_m: float = 0.0,
) -> dict[str, object]:
    run_labels = sorted({str(row["run_label"]) for row in rows})
    if len(run_labels) < 2:
        raise SystemExit("Leave-one-run-out validation needs at least two runs")
    folds = [
        build_fold(
            rows,
            run_label,
            top_rules=top_rules,
            max_thresholds=max_thresholds,
            max_numeric_conditions=max_numeric_conditions,
            numeric_refinement_beam=numeric_refinement_beam,
            numeric_threshold_refinement_beam=numeric_threshold_refinement_beam,
            min_selected_distance_m=min_selected_distance_m,
        )
        for run_label in run_labels
    ]
    return {
        "runs": run_labels,
        "folds": folds,
        "aggregates": aggregate_folds(folds),
        "top_rules": top_rules,
        "max_thresholds": max_thresholds,
        "max_numeric_conditions": max_numeric_conditions,
        "numeric_refinement_beam": numeric_refinement_beam,
        "numeric_threshold_refinement_beam": numeric_threshold_refinement_beam,
        "min_selected_distance_m": min_selected_distance_m,
    }


def fmt(value: object, suffix: str = "") -> str:
    return selector_sweep.fmt(value, suffix)


def render_markdown(payload: dict[str, object]) -> str:
    aggregates = dict(payload["aggregates"])
    lines = [
        "# PPC Segment Selector Leave-One-Run-Out",
        "",
        f"Folds: **{aggregates['fold_count']}**",
        (
            "Holdout selected net: "
            f"**{fmt(aggregates['holdout_selected_score_delta_distance_m'])} m**"
        ),
        (
            "Holdout candidate-all net: "
            f"**{fmt(aggregates['holdout_candidate_all_delta_distance_m'])} m**"
        ),
        (
            "Selector vs candidate-all: "
            f"**{fmt(aggregates['holdout_selector_vs_candidate_all_delta_m'])} m**"
        ),
        (
            "Non-negative holdout runs: "
            f"**{aggregates['nonnegative_holdout_runs']} / {aggregates['fold_count']}**"
        ),
        "",
        "| holdout | train net m | holdout net m | holdout gain m | holdout loss m | candidate-all m | selected | learned rule |",
        "|---|---:|---:|---:|---:|---:|---:|---|",
    ]
    for fold in payload["folds"]:
        lines.append(
            f"| {fold['holdout_run']} | "
            f"{fmt(fold['train_selected_score_delta_distance_m'])} | "
            f"{fmt(fold['holdout_selected_score_delta_distance_m'])} | "
            f"{fmt(fold['holdout_selected_gain_distance_m'])} | "
            f"{fmt(fold['holdout_selected_loss_distance_m'])} | "
            f"{fmt(fold['holdout_candidate_all_delta_distance_m'])} | "
            f"{fold['holdout_selected_segments']} | "
            f"`{fold['learned_rule']}` |"
        )
    return "\n".join(lines) + "\n"


def main() -> None:
    args = parse_args()
    if not args.segment_csv:
        raise SystemExit("At least one --segment-csv LABEL=CSV is required")
    specs = [selector_sweep.parse_segment_csv(value) for value in args.segment_csv]
    rows = selector_sweep.load_segment_rows(specs)
    payload = build_payload(
        rows,
        top_rules=args.top_rules,
        max_thresholds=args.max_thresholds,
        max_numeric_conditions=args.max_numeric_conditions,
        numeric_refinement_beam=args.numeric_refinement_beam,
        numeric_threshold_refinement_beam=args.numeric_threshold_refinement_beam,
        min_selected_distance_m=args.min_selected_distance_m,
    )
    if args.summary_json:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    else:
        print(json.dumps(payload, indent=2, sort_keys=True))
    if args.markdown_output:
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(render_markdown(payload), encoding="utf-8")


if __name__ == "__main__":
    main()
