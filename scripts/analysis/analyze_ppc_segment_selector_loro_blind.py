#!/usr/bin/env python3
"""Truth-blind leave-one-run-out validation for PPC segment selectors.

Training folds may use reference scores to learn a rule and a candidate priority
order. Holdout folds only use the learned rule plus priority order; no holdout
score delta is used to choose between matching candidates.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
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
        help=(
            "Segment CSV from analyze_ppc_profile_segment_delta.py. Repeat per run. "
            "Use --write-all-segments when creating these CSVs."
        ),
    )
    parser.add_argument("--top-rules", type=int, default=8)
    parser.add_argument("--max-thresholds", type=int, default=64)
    parser.add_argument("--max-numeric-conditions", type=int, choices=(0, 1, 2, 3), default=1)
    parser.add_argument("--numeric-refinement-beam", type=int, default=24)
    parser.add_argument("--min-selected-distance-m", type=float, default=0.0)
    parser.add_argument(
        "--preselect-rules",
        type=int,
        default=0,
        help=(
            "If positive, rank rules on rule-source rows first and full-score only "
            "the top N. Useful for candidate-label categorical sweeps."
        ),
    )
    parser.add_argument(
        "--rule-source",
        choices=("changed", "all"),
        default="changed",
        help=(
            "Rows used to generate candidate rules. Final scoring still uses all "
            "loaded rows (default: changed)."
        ),
    )
    parser.add_argument(
        "--include-candidate-label",
        action="store_true",
        help=(
            "Allow candidate_label as a learned categorical condition, including "
            "candidate_label AND one status/category condition."
        ),
    )
    parser.add_argument(
        "--rank-objective",
        choices=selector_sweep.RANK_OBJECTIVES,
        default="robust",
    )
    parser.add_argument(
        "--required-candidate-status",
        default=None,
        help="Keep only rules that explicitly require this candidate solution status.",
    )
    parser.add_argument(
        "--required-categorical",
        action="append",
        default=[],
        metavar="FEATURE=VALUE",
    )
    parser.add_argument(
        "--required-numeric",
        action="append",
        default=[],
        metavar="FEATURE>=VALUE",
    )
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--markdown-output", type=Path, default=None)
    return parser.parse_args()


def rounded(value: float | None) -> float | None:
    return selector_sweep.rounded(value)


def parse_rule(rule_text: str) -> selector_sweep.RuleSpec:
    text = rule_text.strip().strip("`")
    if not text or text == "candidate_all":
        return selector_sweep.RuleSpec()
    categorical: list[selector_sweep.CategoricalCondition] = []
    numeric: list[selector_sweep.NumericCondition] = []
    for part in [item.strip().strip("`") for item in text.split(" AND ") if item.strip()]:
        if "==" in part:
            feature, value = [item.strip().strip("`") for item in part.split("==", 1)]
            categorical.append(selector_sweep.CategoricalCondition(feature, value))
            continue
        operator = "<=" if "<=" in part else ">=" if ">=" in part else None
        if operator is None:
            raise SystemExit(f"Unsupported selector condition: {part}")
        feature, value = [item.strip().strip("`") for item in part.split(operator, 1)]
        numeric.append(selector_sweep.NumericCondition(feature, operator, float(value)))
    return selector_sweep.normalize_rule(
        selector_sweep.RuleSpec(tuple(categorical), tuple(numeric))
    )


def candidate_labels(rows: list[dict[str, object]]) -> list[str]:
    return sorted({str(row["candidate_label"]) for row in rows})


def categorical_features(include_candidate_label: bool) -> tuple[str, ...]:
    features = tuple(selector_sweep.CATEGORICAL_FEATURES)
    if include_candidate_label:
        return (*features, "candidate_label")
    return features


def categorical_values_for_feature(rows: list[dict[str, object]], feature: str) -> list[str]:
    return sorted({str(row.get(feature, "")) for row in rows if str(row.get(feature, ""))})


def categorical_rule_bases(
    rows: list[dict[str, object]],
    *,
    include_candidate_label: bool,
) -> list[selector_sweep.RuleSpec]:
    rules: list[selector_sweep.RuleSpec] = [selector_sweep.RuleSpec()]
    seen = {rules[0].key()}
    non_candidate_features = tuple(selector_sweep.CATEGORICAL_FEATURES)

    for feature in categorical_features(include_candidate_label):
        for value in categorical_values_for_feature(rows, feature):
            rule = selector_sweep.RuleSpec(
                categorical=(selector_sweep.CategoricalCondition(feature, value),)
            )
            if rule.key() not in seen:
                rules.append(rule)
                seen.add(rule.key())

    if include_candidate_label:
        for candidate_label in categorical_values_for_feature(rows, "candidate_label"):
            candidate_condition = selector_sweep.CategoricalCondition(
                "candidate_label", candidate_label
            )
            candidate_rows = [
                row for row in rows if str(row.get("candidate_label", "")) == candidate_label
            ]
            for feature in non_candidate_features:
                for value in categorical_values_for_feature(candidate_rows, feature):
                    rule = selector_sweep.normalize_rule(
                        selector_sweep.RuleSpec(
                            categorical=(
                                candidate_condition,
                                selector_sweep.CategoricalCondition(feature, value),
                            )
                        )
                    )
                    if rule.key() not in seen:
                        rules.append(rule)
                        seen.add(rule.key())
    return rules


def categorical_index(
    rows: list[dict[str, object]],
) -> dict[tuple[str, str], list[dict[str, object]]]:
    indexed: dict[tuple[str, str], list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        for feature in (*selector_sweep.CATEGORICAL_FEATURES, "candidate_label"):
            value = str(row.get(feature, ""))
            if value:
                indexed[(feature, value)].append(row)
    return indexed


def indexed_rows_matching(
    rows: list[dict[str, object]],
    indexes: dict[tuple[str, str], list[dict[str, object]]],
    rule: selector_sweep.RuleSpec,
) -> list[dict[str, object]]:
    if rule.categorical:
        candidates = [
            indexes.get((condition.feature, condition.value), [])
            for condition in rule.categorical
        ]
        source = min(candidates, key=len) if candidates else []
    else:
        source = rows
    return [row for row in source if selector_sweep.matches_rule(row, rule)]


def priority_from_matching_rows(
    matching_rows: list[dict[str, object]],
    all_labels: list[str],
) -> list[str]:
    stats: dict[str, dict[str, float]] = defaultdict(
        lambda: {"net": 0.0, "gain": 0.0, "loss": 0.0, "matched": 0.0}
    )
    for row in matching_rows:
        label = str(row["candidate_label"])
        delta = float(row["score_delta_distance_m"])
        stats[label]["net"] += delta
        stats[label]["gain"] += max(0.0, delta)
        stats[label]["loss"] += min(0.0, delta)
        stats[label]["matched"] += 1.0
    ranked = sorted(
        stats,
        key=lambda label: (
            stats[label]["net"],
            stats[label]["gain"],
            stats[label]["loss"],
            -stats[label]["matched"],
            label,
        ),
        reverse=True,
    )
    seen = set(ranked)
    ranked.extend(label for label in all_labels if label not in seen)
    return ranked


def priority_for_rule(
    rows: list[dict[str, object]],
    rule: selector_sweep.RuleSpec,
) -> list[str]:
    matching_rows = [
        row for row in rows if selector_sweep.matches_rule(row, rule)
    ]
    return priority_from_matching_rows(matching_rows, candidate_labels(rows))


def group_rows(
    rows: list[dict[str, object]],
) -> dict[tuple[str, int], list[dict[str, object]]]:
    grouped: dict[tuple[str, int], list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        grouped[(str(row["run_label"]), int(row["reference_index"]))].append(row)
    return grouped


def score_matching_blind_selection(
    matching_rows: list[dict[str, object]],
    priority_order: list[str],
    *,
    rule_text: str,
    run_labels: list[str],
    include_by_run: bool = True,
) -> dict[str, object]:
    priority_index = {label: index for index, label in enumerate(priority_order)}
    grouped = group_rows(matching_rows)
    by_run: dict[str, dict[str, float | Counter[str]]] = {}
    selected_segments = 0
    selected_distance = 0.0
    selected_gain = 0.0
    selected_loss = 0.0
    selected_candidates: Counter[str] = Counter()

    for run_label in run_labels:
        by_run.setdefault(
            run_label,
            {
                "selected_segments": 0.0,
                "selected_gain": 0.0,
                "selected_loss": 0.0,
                "selected_candidates": Counter(),
            },
        )

    for (run_label, _), segment_rows in grouped.items():
        stats = by_run.setdefault(
            run_label,
            {
                "selected_segments": 0.0,
                "selected_gain": 0.0,
                "selected_loss": 0.0,
                "selected_candidates": Counter(),
            },
        )
        selected = min(
            segment_rows,
            key=lambda row: (
                priority_index.get(str(row["candidate_label"]), len(priority_index)),
                str(row["candidate_label"]),
            ),
        )
        delta = float(selected["score_delta_distance_m"])
        selected_segments += 1
        selected_distance += float(selected["segment_distance_m"] or 0.0)
        label = str(selected["candidate_label"])
        selected_candidates[label] += 1
        stats["selected_segments"] = float(stats["selected_segments"]) + 1.0
        candidate_counter = stats["selected_candidates"]
        assert isinstance(candidate_counter, Counter)
        candidate_counter[label] += 1
        if delta >= 0.0:
            selected_gain += delta
            stats["selected_gain"] = float(stats["selected_gain"]) + delta
        else:
            selected_loss += delta
            stats["selected_loss"] = float(stats["selected_loss"]) + delta

    run_rows: list[dict[str, object]] = []
    run_deltas: list[float] = []
    for run_label, stats in sorted(by_run.items()):
        gain = float(stats["selected_gain"])
        loss = float(stats["selected_loss"])
        net = gain + loss
        run_deltas.append(net)
        if include_by_run:
            counter = stats["selected_candidates"]
            assert isinstance(counter, Counter)
            run_rows.append(
                {
                    "run_label": run_label,
                    "selected_segments": int(stats["selected_segments"]),
                    "selected_score_delta_distance_m": rounded(net),
                    "selected_gain_distance_m": rounded(gain),
                    "selected_loss_distance_m": rounded(loss),
                    "selected_candidates": dict(counter.most_common(12)),
                }
            )

    selected_absolute = selected_gain + abs(selected_loss)
    return {
        "rule": rule_text,
        "candidate_priority": priority_order,
        "selected_segments": selected_segments,
        "selected_distance_m": rounded(selected_distance),
        "selected_score_delta_distance_m": rounded(selected_gain + selected_loss),
        "selected_gain_distance_m": rounded(selected_gain),
        "selected_loss_distance_m": rounded(selected_loss),
        "distance_precision_pct": rounded(
            100.0 * selected_gain / selected_absolute if selected_absolute > 0.0 else None
        ),
        "run_count": len(by_run),
        "negative_run_count": sum(1 for delta in run_deltas if delta < 0.0),
        "nonnegative_run_count": sum(1 for delta in run_deltas if delta >= 0.0),
        "min_run_score_delta_distance_m": rounded(min(run_deltas) if run_deltas else None),
        "max_run_score_delta_distance_m": rounded(max(run_deltas) if run_deltas else None),
        "selected_candidates": dict(selected_candidates.most_common(20)),
        "by_run": run_rows,
    }


def score_blind_selection(
    rows: list[dict[str, object]],
    rule: selector_sweep.RuleSpec,
    priority_order: list[str],
    *,
    include_by_run: bool = True,
) -> dict[str, object]:
    matching_rows = [
        row for row in rows if selector_sweep.matches_rule(row, rule)
    ]
    return score_matching_blind_selection(
        matching_rows,
        priority_order,
        rule_text=rule.expression(),
        run_labels=sorted({str(row["run_label"]) for row in rows}),
        include_by_run=include_by_run,
    )


def rank_key(summary: dict[str, object], rank_objective: str) -> tuple[float, ...]:
    net = float(summary["selected_score_delta_distance_m"] or 0.0)
    loss = float(summary["selected_loss_distance_m"] or 0.0)
    selected_segments = int(summary["selected_segments"])
    if rank_objective == "net":
        return (net, -abs(loss), -selected_segments)
    if rank_objective == "robust":
        return (
            -float(summary["negative_run_count"]),
            float(summary["min_run_score_delta_distance_m"] or 0.0),
            net,
            -abs(loss),
            -selected_segments,
        )
    raise ValueError(f"unsupported rank objective: {rank_objective}")


def ranked_rules(
    rows: list[dict[str, object]],
    *,
    max_thresholds: int,
    max_numeric_conditions: int,
    numeric_refinement_beam: int,
    min_selected_distance_m: float,
    preselect_rules: int,
    rule_source: str,
    include_candidate_label: bool,
    rank_objective: str,
    required_candidate_status: str | None,
    required_categorical: tuple[selector_sweep.CategoricalCondition, ...],
    required_numeric: tuple[selector_sweep.NumericCondition, ...],
) -> list[dict[str, object]]:
    rule_rows = (
        [row for row in rows if abs(float(row["score_delta_distance_m"])) > 1e-9]
        if rule_source == "changed"
        else rows
    )
    if not rule_rows:
        rule_rows = rows
    if max_numeric_conditions == 0:
        rules = categorical_rule_bases(
            rule_rows,
            include_candidate_label=include_candidate_label,
        )
    else:
        if include_candidate_label:
            raise SystemExit(
                "--include-candidate-label currently supports --max-numeric-conditions 0"
            )
        rules = selector_sweep.generate_rules(
            rule_rows,
            max_thresholds,
            max_numeric_conditions=max_numeric_conditions,
            numeric_refinement_beam=numeric_refinement_beam,
            rank_objective=rank_objective,
            required_candidate_status=required_candidate_status,
            required_categorical=required_categorical,
            required_numeric=required_numeric,
        )
    ranked: list[dict[str, object]] = []
    labels = candidate_labels(rows)
    run_labels = sorted({str(row["run_label"]) for row in rows})
    indexes = categorical_index(rows)
    eligible_rules: list[selector_sweep.RuleSpec] = []
    for rule in rules:
        if not selector_sweep.rule_satisfies_required_candidate_status(
            rule, required_candidate_status
        ):
            continue
        if not selector_sweep.rule_satisfies_required_categorical(
            rule, required_categorical
        ):
            continue
        if not selector_sweep.rule_satisfies_required_numeric(rule, required_numeric):
            continue
        eligible_rules.append(rule)

    if preselect_rules > 0 and len(eligible_rules) > preselect_rules:
        rule_indexes = categorical_index(rule_rows)
        prescored: list[tuple[dict[str, object], selector_sweep.RuleSpec]] = []
        for rule in eligible_rules:
            matching_rule_rows = indexed_rows_matching(rule_rows, rule_indexes, rule)
            if not matching_rule_rows:
                continue
            priority = priority_from_matching_rows(matching_rule_rows, labels)
            prescored.append(
                (
                    score_matching_blind_selection(
                        matching_rule_rows,
                        priority,
                        rule_text=rule.expression(),
                        run_labels=run_labels,
                        include_by_run=False,
                    ),
                    rule,
                )
            )
        prescored.sort(
            key=lambda item: rank_key(item[0], rank_objective),
            reverse=True,
        )
        eligible_rules = [rule for _, rule in prescored[:preselect_rules]]

    for rule in eligible_rules:
        matching_rows = indexed_rows_matching(rows, indexes, rule)
        priority = priority_from_matching_rows(matching_rows, labels)
        summary = score_matching_blind_selection(
            matching_rows,
            priority,
            rule_text=rule.expression(),
            run_labels=run_labels,
            include_by_run=True,
        )
        if float(summary["selected_distance_m"] or 0.0) < min_selected_distance_m:
            continue
        ranked.append(summary)
    ranked.sort(key=lambda summary: rank_key(summary, rank_objective), reverse=True)
    return ranked


def fold_rows(
    rows: list[dict[str, object]],
    holdout_run: str,
) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    return (
        [row for row in rows if str(row["run_label"]) != holdout_run],
        [row for row in rows if str(row["run_label"]) == holdout_run],
    )


def build_fold(
    rows: list[dict[str, object]],
    holdout_run: str,
    **kwargs: object,
) -> dict[str, object]:
    train_rows, holdout_rows = fold_rows(rows, holdout_run)
    if not train_rows or not holdout_rows:
        raise SystemExit(f"{holdout_run}: empty train or holdout split")
    ranked = ranked_rules(train_rows, **kwargs)
    best = ranked[0] if ranked else None
    if best is None or float(best["selected_score_delta_distance_m"] or 0.0) <= 0.0:
        rule = selector_sweep.RuleSpec()
        priority: list[str] = []
        learned_rule = "baseline_noop"
        train_score = {
            "selected_score_delta_distance_m": 0.0,
            "selected_gain_distance_m": 0.0,
            "selected_loss_distance_m": 0.0,
            "selected_segments": 0,
            "negative_run_count": 0,
            "min_run_score_delta_distance_m": 0.0,
            "selected_candidates": {},
        }
        holdout_score = dict(train_score)
    else:
        rule = parse_rule(str(best["rule"]))
        priority = list(best["candidate_priority"])
        learned_rule = str(best["rule"])
        train_score = best
        holdout_score = score_blind_selection(
            holdout_rows,
            rule,
            priority,
            include_by_run=False,
        )

    return {
        "holdout_run": holdout_run,
        "learned_rule": learned_rule,
        "candidate_priority": priority[:20],
        "train_evaluated_rules": len(ranked),
        "train_selected_score_delta_distance_m": train_score[
            "selected_score_delta_distance_m"
        ],
        "train_selected_gain_distance_m": train_score["selected_gain_distance_m"],
        "train_selected_loss_distance_m": train_score["selected_loss_distance_m"],
        "train_selected_segments": train_score["selected_segments"],
        "train_negative_run_count": train_score["negative_run_count"],
        "train_min_run_score_delta_distance_m": train_score[
            "min_run_score_delta_distance_m"
        ],
        "train_selected_candidates": train_score["selected_candidates"],
        "holdout_selected_score_delta_distance_m": holdout_score[
            "selected_score_delta_distance_m"
        ],
        "holdout_selected_gain_distance_m": holdout_score["selected_gain_distance_m"],
        "holdout_selected_loss_distance_m": holdout_score["selected_loss_distance_m"],
        "holdout_selected_segments": holdout_score["selected_segments"],
        "holdout_distance_precision_pct": holdout_score["distance_precision_pct"],
        "holdout_selected_candidates": holdout_score["selected_candidates"],
    }


def aggregate_folds(folds: list[dict[str, object]]) -> dict[str, object]:
    net = sum(float(fold["holdout_selected_score_delta_distance_m"]) for fold in folds)
    gain = sum(float(fold["holdout_selected_gain_distance_m"]) for fold in folds)
    loss = sum(float(fold["holdout_selected_loss_distance_m"]) for fold in folds)
    selected_abs = gain + abs(loss)
    negative_runs = [
        str(fold["holdout_run"])
        for fold in folds
        if float(fold["holdout_selected_score_delta_distance_m"]) < 0.0
    ]
    return {
        "fold_count": len(folds),
        "holdout_selected_score_delta_distance_m": rounded(net),
        "holdout_selected_gain_distance_m": rounded(gain),
        "holdout_selected_loss_distance_m": rounded(loss),
        "holdout_distance_precision_pct": rounded(
            100.0 * gain / selected_abs if selected_abs > 0.0 else None
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
    min_selected_distance_m: float,
    preselect_rules: int,
    rule_source: str,
    include_candidate_label: bool,
    rank_objective: str,
    required_candidate_status: str | None,
    required_categorical: tuple[selector_sweep.CategoricalCondition, ...],
    required_numeric: tuple[selector_sweep.NumericCondition, ...],
) -> dict[str, object]:
    run_labels = sorted({str(row["run_label"]) for row in rows})
    if len(run_labels) < 2:
        raise SystemExit("Leave-one-run-out validation needs at least two runs")
    kwargs = {
        "max_thresholds": max_thresholds,
        "max_numeric_conditions": max_numeric_conditions,
        "numeric_refinement_beam": numeric_refinement_beam,
        "min_selected_distance_m": min_selected_distance_m,
        "preselect_rules": preselect_rules,
        "rule_source": rule_source,
        "include_candidate_label": include_candidate_label,
        "rank_objective": rank_objective,
        "required_candidate_status": required_candidate_status,
        "required_categorical": required_categorical,
        "required_numeric": required_numeric,
    }
    folds = [build_fold(rows, run_label, **kwargs) for run_label in run_labels]
    all_ranked = ranked_rules(rows, **kwargs)
    return {
        "runs": run_labels,
        "row_count": len(rows),
        "candidate_count": len(candidate_labels(rows)),
        "reference_segment_groups": len(group_rows(rows)),
        "folds": folds,
        "aggregates": aggregate_folds(folds),
        "top_rules": all_ranked[:top_rules],
        "top_rules_requested": top_rules,
        "max_thresholds": max_thresholds,
        "max_numeric_conditions": max_numeric_conditions,
        "numeric_refinement_beam": numeric_refinement_beam,
        "min_selected_distance_m": min_selected_distance_m,
        "preselect_rules": preselect_rules,
        "rule_source": rule_source,
        "include_candidate_label": include_candidate_label,
        "rank_objective": rank_objective,
        "required_candidate_status": required_candidate_status,
        "required_categorical": [
            condition.expression() for condition in required_categorical
        ],
        "required_numeric": [condition.expression() for condition in required_numeric],
    }


def fmt(value: object, suffix: str = "") -> str:
    return selector_sweep.fmt(value, suffix)


def render_markdown(payload: dict[str, object]) -> str:
    aggregates = dict(payload["aggregates"])
    lines = [
        "# PPC Truth-Blind Segment Selector Leave-One-Run-Out",
        "",
        "Holdout selection uses only the learned rule and candidate priority; "
        "holdout score deltas are used only after selection for evaluation.",
        "",
        f"Rows: **{payload['row_count']}**",
        f"Candidates: **{payload['candidate_count']}**",
        f"Run/segment groups: **{payload['reference_segment_groups']}**",
        f"Folds: **{aggregates['fold_count']}**",
        (
            "Holdout selected net: "
            f"**{fmt(aggregates['holdout_selected_score_delta_distance_m'])} m**"
        ),
        (
            "Holdout gain/loss: "
            f"**{fmt(aggregates['holdout_selected_gain_distance_m'])} / "
            f"{fmt(aggregates['holdout_selected_loss_distance_m'])} m**"
        ),
        (
            "Non-negative holdout runs: "
            f"**{aggregates['nonnegative_holdout_runs']} / {aggregates['fold_count']}**"
        ),
        f"Ranking objective: **{payload['rank_objective']}**",
        f"Rule source: **{payload['rule_source']}**",
        f"Preselected rules: **{payload['preselect_rules']}**",
        f"Candidate-label rules: **{payload['include_candidate_label']}**",
        "",
        "| holdout | train net m | train min run m | train neg runs | holdout net m | holdout gain m | holdout loss m | selected | learned rule |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for fold in payload["folds"]:
        lines.append(
            f"| {fold['holdout_run']} | "
            f"{fmt(fold['train_selected_score_delta_distance_m'])} | "
            f"{fmt(fold['train_min_run_score_delta_distance_m'])} | "
            f"{fold['train_negative_run_count']} | "
            f"{fmt(fold['holdout_selected_score_delta_distance_m'])} | "
            f"{fmt(fold['holdout_selected_gain_distance_m'])} | "
            f"{fmt(fold['holdout_selected_loss_distance_m'])} | "
            f"{fold['holdout_selected_segments']} | "
            f"`{fold['learned_rule']}` |"
        )
    lines.extend(["", "## Top Full-Data Rules", ""])
    lines.append(
        "| rank | net m | gain m | loss m | neg runs | selected | rule | first priority |"
    )
    lines.append("|---:|---:|---:|---:|---:|---:|---|---|")
    for index, rule in enumerate(payload["top_rules"], start=1):
        priority = list(rule["candidate_priority"])
        lines.append(
            f"| {index} | {fmt(rule['selected_score_delta_distance_m'])} | "
            f"{fmt(rule['selected_gain_distance_m'])} | "
            f"{fmt(rule['selected_loss_distance_m'])} | "
            f"{rule['negative_run_count']} | {rule['selected_segments']} | "
            f"`{rule['rule']}` | `{priority[0] if priority else ''}` |"
        )
    return "\n".join(lines) + "\n"


def main() -> None:
    args = parse_args()
    if not args.segment_csv:
        raise SystemExit("At least one --segment-csv LABEL=CSV is required")
    specs = [selector_sweep.parse_segment_csv(value) for value in args.segment_csv]
    required_categorical = tuple(
        selector_sweep.parse_required_categorical(value)
        for value in args.required_categorical
    )
    required_numeric = tuple(
        selector_sweep.parse_required_numeric(value) for value in args.required_numeric
    )
    rows = selector_sweep.load_segment_rows(specs)
    payload = build_payload(
        rows,
        top_rules=args.top_rules,
        max_thresholds=args.max_thresholds,
        max_numeric_conditions=args.max_numeric_conditions,
        numeric_refinement_beam=args.numeric_refinement_beam,
        min_selected_distance_m=args.min_selected_distance_m,
        preselect_rules=args.preselect_rules,
        rule_source=args.rule_source,
        include_candidate_label=args.include_candidate_label,
        rank_objective=args.rank_objective,
        required_candidate_status=args.required_candidate_status,
        required_categorical=required_categorical,
        required_numeric=required_numeric,
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
