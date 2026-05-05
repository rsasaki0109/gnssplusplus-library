#!/usr/bin/env python3
"""Streaming truth-blind LOO validation for large PPC segment CSV sets.

This evaluator is intentionally narrower than the exploratory pandas evaluator:
it supports categorical rules over status fields and streams CSV chunks so the
full active candidate pool does not need to fit in one DataFrame.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
import os
from pathlib import Path
import sys

import pandas as pd


SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import analyze_ppc_segment_selector_sweep as selector_sweep  # noqa: E402


CATEGORICAL_FEATURES = tuple(selector_sweep.CATEGORICAL_FEATURES)
USE_COLUMNS = (
    "candidate_label",
    "reference_index",
    "segment_distance_m",
    "score_delta_distance_m",
    *CATEGORICAL_FEATURES,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--segment-csv", action="append", default=[], metavar="LABEL=CSV")
    parser.add_argument("--top-rules", type=int, default=12)
    parser.add_argument("--chunksize", type=int, default=250_000)
    parser.add_argument(
        "--rule-source",
        choices=("changed", "all"),
        default="changed",
    )
    parser.add_argument(
        "--rank-objective",
        choices=selector_sweep.RANK_OBJECTIVES,
        default="robust",
    )
    parser.add_argument("--exclude-candidate-all", action="store_true")
    parser.add_argument("--skip-full-data-top", action="store_true")
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--markdown-output", type=Path, default=None)
    return parser.parse_args()


def rounded(value: float | None) -> float | None:
    return None if value is None else round(float(value), 6)


def iter_chunks(
    specs: list[selector_sweep.SegmentCsvSpec],
    *,
    chunksize: int,
) -> object:
    for spec in specs:
        if not spec.path.exists():
            raise SystemExit(f"{spec.label}: missing segment CSV: {spec.path}")
        for chunk in pd.read_csv(
            spec.path,
            usecols=lambda column: column in USE_COLUMNS,
            chunksize=chunksize,
            low_memory=False,
        ):
            chunk["run_label"] = spec.label
            chunk["reference_index"] = pd.to_numeric(
                chunk["reference_index"], errors="coerce"
            ).astype("int64")
            chunk["segment_distance_m"] = pd.to_numeric(
                chunk["segment_distance_m"], errors="coerce"
            ).fillna(0.0)
            chunk["score_delta_distance_m"] = pd.to_numeric(
                chunk["score_delta_distance_m"], errors="coerce"
            ).fillna(0.0)
            chunk["candidate_label"] = chunk["candidate_label"].fillna("").astype(str)
            for feature in CATEGORICAL_FEATURES:
                chunk[feature] = chunk[feature].fillna("").astype(str)
            yield chunk


def rule_key(feature: str | None, value: str | None) -> str:
    return "candidate_all" if feature is None else f"{feature} == {value}"


def discover_rules(
    specs: list[selector_sweep.SegmentCsvSpec],
    *,
    chunksize: int,
    rule_source: str,
    exclude_candidate_all: bool,
) -> list[tuple[str | None, str | None]]:
    values: dict[str, set[str]] = {feature: set() for feature in CATEGORICAL_FEATURES}
    for chunk in iter_chunks(specs, chunksize=chunksize):
        source = (
            chunk[chunk["score_delta_distance_m"].abs() > 1e-9]
            if rule_source == "changed"
            else chunk
        )
        for feature in CATEGORICAL_FEATURES:
            values[feature].update(value for value in source[feature].unique() if value)

    rules: list[tuple[str | None, str | None]] = []
    if not exclude_candidate_all:
        rules.append((None, None))
    for feature in CATEGORICAL_FEATURES:
        for value in sorted(values[feature]):
            rules.append((feature, value))
    if not rules:
        raise SystemExit("No categorical rules discovered")
    return rules


def run_labels_from_specs(specs: list[selector_sweep.SegmentCsvSpec]) -> list[str]:
    labels = sorted({spec.label for spec in specs})
    if len(labels) < 2:
        raise SystemExit("Leave-one-run-out validation needs at least two runs")
    return labels


def candidate_stats_for_fold(
    specs: list[selector_sweep.SegmentCsvSpec],
    rules: list[tuple[str | None, str | None]],
    *,
    holdout_run: str,
    chunksize: int,
) -> dict[str, dict[str, dict[str, float]]]:
    stats: dict[str, dict[str, dict[str, float]]] = {
        rule_key(feature, value): defaultdict(
            lambda: {"net": 0.0, "gain": 0.0, "loss": 0.0, "matched": 0.0}
        )
        for feature, value in rules
    }
    for chunk in iter_chunks(specs, chunksize=chunksize):
        train = chunk[chunk["run_label"] != holdout_run]
        if train.empty:
            continue
        for feature, value in rules:
            key = rule_key(feature, value)
            matched = train if feature is None else train[train[feature] == value]
            if matched.empty:
                continue
            grouped = matched.groupby("candidate_label", sort=False)[
                "score_delta_distance_m"
            ]
            net = grouped.sum()
            matched_count = grouped.size()
            gain = (
                matched.assign(_gain=matched["score_delta_distance_m"].clip(lower=0.0))
                .groupby("candidate_label", sort=False)["_gain"]
                .sum()
            )
            loss = (
                matched.assign(_loss=matched["score_delta_distance_m"].clip(upper=0.0))
                .groupby("candidate_label", sort=False)["_loss"]
                .sum()
            )
            for label, value_net in net.items():
                bucket = stats[key][str(label)]
                bucket["net"] += float(value_net)
                bucket["gain"] += float(gain.get(label, 0.0))
                bucket["loss"] += float(loss.get(label, 0.0))
                bucket["matched"] += float(matched_count.get(label, 0.0))
    return stats


def candidate_stats_by_run(
    specs: list[selector_sweep.SegmentCsvSpec],
    rules: list[tuple[str | None, str | None]],
    *,
    chunksize: int,
) -> dict[str, dict[str, dict[str, dict[str, float]]]]:
    stats: dict[str, dict[str, dict[str, dict[str, float]]]] = {
        rule_key(feature, value): defaultdict(
            lambda: defaultdict(
                lambda: {"net": 0.0, "gain": 0.0, "loss": 0.0, "matched": 0.0}
            )
        )
        for feature, value in rules
    }
    for chunk in iter_chunks(specs, chunksize=chunksize):
        for feature, value in rules:
            key = rule_key(feature, value)
            matched = chunk if feature is None else chunk[chunk[feature] == value]
            if matched.empty:
                continue
            grouped = matched.groupby(["run_label", "candidate_label"], sort=False)[
                "score_delta_distance_m"
            ]
            net = grouped.sum()
            matched_count = grouped.size()
            gain = (
                matched.assign(_gain=matched["score_delta_distance_m"].clip(lower=0.0))
                .groupby(["run_label", "candidate_label"], sort=False)["_gain"]
                .sum()
            )
            loss = (
                matched.assign(_loss=matched["score_delta_distance_m"].clip(upper=0.0))
                .groupby(["run_label", "candidate_label"], sort=False)["_loss"]
                .sum()
            )
            for (run_label, label), value_net in net.items():
                bucket = stats[key][str(run_label)][str(label)]
                bucket["net"] += float(value_net)
                bucket["gain"] += float(gain.get((run_label, label), 0.0))
                bucket["loss"] += float(loss.get((run_label, label), 0.0))
                bucket["matched"] += float(matched_count.get((run_label, label), 0.0))
    return stats


def priorities_for_all_folds(
    stats_by_run: dict[str, dict[str, dict[str, dict[str, float]]]],
    runs: list[str],
) -> dict[str, dict[str, list[str]]]:
    priorities: dict[str, dict[str, list[str]]] = {}
    for holdout_run in runs:
        fold_stats: dict[str, dict[str, dict[str, float]]] = {}
        for key, by_run in stats_by_run.items():
            merged: dict[str, dict[str, float]] = defaultdict(
                lambda: {"net": 0.0, "gain": 0.0, "loss": 0.0, "matched": 0.0}
            )
            for run_label, by_label in by_run.items():
                if run_label == holdout_run:
                    continue
                for label, values in by_label.items():
                    bucket = merged[label]
                    bucket["net"] += values["net"]
                    bucket["gain"] += values["gain"]
                    bucket["loss"] += values["loss"]
                    bucket["matched"] += values["matched"]
            fold_stats[key] = merged
        priorities[holdout_run] = priorities_from_stats(fold_stats)
    return priorities


def priorities_from_stats(
    stats: dict[str, dict[str, dict[str, float]]],
) -> dict[str, list[str]]:
    priorities: dict[str, list[str]] = {}
    for key, by_label in stats.items():
        priorities[key] = sorted(
            by_label,
            key=lambda label: (
                by_label[label]["net"],
                by_label[label]["gain"],
                by_label[label]["loss"],
                -by_label[label]["matched"],
                label,
            ),
            reverse=True,
        )
    return priorities


def init_selection() -> dict[str, object]:
    return {
        "priority": None,
        "delta": 0.0,
        "distance": 0.0,
        "label": "",
    }


def update_selection_map(
    selected: dict[tuple[str, int], dict[str, object]],
    frame: pd.DataFrame,
    priority_index: dict[str, int],
) -> None:
    if frame.empty:
        return
    rows = frame.assign(
        priority_rank=frame["candidate_label"].map(priority_index).fillna(
            len(priority_index)
        )
    )
    best_index = rows.groupby(
        ["run_label", "reference_index"],
        sort=False,
    )["priority_rank"].idxmin()
    rows = rows.loc[best_index]
    for row in rows.itertuples(index=False):
        row_key = (str(row.run_label), int(row.reference_index))
        priority = int(row.priority_rank)
        current = selected.get(row_key)
        if current is not None and current["priority"] is not None:
            if priority >= int(current["priority"]):
                continue
        selected[row_key] = {
            "priority": priority,
            "delta": float(row.score_delta_distance_m),
            "distance": float(row.segment_distance_m),
            "label": str(row.candidate_label),
        }


def score_fold_with_priorities(
    specs: list[selector_sweep.SegmentCsvSpec],
    rules: list[tuple[str | None, str | None]],
    priorities: dict[str, list[str]],
    *,
    holdout_run: str,
    chunksize: int,
) -> tuple[dict[str, dict[str, object]], dict[str, dict[str, object]]]:
    train_selected: dict[str, dict[tuple[str, int], dict[str, object]]] = {
        rule_key(feature, value): {} for feature, value in rules
    }
    holdout_selected: dict[str, dict[tuple[str, int], dict[str, object]]] = {
        rule_key(feature, value): {} for feature, value in rules
    }
    priority_indexes = {
        key: {label: index for index, label in enumerate(order)}
        for key, order in priorities.items()
    }

    for chunk in iter_chunks(specs, chunksize=chunksize):
        for feature, value in rules:
            key = rule_key(feature, value)
            if not priorities.get(key):
                continue
            matched = chunk if feature is None else chunk[chunk[feature] == value]
            if matched.empty:
                continue
            train = matched[matched["run_label"] != holdout_run]
            holdout = matched[matched["run_label"] == holdout_run]
            update_selection_map(train_selected[key], train, priority_indexes[key])
            update_selection_map(holdout_selected[key], holdout, priority_indexes[key])

    return (
        summarize_selection_maps(train_selected),
        summarize_selection_maps(holdout_selected),
    )


def score_all_folds_with_priorities(
    specs: list[selector_sweep.SegmentCsvSpec],
    rules: list[tuple[str | None, str | None]],
    priorities_by_fold: dict[str, dict[str, list[str]]],
    *,
    chunksize: int,
) -> tuple[
    dict[str, dict[str, dict[str, object]]],
    dict[str, dict[str, dict[str, object]]],
]:
    runs = sorted(priorities_by_fold)
    train_selected: dict[str, dict[str, dict[tuple[str, int], dict[str, object]]]] = {
        holdout: {rule_key(feature, value): {} for feature, value in rules}
        for holdout in runs
    }
    holdout_selected: dict[str, dict[str, dict[tuple[str, int], dict[str, object]]]] = {
        holdout: {rule_key(feature, value): {} for feature, value in rules}
        for holdout in runs
    }
    priority_indexes = {
        holdout: {
            key: {label: index for index, label in enumerate(order)}
            for key, order in priorities.items()
        }
        for holdout, priorities in priorities_by_fold.items()
    }

    for chunk in iter_chunks(specs, chunksize=chunksize):
        for feature, value in rules:
            key = rule_key(feature, value)
            matched = chunk if feature is None else chunk[chunk[feature] == value]
            if matched.empty:
                continue
            for holdout in runs:
                order = priority_indexes[holdout].get(key)
                if not order:
                    continue
                is_holdout = matched["run_label"] == holdout
                update_selection_map(
                    holdout_selected[holdout][key],
                    matched[is_holdout],
                    order,
                )
                update_selection_map(
                    train_selected[holdout][key],
                    matched[~is_holdout],
                    order,
                )

    return (
        {
            holdout: summarize_selection_maps(by_rule)
            for holdout, by_rule in train_selected.items()
        },
        {
            holdout: summarize_selection_maps(by_rule)
            for holdout, by_rule in holdout_selected.items()
        },
    )


def summarize_selection_maps(
    selections: dict[str, dict[tuple[str, int], dict[str, object]]],
) -> dict[str, dict[str, object]]:
    summaries: dict[str, dict[str, object]] = {}
    for key, selected in selections.items():
        gain = 0.0
        loss = 0.0
        distance = 0.0
        by_run: dict[str, float] = defaultdict(float)
        candidates: Counter[str] = Counter()
        for (run_label, _), row in selected.items():
            delta = float(row["delta"])
            distance += float(row["distance"])
            by_run[run_label] += delta
            candidates[str(row["label"])] += 1
            if delta >= 0.0:
                gain += delta
            else:
                loss += delta
        run_deltas = list(by_run.values()) or [0.0]
        absolute = gain + abs(loss)
        summaries[key] = {
            "rule": key,
            "selected_segments": len(selected),
            "selected_distance_m": rounded(distance),
            "selected_score_delta_distance_m": rounded(gain + loss),
            "selected_gain_distance_m": rounded(gain),
            "selected_loss_distance_m": rounded(loss),
            "distance_precision_pct": rounded(
                100.0 * gain / absolute if absolute > 0.0 else None
            ),
            "negative_run_count": sum(1 for delta in run_deltas if delta < 0.0),
            "nonnegative_run_count": sum(1 for delta in run_deltas if delta >= 0.0),
            "min_run_score_delta_distance_m": rounded(min(run_deltas)),
            "max_run_score_delta_distance_m": rounded(max(run_deltas)),
            "selected_candidates": dict(candidates.most_common(20)),
        }
    return summaries


def rank_key(summary: dict[str, object], objective: str) -> tuple[float, ...]:
    net = float(summary["selected_score_delta_distance_m"] or 0.0)
    loss = float(summary["selected_loss_distance_m"] or 0.0)
    selected_segments = int(summary["selected_segments"])
    if objective == "net":
        return (net, -abs(loss), -selected_segments)
    if objective == "robust":
        return (
            -float(summary["negative_run_count"]),
            float(summary["min_run_score_delta_distance_m"] or 0.0),
            net,
            -abs(loss),
            -selected_segments,
        )
    raise ValueError(f"unsupported rank objective: {objective}")


def build_fold(
    specs: list[selector_sweep.SegmentCsvSpec],
    rules: list[tuple[str | None, str | None]],
    holdout_run: str,
    *,
    chunksize: int,
    rank_objective: str,
) -> dict[str, object]:
    stats = candidate_stats_for_fold(
        specs,
        rules,
        holdout_run=holdout_run,
        chunksize=chunksize,
    )
    priorities = priorities_from_stats(stats)
    train_scores, holdout_scores = score_fold_with_priorities(
        specs,
        rules,
        priorities,
        holdout_run=holdout_run,
        chunksize=chunksize,
    )
    ranked = sorted(
        train_scores.values(),
        key=lambda summary: rank_key(summary, rank_objective),
        reverse=True,
    )
    best = ranked[0]
    holdout = holdout_scores[best["rule"]]
    return {
        "holdout_run": holdout_run,
        "learned_rule": best["rule"],
        "candidate_priority": priorities.get(str(best["rule"]), [])[:20],
        "train_evaluated_rules": len(ranked),
        "train_selected_score_delta_distance_m": best[
            "selected_score_delta_distance_m"
        ],
        "train_selected_gain_distance_m": best["selected_gain_distance_m"],
        "train_selected_loss_distance_m": best["selected_loss_distance_m"],
        "train_selected_segments": best["selected_segments"],
        "train_negative_run_count": best["negative_run_count"],
        "train_min_run_score_delta_distance_m": best["min_run_score_delta_distance_m"],
        "train_selected_candidates": best["selected_candidates"],
        "holdout_selected_score_delta_distance_m": holdout[
            "selected_score_delta_distance_m"
        ],
        "holdout_selected_gain_distance_m": holdout["selected_gain_distance_m"],
        "holdout_selected_loss_distance_m": holdout["selected_loss_distance_m"],
        "holdout_selected_segments": holdout["selected_segments"],
        "holdout_distance_precision_pct": holdout["distance_precision_pct"],
        "holdout_selected_candidates": holdout["selected_candidates"],
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


def full_data_top_rules(
    specs: list[selector_sweep.SegmentCsvSpec],
    rules: list[tuple[str | None, str | None]],
    *,
    chunksize: int,
    rank_objective: str,
    top_rules: int,
) -> list[dict[str, object]]:
    stats = candidate_stats_for_fold(
        specs,
        rules,
        holdout_run="__NO_HOLDOUT__",
        chunksize=chunksize,
    )
    priorities = priorities_from_stats(stats)
    train_scores, _ = score_fold_with_priorities(
        specs,
        rules,
        priorities,
        holdout_run="__NO_HOLDOUT__",
        chunksize=chunksize,
    )
    ranked = sorted(
        train_scores.values(),
        key=lambda summary: rank_key(summary, rank_objective),
        reverse=True,
    )
    top = []
    for summary in ranked[:top_rules]:
        item = dict(summary)
        item["candidate_priority"] = priorities.get(str(summary["rule"]), [])[:20]
        top.append(item)
    return top


def build_payload(
    specs: list[selector_sweep.SegmentCsvSpec],
    args: argparse.Namespace,
) -> dict[str, object]:
    rules = discover_rules(
        specs,
        chunksize=args.chunksize,
        rule_source=args.rule_source,
        exclude_candidate_all=args.exclude_candidate_all,
    )
    runs = run_labels_from_specs(specs)
    stats_by_run = candidate_stats_by_run(specs, rules, chunksize=args.chunksize)
    priorities_by_fold = priorities_for_all_folds(stats_by_run, runs)
    train_scores_by_fold, holdout_scores_by_fold = score_all_folds_with_priorities(
        specs,
        rules,
        priorities_by_fold,
        chunksize=args.chunksize,
    )
    folds = []
    for run_label in runs:
        ranked = sorted(
            train_scores_by_fold[run_label].values(),
            key=lambda summary: rank_key(summary, args.rank_objective),
            reverse=True,
        )
        best = ranked[0]
        holdout = holdout_scores_by_fold[run_label][best["rule"]]
        folds.append(
            {
                "holdout_run": run_label,
                "learned_rule": best["rule"],
                "candidate_priority": priorities_by_fold[run_label].get(
                    str(best["rule"]), []
                )[:20],
                "train_evaluated_rules": len(ranked),
                "train_selected_score_delta_distance_m": best[
                    "selected_score_delta_distance_m"
                ],
                "train_selected_gain_distance_m": best["selected_gain_distance_m"],
                "train_selected_loss_distance_m": best["selected_loss_distance_m"],
                "train_selected_segments": best["selected_segments"],
                "train_negative_run_count": best["negative_run_count"],
                "train_min_run_score_delta_distance_m": best[
                    "min_run_score_delta_distance_m"
                ],
                "train_selected_candidates": best["selected_candidates"],
                "holdout_selected_score_delta_distance_m": holdout[
                    "selected_score_delta_distance_m"
                ],
                "holdout_selected_gain_distance_m": holdout[
                    "selected_gain_distance_m"
                ],
                "holdout_selected_loss_distance_m": holdout[
                    "selected_loss_distance_m"
                ],
                "holdout_selected_segments": holdout["selected_segments"],
                "holdout_distance_precision_pct": holdout["distance_precision_pct"],
                "holdout_selected_candidates": holdout["selected_candidates"],
            }
        )
    return {
        "runs": runs,
        "rule_count": len(rules),
        "folds": folds,
        "aggregates": aggregate_folds(folds),
        "top_rules": []
        if args.skip_full_data_top
        else full_data_top_rules(
            specs,
            rules,
            chunksize=args.chunksize,
            rank_objective=args.rank_objective,
            top_rules=args.top_rules,
        ),
        "top_rules_requested": args.top_rules,
        "chunksize": args.chunksize,
        "rule_source": args.rule_source,
        "rank_objective": args.rank_objective,
        "exclude_candidate_all": args.exclude_candidate_all,
        "skip_full_data_top": args.skip_full_data_top,
    }


def fmt(value: object) -> str:
    return selector_sweep.fmt(value)


def render_markdown(payload: dict[str, object]) -> str:
    aggregates = dict(payload["aggregates"])
    lines = [
        "# PPC Truth-Blind Segment Selector Leave-One-Run-Out",
        "",
        "Streaming categorical engine. Holdout selection uses learned rule and priority only.",
        "",
        f"Rules: **{payload['rule_count']}**",
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
        f"Exclude candidate_all: **{payload['exclude_candidate_all']}**",
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
        priority = list(rule.get("candidate_priority", []))
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
    payload = build_payload(specs, args)
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
