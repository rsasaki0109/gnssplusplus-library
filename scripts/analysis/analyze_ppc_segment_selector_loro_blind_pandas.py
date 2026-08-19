#!/usr/bin/env python3
"""Pandas-backed truth-blind LOO validation for PPC segment selectors."""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import sys

import pandas as pd


SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import analyze_ppc_segment_selector_sweep as selector_sweep  # noqa: E402


REQUIRED_COLUMNS = (
    "candidate_label",
    "reference_index",
    "segment_distance_m",
    "score_delta_distance_m",
    *selector_sweep.CATEGORICAL_FEATURES,
    *selector_sweep.NUMERIC_FEATURES,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--segment-csv", action="append", default=[], metavar="LABEL=CSV")
    parser.add_argument("--top-rules", type=int, default=12)
    parser.add_argument("--max-thresholds", type=int, default=8)
    parser.add_argument("--max-numeric-conditions", type=int, choices=(0, 1), default=1)
    parser.add_argument("--preselect-rules", type=int, default=64)
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
    parser.add_argument("--include-candidate-label", action="store_true")
    parser.add_argument(
        "--numeric-feature",
        action="append",
        choices=selector_sweep.NUMERIC_FEATURES,
        default=[],
        help="Restrict numeric rule generation to this feature. Repeat as needed.",
    )
    parser.add_argument(
        "--numeric-base-only",
        action="store_true",
        help="When numeric rules are enabled, combine numeric conditions only with candidate_all.",
    )
    parser.add_argument(
        "--exclude-candidate-all",
        action="store_true",
        help="Do not score the unconstrained candidate_all rule.",
    )
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--markdown-output", type=Path, default=None)
    return parser.parse_args()


def rounded(value: float | None) -> float | None:
    return None if value is None else round(float(value), 6)


def read_frame(specs: list[selector_sweep.SegmentCsvSpec]) -> pd.DataFrame:
    frames: list[pd.DataFrame] = []
    for spec in specs:
        if not spec.path.exists():
            raise SystemExit(f"{spec.label}: missing segment CSV: {spec.path}")
        frame = pd.read_csv(
            spec.path,
            usecols=lambda column: column in REQUIRED_COLUMNS,
            low_memory=False,
        )
        frame["run_label"] = spec.label
        frames.append(frame)
    if not frames:
        raise SystemExit("No segment CSVs loaded")
    frame = pd.concat(frames, ignore_index=True)
    for column in (
        "reference_index",
        "segment_distance_m",
        "score_delta_distance_m",
        *selector_sweep.NUMERIC_FEATURES,
    ):
        frame[column] = pd.to_numeric(frame[column], errors="coerce")
    for column in (
        "candidate_label",
        "run_label",
        *selector_sweep.CATEGORICAL_FEATURES,
    ):
        frame[column] = frame[column].fillna("").astype(str)
    frame["reference_index"] = frame["reference_index"].astype("int64")
    return frame


def categorical_features(include_candidate_label: bool) -> tuple[str, ...]:
    features = tuple(selector_sweep.CATEGORICAL_FEATURES)
    return (*features, "candidate_label") if include_candidate_label else features


def categorical_values(frame: pd.DataFrame, feature: str) -> list[str]:
    return sorted(value for value in frame[feature].dropna().astype(str).unique() if value)


def categorical_bases(
    frame: pd.DataFrame,
    *,
    include_candidate_label: bool,
) -> list[selector_sweep.RuleSpec]:
    rules = [selector_sweep.RuleSpec()]
    seen = {rules[0].key()}
    for feature in categorical_features(include_candidate_label):
        for value in categorical_values(frame, feature):
            rule = selector_sweep.RuleSpec(
                categorical=(selector_sweep.CategoricalCondition(feature, value),)
            )
            if rule.key() not in seen:
                rules.append(rule)
                seen.add(rule.key())
    if include_candidate_label:
        for label in categorical_values(frame, "candidate_label"):
            label_condition = selector_sweep.CategoricalCondition("candidate_label", label)
            label_frame = frame[frame["candidate_label"] == label]
            for feature in selector_sweep.CATEGORICAL_FEATURES:
                for value in categorical_values(label_frame, feature):
                    rule = selector_sweep.normalize_rule(
                        selector_sweep.RuleSpec(
                            categorical=(
                                label_condition,
                                selector_sweep.CategoricalCondition(feature, value),
                            )
                        )
                    )
                    if rule.key() not in seen:
                        rules.append(rule)
                        seen.add(rule.key())
    return rules


def threshold_values(series: pd.Series, max_thresholds: int) -> list[float]:
    values = sorted(series.dropna().astype(float).unique())
    values = [float(value) for value in values if math.isfinite(float(value))]
    if max_thresholds <= 0 or len(values) <= max_thresholds:
        return values
    selected: list[float] = []
    for index in range(max_thresholds):
        source_index = round(index * (len(values) - 1) / max(max_thresholds - 1, 1))
        selected.append(values[source_index])
    return sorted(set(selected))


def mask_for_rule(frame: pd.DataFrame, rule: selector_sweep.RuleSpec) -> pd.Series:
    mask = pd.Series(True, index=frame.index)
    for condition in rule.categorical:
        mask &= frame[condition.feature].astype(str) == condition.value
    for condition in rule.numeric:
        values = frame[condition.feature]
        if condition.operator == "<=":
            mask &= values <= condition.threshold
        elif condition.operator == ">=":
            mask &= values >= condition.threshold
        else:
            raise ValueError(f"unsupported operator: {condition.operator}")
    return mask.fillna(False)


def generate_rules(
    frame: pd.DataFrame,
    *,
    max_thresholds: int,
    max_numeric_conditions: int,
    include_candidate_label: bool,
    numeric_features: tuple[str, ...],
    numeric_base_only: bool,
    exclude_candidate_all: bool,
) -> list[selector_sweep.RuleSpec]:
    rules = categorical_bases(frame, include_candidate_label=include_candidate_label)
    if exclude_candidate_all:
        rules = [rule for rule in rules if rule.expression() != "candidate_all"]
    seen = {rule.key() for rule in rules}
    if max_numeric_conditions == 0:
        return rules

    bases = [selector_sweep.RuleSpec()] if numeric_base_only else list(rules)
    for base in bases:
        base_frame = frame[mask_for_rule(frame, base)] if base.categorical else frame
        if base_frame.empty:
            continue
        for feature in numeric_features:
            for threshold in threshold_values(base_frame[feature], max_thresholds):
                for operator in ("<=", ">="):
                    rule = selector_sweep.normalize_rule(
                        selector_sweep.RuleSpec(
                            categorical=base.categorical,
                            numeric=(
                                selector_sweep.NumericCondition(
                                    feature,
                                    operator,
                                    threshold,
                                ),
                            ),
                        )
                    )
                    if rule.key() not in seen:
                        rules.append(rule)
                        seen.add(rule.key())
    return rules


def priority_from_frame(frame: pd.DataFrame, all_labels: list[str]) -> list[str]:
    if frame.empty:
        return list(all_labels)
    grouped = frame.groupby("candidate_label", sort=False)["score_delta_distance_m"]
    stats = pd.DataFrame(
        {
            "net": grouped.sum(),
            "matched": grouped.size(),
            "gain": frame.assign(
                gain=frame["score_delta_distance_m"].clip(lower=0.0)
            )
            .groupby("candidate_label", sort=False)["gain"]
            .sum(),
            "loss": frame.assign(
                loss=frame["score_delta_distance_m"].clip(upper=0.0)
            )
            .groupby("candidate_label", sort=False)["loss"]
            .sum(),
        }
    )
    stats["label"] = stats.index.astype(str)
    stats = stats.sort_values(
        ["net", "gain", "loss", "matched", "label"],
        ascending=[False, False, False, True, False],
    )
    order = [str(label) for label in stats.index]
    seen = set(order)
    order.extend(label for label in all_labels if label not in seen)
    return order


def score_rule_frame(
    frame: pd.DataFrame,
    rule: selector_sweep.RuleSpec,
    *,
    priority_order: list[str] | None,
    run_labels: list[str],
    all_labels: list[str],
    include_by_run: bool,
) -> dict[str, object]:
    matched = frame[mask_for_rule(frame, rule)]
    if priority_order is None:
        priority_order = priority_from_frame(matched, all_labels)
    priority_index = {label: index for index, label in enumerate(priority_order)}

    if matched.empty:
        selected = matched
    else:
        selected = matched.assign(
            _priority=matched["candidate_label"].map(priority_index).fillna(
                len(priority_index)
            )
        )
        selected_index = selected.groupby(
            ["run_label", "reference_index"],
            sort=False,
        )["_priority"].idxmin()
        selected = selected.loc[selected_index]

    selected_gain = float(selected["score_delta_distance_m"].clip(lower=0.0).sum())
    selected_loss = float(selected["score_delta_distance_m"].clip(upper=0.0).sum())
    selected_distance = float(selected["segment_distance_m"].sum())
    selected_abs = selected_gain + abs(selected_loss)
    by_run: list[dict[str, object]] = []
    run_deltas: list[float] = []
    for run_label in run_labels:
        run_selected = selected[selected["run_label"] == run_label]
        gain = float(run_selected["score_delta_distance_m"].clip(lower=0.0).sum())
        loss = float(run_selected["score_delta_distance_m"].clip(upper=0.0).sum())
        net = gain + loss
        run_deltas.append(net)
        if include_by_run:
            by_run.append(
                {
                    "run_label": run_label,
                    "selected_segments": int(len(run_selected)),
                    "selected_score_delta_distance_m": rounded(net),
                    "selected_gain_distance_m": rounded(gain),
                    "selected_loss_distance_m": rounded(loss),
                    "selected_candidates": dict(
                        Counter(run_selected["candidate_label"]).most_common(12)
                    ),
                }
            )
    return {
        "rule": rule.expression(),
        "candidate_priority": priority_order,
        "selected_segments": int(len(selected)),
        "selected_distance_m": rounded(selected_distance),
        "selected_score_delta_distance_m": rounded(selected_gain + selected_loss),
        "selected_gain_distance_m": rounded(selected_gain),
        "selected_loss_distance_m": rounded(selected_loss),
        "distance_precision_pct": rounded(
            100.0 * selected_gain / selected_abs if selected_abs > 0.0 else None
        ),
        "run_count": len(run_labels),
        "negative_run_count": sum(1 for delta in run_deltas if delta < 0.0),
        "nonnegative_run_count": sum(1 for delta in run_deltas if delta >= 0.0),
        "min_run_score_delta_distance_m": rounded(min(run_deltas) if run_deltas else None),
        "max_run_score_delta_distance_m": rounded(max(run_deltas) if run_deltas else None),
        "selected_candidates": dict(Counter(selected["candidate_label"]).most_common(20)),
        "by_run": by_run,
    }


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


def ranked_rules(
    frame: pd.DataFrame,
    *,
    max_thresholds: int,
    max_numeric_conditions: int,
    preselect_rules: int,
    rule_source: str,
    rank_objective: str,
    include_candidate_label: bool,
    numeric_features: tuple[str, ...],
    numeric_base_only: bool,
    exclude_candidate_all: bool,
) -> list[dict[str, object]]:
    source = (
        frame[frame["score_delta_distance_m"].abs() > 1e-9]
        if rule_source == "changed"
        else frame
    )
    if source.empty:
        source = frame
    rules = generate_rules(
        source,
        max_thresholds=max_thresholds,
        max_numeric_conditions=max_numeric_conditions,
        include_candidate_label=include_candidate_label,
        numeric_features=numeric_features,
        numeric_base_only=numeric_base_only,
        exclude_candidate_all=exclude_candidate_all,
    )
    all_labels = sorted(frame["candidate_label"].unique())
    run_labels = sorted(frame["run_label"].unique())
    if preselect_rules > 0 and len(rules) > preselect_rules:
        prescored = [
            (
                score_rule_frame(
                    source,
                    rule,
                    priority_order=None,
                    run_labels=run_labels,
                    all_labels=all_labels,
                    include_by_run=False,
                ),
                rule,
            )
            for rule in rules
        ]
        prescored.sort(key=lambda item: rank_key(item[0], rank_objective), reverse=True)
        rules = [rule for _, rule in prescored[:preselect_rules]]

    scored = [
        score_rule_frame(
            frame,
            rule,
            priority_order=None,
            run_labels=run_labels,
            all_labels=all_labels,
            include_by_run=True,
        )
        for rule in rules
    ]
    scored.sort(key=lambda summary: rank_key(summary, rank_objective), reverse=True)
    return scored


def parse_rule(rule_text: str) -> selector_sweep.RuleSpec:
    text = rule_text.strip().strip("`")
    if not text or text == "candidate_all":
        return selector_sweep.RuleSpec()
    categorical: list[selector_sweep.CategoricalCondition] = []
    numeric: list[selector_sweep.NumericCondition] = []
    for part in [item.strip() for item in text.split(" AND ") if item.strip()]:
        if "==" in part:
            feature, value = [item.strip() for item in part.split("==", 1)]
            categorical.append(selector_sweep.CategoricalCondition(feature, value))
            continue
        operator = "<=" if "<=" in part else ">=" if ">=" in part else None
        if operator is None:
            raise SystemExit(f"Unsupported condition: {part}")
        feature, value = [item.strip() for item in part.split(operator, 1)]
        numeric.append(selector_sweep.NumericCondition(feature, operator, float(value)))
    return selector_sweep.normalize_rule(
        selector_sweep.RuleSpec(tuple(categorical), tuple(numeric))
    )


def build_fold(
    frame: pd.DataFrame,
    holdout_run: str,
    kwargs: dict[str, object],
) -> dict[str, object]:
    train = frame[frame["run_label"] != holdout_run]
    holdout = frame[frame["run_label"] == holdout_run]
    ranked = ranked_rules(train, **kwargs)
    best = ranked[0] if ranked else None
    if best is None or float(best["selected_score_delta_distance_m"] or 0.0) <= 0.0:
        return {
            "holdout_run": holdout_run,
            "learned_rule": "baseline_noop",
            "candidate_priority": [],
            "train_evaluated_rules": len(ranked),
            "train_selected_score_delta_distance_m": 0.0,
            "train_selected_gain_distance_m": 0.0,
            "train_selected_loss_distance_m": 0.0,
            "train_selected_segments": 0,
            "train_negative_run_count": 0,
            "train_min_run_score_delta_distance_m": 0.0,
            "train_selected_candidates": {},
            "holdout_selected_score_delta_distance_m": 0.0,
            "holdout_selected_gain_distance_m": 0.0,
            "holdout_selected_loss_distance_m": 0.0,
            "holdout_selected_segments": 0,
            "holdout_distance_precision_pct": None,
            "holdout_selected_candidates": {},
        }
    rule = parse_rule(str(best["rule"]))
    holdout_score = score_rule_frame(
        holdout,
        rule,
        priority_order=list(best["candidate_priority"]),
        run_labels=[holdout_run],
        all_labels=sorted(train["candidate_label"].unique()),
        include_by_run=False,
    )
    return {
        "holdout_run": holdout_run,
        "learned_rule": best["rule"],
        "candidate_priority": list(best["candidate_priority"])[:20],
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


def build_payload(frame: pd.DataFrame, args: argparse.Namespace) -> dict[str, object]:
    run_labels = sorted(frame["run_label"].unique())
    kwargs: dict[str, object] = {
        "max_thresholds": args.max_thresholds,
        "max_numeric_conditions": args.max_numeric_conditions,
        "preselect_rules": args.preselect_rules,
        "rule_source": args.rule_source,
        "rank_objective": args.rank_objective,
        "include_candidate_label": args.include_candidate_label,
        "numeric_features": tuple(args.numeric_feature or selector_sweep.NUMERIC_FEATURES),
        "numeric_base_only": args.numeric_base_only,
        "exclude_candidate_all": args.exclude_candidate_all,
    }
    folds = [build_fold(frame, run_label, kwargs) for run_label in run_labels]
    all_ranked = ranked_rules(frame, **kwargs)
    return {
        "runs": run_labels,
        "row_count": int(len(frame)),
        "candidate_count": int(frame["candidate_label"].nunique()),
        "reference_segment_groups": int(
            frame[["run_label", "reference_index"]].drop_duplicates().shape[0]
        ),
        "folds": folds,
        "aggregates": aggregate_folds(folds),
        "top_rules": all_ranked[: args.top_rules],
        "top_rules_requested": args.top_rules,
        **kwargs,
    }


def fmt(value: object) -> str:
    return selector_sweep.fmt(value)


def render_markdown(payload: dict[str, object]) -> str:
    aggregates = dict(payload["aggregates"])
    lines = [
        "# PPC Truth-Blind Segment Selector Leave-One-Run-Out",
        "",
        "Pandas engine. Holdout selection uses learned rule and priority only.",
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
        f"Max thresholds: **{payload['max_thresholds']}**",
        f"Max numeric conditions: **{payload['max_numeric_conditions']}**",
        f"Preselected rules: **{payload['preselect_rules']}**",
        f"Exclude candidate_all: **{payload['exclude_candidate_all']}**",
        f"Candidate-label rules: **{payload['include_candidate_label']}**",
        f"Numeric base only: **{payload['numeric_base_only']}**",
        f"Numeric features: **{', '.join(payload['numeric_features'])}**",
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
    frame = read_frame(specs)
    payload = build_payload(frame, args)
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
