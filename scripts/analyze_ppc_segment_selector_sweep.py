#!/usr/bin/env python3
"""Sweep PPC segment-local selector rules from profile-delta CSVs."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path


CATEGORICAL_FEATURES = (
    "status_transition",
    "baseline_status_name",
    "candidate_status_name",
)
NUMERIC_FEATURES = (
    "candidate_ratio",
    "candidate_num_satellites",
    "candidate_rtk_update_observations",
    "candidate_rtk_update_suppressed_outliers",
    "candidate_rtk_update_prefit_residual_rms_m",
    "candidate_rtk_update_prefit_residual_max_m",
    "candidate_rtk_update_post_suppression_residual_rms_m",
    "candidate_rtk_update_post_suppression_residual_max_m",
    "baseline_ratio",
    "baseline_num_satellites",
)
BASE_NUMERIC_FIELDS = (
    "segment_distance_m",
    "score_delta_distance_m",
)


@dataclass(frozen=True)
class SegmentCsvSpec:
    label: str
    path: Path


@dataclass(frozen=True)
class CategoricalCondition:
    feature: str
    value: str

    def matches(self, row: dict[str, object]) -> bool:
        return str(row.get(self.feature, "")) == self.value

    def expression(self) -> str:
        return f"{self.feature} == {self.value}"


@dataclass(frozen=True)
class NumericCondition:
    feature: str
    operator: str
    threshold: float

    def matches(self, row: dict[str, object]) -> bool:
        value = row.get(self.feature)
        if value is None:
            return False
        numeric = float(value)
        if self.operator == "<=":
            return numeric <= self.threshold
        if self.operator == ">=":
            return numeric >= self.threshold
        raise ValueError(f"unsupported operator: {self.operator}")

    def expression(self) -> str:
        return f"{self.feature} {self.operator} {self.threshold:.6g}"


@dataclass(frozen=True)
class RuleSpec:
    categorical: tuple[CategoricalCondition, ...] = ()
    numeric: tuple[NumericCondition, ...] = ()

    def matches(self, row: dict[str, object]) -> bool:
        return all(condition.matches(row) for condition in self.categorical) and all(
            condition.matches(row) for condition in self.numeric
        )

    def expression(self) -> str:
        parts = [condition.expression() for condition in self.categorical]
        parts.extend(condition.expression() for condition in self.numeric)
        return "candidate_all" if not parts else " AND ".join(parts)

    def key(self) -> tuple[tuple[tuple[str, str], ...], tuple[tuple[str, str, float], ...]]:
        return (
            tuple((condition.feature, condition.value) for condition in self.categorical),
            tuple(
                (condition.feature, condition.operator, round(condition.threshold, 9))
                for condition in self.numeric
            ),
        )


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
        default=20,
        help="Number of ranked selector rules to include in Markdown and JSON (default: 20).",
    )
    parser.add_argument(
        "--max-thresholds",
        type=int,
        default=128,
        help="Maximum numeric thresholds to test per feature/category (default: 128).",
    )
    parser.add_argument(
        "--max-numeric-conditions",
        type=int,
        choices=(1, 2),
        default=1,
        help="Maximum numeric conditions per rule. Use 2 for one-step rule refinement (default: 1).",
    )
    parser.add_argument(
        "--numeric-refinement-beam",
        type=int,
        default=64,
        help="Top one-numeric rules per categorical base to refine when --max-numeric-conditions=2.",
    )
    parser.add_argument(
        "--numeric-threshold-refinement-beam",
        type=int,
        default=32,
        help=(
            "Top ranked rules whose numeric thresholds are rescored against exact local "
            "thresholds after the coarse sweep (default: 32)."
        ),
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


def parse_segment_csv(value: str) -> SegmentCsvSpec:
    label, sep, path_text = value.partition("=")
    if not sep or not label.strip() or not path_text.strip():
        raise SystemExit("--segment-csv must use LABEL=CSV")
    return SegmentCsvSpec(label=label.strip(), path=Path(path_text.strip()))


def rounded(value: float | None) -> float | None:
    return None if value is None else round(float(value), 6)


def parse_float(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    numeric = float(value)
    return numeric if math.isfinite(numeric) else None


def read_segment_csv(spec: SegmentCsvSpec) -> list[dict[str, object]]:
    if not spec.path.exists():
        raise SystemExit(f"{spec.label}: missing segment CSV: {spec.path}")
    rows: list[dict[str, object]] = []
    with spec.path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            row: dict[str, object] = dict(raw)
            row["run_label"] = spec.label
            for key in (*BASE_NUMERIC_FIELDS, *NUMERIC_FEATURES):
                row[key] = parse_float(raw.get(key))
            if row["score_delta_distance_m"] is None or row["segment_distance_m"] is None:
                raise SystemExit(f"{spec.label}: segment CSV has a row without score/distance")
            rows.append(row)
    return rows


def load_segment_rows(specs: list[SegmentCsvSpec]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for spec in specs:
        rows.extend(read_segment_csv(spec))
    return rows


def threshold_values(
    rows: list[dict[str, object]],
    feature: str,
    max_thresholds: int,
) -> list[float]:
    values = sorted(
        {
            float(row[feature])
            for row in rows
            if row.get(feature) is not None and math.isfinite(float(row[feature]))
        }
    )
    if max_thresholds <= 0 or len(values) <= max_thresholds:
        return values
    selected: list[float] = []
    for index in range(max_thresholds):
        source_index = round(index * (len(values) - 1) / max(max_thresholds - 1, 1))
        selected.append(values[source_index])
    return sorted(set(selected))


def categorical_values(rows: list[dict[str, object]], feature: str) -> list[str]:
    return sorted({str(row.get(feature, "")) for row in rows if str(row.get(feature, ""))})


def matches_rule(row: dict[str, object], rule: RuleSpec) -> bool:
    for condition in rule.categorical:
        if str(row.get(condition.feature, "")) != condition.value:
            return False
    for condition in rule.numeric:
        value = row.get(condition.feature)
        if value is None:
            return False
        numeric = float(value)
        if condition.operator == "<=":
            if numeric > condition.threshold:
                return False
        elif condition.operator == ">=":
            if numeric < condition.threshold:
                return False
        else:
            raise ValueError(f"unsupported operator: {condition.operator}")
    return True


def categorical_rule_bases(rows: list[dict[str, object]]) -> list[RuleSpec]:
    rules: list[RuleSpec] = [RuleSpec()]
    for feature in CATEGORICAL_FEATURES:
        for value in categorical_values(rows, feature):
            rules.append(RuleSpec(categorical=(CategoricalCondition(feature, value),)))
    return rules


def rows_matching(rows: list[dict[str, object]], rule: RuleSpec) -> list[dict[str, object]]:
    return [row for row in rows if matches_rule(row, rule)]


def dataset_gain_loss(rows: list[dict[str, object]]) -> tuple[float, float]:
    gain = sum(max(0.0, float(row["score_delta_distance_m"])) for row in rows)
    loss = sum(min(0.0, float(row["score_delta_distance_m"])) for row in rows)
    return gain, loss


def normalize_rule(rule: RuleSpec) -> RuleSpec:
    return RuleSpec(
        categorical=tuple(sorted(rule.categorical, key=lambda item: (item.feature, item.value))),
        numeric=tuple(
            sorted(
                rule.numeric,
                key=lambda item: (item.feature, item.operator, round(item.threshold, 9)),
            )
        ),
    )


def numeric_conditions(rows: list[dict[str, object]], max_thresholds: int) -> list[NumericCondition]:
    conditions: list[NumericCondition] = []
    for feature in NUMERIC_FEATURES:
        thresholds = threshold_values(rows, feature, max_thresholds)
        for threshold in thresholds:
            for operator in ("<=", ">="):
                conditions.append(NumericCondition(feature, operator, threshold))
    return conditions


def rule_rank_key(summary: dict[str, object]) -> tuple[float, float, float, int]:
    return (
        float(summary["selected_score_delta_distance_m"]),
        -abs(float(summary["selected_loss_distance_m"])),
        float(summary["gain_recall_pct"] or 0.0),
        -int(summary["selected_segments"]),
    )


def generate_rules(
    rows: list[dict[str, object]],
    max_thresholds: int,
    max_numeric_conditions: int = 1,
    numeric_refinement_beam: int = 64,
) -> list[RuleSpec]:
    rules: list[RuleSpec] = []
    seen: set[tuple[tuple[tuple[str, str], ...], tuple[tuple[str, str, float], ...]]] = set()
    bases = categorical_rule_bases(rows)
    for base in bases:
        base_rows = rows_matching(rows, base)
        if not base_rows:
            continue
        for rule in (base,):
            if rule.key() not in seen:
                rules.append(rule)
                seen.add(rule.key())
        one_numeric_rules: list[RuleSpec] = []
        for numeric in numeric_conditions(base_rows, max_thresholds):
            rule = normalize_rule(
                RuleSpec(
                    categorical=base.categorical,
                    numeric=(numeric,),
                )
            )
            one_numeric_rules.append(rule)
            if rule.key() in seen:
                continue
            rules.append(rule)
            seen.add(rule.key())
        if max_numeric_conditions < 2:
            continue

        scored_one_numeric: list[tuple[dict[str, object], RuleSpec]] = []
        for rule in one_numeric_rules:
            scored_one_numeric.append((score_rule(base_rows, rule, include_by_run=False), rule))
        scored_one_numeric.sort(key=lambda item: rule_rank_key(item[0]), reverse=True)
        for _, seed in scored_one_numeric[: max(0, numeric_refinement_beam)]:
            seed_rows = rows_matching(base_rows, seed)
            if not seed_rows:
                continue
            existing = set(seed.numeric)
            for extra in numeric_conditions(seed_rows, max_thresholds):
                if extra in existing:
                    continue
                rule = normalize_rule(
                    RuleSpec(
                        categorical=seed.categorical,
                        numeric=(*seed.numeric, extra),
                    )
                )
                if rule.key() in seen:
                    continue
                rules.append(rule)
                seen.add(rule.key())
    return rules


def rule_with_numeric_threshold(rule: RuleSpec, index: int, threshold: float) -> RuleSpec:
    numeric = list(rule.numeric)
    condition = numeric[index]
    numeric[index] = NumericCondition(condition.feature, condition.operator, threshold)
    return normalize_rule(RuleSpec(categorical=rule.categorical, numeric=tuple(numeric)))


def numeric_threshold_refinement_rules(
    rows: list[dict[str, object]],
    ranked: list[tuple[dict[str, object], RuleSpec]],
    beam: int,
) -> list[RuleSpec]:
    rules: list[RuleSpec] = []
    seen: set[tuple[tuple[tuple[str, str], ...], tuple[tuple[str, str, float], ...]]] = set()
    for _, rule in ranked[: max(0, beam)]:
        if not rule.numeric:
            continue
        for index, condition in enumerate(rule.numeric):
            fixed_numeric = tuple(
                other_condition
                for other_index, other_condition in enumerate(rule.numeric)
                if other_index != index
            )
            local_rows = rows_matching(
                rows,
                RuleSpec(categorical=rule.categorical, numeric=fixed_numeric),
            )
            for threshold in threshold_values(local_rows, condition.feature, 0):
                refined_rule = rule_with_numeric_threshold(rule, index, threshold)
                if refined_rule.key() in seen:
                    continue
                rules.append(refined_rule)
                seen.add(refined_rule.key())
    return rules


def score_rule(
    rows: list[dict[str, object]],
    rule: RuleSpec,
    *,
    totals: tuple[float, float] | None = None,
    include_by_run: bool = True,
) -> dict[str, object]:
    selected_segments = 0
    selected_distance = 0.0
    selected_gain = 0.0
    selected_loss = 0.0
    for row in rows:
        if not matches_rule(row, rule):
            continue
        selected_segments += 1
        selected_distance += float(row["segment_distance_m"])
        delta = float(row["score_delta_distance_m"])
        if delta >= 0.0:
            selected_gain += delta
        else:
            selected_loss += delta
    total_gain, total_loss = totals if totals is not None else dataset_gain_loss(rows)
    selected_absolute = selected_gain + abs(selected_loss)
    by_run: list[dict[str, object]] = []
    if include_by_run:
        for run_label in sorted({str(row["run_label"]) for row in rows}):
            run_rows = [row for row in rows if str(row["run_label"]) == run_label]
            run_selected = [row for row in run_rows if matches_rule(row, rule)]
            run_gain = sum(max(0.0, float(row["score_delta_distance_m"])) for row in run_selected)
            run_loss = sum(min(0.0, float(row["score_delta_distance_m"])) for row in run_selected)
            by_run.append(
                {
                    "run_label": run_label,
                    "selected_segments": len(run_selected),
                    "selected_score_delta_distance_m": rounded(run_gain + run_loss),
                    "selected_gain_distance_m": rounded(run_gain),
                    "selected_loss_distance_m": rounded(run_loss),
                    "candidate_all_delta_distance_m": rounded(
                        sum(float(row["score_delta_distance_m"]) for row in run_rows)
                    ),
                }
            )

    return {
        "rule": rule.expression(),
        "selected_segments": selected_segments,
        "selected_distance_m": rounded(selected_distance),
        "selected_score_delta_distance_m": rounded(selected_gain + selected_loss),
        "selected_gain_distance_m": rounded(selected_gain),
        "selected_loss_distance_m": rounded(selected_loss),
        "missed_gain_distance_m": rounded(total_gain - selected_gain),
        "avoided_loss_distance_m": rounded(abs(total_loss - selected_loss)),
        "distance_precision_pct": rounded(
            100.0 * selected_gain / selected_absolute if selected_absolute > 0.0 else None
        ),
        "gain_recall_pct": rounded(100.0 * selected_gain / total_gain if total_gain > 0.0 else None),
        "loss_exposure_pct": rounded(
            100.0 * abs(selected_loss) / abs(total_loss) if total_loss < 0.0 else None
        ),
        "by_run": by_run,
    }


def run_summaries(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    summaries: list[dict[str, object]] = []
    for run_label in sorted({str(row["run_label"]) for row in rows}):
        run_rows = [row for row in rows if str(row["run_label"]) == run_label]
        gain = sum(max(0.0, float(row["score_delta_distance_m"])) for row in run_rows)
        loss = sum(min(0.0, float(row["score_delta_distance_m"])) for row in run_rows)
        summaries.append(
            {
                "run_label": run_label,
                "changed_segments": len(run_rows),
                "candidate_all_delta_distance_m": rounded(gain + loss),
                "candidate_gain_distance_m": rounded(gain),
                "candidate_loss_distance_m": rounded(loss),
            }
        )
    return summaries


def build_payload(
    rows: list[dict[str, object]],
    *,
    top_rules: int,
    max_thresholds: int,
    max_numeric_conditions: int = 1,
    numeric_refinement_beam: int = 64,
    numeric_threshold_refinement_beam: int = 32,
    min_selected_distance_m: float = 0.0,
) -> dict[str, object]:
    if not rows:
        raise SystemExit("No segment-delta rows were loaded")
    rules = generate_rules(
        rows,
        max_thresholds,
        max_numeric_conditions=max_numeric_conditions,
        numeric_refinement_beam=numeric_refinement_beam,
    )
    totals = dataset_gain_loss(rows)
    ranked: list[tuple[dict[str, object], RuleSpec]] = []
    for rule in rules:
        summary = score_rule(rows, rule, totals=totals, include_by_run=False)
        if float(summary["selected_distance_m"] or 0.0) < min_selected_distance_m:
            continue
        ranked.append((summary, rule))
    ranked.sort(key=lambda item: rule_rank_key(item[0]), reverse=True)
    if numeric_threshold_refinement_beam > 0:
        seen = {rule.key() for _, rule in ranked}
        for rule in numeric_threshold_refinement_rules(
            rows,
            ranked,
            numeric_threshold_refinement_beam,
        ):
            if rule.key() in seen:
                continue
            summary = score_rule(rows, rule, totals=totals, include_by_run=False)
            if float(summary["selected_distance_m"] or 0.0) < min_selected_distance_m:
                continue
            ranked.append((summary, rule))
            seen.add(rule.key())
        ranked.sort(key=lambda item: rule_rank_key(item[0]), reverse=True)
    top_rule_summaries = [
        score_rule(rows, rule, totals=totals, include_by_run=True)
        for _, rule in ranked[:top_rules]
    ]
    candidate_all = score_rule(rows, RuleSpec(), totals=totals, include_by_run=True)
    baseline_noop = {
        "rule": "baseline_noop",
        "selected_segments": 0,
        "selected_distance_m": 0.0,
        "selected_score_delta_distance_m": 0.0,
        "selected_gain_distance_m": 0.0,
        "selected_loss_distance_m": 0.0,
        "missed_gain_distance_m": candidate_all["selected_gain_distance_m"],
        "avoided_loss_distance_m": abs(float(candidate_all["selected_loss_distance_m"])),
        "distance_precision_pct": None,
        "gain_recall_pct": 0.0,
        "loss_exposure_pct": 0.0,
        "by_run": [],
    }
    return {
        "changed_segments": len(rows),
        "runs": run_summaries(rows),
        "candidate_all": candidate_all,
        "baseline_noop": baseline_noop,
        "top_rules": top_rule_summaries,
        "evaluated_rules": len(ranked),
        "max_thresholds": max_thresholds,
        "max_numeric_conditions": max_numeric_conditions,
        "numeric_refinement_beam": numeric_refinement_beam,
        "numeric_threshold_refinement_beam": numeric_threshold_refinement_beam,
        "min_selected_distance_m": min_selected_distance_m,
    }


def fmt(value: object, suffix: str = "") -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.3f}{suffix}"
    return f"{value}{suffix}"


def render_rule_table(rules: list[dict[str, object]]) -> list[str]:
    lines = [
        "| rank | rule | net m | gain m | loss m | precision | gain recall | loss exposure | selected |",
        "|---:|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for index, rule in enumerate(rules, start=1):
        lines.append(
            f"| {index} | `{rule['rule']}` | "
            f"{fmt(rule['selected_score_delta_distance_m'])} | "
            f"{fmt(rule['selected_gain_distance_m'])} | "
            f"{fmt(rule['selected_loss_distance_m'])} | "
            f"{fmt(rule['distance_precision_pct'], '%')} | "
            f"{fmt(rule['gain_recall_pct'], '%')} | "
            f"{fmt(rule['loss_exposure_pct'], '%')} | "
            f"{rule['selected_segments']} |"
        )
    return lines


def render_by_run_table(rule: dict[str, object]) -> list[str]:
    rows = list(rule.get("by_run", []))
    if not rows:
        return ["No per-run rows."]
    lines = [
        "| run | selected net m | selected gain m | selected loss m | candidate-all net m | selected |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"| {row['run_label']} | "
            f"{fmt(row['selected_score_delta_distance_m'])} | "
            f"{fmt(row['selected_gain_distance_m'])} | "
            f"{fmt(row['selected_loss_distance_m'])} | "
            f"{fmt(row['candidate_all_delta_distance_m'])} | "
            f"{row['selected_segments']} |"
        )
    return lines


def render_markdown(payload: dict[str, object]) -> str:
    candidate_all = dict(payload["candidate_all"])
    top_rules = list(payload["top_rules"])
    best_rule = top_rules[0] if top_rules else None
    lines = [
        "# PPC Segment Selector Sweep",
        "",
        f"Changed segments: **{payload['changed_segments']}**",
        f"Evaluated rules: **{payload['evaluated_rules']}**",
        "",
        "## Candidate-All Baseline",
        "",
        "| rule | net m | gain m | loss m | precision | gain recall | loss exposure |",
        "|---|---:|---:|---:|---:|---:|---:|",
        (
            f"| `{candidate_all['rule']}` | "
            f"{fmt(candidate_all['selected_score_delta_distance_m'])} | "
            f"{fmt(candidate_all['selected_gain_distance_m'])} | "
            f"{fmt(candidate_all['selected_loss_distance_m'])} | "
            f"{fmt(candidate_all['distance_precision_pct'], '%')} | "
            f"{fmt(candidate_all['gain_recall_pct'], '%')} | "
            f"{fmt(candidate_all['loss_exposure_pct'], '%')} |"
        ),
        "",
        "## Top Rules",
        "",
    ]
    lines.extend(render_rule_table(top_rules))
    if best_rule is not None:
        lines.extend(["", "## Best Rule By Run", ""])
        lines.extend(render_by_run_table(best_rule))
    return "\n".join(lines) + "\n"


def main() -> None:
    args = parse_args()
    if not args.segment_csv:
        raise SystemExit("At least one --segment-csv LABEL=CSV is required")
    specs = [parse_segment_csv(value) for value in args.segment_csv]
    rows = load_segment_rows(specs)
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
