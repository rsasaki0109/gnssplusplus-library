#!/usr/bin/env python3
"""Analyze residual PPC wrong-FIX epochs in deployable selector outputs."""

from __future__ import annotations

import argparse
import csv
from collections import Counter
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Callable, Iterable


RUNS: tuple[tuple[str, str], ...] = (
    ("tokyo_run1", "tokyo/run1"),
    ("tokyo_run2", "tokyo/run2"),
    ("tokyo_run3", "tokyo/run3"),
    ("nagoya_run1", "nagoya/run1"),
    ("nagoya_run2", "nagoya/run2"),
    ("nagoya_run3", "nagoya/run3"),
)


@dataclass(frozen=True)
class ProfileSpec:
    name: str
    solution_dir: Path
    segments_dir: Path


@dataclass(frozen=True)
class SolutionEpoch:
    week: int
    tow_s: float
    ecef: tuple[float, float, float]
    status: int
    nsat: int | None
    ratio: float | None
    baseline_m: float | None
    outliers: int | None
    prefit_rms_m: float | None
    prefit_max_m: float | None
    post_rms_m: float | None
    post_max_m: float | None
    nis_per_obs: float | None


@dataclass(frozen=True)
class Gate:
    name: str
    predicate: Callable[[SolutionEpoch], bool]


GATES: tuple[Gate, ...] = (
    Gate("ratio < 6", lambda epoch: finite_lt(epoch.ratio, 6.0)),
    Gate("ratio < 8", lambda epoch: finite_lt(epoch.ratio, 8.0)),
    Gate("post_rms > 4 m", lambda epoch: finite_gt(epoch.post_rms_m, 4.0)),
    Gate("post_rms > 6 m", lambda epoch: finite_gt(epoch.post_rms_m, 6.0)),
    Gate("nis_per_obs > 20", lambda epoch: finite_gt(epoch.nis_per_obs, 20.0)),
    Gate("nis_per_obs > 50", lambda epoch: finite_gt(epoch.nis_per_obs, 50.0)),
    Gate("prefit_rms > 5 m", lambda epoch: finite_gt(epoch.prefit_rms_m, 5.0)),
    Gate(
        "baseline > 7000 m and post_rms > 4 m",
        lambda epoch: finite_gt(epoch.baseline_m, 7000.0) and finite_gt(epoch.post_rms_m, 4.0),
    ),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument(
        "--profile",
        action="append",
        required=True,
        metavar="NAME=SOLUTION_DIR",
        help="Profile label and directory containing {run_key}.pos and optional {run_key}_segments.csv.",
    )
    parser.add_argument(
        "--wrong-fix-threshold-m",
        type=float,
        default=0.50,
        help="Classify status=FIXED epochs above this 3D truth error as wrong FIX.",
    )
    parser.add_argument("--summary-json", type=Path, required=True)
    parser.add_argument("--markdown-output", type=Path, required=True)
    return parser.parse_args()


def finite_gt(value: float | None, threshold: float) -> bool:
    return value is not None and math.isfinite(value) and value > threshold


def finite_lt(value: float | None, threshold: float) -> bool:
    return value is not None and math.isfinite(value) and value < threshold


def maybe_float(parts: list[str], index: int) -> float | None:
    if index >= len(parts):
        return None
    try:
        value = float(parts[index])
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def maybe_int(parts: list[str], index: int) -> int | None:
    if index >= len(parts):
        return None
    try:
        return int(parts[index])
    except ValueError:
        return None


def parse_profile(spec: str) -> ProfileSpec:
    name, separator, path_text = spec.partition("=")
    if not separator or not name.strip() or not path_text.strip():
        raise SystemExit("--profile must use NAME=SOLUTION_DIR")
    solution_dir = Path(path_text)
    return ProfileSpec(name=name.strip(), solution_dir=solution_dir, segments_dir=solution_dir)


def load_reference(path: Path) -> dict[tuple[int, float], tuple[float, float, float]]:
    records: dict[tuple[int, float], tuple[float, float, float]] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                records[(int(row["GPS Week"]), round(float(row["GPS TOW (s)"]), 3))] = (
                    float(row["ECEF X (m)"]),
                    float(row["ECEF Y (m)"]),
                    float(row["ECEF Z (m)"]),
                )
            except (KeyError, ValueError):
                continue
    return records


def load_solution(path: Path) -> list[SolutionEpoch]:
    records: list[SolutionEpoch] = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 9:
                continue
            try:
                records.append(
                    SolutionEpoch(
                        week=int(parts[0]),
                        tow_s=round(float(parts[1]), 3),
                        ecef=(float(parts[2]), float(parts[3]), float(parts[4])),
                        status=int(parts[8]),
                        nsat=maybe_int(parts, 9),
                        ratio=maybe_float(parts, 11),
                        baseline_m=maybe_float(parts, 12),
                        outliers=maybe_int(parts, 17),
                        prefit_rms_m=maybe_float(parts, 18),
                        prefit_max_m=maybe_float(parts, 19),
                        post_rms_m=maybe_float(parts, 20),
                        post_max_m=maybe_float(parts, 21),
                        nis_per_obs=maybe_float(parts, 23),
                    )
                )
            except ValueError:
                continue
    return records


def load_segments(path: Path) -> dict[float, dict[str, str]]:
    if not path.exists():
        return {}
    records: dict[float, dict[str, str]] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                tow_s = round(float(row["selected_solution_tow_s"]), 3)
            except (KeyError, ValueError):
                continue
            records[tow_s] = row
    return records


def error_3d_m(lhs: tuple[float, float, float], rhs: tuple[float, float, float]) -> float:
    return math.sqrt(
        (lhs[0] - rhs[0]) ** 2
        + (lhs[1] - rhs[1]) ** 2
        + (lhs[2] - rhs[2]) ** 2
    )


def percentile(values: list[float], p: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int(round((p / 100.0) * (len(ordered) - 1)))))
    return round(ordered[index], 6)


def summarize_values(values: Iterable[float | int | None]) -> dict[str, float | None]:
    finite = [float(value) for value in values if value is not None and math.isfinite(float(value))]
    return {
        "p50": percentile(finite, 50.0),
        "p90": percentile(finite, 90.0),
        "p95": percentile(finite, 95.0),
    }


def rule_matched_label(row: dict[str, str] | None) -> str:
    if row is None:
        return "unknown"
    return "true" if row.get("rule_matched") == "True" else "false"


def selected_candidate_label(row: dict[str, str] | None) -> str:
    if row is None:
        return "unknown"
    return row.get("selected_candidate") or "unknown"


def longest_spans(epochs: list[SolutionEpoch], wrong_keys: set[tuple[int, float]]) -> list[dict[str, Any]]:
    spans: list[dict[str, Any]] = []
    current: list[SolutionEpoch] = []
    for epoch in epochs:
        is_wrong = (epoch.week, epoch.tow_s) in wrong_keys
        if is_wrong:
            if current and (epoch.week != current[-1].week or epoch.tow_s - current[-1].tow_s > 0.25):
                spans.append(span_payload(current))
                current = []
            current.append(epoch)
        elif current:
            spans.append(span_payload(current))
            current = []
    if current:
        spans.append(span_payload(current))
    spans.sort(key=lambda span: (float(span["duration_s"]), int(span["epochs"])), reverse=True)
    return spans[:5]


def span_payload(epochs: list[SolutionEpoch]) -> dict[str, Any]:
    start = epochs[0].tow_s
    end = epochs[-1].tow_s
    return {
        "start_tow_s": start,
        "end_tow_s": end,
        "duration_s": round(end - start + 0.2, 3),
        "epochs": len(epochs),
    }


def gate_summary(good: list[SolutionEpoch], wrong: list[SolutionEpoch]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for gate in GATES:
        wrong_hit = sum(1 for epoch in wrong if gate.predicate(epoch))
        good_hit = sum(1 for epoch in good if gate.predicate(epoch))
        rows.append(
            {
                "gate": gate.name,
                "wrong_caught": wrong_hit,
                "wrong_caught_pct": round(100.0 * wrong_hit / len(wrong), 6) if wrong else None,
                "good_harmed": good_hit,
                "good_harmed_pct": round(100.0 * good_hit / len(good), 6) if good else None,
            }
        )
    return rows


def analyze_profile(profile: ProfileSpec, dataset_root: Path, wrong_fix_threshold_m: float) -> dict[str, Any]:
    total_source_counts: Counter[str] = Counter()
    total_rule_counts: Counter[str] = Counter()
    total_fix = 0
    total_wrong = 0
    good_epochs: list[SolutionEpoch] = []
    wrong_epochs: list[SolutionEpoch] = []
    run_rows: list[dict[str, Any]] = []

    for run_key, run_relpath in RUNS:
        reference_path = dataset_root / run_relpath / "reference.csv"
        solution_path = profile.solution_dir / f"{run_key}.pos"
        if not reference_path.exists() or not solution_path.exists():
            run_rows.append({"key": run_key, "available": False})
            continue
        reference = load_reference(reference_path)
        solution = load_solution(solution_path)
        segments = load_segments(profile.segments_dir / f"{run_key}_segments.csv")
        run_source_counts: Counter[str] = Counter()
        run_rule_counts: Counter[str] = Counter()
        run_fix = 0
        run_wrong = 0
        wrong_keys: set[tuple[int, float]] = set()
        run_wrong_errors: list[float] = []

        for epoch in solution:
            if epoch.status != 4:
                continue
            truth = reference.get((epoch.week, epoch.tow_s))
            if truth is None:
                continue
            run_fix += 1
            error_m = error_3d_m(epoch.ecef, truth)
            if error_m > wrong_fix_threshold_m:
                run_wrong += 1
                run_wrong_errors.append(error_m)
                wrong_keys.add((epoch.week, epoch.tow_s))
                wrong_epochs.append(epoch)
                segment_row = segments.get(epoch.tow_s)
                run_source_counts[selected_candidate_label(segment_row)] += 1
                run_rule_counts[rule_matched_label(segment_row)] += 1
            else:
                good_epochs.append(epoch)

        total_fix += run_fix
        total_wrong += run_wrong
        total_source_counts.update(run_source_counts)
        total_rule_counts.update(run_rule_counts)
        run_rows.append(
            {
                "key": run_key,
                "available": True,
                "fixed": run_fix,
                "wrong_fix": run_wrong,
                "wrong_fix_rate_pct": round(100.0 * run_wrong / run_fix, 6) if run_fix else None,
                "wrong_error_p95_m": percentile(run_wrong_errors, 95.0),
                "selected_candidate_counts": dict(sorted(run_source_counts.items())),
                "rule_matched_counts": dict(sorted(run_rule_counts.items())),
                "longest_wrong_spans": longest_spans(solution, wrong_keys),
            }
        )

    metrics = {
        "ratio": "ratio",
        "baseline_m": "baseline_m",
        "nsat": "nsat",
        "prefit_rms_m": "prefit_rms_m",
        "prefit_max_m": "prefit_max_m",
        "post_rms_m": "post_rms_m",
        "post_max_m": "post_max_m",
        "nis_per_obs": "nis_per_obs",
        "outliers": "outliers",
    }
    discriminator_rows = []
    for label, attr in metrics.items():
        discriminator_rows.append(
            {
                "metric": label,
                "good_fix": summarize_values(getattr(epoch, attr) for epoch in good_epochs),
                "wrong_fix": summarize_values(getattr(epoch, attr) for epoch in wrong_epochs),
            }
        )

    return {
        "name": profile.name,
        "solution_dir": str(profile.solution_dir),
        "wrong_fix_threshold_m": wrong_fix_threshold_m,
        "fixed": total_fix,
        "wrong_fix": total_wrong,
        "wrong_fix_rate_pct": round(100.0 * total_wrong / total_fix, 6) if total_fix else None,
        "selected_candidate_counts": dict(sorted(total_source_counts.items())),
        "rule_matched_counts": dict(sorted(total_rule_counts.items())),
        "runs": run_rows,
        "discriminators": discriminator_rows,
        "gate_simulation": gate_summary(good_epochs, wrong_epochs),
    }


def fmt(value: object, suffix: str = "") -> str:
    if not isinstance(value, (int, float)):
        return "n/a"
    return f"{float(value):.3f}{suffix}"


def render_markdown(payload: dict[str, Any]) -> str:
    lines = [
        "# PPC residual wrong-FIX analysis",
        "",
        (
            "Reference truth is used only after the run to label wrong FIX; "
            "the source and gate columns use data already present in the real-time POS/selector outputs."
        ),
        "",
        f"Wrong-FIX threshold: **{fmt(payload['wrong_fix_threshold_m'], ' m')}** 3D error.",
        "",
        "## Key findings",
        "",
    ]
    for profile in payload["profiles"]:
        source_counts = profile["selected_candidate_counts"]
        rule_counts = profile["rule_matched_counts"]
        wrong_fix = profile["wrong_fix"]
        if source_counts.get("baseline") == wrong_fix and rule_counts.get("false") == wrong_fix:
            lines.append(
                f"- `{profile['name']}` residual wrong FIX is all baseline-selected "
                "and did not match a selector rule."
            )
        else:
            source = ", ".join(f"{key}={value}" for key, value in source_counts.items()) or "n/a"
            rules = ", ".join(f"{key}={value}" for key, value in rule_counts.items()) or "n/a"
            lines.append(f"- `{profile['name']}` wrong source: {source}; rule matched: {rules}.")
        precise_gates = [
            row
            for row in profile["gate_simulation"]
            if isinstance(row.get("good_harmed_pct"), (int, float))
            and float(row["good_harmed_pct"]) <= 6.1
        ]
        if precise_gates:
            best_gate = max(precise_gates, key=lambda row: float(row["wrong_caught_pct"] or 0.0))
            lines.append(
                f"- Best low-collateral deployable gate for `{profile['name']}` in this POS-only replay: "
                f"{best_gate['gate']} catches {fmt(best_gate['wrong_caught_pct'], '%')} of wrong FIX "
                f"while touching {fmt(best_gate['good_harmed_pct'], '%')} of good FIX."
            )
    lines.extend(
        [
            "",
            "## Profile summary",
            "",
            "| Profile | FIX epochs | Wrong FIX | Wrong/FIX | Wrong source | Rule matched |",
            "|---|---:|---:|---:|---|---|",
        ]
    )
    for profile in payload["profiles"]:
        source = ", ".join(f"{key}={value}" for key, value in profile["selected_candidate_counts"].items()) or "n/a"
        rules = ", ".join(f"{key}={value}" for key, value in profile["rule_matched_counts"].items()) or "n/a"
        lines.append(
            f"| {profile['name']} | {profile['fixed']} | {profile['wrong_fix']} | "
            f"{fmt(profile['wrong_fix_rate_pct'], '%')} | {source} | {rules} |"
        )

    for profile in payload["profiles"]:
        lines.extend(
            [
                "",
                f"## {profile['name']}",
                "",
                "| Run | FIX epochs | Wrong FIX | Wrong/FIX | Wrong p95 | Longest wrong span |",
                "|---|---:|---:|---:|---:|---|",
            ]
        )
        for run in profile["runs"]:
            if not run.get("available"):
                lines.append(f"| {run['key']} | n/a | n/a | n/a | n/a | missing |")
                continue
            spans = run.get("longest_wrong_spans") or []
            span_text = "n/a"
            if spans:
                span = spans[0]
                span_text = (
                    f"{fmt(span['start_tow_s'])}-{fmt(span['end_tow_s'])} "
                    f"({fmt(span['duration_s'], ' s')})"
                )
            lines.append(
                f"| {run['key']} | {run['fixed']} | {run['wrong_fix']} | "
                f"{fmt(run['wrong_fix_rate_pct'], '%')} | {fmt(run['wrong_error_p95_m'], ' m')} | "
                f"{span_text} |"
            )

        lines.extend(
            [
                "",
                "### Real-time discriminator percentiles",
                "",
                "| Metric | Good FIX p50/p90/p95 | Wrong FIX p50/p90/p95 |",
                "|---|---:|---:|",
            ]
        )
        for row in profile["discriminators"]:
            good = row["good_fix"]
            wrong = row["wrong_fix"]
            lines.append(
                f"| {row['metric']} | {fmt(good['p50'])}/{fmt(good['p90'])}/{fmt(good['p95'])} | "
                f"{fmt(wrong['p50'])}/{fmt(wrong['p90'])}/{fmt(wrong['p95'])} |"
            )

        lines.extend(
            [
                "",
                "### Deployable gate simulation",
                "",
                "| Gate | Wrong caught | Wrong caught | Good FIX harmed | Good FIX harmed |",
                "|---|---:|---:|---:|---:|",
            ]
        )
        for row in profile["gate_simulation"]:
            lines.append(
                f"| {row['gate']} | {row['wrong_caught']} | {fmt(row['wrong_caught_pct'], '%')} | "
                f"{row['good_harmed']} | {fmt(row['good_harmed_pct'], '%')} |"
            )

    lines.append("")
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    profiles = [analyze_profile(parse_profile(spec), args.dataset_root, args.wrong_fix_threshold_m) for spec in args.profile]
    payload = {
        "title": "PPC residual wrong-FIX analysis",
        "wrong_fix_threshold_m": args.wrong_fix_threshold_m,
        "profiles": profiles,
    }
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
    args.markdown_output.write_text(render_markdown(payload), encoding="utf-8")
    print(f"wrote {args.summary_json}")
    print(f"wrote {args.markdown_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
