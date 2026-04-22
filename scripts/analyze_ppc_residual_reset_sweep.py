#!/usr/bin/env python3
"""Analyze PPC residual-reset sweep summaries and selector upper bounds."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class Profile:
    label: str
    summary_json: Path
    payload: dict[str, Any]
    runs: dict[str, dict[str, Any]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--baseline-summary-json",
        type=Path,
        required=True,
        help="Baseline ppc-coverage-matrix summary JSON.",
    )
    parser.add_argument(
        "--baseline-label",
        default="baseline",
        help="Label for the baseline profile (default: baseline).",
    )
    parser.add_argument(
        "--candidate",
        action="append",
        default=[],
        metavar="LABEL=SUMMARY_JSON",
        help="Candidate ppc-coverage-matrix summary. Repeat for sweep profiles.",
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=None,
        help="Optional output JSON path for the selector analysis.",
    )
    parser.add_argument(
        "--markdown-output",
        type=Path,
        default=None,
        help="Optional Markdown report path for the selector analysis.",
    )
    return parser.parse_args()


def parse_candidate(value: str) -> tuple[str, Path]:
    label, sep, path_text = value.partition("=")
    if not sep or not label.strip() or not path_text.strip():
        raise SystemExit("--candidate must use LABEL=SUMMARY_JSON")
    return label.strip(), Path(path_text.strip())


def load_profile(label: str, summary_json: Path) -> Profile:
    if not summary_json.exists():
        raise SystemExit(f"{label}: missing summary JSON: {summary_json}")
    payload = json.loads(summary_json.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise SystemExit(f"{label}: summary JSON must contain an object")
    raw_runs = payload.get("runs")
    if not isinstance(raw_runs, list) or not raw_runs:
        raise SystemExit(f"{label}: summary JSON is missing a non-empty `runs` list")

    runs: dict[str, dict[str, Any]] = {}
    for raw_run in raw_runs:
        if not isinstance(raw_run, dict):
            raise SystemExit(f"{label}: run entry is not an object")
        key = raw_run.get("key")
        if not isinstance(key, str) or not key:
            raise SystemExit(f"{label}: run entry is missing string `key`")
        if key in runs:
            raise SystemExit(f"{label}: duplicate run key `{key}`")
        runs[key] = raw_run
    return Profile(label=label, summary_json=summary_json, payload=payload, runs=runs)


def require_number(mapping: dict[str, Any], name: str, context: str) -> float:
    value = mapping.get(name)
    if not isinstance(value, (int, float)):
        raise SystemExit(f"{context}: missing numeric `{name}`")
    return float(value)


def metrics(run: dict[str, Any], context: str) -> dict[str, Any]:
    section = run.get("metrics")
    if not isinstance(section, dict):
        raise SystemExit(f"{context}: missing `metrics` block")
    return section


def score_distance_m(run: dict[str, Any], context: str) -> float:
    return require_number(metrics(run, context), "ppc_official_score_distance_m", context)


def total_distance_m(run: dict[str, Any], context: str) -> float:
    total = require_number(metrics(run, context), "ppc_official_total_distance_m", context)
    if total <= 0.0:
        raise SystemExit(f"{context}: `ppc_official_total_distance_m` must be positive")
    return total


def optional_metric(run: dict[str, Any], name: str) -> float | None:
    section = run.get("metrics")
    if not isinstance(section, dict):
        return None
    value = section.get(name)
    return float(value) if isinstance(value, (int, float)) else None


def profile_score(profile: Profile, run_keys: list[str]) -> tuple[float, float]:
    score = 0.0
    total = 0.0
    for key in run_keys:
        context = f"{profile.label}:{key}"
        run = profile.runs[key]
        score += score_distance_m(run, context)
        total += total_distance_m(run, context)
    return score, total


def weighted_pct(score: float, total: float) -> float:
    if total <= 0.0:
        raise SystemExit("cannot compute weighted score with non-positive total distance")
    return 100.0 * score / total


def round6(value: float | None) -> float | None:
    return None if value is None else round(value, 6)


def average(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def average_metric_delta(
    profile: Profile,
    baseline: Profile,
    run_keys: list[str],
    name: str,
) -> float | None:
    deltas: list[float] = []
    for key in run_keys:
        value = optional_metric(profile.runs[key], name)
        baseline_value = optional_metric(baseline.runs[key], name)
        if value is not None and baseline_value is not None:
            deltas.append(value - baseline_value)
    return average(deltas)


def profile_settings(profile: Profile) -> dict[str, Any]:
    payload = profile.payload
    names = (
        "ratio",
        "max_consec_float_reset",
        "max_float_prefit_rms",
        "max_float_prefit_max",
        "max_float_prefit_reset_streak",
        "no_float_bridge_tail_guard",
        "fixed_bridge_burst_guard",
    )
    return {name: payload.get(name) for name in names if name in payload}


def profile_summary(profile: Profile, baseline: Profile, run_keys: list[str]) -> dict[str, Any]:
    score, total = profile_score(profile, run_keys)
    baseline_score, baseline_total = profile_score(baseline, run_keys)
    if abs(total - baseline_total) > 1e-3:
        raise SystemExit(
            f"{profile.label}: total official distance differs from baseline "
            f"({total:.3f} m vs {baseline_total:.3f} m)"
        )
    score_pct = weighted_pct(score, total)
    baseline_pct = weighted_pct(baseline_score, baseline_total)
    return {
        "label": profile.label,
        "summary_json": str(profile.summary_json),
        "weighted_official_score_pct": round6(score_pct),
        "score_distance_m": round6(score),
        "total_distance_m": round6(total),
        "delta_vs_baseline_score_distance_m": round6(score - baseline_score),
        "delta_vs_baseline_score_pct": round6(score_pct - baseline_pct),
        "avg_positioning_rate_delta_vs_baseline_pct": round6(
            average_metric_delta(profile, baseline, run_keys, "positioning_rate_pct")
        ),
        "avg_p95_h_delta_vs_baseline_m": round6(
            average_metric_delta(profile, baseline, run_keys, "p95_h_m")
        ),
        "settings": profile_settings(profile),
    }


def run_city(key: str) -> str:
    city, _, _ = key.partition("_")
    return city or key


def validate_run_sets(baseline: Profile, candidates: list[Profile]) -> list[str]:
    run_keys = sorted(baseline.runs)
    baseline_set = set(run_keys)
    for profile in candidates:
        profile_set = set(profile.runs)
        if profile_set != baseline_set:
            missing = sorted(baseline_set - profile_set)
            extra = sorted(profile_set - baseline_set)
            details: list[str] = []
            if missing:
                details.append("missing " + ", ".join(missing))
            if extra:
                details.append("extra " + ", ".join(extra))
            raise SystemExit(f"{profile.label}: run-key mismatch with baseline ({'; '.join(details)})")
    return run_keys


def best_profile_for_keys(profiles: list[Profile], run_keys: list[str]) -> tuple[Profile, float, float]:
    best_profile = profiles[0]
    best_score, best_total = profile_score(best_profile, run_keys)
    for profile in profiles[1:]:
        score, total = profile_score(profile, run_keys)
        if abs(total - best_total) > 1e-3:
            raise SystemExit(f"{profile.label}: total official distance mismatch in selector group")
        if score > best_score:
            best_profile = profile
            best_score = score
            best_total = total
    return best_profile, best_score, best_total


def build_run_rows(profiles: list[Profile], baseline: Profile, run_keys: list[str]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for key in run_keys:
        baseline_run = baseline.runs[key]
        baseline_score = score_distance_m(baseline_run, f"{baseline.label}:{key}")
        baseline_total = total_distance_m(baseline_run, f"{baseline.label}:{key}")
        best_profile, best_score, best_total = best_profile_for_keys(profiles, [key])
        if abs(best_total - baseline_total) > 1e-3:
            raise SystemExit(f"{key}: total official distance differs across profiles")

        candidate_rows: list[dict[str, Any]] = []
        for profile in profiles:
            run = profile.runs[key]
            score = score_distance_m(run, f"{profile.label}:{key}")
            total = total_distance_m(run, f"{profile.label}:{key}")
            if abs(total - baseline_total) > 1e-3:
                raise SystemExit(f"{profile.label}:{key}: total official distance differs from baseline")
            positioning = optional_metric(run, "positioning_rate_pct")
            baseline_positioning = optional_metric(baseline_run, "positioning_rate_pct")
            p95_h = optional_metric(run, "p95_h_m")
            baseline_p95_h = optional_metric(baseline_run, "p95_h_m")
            candidate_rows.append(
                {
                    "label": profile.label,
                    "official_score_pct": round6(weighted_pct(score, total)),
                    "score_distance_m": round6(score),
                    "delta_score_distance_m": round6(score - baseline_score),
                    "delta_official_score_pct": round6(
                        weighted_pct(score, total) - weighted_pct(baseline_score, baseline_total)
                    ),
                    "positioning_rate_pct": round6(positioning),
                    "delta_positioning_rate_pct": round6(
                        positioning - baseline_positioning
                        if positioning is not None and baseline_positioning is not None
                        else None
                    ),
                    "p95_h_m": round6(p95_h),
                    "delta_p95_h_m": round6(
                        p95_h - baseline_p95_h
                        if p95_h is not None and baseline_p95_h is not None
                        else None
                    ),
                }
            )

        rows.append(
            {
                "key": key,
                "city": run_city(key),
                "baseline_official_score_pct": round6(weighted_pct(baseline_score, baseline_total)),
                "baseline_score_distance_m": round6(baseline_score),
                "total_distance_m": round6(baseline_total),
                "best_profile": best_profile.label,
                "best_official_score_pct": round6(weighted_pct(best_score, best_total)),
                "best_score_distance_m": round6(best_score),
                "best_delta_score_distance_m": round6(best_score - baseline_score),
                "best_delta_official_score_pct": round6(
                    weighted_pct(best_score, best_total) - weighted_pct(baseline_score, baseline_total)
                ),
                "profiles": candidate_rows,
            }
        )
    return rows


def build_city_selector(profiles: list[Profile], baseline: Profile, run_keys: list[str]) -> dict[str, Any]:
    baseline_score, baseline_total = profile_score(baseline, run_keys)
    selected_score = 0.0
    selected_total = 0.0
    selections: list[dict[str, Any]] = []
    for city in sorted({run_city(key) for key in run_keys}):
        city_keys = [key for key in run_keys if run_city(key) == city]
        base_city_score, base_city_total = profile_score(baseline, city_keys)
        profile, score, total = best_profile_for_keys(profiles, city_keys)
        selected_score += score
        selected_total += total
        selections.append(
            {
                "city": city,
                "profile": profile.label,
                "weighted_official_score_pct": round6(weighted_pct(score, total)),
                "delta_vs_baseline_score_distance_m": round6(score - base_city_score),
                "delta_vs_baseline_score_pct": round6(
                    weighted_pct(score, total) - weighted_pct(base_city_score, base_city_total)
                ),
                "run_count": len(city_keys),
            }
        )
    return {
        "weighted_official_score_pct": round6(weighted_pct(selected_score, selected_total)),
        "score_distance_m": round6(selected_score),
        "total_distance_m": round6(selected_total),
        "delta_vs_baseline_score_distance_m": round6(selected_score - baseline_score),
        "delta_vs_baseline_score_pct": round6(
            weighted_pct(selected_score, selected_total) - weighted_pct(baseline_score, baseline_total)
        ),
        "selections": selections,
    }


def build_run_oracle(run_rows: list[dict[str, Any]], baseline: Profile, run_keys: list[str]) -> dict[str, Any]:
    baseline_score, baseline_total = profile_score(baseline, run_keys)
    selected_score = sum(float(row["best_score_distance_m"]) for row in run_rows)
    selected_total = sum(float(row["total_distance_m"]) for row in run_rows)
    return {
        "weighted_official_score_pct": round6(weighted_pct(selected_score, selected_total)),
        "score_distance_m": round6(selected_score),
        "total_distance_m": round6(selected_total),
        "delta_vs_baseline_score_distance_m": round6(selected_score - baseline_score),
        "delta_vs_baseline_score_pct": round6(
            weighted_pct(selected_score, selected_total) - weighted_pct(baseline_score, baseline_total)
        ),
        "selections": [
            {
                "key": row["key"],
                "profile": row["best_profile"],
                "delta_vs_baseline_score_distance_m": row["best_delta_score_distance_m"],
                "delta_vs_baseline_score_pct": row["best_delta_official_score_pct"],
            }
            for row in run_rows
        ],
    }


def build_payload(baseline: Profile, candidates: list[Profile]) -> dict[str, Any]:
    run_keys = validate_run_sets(baseline, candidates)
    profiles = [baseline, *candidates]
    profile_rows = [profile_summary(profile, baseline, run_keys) for profile in profiles]
    global_best = max(profile_rows, key=lambda row: float(row["score_distance_m"]))
    run_rows = build_run_rows(profiles, baseline, run_keys)
    return {
        "baseline_label": baseline.label,
        "profiles": profile_rows,
        "global_best_profile": {
            "label": global_best["label"],
            "weighted_official_score_pct": global_best["weighted_official_score_pct"],
            "delta_vs_baseline_score_distance_m": global_best["delta_vs_baseline_score_distance_m"],
            "delta_vs_baseline_score_pct": global_best["delta_vs_baseline_score_pct"],
        },
        "best_by_city_selector": build_city_selector(profiles, baseline, run_keys),
        "best_per_run_oracle": build_run_oracle(run_rows, baseline, run_keys),
        "runs": run_rows,
    }


def fmt_pct(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.2f}%"


def fmt_pp(value: float | None) -> str:
    return "n/a" if value is None else f"{value:+.3f} pp"


def fmt_m(value: float | None) -> str:
    return "n/a" if value is None else f"{value:+.1f} m"


def render_markdown(payload: dict[str, Any]) -> str:
    lines = [
        "# PPC Residual Reset Selector Analysis",
        "",
        "| Profile | Weighted official | Delta | Score-distance delta | Avg positioning delta | Avg P95 H delta |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for profile in payload["profiles"]:
        lines.append(
            f"| {profile['label']} | {fmt_pct(profile['weighted_official_score_pct'])} | "
            f"{fmt_pp(profile['delta_vs_baseline_score_pct'])} | "
            f"{fmt_m(profile['delta_vs_baseline_score_distance_m'])} | "
            f"{fmt_pp(profile['avg_positioning_rate_delta_vs_baseline_pct'])} | "
            f"{fmt_m(profile['avg_p95_h_delta_vs_baseline_m'])} |"
        )

    global_best = payload["global_best_profile"]
    city = payload["best_by_city_selector"]
    oracle = payload["best_per_run_oracle"]
    lines.extend(
        [
            "",
            f"Global best profile: **{global_best['label']}** at "
            f"**{fmt_pct(global_best['weighted_official_score_pct'])}** "
            f"({fmt_pp(global_best['delta_vs_baseline_score_pct'])}, "
            f"{fmt_m(global_best['delta_vs_baseline_score_distance_m'])}).",
            f"City selector: **{fmt_pct(city['weighted_official_score_pct'])}** "
            f"({fmt_pp(city['delta_vs_baseline_score_pct'])}, "
            f"{fmt_m(city['delta_vs_baseline_score_distance_m'])}).",
            f"Per-run oracle: **{fmt_pct(oracle['weighted_official_score_pct'])}** "
            f"({fmt_pp(oracle['delta_vs_baseline_score_pct'])}, "
            f"{fmt_m(oracle['delta_vs_baseline_score_distance_m'])}).",
            "",
            "## City Selector",
            "",
            "| City | Profile | City official | Delta | Score-distance delta | Runs |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for row in city["selections"]:
        lines.append(
            f"| {row['city']} | {row['profile']} | {fmt_pct(row['weighted_official_score_pct'])} | "
            f"{fmt_pp(row['delta_vs_baseline_score_pct'])} | "
            f"{fmt_m(row['delta_vs_baseline_score_distance_m'])} | {row['run_count']} |"
        )

    lines.extend(
        [
            "",
            "## Per-Run Oracle",
            "",
            "| Run | Best profile | Official | Delta | Score-distance delta |",
            "|---|---:|---:|---:|---:|",
        ]
    )
    for row in payload["runs"]:
        lines.append(
            f"| {row['key']} | {row['best_profile']} | {fmt_pct(row['best_official_score_pct'])} | "
            f"{fmt_pp(row['best_delta_official_score_pct'])} | "
            f"{fmt_m(row['best_delta_score_distance_m'])} |"
        )
    return "\n".join(lines)


def write_outputs(payload: dict[str, Any], args: argparse.Namespace) -> None:
    if args.summary_json is not None:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    if args.markdown_output is not None:
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(render_markdown(payload) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    baseline = load_profile(args.baseline_label, args.baseline_summary_json)
    candidates = [
        load_profile(label, path)
        for label, path in (parse_candidate(item) for item in args.candidate)
    ]
    payload = build_payload(baseline, candidates)
    write_outputs(payload, args)
    print(render_markdown(payload))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
