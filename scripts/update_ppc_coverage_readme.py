#!/usr/bin/env python3
"""Synchronize README/docs PPC coverage tables from ppc-coverage-matrix JSON."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
from typing import Any


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_SUMMARY_JSON = ROOT_DIR / "output" / "ppc_coverage_matrix" / "summary.json"
DEFAULT_TARGETS = (
    ROOT_DIR / "README.md",
    ROOT_DIR / "docs" / "benchmarks.md",
)
START_MARKER = "<!-- PPC_COVERAGE_MATRIX:START -->"
END_MARKER = "<!-- PPC_COVERAGE_MATRIX:END -->"
NUMBER_WORDS = {
    1: "one",
    2: "two",
    3: "three",
    4: "four",
    5: "five",
    6: "six",
    7: "seven",
    8: "eight",
    9: "nine",
    10: "ten",
}


@dataclass(frozen=True)
class CoverageRun:
    key: str
    label: str
    lib_positioning_pct: float
    rtklib_positioning_pct: float
    positioning_delta_pct: float
    lib_fix_pct: float
    rtklib_fix_pct: float
    score_3d_50cm_ref_delta_pct: float
    p95_h_delta_m: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=DEFAULT_SUMMARY_JSON,
        help="ppc-coverage-matrix summary JSON (default: output/ppc_coverage_matrix/summary.json).",
    )
    parser.add_argument(
        "--target",
        type=Path,
        action="append",
        default=None,
        help="Markdown file to update. Repeat to override the default README/docs pair.",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Fail if a target is stale instead of writing changes.",
    )
    return parser.parse_args()


def required_number(mapping: dict[str, Any], name: str, context: str) -> float:
    value = mapping.get(name)
    if not isinstance(value, (int, float)):
        raise SystemExit(f"{context}: missing numeric `{name}`")
    return float(value)


def display_name(key: str) -> str:
    city, _, run_name = key.partition("_")
    if not city or not run_name:
        return key
    return f"{city.capitalize()} {run_name}"


def load_summary(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise SystemExit(f"Missing PPC coverage matrix summary JSON: {path}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise SystemExit(f"{path} does not contain a JSON object")
    return payload


def extract_runs(payload: dict[str, Any]) -> list[CoverageRun]:
    raw_runs = payload.get("runs")
    if not isinstance(raw_runs, list):
        raise SystemExit("PPC coverage matrix summary is missing a `runs` list")

    runs: list[CoverageRun] = []
    for item in raw_runs:
        if not isinstance(item, dict):
            raise SystemExit("PPC coverage matrix summary contains a non-object run entry")
        key = str(item.get("key", ""))
        metrics = item.get("metrics")
        rtklib = item.get("rtklib")
        delta = item.get("delta_vs_rtklib")
        if not isinstance(metrics, dict) or not isinstance(rtklib, dict) or not isinstance(delta, dict):
            raise SystemExit(f"{key or '<unknown>'}: missing metrics, rtklib, or delta_vs_rtklib block")
        runs.append(
            CoverageRun(
                key=key,
                label=display_name(key),
                lib_positioning_pct=required_number(metrics, "positioning_rate_pct", key),
                rtklib_positioning_pct=required_number(rtklib, "positioning_rate_pct", key),
                positioning_delta_pct=required_number(delta, "positioning_rate_pct", key),
                lib_fix_pct=required_number(metrics, "fix_rate_pct", key),
                rtklib_fix_pct=required_number(rtklib, "fix_rate_pct", key),
                score_3d_50cm_ref_delta_pct=required_number(delta, "ppc_score_3d_50cm_ref_pct", key),
                p95_h_delta_m=required_number(delta, "p95_h_m", key),
            )
        )
    if not runs:
        raise SystemExit("PPC coverage matrix summary has no runs")
    return runs


def average(values: list[float]) -> float:
    return sum(values) / len(values)


def pct(value: float) -> str:
    return f"{value:.1f}%"


def pp(value: float) -> str:
    return f"{value:+.1f} pp"


def meters(value: float) -> str:
    return f"{value:+.2f} m" if value > 0 else f"{value:.2f} m"


def run_count_text(count: int) -> str:
    return NUMBER_WORDS.get(count, str(count))


def render_coverage_block(payload: dict[str, Any]) -> str:
    runs = extract_runs(payload)
    lines = [
        "| Run | gnssplusplus Positioning | RTKLIB Positioning | Delta | gnssplusplus Fix | RTKLIB Fix | 3D <= 50 cm / ref delta | P95 H delta |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for run in runs:
        lines.append(
            f"| {run.label} | **{pct(run.lib_positioning_pct)}** | "
            f"{pct(run.rtklib_positioning_pct)} | **{pp(run.positioning_delta_pct)}** | "
            f"**{pct(run.lib_fix_pct)}** | {pct(run.rtklib_fix_pct)} | "
            f"**{pp(run.score_3d_50cm_ref_delta_pct)}** | {meters(run.p95_h_delta_m)} |"
        )

    avg_positioning_delta = average([run.positioning_delta_pct for run in runs])
    avg_score_delta = average([run.score_3d_50cm_ref_delta_pct for run in runs])
    avg_p95_delta = average([run.p95_h_delta_m for run in runs])
    lines.extend(
        [
            "",
            f"Across these {run_count_text(len(runs))} public runs, the coverage profile averages "
            f"**{pp(avg_positioning_delta)}**",
            f"Positioning-rate lead, **{pp(avg_score_delta)}** 3D<=50cm/reference-score lead, and",
            f"**{meters(avg_p95_delta)}** P95 horizontal-error delta versus RTKLIB `demo5`.",
        ]
    )
    return "\n".join(lines)


def replace_marked_block(text: str, block: str) -> str:
    if text.count(START_MARKER) != 1 or text.count(END_MARKER) != 1:
        raise SystemExit(f"Expected exactly one {START_MARKER} / {END_MARKER} marker pair")
    prefix, rest = text.split(START_MARKER, 1)
    _, suffix = rest.split(END_MARKER, 1)
    return f"{prefix}{START_MARKER}\n{block.rstrip()}\n{END_MARKER}{suffix}"


def update_target(path: Path, block: str, *, check: bool) -> bool:
    original = path.read_text(encoding="utf-8")
    updated = replace_marked_block(original, block)
    changed = updated != original
    if changed and not check:
        path.write_text(updated, encoding="utf-8")
    return changed


def main() -> int:
    args = parse_args()
    payload = load_summary(args.summary_json)
    block = render_coverage_block(payload)
    targets = tuple(args.target) if args.target is not None else DEFAULT_TARGETS
    changed = [path for path in targets if update_target(path, block, check=args.check)]

    if args.check and changed:
        changed_list = "\n".join(f"  - {path}" for path in changed)
        raise SystemExit("PPC coverage README blocks are stale:\n" + changed_list)

    if changed:
        for path in changed:
            print(f"Updated: {path}")
    else:
        print("PPC coverage README blocks already up to date.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
