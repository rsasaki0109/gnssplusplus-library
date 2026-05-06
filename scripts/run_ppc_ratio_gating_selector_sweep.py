#!/usr/bin/env python3
"""Run PPC multi-candidate selector matrices across ratio-gating rule sets.

This is a small recipe wrapper around ``run_ppc_multi_candidate_selector_matrix.py``.
It keeps the expensive 6-run selector machinery in one place and varies only the
per-candidate rule set:

``candidate_status_name == FIXED``
``candidate_status_name == FIXED AND candidate_ratio >= <threshold>``
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


SCRIPTS_DIR = Path(__file__).resolve().parent
MATRIX_SCRIPT = SCRIPTS_DIR / "run_ppc_multi_candidate_selector_matrix.py"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description="Run ratio-gated PPC multi-candidate selector Pareto sweeps.",
    )
    parser.add_argument(
        "--run",
        action="append",
        default=[],
        metavar="CITY/RUN",
        help=(
            "PPC run to process, e.g. tokyo/run1. Repeat to override the "
            "matrix driver's default six runs."
        ),
    )
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=Path("/datasets/PPC-Dataset"),
        help="Root directory of PPC-Dataset.",
    )
    parser.add_argument(
        "--baseline-pos-template",
        default="output/ppc_coverage_matrix_floatreset10/{key}.pos",
        help="Baseline POS path template. Supports {city}, {run}, and {key}.",
    )
    parser.add_argument(
        "--reference-csv-template",
        default="{dataset_root}/{city}/{run}/reference.csv",
        help=(
            "Reference CSV template. Supports {dataset_root}, {city}, {run}, "
            "and {key}."
        ),
    )
    parser.add_argument(
        "--candidate",
        dest="candidates",
        action="append",
        default=[],
        metavar="LABEL=PATH_TEMPLATE",
        help=(
            "Candidate POS path template per label. Repeat for each candidate. "
            "Supports {city}, {run}, and {key}."
        ),
    )
    parser.add_argument(
        "--priority-order",
        default="",
        help="Comma-separated candidate labels; earlier = higher priority on tie.",
    )
    parser.add_argument(
        "--selection-mode",
        choices=("oracle_delta", "priority_first"),
        default="oracle_delta",
        help=(
            "Selector mode passed to the matrix driver. Use priority_first for "
            "a deployable fixed rule/priority selector."
        ),
    )
    parser.add_argument(
        "--threshold-set",
        dest="threshold_sets",
        action="append",
        default=[],
        metavar="SPEC",
        help=(
            "Ratio-gating set. Use NAME=THRESHOLD to apply one threshold to all "
            "candidates, NAME=none for status-only FIXED gating, or "
            "NAME:label=threshold,label2=none for per-candidate thresholds. "
            "Repeat to build a Pareto table."
        ),
    )
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--threshold-m", type=float, default=0.50)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("output/ppc_ratio_gating_selector_sweep"),
        help="Directory for per-set matrix outputs.",
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=Path("output/ppc_ratio_gating_selector_sweep/summary.json"),
        help="Pareto-level summary JSON output path.",
    )
    parser.add_argument(
        "--markdown-output",
        type=Path,
        default=None,
        help="Pareto-level Markdown table output path.",
    )
    parser.add_argument(
        "--title",
        default="PPC ratio-gating selector sweep",
        help="Title used in Pareto JSON and Markdown output.",
    )
    return parser.parse_args()


def parse_candidate_label(spec: str) -> str:
    if "=" not in spec:
        raise SystemExit(f"--candidate must use LABEL=PATH_TEMPLATE, got: {spec!r}")
    label, _ = spec.split("=", 1)
    label = label.strip()
    if not label:
        raise SystemExit(f"--candidate label is empty in: {spec!r}")
    return label


def candidate_labels(candidate_specs: list[str]) -> list[str]:
    labels = [parse_candidate_label(spec) for spec in candidate_specs]
    if not labels:
        raise SystemExit("At least one --candidate is required")
    if len(labels) != len(set(labels)):
        raise SystemExit(f"Candidate labels must be unique: {', '.join(labels)}")
    return labels


def parse_threshold_value(text: str) -> float | None:
    normalized = text.strip().lower()
    if normalized in {"none", "status", "status-only", "wide"}:
        return None
    try:
        return float(normalized)
    except ValueError as exc:
        raise SystemExit(f"Invalid ratio threshold: {text!r}") from exc


def format_threshold(value: float) -> str:
    return f"{value:g}"


def parse_threshold_set(
    spec: str,
    labels: list[str],
) -> tuple[str, dict[str, float | None]]:
    """Parse one --threshold-set specification.

    Accepted forms:
    - ``wide=none``: apply status-only FIXED gating to all candidates.
    - ``ratio4=4``: apply ratio >= 4 to all candidates.
    - ``tight:a=4,b=5``: per-candidate thresholds.
    """
    if ":" in spec:
        name, mapping_text = spec.split(":", 1)
        name = name.strip()
        if not name:
            raise SystemExit(f"Threshold-set name is empty in: {spec!r}")
        thresholds: dict[str, float | None] = {}
        for item in mapping_text.split(","):
            item = item.strip()
            if not item:
                continue
            if "=" not in item:
                raise SystemExit(
                    f"Per-candidate threshold must use LABEL=VALUE, got: {item!r}"
                )
            label, value = item.split("=", 1)
            label = label.strip()
            if label not in labels:
                raise SystemExit(
                    f"Threshold-set {name!r} references unknown candidate {label!r}"
                )
            thresholds[label] = parse_threshold_value(value)
        missing = [label for label in labels if label not in thresholds]
        if missing:
            raise SystemExit(
                f"Threshold-set {name!r} missing candidate thresholds: "
                f"{', '.join(missing)}"
            )
        return name, thresholds

    if "=" not in spec:
        raise SystemExit(
            "--threshold-set must use NAME=VALUE or NAME:LABEL=VALUE,..."
        )
    name, value = spec.split("=", 1)
    name = name.strip()
    if not name:
        raise SystemExit(f"Threshold-set name is empty in: {spec!r}")
    threshold = parse_threshold_value(value)
    return name, {label: threshold for label in labels}


def threshold_rule(label: str, threshold: float | None) -> str:
    if threshold is None:
        return f"{label}=candidate_status_name == FIXED"
    return (
        f"{label}=candidate_status_name == FIXED "
        f"AND candidate_ratio >= {format_threshold(threshold)}"
    )


def sanitize_name(name: str) -> str:
    allowed = [ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in name]
    sanitized = "".join(allowed).strip("_")
    if not sanitized:
        raise SystemExit(f"Threshold-set name cannot be used as a path: {name!r}")
    return sanitized


def build_matrix_argv(
    args: argparse.Namespace,
    set_name: str,
    thresholds: dict[str, float | None],
    labels: list[str],
) -> tuple[list[str], Path, Path]:
    set_dir = args.output_dir / sanitize_name(set_name)
    matrix_json = set_dir / "matrix.json"
    matrix_md = set_dir / "matrix.md"
    run_output_template = str(set_dir / "{key}.pos")

    argv = [
        sys.executable,
        str(MATRIX_SCRIPT),
        "--dataset-root",
        str(args.dataset_root),
        "--baseline-pos-template",
        args.baseline_pos_template,
        "--reference-csv-template",
        args.reference_csv_template,
        "--run-output-template",
        run_output_template,
        "--summary-json",
        str(matrix_json),
        "--markdown-output",
        str(matrix_md),
        "--title",
        f"{args.title}: {set_name}",
        "--match-tolerance-s",
        str(args.match_tolerance_s),
        "--threshold-m",
        str(args.threshold_m),
        "--selection-mode",
        args.selection_mode,
    ]

    for run_spec in args.run:
        argv += ["--run", run_spec]
    for candidate in args.candidates:
        argv += ["--candidate", candidate]
    for label in labels:
        argv += ["--candidate-rule", threshold_rule(label, thresholds[label])]
    if args.priority_order:
        argv += ["--priority-order", args.priority_order]
    return argv, matrix_json, matrix_md


def invoke_matrix(
    args: argparse.Namespace,
    set_name: str,
    thresholds: dict[str, float | None],
    labels: list[str],
) -> tuple[Path, Path]:
    argv, matrix_json, matrix_md = build_matrix_argv(args, set_name, thresholds, labels)
    matrix_json.parent.mkdir(parents=True, exist_ok=True)
    result = subprocess.run(argv, check=False)
    if result.returncode != 0:
        raise SystemExit(
            f"{set_name}: run_ppc_multi_candidate_selector_matrix.py exited "
            f"with code {result.returncode}"
        )
    if not matrix_json.exists():
        raise SystemExit(f"{set_name}: missing matrix JSON: {matrix_json}")
    return matrix_json, matrix_md


def load_matrix_payload(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise SystemExit(f"{path}: expected a JSON object")
    return payload


def _num(mapping: dict[str, object], key: str) -> float:
    value = mapping.get(key)
    return float(value) if isinstance(value, (int, float)) else 0.0


def threshold_description(thresholds: dict[str, float | None]) -> str:
    unique_values = set(thresholds.values())
    if unique_values == {None}:
        return "status==FIXED"
    if len(unique_values) == 1:
        value = next(iter(unique_values))
        if value is None:
            return "status==FIXED"
        return f"ratio>={format_threshold(value)}"
    parts = []
    for label in sorted(thresholds):
        value = thresholds[label]
        parts.append(
            f"{label}=FIXED"
            if value is None
            else f"{label} ratio>={format_threshold(value)}"
        )
    return ", ".join(parts)


def build_sweep_payload(
    title: str,
    set_results: list[
        tuple[str, dict[str, float | None], Path, Path, dict[str, object]]
    ],
) -> dict[str, object]:
    entries: list[dict[str, object]] = []
    for set_name, thresholds, matrix_json, matrix_md, matrix_payload in set_results:
        aggregates = matrix_payload.get("aggregates", {})
        if not isinstance(aggregates, dict):
            raise SystemExit(f"{matrix_json}: missing aggregates object")
        entries.append(
            {
                "name": set_name,
                "rule": threshold_description(thresholds),
                "thresholds": {
                    label: (
                        None
                        if threshold is None
                        else float(format_threshold(threshold))
                    )
                    for label, threshold in sorted(thresholds.items())
                },
                "matrix_json": str(matrix_json),
                "matrix_markdown": str(matrix_md),
                "weighted_baseline_official_score_pct": _num(
                    aggregates, "weighted_baseline_official_score_pct"
                ),
                "weighted_selector_official_score_pct": _num(
                    aggregates, "weighted_selector_official_score_pct"
                ),
                "selector_official_score_delta_pct": _num(
                    aggregates, "selector_official_score_delta_pct"
                ),
                "selector_official_score_delta_m": _num(
                    aggregates, "selector_official_score_delta_m"
                ),
                "min_official_score_delta_m": _num(
                    aggregates, "min_official_score_delta_m"
                ),
                "max_official_score_delta_m": _num(
                    aggregates, "max_official_score_delta_m"
                ),
                "total_candidate_selected_segments": int(
                    _num(aggregates, "total_candidate_selected_segments")
                ),
                "run_count": int(_num(aggregates, "run_count")),
            }
        )
    entries.sort(
        key=lambda entry: (
            -float(entry["selector_official_score_delta_pct"]),
            str(entry["name"]),
        )
    )
    return {
        "title": title,
        "sets": entries,
    }


def render_markdown(payload: dict[str, object]) -> str:
    title = str(payload.get("title", "PPC ratio-gating selector sweep"))
    sets = payload.get("sets", [])
    lines = [
        f"# {title}",
        "",
        "| set | rule | selector | delta | gain m | cand segs | run delta min/max |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    if not isinstance(sets, list):
        raise SystemExit("payload: sets must be a list")
    for entry in sets:
        if not isinstance(entry, dict):
            continue
        lines.append(
            "| {name} | {rule} | {selector:.6f}% | {delta:+.2f} pp | "
            "{gain:+.3f} | {segments} | {min_delta:+.3f}/{max_delta:+.3f} |".format(
                name=entry.get("name", "?"),
                rule=entry.get("rule", ""),
                selector=float(entry.get("weighted_selector_official_score_pct", 0.0)),
                delta=float(entry.get("selector_official_score_delta_pct", 0.0)),
                gain=float(entry.get("selector_official_score_delta_m", 0.0)),
                segments=int(entry.get("total_candidate_selected_segments", 0)),
                min_delta=float(entry.get("min_official_score_delta_m", 0.0)),
                max_delta=float(entry.get("max_official_score_delta_m", 0.0)),
            )
        )
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    labels = candidate_labels(args.candidates)
    if not args.threshold_sets:
        raise SystemExit("At least one --threshold-set is required")

    parsed_sets = [
        parse_threshold_set(spec, labels)
        for spec in args.threshold_sets
    ]
    set_results = []
    for set_name, thresholds in parsed_sets:
        matrix_json, matrix_md = invoke_matrix(args, set_name, thresholds, labels)
        matrix_payload = load_matrix_payload(matrix_json)
        set_results.append((set_name, thresholds, matrix_json, matrix_md, matrix_payload))

    payload = build_sweep_payload(args.title, set_results)
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    markdown = render_markdown(payload)
    if args.markdown_output is not None:
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(markdown, encoding="utf-8")
    else:
        print(markdown)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
