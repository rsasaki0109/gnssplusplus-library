#!/usr/bin/env python3
"""Drive apply_ppc_multi_candidate_selector across PPC runs and aggregate results.

Invokes apply_ppc_multi_candidate_selector.py as a subprocess for each of the
six default PPC runs (tokyo/nagoya x run1-3, overridable via --run), then
aggregates per-run summary JSON files into a matrix-level JSON and Markdown table.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = Path(__file__).resolve().parent
APPLY_SCRIPT = SCRIPTS_DIR / "apply_ppc_multi_candidate_selector.py"

PPC_RUNS: tuple[tuple[str, str], ...] = (
    ("tokyo", "run1"),
    ("tokyo", "run2"),
    ("tokyo", "run3"),
    ("nagoya", "run1"),
    ("nagoya", "run2"),
    ("nagoya", "run3"),
)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description="Multi-candidate PPC selector matrix driver.",
    )
    parser.add_argument(
        "--run",
        action="append",
        default=[],
        metavar="CITY/RUN",
        help=(
            "PPC run to process, e.g. tokyo/run1. "
            "Repeat to override the default six runs."
        ),
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
            "Reference CSV template. "
            "Supports {dataset_root}, {city}, {run}, and {key}."
        ),
    )
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=Path("/datasets/PPC-Dataset"),
        help="Root directory of PPC-Dataset (used in --reference-csv-template).",
    )
    parser.add_argument(
        "--candidate",
        dest="candidates",
        action="append",
        default=[],
        metavar="LABEL=PATH_TEMPLATE",
        help=(
            "Candidate POS path template per label, e.g. "
            "nis5=output/ppc_coverage_matrix_nis_5_v2/{key}.pos. "
            "Supports {city}, {run}, and {key}. Repeat for each candidate."
        ),
    )
    parser.add_argument(
        "--candidate-rule",
        dest="candidate_rules",
        action="append",
        default=[],
        metavar="LABEL=RULE",
        help="Selector rule per candidate label. Repeat for each candidate.",
    )
    parser.add_argument(
        "--priority-order",
        default="",
        help=(
            "Comma-separated candidate labels; earlier = higher priority on tie. "
            "Passed through to apply_ppc_multi_candidate_selector.py."
        ),
    )
    parser.add_argument(
        "--run-output-template",
        default="output/ppc_multi_selector/{key}.pos",
        help="Selected POS output template. Supports {city}, {run}, and {key}.",
    )
    parser.add_argument(
        "--run-summary-template",
        default=None,
        help=(
            "Per-run summary JSON template. "
            "Defaults to <run-output-stem>_summary.json."
        ),
    )
    parser.add_argument(
        "--run-segments-template",
        default=None,
        help=(
            "Per-run selected-segment CSV template. "
            "Defaults to <run-output-stem>_segments.csv."
        ),
    )
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--threshold-m", type=float, default=0.50)
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=Path("output/ppc_multi_selector_matrix.json"),
        help="Matrix-level summary JSON output path.",
    )
    parser.add_argument(
        "--markdown-output",
        type=Path,
        default=None,
        help="Matrix-level Markdown table output path.",
    )
    parser.add_argument(
        "--title",
        default="PPC multi-candidate selector",
        help="Title used in matrix Markdown output.",
    )
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Path helpers
# ---------------------------------------------------------------------------


def run_key(city: str, run: str) -> str:
    return f"{city}_{run}"


def format_template(
    template: str, dataset_root: Path, city: str, run: str
) -> Path:
    key = run_key(city, run)
    return Path(
        template.format(
            dataset_root=str(dataset_root),
            city=city,
            run=run,
            key=key,
        )
    )


def companion_path(path: Path, suffix: str, extension: str) -> Path:
    return path.with_name(f"{path.stem}_{suffix}{extension}")


def parse_run_arg(value: str) -> tuple[str, str]:
    normalized = value.replace("_", "/")
    city, separator, run = normalized.partition("/")
    if not separator or not city.strip() or not run.strip():
        raise SystemExit("--run must use CITY/RUN, e.g. tokyo/run1")
    return city.strip(), run.strip()


def selected_runs(values: list[str]) -> list[tuple[str, str]]:
    return [parse_run_arg(v) for v in values] if values else list(PPC_RUNS)


# ---------------------------------------------------------------------------
# Per-run subprocess invocation
# ---------------------------------------------------------------------------


def build_apply_argv(
    args: argparse.Namespace,
    city: str,
    run: str,
    out_pos: Path,
    summary_json: Path,
    segments_csv: Path,
) -> list[str]:
    """Build the argv list for apply_ppc_multi_candidate_selector.py."""
    key = run_key(city, run)
    reference_csv = format_template(
        args.reference_csv_template, args.dataset_root, city, run
    )
    baseline_pos = format_template(
        args.baseline_pos_template, args.dataset_root, city, run
    )

    argv = [
        sys.executable,
        str(APPLY_SCRIPT),
        "--reference-csv",
        str(reference_csv),
        "--baseline-pos",
        str(baseline_pos),
    ]

    for candidate_spec in args.candidates:
        if "=" not in candidate_spec:
            raise SystemExit(
                f"--candidate must use LABEL=PATH_TEMPLATE, got: {candidate_spec!r}"
            )
        label, path_template = candidate_spec.split("=", 1)
        label = label.strip()
        candidate_path = format_template(
            path_template.strip(), args.dataset_root, city, run
        )
        argv += ["--candidate", f"{label}={candidate_path}"]

    for rule_spec in args.candidate_rules:
        argv += ["--candidate-rule", rule_spec]

    if args.priority_order:
        argv += ["--priority-order", args.priority_order]

    argv += ["--match-tolerance-s", str(args.match_tolerance_s)]
    argv += ["--threshold-m", str(args.threshold_m)]
    argv += ["--out-pos", str(out_pos)]
    argv += ["--summary-json", str(summary_json)]
    argv += ["--segments-csv", str(segments_csv)]

    _ = key  # key used via format_template
    return argv


def run_output_artifacts(
    args: argparse.Namespace,
    city: str,
    run: str,
) -> tuple[Path, Path, Path]:
    out_pos = format_template(
        args.run_output_template, args.dataset_root, city, run
    )
    summary = (
        format_template(args.run_summary_template, args.dataset_root, city, run)
        if args.run_summary_template
        else companion_path(out_pos, "summary", ".json")
    )
    segments = (
        format_template(args.run_segments_template, args.dataset_root, city, run)
        if args.run_segments_template
        else companion_path(out_pos, "segments", ".csv")
    )
    return out_pos, summary, segments


def invoke_apply_for_run(
    args: argparse.Namespace,
    city: str,
    run: str,
) -> Path:
    """Invoke apply_ppc_multi_candidate_selector.py for one run.

    Returns the path to the per-run summary JSON.
    """
    out_pos, summary_json, segments_csv = run_output_artifacts(args, city, run)
    argv = build_apply_argv(args, city, run, out_pos, summary_json, segments_csv)
    result = subprocess.run(argv, check=False)
    if result.returncode != 0:
        raise SystemExit(
            f"{run_key(city, run)}: apply_ppc_multi_candidate_selector.py "
            f"exited with code {result.returncode}"
        )
    return summary_json


# ---------------------------------------------------------------------------
# Aggregation helpers
# ---------------------------------------------------------------------------


def _required_dict(
    payload: dict[str, object], key: str, context: str
) -> dict[str, object]:
    value = payload.get(key)
    if not isinstance(value, dict):
        raise SystemExit(f"{context}: missing or non-dict key {key!r}")
    return value


def _required_number(
    mapping: dict[str, object], key: str, context: str
) -> float:
    value = mapping.get(key)
    if value is None:
        raise SystemExit(f"{context}: missing key {key!r}")
    return float(value)


def _optional_number(mapping: dict[str, object], key: str) -> float:
    value = mapping.get(key)
    return float(value) if value is not None else 0.0


def load_run_summary(
    key: str, summary_json: Path
) -> dict[str, object]:
    if not summary_json.exists():
        raise SystemExit(f"{key}: missing summary JSON: {summary_json}")
    payload = json.loads(summary_json.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise SystemExit(f"{summary_json}: expected a JSON object")
    return payload


def _rounded(value: float) -> float:
    return round(value, 6)


def aggregate_runs(
    run_payloads: list[tuple[str, str, dict[str, object]]],
) -> dict[str, object]:
    """Aggregate per-run summaries into matrix-level statistics."""
    total_distance_m = 0.0
    baseline_score_m = 0.0
    selected_score_m = 0.0
    total_candidate_selected_segments = 0
    total_baseline_selected_segments = 0
    total_score_delta_m = 0.0
    total_dropped: list[str] = []
    per_run_deltas: list[float] = []

    for city, run, payload in run_payloads:
        key = run_key(city, run)
        metrics = _required_dict(payload, "metrics", key)
        baseline = _required_dict(payload, "baseline", key)
        selection = _required_dict(payload, "selection", key)

        total_m = _required_number(
            metrics, "ppc_official_total_distance_m", f"{key}.metrics"
        )
        sel_score_m = _required_number(
            metrics, "ppc_official_score_distance_m", f"{key}.metrics"
        )
        bl_score_m = _required_number(
            baseline, "ppc_official_score_distance_m", f"{key}.baseline"
        )
        total_distance_m += total_m
        baseline_score_m += bl_score_m
        selected_score_m += sel_score_m
        per_run_deltas.append(sel_score_m - bl_score_m)
        total_candidate_selected_segments += int(
            _optional_number(selection, "candidate_selected_segments")
        )
        total_baseline_selected_segments += int(
            _optional_number(selection, "baseline_selected_segments")
        )
        total_score_delta_m += _optional_number(
            selection, "total_score_delta_distance_m"
        )
        for label in payload.get("dropped_candidates", []):
            if label not in total_dropped:
                total_dropped.append(label)

    if total_distance_m <= 0.0:
        raise SystemExit(
            "Cannot compute weighted official score: total distance is zero"
        )

    baseline_weighted = 100.0 * baseline_score_m / total_distance_m
    selected_weighted = 100.0 * selected_score_m / total_distance_m

    return {
        "run_count": len(run_payloads),
        "official_total_distance_m": _rounded(total_distance_m),
        "weighted_baseline_official_score_pct": _rounded(baseline_weighted),
        "weighted_selector_official_score_pct": _rounded(selected_weighted),
        "selector_official_score_delta_m": _rounded(
            selected_score_m - baseline_score_m
        ),
        "selector_official_score_delta_pct": _rounded(
            selected_weighted - baseline_weighted
        ),
        "min_official_score_delta_m": _rounded(min(per_run_deltas)),
        "max_official_score_delta_m": _rounded(max(per_run_deltas)),
        "total_candidate_selected_segments": total_candidate_selected_segments,
        "total_baseline_selected_segments": total_baseline_selected_segments,
        "total_score_delta_distance_m": _rounded(total_score_delta_m),
        "dropped_candidates_any_run": sorted(total_dropped),
    }


def build_run_entry(
    city: str, run: str, payload: dict[str, object]
) -> dict[str, object]:
    key = run_key(city, run)
    metrics = _required_dict(payload, "metrics", key)
    baseline = _required_dict(payload, "baseline", key)
    delta = _required_dict(payload, "delta_vs_baseline", key)
    selection = _required_dict(payload, "selection", key)

    # Candidate distribution: per_candidate is nested under selection
    per_candidate_raw = selection.get("per_candidate")
    per_candidate: dict[str, object] = {}
    if isinstance(per_candidate_raw, dict):
        for label in sorted(per_candidate_raw):
            entry = per_candidate_raw[label]
            if isinstance(entry, dict):
                per_candidate[label] = {
                    "selected_segments": entry.get("selected_segments", 0),
                    "score_delta_distance_m": entry.get(
                        "score_delta_distance_m", 0.0
                    ),
                }

    return {
        "key": key,
        "city": city,
        "run": run,
        "baseline": {
            "ppc_official_score_pct": _required_number(
                baseline, "ppc_official_score_pct", f"{key}.baseline"
            ),
            "ppc_official_score_distance_m": _required_number(
                baseline, "ppc_official_score_distance_m", f"{key}.baseline"
            ),
            "ppc_official_total_distance_m": _required_number(
                baseline, "ppc_official_total_distance_m", f"{key}.baseline"
            ),
        },
        "selector": {
            "ppc_official_score_pct": _required_number(
                metrics, "ppc_official_score_pct", f"{key}.metrics"
            ),
            "ppc_official_score_distance_m": _required_number(
                metrics, "ppc_official_score_distance_m", f"{key}.metrics"
            ),
            "ppc_official_total_distance_m": _required_number(
                metrics, "ppc_official_total_distance_m", f"{key}.metrics"
            ),
        },
        "delta_vs_baseline": {
            "ppc_official_score_distance_m": _required_number(
                delta, "ppc_official_score_distance_m", f"{key}.delta"
            ),
            "ppc_official_score_pct": _required_number(
                delta, "ppc_official_score_pct", f"{key}.delta"
            ),
        },
        "selection": {
            "segments": int(_optional_number(selection, "segments")),
            "candidate_selected_segments": int(
                _optional_number(selection, "candidate_selected_segments")
            ),
            "baseline_selected_segments": int(
                _optional_number(selection, "baseline_selected_segments")
            ),
            "total_score_delta_distance_m": _optional_number(
                selection, "total_score_delta_distance_m"
            ),
        },
        "per_candidate": per_candidate,
        "dropped_candidates": list(payload.get("dropped_candidates", [])),
        "active_candidates": list(payload.get("active_candidates", [])),
        "priority_order": list(payload.get("priority_order", [])),
    }


def build_payload(
    run_payloads: list[tuple[str, str, dict[str, object]]],
    title: str,
) -> dict[str, object]:
    run_entries = [
        build_run_entry(city, run, payload)
        for city, run, payload in run_payloads
    ]
    aggregates = aggregate_runs(run_payloads)
    # Collect all candidate labels seen across runs (stable order)
    seen_candidates: list[str] = []
    for _, _, payload in run_payloads:
        for label in payload.get("active_candidates", []):
            if label not in seen_candidates:
                seen_candidates.append(label)
    return {
        "title": title,
        "candidates": seen_candidates,
        "aggregates": aggregates,
        "runs": run_entries,
    }


# ---------------------------------------------------------------------------
# Markdown renderer
# ---------------------------------------------------------------------------


def _fmt_pp(value: float) -> str:
    return f"{value:+.2f} pp"


def _fmt_m(value: float) -> str:
    return f"{value:+.3f} m"


def render_markdown(payload: dict[str, object]) -> str:
    title = payload.get("title", "PPC multi-candidate selector")
    aggregates = payload.get("aggregates", {})
    runs = payload.get("runs", [])

    baseline_weighted = float(
        aggregates.get("weighted_baseline_official_score_pct", 0.0)
    )
    selector_weighted = float(
        aggregates.get("weighted_selector_official_score_pct", 0.0)
    )
    delta_m = float(aggregates.get("selector_official_score_delta_m", 0.0))
    delta_pp = float(aggregates.get("selector_official_score_delta_pct", 0.0))
    min_delta = float(aggregates.get("min_official_score_delta_m", 0.0))
    max_delta = float(aggregates.get("max_official_score_delta_m", 0.0))
    total_cand_segs = int(aggregates.get("total_candidate_selected_segments", 0))
    dropped = aggregates.get("dropped_candidates_any_run", [])

    lines = [
        f"# {title}",
        "",
        "## Aggregate",
        "",
        "| metric | value |",
        "|---|---:|",
        f"| baseline weighted official | {baseline_weighted:.6f}% |",
        f"| multi-candidate selector weighted official | {selector_weighted:.6f}% |",
        f"| selector vs baseline | {_fmt_m(delta_m)} / {_fmt_pp(delta_pp)} |",
        f"| per-run delta min / max | {_fmt_m(min_delta)} / {_fmt_m(max_delta)} |",
        f"| total candidate-selected segments | {total_cand_segs} |",
    ]
    if dropped:
        lines.append(f"| dropped candidates (any run) | {', '.join(dropped)} |")
    lines += [
        "",
        "## Runs",
        "",
        "| run | baseline | selector | delta m | delta pp |",
        "|---|---:|---:|---:|---:|",
    ]
    for run_entry in runs:
        if not isinstance(run_entry, dict):
            continue
        key = run_entry.get("key", "?")
        bl = run_entry.get("baseline", {})
        sel = run_entry.get("selector", {})
        dlt = run_entry.get("delta_vs_baseline", {})
        bl_pct = float(bl.get("ppc_official_score_pct", 0.0))
        sel_pct = float(sel.get("ppc_official_score_pct", 0.0))
        dlt_m = float(dlt.get("ppc_official_score_distance_m", 0.0))
        dlt_pp = float(dlt.get("ppc_official_score_pct", 0.0))
        lines.append(
            f"| {key} | {bl_pct:.6f}% | {sel_pct:.6f}% |"
            f" {dlt_m:+.3f} | {dlt_pp:+.3f} pp |"
        )
    lines.append("")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------


def main() -> int:
    args = parse_args()
    runs = selected_runs(args.run)

    run_payloads: list[tuple[str, str, dict[str, object]]] = []
    for city, run in runs:
        key = run_key(city, run)
        summary_json = invoke_apply_for_run(args, city, run)
        payload = load_run_summary(key, summary_json)
        run_payloads.append((city, run, payload))

    matrix_payload = build_payload(run_payloads, args.title)

    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(
        json.dumps(matrix_payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    if args.markdown_output is not None:
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(
            render_markdown(matrix_payload),
            encoding="utf-8",
        )
    else:
        print(render_markdown(matrix_payload))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
