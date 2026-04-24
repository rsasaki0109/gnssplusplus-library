#!/usr/bin/env python3
"""Apply a PPC dual-profile selector across runs and aggregate the matrix."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = Path(__file__).resolve().parent
APPS_DIR = ROOT_DIR / "apps"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
if str(APPS_DIR) not in sys.path:
    sys.path.insert(0, str(APPS_DIR))

import analyze_ppc_dual_profile_selector_matrix as selector_matrix  # noqa: E402
import apply_ppc_dual_profile_selector as dual_selector  # noqa: E402
import generate_driving_comparison as comparison  # noqa: E402
import gnss_ppc_metrics as ppc_metrics  # noqa: E402


PPC_RUNS: tuple[tuple[str, str], ...] = (
    ("tokyo", "run1"),
    ("tokyo", "run2"),
    ("tokyo", "run3"),
    ("nagoya", "run1"),
    ("nagoya", "run2"),
    ("nagoya", "run3"),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument(
        "--ppc-run",
        action="append",
        default=[],
        metavar="CITY/RUN",
        help="PPC run to process, e.g. tokyo/run1. Repeat to override the default six runs.",
    )
    parser.add_argument(
        "--baseline-pos-template",
        default="output/ppc_coverage_matrix_floatreset10/{key}.pos",
        help="Baseline POS path template. Supports {city}, {run}, and {key}.",
    )
    parser.add_argument(
        "--candidate-pos-template",
        default="output/ppc_{key}_rtk_prefit_s5_jump0p5_matrixprofile.pos",
        help="Candidate POS path template. Supports {city}, {run}, and {key}.",
    )
    parser.add_argument(
        "--reference-csv-template",
        default="{dataset_root}/{city}/{run}/reference.csv",
        help="Reference CSV template. Supports {dataset_root}, {city}, {run}, and {key}.",
    )
    parser.add_argument(
        "--run-output-template",
        required=True,
        help="Selected POS output template. Supports {city}, {run}, and {key}.",
    )
    parser.add_argument(
        "--run-summary-template",
        default=None,
        help="Per-run summary JSON template. Defaults to <run-output-stem>_summary.json.",
    )
    parser.add_argument(
        "--run-segments-template",
        default=None,
        help="Per-run selected-segment CSV template. Defaults to <run-output-stem>_segments.csv.",
    )
    parser.add_argument(
        "--rule",
        default=None,
        help="Selector rule to apply. Mutually exclusive with --selector-summary-json.",
    )
    parser.add_argument(
        "--selector-summary-json",
        type=Path,
        default=None,
        help="Selector sweep JSON whose top rule should be applied.",
    )
    parser.add_argument(
        "--rule-index",
        type=int,
        default=1,
        help="1-based top_rules index when --selector-summary-json is used (default: 1).",
    )
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--threshold-m", type=float, default=0.50)
    parser.add_argument("--matrix-summary-json", type=Path, required=True)
    parser.add_argument("--matrix-markdown-output", type=Path, default=None)
    parser.add_argument("--matrix-output-png", type=Path, default=None)
    parser.add_argument(
        "--title",
        default="PPC dual-profile selector",
        help="Title used in matrix Markdown and PNG outputs.",
    )
    return parser.parse_args()


def parse_ppc_run(value: str) -> tuple[str, str]:
    normalized = value.replace("_", "/")
    city, separator, run = normalized.partition("/")
    if not separator or not city.strip() or not run.strip():
        raise SystemExit("--ppc-run must use CITY/RUN, e.g. tokyo/run1")
    return city.strip(), run.strip()


def run_key(city: str, run: str) -> str:
    return f"{city}_{run}"


def format_template(template: str, dataset_root: Path, city: str, run: str) -> Path:
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


def ensure_file(path: Path, label: str) -> None:
    if not path.exists():
        raise SystemExit(f"{label} does not exist: {path}")


def selected_run_artifacts(
    args: argparse.Namespace,
    city: str,
    run: str,
) -> tuple[Path, Path, Path]:
    out_pos = format_template(args.run_output_template, args.dataset_root, city, run)
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


def build_run(
    args: argparse.Namespace,
    city: str,
    run: str,
    rule_text: str,
) -> selector_matrix.RunSpec:
    key = run_key(city, run)
    reference_csv = format_template(args.reference_csv_template, args.dataset_root, city, run)
    baseline_pos = format_template(args.baseline_pos_template, args.dataset_root, city, run)
    candidate_pos = format_template(args.candidate_pos_template, args.dataset_root, city, run)
    out_pos, summary_json, segments_csv = selected_run_artifacts(args, city, run)

    ensure_file(reference_csv, f"{key} reference CSV")
    ensure_file(baseline_pos, f"{key} baseline POS")
    ensure_file(candidate_pos, f"{key} candidate POS")

    rule = dual_selector.parse_rule(rule_text)
    reference = comparison.read_reference_csv(reference_csv)
    baseline_epochs = comparison.read_libgnss_pos(baseline_pos)
    candidate_epochs = comparison.read_libgnss_pos(candidate_pos)
    baseline_records = ppc_metrics.ppc_official_segment_records(
        reference,
        baseline_epochs,
        args.match_tolerance_s,
        args.threshold_m,
    )
    candidate_records = ppc_metrics.ppc_official_segment_records(
        reference,
        candidate_epochs,
        args.match_tolerance_s,
        args.threshold_m,
    )
    segment_rows = dual_selector.all_segment_rows(
        baseline_records,
        candidate_records,
        "candidate",
    )
    dual_selector.augment_solution_tows(segment_rows, baseline_records, candidate_records)
    selected_epochs, selected_rows = dual_selector.selected_solution_epochs(
        reference,
        baseline_epochs,
        candidate_epochs,
        segment_rows,
        rule,
        args.match_tolerance_s,
    )
    if not selected_epochs:
        raise SystemExit(f"{key}: selector produced no solution epochs")

    dual_selector.write_pos(out_pos, selected_epochs)
    payload = dual_selector.build_payload(
        reference,
        baseline_epochs,
        candidate_epochs,
        selected_epochs,
        selected_rows,
        rule_text,
        args.match_tolerance_s,
        out_pos,
    )
    summary_json.parent.mkdir(parents=True, exist_ok=True)
    summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    dual_selector.write_segments_csv(segments_csv, selected_rows)
    return selector_matrix.RunSpec(key=key, path=summary_json)


def selected_runs(values: list[str]) -> list[tuple[str, str]]:
    return [parse_ppc_run(value) for value in values] if values else list(PPC_RUNS)


def write_matrix_outputs(payload: dict[str, object], args: argparse.Namespace) -> None:
    args.matrix_summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.matrix_summary_json.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    if args.matrix_markdown_output is not None:
        args.matrix_markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.matrix_markdown_output.write_text(
            selector_matrix.render_markdown(payload),
            encoding="utf-8",
        )
    if args.matrix_output_png is not None:
        selector_matrix.render_png(payload, args.matrix_output_png)


def main() -> int:
    args = parse_args()
    rule_text = dual_selector.load_rule_text(
        args.rule,
        args.selector_summary_json,
        args.rule_index,
    )
    run_specs = [
        build_run(args, city, run, rule_text)
        for city, run in selected_runs(args.ppc_run)
    ]
    matrix_runs = selector_matrix.load_runs(run_specs)
    payload = selector_matrix.build_payload(matrix_runs, args.title)
    write_matrix_outputs(payload, args)
    if args.matrix_markdown_output is None and args.matrix_output_png is None:
        print(selector_matrix.render_markdown(payload))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
