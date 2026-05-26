#!/usr/bin/env python3
"""Build PPC candidate features and run the candidate feature ranker matrix."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = ROOT_DIR / "scripts"
BUILD_FEATURES_SCRIPT = SCRIPTS_DIR / "build_ppc_candidate_feature_table.py"
EVAL_RANKER_SCRIPT = SCRIPTS_DIR / "eval_ppc_candidate_feature_ranker.py"

PPC_RUNS: tuple[tuple[str, str], ...] = (
    ("tokyo", "run1"),
    ("tokyo", "run2"),
    ("tokyo", "run3"),
    ("nagoya", "run1"),
    ("nagoya", "run2"),
    ("nagoya", "run3"),
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description="Build PPC candidate feature tables and run LORO ranker evaluation.",
    )
    parser.add_argument(
        "--run",
        action="append",
        default=[],
        metavar="CITY/RUN",
        help="PPC run to process, e.g. tokyo/run1. Repeat to override all six runs.",
    )
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=Path("/datasets/PPC-Dataset"),
        help="Root directory containing city/run/reference.csv files.",
    )
    parser.add_argument(
        "--reference-csv-template",
        default="{dataset_root}/{city}/{run}/reference.csv",
        help="Reference CSV template. Supports {dataset_root}, {city}, {run}, and {key}.",
    )
    parser.add_argument(
        "--candidate",
        dest="candidates",
        action="append",
        required=True,
        metavar="LABEL=PATH_TEMPLATE",
        help=(
            "Candidate POS path template. Supports {city}, {run}, and {key}. "
            "Repeat for each candidate."
        ),
    )
    parser.add_argument(
        "--features-dir",
        type=Path,
        default=Path("output/ppc_candidate_ranker_matrix/features"),
    )
    parser.add_argument(
        "--feature-csv-template",
        default="{features_dir}/{key}_features.csv",
        help="Feature CSV output template. Supports {features_dir}, {city}, {run}, and {key}.",
    )
    parser.add_argument(
        "--feature-summary-template",
        default="{features_dir}/{key}_features.summary.json",
        help="Feature summary JSON template. Supports {features_dir}, {city}, {run}, and {key}.",
    )
    parser.add_argument(
        "--reuse-features",
        action="store_true",
        help="Skip per-run feature generation when feature CSV and summary already exist.",
    )
    parser.add_argument(
        "--ranker-output-dir",
        type=Path,
        default=Path("output/ppc_candidate_ranker_matrix/ranker"),
    )
    parser.add_argument("--fold-mode", choices=("auto", "run", "time-block", "none"), default="run")
    parser.add_argument("--time-blocks", type=int, default=5)
    parser.add_argument("--model", choices=("ridge", "hist-gradient"), default="ridge")
    parser.add_argument("--ridge-alpha", type=float, default=1.0)
    parser.add_argument("--hgb-max-iter", type=int, default=160)
    parser.add_argument("--hgb-learning-rate", type=float, default=0.06)
    parser.add_argument("--hgb-max-leaf-nodes", type=int, default=31)
    parser.add_argument("--hgb-l2-regularization", type=float, default=0.02)
    parser.add_argument("--hgb-random-state", type=int, default=7)
    parser.add_argument(
        "--target-weight-mode",
        choices=("uniform", "path", "sqrt-path", "log-path", "pass-focus"),
        default="uniform",
    )
    parser.add_argument("--selection-mode", choices=("point", "viterbi"), default="point")
    parser.add_argument("--viterbi-switch-penalty-m", type=float, default=0.0)
    parser.add_argument("--viterbi-switch-margin-m", type=float, default=0.0)
    parser.add_argument("--viterbi-jump-excess-penalty-per-m", type=float, default=0.0)
    parser.add_argument("--viterbi-jump-excess-margin-m", type=float, default=2.0)
    parser.add_argument("--viterbi-max-gap-s", type=float, default=2.0)
    parser.add_argument("--viterbi-max-segment-epochs", type=int, default=0)
    parser.add_argument("--no-label-features", action="store_true")
    parser.add_argument(
        "--exclude-label",
        action="append",
        default=[],
        help="Candidate label to exclude from ranker training/selection. Repeatable.",
    )
    parser.add_argument("--postselect-override-to-label", default="")
    parser.add_argument(
        "--postselect-override-from-label",
        action="append",
        default=[],
        help="Selected label eligible for postselect override. Repeatable.",
    )
    parser.add_argument(
        "--postselect-override-from-status-name",
        action="append",
        default=[],
        help="Selected status_name eligible for postselect override. Repeatable.",
    )
    parser.add_argument("--postselect-override-rank-accel-le-selected", action="store_true")
    parser.add_argument("--postselect-override-rank-jump-le-selected", action="store_true")
    parser.add_argument("--postselect-override-min-smooth-outlier-score", type=float, default=None)
    parser.add_argument(
        "--postselect-override-rule",
        action="append",
        default=[],
        help="Additional postselect override rule forwarded to eval_ppc_candidate_feature_ranker.py.",
    )
    parser.add_argument("--threshold-m", type=float, default=0.50)
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=None,
        help="Matrix driver summary JSON. Defaults to <ranker-output-dir>/ppc_candidate_ranker_matrix_summary.json.",
    )
    parser.add_argument(
        "--markdown-output",
        type=Path,
        default=None,
        help="Matrix driver Markdown. Defaults to <ranker-output-dir>/ppc_candidate_ranker_matrix.md.",
    )
    return parser.parse_args(argv)


def run_key(city: str, run: str) -> str:
    return f"{city}_{run}"


def parse_run_arg(value: str) -> tuple[str, str]:
    normalized = value.replace("_", "/")
    city, separator, run = normalized.partition("/")
    if not separator or not city.strip() or not run.strip():
        raise SystemExit("--run must use CITY/RUN, e.g. tokyo/run1")
    return city.strip(), run.strip()


def selected_runs(values: list[str]) -> list[tuple[str, str]]:
    return [parse_run_arg(value) for value in values] if values else list(PPC_RUNS)


def format_template(
    template: str,
    *,
    dataset_root: Path,
    features_dir: Path,
    city: str,
    run: str,
) -> Path:
    key = run_key(city, run)
    return Path(
        template.format(
            dataset_root=str(dataset_root),
            features_dir=str(features_dir),
            city=city,
            run=run,
            key=key,
        )
    )


def parse_candidate_spec(spec: str) -> tuple[str, str]:
    if "=" not in spec:
        raise SystemExit(f"--candidate must use LABEL=PATH_TEMPLATE, got: {spec!r}")
    label, template = spec.split("=", 1)
    label = label.strip()
    template = template.strip()
    if not label or not template:
        raise SystemExit(f"--candidate must use LABEL=PATH_TEMPLATE, got: {spec!r}")
    return label, template


def run_artifacts(
    args: argparse.Namespace,
    city: str,
    run: str,
) -> tuple[Path, Path, Path]:
    reference_csv = format_template(
        args.reference_csv_template,
        dataset_root=args.dataset_root,
        features_dir=args.features_dir,
        city=city,
        run=run,
    )
    feature_csv = format_template(
        args.feature_csv_template,
        dataset_root=args.dataset_root,
        features_dir=args.features_dir,
        city=city,
        run=run,
    )
    feature_summary = format_template(
        args.feature_summary_template,
        dataset_root=args.dataset_root,
        features_dir=args.features_dir,
        city=city,
        run=run,
    )
    return reference_csv, feature_csv, feature_summary


def build_feature_argv(
    args: argparse.Namespace,
    city: str,
    run: str,
    reference_csv: Path,
    feature_csv: Path,
    feature_summary: Path,
) -> list[str]:
    key = run_key(city, run)
    argv = [
        sys.executable,
        str(BUILD_FEATURES_SCRIPT),
        "--run-id",
        key,
        "--reference-csv",
        str(reference_csv),
        "--out-csv",
        str(feature_csv),
        "--summary-json",
        str(feature_summary),
        "--match-tolerance-s",
        str(args.match_tolerance_s),
        "--threshold-m",
        str(args.threshold_m),
    ]
    for candidate in args.candidates:
        label, template = parse_candidate_spec(candidate)
        path = format_template(
            template,
            dataset_root=args.dataset_root,
            features_dir=args.features_dir,
            city=city,
            run=run,
        )
        argv += ["--candidate", f"{label}={path}"]
    return argv


def invoke_features_for_run(
    args: argparse.Namespace,
    city: str,
    run: str,
) -> tuple[Path, Path, Path]:
    reference_csv, feature_csv, feature_summary = run_artifacts(args, city, run)
    if args.reuse_features and feature_csv.exists() and feature_summary.exists():
        return reference_csv, feature_csv, feature_summary
    feature_csv.parent.mkdir(parents=True, exist_ok=True)
    feature_summary.parent.mkdir(parents=True, exist_ok=True)
    argv = build_feature_argv(args, city, run, reference_csv, feature_csv, feature_summary)
    result = subprocess.run(argv, check=False)
    if result.returncode != 0:
        raise SystemExit(
            f"{run_key(city, run)}: build_ppc_candidate_feature_table.py exited "
            f"with code {result.returncode}"
        )
    return reference_csv, feature_csv, feature_summary


def build_ranker_argv(
    args: argparse.Namespace,
    run_rows: list[dict[str, object]],
    summary_json: Path,
    markdown_output: Path,
) -> list[str]:
    argv = [
        sys.executable,
        str(EVAL_RANKER_SCRIPT),
        "--fold-mode",
        args.fold_mode,
        "--time-blocks",
        str(args.time_blocks),
        "--model",
        args.model,
        "--ridge-alpha",
        str(args.ridge_alpha),
        "--hgb-max-iter",
        str(args.hgb_max_iter),
        "--hgb-learning-rate",
        str(args.hgb_learning_rate),
        "--hgb-max-leaf-nodes",
        str(args.hgb_max_leaf_nodes),
        "--hgb-l2-regularization",
        str(args.hgb_l2_regularization),
        "--hgb-random-state",
        str(args.hgb_random_state),
        "--target-weight-mode",
        args.target_weight_mode,
        "--selection-mode",
        args.selection_mode,
        "--viterbi-switch-penalty-m",
        str(args.viterbi_switch_penalty_m),
        "--viterbi-switch-margin-m",
        str(args.viterbi_switch_margin_m),
        "--viterbi-jump-excess-penalty-per-m",
        str(args.viterbi_jump_excess_penalty_per_m),
        "--viterbi-jump-excess-margin-m",
        str(args.viterbi_jump_excess_margin_m),
        "--viterbi-max-gap-s",
        str(args.viterbi_max_gap_s),
        "--viterbi-max-segment-epochs",
        str(args.viterbi_max_segment_epochs),
        "--threshold-m",
        str(args.threshold_m),
        "--match-tolerance-s",
        str(args.match_tolerance_s),
        "--out-dir",
        str(args.ranker_output_dir),
        "--out-pos-dir",
        str(args.ranker_output_dir / "pos"),
        "--summary-json",
        str(summary_json),
        "--markdown-output",
        str(markdown_output),
    ]
    if args.no_label_features:
        argv.append("--no-label-features")
    for label in args.exclude_label:
        argv += ["--exclude-label", label]
    if args.postselect_override_to_label:
        argv += ["--postselect-override-to-label", args.postselect_override_to_label]
    for label in args.postselect_override_from_label:
        argv += ["--postselect-override-from-label", label]
    for status_name in args.postselect_override_from_status_name:
        argv += ["--postselect-override-from-status-name", status_name]
    if args.postselect_override_rank_accel_le_selected:
        argv.append("--postselect-override-rank-accel-le-selected")
    if args.postselect_override_rank_jump_le_selected:
        argv.append("--postselect-override-rank-jump-le-selected")
    if args.postselect_override_min_smooth_outlier_score is not None:
        argv += [
            "--postselect-override-min-smooth-outlier-score",
            str(args.postselect_override_min_smooth_outlier_score),
        ]
    for rule in args.postselect_override_rule:
        argv += ["--postselect-override-rule", rule]
    for row in run_rows:
        argv += ["--input-csv", str(row["feature_csv"])]
    for row in run_rows:
        argv += ["--reference-map", f"{row['key']}={row['reference_csv']}"]
    return argv


def invoke_ranker(
    args: argparse.Namespace,
    run_rows: list[dict[str, object]],
    summary_json: Path,
    markdown_output: Path,
) -> None:
    args.ranker_output_dir.mkdir(parents=True, exist_ok=True)
    argv = build_ranker_argv(args, run_rows, summary_json, markdown_output)
    result = subprocess.run(argv, check=False)
    if result.returncode != 0:
        raise SystemExit(
            f"eval_ppc_candidate_feature_ranker.py exited with code {result.returncode}"
        )


def load_json(path: Path) -> dict[str, object]:
    if not path.exists():
        raise SystemExit(f"missing JSON output: {path}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise SystemExit(f"{path}: expected JSON object")
    return payload


def build_payload(
    args: argparse.Namespace,
    run_rows: list[dict[str, object]],
    ranker_summary_json: Path,
    ranker_markdown: Path,
) -> dict[str, object]:
    ranker_summary = load_json(ranker_summary_json)
    return {
        "runs": run_rows,
        "candidates": [parse_candidate_spec(spec)[0] for spec in args.candidates],
        "ranker_summary_json": str(ranker_summary_json),
        "ranker_markdown": str(ranker_markdown),
        "ranker_pos_dir": str(args.ranker_output_dir / "pos"),
        "fold_mode": args.fold_mode,
        "model": args.model,
        "target_weight_mode": args.target_weight_mode,
        "selection_mode": args.selection_mode,
        "viterbi_switch_penalty_m": args.viterbi_switch_penalty_m,
        "viterbi_switch_margin_m": args.viterbi_switch_margin_m,
        "viterbi_jump_excess_penalty_per_m": args.viterbi_jump_excess_penalty_per_m,
        "viterbi_jump_excess_margin_m": args.viterbi_jump_excess_margin_m,
        "viterbi_max_gap_s": args.viterbi_max_gap_s,
        "viterbi_max_segment_epochs": args.viterbi_max_segment_epochs,
        "include_label_features": not args.no_label_features,
        "excluded_labels": [label for label in args.exclude_label if label],
        "postselect_override": ranker_summary.get("postselect_override"),
        "ranker": {
            "epochs": ranker_summary.get("epochs"),
            "row_count": ranker_summary.get("row_count"),
            "baseline": ranker_summary.get("baseline"),
            "ranker": ranker_summary.get("ranker"),
            "oracle": ranker_summary.get("oracle"),
            "pos_metrics": ranker_summary.get("pos_metrics"),
            "pos_metric_delta_vs_baseline": ranker_summary.get(
                "pos_metric_delta_vs_baseline"
            ),
        },
    }


def fmt(value: object) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)


def render_markdown(payload: dict[str, object]) -> str:
    ranker = payload.get("ranker", {})
    pos_metrics = ranker.get("pos_metrics", {}) if isinstance(ranker, dict) else {}
    delta = (
        ranker.get("pos_metric_delta_vs_baseline", {})
        if isinstance(ranker, dict)
        else {}
    )
    ranker_delta = delta.get("ranker", {}) if isinstance(delta, dict) else {}
    lines = [
        "# PPC Candidate Ranker Matrix",
        "",
        f"Runs: **{len(payload.get('runs', []))}**",
        f"Candidates: **{', '.join(str(x) for x in payload.get('candidates', []))}**",
        f"Fold mode: **{payload.get('fold_mode')}**",
        f"Model: **{payload.get('model')}**",
        f"Target weight mode: **{payload.get('target_weight_mode')}**",
        f"Selection mode: **{payload.get('selection_mode')}**",
        f"Label features: **{payload.get('include_label_features')}**",
        "",
        "| strategy | official score pct | 3D 50cm ref pct | mean h m |",
        "|---|---:|---:|---:|",
    ]
    if isinstance(pos_metrics, dict):
        for strategy in ("baseline", "ranker", "oracle"):
            metrics = pos_metrics.get(strategy)
            if not isinstance(metrics, dict):
                continue
            lines.append(
                f"| {strategy} | {fmt(metrics.get('ppc_official_score_pct'))} | "
                f"{fmt(metrics.get('ppc_score_3d_50cm_ref_pct'))} | "
                f"{fmt(metrics.get('mean_h_m'))} |"
            )
    lines.extend(
        [
            "",
            "Ranker official delta vs baseline: "
            f"**{fmt(ranker_delta.get('ppc_official_score_pct') if isinstance(ranker_delta, dict) else None)} pp**",
            "",
            f"Ranker summary: `{payload.get('ranker_summary_json')}`",
            f"Ranker POS dir: `{payload.get('ranker_pos_dir')}`",
            "",
        ]
    )
    return "\n".join(lines)


def write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    runs = selected_runs(args.run)
    run_rows: list[dict[str, object]] = []
    for city, run in runs:
        reference_csv, feature_csv, feature_summary = invoke_features_for_run(args, city, run)
        run_rows.append(
            {
                "key": run_key(city, run),
                "city": city,
                "run": run,
                "reference_csv": str(reference_csv),
                "feature_csv": str(feature_csv),
                "feature_summary_json": str(feature_summary),
            }
        )

    ranker_summary_json = args.ranker_output_dir / "ppc_candidate_ranker_summary.json"
    ranker_markdown = args.ranker_output_dir / "ppc_candidate_ranker.md"
    invoke_ranker(args, run_rows, ranker_summary_json, ranker_markdown)

    summary_json = args.summary_json or args.ranker_output_dir / "ppc_candidate_ranker_matrix_summary.json"
    markdown_output = args.markdown_output or args.ranker_output_dir / "ppc_candidate_ranker_matrix.md"
    payload = build_payload(args, run_rows, ranker_summary_json, ranker_markdown)
    write_json(summary_json, payload)
    markdown_output.parent.mkdir(parents=True, exist_ok=True)
    markdown_output.write_text(render_markdown(payload) + "\n", encoding="utf-8")
    print(
        "PPC candidate ranker matrix: "
        f"runs={len(run_rows)} out={summary_json} ranker={ranker_summary_json}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
