#!/usr/bin/env python3
"""Learned truth-blind leave-one-run-out PPC segment selector.

The model is trained only on non-holdout runs. Holdout selection uses predicted
delta only: for each official segment, choose the candidate row with the largest
predicted gain if it exceeds a threshold tuned on training runs.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import sys

import numpy as np
import pandas as pd
from sklearn.compose import ColumnTransformer
from sklearn.ensemble import HistGradientBoostingRegressor
from sklearn.impute import SimpleImputer
from sklearn.metrics import mean_absolute_error
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import OrdinalEncoder


SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import analyze_ppc_segment_selector_sweep as selector_sweep  # noqa: E402


CATEGORICAL_FEATURES = (
    "candidate_label",
    "status_transition",
    "baseline_status_name",
    "candidate_status_name",
)
NUMERIC_FEATURES = tuple(selector_sweep.NUMERIC_FEATURES)
BASE_COLUMNS = (
    "reference_index",
    "segment_distance_m",
    "score_delta_distance_m",
)
USE_COLUMNS = (*BASE_COLUMNS, *CATEGORICAL_FEATURES, *NUMERIC_FEATURES)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--segment-csv", action="append", default=[], metavar="LABEL=CSV")
    parser.add_argument("--top-thresholds", type=int, default=32)
    parser.add_argument("--max-iter", type=int, default=160)
    parser.add_argument("--learning-rate", type=float, default=0.05)
    parser.add_argument("--l2-regularization", type=float, default=0.1)
    parser.add_argument("--changed-weight", type=float, default=80.0)
    parser.add_argument(
        "--train-changed-only",
        action="store_true",
        help="Fit the model only on rows with non-zero score delta.",
    )
    parser.add_argument(
        "--rank-objective",
        choices=selector_sweep.RANK_OBJECTIVES,
        default="robust",
    )
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--markdown-output", type=Path, default=None)
    return parser.parse_args()


def rounded(value: float | None) -> float | None:
    return None if value is None else round(float(value), 6)


def read_frame(specs: list[selector_sweep.SegmentCsvSpec]) -> pd.DataFrame:
    frames = []
    for spec in specs:
        if not spec.path.exists():
            raise SystemExit(f"{spec.label}: missing segment CSV: {spec.path}")
        frame = pd.read_csv(
            spec.path,
            usecols=lambda column: column in USE_COLUMNS,
            low_memory=False,
        )
        frame["run_label"] = spec.label
        frames.append(frame)
    if not frames:
        raise SystemExit("No segment CSVs loaded")
    frame = pd.concat(frames, ignore_index=True)
    frame["reference_index"] = pd.to_numeric(frame["reference_index"], errors="coerce").astype(
        "int64"
    )
    for column in ("segment_distance_m", "score_delta_distance_m", *NUMERIC_FEATURES):
        frame[column] = pd.to_numeric(frame[column], errors="coerce")
    frame["segment_distance_m"] = frame["segment_distance_m"].fillna(0.0)
    frame["score_delta_distance_m"] = frame["score_delta_distance_m"].fillna(0.0)
    for column in CATEGORICAL_FEATURES:
        frame[column] = frame[column].fillna("").astype(str)
    return frame


def make_model(args: argparse.Namespace) -> Pipeline:
    preprocessor = ColumnTransformer(
        transformers=[
            (
                "cat",
                OrdinalEncoder(
                    handle_unknown="use_encoded_value",
                    unknown_value=-1,
                    encoded_missing_value=-1,
                ),
                list(CATEGORICAL_FEATURES),
            ),
            (
                "num",
                SimpleImputer(strategy="median"),
                list(NUMERIC_FEATURES),
            ),
        ],
        remainder="drop",
        verbose_feature_names_out=False,
    )
    regressor = HistGradientBoostingRegressor(
        loss="squared_error",
        learning_rate=args.learning_rate,
        max_iter=args.max_iter,
        l2_regularization=args.l2_regularization,
        max_leaf_nodes=31,
        random_state=20260505,
    )
    return Pipeline([("preprocess", preprocessor), ("model", regressor)])


def selected_for_threshold(frame: pd.DataFrame, threshold: float) -> pd.DataFrame:
    eligible = frame[frame["predicted_delta_m"] > threshold]
    if eligible.empty:
        return eligible
    index = eligible.groupby(["run_label", "reference_index"], sort=False)[
        "predicted_delta_m"
    ].idxmax()
    return eligible.loc[index]


def score_selection(selected: pd.DataFrame, run_labels: list[str]) -> dict[str, object]:
    gain = float(selected["score_delta_distance_m"].clip(lower=0.0).sum())
    loss = float(selected["score_delta_distance_m"].clip(upper=0.0).sum())
    distance = float(selected["segment_distance_m"].sum())
    selected_abs = gain + abs(loss)
    by_run = []
    run_deltas = []
    for run_label in run_labels:
        run_selected = selected[selected["run_label"] == run_label]
        run_gain = float(run_selected["score_delta_distance_m"].clip(lower=0.0).sum())
        run_loss = float(run_selected["score_delta_distance_m"].clip(upper=0.0).sum())
        net = run_gain + run_loss
        run_deltas.append(net)
        by_run.append(
            {
                "run_label": run_label,
                "selected_segments": int(len(run_selected)),
                "selected_score_delta_distance_m": rounded(net),
                "selected_gain_distance_m": rounded(run_gain),
                "selected_loss_distance_m": rounded(run_loss),
                "selected_candidates": dict(
                    Counter(run_selected["candidate_label"]).most_common(12)
                ),
            }
        )
    return {
        "selected_segments": int(len(selected)),
        "selected_distance_m": rounded(distance),
        "selected_score_delta_distance_m": rounded(gain + loss),
        "selected_gain_distance_m": rounded(gain),
        "selected_loss_distance_m": rounded(loss),
        "distance_precision_pct": rounded(
            100.0 * gain / selected_abs if selected_abs > 0.0 else None
        ),
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
    raise ValueError(f"unsupported objective: {objective}")


def threshold_candidates(predictions: np.ndarray, top_thresholds: int) -> list[float]:
    finite = predictions[np.isfinite(predictions)]
    if finite.size == 0:
        return [0.0]
    percentiles = np.linspace(0, 100, max(2, top_thresholds))
    values = [0.0, float(finite.min()) - 1e-9]
    values.extend(float(value) for value in np.percentile(finite, percentiles))
    positive = finite[finite > 0.0]
    if positive.size:
        values.extend(float(value) for value in np.percentile(positive, percentiles))
    return sorted(set(round(value, 9) for value in values))


def tune_threshold(
    train: pd.DataFrame,
    *,
    run_labels: list[str],
    top_thresholds: int,
    rank_objective: str,
) -> tuple[float, dict[str, object]]:
    ranked = []
    for threshold in threshold_candidates(
        train["predicted_delta_m"].to_numpy(),
        top_thresholds,
    ):
        selected = selected_for_threshold(train, threshold)
        summary = score_selection(selected, run_labels)
        summary["threshold"] = threshold
        ranked.append(summary)
    ranked.sort(key=lambda item: rank_key(item, rank_objective), reverse=True)
    best = ranked[0]
    return float(best["threshold"]), best


def build_fold(frame: pd.DataFrame, holdout_run: str, args: argparse.Namespace) -> dict[str, object]:
    train = frame[frame["run_label"] != holdout_run].copy()
    holdout = frame[frame["run_label"] == holdout_run].copy()
    run_labels = sorted(train["run_label"].unique())
    if train.empty or holdout.empty:
        raise SystemExit(f"{holdout_run}: empty train or holdout split")

    model = make_model(args)
    fit_frame = (
        train[np.abs(train["score_delta_distance_m"]) > 1e-9].copy()
        if args.train_changed_only
        else train
    )
    if fit_frame.empty:
        fit_frame = train
    y_train = fit_frame["score_delta_distance_m"].to_numpy()
    sample_weight = np.ones(len(fit_frame), dtype=float)
    sample_weight += args.changed_weight * (np.abs(y_train) > 1e-9)
    sample_weight += args.changed_weight * 0.25 * (y_train > 0.0)
    model.fit(
        fit_frame[list(CATEGORICAL_FEATURES + NUMERIC_FEATURES)],
        y_train,
        model__sample_weight=sample_weight,
    )

    train["predicted_delta_m"] = model.predict(train[list(CATEGORICAL_FEATURES + NUMERIC_FEATURES)])
    holdout["predicted_delta_m"] = model.predict(
        holdout[list(CATEGORICAL_FEATURES + NUMERIC_FEATURES)]
    )
    threshold, train_score = tune_threshold(
        train,
        run_labels=run_labels,
        top_thresholds=args.top_thresholds,
        rank_objective=args.rank_objective,
    )
    holdout_selected = selected_for_threshold(holdout, threshold)
    holdout_score = score_selection(holdout_selected, [holdout_run])

    changed_train = train[np.abs(train["score_delta_distance_m"]) > 1e-9]
    changed_mae = (
        mean_absolute_error(
            changed_train["score_delta_distance_m"],
            changed_train["predicted_delta_m"],
        )
        if not changed_train.empty
        else None
    )
    return {
        "holdout_run": holdout_run,
        "threshold": rounded(threshold),
        "train_rows": int(len(train)),
        "fit_rows": int(len(fit_frame)),
        "holdout_rows": int(len(holdout)),
        "train_changed_rows": int(len(changed_train)),
        "train_changed_mae_m": rounded(changed_mae),
        "train_selected_score_delta_distance_m": train_score[
            "selected_score_delta_distance_m"
        ],
        "train_selected_gain_distance_m": train_score["selected_gain_distance_m"],
        "train_selected_loss_distance_m": train_score["selected_loss_distance_m"],
        "train_selected_segments": train_score["selected_segments"],
        "train_negative_run_count": train_score["negative_run_count"],
        "train_min_run_score_delta_distance_m": train_score[
            "min_run_score_delta_distance_m"
        ],
        "train_selected_candidates": train_score["selected_candidates"],
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
    runs = sorted(frame["run_label"].unique())
    folds = [build_fold(frame, run_label, args) for run_label in runs]
    return {
        "runs": runs,
        "row_count": int(len(frame)),
        "candidate_count": int(frame["candidate_label"].nunique()),
        "reference_segment_groups": int(
            frame[["run_label", "reference_index"]].drop_duplicates().shape[0]
        ),
        "folds": folds,
        "aggregates": aggregate_folds(folds),
        "top_thresholds": args.top_thresholds,
        "max_iter": args.max_iter,
        "learning_rate": args.learning_rate,
        "l2_regularization": args.l2_regularization,
        "changed_weight": args.changed_weight,
        "train_changed_only": args.train_changed_only,
        "rank_objective": args.rank_objective,
        "categorical_features": list(CATEGORICAL_FEATURES),
        "numeric_features": list(NUMERIC_FEATURES),
    }


def fmt(value: object) -> str:
    return selector_sweep.fmt(value)


def render_markdown(payload: dict[str, object]) -> str:
    aggregates = dict(payload["aggregates"])
    lines = [
        "# PPC Learned Segment Selector Leave-One-Run-Out",
        "",
        "Holdout selection uses model predictions only; thresholds are tuned on training runs.",
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
        f"Changed-row sample weight: **{payload['changed_weight']}**",
        f"Train changed rows only: **{payload['train_changed_only']}**",
        "",
        "| holdout | threshold | train net m | train min run m | train neg runs | holdout net m | holdout gain m | holdout loss m | selected | changed MAE m |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for fold in payload["folds"]:
        lines.append(
            f"| {fold['holdout_run']} | "
            f"{fmt(fold['threshold'])} | "
            f"{fmt(fold['train_selected_score_delta_distance_m'])} | "
            f"{fmt(fold['train_min_run_score_delta_distance_m'])} | "
            f"{fold['train_negative_run_count']} | "
            f"{fmt(fold['holdout_selected_score_delta_distance_m'])} | "
            f"{fmt(fold['holdout_selected_gain_distance_m'])} | "
            f"{fmt(fold['holdout_selected_loss_distance_m'])} | "
            f"{fold['holdout_selected_segments']} | "
            f"{fmt(fold['train_changed_mae_m'])} |"
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
