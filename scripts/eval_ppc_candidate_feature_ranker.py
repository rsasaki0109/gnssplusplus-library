#!/usr/bin/env python3
"""Evaluate a truth-trained PPC candidate ranker from feature-table CSVs.

Input rows come from ``build_ppc_candidate_feature_table.py``.  The model uses
only solver-native deployable columns for selection; reference-derived
``err_3d_m`` is used as the training/evaluation target.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import json
import math
import os
from pathlib import Path
import sys

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"
SCRIPTS_DIR = ROOT_DIR / "scripts"
for path in (APPS_DIR, SCRIPTS_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import generate_driving_comparison as comparison  # noqa: E402
import gnss_ppc_metrics as ppc_metrics  # noqa: E402


NUMERIC_FEATURES = [
    "status",
    "sats",
    "ratio",
    "baseline_m",
    "rms",
    "abs_max",
    "update_rows",
    "phase_update_rows",
    "code_update_rows",
    "suppressed_outliers",
    "nis",
    "nis_per_obs",
    "selector_base_score",
    "rank_by_selector_score",
    "rank_by_rms",
    "n_candidates_in_epoch",
    "cluster_size_50cm",
    "cluster_size_25cm",
    "cluster_size_10cm",
    "max_cluster_size_50cm",
    "is_in_max_cluster_50cm",
    "n_clusters_50cm",
    "dist_to_median_m",
    "dist_to_max_cluster_centroid_m",
    "candidate_jump_m",
    "rank_by_candidate_jump",
    "rank_by_delta_pos_accel",
    "candidate_jump_minus_min_m",
    "delta_pos_accel_minus_min_m",
    "dist_to_median_over_jump",
    "dist_to_max_cluster_over_jump",
    "smooth_outlier_score",
    "delta_pos_norm_m",
    "delta_pos_2step_m",
    "delta_pos_3step_m",
    "delta_pos_horizontal_m",
    "delta_pos_vertical_m",
    "delta_pos_accel_m",
    "libgnss_low_sat_risk",
    "libgnss_residual_risk",
    "libgnss_consensus_risk",
    "libgnss_jump_risk",
    "libgnss_nonfix_risk",
    "libgnss_urban_risk_score",
]

SELECTION_FIELDNAMES = [
    "fold",
    "run_id",
    "week",
    "tow",
    "baseline_label",
    "ranker_label",
    "oracle_label",
    "baseline_status_name",
    "ranker_status_name",
    "oracle_status_name",
    "baseline_err_3d_m",
    "ranker_err_3d_m",
    "oracle_err_3d_m",
    "baseline_selector_base_score",
    "ranker_pred_err_m",
    "oracle_pred_err_m",
    "baseline_risk",
    "ranker_risk",
    "oracle_risk",
    "selection_changed",
    "ranker_minus_baseline_err_m",
    "ranker_regret_m",
    "baseline_pass",
    "ranker_pass",
    "oracle_pass",
    "path_weight_m",
]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description="Evaluate a ridge ranker over PPC candidate feature CSVs.",
    )
    parser.add_argument(
        "--input-csv",
        action="append",
        required=True,
        metavar="CSV",
        help="Candidate feature CSV. Repeat to combine runs.",
    )
    parser.add_argument(
        "--fold-mode",
        choices=("auto", "run", "time-block", "none"),
        default="auto",
        help="Validation split. auto uses run folds when possible, otherwise time blocks.",
    )
    parser.add_argument("--time-blocks", type=int, default=5)
    parser.add_argument(
        "--model",
        choices=("ridge", "hist-gradient"),
        default="ridge",
        help="Candidate error model. hist-gradient requires scikit-learn.",
    )
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
        help=(
            "Training row weights. pass-focus emphasizes path-weighted rows near "
            "the 50cm pass/fail boundary."
        ),
    )
    parser.add_argument(
        "--selection-mode",
        choices=("point", "viterbi"),
        default="point",
        help="How to turn per-candidate predictions into a per-run trajectory.",
    )
    parser.add_argument(
        "--viterbi-switch-penalty-m",
        type=float,
        default=0.0,
        help="Additional Viterbi cost when the selected candidate label changes.",
    )
    parser.add_argument(
        "--viterbi-switch-margin-m",
        type=float,
        default=0.0,
        help=(
            "Reduce switch penalty when the current epoch's point-best candidate "
            "beats the runner-up by this predicted-error margin. Non-positive disables gating."
        ),
    )
    parser.add_argument(
        "--viterbi-jump-excess-penalty-per-m",
        type=float,
        default=0.0,
        help=(
            "Viterbi transition cost per meter above the epoch-pair minimum "
            "transition distance plus margin."
        ),
    )
    parser.add_argument(
        "--viterbi-jump-excess-margin-m",
        type=float,
        default=2.0,
        help="Free margin above the epoch-pair minimum transition distance.",
    )
    parser.add_argument(
        "--viterbi-max-gap-s",
        type=float,
        default=2.0,
        help="Reset the Viterbi path across larger time gaps. Non-positive disables the reset.",
    )
    parser.add_argument(
        "--viterbi-max-segment-epochs",
        type=int,
        default=0,
        help="Reset Viterbi after this many epochs. Non-positive keeps full contiguous segments.",
    )
    parser.add_argument("--threshold-m", type=float, default=0.50)
    parser.add_argument(
        "--no-label-features",
        action="store_true",
        help="Do not include candidate-label one-hot features.",
    )
    parser.add_argument(
        "--exclude-label",
        action="append",
        default=[],
        metavar="LABEL",
        help=(
            "Exclude a candidate label from training and selection. Repeat to "
            "evaluate pruned candidate pools without rewriting feature CSVs."
        ),
    )
    parser.add_argument(
        "--postselect-override-to-label",
        default="",
        metavar="LABEL",
        help=(
            "After point/Viterbi selection, replace selected epochs with this "
            "candidate label when all postselect override conditions match."
        ),
    )
    parser.add_argument(
        "--postselect-override-from-label",
        action="append",
        default=[],
        metavar="LABEL",
        help=(
            "Only postselect-override epochs whose originally selected label "
            "matches LABEL. Repeatable; omitted means any selected label."
        ),
    )
    parser.add_argument(
        "--postselect-override-from-status-name",
        action="append",
        default=[],
        metavar="STATUS",
        help=(
            "Only postselect-override epochs whose originally selected "
            "status_name matches STATUS. Repeatable; omitted means any status."
        ),
    )
    parser.add_argument(
        "--postselect-override-rank-accel-le-selected",
        action="store_true",
        help=(
            "Require the override candidate's rank_by_delta_pos_accel to be "
            "less than or equal to the originally selected candidate's rank."
        ),
    )
    parser.add_argument(
        "--postselect-override-rank-jump-le-selected",
        action="store_true",
        help=(
            "Require the override candidate's rank_by_candidate_jump to be "
            "less than or equal to the originally selected candidate's rank."
        ),
    )
    parser.add_argument(
        "--postselect-override-min-smooth-outlier-score",
        type=float,
        default=None,
        metavar="VALUE",
        help=(
            "Require the override candidate's smooth_outlier_score to be at "
            "least VALUE."
        ),
    )
    parser.add_argument(
        "--postselect-override-rule",
        action="append",
        default=[],
        metavar="SPEC",
        help=(
            "Additional postselect override rule. SPEC is comma-separated, e.g. "
            "from=libgnss_v5,to=gici_zeroarm,min_cluster50=4. Supported keys: "
            "from, to, from_status, to_status, min_smooth, min_cluster50, "
            "rank_accel_le_selected, rank_jump_le_selected. Repeatable."
        ),
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("output/ppc_candidate_feature_ranker"),
    )
    parser.add_argument("--rows-csv", type=Path, default=None)
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--markdown-output", type=Path, default=None)
    parser.add_argument(
        "--out-pos",
        type=Path,
        default=None,
        help="Optional libgnss++ POS file containing ranker-selected epochs.",
    )
    parser.add_argument(
        "--out-pos-dir",
        type=Path,
        default=None,
        help="Optional directory for per-run ranker-selected POS files.",
    )
    parser.add_argument(
        "--reference-csv",
        type=Path,
        default=None,
        help="Optional PPC reference.csv for official distance scoring of selected POS.",
    )
    parser.add_argument(
        "--reference-map",
        action="append",
        default=[],
        metavar="RUN_ID=CSV",
        help="Per-run PPC reference.csv for multi-run scoring. Repeat for each run_id.",
    )
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    return parser.parse_args(argv)


def to_float(value: object, default: float = math.nan) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def rounded(value: float | None) -> float | None:
    if value is None:
        return None
    value = float(value)
    return round(value, 6) if math.isfinite(value) else None


def read_rows(paths: list[Path]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for path in paths:
        if not path.exists():
            raise SystemExit(f"input CSV not found: {path}")
        with path.open(newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                target = to_float(row.get("err_3d_m"))
                if not math.isfinite(target):
                    continue
                row = dict(row)
                row["_source_csv"] = str(path)
                row["_row_index"] = len(rows)
                rows.append(row)
    if not rows:
        raise SystemExit("No rows with finite err_3d_m were loaded")
    return rows


def exclude_label_rows(rows: list[dict[str, object]], labels: list[str]) -> list[dict[str, object]]:
    excluded = {label for label in labels if label}
    if not excluded:
        return rows
    before_epochs = {epoch_key(row) for row in rows}
    filtered = [row for row in rows if str(row.get("label", "")) not in excluded]
    if not filtered:
        raise SystemExit("--exclude-label removed every candidate row")
    after_epochs = {epoch_key(row) for row in filtered}
    missing_epochs = before_epochs - after_epochs
    if missing_epochs:
        filtered.extend(row for row in rows if epoch_key(row) in missing_epochs)
    for row_index, row in enumerate(filtered):
        row["_row_index"] = row_index
    return filtered


def epoch_key(row: dict[str, object]) -> tuple[str, str, float]:
    return (
        str(row.get("run_id", "")),
        str(row.get("week", "")),
        round(to_float(row.get("tow"), 0.0), 6),
    )


def sort_epoch_key(key: tuple[str, str, float]) -> tuple[str, float, float]:
    run_id, week, tow = key
    return run_id, to_float(week, 0.0), tow


def row_target(row: dict[str, object]) -> float:
    return max(0.0, to_float(row.get("err_3d_m"), 0.0))


def row_score(row: dict[str, object], key: str, default: float = math.inf) -> float:
    return to_float(row.get(key), default)


def feature_spec(
    rows: list[dict[str, object]],
    *,
    include_label_features: bool,
) -> tuple[list[str], list[str]]:
    present_numeric = [
        name
        for name in NUMERIC_FEATURES
        if any(str(row.get(name, "")) != "" for row in rows)
    ]
    labels = sorted({str(row.get("label", "")) for row in rows}) if include_label_features else []
    return present_numeric, labels


def raw_feature_matrix(
    rows: list[dict[str, object]],
    numeric_features: list[str],
    label_features: list[str],
) -> np.ndarray:
    label_index = {label: idx for idx, label in enumerate(label_features)}
    matrix = np.full((len(rows), len(numeric_features) + len(label_features)), np.nan, dtype=float)
    for row_idx, row in enumerate(rows):
        for col_idx, name in enumerate(numeric_features):
            matrix[row_idx, col_idx] = to_float(row.get(name))
        if label_features:
            offset = len(numeric_features)
            label = str(row.get("label", ""))
            idx = label_index.get(label)
            if idx is not None:
                matrix[row_idx, offset + idx] = 1.0
            for idx2 in range(len(label_features)):
                col = offset + idx2
                if math.isnan(matrix[row_idx, col]):
                    matrix[row_idx, col] = 0.0
    return matrix


def impute_and_scale_train(matrix: np.ndarray) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    medians = np.nanmedian(matrix, axis=0)
    medians = np.where(np.isfinite(medians), medians, 0.0)
    imputed = np.where(np.isfinite(matrix), matrix, medians)
    means = imputed.mean(axis=0)
    scales = imputed.std(axis=0)
    scales = np.where(scales > 1e-12, scales, 1.0)
    return (imputed - means) / scales, {"medians": medians, "means": means, "scales": scales}


def impute_and_scale_apply(matrix: np.ndarray, prep: dict[str, np.ndarray]) -> np.ndarray:
    imputed = np.where(np.isfinite(matrix), matrix, prep["medians"])
    return (imputed - prep["means"]) / prep["scales"]


def target_weight(row: dict[str, object], mode: str) -> float:
    path_weight = to_float(row.get("path_weight_m"), 1.0)
    if not math.isfinite(path_weight) or path_weight <= 0.0:
        path_weight = 1.0
    if mode == "uniform":
        return 1.0
    if mode == "path":
        return path_weight
    if mode == "sqrt-path":
        return math.sqrt(path_weight)
    if mode == "log-path":
        return max(math.log1p(path_weight), 1e-6)
    if mode == "pass-focus":
        weight = path_weight
        target = row_target(row)
        if 0.20 <= target <= 2.00:
            weight *= 3.0
        return weight
    raise ValueError(f"unsupported target weight mode: {mode}")


def fit_ridge(
    train_rows: list[dict[str, object]],
    numeric_features: list[str],
    label_features: list[str],
    alpha: float,
    target_weight_mode: str,
) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    matrix = raw_feature_matrix(train_rows, numeric_features, label_features)
    x_scaled, prep = impute_and_scale_train(matrix)
    design = np.column_stack([np.ones(len(train_rows)), x_scaled])
    y = np.log1p(np.array([row_target(row) for row in train_rows], dtype=float))
    weights = np.array(
        [target_weight(row, target_weight_mode) for row in train_rows],
        dtype=float,
    )
    weight_mean = float(weights.mean()) if len(weights) else 1.0
    if not math.isfinite(weight_mean) or weight_mean <= 0.0:
        weight_mean = 1.0
    weights = weights / weight_mean
    sqrt_weights = np.sqrt(weights)
    weighted_design = design * sqrt_weights[:, None]
    weighted_y = y * sqrt_weights
    reg = np.eye(design.shape[1], dtype=float) * max(0.0, alpha)
    reg[0, 0] = 0.0
    lhs = weighted_design.T @ weighted_design + reg
    rhs = weighted_design.T @ weighted_y
    try:
        coef = np.linalg.solve(lhs, rhs)
    except np.linalg.LinAlgError:
        coef = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
    return coef, prep


def row_weights(rows: list[dict[str, object]], mode: str) -> np.ndarray:
    weights = np.array([target_weight(row, mode) for row in rows], dtype=float)
    weight_mean = float(weights.mean()) if len(weights) else 1.0
    if not math.isfinite(weight_mean) or weight_mean <= 0.0:
        weight_mean = 1.0
    return weights / weight_mean


def fit_hist_gradient(
    train_rows: list[dict[str, object]],
    numeric_features: list[str],
    label_features: list[str],
    args: argparse.Namespace,
):
    try:
        from sklearn.ensemble import HistGradientBoostingRegressor
    except ImportError as exc:
        raise SystemExit(
            "--model hist-gradient requires scikit-learn to be installed"
        ) from exc

    matrix = raw_feature_matrix(train_rows, numeric_features, label_features)
    target = np.log1p(np.array([row_target(row) for row in train_rows], dtype=float))
    model = HistGradientBoostingRegressor(
        max_iter=args.hgb_max_iter,
        learning_rate=args.hgb_learning_rate,
        max_leaf_nodes=args.hgb_max_leaf_nodes,
        l2_regularization=args.hgb_l2_regularization,
        random_state=args.hgb_random_state,
    )
    model.fit(
        matrix,
        target,
        sample_weight=row_weights(train_rows, args.target_weight_mode),
    )
    return model


def predict_hist_gradient(
    rows: list[dict[str, object]],
    numeric_features: list[str],
    label_features: list[str],
    model,
) -> np.ndarray:
    matrix = raw_feature_matrix(rows, numeric_features, label_features)
    return np.maximum(0.0, np.expm1(model.predict(matrix)))


def predict_rows(
    rows: list[dict[str, object]],
    numeric_features: list[str],
    label_features: list[str],
    coef: np.ndarray,
    prep: dict[str, np.ndarray],
) -> np.ndarray:
    matrix = raw_feature_matrix(rows, numeric_features, label_features)
    x_scaled = impute_and_scale_apply(matrix, prep)
    design = np.column_stack([np.ones(len(rows)), x_scaled])
    pred_log = design @ coef
    return np.maximum(0.0, np.expm1(pred_log))


def run_ids(rows: list[dict[str, object]]) -> list[str]:
    return sorted({str(row.get("run_id", "")) for row in rows})


def build_folds(
    rows: list[dict[str, object]],
    *,
    fold_mode: str,
    time_blocks: int,
) -> list[tuple[str, set[tuple[str, str, float]]]]:
    keys = sorted({epoch_key(row) for row in rows}, key=sort_epoch_key)
    runs = run_ids(rows)
    mode = fold_mode
    if mode == "auto":
        mode = "run" if len(runs) >= 2 else "time-block"
    if mode == "none":
        return [("in_sample", set(keys))]
    if mode == "run":
        if len(runs) < 2:
            raise SystemExit("--fold-mode run requires at least two run_id values")
        return [
            (f"run:{run_id}", {key for key in keys if key[0] == run_id})
            for run_id in runs
        ]
    if mode == "time-block":
        if len(keys) < 2:
            return [("in_sample", set(keys))]
        n_folds = min(max(2, int(time_blocks)), len(keys))
        block_size = int(math.ceil(len(keys) / n_folds))
        folds = []
        for fold_idx in range(n_folds):
            block = keys[fold_idx * block_size : (fold_idx + 1) * block_size]
            if block:
                folds.append((f"block:{fold_idx + 1}/{n_folds}", set(block)))
        return folds
    raise ValueError(f"unsupported fold mode: {fold_mode}")


def train_predict_cv(
    rows: list[dict[str, object]],
    *,
    fold_mode: str,
    time_blocks: int,
    alpha: float,
    include_label_features: bool,
    target_weight_mode: str,
    args: argparse.Namespace,
) -> tuple[dict[int, float], list[dict[str, object]], list[str], list[str]]:
    numeric_features, label_features = feature_spec(
        rows,
        include_label_features=include_label_features,
    )
    folds = build_folds(rows, fold_mode=fold_mode, time_blocks=time_blocks)
    predictions: dict[int, float] = {}
    fold_summaries: list[dict[str, object]] = []
    all_keys = {epoch_key(row) for row in rows}
    for fold_name, holdout_keys in folds:
        if fold_name == "in_sample":
            train_rows = rows
        else:
            train_rows = [row for row in rows if epoch_key(row) not in holdout_keys]
        holdout_rows = [row for row in rows if epoch_key(row) in holdout_keys]
        if not train_rows or not holdout_rows:
            continue
        if args.model == "hist-gradient":
            model = fit_hist_gradient(train_rows, numeric_features, label_features, args)
            pred = predict_hist_gradient(
                holdout_rows,
                numeric_features,
                label_features,
                model,
            )
        else:
            coef, prep = fit_ridge(
                train_rows,
                numeric_features,
                label_features,
                alpha,
                target_weight_mode,
            )
            pred = predict_rows(holdout_rows, numeric_features, label_features, coef, prep)
        for row, value in zip(holdout_rows, pred):
            predictions[int(row["_row_index"])] = float(value)
        fold_summaries.append(
            {
                "fold": fold_name,
                "train_rows": len(train_rows),
                "holdout_rows": len(holdout_rows),
                "train_epochs": (
                    len(all_keys - holdout_keys)
                    if fold_name != "in_sample"
                    else len(all_keys)
                ),
                "holdout_epochs": len(holdout_keys),
            }
        )
    if len(predictions) != len(rows):
        missing = len(rows) - len(predictions)
        raise SystemExit(f"internal error: missing predictions for {missing} rows")
    return predictions, fold_summaries, numeric_features, label_features


def select_epoch(
    items: list[dict[str, object]],
    predictions: dict[int, float],
    ranker_override: dict[str, object] | None = None,
) -> tuple[dict[str, object], dict[str, object], dict[str, object]]:
    baseline = min(
        items,
        key=lambda row: (
            row_score(row, "selector_base_score"),
            row_score(row, "rank_by_rms"),
            str(row.get("label", "")),
        ),
    )
    ranker = ranker_override if ranker_override is not None else point_ranker_row(items, predictions)
    oracle = min(items, key=lambda row: (row_target(row), str(row.get("label", ""))))
    return baseline, ranker, oracle


def point_ranker_row(
    items: list[dict[str, object]],
    predictions: dict[int, float],
) -> dict[str, object]:
    return min(
        items,
        key=lambda row: (
            predictions[int(row["_row_index"])],
            row_score(row, "selector_base_score"),
            str(row.get("label", "")),
        ),
    )


def grouped_epoch_rows(
    rows: list[dict[str, object]],
) -> dict[tuple[str, str, float], list[dict[str, object]]]:
    grouped: dict[tuple[str, str, float], list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        grouped[epoch_key(row)].append(row)
    return {
        key: sorted(items, key=lambda row: str(row.get("label", "")))
        for key, items in sorted(grouped.items(), key=lambda item: sort_epoch_key(item[0]))
    }


def row_xyz(row: dict[str, object]) -> tuple[float, float, float] | None:
    xyz = (
        to_float(row.get("x_m")),
        to_float(row.get("y_m")),
        to_float(row.get("z_m")),
    )
    return xyz if all(math.isfinite(value) for value in xyz) else None


def transition_distance_m(left: dict[str, object], right: dict[str, object]) -> float:
    left_xyz = row_xyz(left)
    right_xyz = row_xyz(right)
    if left_xyz is None or right_xyz is None:
        return math.inf
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left_xyz, right_xyz)))


def min_transition_distance_m(
    previous_items: list[dict[str, object]],
    current_items: list[dict[str, object]],
) -> float:
    best = math.inf
    for previous in previous_items:
        for current in current_items:
            best = min(best, transition_distance_m(previous, current))
    return best


def transition_allowed(
    previous_key: tuple[str, str, float],
    current_key: tuple[str, str, float],
    max_gap_s: float,
) -> bool:
    if previous_key[0] != current_key[0] or previous_key[1] != current_key[1]:
        return False
    gap_s = current_key[2] - previous_key[2]
    if gap_s <= 0.0:
        return False
    return max_gap_s <= 0.0 or gap_s <= max_gap_s


def viterbi_unary_cost(row: dict[str, object], predictions: dict[int, float]) -> float:
    return (
        predictions[int(row["_row_index"])]
        + 1e-6 * row_score(row, "selector_base_score", 0.0)
    )


def viterbi_switch_advantages(
    items: list[dict[str, object]],
    predictions: dict[int, float],
) -> dict[int, float]:
    if len(items) < 2:
        return {int(row["_row_index"]): 0.0 for row in items}
    ranked = sorted(
        items,
        key=lambda row: (
            predictions[int(row["_row_index"])],
            row_score(row, "selector_base_score", 0.0),
            str(row.get("label", "")),
        ),
    )
    best = ranked[0]
    best_index = int(best["_row_index"])
    best_pred = predictions[best_index]
    second_pred = predictions[int(ranked[1]["_row_index"])]
    return {
        int(row["_row_index"]): max(0.0, second_pred - best_pred)
        if int(row["_row_index"]) == best_index
        else 0.0
        for row in items
    }


def switch_penalty_scale(advantage_m: float, switch_margin_m: float) -> float:
    if switch_margin_m <= 0.0 or not math.isfinite(advantage_m):
        return 1.0
    return max(0.0, 1.0 - max(0.0, advantage_m) / switch_margin_m)


def viterbi_transition_cost(
    previous: dict[str, object],
    current: dict[str, object],
    *,
    min_distance_m: float,
    switch_penalty_m: float,
    switch_margin_m: float,
    switch_advantage_m: float,
    jump_excess_penalty_per_m: float,
    jump_excess_margin_m: float,
) -> float:
    cost = 0.0
    if str(previous.get("label", "")) != str(current.get("label", "")):
        cost += max(0.0, switch_penalty_m) * switch_penalty_scale(
            switch_advantage_m,
            switch_margin_m,
        )
    if jump_excess_penalty_per_m > 0.0 and math.isfinite(min_distance_m):
        distance_m = transition_distance_m(previous, current)
        if math.isfinite(distance_m):
            excess_m = distance_m - min_distance_m - max(0.0, jump_excess_margin_m)
            cost += jump_excess_penalty_per_m * max(0.0, excess_m)
    return cost


def viterbi_select_segment(
    segment: list[tuple[tuple[str, str, float], list[dict[str, object]]]],
    predictions: dict[int, float],
    args: argparse.Namespace,
) -> dict[tuple[str, str, float], dict[str, object]]:
    if not segment:
        return {}

    scores: list[list[float]] = []
    backptrs: list[list[int]] = []
    first_items = segment[0][1]
    scores.append([viterbi_unary_cost(row, predictions) for row in first_items])
    backptrs.append([-1] * len(first_items))

    for epoch_index in range(1, len(segment)):
        previous_items = segment[epoch_index - 1][1]
        current_items = segment[epoch_index][1]
        previous_scores = scores[-1]
        min_distance = min_transition_distance_m(previous_items, current_items)
        switch_advantages = viterbi_switch_advantages(current_items, predictions)
        current_scores: list[float] = []
        current_backptrs: list[int] = []
        for current in current_items:
            best_score = math.inf
            best_index = 0
            for previous_index, previous in enumerate(previous_items):
                candidate_score = previous_scores[previous_index] + viterbi_transition_cost(
                    previous,
                    current,
                    min_distance_m=min_distance,
                    switch_penalty_m=args.viterbi_switch_penalty_m,
                    switch_margin_m=getattr(args, "viterbi_switch_margin_m", 0.0),
                    switch_advantage_m=switch_advantages[int(current["_row_index"])],
                    jump_excess_penalty_per_m=args.viterbi_jump_excess_penalty_per_m,
                    jump_excess_margin_m=args.viterbi_jump_excess_margin_m,
                )
                if candidate_score < best_score:
                    best_score = candidate_score
                    best_index = previous_index
            current_scores.append(best_score + viterbi_unary_cost(current, predictions))
            current_backptrs.append(best_index)
        scores.append(current_scores)
        backptrs.append(current_backptrs)

    final_items = segment[-1][1]
    state_index = min(
        range(len(final_items)),
        key=lambda index: (
            scores[-1][index],
            row_score(final_items[index], "selector_base_score", 0.0),
            str(final_items[index].get("label", "")),
        ),
    )
    selected: dict[tuple[str, str, float], dict[str, object]] = {}
    for epoch_index in range(len(segment) - 1, -1, -1):
        key, items = segment[epoch_index]
        selected[key] = items[state_index]
        state_index = backptrs[epoch_index][state_index]
        if state_index < 0 and epoch_index > 0:
            state_index = 0
    return selected


def viterbi_ranker_rows_by_epoch(
    grouped: dict[tuple[str, str, float], list[dict[str, object]]],
    predictions: dict[int, float],
    args: argparse.Namespace,
) -> dict[tuple[str, str, float], dict[str, object]]:
    out: dict[tuple[str, str, float], dict[str, object]] = {}
    by_run: dict[str, list[tuple[tuple[str, str, float], list[dict[str, object]]]]] = defaultdict(list)
    for key, items in grouped.items():
        by_run[key[0]].append((key, items))

    for run_items in by_run.values():
        segment: list[tuple[tuple[str, str, float], list[dict[str, object]]]] = []
        previous_key: tuple[str, str, float] | None = None
        for key, items in sorted(run_items, key=lambda item: sort_epoch_key(item[0])):
            if previous_key is not None and not transition_allowed(
                previous_key,
                key,
                args.viterbi_max_gap_s,
            ):
                out.update(viterbi_select_segment(segment, predictions, args))
                segment = []
            if (
                getattr(args, "viterbi_max_segment_epochs", 0) > 0
                and len(segment) >= args.viterbi_max_segment_epochs
            ):
                out.update(viterbi_select_segment(segment, predictions, args))
                segment = []
            segment.append((key, items))
            previous_key = key
        out.update(viterbi_select_segment(segment, predictions, args))
    return out


def ranker_rows_by_epoch(
    rows: list[dict[str, object]],
    predictions: dict[int, float],
    args: argparse.Namespace,
) -> dict[tuple[str, str, float], dict[str, object]]:
    grouped = grouped_epoch_rows(rows)
    if args.selection_mode == "point":
        return {
            key: point_ranker_row(items, predictions)
            for key, items in grouped.items()
        }
    if args.selection_mode == "viterbi":
        return viterbi_ranker_rows_by_epoch(grouped, predictions, args)
    raise ValueError(f"unsupported selection mode: {args.selection_mode}")


def split_rule_values(value: str) -> list[str]:
    return [part.strip() for part in value.split("|") if part.strip()]


def parse_postselect_rule_spec(spec: str) -> dict[str, object]:
    rule: dict[str, object] = {
        "from_labels": [],
        "to_label": "",
        "from_status_names": [],
        "to_status_names": [],
        "rank_accel_le_selected": False,
        "rank_jump_le_selected": False,
        "min_smooth_outlier_score": None,
        "min_cluster_size_50cm": None,
    }
    aliases = {
        "from": "from_labels",
        "from_label": "from_labels",
        "to": "to_label",
        "to_label": "to_label",
        "from_status": "from_status_names",
        "from_status_name": "from_status_names",
        "to_status": "to_status_names",
        "to_status_name": "to_status_names",
        "min_smooth": "min_smooth_outlier_score",
        "min_smooth_outlier_score": "min_smooth_outlier_score",
        "min_cluster50": "min_cluster_size_50cm",
        "min_cluster_size_50cm": "min_cluster_size_50cm",
    }
    for raw_part in spec.split(","):
        part = raw_part.strip()
        if not part:
            continue
        if "=" not in part:
            if part in ("rank_accel_le_selected", "rank_jump_le_selected"):
                rule[part] = True
                continue
            raise SystemExit(f"invalid --postselect-override-rule token: {part!r}")
        key, value = [text.strip() for text in part.split("=", 1)]
        key = aliases.get(key, key)
        if key in ("from_labels", "from_status_names", "to_status_names"):
            rule[key] = split_rule_values(value)
        elif key == "to_label":
            rule[key] = value
        elif key in ("min_smooth_outlier_score", "min_cluster_size_50cm"):
            rule[key] = float(value)
        elif key in ("rank_accel_le_selected", "rank_jump_le_selected"):
            rule[key] = value.lower() in ("1", "true", "yes", "on")
        else:
            raise SystemExit(f"unknown --postselect-override-rule key: {key!r}")
    if not rule["to_label"]:
        raise SystemExit("--postselect-override-rule requires to=LABEL")
    return rule


def legacy_postselect_rule(args: argparse.Namespace) -> dict[str, object] | None:
    to_label = getattr(args, "postselect_override_to_label", "")
    if not to_label:
        return None
    return {
        "from_labels": [
            label for label in getattr(args, "postselect_override_from_label", []) if label
        ],
        "to_label": to_label,
        "from_status_names": [
            status
            for status in getattr(args, "postselect_override_from_status_name", [])
            if status
        ],
        "to_status_names": [],
        "rank_accel_le_selected": bool(
            getattr(args, "postselect_override_rank_accel_le_selected", False)
        ),
        "rank_jump_le_selected": bool(
            getattr(args, "postselect_override_rank_jump_le_selected", False)
        ),
        "min_smooth_outlier_score": getattr(
            args,
            "postselect_override_min_smooth_outlier_score",
            None,
        ),
        "min_cluster_size_50cm": None,
    }


def postselect_override_rules(args: argparse.Namespace) -> list[dict[str, object]]:
    rules: list[dict[str, object]] = []
    legacy = legacy_postselect_rule(args)
    if legacy is not None:
        rules.append(legacy)
    for spec in getattr(args, "postselect_override_rule", []):
        if spec:
            rules.append(parse_postselect_rule_spec(spec))
    return rules


def summarize_postselect_rule(rule: dict[str, object]) -> dict[str, object]:
    return {
        "to_label": rule["to_label"],
        "from_labels": sorted(str(label) for label in rule.get("from_labels", [])),
        "from_status_names": sorted(
            str(status) for status in rule.get("from_status_names", [])
        ),
        "to_status_names": sorted(str(status) for status in rule.get("to_status_names", [])),
        "rank_accel_le_selected": bool(rule.get("rank_accel_le_selected", False)),
        "rank_jump_le_selected": bool(rule.get("rank_jump_le_selected", False)),
        "min_smooth_outlier_score": rounded(rule.get("min_smooth_outlier_score")),
        "min_cluster_size_50cm": rounded(rule.get("min_cluster_size_50cm")),
        "epochs": 0,
        "path_weight_m": 0.0,
    }


def postselect_override_allowed(
    selected: dict[str, object],
    replacement: dict[str, object],
    rule: dict[str, object],
) -> bool:
    from_labels = {str(label) for label in rule.get("from_labels", []) if label}
    if from_labels and str(selected.get("label", "")) not in from_labels:
        return False
    from_status_names = {
        str(status) for status in rule.get("from_status_names", []) if status
    }
    if from_status_names and str(selected.get("status_name", "")) not in from_status_names:
        return False
    to_status_names = {str(status) for status in rule.get("to_status_names", []) if status}
    if to_status_names and str(replacement.get("status_name", "")) not in to_status_names:
        return False
    if bool(rule.get("rank_accel_le_selected", False)):
        if row_score(replacement, "rank_by_delta_pos_accel") > row_score(
            selected,
            "rank_by_delta_pos_accel",
        ):
            return False
    if bool(rule.get("rank_jump_le_selected", False)):
        if row_score(replacement, "rank_by_candidate_jump") > row_score(
            selected,
            "rank_by_candidate_jump",
        ):
            return False
    min_smooth = rule.get("min_smooth_outlier_score")
    if min_smooth is not None:
        smooth = to_float(replacement.get("smooth_outlier_score"))
        if not math.isfinite(smooth) or smooth < float(min_smooth):
            return False
    min_cluster = rule.get("min_cluster_size_50cm")
    if min_cluster is not None:
        cluster_size = to_float(replacement.get("cluster_size_50cm"))
        if not math.isfinite(cluster_size) or cluster_size < float(min_cluster):
            return False
    return True


def apply_postselect_overrides(
    grouped: dict[tuple[str, str, float], list[dict[str, object]]],
    ranker_by_epoch: dict[tuple[str, str, float], dict[str, object]],
    args: argparse.Namespace,
) -> tuple[dict[tuple[str, str, float], dict[str, object]], dict[str, object]]:
    rules = postselect_override_rules(args)
    rule_summaries = [summarize_postselect_rule(rule) for rule in rules]
    summary: dict[str, object] = {
        "enabled": bool(rules),
        "rules": rule_summaries,
        "epochs": 0,
        "path_weight_m": 0.0,
    }
    if len(rule_summaries) == 1:
        summary.update(rule_summaries[0])
    if not rules:
        return ranker_by_epoch, summary

    out = dict(ranker_by_epoch)
    override_epochs = 0
    override_path = 0.0
    for key, selected in ranker_by_epoch.items():
        for rule_index, rule in enumerate(rules):
            to_label = str(rule["to_label"])
            if str(selected.get("label", "")) == to_label:
                continue
            replacement = next(
                (row for row in grouped.get(key, []) if str(row.get("label", "")) == to_label),
                None,
            )
            if replacement is None:
                continue
            if not postselect_override_allowed(selected, replacement, rule):
                continue
            out[key] = replacement
            override_epochs += 1
            override_path += row_score(selected, "path_weight_m", 0.0)
            rule_summaries[rule_index]["epochs"] = int(rule_summaries[rule_index]["epochs"]) + 1
            rule_summaries[rule_index]["path_weight_m"] = (
                float(rule_summaries[rule_index]["path_weight_m"])
                + row_score(selected, "path_weight_m", 0.0)
            )
            break

    summary["epochs"] = override_epochs
    summary["path_weight_m"] = rounded(override_path) or 0.0
    for rule_summary in rule_summaries:
        rule_summary["path_weight_m"] = rounded(float(rule_summary["path_weight_m"])) or 0.0
    if len(rule_summaries) == 1:
        summary.update(rule_summaries[0])
    return out, summary


def fold_for_epoch(
    key: tuple[str, str, float],
    folds: list[tuple[str, set[tuple[str, str, float]]]],
) -> str:
    for fold_name, keys in folds:
        if key in keys:
            return fold_name
    return ""


def build_selection_rows(
    rows: list[dict[str, object]],
    predictions: dict[int, float],
    ranker_by_epoch: dict[tuple[str, str, float], dict[str, object]],
    *,
    fold_mode: str,
    time_blocks: int,
    threshold_m: float,
) -> list[dict[str, object]]:
    grouped = grouped_epoch_rows(rows)
    folds = build_folds(rows, fold_mode=fold_mode, time_blocks=time_blocks)
    out: list[dict[str, object]] = []
    for key in sorted(grouped, key=sort_epoch_key):
        baseline, ranker, oracle = select_epoch(
            grouped[key],
            predictions,
            ranker_by_epoch.get(key),
        )
        baseline_err = row_target(baseline)
        ranker_err = row_target(ranker)
        oracle_err = row_target(oracle)
        weight = to_float(ranker.get("path_weight_m"), 1.0)
        if not math.isfinite(weight) or weight <= 0.0:
            weight = 1.0
        out.append(
            {
                "fold": fold_for_epoch(key, folds),
                "run_id": key[0],
                "week": key[1],
                "tow": key[2],
                "baseline_label": baseline.get("label", ""),
                "ranker_label": ranker.get("label", ""),
                "oracle_label": oracle.get("label", ""),
                "baseline_status_name": baseline.get("status_name", ""),
                "ranker_status_name": ranker.get("status_name", ""),
                "oracle_status_name": oracle.get("status_name", ""),
                "baseline_err_3d_m": baseline_err,
                "ranker_err_3d_m": ranker_err,
                "oracle_err_3d_m": oracle_err,
                "baseline_selector_base_score": row_score(baseline, "selector_base_score", 0.0),
                "ranker_pred_err_m": predictions[int(ranker["_row_index"])],
                "oracle_pred_err_m": predictions[int(oracle["_row_index"])],
                "baseline_risk": row_score(baseline, "libgnss_urban_risk_score", 0.0),
                "ranker_risk": row_score(ranker, "libgnss_urban_risk_score", 0.0),
                "oracle_risk": row_score(oracle, "libgnss_urban_risk_score", 0.0),
                "selection_changed": int(str(baseline.get("label", "")) != str(ranker.get("label", ""))),
                "ranker_minus_baseline_err_m": ranker_err - baseline_err,
                "ranker_regret_m": ranker_err - oracle_err,
                "baseline_pass": int(baseline_err <= threshold_m),
                "ranker_pass": int(ranker_err <= threshold_m),
                "oracle_pass": int(oracle_err <= threshold_m),
                "path_weight_m": weight,
            }
        )
    return out


def selected_source_rows_by_strategy(
    rows: list[dict[str, object]],
    predictions: dict[int, float],
    ranker_by_epoch: dict[tuple[str, str, float], dict[str, object]],
) -> dict[str, list[dict[str, object]]]:
    grouped = grouped_epoch_rows(rows)
    out = {"baseline": [], "ranker": [], "oracle": []}
    for key in sorted(grouped, key=sort_epoch_key):
        baseline, ranker, oracle = select_epoch(
            grouped[key],
            predictions,
            ranker_by_epoch.get(key),
        )
        out["baseline"].append(baseline)
        out["ranker"].append(ranker)
        out["oracle"].append(oracle)
    return out


def finite_required(row: dict[str, object], name: str) -> float:
    value = to_float(row.get(name))
    if math.isfinite(value):
        return value
    label = row.get("label", "")
    tow = row.get("tow", "")
    raise SystemExit(f"selected row missing finite {name}: label={label} tow={tow}")


def optional_float(row: dict[str, object], name: str) -> float | None:
    value = to_float(row.get(name))
    if not math.isfinite(value) or value < 0.0:
        return None
    return float(value)


def optional_int(row: dict[str, object], name: str) -> int | None:
    value = optional_float(row, name)
    if value is None:
        return None
    return int(round(value))


def row_to_solution_epoch(row: dict[str, object]) -> comparison.SolutionEpoch:
    rms = optional_float(row, "rms")
    abs_max = optional_float(row, "abs_max")
    return comparison.SolutionEpoch(
        week=int(round(finite_required(row, "week"))),
        tow=finite_required(row, "tow"),
        lat_deg=finite_required(row, "lat_deg"),
        lon_deg=finite_required(row, "lon_deg"),
        height_m=finite_required(row, "height_m"),
        ecef=np.array(
            [
                finite_required(row, "x_m"),
                finite_required(row, "y_m"),
                finite_required(row, "z_m"),
            ],
            dtype=float,
        ),
        status=int(round(finite_required(row, "status"))),
        num_satellites=int(round(finite_required(row, "sats"))),
        ratio=optional_float(row, "ratio"),
        baseline_m=optional_float(row, "baseline_m"),
        rtk_iterations=None,
        rtk_update_observations=optional_int(row, "update_rows"),
        rtk_update_phase_observations=optional_int(row, "phase_update_rows"),
        rtk_update_code_observations=optional_int(row, "code_update_rows"),
        rtk_update_suppressed_outliers=optional_int(row, "suppressed_outliers"),
        rtk_update_prefit_residual_rms_m=rms,
        rtk_update_prefit_residual_max_m=abs_max,
        rtk_update_post_suppression_residual_rms_m=rms,
        rtk_update_post_suppression_residual_max_m=abs_max,
        rtk_update_normalized_innovation_squared=optional_float(row, "nis"),
        rtk_update_normalized_innovation_squared_per_observation=optional_float(
            row,
            "nis_per_obs",
        ),
        rtk_update_rejected_by_innovation_gate=None,
    )


def rows_to_solution_epochs(rows: list[dict[str, object]]) -> list[comparison.SolutionEpoch]:
    return [row_to_solution_epoch(row) for row in rows]


def format_float(value: float, decimals: int) -> str:
    return f"{value:.{decimals}f}"


def optional_tokens(epoch: comparison.SolutionEpoch) -> list[object | None]:
    return [
        epoch.ratio,
        epoch.baseline_m,
        epoch.rtk_iterations,
        epoch.rtk_update_observations,
        epoch.rtk_update_phase_observations,
        epoch.rtk_update_code_observations,
        epoch.rtk_update_suppressed_outliers,
        epoch.rtk_update_prefit_residual_rms_m,
        epoch.rtk_update_prefit_residual_max_m,
        epoch.rtk_update_post_suppression_residual_rms_m,
        epoch.rtk_update_post_suppression_residual_max_m,
        epoch.rtk_update_normalized_innovation_squared,
        epoch.rtk_update_normalized_innovation_squared_per_observation,
        epoch.rtk_update_rejected_by_innovation_gate,
    ]


def write_pos(path: Path, rows: list[dict[str, object]]) -> None:
    epochs = rows_to_solution_epochs(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as handle:
        handle.write("% PPC candidate feature ranker position solution\n")
        handle.write(
            "% GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) "
            "Status NumSat PDOP Ratio Baseline(m) RtkIter RtkObs RtkPhaseObs "
            "RtkCodeObs RtkOutliers RtkPrefitRms(m) RtkPrefitMax(m) "
            "RtkPostRms(m) RtkPostMax(m) RtkUpdateNIS RtkUpdateNISPerObs "
            "RtkUpdateNISRejected\n"
        )
        for epoch in epochs:
            tokens: list[str] = [
                str(epoch.week),
                format_float(epoch.tow, 3),
                format_float(float(epoch.ecef[0]), 4),
                format_float(float(epoch.ecef[1]), 4),
                format_float(float(epoch.ecef[2]), 4),
                format_float(epoch.lat_deg, 9),
                format_float(epoch.lon_deg, 9),
                format_float(epoch.height_m, 4),
                str(epoch.status),
                str(epoch.num_satellites),
                "2.00",
            ]
            optional = optional_tokens(epoch)
            while optional and optional[-1] is None:
                optional.pop()
            for value in optional:
                if value is None:
                    tokens.append("0")
                elif isinstance(value, int):
                    tokens.append(str(value))
                else:
                    tokens.append(format_float(float(value), 4))
            handle.write(" ".join(tokens) + "\n")


def score_selected_pos(
    reference_csv: Path,
    selected_rows: dict[str, list[dict[str, object]]],
    match_tolerance_s: float,
) -> tuple[dict[str, dict[str, object]], dict[str, object]]:
    reference = comparison.read_reference_csv(reference_csv)
    metrics: dict[str, dict[str, object]] = {}
    for strategy in ("baseline", "ranker", "oracle"):
        metrics[strategy] = ppc_metrics.summarize_solution_epochs(
            reference,
            rows_to_solution_epochs(selected_rows[strategy]),
            fixed_status=4,
            label=f"{strategy} candidate feature selector",
            match_tolerance_s=match_tolerance_s,
            solver_wall_time_s=None,
        )
        metrics[strategy]["reference_epochs"] = len(reference)
    delta = {
        "ranker": ppc_metrics.solution_metric_delta(
            metrics["ranker"],
            metrics["baseline"],
        ),
        "oracle": ppc_metrics.solution_metric_delta(
            metrics["oracle"],
            metrics["baseline"],
        ),
    }
    return metrics, delta


def parse_reference_map(entries: list[str]) -> dict[str, Path]:
    out: dict[str, Path] = {}
    for entry in entries:
        if "=" not in entry:
            raise SystemExit(f"--reference-map must use RUN_ID=CSV, got: {entry!r}")
        run_id, path_text = entry.split("=", 1)
        run_id = run_id.strip()
        if not run_id:
            raise SystemExit(f"--reference-map run_id is empty in: {entry!r}")
        path = Path(path_text.strip())
        if not path.exists():
            raise SystemExit(f"reference CSV not found for {run_id}: {path}")
        out[run_id] = path
    return out


def group_rows_by_run(rows: list[dict[str, object]]) -> dict[str, list[dict[str, object]]]:
    grouped: dict[str, list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        grouped[str(row.get("run_id", ""))].append(row)
    return dict(sorted(grouped.items()))


def aggregate_pos_metrics(
    per_run: dict[str, dict[str, object]],
) -> tuple[dict[str, dict[str, object]], dict[str, object]]:
    metrics: dict[str, dict[str, object]] = {}
    for strategy in ("baseline", "ranker", "oracle"):
        parts = [
            run_payload["metrics"][strategy]
            for run_payload in per_run.values()
            if strategy in run_payload.get("metrics", {})
        ]
        valid_epochs = sum(int(part["valid_epochs"]) for part in parts)
        matched_epochs = sum(int(part["matched_epochs"]) for part in parts)
        fixed_epochs = sum(int(part["fixed_epochs"]) for part in parts)
        total_distance = sum(float(part["ppc_official_total_distance_m"]) for part in parts)
        matched_distance = sum(float(part["ppc_official_matched_distance_m"]) for part in parts)
        score_distance = sum(float(part["ppc_official_score_distance_m"]) for part in parts)
        score_epochs = sum(int(part["ppc_score_3d_50cm_epochs"]) for part in parts)
        reference_epochs = sum(int(part.get("reference_epochs", 0)) for part in parts)
        mean_h_numerator = sum(
            float(part["mean_h_m"]) * int(part["matched_epochs"])
            for part in parts
            if part.get("mean_h_m") is not None
        )
        metrics[strategy] = {
            "runs": len(parts),
            "reference_epochs": reference_epochs,
            "valid_epochs": valid_epochs,
            "matched_epochs": matched_epochs,
            "fixed_epochs": fixed_epochs,
            "positioning_rate_pct": rounded(100.0 * matched_epochs / reference_epochs)
            if reference_epochs > 0
            else 0.0,
            "fix_rate_pct": rounded(100.0 * fixed_epochs / matched_epochs)
            if matched_epochs > 0
            else 0.0,
            "mean_h_m": rounded(mean_h_numerator / matched_epochs)
            if matched_epochs > 0 and mean_h_numerator > 0.0
            else 0.0,
            "ppc_score_3d_50cm_epochs": score_epochs,
            "ppc_score_3d_50cm_matched_pct": rounded(100.0 * score_epochs / matched_epochs)
            if matched_epochs > 0
            else 0.0,
            "ppc_score_3d_50cm_ref_pct": rounded(100.0 * score_epochs / reference_epochs)
            if reference_epochs > 0
            else 0.0,
            "ppc_official_score_threshold_m": rounded(
                float(parts[0]["ppc_official_score_threshold_m"])
            )
            if parts
            else None,
            "ppc_official_total_distance_m": rounded(total_distance),
            "ppc_official_matched_distance_m": rounded(matched_distance),
            "ppc_official_score_distance_m": rounded(score_distance),
            "ppc_official_score_pct": rounded(100.0 * score_distance / total_distance)
            if total_distance > 0.0
            else 0.0,
        }
    delta = {
        "ranker": ppc_metrics.solution_metric_delta(metrics["ranker"], metrics["baseline"]),
        "oracle": ppc_metrics.solution_metric_delta(metrics["oracle"], metrics["baseline"]),
    }
    return metrics, delta


def score_selected_pos_by_run(
    reference_map: dict[str, Path],
    selected_rows: dict[str, list[dict[str, object]]],
    match_tolerance_s: float,
) -> tuple[dict[str, dict[str, object]], dict[str, dict[str, object]], dict[str, object]]:
    grouped = {
        strategy: group_rows_by_run(rows)
        for strategy, rows in selected_rows.items()
    }
    run_ids = sorted({run_id for rows in grouped.values() for run_id in rows})
    missing = [run_id for run_id in run_ids if run_id not in reference_map]
    if missing:
        raise SystemExit(
            "missing --reference-map entries for run_id(s): " + ", ".join(missing)
        )

    per_run: dict[str, dict[str, object]] = {}
    for run_id in run_ids:
        run_selected = {
            strategy: grouped[strategy].get(run_id, [])
            for strategy in ("baseline", "ranker", "oracle")
        }
        metrics, delta = score_selected_pos(
            reference_map[run_id],
            run_selected,
            match_tolerance_s,
        )
        per_run[run_id] = {
            "reference_csv": str(reference_map[run_id]),
            "metrics": metrics,
            "delta_vs_baseline": delta,
        }
    aggregate, aggregate_delta = aggregate_pos_metrics(per_run)
    return aggregate, per_run, aggregate_delta


def write_ranker_pos_by_run(
    out_dir: Path,
    rows: list[dict[str, object]],
) -> dict[str, str]:
    out: dict[str, str] = {}
    out_dir.mkdir(parents=True, exist_ok=True)
    for run_id, run_rows in group_rows_by_run(rows).items():
        safe_run_id = "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in run_id)
        path = out_dir / f"{safe_run_id}_ranker.pos"
        write_pos(path, run_rows)
        out[run_id] = str(path)
    return out


def percentile(values: list[float], q: float) -> float:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return 0.0
    if len(finite) == 1:
        return finite[0]
    pos = (len(finite) - 1) * q / 100.0
    lo = int(pos)
    hi = min(lo + 1, len(finite) - 1)
    frac = pos - lo
    return finite[lo] * (1.0 - frac) + finite[hi] * frac


def strategy_metrics(
    selection_rows: list[dict[str, object]],
    prefix: str,
    threshold_m: float,
) -> dict[str, object]:
    errors = [float(row[f"{prefix}_err_3d_m"]) for row in selection_rows]
    pass_values = [1 if err <= threshold_m else 0 for err in errors]
    weights = [float(row["path_weight_m"]) for row in selection_rows]
    pass_weight = sum(weight for weight, passed in zip(weights, pass_values) if passed)
    total_weight = sum(weights)
    labels = Counter(str(row[f"{prefix}_label"]) for row in selection_rows)
    return {
        "epochs": len(selection_rows),
        "mean_err_3d_m": rounded(sum(errors) / len(errors) if errors else 0.0),
        "p50_err_3d_m": rounded(percentile(errors, 50.0)),
        "p95_err_3d_m": rounded(percentile(errors, 95.0)),
        "pass_rate": rounded(sum(pass_values) / len(pass_values) if pass_values else 0.0),
        "weighted_pass_rate": rounded(pass_weight / total_weight if total_weight > 0.0 else None),
        "selected_label_counts": dict(sorted(labels.items())),
    }


def build_summary(
    rows: list[dict[str, object]],
    selection_rows: list[dict[str, object]],
    fold_summaries: list[dict[str, object]],
    numeric_features: list[str],
    label_features: list[str],
    args: argparse.Namespace,
    postselect_override: dict[str, object],
) -> dict[str, object]:
    deltas = [float(row["ranker_minus_baseline_err_m"]) for row in selection_rows]
    regrets = [float(row["ranker_regret_m"]) for row in selection_rows]
    changed = [row for row in selection_rows if int(row["selection_changed"]) == 1]
    return {
        "input_csvs": [str(path) for path in args.input_csv],
        "row_count": len(rows),
        "epochs": len(selection_rows),
        "run_ids": run_ids(rows),
        "candidate_labels": sorted({str(row.get("label", "")) for row in rows}),
        "excluded_labels": sorted({label for label in args.exclude_label if label}),
        "fold_mode": args.fold_mode,
        "time_blocks": args.time_blocks,
        "model": args.model,
        "ridge_alpha": args.ridge_alpha,
        "hgb_max_iter": args.hgb_max_iter,
        "hgb_learning_rate": args.hgb_learning_rate,
        "hgb_max_leaf_nodes": args.hgb_max_leaf_nodes,
        "hgb_l2_regularization": args.hgb_l2_regularization,
        "hgb_random_state": args.hgb_random_state,
        "target_weight_mode": args.target_weight_mode,
        "selection_mode": args.selection_mode,
        "viterbi_switch_penalty_m": args.viterbi_switch_penalty_m,
        "viterbi_switch_margin_m": args.viterbi_switch_margin_m,
        "viterbi_jump_excess_penalty_per_m": args.viterbi_jump_excess_penalty_per_m,
        "viterbi_jump_excess_margin_m": args.viterbi_jump_excess_margin_m,
        "viterbi_max_gap_s": args.viterbi_max_gap_s,
        "viterbi_max_segment_epochs": args.viterbi_max_segment_epochs,
        "postselect_override": postselect_override,
        "threshold_m": args.threshold_m,
        "include_label_features": not args.no_label_features,
        "numeric_features": numeric_features,
        "label_features": label_features,
        "folds": fold_summaries,
        "baseline": strategy_metrics(selection_rows, "baseline", args.threshold_m),
        "ranker": strategy_metrics(selection_rows, "ranker", args.threshold_m),
        "oracle": strategy_metrics(selection_rows, "oracle", args.threshold_m),
        "selection_changed_epochs": len(changed),
        "selection_change_rate": rounded(len(changed) / len(selection_rows) if selection_rows else 0.0),
        "mean_ranker_minus_baseline_err_m": rounded(sum(deltas) / len(deltas) if deltas else 0.0),
        "p50_ranker_minus_baseline_err_m": rounded(percentile(deltas, 50.0)),
        "mean_ranker_regret_m": rounded(sum(regrets) / len(regrets) if regrets else 0.0),
        "improved_epochs": sum(1 for value in deltas if value < -1e-9),
        "worse_epochs": sum(1 for value in deltas if value > 1e-9),
        "equal_epochs": sum(1 for value in deltas if abs(value) <= 1e-9),
    }


def csv_value(value: object) -> object:
    if isinstance(value, float):
        return f"{value:.9f}" if math.isfinite(value) else ""
    return value


def write_selection_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=SELECTION_FIELDNAMES, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({name: csv_value(row.get(name, "")) for name in SELECTION_FIELDNAMES})


def write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def fmt(value: object) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)


def render_markdown(summary: dict[str, object]) -> str:
    baseline = dict(summary["baseline"])
    ranker = dict(summary["ranker"])
    oracle = dict(summary["oracle"])
    lines = [
        "# PPC Candidate Feature Ranker",
        "",
        f"Rows: **{summary['row_count']}**",
        f"Epochs: **{summary['epochs']}**",
        f"Candidates: **{', '.join(summary['candidate_labels'])}**",
        f"Fold mode: **{summary['fold_mode']}**",
        f"Model: **{summary.get('model', 'ridge')}**",
        f"Selection mode: **{summary.get('selection_mode', 'point')}**",
        f"Ridge alpha: **{summary['ridge_alpha']}**",
        f"Target weight mode: **{summary.get('target_weight_mode', 'uniform')}**",
        "",
        "| strategy | mean err m | p50 err m | p95 err m | pass rate | weighted pass |",
        "|---|---:|---:|---:|---:|---:|",
        (
            f"| baseline | {fmt(baseline['mean_err_3d_m'])} | {fmt(baseline['p50_err_3d_m'])} | "
            f"{fmt(baseline['p95_err_3d_m'])} | {fmt(baseline['pass_rate'])} | "
            f"{fmt(baseline['weighted_pass_rate'])} |"
        ),
        (
            f"| ranker | {fmt(ranker['mean_err_3d_m'])} | {fmt(ranker['p50_err_3d_m'])} | "
            f"{fmt(ranker['p95_err_3d_m'])} | {fmt(ranker['pass_rate'])} | "
            f"{fmt(ranker['weighted_pass_rate'])} |"
        ),
        (
            f"| oracle | {fmt(oracle['mean_err_3d_m'])} | {fmt(oracle['p50_err_3d_m'])} | "
            f"{fmt(oracle['p95_err_3d_m'])} | {fmt(oracle['pass_rate'])} | "
            f"{fmt(oracle['weighted_pass_rate'])} |"
        ),
        "",
        (
            "Ranker minus baseline mean error: "
            f"**{fmt(summary['mean_ranker_minus_baseline_err_m'])} m**"
        ),
        (
            "Changed epochs: "
            f"**{summary['selection_changed_epochs']} / {summary['epochs']}**"
        ),
        (
            "Improved / worse / equal: "
            f"**{summary['improved_epochs']} / {summary['worse_epochs']} / {summary['equal_epochs']}**"
        ),
        "",
    ]
    pos_metrics = summary.get("pos_metrics")
    if isinstance(pos_metrics, dict):
        lines.extend(
            [
                "## PPC POS Metrics",
                "",
                "| strategy | valid epochs | matched epochs | official score pct | 3D 50cm ref pct | mean h m | p95 h m |",
                "|---|---:|---:|---:|---:|---:|---:|",
            ]
        )
        for strategy in ("baseline", "ranker", "oracle"):
            metrics = pos_metrics.get(strategy)
            if not isinstance(metrics, dict):
                continue
            lines.append(
                f"| {strategy} | {fmt(metrics.get('valid_epochs'))} | "
                f"{fmt(metrics.get('matched_epochs'))} | "
                f"{fmt(metrics.get('ppc_official_score_pct'))} | "
                f"{fmt(metrics.get('ppc_score_3d_50cm_ref_pct'))} | "
                f"{fmt(metrics.get('mean_h_m'))} | "
                f"{fmt(metrics.get('p95_h_m'))} |"
            )
        lines.append("")
    pos_metrics_by_run = summary.get("pos_metrics_by_run")
    if isinstance(pos_metrics_by_run, dict):
        lines.extend(
            [
                "## PPC POS Metrics By Run",
                "",
                "| run | baseline official pct | ranker official pct | oracle official pct | ranker delta pp |",
                "|---|---:|---:|---:|---:|",
            ]
        )
        for run_id, payload in sorted(pos_metrics_by_run.items()):
            if not isinstance(payload, dict):
                continue
            metrics = payload.get("metrics", {})
            delta = payload.get("delta_vs_baseline", {})
            if not isinstance(metrics, dict) or not isinstance(delta, dict):
                continue
            baseline = metrics.get("baseline", {})
            ranker = metrics.get("ranker", {})
            oracle = metrics.get("oracle", {})
            ranker_delta = delta.get("ranker", {})
            lines.append(
                f"| {run_id} | "
                f"{fmt(baseline.get('ppc_official_score_pct') if isinstance(baseline, dict) else None)} | "
                f"{fmt(ranker.get('ppc_official_score_pct') if isinstance(ranker, dict) else None)} | "
                f"{fmt(oracle.get('ppc_official_score_pct') if isinstance(oracle, dict) else None)} | "
                f"{fmt(ranker_delta.get('ppc_official_score_pct') if isinstance(ranker_delta, dict) else None)} |"
            )
        lines.append("")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    args.input_csv = [Path(path) for path in args.input_csv]
    rows = read_rows(args.input_csv)
    rows = exclude_label_rows(rows, args.exclude_label)
    predictions, fold_summaries, numeric_features, label_features = train_predict_cv(
        rows,
        fold_mode=args.fold_mode,
        time_blocks=args.time_blocks,
        alpha=args.ridge_alpha,
        include_label_features=not args.no_label_features,
        target_weight_mode=args.target_weight_mode,
        args=args,
    )
    grouped = grouped_epoch_rows(rows)
    ranker_by_epoch = ranker_rows_by_epoch(rows, predictions, args)
    ranker_by_epoch, postselect_override = apply_postselect_overrides(
        grouped,
        ranker_by_epoch,
        args,
    )
    selection_rows = build_selection_rows(
        rows,
        predictions,
        ranker_by_epoch,
        fold_mode=args.fold_mode,
        time_blocks=args.time_blocks,
        threshold_m=args.threshold_m,
    )
    summary = build_summary(
        rows,
        selection_rows,
        fold_summaries,
        numeric_features,
        label_features,
        args,
        postselect_override,
    )
    rows_csv = args.rows_csv or args.out_dir / "ppc_candidate_ranker_rows.csv"
    summary_json = args.summary_json or args.out_dir / "ppc_candidate_ranker_summary.json"
    markdown_output = args.markdown_output or args.out_dir / "ppc_candidate_ranker.md"
    selected_rows = selected_source_rows_by_strategy(rows, predictions, ranker_by_epoch)
    if args.out_pos is not None:
        write_pos(args.out_pos, selected_rows["ranker"])
        summary["out_pos"] = str(args.out_pos)
    if args.out_pos_dir is not None:
        summary["out_pos_by_run"] = write_ranker_pos_by_run(
            args.out_pos_dir,
            selected_rows["ranker"],
        )
    if args.reference_csv is not None:
        pos_metrics, pos_delta = score_selected_pos(
            args.reference_csv,
            selected_rows,
            args.match_tolerance_s,
        )
        summary["reference_csv"] = str(args.reference_csv)
        summary["match_tolerance_s"] = args.match_tolerance_s
        summary["pos_metrics"] = pos_metrics
        summary["pos_metric_delta_vs_baseline"] = pos_delta
    reference_map = parse_reference_map(args.reference_map)
    if reference_map:
        pos_metrics, pos_metrics_by_run, pos_delta = score_selected_pos_by_run(
            reference_map,
            selected_rows,
            args.match_tolerance_s,
        )
        summary["reference_map"] = {
            run_id: str(path)
            for run_id, path in sorted(reference_map.items())
        }
        summary["match_tolerance_s"] = args.match_tolerance_s
        summary["pos_metrics"] = pos_metrics
        summary["pos_metrics_by_run"] = pos_metrics_by_run
        summary["pos_metric_delta_vs_baseline"] = pos_delta
    write_selection_csv(rows_csv, selection_rows)
    write_json(summary_json, summary)
    markdown_output.parent.mkdir(parents=True, exist_ok=True)
    markdown_output.write_text(render_markdown(summary) + "\n", encoding="utf-8")
    print(
        "PPC candidate ranker: "
        f"epochs={summary['epochs']} changed={summary['selection_changed_epochs']} "
        f"mean_delta={summary['mean_ranker_minus_baseline_err_m']} "
        f"out={summary_json}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
