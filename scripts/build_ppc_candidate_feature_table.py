#!/usr/bin/env python3
"""Build a PPC candidate feature table from libgnss++ POS files.

The table is intentionally solver-native: it uses only libgnss++ solution
metadata, cross-candidate consensus, and candidate motion history.  Reference
truth is optional and is used only for offline labels/metrics when provided.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
import sys

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = ROOT_DIR / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import generate_driving_comparison as comparison  # noqa: E402


FIELDNAMES = [
    "run_id",
    "week",
    "tow",
    "label",
    "status",
    "status_name",
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
    "x_m",
    "y_m",
    "z_m",
    "lat_deg",
    "lon_deg",
    "height_m",
    "matched_reference_tow_s",
    "err_3d_m",
    "horiz_error_m",
    "up_error_m",
    "is_pass_50cm",
    "path_weight_m",
]


@dataclass(frozen=True)
class CandidateInput:
    label: str
    pos_path: Path


@dataclass(frozen=True)
class ReferenceIndex:
    rows_by_week: dict[int, list[comparison.ReferenceEpoch]]
    tows_by_week: dict[int, list[float]]
    path_weight_by_key: dict[tuple[int, float], float]


def parse_candidate_arg(text: str) -> CandidateInput:
    if "=" not in text:
        raise SystemExit(f"--candidate must use LABEL=PATH, got: {text!r}")
    label, path_text = text.split("=", 1)
    label = label.strip()
    if not label:
        raise SystemExit(f"--candidate label is empty in: {text!r}")
    path = Path(path_text.strip())
    if not path.exists():
        raise SystemExit(f"candidate POS file not found for {label}: {path}")
    return CandidateInput(label, path)


def epoch_key(epoch: comparison.SolutionEpoch) -> tuple[int, float]:
    return epoch.week, round(float(epoch.tow), 6)


def status_name(status: int) -> str:
    extra = {5: "SPP"}
    return comparison.STATUS_MAPS["libgnss++"].get(
        int(status),
        extra.get(int(status), f"status {status}"),
    )


def first_finite(*values: float | int | None, default: float = -1.0) -> float:
    for value in values:
        if value is None:
            continue
        out = float(value)
        if math.isfinite(out):
            return out
    return float(default)


def clip01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def nonnegative_or_inf(value: float) -> float:
    return value if math.isfinite(value) and value >= 0.0 else math.inf


def nonnegative_min(values: list[float]) -> float:
    finite = [value for value in values if math.isfinite(value) and value >= 0.0]
    return min(finite) if finite else -1.0


def positive_ratio(numerator: float, denominator: float, floor: float = 0.20) -> float:
    if not math.isfinite(numerator) or not math.isfinite(denominator) or denominator < 0.0:
        return -1.0
    return numerator / max(denominator, floor)


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


def path_weights(reference: list[comparison.ReferenceEpoch]) -> dict[tuple[int, float], float]:
    out: dict[tuple[int, float], float] = {}
    rows = sorted(reference, key=lambda row: (row.week, row.tow))
    for index, ref in enumerate(rows):
        weight = 0.0
        if index > 0 and rows[index - 1].week == ref.week:
            weight += 0.5 * float(np.linalg.norm(ref.ecef - rows[index - 1].ecef))
        if index + 1 < len(rows) and rows[index + 1].week == ref.week:
            weight += 0.5 * float(np.linalg.norm(rows[index + 1].ecef - ref.ecef))
        out[(ref.week, round(float(ref.tow), 6))] = weight
    return out


def build_reference_index(reference: list[comparison.ReferenceEpoch]) -> ReferenceIndex:
    rows_by_week: dict[int, list[comparison.ReferenceEpoch]] = {}
    for row in reference:
        rows_by_week.setdefault(row.week, []).append(row)
    for rows in rows_by_week.values():
        rows.sort(key=lambda row: row.tow)
    return ReferenceIndex(
        rows_by_week=rows_by_week,
        tows_by_week={week: [row.tow for row in rows] for week, rows in rows_by_week.items()},
        path_weight_by_key=path_weights(reference),
    )


def match_reference(
    epoch: comparison.SolutionEpoch,
    ref_index: ReferenceIndex | None,
    tolerance_s: float,
) -> comparison.ReferenceEpoch | None:
    if ref_index is None:
        return None
    rows = ref_index.rows_by_week.get(epoch.week)
    tows = ref_index.tows_by_week.get(epoch.week)
    if not rows or not tows:
        return None
    index = bisect.bisect_left(tows, epoch.tow)
    candidates = [idx for idx in (index - 1, index, index + 1) if 0 <= idx < len(rows)]
    if not candidates:
        return None
    best = min(candidates, key=lambda idx: abs(rows[idx].tow - epoch.tow))
    ref = rows[best]
    if abs(ref.tow - epoch.tow) > tolerance_s:
        return None
    return ref


def cluster_ids(points: np.ndarray, radius_m: float) -> np.ndarray:
    n = int(points.shape[0])
    parent = np.arange(n)

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = int(parent[index])
        return index

    def union(left: int, right: int) -> None:
        left_root = find(left)
        right_root = find(right)
        if left_root != right_root:
            parent[left_root] = right_root

    for left in range(n):
        for right in range(left + 1, n):
            if float(np.linalg.norm(points[left] - points[right])) < radius_m:
                union(left, right)

    roots = np.array([find(index) for index in range(n)])
    mapping = {root: idx for idx, root in enumerate(sorted(set(roots.tolist())))}
    return np.array([mapping[int(root)] for root in roots], dtype=int)


def rank_low_to_high(values: list[float]) -> list[int]:
    def key(index: int) -> tuple[float, int]:
        value = values[index]
        return (value if math.isfinite(value) else math.inf, index)

    ranks = [0] * len(values)
    for rank, index in enumerate(sorted(range(len(values)), key=key), start=1):
        ranks[index] = rank
    return ranks


def residual_rms(epoch: comparison.SolutionEpoch) -> float:
    return first_finite(
        epoch.rtk_update_post_suppression_residual_rms_m,
        epoch.rtk_update_prefit_residual_rms_m,
        default=-1.0,
    )


def residual_abs_max(epoch: comparison.SolutionEpoch) -> float:
    return first_finite(
        epoch.rtk_update_post_suppression_residual_max_m,
        epoch.rtk_update_prefit_residual_max_m,
        default=-1.0,
    )


def selector_base_score(epoch: comparison.SolutionEpoch) -> float:
    status_penalty = {4: 0.0, 3: 0.35, 2: 0.70, 1: 1.10, 5: 1.10}.get(
        int(epoch.status),
        1.30,
    )
    rms = residual_rms(epoch)
    rms_term = clip01(rms / 8.0) if rms >= 0.0 else 0.65
    ratio = first_finite(epoch.ratio, default=0.0)
    ratio_credit = min(max(ratio, 0.0), 20.0) * 0.005
    return status_penalty + rms_term - ratio_credit


def nonfix_risk(status: int) -> float:
    return {4: 0.0, 3: 0.35, 2: 0.65, 1: 0.85, 5: 0.85}.get(int(status), 0.75)


def urban_risk_score(
    *,
    sats: int,
    rms: float,
    dist_to_median_m: float,
    jump_m: float,
    status: int,
    cluster_size: int,
    n_candidates: int,
) -> tuple[float, dict[str, float]]:
    low_sat = clip01((10.0 - float(sats)) / 8.0)
    residual = clip01(rms / 5.0) if rms >= 0.0 else 0.50
    consensus = clip01(dist_to_median_m / 5.0)
    jump = clip01(jump_m / 10.0) if jump_m >= 0.0 else 0.0
    status_risk = nonfix_risk(status)
    cluster_support = clip01(float(cluster_size) / max(float(n_candidates), 1.0))
    risk = clip01(
        0.28 * (1.0 - cluster_support)
        + 0.22 * consensus
        + 0.18 * residual
        + 0.14 * low_sat
        + 0.10 * status_risk
        + 0.08 * jump
    )
    return risk, {
        "libgnss_low_sat_risk": low_sat,
        "libgnss_residual_risk": residual,
        "libgnss_consensus_risk": consensus,
        "libgnss_jump_risk": jump,
        "libgnss_nonfix_risk": status_risk,
    }


def load_candidate_maps(
    candidates: list[CandidateInput],
) -> dict[str, dict[tuple[int, float], comparison.SolutionEpoch]]:
    out: dict[str, dict[tuple[int, float], comparison.SolutionEpoch]] = {}
    seen_labels: set[str] = set()
    for candidate in candidates:
        if candidate.label in seen_labels:
            raise SystemExit(f"duplicate candidate label: {candidate.label}")
        seen_labels.add(candidate.label)
        epochs = comparison.read_libgnss_pos(candidate.pos_path)
        out[candidate.label] = {epoch_key(epoch): epoch for epoch in epochs}
    return out


def reference_error_fields(
    epoch: comparison.SolutionEpoch,
    ref: comparison.ReferenceEpoch | None,
    ref_index: ReferenceIndex | None,
    threshold_m: float,
) -> dict[str, object]:
    if ref is None:
        return {
            "matched_reference_tow_s": "",
            "err_3d_m": "",
            "horiz_error_m": "",
            "up_error_m": "",
            "is_pass_50cm": "",
            "path_weight_m": "",
        }
    diff = epoch.ecef - ref.ecef
    enu = comparison.ecef_to_enu(diff, ref.lat_deg, ref.lon_deg)
    err_3d = float(np.linalg.norm(diff))
    horiz = float(math.hypot(float(enu[0]), float(enu[1])))
    up = float(enu[2])
    weight = ""
    if ref_index is not None:
        weight = ref_index.path_weight_by_key.get((ref.week, round(float(ref.tow), 6)), "")
    return {
        "matched_reference_tow_s": ref.tow,
        "err_3d_m": err_3d,
        "horiz_error_m": horiz,
        "up_error_m": up,
        "is_pass_50cm": int(err_3d <= threshold_m),
        "path_weight_m": weight,
    }


def build_feature_rows(
    candidates: list[CandidateInput],
    *,
    run_id: str,
    reference: list[comparison.ReferenceEpoch] | None = None,
    match_tolerance_s: float = 0.25,
    threshold_m: float = 0.50,
) -> list[dict[str, object]]:
    candidate_maps = load_candidate_maps(candidates)
    ref_index = build_reference_index(reference) if reference else None
    labels = [candidate.label for candidate in candidates]
    all_keys = sorted({key for mapping in candidate_maps.values() for key in mapping})
    history_by_label: dict[str, list[comparison.SolutionEpoch]] = {}
    rows: list[dict[str, object]] = []

    for key in all_keys:
        epoch_items = [
            (label, candidate_maps[label][key])
            for label in labels
            if key in candidate_maps[label]
        ]
        if not epoch_items:
            continue

        points = np.array([epoch.ecef for _label, epoch in epoch_items], dtype=float)
        n_candidates = len(epoch_items)
        distances = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=-1)
        size_50 = (distances < 0.50).sum(axis=1)
        size_25 = (distances < 0.25).sum(axis=1)
        size_10 = (distances < 0.10).sum(axis=1)
        ids_50 = cluster_ids(points, 0.50)
        cluster_counts = {int(cid): int((ids_50 == cid).sum()) for cid in set(ids_50.tolist())}
        largest_cluster_id = max(cluster_counts, key=cluster_counts.get)
        largest_cluster_size = int(cluster_counts[largest_cluster_id])
        largest_centroid = points[ids_50 == largest_cluster_id].mean(axis=0)
        median_xyz = np.median(points, axis=0)

        rms_values = [residual_rms(epoch) for _label, epoch in epoch_items]
        score_values = [selector_base_score(epoch) for _label, epoch in epoch_items]
        rms_ranks = rank_low_to_high(rms_values)
        score_ranks = rank_low_to_high(score_values)

        deltas: list[dict[str, float]] = []
        for label, epoch in epoch_items:
            history = history_by_label.get(label, [])
            d1 = float(np.linalg.norm(epoch.ecef - history[-1].ecef)) if history else -1.0
            d2 = float(np.linalg.norm(epoch.ecef - history[-2].ecef)) if len(history) >= 2 else -1.0
            d3 = float(np.linalg.norm(epoch.ecef - history[-3].ecef)) if len(history) >= 3 else -1.0
            dh = float(np.linalg.norm(epoch.ecef[:2] - history[-1].ecef[:2])) if history else -1.0
            dv = float(abs(epoch.ecef[2] - history[-1].ecef[2])) if history else -1.0
            if len(history) >= 2:
                velocity_now = epoch.ecef - history[-1].ecef
                velocity_prev = history[-1].ecef - history[-2].ecef
                accel = float(np.linalg.norm(velocity_now - velocity_prev))
            else:
                accel = -1.0
            deltas.append(
                {
                    "delta_pos_norm_m": d1,
                    "delta_pos_2step_m": d2,
                    "delta_pos_3step_m": d3,
                    "delta_pos_horizontal_m": dh,
                    "delta_pos_vertical_m": dv,
                    "delta_pos_accel_m": accel,
                }
            )
        jump_values = [delta["delta_pos_norm_m"] for delta in deltas]
        accel_values = [delta["delta_pos_accel_m"] for delta in deltas]
        jump_ranks = rank_low_to_high([nonnegative_or_inf(value) for value in jump_values])
        accel_ranks = rank_low_to_high([nonnegative_or_inf(value) for value in accel_values])
        min_jump = nonnegative_min(jump_values)
        min_accel = nonnegative_min(accel_values)

        for index, (label, epoch) in enumerate(epoch_items):
            dist_to_median = float(np.linalg.norm(epoch.ecef - median_xyz))
            dist_to_largest = float(np.linalg.norm(epoch.ecef - largest_centroid))
            cluster_size = int(size_50[index])
            jump = deltas[index]["delta_pos_norm_m"]
            accel = deltas[index]["delta_pos_accel_m"]
            dist_jump_ratio = positive_ratio(dist_to_median, jump)
            largest_jump_ratio = positive_ratio(dist_to_largest, jump)
            smooth_outlier_score = 0.0
            if math.isfinite(jump) and jump >= 0.0:
                smooth_outlier_score = clip01(dist_to_median / 50.0) * clip01(
                    (5.0 - min(jump, 5.0)) / 5.0
                )
            rms = rms_values[index]
            risk, risk_parts = urban_risk_score(
                sats=epoch.num_satellites,
                rms=rms,
                dist_to_median_m=dist_to_median,
                jump_m=jump,
                status=epoch.status,
                cluster_size=cluster_size,
                n_candidates=n_candidates,
            )
            ref = match_reference(epoch, ref_index, match_tolerance_s)
            row = {
                "run_id": run_id,
                "week": epoch.week,
                "tow": round(float(epoch.tow), 6),
                "label": label,
                "status": epoch.status,
                "status_name": status_name(epoch.status),
                "sats": epoch.num_satellites,
                "ratio": first_finite(epoch.ratio, default=0.0),
                "baseline_m": first_finite(epoch.baseline_m, default=-1.0),
                "rms": rms,
                "abs_max": residual_abs_max(epoch),
                "update_rows": first_finite(epoch.rtk_update_observations, default=-1.0),
                "phase_update_rows": first_finite(
                    epoch.rtk_update_phase_observations, default=-1.0
                ),
                "code_update_rows": first_finite(epoch.rtk_update_code_observations, default=-1.0),
                "suppressed_outliers": first_finite(
                    epoch.rtk_update_suppressed_outliers, default=-1.0
                ),
                "nis": first_finite(epoch.rtk_update_normalized_innovation_squared, default=-1.0),
                "nis_per_obs": first_finite(
                    epoch.rtk_update_normalized_innovation_squared_per_observation,
                    default=-1.0,
                ),
                "selector_base_score": score_values[index],
                "rank_by_selector_score": score_ranks[index],
                "rank_by_rms": rms_ranks[index],
                "n_candidates_in_epoch": n_candidates,
                "cluster_size_50cm": cluster_size,
                "cluster_size_25cm": int(size_25[index]),
                "cluster_size_10cm": int(size_10[index]),
                "max_cluster_size_50cm": largest_cluster_size,
                "is_in_max_cluster_50cm": int(cluster_counts[int(ids_50[index])] == largest_cluster_size),
                "n_clusters_50cm": len(cluster_counts),
                "dist_to_median_m": dist_to_median,
                "dist_to_max_cluster_centroid_m": dist_to_largest,
                "candidate_jump_m": jump,
                "rank_by_candidate_jump": jump_ranks[index],
                "rank_by_delta_pos_accel": accel_ranks[index],
                "candidate_jump_minus_min_m": (
                    jump - min_jump if min_jump >= 0.0 and jump >= 0.0 else -1.0
                ),
                "delta_pos_accel_minus_min_m": (
                    accel - min_accel if min_accel >= 0.0 and accel >= 0.0 else -1.0
                ),
                "dist_to_median_over_jump": dist_jump_ratio,
                "dist_to_max_cluster_over_jump": largest_jump_ratio,
                "smooth_outlier_score": smooth_outlier_score,
                **deltas[index],
                **risk_parts,
                "libgnss_urban_risk_score": risk,
                "x_m": float(epoch.ecef[0]),
                "y_m": float(epoch.ecef[1]),
                "z_m": float(epoch.ecef[2]),
                "lat_deg": epoch.lat_deg,
                "lon_deg": epoch.lon_deg,
                "height_m": epoch.height_m,
                **reference_error_fields(epoch, ref, ref_index, threshold_m),
            }
            rows.append(row)

        for label, epoch in epoch_items:
            history = history_by_label.setdefault(label, [])
            history.append(epoch)
            if len(history) > 3:
                history.pop(0)

    return rows


def csv_value(value: object) -> object:
    if value is None:
        return ""
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return f"{value:.9f}"
    return value


def write_rows(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({name: csv_value(row.get(name, "")) for name in FIELDNAMES})


def build_summary(rows: list[dict[str, object]], candidates: list[CandidateInput]) -> dict[str, object]:
    risks = [float(row["libgnss_urban_risk_score"]) for row in rows]
    matched = [row for row in rows if row.get("err_3d_m") not in ("", None)]
    pass_rows = [row for row in matched if int(row.get("is_pass_50cm", 0)) == 1]
    per_label: dict[str, dict[str, object]] = {}
    for candidate in candidates:
        label_rows = [row for row in rows if row["label"] == candidate.label]
        label_risks = [float(row["libgnss_urban_risk_score"]) for row in label_rows]
        label_err = [
            float(row["err_3d_m"])
            for row in label_rows
            if row.get("err_3d_m") not in ("", None)
        ]
        per_label[candidate.label] = {
            "rows": len(label_rows),
            "mean_libgnss_urban_risk_score": sum(label_risks) / len(label_risks)
            if label_risks
            else 0.0,
            "p95_libgnss_urban_risk_score": percentile(label_risks, 95.0),
            "mean_err_3d_m": sum(label_err) / len(label_err) if label_err else None,
        }
    return {
        "rows": len(rows),
        "epochs": len({(row["week"], row["tow"]) for row in rows}),
        "candidate_labels": [candidate.label for candidate in candidates],
        "matched_reference_rows": len(matched),
        "pass_50cm_rows": len(pass_rows),
        "pass_50cm_rate": len(pass_rows) / len(matched) if matched else None,
        "mean_libgnss_urban_risk_score": sum(risks) / len(risks) if risks else 0.0,
        "p95_libgnss_urban_risk_score": percentile(risks, 95.0),
        "high_risk_rows": sum(1 for value in risks if value >= 0.50),
        "per_label": per_label,
    }


def write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description="Build candidate-level PPC selector features from libgnss++ POS files.",
    )
    parser.add_argument(
        "--candidate",
        dest="candidates",
        action="append",
        required=True,
        metavar="LABEL=PATH",
        help="Candidate POS file. Repeat for each candidate profile.",
    )
    parser.add_argument("--run-id", default="ppc_run")
    parser.add_argument("--reference-csv", type=Path, default=None)
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--threshold-m", type=float, default=0.50)
    parser.add_argument("--out-csv", type=Path, required=True)
    parser.add_argument("--summary-json", type=Path, default=None)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    candidates = [parse_candidate_arg(value) for value in args.candidates]
    reference = None
    if args.reference_csv is not None:
        if not args.reference_csv.exists():
            raise SystemExit(f"reference CSV not found: {args.reference_csv}")
        reference = comparison.read_reference_csv(args.reference_csv)
    rows = build_feature_rows(
        candidates,
        run_id=args.run_id,
        reference=reference,
        match_tolerance_s=args.match_tolerance_s,
        threshold_m=args.threshold_m,
    )
    write_rows(args.out_csv, rows)
    summary = build_summary(rows, candidates)
    summary_json = args.summary_json or args.out_csv.with_suffix(".summary.json")
    write_json(summary_json, summary)
    print(
        "PPC candidate features: "
        f"rows={summary['rows']} epochs={summary['epochs']} "
        f"candidates={len(candidates)} out={args.out_csv}"
    )
    print(f"PPC candidate feature summary: {summary_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
