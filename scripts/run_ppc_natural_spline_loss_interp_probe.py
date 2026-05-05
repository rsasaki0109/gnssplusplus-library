#!/usr/bin/env python3
"""Natural cubic spline (scipy CubicSpline) over PPC loss spans.

Like run_ppc_cubic_loss_interp_probe.py but uses scipy.interpolate.CubicSpline
fitted through 4-6 scored anchors (2-3 before + 2-3 after) for an alternative
smoothing characteristic vs Hermite Hermite (different tangent definition).
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline, Akima1DInterpolator, PchipInterpolator, UnivariateSpline

ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = Path(__file__).resolve().parent
APPS_DIR = ROOT_DIR / "apps"
for _p in (str(SCRIPTS_DIR), str(APPS_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import generate_driving_comparison as comparison  # noqa: E402
import gnss_ppc_metrics as ppc_metrics  # noqa: E402
import apply_ppc_dual_profile_selector as pos_writer  # noqa: E402

DEFAULT_RUNS = (
    ("tokyo", "run1"),
    ("tokyo", "run2"),
    ("tokyo", "run3"),
    ("nagoya", "run1"),
    ("nagoya", "run2"),
    ("nagoya", "run3"),
)
THRESHOLD_M = 0.50
MATCH_TOL_S = 0.25
BRIDGE_STATUS = 5
BRIDGE_NUM_SAT = 0


@dataclass(frozen=True)
class LossSpan:
    start_ref_index: int
    end_ref_index: int


def gps_seconds(week: int, tow: float) -> float:
    return week * 604800.0 + tow


def _anchor_passes_internal_quality(epoch, min_ratio, max_post_rms_m, min_satellites):
    """Truth-blind anchor filter: status == FIXED + ratio/post_rms/sat thresholds."""
    if epoch.status != 4:  # FIXED
        return False
    if min_ratio is not None:
        if epoch.ratio is None or epoch.ratio < min_ratio:
            return False
    if max_post_rms_m is not None:
        post_rms = epoch.rtk_update_post_suppression_residual_rms_m
        if post_rms is None or post_rms > max_post_rms_m:
            return False
    if min_satellites is not None:
        if epoch.num_satellites is None or epoch.num_satellites < min_satellites:
            return False
    return True


def load_proba_map(proba_csv_path):
    """Load per-epoch probability map keyed by (run_key, round(tow,3))."""
    import csv as _csv
    out = {}
    with open(proba_csv_path) as h:
        rd = _csv.DictReader(h)
        for row in rd:
            key = (row["run_key"], round(float(row["tow_s"]), 3))
            out[key] = float(row["cv_proba"])
    return out


def load_predmag_map(predmag_csv_path):
    """Load per-epoch predicted error magnitude (m) keyed by (run_key, round(tow,3))."""
    import csv as _csv
    out = {}
    with open(predmag_csv_path) as h:
        rd = _csv.DictReader(h)
        for row in rd:
            key = (row["run_key"], round(float(row["tow_s"]), 3))
            out[key] = float(row["cv_pred_err_m"])
    return out


def collect_anchors(
    records,
    epoch_by_sol_tow,
    reference,
    anchor_mode="truth_scored",
    min_ratio=None,
    max_post_rms_m=None,
    min_satellites=None,
    proba_map=None,
    proba_run_key=None,
    proba_anchor_threshold=0.6,
    predmag_map=None,
    predmag_anchor_max_m=0.5,
    weight_source=None,
    weight_sigma_floor_m=0.05,
):
    """Collect anchor (time, ecef) pairs.

    anchor_mode:
      - "truth_scored": use score_state=='scored' (within 0.5m of reference); benchmark only.
      - "internal_quality": use status==FIXED + ratio/post_rms/satellite thresholds; deployable.
      - "learned": use pre-computed classifier probability >= proba_anchor_threshold.
      - "regressor": use predicted error magnitude <= predmag_anchor_max_m.
    """
    out = []
    for rec in records:
        sol_tow = rec.get("solution_tow_s")
        if sol_tow is None:
            continue
        ref_idx = int(rec["reference_index"])
        ref = reference[ref_idx]
        epoch = epoch_by_sol_tow.get((ref.week, round(float(sol_tow), 6)))
        if epoch is None:
            continue
        if anchor_mode == "truth_scored":
            if rec["score_state"] != "scored":
                continue
        elif anchor_mode == "internal_quality":
            if not _anchor_passes_internal_quality(
                epoch, min_ratio, max_post_rms_m, min_satellites
            ):
                continue
        elif anchor_mode == "learned":
            if proba_map is None or proba_run_key is None:
                raise ValueError("learned anchor mode requires proba_map and proba_run_key")
            proba = proba_map.get((proba_run_key, round(float(epoch.tow), 3)))
            if proba is None or proba < proba_anchor_threshold:
                continue
        elif anchor_mode == "regressor":
            if predmag_map is None or proba_run_key is None:
                raise ValueError("regressor anchor mode requires predmag_map and proba_run_key")
            pred = predmag_map.get((proba_run_key, round(float(epoch.tow), 3)))
            if pred is None or pred > predmag_anchor_max_m:
                continue
        else:
            raise ValueError(f"unknown anchor_mode: {anchor_mode}")
        weight = None
        if weight_source == "predmag":
            if predmag_map is not None and proba_run_key is not None:
                pred = predmag_map.get((proba_run_key, round(float(epoch.tow), 3)))
                if pred is not None:
                    sigma = max(weight_sigma_floor_m, float(pred))
                    weight = 1.0 / (sigma * sigma)
        elif weight_source == "proba":
            if proba_map is not None and proba_run_key is not None:
                proba = proba_map.get((proba_run_key, round(float(epoch.tow), 3)))
                if proba is not None:
                    # sigma ~ (1 - proba) * scale + floor
                    sigma = max(weight_sigma_floor_m, (1.0 - float(proba)) * 1.0)
                    weight = 1.0 / (sigma * sigma)
        out.append(
            (
                gps_seconds(ref.week, float(rec["end_tow_s"])),
                np.asarray(epoch.ecef, dtype=float),
                weight,
            )
        )
    out.sort(key=lambda item: item[0])
    return out


def _is_loss_epoch(rec, epoch, loss_mode, loss_max_ratio,
                   proba_map=None, proba_run_key=None, proba_loss_threshold=0.3,
                   predmag_map=None, predmag_loss_min_m=1.0):
    """Decide whether to OVERWRITE this baseline epoch with interpolation."""
    if loss_mode == "truth":
        return rec["score_state"] in ("no_solution", "high_error")
    if epoch is None:
        return True
    if loss_mode == "no_solution_only":
        return False
    if loss_mode == "internal":
        if epoch.status != 4:
            return True
        if loss_max_ratio is not None:
            if epoch.ratio is None or epoch.ratio < loss_max_ratio:
                return True
        return False
    if loss_mode == "learned":
        if proba_map is None or proba_run_key is None:
            raise ValueError("learned loss mode requires proba_map and proba_run_key")
        proba = proba_map.get((proba_run_key, round(float(epoch.tow), 3)))
        if proba is None:
            return True
        return proba < proba_loss_threshold
    if loss_mode == "regressor":
        if predmag_map is None or proba_run_key is None:
            raise ValueError("regressor loss mode requires predmag_map and proba_run_key")
        pred = predmag_map.get((proba_run_key, round(float(epoch.tow), 3)))
        if pred is None:
            return True
        return pred >= predmag_loss_min_m
    raise ValueError(f"unknown loss_mode: {loss_mode}")


def loss_spans(
    records,
    epoch_by_sol_tow=None,
    reference=None,
    loss_mode="truth",
    loss_max_ratio=None,
    proba_map=None,
    proba_run_key=None,
    proba_loss_threshold=0.3,
    predmag_map=None,
    predmag_loss_min_m=1.0,
):
    """Identify loss spans = contiguous epochs to interpolate over."""
    spans = []
    cur = []
    for rec in records:
        sol_tow = rec.get("solution_tow_s")
        if sol_tow is None:
            epoch = None
        elif epoch_by_sol_tow is not None and reference is not None:
            ref_idx = int(rec["reference_index"])
            ref = reference[ref_idx]
            epoch = epoch_by_sol_tow.get((ref.week, round(float(sol_tow), 6)))
        else:
            epoch = None
        if _is_loss_epoch(rec, epoch, loss_mode, loss_max_ratio,
                          proba_map=proba_map, proba_run_key=proba_run_key,
                          proba_loss_threshold=proba_loss_threshold,
                          predmag_map=predmag_map,
                          predmag_loss_min_m=predmag_loss_min_m):
            cur.append(rec)
            continue
        if cur:
            spans.append(LossSpan(int(cur[0]["reference_index"]),
                                   int(cur[-1]["reference_index"])))
            cur = []
    if cur:
        spans.append(LossSpan(int(cur[0]["reference_index"]),
                               int(cur[-1]["reference_index"])))
    return spans


_INTERP_MIN_PTS = {"natural": 3, "akima": 5, "pchip": 4, "weighted": 4, "linear_extrap": 4}


def _estimate_velocity(pts):
    """Linear least-squares velocity from a list of (time, ecef) points."""
    if len(pts) < 2:
        return None, None
    ts = np.array([p[0] for p in pts])
    xs = np.array([[p[1][0], p[1][1], p[1][2]] for p in pts])
    t_mean = ts.mean()
    dt = ts - t_mean
    denom = float((dt * dt).sum())
    if denom <= 0.0:
        return None, None
    v = np.array([float((dt * xs[:, k]).sum() / denom) for k in range(3)])
    p_at_mean = xs.mean(axis=0)
    return v, (t_mean, p_at_mean)


def spline_interp(
    anchors,
    target_seconds,
    span_start_s,
    span_end_s,
    max_anchor_age_s,
    n_each_side,
    interp_kind="natural",
    max_future_lag_s=None,
    weighted_smoothing_factor=1.0,
):
    times = [a[0] for a in anchors]
    idx = bisect.bisect_left(times, span_start_s)
    before = [a for a in anchors[:idx] if span_start_s - a[0] <= max_anchor_age_s]
    after_cap = max_anchor_age_s if max_future_lag_s is None else max_future_lag_s
    after = [a for a in anchors[idx:] if a[0] - span_end_s <= after_cap]
    before = before[-n_each_side:]
    after = after[:n_each_side]
    if len(before) < 1 or len(after) < 1:
        return None
    pts = before + after
    if len(pts) < 2:
        return None
    min_pts = _INTERP_MIN_PTS.get(interp_kind, 3)
    if len(pts) < min_pts:
        # Fall back to linear when not enough points for chosen interpolator
        p0, p1 = before[-1], after[0]
        if p1[0] - p0[0] <= 0:
            return None
        s = (target_seconds - p0[0]) / (p1[0] - p0[0])
        if not (0.0 <= s <= 1.0):
            return None
        return (1 - s) * p0[1] + s * p1[1]
    ts = np.array([p[0] for p in pts])
    xs = np.array([p[1][0] for p in pts])
    ys = np.array([p[1][1] for p in pts])
    zs = np.array([p[1][2] for p in pts])
    if interp_kind == "linear_extrap":
        # Dead-reckoning: estimate velocity from each side, blend.
        # Requires at least 2 anchors per side. Works even when target is
        # outside the fitting region of a spline (long unbridgeable spans).
        if len(before) < 2 or len(after) < 2:
            return None
        v_b, anchor_b = _estimate_velocity(before)
        v_a, anchor_a = _estimate_velocity(after)
        if v_b is None or v_a is None:
            return None
        t_b, p_b = anchor_b
        t_a, p_a = anchor_a
        x_fwd = p_b + v_b * (target_seconds - t_b)
        x_bwd = p_a + v_a * (target_seconds - t_a)
        span_len = max(1e-6, span_end_s - span_start_s)
        alpha = (target_seconds - span_start_s) / span_len
        alpha = max(0.0, min(1.0, alpha))
        return (1.0 - alpha) * x_fwd + alpha * x_bwd
    if not (ts[0] < target_seconds < ts[-1]):
        return None
    if interp_kind == "natural":
        cs_x = CubicSpline(ts, xs, bc_type="natural")
        cs_y = CubicSpline(ts, ys, bc_type="natural")
        cs_z = CubicSpline(ts, zs, bc_type="natural")
    elif interp_kind == "akima":
        cs_x = Akima1DInterpolator(ts, xs)
        cs_y = Akima1DInterpolator(ts, ys)
        cs_z = Akima1DInterpolator(ts, zs)
    elif interp_kind == "pchip":
        cs_x = PchipInterpolator(ts, xs)
        cs_y = PchipInterpolator(ts, ys)
        cs_z = PchipInterpolator(ts, zs)
    elif interp_kind == "weighted":
        # Weighted least-squares cubic UnivariateSpline.
        # Each anchor (t_i, p_i) has implied sigma_i; weight = 1/sigma_i.
        # UnivariateSpline w argument expects 1/sigma (not 1/sigma^2).
        w = np.array([
            (1.0 if (p[2] is None) else math.sqrt(float(p[2])))
            for p in pts
        ])
        # smoothing factor s scales with sum(w^2 * residual^2). With unit
        # variance and weight=1/sigma, target s = N * smoothing_factor where
        # smoothing_factor=1.0 means "expected residual ~1 sigma per anchor".
        s_target = float(len(pts)) * float(weighted_smoothing_factor)
        k = min(3, len(pts) - 1)
        try:
            sx = UnivariateSpline(ts, xs, w=w, k=k, s=s_target)
            sy = UnivariateSpline(ts, ys, w=w, k=k, s=s_target)
            sz = UnivariateSpline(ts, zs, w=w, k=k, s=s_target)
            return np.array([float(sx(target_seconds)),
                             float(sy(target_seconds)),
                             float(sz(target_seconds))])
        except Exception:
            return None
    else:
        raise ValueError(f"unknown interp_kind: {interp_kind}")
    return np.array([cs_x(target_seconds), cs_y(target_seconds), cs_z(target_seconds)])


def epoch_key(week, tow):
    return week, round(tow, 6)


def write_pos_with_overrides(out_path, baseline_epochs, overrides, new_epochs):
    by_key = {}
    for ep in baseline_epochs:
        key = epoch_key(ep.week, ep.tow)
        ecef = overrides.get(key)
        if ecef is None:
            by_key[key] = ep
        else:
            lat, lon, h = ppc_metrics.llh_from_ecef(
                float(ecef[0]), float(ecef[1]), float(ecef[2])
            )
            by_key[key] = comparison.SolutionEpoch(
                ep.week, ep.tow, lat, lon, h, ecef.copy(),
                BRIDGE_STATUS, BRIDGE_NUM_SAT,
            )
    for ep in new_epochs:
        by_key[epoch_key(ep.week, ep.tow)] = ep
    pos_writer.write_pos(out_path, [by_key[k] for k in sorted(by_key)])


def process_run(city, run, dataset_root, baseline_template, output_template,
                max_anchor_age_s, n_each_side, interp_kind="natural",
                max_future_lag_s=None, anchor_mode="truth_scored",
                min_ratio=None, max_post_rms_m=None, min_satellites=None,
                loss_mode="truth", loss_max_ratio=None,
                proba_map=None, proba_anchor_threshold=0.6,
                proba_loss_threshold=0.3,
                predmag_map=None, predmag_anchor_max_m=0.5,
                predmag_loss_min_m=1.0,
                weight_source=None, weight_sigma_floor_m=0.05,
                weighted_smoothing_factor=1.0):
    key = f"{city}_{run}"
    ref_csv = dataset_root / city / run / "reference.csv"
    baseline_pos = Path(baseline_template.format(key=key))
    out_pos = Path(output_template.format(key=key))
    out_pos.parent.mkdir(parents=True, exist_ok=True)

    reference = comparison.read_reference_csv(ref_csv)
    baseline_epochs = comparison.read_libgnss_pos(baseline_pos)
    epoch_by_sol_tow = {epoch_key(ep.week, ep.tow): ep for ep in baseline_epochs}
    records = ppc_metrics.ppc_official_segment_records(
        reference, baseline_epochs, MATCH_TOL_S, threshold_m=THRESHOLD_M
    )
    anchors = collect_anchors(
        records, epoch_by_sol_tow, reference,
        anchor_mode=anchor_mode,
        min_ratio=min_ratio,
        max_post_rms_m=max_post_rms_m,
        min_satellites=min_satellites,
        proba_map=proba_map,
        proba_run_key=key,
        proba_anchor_threshold=proba_anchor_threshold,
        predmag_map=predmag_map,
        predmag_anchor_max_m=predmag_anchor_max_m,
        weight_source=weight_source,
        weight_sigma_floor_m=weight_sigma_floor_m,
    )
    spans = loss_spans(
        records,
        epoch_by_sol_tow=epoch_by_sol_tow,
        reference=reference,
        loss_mode=loss_mode,
        loss_max_ratio=loss_max_ratio,
        proba_map=proba_map,
        proba_run_key=key,
        proba_loss_threshold=proba_loss_threshold,
        predmag_map=predmag_map,
        predmag_loss_min_m=predmag_loss_min_m,
    )

    overrides = {}
    new_epochs = []

    for span in spans:
        ref_start = reference[span.start_ref_index]
        ref_end = reference[span.end_ref_index]
        span_start_s = gps_seconds(ref_start.week, ref_start.tow)
        span_end_s = gps_seconds(ref_end.week, ref_end.tow)
        for ref_idx in range(span.start_ref_index, span.end_ref_index + 1):
            if ref_idx >= len(reference):
                continue
            ref = reference[ref_idx]
            target_s = gps_seconds(ref.week, ref.tow)
            ecef = spline_interp(
                anchors, target_s, span_start_s, span_end_s,
                max_anchor_age_s, n_each_side, interp_kind,
                max_future_lag_s=max_future_lag_s,
                weighted_smoothing_factor=weighted_smoothing_factor,
            )
            if ecef is None:
                continue
            k_ = epoch_key(ref.week, ref.tow)
            if k_ in epoch_by_sol_tow:
                overrides[k_] = ecef
            else:
                lat, lon, h = ppc_metrics.llh_from_ecef(
                    float(ecef[0]), float(ecef[1]), float(ecef[2])
                )
                new_epochs.append(
                    comparison.SolutionEpoch(
                        ref.week, ref.tow, lat, lon, h, ecef.copy(),
                        BRIDGE_STATUS, BRIDGE_NUM_SAT,
                    )
                )

    write_pos_with_overrides(out_pos, baseline_epochs, overrides, new_epochs)
    new_eps = comparison.read_libgnss_pos(out_pos)
    metrics = ppc_metrics.ppc_official_distance_score(
        reference, new_eps, MATCH_TOL_S, threshold_m=THRESHOLD_M
    )
    base_metrics = ppc_metrics.ppc_official_distance_score(
        reference, baseline_epochs, MATCH_TOL_S, threshold_m=THRESHOLD_M
    )
    return {
        "key": key,
        "baseline_pct": base_metrics["ppc_official_score_pct"],
        "new_pct": metrics["ppc_official_score_pct"],
        "delta_m": metrics["ppc_official_score_distance_m"]
        - base_metrics["ppc_official_score_distance_m"],
        "total_m": base_metrics["ppc_official_total_distance_m"],
        "scored_m": metrics["ppc_official_score_distance_m"],
    }


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--dataset-root", type=Path,
                   default=Path("/media/sasaki/aiueo/ai_coding_ws/datasets/PPC-Dataset-data"))
    p.add_argument("--baseline-pos-template",
                   default="output/ppc_selector_refresh/bridge_iter_after73114/oracle_current_plus_unsummarized16/{key}.pos")
    p.add_argument("--output-template",
                   default="output/ppc_selector_refresh/bridge_iter_after73270/spline_loss_interp_probe/{key}.pos")
    p.add_argument("--max-anchor-age-s", type=float, default=120.0)
    p.add_argument("--n-each-side", type=int, default=2)
    p.add_argument("--interp-kind", choices=("natural", "akima", "pchip", "weighted", "linear_extrap"), default="natural")
    p.add_argument("--weight-source", choices=("predmag", "proba"), default=None,
                   help="Source for per-anchor weight in weighted interp_kind. predmag uses sigma=max(floor,pred_err_m); proba uses sigma=(1-p).")
    p.add_argument("--weight-sigma-floor-m", type=float, default=0.05,
                   help="Minimum sigma when computing per-anchor weight from predmag/proba.")
    p.add_argument("--weighted-smoothing-factor", type=float, default=1.0,
                   help="UnivariateSpline smoothing target = N * factor. 0 = exact interp, 1 = ~1 sigma residual per anchor, larger = smoother.")
    p.add_argument("--max-future-lag-s", type=float, default=None,
                   help="If set, cap future anchor lookahead from span_end (causal fixed-lag smoother).")
    p.add_argument("--anchor-mode", choices=("truth_scored", "internal_quality", "learned", "regressor"),
                   default="truth_scored",
                   help="Anchor selection: truth_scored (benchmark, uses reference.csv), internal_quality (rules), learned (classifier probability), or regressor (predicted error magnitude).")
    p.add_argument("--anchor-min-ratio", type=float, default=None,
                   help="In internal_quality mode, minimum LAMBDA ratio for an anchor.")
    p.add_argument("--anchor-max-post-rms-m", type=float, default=None,
                   help="In internal_quality mode, maximum post-suppression residual RMS (m).")
    p.add_argument("--anchor-min-satellites", type=int, default=None,
                   help="In internal_quality mode, minimum number of satellites for an anchor.")
    p.add_argument("--loss-mode", choices=("truth", "no_solution_only", "internal", "learned", "regressor"),
                   default="truth",
                   help="Loss-span detection.")
    p.add_argument("--loss-max-ratio", type=float, default=None,
                   help="In internal loss-mode, FIXED epochs with ratio < this are loss.")
    p.add_argument("--proba-csv", type=Path, default=None,
                   help="Path to per-epoch CV probabilities CSV (run_key,tow_s,cv_proba).")
    p.add_argument("--proba-anchor-threshold", type=float, default=0.6,
                   help="learned anchor mode: proba >= this = anchor.")
    p.add_argument("--proba-loss-threshold", type=float, default=0.3,
                   help="learned loss mode: proba < this = loss epoch.")
    p.add_argument("--predmag-csv", type=Path, default=None,
                   help="Path to per-epoch predicted error magnitude CSV (run_key,tow_s,cv_pred_err_m).")
    p.add_argument("--predmag-anchor-max-m", type=float, default=0.5,
                   help="regressor anchor mode: pred <= this = anchor.")
    p.add_argument("--predmag-loss-min-m", type=float, default=1.0,
                   help="regressor loss mode: pred >= this = loss epoch.")
    p.add_argument("--summary-json", type=Path, default=None)
    p.add_argument("--summary-md", type=Path, default=None)
    p.add_argument("--run", action="append", default=[])
    return p.parse_args()


def main():
    args = parse_args()
    runs = ([tuple(r.split("/")) for r in args.run] if args.run else list(DEFAULT_RUNS))
    proba_map = load_proba_map(args.proba_csv) if args.proba_csv else None
    predmag_map = load_predmag_map(args.predmag_csv) if args.predmag_csv else None
    summaries = []
    for city, run in runs:
        print(f"[{city}/{run}] processing", flush=True)
        s = process_run(
            city, run, args.dataset_root, args.baseline_pos_template,
            args.output_template, args.max_anchor_age_s, args.n_each_side,
            args.interp_kind,
            max_future_lag_s=args.max_future_lag_s,
            anchor_mode=args.anchor_mode,
            min_ratio=args.anchor_min_ratio,
            max_post_rms_m=args.anchor_max_post_rms_m,
            min_satellites=args.anchor_min_satellites,
            loss_mode=args.loss_mode,
            loss_max_ratio=args.loss_max_ratio,
            proba_map=proba_map,
            proba_anchor_threshold=args.proba_anchor_threshold,
            proba_loss_threshold=args.proba_loss_threshold,
            predmag_map=predmag_map,
            predmag_anchor_max_m=args.predmag_anchor_max_m,
            predmag_loss_min_m=args.predmag_loss_min_m,
            weight_source=args.weight_source,
            weight_sigma_floor_m=args.weight_sigma_floor_m,
            weighted_smoothing_factor=args.weighted_smoothing_factor,
        )
        print(f"  baseline {s['baseline_pct']:.6f}% -> new {s['new_pct']:.6f}% "
              f"(Δ {s['delta_m']:+.3f} m)", flush=True)
        summaries.append(s)

    total = sum(s["total_m"] for s in summaries)
    score = sum(s["scored_m"] for s in summaries)
    base = sum(s["scored_m"] - s["delta_m"] for s in summaries)
    print(f"\nAggregate: {100*base/total:.6f}% -> {100*score/total:.6f}% "
          f"(Δ {score - base:+.3f} m)")

    if args.summary_md is None:
        args.summary_md = Path(args.output_template.format(key="matrix")).with_name("matrix.md")
    args.summary_md.parent.mkdir(parents=True, exist_ok=True)
    lag_label = "" if args.max_future_lag_s is None else f", lag={args.max_future_lag_s}s"
    lines = [
        f"# Spline loss-interp probe ({args.interp_kind}, n={args.n_each_side}, anchor_age={args.max_anchor_age_s}s{lag_label})",
        "",
        f"Aggregate: **{100*base/total:.6f}% -> {100*score/total:.6f}%** (Δ {score-base:+.3f} m)",
        "",
        "| run | baseline | new | delta m |",
        "|---|---:|---:|---:|",
    ]
    for s in summaries:
        lines.append(f"| {s['key']} | {s['baseline_pct']:.6f}% | {s['new_pct']:.6f}% | {s['delta_m']:+.3f} |")
    args.summary_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote {args.summary_md}")


if __name__ == "__main__":
    main()
