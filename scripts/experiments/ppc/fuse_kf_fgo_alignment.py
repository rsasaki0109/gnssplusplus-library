#!/usr/bin/env python3
"""Causally align a stable FGO trajectory with trusted KF fixes.

The combiner consumes only solver outputs. Reference truth is deliberately not
an input: FGO CSVs must contain the absolute ``x/y/z_ecef_m`` columns emitted
by ``gnss_fgo_parity --dump-csv``.
"""

from __future__ import annotations

from _paths import ANALYSIS_DIR, APPS_DIR, COMMANDS_DIR, PPC_DIR, ROOT_DIR, SCRIPTS_DIR

import argparse
import bisect
from collections import deque
import csv
import json
import math
from pathlib import Path
import statistics

import numpy as np


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kf-pos", type=Path, required=True)
    parser.add_argument(
        "--fgo-csv",
        type=Path,
        action="append",
        required=True,
        help="FGO parity CSV; repeat for independently initialized gap-fill windows.",
    )
    parser.add_argument("--output-pos", type=Path, required=True)
    parser.add_argument("--summary-json", type=Path)
    parser.add_argument("--match-tolerance-s", type=float, default=0.11)
    parser.add_argument("--horizontal-ratio-min", type=float, default=20.0)
    parser.add_argument("--horizontal-window", type=int, default=50)
    parser.add_argument("--horizontal-gate-m", type=float, default=0.5)
    parser.add_argument("--vertical-ratio-min", type=float, default=3.0)
    parser.add_argument("--vertical-window", type=int, default=5)
    parser.add_argument("--vertical-gate-m", type=float, default=0.75)
    parser.add_argument(
        "--alignment-kf-max-nis-per-observation",
        type=float,
        default=0.0,
        help=(
            "Accept KF FIX alignment updates only at or below this normalized "
            "innovation squared per observation; 0 disables."
        ),
    )
    parser.add_argument(
        "--min-window-alignment-updates",
        type=int,
        default=0,
        help=(
            "For multi-window fusion, reject a window unless both horizontal and "
            "vertical alignment accepted at least this many trusted KF FIX updates."
        ),
    )
    parser.add_argument(
        "--max-alignment-age-epochs",
        type=int,
        default=0,
        help=(
            "Use aligned FGO only this many epochs after the latest accepted "
            "horizontal and vertical KF FIX alignment update; 0 disables the age gate."
        ),
    )
    parser.add_argument(
        "--require-alignment-initialized",
        action="store_true",
        help=(
            "Use gap-fill FGO only after both horizontal and vertical corrections "
            "have accepted a trusted KF FIX update."
        ),
    )
    parser.add_argument(
        "--align-fgo-fixed",
        action="store_true",
        help=(
            "Apply the same causal KF-to-FGO frame alignment to FGO FIXED "
            "epochs. Disabled by default so standalone FGO integer positions "
            "retain their native absolute frame."
        ),
    )
    parser.add_argument(
        "--alignment-trend-window",
        type=int,
        default=0,
        help=(
            "Causally extrapolate the KF-FGO correction from this many accepted "
            "KF FIX updates; 0 keeps the median-only alignment."
        ),
    )
    parser.add_argument(
        "--alignment-trend-max-rate-mps",
        type=float,
        default=0.0,
        help="Cap the horizontal and vertical correction trend rates; 0 disables the cap.",
    )
    parser.add_argument(
        "--alignment-trend-min-ddpr-rms-m",
        type=float,
        default=0.0,
        help="Apply correction trend only at or above this runtime DDPR RMS; 0 disables.",
    )
    parser.add_argument(
        "--alignment-trend-max-ddpr-rms-m",
        type=float,
        default=0.0,
        help="Apply correction trend only at or below this runtime DDPR RMS; 0 disables.",
    )
    parser.add_argument(
        "--alignment-trend-max-gdop",
        type=float,
        default=0.0,
        help="Apply correction trend only at or below this runtime GDOP; 0 disables.",
    )
    parser.add_argument(
        "--alignment-trend-dropout-max-ddpr-rms-m",
        type=float,
        default=0.0,
        help=(
            "For epochs with unavailable GDOP and DDPR RMS at or below this "
            "value, use the dropout-specific trend rate; 0 disables."
        ),
    )
    parser.add_argument(
        "--alignment-trend-dropout-max-rate-mps",
        type=float,
        default=0.0,
        help=(
            "Trend-rate cap for the bounded nonfinite-GDOP dropout bridge; "
            "0 disables."
        ),
    )
    parser.add_argument(
        "--kf-fallback",
        action="store_true",
        help="Keep KF epochs outside the FGO time span in the output.",
    )
    parser.add_argument(
        "--kf-gap-fill-only",
        action="store_true",
        help=(
            "Preserve every matched KF epoch and use aligned FGO only where KF is absent; "
            "--replace-kf-nonfixed overrides this for non-FIXED KF epochs."
        ),
    )
    parser.add_argument(
        "--replace-kf-nonfixed",
        action="store_true",
        help="Inside FGO windows preserve KF FIXED epochs, but replace KF non-FIXED epochs with aligned FGO FLOAT.",
    )
    parser.add_argument(
        "--replace-kf-nis-rejected",
        action="store_true",
        help=(
            "Inside FGO windows replace even KF FIXED epochs when their NIS per "
            "observation exceeds --alignment-kf-max-nis-per-observation; the "
            "replacement is conservatively labelled FLOAT."
        ),
    )
    parser.add_argument(
        "--replace-kf-nis-threshold",
        type=float,
        default=0.0,
        help=(
            "NIS-per-observation threshold used only for KF replacement. "
            "When 0, reuse --alignment-kf-max-nis-per-observation for "
            "backward compatibility."
        ),
    )
    parser.add_argument(
        "--replace-kf-nis-min-fgo-ddpr-rms-m",
        type=float,
        default=0.0,
        help=(
            "Require at least this FGO DD pseudorange RMS before replacing a "
            "KF FIX rejected by NIS; 0 disables the cross-check."
        ),
    )
    parser.add_argument(
        "--replace-kf-nis-min-fgo-gdop",
        type=float,
        default=0.0,
        help=(
            "Require at least this FGO GDOP before replacing a KF FIX rejected "
            "by NIS; 0 disables the cross-check."
        ),
    )
    return parser.parse_args()


def ecef_to_llh(ecef: np.ndarray) -> tuple[float, float, float]:
    x_m, y_m, z_m = (float(value) for value in ecef)
    lon = math.atan2(y_m, x_m)
    p = math.hypot(x_m, y_m)
    lat = math.atan2(z_m, p * (1.0 - WGS84_E2))
    for _ in range(8):
        sin_lat = math.sin(lat)
        radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        height = p / max(abs(math.cos(lat)), 1e-12) - radius
        lat = math.atan2(z_m, p * (1.0 - WGS84_E2 * radius / (radius + height)))
    sin_lat = math.sin(lat)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    height = p / max(abs(math.cos(lat)), 1e-12) - radius
    return math.degrees(lat), math.degrees(lon), height


def ecef_to_enu_rotation(ecef: np.ndarray) -> np.ndarray:
    lat_deg, lon_deg, _ = ecef_to_llh(ecef)
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    return np.asarray(
        [
            [-math.sin(lon), math.cos(lon), 0.0],
            [-math.sin(lat) * math.cos(lon), -math.sin(lat) * math.sin(lon), math.cos(lat)],
            [math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat)],
        ]
    )


def median_vector(values: deque[np.ndarray]) -> np.ndarray:
    return np.median(np.asarray(values), axis=0)


def read_kf(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            fields = line.split()
            if len(fields) < 11:
                raise ValueError(f"Malformed KF position row: {line.rstrip()}")
            rows.append(
                {
                    "week": int(fields[0]),
                    "tow": float(fields[1]),
                    "ecef": np.asarray([float(value) for value in fields[2:5]]),
                    "status": int(fields[8]),
                    "nsat": int(fields[9]),
                    "pdop": float(fields[10]),
                    # Interpolated/fallback POS rows may legitimately omit
                    # optional telemetry.  Zero is conservative: such rows
                    # cannot become trusted alignment updates.
                    "ratio": float(fields[11]) if len(fields) >= 12 else 0.0,
                    "baseline": float(fields[12]) if len(fields) >= 13 else 0.0,
                    "rtk_fields": fields[13:25] if len(fields) >= 25 else None,
                    "nis_per_observation": (
                        float(fields[23]) if len(fields) >= 25 else math.nan
                    ),
                }
            )
    if not rows:
        raise ValueError(f"No KF solution rows in {path}")
    return rows


def read_fgo(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    with path.open(encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            for name in ("x_ecef_m", "y_ecef_m", "z_ecef_m"):
                if not row.get(name, "").strip():
                    raise ValueError(
                        f"{path} lacks {name}; regenerate it with the current gnss_fgo_parity"
                    )
            status = row.get("status", "").strip().upper()
            if status not in {"FIXED", "FLOAT"}:
                continue
            rows.append(
                {
                    "tow": float(row["tow"]),
                    "ecef": np.asarray(
                        [float(row["x_ecef_m"]), float(row["y_ecef_m"]), float(row["z_ecef_m"])]
                    ),
                    "status": 4 if status == "FIXED" else 3,
                    "nsat": int(row.get("nsat") or 0),
                    "ratio": float(row.get("ratio") or 0.0),
                    "ddpr_rms_m": float(row.get("ddpr_rms_m") or "nan"),
                    "gdop": float(row.get("gdop") or "nan"),
                }
            )
    if not rows:
        raise ValueError(f"No FGO solution rows in {path}")
    return rows


def nearest_kf(
    rows: list[dict[str, object]], tows: list[float], tow: float, tolerance_s: float
) -> dict[str, object] | None:
    index = bisect.bisect_left(tows, tow)
    candidates = [i for i in (index - 1, index) if 0 <= i < len(rows)]
    if not candidates:
        return None
    best = min(candidates, key=lambda i: abs(tows[i] - tow))
    return rows[best] if abs(tows[best] - tow) <= tolerance_s else None


def accept_vector(
    history: deque[np.ndarray], candidate: np.ndarray, gate_m: float
) -> bool:
    center = median_vector(history) if history else candidate
    if len(history) >= 5 and float(np.linalg.norm(candidate - center)) > gate_m:
        return False
    history.append(candidate)
    return True


def accept_scalar(history: deque[float], candidate: float, gate_m: float) -> bool:
    center = statistics.median(history) if history else candidate
    if len(history) >= 5 and abs(candidate - center) > gate_m:
        return False
    history.append(candidate)
    return True


def kf_nis_rejected(
    kf: dict[str, object] | None, maximum: float
) -> bool:
    if kf is None:
        return False
    value = float(kf.get("nis_per_observation", math.nan))
    return maximum > 0.0 and (not math.isfinite(value) or value > maximum)


def replacement_nis_threshold(args: argparse.Namespace) -> float:
    replacement = float(getattr(args, "replace_kf_nis_threshold", 0.0))
    if replacement > 0.0:
        return replacement
    return float(getattr(args, "alignment_kf_max_nis_per_observation", 0.0))


def replacement_fgo_quality_ok(
    fgo: dict[str, object], args: argparse.Namespace
) -> bool:
    minimum_ddpr = float(
        getattr(args, "replace_kf_nis_min_fgo_ddpr_rms_m", 0.0)
    )
    minimum_gdop = float(getattr(args, "replace_kf_nis_min_fgo_gdop", 0.0))
    ddpr = float(fgo.get("ddpr_rms_m", math.nan))
    gdop = float(fgo.get("gdop", math.nan))
    return (
        minimum_ddpr <= 0.0 or (math.isfinite(ddpr) and ddpr >= minimum_ddpr)
    ) and (
        minimum_gdop <= 0.0 or (math.isfinite(gdop) and gdop >= minimum_gdop)
    )


def robust_trend_prediction(
    history: deque[tuple[float, np.ndarray]],
    tow: float,
    max_rate_mps: float,
) -> np.ndarray | None:
    """Theil-Sen extrapolation of accepted corrections, using no reference truth."""
    if len(history) < 5:
        return None
    times = np.asarray([item[0] for item in history], dtype=float)
    values = np.asarray([item[1] for item in history], dtype=float)
    slopes: list[np.ndarray] = []
    for end in range(1, len(history)):
        dt = times[end] - times[:end]
        valid = np.abs(dt) > 1e-9
        if np.any(valid):
            slopes.extend((values[end] - values[:end][valid]) / dt[valid, None])
    if not slopes:
        return None
    slope = np.median(np.asarray(slopes), axis=0)
    rate = float(np.linalg.norm(slope))
    if max_rate_mps > 0.0 and rate > max_rate_mps:
        slope *= max_rate_mps / rate
    intercepts_at_tow = values + (tow - times)[:, None] * slope
    return np.median(intercepts_at_tow, axis=0)


def fuse(
    kf_rows: list[dict[str, object]],
    fgo_rows: list[dict[str, object]],
    args: argparse.Namespace,
) -> tuple[list[dict[str, object]], dict[str, object]]:
    kf_tows = [float(row["tow"]) for row in kf_rows]
    horizontal: deque[np.ndarray] = deque(maxlen=args.horizontal_window)
    vertical: deque[float] = deque(maxlen=args.vertical_window)
    trend_window = int(getattr(args, "alignment_trend_window", 0))
    horizontal_trend: deque[tuple[float, np.ndarray]] = deque(
        maxlen=max(1, trend_window)
    )
    vertical_trend: deque[tuple[float, np.ndarray]] = deque(
        maxlen=max(1, trend_window)
    )
    fused: list[dict[str, object]] = []
    horizontal_accepts = 0
    vertical_accepts = 0
    horizontal_age = 1 << 30
    vertical_age = 1 << 30
    stale_fgo_rejections = 0
    uninitialized_alignment_rejections = 0
    trend_prediction_epochs = 0
    kf_telemetry_rejections = 0
    kf_nis_rejected_replacements = 0
    retained_kf_keys: set[tuple[int, int]] = set()
    covered_kf_keys: set[tuple[int, int]] = set()

    for fgo in fgo_rows:
        horizontal_age += 1
        vertical_age += 1
        tow = float(fgo["tow"])
        fgo_ddpr_rms = float(fgo.get("ddpr_rms_m", math.nan))
        fgo_gdop = float(fgo.get("gdop", math.nan))
        trend_min_ddpr = float(
            getattr(args, "alignment_trend_min_ddpr_rms_m", 0.0)
        )
        trend_max_ddpr = float(
            getattr(args, "alignment_trend_max_ddpr_rms_m", 0.0)
        )
        trend_max_gdop = float(getattr(args, "alignment_trend_max_gdop", 0.0))
        trend_telemetry_ok = (
            (trend_min_ddpr <= 0.0 or (math.isfinite(fgo_ddpr_rms) and fgo_ddpr_rms >= trend_min_ddpr))
            and (trend_max_ddpr <= 0.0 or (math.isfinite(fgo_ddpr_rms) and fgo_ddpr_rms <= trend_max_ddpr))
            and (trend_max_gdop <= 0.0 or (math.isfinite(fgo_gdop) and fgo_gdop <= trend_max_gdop))
        )
        trend_rate_mps = float(
            getattr(args, "alignment_trend_max_rate_mps", 0.0)
        )
        dropout_max_ddpr = float(
            getattr(args, "alignment_trend_dropout_max_ddpr_rms_m", 0.0)
        )
        dropout_rate_mps = float(
            getattr(args, "alignment_trend_dropout_max_rate_mps", 0.0)
        )
        if (
            dropout_max_ddpr > 0.0
            and dropout_rate_mps > 0.0
            and not math.isfinite(fgo_gdop)
            and math.isfinite(fgo_ddpr_rms)
            and fgo_ddpr_rms <= dropout_max_ddpr
        ):
            trend_rate_mps = dropout_rate_mps
        fgo_ecef = np.asarray(fgo["ecef"])
        rotation = ecef_to_enu_rotation(fgo_ecef)
        kf = nearest_kf(kf_rows, kf_tows, tow, args.match_tolerance_s)
        if kf is not None:
            covered_kf_keys.add(
                (int(kf["week"]), round(float(kf["tow"]) * 1000.0))
            )
        if kf is not None and int(kf["status"]) == 4:
            candidate = rotation @ (np.asarray(kf["ecef"]) - fgo_ecef)
            ratio = float(kf["ratio"])
            kf_telemetry_ok = not kf_nis_rejected(
                kf,
                float(getattr(args, "alignment_kf_max_nis_per_observation", 0.0)),
            )
            if not kf_telemetry_ok:
                kf_telemetry_rejections += 1
            if kf_telemetry_ok and ratio >= args.horizontal_ratio_min and accept_vector(
                horizontal, candidate[:2], args.horizontal_gate_m
            ):
                horizontal_accepts += 1
                horizontal_age = 0
                if trend_window > 0:
                    horizontal_trend.append((tow, candidate[:2].copy()))
            if kf_telemetry_ok and ratio >= args.vertical_ratio_min and accept_scalar(
                vertical, float(candidate[2]), args.vertical_gate_m
            ):
                vertical_accepts += 1
                vertical_age = 0
                if trend_window > 0:
                    vertical_trend.append(
                        (tow, np.asarray([float(candidate[2])], dtype=float))
                    )

        correction = np.zeros(3)
        # Standalone FGO FIXED epochs retain their native absolute frame by
        # default.  Consensus/fusion experiments can opt into a common causal
        # frame because integer fixing does not itself eliminate a persistent
        # position-frame offset.
        if int(fgo["status"]) != 4 or bool(
            getattr(args, "align_fgo_fixed", False)
        ):
            used_trend = False
            if horizontal:
                correction[:2] = median_vector(horizontal)
                horizontal_prediction = (
                    robust_trend_prediction(
                        horizontal_trend,
                        tow,
                        trend_rate_mps,
                    )
                    if trend_telemetry_ok
                    else None
                )
                if horizontal_prediction is not None:
                    correction[:2] = horizontal_prediction
                    used_trend = True
            if vertical:
                correction[2] = statistics.median(vertical)
                vertical_prediction = (
                    robust_trend_prediction(
                        vertical_trend,
                        tow,
                        trend_rate_mps,
                    )
                    if trend_telemetry_ok
                    else None
                )
                if vertical_prediction is not None:
                    correction[2] = float(vertical_prediction[0])
                    used_trend = True
            if used_trend:
                trend_prediction_epochs += 1
        corrected_ecef = fgo_ecef + rotation.transpose() @ correction
        fused_row: dict[str, object] = {
                "week": int(kf["week"]) if kf is not None else int(kf_rows[0]["week"]),
                "tow": tow,
                "ecef": corrected_ecef,
                "status": int(fgo["status"]),
                "nsat": int(fgo["nsat"]),
                "pdop": 0.0,
                "ratio": float(fgo["ratio"]),
                "baseline": 0.0,
            }
        if getattr(args, "kf_gap_fill_only", False):
            # Gap fill is supplemental positioning coverage, not a claim that
            # the KF/FGO fused ambiguity state is integer-fixed. Preserve KF
            # FIX labels where KF exists; conservatively report inserted FGO
            # epochs as FLOAT even if FGO's standalone AR said FIXED.
            fused_row["status"] = 3
        replace_nis_rejected = bool(
            getattr(args, "replace_kf_nis_rejected", False)
            and kf_nis_rejected(kf, replacement_nis_threshold(args))
            and replacement_fgo_quality_ok(fgo, args)
        )
        replace_nonfixed = bool(
            getattr(args, "replace_kf_nonfixed", False)
            and kf is not None
            and int(kf["status"]) != 4
        )
        preserve_kf = (
            getattr(args, "kf_gap_fill_only", False)
            and kf is not None
            and not replace_nonfixed
            and not replace_nis_rejected
        )
        max_alignment_age = int(getattr(args, "max_alignment_age_epochs", 0))
        alignment_initialized = bool(horizontal) and bool(vertical)
        alignment_fresh = (
            max_alignment_age <= 0
            or (horizontal_age <= max_alignment_age and vertical_age <= max_alignment_age)
        )
        require_initialized = bool(
            getattr(args, "require_alignment_initialized", False)
        )
        reject_uninitialized = require_initialized and not alignment_initialized
        if getattr(args, "kf_gap_fill_only", False) and (
            reject_uninitialized or not alignment_fresh
        ):
            if reject_uninitialized:
                uninitialized_alignment_rejections += 1
            else:
                stale_fgo_rejections += 1
            if kf is None:
                continue
            fused_row = dict(kf)
            retained_kf_keys.add(
                (int(kf["week"]), round(float(kf["tow"]) * 1000.0))
            )
        if preserve_kf:
            fused_row = dict(kf)
            retained_kf_keys.add(
                (int(kf["week"]), round(float(kf["tow"]) * 1000.0))
            )
        elif replace_nis_rejected:
            kf_nis_rejected_replacements += 1
        fused.append(fused_row)

    if args.kf_fallback:
        first_tow = float(fgo_rows[0]["tow"])
        last_tow = float(fgo_rows[-1]["tow"])
        if getattr(args, "kf_gap_fill_only", False):
            excluded_kf_keys = (
                covered_kf_keys
                if (
                    getattr(args, "replace_kf_nonfixed", False)
                    or getattr(args, "replace_kf_nis_rejected", False)
                )
                else retained_kf_keys
            )
            fused.extend(
                row
                for row in kf_rows
                if (int(row["week"]), round(float(row["tow"]) * 1000.0))
                not in excluded_kf_keys
            )
        else:
            fused.extend(
                row
                for row in kf_rows
                if float(row["tow"]) < first_tow or float(row["tow"]) > last_tow
            )
        fused.sort(key=lambda row: (int(row["week"]), float(row["tow"])))

    summary = {
        "reference_truth_used": False,
        "fgo_epochs": len(fgo_rows),
        "output_epochs": len(fused),
        "horizontal_updates_accepted": horizontal_accepts,
        "vertical_updates_accepted": vertical_accepts,
        "stale_fgo_rejections": stale_fgo_rejections,
        "uninitialized_alignment_rejections": uninitialized_alignment_rejections,
        "require_alignment_initialized": bool(
            getattr(args, "require_alignment_initialized", False)
        ),
        "align_fgo_fixed": bool(getattr(args, "align_fgo_fixed", False)),
        "alignment_trend_window": trend_window,
        "alignment_trend_max_rate_mps": float(
            getattr(args, "alignment_trend_max_rate_mps", 0.0)
        ),
        "alignment_trend_min_ddpr_rms_m": float(
            getattr(args, "alignment_trend_min_ddpr_rms_m", 0.0)
        ),
        "alignment_trend_max_ddpr_rms_m": float(
            getattr(args, "alignment_trend_max_ddpr_rms_m", 0.0)
        ),
        "alignment_trend_max_gdop": float(
            getattr(args, "alignment_trend_max_gdop", 0.0)
        ),
        "alignment_trend_dropout_max_ddpr_rms_m": float(
            getattr(args, "alignment_trend_dropout_max_ddpr_rms_m", 0.0)
        ),
        "alignment_trend_dropout_max_rate_mps": float(
            getattr(args, "alignment_trend_dropout_max_rate_mps", 0.0)
        ),
        "trend_prediction_epochs": trend_prediction_epochs,
        "kf_telemetry_rejections": kf_telemetry_rejections,
        "kf_nis_rejected_replacements": kf_nis_rejected_replacements,
        "alignment_kf_max_nis_per_observation": float(
            getattr(args, "alignment_kf_max_nis_per_observation", 0.0)
        ),
        "max_alignment_age_epochs": int(getattr(args, "max_alignment_age_epochs", 0)),
        "horizontal_ratio_min": args.horizontal_ratio_min,
        "horizontal_window": args.horizontal_window,
        "horizontal_gate_m": args.horizontal_gate_m,
        "vertical_ratio_min": args.vertical_ratio_min,
        "vertical_window": args.vertical_window,
        "vertical_gate_m": args.vertical_gate_m,
        "kf_fallback": bool(args.kf_fallback),
        "kf_gap_fill_only": bool(getattr(args, "kf_gap_fill_only", False)),
        "replace_kf_nonfixed": bool(getattr(args, "replace_kf_nonfixed", False)),
        "replace_kf_nis_rejected": bool(
            getattr(args, "replace_kf_nis_rejected", False)
        ),
        "replace_kf_nis_threshold": replacement_nis_threshold(args),
        "replace_kf_nis_min_fgo_ddpr_rms_m": float(
            getattr(args, "replace_kf_nis_min_fgo_ddpr_rms_m", 0.0)
        ),
        "replace_kf_nis_min_fgo_gdop": float(
            getattr(args, "replace_kf_nis_min_fgo_gdop", 0.0)
        ),
        "gap_fill_fgo_status": "FLOAT" if getattr(args, "kf_gap_fill_only", False) else None,
    }
    return fused, summary


def fuse_windows(
    kf_rows: list[dict[str, object]],
    fgo_windows: list[list[dict[str, object]]],
    args: argparse.Namespace,
) -> tuple[list[dict[str, object]], dict[str, object]]:
    if len(fgo_windows) == 1:
        return fuse(kf_rows, fgo_windows[0], args)
    if not getattr(args, "kf_gap_fill_only", False):
        raise ValueError("multiple --fgo-csv inputs require --kf-gap-fill-only")

    merged: dict[tuple[int, int], dict[str, object]] = {}
    replace_nonfixed = bool(getattr(args, "replace_kf_nonfixed", False))
    replace_nis_rejected = bool(getattr(args, "replace_kf_nis_rejected", False))
    replace_any = replace_nonfixed or replace_nis_rejected
    if args.kf_fallback and not replace_any:
        for row in kf_rows:
            merged[(int(row["week"]), round(float(row["tow"]) * 1000.0))] = row

    window_summaries: list[dict[str, object]] = []
    accepted_window_count = 0
    for window in fgo_windows:
        window_args = argparse.Namespace(**vars(args))
        window_args.kf_fallback = False
        rows, summary = fuse(kf_rows, window, window_args)
        min_updates = int(getattr(args, "min_window_alignment_updates", 0))
        accepted = (
            int(summary["horizontal_updates_accepted"]) >= min_updates
            and int(summary["vertical_updates_accepted"]) >= min_updates
        )
        summary["window_accepted"] = accepted
        summary["min_window_alignment_updates"] = min_updates
        window_summaries.append(summary)
        if not accepted:
            continue
        accepted_window_count += 1
        for row in rows:
            key = (int(row["week"]), round(float(row["tow"]) * 1000.0))
            if key not in merged:
                merged[key] = row

    if args.kf_fallback and replace_any:
        for row in kf_rows:
            key = (int(row["week"]), round(float(row["tow"]) * 1000.0))
            if key not in merged:
                merged[key] = row

    output = [merged[key] for key in sorted(merged)]
    if replace_nis_rejected:
        merge_policy = (
            "FGO windows in CLI order replace requested KF epochs/missing; "
            "first solution wins per week/tow"
        )
    elif replace_nonfixed:
        merge_policy = (
            "KF FIXED preserved; FGO windows in CLI order replace non-FIXED/missing; "
            "first solution wins per week/tow"
        )
    else:
        merge_policy = "KF first; FGO windows in CLI order; first solution wins per week/tow"
    return output, {
        "reference_truth_used": False,
        "merge_policy": merge_policy,
        "kf_fallback": bool(args.kf_fallback),
        "kf_gap_fill_only": True,
        "replace_kf_nonfixed": replace_nonfixed,
        "replace_kf_nis_rejected": replace_nis_rejected,
        "gap_fill_fgo_status": "FLOAT",
        "fgo_window_count": len(fgo_windows),
        "fgo_windows_accepted": accepted_window_count,
        "fgo_epochs": sum(len(window) for window in fgo_windows),
        "output_epochs": len(output),
        "windows": window_summaries,
    }


def write_pos(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    keep_rtk_telemetry = any(row.get("rtk_fields") is not None for row in rows)
    with path.open("w", encoding="ascii", newline="\n") as handle:
        handle.write("% LibGNSS++ causal KF/FGO aligned solution\n")
        handle.write(
            "% GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) "
            "Status NumSat PDOP Ratio Baseline(m)"
            + (
                " RTKIter RTKObs RTKPhaseObs RTKCodeObs RTKOutliers "
                "RTKPrefitRMS(m) RTKPrefitMax(m) RTKPostSuppressRMS(m) "
                "RTKPostSuppressMax(m) RTKUpdateNIS RTKUpdateNISPerObs "
                "RTKUpdateNISRejected"
                if keep_rtk_telemetry
                else ""
            )
            + "\n"
        )
        for row in rows:
            ecef = np.asarray(row["ecef"])
            lat, lon, height = ecef_to_llh(ecef)
            line = (
                f"{int(row['week'])} {float(row['tow']):.3f} "
                f"{ecef[0]:.4f} {ecef[1]:.4f} {ecef[2]:.4f} "
                f"{lat:.9f} {lon:.9f} {height:.4f} {int(row['status'])} "
                f"{int(row['nsat'])} {float(row['pdop']):.2f} "
                f"{float(row['ratio']):.1f} {float(row['baseline']):.4f}"
            )
            if keep_rtk_telemetry:
                rtk_fields = row.get("rtk_fields")
                line += " " + " ".join(
                    str(value) for value in (rtk_fields if rtk_fields is not None else [0] * 12)
                )
            handle.write(line + "\n")


def main() -> int:
    args = parse_args()
    if args.horizontal_window < 1 or args.vertical_window < 1:
        raise SystemExit("alignment windows must be positive")
    if args.min_window_alignment_updates < 0:
        raise SystemExit("--min-window-alignment-updates must be non-negative")
    if args.max_alignment_age_epochs < 0:
        raise SystemExit("--max-alignment-age-epochs must be non-negative")
    if args.alignment_trend_window < 0:
        raise SystemExit("--alignment-trend-window must be non-negative")
    if args.alignment_trend_max_rate_mps < 0.0:
        raise SystemExit("--alignment-trend-max-rate-mps must be non-negative")
    if (
        args.alignment_trend_min_ddpr_rms_m < 0.0
        or args.alignment_trend_max_ddpr_rms_m < 0.0
    ):
        raise SystemExit("alignment trend DDPR bounds must be non-negative")
    if args.alignment_trend_max_gdop < 0.0:
        raise SystemExit("--alignment-trend-max-gdop must be non-negative")
    if args.alignment_trend_dropout_max_ddpr_rms_m < 0.0:
        raise SystemExit("--alignment-trend-dropout-max-ddpr-rms-m must be non-negative")
    if args.alignment_trend_dropout_max_rate_mps < 0.0:
        raise SystemExit("--alignment-trend-dropout-max-rate-mps must be non-negative")
    if args.alignment_kf_max_nis_per_observation < 0.0:
        raise SystemExit("--alignment-kf-max-nis-per-observation must be non-negative")
    if args.replace_kf_nis_threshold < 0.0:
        raise SystemExit("--replace-kf-nis-threshold must be non-negative")
    if args.replace_kf_nis_min_fgo_ddpr_rms_m < 0.0:
        raise SystemExit("--replace-kf-nis-min-fgo-ddpr-rms-m must be non-negative")
    if args.replace_kf_nis_min_fgo_gdop < 0.0:
        raise SystemExit("--replace-kf-nis-min-fgo-gdop must be non-negative")
    if (
        args.alignment_trend_min_ddpr_rms_m > 0.0
        and args.alignment_trend_max_ddpr_rms_m > 0.0
        and args.alignment_trend_min_ddpr_rms_m
        > args.alignment_trend_max_ddpr_rms_m
    ):
        raise SystemExit("alignment trend DDPR minimum exceeds maximum")
    if args.replace_kf_nonfixed and not args.kf_gap_fill_only:
        raise SystemExit("--replace-kf-nonfixed requires --kf-gap-fill-only")
    if args.replace_kf_nis_rejected and not args.kf_gap_fill_only:
        raise SystemExit("--replace-kf-nis-rejected requires --kf-gap-fill-only")
    if (
        args.replace_kf_nis_rejected
        and replacement_nis_threshold(args) <= 0.0
    ):
        raise SystemExit(
            "--replace-kf-nis-rejected requires a positive replacement or "
            "alignment NIS threshold"
        )
    kf_rows = read_kf(args.kf_pos)
    fgo_windows = [read_fgo(path) for path in args.fgo_csv]
    fused, summary = fuse_windows(kf_rows, fgo_windows, args)
    write_pos(args.output_pos, fused)
    if args.summary_json:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
