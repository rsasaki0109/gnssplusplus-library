#!/usr/bin/env python3
"""Truth-based honesty check of the per-epoch uncertainty in a libgnss++ .pos.

Reads a .pos written by ``gnss solve --pos-uncertainty`` (columns sdn/sde/sdu
in metres, 1-sigma, local ENU), matches the epochs to a reference trajectory
(PPC / UrbanNav ``reference.csv``), and asks whether the reported sigma
describes the observed error: normalized-error quantiles, exceedance
fractions against the Gaussian expectation, and Stanford-diagram bin counts.

IMPORTANT: the bound used in the Stanford diagram is ``K * sigma``.  It is a
"protection level proxy" for diagnostics only.  It is NOT a certified or
integrity-monitored protection level and must not be used as one.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import os
from pathlib import Path
import sys
from typing import Any, Sequence

from support.gnss_runtime import application_root

ROOT_DIR = application_root(__file__)

DISCLAIMER = (
    "K*sigma is a diagnostic protection-level PROXY. It is NOT a certified "
    "protection level: no fault detection, no integrity risk allocation, no "
    "ambiguity-fault model is applied."
)

FIXED_STATUSES = (4, 6)
FLOAT_STATUSES = (3, 5)
EXCEEDANCE_KS = (1.0, 2.0, 3.0, 5.0)
DEFAULT_K = 5.33
DEFAULT_BIN_EDGES_M = (0.0, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0)
REQUIRED_COLUMNS = ("sdn(m)", "sde(m)", "sdu(m)")
OPTIONAL_COLUMNS = ("sdne(m)", "sdeu(m)", "sdun(m)", "DiffAge(s)")
GROUPS = ("FIXED", "FLOAT", "ALL")

# Verdict thresholds (indicative, documented in docs/interfaces.md).
OVERCONFIDENT_P95_RATIO = 1.5
CONSERVATIVE_P95_RATIO = 0.5
HEAVY_TAIL_FACTOR = 10.0
HEAVY_TAIL_FLOOR = 0.01  # observed P(>3 sigma) must exceed this as well.
FAR_TAIL_MIN_COUNT = 3  # >= this many epochs beyond 5 sigma (Gaussian expects ~0).


# ---------------------------------------------------------------------------
# Gaussian reference values
# ---------------------------------------------------------------------------
def gaussian_exceedance_h(k: float) -> float:
    """P(e_H > k*sigma_H) for isotropic N(0, sigma_axis^2) with sigma_H^2 = 2*sigma_axis^2."""
    return math.exp(-k * k)


def gaussian_exceedance_v(k: float) -> float:
    """Two-sided P(|e_U| > k*sigma_U) for N(0, sigma_U^2)."""
    return math.erfc(k / math.sqrt(2.0))


def gaussian_quantile_h(p: float) -> float:
    """Quantile of e_H/sigma_H under the isotropic assumption (Rayleigh)."""
    return math.sqrt(-math.log(1.0 - p))


def gaussian_quantile_v(p: float) -> float:
    """Quantile of |e_U|/sigma_U under N(0,1): solve erfc(q/sqrt2) = 1-p."""
    target = 1.0 - p
    lo, hi = 0.0, 12.0
    for _ in range(100):
        mid = 0.5 * (lo + hi)
        if math.erfc(mid / math.sqrt(2.0)) > target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def percentile(values: Sequence[float], percent: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * percent / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def rounded(value: float | None, digits: int = 6) -> float | None:
    if value is None or not math.isfinite(value):
        return None
    return round(value, digits)


# ---------------------------------------------------------------------------
# .pos reader (header driven)
# ---------------------------------------------------------------------------
def parse_pos_header(line: str) -> list[str] | None:
    """Return column names for a ``% Columns:`` or ``% GPS_Week ...`` header."""
    if not line.startswith("%"):
        return None
    body = line[1:].strip()
    if body.startswith("Columns:"):
        body = body[len("Columns:"):]
    tokens = body.split()
    if tokens and tokens[0] == "GPS_Week":
        return tokens
    return None


def _to_float(token: str) -> float:
    try:
        return float(token)
    except ValueError:
        return math.nan


def read_pos_uncertainty(path: Path) -> list[dict[str, float]]:
    """Read libgnss++ .pos rows including the --pos-uncertainty columns."""
    columns: list[str] | None = None
    rows: list[dict[str, float]] = []
    with path.open(encoding="ascii", errors="ignore") as handle:
        for raw in handle:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("%"):
                parsed = parse_pos_header(line)
                if parsed is not None:
                    columns = parsed
                continue
            parts = line.split()
            if len(parts) < 10:
                continue
            if columns is None:
                raise SystemExit(
                    f"{path}: no '% GPS_Week ...' column header; re-run gnss solve with --pos-uncertainty"
                )
            missing = [name for name in REQUIRED_COLUMNS if name not in columns]
            if missing:
                raise SystemExit(
                    f"{path}: missing uncertainty columns {', '.join(missing)}; "
                    "re-run gnss solve with --pos-uncertainty"
                )
            row = {
                "week": float(parts[0]),
                "tow": float(parts[1]),
                "x": float(parts[2]),
                "y": float(parts[3]),
                "z": float(parts[4]),
                "status": float(parts[8]),
            }
            for name in REQUIRED_COLUMNS + OPTIONAL_COLUMNS:
                index = columns.index(name) if name in columns else -1
                row[name] = _to_float(parts[index]) if 0 <= index < len(parts) else math.nan
            rows.append(row)
    return rows


# ---------------------------------------------------------------------------
# Reference matching
# ---------------------------------------------------------------------------
def _comparison_module():
    scripts_dir = ROOT_DIR / "scripts"
    if str(scripts_dir) not in sys.path:
        sys.path.insert(0, str(scripts_dir))
    import generate_driving_comparison as comparison

    return comparison


def load_reference(path: Path) -> list[Any]:
    """PPC / UrbanNav reference.csv via the shared scripts loader."""
    return _comparison_module().read_reference_csv(path)


def match_epochs(
    rows: Sequence[dict[str, float]],
    reference: Sequence[Any],
    tolerance_s: float,
) -> tuple[list[dict[str, float]], int]:
    """Nearest-reference matching with generate_driving_comparison conventions.

    Same week and ``|dt| <= tolerance_s``; ENU error is expressed in the local
    frame at the matched reference epoch.  Returns (matched, unmatched_count).
    """
    import numpy as np

    comparison = _comparison_module()
    ref_tows = [epoch.tow for epoch in reference]
    matched: list[dict[str, float]] = []
    unmatched = 0
    for row in rows:
        idx = bisect.bisect_left(ref_tows, row["tow"])
        candidates = [reference[j] for j in (idx - 1, idx, idx + 1) if 0 <= j < len(reference)]
        if not candidates:
            unmatched += 1
            continue
        ref = min(candidates, key=lambda item: abs(item.tow - row["tow"]))
        if ref.week != int(row["week"]) or abs(ref.tow - row["tow"]) > tolerance_s:
            unmatched += 1
            continue
        enu = comparison.ecef_to_enu(
            np.array([row["x"], row["y"], row["z"]]) - ref.ecef, ref.lat_deg, ref.lon_deg
        )
        out = dict(row)
        out["e_east"] = float(enu[0])
        out["e_north"] = float(enu[1])
        out["e_up"] = float(enu[2])
        matched.append(out)
    return matched, unmatched


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------
def _quantile_block(values: Sequence[float], gauss) -> dict[str, Any]:
    return {
        "median": rounded(percentile(values, 50.0)),
        "p95": rounded(percentile(values, 95.0)),
        "p99": rounded(percentile(values, 99.0)),
        "gaussian_median": rounded(gauss(0.5)),
        "gaussian_p95": rounded(gauss(0.95)),
        "gaussian_p99": rounded(gauss(0.99)),
    }


def stanford_bins(
    errors: Sequence[float],
    bounds: Sequence[float],
    edges: Sequence[float],
) -> list[list[int]]:
    """counts[bound_bin][error_bin]; the last bin of each axis is open-ended."""
    n_bins = len(edges)
    counts = [[0] * n_bins for _ in range(n_bins)]

    def bin_of(value: float) -> int:
        return max(0, min(n_bins - 1, bisect.bisect_right(edges, value) - 1))

    for error, bound in zip(errors, bounds):
        counts[bin_of(bound)][bin_of(error)] += 1
    return counts


def stanford_regions(
    errors: Sequence[float],
    bounds: Sequence[float],
    alert_limit: float | None,
) -> dict[str, int]:
    nominal = misleading = unavailable = hazardous = 0
    for error, bound in zip(errors, bounds):
        if alert_limit is not None and bound > alert_limit:
            unavailable += 1
        elif error > bound:
            misleading += 1
            if alert_limit is not None and error > alert_limit:
                hazardous += 1
        else:
            nominal += 1
    out = {"nominal": nominal, "misleading": misleading, "unavailable": unavailable}
    if alert_limit is not None:
        out["hazardously_misleading"] = hazardous
    return out


def _axis_verdict(values: Sequence[float], gauss_p95: float, exceed3: float, gauss_exceed3: float,
                  count5: int, gauss_exceed5: float, min_samples: int) -> str:
    if len(values) < min_samples:
        return "insufficient_data"
    p95 = percentile(values, 95.0)
    ratio = p95 / gauss_p95 if gauss_p95 > 0 else math.inf
    heavy_tail = exceed3 > max(HEAVY_TAIL_FACTOR * gauss_exceed3, HEAVY_TAIL_FLOOR) or (
        count5 >= FAR_TAIL_MIN_COUNT and count5 > HEAVY_TAIL_FACTOR * gauss_exceed5 * len(values)
    )
    if ratio > OVERCONFIDENT_P95_RATIO or heavy_tail:
        return "overconfident"
    if ratio < CONSERVATIVE_P95_RATIO:
        return "over_conservative"
    return "consistent"


def summarize_samples(
    horizontal_error: Sequence[float],
    vertical_error: Sequence[float],
    sigma_h: Sequence[float],
    sigma_v: Sequence[float],
    *,
    age_s: Sequence[float] | None = None,
    k_h: float = DEFAULT_K,
    k_v: float = DEFAULT_K,
    alert_limit_h: float | None = None,
    alert_limit_v: float | None = None,
    bin_edges: Sequence[float] = DEFAULT_BIN_EDGES_M,
    min_samples: int = 30,
) -> dict[str, Any]:
    """Summarize one group; ``vertical_error`` is |e_U|, all inputs in metres."""
    n = len(horizontal_error)
    z_h = [e / s for e, s in zip(horizontal_error, sigma_h)]
    z_v = [e / s for e, s in zip(vertical_error, sigma_v)]
    summary: dict[str, Any] = {"n": n}
    if n == 0:
        return summary

    summary["normalized_error"] = {
        "H": _quantile_block(z_h, gaussian_quantile_h),
        "V": _quantile_block(z_v, gaussian_quantile_v),
    }
    exceedance: dict[str, Any] = {}
    for k in EXCEEDANCE_KS:
        count_h = sum(1 for z in z_h if z > k)
        count_v = sum(1 for z in z_v if z > k)
        exceedance[f"{k:g}"] = {
            "H": {
                "count": count_h,
                "fraction": rounded(count_h / n, 8),
                "gaussian": gaussian_exceedance_h(k),
            },
            "V": {
                "count": count_v,
                "fraction": rounded(count_v / n, 8),
                "gaussian": gaussian_exceedance_v(k),
            },
        }
    summary["exceedance"] = exceedance
    summary["gaussian_assumption"] = (
        "H: isotropic, sigma_H^2 = sdn^2+sde^2 = 2*sigma_axis^2 so P(e_H > k*sigma_H) = exp(-k^2); "
        "V: two-sided N(0,1) so P(|e_U| > k*sigma_U) = erfc(k/sqrt(2))"
    )

    edges = list(bin_edges)
    bounds_h = [k_h * s for s in sigma_h]
    bounds_v = [k_v * s for s in sigma_v]
    summary["stanford"] = {
        "label": "K*sigma proxy (NOT a certified protection level)",
        "bin_edges_m": edges,
        "last_bin_open": True,
        "H": {
            "K": k_h,
            "alert_limit_m": alert_limit_h,
            "regions": stanford_regions(horizontal_error, bounds_h, alert_limit_h),
            "counts_bound_by_error": stanford_bins(horizontal_error, bounds_h, edges),
        },
        "V": {
            "K": k_v,
            "alert_limit_m": alert_limit_v,
            "regions": stanford_regions(vertical_error, bounds_v, alert_limit_v),
            "counts_bound_by_error": stanford_bins(vertical_error, bounds_v, edges),
        },
    }
    summary["error_m"] = {
        "H": {
            "median": rounded(percentile(horizontal_error, 50.0)),
            "p95": rounded(percentile(horizontal_error, 95.0)),
            "rms": rounded(math.sqrt(sum(e * e for e in horizontal_error) / n)),
        },
        "V": {
            "median": rounded(percentile(vertical_error, 50.0)),
            "p95": rounded(percentile(vertical_error, 95.0)),
            "rms": rounded(math.sqrt(sum(e * e for e in vertical_error) / n)),
        },
    }
    summary["sigma_m"] = {
        "H_median": rounded(percentile(sigma_h, 50.0)),
        "V_median": rounded(percentile(sigma_v, 50.0)),
    }
    if age_s is not None:
        finite_age = [a for a in age_s if math.isfinite(a)]
        summary["differential_age_s"] = {
            "n_finite": len(finite_age),
            "median": rounded(percentile(finite_age, 50.0)),
            "max": rounded(max(finite_age)) if finite_age else None,
        }
    summary["verdict"] = {
        "H": _axis_verdict(z_h, gaussian_quantile_h(0.95), exceedance["3"]["H"]["fraction"],
                           gaussian_exceedance_h(3.0), exceedance["5"]["H"]["count"],
                           gaussian_exceedance_h(5.0), min_samples),
        "V": _axis_verdict(z_v, gaussian_quantile_v(0.95), exceedance["3"]["V"]["fraction"],
                           gaussian_exceedance_v(3.0), exceedance["5"]["V"]["count"],
                           gaussian_exceedance_v(5.0), min_samples),
        "rule": (
            f"overconfident if p95(z) > {OVERCONFIDENT_P95_RATIO}x Gaussian p95 or "
            f"P(z>3) > max({HEAVY_TAIL_FACTOR:g}x Gaussian, {HEAVY_TAIL_FLOOR}) or "
            f">= {FAR_TAIL_MIN_COUNT} epochs beyond 5 sigma; "
            f"over_conservative if p95(z) < {CONSERVATIVE_P95_RATIO}x Gaussian p95; "
            f"insufficient_data below {min_samples} epochs. Indicative only."
        ),
    }
    return summary


def split_groups(matched: Sequence[dict[str, float]]) -> dict[str, list[dict[str, float]]]:
    return {
        "FIXED": [r for r in matched if int(r["status"]) in FIXED_STATUSES],
        "FLOAT": [r for r in matched if int(r["status"]) in FLOAT_STATUSES],
        "ALL": list(matched),
    }


def sigma_of(row: dict[str, float]) -> tuple[float, float]:
    return math.hypot(row["sdn(m)"], row["sde(m)"]), row["sdu(m)"]


def evaluate(
    matched: Sequence[dict[str, float]],
    *,
    k_h: float = DEFAULT_K,
    k_v: float = DEFAULT_K,
    alert_limit_h: float | None = None,
    alert_limit_v: float | None = None,
    bin_edges: Sequence[float] = DEFAULT_BIN_EDGES_M,
    min_samples: int = 30,
) -> tuple[dict[str, Any], int]:
    """Per-status summaries; returns (groups, rows_without_valid_sigma)."""
    usable: list[dict[str, float]] = []
    no_sigma = 0
    for row in matched:
        sigma_h, sigma_v = sigma_of(row)
        if (math.isfinite(sigma_h) and sigma_h > 0.0 and math.isfinite(sigma_v) and sigma_v > 0.0
                and all(math.isfinite(row[key]) for key in ("e_east", "e_north", "e_up"))):
            usable.append(row)
        else:
            no_sigma += 1
    groups: dict[str, Any] = {}
    for name, members in split_groups(usable).items():
        sigmas = [sigma_of(r) for r in members]
        groups[name] = summarize_samples(
            [math.hypot(r["e_east"], r["e_north"]) for r in members],
            [abs(r["e_up"]) for r in members],
            [s[0] for s in sigmas],
            [s[1] for s in sigmas],
            age_s=[r["DiffAge(s)"] for r in members],
            k_h=k_h,
            k_v=k_v,
            alert_limit_h=alert_limit_h,
            alert_limit_v=alert_limit_v,
            bin_edges=bin_edges,
            min_samples=min_samples,
        )
    return groups, no_sigma


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------
def write_epoch_csv(path: Path, matched: Sequence[dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="ascii") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "week", "tow", "status", "e_east_m", "e_north_m", "e_up_m", "e_h_m",
            "sigma_h_m", "sigma_u_m", "z_h", "z_v", "diff_age_s",
        ])
        for r in matched:
            sigma_h, sigma_v = sigma_of(r)
            e_h = math.hypot(r["e_east"], r["e_north"])
            z_h = e_h / sigma_h if sigma_h > 0 else math.nan
            z_v = abs(r["e_up"]) / sigma_v if sigma_v > 0 else math.nan
            writer.writerow([
                int(r["week"]), f"{r['tow']:.3f}", int(r["status"]),
                f"{r['e_east']:.4f}", f"{r['e_north']:.4f}", f"{r['e_up']:.4f}", f"{e_h:.4f}",
                f"{sigma_h:.5f}", f"{sigma_v:.5f}", f"{z_h:.4f}", f"{z_v:.4f}",
                f"{r['DiffAge(s)']:.3f}",
            ])


def write_stanford_csv(path: Path, groups: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="ascii") as handle:
        writer = csv.writer(handle)
        writer.writerow(["status", "axis", "K", "bound_lo_m", "bound_hi_m", "error_lo_m", "error_hi_m", "count"])
        for name, group in groups.items():
            stanford = group.get("stanford")
            if not stanford:
                continue
            edges = stanford["bin_edges_m"]
            for axis in ("H", "V"):
                counts = stanford[axis]["counts_bound_by_error"]
                for i, row in enumerate(counts):
                    for j, count in enumerate(row):
                        writer.writerow([
                            name, axis, stanford[axis]["K"],
                            edges[i], edges[i + 1] if i + 1 < len(edges) else "",
                            edges[j], edges[j + 1] if j + 1 < len(edges) else "",
                            count,
                        ])


def write_png(path: Path, matched: Sequence[dict[str, float]], k_h: float, k_v: float) -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    figure, axes = plt.subplots(1, 2, figsize=(11, 5))
    for axis, label, k in ((axes[0], "Horizontal", k_h), (axes[1], "Vertical", k_v)):
        errors, bounds = [], []
        for r in matched:
            sigma_h, sigma_v = sigma_of(r)
            if label == "Horizontal":
                errors.append(math.hypot(r["e_east"], r["e_north"]))
                bounds.append(k * sigma_h)
            else:
                errors.append(abs(r["e_up"]))
                bounds.append(k * sigma_v)
        axis.scatter(errors, bounds, s=3, alpha=0.3)
        top = max([1.0] + errors + bounds)
        axis.plot([1e-3, top], [1e-3, top], color="red", lw=1)
        axis.set_xscale("log")
        axis.set_yscale("log")
        axis.set_xlim(1e-3, top)
        axis.set_ylim(1e-3, top)
        axis.set_xlabel(f"{label} error (m)")
        axis.set_ylabel(f"K*sigma proxy bound, K={k:g} (m)")
        axis.set_title(f"{label} Stanford diagram (below red line = misleading)")
    figure.suptitle("K*sigma is NOT a certified protection level")
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(figure)
    return True


def format_text(payload: dict[str, Any]) -> str:
    lines = [
        f"gnss uncertainty-eval: {payload['inputs']['pos']}",
        f"  matched={payload['counts']['matched']} unmatched={payload['counts']['unmatched']} "
        f"no_sigma={payload['counts']['no_sigma']}",
        f"  NOTE: {DISCLAIMER}",
    ]
    for name in GROUPS:
        group = payload["groups"].get(name, {})
        if not group.get("n"):
            lines.append(f"  [{name}] n=0")
            continue
        nz = group["normalized_error"]
        ex = group["exceedance"]
        st = group["stanford"]
        lines.append(
            f"  [{name}] n={group['n']}"
            f"  H z med/p95/p99={nz['H']['median']}/{nz['H']['p95']}/{nz['H']['p99']}"
            f"  V z med/p95/p99={nz['V']['median']}/{nz['V']['p95']}/{nz['V']['p99']}"
        )
        lines.append(
            f"      P(>3sigma) H={ex['3']['H']['fraction']} (gauss {ex['3']['H']['gaussian']:.2e})"
            f"  V={ex['3']['V']['fraction']} (gauss {ex['3']['V']['gaussian']:.2e})"
            f"  misleading H={st['H']['regions']['misleading']} V={st['V']['regions']['misleading']}"
            f"  verdict H={group['verdict']['H']} V={group['verdict']['V']}"
        )
    return "\n".join(lines)


def parse_bin_edges(text: str) -> list[float]:
    edges = [float(token) for token in text.split(",") if token.strip()]
    if len(edges) < 2 or any(b <= a for a, b in zip(edges, edges[1:])):
        raise argparse.ArgumentTypeError("--bin-edges must be >= 2 strictly increasing values")
    return edges


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description=(
            "Check whether the per-epoch uncertainty (sdn/sde/sdu from "
            "'gnss solve --pos-uncertainty') is honest against a truth reference. "
            + DISCLAIMER
        ),
    )
    parser.add_argument("--pos", type=Path, required=True, help="libgnss++ .pos with uncertainty columns.")
    parser.add_argument("--reference-csv", type=Path, required=True,
                        help="PPC / UrbanNav style reference.csv (TOW, week, lat, lon, h, ECEF XYZ ...).")
    parser.add_argument("--match-tolerance-s", type=float, default=0.25,
                        help="Epoch matching tolerance (same default as gnss compare-modes reference metrics).")
    parser.add_argument("--k-h", type=float, default=DEFAULT_K,
                        help="Horizontal K of the K*sigma proxy bound (default 5.33; 6.0 is the other common choice).")
    parser.add_argument("--k-v", type=float, default=DEFAULT_K, help="Vertical K of the proxy bound (default 5.33).")
    parser.add_argument("--alert-limit-h", type=float, help="Horizontal alert limit (m); enables the 'unavailable' region.")
    parser.add_argument("--alert-limit-v", type=float, help="Vertical alert limit (m); enables the 'unavailable' region.")
    parser.add_argument("--bin-edges", type=parse_bin_edges, default=list(DEFAULT_BIN_EDGES_M),
                        help="Comma-separated Stanford bin edges in metres (last bin is open-ended).")
    parser.add_argument("--min-samples", type=int, default=30, help="Minimum epochs for a verdict.")
    parser.add_argument("--out-json", type=Path, help="Write the JSON summary here.")
    parser.add_argument("--out-csv", type=Path, help="Write the per-epoch error/sigma table here.")
    parser.add_argument("--out-stanford-csv", type=Path, help="Write the Stanford bin counts here.")
    parser.add_argument("--out-png", type=Path, help="Optional Stanford PNG (needs matplotlib; skipped if missing).")
    parser.add_argument("--json", action="store_true", help="Print the JSON summary instead of text.")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not args.pos.is_file():
        raise SystemExit(f"Missing .pos file: {args.pos}")
    if not args.reference_csv.is_file():
        raise SystemExit(f"Missing reference CSV: {args.reference_csv}")
    rows = read_pos_uncertainty(args.pos)
    if not rows:
        raise SystemExit(f"No solution epochs in {args.pos}")
    reference = load_reference(args.reference_csv)
    matched, unmatched = match_epochs(rows, reference, args.match_tolerance_s)
    groups, no_sigma = evaluate(
        matched,
        k_h=args.k_h,
        k_v=args.k_v,
        alert_limit_h=args.alert_limit_h,
        alert_limit_v=args.alert_limit_v,
        bin_edges=args.bin_edges,
        min_samples=args.min_samples,
    )
    payload = {
        "tool": "gnss uncertainty-eval",
        "disclaimer": DISCLAIMER,
        "inputs": {"pos": str(args.pos), "reference_csv": str(args.reference_csv)},
        "parameters": {
            "match_tolerance_s": args.match_tolerance_s,
            "k_h": args.k_h,
            "k_v": args.k_v,
            "alert_limit_h_m": args.alert_limit_h,
            "alert_limit_v_m": args.alert_limit_v,
            "sigma_h_definition": "sqrt(sdn^2 + sde^2)",
            "fixed_statuses": list(FIXED_STATUSES),
            "float_statuses": list(FLOAT_STATUSES),
        },
        "counts": {
            "pos_rows": len(rows),
            "matched": len(matched),
            "unmatched": unmatched,
            "no_sigma": no_sigma,
        },
        "groups": groups,
    }
    if args.out_json:
        args.out_json.parent.mkdir(parents=True, exist_ok=True)
        args.out_json.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    if args.out_csv:
        write_epoch_csv(args.out_csv, matched)
    if args.out_stanford_csv:
        write_stanford_csv(args.out_stanford_csv, groups)
    if args.out_png and not write_png(args.out_png, matched, args.k_h, args.k_v):
        print("matplotlib not available; skipping --out-png", file=sys.stderr)
    print(json.dumps(payload, indent=2) if args.json else format_text(payload))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
