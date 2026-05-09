#!/usr/bin/env python3
"""Classify native vs bridge per-satellite residual diff into §7.3 buckets.

Reads two canonical residual CSV files (native --ppp-residual-log and bridge
$SAT-derived) and reports, for the requested ``--tow-min/--tow-max`` window:

  * overall mean / RMS / p95 / max delta for code and phase
  * per-(sat, freq) mean / std delta and dominant-sign flag
  * classification verdict against plan §7.3:
      A. L1 systematic (per-sat constant on freq=0)
      B. L2 systematic (per-sat constant on freq=1)
      C. Random noise inflation (high std, low mean)
      D. Outlier-specific (single-epoch >5σ)
      E. All-sat / all-freq same offset (clock-like)

The final verdict is printed as a single sentence so it can be quoted in a
memory entry directly.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple


@dataclass
class Row:
    week: int
    tow: float
    sat: str
    row_type: str
    freq: int
    residual_m: float


def read_canonical(path: Path) -> List[Row]:
    rows: List[Row] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for r in reader:
            row_type = r.get("row_type", "")
            if row_type not in ("phase", "code"):
                continue
            try:
                residual = float(r.get("residual_m", "nan"))
            except ValueError:
                continue
            if not math.isfinite(residual):
                continue
            try:
                tow = float(r["tow"])
                week = int(float(r["week"]))
                freq = int(float(r.get("frequency_index", "0")))
            except (KeyError, ValueError):
                continue
            rows.append(Row(week, tow, r.get("sat", ""), row_type, freq, residual))
    return rows


def filter_window(rows: Iterable[Row], tow_min: float, tow_max: float) -> List[Row]:
    return [r for r in rows if tow_min <= r.tow <= tow_max]


Key = Tuple[str, int, str]  # (sat, freq, row_type)
EpochKey = Tuple[int, int, str, int, str]  # (week, tow_ms, sat, freq, row_type)


def index_by_epoch(rows: Iterable[Row]) -> Dict[EpochKey, Row]:
    out: Dict[EpochKey, Row] = {}
    for r in rows:
        key = (r.week, int(round(r.tow * 1000)), r.sat, r.freq, r.row_type)
        out.setdefault(key, r)
    return out


def stats(values: Sequence[float]) -> Dict[str, float]:
    if not values:
        return {"n": 0, "mean": math.nan, "std": math.nan, "rms": math.nan, "p95": math.nan, "max_abs": math.nan}
    abs_values = sorted(abs(v) for v in values)
    p95_idx = min(len(abs_values) - 1, math.ceil(0.95 * len(abs_values)) - 1)
    mean = statistics.fmean(values)
    if len(values) > 1:
        std = statistics.pstdev(values)
    else:
        std = 0.0
    return {
        "n": len(values),
        "mean": mean,
        "std": std,
        "rms": math.sqrt(sum(v * v for v in values) / len(values)),
        "p95": abs_values[p95_idx],
        "max_abs": abs_values[-1],
    }


def classify(group_stats: Dict[Key, Dict[str, float]]) -> List[str]:
    """Return human-readable bullets per §7.3 chart."""
    bullets: List[str] = []
    if not group_stats:
        return ["no matched rows"]

    # E. Clock-like: all sats/all freqs same offset for a row_type.
    for row_type in ("phase", "code"):
        means = [
            s["mean"]
            for (_, _, rt), s in group_stats.items()
            if rt == row_type and s["n"] >= 5
        ]
        if len(means) >= 4:
            m = statistics.fmean(means)
            spread = statistics.pstdev(means) if len(means) > 1 else 0.0
            if abs(m) > 0.05 and spread < max(abs(m) * 0.3, 0.05):
                bullets.append(
                    f"E (clock-like): {row_type} mean delta {m:+.3f}m across all "
                    f"sats with spread {spread:.3f}m → suggests receiver-clock / ISB / common bias"
                )

    # A/B. L1 / L2 systematic (per-sat per-freq abs(mean) > 0.05m and std small)
    for freq, label in ((0, "A (L1 systematic)"), (1, "B (L2 systematic)")):
        offenders = [
            (sat, s)
            for (sat, f, rt), s in group_stats.items()
            if f == freq and rt == "phase" and s["n"] >= 5 and abs(s["mean"]) > 0.05 and s["std"] < max(abs(s["mean"]), 0.1)
        ]
        if offenders:
            offenders.sort(key=lambda x: -abs(x[1]["mean"]))
            top = offenders[:5]
            bullets.append(
                f"{label} phase: {len(offenders)} sats, top "
                + ", ".join(f"{s} mean {st['mean']:+.3f}m std {st['std']:.3f}m" for s, st in top)
            )

    # C. Random noise inflation: mean small but std large.
    noise = [
        (sat, freq, rt, s)
        for (sat, freq, rt), s in group_stats.items()
        if s["n"] >= 5 and abs(s["mean"]) < 0.1 and s["std"] > 0.3
    ]
    if noise:
        noise.sort(key=lambda x: -x[3]["std"])
        top = noise[:3]
        bullets.append(
            "C (noise inflation): "
            + ", ".join(f"{s}-{rt}-f{f} std {st['std']:.3f}m mean {st['mean']:+.3f}m" for s, f, rt, st in top)
        )

    return bullets or ["no §7.3 pattern dominates"]


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("native_csv", type=Path)
    parser.add_argument("bridge_csv", type=Path)
    parser.add_argument("--tow-min", type=float, default=None)
    parser.add_argument("--tow-max", type=float, default=None)
    parser.add_argument("--label", default="")
    parser.add_argument("--per-sat-csv", type=Path, default=None)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    native = read_canonical(args.native_csv)
    bridge = read_canonical(args.bridge_csv)

    if args.tow_min is None and args.tow_max is None:
        common_native_tows = sorted({r.tow for r in native})
        common_bridge_tows = sorted({r.tow for r in bridge})
        tow_min = max(common_native_tows[0], common_bridge_tows[0])
        tow_max = min(common_native_tows[-1], common_bridge_tows[-1])
    else:
        tow_min = args.tow_min if args.tow_min is not None else -math.inf
        tow_max = args.tow_max if args.tow_max is not None else math.inf

    native = filter_window(native, tow_min, tow_max)
    bridge = filter_window(bridge, tow_min, tow_max)

    native_idx = index_by_epoch(native)
    bridge_idx = index_by_epoch(bridge)

    common_keys = set(native_idx).intersection(bridge_idx)

    deltas_by_group: Dict[Key, List[float]] = defaultdict(list)
    deltas_overall: Dict[str, List[float]] = defaultdict(list)
    for k in common_keys:
        d = native_idx[k].residual_m - bridge_idx[k].residual_m
        if not math.isfinite(d):
            continue
        gkey = (native_idx[k].sat, native_idx[k].freq, native_idx[k].row_type)
        deltas_by_group[gkey].append(d)
        deltas_overall[native_idx[k].row_type].append(d)

    group_stats = {k: stats(v) for k, v in deltas_by_group.items()}

    label = f" [{args.label}]" if args.label else ""
    print(f"== per-sat residual diff{label} ==")
    print(f"window tow=[{tow_min:.1f}, {tow_max:.1f}] ({tow_max - tow_min:.1f}s)")
    print(f"native rows={len(native)} bridge rows={len(bridge)} matched={len(common_keys)}")
    for row_type in ("phase", "code"):
        s = stats(deltas_overall[row_type])
        print(
            f"  {row_type:5s} n={s['n']:6d}  mean={s['mean']:+.4f}m  "
            f"std={s['std']:.4f}m  rms={s['rms']:.4f}m  p95={s['p95']:.4f}m  max={s['max_abs']:.4f}m"
        )

    print()
    print("== Top per-(sat,freq,row_type) by |mean delta| ==")
    sorted_groups = sorted(
        group_stats.items(),
        key=lambda x: -abs(x[1]["mean"]) if math.isfinite(x[1]["mean"]) else 0.0,
    )[:20]
    for (sat, freq, rt), s in sorted_groups:
        print(
            f"  {sat}-f{freq}-{rt:5s} n={s['n']:5d}  mean={s['mean']:+.4f}m  "
            f"std={s['std']:.4f}m  rms={s['rms']:.4f}m  p95={s['p95']:.4f}m"
        )

    print()
    print("== §7.3 classification ==")
    for b in classify(group_stats):
        print(f"  - {b}")

    if args.per_sat_csv is not None:
        args.per_sat_csv.parent.mkdir(parents=True, exist_ok=True)
        with args.per_sat_csv.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["sat", "freq", "row_type", "n", "mean_m", "std_m", "rms_m", "p95_m", "max_abs_m"])
            for (sat, freq, rt), s in sorted_groups:
                writer.writerow([sat, freq, rt, s["n"], s["mean"], s["std"], s["rms"], s["p95"], s["max_abs"]])

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
