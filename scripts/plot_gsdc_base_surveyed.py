#!/usr/bin/env python3
"""GSDC smartphone base-surveyed dev-route summary figure.

Panel (a): per-route (P50+P95)/2, no-base vs base-surveyed.
Panel (b): horizontal-error CDF of the base-surveyed runs.

Predictions are submission CSVs (phone,UnixTimeMillis,LatitudeDegrees,
LongitudeDegrees); truth CSVs carry UnixTimeMillis/LatitudeDegrees/
LongitudeDegrees. Metric: exact (phone, UnixTimeMillis) join, spherical
Haversine R=6371008.8, linear percentile.
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

R = 6371008.8


def haversine(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(min(1.0, math.sqrt(a)))


def read_latlon(path):
    out = {}
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            if not row.get("UnixTimeMillis"):
                continue
            out[int(row["UnixTimeMillis"])] = (
                float(row["LatitudeDegrees"]), float(row["LongitudeDegrees"]))
    return out


def errors(pred, truth):
    keys = sorted(set(pred) & set(truth))
    e = [haversine(*pred[k], *truth[k]) for k in keys]
    return np.sort(np.array(e))


def stats(err):
    p50 = float(np.percentile(err, 50))
    p95 = float(np.percentile(err, 95))
    return p50, p95, (p50 + p95) / 2.0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--routes", nargs="+", required=True,
                    help="name:pred.csv:truth.csv")
    ap.add_argument("--no-base", default="",
                    help="comma list of no-base scores aligned with --routes")
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--dpi", type=int, default=160)
    args = ap.parse_args()

    no_base_scores = [float(x) for x in args.no_base.split(",")] if args.no_base else []
    names, base_scores, cdfs = [], [], []
    for idx, spec in enumerate(args.routes):
        name, pred_path, truth_path = spec.split(":")
        err = errors(read_latlon(pred_path), read_latlon(truth_path))
        _, _, score = stats(err)
        names.append(name)
        base_scores.append(score)
        cdfs.append((err, no_base_scores[idx] if idx < len(no_base_scores) else None))

    fig, (ax_bar, ax_cdf) = plt.subplots(1, 2, figsize=(13, 5.2))
    x = np.arange(len(names))
    w = 0.38
    if no_base_scores:
        ax_bar.bar(x - w / 2, no_base_scores, w, color="#d62728", label="no-base")
    ax_bar.bar(x + (w / 2 if no_base_scores else 0), base_scores, w,
               color="#1a7f37", label="base-surveyed")
    for i, s in enumerate(base_scores):
        ax_bar.text(i + (w / 2 if no_base_scores else 0), s, f"{s:.2f}",
                    ha="center", va="bottom", fontsize=11, fontweight="bold")
    if no_base_scores:
        for i, s in enumerate(no_base_scores):
            ax_bar.text(i - w / 2, s, f"{s:.2f}", ha="center", va="bottom",
                        fontsize=11)
    ax_bar.axhline(1.0, color="#999999", ls="--", lw=1)
    ax_bar.set_xticks(x, names)
    ax_bar.set_ylabel("(P50+P95)/2  [m]")
    ax_bar.set_title("GSDC dev routes (Pixel5)")
    ax_bar.legend(fontsize=11)
    ax_bar.grid(axis="y", alpha=0.3)

    for (err, _), name in zip(cdfs, names):
        ax_cdf.plot(err, np.arange(1, err.size + 1) / err.size, lw=2.0, label=name)
    ax_cdf.axvline(1.0, color="#999999", ls="--", lw=1)
    ax_cdf.set_xlim(0, 2.0)
    ax_cdf.set_xlabel("horizontal error [m]")
    ax_cdf.set_ylabel("fraction of epochs")
    ax_cdf.set_title("Base-surveyed horizontal-error CDF")
    ax_cdf.grid(alpha=0.3)
    ax_cdf.legend(fontsize=10, loc="lower right")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(args.output, dpi=args.dpi)
    print(f"wrote {args.output}")
    for name, s in zip(names, base_scores):
        print(f"  {name}: base-surveyed {s:.4f} m")


if __name__ == "__main__":
    raise SystemExit(main())
