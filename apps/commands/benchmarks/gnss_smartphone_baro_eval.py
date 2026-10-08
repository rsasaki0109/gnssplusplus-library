#!/usr/bin/env python3
"""Score standalone smartphone positions against a weak (Awinda) reference.

The Nantes Awinda trajectory is a body-suit reference georeferenced in
post-processing: its accuracy is not documented and its time stamps were
assigned afterwards.  This scorer therefore

* aligns by GPST time of week with an explicit, per-run, documented lag,
* reports horizontal and vertical error three ways for the height (raw,
  per-method median removed, common offset removed) because the reference
  altitude datum is not the GNSS ellipsoid,
* reports availability, jump and level-split diagnostics, and
* never reads the reference when writing positions (it is evaluation only).
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import os
from pathlib import Path

EARTH_RADIUS_M = 6378137.0
GPS_UNIX_OFFSET_S = 315_964_800.0
WEEK_S = 604800.0


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - index) + ordered[upper] * (index - lower)


def rmse(values: list[float]) -> float | None:
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else None


def median(values: list[float]) -> float | None:
    return percentile(values, 0.5)


def enu_xy(lat_deg: float, lon_deg: float, lat0_deg: float, lon0_deg: float) -> tuple[float, float]:
    east = math.radians(lon_deg - lon0_deg) * EARTH_RADIUS_M * math.cos(math.radians(lat0_deg))
    north = math.radians(lat_deg - lat0_deg) * EARTH_RADIUS_M
    return east, north


def read_awinda(path: Path) -> list[tuple[float, float, float, float]]:
    rows = []
    with path.open(encoding="utf-8", newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        if [h.strip() for h in header] != ["Awinda_TOW", "Awinda_lat", "Awinda_lon", "Awinda_alt"]:
            raise SystemExit(f"unexpected Awinda header: {header}")
        last = None
        for number, row in enumerate(reader, start=2):
            values = tuple(float(v) for v in row)
            if not all(math.isfinite(v) for v in values):
                raise SystemExit(f"non-finite Awinda row {number}")
            if last is not None and values[0] <= last:
                raise SystemExit(f"Awinda TOW not strictly increasing at row {number}")
            last = values[0]
            rows.append(values)
    if not rows:
        raise SystemExit("empty Awinda reference")
    return rows


class Reference:
    def __init__(self, rows: list[tuple[float, float, float, float]], lag_s: float = 0.0):
        self.rows = rows
        self.lag_s = lag_s
        self.times = [r[0] for r in rows]
        self.lat0 = rows[0][1]
        self.lon0 = rows[0][2]
        self.xy = [enu_xy(r[1], r[2], self.lat0, self.lon0) for r in rows]
        self.alt = [r[3] for r in rows]

    @property
    def span(self) -> tuple[float, float]:
        return self.times[0] - self.lag_s, self.times[-1] - self.lag_s

    def at(self, tow: float) -> tuple[float, float, float] | None:
        """Reference (east, north, alt) for the *phone* GPST time ``tow``.

        ``lag_s`` is the Awinda timestamp minus true GPST: the phone epoch
        ``tow`` corresponds to Awinda time ``tow + lag_s``.
        """

        t = tow + self.lag_s
        if t < self.times[0] or t > self.times[-1]:
            return None
        i = bisect.bisect_left(self.times, t)
        if i == 0:
            return (*self.xy[0], self.alt[0])
        t0, t1 = self.times[i - 1], self.times[i]
        w = (t - t0) / (t1 - t0)
        x = self.xy[i - 1][0] * (1 - w) + self.xy[i][0] * w
        y = self.xy[i - 1][1] * (1 - w) + self.xy[i][1] * w
        a = self.alt[i - 1] * (1 - w) + self.alt[i] * w
        return x, y, a


def read_pos(path: Path) -> list[tuple[float, float, float, float, int]]:
    """Return (tow, lat, lon, height, num_sat) for valid solution rows."""

    rows = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if line.startswith("%") or not line.strip():
                continue
            f = line.split()
            rows.append((float(f[1]), float(f[5]), float(f[6]), float(f[7]), int(f[9])))
    return rows


def read_fix(path: Path, utc_to_gpst_offset_ms: float) -> list[tuple[float, float, float, float, int]]:
    """Android Fix.csv comparator mapped to GPST with the adapter clock offset."""

    rows = []
    with path.open(encoding="utf-8", newline="") as handle:
        for fields in csv.reader(handle):
            if not fields:
                continue
            if fields[0] != "Fix" or len(fields) != 14:
                raise SystemExit("unexpected Fix.csv row")
            utc_ms = float(fields[8])
            gpst = (utc_ms + utc_to_gpst_offset_ms) / 1000.0
            rows.append((gpst % WEEK_S, float(fields[2]), float(fields[3]), float(fields[4]), 0))
    return rows


def read_epoch_times(rinex_path: Path) -> list[float]:
    """GPST TOW of every epoch in a RINEX 3 observation file."""

    out = []
    with rinex_path.open(encoding="ascii") as handle:
        for line in handle:
            if line.startswith(">"):
                f = line[1:].split()
                y, mo, d, h, mi = int(f[0]), int(f[1]), int(f[2]), int(f[3]), int(f[4])
                s = float(f[5])
                # days since GPS epoch via ordinal arithmetic
                import datetime as _dt

                days = (_dt.date(y, mo, d) - _dt.date(1980, 1, 6)).days
                total = days * 86400.0 + h * 3600.0 + mi * 60.0 + s
                out.append(total % WEEK_S)
    return out


def match_epochs(
    solution: list[tuple[float, float, float, float, int]],
    reference: Reference,
) -> list[dict]:
    matched = []
    for tow, lat, lon, h, _nsat in solution:
        ref = reference.at(tow)
        if ref is None:
            continue
        east, north = enu_xy(lat, lon, reference.lat0, reference.lon0)
        matched.append(
            {
                "tow": tow,
                "dh": math.hypot(east - ref[0], north - ref[1]),
                "dv_raw": h - ref[2],
                "ref_alt": ref[2],
                "h": h,
                "x": east,
                "y": north,
            }
        )
    return matched


def _stats(vals: list[float]) -> dict:
    return {
        "rmse": rmse(vals),
        "p50": percentile([abs(v) for v in vals], 0.5),
        "p95": percentile([abs(v) for v in vals], 0.95),
        "max": max(abs(v) for v in vals),
        "mean": sum(vals) / len(vals),
    }


def summarize(
    runs: list[list[dict]],
    window_epochs: list[int | None] | None = None,
    common_offsets: list[float | None] | None = None,
    start_alts: list[float] | None = None,
    level_split_m: float | None = None,
) -> dict:
    """Summarise one or several runs; several runs are pooled epoch-wise.

    Vertical error is reported three ways because the reference altitude datum
    is not the GNSS ellipsoid: raw, per-run median removed (relative height
    dispersion) and per-run common offset removed (offset taken from the OFF
    baseline so ON/OFF bias differences stay visible).
    """

    h_all: list[float] = []
    v_raw: list[float] = []
    v_dm: list[float] = []
    v_c: list[float] = []
    steps_h: list[float] = []
    steps_v: list[float] = []
    levels: dict[str, dict[str, list[float]]] = {
        "ground": {"h": [], "v": []},
        "upper": {"h": [], "v": []},
    }
    medians = []
    n_matched = 0
    n_window = 0
    have_window = window_epochs is not None and all(w for w in window_epochs)
    for idx, matched in enumerate(runs):
        if not matched:
            continue
        n_matched += len(matched)
        if have_window:
            n_window += window_epochs[idx]
        h_all += [m["dh"] for m in matched]
        raw = [m["dv_raw"] for m in matched]
        v_raw += raw
        med = median(raw)
        medians.append(med)
        v_dm += [v - med for v in raw]
        if common_offsets is not None and common_offsets[idx] is not None:
            v_c += [v - common_offsets[idx] for v in raw]
        for a, b in zip(matched, matched[1:]):
            dt = b["tow"] - a["tow"]
            if 0.0 < dt <= 2.5:
                steps_h.append(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) / dt)
                steps_v.append(abs(b["h"] - a["h"]) / dt)
        if level_split_m is not None and start_alts is not None:
            for m in matched:
                key = "ground" if m["ref_alt"] < start_alts[idx] + level_split_m else "upper"
                levels[key]["h"].append(m["dh"])
                levels[key]["v"].append(m["dv_raw"] - med)
    out: dict = {"matched_epochs": n_matched}
    if have_window:
        out["window_epochs"] = n_window
        out["availability"] = n_matched / n_window if n_window else None
    if not h_all:
        return out
    out["horizontal_m"] = _stats(h_all)
    out["vertical_raw_m"] = _stats(v_raw)
    out["vertical_demeaned_m"] = _stats(v_dm)
    if v_c:
        out["vertical_common_offset_m"] = _stats(v_c)
    out["per_run_vertical_median_offset_m"] = medians
    if steps_h:
        out["max_horizontal_step_mps"] = max(steps_h)
        out["max_vertical_step_mps"] = max(steps_v)
        out["horizontal_steps_over_10mps"] = sum(1 for x in steps_h if x > 10.0)
        out["vertical_steps_over_10mps"] = sum(1 for x in steps_v if x > 10.0)
    for name, vals in levels.items():
        if len(vals["h"]) >= 10:
            out[f"level_{name}"] = {
                "epochs": len(vals["h"]),
                "horizontal_rmse_m": rmse(vals["h"]),
                "horizontal_p95_m": percentile(vals["h"], 0.95),
                "vertical_demeaned_rmse_m": rmse(vals["v"]),
                "vertical_demeaned_p95_m": percentile([abs(v) for v in vals["v"]], 0.95),
            }
    return out


def score(
    solution: list[tuple[float, float, float, float, int]],
    reference: Reference,
    epoch_times: list[float] | None = None,
    common_height_offset: float | None = None,
    level_split_m: float | None = None,
    start_alt: float | None = None,
) -> dict:
    ref_lo, ref_hi = reference.span
    matched = match_epochs(solution, reference)
    window = None
    if epoch_times is not None:
        window = [sum(1 for t in epoch_times if ref_lo <= t <= ref_hi)]
    out = summarize(
        [matched],
        window,
        [common_height_offset],
        [start_alt] if start_alt is not None else None,
        level_split_m,
    )
    out["reference_span_gpst_tow"] = [ref_lo, ref_hi]
    return out


def closure(
    solution: list[tuple[float, float, float, float, int]], window_s: float = 30.0
) -> dict | None:
    """Start/end same-point closure that needs no reference.

    The Nantes protocol starts and ends every S3/S4 run at the same point
    (reference closure < 1.1 m).  The mean position of the first and last
    ``window_s`` seconds of a solution should therefore coincide; their
    difference is a reference-free consistency measure.
    """

    if len(solution) < 20:
        return None
    t0, t1 = solution[0][0], solution[-1][0]
    first = [r for r in solution if r[0] <= t0 + window_s]
    last = [r for r in solution if r[0] >= t1 - window_s]
    if len(first) < 5 or len(last) < 5:
        return None
    lat0, lon0 = first[0][1], first[0][2]

    def mean_xyz(rows):
        xs = [enu_xy(r[1], r[2], lat0, lon0) for r in rows]
        return (
            sum(x for x, _ in xs) / len(xs),
            sum(y for _, y in xs) / len(xs),
            sum(r[3] for r in rows) / len(rows),
        )

    a, b = mean_xyz(first), mean_xyz(last)
    return {
        "window_s": window_s,
        "horizontal_m": math.hypot(b[0] - a[0], b[1] - a[1]),
        "vertical_m": b[2] - a[2],
    }


def lag_curve(
    solution: list[tuple[float, float, float, float, int]],
    rows: list[tuple[float, float, float, float]],
    lags: list[float],
) -> list[tuple[float, float, int]]:
    """Median horizontal error vs reference lag (used to document alignment)."""

    out = []
    for lag in lags:
        ref = Reference(rows, lag)
        errs = []
        for tow, lat, lon, _h, _n in solution:
            r = ref.at(tow)
            if r is None:
                continue
            e, n = enu_xy(lat, lon, ref.lat0, ref.lon0)
            errs.append(math.hypot(e - r[0], n - r[1]))
        out.append((lag, median(errs) if errs else float("nan"), len(errs)))
    return out


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    p.add_argument("--pos", type=Path, required=True)
    p.add_argument("--awinda", type=Path, required=True)
    p.add_argument("--lag-s", type=float, required=True)
    p.add_argument("--rover-obs", type=Path)
    p.add_argument("--common-height-offset", type=float)
    p.add_argument("--level-split-m", type=float, default=3.0)
    p.add_argument("--output-json", type=Path, required=True)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    rows = read_awinda(args.awinda)
    ref = Reference(rows, args.lag_s)
    sol = read_pos(args.pos)
    epochs = read_epoch_times(args.rover_obs) if args.rover_obs else None
    result = score(
        sol,
        ref,
        epochs,
        args.common_height_offset,
        args.level_split_m,
        rows[0][3],
    )
    result["lag_s"] = args.lag_s
    args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
