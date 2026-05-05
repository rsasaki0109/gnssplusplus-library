#!/usr/bin/env python3
"""Compare native PPP state log against MADOCALIB bridge .stat output.

Both inputs use the MADOCALIB record format (`$POS,week,tow,stat,X,Y,Z,sigX,sigY,sigZ` etc.).
The script aligns rows by (week, tow, tag, key) where `key` is satellite/freq for
$ION/$AMB and absent for $POS/$CLK/$TROP/$ISB.

Output: per-tag summary statistics + per-epoch worst diff for each tag, plus an
optional per-row CSV for downstream plotting.

Usage:
    ppp_state_log_diff.py NATIVE.stat BRIDGE.stat \\
        [--start-tow SEC] [--end-tow SEC] [--out-csv PATH]
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple


@dataclass
class Row:
    week: int
    tow: float
    stat: int
    tag: str
    key: str  # "" for tags without per-sat key
    values: List[float]  # numeric payload (interpretation depends on tag)
    raw_fields: List[str]


# Column slices (after week/tow/stat) used to extract numeric payload.
# For $ION, $AMB the per-sat key is field index 4 (sat name) and (for AMB) field index 5 (freq idx).
TAG_LAYOUTS: Dict[str, Tuple[List[int], List[str]]] = {
    # tag -> (numeric_payload_indices, value_labels)
    "$POS":  ([4, 5, 6, 7, 8, 9],            ["X_m", "Y_m", "Z_m", "sigX_m", "sigY_m", "sigZ_m"]),
    "$CLK":  ([5, 6, 7, 8],                  ["c1_ns", "c2_ns", "sig1_ns", "sig2_ns"]),
    "$TROP": ([5, 6],                        ["ztd_m", "sig_m"]),
    "$ISB":  ([5, 6],                        ["isb_ns", "sig_ns"]),
    "$ION":  ([5, 6, 7, 8],                  ["az_deg", "el_deg", "iono_m", "sig_m"]),
    "$AMB":  ([6, 7],                        ["amb_m", "sig_m"]),
}


def parse_stat_file(path: Path) -> List[Row]:
    rows: List[Row] = []
    with path.open() as fh:
        for line in fh:
            line = line.strip()
            if not line or not line.startswith("$"):
                continue
            fields = line.split(",")
            tag = fields[0]
            if tag not in TAG_LAYOUTS:
                continue
            try:
                week = int(fields[1])
                tow = float(fields[2])
                stat = int(fields[3])
            except (ValueError, IndexError):
                continue
            indices, _ = TAG_LAYOUTS[tag]
            try:
                values = [float(fields[i]) for i in indices]
            except (ValueError, IndexError):
                continue
            if tag == "$POS":
                key = ""
            elif tag == "$CLK":
                key = ""
            elif tag == "$TROP":
                key = ""  # MADOCALIB writes per-grad; we use a single TROP. field[4]=='1' for both.
            elif tag == "$ISB":
                key = fields[4] if len(fields) > 4 else ""  # system label (native only)
            elif tag == "$ION":
                key = fields[4] if len(fields) > 4 else ""  # sat id
            elif tag == "$AMB":
                sat = fields[4] if len(fields) > 4 else ""
                freq = fields[5] if len(fields) > 5 else ""
                key = f"{sat}:{freq}"
            else:
                key = ""
            rows.append(Row(week=week, tow=tow, stat=stat, tag=tag, key=key,
                            values=values, raw_fields=fields))
    return rows


def index_rows(rows: List[Row]) -> Dict[Tuple[int, float, str, str], Row]:
    out: Dict[Tuple[int, float, str, str], Row] = {}
    for r in rows:
        out[(r.week, round(r.tow, 3), r.tag, r.key)] = r
    return out


def rms(values: List[float]) -> float:
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def main(argv: List[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("native_stat", type=Path, help="Native --ppp-state-log output")
    ap.add_argument("bridge_stat", type=Path, help="MADOCALIB bridge .stat output")
    ap.add_argument("--start-tow", type=float, default=None,
                    help="Restrict to rows with TOW >= this (seconds of week)")
    ap.add_argument("--end-tow", type=float, default=None,
                    help="Restrict to rows with TOW <= this")
    ap.add_argument("--tail-epochs", type=int, default=None,
                    help="Restrict to the last N epochs present in BOTH files")
    ap.add_argument("--out-csv", type=Path, default=None,
                    help="Optional per-row diff CSV")
    ap.add_argument("--per-tag-summary", action="store_true",
                    help="Print per-tag summary table (default on)")
    ap.add_argument("--worst-n", type=int, default=10,
                    help="Show top-N worst per-row diffs per tag")
    args = ap.parse_args(argv)

    native_rows = parse_stat_file(args.native_stat)
    bridge_rows = parse_stat_file(args.bridge_stat)
    if not native_rows:
        print(f"error: no parseable rows in {args.native_stat}", file=sys.stderr)
        return 1
    if not bridge_rows:
        print(f"error: no parseable rows in {args.bridge_stat}", file=sys.stderr)
        return 1

    def filter_rows(rows: List[Row]) -> List[Row]:
        out = rows
        if args.start_tow is not None:
            out = [r for r in out if r.tow >= args.start_tow]
        if args.end_tow is not None:
            out = [r for r in out if r.tow <= args.end_tow]
        return out

    native_rows = filter_rows(native_rows)
    bridge_rows = filter_rows(bridge_rows)

    if args.tail_epochs is not None and args.tail_epochs > 0:
        common_tow = sorted(set(r.tow for r in native_rows) & set(r.tow for r in bridge_rows))
        if common_tow:
            tail = set(common_tow[-args.tail_epochs:])
            native_rows = [r for r in native_rows if r.tow in tail]
            bridge_rows = [r for r in bridge_rows if r.tow in tail]

    native_idx = index_rows(native_rows)
    bridge_idx = index_rows(bridge_rows)

    # Group payload index → label per tag
    diffs_per_tag: Dict[str, List[List[float]]] = defaultdict(list)
    matched_keys: Dict[str, int] = defaultdict(int)
    only_native: Dict[str, int] = defaultdict(int)
    only_bridge: Dict[str, int] = defaultdict(int)

    rows_for_csv: List[List] = []

    all_keys = sorted(set(native_idx.keys()) | set(bridge_idx.keys()))
    for k in all_keys:
        n = native_idx.get(k)
        b = bridge_idx.get(k)
        tag = k[2]
        if n is None:
            only_bridge[tag] += 1
            continue
        if b is None:
            only_native[tag] += 1
            continue
        matched_keys[tag] += 1
        # Per-payload diff (native - bridge), only for matched indices
        n_vals = n.values
        b_vals = b.values
        m = min(len(n_vals), len(b_vals))
        diff = [n_vals[i] - b_vals[i] for i in range(m)]
        diffs_per_tag[tag].append(diff)
        if args.out_csv is not None:
            rows_for_csv.append([k[0], k[1], tag, k[3], n.stat, b.stat, *n_vals, *b_vals, *diff])

    # Print summary
    print(f"native: {args.native_stat}")
    print(f"bridge: {args.bridge_stat}")
    print(f"native rows: {len(native_rows)}, bridge rows: {len(bridge_rows)}")
    print()

    print(f"{'Tag':<6} {'matched':>8} {'native_only':>12} {'bridge_only':>12}  Per-payload RMS / mean / max-abs")
    print("-" * 100)
    for tag in ("$POS", "$CLK", "$TROP", "$ISB", "$ION", "$AMB"):
        m = matched_keys.get(tag, 0)
        no = only_native.get(tag, 0)
        bo = only_bridge.get(tag, 0)
        if m == 0:
            print(f"{tag:<6} {m:>8d} {no:>12d} {bo:>12d}")
            continue
        diffs = diffs_per_tag[tag]
        cols = list(zip(*diffs))
        labels = TAG_LAYOUTS[tag][1]
        # Handle native value-count differences (e.g., $TROP doesn't have grad)
        per_col = []
        for i, col in enumerate(cols):
            if i >= len(labels):
                continue
            lo = labels[i]
            r = rms(list(col))
            mean = sum(col) / len(col)
            mx = max(abs(v) for v in col)
            per_col.append(f"{lo}: rms={r:.4f}, mean={mean:+.4f}, max={mx:.4f}")
        print(f"{tag:<6} {m:>8d} {no:>12d} {bo:>12d}  " + " | ".join(per_col))
    print()

    # Worst-N per tag
    if args.worst_n and args.worst_n > 0:
        print(f"=== Top-{args.worst_n} worst per-row diffs by tag (rank by max-abs across payload columns) ===")
        for tag in ("$POS", "$CLK", "$TROP", "$ISB", "$ION", "$AMB"):
            diffs = diffs_per_tag.get(tag, [])
            if not diffs:
                continue
            scored = []
            i = 0
            for k in all_keys:
                if k[2] != tag:
                    continue
                n = native_idx.get(k)
                b = bridge_idx.get(k)
                if n is None or b is None:
                    continue
                d = diffs[i]
                i += 1
                worst = max(abs(v) for v in d)
                scored.append((worst, k, d, n.values, b.values))
            scored.sort(reverse=True)
            print(f"\n--- {tag} (top {min(args.worst_n, len(scored))}) ---")
            for s in scored[:args.worst_n]:
                worst, k, d, nv, bv = s
                print(f"  tow={k[1]:.0f} key={k[3]:<8} max_abs={worst:.4f}  "
                      f"native={nv} bridge={bv} diff={d}")

    if args.out_csv is not None:
        with args.out_csv.open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["week", "tow", "tag", "key", "stat_native", "stat_bridge",
                        "native_values...", "bridge_values...", "diff_values..."])
            for row in rows_for_csv:
                w.writerow(row)
        print(f"\nwrote {len(rows_for_csv)} rows to {args.out_csv}")

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
