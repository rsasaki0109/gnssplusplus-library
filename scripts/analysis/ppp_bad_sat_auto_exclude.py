#!/usr/bin/env python3
"""Auto-exclude MADOCA bad satellites based on tail phase residuals.

Reads a gnss_ppp --ppp-residual-log CSV, computes per-(satellite, frequency)
tail-window mean phase residual on the final filter iteration, and emits
pairs whose |mean| exceeds a threshold. Output is a CSV list formatted for
`--phase-admission-exclude-sat-frequency-pairs`.

Why: MADOCA SSR phase bias does not capture certain per-satellite hardware
biases (block-dependent, e.g. GPS IIF/IIR-M and QZSS-2I). On MIZU 2025-04
day 091 four satellites (J02, J03, G26, G16) hold ~30–40 mm residuals that
prevent LAMBDA from converging to the true integer. Excluding the relevant
sat/freq pairs halves tail 3D RMS (0.304 m → 0.156 m).

Typical usage (two-pass pipeline)::

    gnss_ppp ... --ppp-residual-log /tmp/resid.csv --out /tmp/first.pos
    pairs=$(python scripts/analysis/ppp_bad_sat_auto_exclude.py /tmp/resid.csv \
        --threshold-m 0.015 --tail-seconds 900 --min-samples 10)
    gnss_ppp ... --phase-admission-exclude-sat-frequency-pairs "$pairs" \
                 --out /tmp/second.pos

The script exits 0 even when no pair exceeds the threshold (prints an empty
string). It exits non-zero only on malformed input or missing columns.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Tuple

REQUIRED_COLUMNS = [
    "tow",
    "iteration",
    "sat",
    "row_type",
    "residual_m",
    "frequency_index",
    "phase_accepted",
]

PairKey = Tuple[str, int]


def _parse_float(value: str) -> float:
    return float(value) if value not in ("", None) else 0.0


def _parse_int(value: str) -> int:
    return int(float(value)) if value not in ("", None) else 0


def collect_pairs(
    rows: List[Dict[str, str]],
    tail_seconds: float,
    threshold_m: float,
    min_samples: int,
    exclude_systems: List[str] | None = None,
) -> Dict[PairKey, Dict[str, float]]:
    """Return pairs whose tail |mean residual| exceeds ``threshold_m``.

    The "tail" is the last ``tail_seconds`` of the log; only rows that are
    accepted phase observations at the final filter iteration per epoch are
    counted. System prefixes in ``exclude_systems`` (e.g. ``["C"]``) are
    dropped before thresholding — MADOCA MIZU runs show BDS residuals are
    high but filter already de-weights them, so excluding BDS loses more
    geometry than it buys back.
    """

    if not rows:
        return {}
    banned_systems = {s.strip().upper() for s in (exclude_systems or []) if s.strip()}
    # Maximum iteration index per tow; we only sample the final iteration so
    # residuals reflect the converged filter state, not early warm-up.
    final_iter_per_tow: Dict[float, int] = {}
    for row in rows:
        tow = _parse_float(row["tow"])
        iteration = _parse_int(row["iteration"])
        prev = final_iter_per_tow.get(tow)
        if prev is None or iteration > prev:
            final_iter_per_tow[tow] = iteration

    max_tow = max(final_iter_per_tow)
    tow_cutoff = max_tow - tail_seconds

    buckets: Dict[PairKey, List[float]] = defaultdict(list)
    for row in rows:
        if row.get("row_type") != "phase":
            continue
        if _parse_int(row.get("phase_accepted", "0")) != 1:
            continue
        tow = _parse_float(row["tow"])
        if tow <= tow_cutoff:
            continue
        if _parse_int(row["iteration"]) != final_iter_per_tow.get(tow):
            continue
        sat = row["sat"].strip()
        freq = _parse_int(row["frequency_index"])
        if not sat:
            continue
        if sat[:1].upper() in banned_systems:
            continue
        try:
            residual = float(row["residual_m"])
        except ValueError:
            continue
        buckets[(sat, freq)].append(residual)

    flagged: Dict[PairKey, Dict[str, float]] = {}
    for key, values in buckets.items():
        if len(values) < min_samples:
            continue
        mean = sum(values) / len(values)
        if abs(mean) < threshold_m:
            continue
        flagged[key] = {
            "mean_m": mean,
            "abs_mean_m": abs(mean),
            "samples": float(len(values)),
        }
    return flagged


def select_pairs_by_satellite(
    flagged: Dict[PairKey, Dict[str, float]],
    max_satellites: int | None,
) -> Dict[PairKey, Dict[str, float]]:
    """Rank flagged pairs by satellite rather than by individual frequency.

    Bad satellites typically show correlated bias across all frequencies
    (shared hardware path). Selecting top-K by individual pair splits the
    exclusion power; grouping pairs per satellite and taking the top-K
    satellites (ranked by max |mean| across their flagged frequencies)
    keeps sibling frequencies together and matches empirical MIZU results.
    """

    if max_satellites is None or max_satellites <= 0:
        return flagged
    per_sat: Dict[str, Dict[PairKey, Dict[str, float]]] = defaultdict(dict)
    per_sat_score: Dict[str, float] = {}
    for key, stats in flagged.items():
        sat, _ = key
        per_sat[sat][key] = stats
        score = stats["abs_mean_m"]
        if score > per_sat_score.get(sat, 0.0):
            per_sat_score[sat] = score
    top_sats = sorted(per_sat_score, key=per_sat_score.get, reverse=True)[:max_satellites]
    selected: Dict[PairKey, Dict[str, float]] = {}
    for sat in top_sats:
        selected.update(per_sat[sat])
    return selected


def read_residual_csv(path: Path) -> List[Dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = [column for column in REQUIRED_COLUMNS if column not in reader.fieldnames]
        if missing:
            raise ValueError(f"{path}: missing required columns: {', '.join(missing)}")
        return [dict(row) for row in reader]


def format_pair_csv(flagged: Dict[PairKey, Dict[str, float]]) -> str:
    # Sort deterministically: satellite then frequency index.
    return ",".join(f"{sat}:{freq}" for (sat, freq) in sorted(flagged))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Emit a --phase-admission-exclude-sat-frequency-pairs "
        "CSV derived from a gnss_ppp --ppp-residual-log.",
    )
    parser.add_argument("residual_log", type=Path)
    parser.add_argument(
        "--threshold-m",
        type=float,
        default=0.020,
        help="Absolute tail mean phase residual (metres) above which a "
        "(sat, frequency) pair is flagged. Default 0.020 m "
        "(~0.1 cycle at L1); matches the MIZU 2025-04 sweet spot where "
        "real bad sats sit at 0.030-0.040 m. Lower the threshold to "
        "catch borderline biases; raise it to avoid flagging sibling "
        "frequencies of marginal cases.",
    )
    parser.add_argument(
        "--tail-seconds",
        type=float,
        default=900.0,
        help="Length of the tail window in seconds used to average residuals "
        "(default 900 = last 15 minutes).",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=10,
        help="Require at least this many accepted phase rows in the tail "
        "window before flagging (default 10).",
    )
    parser.add_argument(
        "--exclude-systems",
        default="C",
        help="Comma-separated satellite system prefixes to skip (default "
        "'C' for BDS; on MADOCA MIZU the filter already de-weights BDS "
        "and explicit exclusion hurts geometry). Use '' to include all "
        "systems.",
    )
    parser.add_argument(
        "--top-k-satellites",
        type=int,
        default=4,
        help="Return all flagged frequencies for the top-K satellites, "
        "ranked by max |mean residual| across their frequencies (default "
        "4; matches the MIZU-validated 4-satellite sweet spot: J02, J03, "
        "G26, G16). Bad satellites usually bias all frequencies together, "
        "so grouping by satellite keeps sibling pairs attached. Set to 0 "
        "or negative to return every pair that crosses the threshold.",
    )
    parser.add_argument(
        "--json",
        dest="emit_json",
        action="store_true",
        help="Emit per-pair stats as JSON on stderr alongside the CSV on stdout.",
    )
    return parser


def main(argv: List[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    rows = read_residual_csv(args.residual_log)
    exclude_systems = [s for s in args.exclude_systems.split(",") if s.strip()]
    max_satellites = (
        args.top_k_satellites if args.top_k_satellites and args.top_k_satellites > 0 else None
    )
    flagged = collect_pairs(
        rows,
        tail_seconds=args.tail_seconds,
        threshold_m=args.threshold_m,
        min_samples=args.min_samples,
        exclude_systems=exclude_systems,
    )
    flagged = select_pairs_by_satellite(flagged, max_satellites)
    sys.stdout.write(format_pair_csv(flagged))
    if flagged:
        sys.stdout.write("\n")
    if args.emit_json:
        summary = {
            f"{sat}:{freq}": stats
            for (sat, freq), stats in sorted(flagged.items())
        }
        sys.stderr.write(json.dumps(summary, indent=2, sort_keys=True))
        sys.stderr.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
