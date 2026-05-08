#!/usr/bin/env python3
"""Convert RTKLIB-style .stat $SAT rows to canonical residual CSV.

Reads a CLASLIB bridge ``<out>.pos.stat`` file (sstat=2 output) and emits
the same column schema as ``--ppp-residual-log`` so that
``ppp_per_sat_residual_diff.py`` can diff native vs bridge directly.

For each ``$SAT`` row two canonical rows are emitted:
  * ``row_type=phase``  with ``residual_m=resp`` (phase residual)
  * ``row_type=code``   with ``residual_m=resc`` (code residual)

``frequency_index`` is converted from RTKLIB 1-based ``frq`` to 0-based
(L1=0, L2=1, L5=2). Rows with ``vsat=0`` (satellite not used at the freq)
are skipped. Signal/observation-code columns are emitted as empty
strings; pair this with ``ppp_per_sat_residual_diff.py --ignore-signals``
to avoid match-key mismatch with native logs.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Iterator, Sequence


CANONICAL_FIELDS = [
    "week",
    "tow",
    "iteration",
    "row_index",
    "sat",
    "row_type",
    "observation_m",
    "predicted_m",
    "residual_m",
    "variance_m2",
    "outlier_inflated",
    "elevation_deg",
    "iono_state_m",
    "solution_status",
    "primary_signal",
    "secondary_signal",
    "primary_observation_code",
    "secondary_observation_code",
    "frequency_index",
    "ionosphere_coefficient",
]


def parse_sat_rows(path: Path) -> Iterator[tuple[int, float, str, int, float, float, float, int, int]]:
    """Yield (week, tow, sat, frq, az_deg, el_deg, resp, resc, vsat) per $SAT row."""
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line.startswith("$SAT"):
                continue
            parts = line.split(",")
            if len(parts) < 10:
                continue
            try:
                week = int(parts[1])
                tow = float(parts[2])
                sat = parts[3]
                frq = int(parts[4])
                az = float(parts[5])
                el = float(parts[6])
                resp = float(parts[7])
                resc = float(parts[8])
                vsat = int(parts[9])
            except ValueError:
                continue
            yield week, tow, sat, frq, az, el, resp, resc, vsat


def write_canonical_csv(
    stat_path: Path,
    out_path: Path,
    *,
    include_invalid: bool = False,
    solution_status: int = 0,
) -> tuple[int, int]:
    """Convert .stat to canonical residual CSV. Returns (sat_rows, emitted_rows)."""
    sat_rows = 0
    emitted = 0
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CANONICAL_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for week, tow, sat, frq, az, el, resp, resc, vsat in parse_sat_rows(stat_path):
            sat_rows += 1
            if not include_invalid and vsat == 0:
                continue
            freq_index = frq - 1
            base = {
                "week": week,
                "tow": f"{tow:.6f}",
                "iteration": 0,
                "row_index": 0,
                "sat": sat,
                "observation_m": "",
                "predicted_m": "",
                "variance_m2": "",
                "outlier_inflated": 0,
                "elevation_deg": f"{el:.6f}",
                "iono_state_m": "",
                "solution_status": solution_status,
                "primary_signal": "",
                "secondary_signal": "",
                "primary_observation_code": "",
                "secondary_observation_code": "",
                "frequency_index": freq_index,
                "ionosphere_coefficient": "",
            }
            writer.writerow(
                {**base, "row_type": "phase", "residual_m": f"{resp:.9f}"}
            )
            writer.writerow(
                {**base, "row_type": "code", "residual_m": f"{resc:.9f}"}
            )
            emitted += 2
    return sat_rows, emitted


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stat_path", type=Path, help="CLASLIB bridge .pos.stat file")
    parser.add_argument("out_csv", type=Path, help="Canonical residual CSV path")
    parser.add_argument(
        "--include-invalid",
        action="store_true",
        help="Include $SAT rows with vsat=0 (default: skip them)",
    )
    parser.add_argument(
        "--solution-status",
        type=int,
        default=0,
        help="Value to write into solution_status column (default: 0)",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not args.stat_path.is_file():
        raise SystemExit(f"input not found: {args.stat_path}")
    sat_rows, emitted = write_canonical_csv(
        args.stat_path,
        args.out_csv,
        include_invalid=args.include_invalid,
        solution_status=args.solution_status,
    )
    print(
        f"converted {sat_rows} $SAT rows -> {emitted} canonical rows "
        f"({args.stat_path} -> {args.out_csv})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
