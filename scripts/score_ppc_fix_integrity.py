#!/usr/bin/env python3
"""PPC FIX-integrity report: fix%, wrong-fix%, correct-fix% per solution.

wrong-fix = FIXED (status 4) epochs whose 3D error exceeds 0.5 m;
correct-fix% = correct fixed epochs / matched epochs (PPC convention).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

SCRIPTS = Path(__file__).resolve().parent
for p in (SCRIPTS, SCRIPTS.parent / "apps/commands/benchmarks"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import generate_driving_comparison as comparison  # noqa: E402


def report(label: str, pos: Path, reference) -> None:
    sol = comparison.read_libgnss_pos(pos)
    matched = comparison.match_to_reference(sol, reference, 0.25)
    n = max(len(matched), 1)
    fixed = [e for e in matched if e.status == 4]
    if fixed:
        err3 = np.array([np.hypot(e.horiz_error_m, e.up_m) for e in fixed])
        wrong = 100.0 * float(np.mean(err3 > 0.50))
        correct = 100.0 * float(np.sum(err3 <= 0.50)) / n
        med = float(np.median(err3))
    else:
        wrong = correct = med = float("nan")
    p50 = float(np.percentile([e.horiz_error_m for e in matched], 50)) if matched else float("nan")
    print(f"{label:28s} fix%={100.0*len(fixed)/n:5.1f} wrong-fix%={wrong:5.1f} "
          f"correct-fix%={correct:5.1f} fixed_med3d={med:5.2f} all_P50={p50:5.2f}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--reference", type=Path, required=True)
    ap.add_argument("arms", nargs="+", help="label=path.pos")
    args = ap.parse_args()
    reference = comparison.read_reference_csv(args.reference)
    for spec in args.arms:
        label, path = spec.split("=", 1)
        report(label, Path(path), reference)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
