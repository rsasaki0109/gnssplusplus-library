#!/usr/bin/env python3
"""Check that candidate "none" replays are bit-identical to a control build.

Compares every CSV field except wall-clock processing_ms for each directory
present in both trees (normal/ and scenario/ layouts of compare_online_pva.py).
"""
import argparse
import csv
import json
from pathlib import Path
import sys


def rows(path):
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--control", type=Path, required=True)
    p.add_argument("--other", type=Path, required=True)
    args = p.parse_args()
    report, bad = [], 0
    for sub in ("normal", "scenario"):
        for directory in sorted((args.control/sub).iterdir()) if (args.control/sub).is_dir() else []:
            if not (directory/"replay/pva.csv").is_file(): continue
            a, b = rows(directory/"replay/pva.csv"), rows(args.other/sub/directory.name/"replay/pva.csv")
            keys = (set(a[0]) & set(b[0])) - {"processing_ms"}
            same = len(a) == len(b) and set(a[0]) == set(b[0]) and all(x[k] == y[k] for x, y in zip(a, b) for k in keys)
            bad += not same
            report.append(dict(run=directory.name, epochs=len(a), fields=len(keys), identical=same))
    print(json.dumps(dict(runs=len(report), failures=bad, detail=report), indent=1))
    return 1 if bad or not report else 0


if __name__ == "__main__":
    sys.exit(main())
