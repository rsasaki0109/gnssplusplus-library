#!/usr/bin/env python3
"""Slice an expanded SSR CSV to an inclusive GPS time-of-week window."""

from __future__ import annotations

import argparse
from pathlib import Path


def slice_ssr(source: Path, destination: Path, tow_start: float, tow_end: float) -> int:
    if tow_end < tow_start:
        raise ValueError("tow_end must be greater than or equal to tow_start")

    destination.parent.mkdir(parents=True, exist_ok=True)
    rows_written = 0
    with source.open("rb") as reader, destination.open("wb") as writer:
        for line in reader:
            if line.startswith(b"#"):
                writer.write(line)
                continue
            fields = line.split(b",", 2)
            if len(fields) < 3:
                continue
            try:
                tow = float(fields[1])
            except ValueError:
                continue
            if tow_start <= tow <= tow_end:
                writer.write(line)
                rows_written += 1

    if rows_written == 0:
        destination.unlink(missing_ok=True)
        raise ValueError("the requested window contains no SSR rows")
    return rows_written


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--tow-start", type=float, required=True)
    parser.add_argument("--tow-end", type=float, required=True)
    args = parser.parse_args()
    rows = slice_ssr(args.source, args.destination, args.tow_start, args.tow_end)
    print(f"wrote {rows} SSR rows")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
