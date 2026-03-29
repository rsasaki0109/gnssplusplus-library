#!/usr/bin/env python3
"""Inspect DCB / Bias-SINEX style files."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--input", type=Path, required=True, help="DCB or Bias-SINEX input file.")
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=None,
        help="Optional JSON summary output path.",
    )
    return parser.parse_args()


def parse_dcb(path: Path) -> dict[str, object]:
    payload: dict[str, object] = {
        "input": str(path),
        "format": "unknown",
        "entry_count": 0,
        "bias_types": [],
        "systems": [],
        "units": [],
        "aux_dcb_count": 0,
    }
    bias_types: set[str] = set()
    systems: set[str] = set()
    units: set[str] = set()
    in_bias_solution = False

    with path.open(encoding="ascii", errors="ignore") as handle:
        for raw_line in handle:
            line = raw_line.rstrip("\n")
            stripped = line.strip()
            if not stripped:
                continue
            if stripped.startswith("+BIAS/SOLUTION"):
                in_bias_solution = True
                payload["format"] = "SINEX_BIAS"
                continue
            if stripped.startswith("-BIAS/SOLUTION"):
                in_bias_solution = False
                continue

            if in_bias_solution:
                if stripped.startswith("*"):
                    continue
                fields = stripped.split()
                if len(fields) < 8:
                    continue
                bias_type = fields[0]
                sat_token = fields[1]
                unit = fields[6]
                bias_types.add(bias_type)
                systems.add(sat_token[0])
                units.add(unit)
                payload["entry_count"] = int(payload["entry_count"]) + 1
                continue

            if "PRN / BIAS / RMS" in line:
                payload["format"] = "IONEX_AUX_DCB"
                fields = line.split()
                if fields:
                    systems.add(fields[0][0])
                units.add("ns")
                payload["aux_dcb_count"] = int(payload["aux_dcb_count"]) + 1

    if payload["format"] == "IONEX_AUX_DCB":
        payload["entry_count"] = payload["aux_dcb_count"]
        bias_types.add("DCB")
    payload["bias_types"] = sorted(bias_types)
    payload["systems"] = sorted(system for system in systems if system)
    payload["units"] = sorted(units)
    return payload


def main() -> int:
    args = parse_args()
    if not args.input.exists():
        raise SystemExit(f"Missing DCB input: {args.input}")
    payload = parse_dcb(args.input)
    if args.summary_json is not None:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print(f"input: {payload['input']}")
    print(f"format: {payload['format']}")
    print(f"entries: {payload['entry_count']}")
    print(f"bias types: {', '.join(payload['bias_types']) or 'n/a'}")
    print(f"systems: {', '.join(payload['systems']) or 'n/a'}")
    print(f"units: {', '.join(payload['units']) or 'n/a'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
