#!/usr/bin/env python3
"""Audit time-varying GLONASS phase residuals in a MADOCA postfit shadow CSV."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Sequence


SCHEMA = "madoca_glonass_phase_audit.v1"
REQUIRED_COLUMNS = {"record", "tow", "sat", "system", "obs_type", "residual_m", "elevation_deg"}


def _finite(value: str) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _correlation(xs: list[float], ys: list[float]) -> float | None:
    if len(xs) < 2:
        return None
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    dx = [value - mx for value in xs]
    dy = [value - my for value in ys]
    denom = math.sqrt(sum(value * value for value in dx) * sum(value * value for value in dy))
    return sum(x * y for x, y in zip(dx, dy)) / denom if denom > 0.0 else None


def audit_rows(rows: list[dict[str, str]]) -> dict[str, object]:
    grouped: dict[str, list[tuple[float, float, float]]] = defaultdict(list)
    for row in rows:
        if row.get("record") != "row" or row.get("system") != "GLO" or row.get("obs_type") != "phase":
            continue
        tow = _finite(row.get("tow", ""))
        residual = _finite(row.get("residual_m", ""))
        elevation = _finite(row.get("elevation_deg", ""))
        if tow is not None and residual is not None and elevation is not None and row.get("sat"):
            grouped[row["sat"]].append((tow, residual, elevation))

    satellites: dict[str, object] = {}
    for sat, samples in sorted(grouped.items()):
        samples.sort()
        residuals = [sample[1] for sample in samples]
        elevations = [sample[2] for sample in samples]
        mean = sum(residuals) / len(residuals)
        demeaned_rms = math.sqrt(sum((value - mean) ** 2 for value in residuals) / len(residuals))
        satellites[sat] = {
            "rows": len(samples),
            "duration_s": samples[-1][0] - samples[0][0],
            "mean_residual_m": mean,
            "rms_residual_m": math.sqrt(sum(value * value for value in residuals) / len(residuals)),
            "demeaned_rms_m": demeaned_rms,
            "residual_span_m": max(residuals) - min(residuals),
            "residual_elevation_correlation": _correlation(elevations, residuals),
        }
    return {
        "schema": SCHEMA,
        "status": "passed" if satellites else "failed",
        "glonass_phase_rows": sum(item["rows"] for item in satellites.values()),
        "satellites": satellites,
        "failures": [] if satellites else ["no valid GLONASS phase rows"],
    }


def audit_file(path: Path) -> dict[str, object]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        columns = set(reader.fieldnames or [])
        missing = sorted(REQUIRED_COLUMNS - columns)
        if missing:
            return {"schema": SCHEMA, "status": "failed", "failures": [f"missing columns: {', '.join(missing)}"]}
        return audit_rows(list(reader))


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("shadow_csv", type=Path)
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args(argv)
    report = audit_file(args.shadow_csv)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"madoca_glonass_phase_audit: {report['status']}")
    print(f"  GLONASS phase rows: {report.get('glonass_phase_rows', 0)}")
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
