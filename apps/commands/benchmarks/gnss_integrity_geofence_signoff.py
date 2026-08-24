#!/usr/bin/env python3
"""Truth-score empirical protection envelopes and geofence decisions."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import sys
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from support.gnss_runtime import application_root

ROOT = application_root(__file__)
sys.path[:0] = [str(ROOT / "scripts"), str(ROOT / "apps/commands/benchmarks")]
import generate_driving_comparison as comparison  # noqa: E402


SCHEMA_VERSION = "libgnsspp.integrity_geofence.v1"
AGE_BINS = (
    ("fixed", None, None),
    ("bridge_0_5s", 0.0, 5.0),
    ("bridge_5_15s", 5.0, 15.0),
    ("bridge_15_30s", 15.0, 30.0),
    ("bridge_30_60s", 30.0, 60.0),
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss integrity-geofence-signoff")
    )
    parser.add_argument("--rtk-pos", type=Path, required=True)
    parser.add_argument("--fused-pos", type=Path, required=True)
    parser.add_argument("--reference-csv", type=Path, required=True)
    parser.add_argument("--upstream-manifest", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--role", choices=("development", "sealed_holdout"), required=True)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--geofence-radius-m", type=float, default=500.0)
    parser.add_argument("--max-reference-epochs", type=int, default=0)
    return parser.parse_args(argv)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def record(path: Path) -> dict[str, Any]:
    return {"path": str(path.resolve()), "bytes": path.stat().st_size, "sha256": sha256_file(path)}


def epoch_key(row: Any) -> int:
    return round((int(row.week) * 604800.0 + float(row.tow)) * 10)


def horizontal_error(solution: Any, truth: Any) -> float:
    enu = comparison.ecef_to_enu(solution.ecef - truth.ecef, truth.lat_deg, truth.lon_deg)
    return math.hypot(float(enu[0]), float(enu[1]))


def surface_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    radius = 6371008.8
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    return 2.0 * radius * math.asin(min(1.0, math.sqrt(a)))


def age_bin(age_s: float | None, fixed: bool) -> str | None:
    if fixed:
        return "fixed"
    if age_s is None or age_s < 0.0 or age_s > 60.0:
        return None
    for name, lower, upper in AGE_BINS[1:]:
        assert lower is not None and upper is not None
        if lower <= age_s <= upper if lower == 0.0 else lower < age_s <= upper:
            return name
    return None


def percentile(values: list[float], q: float) -> float | None:
    return float(np.percentile(values, q)) if values else None


def load_inputs(args: argparse.Namespace) -> tuple[list[Any], list[Any], list[Any], dict[str, Any]]:
    for path in (args.rtk_pos, args.fused_pos, args.reference_csv, args.upstream_manifest):
        if not path.is_file():
            raise ValueError(f"missing input: {path}")
    manifest = json.loads(args.upstream_manifest.read_text(encoding="utf-8"))
    if manifest.get("status") != "passed":
        raise ValueError("upstream quality manifest did not pass")
    rtk = comparison.read_libgnss_pos(args.rtk_pos)
    fused = comparison.read_libgnss_pos(args.fused_pos)
    reference = comparison.read_reference_csv(args.reference_csv)
    if args.max_reference_epochs > 0:
        reference = reference[: args.max_reference_epochs]
    if not rtk or not fused or not reference:
        raise ValueError("empty RTK, fused, or independent-reference population")
    return rtk, fused, reference, manifest


def build_rows(
    rtk: list[Any], fused: list[Any], reference: list[Any], radius_m: float,
    envelopes: dict[str, float] | None,
) -> tuple[list[dict[str, Any]], tuple[float, float], dict[str, list[float]]]:
    rtk_by_time = {epoch_key(row): row for row in rtk}
    fused_by_time = {epoch_key(row): row for row in fused}
    fixed_rows = [row for row in rtk if int(row.status) == 4]
    if not fixed_rows:
        raise ValueError("geofence anchor unavailable: no RTK FIX epoch")
    anchor = (float(fixed_rows[0].lat_deg), float(fixed_rows[0].lon_deg))
    errors: dict[str, list[float]] = {name: [] for name, _, _ in AGE_BINS}
    rows: list[dict[str, Any]] = []
    last_fixed_time: float | None = None
    for truth in reference:
        key = epoch_key(truth)
        stamp = key / 10.0
        rtk_row = rtk_by_time.get(key)
        fused_row = fused_by_time.get(key)
        fixed = rtk_row is not None and int(rtk_row.status) == 4
        if fixed:
            last_fixed_time = stamp
            solution = rtk_row
        else:
            solution = fused_row
        age_s = 0.0 if fixed else (stamp - last_fixed_time if last_fixed_time is not None else None)
        population = age_bin(age_s, fixed)
        reason = None
        if solution is None:
            reason = "solution_missing"
        elif population is None:
            reason = "unqualified_or_over_60s_since_fix"
        elif not all(
            math.isfinite(float(value))
            for value in (solution.lat_deg, solution.lon_deg, *solution.ecef)
        ):
            reason = "solution_nonfinite"
        error = None
        signed_solution = None
        decision = "unknown"
        envelope = None
        if reason is None and population is not None and solution is not None:
            error = horizontal_error(solution, truth)
            errors[population].append(error)
            signed_solution = radius_m - surface_distance_m(
                anchor[0], anchor[1], float(solution.lat_deg), float(solution.lon_deg)
            )
            if envelopes is not None:
                envelope = envelopes.get(population)
                if envelope is None:
                    reason = "protection_envelope_missing"
                elif signed_solution > envelope:
                    decision = "inside"
                elif signed_solution < -envelope:
                    decision = "outside"
                else:
                    reason = "boundary_intersects_protection_envelope"
        truth_signed = radius_m - surface_distance_m(
            anchor[0], anchor[1], float(truth.lat_deg), float(truth.lon_deg)
        )
        truth_decision = "inside" if truth_signed >= 0.0 else "outside"
        rows.append(
            {
                "week": int(truth.week), "tow_s": float(truth.tow),
                "population": population, "age_since_fix_s": age_s,
                "horizontal_error_m": error, "protection_envelope_m": envelope,
                "signed_boundary_distance_m": signed_solution,
                "decision": decision, "unknown_reason": reason,
                "truth_signed_boundary_distance_m": truth_signed,
                "truth_decision": truth_decision,
                "misleading_decision": decision != "unknown" and decision != truth_decision,
                "envelope_exceeded": envelope is not None and error is not None and error > envelope,
            }
        )
    return rows, anchor, errors


def candidate_profile(errors: dict[str, list[float]], radius_m: float) -> dict[str, Any]:
    envelopes = {}
    populations = {}
    for name, _, _ in AGE_BINS:
        values = errors[name]
        if not values:
            raise ValueError(f"development population is empty: {name}")
        maximum = max(values)
        envelopes[name] = math.ceil((1.10 * maximum + 0.50) * 10.0) / 10.0
        populations[name] = {
            "epochs": len(values), "p95_m": percentile(values, 95),
            "p99_m": percentile(values, 99), "maximum_m": maximum,
        }
    running_bridge_envelope = 0.0
    for name, _, _ in AGE_BINS[1:]:
        running_bridge_envelope = max(running_bridge_envelope, envelopes[name])
        envelopes[name] = running_bridge_envelope
    return {
        "schema_version": "libgnsspp.integrity_geofence.profile.v1",
        "release_state": "candidate",
        "geofence": {"shape": "circle", "radius_m": radius_m, "anchor_rule": "first_rtk_fixed_epoch"},
        "maximum_qualified_age_s": 60.0,
        "empirical_protection_envelopes_m": envelopes,
        "development_error_populations": populations,
        "gates": {
            "maximum_misleading_decisions": 0,
            "maximum_envelope_exceedance_pct": 0.1,
            "minimum_decisive_availability_pct": 80.0,
        },
        "claim_boundary": "Empirical dataset envelope, not a certified integrity risk or aviation HPL/VPL.",
    }


def score_rows(rows: list[dict[str, Any]], gates: dict[str, Any]) -> dict[str, Any]:
    total = len(rows)
    decisive = [row for row in rows if row["decision"] != "unknown"]
    qualified = [row for row in rows if row["population"] is not None and row["horizontal_error_m"] is not None]
    misleading = [row for row in decisive if row["misleading_decision"]]
    exceeded = [row for row in qualified if row["envelope_exceeded"]]
    counts = {state: sum(row["decision"] == state for row in rows) for state in ("inside", "outside", "unknown")}
    reasons: dict[str, int] = {}
    for row in rows:
        if row["unknown_reason"]:
            reasons[row["unknown_reason"]] = reasons.get(row["unknown_reason"], 0) + 1
    availability = 100.0 * len(decisive) / total if total else 0.0
    exceedance = 100.0 * len(exceeded) / len(qualified) if qualified else 100.0
    failures = []
    if len(misleading) > int(gates["maximum_misleading_decisions"]):
        failures.append("misleading_decision_count_above_maximum")
    if exceedance > float(gates["maximum_envelope_exceedance_pct"]):
        failures.append("protection_envelope_exceedance_above_maximum")
    if availability < float(gates["minimum_decisive_availability_pct"]):
        failures.append("decisive_availability_below_minimum")
    return {
        "populations": {"truth_epochs": total, "qualified_epochs": len(qualified), **counts},
        "metrics": {
            "decisive_availability_pct": availability,
            "misleading_decisions": len(misleading),
            "envelope_exceedance_epochs": len(exceeded),
            "envelope_exceedance_pct": exceedance,
            "unknown_reasons": reasons,
        },
        "gate": {"status": "passed" if not failures else "failed", "thresholds": gates, "failures": failures},
    }


def write_artifacts(output_dir: Path, rows: list[dict[str, Any]], anchor: tuple[float, float], radius_m: float) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0]) if rows else []
    with (output_dir / "decisions.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)
    coordinates = []
    lat_scale = radius_m / 111320.0
    lon_scale = radius_m / (111320.0 * math.cos(math.radians(anchor[0])))
    for index in range(73):
        angle = 2.0 * math.pi * index / 72.0
        coordinates.append([anchor[1] + lon_scale * math.cos(angle), anchor[0] + lat_scale * math.sin(angle)])
    geojson = {
        "type": "FeatureCollection",
        "features": [{"type": "Feature", "properties": {"radius_m": radius_m, "anchor_rule": "first_rtk_fixed_epoch"},
                      "geometry": {"type": "Polygon", "coordinates": [coordinates]}}],
    }
    (output_dir / "geofence.geojson").write_text(json.dumps(geojson, indent=2) + "\n", encoding="utf-8")
    colors = {"inside": "tab:green", "outside": "tab:blue", "unknown": "tab:orange"}
    fig, ax = plt.subplots(figsize=(10, 3.5))
    for state in colors:
        selected = [(i, row["truth_signed_boundary_distance_m"]) for i, row in enumerate(rows) if row["decision"] == state]
        if selected:
            ax.scatter([item[0] for item in selected], [item[1] for item in selected], s=3, c=colors[state], label=state)
    ax.axhline(0.0, color="black", linewidth=1); ax.set_xlabel("truth epoch index")
    ax.set_ylabel("truth signed boundary distance (m)"); ax.legend(); fig.tight_layout()
    fig.savefig(output_dir / "decision_scorecard.png", dpi=150); plt.close(fig)


def run(args: argparse.Namespace) -> int:
    output_dir = args.output_dir.resolve()
    try:
        if not math.isfinite(args.geofence_radius_m) or args.geofence_radius_m <= 0.0:
            raise ValueError("geofence radius must be finite and positive")
        rtk, fused, reference, upstream = load_inputs(args)
        if args.role == "development":
            preliminary, _, errors = build_rows(rtk, fused, reference, args.geofence_radius_m, None)
            profile = candidate_profile(errors, args.geofence_radius_m)
        else:
            if args.profile is None:
                raise ValueError("sealed_holdout requires --profile")
            profile = json.loads(args.profile.read_text(encoding="utf-8"))
            if profile.get("release_state") != "sealed":
                raise ValueError("holdout profile is not sealed")
            if float(profile["geofence"]["radius_m"]) != args.geofence_radius_m:
                raise ValueError("geofence radius differs from frozen profile")
        rows, anchor, errors = build_rows(
            rtk, fused, reference, args.geofence_radius_m,
            {name: float(value) for name, value in profile["empirical_protection_envelopes_m"].items()},
        )
        scoring = score_rows(rows, profile["gates"])
        write_artifacts(output_dir, rows, anchor, args.geofence_radius_m)
        payload = {
            "schema_version": SCHEMA_VERSION, "role": args.role,
            "status": scoring["gate"]["status"],
            "decision": "usable" if scoring["gate"]["status"] == "passed" else "degraded",
            "profile": profile, "geofence_anchor_lat_lon_deg": list(anchor),
            **scoring,
            "error_populations": {
                name: {"epochs": len(values), "p95_m": percentile(values, 95), "maximum_m": max(values) if values else None}
                for name, values in errors.items()
            },
            "inputs": {
                "rtk_pos": record(args.rtk_pos), "fused_pos": record(args.fused_pos),
                "independent_reference": record(args.reference_csv),
                "upstream_manifest": record(args.upstream_manifest),
            },
            "upstream_quality_state": upstream.get("status"),
            "claim_boundary": "Empirical protection envelope (EPE); not certified integrity, HPL/VPL, or a safety-of-life authorization.",
        }
    except Exception as exc:
        rows = []
        payload = {
            "schema_version": SCHEMA_VERSION, "role": args.role, "status": "failed",
            "decision": "unusable", "failure": str(exc),
        }
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = output_dir / "summary.json"
    summary_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    artifacts = {
        name: record(output_dir / name) if (output_dir / name).is_file() else {"path": str(output_dir / name), "exists": False}
        for name in ("summary.json", "decisions.csv", "geofence.geojson", "decision_scorecard.png")
    }
    manifest = {"schema_version": SCHEMA_VERSION, "status": payload["status"], "artifacts": artifacts}
    (output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Integrity/geofence sign-off: {payload['decision']}")
    print(f"Summary: {summary_path}")
    return 0 if payload["status"] == "passed" else 1


def main(argv: list[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
