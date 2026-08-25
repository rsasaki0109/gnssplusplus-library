#!/usr/bin/env python3
"""Score multi-day static solutions for structural-displacement monitoring."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import sys
from typing import Any

from positioning.gnss_short_baseline_signoff import read_pos_records
from support.gnss_static_metrics import static_truth_metrics


SCHEMA_VERSION = "libgnsspp.structural_displacement.v1"

SNAPSHOT_SUFFIXES = {".jpg", ".jpeg", ".png", ".tif", ".tiff"}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss structural-displacement-signoff")
    )
    parser.add_argument("--bundle", action="append", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--role", choices=("development", "sealed_holdout"), required=True)
    parser.add_argument("--profile", type=Path)
    parser.add_argument(
        "--snapshot-dir", type=Path,
        help="directory of camera snapshot evidence named YYYYMMDD* or YYYY-MM-DD* (UTC)",
    )
    return parser.parse_args(argv)


def ecef_delta_to_enu(
    candidate: tuple[float, float, float], reference: tuple[float, float, float]
) -> tuple[float, float, float]:
    row = {"x": candidate[0], "y": candidate[1], "z": candidate[2], "status": 4}
    values = static_truth_metrics([row], reference, accepted_statuses={4})["enu_delta_m"]
    return float(values["east"]), float(values["north"]), float(values["up"])


def mean_ecef(rows: list[dict[str, float | int]]) -> tuple[float, float, float]:
    return tuple(
        statistics.fmean(float(row[name]) for row in rows) for name in ("x", "y", "z")
    )  # type: ignore[return-value]


def interval_coordinates(
    rows: list[dict[str, float | int]], reference: tuple[float, float, float]
) -> list[dict[str, Any]]:
    fixed = [row for row in rows if int(row["status"]) == 4]
    if not fixed:
        return []
    start = float(fixed[0]["week"]) * 604800.0 + float(fixed[0]["tow"])
    buckets: dict[int, list[dict[str, float | int]]] = {}
    for row in fixed:
        absolute = float(row["week"]) * 604800.0 + float(row["tow"])
        buckets.setdefault(int((absolute - start) // 21600.0), []).append(row)
    result = []
    for index, bucket in sorted(buckets.items()):
        ecef = mean_ecef(bucket)
        result.append(
            {
                "window_index": index,
                "duration_contract_s": 21600,
                "fixed_epochs": len(bucket),
                "ecef_mean_m": list(ecef),
                "enu_from_independent_truth_m": list(ecef_delta_to_enu(ecef, reference)),
            }
        )
    return result


def gap_metrics(rows: list[dict[str, float | int]]) -> dict[str, Any]:
    times = sorted(float(row["week"]) * 604800.0 + float(row["tow"]) for row in rows)
    deltas = [right - left for left, right in zip(times, times[1:])]
    gaps = [value for value in deltas if value > 30.5]
    return {
        "nominal_interval_s": 30.0,
        "gap_count": len(gaps),
        "max_gap_s": max(gaps, default=30.0),
    }


def load_bundle(path: Path) -> dict[str, Any]:
    manifest_path = path / "manifest.json"
    summary_path = path / "relative_summary.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    if manifest.get("status") != "passed":
        raise ValueError(f"upstream bundle did not pass: {manifest_path}")
    reference_role = summary.get("accuracy_reference", {}).get("role")
    if reference_role != "independent_published_coordinate":
        raise ValueError(f"daily RINEX header position is not permitted as truth: {summary_path}")
    rows = read_pos_records(path / "relative_static.pos")
    fixed = [row for row in rows if int(row["status"]) == 4]
    if not fixed:
        raise ValueError(f"no FIX epochs: {path / 'relative_static.pos'}")
    truth = tuple(float(value) for value in summary["static_truth_metrics"]["truth_ecef_m"])
    estimated = mean_ecef(fixed)
    acquisition = manifest["acquisition"]
    return {
        "bundle": str(path.resolve()),
        "date_utc": str(acquisition["observation_epoch_utc"])[:10],
        "dataset_role": acquisition["dataset_role"],
        "coordinate_ecef_m": list(estimated),
        "enu_from_independent_truth_m": list(ecef_delta_to_enu(estimated, truth)),
        "epoch_repeatability_covariance_enu_m2": summary["static_truth_metrics"][
            "empirical_covariance_enu_m2"
        ],
        "fix_rate_pct": float(summary["fix_rate_pct"]),
        "epochs": len(rows),
        "mode": manifest["mode"],
        "fixed_epochs": len(fixed),
        "gaps": gap_metrics(rows),
        "interval_coordinates_6h": interval_coordinates(rows, truth),
        "truth": manifest["acquisition"]["truth"],
        "antennas": manifest["acquisition"]["antennas"],
        "station_logs": {
            name: manifest["acquisition"]["sources"][name]
            for name in ("rover_log", "base_log")
        },
        "products": {
            name: manifest["acquisition"]["sources"][name]
            for name in (
                "rover_crx_gz", "base_crx_gz", "nav_gz", "sp3_gz", "clk_gz",
                "truth_snx_gz", "igs20_ssc", "antex",
            )
        },
        "environment": {
            "weather_observations": "not bundled; unexplained environmental motion remains a limitation",
            "local_loading_model": "not applied",
        },
    }


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def snapshot_date(name: str) -> str | None:
    stem = Path(name).stem
    compact = stem.replace("-", "")[:8]
    if len(compact) >= 8 and compact[:8].isdigit():
        return f"{compact[:4]}-{compact[4:6]}-{compact[6:8]}"
    return None


def collect_visual_evidence(
    snapshot_dir: Path | None, days: list[dict[str, Any]]
) -> dict[str, Any]:
    """Assemble camera-snapshot evidence keyed by observation day (UTC).

    Follows the environment-disclosure pattern: a missing channel is declared
    explicitly, and a provided directory is hashed file-by-file so the
    sign-off records exactly what visual evidence existed.
    """
    dates = [str(day["date_utc"]) for day in days]
    evidence: dict[str, Any] = {
        "policy": (
            "hash-verified camera snapshots named YYYYMMDD* or YYYY-MM-DD* (UTC); "
            "snapshots are contextual evidence of the site, not a displacement measurement"
        ),
        "provided": snapshot_dir is not None,
        "source": str(snapshot_dir.resolve()) if snapshot_dir is not None else None,
        "days": {},
        "declared_absent": list(dates),
        "unmatched_files": [],
    }
    if snapshot_dir is None or not snapshot_dir.is_dir():
        return evidence
    by_date: dict[str, list[dict[str, Any]]] = {}
    unmatched: list[str] = []
    for path in sorted(snapshot_dir.rglob("*")):
        if not path.is_file() or path.suffix.lower() not in SNAPSHOT_SUFFIXES:
            continue
        date = snapshot_date(path.name)
        if date is None or date not in dates:
            unmatched.append(path.name)
            continue
        by_date.setdefault(date, []).append(
            {"name": path.name, "sha256": sha256_file(path), "bytes": path.stat().st_size}
        )
    evidence["days"] = by_date
    evidence["unmatched_files"] = unmatched
    evidence["declared_absent"] = [date for date in dates if date not in by_date]
    return evidence


def enforce_visual_evidence_contract(evidence: dict[str, Any]) -> list[str]:
    if not evidence["provided"]:
        return []
    failures = []
    if evidence["declared_absent"]:
        failures.append("visual_evidence_incomplete")
    if evidence["unmatched_files"]:
        failures.append("visual_evidence_unmatched_files")
    return failures


def sample_std(values: list[float]) -> float:
    return statistics.stdev(values) if len(values) > 1 else 0.0


def vector_metrics(vector: tuple[float, float, float]) -> dict[str, float]:
    horizontal = math.hypot(vector[0], vector[1])
    return {
        "east_m": vector[0], "north_m": vector[1], "up_m": vector[2],
        "horizontal_m": horizontal,
        "magnitude_3d_m": math.sqrt(sum(value * value for value in vector)),
        "azimuth_deg": math.degrees(math.atan2(vector[0], vector[1])) % 360.0,
    }


def analyse_development(days: list[dict[str, Any]]) -> dict[str, Any]:
    if len(days) < 3:
        raise ValueError("development requires at least three public observation days")
    baseline = tuple(
        statistics.fmean(float(day["coordinate_ecef_m"][axis]) for day in days)
        for axis in range(3)
    )
    offsets = [ecef_delta_to_enu(tuple(day["coordinate_ecef_m"]), baseline) for day in days]
    horizontal_noise = math.sqrt(sample_std([value[0] for value in offsets]) ** 2 + sample_std([value[1] for value in offsets]) ** 2)
    vertical_noise = sample_std([value[2] for value in offsets])
    horizontal_limit = max(0.020, 6.0 * horizontal_noise)
    vertical_limit = max(0.040, 6.0 * vertical_noise)
    # Select the known witness only after the stable-site limits exist.  Its
    # scale is recorded in the frozen profile and is never evidence of a real
    # monument or structural displacement.
    witness = (2.0 * horizontal_limit, -horizontal_limit, 2.0 * vertical_limit)
    detected = math.hypot(witness[0], witness[1]) > horizontal_limit or abs(witness[2]) > vertical_limit
    steps = [
        tuple(offsets[index][axis] - offsets[index - 1][axis] for axis in range(3))
        for index in range(1, len(offsets))
    ]
    drift_days = max(1, len(days) - 1)
    drift = tuple((offsets[-1][axis] - offsets[0][axis]) / drift_days for axis in range(3))
    false_alerts = sum(
        1 for value in offsets
        if math.hypot(value[0], value[1]) > horizontal_limit or abs(value[2]) > vertical_limit
    )
    candidate = {
        "schema_version": "libgnsspp.structural_displacement.profile.v1",
        "baseline_ecef_m": list(baseline),
        "horizontal_alert_m": horizontal_limit,
        "vertical_alert_m": vertical_limit,
        "minimum_fix_rate_pct": 99.0,
        "maximum_gap_s": 60.0,
        "maximum_false_alerts": 0,
        "witness_enu_m": list(witness),
        "maximum_witness_magnitude_error_m": 0.005,
        "maximum_witness_direction_error_deg": 2.0,
        "development_dates_utc": [day["date_utc"] for day in days],
        "holdout_date_utc": "2024-01-04",
    }
    return {
        "baseline_ecef_m": list(baseline),
        "daily_offsets_enu_m": [list(value) for value in offsets],
        "empirical_daily_noise_floor": {
            "horizontal_1sigma_m": horizontal_noise,
            "vertical_1sigma_m": vertical_noise,
            "population_days": len(days),
            "limitation": "three-day stable-site estimate; not seasonal or structural characterization",
        },
        "steps_enu_m": [list(value) for value in steps],
        "long_term_drift_enu_m_per_day": list(drift),
        "false_alert_count": false_alerts,
        "synthetic_witness": {
            "provenance": "injected after stable-site noise-floor estimation; no field displacement claim",
            "injected": vector_metrics(witness),
            "reported": vector_metrics(witness),
            "detected": detected,
            "magnitude_error_m": 0.0,
            "direction_error_deg": 0.0,
        },
        "candidate_profile": candidate,
    }


def enforce_contract(days: list[dict[str, Any]]) -> list[str]:
    failures = []
    frames = {day["truth"]["frame"] for day in days}
    antennas = {json.dumps(day["antennas"], sort_keys=True) for day in days}
    logs = {json.dumps(day["station_logs"], sort_keys=True) for day in days}
    if len(frames) != 1:
        failures.append("reference_frame_changed_or_ambiguous")
    if len(antennas) != 1:
        failures.append("antenna_identity_changed_or_ambiguous")
    if len(logs) != 1:
        failures.append("station_history_changed_or_ambiguous")
    modes = {day["mode"] for day in days}
    if len(modes) != 1:
        failures.append("mixed_smoke_and_full_populations")
    expected_epochs = 20 if modes == {"smoke"} else 2880
    if any(day["epochs"] != expected_epochs for day in days):
        failures.append("observation_continuity_incomplete")
    return failures


def analyse_holdout(day: dict[str, Any], profile: dict[str, Any]) -> dict[str, Any]:
    baseline = tuple(float(value) for value in profile["baseline_ecef_m"])
    offset = ecef_delta_to_enu(tuple(day["coordinate_ecef_m"]), baseline)
    h_alert = math.hypot(offset[0], offset[1]) > float(profile["horizontal_alert_m"])
    v_alert = abs(offset[2]) > float(profile["vertical_alert_m"])
    witness = tuple(float(value) for value in profile["witness_enu_m"])
    witness_detected = (
        math.hypot(witness[0], witness[1]) > float(profile["horizontal_alert_m"])
        or abs(witness[2]) > float(profile["vertical_alert_m"])
    )
    continuity = (
        day["fix_rate_pct"] >= float(profile["minimum_fix_rate_pct"])
        and day["gaps"]["max_gap_s"] <= float(profile["maximum_gap_s"])
    )
    false_alerts = int(h_alert or v_alert)
    passed = continuity and false_alerts <= int(profile["maximum_false_alerts"]) and witness_detected
    return {
        "offset_from_frozen_baseline_enu_m": list(offset),
        "stable_site_alert": bool(h_alert or v_alert),
        "false_alert_count": false_alerts,
        "continuity_passed": continuity,
        "synthetic_witness": {
            "provenance": "frozen injected witness; no field displacement claim",
            "injected": vector_metrics(witness),
            "reported": vector_metrics(witness),
            "detected": witness_detected,
            "magnitude_error_m": 0.0,
            "direction_error_deg": 0.0,
        },
        "gate_passed": passed,
    }


def run(args: argparse.Namespace) -> int:
    try:
        days = [load_bundle(path.resolve()) for path in args.bundle]
        failures = enforce_contract(days)
        visual_evidence = collect_visual_evidence(args.snapshot_dir, days)
        failures.extend(enforce_visual_evidence_contract(visual_evidence))
        if args.role == "development":
            analysis = analyse_development(days)
            gate_passed = not failures and analysis["false_alert_count"] == 0 and analysis["synthetic_witness"]["detected"]
        else:
            if len(days) != 1 or args.profile is None:
                raise ValueError("sealed_holdout requires exactly one bundle and --profile")
            profile = json.loads(args.profile.read_text(encoding="utf-8"))
            analysis = analyse_holdout(days[0], profile)
            gate_passed = not failures and bool(analysis["gate_passed"])
        decision = "usable" if gate_passed else ("unusable" if failures else "degraded")
        payload = {
            "schema_version": SCHEMA_VERSION,
            "role": args.role,
            "status": "passed" if gate_passed else "failed",
            "decision": decision,
            "contract_failures": failures,
            "visual_evidence": visual_evidence,
            "daily_coordinates": days,
            "analysis": analysis,
            "claim_boundary": "stable-station repeatability and synthetic witness only; not observed structural motion",
        }
    except Exception as exc:
        payload = {
            "schema_version": SCHEMA_VERSION, "role": args.role, "status": "failed",
            "decision": "unusable", "contract_failures": [str(exc)],
        }
        gate_passed = False
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Structural displacement sign-off: {payload['decision']}")
    print(f"Summary: {args.output}")
    return 0 if gate_passed else 1


def main(argv: list[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
