#!/usr/bin/env python3
"""Build a consumer-facing, truth-scored trajectory bundle without re-solving."""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Any

import numpy as np

from support.gnss_runtime import application_root

ROOT = application_root(__file__)
sys.path[:0] = [str(ROOT / "scripts"), str(ROOT / "apps/commands/benchmarks")]
import generate_driving_comparison as comparison  # noqa: E402

SCHEMA = "libgnsspp.trajectory_bundle.v1"
REQUIRED_ARTIFACTS = (
    "raw.pos",
    "accepted.pos",
    "raw.kml",
    "accepted.kml",
    "trajectory.png",
    "segments.csv",
    "ros2_metadata.json",
    "summary.json",
    "bundle.log",
)
PROFILES = {
    "slam_evaluation": {"statuses": [4], "coverage_min_pct": 85.0, "horizontal_p95_max_m": 0.30, "max_gap_s": 3.0, "max_jump_m": 3.0},
    "map_prototyping": {"statuses": [4], "coverage_min_pct": 75.0, "horizontal_p95_max_m": 1.0, "max_gap_s": 10.0, "max_jump_m": 10.0},
    "visualization": {"statuses": [1, 2, 3, 4, 5, 6, 7], "coverage_min_pct": 95.0, "horizontal_p95_max_m": 25.0, "max_gap_s": 10.0, "max_jump_m": 125.0},
}


def match_solutions(reference, solutions, tolerance_s):
    ref_times = [r.week * 604800.0 + r.tow for r in reference]
    matched = {}
    for solution in solutions:
        stamp = solution.week * 604800.0 + solution.tow
        at = bisect.bisect_left(ref_times, stamp)
        choices = [i for i in (at - 1, at, at + 1) if 0 <= i < len(reference)]
        if not choices:
            continue
        index = min(choices, key=lambda i: abs(ref_times[i] - stamp))
        gap = abs(ref_times[index] - stamp)
        if gap > tolerance_s or (index in matched and matched[index]["time_gap_s"] <= gap):
            continue
        ref = reference[index]
        enu = comparison.ecef_to_enu(solution.ecef - ref.ecef, ref.lat_deg, ref.lon_deg)
        matched[index] = {"time_gap_s": gap, "horizontal_error_m": math.hypot(float(enu[0]), float(enu[1]))}
    return matched


def digest(path: Path) -> dict[str, Any]:
    h = hashlib.sha256(path.read_bytes()).hexdigest()
    return {"path": str(path), "bytes": path.stat().st_size, "sha256": h}


def git_revision() -> str:
    run = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True, capture_output=True, check=False)
    return run.stdout.strip() if run.returncode == 0 else "unknown"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME", "gnss trajectory-bundle"))
    p.add_argument("--pos", type=Path, required=True)
    p.add_argument("--reference-csv", type=Path)
    p.add_argument("--output-dir", type=Path, required=True)
    p.add_argument("--profile", choices=tuple(PROFILES), required=True)
    p.add_argument("--target-frame", required=True)
    p.add_argument("--lever-arm-m", required=True, help="antenna-to-target x,y,z in target-frame axes")
    p.add_argument("--time-system", default="GPST")
    p.add_argument("--match-tolerance-s", type=float, default=0.11)
    raw_argv = list(argv) if argv is not None else list(sys.argv[1:])
    args = p.parse_args(raw_argv)
    # Keep the parsed invocation independent from a unittest/process argv.
    args._raw_argv = raw_argv
    return args


def write_subset(source: Path, destination: Path, statuses: set[int]) -> None:
    with source.open(encoding="utf-8") as src, destination.open("w", encoding="utf-8") as dst:
        for line in src:
            stripped = line.strip()
            if line.lstrip().startswith(("%", "#")) or not stripped:
                dst.write(line)
                continue
            parts = stripped.split()
            if len(parts) <= 8:
                continue
            try:
                status = int(float(parts[8]))
                values = [float(value) for value in parts[0:8]]
            except (TypeError, ValueError):
                continue
            if status in statuses and all(math.isfinite(value) for value in values):
                dst.write(line)


def _finite_solution(row: Any) -> bool:
    values = (row.tow, row.lat_deg, row.lon_deg, row.height_m)
    return all(math.isfinite(float(value)) for value in values) and np.isfinite(row.ecef).all()


def _finite_reference(row: Any) -> bool:
    values = (row.week, row.tow, row.lat_deg, row.lon_deg, row.height_m)
    return all(math.isfinite(float(value)) for value in values) and np.isfinite(row.ecef).all()


def _artifact_record(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {"path": str(path), "bytes": None, "sha256": None, "state": "missing"}
    return digest(path)


def _manifest_argv(args: argparse.Namespace) -> list[str]:
    if args._raw_argv == list(sys.argv[1:]):
        return list(sys.argv)
    return [sys.executable, str(Path(__file__).resolve()), *args._raw_argv]


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    for path in (args.pos, args.reference_csv):
        if path is not None and not path.is_file():
            raise SystemExit(f"missing input: {path}")
    try:
        lever = [float(x) for x in args.lever_arm_m.split(",")]
    except ValueError as exc:
        raise SystemExit("--lever-arm-m must contain three finite numbers") from exc
    if len(lever) != 3 or not all(math.isfinite(x) for x in lever):
        raise SystemExit("--lever-arm-m must contain three finite numbers")
    if not args.target_frame.strip():
        raise SystemExit("--target-frame must be non-empty")
    if not args.time_system.strip():
        raise SystemExit("--time-system must be non-empty")
    if not math.isfinite(args.match_tolerance_s) or args.match_tolerance_s < 0.0:
        raise SystemExit("--match-tolerance-s must be finite and non-negative")
    out = args.output_dir.resolve(); out.mkdir(parents=True, exist_ok=True)
    raw = out / "raw.pos"; shutil.copyfile(args.pos, raw)
    try:
        solutions = comparison.read_libgnss_pos(raw)
    except (OSError, IndexError, TypeError, ValueError) as exc:
        raise SystemExit(f"invalid solution POS: {exc}") from exc
    cfg = PROFILES[args.profile]
    invalid_solution_count = sum(not _finite_solution(row) for row in solutions)
    accepted = [row for row in solutions if row.status in cfg["statuses"] and _finite_solution(row)]
    accepted_path = out / "accepted.pos"; write_subset(raw, accepted_path, set(cfg["statuses"]))

    times = [row.week * 604800.0 + row.tow for row in accepted]
    gaps = [times[i] - times[i - 1] for i in range(1, len(times))]
    jumps = [float(np.linalg.norm(accepted[i].ecef - accepted[i - 1].ecef)) for i in range(1, len(accepted))]
    transitions = sum(solutions[i].status != solutions[i - 1].status for i in range(1, len(solutions)))
    rejected_spans = []
    rejected_start = None
    for i in range(len(solutions) + 1):
        rejected = i < len(solutions) and solutions[i].status not in cfg["statuses"]
        if rejected and rejected_start is None:
            rejected_start = i
        elif not rejected and rejected_start is not None:
            rejected_spans.append((rejected_start, i - 1))
            rejected_start = None
    truth_role = "absent_candidate_trajectory"
    errors: list[float] = []
    route_covered = 0.0; route_total = 0.0; matched_count = 0
    reference = []
    reference_valid = False
    failures: list[str] = []
    if args.reference_csv:
        try:
            reference = comparison.read_reference_csv(args.reference_csv)
        except (OSError, IndexError, TypeError, ValueError) as exc:
            failures.append(f"independent_truth_invalid:{exc}")
        else:
            reference_valid = bool(reference) and all(_finite_reference(row) for row in reference)
            if reference_valid:
                matches = match_solutions(reference, accepted, args.match_tolerance_s)
                matched_count = len(matches); truth_role = "independent_ppc_applanix_ground_truth"
                errors = [float(v["horizontal_error_m"]) for v in matches.values() if math.isfinite(float(v["horizontal_error_m"]))]
                for i in range(1, len(reference)):
                    distance = float(np.linalg.norm(reference[i].ecef - reference[i - 1].ecef))
                    route_total += distance
                    if i in matches and i - 1 in matches:
                        route_covered += distance
            else:
                failures.append("independent_truth_empty_or_nonfinite")
    coverage = 100.0 * route_covered / route_total if route_total > 0 else 0.0
    p95 = float(np.percentile(errors, 95)) if errors else None
    if invalid_solution_count: failures.append("nonfinite_solution_epoch")
    if not args.reference_csv: failures.append("independent_truth_missing")
    if args.reference_csv and reference_valid and len(reference) < 2: failures.append("independent_truth_too_short")
    if any(gap <= 0.0 or not math.isfinite(gap) for gap in gaps): failures.append("non_increasing_solution_time")
    if not math.isfinite(coverage): failures.append("nonfinite_route_coverage")
    if p95 is not None and not math.isfinite(p95): failures.append("nonfinite_horizontal_error")
    if coverage < cfg["coverage_min_pct"]: failures.append("route_coverage_below_minimum")
    if p95 is None or p95 > cfg["horizontal_p95_max_m"]: failures.append("horizontal_p95_above_maximum")
    if (max(gaps) if gaps else 0.0) > cfg["max_gap_s"]: failures.append("time_gap_above_maximum")
    if (max(jumps) if jumps else 0.0) > cfg["max_jump_m"]: failures.append("position_jump_above_maximum")

    with (out / "segments.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f); writer.writerow(["kind", "index", "value", "threshold", "state"])
        for kind, values, limit in (("time_gap_s", gaps, cfg["max_gap_s"]), ("position_jump_m", jumps, cfg["max_jump_m"])):
            for i, value in enumerate(values, 1):
                if value > limit: writer.writerow([kind, i, value, limit, "unusable"])
        for start, end in rejected_spans:
            writer.writerow(["rejected_span_epochs", start, end - start + 1, "profile_status_filter", "declared"])
    ros = {"schema_version": "libgnsspp.trajectory_ros2_metadata.v1", "frame_id": args.target_frame, "time_system": args.time_system, "topics": {"fix": "/gnss/fix", "ecef_pose": "/gnss/ecef_pose", "path": "/gnss/path"}, "solution_file": "accepted.pos"}
    (out / "ros2_metadata.json").write_text(json.dumps(ros, indent=2) + "\n")
    dispatcher = ROOT / "apps/gnss.py"
    commands = [
        [sys.executable, str(dispatcher), "pos2kml", raw, out / "raw.kml", "--status", "all"],
        [sys.executable, str(dispatcher), "pos2kml", accepted_path, out / "accepted.kml", "--status", "all"],
        [sys.executable, str(dispatcher), "trackplot", accepted_path],
    ]
    log = out / "bundle.log"
    with log.open("w", encoding="utf-8") as handle:
        for cmd in commands:
            result = subprocess.run([str(x) for x in cmd], cwd=ROOT, stdout=handle, stderr=subprocess.STDOUT, check=False)
            if result.returncode: failures.append("artifact_conversion_failed")
    generated_plot = accepted_path.with_name(accepted_path.stem + "_trajectory.png")
    if generated_plot.is_file():
        os.replace(generated_plot, out / "trajectory.png")
    artifact_names = list(REQUIRED_ARTIFACTS)
    for name in artifact_names:
        # summary.json is the gate record being assembled below.
        if name == "summary.json":
            continue
        if not (out / name).is_file():
            failures.append(f"required_artifact_missing:{name}")
    state = "usable" if not failures else ("degraded" if args.reference_csv else "unusable")
    summary = {"schema_version": SCHEMA, "profile": args.profile, "state": state, "truth_role": truth_role,
        "populations": {"all_epochs": len(solutions), "accepted_epochs": len(accepted), "truth_matched_epochs": matched_count},
        "metrics": {"route_distance_m": route_total, "accepted_route_distance_m": route_covered, "distance_coverage_pct": coverage, "horizontal_error_p95_m": p95, "max_time_gap_s": max(gaps) if gaps else 0.0, "max_position_jump_m": max(jumps) if jumps else 0.0, "status_transitions": transitions, "rejected_spans": len(rejected_spans)},
        "gate": {"status": "pass" if not failures else "fail", "thresholds": cfg, "failure_reasons": sorted(set(failures))}}
    (out / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    if not (out / "summary.json").is_file():
        failures.append("required_artifact_missing:summary.json")
    manifest = {"schema_version": SCHEMA, "label": "ground_truth" if args.reference_csv and not failures else "candidate_trajectory", "state": state,
        "inputs": {"solution": digest(args.pos), "reference": digest(args.reference_csv) if args.reference_csv else None},
        "run": {"argv": _manifest_argv(args), "software_revision": git_revision()},
        "frames": {"target_frame": args.target_frame, "source_position": "GNSS_antenna_phase_center", "antenna_to_target_lever_arm_m": lever, "lever_arm_application": "metadata_only_not_applied_to_position_rows; consumer rotates it with synchronized attitude", "time_system": args.time_system},
        "populations": summary["populations"], "metrics": summary["metrics"], "gate": summary["gate"],
        "artifacts": {name: _artifact_record(out / name) for name in artifact_names}}
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(f"Trajectory bundle: {state} ({manifest['label']})")
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(run())
