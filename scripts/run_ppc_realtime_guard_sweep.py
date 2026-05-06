#!/usr/bin/env python3
"""Run PPC coverage matrices across deployable real-time RTK guard profiles."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
from typing import Any


ROOT_DIR = Path(__file__).resolve().parents[1]
GNSS_CLI = ROOT_DIR / "apps" / "gnss.py"
DEFAULT_PROFILES = (
    "coverage=--ratio 2.4",
    (
        "fixed_update_guard=--ratio 2.4 --max-fixed-update-nis-per-obs 10 "
        "--max-fixed-update-post-rms 6 --max-fixed-update-gate-ratio 8"
    ),
    "nonfix_reset10=--ratio 2.4 --max-consec-nonfix-reset 10",
)
DEFAULT_RUN_KEYS = (
    "tokyo_run1",
    "tokyo_run2",
    "tokyo_run3",
    "nagoya_run1",
    "nagoya_run2",
    "nagoya_run3",
)


@dataclass(frozen=True)
class GuardProfile:
    name: str
    argv: tuple[str, ...]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description=(
            "Run deployable PPC real-time guard profiles through the six-run "
            "coverage matrix. PPC reference truth is used only by the matrix "
            "summaries for post-run scoring."
        ),
    )
    parser.add_argument("--dataset-root", type=Path, default=Path("/datasets/PPC-Dataset"))
    parser.add_argument("--output-dir", type=Path, default=ROOT_DIR / "output" / "ppc_realtime_guard_sweep")
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=ROOT_DIR / "output" / "ppc_realtime_guard_sweep" / "summary.json",
    )
    parser.add_argument("--markdown-output", type=Path, default=None)
    parser.add_argument("--max-epochs", type=int, default=-1)
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--preset", choices=("survey", "low-cost", "moving-base"), default="low-cost")
    parser.add_argument("--rtklib-root", type=Path, default=None)
    parser.add_argument("--rtklib-bin", type=Path, default=None)
    parser.add_argument("--rtklib-config", type=Path, default=ROOT_DIR / "scripts" / "rtklib_odaiba.conf")
    parser.add_argument("--use-existing-solutions", action="store_true")
    parser.add_argument(
        "--use-existing-matrices",
        action="store_true",
        help="Skip solver execution and summarize existing per-profile matrix.json files.",
    )
    parser.add_argument(
        "--profile",
        action="append",
        default=[],
        metavar="NAME=ARGS",
        help=(
            "Guard profile to run. ARGS are ppc-coverage-matrix arguments, "
            "for example: fixed=--ratio 2.4 --max-fixed-update-nis-per-obs 10. "
            "Repeat to override the built-in profiles."
        ),
    )
    parser.add_argument("--require-positioning-delta-min", type=float, default=None)
    parser.add_argument("--require-official-score-delta-min", type=float, default=None)
    parser.add_argument("--require-p95-h-delta-max", type=float, default=None)
    parser.add_argument("--require-solver-wall-time-max", type=float, default=None)
    parser.add_argument("--require-realtime-factor-min", type=float, default=None)
    parser.add_argument("--require-effective-epoch-rate-min", type=float, default=None)
    return parser.parse_args()


def sanitize_name(name: str) -> str:
    sanitized = "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in name).strip("_")
    if not sanitized:
        raise SystemExit(f"Profile name cannot be used as a path: {name!r}")
    return sanitized


def parse_profile_spec(spec: str) -> GuardProfile:
    name, separator, argv_text = spec.partition("=")
    name = name.strip()
    if not separator or not name:
        raise SystemExit("--profile must use NAME=ARGS")
    argv = tuple(shlex.split(argv_text))
    return GuardProfile(name=name, argv=argv)


def profiles_from_args(args: argparse.Namespace) -> list[GuardProfile]:
    specs = args.profile or list(DEFAULT_PROFILES)
    profiles = [parse_profile_spec(spec) for spec in specs]
    names = [profile.name for profile in profiles]
    if len(names) != len(set(names)):
        raise SystemExit(f"Profile names must be unique: {', '.join(names)}")
    return profiles


def append_optional(argv: list[str], flag: str, value: object | None) -> None:
    if value is not None:
        argv.extend([flag, str(value)])


def build_matrix_argv(args: argparse.Namespace, profile: GuardProfile) -> tuple[list[str], Path, Path]:
    profile_dir = args.output_dir / sanitize_name(profile.name)
    matrix_json = profile_dir / "matrix.json"
    matrix_md = profile_dir / "matrix.md"
    argv = [
        sys.executable,
        str(GNSS_CLI),
        "ppc-coverage-matrix",
        "--dataset-root",
        str(args.dataset_root),
        "--output-dir",
        str(profile_dir),
        "--summary-json",
        str(matrix_json),
        "--markdown-output",
        str(matrix_md),
        "--max-epochs",
        str(args.max_epochs),
        "--match-tolerance-s",
        str(args.match_tolerance_s),
        "--preset",
        args.preset,
    ]
    append_optional(argv, "--rtklib-root", args.rtklib_root)
    append_optional(argv, "--rtklib-bin", args.rtklib_bin)
    if args.rtklib_bin is not None:
        argv.extend(["--rtklib-config", str(args.rtklib_config)])
    if args.use_existing_solutions:
        argv.append("--use-existing-solutions")
    append_optional(argv, "--require-positioning-delta-min", args.require_positioning_delta_min)
    append_optional(argv, "--require-official-score-delta-min", args.require_official_score_delta_min)
    append_optional(argv, "--require-p95-h-delta-max", args.require_p95_h_delta_max)
    append_optional(argv, "--require-solver-wall-time-max", args.require_solver_wall_time_max)
    append_optional(argv, "--require-realtime-factor-min", args.require_realtime_factor_min)
    append_optional(argv, "--require-effective-epoch-rate-min", args.require_effective_epoch_rate_min)
    argv.extend(profile.argv)
    return argv, matrix_json, matrix_md


def load_matrix_payload(path: Path, profile_name: str) -> dict[str, Any]:
    if not path.exists():
        raise SystemExit(f"{profile_name}: missing matrix JSON: {path}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise SystemExit(f"{profile_name}: matrix JSON must contain an object")
    return payload


def run_profile(args: argparse.Namespace, profile: GuardProfile) -> tuple[Path, Path, dict[str, Any]]:
    argv, matrix_json, matrix_md = build_matrix_argv(args, profile)
    matrix_json.parent.mkdir(parents=True, exist_ok=True)
    if args.use_existing_matrices:
        return matrix_json, matrix_md, load_matrix_payload(matrix_json, profile.name)
    print("+", " ".join(argv))
    result = subprocess.run(argv, cwd=ROOT_DIR, check=False)
    if result.returncode != 0:
        raise SystemExit(f"{profile.name}: ppc-coverage-matrix exited with code {result.returncode}")
    return matrix_json, matrix_md, load_matrix_payload(matrix_json, profile.name)


def required_number(mapping: dict[str, Any], name: str, context: str) -> float:
    value = mapping.get(name)
    if not isinstance(value, (int, float)):
        raise SystemExit(f"{context}: missing numeric `{name}`")
    return float(value)


def optional_number(mapping: dict[str, Any], name: str) -> float | None:
    value = mapping.get(name)
    return float(value) if isinstance(value, (int, float)) else None


def matrix_score_distance(payload: dict[str, Any], context: str) -> tuple[float, float]:
    score = 0.0
    total = 0.0
    runs = payload.get("runs")
    if not isinstance(runs, list) or not runs:
        raise SystemExit(f"{context}: missing non-empty `runs`")
    for index, raw_run in enumerate(runs):
        if not isinstance(raw_run, dict):
            raise SystemExit(f"{context}: run {index} is not an object")
        metrics = raw_run.get("metrics")
        if not isinstance(metrics, dict):
            raise SystemExit(f"{context}: run {index} missing `metrics`")
        score += required_number(metrics, "ppc_official_score_distance_m", context)
        total += required_number(metrics, "ppc_official_total_distance_m", context)
    if total <= 0.0:
        raise SystemExit(f"{context}: official total distance must be positive")
    return score, total


def reference_path(dataset_root: Path, key: str) -> Path:
    city, _, run = key.partition("_")
    return dataset_root / city / run / "reference.csv"


def load_reference(path: Path) -> dict[tuple[int, float], tuple[float, float, float]]:
    records: dict[tuple[int, float], tuple[float, float, float]] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                week = int(row["GPS Week"])
                tow = round(float(row["GPS TOW (s)"]), 3)
                x = float(row["ECEF X (m)"])
                y = float(row["ECEF Y (m)"])
                z = float(row["ECEF Z (m)"])
            except (KeyError, ValueError):
                continue
            records[(week, tow)] = (x, y, z)
    return records


def load_solution(path: Path) -> dict[tuple[int, float], tuple[float, float, float, int]]:
    records: dict[tuple[int, float], tuple[float, float, float, int]] = {}
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 9:
                continue
            try:
                week = int(parts[0])
                tow = round(float(parts[1]), 3)
                x = float(parts[2])
                y = float(parts[3])
                z = float(parts[4])
                status = int(parts[8])
            except ValueError:
                continue
            records[(week, tow)] = (x, y, z, status)
    return records


def percentile(values: list[float], p: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int(round((p / 100.0) * (len(ordered) - 1)))))
    return ordered[index]


def truth_diagnostics(dataset_root: Path, solution_dir: Path, run_keys: list[str]) -> dict[str, Any]:
    counts = {
        "matched": 0,
        "reference_epochs": 0,
        "fix_ok": 0,
        "fix_wrong": 0,
        "float_ok": 0,
        "float_drift": 0,
        "missing": 0,
    }
    fix_errors: list[float] = []
    available = False
    for key in run_keys:
        ref_path = reference_path(dataset_root, key)
        sol_path = solution_dir / f"{key}.pos"
        if not ref_path.exists() or not sol_path.exists():
            continue
        available = True
        reference = load_reference(ref_path)
        solution = load_solution(sol_path)
        counts["reference_epochs"] += len(reference)
        for epoch_key, ref_ecef in reference.items():
            sol = solution.get(epoch_key)
            if sol is None:
                counts["missing"] += 1
                continue
            counts["matched"] += 1
            error_m = math.sqrt(
                (sol[0] - ref_ecef[0]) ** 2
                + (sol[1] - ref_ecef[1]) ** 2
                + (sol[2] - ref_ecef[2]) ** 2
            )
            if sol[3] == 4:
                fix_errors.append(error_m)
                if error_m <= 0.10:
                    counts["fix_ok"] += 1
                else:
                    counts["fix_wrong"] += 1
            elif error_m <= 1.00:
                counts["float_ok"] += 1
            else:
                counts["float_drift"] += 1
    if not available:
        return {"available": False}
    total_fixes = counts["fix_ok"] + counts["fix_wrong"]
    return {
        "available": True,
        **counts,
        "fix_wrong_rate_pct": round(100.0 * counts["fix_wrong"] / total_fixes, 6)
        if total_fixes
        else None,
        "fix50_m": percentile(fix_errors, 50.0),
        "fix95_m": percentile(fix_errors, 95.0),
    }


def aggregate(payload: dict[str, Any], name: str) -> float | None:
    aggregates = payload.get("aggregates")
    if not isinstance(aggregates, dict):
        return None
    return optional_number(aggregates, name)


def build_payload(
    profiles: list[tuple[GuardProfile, Path, Path, dict[str, Any]]],
    dataset_root: Path | None = None,
) -> dict[str, Any]:
    if not profiles:
        raise SystemExit("No profile results to summarize")
    baseline_profile, _, _, baseline_payload = profiles[0]
    baseline_score_m, baseline_total_m = matrix_score_distance(baseline_payload, baseline_profile.name)
    entries: list[dict[str, Any]] = []
    for profile, matrix_json, matrix_md, payload in profiles:
        score_m, total_m = matrix_score_distance(payload, profile.name)
        if abs(total_m - baseline_total_m) > 1e-3:
            raise SystemExit(f"{profile.name}: official total distance differs from baseline")
        score_pct = 100.0 * score_m / total_m
        baseline_score_pct = 100.0 * baseline_score_m / baseline_total_m
        run_keys = [
            str(raw_run["key"])
            for raw_run in payload.get("runs", [])
            if isinstance(raw_run, dict) and isinstance(raw_run.get("key"), str)
        ]
        diagnostics = (
            truth_diagnostics(dataset_root, matrix_json.parent, run_keys or list(DEFAULT_RUN_KEYS))
            if dataset_root is not None
            else {"available": False}
        )
        entries.append(
            {
                "name": profile.name,
                "argv": list(profile.argv),
                "matrix_json": str(matrix_json),
                "matrix_markdown": str(matrix_md),
                "weighted_official_score_pct": round(score_pct, 6),
                "delta_vs_baseline_score_distance_m": round(score_m - baseline_score_m, 6),
                "delta_vs_baseline_score_pct": round(score_pct - baseline_score_pct, 6),
                "avg_positioning_delta_vs_rtklib_pct": aggregate(payload, "avg_positioning_delta_pct"),
                "avg_p95_h_delta_vs_rtklib_m": aggregate(payload, "avg_p95_h_delta_m"),
                "max_solver_wall_time_s": aggregate(payload, "max_solver_wall_time_s"),
                "min_realtime_factor": aggregate(payload, "min_realtime_factor"),
                "min_effective_epoch_rate_hz": aggregate(payload, "min_effective_epoch_rate_hz"),
                "truth_diagnostics": diagnostics,
            }
        )
    best = max(entries, key=lambda item: float(item["weighted_official_score_pct"]))
    baseline_diag = entries[0].get("truth_diagnostics")
    baseline_wrong = (
        int(baseline_diag["fix_wrong"])
        if isinstance(baseline_diag, dict) and isinstance(baseline_diag.get("fix_wrong"), int)
        else None
    )
    safe_entries = []
    for entry in entries:
        diagnostics = entry.get("truth_diagnostics")
        if baseline_wrong is None:
            safe_entries.append(entry)
            continue
        wrong_count = diagnostics.get("fix_wrong") if isinstance(diagnostics, dict) else None
        if isinstance(wrong_count, int) and wrong_count <= baseline_wrong:
            safe_entries.append(entry)
    best_safe = max(
        safe_entries,
        key=lambda item: float(item["weighted_official_score_pct"]),
    ) if safe_entries else None
    return {
        "title": "PPC real-time RTK guard sweep",
        "baseline_profile": baseline_profile.name,
        "profiles": entries,
        "best_profile": best,
        "best_safe_profile": best_safe,
    }


def fmt(value: object, suffix: str = "") -> str:
    if not isinstance(value, (int, float)):
        return "n/a"
    return f"{float(value):.3f}{suffix}"


def render_markdown(payload: dict[str, Any]) -> str:
    lines = [
        f"# {payload['title']}",
        "",
        "Profiles are fixed command-line guard settings applied before scoring; PPC reference truth is post-run evaluation only.",
        "",
        "| Profile | Official | Delta | Wrong FIX | Wrong/FIX | Fix95 | Max wall | Min realtime | Min epoch rate | Args |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for profile in payload["profiles"]:
        assert isinstance(profile, dict)
        args_text = " ".join(str(item) for item in profile["argv"]) or "(coverage defaults)"
        diagnostics = profile.get("truth_diagnostics")
        if not isinstance(diagnostics, dict) or not diagnostics.get("available"):
            wrong_fix = wrong_rate = fix95 = "n/a"
        else:
            wrong_fix = str(diagnostics.get("fix_wrong", "n/a"))
            wrong_rate = fmt(diagnostics.get("fix_wrong_rate_pct"), "%")
            fix95 = fmt(diagnostics.get("fix95_m"), " m")
        lines.append(
            f"| {profile['name']} | "
            f"{fmt(profile.get('weighted_official_score_pct'), '%')} | "
            f"{fmt(profile.get('delta_vs_baseline_score_pct'), ' pp')} | "
            f"{wrong_fix} | "
            f"{wrong_rate} | "
            f"{fix95} | "
            f"{fmt(profile.get('max_solver_wall_time_s'), ' s')} | "
            f"{fmt(profile.get('min_realtime_factor'))} | "
            f"{fmt(profile.get('min_effective_epoch_rate_hz'), ' Hz')} | "
            f"`{args_text}` |"
        )
    best = payload["best_profile"]
    assert isinstance(best, dict)
    lines.extend(
        [
            "",
            f"Best official-score profile: **{best['name']}** "
            f"({fmt(best.get('weighted_official_score_pct'), '%')}).",
        ]
    )
    best_safe = payload.get("best_safe_profile")
    if isinstance(best_safe, dict):
        lines.append(
            f"Best no-worse-Wrong-FIX profile: **{best_safe['name']}** "
            f"({fmt(best_safe.get('weighted_official_score_pct'), '%')})."
        )
    else:
        lines.append("Best no-worse-Wrong-FIX profile: n/a.")
    lines.append("")
    return "\n".join(lines)


def write_outputs(args: argparse.Namespace, payload: dict[str, Any]) -> None:
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    markdown = render_markdown(payload)
    if args.markdown_output is not None:
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(markdown + "\n", encoding="utf-8")
    print(markdown)


def main() -> int:
    args = parse_args()
    results = []
    for profile in profiles_from_args(args):
        matrix_json, matrix_md, payload = run_profile(args, profile)
        results.append((profile, matrix_json, matrix_md, payload))
    write_outputs(args, build_payload(results, dataset_root=args.dataset_root))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
