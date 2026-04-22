#!/usr/bin/env python3
"""Run the full PPC Tokyo/Nagoya RTK coverage matrix and summarize it."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time
from typing import Any

from gnss_runtime import ensure_input_exists, resolve_gnss_command


ROOT_DIR = Path(__file__).resolve().parent.parent
PPC_RUNS: tuple[tuple[str, str], ...] = (
    ("tokyo", "run1"),
    ("tokyo", "run2"),
    ("tokyo", "run3"),
    ("nagoya", "run1"),
    ("nagoya", "run2"),
    ("nagoya", "run3"),
)
GUARD_PATTERNS = {
    "nonfix_drift_guard": re.compile(
        r"non-FIX drift guard: (?P<state>enabled|disabled) "
        r"inspected_segments=(?P<inspected_segments>\d+) "
        r"rejected_segments=(?P<rejected_segments>\d+) "
        r"rejected_epochs=(?P<rejected_epochs>\d+)"
    ),
    "spp_height_step_guard": re.compile(
        r"SPP height-step guard: (?P<state>enabled|disabled) "
        r"rejected_epochs=(?P<rejected_epochs>\d+)"
    ),
    "float_bridge_tail_guard": re.compile(
        r"FLOAT bridge-tail guard: (?P<state>enabled|disabled) "
        r"inspected_segments=(?P<inspected_segments>\d+) "
        r"rejected_segments=(?P<rejected_segments>\d+) "
        r"rejected_epochs=(?P<rejected_epochs>\d+)"
    ),
    "fixed_bridge_burst_guard": re.compile(
        r"fixed bridge-burst guard: (?P<state>enabled|disabled) "
        r"inspected_segments=(?P<inspected_segments>\d+) "
        r"rejected_segments=(?P<rejected_segments>\d+) "
        r"rejected_epochs=(?P<rejected_epochs>\d+)"
    ),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT_DIR / "output" / "ppc_coverage_matrix",
        help="Directory for per-run .pos/.log/summary artifacts.",
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=None,
        help="Matrix summary JSON path (default: <output-dir>/ppc_coverage_matrix_summary.json).",
    )
    parser.add_argument(
        "--markdown-output",
        type=Path,
        default=None,
        help="Optional Markdown table output path.",
    )
    parser.add_argument(
        "--max-epochs",
        type=int,
        default=-1,
        help="Epoch limit passed to each ppc-demo run (default: -1 for full runs).",
    )
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--preset", choices=("survey", "low-cost", "moving-base"), default="low-cost")
    parser.add_argument(
        "--iono",
        choices=("auto", "off", "iflc", "est"),
        default=None,
        help="Optional RTK ionosphere option passed to each ppc-demo run.",
    )
    parser.add_argument(
        "--ratio",
        type=float,
        default=None,
        help="Optional RTK ambiguity ratio threshold passed to each ppc-demo run.",
    )
    parser.add_argument(
        "--max-hold-div",
        type=float,
        default=None,
        help="Optional RTK hold-fix divergence guard passed to each ppc-demo run.",
    )
    parser.add_argument(
        "--max-pos-jump",
        type=float,
        default=None,
        help="Optional RTK fixed-position jump guard passed to each ppc-demo run.",
    )
    parser.add_argument(
        "--max-pos-jump-min",
        type=float,
        default=None,
        help="Optional RTK adaptive fixed-position jump floor passed to each ppc-demo run.",
    )
    parser.add_argument(
        "--max-pos-jump-rate",
        type=float,
        default=None,
        help="Optional RTK adaptive fixed-position jump rate passed to each ppc-demo run.",
    )
    parser.add_argument(
        "--max-consec-float-reset",
        type=int,
        default=None,
        help="Optional RTK ambiguity reset after N consecutive FLOAT epochs.",
    )
    parser.add_argument(
        "--max-consec-nonfix-reset",
        type=int,
        default=None,
        help="Optional RTK ambiguity reset after N consecutive non-FIX epochs.",
    )
    parser.add_argument("--max-postfix-rms", type=float, default=None)
    parser.add_argument("--enable-wide-lane-ar", action="store_true")
    parser.add_argument("--wide-lane-threshold", type=float, default=None)
    parser.add_argument(
        "--rtklib-root",
        type=Path,
        default=None,
        help="Directory containing <city>_<run>/rtklib.pos files for side-by-side deltas.",
    )
    parser.add_argument("--rtklib-bin", type=Path, default=None)
    parser.add_argument(
        "--rtklib-config",
        type=Path,
        default=ROOT_DIR / "scripts" / "rtklib_odaiba.conf",
    )
    parser.add_argument("--use-existing-solutions", action="store_true")
    parser.add_argument(
        "--no-nonfix-drift-guard",
        action="store_true",
        help="Pass through to ppc-demo when reproducing raw non-FIX fallback drift.",
    )
    parser.add_argument("--nonfix-drift-max-anchor-gap", type=float, default=None)
    parser.add_argument("--nonfix-drift-max-anchor-speed", type=float, default=None)
    parser.add_argument("--nonfix-drift-max-residual", type=float, default=None)
    parser.add_argument("--nonfix-drift-min-horizontal-residual", type=float, default=None)
    parser.add_argument("--nonfix-drift-min-segment-epochs", type=int, default=None)
    parser.add_argument("--nonfix-drift-max-segment-epochs", type=int, default=None)
    parser.add_argument(
        "--no-spp-height-step-guard",
        action="store_true",
        help="Pass through to ppc-demo when reproducing raw SPP fallback spikes.",
    )
    parser.add_argument("--spp-height-step-min", type=float, default=None)
    parser.add_argument("--spp-height-step-rate", type=float, default=None)
    parser.add_argument(
        "--no-float-bridge-tail-guard",
        action="store_true",
        help="Pass through to ppc-demo for pre-bridge-tail coverage reproduction.",
    )
    parser.add_argument("--float-bridge-tail-max-anchor-gap", type=float, default=None)
    parser.add_argument("--float-bridge-tail-min-anchor-speed", type=float, default=None)
    parser.add_argument("--float-bridge-tail-max-anchor-speed", type=float, default=None)
    parser.add_argument("--float-bridge-tail-max-residual", type=float, default=None)
    parser.add_argument("--float-bridge-tail-min-segment-epochs", type=int, default=None)
    parser.add_argument("--fixed-bridge-burst-guard", action="store_true")
    parser.add_argument("--fixed-bridge-burst-max-anchor-gap", type=float, default=None)
    parser.add_argument("--fixed-bridge-burst-min-boundary-gap", type=float, default=None)
    parser.add_argument("--fixed-bridge-burst-max-residual", type=float, default=None)
    parser.add_argument("--fixed-bridge-burst-max-segment-epochs", type=int, default=None)
    parser.add_argument("--require-positioning-delta-min", type=float, default=None)
    parser.add_argument("--require-fix-delta-min", type=float, default=None)
    parser.add_argument("--require-official-score-delta-min", type=float, default=None)
    parser.add_argument("--require-score-3d-50cm-ref-delta-min", type=float, default=None)
    parser.add_argument("--require-p95-h-delta-max", type=float, default=None)
    return parser.parse_args()


def run_key(city: str, run_name: str) -> str:
    return f"{city}_{run_name}"


def output_paths(output_dir: Path, city: str, run_name: str) -> dict[str, Path]:
    key = run_key(city, run_name)
    return {
        "solution": output_dir / f"{key}.pos",
        "summary": output_dir / f"{key}_summary.json",
        "log": output_dir / f"{key}.log",
        "rtklib": output_dir / f"{key}_rtklib.pos",
    }


def rtklib_pos_for(args: argparse.Namespace, paths: dict[str, Path], city: str, run_name: str) -> Path | None:
    if args.rtklib_root is not None:
        return args.rtklib_root / run_key(city, run_name) / "rtklib.pos"
    if args.rtklib_bin is not None:
        return paths["rtklib"]
    return None


def build_ppc_demo_command(
    args: argparse.Namespace,
    city: str,
    run_name: str,
    paths: dict[str, Path],
) -> list[str]:
    command = [
        *resolve_gnss_command(ROOT_DIR),
        "ppc-demo",
        "--dataset-root",
        str(args.dataset_root),
        "--city",
        city,
        "--run",
        run_name,
        "--solver",
        "rtk",
        "--out",
        str(paths["solution"]),
        "--summary-json",
        str(paths["summary"]),
        "--max-epochs",
        str(args.max_epochs),
        "--match-tolerance-s",
        str(args.match_tolerance_s),
        "--preset",
        args.preset,
        "--no-arfilter",
        "--no-kinematic-post-filter",
    ]
    if getattr(args, "iono", None) is not None:
        command.extend(["--iono", args.iono])
    if getattr(args, "ratio", None) is not None:
        command.extend(["--ratio", str(args.ratio)])
    if getattr(args, "max_hold_div", None) is not None:
        command.extend(["--max-hold-div", str(args.max_hold_div)])
    if getattr(args, "max_pos_jump", None) is not None:
        command.extend(["--max-pos-jump", str(args.max_pos_jump)])
    if getattr(args, "max_pos_jump_min", None) is not None:
        command.extend(["--max-pos-jump-min", str(args.max_pos_jump_min)])
    if getattr(args, "max_pos_jump_rate", None) is not None:
        command.extend(["--max-pos-jump-rate", str(args.max_pos_jump_rate)])
    if getattr(args, "max_consec_float_reset", None) is not None:
        command.extend(["--max-consec-float-reset", str(args.max_consec_float_reset)])
    if getattr(args, "max_consec_nonfix_reset", None) is not None:
        command.extend(["--max-consec-nonfix-reset", str(args.max_consec_nonfix_reset)])
    if getattr(args, "max_postfix_rms", None) is not None:
        command.extend(["--max-postfix-rms", str(args.max_postfix_rms)])
    if getattr(args, "enable_wide_lane_ar", False):
        command.append("--enable-wide-lane-ar")
    if getattr(args, "wide_lane_threshold", None) is not None:
        command.extend(["--wide-lane-threshold", str(args.wide_lane_threshold)])
    if args.use_existing_solutions:
        command.append("--use-existing-solution")
    if getattr(args, "no_nonfix_drift_guard", False):
        command.append("--no-nonfix-drift-guard")
    if getattr(args, "nonfix_drift_max_anchor_gap", None) is not None:
        command.extend(["--nonfix-drift-max-anchor-gap", str(args.nonfix_drift_max_anchor_gap)])
    if getattr(args, "nonfix_drift_max_anchor_speed", None) is not None:
        command.extend(["--nonfix-drift-max-anchor-speed", str(args.nonfix_drift_max_anchor_speed)])
    if getattr(args, "nonfix_drift_max_residual", None) is not None:
        command.extend(["--nonfix-drift-max-residual", str(args.nonfix_drift_max_residual)])
    if getattr(args, "nonfix_drift_min_horizontal_residual", None) is not None:
        command.extend([
            "--nonfix-drift-min-horizontal-residual",
            str(args.nonfix_drift_min_horizontal_residual),
        ])
    if getattr(args, "nonfix_drift_min_segment_epochs", None) is not None:
        command.extend(["--nonfix-drift-min-segment-epochs", str(args.nonfix_drift_min_segment_epochs)])
    if getattr(args, "nonfix_drift_max_segment_epochs", None) is not None:
        command.extend(["--nonfix-drift-max-segment-epochs", str(args.nonfix_drift_max_segment_epochs)])
    if getattr(args, "no_spp_height_step_guard", False):
        command.append("--no-spp-height-step-guard")
    if getattr(args, "spp_height_step_min", None) is not None:
        command.extend(["--spp-height-step-min", str(args.spp_height_step_min)])
    if getattr(args, "spp_height_step_rate", None) is not None:
        command.extend(["--spp-height-step-rate", str(args.spp_height_step_rate)])
    if args.no_float_bridge_tail_guard:
        command.append("--no-float-bridge-tail-guard")
    if getattr(args, "float_bridge_tail_max_anchor_gap", None) is not None:
        command.extend([
            "--float-bridge-tail-max-anchor-gap",
            str(args.float_bridge_tail_max_anchor_gap),
        ])
    if getattr(args, "float_bridge_tail_min_anchor_speed", None) is not None:
        command.extend([
            "--float-bridge-tail-min-anchor-speed",
            str(args.float_bridge_tail_min_anchor_speed),
        ])
    if getattr(args, "float_bridge_tail_max_anchor_speed", None) is not None:
        command.extend([
            "--float-bridge-tail-max-anchor-speed",
            str(args.float_bridge_tail_max_anchor_speed),
        ])
    if getattr(args, "float_bridge_tail_max_residual", None) is not None:
        command.extend([
            "--float-bridge-tail-max-residual",
            str(args.float_bridge_tail_max_residual),
        ])
    if getattr(args, "float_bridge_tail_min_segment_epochs", None) is not None:
        command.extend([
            "--float-bridge-tail-min-segment-epochs",
            str(args.float_bridge_tail_min_segment_epochs),
        ])
    if getattr(args, "fixed_bridge_burst_guard", False):
        command.append("--fixed-bridge-burst-guard")
    if getattr(args, "fixed_bridge_burst_max_anchor_gap", None) is not None:
        command.extend([
            "--fixed-bridge-burst-max-anchor-gap",
            str(args.fixed_bridge_burst_max_anchor_gap),
        ])
    if getattr(args, "fixed_bridge_burst_min_boundary_gap", None) is not None:
        command.extend([
            "--fixed-bridge-burst-min-boundary-gap",
            str(args.fixed_bridge_burst_min_boundary_gap),
        ])
    if getattr(args, "fixed_bridge_burst_max_residual", None) is not None:
        command.extend([
            "--fixed-bridge-burst-max-residual",
            str(args.fixed_bridge_burst_max_residual),
        ])
    if getattr(args, "fixed_bridge_burst_max_segment_epochs", None) is not None:
        command.extend([
            "--fixed-bridge-burst-max-segment-epochs",
            str(args.fixed_bridge_burst_max_segment_epochs),
        ])

    rtklib_pos = rtklib_pos_for(args, paths, city, run_name)
    if rtklib_pos is not None:
        command.extend(["--rtklib-pos", str(rtklib_pos)])
        if args.rtklib_root is not None:
            command.append("--use-existing-rtklib-solution")
    if args.rtklib_bin is not None:
        command.extend(["--rtklib-bin", str(args.rtklib_bin), "--rtklib-config", str(args.rtklib_config)])
    return command


def run_command(command: list[str], log_path: Path) -> dict[str, object]:
    print("+", " ".join(command))
    started = time.perf_counter()
    completed = subprocess.run(command, cwd=ROOT_DIR, text=True, capture_output=True, check=False)
    elapsed = time.perf_counter() - started
    combined = completed.stdout
    if completed.stderr:
        if combined and not combined.endswith("\n"):
            combined += "\n"
        combined += completed.stderr
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.write_text(f"$ {' '.join(command)}\n\n{combined}", encoding="utf-8")
    if completed.returncode != 0:
        tail = "\n".join(combined.splitlines()[-30:])
        raise SystemExit(f"PPC coverage matrix run failed with exit code {completed.returncode}\n{tail}")
    return {"elapsed_s": elapsed, "stdout": combined}


def guard_counts(log_text: str) -> dict[str, dict[str, object] | None]:
    guards: dict[str, dict[str, object] | None] = {}
    for name, pattern in GUARD_PATTERNS.items():
        match = pattern.search(log_text)
        if match is None:
            guards[name] = None
            continue
        groups = match.groupdict()
        guards[name] = {
            "enabled": groups["state"] == "enabled",
            "inspected_segments": int(groups["inspected_segments"]) if "inspected_segments" in groups else 0,
            "rejected_segments": int(groups["rejected_segments"]) if "rejected_segments" in groups else 0,
            "rejected_epochs": int(groups["rejected_epochs"]),
        }
    return guards


def load_run_record(
    city: str,
    run_name: str,
    paths: dict[str, Path],
    command: list[str],
    log_text: str,
    elapsed_s: float,
) -> dict[str, object]:
    payload = json.loads(paths["summary"].read_text(encoding="utf-8"))
    delta_vs_rtklib = payload.get("delta_vs_rtklib")
    return {
        "city": city,
        "run": run_name,
        "key": run_key(city, run_name),
        "command": command,
        "solution_pos": str(paths["solution"]),
        "summary_json": str(paths["summary"]),
        "log_path": str(paths["log"]),
        "elapsed_s": round(elapsed_s, 6),
        "metrics": {
            "positioning_rate_pct": payload.get("positioning_rate_pct"),
            "fix_rate_pct": payload.get("fix_rate_pct"),
            "ppc_official_score_pct": payload.get("ppc_official_score_pct"),
            "ppc_official_score_distance_m": payload.get("ppc_official_score_distance_m"),
            "ppc_official_total_distance_m": payload.get("ppc_official_total_distance_m"),
            "ppc_score_3d_50cm_ref_pct": payload.get("ppc_score_3d_50cm_ref_pct"),
            "p95_h_m": payload.get("p95_h_m"),
            "max_h_m": payload.get("max_h_m"),
            "solver_wall_time_s": payload.get("solver_wall_time_s"),
            "realtime_factor": payload.get("realtime_factor"),
        },
        "rtklib": payload.get("rtklib"),
        "delta_vs_rtklib": delta_vs_rtklib if isinstance(delta_vs_rtklib, dict) else None,
        "guards": guard_counts(log_text),
    }


def average(values: list[float]) -> float | None:
    if not values:
        return None
    return round(sum(values) / len(values), 6)


def aggregate_runs(runs: list[dict[str, object]]) -> dict[str, object]:
    deltas = [run.get("delta_vs_rtklib") for run in runs]
    delta_dicts = [delta for delta in deltas if isinstance(delta, dict)]

    def delta_values(name: str) -> list[float]:
        values: list[float] = []
        for delta in delta_dicts:
            value = delta.get(name)
            if isinstance(value, (int, float)):
                values.append(float(value))
        return values

    def weighted_official_score(section_name: str) -> float | None:
        score_distance_m = 0.0
        total_distance_m = 0.0
        for run in runs:
            section = run.get(section_name)
            if not isinstance(section, dict):
                continue
            score = section.get("ppc_official_score_distance_m")
            total = section.get("ppc_official_total_distance_m")
            if isinstance(score, (int, float)) and isinstance(total, (int, float)) and total > 0.0:
                score_distance_m += float(score)
                total_distance_m += float(total)
        if total_distance_m <= 0.0:
            return None
        return round(100.0 * score_distance_m / total_distance_m, 6)

    bridge_rejected = 0
    bridge_seen = False
    fixed_burst_rejected = 0
    fixed_burst_seen = False
    for run in runs:
        guards = run.get("guards")
        if not isinstance(guards, dict):
            continue
        bridge = guards.get("float_bridge_tail_guard")
        if isinstance(bridge, dict):
            bridge_seen = True
            bridge_rejected += int(bridge.get("rejected_epochs", 0))
        fixed_burst = guards.get("fixed_bridge_burst_guard")
        if isinstance(fixed_burst, dict):
            fixed_burst_seen = True
            fixed_burst_rejected += int(fixed_burst.get("rejected_epochs", 0))

    weighted_official = weighted_official_score("metrics")
    weighted_rtklib_official = weighted_official_score("rtklib")
    weighted_official_delta = (
        round(weighted_official - weighted_rtklib_official, 6)
        if weighted_official is not None and weighted_rtklib_official is not None
        else None
    )

    return {
        "run_count": len(runs),
        "rtklib_comparison_run_count": len(delta_dicts),
        "weighted_official_score_pct": weighted_official,
        "weighted_rtklib_official_score_pct": weighted_rtklib_official,
        "weighted_official_score_delta_pct": weighted_official_delta,
        "avg_positioning_delta_pct": average(delta_values("positioning_rate_pct")),
        "avg_fix_delta_pct": average(delta_values("fix_rate_pct")),
        "avg_official_score_delta_pct": average(delta_values("ppc_official_score_pct")),
        "avg_score_3d_50cm_ref_delta_pct": average(delta_values("ppc_score_3d_50cm_ref_pct")),
        "avg_p95_h_delta_m": average(delta_values("p95_h_m")),
        "min_positioning_delta_pct": min(delta_values("positioning_rate_pct"), default=None),
        "min_official_score_delta_pct": min(delta_values("ppc_official_score_pct"), default=None),
        "max_p95_h_delta_m": max(delta_values("p95_h_m"), default=None),
        "float_bridge_tail_rejected_epochs": bridge_rejected if bridge_seen else None,
        "fixed_bridge_burst_rejected_epochs": fixed_burst_rejected if fixed_burst_seen else None,
    }


def build_matrix_payload(args: argparse.Namespace, runs: list[dict[str, object]]) -> dict[str, object]:
    return {
        "dataset_root": str(args.dataset_root),
        "output_dir": str(args.output_dir),
        "max_epochs": args.max_epochs,
        "match_tolerance_s": args.match_tolerance_s,
        "preset": args.preset,
        "iono": getattr(args, "iono", None),
        "ratio": getattr(args, "ratio", None),
        "max_hold_div": getattr(args, "max_hold_div", None),
        "max_pos_jump": getattr(args, "max_pos_jump", None),
        "max_pos_jump_min": getattr(args, "max_pos_jump_min", None),
        "max_pos_jump_rate": getattr(args, "max_pos_jump_rate", None),
        "max_consec_float_reset": getattr(args, "max_consec_float_reset", None),
        "max_consec_nonfix_reset": getattr(args, "max_consec_nonfix_reset", None),
        "max_postfix_rms": getattr(args, "max_postfix_rms", None),
        "enable_wide_lane_ar": getattr(args, "enable_wide_lane_ar", False),
        "wide_lane_threshold": getattr(args, "wide_lane_threshold", None),
        "no_nonfix_drift_guard": getattr(args, "no_nonfix_drift_guard", False),
        "nonfix_drift_max_anchor_gap": getattr(args, "nonfix_drift_max_anchor_gap", None),
        "nonfix_drift_max_anchor_speed": getattr(args, "nonfix_drift_max_anchor_speed", None),
        "nonfix_drift_max_residual": getattr(args, "nonfix_drift_max_residual", None),
        "nonfix_drift_min_horizontal_residual": getattr(
            args,
            "nonfix_drift_min_horizontal_residual",
            None,
        ),
        "nonfix_drift_min_segment_epochs": getattr(args, "nonfix_drift_min_segment_epochs", None),
        "nonfix_drift_max_segment_epochs": getattr(args, "nonfix_drift_max_segment_epochs", None),
        "no_spp_height_step_guard": getattr(args, "no_spp_height_step_guard", False),
        "spp_height_step_min": getattr(args, "spp_height_step_min", None),
        "spp_height_step_rate": getattr(args, "spp_height_step_rate", None),
        "no_float_bridge_tail_guard": getattr(args, "no_float_bridge_tail_guard", False),
        "float_bridge_tail_max_anchor_gap": getattr(args, "float_bridge_tail_max_anchor_gap", None),
        "float_bridge_tail_min_anchor_speed": getattr(args, "float_bridge_tail_min_anchor_speed", None),
        "float_bridge_tail_max_anchor_speed": getattr(args, "float_bridge_tail_max_anchor_speed", None),
        "float_bridge_tail_max_residual": getattr(args, "float_bridge_tail_max_residual", None),
        "float_bridge_tail_min_segment_epochs": getattr(args, "float_bridge_tail_min_segment_epochs", None),
        "fixed_bridge_burst_guard": getattr(args, "fixed_bridge_burst_guard", False),
        "fixed_bridge_burst_max_anchor_gap": getattr(args, "fixed_bridge_burst_max_anchor_gap", None),
        "fixed_bridge_burst_min_boundary_gap": getattr(args, "fixed_bridge_burst_min_boundary_gap", None),
        "fixed_bridge_burst_max_residual": getattr(args, "fixed_bridge_burst_max_residual", None),
        "fixed_bridge_burst_max_segment_epochs": getattr(args, "fixed_bridge_burst_max_segment_epochs", None),
        "coverage_profile": {
            "no_arfilter": True,
            "no_kinematic_post_filter": True,
            "float_bridge_tail_guard_enabled": not args.no_float_bridge_tail_guard,
        },
        "runs": runs,
        "aggregates": aggregate_runs(runs),
    }


def metric(metrics: dict[str, object] | None, name: str) -> float | None:
    if metrics is None:
        return None
    value = metrics.get(name)
    return float(value) if isinstance(value, (int, float)) else None


def render_markdown(payload: dict[str, object]) -> str:
    lines = [
        "| Run | Positioning | RTKLIB Positioning | Delta | Fix | RTKLIB Fix | PPC official | RTKLIB official | Official delta | P95 H delta | FLOAT bridge-tail rejected | FIX bridge-burst rejected |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for run in payload["runs"]:
        assert isinstance(run, dict)
        metrics = run.get("metrics")
        rtklib = run.get("rtklib")
        delta = run.get("delta_vs_rtklib")
        guards = run.get("guards")
        bridge_rejected = "n/a"
        if isinstance(guards, dict) and isinstance(guards.get("float_bridge_tail_guard"), dict):
            bridge_rejected = str(guards["float_bridge_tail_guard"].get("rejected_epochs", ""))
        fixed_burst_rejected = "n/a"
        if isinstance(guards, dict) and isinstance(guards.get("fixed_bridge_burst_guard"), dict):
            fixed_burst_rejected = str(
                guards["fixed_bridge_burst_guard"].get("rejected_epochs", "")
            )

        positioning = metric(metrics if isinstance(metrics, dict) else None, "positioning_rate_pct")
        fix = metric(metrics if isinstance(metrics, dict) else None, "fix_rate_pct")
        official = metric(metrics if isinstance(metrics, dict) else None, "ppc_official_score_pct")
        rtklib_positioning = metric(rtklib if isinstance(rtklib, dict) else None, "positioning_rate_pct")
        rtklib_fix = metric(rtklib if isinstance(rtklib, dict) else None, "fix_rate_pct")
        rtklib_official = metric(rtklib if isinstance(rtklib, dict) else None, "ppc_official_score_pct")
        positioning_delta = metric(delta if isinstance(delta, dict) else None, "positioning_rate_pct")
        official_delta = metric(delta if isinstance(delta, dict) else None, "ppc_official_score_pct")
        p95_delta = metric(delta if isinstance(delta, dict) else None, "p95_h_m")

        def pct(value: float | None) -> str:
            return "" if value is None else f"{value:.1f}%"

        def pp(value: float | None) -> str:
            return "" if value is None else f"{value:+.1f} pp"

        def meters(value: float | None) -> str:
            return "" if value is None else f"{value:+.2f} m"

        lines.append(
            f"| {run['key']} | {pct(positioning)} | {pct(rtklib_positioning)} | "
            f"{pp(positioning_delta)} | {pct(fix)} | {pct(rtklib_fix)} | "
            f"{pct(official)} | {pct(rtklib_official)} | {pp(official_delta)} | "
            f"{meters(p95_delta)} | {bridge_rejected} | {fixed_burst_rejected} |"
        )

    aggregates = payload["aggregates"]
    assert isinstance(aggregates, dict)

    def aggregate_text(name: str, suffix: str = "") -> str:
        value = aggregates.get(name)
        return "n/a" if value is None else f"{value}{suffix}"

    lines.extend(
        [
            "",
            "Averages:",
            f"- Positioning delta: {aggregate_text('avg_positioning_delta_pct', ' pp')}",
            f"- PPC official score delta: {aggregate_text('avg_official_score_delta_pct', ' pp')}",
            f"- PPC official weighted score: {aggregate_text('weighted_official_score_pct', '%')}",
            f"- RTKLIB official weighted score: {aggregate_text('weighted_rtklib_official_score_pct', '%')}",
            f"- PPC official weighted delta: {aggregate_text('weighted_official_score_delta_pct', ' pp')}",
            f"- P95 H delta: {aggregate_text('avg_p95_h_delta_m', ' m')}",
            f"- FLOAT bridge-tail rejected epochs: {aggregate_text('float_bridge_tail_rejected_epochs')}",
            f"- FIX bridge-burst rejected epochs: {aggregate_text('fixed_bridge_burst_rejected_epochs')}",
            "",
        ]
    )
    return "\n".join(lines)


def enforce_requirements(payload: dict[str, object], args: argparse.Namespace) -> None:
    requirements = {
        "positioning_rate_pct": (args.require_positioning_delta_min, ">="),
        "fix_rate_pct": (args.require_fix_delta_min, ">="),
        "ppc_official_score_pct": (args.require_official_score_delta_min, ">="),
        "ppc_score_3d_50cm_ref_pct": (args.require_score_3d_50cm_ref_delta_min, ">="),
        "p95_h_m": (args.require_p95_h_delta_max, "<="),
    }
    failures: list[str] = []
    for run in payload["runs"]:
        assert isinstance(run, dict)
        delta = run.get("delta_vs_rtklib")
        for name, (threshold, op) in requirements.items():
            if threshold is None:
                continue
            if not isinstance(delta, dict) or not isinstance(delta.get(name), (int, float)):
                failures.append(f"{run['key']}: missing RTKLIB delta `{name}`")
                continue
            value = float(delta[name])
            if op == ">=" and value < threshold:
                failures.append(f"{run['key']}: {name} delta {value:.6g} < {threshold:.6g}")
            if op == "<=" and value > threshold:
                failures.append(f"{run['key']}: {name} delta {value:.6g} > {threshold:.6g}")
    if failures:
        raise SystemExit("PPC coverage matrix requirements failed:\n" + "\n".join(failures))


def validate_inputs(args: argparse.Namespace) -> None:
    ensure_input_exists(args.dataset_root, "PPC-Dataset root", ROOT_DIR)
    if args.max_epochs == 0 or args.max_epochs < -1:
        raise SystemExit("--max-epochs must be positive or -1")
    if args.rtklib_root is not None and args.rtklib_bin is not None:
        raise SystemExit("Use either --rtklib-root or --rtklib-bin, not both")
    if args.rtklib_root is not None:
        ensure_input_exists(args.rtklib_root, "RTKLIB solution root", ROOT_DIR)
    if args.rtklib_bin is not None:
        ensure_input_exists(args.rtklib_bin, "RTKLIB binary", ROOT_DIR)
        ensure_input_exists(args.rtklib_config, "RTKLIB config", ROOT_DIR)


def main() -> int:
    args = parse_args()
    validate_inputs(args)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary_json = args.summary_json or args.output_dir / "ppc_coverage_matrix_summary.json"

    runs: list[dict[str, object]] = []
    for city, run_name in PPC_RUNS:
        paths = output_paths(args.output_dir, city, run_name)
        paths["solution"].parent.mkdir(parents=True, exist_ok=True)
        paths["summary"].parent.mkdir(parents=True, exist_ok=True)
        if args.rtklib_root is not None:
            rtklib_pos = rtklib_pos_for(args, paths, city, run_name)
            assert rtklib_pos is not None
            ensure_input_exists(rtklib_pos, f"RTKLIB {run_key(city, run_name)} solution", ROOT_DIR)
        if args.use_existing_solutions:
            ensure_input_exists(paths["solution"], f"existing {run_key(city, run_name)} solution", ROOT_DIR)

        command = build_ppc_demo_command(args, city, run_name, paths)
        result = run_command(command, paths["log"])
        runs.append(
            load_run_record(
                city,
                run_name,
                paths,
                command,
                str(result["stdout"]),
                float(result["elapsed_s"]),
            )
        )

    payload = build_matrix_payload(args, runs)
    enforce_requirements(payload, args)
    summary_json.parent.mkdir(parents=True, exist_ok=True)
    summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    markdown = render_markdown(payload)
    if args.markdown_output is not None:
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(markdown + "\n", encoding="utf-8")

    print("Finished PPC coverage matrix.")
    print(f"  summary: {summary_json}")
    if args.markdown_output is not None:
        print(f"  markdown: {args.markdown_output}")
    print(markdown)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
