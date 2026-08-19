#!/usr/bin/env python3
"""Local web UI for browsing libgnss++ benchmark artifacts and receiver status."""

from __future__ import annotations

import argparse
import csv
import json
import os
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import signal
import threading
from typing import Any
from urllib.parse import parse_qs, urlparse

from support.gnss_toml_config import parse_args_with_toml

from support.gnss_runtime import application_root, load_python_module

ROOT_DIR = application_root(__file__)
driving_comparison = load_python_module(
    "gnssplusplus_generate_driving_comparison",
    ROOT_DIR / "scripts" / "generate_driving_comparison.py",
)


STATUS_COLORS = {
    "FIXED": "#2ecc71",
    "FLOAT": "#f39c12",
    "DGPS": "#3498db",
    "SPP": "#e74c3c",
}

MAX_RENDER_POINTS = 2000
DOCS_SITE_URL = "https://rsasaki0109.github.io/gnssplusplus-library/"


def default_root_dir() -> Path:
    source_root = application_root(__file__)
    if (source_root / "output").exists():
        return source_root
    return Path.cwd()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description="Serve a local web UI for benchmark artifacts, trajectories, and receiver status.",
    )
    parser.add_argument(
        "--config-toml",
        type=Path,
        default=None,
        help="Optional TOML config. Uses [web] or top-level keys.",
    )
    parser.add_argument("--host", default="127.0.0.1", help="Bind host (default: 127.0.0.1).")
    parser.add_argument("--port", type=int, default=8085, help="Bind port (default: 8085, use 0 for auto).")
    parser.add_argument(
        "--port-file",
        type=Path,
        default=None,
        help="Optional file that receives the bound port after startup.",
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=default_root_dir(),
        help="Artifact root directory (default: source tree root or current working directory).",
    )
    parser.add_argument(
        "--lib-pos",
        type=Path,
        default=None,
        help="libgnss++ .pos path (default: <root>/output/rtk_solution.pos).",
    )
    parser.add_argument(
        "--rtklib-pos",
        type=Path,
        default=None,
        help="RTKLIB .pos path (default: <root>/output/driving_rtklib_rtk.pos).",
    )
    parser.add_argument(
        "--odaiba-summary",
        type=Path,
        default=None,
        help="Odaiba summary JSON path (default: <root>/output/odaiba_summary.json).",
    )
    parser.add_argument(
        "--rcv-status",
        type=Path,
        default=None,
        help="Receiver status JSON path exported by gnss rcv.",
    )
    parser.add_argument(
        "--ppc-summary-glob",
        default="output/ppc_*_summary.json",
        help="Glob under --root for PPC summary JSON files.",
    )
    parser.add_argument(
        "--live-summary-glob",
        default="output/live*_summary.json",
        help="Glob under --root for gnss live-signoff summary JSON files.",
    )
    parser.add_argument(
        "--robotics-summary-glob",
        default="output/robotics_smoke*/**/*.json",
        help="Glob under --root for gnss robotics-smoke summary JSON files.",
    )
    parser.add_argument(
        "--ros2-bag-summary-glob",
        default="output/ros2_bag*_summary.json",
        help="Glob under --root for gnss ros2-bag-doctor summary JSON files.",
    )
    parser.add_argument(
        "--field-report-glob",
        default="output/field_report*.json",
        help="Glob under --root for gnss field-report JSON files.",
    )
    parser.add_argument(
        "--visibility-summary-glob",
        default="output/visibility*_summary.json",
        help="Glob under --root for gnss visibility summary JSON files.",
    )
    parser.add_argument(
        "--moving-base-summary-glob",
        default="output/*moving_base_summary.json",
        help="Glob under --root for gnss moving-base signoff summary JSON files.",
    )
    parser.add_argument(
        "--ppp-products-summary-glob",
        default="output/*ppp*_products*_summary.json",
        help="Glob under --root for gnss PPP product signoff summary JSON files.",
    )
    parser.add_argument(
        "--artifact-manifest",
        type=Path,
        default=None,
        help="Artifact manifest JSON path (default: <root>/output/artifact_manifest.json).",
    )
    parser.add_argument(
        "--docs-url",
        default=os.environ.get("GNSSPP_DOCS_URL", DOCS_SITE_URL),
        help="Optional docs site URL shown in the web UI header.",
    )
    return parse_args_with_toml(parser, "web")


def resolve_path(explicit: Path | None, root_dir: Path, relative: str) -> Path:
    if explicit is not None:
        return explicit
    return root_dir / relative


def relative_display(path: Path, root_dir: Path) -> str:
    try:
        # as_posix() keeps displayed paths portable (forward slashes) on Windows.
        return path.resolve().relative_to(root_dir.resolve()).as_posix()
    except ValueError:
        return str(path)


def load_json(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def normalize_artifact_path(root_dir: Path, value: Any) -> str | None:
    if not isinstance(value, str) or not value:
        return None
    if value.startswith(("http://", "https://")):
        return value
    try:
        return relative_display(resolve_under_root(root_dir, value), root_dir)
    except ValueError:
        return value
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def normalize_commercial_receiver(root_dir: Path, payload: Any) -> dict[str, Any] | None:
    if not isinstance(payload, dict):
        return None
    normalized = dict(payload)
    for key in ("solution_pos", "matched_csv"):
        normalized[key] = normalize_artifact_path(root_dir, normalized.get(key))
    return normalized


def resolve_under_root(root_dir: Path, path_text: str) -> Path:
    candidate = Path(path_text)
    if not candidate.is_absolute():
        candidate = (root_dir / candidate).resolve()
    else:
        candidate = candidate.resolve()
    candidate.relative_to(root_dir.resolve())
    return candidate


def load_visibility_rows(csv_path: Path) -> list[dict[str, Any]]:
    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows: list[dict[str, Any]] = []
        for row in reader:
            rows.append(
                {
                    "satellite": row.get("satellite"),
                    "system": row.get("system"),
                    "signal": row.get("signal"),
                    "azimuth_deg": float(row["azimuth_deg"]),
                    "elevation_deg": float(row["elevation_deg"]),
                    "snr_dbhz": float(row["snr_dbhz"]) if row.get("snr_dbhz") else None,
                }
            )
        return downsample_points(rows)


def load_moving_base_matches(csv_path: Path) -> list[dict[str, Any]]:
    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows: list[dict[str, Any]] = []
        for row in reader:
            heading_text = row.get("heading_error_deg", "")
            rows.append(
                {
                    "gps_week": int(row["gps_week"]),
                    "gps_tow_s": float(row["gps_tow_s"]),
                    "baseline_error_m": float(row["baseline_error_m"]),
                    "baseline_length_m": float(row["baseline_length_m"]),
                    "heading_error_deg": float(heading_text) if heading_text else None,
                    "status": int(row["status"]),
                    "satellites": int(row["satellites"]),
                }
            )
        return downsample_points(rows)


def classify_realtime_status(realtime_factor: Any) -> str:
    if not isinstance(realtime_factor, (int, float)):
        return "n/a"
    if realtime_factor >= 1.0:
        return "realtime"
    if realtime_factor >= 0.5:
        return "near-realtime"
    return "offline"


def classify_accuracy_status(p95_h_m: Any) -> str:
    if not isinstance(p95_h_m, (int, float)):
        return "n/a"
    if p95_h_m <= 0.5:
        return "excellent"
    if p95_h_m <= 2.0:
        return "good"
    if p95_h_m <= 10.0:
        return "rough"
    return "poor"


def classify_baseline_status(p95_baseline_error_m: Any) -> str:
    if not isinstance(p95_baseline_error_m, (int, float)):
        return "n/a"
    if p95_baseline_error_m <= 0.2:
        return "excellent"
    if p95_baseline_error_m <= 0.5:
        return "good"
    if p95_baseline_error_m <= 2.0:
        return "rough"
    return "poor"


def classify_ppp_products_status(
    converged: Any,
    p95_position_error_m: Any,
    solution_rate_pct: Any,
) -> str:
    if converged is True and isinstance(p95_position_error_m, (int, float)):
        return classify_accuracy_status(p95_position_error_m)
    if converged is True:
        return "converged"
    if isinstance(solution_rate_pct, (int, float)) and solution_rate_pct >= 95.0:
        return "tracking"
    return "warming"


def classify_comparison_status(*deltas: Any) -> str | None:
    numeric = [float(value) for value in deltas if isinstance(value, (int, float))]
    if not numeric:
        return None
    worst = max(numeric)
    if worst <= 0.0:
        return "better"
    if worst <= 0.25:
        return "close"
    return "worse"


def classify_robotics_smoke_status(payload: dict[str, Any]) -> str:
    explicit_status = payload.get("robotics_smoke_status")
    if explicit_status in ("passed", "failed"):
        return str(explicit_status)
    thresholds = payload.get("robotics_smoke_thresholds")
    if not isinstance(thresholds, dict):
        thresholds = payload.get("signoff_thresholds")
    if not isinstance(thresholds, dict):
        return "n/a"
    checks: list[bool] = []
    realtime_min = thresholds.get("require_realtime_factor_min")
    if isinstance(realtime_min, (int, float)):
        realtime_factor = payload.get("realtime_factor")
        checks.append(isinstance(realtime_factor, (int, float)) and realtime_factor >= realtime_min)
    rate_min = thresholds.get("require_effective_epoch_rate_min")
    if isinstance(rate_min, (int, float)):
        epoch_rate = payload.get("effective_epoch_rate_hz")
        checks.append(isinstance(epoch_rate, (int, float)) and epoch_rate >= rate_min)
    wall_max = thresholds.get("require_solver_wall_time_max")
    if isinstance(wall_max, (int, float)):
        wall_time = payload.get("solver_wall_time_s")
        checks.append(isinstance(wall_time, (int, float)) and wall_time <= wall_max)
    positioning_min = thresholds.get("require_positioning_rate_min")
    if isinstance(positioning_min, (int, float)) and positioning_min > 0.0:
        positioning_rate = payload.get("positioning_rate_pct")
        checks.append(
            isinstance(positioning_rate, (int, float))
            and positioning_rate >= positioning_min
        )
    if not checks:
        return "n/a"
    return "passed" if all(checks) else "failed"


def classify_ros2_bag_status(payload: dict[str, Any]) -> str:
    status = payload.get("status")
    if status in ("ready", "partial", "partial-metadata", "missing"):
        return str(status)
    if payload.get("replayable_raw_binary") is True:
        return "ready"
    if payload.get("message_count"):
        return "partial"
    return "missing"


def classify_field_report_status(payload: dict[str, Any]) -> str:
    setup = payload.get("setup_doctor")
    ros2 = payload.get("ros2_doctor")
    setup_status = setup.get("status") if isinstance(setup, dict) else None
    ros2_status = ros2.get("status") if isinstance(ros2, dict) else None
    if setup_status == "missing" or ros2_status == "missing":
        return "missing"

    ros2_bags = payload.get("ros2_bags")
    if not isinstance(ros2_bags, list):
        ros2_bags = []
    robotics_smoke = payload.get("robotics_smoke")
    if not isinstance(robotics_smoke, list):
        robotics_smoke = []
    has_bag_attention = any(item.get("status") != "ready" for item in ros2_bags if isinstance(item, dict))
    has_smoke_attention = any(item.get("status") != "passed" for item in robotics_smoke if isinstance(item, dict))
    if setup_status == "warn" or ros2_status == "warn" or has_bag_attention or has_smoke_attention:
        return "warn"
    if not ros2_bags or not robotics_smoke:
        return "partial"
    return "ready"


def summarize_field_report(payload: dict[str, Any], path: Path, root_dir: Path) -> dict[str, Any]:
    setup = payload.get("setup_doctor")
    if not isinstance(setup, dict):
        setup = {}
    ros2 = payload.get("ros2_doctor")
    if not isinstance(ros2, dict):
        ros2 = {}
    ros2_bags = payload.get("ros2_bags")
    if not isinstance(ros2_bags, list):
        ros2_bags = []
    robotics_smoke = payload.get("robotics_smoke")
    if not isinstance(robotics_smoke, list):
        robotics_smoke = []
    next_actions = payload.get("next_actions")
    if not isinstance(next_actions, list):
        next_actions = []
    markdown_report = normalize_artifact_path(root_dir, payload.get("markdown_report"))
    if markdown_report is None:
        markdown_candidate = path.with_suffix(".md")
        if markdown_candidate.exists():
            markdown_report = relative_display(markdown_candidate, root_dir)
    return {
        "_path": relative_display(path, root_dir),
        "summary_path": relative_display(path, root_dir),
        "markdown_report": markdown_report,
        "root": payload.get("root"),
        "device": payload.get("device"),
        "web_url": payload.get("web_url"),
        "status": classify_field_report_status(payload),
        "setup_status": setup.get("status"),
        "setup_counts": setup.get("status_counts") if isinstance(setup.get("status_counts"), dict) else {},
        "ros2_status": ros2.get("status"),
        "ros2_counts": ros2.get("status_counts") if isinstance(ros2.get("status_counts"), dict) else {},
        "ros2_bag_count": len(ros2_bags),
        "ros2_bag_ready": sum(1 for item in ros2_bags if isinstance(item, dict) and item.get("status") == "ready"),
        "ros2_bag_metadata": sum(
            1
            for item in ros2_bags
            if isinstance(item, dict) and item.get("status") == "partial-metadata"
        ),
        "robotics_count": len(robotics_smoke),
        "robotics_passed": sum(
            1
            for item in robotics_smoke
            if isinstance(item, dict) and item.get("status") == "passed"
        ),
        "next_actions": [str(action) for action in next_actions if action],
    }


def downsample_points(points: list[dict[str, Any]], limit: int = MAX_RENDER_POINTS) -> list[dict[str, Any]]:
    if len(points) <= limit:
        return points
    stride = max(1, len(points) // limit)
    sampled = points[::stride]
    if sampled[-1] != points[-1]:
        sampled.append(points[-1])
    return sampled


def build_solution_payload(name: str, path: Path, root_dir: Path, solver_label: str) -> dict[str, Any]:
    if not path.exists():
        return {
            "name": name,
            "available": False,
            "path": relative_display(path, root_dir),
            "error": "file not found",
        }

    if name == "libgnsspp":
        epochs = driving_comparison.read_libgnss_pos(path)
    elif name == "rtklib":
        epochs = driving_comparison.read_rtklib_pos(path)
    else:
        raise ValueError(f"unsupported solution name: {name}")

    if not epochs:
        return {
            "name": name,
            "available": False,
            "path": relative_display(path, root_dir),
            "error": "no epochs",
        }

    origin = driving_comparison.ReferenceEpoch(
        week=epochs[0].week,
        tow=epochs[0].tow,
        lat_deg=epochs[0].lat_deg,
        lon_deg=epochs[0].lon_deg,
        height_m=epochs[0].height_m,
        ecef=epochs[0].ecef,
    )
    trajectory = driving_comparison.trajectory_enu(epochs, origin)
    points: list[dict[str, Any]] = []
    status_counts: dict[str, int] = {}
    for epoch, enu in zip(epochs, trajectory):
        status_name, color = driving_comparison.status_style(solver_label, epoch.status)
        status_counts[status_name] = status_counts.get(status_name, 0) + 1
        points.append(
            {
                "tow": round(epoch.tow, 3),
                "east_m": round(float(enu[0]), 3),
                "north_m": round(float(enu[1]), 3),
                "up_m": round(float(enu[2]), 3),
                "status": status_name,
                "color": color,
                "satellites": epoch.num_satellites,
            }
        )

    return {
        "name": name,
        "available": True,
        "path": relative_display(path, root_dir),
        "epoch_count": len(epochs),
        "status_counts": status_counts,
        "points": downsample_points(points),
    }


def build_policy_suite_summaries(artifact_manifest: list[Any]) -> list[dict[str, Any]]:
    summaries: list[dict[str, Any]] = []
    for bundle in artifact_manifest:
        if not isinstance(bundle, dict) or bundle.get("category") != "ppc-spp-policy-suite":
            continue
        metrics = bundle.get("metrics")
        if not isinstance(metrics, dict):
            metrics = {}
        artifacts = bundle.get("artifacts")
        if not isinstance(artifacts, dict):
            artifacts = {}
        runs = metrics.get("runs")
        if not isinstance(runs, list):
            runs = []
        run_artifacts = artifacts.get("runs")
        if not isinstance(run_artifacts, list):
            run_artifacts = []
        summaries.append(
            {
                "label": bundle.get("label"),
                "summary_json": bundle.get("summary_json"),
                "headline": bundle.get("headline"),
                "status": bundle.get("status"),
                "comparison_status": bundle.get("comparison_status"),
                "policy_report": artifacts.get("policy_report"),
                "policy_csv": artifacts.get("policy_csv"),
                "run_count": metrics.get("run_count"),
                "rates_mps": metrics.get("rates_mps"),
                "min_jumps_m": metrics.get("min_jumps_m"),
                "bridge_max_gap_s": metrics.get("bridge_max_gap_s"),
                "bridge_max_anchor_speed_mps": metrics.get("bridge_max_anchor_speed_mps"),
                "max_positioning_drop_pct": metrics.get("max_positioning_drop_pct"),
                "min_positioning_rate_pct": metrics.get("min_positioning_rate_pct"),
                "max_p95_delta_m": metrics.get("max_p95_delta_m"),
                "worst_p95_h_delta_m": metrics.get("worst_p95_h_delta_m"),
                "worst_positioning_drop_pct": metrics.get("worst_positioning_drop_pct"),
                "checks": metrics.get("checks"),
                "runs": runs,
                "run_artifacts": run_artifacts,
            }
        )
    return summaries


def build_overview(args: argparse.Namespace) -> dict[str, Any]:
    root_dir = args.root.resolve()
    odaiba_summary_path = resolve_path(args.odaiba_summary, root_dir, "output/odaiba_summary.json")
    lib_pos_path = resolve_path(args.lib_pos, root_dir, "output/rtk_solution.pos")
    rtklib_pos_path = resolve_path(args.rtklib_pos, root_dir, "output/driving_rtklib_rtk.pos")
    artifact_manifest_path = resolve_path(args.artifact_manifest, root_dir, "output/artifact_manifest.json")
    rcv_status = load_json(args.rcv_status) if args.rcv_status is not None else None

    ppc_summaries: list[dict[str, Any]] = []
    for path in sorted(root_dir.glob(args.ppc_summary_glob)):
        payload = load_json(path)
        if payload is None:
            continue
        payload = dict(payload)
        payload["_path"] = relative_display(path, root_dir)
        payload["runtime_status"] = classify_realtime_status(payload.get("realtime_factor"))
        payload["quality_status"] = classify_accuracy_status(payload.get("p95_h_m"))
        commercial_receiver = normalize_commercial_receiver(root_dir, payload.get("commercial_receiver"))
        commercial_delta = payload.get("delta_vs_commercial_receiver")
        if not isinstance(commercial_delta, dict):
            commercial_delta = None
        if commercial_receiver is not None:
            payload["commercial_receiver"] = commercial_receiver
            payload["commercial_comparison_status"] = classify_comparison_status(
                commercial_delta.get("median_h_m") if commercial_delta else None,
                commercial_delta.get("p95_h_m") if commercial_delta else None,
                commercial_delta.get("max_h_m") if commercial_delta else None,
                commercial_delta.get("p95_abs_up_m") if commercial_delta else None,
            )
        ppc_summaries.append(payload)

    live_summaries: list[dict[str, Any]] = []
    for path in sorted(root_dir.glob(args.live_summary_glob)):
        payload = load_json(path)
        if payload is None:
            continue
        metrics = payload.get("metrics")
        if not isinstance(metrics, dict):
            continue
        live_summaries.append(
            {
                "_path": relative_display(path, root_dir),
                "execution_mode": payload.get("execution_mode"),
                "exit_code": payload.get("exit_code"),
                "termination": metrics.get("termination"),
                "aligned_epochs": metrics.get("aligned_epochs"),
                "written_solutions": metrics.get("written_solutions"),
                "fixed_solutions": metrics.get("fixed_solutions"),
                "realtime_factor": metrics.get("realtime_factor"),
                "effective_epoch_rate_hz": metrics.get("effective_epoch_rate_hz"),
                "rover_decoder_errors": metrics.get("rover_decoder_errors"),
                "base_decoder_errors": metrics.get("base_decoder_errors"),
                "runtime_status": classify_realtime_status(metrics.get("realtime_factor")),
            }
        )

    robotics_summaries: list[dict[str, Any]] = []
    for path in sorted(root_dir.glob(args.robotics_summary_glob)):
        payload = load_json(path)
        if payload is None:
            continue
        thresholds = payload.get("robotics_smoke_thresholds")
        if not isinstance(thresholds, dict):
            thresholds = payload.get("signoff_thresholds")
        if not isinstance(thresholds, dict):
            thresholds = {}
        tuning = payload.get("tuning_profile")
        if not isinstance(tuning, dict):
            tuning = {}
        failure_reasons = payload.get("robotics_smoke_failure_reasons")
        if not isinstance(failure_reasons, list):
            failure_reasons = []
        command = payload.get("robotics_smoke_command")
        if not isinstance(command, list):
            command = []
        robotics_summaries.append(
            {
                "_path": relative_display(path, root_dir),
                "summary_path": relative_display(path, root_dir),
                "dataset": payload.get("dataset"),
                "robotics_smoke_profile": payload.get("robotics_smoke_profile"),
                "signoff_profile": payload.get("signoff_profile"),
                "solver": payload.get("solver"),
                "matched_epochs": payload.get("matched_epochs"),
                "valid_epochs": payload.get("valid_epochs"),
                "fixed_epochs": payload.get("fixed_epochs"),
                "fix_rate_pct": payload.get("fix_rate_pct"),
                "positioning_rate_pct": payload.get("positioning_rate_pct"),
                "ppc_score_3d_50cm_ref_pct": payload.get("ppc_score_3d_50cm_ref_pct"),
                "median_h_m": payload.get("median_h_m"),
                "p95_h_m": payload.get("p95_h_m"),
                "solver_wall_time_s": payload.get("solver_wall_time_s"),
                "solution_span_s": payload.get("solution_span_s"),
                "realtime_factor": payload.get("realtime_factor"),
                "effective_epoch_rate_hz": payload.get("effective_epoch_rate_hz"),
                "solution_pos": normalize_artifact_path(root_dir, payload.get("solution_pos")),
                "reference_csv": normalize_artifact_path(root_dir, payload.get("reference_csv")),
                "run_dir": normalize_artifact_path(root_dir, payload.get("run_dir")),
                "rover": normalize_artifact_path(root_dir, payload.get("rover")),
                "base": normalize_artifact_path(root_dir, payload.get("base")),
                "nav": normalize_artifact_path(root_dir, payload.get("nav")),
                "runtime_status": classify_realtime_status(payload.get("realtime_factor")),
                "quality_status": classify_accuracy_status(payload.get("p95_h_m")),
                "smoke_status": classify_robotics_smoke_status(payload),
                "failure_reasons": failure_reasons,
                "command": command,
                "thresholds": thresholds,
                "tuning_profile": tuning,
            }
        )

    ros2_bag_summaries: list[dict[str, Any]] = []
    for path in sorted(root_dir.glob(args.ros2_bag_summary_glob)):
        payload = load_json(path)
        if payload is None:
            continue
        if payload.get("tool") not in (None, "ros2-bag-doctor"):
            continue
        topic_status = payload.get("topic_status")
        if not isinstance(topic_status, dict):
            topic_status = {}
        topics = payload.get("topics")
        if not isinstance(topics, list):
            topics = []
        gap_topics = [
            topic
            for topic in topics
            if isinstance(topic, dict) and isinstance(topic.get("gap_count"), int) and topic["gap_count"] > 0
        ]
        ros2_bag_summaries.append(
            {
                "_path": relative_display(path, root_dir),
                "summary_path": relative_display(path, root_dir),
                "bag": normalize_artifact_path(root_dir, payload.get("bag")),
                "status": classify_ros2_bag_status(payload),
                "diagnostic_depth": payload.get("diagnostic_depth"),
                "message_source": payload.get("message_source"),
                "storage_identifier": payload.get("storage_identifier"),
                "replayable_raw_binary": payload.get("replayable_raw_binary"),
                "message_count": payload.get("message_count"),
                "topic_count": payload.get("topic_count"),
                "duration_s": payload.get("duration_s"),
                "topic_status": topic_status,
                "topics": topics,
                "gap_topic_count": len(gap_topics),
                "commands": payload.get("commands") if isinstance(payload.get("commands"), dict) else {},
            }
        )

    visibility_summaries: list[dict[str, Any]] = []
    for path in sorted(root_dir.glob(args.visibility_summary_glob)):
        payload = load_json(path)
        if payload is None:
            continue
        csv_path = None
        png_path = None
        csv_value = payload.get("csv")
        if isinstance(csv_value, str):
            try:
                resolved_csv = resolve_under_root(root_dir, csv_value)
                csv_path = relative_display(resolved_csv, root_dir)
                sibling_png = resolved_csv.with_suffix(".png")
                if sibling_png.exists():
                    png_path = relative_display(sibling_png, root_dir)
            except ValueError:
                csv_path = csv_value
        png_value = payload.get("png")
        if isinstance(png_value, str):
            try:
                png_path = relative_display(resolve_under_root(root_dir, png_value), root_dir)
            except ValueError:
                png_path = png_value
        visibility_summaries.append(
            {
                "_path": relative_display(path, root_dir),
                "csv_path": csv_path,
                "png_path": png_path,
                "epochs_processed": payload.get("epochs_processed"),
                "epochs_with_rows": payload.get("epochs_with_rows"),
                "rows_written": payload.get("rows_written"),
                "unique_satellites": payload.get("unique_satellites"),
                "mean_satellites_per_epoch": payload.get("mean_satellites_per_epoch"),
                "max_satellites_per_epoch": payload.get("max_satellites_per_epoch"),
                "mean_elevation_deg": payload.get("mean_elevation_deg"),
                "mean_snr_dbhz": payload.get("mean_snr_dbhz"),
            }
        )

    moving_base_summaries: list[dict[str, Any]] = []
    for path in sorted(root_dir.glob(args.moving_base_summary_glob)):
        payload = load_json(path)
        if payload is None:
            continue
        commercial_receiver = normalize_commercial_receiver(root_dir, payload.get("commercial_receiver"))
        commercial_delta = payload.get("libgnss_vs_commercial_receiver")
        if not isinstance(commercial_delta, dict):
            commercial_delta = None
        moving_base_summaries.append(
            {
                "_path": relative_display(path, root_dir),
                "summary_path": relative_display(path, root_dir),
                "execution_mode": payload.get("execution_mode"),
                "solver": payload.get("solver"),
                "matched_epochs": payload.get("matched_epochs"),
                "valid_epochs": payload.get("valid_epochs"),
                "fix_rate_pct": payload.get("fix_rate_pct"),
                "median_baseline_error_m": payload.get("median_baseline_error_m"),
                "p95_baseline_error_m": payload.get("p95_baseline_error_m"),
                "p95_heading_error_deg": payload.get("p95_heading_error_deg"),
                "termination": payload.get("termination"),
                "realtime_factor": payload.get("realtime_factor"),
                "effective_epoch_rate_hz": payload.get("effective_epoch_rate_hz"),
                "solution_pos": normalize_artifact_path(root_dir, payload.get("solution_pos")),
                "matched_csv": normalize_artifact_path(root_dir, payload.get("matched_csv")),
                "prepare_summary_json": normalize_artifact_path(root_dir, payload.get("prepare_summary_json")),
                "products_summary_json": normalize_artifact_path(root_dir, payload.get("products_summary_json")),
                "plot_png": normalize_artifact_path(root_dir, payload.get("plot_png")),
                "commercial_receiver_csv": normalize_artifact_path(
                    root_dir,
                    payload.get("commercial_receiver_csv"),
                ),
                "commercial_receiver_matched_csv": normalize_artifact_path(
                    root_dir,
                    payload.get("commercial_receiver_matched_csv"),
                ),
                "commercial_receiver": commercial_receiver,
                "libgnss_vs_commercial_receiver": commercial_delta,
                "commercial_comparison_status": classify_comparison_status(
                    commercial_delta.get("median_baseline_error_m_delta") if commercial_delta else None,
                    commercial_delta.get("p95_baseline_error_m_delta") if commercial_delta else None,
                    commercial_delta.get("max_baseline_error_m_delta") if commercial_delta else None,
                    commercial_delta.get("p95_heading_error_deg_delta") if commercial_delta else None,
                ),
                "signoff_profile": payload.get("signoff_profile"),
                "nav_rinex": normalize_artifact_path(root_dir, payload.get("nav_rinex")),
                "input": normalize_artifact_path(root_dir, payload.get("input")),
                "input_url": normalize_artifact_path(root_dir, payload.get("input_url")),
                "prepare_summary": payload.get("prepare_summary"),
                "products_summary": payload.get("products_summary"),
                "runtime_status": classify_realtime_status(payload.get("realtime_factor")),
                "quality_status": classify_baseline_status(payload.get("p95_baseline_error_m")),
            }
        )

    ppp_products_summaries: list[dict[str, Any]] = []
    for path in sorted(root_dir.glob(args.ppp_products_summary_glob)):
        payload = load_json(path)
        if payload is None:
            continue
        p95_error = payload.get("p95_position_error_m")
        if not isinstance(p95_error, (int, float)):
            p95_error = payload.get("max_position_error_m")
        ppp_products_summaries.append(
            {
                "_path": relative_display(path, root_dir),
                "summary_path": relative_display(path, root_dir),
                "dataset": payload.get("dataset"),
                "run_dir": normalize_artifact_path(root_dir, payload.get("run_dir")),
                "reference_csv": normalize_artifact_path(root_dir, payload.get("reference_csv")),
                "profile": payload.get("products_signoff_profile"),
                "product_presets": payload.get("product_presets"),
                "fetched_product_date": payload.get("fetched_product_date"),
                "ppp_solution_rate_pct": payload.get("ppp_solution_rate_pct"),
                "ppp_converged": payload.get("ppp_converged"),
                "ppp_convergence_time_s": payload.get("ppp_convergence_time_s"),
                "mean_position_error_m": payload.get("mean_position_error_m"),
                "p95_position_error_m": payload.get("p95_position_error_m"),
                "max_position_error_m": payload.get("max_position_error_m"),
                "ionex_corrections": payload.get("ionex_corrections"),
                "dcb_corrections": payload.get("dcb_corrections"),
                "solution_pos": normalize_artifact_path(root_dir, payload.get("solution_pos")),
                "sp3": normalize_artifact_path(root_dir, payload.get("sp3")),
                "clk": normalize_artifact_path(root_dir, payload.get("clk")),
                "ionex": normalize_artifact_path(root_dir, payload.get("ionex")),
                "dcb": normalize_artifact_path(root_dir, payload.get("dcb")),
                "malib_solution_pos": normalize_artifact_path(root_dir, payload.get("malib_solution_pos")),
                "comparison_target": payload.get("comparison_target"),
                "comparison_status": payload.get("comparison_status"),
                "comparison_csv": normalize_artifact_path(root_dir, payload.get("comparison_csv")),
                "comparison_png": normalize_artifact_path(root_dir, payload.get("comparison_png")),
                "common_epoch_pairs": payload.get("common_epoch_pairs"),
                "libgnss_minus_malib_mean_error_m": payload.get("libgnss_minus_malib_mean_error_m"),
                "libgnss_minus_malib_p95_error_m": payload.get("libgnss_minus_malib_p95_error_m"),
                "libgnss_minus_malib_max_error_m": payload.get("libgnss_minus_malib_max_error_m"),
                "quality_status": classify_ppp_products_status(
                    payload.get("ppp_converged"),
                    p95_error,
                    payload.get("ppp_solution_rate_pct"),
                ),
            }
        )

    field_report_summaries: list[dict[str, Any]] = []
    for path in sorted(root_dir.glob(args.field_report_glob)):
        payload = load_json(path)
        if payload is None:
            continue
        if payload.get("tool") not in (None, "field-report"):
            continue
        field_report_summaries.append(summarize_field_report(payload, path, root_dir))

    artifact_manifest_payload = load_json(artifact_manifest_path)
    artifact_manifest = []
    if artifact_manifest_payload is not None:
        bundles = artifact_manifest_payload.get("bundles")
        if isinstance(bundles, list):
            artifact_manifest = bundles
    ppc_spp_policy_suites = build_policy_suite_summaries(artifact_manifest)

    return {
        "title": "libgnss++ web",
        "root": str(root_dir),
        "docs_url": args.docs_url,
        "artifacts": {
            "odaiba_summary": relative_display(odaiba_summary_path, root_dir),
            "lib_pos": relative_display(lib_pos_path, root_dir),
            "rtklib_pos": relative_display(rtklib_pos_path, root_dir),
            "rcv_status": relative_display(args.rcv_status, root_dir) if args.rcv_status else None,
            "artifact_manifest": relative_display(artifact_manifest_path, root_dir) if artifact_manifest else None,
        },
        "odaiba_summary": load_json(odaiba_summary_path),
        "ppc_summaries": ppc_summaries,
        "live_summaries": live_summaries,
        "field_report_summaries": field_report_summaries,
        "robotics_summaries": robotics_summaries,
        "ros2_bag_summaries": ros2_bag_summaries,
        "visibility_summaries": visibility_summaries,
        "moving_base_summaries": moving_base_summaries,
        "ppp_products_summaries": ppp_products_summaries,
        "ppc_spp_policy_suites": ppc_spp_policy_suites,
        "artifact_manifest": artifact_manifest,
        "receiver_status": rcv_status,
    }


def render_html() -> str:
    return Path(__file__).with_name("gnss_web.html").read_text(encoding="utf-8")


def make_handler(args: argparse.Namespace):
    root_dir = args.root.resolve()
    lib_pos_path = resolve_path(args.lib_pos, root_dir, "output/rtk_solution.pos")
    rtklib_pos_path = resolve_path(args.rtklib_pos, root_dir, "output/driving_rtklib_rtk.pos")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *values: Any) -> None:
            return

        def _write(self, body: bytes, content_type: str, status: HTTPStatus = HTTPStatus.OK) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _write_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
            self._write(json.dumps(payload, indent=2, sort_keys=True).encode("utf-8"),
                        "application/json; charset=utf-8", status)

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            query = parse_qs(parsed.query)

            try:
                if parsed.path == "/":
                    self._write(render_html().encode("utf-8"), "text/html; charset=utf-8")
                    return
                if parsed.path == "/api/health":
                    self._write_json({"ok": True})
                    return
                if parsed.path == "/api/overview":
                    self._write_json(build_overview(args))
                    return
                if parsed.path == "/api/status":
                    if args.rcv_status is None:
                        self._write_json({"available": False, "message": "No --rcv-status configured."})
                        return
                    raw = load_json(args.rcv_status)
                    if raw is None:
                        self._write_json(
                            {
                                "available": False,
                                "message": f"Status file unavailable: {args.rcv_status}",
                            }
                        )
                        return
                    self._write_json(
                        {
                            "available": True,
                            "state": raw.get("state"),
                            "pid": raw.get("pid"),
                            "pid_running": raw.get("pid_running"),
                            "uptime_seconds": raw.get("uptime_seconds"),
                            "restart_count": raw.get("restart_count"),
                            "raw": raw,
                        }
                    )
                    return
                if parsed.path == "/api/solution":
                    name = query.get("name", [""])[0]
                    if name == "libgnsspp":
                        self._write_json(build_solution_payload(name, lib_pos_path, root_dir, "libgnss++"))
                        return
                    if name == "rtklib":
                        self._write_json(build_solution_payload(name, rtklib_pos_path, root_dir, "RTKLIB"))
                        return
                    self._write_json({"error": "unknown solution name"}, HTTPStatus.BAD_REQUEST)
                    return
                if parsed.path == "/api/visibility":
                    csv_arg = query.get("path", [""])[0]
                    if not csv_arg:
                        self._write_json({"error": "missing visibility CSV path"}, HTTPStatus.BAD_REQUEST)
                        return
                    try:
                        csv_path = resolve_under_root(root_dir, csv_arg)
                    except ValueError:
                        self._write_json({"error": "visibility CSV path escapes artifact root"}, HTTPStatus.BAD_REQUEST)
                        return
                    if not csv_path.exists():
                        self._write_json({"error": f"visibility CSV not found: {csv_arg}"}, HTTPStatus.NOT_FOUND)
                        return
                    self._write_json(
                        {
                            "available": True,
                            "path": relative_display(csv_path, root_dir),
                            "rows": load_visibility_rows(csv_path),
                        }
                    )
                    return
                if parsed.path == "/api/live-pos":
                    # Stream incremental updates from a live gnss_ppp --live-output file.
                    # Query params: path=<relative .pos path>, after=<GPS TOW float, default 0>
                    pos_arg = query.get("path", [""])[0]
                    after_tow = float(query.get("after", ["0"])[0])
                    if not pos_arg:
                        self._write_json({"error": "missing pos path"}, HTTPStatus.BAD_REQUEST)
                        return
                    try:
                        pos_path = resolve_under_root(root_dir, pos_arg)
                    except ValueError:
                        self._write_json({"error": "pos path escapes artifact root"}, HTTPStatus.BAD_REQUEST)
                        return
                    if not pos_path.exists():
                        self._write_json({"available": False, "points": [], "last_tow": after_tow})
                        return
                    STATUS_INT = {1: "SPP", 2: "DGPS", 3: "FLOAT", 4: "FIXED", 5: "PPP_FLOAT", 6: "PPP_FIXED", 7: "PROPAGATED"}
                    STATUS_COLOR = {"PPP_FIXED": "#2ecc71", "FIXED": "#2ecc71", "PPP_FLOAT": "#f39c12", "FLOAT": "#f39c12"}
                    points: list[dict] = []
                    last_tow = after_tow
                    try:
                        with pos_path.open() as fh:
                            for line in fh:
                                if line.startswith("%"):
                                    continue
                                parts = line.split()
                                if len(parts) < 10:
                                    continue
                                tow = float(parts[1])
                                if tow <= after_tow:
                                    continue
                                status_int = int(parts[8])
                                status = STATUS_INT.get(status_int, "UNKNOWN")
                                points.append({
                                    "tow": round(tow, 3),
                                    "lat": float(parts[5]),
                                    "lon": float(parts[6]),
                                    "height": float(parts[7]),
                                    "status": status,
                                    "color": STATUS_COLOR.get(status, "#e74c3c"),
                                    "satellites": int(parts[9]),
                                })
                                last_tow = tow
                    except OSError:
                        pass
                    self._write_json({"available": True, "points": points, "last_tow": last_tow})
                    return
                if parsed.path == "/api/stream-stats":
                    stats_arg = query.get("path", [""])[0]
                    if not stats_arg:
                        self._write_json({"error": "missing stream stats path"}, HTTPStatus.BAD_REQUEST)
                        return
                    try:
                        stats_path = resolve_under_root(root_dir, stats_arg)
                    except ValueError:
                        self._write_json({"error": "stream stats path escapes artifact root"}, HTTPStatus.BAD_REQUEST)
                        return
                    if not stats_path.exists():
                        self._write_json({"available": False, "error": "stats file not found"})
                        return
                    try:
                        payload = json.loads(stats_path.read_text(encoding="utf-8"))
                    except (OSError, json.JSONDecodeError) as exc:
                        self._write_json({"available": False, "error": str(exc)})
                        return
                    payload["available"] = True
                    payload["path"] = relative_display(stats_path, root_dir)
                    self._write_json(payload)
                    return
                if parsed.path == "/api/moving-base-matches":
                    csv_arg = query.get("path", [""])[0]
                    if not csv_arg:
                        self._write_json({"error": "missing moving-base CSV path"}, HTTPStatus.BAD_REQUEST)
                        return
                    try:
                        csv_path = resolve_under_root(root_dir, csv_arg)
                    except ValueError:
                        self._write_json(
                            {"error": "moving-base CSV path escapes artifact root"},
                            HTTPStatus.BAD_REQUEST,
                        )
                        return
                    if not csv_path.exists():
                        self._write_json(
                            {"error": f"moving-base CSV not found: {csv_arg}"},
                            HTTPStatus.NOT_FOUND,
                        )
                        return
                    self._write_json(
                        {
                            "available": True,
                            "path": relative_display(csv_path, root_dir),
                            "rows": load_moving_base_matches(csv_path),
                        }
                    )
                    return
                if parsed.path == "/artifact":
                    artifact_arg = query.get("path", [""])[0]
                    if not artifact_arg:
                        self._write_json({"error": "missing artifact path"}, HTTPStatus.BAD_REQUEST)
                        return
                    try:
                        artifact_path = resolve_under_root(root_dir, artifact_arg)
                    except ValueError:
                        self._write_json({"error": "artifact path escapes artifact root"}, HTTPStatus.BAD_REQUEST)
                        return
                    if not artifact_path.exists():
                        self._write_json({"error": f"artifact not found: {artifact_arg}"}, HTTPStatus.NOT_FOUND)
                        return
                    suffix = artifact_path.suffix.lower()
                    content_type = {
                        ".png": "image/png",
                        ".jpg": "image/jpeg",
                        ".jpeg": "image/jpeg",
                        ".svg": "image/svg+xml",
                        ".json": "application/json; charset=utf-8",
                        ".pos": "text/plain; charset=utf-8",
                        ".csv": "text/plain; charset=utf-8",
                        ".md": "text/markdown; charset=utf-8",
                        ".txt": "text/plain; charset=utf-8",
                    }.get(suffix, "application/octet-stream")
                    self._write(artifact_path.read_bytes(), content_type)
                    return

                self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            except Exception as exc:  # pragma: no cover - tested through API failure shape
                self._write_json({"error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)

    return Handler


def main() -> int:
    args = parse_args()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(args))
    shutdown_started = False
    previous_handlers: dict[int, Any] = {}

    def request_shutdown(signum: int, _frame: Any) -> None:
        nonlocal shutdown_started
        if shutdown_started:
            return
        shutdown_started = True
        threading.Thread(target=server.shutdown, daemon=True).start()

    for sig in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[sig] = signal.getsignal(sig)
        signal.signal(sig, request_shutdown)

    bound_host, bound_port = server.server_address[:2]
    if args.port_file is not None:
        args.port_file.parent.mkdir(parents=True, exist_ok=True)
        args.port_file.write_text(f"{bound_port}\n", encoding="utf-8")
    print(f"Serving gnss web at http://{bound_host}:{bound_port}", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
