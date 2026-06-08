#!/usr/bin/env python3
"""Aggregate field diagnostics into Markdown and JSON artifacts."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Any

import gnss_doctor
import gnss_ros2_bag_doctor
import gnss_ros2_doctor


ROOT_DIR = Path(__file__).resolve().parents[1]


def rel(path: Path, root_dir: Path) -> str:
    try:
        return str(path.resolve().relative_to(root_dir.resolve()))
    except ValueError:
        return str(path)


def load_json(path: Path) -> dict[str, Any] | None:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def count_status(checks: list[dict[str, Any]]) -> dict[str, int]:
    counts = {"ok": 0, "warn": 0, "missing": 0}
    for item in checks:
        status = str(item.get("status", "warn"))
        counts[status] = counts.get(status, 0) + 1
    return counts


def section_status_from_checks(checks: list[dict[str, Any]]) -> str:
    counts = count_status(checks)
    if counts.get("missing", 0) > 0:
        return "missing"
    if counts.get("warn", 0) > 0:
        return "warn"
    return "ok"


def summarize_robotics_smoke(payload: dict[str, Any], path: Path, root_dir: Path) -> dict[str, Any]:
    status = payload.get("robotics_smoke_status")
    if status not in ("passed", "failed"):
        status = "n/a"
    reasons = payload.get("robotics_smoke_failure_reasons")
    if not isinstance(reasons, list):
        reasons = []
    return {
        "_path": rel(path, root_dir),
        "status": status,
        "profile": payload.get("robotics_smoke_profile") or payload.get("signoff_profile"),
        "dataset": payload.get("dataset"),
        "matched_epochs": payload.get("matched_epochs"),
        "fix_rate_pct": payload.get("fix_rate_pct"),
        "positioning_rate_pct": payload.get("positioning_rate_pct"),
        "realtime_factor": payload.get("realtime_factor"),
        "effective_epoch_rate_hz": payload.get("effective_epoch_rate_hz"),
        "solver_wall_time_s": payload.get("solver_wall_time_s"),
        "failure_reasons": reasons,
    }


def summarize_ros2_bag(payload: dict[str, Any], source: str) -> dict[str, Any]:
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
    raw_binary = topic_status.get("raw_binary") if isinstance(topic_status.get("raw_binary"), dict) else {}
    fix = topic_status.get("fix") if isinstance(topic_status.get("fix"), dict) else {}
    return {
        "_source": source,
        "bag": payload.get("bag"),
        "status": payload.get("status"),
        "diagnostic_depth": payload.get("diagnostic_depth"),
        "message_source": payload.get("message_source"),
        "storage_identifier": payload.get("storage_identifier"),
        "replayable_raw_binary": payload.get("replayable_raw_binary"),
        "message_count": payload.get("message_count"),
        "topic_count": payload.get("topic_count"),
        "duration_s": payload.get("duration_s"),
        "raw_binary_status": raw_binary.get("status"),
        "raw_binary_messages": raw_binary.get("message_count"),
        "fix_status": fix.get("status"),
        "fix_messages": fix.get("message_count"),
        "gap_topic_count": None if payload.get("diagnostic_depth") == "metadata" else len(gap_topics),
        "commands": payload.get("commands") if isinstance(payload.get("commands"), dict) else {},
    }


def collect_robotics_summaries(root_dir: Path, pattern: str) -> list[dict[str, Any]]:
    summaries: list[dict[str, Any]] = []
    for path in sorted(root_dir.glob(pattern)):
        payload = load_json(path)
        if payload is None:
            continue
        if "robotics_smoke_status" not in payload and "robotics_smoke_profile" not in payload:
            continue
        summaries.append(summarize_robotics_smoke(payload, path, root_dir))
    return summaries


def collect_ros2_bag_summaries(root_dir: Path, pattern: str) -> list[dict[str, Any]]:
    summaries: list[dict[str, Any]] = []
    for path in sorted(root_dir.glob(pattern)):
        payload = load_json(path)
        if payload is None:
            continue
        if payload.get("tool") not in (None, "ros2-bag-doctor"):
            continue
        summaries.append(summarize_ros2_bag(payload, rel(path, root_dir)))
    return summaries


def inspect_bags(args: argparse.Namespace) -> list[dict[str, Any]]:
    summaries: list[dict[str, Any]] = []
    for bag in args.bag or []:
        bag_args = argparse.Namespace(
            bag=bag,
            gap_factor=args.gap_factor,
            gap_min_s=args.gap_min_s,
            protocol=args.protocol,
            output_pos=args.bag_output_pos,
            output_kml=args.bag_output_kml,
        )
        payload = gnss_ros2_bag_doctor.build_payload(bag_args)
        summaries.append(summarize_ros2_bag(payload, f"inspected:{bag}"))
    return summaries


def build_actions(payload: dict[str, Any], args: argparse.Namespace) -> list[str]:
    actions: list[str] = []
    setup_counts = payload["setup_doctor"]["status_counts"]
    if setup_counts.get("missing", 0) > 0:
        actions.append("cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j")

    ros2_counts = payload["ros2_doctor"]["status_counts"]
    if ros2_counts.get("warn", 0) > 0 or ros2_counts.get("missing", 0) > 0:
        commands = payload["ros2_doctor"].get("commands", {})
        for key in ("build", "source", "launch", "record"):
            command = commands.get(key)
            if command:
                actions.append(str(command))

    ros2_bags = payload["ros2_bags"]
    if not ros2_bags:
        actions.append(
            "python3 apps/gnss.py ros2-bag-doctor --bag <bag-directory> "
            "--summary-json output/ros2_bag_doctor_summary.json"
        )
    elif any(not item.get("replayable_raw_binary") for item in ros2_bags):
        actions.append("ros2 bag record /gnss/raw_binary /gnss/raw /gnss/fix")

    robotics = payload["robotics_smoke"]
    if not robotics:
        actions.append("python3 apps/gnss.py robotics-smoke --profile realtime")
    elif any(item.get("status") == "failed" for item in robotics):
        actions.append("python3 apps/gnss.py robotics-smoke --profile realtime --summary-json output/robotics_smoke/realtime.json")

    actions.append(f"python3 apps/gnss.py web --port 8085 --root {args.root}")
    return list(dict.fromkeys(actions))


def markdown_status(status: Any) -> str:
    return str(status) if status is not None else "n/a"


def fmt(value: Any, suffix: str = "") -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.3f}{suffix}"
    return f"{value}{suffix}"


def render_check_table(checks: list[dict[str, Any]], limit: int = 12) -> list[str]:
    rows = ["| Status | Check | Detail |", "|---|---|---|"]
    prioritized = sorted(
        checks,
        key=lambda item: {"missing": 0, "warn": 1, "ok": 2}.get(str(item.get("status")), 1),
    )
    for item in prioritized[:limit]:
        detail = str(item.get("detail", "")).replace("|", "\\|")
        rows.append(f"| {item.get('status')} | {item.get('name')} | {detail} |")
    return rows


def render_markdown(payload: dict[str, Any]) -> str:
    setup = payload["setup_doctor"]
    ros2 = payload["ros2_doctor"]
    ros2_bags = payload["ros2_bags"]
    robotics = payload["robotics_smoke"]
    actions = payload["next_actions"]

    lines = [
        "# libgnss++ Field Report",
        "",
        f"- Root: `{payload['root']}`",
        f"- Device: `{payload['device']}`",
        f"- Web UI: `{payload['web_url']}`",
        "",
        "## Executive Summary",
        "",
        "| Area | Status | Detail |",
        "|---|---|---|",
        f"| Setup doctor | {setup['status']} | {setup['status_counts']} |",
        f"| ROS2 readiness | {ros2['status']} | {ros2['status_counts']} |",
        f"| ROS2 bags | {'ready' if ros2_bags and all(item.get('status') == 'ready' for item in ros2_bags) else ('missing' if not ros2_bags else 'partial')} | {len(ros2_bags)} summary item(s) |",
        f"| Robotics realtime smoke | {'passed' if robotics and all(item.get('status') == 'passed' for item in robotics) else ('missing' if not robotics else 'failed')} | {len(robotics)} summary item(s) |",
        "",
        "## Setup Doctor",
        "",
    ]
    lines.extend(render_check_table(setup["checks"]))
    lines.extend(["", "## ROS2 Field Readiness", ""])
    lines.extend(render_check_table(ros2["checks"]))

    lines.extend(["", "## ROS2 Bag Diagnostics", ""])
    if ros2_bags:
        lines.extend(
            [
                "| Source | Status | Depth | Reader | Raw replay | Messages | Topics | Duration | Gaps |",
                "|---|---|---|---|---:|---:|---:|---:|---:|",
            ]
        )
        for item in ros2_bags:
            lines.append(
                f"| `{item.get('_source')}` | {markdown_status(item.get('status'))} | "
                f"{markdown_status(item.get('diagnostic_depth'))} | "
                f"{markdown_status(item.get('message_source'))} | "
                f"{item.get('replayable_raw_binary')} | {fmt(item.get('message_count'))} | "
                f"{fmt(item.get('topic_count'))} | {fmt(item.get('duration_s'), ' s')} | "
                f"{fmt(item.get('gap_topic_count'))} |"
            )
    else:
        lines.append("No ROS2 bag doctor summaries found.")

    lines.extend(["", "## Robotics Realtime Smoke", ""])
    if robotics:
        lines.extend(
            [
                "| Summary | Status | Profile | Realtime | Epoch rate | Fix rate | P95/positioning |",
                "|---|---|---|---:|---:|---:|---:|",
            ]
        )
        for item in robotics:
            lines.append(
                f"| `{item.get('_path')}` | {markdown_status(item.get('status'))} | "
                f"{markdown_status(item.get('profile'))} | {fmt(item.get('realtime_factor'), 'x')} | "
                f"{fmt(item.get('effective_epoch_rate_hz'), ' Hz')} | "
                f"{fmt(item.get('fix_rate_pct'), '%')} | {fmt(item.get('positioning_rate_pct'), '%')} |"
            )
            for reason in item.get("failure_reasons", []):
                lines.append(f"| | why | {reason} | | | | |")
    else:
        lines.append("No robotics smoke summaries found.")

    lines.extend(["", "## Next Commands", ""])
    for command in actions:
        lines.append(f"- `{command}`")

    lines.extend(
        [
            "",
            "## Artifact Paths",
            "",
            f"- Markdown report: `{payload['markdown_report']}`",
            f"- JSON report: `{payload['json_report']}`",
        ]
    )
    return "\n".join(lines) + "\n"


def build_payload(args: argparse.Namespace) -> dict[str, Any]:
    root_dir = args.root.resolve()
    setup_checks = gnss_doctor.collect_checks(root_dir)
    ros2_args = argparse.Namespace(
        device=args.device,
        baud_rate=args.baud_rate,
        protocol=args.protocol,
        frame_id=args.frame_id,
    )
    ros2_payload = gnss_ros2_doctor.build_payload(ros2_args, root_dir)
    ros2_checks = ros2_payload["checks"]

    payload: dict[str, Any] = {
        "tool": "field-report",
        "root": str(root_dir),
        "device": str(args.device),
        "web_url": args.web_url,
        "markdown_report": str(args.out),
        "json_report": str(args.json_out),
        "setup_doctor": {
            "status": section_status_from_checks(setup_checks),
            "status_counts": count_status(setup_checks),
            "checks": setup_checks,
        },
        "ros2_doctor": {
            "status": section_status_from_checks(ros2_checks),
            "status_counts": count_status(ros2_checks),
            "checks": ros2_checks,
            "commands": ros2_payload["commands"],
        },
        "ros2_bags": collect_ros2_bag_summaries(root_dir, args.ros2_bag_summary_glob) + inspect_bags(args),
        "robotics_smoke": collect_robotics_summaries(root_dir, args.robotics_summary_glob),
    }
    payload["next_actions"] = build_actions(payload, args)
    return payload


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description="Build a Markdown/JSON field diagnostic report from libgnss++ doctor summaries.",
    )
    parser.add_argument("--root", type=Path, default=ROOT_DIR, help="Repository/artifact root.")
    parser.add_argument("--out", type=Path, default=Path("output/field_report.md"), help="Markdown report path.")
    parser.add_argument("--json-out", type=Path, default=None, help="JSON report path. Defaults to --out with .json suffix.")
    parser.add_argument("--device", type=Path, default=Path("/dev/ttyUSB0"), help="Receiver serial device for ros2-doctor.")
    parser.add_argument("--baud-rate", type=int, default=115200)
    parser.add_argument("--protocol", choices=("auto", "ubx", "sbf"), default="auto")
    parser.add_argument("--frame-id", default="gnss")
    parser.add_argument("--bag", type=Path, action="append", default=None, help="Optional ROS2 bag directory to inspect now.")
    parser.add_argument("--gap-factor", type=float, default=3.0)
    parser.add_argument("--gap-min-s", type=float, default=1.0)
    parser.add_argument("--bag-output-pos", type=Path, default=Path("output/ros2_bag_replay.pos"))
    parser.add_argument("--bag-output-kml", type=Path, default=Path("output/ros2_bag_replay.kml"))
    parser.add_argument("--ros2-bag-summary-glob", default="output/ros2_bag*_summary.json")
    parser.add_argument("--robotics-summary-glob", default="output/robotics_smoke*/**/*.json")
    parser.add_argument("--web-url", default="http://127.0.0.1:8085")
    parser.add_argument("--json", action="store_true", help="Also print the JSON report to stdout.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.json_out is None:
        args.json_out = args.out.with_suffix(".json")

    payload = build_payload(args)
    markdown = render_markdown(payload)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.json_out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(markdown, encoding="utf-8")
    args.json_out.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(f"Wrote field report: {args.out}")
        print(f"Wrote field report JSON: {args.json_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
