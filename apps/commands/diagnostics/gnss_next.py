#!/usr/bin/env python3
"""Recommend one concrete next step from the user's local progress."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys


GOALS = {
    "spp": {
        "label": "single-receiver positioning (SPP)",
        "command": (
            "{cli} spp --obs data/rover_static.obs "
            "--nav data/navigation_static.nav --out output/spp_solution.pos"
        ),
        "guide": "docs/quickstart.md",
        "inputs": "rover observation RINEX and navigation RINEX",
        "result": "output/spp_solution.pos",
        "entrypoint": "spp",
        "required": ("data/rover_static.obs", "data/navigation_static.nav"),
    },
    "rtk": {
        "label": "rover/base positioning (RTK)",
        "command": (
            "{cli} solve "
            "--rover data/short_baseline/TSK200JPN_R_20240010000_01D_30S_MO.rnx "
            "--base data/short_baseline/TSKB00JPN_R_20240010000_01D_30S_MO.rnx "
            "--nav data/short_baseline/BRDC00IGS_R_20240010000_01D_MN.rnx "
            "--mode static --out output/rtk_solution.pos"
        ),
        "guide": "docs/use_cases/rtklib_migration.md",
        "inputs": "rover, base, and navigation RINEX",
        "result": "output/rtk_solution.pos",
        "entrypoint": "solve",
        "required": (
            "data/short_baseline/TSK200JPN_R_20240010000_01D_30S_MO.rnx",
            "data/short_baseline/TSKB00JPN_R_20240010000_01D_30S_MO.rnx",
            "data/short_baseline/BRDC00IGS_R_20240010000_01D_MN.rnx",
        ),
    },
    "ppp": {
        "label": "precise point positioning (PPP)",
        "command": (
            "{cli} ppp --static --obs data/rover_static.obs "
            "--nav data/navigation_static.nav --out output/ppp_solution.pos"
        ),
        "guide": "docs/quickstart.md",
        "inputs": "rover observation RINEX plus navigation or precise products",
        "result": "output/ppp_solution.pos",
        "entrypoint": "ppp",
        "required": ("data/rover_static.obs", "data/navigation_static.nav"),
    },
    "robotics": {
        "label": "ROS2 and robotics integration",
        "command": "{cli} ros2-doctor --json-out output/ros2_doctor.json",
        "guide": "docs/use_cases/ros2.md",
        "inputs": "a ROS2 environment, receiver, or recorded bag",
    },
    "qzss": {
        "label": "QZSS L6 / CLAS / MADOCA",
        "command": "{cli} qzss-l6-info --help",
        "guide": "docs/use_cases/qzss_l6.md",
        "inputs": "raw QZSS L6, Compact SSR, or matching RINEX",
    },
    "cpp": {
        "label": "C++20 library integration",
        "command": "cmake --build build --parallel 2",
        "guide": "docs/interfaces.md",
        "inputs": "a configured source build and a C++20 application",
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description=(
            "Inspect local first-run progress and recommend exactly one next command. "
            "No usage data is transmitted or stored."
        ),
    )
    parser.add_argument(
        "--goal",
        choices=tuple(GOALS),
        help="Choose a route after the offline demo succeeds.",
    )
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path.cwd(),
        help="Workspace containing output/ (default: current directory).",
    )
    parser.add_argument(
        "--format",
        choices=("text", "json"),
        default="text",
        help="Output format (default: text).",
    )
    return parser.parse_args()


def valid_demo_summary(path: Path) -> bool:
    if not path.is_file():
        return False
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    if not isinstance(payload, dict):
        return False
    demo = payload.get("demo")
    return (
        isinstance(demo, dict)
        and demo.get("schema_version") == "self-contained-demo.v1"
        and payload.get("processed_epochs") == 8
        and payload.get("valid_solutions") == 8
    )


def count_position_epochs(path: Path) -> int:
    """Count data rows in a libgnss++/RTKLIB-style position output."""
    if not path.is_file():
        return 0
    try:
        return sum(
            1
            for line in path.read_text(encoding="utf-8", errors="replace").splitlines()
            if line.strip()
            and not line.lstrip().startswith(("%", "#"))
        )
    except OSError:
        return 0


def completed_result(
    workspace: Path,
    goal: str | None,
) -> tuple[str, Path, int] | None:
    """Return the selected or newest valid standard positioning result."""
    goal_ids = (goal,) if goal is not None else tuple(GOALS)
    candidates: list[tuple[int, str, Path, int]] = []
    for goal_id in goal_ids:
        route = GOALS[goal_id]
        result_relative = route.get("result")
        if not isinstance(result_relative, str):
            continue
        result_path = workspace / result_relative
        epochs = count_position_epochs(result_path)
        if epochs < 1:
            continue
        try:
            modified_ns = result_path.stat().st_mtime_ns
        except OSError:
            continue
        candidates.append((modified_ns, goal_id, result_path, epochs))
    if not candidates:
        return None
    _, goal_id, result_path, epochs = max(candidates)
    return goal_id, result_path, epochs


def cli_prefix(workspace: Path) -> str:
    if (workspace / "apps" / "gnss.py").is_file():
        if os.name == "nt":
            return "py apps/gnss.py"
        return "python3 apps/gnss.py"
    return "gnss"


def open_command(path: str) -> str:
    if os.name == "nt":
        return f"Start-Process {path}"
    if sys.platform == "darwin":
        return f"open {path}"
    return f"xdg-open {path}"


def build_payload(workspace: Path, goal: str | None) -> dict[str, object]:
    workspace = workspace.expanduser().resolve()
    demo_summary = workspace / "output" / "self-contained-demo" / "demo_summary.json"
    demo_complete = valid_demo_summary(demo_summary)
    cli = cli_prefix(workspace)
    result = completed_result(workspace, goal) if demo_complete else None

    if not demo_complete:
        recommendation = {
            "id": "run-offline-demo",
            "label": "verify the installation with the offline PPP demo",
            "command": f"{cli} demo --output-dir output/self-contained-demo",
            "guide": "docs/self_contained_demo.md",
        }
        stage = "first-run"
    elif result is not None:
        detected_goal, result_path, epochs = result
        relative_result = result_path.relative_to(workspace).as_posix()
        kml_relative = Path(relative_result).with_suffix(".kml").as_posix()
        kml_path = workspace / kml_relative
        kml_ready = kml_path.is_file() and kml_path.stat().st_size > 0
        if kml_ready:
            recommendation = {
                "id": f"open-{detected_goal}-result",
                "label": f"open the completed {detected_goal.upper()} KML track",
                "command": open_command(kml_relative),
                "guide": GOALS[detected_goal]["guide"],
                "result": relative_result,
                "kml": kml_relative,
                "epochs": epochs,
            }
        else:
            recommendation = {
                "id": f"inspect-{detected_goal}-result",
                "label": f"inspect the completed {detected_goal.upper()} result as KML",
                "command": (
                    f"{cli} pos2kml {relative_result} {kml_relative} --status all"
                ),
                "guide": GOALS[detected_goal]["guide"],
                "result": relative_result,
                "epochs": epochs,
            }
        stage = "inspect-result"
    elif goal is None:
        recommendation = {
            "id": "choose-goal",
            "label": "choose the route closest to your own data",
            "command": f"{cli} next --goal spp",
            "guide": "docs/use_cases.md",
        }
        stage = "choose-goal"
    else:
        route = GOALS[goal]
        required = route.get("required", ())
        assert isinstance(required, tuple)
        missing_inputs = [
            relative_path
            for relative_path in required
            if not (workspace / relative_path).is_file()
        ]
        if missing_inputs:
            recommendation = {
                "id": f"prepare-{goal}-inputs",
                "label": f"prepare the inputs for {route['label']}",
                "command": f"{cli} help {route['entrypoint']}",
                "guide": route["guide"],
                "inputs": route["inputs"],
                "inputs_ready": False,
                "missing_inputs": missing_inputs,
            }
        else:
            recommendation = {
                "id": goal,
                "label": route["label"],
                "command": route["command"].format(cli=cli),
                "guide": route["guide"],
                "inputs": route["inputs"],
                "inputs_ready": True,
            }
        stage = "apply-to-data"

    return {
        "schema_version": "gnss-next.v2",
        "stage": stage,
        "workspace": str(workspace),
        "demo": {
            "complete": demo_complete,
            "summary": str(demo_summary),
        },
        "selected_goal": goal,
        "detected_goal": result[0] if result is not None else None,
        "recommendation": recommendation,
        "available_goals": [
            {"id": goal_id, "label": route["label"]}
            for goal_id, route in GOALS.items()
        ],
        "privacy": "Local artifact inspection only; no usage data is transmitted or stored.",
    }


def format_text(payload: dict[str, object]) -> str:
    recommendation = payload["recommendation"]
    assert isinstance(recommendation, dict)
    lines = [
        f"Progress: {payload['stage']}",
        f"Next: {recommendation['label']}",
        "",
        f"  {recommendation['command']}",
        "",
    ]
    if "inputs" in recommendation:
        lines.append(f"Inputs: {recommendation['inputs']}")
    if "result" in recommendation:
        lines.append(f"Result: {recommendation['result']} ({recommendation['epochs']} epochs)")
    if recommendation.get("missing_inputs"):
        lines.append("Missing:")
        for relative_path in recommendation["missing_inputs"]:
            lines.append(f"  {relative_path}")
    lines.append(f"Guide:  {recommendation['guide']}")

    if payload["stage"] == "choose-goal":
        lines.extend(["", "Other goals:"])
        for route in payload["available_goals"]:
            assert isinstance(route, dict)
            lines.append(f"  --goal {route['id']:<9} {route['label']}")
    lines.extend(["", str(payload["privacy"])])
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    payload = build_payload(args.workspace, args.goal)
    if args.format == "json":
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(format_text(payload))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
