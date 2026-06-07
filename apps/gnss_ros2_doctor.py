#!/usr/bin/env python3
"""ROS2 field-readiness diagnostics for libgnss++ receiver workflows."""

from __future__ import annotations

import argparse
import glob
import json
import os
from pathlib import Path
import shutil
import stat
from typing import Any

try:
    import grp
except ImportError:  # pragma: no cover - Windows fallback
    grp = None  # type: ignore[assignment]


ROOT_DIR = Path(__file__).resolve().parents[1]
SERIAL_GLOBS = ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/serial/by-id/*")


def status_for(ok: bool, required: bool) -> str:
    if ok:
        return "ok"
    return "missing" if required else "warn"


def add_check(
    checks: list[dict[str, Any]],
    name: str,
    ok: bool,
    detail: str,
    *,
    required: bool = False,
    hint: str | None = None,
) -> None:
    item: dict[str, Any] = {
        "name": name,
        "status": status_for(ok, required),
        "detail": detail,
    }
    if hint and not ok:
        item["hint"] = hint
    checks.append(item)


def group_names() -> list[str]:
    if os.name == "nt" or grp is None:
        return []
    names: list[str] = []
    for gid in os.getgroups():
        try:
            names.append(grp.getgrgid(gid).gr_name)
        except KeyError:
            names.append(str(gid))
    return sorted(set(names))


def serial_candidates() -> list[str]:
    candidates: list[str] = []
    for pattern in SERIAL_GLOBS:
        candidates.extend(glob.glob(pattern))
    return sorted(set(candidates))


def describe_device(path: Path) -> tuple[bool, str, str | None]:
    if not path.exists():
        candidates = serial_candidates()
        hint = f"Detected serial devices: {', '.join(candidates)}" if candidates else None
        return False, f"{path} not found", hint

    readable_writable = os.access(path, os.R_OK | os.W_OK)
    try:
        mode = stat.filemode(path.stat().st_mode)
        group = grp.getgrgid(path.stat().st_gid).gr_name if os.name != "nt" and grp is not None else "n/a"
    except (OSError, KeyError):
        mode = "unknown"
        group = "unknown"

    user_groups = group_names()
    detail = (
        f"{path} exists, access={'rw' if readable_writable else 'blocked'}, "
        f"mode={mode}, group={group}, user_groups={','.join(user_groups) or 'n/a'}"
    )
    hint = None
    if not readable_writable and os.name != "nt":
        if group and group not in user_groups:
            hint = f"Add the user to `{group}` or `dialout`, then log out/in."
        else:
            hint = f"Temporary debug only: sudo chmod a+rw {path}"
    return readable_writable, detail, hint


def collect_checks(root_dir: Path, device: Path) -> list[dict[str, Any]]:
    checks: list[dict[str, Any]] = []
    ros2_dir = root_dir / "ros2"
    package_dir = ros2_dir / "gnss_raw_driver"
    install_dir = ros2_dir / "install"

    add_check(
        checks,
        "repository root",
        (root_dir / "apps" / "gnss.py").exists() and (root_dir / "CMakeLists.txt").exists(),
        str(root_dir),
        required=True,
    )
    add_check(
        checks,
        "ros2 executable",
        shutil.which("ros2") is not None,
        shutil.which("ros2") or "not on PATH",
        hint="Source your ROS2 distro setup, for example: source /opt/ros/humble/setup.bash",
    )
    add_check(
        checks,
        "colcon executable",
        shutil.which("colcon") is not None,
        shutil.which("colcon") or "not on PATH",
        hint="Install colcon or source an environment that provides it.",
    )
    add_check(
        checks,
        "ROS_DISTRO",
        bool(os.environ.get("ROS_DISTRO")),
        os.environ.get("ROS_DISTRO", "not set"),
        hint="ROS2 is probably not sourced in this shell.",
    )
    add_check(
        checks,
        "AMENT_PREFIX_PATH",
        bool(os.environ.get("AMENT_PREFIX_PATH")),
        os.environ.get("AMENT_PREFIX_PATH", "not set"),
        hint="Run `source ros2/install/setup.bash` after building the package.",
    )
    add_check(
        checks,
        "gnss_raw_driver source",
        (package_dir / "package.xml").exists(),
        str(package_dir),
        required=True,
    )
    add_check(
        checks,
        "launch file",
        (package_dir / "launch" / "gnss_raw_driver.launch.py").exists(),
        str(package_dir / "launch" / "gnss_raw_driver.launch.py"),
        required=True,
    )
    add_check(
        checks,
        "custom messages",
        (package_dir / "msg" / "GnssRawEpoch.msg").exists()
        and (package_dir / "msg" / "GnssRawObservation.msg").exists(),
        str(package_dir / "msg"),
        required=True,
    )
    add_check(
        checks,
        "install setup",
        (install_dir / "setup.bash").exists(),
        str(install_dir / "setup.bash"),
        hint="Build with: cd ros2 && colcon build --symlink-install --packages-select gnss_raw_driver",
    )
    add_check(
        checks,
        "driver node binary",
        (install_dir / "gnss_raw_driver" / "lib" / "gnss_raw_driver" / "gnss_raw_driver_node").exists(),
        str(install_dir / "gnss_raw_driver" / "lib" / "gnss_raw_driver" / "gnss_raw_driver_node"),
        hint="Build with: cd ros2 && colcon build --symlink-install --packages-select gnss_raw_driver",
    )
    add_check(
        checks,
        "bag processor binary",
        (install_dir / "gnss_raw_driver" / "lib" / "gnss_raw_driver" / "gnss_bag_processor_node").exists(),
        str(install_dir / "gnss_raw_driver" / "lib" / "gnss_raw_driver" / "gnss_bag_processor_node"),
        hint="Build with: cd ros2 && colcon build --symlink-install --packages-select gnss_raw_driver",
    )

    ok, detail, hint = describe_device(device)
    add_check(checks, "serial device", ok, detail, hint=hint)
    return checks


def launch_command(args: argparse.Namespace) -> str:
    return (
        "ros2 launch gnss_raw_driver gnss_raw_driver.launch.py "
        f"device:={args.device} "
        f"baud_rate:={args.baud_rate} "
        f"protocol:={args.protocol} "
        f"frame_id:={args.frame_id} "
        "publish_raw_binary:=true"
    )


def build_payload(args: argparse.Namespace, root_dir: Path) -> dict[str, Any]:
    checks = collect_checks(root_dir, args.device)
    missing_required = [item for item in checks if item["status"] == "missing"]
    warn = [item for item in checks if item["status"] == "warn"]
    commands = {
        "build": "cd ros2 && colcon build --symlink-install --packages-select gnss_raw_driver",
        "source": "source ros2/install/setup.bash",
        "launch": launch_command(args),
        "record": "ros2 bag record /gnss/raw_binary /gnss/raw /gnss/fix",
        "topic_list": "ros2 topic list | grep gnss",
        "fix_once": "ros2 topic echo --once /gnss/fix",
        "fix_rate": "ros2 topic hz /gnss/fix",
        "raw_once": "ros2 topic echo --once /gnss/raw",
    }
    hints = [item["hint"] for item in checks if item.get("hint")]
    return {
        "root": str(root_dir),
        "device": str(args.device),
        "baud_rate": args.baud_rate,
        "protocol": args.protocol,
        "frame_id": args.frame_id,
        "ok": not missing_required,
        "warnings": len(warn),
        "checks": checks,
        "commands": commands,
        "hints": hints,
    }


def render_text(payload: dict[str, Any]) -> str:
    checks = payload["checks"]
    status_width = max(len(item["status"]) for item in checks)
    lines = ["libgnss++ ROS2 doctor", ""]
    for item in checks:
        lines.append(f"[{item['status']:<{status_width}}] {item['name']}: {item['detail']}")
        if item.get("hint"):
            lines.append(f"{'':<{status_width + 3}} hint: {item['hint']}")
    commands = payload["commands"]
    lines.extend(
        [
            "",
            "Next commands:",
            f"  {commands['build']}",
            f"  {commands['source']}",
            f"  {commands['launch']}",
            f"  {commands['record']}",
            "",
            "Debug commands:",
            f"  {commands['topic_list']}",
            f"  {commands['fix_once']}",
            f"  {commands['fix_rate']}",
            f"  {commands['raw_once']}",
        ]
    )
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check ROS2 GNSS receiver readiness and print launch/debug commands."
    )
    parser.add_argument("--root", type=Path, default=ROOT_DIR, help="Repository root to inspect.")
    parser.add_argument("--device", type=Path, default=Path("/dev/ttyUSB0"))
    parser.add_argument("--baud-rate", type=int, default=115200)
    parser.add_argument("--protocol", choices=("auto", "ubx", "sbf"), default="auto")
    parser.add_argument("--frame-id", default="gnss")
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON.")
    parser.add_argument("--strict", action="store_true", help="Return non-zero when required checks are missing.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root_dir = args.root.resolve()
    payload = build_payload(args, root_dir)
    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(render_text(payload))
    if args.strict and not payload["ok"]:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
