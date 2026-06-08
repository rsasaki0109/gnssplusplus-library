#!/usr/bin/env python3
"""User-facing setup diagnostics for libgnss++."""

from __future__ import annotations

import argparse
import glob
import json
import os
from pathlib import Path
import shutil
import sys


ROOT_DIR = Path(__file__).resolve().parents[1]
EXE_SUFFIX = ".exe" if os.name == "nt" else ""
BUILD_CONFIGS = ("Release", "RelWithDebInfo", "Debug", "MinSizeRel")


def find_binary(root_dir: Path, target_name: str) -> Path | None:
    filename = target_name + EXE_SUFFIX
    build_roots = [
        Path(path)
        for path in sorted(glob.glob(str(root_dir / "build*")))
        if Path(path).is_dir()
    ]
    default_build = root_dir / "build"
    if default_build not in build_roots:
        build_roots.insert(0, default_build)

    candidates: list[Path] = [root_dir / "apps" / filename]
    for build_root in build_roots:
        candidates.append(build_root / "apps" / filename)
        for config in BUILD_CONFIGS:
            candidates.append(build_root / "apps" / config / filename)
            candidates.append(build_root / config / "apps" / filename)
            candidates.append(build_root / config / filename)

    for candidate in candidates:
        if candidate.is_file():
            return candidate

    for build_root in build_roots:
        hits = sorted(build_root.glob(f"**/{filename}"))
        for hit in hits:
            if hit.is_file():
                return hit
    return None


def check(
    checks: list[dict[str, str]],
    name: str,
    ok: bool,
    detail: str,
    *,
    required: bool = False,
) -> None:
    if ok:
        status = "ok"
    elif required:
        status = "missing"
    else:
        status = "warn"
    checks.append({"name": name, "status": status, "detail": detail})


def collect_checks(root_dir: Path) -> list[dict[str, str]]:
    checks: list[dict[str, str]] = []

    check(
        checks,
        "repository root",
        (root_dir / "CMakeLists.txt").exists() and (root_dir / "apps" / "gnss.py").exists(),
        str(root_dir),
        required=True,
    )
    check(
        checks,
        "python",
        sys.version_info >= (3, 10),
        f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}",
        required=True,
    )

    for tool in ("cmake", "python3"):
        path = shutil.which(tool)
        check(checks, tool, path is not None, path or "not on PATH", required=(tool == "cmake"))

    for optional_tool in ("docker", "colcon", "ros2"):
        path = shutil.which(optional_tool)
        check(checks, optional_tool, path is not None, path or "not on PATH")

    for target in ("gnss_solve", "gnss_spp", "gnss_ppp", "gnss_visibility"):
        binary = find_binary(root_dir, target)
        check(
            checks,
            target,
            binary is not None,
            str(binary) if binary is not None else "build with: cmake --build build -j",
            required=True,
        )

    ppc_root = root_dir / "data" / "PPC-Dataset"
    ppc_required = (
        ppc_root / "tokyo" / "run1" / "rover.obs",
        ppc_root / "tokyo" / "run1" / "base.obs",
        ppc_root / "tokyo" / "run1" / "base.nav",
        ppc_root / "tokyo" / "run1" / "reference.csv",
    )
    check(
        checks,
        "bundled PPC-Dataset",
        all(path.exists() for path in ppc_required),
        str(ppc_root),
    )

    for path in (
        root_dir / "docs" / "robotics_quickstart.md",
        root_dir / "docs" / "research_quickstart.md",
        root_dir / "docs" / "dataset_gallery.md",
    ):
        check(checks, path.name, path.exists(), str(path), required=True)

    check(
        checks,
        "ROS2 raw driver",
        (root_dir / "ros2" / "gnss_raw_driver" / "launch" / "gnss_raw_driver.launch.py").exists(),
        str(root_dir / "ros2" / "gnss_raw_driver"),
    )
    check(checks, "Dockerfile", (root_dir / "Dockerfile").exists(), str(root_dir / "Dockerfile"))
    check(checks, "compose.yaml", (root_dir / "compose.yaml").exists(), str(root_dir / "compose.yaml"))

    return checks


def render_text(checks: list[dict[str, str]]) -> str:
    status_width = max(len(item["status"]) for item in checks)
    lines = ["libgnss++ doctor", ""]
    for item in checks:
        lines.append(
            f"[{item['status']:<{status_width}}] {item['name']}: {item['detail']}"
        )
    lines.extend(
        [
            "",
            "Recommended next commands:",
            "  python3 apps/gnss.py robotics-smoke --profile realtime",
            "  python3 apps/gnss.py ros2-doctor --device /dev/ttyUSB0",
            "  python3 apps/gnss.py ros2-bag-doctor --bag <bag-directory>",
            "  python3 apps/gnss.py field-report --out output/field_report.md",
            "  python3 apps/gnss.py web --port 8085 --root .",
            "  python3 -m mkdocs serve",
        ]
    )
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check local libgnss++ setup and print first useful commands."
    )
    parser.add_argument("--root", type=Path, default=ROOT_DIR, help="Repository root to inspect.")
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON.")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Return non-zero when any required check is missing.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root_dir = args.root.resolve()
    checks = collect_checks(root_dir)
    missing_required = [item for item in checks if item["status"] == "missing"]

    payload = {
        "root": str(root_dir),
        "ok": not missing_required,
        "checks": checks,
        "next_commands": [
            "python3 apps/gnss.py robotics-smoke --profile realtime",
            "python3 apps/gnss.py ros2-doctor --device /dev/ttyUSB0",
            "python3 apps/gnss.py ros2-bag-doctor --bag <bag-directory>",
            "python3 apps/gnss.py field-report --out output/field_report.md",
            "python3 apps/gnss.py web --port 8085 --root .",
            "python3 -m mkdocs serve",
        ],
    }

    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(render_text(checks))

    if args.strict and missing_required:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
