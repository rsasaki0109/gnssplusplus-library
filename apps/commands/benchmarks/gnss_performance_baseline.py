#!/usr/bin/env python3
"""Run a reproducible Release SPP/RTK timing baseline.

This command is intentionally a measurement harness.  It records the build,
source, input, command, wall time, and interval timing artifacts, but it does
not claim to implement the hidden Kaggle/GSDC leaderboard score.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import time
from typing import Any

from support.gnss_runtime import application_root, ensure_input_exists

from gnss_performance_report import build_report, write_report


ROOT = application_root(__file__)
SCHEMA_VERSION = "gnss-performance-baseline.v1"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git_revision() -> str | None:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError:
        return None
    revision = result.stdout.strip()
    return revision if result.returncode == 0 and revision else None


def _git_status() -> list[str] | None:
    try:
        result = subprocess.run(
            ["git", "status", "--short"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError:
        return None
    if result.returncode != 0:
        return None
    return [line for line in result.stdout.splitlines() if line]


def _build_metadata() -> dict[str, Any]:
    cache_path = ROOT / "build" / "CMakeCache.txt"
    metadata: dict[str, Any] = {"cache_path": str(cache_path)}
    if not cache_path.is_file():
        metadata["build_type"] = None
        return metadata
    try:
        lines = cache_path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        metadata["build_type"] = None
        return metadata
    values: dict[str, str] = {}
    for line in lines:
        if "=" not in line or line.startswith("//"):
            continue
        key, value = line.split("=", 1)
        if key.endswith(":STRING") or key.endswith(":FILEPATH"):
            values[key.rsplit(":", 1)[0]] = value
    metadata.update(
        {
            "build_type": values.get("CMAKE_BUILD_TYPE"),
            "cxx_compiler": values.get("CMAKE_CXX_COMPILER"),
            "cxx_flags_release": values.get("CMAKE_CXX_FLAGS_RELEASE"),
        }
    )
    return metadata


def _binary_path(target: str) -> Path | None:
    suffix = ".exe" if os.name == "nt" else ""
    candidates = (
        ROOT / "build" / "apps" / f"{target}{suffix}",
        ROOT / "build" / "Release" / "apps" / f"{target}{suffix}",
        ROOT / "build" / "apps" / "Release" / f"{target}{suffix}",
    )
    return next((path for path in candidates if path.is_file()), None)


def _relative_or_absolute(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT.resolve()))
    except ValueError:
        return str(path.resolve())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a Release SPP/RTK baseline and write interval timing artifacts."
    )
    parser.add_argument("--mode", choices=("spp", "rtk"), required=True)
    parser.add_argument("--obs", type=Path, required=True, help="Rover observation RINEX")
    parser.add_argument("--nav", type=Path, required=True, help="Navigation RINEX")
    parser.add_argument("--base", type=Path, help="Base observation RINEX (RTK mode)")
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--max-epochs", type=int, default=0)
    parser.add_argument("--interval-s", type=float, default=60.0)
    parser.add_argument("--label", default="release-baseline")
    parser.add_argument(
        "--allow-non-release",
        action="store_true",
        help="Allow a missing/non-Release CMake cache (recorded in the manifest)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.max_epochs < 0:
        raise SystemExit("--max-epochs must be non-negative")
    if not math.isfinite(args.interval_s) or args.interval_s <= 0.0:
        raise SystemExit("--interval-s must be positive")
    if args.mode == "rtk" and args.base is None:
        raise SystemExit("--base is required in rtk mode")

    obs_path = args.obs.resolve()
    nav_path = args.nav.resolve()
    base_path = args.base.resolve() if args.base is not None else None
    ensure_input_exists(obs_path, "rover observation file", ROOT)
    ensure_input_exists(nav_path, "navigation file", ROOT)
    if base_path is not None:
        ensure_input_exists(base_path, "base observation file", ROOT)

    build = _build_metadata()
    if build.get("build_type") != "Release" and not args.allow_non_release:
        raise SystemExit(
            "Release baseline requires build/CMakeCache.txt with "
            "CMAKE_BUILD_TYPE=Release; use --allow-non-release only for diagnostics"
        )

    binary_target = "gnss_spp" if args.mode == "spp" else "gnss_solve"
    binary = _binary_path(binary_target)
    build["solver_binary"] = (
        {"path": str(binary), "sha256": _sha256(binary)}
        if binary is not None
        else {"path": None, "sha256": None}
    )

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    timing_path = out_dir / "epoch_timing.csv"
    report_json_path = out_dir / "performance_summary.json"
    report_csv_path = out_dir / "performance_intervals.csv"
    solver_log_path = out_dir / "solver.log"
    position_path = out_dir / "solution.pos"
    summary_path = out_dir / "solver_summary.json"

    command = [
        sys.executable,
        str(ROOT / "apps" / "gnss.py"),
        "spp" if args.mode == "spp" else "solve",
    ]
    if args.mode == "spp":
        command.extend(
            [
                "--obs",
                str(obs_path),
                "--nav",
                str(nav_path),
                "--out",
                str(position_path),
                "--summary-json",
                str(summary_path),
                "--timing-csv",
                str(timing_path),
                "--quiet",
            ]
        )
    else:
        assert base_path is not None
        command.extend(
            [
                "--rover",
                str(obs_path),
                "--base",
                str(base_path),
                "--nav",
                str(nav_path),
                "--out",
                str(position_path),
                "--no-kml",
                "--timing-csv",
                str(timing_path),
            ]
        )
    if args.max_epochs > 0:
        command.extend(["--max-epochs", str(args.max_epochs)])

    started = time.perf_counter()
    result = subprocess.run(
        command,
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    solver_wall_time_s = time.perf_counter() - started
    solver_log_path.write_text(
        "$ " + " ".join(command) + "\n" + result.stdout + result.stderr,
        encoding="utf-8",
    )

    report_error: str | None = None
    report: dict[str, Any] | None = None
    if timing_path.is_file():
        try:
            report = build_report(timing_path, args.interval_s)
            write_report(report, report_json_path, report_csv_path)
        except (OSError, ValueError) as exc:
            report_error = str(exc)

    inputs: dict[str, Any] = {
        "obs": {"path": _relative_or_absolute(obs_path), "sha256": _sha256(obs_path)},
        "nav": {"path": _relative_or_absolute(nav_path), "sha256": _sha256(nav_path)},
    }
    if base_path is not None:
        inputs["base"] = {
            "path": _relative_or_absolute(base_path),
            "sha256": _sha256(base_path),
        }

    artifacts: dict[str, Any] = {}
    for path in (
        position_path,
        summary_path,
        timing_path,
        report_json_path,
        report_csv_path,
        solver_log_path,
    ):
        if path.is_file():
            artifacts[path.name] = {"path": str(path), "sha256": _sha256(path)}

    manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "label": args.label,
        "mode": args.mode,
        "max_epochs": args.max_epochs,
        "interval_s": args.interval_s,
        "decision": "pass" if result.returncode == 0 and report_error is None else "fail",
        "return_code": result.returncode,
        "command": command,
        "solver_wall_time_s": round(solver_wall_time_s, 6),
        "source_revision": _git_revision(),
        "source_status": _git_status(),
        "build": build,
        "platform": {
            "system": platform.system(),
            "release": platform.release(),
            "machine": platform.machine(),
            "python": platform.python_version(),
        },
        "inputs": inputs,
        "artifacts": artifacts,
        "performance_report": str(report_json_path) if report is not None else None,
        "performance_report_error": report_error,
        "official_score": {
            "available": False,
            "reason": (
                "The repository has no Kaggle/GSDC hidden-test evaluator or official "
                "submission scoring contract; this manifest records local timing only."
            ),
        },
    }
    manifest_path = out_dir / "baseline_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(f"Performance baseline {manifest['decision']}: {manifest_path}")
    return 0 if manifest["decision"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
