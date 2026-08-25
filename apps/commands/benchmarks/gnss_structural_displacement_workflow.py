#!/usr/bin/env python3
"""Run the frozen R7 Tsukuba structural-displacement workflow."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
from typing import Any

from support.gnss_runtime import application_root


ROOT_DIR = application_root(__file__)
DISPATCHER = ROOT_DIR / "apps" / "gnss.py"
DEFAULT_PROFILE = ROOT_DIR / "configs/benchmarks/structural_displacement_r7_tsukuba.json"
SCHEMA_VERSION = "libgnsspp.structural_displacement.workflow.v1"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss structural-displacement-workflow")
    )
    parser.add_argument("--phase", choices=("development", "holdout"), required=True)
    parser.add_argument("--mode", choices=("smoke", "full"), default="smoke")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--cache-dir", type=Path, required=True)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--offline", action="store_true")
    parser.add_argument(
        "--snapshot-dir", type=Path,
        help="camera snapshot evidence directory forwarded to the sign-off",
    )
    args = parser.parse_args(argv)
    if args.phase == "holdout" and args.mode != "full":
        parser.error("the sealed holdout may only be opened in full mode")
    return args


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def run_step(argv: list[str], log_path: Path) -> dict[str, Any]:
    with log_path.open("a", encoding="utf-8") as log:
        log.write("\n$ " + shlex.join(argv) + "\n")
        log.flush()
        completed = subprocess.run(argv, cwd=ROOT_DIR, stdout=log, stderr=subprocess.STDOUT, check=False)
    return {"argv": argv, "exit_status": completed.returncode, "status": "passed" if completed.returncode == 0 else "failed"}


def command(*parts: str | Path) -> list[str]:
    return [sys.executable, str(DISPATCHER), *[str(part) for part in parts]]


def run(args: argparse.Namespace) -> int:
    output_dir = args.output_dir.resolve()
    manifest_path = output_dir / "workflow_manifest.json"
    if args.phase == "holdout" and manifest_path.exists():
        print(f"Refusing to rerun opened R7 holdout: {manifest_path}", file=sys.stderr)
        return 2
    output_dir.mkdir(parents=True, exist_ok=True)
    log_path = output_dir / "workflow.log"
    log_path.write_text("libgnss++ R7 structural-displacement workflow\n", encoding="utf-8")
    profile_path = args.profile.resolve()
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    if args.phase == "holdout" and profile.get("release_state") != "sealed":
        print("R7 holdout profile is not sealed", file=sys.stderr)
        return 2
    datasets = (
        ["r7-development-day1", "r7-development-day2", "r7-development-day3"]
        if args.phase == "development" else ["r7-holdout-day4"]
    )
    manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "phase": args.phase,
        "mode": args.mode,
        "status": "running",
        "profile": {"path": str(profile_path), "sha256": sha256_file(profile_path), "content": profile},
        "steps": [],
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    bundles = []
    for index, dataset in enumerate(datasets, start=1):
        bundle = output_dir / f"day{index}"
        bundles.append(bundle)
        argv = command(
            "japan-static-survey", "--dataset", dataset, "--output-dir", bundle,
            "--cache-dir", args.cache_dir.resolve(), "--mode", args.mode,
            *(["--offline"] if args.offline else []),
        )
        step = run_step(argv, log_path)
        manifest["steps"].append(step)
        manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if step["status"] != "passed":
            manifest["status"] = "failed"
            manifest["failure_reason"] = f"upstream_day{index}"
            manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            return 1
    signoff_path = output_dir / "signoff.json"
    argv = command(
        "structural-displacement-signoff", "--role",
        "development" if args.phase == "development" else "sealed_holdout",
        *[part for bundle in bundles for part in ("--bundle", bundle)],
        "--output", signoff_path,
        *(["--profile", profile_path] if args.phase == "holdout" else []),
        *(["--snapshot-dir", args.snapshot_dir.resolve()] if args.snapshot_dir else []),
    )
    step = run_step(argv, log_path)
    manifest["steps"].append(step)
    manifest["signoff"] = {
        "path": str(signoff_path), "exists": signoff_path.is_file(),
        "sha256": sha256_file(signoff_path) if signoff_path.is_file() else None,
    }
    manifest["status"] = step["status"]
    manifest["exit_status"] = step["exit_status"]
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"R7 structural-displacement workflow: {manifest['status']}")
    print(f"Manifest: {manifest_path}")
    return int(step["exit_status"])


def main(argv: list[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
