#!/usr/bin/env python3
"""Build the upstream urban bundle and run the frozen R8 geofence sign-off."""

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

ROOT = application_root(__file__)
DISPATCHER = ROOT / "apps/gnss.py"
DEFAULT_PROFILE = ROOT / "configs/benchmarks/integrity_geofence_r8.json"
SCHEMA_VERSION = "libgnsspp.integrity_geofence.workflow.v1"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME", "gnss integrity-geofence-workflow"))
    parser.add_argument("--phase", choices=("development", "holdout"), required=True)
    parser.add_argument("--data-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--mode", choices=("smoke", "full"), default="smoke")
    args = parser.parse_args(argv)
    if args.phase == "holdout" and args.mode != "full":
        parser.error("sealed holdout requires --mode full")
    return args


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def run_step(argv: list[str], log_path: Path) -> dict[str, Any]:
    with log_path.open("a", encoding="utf-8") as log:
        log.write("\n$ " + shlex.join(argv) + "\n"); log.flush()
        completed = subprocess.run(argv, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=False)
    return {"argv": argv, "exit_status": completed.returncode, "status": "passed" if completed.returncode == 0 else "failed"}


def verify_holdout(data_dir: Path, profile: dict[str, Any]) -> None:
    for name, expected in profile["sealed_holdout"]["inputs"].items():
        path = data_dir / name
        if not path.is_file() or path.stat().st_size != int(expected["bytes"]) or sha256_file(path) != expected["sha256"]:
            raise ValueError(f"sealed holdout input mismatch: {path}")


def command(*parts: str | Path) -> list[str]:
    return [sys.executable, str(DISPATCHER), *[str(value) for value in parts]]


def run(args: argparse.Namespace) -> int:
    output = args.output_dir.resolve(); workflow_manifest = output / "workflow_manifest.json"
    if args.phase == "holdout" and workflow_manifest.exists():
        print(f"Refusing to rerun opened R8 holdout: {workflow_manifest}", file=sys.stderr); return 2
    output.mkdir(parents=True, exist_ok=True)
    log_path = output / "workflow.log"; log_path.write_text("libgnss++ R8 workflow\n", encoding="utf-8")
    profile_path = args.profile.resolve(); profile = json.loads(profile_path.read_text(encoding="utf-8"))
    if args.phase == "holdout":
        if profile.get("release_state") != "sealed":
            print("R8 holdout profile is not sealed", file=sys.stderr); return 2
        try: verify_holdout(args.data_dir.resolve(), profile)
        except ValueError as exc:
            print(str(exc), file=sys.stderr); return 2
    manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION, "phase": args.phase, "mode": args.mode,
        "status": "running", "profile_sha256": sha256_file(profile_path), "steps": [],
    }
    workflow_manifest.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    upstream = output / "upstream"
    bundle_command = command(
        "urban-continuity-bundle", "--data-dir", args.data_dir.resolve(),
        "--reference-csv", args.data_dir.resolve() / "reference.csv", "--output-dir", upstream,
        *(["--max-epochs", "3500"] if args.mode == "smoke" else []),
    )
    step = run_step(bundle_command, log_path); manifest["steps"].append(step)
    if step["status"] != "passed":
        manifest.update(status="failed", failure_reason="upstream_quality_bundle_failed")
        workflow_manifest.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"); return 1
    signoff_command = command(
        "integrity-geofence-signoff", "--role", "development" if args.phase == "development" else "sealed_holdout",
        "--rtk-pos", upstream / "rtk.pos", "--fused-pos", upstream / "fused.pos",
        "--reference-csv", args.data_dir.resolve() / "reference.csv",
        "--upstream-manifest", upstream / "manifest.json", "--output-dir", output / "decision",
        *(["--max-reference-epochs", "3500"] if args.mode == "smoke" else []),
        *(["--profile", profile_path] if args.phase == "holdout" else []),
    )
    step = run_step(signoff_command, log_path); manifest["steps"].append(step)
    manifest.update(status=step["status"], exit_status=step["exit_status"])
    workflow_manifest.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"R8 integrity/geofence workflow: {manifest['status']}")
    print(f"Manifest: {workflow_manifest}")
    return int(step["exit_status"])


def main(argv: list[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
