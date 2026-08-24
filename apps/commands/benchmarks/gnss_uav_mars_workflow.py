#!/usr/bin/env python3
"""Run the frozen R6 MARS-LVIG container-to-flight-decision workflow."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
DEFAULT_PROFILE = ROOT / "configs" / "benchmarks" / "uav_r6_mars_lvig_development.json"
SCHEMA_VERSION = "uav-mars-workflow.v1"


def fail(message: str) -> "NoReturn":
    raise SystemExit(message)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_profile(path: Path) -> dict[str, object]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        fail(f"invalid R6 profile: {exc}")
    if not isinstance(payload, dict) or payload.get("schema_version") != "uav-r6-profile.v1":
        fail("R6 profile schema is not uav-r6-profile.v1")
    return payload


def validate_navigation(path: Path, profile: dict[str, object], role: str) -> dict[str, object]:
    products = profile.get("navigation_products")
    product = dict(products.get(role, {})) if isinstance(products, dict) else {}
    if not product:
        fail(f"R6 profile has no {role} navigation product")
    if not path.is_file():
        fail(f"missing navigation product: {path}")
    actual_bytes = path.stat().st_size
    actual_hash = sha256(path)
    if actual_bytes != int(product.get("uncompressed_bytes", -1)):
        fail("navigation product byte count does not match the frozen profile")
    if actual_hash != product.get("uncompressed_sha256"):
        fail("navigation product SHA-256 does not match the frozen profile")
    return {
        "path": str(path),
        "bytes": actual_bytes,
        "sha256": actual_hash,
        "date_utc": product.get("date_utc"),
        "provider": product.get("provider"),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--input", type=Path, required=True, help="Frozen ROS1 bag or MCAP flight container")
    parser.add_argument("--navigation", type=Path, required=True, help="Frozen uncompressed IGS broadcast NAV")
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--role", choices=("development", "holdout"), required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-epochs", type=int, default=-1)
    return parser.parse_args()


def run(command: list[str], log_handle) -> int:
    log_handle.write("$ " + " ".join(command) + "\n")
    result = subprocess.run(command, cwd=ROOT, text=True, capture_output=True, check=False)
    log_handle.write(result.stdout)
    log_handle.write(result.stderr)
    log_handle.flush()
    return result.returncode


def main() -> int:
    args = parse_args()
    if args.max_epochs == 0 or args.max_epochs < -1:
        fail("--max-epochs must be -1 or a positive integer")
    profile = load_profile(args.profile)
    navigation = validate_navigation(args.navigation, profile, args.role)
    dataset = dict(dict(profile.get("datasets", {})).get(args.role, {}))
    if not dataset:
        fail(f"R6 profile has no {args.role} dataset")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    acquire_dir = args.output_dir / "acquire"
    cli = [sys.executable, str(ROOT / "apps" / "gnss.py")]
    acquire = [
        *cli, "uav-mars-acquire", "--profile", str(args.profile), "--role", args.role,
        "--output-dir", str(acquire_dir), "--input", str(args.input),
    ]
    adapter = [
        *cli, "uav-mars-adapter", "--bag", str(args.input),
        "--acquire-summary", str(acquire_dir / "acquire_summary.json"),
        "--profile", str(args.profile), "--role", args.role,
        "--output-dir", str(args.output_dir),
    ]
    if args.max_epochs > 0:
        adapter.extend(("--max-epochs", str(args.max_epochs)))
    solver = [
        *cli, "spp", "--obs", str(args.output_dir / "uav_observations.rnx"),
        "--nav", str(args.navigation), "--out", str(args.output_dir / "libgnsspp_spp.pos"),
        "--summary-json", str(args.output_dir / "libgnsspp_spp_summary.json"),
    ]
    commands = [acquire, adapter, solver]
    bounded = args.max_epochs > 0
    if not bounded:
        commands.extend(
            (
                [
                    *cli, "uav-mars-signoff", "--position", str(args.output_dir / "libgnsspp_spp.pos"),
                    "--truth", str(args.output_dir / "rtk_truth.csv"),
                    "--attitude", str(args.output_dir / "attitude.csv"),
                    "--rtk-yaw", str(args.output_dir / "rtk_yaw.csv"),
                    "--rtk-status", str(args.output_dir / "rtk_status.csv"),
                    "--adapter-summary", str(args.output_dir / "adapter_summary.json"),
                    "--solver-summary", str(args.output_dir / "libgnsspp_spp_summary.json"),
                    "--profile", str(args.profile), "--output-dir", str(args.output_dir),
                ],
                [
                    *cli, "pos2kml", str(args.output_dir / "libgnsspp_spp.pos"),
                    str(args.output_dir / "libgnsspp_spp.kml"), "--name",
                    f"R6 {args.role} UAV standalone SPP", "--status", "all", "--sample-points", "100",
                ],
                [*cli, "trackplot", str(args.output_dir / "libgnsspp_spp.pos")],
            )
        )
    log_path = args.output_dir / "workflow.log"
    return_codes: list[int] = []
    with log_path.open("w", encoding="utf-8") as log_handle:
        for command in commands:
            code = run(command, log_handle)
            return_codes.append(code)
            if code != 0:
                break
    artifact_names = (
        "adapter_summary.json", "uav_observations.rnx", "imu.csv", "rtk_truth.csv",
        "attitude.csv", "rtk_yaw.csv", "rtk_status.csv", "libgnsspp_spp.pos",
        "libgnsspp_spp_summary.json", "uav_signoff_summary.json", "uav_matches.csv",
        "libgnsspp_spp.kml", "libgnsspp_spp_trajectory.png", "workflow.log",
    )
    artifacts = {
        name: {"path": str(args.output_dir / name), "sha256": sha256(args.output_dir / name)}
        for name in artifact_names
        if (args.output_dir / name).is_file()
    }
    success = len(return_codes) == len(commands) and all(code == 0 for code in return_codes)
    decision = "smoke_pass" if success and bounded else "pass" if success else "fail"
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "dataset": {"id": dataset.get("id"), "role": args.role, "container": dataset.get("container")},
        "decision": decision,
        "bounded": bounded,
        "max_epochs": args.max_epochs,
        "return_codes": return_codes,
        "commands": commands,
        "input": {"path": str(args.input), "bytes": args.input.stat().st_size, "sha256": sha256(args.input)},
        "navigation": navigation,
        "profile": {"path": str(args.profile), "sha256": sha256(args.profile)},
        "artifacts": artifacts,
    }
    manifest_path = args.output_dir / "workflow_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"R6 UAV workflow {decision}: {manifest_path}")
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
