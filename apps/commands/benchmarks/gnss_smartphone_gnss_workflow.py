#!/usr/bin/env python3
"""Run the frozen R5 smartphone workflow from a local source archive."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
DEFAULT_PROFILE = ROOT / "configs" / "benchmarks" / "smartphone_r5_gsdc2023.json"
SCHEMA_VERSION = "smartphone-gnss-workflow.v1"


def fail(message: str) -> "NoReturn":
    raise SystemExit(message)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_profile(path: Path) -> dict[str, object]:
    if not path.is_file():
        fail(f"missing R5 profile: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        fail(f"invalid R5 profile: {exc}")
    if not isinstance(payload, dict) or payload.get("schema_version") != "smartphone-r5-profile.v1":
        fail("R5 profile schema is not smartphone-r5-profile.v1")
    return payload


def materialize_inputs(
    archive_path: Path,
    profile: dict[str, object],
    role: str,
    destination: Path,
) -> dict[str, Path]:
    if not archive_path.is_file():
        fail(f"missing GSDC archive: {archive_path}")
    archive_contract = dict(profile.get("archive", {}))
    expected_archive_hash = str(archive_contract.get("sha256", ""))
    actual_archive_hash = sha256(archive_path)
    if actual_archive_hash != expected_archive_hash:
        fail("GSDC archive hash does not match the frozen profile")
    dataset = dict(dict(profile.get("datasets", {})).get(role, {}))
    dataset_id = str(dataset.get("id", ""))
    if "/" not in dataset_id:
        fail(f"invalid frozen {role} dataset id")
    route, phone = dataset_id.split("/", 1)
    members = {
        "device_gnss": f"dataset_2023/train/{route}/{phone}/device_gnss.csv",
        "ground_truth": f"dataset_2023/train/{route}/{phone}/ground_truth.csv",
        "broadcast_nav": f"dataset_2023/train/{route}/brdc.nav",
    }
    expected_hashes = {
        "device_gnss": str(dataset.get("device_gnss_sha256", "")),
        "ground_truth": str(dataset.get("ground_truth_sha256", "")),
        "broadcast_nav": str(dataset.get("broadcast_nav_sha256", "")),
    }
    destination.mkdir(parents=True, exist_ok=True)
    paths: dict[str, Path] = {}
    with zipfile.ZipFile(archive_path) as archive:
        available = set(archive.namelist())
        for key, member in members.items():
            if member not in available:
                fail(f"archive is missing frozen member: {member}")
            suffix = "device_gnss.csv" if key == "device_gnss" else "ground_truth.csv" if key == "ground_truth" else "brdc.nav"
            output = destination / suffix
            with archive.open(member) as source, output.open("wb") as target:
                shutil.copyfileobj(source, target)
            if sha256(output) != expected_hashes[key]:
                fail(f"extracted {key} hash does not match the frozen profile")
            paths[key] = output
    return paths


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--role", choices=("development", "holdout"), required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-epochs", type=int, default=-1)
    return parser.parse_args()


def run_command(command: list[str], log_handle) -> int:
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
    dataset = dict(dict(profile.get("datasets", {})).get(args.role, {}))
    inputs = materialize_inputs(args.archive, profile, args.role, args.output_dir / "inputs")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    log_path = args.output_dir / "workflow.log"
    cli = [sys.executable, str(ROOT / "apps" / "gnss.py")]

    adapter_command = [
        *cli,
        "smartphone-gnss-adapter",
        "--device-gnss",
        str(inputs["device_gnss"]),
        "--ground-truth",
        str(inputs["ground_truth"]),
        "--output-dir",
        str(args.output_dir),
        "--dataset-id",
        str(dataset["id"]),
        "--device-model",
        str(dataset["device_model"]),
        "--source-url",
        str(dict(profile["archive"])["url"]),
        "--source-terms",
        str(dict(profile["archive"])["source_terms"]),
        "--role",
        args.role,
        "--skip-epochs",
        str(dataset.get("skip_epochs", 0)),
    ]
    if args.max_epochs > 0:
        adapter_command.extend(("--max-epochs", str(args.max_epochs)))

    solver_command = [
        *cli,
        "spp",
        "--obs",
        str(args.output_dir / "rover.obs"),
        "--nav",
        str(inputs["broadcast_nav"]),
        "--out",
        str(args.output_dir / "libgnsspp_spp.pos"),
        "--summary-json",
        str(args.output_dir / "libgnsspp_spp_summary.json"),
    ]
    signoff_command = [
        *cli,
        "smartphone-gnss-signoff",
        "--position",
        str(args.output_dir / "libgnsspp_spp.pos"),
        "--ground-truth",
        str(inputs["ground_truth"]),
        "--adapter-summary",
        str(args.output_dir / "summary.json"),
        "--solver-summary",
        str(args.output_dir / "libgnsspp_spp_summary.json"),
        "--output-dir",
        str(args.output_dir),
        "--profile",
        str(args.profile),
    ]
    kml_command = [
        *cli,
        "pos2kml",
        str(args.output_dir / "libgnsspp_spp.pos"),
        str(args.output_dir / "libgnsspp_spp.kml"),
        "--name",
        f"R5 {args.role} standalone SPP",
        "--status",
        "all",
        "--sample-points",
        "60",
    ]
    plot_command = [*cli, "trackplot", str(args.output_dir / "libgnsspp_spp.pos")]
    commands = (adapter_command, solver_command, signoff_command, kml_command, plot_command)
    return_codes: list[int] = []
    with log_path.open("w", encoding="utf-8") as log_handle:
        for command in commands:
            code = run_command(command, log_handle)
            return_codes.append(code)
            if code != 0:
                break

    artifact_names = (
        "summary.json",
        "rover.obs",
        "libgnsspp_spp.pos",
        "libgnsspp_spp_summary.json",
        "signoff_summary.json",
        "matches.csv",
        "libgnsspp_spp.kml",
        "libgnsspp_spp_trajectory.png",
        "workflow.log",
    )
    artifacts = {
        name: {"path": str(args.output_dir / name), "sha256": sha256(args.output_dir / name)}
        for name in artifact_names
        if (args.output_dir / name).is_file()
    }
    success = len(return_codes) == len(commands) and all(code == 0 for code in return_codes)
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "profile_id": profile.get("profile_id"),
        "role": args.role,
        "dataset_id": dataset.get("id"),
        "max_epochs": args.max_epochs,
        "decision": "pass" if success else "fail",
        "return_codes": return_codes,
        "commands": commands,
        "archive": {"path": str(args.archive), "sha256": sha256(args.archive)},
        "profile": {"path": str(args.profile), "sha256": sha256(args.profile)},
        "artifacts": artifacts,
    }
    manifest_path = args.output_dir / "workflow_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Smartphone GNSS workflow {manifest['decision']}: {manifest_path}")
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
