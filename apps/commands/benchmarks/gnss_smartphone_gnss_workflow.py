#!/usr/bin/env python3
"""Run the frozen R5 smartphone workflow from a local source archive."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import zipfile

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
DEFAULT_PROFILE = ROOT / "configs" / "benchmarks" / "smartphone_r5_gsdc2023.json"
SCHEMA_VERSION = "smartphone-gnss-workflow.v2"


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
    *,
    verified_archive_sha256: str | None = None,
) -> dict[str, Path]:
    if not archive_path.is_file():
        fail(f"missing GSDC archive: {archive_path}")
    archive_contract = dict(profile.get("archive", {}))
    expected_archive_hash = str(archive_contract.get("sha256", ""))
    # A caller that has already verified the immutable archive may pass its
    # digest here.  This is intentionally a digest value, not a "skip hash"
    # switch: the frozen profile is still compared before any member is
    # opened.  The regular workflow keeps hashing by default.
    actual_archive_hash = (
        verified_archive_sha256
        if verified_archive_sha256 is not None
        else sha256(archive_path)
    )
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
    if dataset.get("device_imu_sha256"):
        members["device_imu"] = f"dataset_2023/train/{route}/{phone}/device_imu.csv"
    expected_hashes = {
        "device_gnss": str(dataset.get("device_gnss_sha256", "")),
        "ground_truth": str(dataset.get("ground_truth_sha256", "")),
        "broadcast_nav": str(dataset.get("broadcast_nav_sha256", "")),
    }
    if "device_imu" in members:
        expected_hashes["device_imu"] = str(dataset.get("device_imu_sha256", ""))
    if any(not value for value in expected_hashes.values()):
        fail(f"frozen {role} dataset is missing one or more member hashes")
    destination.mkdir(parents=True, exist_ok=True)
    paths: dict[str, Path] = {}
    with zipfile.ZipFile(archive_path) as archive:
        available = set(archive.namelist())
        for key, member in members.items():
            if member not in available:
                fail(f"archive is missing frozen member: {member}")
            suffix = {
                "device_gnss": "device_gnss.csv",
                "ground_truth": "ground_truth.csv",
                "broadcast_nav": "brdc.nav",
                "device_imu": "device_imu.csv",
            }[key]
            output = destination / suffix
            with archive.open(member) as source, output.open("wb") as target:
                shutil.copyfileobj(source, target)
            if sha256(output) != expected_hashes[key]:
                fail(f"extracted {key} hash does not match the frozen profile")
            paths[key] = output
    return paths


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


def _build_type() -> str | None:
    cache_path = ROOT / "build" / "CMakeCache.txt"
    if not cache_path.is_file():
        return None
    try:
        for line in cache_path.read_text(
            encoding="utf-8", errors="replace"
        ).splitlines():
            if line.startswith("CMAKE_BUILD_TYPE:STRING="):
                return line.split("=", 1)[1]
    except OSError:
        return None
    return None


def _spp_binary_metadata() -> dict[str, str | None]:
    suffix = ".exe" if os.name == "nt" else ""
    candidates = (
        ROOT / "build" / "apps" / f"gnss_spp{suffix}",
        ROOT / "build" / "Release" / "apps" / f"gnss_spp{suffix}",
    )
    binary = next((path for path in candidates if path.is_file()), None)
    if binary is None:
        return {"path": None, "sha256": None}
    return {"path": str(binary), "sha256": sha256(binary)}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--role", choices=("development", "holdout"), required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-epochs", type=int, default=-1)
    parser.add_argument(
        "--performance",
        action="store_true",
        help="Collect native per-epoch timing and write GPST interval metrics",
    )
    parser.add_argument(
        "--performance-interval-s",
        type=float,
        default=60.0,
        help="GPST interval width for --performance (default: 60)",
    )
    parser.add_argument(
        "--experimental-galileo-e1",
        action="store_true",
        help="Development-only candidate: include exactly mapped Galileo E1 observations.",
    )
    parser.add_argument(
        "--experimental-galileo-e1-hatch-window-s",
        type=int,
        choices=(10, 30, 60),
        help=(
            "Development-only truth-free Hatch C1C candidate for Galileo E1 "
            "using a pre-declared 10, 30, or 60 second window."
        ),
    )
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
    workflow_started = time.perf_counter()
    if args.max_epochs == 0 or args.max_epochs < -1:
        fail("--max-epochs must be -1 or a positive integer")
    if args.experimental_galileo_e1 and args.role != "development":
        fail("--experimental-galileo-e1 is development-only; holdout is sealed")
    if args.experimental_galileo_e1_hatch_window_s is not None:
        if not args.experimental_galileo_e1:
            fail(
                "--experimental-galileo-e1-hatch-window-s requires "
                "--experimental-galileo-e1"
            )
        if args.role != "development":
            fail("Hatch smoothing is development-only; holdout is sealed")
    if not math.isfinite(args.performance_interval_s) or args.performance_interval_s <= 0.0:
        fail("--performance-interval-s must be a finite positive number")
    profile = load_profile(args.profile)
    dataset = dict(dict(profile.get("datasets", {})).get(args.role, {}))
    materialization_started = time.perf_counter()
    inputs = materialize_inputs(args.archive, profile, args.role, args.output_dir / "inputs")
    materialization_wall_time_s = time.perf_counter() - materialization_started
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
    if args.experimental_galileo_e1:
        adapter_command.extend(
            (
                "--experimental-galileo-e1",
                "--broadcast-nav",
                str(inputs["broadcast_nav"]),
            )
        )
        if args.experimental_galileo_e1_hatch_window_s is not None:
            adapter_command.extend(
                (
                    "--experimental-galileo-e1-hatch-window-s",
                    str(args.experimental_galileo_e1_hatch_window_s),
                )
            )
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
    timing_csv_path = args.output_dir / "libgnsspp_spp_timing.csv"
    performance_summary_path = args.output_dir / "performance_summary.json"
    performance_intervals_path = args.output_dir / "performance_intervals.csv"
    if args.performance:
        solver_command.extend(("--timing-csv", str(timing_csv_path)))
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
    commands: list[list[str]] = [
        adapter_command,
        solver_command,
        signoff_command,
        kml_command,
        plot_command,
    ]
    if args.performance:
        commands.append(
            [
                *cli,
                "performance-report",
                "--timing-csv",
                str(timing_csv_path),
                "--output-json",
                str(performance_summary_path),
                "--output-csv",
                str(performance_intervals_path),
                "--interval-s",
                str(args.performance_interval_s),
            ]
        )
    return_codes: list[int] = []
    stage_timings: list[dict[str, object]] = []
    with log_path.open("w", encoding="utf-8") as log_handle:
        for index, command in enumerate(commands):
            started = time.perf_counter()
            code = run_command(command, log_handle)
            elapsed_s = time.perf_counter() - started
            return_codes.append(code)
            stage_timings.append(
                {
                    "index": index,
                    "command": command,
                    "return_code": code,
                    "wall_time_s": round(elapsed_s, 6),
                }
            )
            if code != 0:
                break

    artifact_names = [
        "summary.json",
        "rover.obs",
        "libgnsspp_spp.pos",
        "libgnsspp_spp_summary.json",
        "signoff_summary.json",
        "matches.csv",
        "libgnsspp_spp.kml",
        "libgnsspp_spp_trajectory.png",
        "workflow.log",
    ]
    if args.performance:
        artifact_names.extend(
            (timing_csv_path.name, performance_summary_path.name, performance_intervals_path.name)
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
        "stage_timings": stage_timings,
        "materialization_wall_time_s": round(materialization_wall_time_s, 6),
        "workflow_wall_time_s": round(time.perf_counter() - workflow_started, 6),
        "performance": {
            "enabled": args.performance,
            "interval_s": args.performance_interval_s if args.performance else None,
            "timing_csv": str(timing_csv_path) if args.performance else None,
            "summary_json": str(performance_summary_path) if args.performance else None,
            "intervals_csv": str(performance_intervals_path) if args.performance else None,
        },
        "experimental_signal_candidate": {
            "name": "galileo-e1",
            "enabled": args.experimental_galileo_e1,
            "role_policy": "development-only; holdout untouched",
            "hatch_carrier_smoothing": {
                "enabled": args.experimental_galileo_e1_hatch_window_s is not None,
                "window_seconds": args.experimental_galileo_e1_hatch_window_s,
            },
        },
        "development_only_submission_lane": (
            profile.get("development_only_submission_lane")
            if args.role == "development"
            else None
        ),
        "reproducibility": {
            "source_revision": _git_revision(),
            "source_status": _git_status(),
            "build_type": _build_type(),
            "spp_binary": _spp_binary_metadata(),
        },
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
