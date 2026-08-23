#!/usr/bin/env python3
"""Build one auditable Tokyo-style GNSS/IMU continuity artifact bundle.

The bundle command deliberately orchestrates the existing dispatcher commands
instead of duplicating fusion, KML, plotting, or scoring logic.  It records
every child argv and exit status in a manifest and keeps writing that manifest
when a later artifact step fails.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shlex
import subprocess
import sys
from datetime import datetime, timezone
from typing import Any


ROOT_DIR = Path(__file__).resolve().parents[3]
DISPATCHER = ROOT_DIR / "apps" / "gnss.py"
SCHEMA_VERSION = "libgnsspp.urban_continuity_bundle.v1"
OPERATIONAL_MAX_BRIDGE_AGE_S = 60.0
VALIDATED_MAX_PROPAGATION_AGE_S = 87.2
FROZEN_THRESHOLDS: dict[str, Any] = {
    "fused_bridge_coverage_pct_min": 99.0,
    "max_bridge_horizontal_error_m_max": 75.0,
    "max_reacquisition_error_step_m_max": 15.0,
    "fixed_p95_regression_m_max": 5.0,
    "fused_availability_at_least_rtk": True,
    "fused_nonfinite_epochs_max": 0,
}
FRAME_CONTRACT = {
    "rtk_pos": "antenna_ecef",
    "fused_pos": "antenna_ecef",
    "reference_csv": "antenna_ecef",
    "internal_eskf_output": "imu_origin_local_enu",
}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    raw_argv = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss urban-continuity-bundle"),
        description=(
            "Run the frozen urban RTK/IMU continuity recipe and emit POS, KML, "
            "PNG, log, score, segments, and an auditable manifest."
        ),
    )
    parser.add_argument("--data-dir", type=Path, required=True)
    parser.add_argument(
        "--reference-csv",
        type=Path,
        default=None,
        help="PPC reference CSV (default: <data-dir>/reference.csv).",
    )
    parser.add_argument(
        "--output-dir",
        "--out-dir",
        dest="output_dir",
        type=Path,
        required=True,
        help="Directory receiving the complete bundle.",
    )
    parser.add_argument(
        "--lever-arm",
        default="0.31,0,-0.55",
        help="IMU-to-antenna body-FLU lever arm in metres (default: PPC Tokyo).",
    )
    parser.add_argument("--preset", default="low-cost")
    parser.add_argument(
        "--no-zupt",
        action="store_true",
        help="Disable ZUPT for an explicit ablation; the frozen recipe leaves it enabled.",
    )
    parser.add_argument("--max-velocity-nis", type=float, default=25.0)
    parser.add_argument(
        "--max-consecutive-velocity-gate-rejections", type=int, default=3
    )
    parser.add_argument("--max-gnss-velocity-reanchor-mps", type=float, default=20.0)
    parser.add_argument(
        "--max-epochs",
        type=int,
        default=0,
        help="Stop after N GNSS epochs (0 = full input).",
    )
    args = parser.parse_args(raw_argv)
    # The dispatcher execs this module as a child. Preserve the caller's exact
    # options instead of reconstructing them from a test/process sys.argv.
    setattr(args, "_raw_argv", raw_argv)
    return args


def parse_lever_arm(raw: str) -> list[float]:
    try:
        values = [float(value.strip()) for value in raw.split(",")]
    except ValueError as exc:
        raise ValueError("--lever-arm must contain three comma-separated numbers") from exc
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError("--lever-arm must contain three finite comma-separated numbers")
    return values


def display_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(ROOT_DIR).as_posix()
    except ValueError:
        return str(resolved)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def file_record(path: Path) -> dict[str, Any]:
    resolved = path.resolve()
    if not resolved.is_file():
        return {
            "path": display_path(resolved),
            "exists": False,
            "bytes": None,
            "sha256": None,
        }
    return {
        "path": display_path(resolved),
        "exists": True,
        "bytes": resolved.stat().st_size,
        "sha256": sha256_file(resolved),
    }


def collect_input_records(data_dir: Path, reference_csv: Path) -> dict[str, dict[str, Any]]:
    records: dict[str, dict[str, Any]] = {}
    if data_dir.is_dir():
        for path in sorted(data_dir.rglob("*")):
            if path.is_file():
                records[display_path(path)] = file_record(path)
    if reference_csv.is_file():
        records[display_path(reference_csv)] = file_record(reference_csv)
    elif display_path(reference_csv) not in records:
        records[display_path(reference_csv)] = file_record(reference_csv)
    return records


def find_fuse_binary() -> Path | None:
    candidates = [
        ROOT_DIR / "apps" / "gnss_fuse",
        ROOT_DIR / "build" / "apps" / "gnss_fuse",
    ]
    for build_root in sorted(ROOT_DIR.glob("build*")):
        candidates.extend(
            [
                build_root / "apps" / "gnss_fuse",
                build_root / "apps" / "Release" / "gnss_fuse",
                build_root / "Release" / "apps" / "gnss_fuse",
            ]
        )
    seen: set[Path] = set()
    for candidate in candidates:
        candidate = candidate.resolve()
        if candidate in seen:
            continue
        seen.add(candidate)
        if candidate.is_file():
            return candidate
    return None


def binary_record() -> dict[str, Any]:
    binary = find_fuse_binary()
    record: dict[str, Any] = {
        "path": display_path(binary) if binary else None,
        "exists": binary is not None,
        "bytes": binary.stat().st_size if binary else None,
        "sha256": sha256_file(binary) if binary else None,
        "version_command": [str(binary), "--version"] if binary else None,
        "version_available": False,
        "version": None,
        "version_exit_status": None,
    }
    if binary is None:
        record["version_error"] = "gnss_fuse binary not found"
        return record
    try:
        completed = subprocess.run(
            [str(binary), "--version"],
            cwd=ROOT_DIR,
            capture_output=True,
            text=True,
            check=False,
            timeout=5,
        )
        output = (completed.stdout + completed.stderr).strip()
        record["version_exit_status"] = completed.returncode
        if completed.returncode == 0 and output:
            record["version_available"] = True
            record["version"] = output
        elif output:
            record["version_error"] = output[-1000:]
    except (OSError, subprocess.SubprocessError) as exc:
        record["version_error"] = str(exc)
    return record


def software_revision_record() -> dict[str, Any]:
    record: dict[str, Any] = {
        "repository_root": display_path(ROOT_DIR),
        "git_head": None,
        "git_head_available": False,
        "dirty": None,
        "changed_files": [],
        "status_command": ["git", "status", "--short", "--untracked-files=all"],
    }
    try:
        head = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=ROOT_DIR,
            capture_output=True,
            text=True,
            check=False,
            timeout=5,
        )
        if head.returncode == 0 and head.stdout.strip():
            record["git_head"] = head.stdout.strip()
            record["git_head_available"] = True
        status = subprocess.run(
            record["status_command"],
            cwd=ROOT_DIR,
            capture_output=True,
            text=True,
            check=False,
            timeout=5,
        )
        if status.returncode == 0:
            lines = [line for line in status.stdout.splitlines() if line.strip()]
            record["dirty"] = bool(lines)
            record["changed_files"] = [
                {
                    "status": line[:2],
                    "path": line[3:] if len(line) > 3 else "",
                }
                for line in lines
            ]
        else:
            record["status_error"] = status.stderr.strip()[-1000:]
    except (OSError, subprocess.SubprocessError) as exc:
        record["error"] = str(exc)
    return record


def dataset_provenance(data_dir: Path) -> dict[str, Any]:
    parts = data_dir.parts
    city = parts[-2] if len(parts) >= 2 else None
    run = parts[-1] if parts else None
    return {
        "dataset": "PPC-Dataset",
        "city": city,
        "run": run,
        "upstream": "https://github.com/taroz/PPC-Dataset",
        "source_documents": [
            "data/PPC-Dataset/README.md",
            "data/PPC-Dataset/tokyo/README.md",
            "docs/ppc_reproduction.md",
        ],
        "ground_truth": "Applanix POS LVX-120 (per data/PPC-Dataset/tokyo/README.md)",
        "license": (
            "unknown from the bundled README; verify upstream terms before redistribution"
        ),
    }


def command(*args: str | Path) -> list[str]:
    return [sys.executable, str(DISPATCHER), *[str(arg) for arg in args]]


def build_step_commands(
    data_dir: Path,
    reference_csv: Path,
    output_dir: Path,
    lever_arm: list[float],
    preset: str,
    zupt_enabled: bool,
    max_velocity_nis: float,
    max_velocity_rejections: int,
    max_velocity_reanchor_mps: float,
    max_epochs: int,
) -> dict[str, list[str]]:
    rtk_pos = output_dir / "rtk.pos"
    fused_pos = output_dir / "fused.pos"
    kml = output_dir / "fused.kml"
    png = output_dir / "fused_trajectory.png"
    score = output_dir / "score.json"
    segments = output_dir / "segments.csv"
    lever_text = ",".join(f"{value:.12g}" for value in lever_arm)
    fuse_args = [
        "fuse",
        "--data-dir",
        data_dir,
        "--lever-arm",
        lever_text,
        "--preset",
        preset,
        "--max-position-nis",
        "500",
        "--max-consecutive-gate-rejections",
        "30",
        "--no-nhc",
        "--max-velocity-nis",
        f"{max_velocity_nis:.12g}",
        "--max-consecutive-velocity-gate-rejections",
        str(max_velocity_rejections),
        "--max-gnss-velocity-reanchor-mps",
        f"{max_velocity_reanchor_mps:.12g}",
        "--rtk-pos-out",
        rtk_pos,
        "--out",
        fused_pos,
    ]
    fuse_args.insert(7, "--zupt" if zupt_enabled else "--no-zupt")
    if max_epochs:
        fuse_args.extend(["--max-epochs", str(max_epochs)])
    return {
        "fuse": command(*fuse_args),
        "kml": command("pos2kml", fused_pos, kml, "--status", "all"),
        # plot_trajectory.py accepts an optional second input only in raw
        # RTKLIB text format. The RTK stream emitted by gnss_fuse is already a
        # libgnss++ .pos, so pass the fused stream alone and avoid a misleading
        # parser failure during an otherwise valid bundle.
        "png": command("trackplot", fused_pos),
        "score": command(
            "urban-bridge-score",
            "--rtk-pos",
            rtk_pos,
            "--fused-pos",
            fused_pos,
            "--reference-csv",
            reference_csv,
            "--summary-json",
            score,
            "--segments-csv",
            segments,
            "--require-fused-bridge-coverage-min",
            "99",
            "--require-max-bridge-error-max",
            "75",
            "--require-max-reacquisition-jump-max",
            "15",
            "--require-fixed-p95-regression-max",
            "5",
            "--require-fused-availability-at-least-rtk",
            "--require-no-nonfinite",
        ),
    }


def initial_manifest(
    args: argparse.Namespace,
    data_dir: Path,
    reference_csv: Path,
    output_dir: Path,
    lever_arm: list[float],
    commands: dict[str, list[str]],
) -> dict[str, Any]:
    raw_argv = list(getattr(args, "_raw_argv", sys.argv[1:]))
    output_paths = {
        "rtk_pos": output_dir / "rtk.pos",
        "fused_pos": output_dir / "fused.pos",
        "kml": output_dir / "fused.kml",
        "png": output_dir / "fused_trajectory.png",
        "log": output_dir / "bundle.log",
        "score_summary": output_dir / "score.json",
        "segments": output_dir / "segments.csv",
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "running",
        "exit_status": None,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "invocation": {
            "argv": [str(DISPATCHER), "urban-continuity-bundle", *raw_argv],
            "dispatcher_argv": [
                sys.executable,
                str(DISPATCHER),
                "urban-continuity-bundle",
                *raw_argv,
            ],
            "cwd": str(Path.cwd().resolve()),
            "python": sys.executable,
            "platform": platform.platform(),
        },
        "configuration": {
            "data_dir": display_path(data_dir),
            "reference_csv": display_path(reference_csv),
            "output_dir": display_path(output_dir),
            "lever_arm_body_imu_to_antenna_m": lever_arm,
            "preset": args.preset,
            "zupt": not args.no_zupt,
            "nhc": False,
            "max_velocity_nis": args.max_velocity_nis,
            "max_position_nis": 500.0,
            "max_consecutive_position_gate_rejections": 30,
            "max_consecutive_velocity_gate_rejections": args.max_consecutive_velocity_gate_rejections,
            "max_gnss_velocity_reanchor_mps": args.max_gnss_velocity_reanchor_mps,
            "max_epochs": args.max_epochs,
        },
        "frame_contract": FRAME_CONTRACT,
        "dataset_provenance": dataset_provenance(data_dir),
        "software_revision": software_revision_record(),
        "thresholds": FROZEN_THRESHOLDS,
        "validated_max_propagation_age_s": None,
        "validated_max_propagation_age_reference_s": VALIDATED_MAX_PROPAGATION_AGE_S,
        "operational_max_bridge_age_recommendation_s": OPERATIONAL_MAX_BRIDGE_AGE_S,
        "operational_max_bridge_age_exceeded": None,
        "inputs": collect_input_records(data_dir, reference_csv),
        "binary": binary_record(),
        "commands": {
            name: [str(value) for value in argv] for name, argv in commands.items()
        },
        "steps": [],
        "artifacts": {name: file_record(path) for name, path in output_paths.items()},
        "manifest_path": display_path(output_dir / "manifest.json"),
        "notes": [
            "The manifest intentionally excludes its own SHA-256 from artifacts to avoid a self-referential hash.",
            "The external validated age reference is the Tokyo run1 frozen observation; score segments provide the measured age.",
        ],
    }


def persist_manifest(path: Path, payload: dict[str, Any], output_paths: dict[str, Path]) -> None:
    payload["artifacts"] = {
        name: file_record(artifact_path) for name, artifact_path in output_paths.items()
    }
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def run_step(name: str, argv: list[str], log_path: Path) -> dict[str, Any]:
    entry: dict[str, Any] = {
        "name": name,
        "argv": argv,
        "exit_status": None,
        "status": "failed",
    }
    try:
        with log_path.open("a", encoding="utf-8") as log:
            log.write(f"\n$ {shlex.join(argv)}\n")
            log.flush()
            completed = subprocess.run(
                argv,
                cwd=ROOT_DIR,
                stdout=log,
                stderr=subprocess.STDOUT,
                check=False,
                text=True,
            )
        entry["exit_status"] = completed.returncode
        entry["status"] = "passed" if completed.returncode == 0 else "failed"
    except (OSError, subprocess.SubprocessError) as exc:
        entry["exception"] = str(exc)
    return entry


def skipped_step(name: str, argv: list[str], reason: str) -> dict[str, Any]:
    return {
        "name": name,
        "argv": argv,
        "exit_status": None,
        "status": "skipped",
        "reason": reason,
    }


def load_score_results(summary_path: Path, manifest: dict[str, Any]) -> None:
    if not summary_path.is_file():
        return
    try:
        payload = json.loads(summary_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        manifest["score_error"] = str(exc)
        return
    aggregate = payload.get("aggregate", {})
    gate = payload.get("gate", {})
    ages = [
        float(segment["propagation_age_s"])
        for segment in payload.get("segments", [])
        if segment.get("propagation_age_s") is not None
        and math.isfinite(float(segment["propagation_age_s"]))
    ]
    measured_age = max(ages) if ages else None
    manifest["score"] = {
        "aggregate": aggregate,
        "gate": gate,
        "validated_max_propagation_age_s": measured_age,
    }
    manifest["validated_max_propagation_age_s"] = measured_age
    manifest["operational_max_bridge_age_exceeded"] = (
        measured_age is not None and measured_age > OPERATIONAL_MAX_BRIDGE_AGE_S
    )


def run_bundle(args: argparse.Namespace) -> int:
    try:
        lever_arm = parse_lever_arm(args.lever_arm)
    except ValueError as exc:
        print(f"Argument error: {exc}", file=sys.stderr)
        return 2
    if args.max_epochs < 0:
        print("Argument error: --max-epochs must be non-negative", file=sys.stderr)
        return 2
    if args.max_consecutive_velocity_gate_rejections < 0:
        print(
            "Argument error: --max-consecutive-velocity-gate-rejections must be non-negative",
            file=sys.stderr,
        )
        return 2
    for value, label in (
        (args.max_velocity_nis, "--max-velocity-nis"),
        (args.max_gnss_velocity_reanchor_mps, "--max-gnss-velocity-reanchor-mps"),
    ):
        if not math.isfinite(value) or value < 0.0:
            print(f"Argument error: {label} must be finite and non-negative", file=sys.stderr)
            return 2

    data_dir = args.data_dir.resolve()
    reference_csv = (args.reference_csv or data_dir / "reference.csv").resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    log_path = output_dir / "bundle.log"
    manifest_path = output_dir / "manifest.json"
    raw_argv = list(getattr(args, "_raw_argv", sys.argv[1:]))
    log_path.write_text(
        "libgnss++ urban-continuity-bundle\n"
        f"invocation: {shlex.join([str(DISPATCHER), 'urban-continuity-bundle', *raw_argv])}\n",
        encoding="utf-8",
    )

    commands = build_step_commands(
        data_dir,
        reference_csv,
        output_dir,
        lever_arm,
        args.preset,
        not args.no_zupt,
        args.max_velocity_nis,
        args.max_consecutive_velocity_gate_rejections,
        args.max_gnss_velocity_reanchor_mps,
        args.max_epochs,
    )
    manifest = initial_manifest(args, data_dir, reference_csv, output_dir, lever_arm, commands)
    output_paths = {
        "rtk_pos": output_dir / "rtk.pos",
        "fused_pos": output_dir / "fused.pos",
        "kml": output_dir / "fused.kml",
        "png": output_dir / "fused_trajectory.png",
        "log": log_path,
        "score_summary": output_dir / "score.json",
        "segments": output_dir / "segments.csv",
    }
    persist_manifest(manifest_path, manifest, output_paths)

    def append_step(entry: dict[str, Any]) -> None:
        manifest["steps"].append(entry)
        persist_manifest(manifest_path, manifest, output_paths)

    append_step(run_step("fuse", commands["fuse"], log_path))

    if output_paths["fused_pos"].is_file():
        append_step(run_step("kml", commands["kml"], log_path))
        append_step(run_step("png", commands["png"], log_path))
    else:
        append_step(skipped_step("kml", commands["kml"], "fused.pos was not produced"))
        append_step(skipped_step("png", commands["png"], "fused.pos was not produced"))

    if output_paths["rtk_pos"].is_file() and output_paths["fused_pos"].is_file() and reference_csv.is_file():
        append_step(run_step("score", commands["score"], log_path))
    else:
        append_step(skipped_step("score", commands["score"], "score inputs are incomplete"))

    load_score_results(output_paths["score_summary"], manifest)
    required_steps_ok = all(step["status"] == "passed" for step in manifest["steps"])
    score_gate_ok = manifest.get("score", {}).get("gate", {}).get("status") == "passed"
    manifest["status"] = "passed" if required_steps_ok and score_gate_ok else "failed"
    manifest["exit_status"] = 0 if manifest["status"] == "passed" else 1
    persist_manifest(manifest_path, manifest, output_paths)

    print(f"Bundle status: {manifest['status']}")
    print(f"Output directory: {output_dir}")
    print(f"Manifest: {manifest_path}")
    if manifest.get("validated_max_propagation_age_s") is not None:
        print(
            "Max propagation age: "
            f"{manifest['validated_max_propagation_age_s']:.3f} s "
            f"(operational recommendation {OPERATIONAL_MAX_BRIDGE_AGE_S:.1f} s)"
        )
    return int(manifest["exit_status"])


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    return run_bundle(args)


if __name__ == "__main__":
    raise SystemExit(main())
