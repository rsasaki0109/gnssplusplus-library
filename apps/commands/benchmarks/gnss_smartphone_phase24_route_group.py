#!/usr/bin/env python3
"""Run the frozen Phase24 raw-only route-group experiment.

This command deliberately has a small surface area: central-directory metadata
and the Phase24 freeze record select the files, and only the three declared
inference members (device_gnss.csv, device_imu.csv, brdc.nav) are opened during
materialisation.  Ground truth is never materialised by this command.  The
native binary is invoked directly for each frozen recipe; no adapter-generated
coordinates, MAT files, sample coordinates, or previous trajectory artifacts
are accepted as inference inputs.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any
import zipfile


ROOT = Path(__file__).resolve().parents[3]
DEFAULT_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase24_route_group_split_freeze_v1.json"
DEFAULT_ARCHIVE = ROOT / "data/gsdc2023/cache/dataset_2023.zip"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase24-route-group-eval-v1"
SCHEMA = "smartphone-r5-phase24-route-group-run.v1"
MATERIALIZATION_SCHEMA = "smartphone-r5-phase24-raw-materialization.v1"
RUN_SCHEMA = "smartphone-r5-phase24-truth-free-run.v1"
RAW_MEMBER_KEYS = ("device_gnss", "device_imu", "broadcast_nav")
RAW_BASENAMES = {
    "device_gnss": "device_gnss.csv",
    "device_imu": "device_imu.csv",
    "broadcast_nav": "brdc.nav",
}

CANDIDATE_FLAGS: dict[str, tuple[str, ...]] = {
    "phase12_control": (
        "--native-pdc-imu-tdcp",
        "--native-signal-bias-states",
        "--native-residual-ionosphere",
    ),
    "phase19_e1_e5a": (
        "--native-pdc-imu-tdcp",
        "--native-signal-bias-states",
        "--native-residual-ionosphere",
        "--native-carrier-code-leveling",
        "--native-carrier-code-innovation-reset",
        "--native-carrier-code-gal-e1-e5a",
    ),
    "phase20_stop": (
        "--native-pdc-imu-tdcp",
        "--native-signal-bias-states",
        "--native-residual-ionosphere",
        "--native-carrier-code-leveling",
        "--native-carrier-code-innovation-reset",
        "--native-carrier-code-gal-e1-e5a",
        "--native-upstream-stop-constraints",
    ),
    "phase21_position_offset": (
        "--native-pdc-imu-tdcp",
        "--native-signal-bias-states",
        "--native-residual-ionosphere",
        "--native-upstream-position-offset",
    ),
}

# The archived 2021 Android exports have no ChipsetElapsedRealtimeNanos
# anchors.  This explicit raw-only portability contract maps their UTC wall
# clock to GPST after the GNSS raw-clock parser has succeeded.  It does not
# change the pseudorange source and is separate from the frozen candidate
# matrix above.
RAW_CLOCK_FLAGS: tuple[str, ...] = (
    "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback",
)


class Phase24Error(ValueError):
    """Raised when a frozen raw-only contract cannot be proven."""


@dataclass(frozen=True)
class SelectedRoute:
    dataset_id: str
    route: str
    phone: str
    role: str


def sha256(path: Path) -> str:
    if not path.is_file():
        raise Phase24Error(f"missing file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as handle:
            fd = -1
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = ""
        directory = os.open(path.parent, getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        if fd >= 0:
            os.close(fd)
        if temporary:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    atomic_write(path, (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def load_freeze(path: Path) -> dict[str, Any]:
    try:
        freeze = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise Phase24Error(f"invalid Phase24 freeze record: {path}") from exc
    if freeze.get("schema_version") != "smartphone-r5-phase24-route-group-split-freeze.v1":
        raise Phase24Error("unexpected Phase24 freeze schema")
    if freeze.get("scope", {}).get("mat_policy", "").lower().find("no .mat") < 0:
        raise Phase24Error("freeze does not contain the no-MAT contract")
    split = freeze.get("frozen_split", {})
    train = split.get("train_calibration")
    validation = split.get("fresh_validation", {}).get("dataset_id")
    holdout = split.get("future_holdout", {}).get("dataset_id")
    if not isinstance(train, list) or len(train) < 2 or not validation or not holdout:
        raise Phase24Error("incomplete frozen Phase24 split")
    ids = [row.get("dataset_id") for row in train]
    ids.extend([validation, holdout])
    if any(not isinstance(value, str) or value.count("/") != 1 for value in ids):
        raise Phase24Error("invalid route/phone identity in frozen split")
    routes = [value.split("/", 1)[0] for value in ids]
    if len(routes) != len(set(routes)):
        raise Phase24Error("Phase24 roles reuse a route")
    if holdout in ids[: len(train) + 1]:
        raise Phase24Error("holdout identity overlaps another role")
    return freeze


def selected_routes(freeze: dict[str, Any], role: str) -> list[SelectedRoute]:
    split = freeze["frozen_split"]
    if role == "train":
        rows = split["train_calibration"]
        return [
            SelectedRoute(row["dataset_id"], row["route"], row["phone"], "train_calibration")
            for row in rows
        ]
    if role == "validation":
        row = split["fresh_validation"]
        return [SelectedRoute(row["dataset_id"], row["route"], row["phone"], "fresh_validation")]
    if role == "holdout":
        raise Phase24Error("Phase24 holdout materialisation is forbidden")
    raise Phase24Error(f"unknown materialisation role: {role}")


def _member_info(archive: zipfile.ZipFile, name: str) -> zipfile.ZipInfo:
    matches = [info for info in archive.infolist() if info.filename == name]
    if len(matches) != 1 or matches[0].is_dir():
        raise Phase24Error(f"allowlisted archive member is not unique: {name}")
    if Path(name).suffix.lower() == ".mat":
        raise Phase24Error("MAT member rejected before open")
    return matches[0]


def _copy_member(archive: zipfile.ZipFile, info: zipfile.ZipInfo, destination: Path) -> dict[str, Any]:
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(f".{destination.name}.{os.getpid()}.tmp")
    if temporary.exists():
        raise Phase24Error(f"stale materialization temporary file: {temporary}")
    digest = hashlib.sha256()
    size = 0
    try:
        with archive.open(info, "r") as source, temporary.open("xb") as target:
            for chunk in iter(lambda: source.read(1024 * 1024), b""):
                if not chunk:
                    break
                target.write(chunk)
                digest.update(chunk)
                size += len(chunk)
            target.flush()
            os.fsync(target.fileno())
        if size != info.file_size:
            raise Phase24Error(f"archive member size mismatch: {info.filename}")
        os.replace(temporary, destination)
        return {
            "member": info.filename,
            "sha256": digest.hexdigest(),
            "file_size": size,
            "compressed_size": info.compress_size,
            "crc32_hex": f"{info.CRC:08x}",
        }
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def materialize_train(freeze_path: Path, archive_path: Path, output_root: Path) -> dict[str, Any]:
    freeze = load_freeze(freeze_path)
    expected_archive = freeze["scope"]["archive_sha256"]
    actual_archive = sha256(archive_path)
    if actual_archive != expected_archive:
        raise Phase24Error("archive SHA256 does not match frozen record")
    routes = selected_routes(freeze, "train")
    route_reports: dict[str, Any] = {}
    with zipfile.ZipFile(archive_path) as archive:
        for selected in routes:
            metadata = freeze["selected_central_directory_metadata"].get(selected.dataset_id)
            if not isinstance(metadata, dict):
                raise Phase24Error(f"missing central metadata for {selected.dataset_id}")
            destination_root = output_root / "raw" / selected.route / selected.phone
            if destination_root.exists() and any(destination_root.iterdir()):
                raise Phase24Error(f"refusing to overwrite existing Phase24 input root: {destination_root}")
            staged = destination_root.with_name(f".{destination_root.name}.{os.getpid()}.staging")
            if staged.exists():
                raise Phase24Error(f"stale staging directory: {staged}")
            staged.mkdir(parents=True)
            members: dict[str, Any] = {}
            try:
                names = {
                    "device_gnss": metadata["device_gnss"]["name"],
                    "device_imu": metadata["device_imu"]["name"],
                    "broadcast_nav": metadata["broadcast_nav"]["name"],
                }
                for key in RAW_MEMBER_KEYS:
                    member = names[key]
                    if Path(member).suffix.lower() == ".mat":
                        raise Phase24Error("MAT member rejected before open")
                    info = _member_info(archive, member)
                    expected = metadata[key]
                    if info.file_size != expected["file_size"] or f"{info.CRC:08x}" != expected["crc32_hex"]:
                        raise Phase24Error(f"central metadata mismatch for {member}")
                    members[key] = _copy_member(archive, info, staged / RAW_BASENAMES[key])
                destination_root.parent.mkdir(parents=True, exist_ok=True)
                os.replace(staged, destination_root)
            finally:
                if staged.exists():
                    for child in staged.iterdir():
                        child.unlink()
                    staged.rmdir()
            report = {
                "schema_version": MATERIALIZATION_SCHEMA,
                "dataset_id": selected.dataset_id,
                "role": selected.role,
                "archive": {"path": str(archive_path), "sha256": actual_archive},
                "central_directory_only_selection": True,
                "members_opened": sorted(members),
                "raw_only": True,
                "truth_materialized": False,
                "truth_open_count": 0,
                "mat_member_opened": False,
                "inputs": {
                    key: {"path": str(output_root / "raw" / selected.route / selected.phone / RAW_BASENAMES[key]), **value}
                    for key, value in members.items()
                },
            }
            manifest_path = destination_root / "materialization_manifest.json"
            atomic_json(manifest_path, report)
            route_reports[selected.dataset_id] = {
                "manifest": str(manifest_path),
                "manifest_sha256": sha256(manifest_path),
                "inputs": report["inputs"],
            }
    summary = {
        "schema_version": SCHEMA,
        "operation": "materialize-train-raw-only",
        "freeze_record": str(freeze_path),
        "freeze_record_sha256": sha256(freeze_path),
        "archive_sha256": actual_archive,
        "truth_open_count": 0,
        "mat_member_opened": False,
        "routes": route_reports,
    }
    atomic_json(output_root / "train_materialization_summary.json", summary)
    return summary


def _run_command(command: list[str], log_path: Path) -> tuple[int, float, str]:
    started = time.perf_counter()
    result = subprocess.run(command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
    elapsed = time.perf_counter() - started
    atomic_write(log_path, result.stdout.encode("utf-8"))
    return result.returncode, elapsed, result.stdout


def run_train_candidates(freeze_path: Path, input_root: Path, output_root: Path, binary: Path) -> dict[str, Any]:
    freeze = load_freeze(freeze_path)
    if not binary.is_file():
        raise Phase24Error(f"missing native binary: {binary}")
    routes = selected_routes(freeze, "train")
    results: dict[str, Any] = {}
    for selected in routes:
        input_dir = input_root / "raw" / selected.route / selected.phone
        paths = {
            "gnss": input_dir / "device_gnss.csv",
            "imu": input_dir / "device_imu.csv",
            "nav": input_dir / "brdc.nav",
        }
        for key, path in paths.items():
            if not path.is_file():
                raise Phase24Error(f"missing raw-only input {key}: {path}")
            if path.suffix.lower() == ".mat":
                raise Phase24Error("MAT inference input rejected")
        results[selected.dataset_id] = {}
        for candidate, flags in CANDIDATE_FLAGS.items():
            destination = output_root / "candidates" / candidate / selected.route / selected.phone
            if destination.exists() and any(destination.iterdir()):
                raise Phase24Error(f"refusing to overwrite candidate output: {destination}")
            destination.mkdir(parents=True, exist_ok=True)
            submission = destination / "submission.csv"
            summary_path = destination / "summary.json"
            log_path = destination / "run.log"
            command = [
                str(binary),
                "--android-gnss", str(paths["gnss"]),
                "--android-imu", str(paths["imu"]),
                "--nav", str(paths["nav"]),
                "--out", str(submission),
                "--summary-json", str(summary_path),
                "--dataset-id", selected.dataset_id,
                "--all-epochs",
                "--android-raw-utc-keys",
                *RAW_CLOCK_FLAGS,
                *flags,
            ]
            return_code, wall_seconds, output = _run_command(command, log_path)
            if return_code != 0:
                results[selected.dataset_id][candidate] = {
                    "status": "fail-closed",
                    "return_code": return_code,
                    "wall_seconds": wall_seconds,
                    "log": str(log_path),
                    "log_sha256": sha256(log_path),
                    "command": command,
                }
                continue
            if not submission.is_file() or not summary_path.is_file():
                raise Phase24Error(f"native run returned success without outputs: {selected.dataset_id}/{candidate}")
            summary: dict[str, Any] = {}
            try:
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
            except json.JSONDecodeError as exc:
                raise Phase24Error(f"native summary is not JSON: {summary_path}") from exc
            run_manifest = {
                "schema_version": RUN_SCHEMA,
                "dataset_id": selected.dataset_id,
                "candidate": candidate,
                "freeze_record": str(freeze_path),
                "freeze_record_sha256": sha256(freeze_path),
                "raw_only_inputs": {key: {"path": str(path), "sha256": sha256(path), "bytes": path.stat().st_size} for key, path in paths.items()},
                "raw_clock_contract": {
                    "flags": list(RAW_CLOCK_FLAGS),
                    "pseudorange_source": "TimeNanos/FullBiasNanos/BiasNanos/TimeOffsetNanos/ReceivedSvTimeNanos",
                    "enriched_pseudorange_used": False,
                },
                "forbidden_inputs": {"mat": False, "truth": False, "sample": False, "precomputed_trajectory": False, "device_wls": False},
                "command": command,
                "return_code": return_code,
                "wall_seconds": wall_seconds,
                "submission": {"path": str(submission), "sha256": sha256(submission), "bytes": submission.stat().st_size},
                "summary": {"path": str(summary_path), "sha256": sha256(summary_path), "bytes": summary_path.stat().st_size},
                "log": {"path": str(log_path), "sha256": sha256(log_path), "bytes": log_path.stat().st_size},
                "truth_open_count": 0,
                "mat_member_opened": False,
                "status": "truth-free-complete",
            }
            atomic_json(destination / "run_manifest.json", run_manifest)
            results[selected.dataset_id][candidate] = {
                "status": "truth-free-complete",
                "run_manifest": str(destination / "run_manifest.json"),
                "run_manifest_sha256": sha256(destination / "run_manifest.json"),
                "submission_sha256": run_manifest["submission"]["sha256"],
                "summary_sha256": run_manifest["summary"]["sha256"],
                "wall_seconds": wall_seconds,
                "native_summary": summary,
            }
    report = {
        "schema_version": SCHEMA,
        "operation": "run-train-candidates-truth-free",
        "freeze_record": str(freeze_path),
        "freeze_record_sha256": sha256(freeze_path),
        "binary": {"path": str(binary), "sha256": sha256(binary)},
        "candidate_flags": CANDIDATE_FLAGS,
        "raw_clock_flags": RAW_CLOCK_FLAGS,
        "enriched_pseudorange_used": False,
        "truth_open_count": 0,
        "mat_member_opened": False,
        "results": results,
    }
    atomic_json(output_root / "truth_free_train_summary.json", report)
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("materialize-train", "run-train"))
    parser.add_argument("--freeze", type=Path, default=DEFAULT_FREEZE)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--input-root", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--binary", type=Path, default=ROOT / "build/apps/gnss_fgo_imu_no_base")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.operation == "materialize-train":
            report = materialize_train(args.freeze, args.archive, args.output_root)
        else:
            report = run_train_candidates(args.freeze, args.input_root, args.output_root, args.binary)
    except (OSError, Phase24Error, zipfile.BadZipFile) as exc:
        print(f"phase24: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
