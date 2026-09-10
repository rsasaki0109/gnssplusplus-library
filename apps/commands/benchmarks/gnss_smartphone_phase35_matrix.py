#!/usr/bin/env python3
"""Run and seal the frozen Phase35 raw-only feature matrix.

The matrix contains only existing, separately frozen native features.  This
command opens raw ``device_gnss.csv``, ``device_imu.csv``, and ``brdc.nav``
inputs and invokes the native binary; it never opens ground truth, MAT files,
sample coordinates, or a previous trajectory.  Scoring is intentionally a
separate post-structural operation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import shutil
import subprocess
import sys
import tempfile
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_phase31_quality_anchor_structural as phase31  # noqa: E402


SCHEMA = "smartphone-r5-phase35-matrix.v1"
RUN_SCHEMA = "smartphone-r5-phase35-matrix-truth-free-run.v1"
SEAL_SCHEMA = "smartphone-r5-phase35-matrix-truth-free-seal.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase35-matrix-manifest.v1"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase35_matrix_freeze_v1.json"
FREEZE_SHA256 = "9be22fd18dd3f818a8c902dafde2aa7b5828ded13df98b138682863e8f14a92a"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase35-matrix-v1"
MAX_SPEED_MPS = 70.0

ROUTES: tuple[dict[str, Any], ...] = (
    {
        "dataset_id": "2021-03-16-18-59-us-ca-mtv-a/pixel5",
        "raw_root": "output/smartphone-r5/phase24-route-group-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5",
    },
    {
        "dataset_id": "2021-07-27-19-49-us-ca-mtv-b/pixel4",
        "raw_root": "output/smartphone-r5/phase24-route-group-eval-v1/raw/2021-07-27-19-49-us-ca-mtv-b/pixel4",
    },
    {
        "dataset_id": "2021-07-14-20-50-us-ca-mtv-e/sm-g988b",
        "raw_root": "output/smartphone-r5/phase24-route-group-eval-v1/raw/2021-07-14-20-50-us-ca-mtv-e/sm-g988b",
    },
    {
        "dataset_id": "2023-05-09-21-32-us-ca-mtv-pe1/pixel5",
        "raw_root": "output/smartphone-r5/phase33-quality-anchor-validation-v1/inputs/2023-05-09-21-32-us-ca-mtv-pe1/pixel5",
    },
)

BASE_FLAGS: tuple[str, ...] = (
    "--native-pdc-imu-tdcp-no-bridge",
    "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback",
    "--android-raw-utc-keys",
    "--native-quality-anchor",
)
MATRIX_FLAGS: dict[str, tuple[str, ...]] = {
    "control_quality_anchor_base": BASE_FLAGS,
    "A_anchor_signal_iono_hatch": BASE_FLAGS
    + (
        "--native-signal-bias-states",
        "--native-residual-ionosphere",
        "--native-carrier-code-leveling",
        "--native-carrier-code-innovation-reset",
        "--native-carrier-code-gal-e1-e5a",
    ),
    "B_A_plus_stop": BASE_FLAGS
    + (
        "--native-signal-bias-states",
        "--native-residual-ionosphere",
        "--native-carrier-code-leveling",
        "--native-carrier-code-innovation-reset",
        "--native-carrier-code-gal-e1-e5a",
        "--native-upstream-stop-constraints",
    ),
    "C_A_plus_position_offset": BASE_FLAGS
    + (
        "--native-signal-bias-states",
        "--native-residual-ionosphere",
        "--native-carrier-code-leveling",
        "--native-carrier-code-innovation-reset",
        "--native-carrier-code-gal-e1-e5a",
        "--native-upstream-position-offset",
    ),
}
LANE_NAMES = tuple(MATRIX_FLAGS)
RAW_NAMES = {"gnss": "device_gnss.csv", "imu": "device_imu.csv", "nav": "brdc.nav"}


class Phase35Error(ValueError):
    """Raised when the frozen raw-only matrix contract cannot be proven."""


def sha256(path: Path) -> str:
    if not path.is_file():
        raise Phase35Error(f"missing file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise Phase35Error(f"invalid {label}: {path}") from exc
    if not isinstance(value, dict):
        raise Phase35Error(f"{label} must be an object: {path}")
    return value


def atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "wb") as handle:
            fd = -1
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = ""
        try:
            directory = os.open(path.parent, getattr(os, "O_DIRECTORY", 0))
        except OSError:
            directory = -1
        if directory >= 0:
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


def verify_freeze() -> dict[str, Any]:
    if sha256(FREEZE) != FREEZE_SHA256:
        raise Phase35Error("Phase35 freeze hash changed")
    freeze = load_json(FREEZE, "Phase35 freeze")
    if freeze.get("schema_version") != "smartphone-r5-phase35-matrix-freeze.v1" or freeze.get("status") != "frozen-before-phase35-structural-runs":
        raise Phase35Error("Phase35 freeze schema/status mismatch")
    matrix = freeze.get("matrix")
    base_control = freeze.get("base_control")
    if not isinstance(base_control, dict) or base_control.get("flags") != list(BASE_FLAGS):
        raise Phase35Error("Phase35 base control flags changed")
    if not isinstance(matrix, dict) or set(matrix) != set(LANE_NAMES) - {"control_quality_anchor_base"}:
        raise Phase35Error("Phase35 matrix lanes changed")
    for lane, flags in MATRIX_FLAGS.items():
        if lane == "control_quality_anchor_base":
            continue
        if matrix[lane].get("flags") != list(flags):
            raise Phase35Error(f"Phase35 flags changed: {lane}")
    cohort = freeze.get("cohort")
    if not isinstance(cohort, dict) or cohort.get("route_disjoint") is not True:
        raise Phase35Error("Phase35 route split contract failed")
    truth_policy = freeze.get("truth_policy")
    if not isinstance(truth_policy, dict) or truth_policy.get("phase35_truth_open_count_before_structural") != 0 or truth_policy.get("mat_read_or_generated") is not False:
        raise Phase35Error("Phase35 truth policy failed")
    if freeze.get("raw_inputs", {}).get("allowed_members") != ["device_gnss.csv", "device_imu.csv", "brdc.nav"]:
        raise Phase35Error("Phase35 raw member contract changed")
    pins = freeze.get("source_and_binary_pins_before_structural_run")
    if not isinstance(pins, dict):
        raise Phase35Error("Phase35 source pins missing")
    for relative_path, expected in pins.items():
        if sha256(ROOT / relative_path) != expected:
            raise Phase35Error(f"Phase35 source pin mismatch: {relative_path}")
    return freeze


def _route_parts(dataset_id: str) -> tuple[str, str]:
    route, phone = dataset_id.split("/", 1)
    return route, phone


def _input_paths(spec: dict[str, Any]) -> dict[str, Path]:
    raw_root = ROOT / str(spec["raw_root"])
    paths = {key: raw_root / name for key, name in RAW_NAMES.items()}
    for key, path in paths.items():
        if not path.is_file() or path.suffix.lower() == ".mat":
            raise Phase35Error(f"missing/forbidden raw input {key}: {path}")
    return paths


def _target_keys(paths: dict[str, Path]) -> list[int]:
    keys = phase31.read_raw_epoch_keys(paths["gnss"])
    return keys[1:]


def _validate_summary(summary_path: Path, dataset_id: str) -> dict[str, Any]:
    summary = load_json(summary_path, "Phase35 native summary")
    projection = phase31.validate_summary(summary, dataset_id)
    if summary.get("truth_used") is not False or summary.get("production_default_changed") is not False:
        raise Phase35Error(f"truth/default contract failed: {dataset_id}")
    return projection


def _validate_output(run_dir: Path, dataset_id: str, target_keys: list[int]) -> dict[str, Any]:
    submission = run_dir / "submission.csv"
    summary = run_dir / "summary.json"
    if not submission.is_file() or not summary.is_file():
        raise Phase35Error(f"native output incomplete: {run_dir}")
    rows = phase31.read_prediction(submission, dataset_id)
    if [row[0] for row in rows] != target_keys:
        raise Phase35Error(f"exact raw target keys failed: {run_dir}")
    speed = phase31.speed_report(rows)
    if not speed["finite"] or speed["over_70_mps_count"] != 0:
        raise Phase35Error(f"continuity safety failed: {run_dir}")
    projection = _validate_summary(summary, dataset_id)
    return {
        "submission": {"path": str(submission.relative_to(ROOT)), "sha256": sha256(submission), "bytes": submission.stat().st_size, "rows": len(rows)},
        "summary": {"path": str(summary.relative_to(ROOT)), "sha256": sha256(summary), "bytes": summary.stat().st_size},
        "projection": projection,
        "speed": speed,
    }


def _command(paths: dict[str, Path], dataset_id: str, lane: str, run_dir: Path) -> list[str]:
    """Build the fixed native command for one matrix lane/run."""

    return [
        str(BINARY),
        "--android-gnss", str(paths["gnss"]),
        "--android-imu", str(paths["imu"]),
        "--nav", str(paths["nav"]),
        "--out", str(run_dir / "submission.csv"),
        "--summary-json", str(run_dir / "summary.json"),
        "--dataset-id", dataset_id,
        "--all-epochs",
        *MATRIX_FLAGS[lane],
    ]


def _record_existing(
    run_dir: Path,
    spec: dict[str, Any],
    lane: str,
    run_number: int,
    *,
    target_keys: list[int],
    paths: dict[str, Path],
    wall_seconds: float = 0.0,
    max_rss_kb_process: int = 0,
) -> dict[str, Any]:
    """Validate an already published run without invoking the native solver."""

    dataset_id = str(spec["dataset_id"])
    log = run_dir / "run.log"
    if not log.is_file():
        raise Phase35Error(f"native output missing run.log: {run_dir}")
    artifact = _validate_output(run_dir, dataset_id, target_keys)
    return {
        "status": "truth-free-complete",
        "dataset_id": dataset_id,
        "lane": lane,
        "run_number": run_number,
        "command": _command(paths, dataset_id, lane, run_dir),
        "return_code": 0,
        "wall_seconds": wall_seconds,
        "max_rss_kb_process": max_rss_kb_process,
        "raw_inputs": {key: {"path": str(path.relative_to(ROOT)), "sha256": sha256(path), "bytes": path.stat().st_size} for key, path in paths.items()},
        "log": {"path": str(log.relative_to(ROOT)), "sha256": sha256(log), "bytes": log.stat().st_size},
        **artifact,
        "truth_open_count": 0,
        "mat_read_or_generated": False,
    }


def _run_native(
    run_dir: Path,
    spec: dict[str, Any],
    lane: str,
    run_number: int,
    *,
    target_keys: list[int],
    paths: dict[str, Path],
) -> dict[str, Any]:
    """Execute one fixed native run into a fresh directory."""

    dataset_id = str(spec["dataset_id"])
    if run_dir.exists() and (not run_dir.is_dir() or any(run_dir.iterdir())):
        raise Phase35Error(f"refusing to overwrite matrix run: {run_dir}")
    route_name, phone = _route_parts(dataset_id)
    run_dir.mkdir(parents=True, exist_ok=True)
    del route_name, phone  # route/phone are encoded by the caller's directory.
    command = _command(paths, dataset_id, lane, run_dir)
    environment = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = local_lib + ((":" + environment["LD_LIBRARY_PATH"]) if environment.get("LD_LIBRARY_PATH") else "")
    started = time.perf_counter()
    process = subprocess.run(command, cwd=ROOT, env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
    wall = time.perf_counter() - started
    log = run_dir / "run.log"
    atomic_write(log, process.stdout.encode("utf-8"))
    if process.returncode != 0:
        raise Phase35Error(f"native exit {process.returncode}: {dataset_id}/{lane}/run{run_number}")
    return _record_existing(
        run_dir,
        spec,
        lane,
        run_number,
        target_keys=target_keys,
        paths=paths,
        wall_seconds=wall,
        max_rss_kb_process=resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
    )


def _run_one(output_root: Path, spec: dict[str, Any], lane: str, run_number: int, *, target_keys: list[int]) -> dict[str, Any]:
    paths = _input_paths(spec)
    dataset_id = str(spec["dataset_id"])
    route_name, phone = _route_parts(dataset_id)
    run_dir = output_root / lane / route_name / phone / f"run{run_number}"
    return _run_native(run_dir, spec, lane, run_number, target_keys=target_keys, paths=paths)


def _ensure_run(
    output_root: Path,
    spec: dict[str, Any],
    lane: str,
    run_number: int,
    *,
    target_keys: list[int],
) -> dict[str, Any]:
    """Resume one run while preserving every existing byte.

    A complete run is only validated and reused.  A directory missing one or
    more published files is rerun in a private staging directory; existing
    files must byte-match the rerun before missing files are atomically added.
    This makes an interrupted matrix resumable without silently replacing a
    partial or inconsistent artifact.
    """

    paths = _input_paths(spec)
    dataset_id = str(spec["dataset_id"])
    route_name, phone = _route_parts(dataset_id)
    run_dir = output_root / lane / route_name / phone / f"run{run_number}"
    required = ("submission.csv", "summary.json", "run.log")
    if run_dir.exists() and not run_dir.is_dir():
        raise Phase35Error(f"matrix run is not a directory: {run_dir}")
    if run_dir.is_dir() and all((run_dir / name).is_file() for name in required):
        return _record_existing(run_dir, spec, lane, run_number, target_keys=target_keys, paths=paths)

    run_dir.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=f".{run_dir.name}.phase35-resume-", dir=str(run_dir.parent)))
    try:
        _run_native(stage, spec, lane, run_number, target_keys=target_keys, paths=paths)
        if run_dir.exists():
            for name in required:
                existing = run_dir / name
                staged = stage / name
                if existing.exists() and (not staged.is_file() or sha256(existing) != sha256(staged)):
                    raise Phase35Error(f"partial matrix run differs from deterministic resume: {existing}")
        else:
            run_dir.mkdir(parents=True)
        for name in required:
            existing = run_dir / name
            staged = stage / name
            if not existing.exists():
                if not staged.is_file():
                    raise Phase35Error(f"staged native output missing: {staged}")
                os.replace(staged, existing)
        return _record_existing(run_dir, spec, lane, run_number, target_keys=target_keys, paths=paths)
    finally:
        shutil.rmtree(stage, ignore_errors=True)


def _new_matrix_report() -> dict[str, Any]:
    return {
        "schema_version": RUN_SCHEMA,
        "status": "truth-free-in-progress",
        "freeze": {"path": str(FREEZE.relative_to(ROOT)), "sha256": FREEZE_SHA256},
        "binary": {"path": str(BINARY.relative_to(ROOT)), "sha256": sha256(BINARY)},
        "routes": {},
        "lanes": list(LANE_NAMES),
        "truth_open_count": 0,
        "mat_read_or_generated": False,
        "solver_rerun_after_truth": False,
    }


def run_matrix(output_root: Path = DEFAULT_OUTPUT, *, resume: bool = False) -> dict[str, Any]:
    verify_freeze()
    output_root = output_root.resolve()
    if output_root.exists() and not output_root.is_dir():
        raise Phase35Error(f"Phase35 output root is not a directory: {output_root}")
    if output_root.exists() and any(output_root.iterdir()) and not resume:
        raise Phase35Error(f"refusing to overwrite Phase35 output: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    report = _new_matrix_report()
    failures: list[str] = []
    for spec in ROUTES:
        dataset_id = str(spec["dataset_id"])
        route_report: dict[str, Any] = {"raw_root": spec["raw_root"], "lanes": {}}
        try:
            target_keys = _target_keys(_input_paths(spec))
            route_report["target_epoch_count"] = len(target_keys)
        except (OSError, Phase35Error, ValueError) as exc:
            failures.append(f"{dataset_id}:input:{exc}")
            route_report["status"] = "fail-closed"
            route_report["error"] = str(exc)
            report["routes"][dataset_id] = route_report
            continue
        for lane in LANE_NAMES:
            lane_report: dict[str, Any] = {"status": "fail-closed", "lane": lane}
            try:
                first = _ensure_run(output_root, spec, lane, 1, target_keys=target_keys)
                repeat = _ensure_run(output_root, spec, lane, 2, target_keys=target_keys)
                identical = first["submission"]["sha256"] == repeat["submission"]["sha256"] and first["summary"]["sha256"] == repeat["summary"]["sha256"]
                lane_report = {"status": "truth-free-complete" if identical else "fail-closed", "first": first, "repeat": repeat, "repeat_byte_identical": identical}
                if not identical:
                    failures.append(f"{dataset_id}/{lane}:repeat-not-identical")
            except (OSError, Phase35Error, ValueError) as exc:
                failures.append(f"{dataset_id}/{lane}:{exc}")
                lane_report["error"] = str(exc)
            route_report["lanes"][lane] = lane_report
        report["routes"][dataset_id] = route_report
        # A checkpoint is published after each route.  If the process is
        # interrupted, resume can validate and reuse completed run directories.
        report["structural_failures"] = failures
        atomic_json(output_root / "truth_free_matrix.json", report)
    report["structural_failures"] = failures
    report["status"] = "truth-free-complete" if not failures else "truth-free-complete-with-structural-failures"
    atomic_json(output_root / "truth_free_matrix.json", report)
    return report


def seal_matrix(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    verify_freeze()
    output_root = output_root.resolve()
    report = load_json(output_root / "truth_free_matrix.json", "Phase35 truth-free matrix")
    if report.get("schema_version") != RUN_SCHEMA or report.get("status") not in {"truth-free-complete", "truth-free-complete-with-structural-failures"} or report.get("truth_open_count") != 0 or report.get("mat_read_or_generated") is not False:
        raise Phase35Error("Phase35 truth-free matrix contract failed")
    failures = list(report.get("structural_failures") or [])
    lanes: dict[str, Any] = {}
    for dataset_id, route_report in report.get("routes", {}).items():
        route_lanes = route_report.get("lanes", {})
        lanes[dataset_id] = {}
        for lane in LANE_NAMES:
            item = route_lanes.get(lane, {})
            if item.get("status") != "truth-free-complete" or item.get("repeat_byte_identical") is not True:
                failures.append(f"{dataset_id}/{lane}:incomplete")
                lanes[dataset_id][lane] = {"status": item.get("status", "missing"), "error": item.get("error")}
                continue
            first = item["first"]
            if first["speed"]["over_70_mps_count"] != 0:
                failures.append(f"{dataset_id}/{lane}:over-70-mps")
            lanes[dataset_id][lane] = {
                "status": "sealed",
                "submission": first["submission"],
                "summary": first["summary"],
                "speed": first["speed"],
                "projection": first["projection"],
                "repeat_byte_identical": True,
            }
    passed = not failures
    seal = {
        "schema_version": SEAL_SCHEMA,
        "status": "sealed-truth-free-structural-pass" if passed else "sealed-truth-free-structural-no-go",
        "decision": "truth-evaluation-authorized" if passed else "do-not-open-development-truth",
        "freeze": {"path": str(FREEZE.relative_to(ROOT)), "sha256": FREEZE_SHA256},
        "truth_free_matrix": {"path": str((output_root / "truth_free_matrix.json").relative_to(ROOT)), "sha256": sha256(output_root / "truth_free_matrix.json")},
        "routes": lanes,
        "structural_gate": {
            "route_count": len(lanes),
            "lane_count": len(LANE_NAMES),
            "finite_converged_exact_keys": passed,
            "repeat_byte_identical": passed,
            "continuity_over_70_mps_zero": passed,
            "failures": failures,
            "passed": passed,
            "truth_open_count": 0,
            "mat_read_or_generated": False,
        },
        "candidate_matrix_selection": "truth is not read unless every route/lane structural gate passes",
        "solver_rerun_after_truth": False,
    }
    atomic_json(output_root / "truth_free_seal.json", seal)
    atomic_json(output_root / "truth_free_seal.manifest.json", {
        "schema_version": MANIFEST_SCHEMA,
        "seal": {"path": str((output_root / "truth_free_seal.json").relative_to(ROOT)), "sha256": sha256(output_root / "truth_free_seal.json")},
        "truth_open_count": 0,
        "mat_read_or_generated": False,
        "atomic_publish": True,
    })
    return seal


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("run", "resume", "seal"))
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        if args.operation == "run":
            result = run_matrix(args.output_root)
        elif args.operation == "resume":
            result = run_matrix(args.output_root, resume=True)
        else:
            result = seal_matrix(args.output_root)
    except (OSError, Phase35Error, ValueError) as exc:
        print(f"phase35: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"status": result.get("status"), "truth_open_count": result.get("truth_open_count"), "operation": args.operation}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
