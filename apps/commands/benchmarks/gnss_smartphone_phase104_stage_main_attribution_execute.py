#!/usr/bin/env python3
"""One-shot raw-only Phase104 native execution wrapper.

This wrapper is intentionally the only process-launch boundary for the
Phase104 candidate.  It resolves the three raw files from the sealed Phase95
path record, performs stat-only availability checks, and passes those files
directly to the native binary.  It never opens, hashes, copies, transforms,
or otherwise materializes raw bytes.  Truth and accuracy evaluation are a
later, separately authorized evaluator operation.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import gnss_smartphone_phase104_stage_main_attribution as contract  # noqa: E402


OUTPUT_ROOT = ROOT / contract.OUTPUT_RELATIVE_ROOT.rstrip("/")


def _route_dir(route: str) -> Path:
    return OUTPUT_ROOT / route.replace("/", "__")


def _raw_materialization(route: str, metadata: dict[str, dict[str, Any]]) -> dict[str, dict[str, Any]]:
    materialized: dict[str, dict[str, Any]] = {}
    for name in contract.RAW_NAMES:
        pin = metadata[route][name]
        path = ROOT / pin["path"]
        # stat() is deliberately the only wrapper-side access to the raw
        # artifact.  The native child is the authorized reader.
        try:
            stat = path.stat()
        except OSError as exc:
            raise contract.fail(f"raw path unavailable for {route}/{name}: {path}: {exc}") from exc
        if not path.is_file() or stat.st_size <= 0:
            raise contract.fail(f"raw path is not a nonempty file: {route}/{name}: {path}")
        materialized[name] = {
            "path": pin["path"],
            "sha256": pin["sha256"],
            "bytes": pin.get("bytes"),
            "exists_before_launch": True,
            "read_by_wrapper": False,
            "hash_read_by_wrapper": False,
            "stat_size_bytes": stat.st_size,
        }
    return materialized


def _command(record: dict[str, Any], raw: dict[str, dict[str, Any]]) -> list[str]:
    command = list(record["command"])
    replacements = {
        "__PHASE95_RAW_DEVICE_GNSS__": raw["device_gnss.csv"]["path"],
        "__PHASE95_RAW_DEVICE_IMU__": raw["device_imu.csv"]["path"],
        "__PHASE95_RAW_BROADCAST_NAV__": raw["brdc.nav"]["path"],
    }
    materialized = [replacements.get(token, token) for token in command]
    contract.validate_command(record["dataset_id"], materialized, record, raw=raw)
    return materialized


def _safe_environment() -> dict[str, str]:
    environment = dict(os.environ)
    # Truth/evaluation channels are not allowed into the native child even if
    # a host shell happens to carry unrelated convenience variables.
    for key in list(environment):
        upper = key.upper()
        if any(token in upper for token in ("TRUTH", "GROUND_TRUTH", "MAT", "KAGGLE")):
            environment.pop(key, None)
    environment["LC_ALL"] = "C"
    environment["LANG"] = "C"
    environment["TZ"] = "UTC"
    return environment


def _artifact_status(route: str, record: dict[str, Any]) -> tuple[dict[str, Any], list[str]]:
    planned = record["planned_output"]
    errors: list[str] = []
    artifacts: dict[str, Any] = {}
    expected = contract.DOMAIN_ROWS[route]
    checks = (
        ("solution", contract.seal_solution_metadata, ROOT / planned["solution"], expected),
        ("stage", contract.seal_stage_metadata, ROOT / planned["stage_ecef"], expected),
        ("displacement_stats", contract.seal_stats_metadata, ROOT / planned["displacement_stats"], None),
    )
    for name, checker, path, rows in checks:
        try:
            if rows is None:
                artifacts[name] = checker(path, route)
            else:
                artifacts[name] = checker(path, route, rows)
            artifacts[name]["present"] = True
        except (contract.Phase104Error, OSError) as exc:
            errors.append(f"{name}: {exc}")
            artifacts[name] = contract._file_metadata(path, f"partial {name} {route}")
            artifacts[name]["pretruth_schema_valid"] = False
    summary_path = ROOT / planned["summary"]
    artifacts["summary"] = contract._file_metadata(summary_path, f"native summary {route}")
    if artifacts["summary"].get("present") is not True:
        errors.append("summary: native summary missing")
    return artifacts, errors


def _write_route_metadata(
    route: str,
    record: dict[str, Any],
    raw: dict[str, dict[str, Any]],
    command: list[str],
    return_code: int,
    log_path: Path,
    launch_error: str | None,
) -> tuple[dict[str, Any], list[str]]:
    artifacts, artifact_errors = _artifact_status(route, record)
    errors = list(artifact_errors)
    if launch_error:
        errors.insert(0, launch_error)
    run = {
        "schema_version": "smartphone-r5-phase104-stage-main-attribution-run.v1",
        "phase": 104,
        "execution_label": "Luna Max",
        "dataset_id": route,
        "run_number": 1,
        "diagnostic_only": True,
        "raw_only_native": True,
        "command": command,
        "raw_inputs": raw,
        "raw_byte_reads_by_wrapper": 0,
        "raw_hash_reads_by_wrapper": 0,
        "truth_read_by_wrapper": False,
        "solution_output_published": False,
        "stage_sidecar_reinput": False,
        "return_code": return_code,
        "launch_error": launch_error,
        "log": {
            "path": contract.relative(log_path),
            "bytes": log_path.stat().st_size if log_path.is_file() else 0,
        },
        "planned_output": record["planned_output"],
        "artifacts": artifacts,
        "artifact_errors": errors,
        "rerun": False,
        "fallback": False,
    }
    contract.atomic_json(_route_dir(route) / "run_metadata.json", run)
    return run, errors


def execute() -> int:
    # Every invocation starts by rechecking the independent pre-raw contract
    # and the separately committed one-shot authorization.
    contract.verify_pre_raw()
    manifest = contract.verify_manifest()
    contract.verify_authorization(manifest)
    raw_metadata = contract.phase95_paths()
    if OUTPUT_ROOT.exists():
        raise contract.fail(f"Phase104 output root already exists; rerun forbidden: {OUTPUT_ROOT}")
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)

    records = manifest["routes"]
    route_runs: list[dict[str, Any]] = []
    all_errors: list[str] = []
    for record in records:
        route = record["dataset_id"]
        route_dir = _route_dir(route)
        route_dir.mkdir(parents=True, exist_ok=False)
        raw = _raw_materialization(route, raw_metadata)
        command = _command(record, raw)
        log_path = route_dir / "native.log"
        launch_error: str | None = None
        return_code = 127
        try:
            with log_path.open("w", encoding="utf-8") as log:
                completed = subprocess.run(
                    command,
                    cwd=ROOT,
                    env=_safe_environment(),
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    check=False,
                )
                return_code = int(completed.returncode)
        except OSError as exc:
            launch_error = f"native launch failed: {exc}"
        run, errors = _write_route_metadata(
            route, record, raw, command, return_code, log_path, launch_error
        )
        route_runs.append(run)
        all_errors.extend(f"{route}: {error}" for error in errors)

    execution = {
        "schema_version": "smartphone-r5-phase104-stage-main-attribution-execution.v1",
        "phase": 104,
        "execution_label": "Luna Max",
        "status": "complete-two-route-one-shot" if not all_errors else "complete-with-failures",
        "route_order": list(contract.ROUTES),
        "native_invocations": len(route_runs),
        "native_invocations_planned": 2,
        "return_codes": {run["dataset_id"]: run["return_code"] for run in route_runs},
        "raw_byte_reads_by_wrapper": 0,
        "raw_hash_reads_by_wrapper": 0,
        "truth_reads_by_wrapper": 0,
        "accuracy_calculations_by_wrapper": 0,
        "raw_content_copied_or_transformed": False,
        "stage_or_main_reinput": 0,
        "reruns": 0,
        "fallbacks": 0,
        "solution_output_published": False,
        "errors": all_errors,
        "route_runs": [run["dataset_id"] for run in route_runs],
    }
    contract.atomic_json(OUTPUT_ROOT / "execution_metadata.json", execution)
    print(json.dumps(execution, indent=2, sort_keys=True))
    return 0 if len(route_runs) == 2 and not all_errors and all(
        run["return_code"] == 0 for run in route_runs
    ) else 1


def main() -> int:
    try:
        return execute()
    except (contract.Phase104Error, OSError) as exc:
        print(f"phase104 raw wrapper: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
