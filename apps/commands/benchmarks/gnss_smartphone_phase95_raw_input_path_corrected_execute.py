#!/usr/bin/env python3
"""Execute and seal the Phase95 path-corrected diagnostic matrix.

The only Phase95 change is raw-input path materialization. The wrapper first
verifies the launch-free Phase95 contract, then resolves the pinned Phase91
route map, stat-checks the three inherited files, and substitutes those paths
into the sealed Phase95 commands. It never copies or transforms raw data. The
native process owns raw-byte reads; truth, MAT, coordinate, base, Kaggle, and
accuracy lanes are not available.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
PRE_RAW = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase95_raw_input_path_corrected.py"
)
MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase95_raw_input_path_corrected_execution_manifest_v1.json"
)
AUTHORIZATION = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase95_raw_input_path_corrected_raw_execution_authorization_v1.json"
)
PHASE91_MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase91_source_clock_c0d_gnss_first_raw_drift_d_initializer_execution_manifest_v1.json"
)
PHASE94_HELPER = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase94_source_clock_c0d_stage_diagnostics_execute.py"
)
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase95-raw-input-path-corrected-v1"
RESULT_JSON = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase95_raw_input_path_corrected_structural_result_v1.json"
)
RESULT_MD = RESULT_JSON.with_suffix(".md")

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
DOMAIN_ROWS = {
    ROUTES[0]: 2158,
    ROUTES[1]: 3139,
    ROUTES[2]: 1465,
    ROUTES[3]: 1101,
}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
RAW_FLAGS = (
    ("--android-gnss", "device_gnss.csv"),
    ("--android-imu", "device_imu.csv"),
    ("--nav", "brdc.nav"),
)
RESULT_SCHEMA = "smartphone-r5-phase95-raw-input-path-corrected-structural-result.v1"
MAX_SPEED_MPS = 70.0


class Phase95StructuralError(ValueError):
    """Raised when the path-corrected structural contract fails closed."""


def fail(message: str) -> Phase95StructuralError:
    return Phase95StructuralError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def sha256_file(path: Path, label: str) -> str:
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise fail(f"failed to hash {label}: {path}: {exc}") from exc
    return digest.hexdigest()


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(value, handle, indent=2, sort_keys=True, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def atomic_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(value)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def _load_evaluator() -> Any:
    if not PRE_RAW.is_file():
        raise fail(f"missing Phase95 pre-raw evaluator: {PRE_RAW}")
    spec = importlib.util.spec_from_file_location("phase95_path_corrected_pre_raw", PRE_RAW)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load Phase95 evaluator: {PRE_RAW}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_pinned_contract() -> tuple[Any, dict[str, Any], dict[str, Any], dict[str, Any]]:
    evaluator = _load_evaluator()
    try:
        pre_raw = evaluator.verify_pre_raw()
        manifest = evaluator.verify_manifest()
    except Exception as exc:
        raise fail(f"Phase95 pinned pre-raw contract failed: {exc}") from exc
    if pre_raw.get("raw_reads") != 0 or pre_raw.get("native_solver_invocations") != 0:
        raise fail("Phase95 pre-raw verifier reported native/raw activity")
    if pre_raw.get("accuracy_scored") is not False:
        raise fail("Phase95 pre-raw verifier reported accuracy")
    execution = manifest.get("execution_authorization")
    if not isinstance(execution, dict) or execution.get("raw_execution_authorized") is not True:
        raise fail("Phase95 raw authorization is not pinned")
    try:
        authorization = evaluator.verify_authorization(manifest)
    except Exception as exc:
        raise fail(f"Phase95 raw authorization verification failed: {exc}") from exc
    return evaluator, pre_raw, manifest, authorization


def _safe_raw_path(path_text: str, route: str, name: str) -> Path:
    path_obj = Path(path_text)
    if path_obj.is_absolute() or ".." in path_obj.parts or path_obj.name != name:
        raise fail(f"unsafe inherited raw path: {route}/{name}")
    return ROOT / path_obj


def load_inherited_raw_inputs() -> dict[str, dict[str, dict[str, Any]]]:
    """Resolve Phase91 metadata and stat actual files; never read raw bytes."""
    inherited = read_json(PHASE91_MANIFEST, "inherited Phase91 raw-input manifest")
    routes = inherited.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("inherited Phase91 route order changed")
    resolved: dict[str, dict[str, dict[str, Any]]] = {}
    for record in routes:
        route = record.get("dataset_id")
        raw = record.get("raw_inputs")
        if route not in ROUTES or not isinstance(raw, dict) or tuple(raw) != RAW_NAMES:
            raise fail(f"inherited Phase91 raw-input set changed: {route}")
        resolved[route] = {}
        for name in RAW_NAMES:
            pin = raw.get(name)
            if not isinstance(pin, dict) or not isinstance(pin.get("path"), str):
                raise fail(f"inherited Phase91 raw pin malformed: {route}/{name}")
            path_text = pin["path"]
            path = _safe_raw_path(path_text, route, name)
            digest = pin.get("sha256")
            if not isinstance(digest, str) or len(digest) != 64:
                raise fail(f"inherited Phase91 SHA pin malformed: {route}/{name}")
            if not path.is_file():
                raise fail(f"missing inherited Phase91 raw input: {path}")
            # Metadata-only provenance. Native owns the subsequent raw read.
            resolved[route][name] = {
                "path": path_text,
                "sha256": digest,
                "sha256_available": True,
                "bytes": path.stat().st_size,
                "manifest": relative(PHASE91_MANIFEST),
                "exists_before_launch": True,
                "read_by_runner": False,
            }
    return resolved


def materialize_command(
    record: dict[str, Any],
    raw: dict[str, dict[str, Any]],
    evaluator: Any,
) -> list[str]:
    route = record.get("dataset_id")
    if route not in ROUTES:
        raise fail(f"unknown Phase95 route: {route}")
    command = record.get("command")
    # The evaluator validates the frozen role placeholders before this exact
    # three-argument substitution. No other command token is changed.
    evaluator._validate_command(route, command, record)
    materialized = list(command)
    for flag, name in RAW_FLAGS:
        index = materialized.index(flag) + 1
        pin = raw.get(name)
        if not isinstance(pin, dict) or not isinstance(pin.get("path"), str):
            raise fail(f"resolved raw pin missing: {route}/{name}")
        _safe_raw_path(pin["path"], route, name)
        materialized[index] = pin["path"]
    return materialized


def execute_matrix(
    manifest: dict[str, Any],
    evaluator: Any,
) -> list[dict[str, Any]]:
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite Phase95 output root: {OUTPUT_ROOT}")
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    records = manifest.get("routes")
    if not isinstance(records, list) or [item.get("dataset_id") for item in records] != list(ROUTES):
        raise fail("Phase95 route order/count changed")
    raw_inputs = load_inherited_raw_inputs()
    environment = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = local_lib + (
        ":" + environment["LD_LIBRARY_PATH"] if environment.get("LD_LIBRARY_PATH") else ""
    )
    metadata: list[dict[str, Any]] = []
    for record in records:
        route = record["dataset_id"]
        planned = record.get("planned_output")
        if not isinstance(planned, dict):
            raise fail(f"planned output missing: {route}")
        summary_path = ROOT / planned["summary"]
        withheld_path = ROOT / planned["withheld_solution_output"]
        route_dir = summary_path.parent
        route_dir.mkdir(parents=True, exist_ok=False)
        command = materialize_command(record, raw_inputs[route], evaluator)
        stdout_path = route_dir / "stdout.log"
        stderr_path = route_dir / "stderr.log"
        started = time.time()
        return_code: int | None = None
        interrupted = False
        launch_error = ""
        with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
            try:
                completed = subprocess.run(
                    command,
                    cwd=ROOT,
                    env=environment,
                    stdout=stdout,
                    stderr=stderr,
                    check=False,
                )
                return_code = completed.returncode
            except KeyboardInterrupt:
                interrupted = True
                stderr.write(b"\nPhase95 runner interrupted; partial output preserved.\n")
            except OSError as exc:
                launch_error = str(exc)
                stderr.write(f"\nPhase95 native launch failed: {exc}\n".encode())
        route_metadata = {
            "schema_version": "smartphone-r5-phase95-raw-input-path-corrected-route-execution.v1",
            "dataset_id": route,
            "run_number": 1,
            "command": command,
            "raw_inputs": raw_inputs[route],
            "planned_output": {
                "summary": relative(summary_path),
                "withheld_solution_output": relative(withheld_path),
            },
            "summary_present_after_launch": summary_path.is_file(),
            "withheld_solution_output_present_after_launch": withheld_path.is_file(),
            "stdout": relative(stdout_path),
            "stderr": relative(stderr_path),
            "started_unix_s": started,
            "ended_unix_s": time.time(),
            "return_code": return_code,
            "interrupted": interrupted,
            "launch_error": launch_error,
            "runner_read_raw_bytes": False,
            "runner_read_truth_mat_coordinate_base_kaggle": False,
            "raw_content_copied_or_transformed": False,
        }
        atomic_json(route_dir / "run_metadata.json", route_metadata)
        metadata.append(route_metadata)
    return metadata


def _load_phase94_helpers(manifest: dict[str, Any]) -> Any:
    if not PHASE94_HELPER.is_file():
        raise fail(f"missing sealed diagnostic helper: {PHASE94_HELPER}")
    spec = importlib.util.spec_from_file_location("phase94_diagnostic_helpers", PHASE94_HELPER)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load diagnostic helper: {PHASE94_HELPER}")
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    helper.ROUTES = ROUTES
    helper.DOMAIN_ROWS = DOMAIN_ROWS
    helper.RAW_NAMES = RAW_NAMES
    helper.OUTPUT_ROOT = OUTPUT_ROOT
    helper.PRE_RAW = PRE_RAW
    helper.MANIFEST = MANIFEST
    helper.AUTHORIZATION = AUTHORIZATION
    helper.RESULT_JSON = RESULT_JSON
    helper.RESULT_MD = RESULT_MD
    helper.SCHEMA = RESULT_SCHEMA
    helper.IMPLEMENTATION_COMMIT = manifest["implementation"]["commit"]
    return helper


def build_result(
    metadata: list[dict[str, Any]],
    manifest: dict[str, Any],
    pre_raw: dict[str, Any],
    authorization: dict[str, Any],
) -> dict[str, Any]:
    helper = _load_phase94_helpers(manifest)
    result = helper.build_result(metadata, manifest, pre_raw, authorization, None)
    passed = result["gates"]["all_gates_anded"]
    result["schema_version"] = RESULT_SCHEMA
    result["phase"] = 95
    result["status"] = (
        "go-phase95-path-corrected-diagnostic-structural"
        if passed
        else "no-go-phase95-path-corrected-diagnostic-structural-gates"
    )
    result["decision"] = (
        "GO: all authorized Phase95 diagnostic gates passed; stop before truth/accuracy."
        if passed
        else "NO-GO: one or more Phase95 diagnostic gates failed; fail closed before truth/accuracy."
    )
    wrapper_hash = sha256_file(Path(__file__).resolve(), "Phase95 execution wrapper")
    evaluator_hash = sha256_file(PRE_RAW, "Phase95 evaluator")
    manifest_hash = sha256_file(MANIFEST, "Phase95 execution manifest")
    authorization_hash = sha256_file(AUTHORIZATION, "Phase95 raw authorization")
    result["evaluator"] = {"path": relative(PRE_RAW), "sha256": evaluator_hash}
    result["manifest"] = {"path": relative(MANIFEST), "sha256": manifest_hash}
    result["authorization"] = {
        "path": relative(AUTHORIZATION),
        "sha256": authorization_hash,
        "status": authorization.get("status"),
    }
    result["execution_wrapper"] = {"path": relative(Path(__file__).resolve()), "sha256": wrapper_hash}
    result["matrix"].update({
        "raw_device_gnss_reads": len(metadata),
        "raw_device_imu_reads": len(metadata),
        "broadcast_navigation_reads": len(metadata),
        "path_materializations": len(metadata),
    })
    result["read_accounting"].update({
        "raw_device_gnss_reads": len(metadata),
        "raw_device_imu_reads": len(metadata),
        "broadcast_navigation_reads": len(metadata),
        "runner_raw_input_byte_reads": 0,
        "raw_content_copied_or_transformed": False,
    })
    result["phase95_path_resolution"] = {
        "repo_root": str(ROOT),
        "cwd": str(ROOT),
        "inherited_manifest": relative(PHASE91_MANIFEST),
        "inherited_manifest_sha256": sha256_file(PHASE91_MANIFEST, "Phase91 inherited manifest"),
        "placeholder_root": "raw/phase93",
        "placeholder_root_exists": False,
        "materialization": "exact Phase91 path substitution for GNSS/IMU/nav only",
        "raw_byte_reads_by_wrapper": 0,
        "copy_or_transform": False,
    }
    return result


def result_markdown(result: dict[str, Any]) -> str:
    lines = [
        "# Phase95 raw-input path-corrected diagnostic structural result",
        "",
        f"- Status: `{result['status']}`",
        f"- Decision: {result['decision']}",
        "- Diagnostic-only: **true**; solution and accuracy output: **withheld**",
        "- Truth/MAT/precomputed-coordinate/base/Kaggle/token reads: **0**",
        "- Matrix: exactly four routes × one native invocation, sequential, no rerun/fallback",
        "",
        "## Route summary",
        "",
        "| Route | Return | Resolved raw files | GNSS accepted / cost | Main accepted / cost | Output |",
        "|---|---:|---|---|---|---|",
    ]
    for route in ROUTES:
        item = result["routes"].get(route, {})
        raw = item.get("raw_inputs", {})
        present = ",".join(name for name in RAW_NAMES if raw.get(name, {}).get("exists_before_launch")) or "none"
        telemetry = item.get("telemetry", {})
        gnss = telemetry.get("gnss_first", {}) if isinstance(telemetry, dict) else {}
        main = telemetry.get("main", {}) if isinstance(telemetry, dict) else {}
        gcost = f"{gnss.get('accepted_outer_iterations', 'n/a')} / {gnss.get('initial_cost', 'n/a')} → {gnss.get('final_cost', 'n/a')}"
        mcost = f"{main.get('accepted_outer_iterations', 'n/a')} / {main.get('initial_cost', 'n/a')} → {main.get('final_cost', 'n/a')}"
        lines.append(
            f"| `{route}` | `{item.get('return_code')}` | `{present}` | `{gcost}` | `{mcost}` | `withheld` |"
        )
        lines.append(f"  - Failure stage: `{item.get('failure_stage', 'n/a')}`; gates: `{item.get('gates', {})}`")
    lines.extend([
        "",
        "Structural diagnostics only; no truth, accuracy, or submission activity is authorized.",
        "",
    ])
    return "\n".join(lines)


def _metadata_from_existing() -> list[dict[str, Any]]:
    metadata: list[dict[str, Any]] = []
    for route in ROUTES:
        path = OUTPUT_ROOT / route.replace("/", "__") / "run_metadata.json"
        metadata.append(read_json(path, f"Phase95 route metadata {route}"))
    return metadata


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="execute exactly four authorized routes once")
    parser.add_argument("--validate", action="store_true", help="validate preserved Phase95 route metadata")
    parser.add_argument("--result-json", type=Path, default=RESULT_JSON)
    args = parser.parse_args(argv)
    if args.execute == args.validate:
        parser.error("choose exactly one of --execute or --validate")
    try:
        evaluator, pre_raw, manifest, authorization = load_pinned_contract()
        metadata = execute_matrix(manifest, evaluator) if args.execute else _metadata_from_existing()
        result = build_result(metadata, manifest, pre_raw, authorization)
        target = args.result_json.resolve()
        atomic_json(target, result)
        atomic_text(target.with_suffix(".md"), result_markdown(result))
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["gates"]["all_passed"] else 1
    except (Phase95StructuralError, OSError) as exc:
        print(f"phase95 structural runner: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
