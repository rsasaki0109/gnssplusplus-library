#!/usr/bin/env python3
"""Execute the authorized Phase102 raw-only runs and isolated evaluator.

This wrapper has one native boundary: it resolves sealed Phase95 path
metadata, substitutes the three raw paths into the Phase102 command, and
launches the native binary exactly once for each fixed route.  It never opens
truth.  Candidate solution metadata is sealed only after native exit; one
separate Phase102 evaluator subprocess then performs the two truth reads.
There is no retry, fallback, route selection, or rerun.
"""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
CONTRACT_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase102_epoch_clock_vector_accuracy.py"


def _load_contract() -> Any:
    spec = importlib.util.spec_from_file_location("phase102_accuracy_contract", CONTRACT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load Phase102 contract: {CONTRACT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


C = _load_contract()


def _environment() -> dict[str, str]:
    """Pass only loader/locale variables; no user truth or evaluation lanes."""
    environment: dict[str, str] = {}
    for key in ("PATH", "LANG", "LC_ALL", "TZ"):
        value = os.environ.get(key)
        if value:
            environment[key] = value
    local_lib = "/home/sasaki/.local/lib"
    inherited = os.environ.get("LD_LIBRARY_PATH", "")
    environment["LD_LIBRARY_PATH"] = local_lib + ((":" + inherited) if inherited else "")
    return environment


def _run_process(command: list[str], cwd: Path, stdout_path: Path, stderr_path: Path, environment: dict[str, str]) -> tuple[int | None, bool, str]:
    interrupted = False
    launch_error = ""
    return_code: int | None = None
    with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
        try:
            completed = subprocess.run(command, cwd=cwd, env=environment, stdout=stdout, stderr=stderr, check=False)
            return_code = completed.returncode
        except KeyboardInterrupt:
            interrupted = True
            stderr.write(b"\nPhase102 wrapper interrupted; partial result preserved.\n")
        except OSError as exc:
            launch_error = str(exc)
            stderr.write(f"\nPhase102 process launch failed: {exc}\n".encode())
    return return_code, interrupted, launch_error


def execute() -> int:
    pre_raw = C.verify_pre_raw()
    manifest = C.verify_manifest()
    authorization = C.verify_authorization(manifest)
    if pre_raw.get("raw_reads") != 0 or pre_raw.get("native_solver_invocations") != 0 or pre_raw.get("truth_reads") != 0:
        raise C.fail("Phase102 pre-raw activity is nonzero")
    if authorization.get("status") != "authorized-for-exact-two-route-phase102-epoch-clock-vector-raw-and-isolated-truth-evaluation":
        raise C.fail("Phase102 authorization status is not exact")
    output_root = C.OUTPUT_ROOT
    if output_root.exists():
        raise C.fail(f"refusing to overwrite one-shot Phase102 output root: {output_root}")
    output_root.mkdir(parents=True, exist_ok=False)
    raw_inputs = C.phase95_raw_paths()
    environment = _environment()
    route_metadata: list[dict[str, Any]] = []
    records = manifest.get("routes")
    if not isinstance(records, list) or [record.get("dataset_id") for record in records] != list(C.ROUTES):
        raise C.fail("Phase102 route order/count changed")
    for record in records:
        route = record["dataset_id"]
        route_dir = output_root / route.replace("/", "__")
        route_dir.mkdir(parents=False, exist_ok=False)
        command = C.materialize_command(record, raw_inputs[route])
        stdout_path = route_dir / "stdout.log"
        stderr_path = route_dir / "stderr.log"
        solution_path = route_dir / "solution.csv"
        summary_path = route_dir / "summary.json"
        started = time.time()
        return_code, interrupted, launch_error = _run_process(command, ROOT, stdout_path, stderr_path, environment)
        ended = time.time()
        # Post-exit seal reads candidate output only.  It never reads raw or
        # truth bytes and does not retain coordinate values.
        solution_metadata = C.seal_solution_metadata(route, solution_path)
        solution_metadata_path = route_dir / "solution_metadata.json"
        C.atomic_json(solution_metadata_path, solution_metadata)
        summary_metadata = C._generic_file_metadata(summary_path, f"native summary {route}")
        run_metadata = {
            "schema_version": "smartphone-r5-phase102-epoch-clock-vector-route-execution.v1",
            "dataset_id": route,
            "run_number": 1,
            "command": command,
            "raw_inputs": raw_inputs[route],
            "planned_output": {"solution": C.relative(solution_path), "summary": C.relative(summary_path), "solution_metadata": C.relative(solution_metadata_path), "run_metadata": C.relative(route_dir / "run_metadata.json")},
            "solution": C.relative(solution_path),
            "summary": C.relative(summary_path),
            "solution_metadata": C.relative(solution_metadata_path),
            "summary_metadata": summary_metadata,
            "stdout": C.relative(stdout_path),
            "stderr": C.relative(stderr_path),
            "started_unix_s": started,
            "ended_unix_s": ended,
            "return_code": return_code,
            "interrupted": interrupted,
            "launch_error": launch_error,
            "native_exit_before_solution_seal": True,
            "raw_byte_reads_by_wrapper": 0,
            "raw_hash_reads_by_wrapper": 0,
            "truth_read_by_wrapper": False,
            "mat_base_pdc_precomputed_coordinate_reads": 0,
            "kaggle_or_token_access": 0,
            "raw_content_copied_or_transformed": False,
            "solution_output_published": False,
            "accuracy_scored_by_wrapper": False,
        }
        C.atomic_json(route_dir / "run_metadata.json", run_metadata)
        route_metadata.append(run_metadata)

    evaluator_stdout = output_root / "evaluator.stdout.log"
    evaluator_stderr = output_root / "evaluator.stderr.log"
    evaluator_command = [sys.executable, str(CONTRACT_PATH), "--evaluate", "--output-root", str(output_root), "--result-json", str(C.RESULT_JSON)]
    evaluator_started = time.time()
    evaluator_return, evaluator_interrupted, evaluator_error = _run_process(evaluator_command, ROOT, evaluator_stdout, evaluator_stderr, _environment())
    evaluator_ended = time.time()
    execution_metadata = {
        "schema_version": "smartphone-r5-phase102-epoch-clock-vector-execution.v1",
        "candidate": C.CANDIDATE_ID,
        "route_order": list(C.ROUTES),
        "native_route_metadata": [C.relative(output_root / route.replace("/", "__") / "run_metadata.json") for route in C.ROUTES],
        "native_invocations": len(route_metadata),
        "native_raw_inputs_only": True,
        "wrapper_raw_byte_reads": 0,
        "wrapper_truth_reads": 0,
        "wrapper_mat_base_pdc_precomputed_coordinate_reads": 0,
        "wrapper_kaggle_or_token_access": 0,
        "raw_content_copied_or_transformed": False,
        "evaluator_subprocess": {"command": evaluator_command, "started_unix_s": evaluator_started, "ended_unix_s": evaluator_ended, "return_code": evaluator_return, "interrupted": evaluator_interrupted, "launch_error": evaluator_error, "truth_reads_are_evaluator_only": True, "stdout": C.relative(evaluator_stdout), "stderr": C.relative(evaluator_stderr)},
        "solution_output_published": False,
        "reruns": 0,
        "fallbacks": 0,
    }
    C.atomic_json(output_root / "execution_metadata.json", execution_metadata)
    return 2 if evaluator_return is None else evaluator_return


def main() -> int:
    try:
        return execute()
    except Exception as exc:
        print(f"phase102 accuracy execution: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
