#!/usr/bin/env python3
"""One-shot Phase105 linkage-fixed runner for the Phase104 raw lane.

The only functional change from the sealed Phase104 wrapper is the child
process environment: the trusted GTSAM directory is prepended to
``LD_LIBRARY_PATH``.  All raw-path resolution, output sealing, route order,
and no-rerun behavior are reused without changing the native command.
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

import gnss_smartphone_phase105_runtime_linkage_attribution as contract  # noqa: E402
import gnss_smartphone_phase104_stage_main_attribution_execute as sealed_wrapper  # noqa: E402


# Reuse the sealed Phase104 helper implementation with Phase105's redirected
# contract module and fresh output root.  No Phase104 execute() entry point is
# called, so its old authorization cannot be reused.
sealed_wrapper.contract = contract
sealed_wrapper.OUTPUT_ROOT = ROOT / contract.PHASE105_OUTPUT_RELATIVE_ROOT.rstrip("/")
OUTPUT_ROOT = sealed_wrapper.OUTPUT_ROOT


def _safe_environment() -> dict[str, str]:
    environment = dict(os.environ)
    for key in list(environment):
        upper = key.upper()
        if any(token in upper for token in ("TRUTH", "GROUND_TRUTH", "MAT", "KAGGLE")):
            environment.pop(key, None)
    local_lib = "/home/sasaki/.local/lib"
    inherited = environment.get("LD_LIBRARY_PATH", "")
    environment["LD_LIBRARY_PATH"] = local_lib + ((":" + inherited) if inherited else "")
    environment["LC_ALL"] = "C"
    environment["LANG"] = "C"
    environment["TZ"] = "UTC"
    return environment


def execute() -> int:
    contract.verify_pre_raw()
    manifest = contract.verify_manifest()
    contract.verify_authorization(manifest)
    raw_metadata = contract.phase95_paths()
    if OUTPUT_ROOT.exists():
        raise contract.fail(f"Phase105 output root already exists; rerun forbidden: {OUTPUT_ROOT}")
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)

    route_runs: list[dict[str, Any]] = []
    all_errors: list[str] = []
    for record in manifest["routes"]:
        route = record["dataset_id"]
        route_dir = sealed_wrapper._route_dir(route)
        route_dir.mkdir(parents=True, exist_ok=False)
        raw = sealed_wrapper._raw_materialization(route, raw_metadata)
        command = sealed_wrapper._command(record, raw)
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
        run, errors = sealed_wrapper._write_route_metadata(
            route, record, raw, command, return_code, log_path, launch_error
        )
        route_runs.append(run)
        all_errors.extend(f"{route}: {error}" for error in errors)

    execution = {
        "schema_version": "smartphone-r5-phase105-runtime-linkage-execution.v1",
        "phase": 105,
        "execution_label": "Luna Max",
        "status": "complete-two-route-one-shot" if not all_errors else "complete-with-failures",
        "candidate": contract.PHASE105_CANDIDATE_ID,
        "route_order": list(contract.ROUTES),
        "native_invocations": len(route_runs),
        "native_invocations_planned": 2,
        "return_codes": {run["dataset_id"]: run["return_code"] for run in route_runs},
        "runtime_linkage": {
            "child_only": True,
            "ld_library_path_prepend": "/home/sasaki/.local/lib",
            "inherited_suffix_preserved": True,
        },
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
        print(f"phase105 raw wrapper: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
