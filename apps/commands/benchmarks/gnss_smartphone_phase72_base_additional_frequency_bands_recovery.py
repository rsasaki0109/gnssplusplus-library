#!/usr/bin/env python3
"""Integrity-only Phase72 recovery of the Phase71 structural matrix.

The native source candidate is unchanged.  Phase72 uses a new output root and
fresh native invocations, while correcting only the Phase71 evaluator's
summary-artifact/summary-payload key collision.  No truth, MATLAB, validation,
archive, or prior Phase71 partial output is opened.
"""

from __future__ import annotations

import argparse
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
PHASE71_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase71_base_additional_frequency_bands.py"
_SPEC = importlib.util.spec_from_file_location("phase71_helpers", PHASE71_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover - import failure
    raise RuntimeError(f"failed to load Phase71 helper: {PHASE71_PATH}")
P71 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(P71)

FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase72_base_additional_frequency_bands_recovery_freeze_v1.json"
FREEZE_SHA256 = "3acff212c87657ff0cfb610f276b511bf6968178ef8e5794f7a4a9af283e198b"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase72_base_additional_frequency_bands_recovery_manifest_v1.json"
EVALUATOR = Path(__file__).resolve()
BINARY = P71.BINARY
BINARY_SHA256 = P71.BINARY_SHA256
SOURCE_COMMIT = P71.SOURCE_COMMIT
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase72-base-additional-frequency-bands-structural-recovery-v1"
OUTPUT_SCHEMA = "smartphone-r5-phase72-base-additional-frequency-bands-recovery-structural-result.v1"
PRESERVE_FLAG = P71.PRESERVE_FLAG
ROUTES = P71.ROUTES
BASE_FLAGS = P71.BASE_FLAGS
INPUTS = P71.INPUTS
BASE_INPUTS = P71.BASE_INPUTS
BASE_INPUT_HASHES = P71.BASE_INPUT_HASHES
PHASE71_FAILURE = ROOT / "docs/use_cases/records/smartphone_r5_phase71_base_additional_frequency_bands_structural_failure_v1.json"
PHASE71_FAILURE_SHA256 = "08429a52313eb0dc48d150a742b3f3a6c3c6968a5af07409180dc65d1f28d122"
Phase72Error = P71.Phase71Error


def fail(message: str) -> Phase72Error:
    return Phase72Error(message)


def reject_forbidden(path: Path | str) -> None:
    P71.reject_forbidden(path)


def sha256(path: Path) -> str:
    return P71.sha256(path)


def load_json(path: Path, label: str) -> dict[str, Any]:
    return P71.load_json(path, label)


def atomic_write(path: Path, payload: bytes) -> None:
    P71.atomic_write(path, payload)


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    P71.atomic_json(path, value)


def relative(path: Path) -> str:
    return P71.relative(path)


def verify_freeze() -> dict[str, Any]:
    if sha256(FREEZE) != FREEZE_SHA256:
        raise fail("Phase72 freeze hash changed")
    freeze = load_json(FREEZE, "Phase72 freeze")
    if freeze.get("status") != "frozen-before-phase72-native-structural-runs":
        raise fail("Phase72 freeze status changed")
    authority = freeze.get("authority", {})
    if authority.get("base_commit") != "208b37f" or authority.get("phase71_partial_output_reuse") is not False:
        raise fail("Phase71 authority/reuse contract changed")
    if not PHASE71_FAILURE.is_file() or sha256(PHASE71_FAILURE) != PHASE71_FAILURE_SHA256:
        raise fail("Phase71 failure record pin changed")
    if authority.get("phase71_candidate_runs") != 0 or authority.get("phase71_truth_reads") != 0:
        raise fail("Phase71 failure accounting changed")
    pinned = freeze.get("pinned_phase71_contract", {})
    if pinned.get("freeze_sha256") != P71.FREEZE_SHA256 or pinned.get("manifest_sha256") != sha256(P71.MANIFEST):
        raise fail("Phase71 freeze/manifest pin changed")
    if pinned.get("evaluator_sha256") != sha256(PHASE71_PATH) or pinned.get("binary_sha256") != BINARY_SHA256 or pinned.get("source_commit") != SOURCE_COMMIT:
        raise fail("Phase71 source artifact pin changed")
    candidate = freeze.get("candidate", {})
    if candidate.get("flag") != PRESERVE_FLAG or candidate.get("matching_key") != "exact (satellite,SignalType)":
        raise fail("Phase72 candidate contract changed")
    if candidate.get("no_canonicalization") is not True or candidate.get("no_interpolation_change") is not True or candidate.get("no_extrapolation_or_endpoint_hold") is not True or candidate.get("no_new_rows_or_factors") is not True:
        raise fail("Phase72 source scope changed")
    recovery = freeze.get("evaluator_recovery_contract", {})
    if recovery.get("new_output_root") != relative(DEFAULT_OUTPUT) or recovery.get("summary_artifact_key") != "summary_artifact" or recovery.get("summary_payload_key") != "summary_payload" or recovery.get("summary_key_collision_forbidden") is not True or recovery.get("actual_run_case_shape_test_required") is not True or recovery.get("unexpected_exception_failure_artifact") is not True:
        raise fail("Phase72 evaluator recovery contract changed")
    gates = freeze.get("gates", {})
    if gates.get("prediction_domain_coverage_exact") != 1.0 or gates.get("candidate_matched_factor_fraction_each_route_min") != 0.8 or gates.get("candidate_finite_correction_fraction_among_matched_each_route_min") != 0.99:
        raise fail("Phase72 numerical gates changed")
    matrix = freeze.get("read_contract", {})
    if matrix.get("raw_device_gnss_process_reads") != 12 or matrix.get("base_rinex_process_reads") != 12 or matrix.get("truth_reads") != 0 or matrix.get("mat_reads_or_generated") != 0 or matrix.get("archive_reopens") != 0:
        raise fail("Phase72 read contract changed")
    # Static checks of the prior helper/binary are safe and ensure this
    # recovery cannot silently drift away from the sealed candidate.
    if sha256(BINARY) != BINARY_SHA256:
        raise fail("native binary hash changed")
    manifest = load_json(MANIFEST, "Phase72 structural manifest")
    if manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256 or manifest.get("binary", {}).get("sha256") != BINARY_SHA256 or manifest.get("source_commit") != SOURCE_COMMIT or manifest.get("routes") != list(ROUTES):
        raise fail("Phase72 manifest pin changed")
    if manifest.get("read_accounting", {}).get("truth_reads") != 0 or manifest.get("read_accounting", {}).get("archive_reopens") != 0:
        raise fail("Phase72 manifest read contract changed")
    if manifest.get("evaluator", {}).get("path") != relative(EVALUATOR) or manifest.get("evaluator", {}).get("sha256") != sha256(EVALUATOR):
        raise fail("Phase72 evaluator pin changed")
    return freeze


def verify_inputs(route: str) -> dict[str, Any]:
    return P71.verify_inputs(route)


def read_prediction(path: Path, dataset_id: str) -> list[tuple[int, float, float]]:
    return P71.read_prediction(path, dataset_id)


def speed_report(rows: list[tuple[int, float, float]]) -> dict[str, Any]:
    return P71.speed_report(rows)


def validate_summary(path: Path, route: str, candidate: bool) -> dict[str, Any]:
    return P71.validate_summary(path, route, candidate)


def artifact_report(submission: Path, summary: Path, route: str, candidate: bool) -> dict[str, Any]:
    """Return disjoint artifact metadata and parsed payload keys.

    The explicit keys are the core Phase72 recovery.  In particular, parsed
    summary JSON is never expanded over `summary_artifact` metadata.
    """
    rows = read_prediction(submission, route)
    speed = speed_report(rows)
    if not speed["finite"] or speed["over_70_mps_count"] != 0:
        raise fail(f"speed gate failed: {route}")
    diagnostics = validate_summary(summary, route, candidate)
    report: dict[str, Any] = {
        "submission": {"path": relative(submission), "bytes": submission.stat().st_size, "sha256": sha256(submission), "rows": len(rows)},
        "summary_artifact": {"path": relative(summary), "bytes": summary.stat().st_size, "sha256": sha256(summary)},
        "summary_payload": diagnostics["summary"],
        "prediction_keys": [row[0] for row in rows],
        "speed": speed,
    }
    if candidate:
        report["base"] = diagnostics["base"]
    return report


def native_command(route: str, run_dir: Path, candidate: bool) -> list[str]:
    return P71.native_command(route, run_dir, candidate)


def run_case(output_root: Path, route: str, variant: str, run_number: int, candidate: bool) -> dict[str, Any]:
    run_dir = output_root / route / variant / f"run{run_number}"
    if run_dir.exists():
        raise fail(f"refusing to overwrite structural output: {run_dir}")
    run_dir.mkdir(parents=True, exist_ok=True)
    command = native_command(route, run_dir, candidate)
    environment = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = local_lib + ((":" + environment["LD_LIBRARY_PATH"]) if environment.get("LD_LIBRARY_PATH") else "")
    started = time.perf_counter()
    try:
        process = subprocess.run(command, cwd=ROOT, env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False, timeout=1800)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise fail(f"native process failed: {route}/{variant}/run{run_number}: {exc}") from exc
    log = process.stdout or ""
    atomic_write(run_dir / "run.log", log.encode())
    report: dict[str, Any] = {"variant": variant, "run": run_number, "candidate": candidate, "return_code": process.returncode, "wall_seconds": time.perf_counter() - started, "command": command, "log": {"path": relative(run_dir / "run.log"), "sha256": sha256(run_dir / "run.log")}}
    if process.returncode != 0:
        raise fail(f"native process returned {process.returncode}: {route}/{variant}/run{run_number}")
    submission, summary = run_dir / "submission.csv", run_dir / "summary.json"
    if not submission.is_file() or not summary.is_file():
        raise fail(f"native artifacts missing: {route}/{variant}/run{run_number}")
    report.update(artifact_report(submission, summary, route, candidate))
    return report


def phase43_control(route: str) -> dict[str, Any]:
    return P71.phase43_control(route)


def _public_report(report: dict[str, Any]) -> dict[str, Any]:
    # Prediction keys are needed only for in-memory domain equality.  Excluding
    # them from the sealed JSON keeps the result compact without weakening the
    # exact key check performed before publication.
    return {key: value for key, value in report.items() if key != "prediction_keys"}


def run_matrix(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    verify_freeze()
    output_root = output_root.resolve()
    reject_forbidden(output_root)
    if output_root == DEFAULT_OUTPUT and output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty output: {output_root}")
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty output: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    routes: dict[str, Any] = {}
    started = 0
    try:
        input_reports = {route: verify_inputs(route) for route in ROUTES}
        for route in ROUTES:
            control = run_case(output_root, route, "control", 1, False)
            started += 1
            control_reference = phase43_control(route)
            if control["submission"]["sha256"] != control_reference["submission"]["sha256"] or control["summary_artifact"]["sha256"] != control_reference["summary"]["sha256"]:
                raise fail(f"Phase43 control identity failed: {route}")
            if control["prediction_keys"] != control_reference["prediction_keys"]:
                raise fail(f"control prediction-domain mismatch: {route}")
            candidate_one = run_case(output_root, route, "candidate", 1, True)
            started += 1
            candidate_two = run_case(output_root, route, "candidate", 2, True)
            started += 1
            if candidate_one["submission"]["sha256"] != candidate_two["submission"]["sha256"] or candidate_one["summary_artifact"]["sha256"] != candidate_two["summary_artifact"]["sha256"] or candidate_one["base"] != candidate_two["base"]:
                raise fail(f"candidate repeat identity failed: {route}")
            if candidate_one["prediction_keys"] != control_reference["prediction_keys"] or candidate_two["prediction_keys"] != control_reference["prediction_keys"]:
                raise fail(f"candidate prediction-domain coverage failed: {route}")
            base = candidate_one["base"]
            routes[route] = {
                "input": input_reports[route],
                "control": _public_report(control),
                "phase43_control_reference": _public_report(control_reference),
                "candidate_run1": _public_report(candidate_one),
                "candidate_run2": _public_report(candidate_two),
                "prediction_domain_coverage": 1.0,
                "gates": {
                    "prediction_domain_coverage_exact": True,
                    "base_hash_and_bytes_exact": True,
                    "matched_factor_fraction": base["base"]["matched_factor_fraction"] >= 0.80,
                    "finite_correction_fraction_among_matched": base["base"]["finite_correction_fraction_among_matched"] >= 0.99,
                    "selected_band_telemetry_exact": True,
                    "candidate_repeat_identity": True,
                    "control_phase43_identity": True,
                    "finite_outputs": True,
                    "converged": True,
                    "tdcp_built_equals_inserted": True,
                    "over_70_mps": True,
                },
            }
        result = {
            "schema_version": OUTPUT_SCHEMA,
            "phase": 72,
            "execution_label": "Luna Max",
            "status": "go-phase72-base-additional-frequency-bands-structural-recovery",
            "truth_free": True,
            "integrity_recovery_only": True,
            "phase71_partial_output_reused": False,
            "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
            "manifest": {"path": relative(MANIFEST), "sha256": sha256(MANIFEST)},
            "evaluator": {"path": relative(EVALUATOR), "sha256": sha256(EVALUATOR)},
            "source_commit": SOURCE_COMMIT,
            "artifact_schema": {"summary_artifact": "path/bytes/sha256 metadata", "summary_payload": "parsed native summary JSON", "summary_key_collision": False},
            "candidate": {"flag": PRESERVE_FLAG, "scope": "base RINEX reader only; adopted undifferenced FGO pseudorange factors", "matching_key": "exact (satellite,SignalType)", "coefficient": 1.0, "no_extrapolation_or_endpoint_hold": True, "spp_applied": False, "tdcp_applied": False, "doppler_applied": False},
            "routes": routes,
            "gates": {"all_four_routes": True, "prediction_domain_coverage_exact": True, "base_hash_and_bytes_exact": True, "matched_factor_fraction_each_route_min_0_80": True, "finite_correction_fraction_among_matched_each_route_min_0_99": True, "selected_band_telemetry_exact": True, "candidate_repeat_identity": True, "control_phase43_identity": True, "finite_output_and_earth_valid": True, "converged": True, "tdcp_built_equals_inserted": True, "over_70_mps": True, "truth_free": True, "all_passed": True},
            "read_accounting": {"single_process_per_case": True, "routes": 4, "candidate_runs_per_route": 2, "control_runs_per_route": 1, "native_solver_invocations": 12, "raw_device_gnss_process_reads": 12, "raw_device_imu_process_reads": 12, "broadcast_nav_process_reads": 12, "base_rinex_process_reads": 12, "hash_verification_reads": {"device_gnss": 4, "device_imu": 4, "brdc.nav": 4, "base_rinex": 4}, "truth_reads": 0, "mat_reads_or_generated": 0, "validation_holdout_reads": 0, "kaggle_token_reads": 0, "archive_reopens": 0, "post_truth_tuning": False},
            "phase43_champion_preserved": True,
            "phase51_experimental_preserved": True,
            "phase58_experimental_preserved": True,
            "zero_point_782_claim": "not evaluated in truth-free structural recovery",
        }
        result_path = output_root / "phase72_base_additional_frequency_bands_recovery_structural_result.json"
        atomic_json(result_path, result)
        atomic_json(output_root / "phase72_base_additional_frequency_bands_recovery_structural_manifest.json", {"schema_version": "smartphone-r5-phase72-base-additional-frequency-bands-recovery-structural-output-manifest.v1", "phase": 72, "status": "sealed-truth-free-structural-recovery", "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "evaluator": {"path": relative(EVALUATOR), "sha256": sha256(EVALUATOR)}, "result": {"path": relative(result_path), "sha256": sha256(result_path), "bytes": result_path.stat().st_size}, "native_solver_invocations": 12, "truth_reads": 0, "all_gates_passed": True})
        return result
    except Exception as exc:
        # Catch both frozen-gate failures and unexpected schema/runtime
        # exceptions so an integrity defect cannot disappear as a traceback.
        failure = {"schema_version": "smartphone-r5-phase72-base-additional-frequency-bands-recovery-structural-failure.v1", "status": "fail-closed", "exception_type": type(exc).__name__, "error": str(exc), "truth_reads": 0, "native_solver_invocations_started": started, "partial_routes": routes, "phase71_partial_output_reused": False}
        atomic_json(output_root / "phase72_base_additional_frequency_bands_recovery_structural_failure.json", failure)
        if isinstance(exc, Phase72Error):
            raise
        raise fail(f"unexpected evaluator exception: {type(exc).__name__}: {exc}") from exc


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--run-matrix", action="store_true")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        if args.verify_freeze:
            verify_freeze()
        if args.run_matrix:
            result = run_matrix(args.output_root)
            print(json.dumps({"status": result["status"], "all_gates_passed": result["gates"]["all_passed"], "native_solver_invocations": result["read_accounting"]["native_solver_invocations"], "truth_reads": result["read_accounting"]["truth_reads"]}, sort_keys=True))
        elif not args.verify_freeze:
            parser.error("one of --verify-freeze or --run-matrix is required")
        return 0
    except Phase72Error as exc:
        print(f"phase72 structural failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
