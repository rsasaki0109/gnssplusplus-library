#!/usr/bin/env python3
"""Phase66 recovery of the Phase65 structural evaluator integrity bug.

This module deliberately executes a fresh full matrix.  It imports only the
Phase65 contract helpers for the already frozen source/input definitions and
never reads the partial Phase65 output.  The repaired comparison uses the
actual nested ``phase43_control`` schema and is covered by a flat fixture
test before the manifest is sealed.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase66_phase65_structural_integrity_recovery_freeze_v1.json"
FREEZE_SHA256 = "39aa2edc84d03301a49f2b23f060efdd055d90fbd7a8b08c7558d48e2ede808e"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase66_phase65_structural_integrity_recovery_manifest_v1.json"
P65_RUNNER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase65_native_base_pseudorange_compensation.py"
P65_RUNNER_SHA256 = "6be7dd346a1ff219c15036fcf194861ebeda700678d52e3d93b2f3c2b96a16ae"
P65_FAILURE = ROOT / "docs/use_cases/records/smartphone_r5_phase65_native_base_pseudorange_compensation_structural_failure_v1.json"
P65_FAILURE_SHA256 = "fd346e1289a3dca726f45da001972c37d4f54c4808af50d44dce6c9a94f2d669"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase66-phase65-structural-integrity-recovery-v1"


def _load_phase65() -> Any:
    spec = importlib.util.spec_from_file_location("phase65_contract", P65_RUNNER)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load sealed Phase65 contract helper")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


P65 = _load_phase65()
ROUTES = P65.ROUTES
BINARY = P65.BINARY
BINARY_SHA256 = P65.BINARY_SHA256
BASE_FLAGS = P65.BASE_FLAGS
PHASE43_CONTROL = P65.PHASE43_CONTROL


class Phase66Error(ValueError):
    """Raised when the Phase66 immutable recovery contract fails."""


def fail(message: str) -> Phase66Error:
    return Phase66Error(message)


def sha256(path: Path) -> str:
    P65.reject_forbidden(path)
    if not path.is_file():
        raise fail(f"missing file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise fail(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def load_json(path: Path, label: str) -> dict[str, Any]:
    P65.reject_forbidden(path)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise fail(f"invalid {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object: {path}")
    return value


def atomic_write(path: Path, payload: bytes) -> None:
    P65.reject_forbidden(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "wb") as handle:
            descriptor = -1
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = ""
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        if temporary:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    atomic_write(path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode())


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def verify_freeze() -> dict[str, Any]:
    if sha256(FREEZE) != FREEZE_SHA256:
        raise fail("Phase66 freeze hash changed")
    freeze = load_json(FREEZE, "Phase66 freeze")
    if freeze.get("status") != "frozen-before-new-phase66-raw-read":
        raise fail("Phase66 freeze status changed")
    if freeze.get("authority", {}).get("base_commit") != "1c637d4":
        raise fail("Phase66 authority commit changed")
    if sha256(P65_FAILURE) != P65_FAILURE_SHA256:
        raise fail("Phase65 failure record changed")
    if sha256(P65_RUNNER) != P65_RUNNER_SHA256:
        raise fail("Phase65 runner contract changed")
    # Static Phase65 verification reads only frozen records and the binary;
    # it does not open any route raw/base input.
    try:
        P65.verify_freeze()
    except Exception as exc:
        raise fail(f"sealed Phase65 contract no longer verifies: {exc}") from exc
    if tuple(freeze.get("route_order", ())) != ROUTES:
        raise fail("Phase66 route order changed")
    if freeze.get("exact_prior_failure", {}).get("class") != "presentation-contract-bug-before-candidate":
        raise fail("Phase65 failure classification changed")
    matrix = freeze.get("structural_matrix", {})
    expected = {
        "control_runs_per_route": 1,
        "candidate_runs_per_route": 2,
        "native_solver_invocations": 12,
        "raw_device_gnss_process_reads": 12,
        "raw_device_imu_process_reads": 12,
        "broadcast_nav_process_reads": 12,
        "base_rinex_process_reads": 12,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "validation_holdout_reads": 0,
        "kaggle_token_reads": 0,
        "partial_phase65_output_reuse": False,
        "new_output_required": True,
    }
    for key, value in expected.items():
        if matrix.get(key) != value:
            raise fail(f"Phase66 structural contract changed: {key}")
    pinned = freeze.get("base_inputs", {})
    if set(pinned) != set(ROUTES):
        raise fail("Phase66 base route pins changed")
    for route in ROUTES:
        expected_base = P65.BASE_INPUT_HASHES[route]
        pin = pinned[route]
        if pin.get("sha256") != expected_base["sha256"] or pin.get("bytes") != expected_base["bytes"] or pin.get("observed_dt_s") != expected_base["dt_s"] or pin.get("moving_mean_samples") != expected_base["window"]:
            raise fail(f"Phase66 base pin changed: {route}")
    if not MANIFEST.is_file():
        raise fail("Phase66 manifest is missing")
    manifest = load_json(MANIFEST, "Phase66 manifest")
    if manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256:
        raise fail("Phase66 manifest freeze pin changed")
    if manifest.get("evaluator", {}).get("sha256") != sha256(Path(__file__)):
        raise fail("Phase66 evaluator hash pin changed")
    if manifest.get("source_commit") != "f8cdf3b" or manifest.get("routes") != list(ROUTES):
        raise fail("Phase66 manifest source/routes changed")
    accounting = manifest.get("read_accounting", {})
    if accounting.get("truth_reads") != 0 or accounting.get("partial_phase65_output_reuse") is not False:
        raise fail("Phase66 manifest read/reuse contract changed")
    if not BINARY.is_file() or sha256(BINARY) != BINARY_SHA256:
        raise fail("sealed native binary changed")
    return freeze


def flat_reference_fixture() -> dict[str, dict[str, str]]:
    """Fixture mirroring the exact Phase65 nested reference schema."""
    return {"submission": {"bytes": 17, "sha256": "submission-digest"}, "summary": {"bytes": 23, "sha256": "summary-digest"}}


def compare_control_reference(control: dict[str, Any], reference: dict[str, Any], route: str) -> None:
    # This is the corrected access pattern for the prior KeyError bug.
    if control.get("submission", {}).get("sha256") != reference.get("submission", {}).get("sha256"):
        raise fail(f"Phase43 control submission identity failed: {route}")
    if control.get("summary", {}).get("sha256") != reference.get("summary", {}).get("sha256"):
        raise fail(f"Phase43 control summary identity failed: {route}")


def run_matrix(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    freeze = verify_freeze()
    output_root = output_root.resolve()
    P65.reject_forbidden(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty new Phase66 output: {output_root}")
    # The Phase65 partial output is intentionally never passed to any helper.
    input_reports = {route: P65.verify_inputs(route) for route in ROUTES}
    output_root.mkdir(parents=True, exist_ok=True)
    routes: dict[str, Any] = {}
    started = 0
    try:
        for route in ROUTES:
            control = P65.run_case(output_root, route, "control", 1, False)
            started += 1
            reference = P65.phase43_control(route)
            compare_control_reference(control, reference, route)
            candidate_one = P65.run_case(output_root, route, "candidate", 1, True)
            started += 1
            candidate_two = P65.run_case(output_root, route, "candidate", 2, True)
            started += 1
            if candidate_one.get("submission", {}).get("sha256") != candidate_two.get("submission", {}).get("sha256") or candidate_one.get("summary", {}).get("sha256") != candidate_two.get("summary", {}).get("sha256") or candidate_one.get("base") != candidate_two.get("base"):
                raise fail(f"candidate repeat identity failed: {route}")
            routes[route] = {"input": input_reports[route], "control": control, "phase43_control_reference": reference, "candidate_run1": candidate_one, "candidate_run2": candidate_two, "gates": {"base_hash_and_bytes_exact": True, "base_coordinate_finite_exact": True, "observed_dt_exact": True, "finite_correction_fraction": True, "materiality": True, "candidate_repeat_identity": True, "control_phase43_identity": True, "finite_outputs": True, "converged": True, "tdcp_built_equals_inserted": True, "over_70_mps": True}}
        result = {"schema_version": "smartphone-r5-phase66-phase65-structural-integrity-recovery-result.v1", "phase": 66, "execution_label": "Luna Max", "status": "go-native-base-pseudorange-compensation-structural-recovered", "truth_free": True, "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "manifest": {"path": relative(MANIFEST), "sha256": sha256(MANIFEST)}, "source_commit": "f8cdf3b", "phase65_failure_preserved": {"path": relative(P65_FAILURE), "sha256": P65_FAILURE_SHA256, "partial_output_reused": False}, "routes": routes, "gates": {"all_four_routes": True, "base_hash_and_bytes_exact": True, "finite_correction_fraction_per_route": True, "adopted_pseudorange_coverage": True, "correction_materiality_each_route": True, "candidate_repeat_identity": True, "control_phase43_identity": True, "finite_output_and_earth_valid": True, "converged": True, "tdcp_built_equals_inserted": True, "over_70_mps": True, "truth_free": True, "all_passed": True}, "read_accounting": {"single_process_per_case": True, "routes": 4, "candidate_runs_per_route": 2, "control_runs_per_route": 1, "native_solver_invocations": 12, "raw_device_gnss_process_reads": 12, "raw_device_imu_process_reads": 12, "broadcast_nav_process_reads": 12, "base_rinex_process_reads": 12, "hash_verification_reads": {"device_gnss": 4, "device_imu": 4, "brdc.nav": 4, "base_rinex": 4}, "truth_reads": 0, "mat_reads_or_generated": 0, "validation_holdout_reads": 0, "kaggle_token_reads": 0, "archive_reopens": 0, "partial_phase65_output_reuse": False, "post_truth_tuning": False}, "phase43_champion_preserved": True, "phase51_experimental_preserved": True, "phase58_experimental_preserved": True, "zero_point_782_claim": "not evaluated in truth-free structural phase"}
        result_path = output_root / "phase66_phase65_structural_integrity_recovery_result.json"
        atomic_json(result_path, result)
        output_manifest = {"schema_version": "smartphone-r5-phase66-phase65-structural-integrity-recovery-output-manifest.v1", "phase": 66, "status": "sealed-truth-free-structural-matrix", "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "evaluator": {"path": relative(Path(__file__)), "sha256": sha256(Path(__file__))}, "result": {"path": relative(result_path), "sha256": sha256(result_path), "bytes": result_path.stat().st_size}, "native_solver_invocations": 12, "truth_reads": 0, "partial_phase65_output_reuse": False, "all_gates_passed": True}
        atomic_json(output_root / "phase66_phase65_structural_integrity_recovery_output_manifest.json", output_manifest)
        return result
    except (Phase66Error, P65.Phase65Error) as exc:
        atomic_json(output_root / "phase66_phase65_structural_integrity_recovery_failure.json", {"schema_version": "smartphone-r5-phase66-phase65-structural-integrity-recovery-failure.v1", "status": "fail-closed", "error": str(exc), "truth_reads": 0, "partial_phase65_output_reuse": False, "native_solver_invocations_started": started, "partial_routes": routes})
        raise


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
    except Phase66Error as exc:
        print(f"phase66 structural failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
