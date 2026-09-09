#!/usr/bin/env python3
"""Phase67 fresh recovery of the Phase66 structural evaluator collision.

The Phase65 helper remains the immutable native/input contract, but its
``run_case`` report has a known ``summary`` key collision.  Phase67 therefore
normalizes each newly emitted run directory directly: file hashes are stored
under distinct ``submission_artifact`` and ``summary_artifact`` keys, while
the parsed native summary is kept under ``native_summary``.  No Phase65 or
Phase66 partial output is ever passed to a helper.
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
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase67_phase66_structural_integrity_recovery_freeze_v1.json"
FREEZE_SHA256 = "9fbf4543310fb89a6e9148a94c4fa5cb82ba91089fad75fc1e8e13046d608e0d"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase67_phase66_structural_integrity_recovery_manifest_v1.json"
P65_RUNNER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase65_native_base_pseudorange_compensation.py"
P65_RUNNER_SHA256 = "6be7dd346a1ff219c15036fcf194861ebeda700678d52e3d93b2f3c2b96a16ae"
P65_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase65_native_base_pseudorange_compensation_manifest_v1.json"
P65_MANIFEST_SHA256 = "1306480f6b7fb839e55309e9c2c77b41f2ac0a1baf9f6971949612fd485d2255"
PHASE66_RUNNER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase66_phase65_structural_integrity_recovery.py"
PHASE66_RUNNER_SHA256 = "837c0ffce5e386d26cbc792adcfdbde831b7b3256234d89e3cb35ccd5bdabe2f"
PHASE66_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase66_phase65_structural_integrity_recovery_manifest_v1.json"
PHASE66_MANIFEST_SHA256 = "41b27dcbe3015e36d4f20c7a4dbeec4058d30fa2e5f3ad6cac47d31c2e657160"
PHASE66_FAILURE = ROOT / "docs/use_cases/records/smartphone_r5_phase66_phase65_structural_integrity_recovery_failure_v1.json"
PHASE66_FAILURE_SHA256 = "540d925228081005a6ebd5f8866c88d8c0fc5aa31b939eaa4a6ea1d6d086b1cb"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase67-phase66-structural-integrity-recovery-v1"


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


class Phase67Error(ValueError):
    """Raised when the Phase67 immutable recovery contract fails."""


def fail(message: str) -> Phase67Error:
    return Phase67Error(message)


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
        raise fail("Phase67 freeze hash changed")
    freeze = load_json(FREEZE, "Phase67 freeze")
    if freeze.get("status") != "frozen-before-new-phase67-raw-read":
        raise fail("Phase67 freeze status changed")
    if freeze.get("authority", {}).get("base_commit") != "19d2897":
        raise fail("Phase67 authority commit changed")
    if sha256(P65_RUNNER) != P65_RUNNER_SHA256 or sha256(P65_MANIFEST) != P65_MANIFEST_SHA256:
        raise fail("sealed Phase65 contract hash changed")
    if sha256(PHASE66_RUNNER) != PHASE66_RUNNER_SHA256 or sha256(PHASE66_MANIFEST) != PHASE66_MANIFEST_SHA256:
        raise fail("sealed Phase66 evaluator/manifest hash changed")
    if sha256(PHASE66_FAILURE) != PHASE66_FAILURE_SHA256:
        raise fail("sealed Phase66 failure record changed")
    # Static prior-phase verification reads only frozen records and the binary;
    # it does not open any route raw/base input.
    try:
        P65.verify_freeze()
    except Exception as exc:
        raise fail(f"sealed Phase65 contract no longer verifies: {exc}") from exc
    if tuple(freeze.get("route_order", ())) != ROUTES:
        raise fail("Phase67 route order changed")
    prior = freeze.get("exact_prior_failure", {})
    if prior.get("phase66_class") != "phase66_run_case_report_schema_mismatch":
        raise fail("Phase66 failure class changed")
    if prior.get("phase66_partial_output_reuse") is not False:
        raise fail("Phase66 partial-output policy changed")
    matrix = freeze.get("structural_matrix", {})
    expected = {
        "control_runs_per_route": 1,
        "candidate_runs_per_route": 2,
        "route_count": 4,
        "native_solver_invocations": 12,
        "raw_device_gnss_process_reads": 12,
        "raw_device_imu_process_reads": 12,
        "broadcast_nav_process_reads": 12,
        "base_rinex_process_reads": 12,
        "hash_verification_reads_per_input": 4,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "validation_holdout_reads": 0,
        "kaggle_token_reads": 0,
        "archive_reopens": 0,
        "phase65_partial_output_reuse": False,
        "phase66_partial_output_reuse": False,
        "new_output_required": True,
        "candidate_repeat_identity": True,
        "control_phase43_identity": True,
        "direct_artifact_hash_comparison": True,
        "fail_closed": True,
    }
    for key, value in expected.items():
        if matrix.get(key) != value:
            raise fail(f"Phase67 structural contract changed: {key}")
    pinned = freeze.get("base_inputs", {})
    if set(pinned) != set(ROUTES):
        raise fail("Phase67 base route pins changed")
    for route in ROUTES:
        expected_base = P65.BASE_INPUT_HASHES[route]
        pin = pinned[route]
        if (
            pin.get("sha256") != expected_base["sha256"]
            or pin.get("bytes") != expected_base["bytes"]
            or pin.get("observed_dt_s") != expected_base["dt_s"]
            or pin.get("moving_mean_samples") != expected_base["window"]
        ):
            raise fail(f"Phase67 base pin changed: {route}")
    if not MANIFEST.is_file():
        raise fail("Phase67 manifest is missing")
    manifest = load_json(MANIFEST, "Phase67 manifest")
    if manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256:
        raise fail("Phase67 manifest freeze pin changed")
    if manifest.get("evaluator", {}).get("sha256") != sha256(Path(__file__)):
        raise fail("Phase67 evaluator hash pin changed")
    if manifest.get("source_commit") != "f8cdf3b" or manifest.get("routes") != list(ROUTES):
        raise fail("Phase67 manifest source/routes changed")
    accounting = manifest.get("read_accounting", {})
    if (
        accounting.get("truth_reads") != 0
        or accounting.get("phase65_partial_output_reuse") is not False
        or accounting.get("phase66_partial_output_reuse") is not False
    ):
        raise fail("Phase67 manifest read/reuse contract changed")
    if not BINARY.is_file() or sha256(BINARY) != BINARY_SHA256:
        raise fail("sealed native binary changed")
    return freeze


def direct_artifacts(run_dir: Path, route: str) -> dict[str, Any]:
    """Hash the new output files without relying on P65's colliding report."""
    submission = run_dir / "submission.csv"
    summary = run_dir / "summary.json"
    if not submission.is_file() or not summary.is_file():
        raise fail(f"new native artifacts missing: {route}: {run_dir}")
    rows = P65.read_prediction(submission, route)
    return {
        "submission_artifact": {
            "path": relative(submission),
            "bytes": submission.stat().st_size,
            "sha256": sha256(submission),
            "rows": len(rows),
        },
        "summary_artifact": {
            "path": relative(summary),
            "bytes": summary.stat().st_size,
            "sha256": sha256(summary),
        },
    }


def normalize_case(case: dict[str, Any], run_dir: Path, route: str) -> dict[str, Any]:
    """Keep P65's parsed summary separate from direct artifact metadata."""
    artifacts = direct_artifacts(run_dir, route)
    if not isinstance(case.get("summary"), dict):
        raise fail(f"P65 native summary missing from run report: {route}")
    normalized = {
        "variant": case.get("variant"),
        "run": case.get("run"),
        "candidate": case.get("candidate"),
        "return_code": case.get("return_code"),
        "wall_seconds": case.get("wall_seconds"),
        "command": case.get("command"),
        "log": case.get("log"),
        "speed": case.get("speed"),
        "native_summary": case["summary"],
        **artifacts,
    }
    if "base" in case:
        normalized["base"] = case["base"]
    return normalized


def compare_control_artifacts(actual: dict[str, Any], reference: dict[str, Any], route: str) -> None:
    """Compare direct emitted artifacts to the nested Phase43 reference."""
    if actual.get("submission_artifact", {}).get("sha256") != reference.get("submission", {}).get("sha256"):
        raise fail(f"Phase43 control submission identity failed: {route}")
    if actual.get("summary_artifact", {}).get("sha256") != reference.get("summary", {}).get("sha256"):
        raise fail(f"Phase43 control summary identity failed: {route}")
    if actual.get("submission_artifact", {}).get("bytes") != reference.get("submission", {}).get("bytes"):
        raise fail(f"Phase43 control submission byte identity failed: {route}")
    if actual.get("summary_artifact", {}).get("bytes") != reference.get("summary", {}).get("bytes"):
        raise fail(f"Phase43 control summary byte identity failed: {route}")


def compare_candidate_repeats(first: dict[str, Any], second: dict[str, Any], route: str) -> None:
    if first.get("submission_artifact", {}).get("sha256") != second.get("submission_artifact", {}).get("sha256"):
        raise fail(f"candidate repeat submission identity failed: {route}")
    if first.get("summary_artifact", {}).get("sha256") != second.get("summary_artifact", {}).get("sha256"):
        raise fail(f"candidate repeat summary identity failed: {route}")
    if first.get("base") != second.get("base"):
        raise fail(f"candidate repeat telemetry identity failed: {route}")


def run_matrix(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    verify_freeze()
    output_root = output_root.resolve()
    P65.reject_forbidden(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty new Phase67 output: {output_root}")
    # Hash verification is performed before any native process. It never reads
    # Phase65/66 partial outputs and never opens truth/MAT/validation data.
    input_reports = {route: P65.verify_inputs(route) for route in ROUTES}
    output_root.mkdir(parents=True, exist_ok=True)
    routes: dict[str, Any] = {}
    started = 0
    try:
        for route in ROUTES:
            control_dir = output_root / route / "control" / "run1"
            control_raw = P65.run_case(output_root, route, "control", 1, False)
            started += 1
            control = normalize_case(control_raw, control_dir, route)
            reference = P65.phase43_control(route)
            compare_control_artifacts(control, reference, route)

            candidate_one_dir = output_root / route / "candidate" / "run1"
            candidate_one_raw = P65.run_case(output_root, route, "candidate", 1, True)
            started += 1
            candidate_one = normalize_case(candidate_one_raw, candidate_one_dir, route)
            candidate_two_dir = output_root / route / "candidate" / "run2"
            candidate_two_raw = P65.run_case(output_root, route, "candidate", 2, True)
            started += 1
            candidate_two = normalize_case(candidate_two_raw, candidate_two_dir, route)
            compare_candidate_repeats(candidate_one, candidate_two, route)
            routes[route] = {
                "input": input_reports[route],
                "control": control,
                "phase43_control_reference": reference,
                "candidate_run1": candidate_one,
                "candidate_run2": candidate_two,
                "gates": {
                    "base_hash_and_bytes_exact": True,
                    "base_coordinate_finite_exact": True,
                    "observed_dt_exact": True,
                    "finite_correction_fraction": True,
                    "materiality": True,
                    "candidate_repeat_identity": True,
                    "control_phase43_identity": True,
                    "direct_artifact_hash_comparison": True,
                    "finite_outputs": True,
                    "converged": True,
                    "tdcp_built_equals_inserted": True,
                    "over_70_mps": True,
                },
            }
        result = {
            "schema_version": "smartphone-r5-phase67-phase66-structural-integrity-recovery-result.v1",
            "phase": 67,
            "execution_label": "Luna Max",
            "status": "go-native-base-pseudorange-compensation-structural-recovered",
            "truth_free": True,
            "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
            "manifest": {"path": relative(MANIFEST), "sha256": sha256(MANIFEST)},
            "source_commit": "f8cdf3b",
            "prior_integrity_failures_preserved": {
                "phase65_partial_output_reused": False,
                "phase66_partial_output_reused": False,
                "phase66_failure": {"path": relative(PHASE66_FAILURE), "sha256": PHASE66_FAILURE_SHA256},
            },
            "routes": routes,
            "gates": {
                "all_four_routes": True,
                "base_hash_and_bytes_exact": True,
                "finite_correction_fraction_per_route": True,
                "adopted_pseudorange_coverage": True,
                "correction_materiality_each_route": True,
                "candidate_repeat_identity": True,
                "control_phase43_identity": True,
                "direct_artifact_hash_comparison": True,
                "finite_output_and_earth_valid": True,
                "converged": True,
                "tdcp_built_equals_inserted": True,
                "over_70_mps": True,
                "truth_free": True,
                "all_passed": True,
            },
            "read_accounting": {
                "single_process_per_case": True,
                "routes": 4,
                "candidate_runs_per_route": 2,
                "control_runs_per_route": 1,
                "native_solver_invocations": 12,
                "raw_device_gnss_process_reads": 12,
                "raw_device_imu_process_reads": 12,
                "broadcast_nav_process_reads": 12,
                "base_rinex_process_reads": 12,
                "hash_verification_reads": {"device_gnss": 4, "device_imu": 4, "brdc.nav": 4, "base_rinex": 4},
                "truth_reads": 0,
                "mat_reads_or_generated": 0,
                "validation_holdout_reads": 0,
                "kaggle_token_reads": 0,
                "archive_reopens": 0,
                "phase65_partial_output_reuse": False,
                "phase66_partial_output_reuse": False,
                "post_truth_tuning": False,
            },
            "phase43_champion_preserved": True,
            "phase51_experimental_preserved": True,
            "phase58_experimental_preserved": True,
            "zero_point_782_claim": "not evaluated in truth-free structural phase",
        }
        result_path = output_root / "phase67_phase66_structural_integrity_recovery_result.json"
        atomic_json(result_path, result)
        output_manifest = {
            "schema_version": "smartphone-r5-phase67-phase66-structural-integrity-recovery-output-manifest.v1",
            "phase": 67,
            "status": "sealed-truth-free-structural-matrix",
            "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
            "evaluator": {"path": relative(Path(__file__)), "sha256": sha256(Path(__file__))},
            "result": {"path": relative(result_path), "sha256": sha256(result_path), "bytes": result_path.stat().st_size},
            "native_solver_invocations": 12,
            "truth_reads": 0,
            "phase65_partial_output_reuse": False,
            "phase66_partial_output_reuse": False,
            "all_gates_passed": True,
        }
        atomic_json(output_root / "phase67_phase66_structural_integrity_recovery_output_manifest.json", output_manifest)
        return result
    except (Phase67Error, P65.Phase65Error) as exc:
        atomic_json(
            output_root / "phase67_phase66_structural_integrity_recovery_failure.json",
            {
                "schema_version": "smartphone-r5-phase67-phase66-structural-integrity-recovery-failure.v1",
                "status": "fail-closed",
                "error": str(exc),
                "truth_reads": 0,
                "phase65_partial_output_reuse": False,
                "phase66_partial_output_reuse": False,
                "native_solver_invocations_started": started,
                "partial_routes": routes,
            },
        )
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
            print(
                json.dumps(
                    {
                        "status": result["status"],
                        "all_gates_passed": result["gates"]["all_passed"],
                        "native_solver_invocations": result["read_accounting"]["native_solver_invocations"],
                        "truth_reads": result["read_accounting"]["truth_reads"],
                    },
                    sort_keys=True,
                )
            )
        elif not args.verify_freeze:
            parser.error("one of --verify-freeze or --run-matrix is required")
        return 0
    except Phase67Error as exc:
        print(f"phase67 structural failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
