#!/usr/bin/env python3
"""Truth-free structural matrix for the Phase71 base-band candidate.

Phase71 changes only the RINEX reader instance used by the native base
pseudorange compensation model.  This evaluator imports the already sealed
Phase65 input/command helpers, adds the new opt-in flag, and validates the
newly frozen telemetry denominators.  It never opens development truth,
validation/holdout data, MATLAB products, or an archive.
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
PHASE65_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase65_native_base_pseudorange_compensation.py"
_SPEC = importlib.util.spec_from_file_location("phase65_helpers", PHASE65_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover - import failure
    raise RuntimeError(f"failed to load Phase65 helper: {PHASE65_PATH}")
P65 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(P65)

FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase71_base_additional_frequency_bands_freeze_v1.json"
FREEZE_SHA256 = "c5a6b4499b6178db439479a18cc0668e14a1868369f35679a0d557846bc0e49a"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase71_base_additional_frequency_bands_manifest_v1.json"
EVALUATOR = Path(__file__).resolve()
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
BINARY_SHA256 = "04543eda588739ac3d3c22296e2bef8ed712b177fa4e47474eebd6eb84b4377a"
SOURCE_COMMIT = "1ee721ebb6d8adfd5ab65fa8f2243ad2926ac982"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase71-base-additional-frequency-bands-structural-v1"
PRESERVE_FLAG = "--native-base-pseudorange-preserve-additional-frequency-bands"
OUTPUT_SCHEMA = "smartphone-r5-phase71-base-additional-frequency-bands-structural-result.v1"

ROUTES = P65.ROUTES
INPUTS = P65.INPUTS
BASE_INPUTS = P65.BASE_INPUTS
RAW_INPUT_HASHES = P65.RAW_INPUT_HASHES
BASE_INPUT_HASHES = P65.BASE_INPUT_HASHES
BASE_FLAGS = P65.BASE_FLAGS
EARTH_RADIUS_M = P65.EARTH_RADIUS_M
MAX_SPEED_MPS = P65.MAX_SPEED_MPS
Phase71Error = P65.Phase65Error


def fail(message: str) -> Phase71Error:
    return Phase71Error(message)


def reject_forbidden(path: Path | str) -> None:
    P65.reject_forbidden(path)


def sha256(path: Path) -> str:
    return P65.sha256(path)


def load_json(path: Path, label: str) -> dict[str, Any]:
    return P65.load_json(path, label)


def atomic_write(path: Path, payload: bytes) -> None:
    P65.atomic_write(path, payload)


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    P65.atomic_json(path, value)


def relative(path: Path) -> str:
    return P65.relative(path)


def verify_freeze() -> dict[str, Any]:
    if sha256(FREEZE) != FREEZE_SHA256:
        raise fail("Phase71 freeze hash changed")
    freeze = load_json(FREEZE, "Phase71 freeze")
    if freeze.get("status") != "frozen-before-phase71-native-structural-runs":
        raise fail("Phase71 freeze status changed")
    authority = freeze.get("authority", {})
    if authority.get("base_commit") != "97e5096":
        raise fail("Phase71 authority commit changed")
    if authority.get("phase70_partial_output_reuse") is not False:
        raise fail("Phase70 partial-output reuse contract changed")
    source = freeze.get("source_contract_before_change", {})
    expected_source = {
        "fgo_entrypoint_sha256": "bad544d64dee176e4e38ed13c595ea10fba515cb2dd7dbd87a3bb2d2cab11f31",
        "rinex_header_sha256": "571ac4e0e3e301e6a8b865ea89caaae3acd40dc8b25f4f1bdb3ede751e9e6e5d",
        "rinex_source_sha256": "92f683e34b4052f8d105f69fc4bf4397ca990be3b5a6b220181a56423f465533",
        "base_model_source_sha256": "55b7a1fbb96b5e3d6599c311c229870d7c319163d43322c01a3881d85dcb25aa",
    }
    for key, expected in expected_source.items():
        if source.get(key) != expected:
            raise fail(f"source pin changed: {key}")
    candidate = freeze.get("candidate", {})
    if candidate.get("flag") != "--native-base-pseudorange-preserve-additional-frequency-bands":
        raise fail("candidate flag changed")
    if candidate.get("matching_key") != "exact (satellite,SignalType) as before":
        raise fail("matching contract changed")
    if candidate.get("no_canonicalization") is not True or candidate.get("no_interpolation_change") is not True:
        raise fail("exact matching/interpolation contract changed")
    if candidate.get("no_extrapolation_or_endpoint_hold") is not True or candidate.get("no_new_rows_or_factors") is not True:
        raise fail("fill/factor contract changed")
    telemetry = freeze.get("telemetry_contract", {})
    required = {
        "preserve_additional_frequency_bands",
        "selected_band_observation_rows",
        "selected_band_streams",
        "selected_band_observation_rows_by_signal",
        "selected_band_streams_by_signal",
        "matched_factor_rows",
        "finite_correction_rows_among_matched",
        "matched_factor_fraction",
        "finite_correction_fraction_among_matched",
    }
    if not required.issubset(set(telemetry.get("required_fields", []))):
        raise fail("telemetry contract lost a required field")
    gates = freeze.get("gates", {})
    if gates.get("prediction_domain_coverage_exact") != 1.0:
        raise fail("prediction-domain gate changed")
    if gates.get("candidate_matched_factor_fraction_each_route_min") != 0.8:
        raise fail("matched-factor gate changed")
    if gates.get("candidate_finite_correction_fraction_among_matched_each_route_min") != 0.99:
        raise fail("finite-among-matched gate changed")
    matrix = freeze.get("structural_matrix", {})
    if matrix.get("routes") != 4 or matrix.get("control_runs_per_route") != 1 or matrix.get("candidate_runs_per_route") != 2 or matrix.get("native_solver_invocations") != 12:
        raise fail("structural run count changed")
    if matrix.get("truth_reads") != 0 or matrix.get("mat_reads_or_generated") != 0 or matrix.get("validation_holdout_reads") != 0 or matrix.get("archive_reopens") != 0:
        raise fail("forbidden read accounting changed")
    if sha256(PHASE65_PATH) != "6be7dd346a1ff219c15036fcf194861ebeda700678d52e3d93b2f3c2b96a16ae":
        raise fail("Phase65 helper hash changed")
    if sha256(BINARY) != BINARY_SHA256:
        raise fail("built native binary hash changed")
    if not MANIFEST.is_file():
        raise fail("Phase71 structural manifest missing")
    manifest = load_json(MANIFEST, "Phase71 structural manifest")
    if manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256:
        raise fail("manifest freeze pin changed")
    if manifest.get("binary", {}).get("sha256") != BINARY_SHA256:
        raise fail("manifest binary pin changed")
    if manifest.get("source_commit") != SOURCE_COMMIT:
        raise fail("manifest source commit changed")
    source_files = manifest.get("source_files", {})
    if not isinstance(source_files, dict) or not source_files:
        raise fail("manifest source-file pins missing")
    for source_path, expected_digest in source_files.items():
        path = ROOT / source_path
        if sha256(path) != expected_digest:
            raise fail(f"manifest source-file hash changed: {source_path}")
    if manifest.get("routes") != list(ROUTES):
        raise fail("manifest route order changed")
    if manifest.get("read_accounting", {}).get("truth_reads") != 0:
        raise fail("manifest truth read contract changed")
    evaluator_pin = manifest.get("evaluator", {})
    if evaluator_pin.get("path") != relative(EVALUATOR) or evaluator_pin.get("sha256") != sha256(EVALUATOR):
        raise fail("manifest evaluator pin changed")
    return freeze


def input_paths(route: str) -> dict[str, Path]:
    return {name: ROOT / path for name, path in INPUTS[route].items()} | {"base.obs": ROOT / BASE_INPUTS[route]}


def verify_inputs(route: str) -> dict[str, Any]:
    # Phase65's helper hashes the four declared input members and rejects any
    # truth/MAT/validation path.  This is a preflight hash read, not a second
    # materialization or an archive access.
    return P65.verify_inputs(route)


def read_prediction(path: Path, dataset_id: str) -> list[tuple[int, float, float]]:
    return P65.read_prediction(path, dataset_id)


def speed_report(rows: list[tuple[int, float, float]]) -> dict[str, Any]:
    return P65.speed_report(rows)


def _finite_float(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def _validate_signal_map(value: Any, total: int, label: str, route: str) -> None:
    if not isinstance(value, dict) or any(not isinstance(key, str) for key in value):
        raise fail(f"{label} map missing: {route}")
    counts: list[int] = []
    for key, count in value.items():
        if not key or not isinstance(count, int) or count < 0:
            raise fail(f"{label} map invalid: {route}/{key}")
        counts.append(count)
    if sum(counts) != total:
        raise fail(f"{label} map does not sum to total: {route}")


def validate_summary(path: Path, route: str, candidate: bool) -> dict[str, Any]:
    summary = load_json(path, "native summary")
    common = (
        ("dataset_id", route),
        ("truth_used", False),
        ("production_default_changed", False),
        ("native_quality_anchor", True),
        ("native_pdc_imu_tdcp_no_bridge", True),
        ("native_pdc_state_bridge", False),
    )
    for key, expected in common:
        if summary.get(key) != expected:
            raise fail(f"summary contract mismatch {route}/{key}")
    graph, epochs, raw, tdcp = (summary.get(key) for key in ("graph", "epochs", "raw_utc_key_contract", "tdcp_contract"))
    if not all(isinstance(value, dict) for value in (graph, epochs, raw, tdcp)):
        raise fail(f"summary diagnostics missing: {route}")
    if graph.get("converged") is not True or int(epochs.get("problem", 0)) <= 1 or int(epochs.get("output", 0)) <= 1:
        raise fail(f"graph/epoch gate failed: {route}")
    if int(epochs.get("pseudorange_factors", 0)) <= 0 or int(tdcp.get("factors_built", 0)) <= 0 or int(tdcp.get("factors_inserted", -1)) != int(tdcp.get("factors_built", 0)) or int(tdcp.get("nonfinite_residuals", -1)) != 0:
        raise fail(f"factor gate failed: {route}")
    if int(raw.get("unresolved_epochs", -1)) != 0 or int(raw.get("target_epochs", 0)) != int(raw.get("exact_solution_epochs", -1)):
        raise fail(f"raw UTC alignment gate failed: {route}")
    if not candidate:
        if "native_base_pseudorange_compensation" in summary:
            raise fail(f"flag-off base telemetry leaked: {route}")
        return {"summary": summary}

    telemetry = summary.get("native_base_pseudorange_compensation")
    if not isinstance(telemetry, dict) or telemetry.get("enabled") is not True or telemetry.get("built") is not True or telemetry.get("applied") is not True:
        raise fail(f"base telemetry missing: {route}")
    if telemetry.get("preserve_additional_frequency_bands") is not True:
        raise fail(f"additional-band option telemetry missing: {route}")
    pin = BASE_INPUT_HASHES[route]
    if telemetry.get("base_member_sha256") != pin["sha256"] or telemetry.get("base_rinex_sha256") != pin["sha256"] or int(telemetry.get("base_rinex_bytes", -1)) != pin["bytes"]:
        raise fail(f"base hash/bytes telemetry mismatch: {route}")
    xyz = telemetry.get("base_coordinate_xyz_m")
    if not isinstance(xyz, list) or len(xyz) != 3 or any(abs(_finite_float(a) - _finite_float(b)) > 1e-6 for a, b in zip(xyz, pin["xyz"])):
        raise fail(f"base coordinate telemetry mismatch: {route}")
    if abs(_finite_float(telemetry.get("observed_interval_s")) - pin["dt_s"]) > 1e-6 or int(telemetry.get("moving_mean_samples", -1)) != pin["window"]:
        raise fail(f"base interval/window telemetry mismatch: {route}")
    if telemetry.get("same_satellite_signal_only") is not True or telemetry.get("spp_applied") is not False or telemetry.get("tdcp_applied") is not False or telemetry.get("doppler_applied") is not False or telemetry.get("no_extrapolation_or_endpoint_hold") is not True:
        raise fail(f"base scope contract failed: {route}")

    selected_rows = telemetry.get("selected_band_observation_rows")
    selected_streams = telemetry.get("selected_band_streams")
    if not isinstance(selected_rows, int) or selected_rows <= 0 or not isinstance(selected_streams, int) or selected_streams <= 0 or selected_streams > selected_rows:
        raise fail(f"selected-band telemetry invalid: {route}")
    _validate_signal_map(telemetry.get("selected_band_observation_rows_by_signal"), selected_rows, "selected observation", route)
    _validate_signal_map(telemetry.get("selected_band_streams_by_signal"), selected_streams, "selected stream", route)

    adopted = telemetry.get("adopted_pseudorange_rows")
    matched = telemetry.get("matched_factor_rows")
    finite_matched = telemetry.get("finite_correction_rows_among_matched")
    corrected = telemetry.get("adopted_rows_corrected")
    if not all(isinstance(value, int) for value in (adopted, matched, finite_matched, corrected)):
        raise fail(f"base denominator telemetry is non-integral: {route}")
    if adopted <= 0 or matched < 0 or matched > adopted or finite_matched < 0 or finite_matched > matched or corrected < 0 or corrected > adopted:
        raise fail(f"base denominator telemetry out of range: {route}")
    matched_fraction = _finite_float(telemetry.get("matched_factor_fraction"))
    finite_fraction = _finite_float(telemetry.get("finite_correction_fraction_among_matched"))
    if not math.isfinite(matched_fraction) or abs(matched_fraction - matched / adopted) > 1e-12 or matched_fraction < 0.80:
        raise fail(f"matched/all coverage gate failed: {route}")
    if matched == 0 or not math.isfinite(finite_fraction) or abs(finite_fraction - finite_matched / matched) > 1e-12 or finite_fraction < 0.99:
        raise fail(f"finite/matched coverage gate failed: {route}")
    # The legacy all-adopted fraction is informational in Phase71; retaining
    # it in the summary is useful, but it is intentionally not the gate.
    all_fraction = _finite_float(telemetry.get("finite_correction_fraction"))
    if corrected > adopted or (math.isfinite(all_fraction) and abs(all_fraction - corrected / adopted) > 1e-12):
        raise fail(f"informational all-adopted fraction inconsistent: {route}")
    return {"summary": summary, "base": telemetry}


def artifact_report(submission: Path, summary: Path, route: str, candidate: bool) -> dict[str, Any]:
    rows = read_prediction(submission, route)
    speed = speed_report(rows)
    if not speed["finite"] or speed["over_70_mps_count"] != 0:
        raise fail(f"speed gate failed: {route}")
    diagnostics = validate_summary(summary, route, candidate)
    return {
        "submission": {"path": relative(submission), "bytes": submission.stat().st_size, "sha256": sha256(submission), "rows": len(rows)},
        "summary": {"path": relative(summary), "bytes": summary.stat().st_size, "sha256": sha256(summary)},
        "prediction_keys": [row[0] for row in rows],
        "speed": speed,
        **diagnostics,
    }


def native_command(route: str, run_dir: Path, candidate: bool) -> list[str]:
    command = P65.native_command(route, run_dir, candidate)
    if candidate:
        command.append(PRESERVE_FLAG)
    for token in command:
        reject_forbidden(token)
    return command


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
    base = P65.PHASE43_ROOT / route / "run1"
    submission, summary = base / "submission.csv", base / "summary.json"
    if not submission.is_file() or not summary.is_file():
        raise fail(f"missing Phase43 control artifact: {route}")
    expected = P65.PHASE43_CONTROL[route]
    actual = {"submission": sha256(submission), "summary": sha256(summary)}
    if actual != expected:
        raise fail(f"Phase43 control artifact hash changed: {route}")
    rows = read_prediction(submission, route)
    return {"path": relative(base), "submission": {"bytes": submission.stat().st_size, "sha256": actual["submission"]}, "summary": {"bytes": summary.stat().st_size, "sha256": actual["summary"]}, "prediction_keys": [row[0] for row in rows]}


def run_matrix(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    verify_freeze()
    output_root = output_root.resolve()
    reject_forbidden(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty output: {output_root}")
    input_reports = {route: verify_inputs(route) for route in ROUTES}
    output_root.mkdir(parents=True, exist_ok=True)
    routes: dict[str, Any] = {}
    started = 0
    try:
        for route in ROUTES:
            control = run_case(output_root, route, "control", 1, False)
            started += 1
            control_reference = phase43_control(route)
            if control["submission"]["sha256"] != control_reference["submission"]["sha256"] or control["summary"]["sha256"] != control_reference["summary"]["sha256"]:
                raise fail(f"Phase43 control identity failed: {route}")
            if control["prediction_keys"] != control_reference["prediction_keys"]:
                raise fail(f"control prediction-domain mismatch: {route}")
            candidate_one = run_case(output_root, route, "candidate", 1, True)
            started += 1
            candidate_two = run_case(output_root, route, "candidate", 2, True)
            started += 1
            if candidate_one["submission"]["sha256"] != candidate_two["submission"]["sha256"] or candidate_one["summary"]["sha256"] != candidate_two["summary"]["sha256"] or candidate_one["base"] != candidate_two["base"]:
                raise fail(f"candidate repeat identity failed: {route}")
            if candidate_one["prediction_keys"] != control_reference["prediction_keys"] or candidate_two["prediction_keys"] != control_reference["prediction_keys"]:
                raise fail(f"candidate prediction-domain coverage failed: {route}")
            base = candidate_one["base"]
            routes[route] = {
                "input": input_reports[route],
                "control": control,
                "phase43_control_reference": control_reference,
                "candidate_run1": candidate_one,
                "candidate_run2": candidate_two,
                "prediction_domain_coverage": 1.0,
                "gates": {
                    "prediction_domain_coverage_exact": True,
                    "base_hash_and_bytes_exact": True,
                    "matched_factor_fraction": base["matched_factor_fraction"] >= 0.80,
                    "finite_correction_fraction_among_matched": base["finite_correction_fraction_among_matched"] >= 0.99,
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
            "phase": 71,
            "execution_label": "Luna Max",
            "status": "go-phase71-base-additional-frequency-bands-structural",
            "truth_free": True,
            "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
            "manifest": {"path": relative(MANIFEST), "sha256": sha256(MANIFEST)},
            "evaluator": {"path": relative(EVALUATOR), "sha256": sha256(EVALUATOR)},
            "source_commit": SOURCE_COMMIT,
            "candidate": {
                "flag": PRESERVE_FLAG,
                "base_compensation_flag": P65.CANDIDATE_FLAG,
                "base_path_flag": P65.BASE_FLAG,
                "base_sha_flag": P65.BASE_SHA_FLAG,
                "scope": "base RINEX reader only; adopted undifferenced FGO pseudorange factors receive existing pc subtraction",
                "matching_key": "exact (satellite,SignalType)",
                "no_extrapolation_or_endpoint_hold": True,
                "spp_applied": False,
                "tdcp_applied": False,
                "doppler_applied": False,
                "coefficient": 1.0,
            },
            "routes": routes,
            "gates": {
                "all_four_routes": True,
                "prediction_domain_coverage_exact": True,
                "base_hash_and_bytes_exact": True,
                "matched_factor_fraction_each_route_min_0_80": True,
                "finite_correction_fraction_among_matched_each_route_min_0_99": True,
                "selected_band_telemetry_exact": True,
                "candidate_repeat_identity": True,
                "control_phase43_identity": True,
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
                "phase70_partial_output_reuse": False,
                "post_truth_tuning": False,
            },
            "phase43_champion_preserved": True,
            "phase51_experimental_preserved": True,
            "phase58_experimental_preserved": True,
            "zero_point_782_claim": "not evaluated in truth-free structural phase",
        }
        result_path = output_root / "phase71_base_additional_frequency_bands_structural_result.json"
        atomic_json(result_path, result)
        output_manifest = {
            "schema_version": "smartphone-r5-phase71-base-additional-frequency-bands-structural-output-manifest.v1",
            "phase": 71,
            "status": "sealed-truth-free-structural-matrix",
            "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
            "evaluator": {"path": relative(EVALUATOR), "sha256": sha256(EVALUATOR)},
            "result": {"path": relative(result_path), "sha256": sha256(result_path), "bytes": result_path.stat().st_size},
            "native_solver_invocations": 12,
            "truth_reads": 0,
            "all_gates_passed": True,
        }
        atomic_json(output_root / "phase71_base_additional_frequency_bands_structural_manifest.json", output_manifest)
        return result
    except Phase71Error as exc:
        atomic_json(output_root / "phase71_base_additional_frequency_bands_structural_failure.json", {
            "schema_version": "smartphone-r5-phase71-base-additional-frequency-bands-structural-failure.v1",
            "status": "fail-closed",
            "error": str(exc),
            "truth_reads": 0,
            "native_solver_invocations_started": started,
            "partial_routes": routes,
        })
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
    except Phase71Error as exc:
        print(f"phase71 structural failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
