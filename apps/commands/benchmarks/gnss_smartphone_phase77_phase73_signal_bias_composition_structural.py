#!/usr/bin/env python3
"""Truth-free Phase77 structural matrix.

This runner evaluates the already implemented native secondary-signal bias
state path composed with the sealed Phase73 finite-base-pc miss mask.  The
control is a fresh Phase73 miss-mask invocation (no signal-bias flag); the
candidate is the same command with ``--native-signal-bias-states`` and is
repeated twice per route.  Phase73 miss-mask telemetry is compared exactly to
the sealed values, rather than applying a new retention threshold.  Summary
artifact metadata and parsed summary payload stay under disjoint keys.

The matrix is deliberately truth-free: no development truth, MAT, archive,
validation/holdout, WLS/SvPosition/SvElevation, or post-truth solver process is
opened.  A structural failure is written atomically with the partial accounting
and is never converted into an accuracy result.
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
PHASE73_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase73_source_exact_pseudorange_miss_mask_structural.py"
_SPEC = importlib.util.spec_from_file_location("phase73_structural_helpers", PHASE73_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load sealed Phase73 helper: {PHASE73_PATH}")
P73 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(P73)


FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase77_phase73_signal_bias_composition_structural_freeze_v1.json"
FREEZE_SHA256 = "4f7bd6136180bb86a81031eaff38689db1b44451fb418d21cf6d796d27fb4e95"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase77_phase73_signal_bias_composition_structural_manifest_v1.json"
EVALUATOR = Path(__file__).resolve()
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
BINARY_SHA256 = "941eec7044775dbc0b0c1f512a3afbc6821f1997ef299a56f3018ce0747c279d"
SOURCE_COMMIT = "8021843b0af5dbfd777976440afa68ab5dfaed9d"
PHASE73_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase73_source_exact_pseudorange_miss_mask_structural_result_v1.json"
PHASE73_RESULT_SHA256 = "8493d16b3d2c5ce8b09b65ab0dada6903fcbac01257fb8db7977cb41c85577fd"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase77-phase73-signal-bias-composition-structural-v1"
OUTPUT_SCHEMA = "smartphone-r5-phase77-phase73-signal-bias-composition-structural-result.v1"
SIGNAL_BIAS_FLAG = "--native-signal-bias-states"
BASE_COMP_FLAG = "--native-base-pseudorange-compensation"
BASE_RINEX_FLAG = "--native-base-rinex"
BASE_SHA_FLAG = "--native-base-rinex-sha256"
MISS_MASK_FLAG = "--native-base-pseudorange-source-miss-mask"

ROUTES = P73.ROUTES
INPUTS = P73.INPUTS
BASE_INPUTS = P73.BASE_INPUTS
RAW_INPUT_HASHES = P73.RAW_INPUT_HASHES
BASE_INPUT_HASHES = P73.BASE_INPUT_HASHES
PHASE43_CONTROL = P73.PHASE43_CONTROL
BASE_FLAGS = P73.BASE_FLAGS
PHASE43_ROOT = P73.PHASE43_ROOT


class Phase77StructuralError(ValueError):
    """Raised when the immutable Phase77 structural contract fails."""


def fail(message: str) -> Phase77StructuralError:
    return Phase77StructuralError(message)


def reject_forbidden(path: Path | str) -> None:
    P73.reject_forbidden(path)


def sha256(path: Path) -> str:
    return P73.sha256(path)


def load_json(path: Path, label: str) -> dict[str, Any]:
    return P73.load_json(path, label)


def atomic_write(path: Path, payload: bytes) -> None:
    P73.atomic_write(path, payload)


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    P73.atomic_json(path, value)


def relative(path: Path) -> str:
    return P73.relative(path)


def _source_hashes() -> dict[str, str]:
    freeze = load_json(FREEZE, "Phase77 freeze")
    source_hashes = freeze.get("source_hashes_at_freeze")
    if not isinstance(source_hashes, dict) or not source_hashes:
        raise fail("Phase77 source hash map is missing")
    return {str(path): str(value) for path, value in source_hashes.items()}


def _expected_miss_telemetry(route: str) -> dict[str, Any]:
    freeze = load_json(FREEZE, "Phase77 freeze")
    gates = freeze.get("truth_free_structural_gates", {})
    expected = gates.get("phase73_miss_mask_expected_by_route", {})
    values = expected.get(route)
    if not isinstance(values, dict):
        raise fail(f"Phase73 expected miss-mask telemetry missing: {route}")
    return values


def verify_freeze() -> dict[str, Any]:
    if sha256(FREEZE) != FREEZE_SHA256:
        raise fail("Phase77 freeze hash changed")
    freeze = load_json(FREEZE, "Phase77 freeze")
    if freeze.get("schema_version") != "smartphone-r5-phase77-phase73-signal-bias-composition-structural-freeze.v1":
        raise fail("Phase77 freeze schema changed")
    if freeze.get("status") != "frozen-before-phase77-raw-read":
        raise fail("Phase77 freeze status changed")
    if freeze.get("cohort", {}).get("raw_truth_read_before_freeze") != 0:
        raise fail("Phase77 pre-freeze read accounting changed")
    authority = freeze.get("authority", {})
    if authority.get("source_commit_at_freeze") != SOURCE_COMMIT:
        raise fail("Phase77 source commit pin changed")
    phase73_result_pin = authority.get("phase73_structural_result_record", {})
    if phase73_result_pin.get("sha256") != PHASE73_RESULT_SHA256 or sha256(PHASE73_RESULT) != PHASE73_RESULT_SHA256:
        raise fail("sealed Phase73 result pin changed")

    base_recipe = freeze.get("base_recipe", {})
    if base_recipe.get("candidate_addition") != SIGNAL_BIAS_FLAG:
        raise fail("Phase77 signal-bias flag changed")
    if base_recipe.get("candidate_preserve_additional_frequency_bands") is not False:
        raise fail("Phase77 additional-frequency-band option changed")
    if base_recipe.get("candidate_no_additional_band_reader") is not True:
        raise fail("Phase77 base-reader scope changed")
    if base_recipe.get("candidate_default_off") is not True:
        raise fail("Phase77 default-off contract changed")
    candidate = freeze.get("candidate_contract", {})
    if candidate.get("coefficient") != 1.0 or candidate.get("pseudorange_population") != "Phase73 finite-pc miss mask remains authoritative; no fill/extrapolation/endpoint hold":
        raise fail("Phase77 candidate formula/population contract changed")
    if candidate.get("signal_bias_eligibility") != "signal_policy::isSecondarySignal only; no canonical-frequency aliasing, no cross-constellation or cross-satellite state":
        raise fail("Phase77 signal-bias eligibility contract changed")
    if candidate.get("tdcp_doppler_imu_spp") != "no dedicated change; any secondary TDCP population change caused by the existing opt-in eligibility gate is reported and must be deterministic":
        raise fail("Phase77 non-pseudorange scope changed")

    source_hashes = _source_hashes()
    for path_text, expected in source_hashes.items():
        path = ROOT / path_text
        if sha256(path) != expected:
            raise fail(f"Phase77 source hash changed: {path_text}")
    if sha256(BINARY) != BINARY_SHA256:
        raise fail("Phase77 native binary hash changed")

    routes = freeze.get("cohort", {}).get("route_order")
    if tuple(routes or ()) != ROUTES:
        raise fail("Phase77 route order changed")
    expected_routes = freeze.get("truth_free_structural_gates", {}).get("phase73_miss_mask_expected_by_route", {})
    if set(expected_routes) != set(ROUTES):
        raise fail("Phase73 miss-mask expected route set changed")
    gates = freeze.get("truth_free_structural_gates", {})
    required_true = (
        "raw_input_hash_and_bytes_exact",
        "base_input_hash_and_bytes_exact",
        "candidate_all_coordinates_finite_and_earth_valid",
        "candidate_converged",
        "candidate_repeat_submission_and_summary_byte_identical",
        "signal_bias_estimates_all_finite",
        "phase73_miss_mask_telemetry_identity",
        "base_no_extrapolation_or_fill",
        "factor_count_consistency",
        "tdcp_doppler_imu_spp_invariants_reported",
        "truth_free",
        "all_gates_anded",
        "accuracy_truth_not_in_this_stage",
    )
    for key in required_true:
        if gates.get(key) is not True:
            raise fail(f"Phase77 gate declaration changed: {key}")
    if gates.get("control_phase73_run_count") != 1 or gates.get("candidate_signal_bias_states_min_per_route") != 1 or gates.get("candidate_signal_bias_factors_min_per_route") != 1:
        raise fail("Phase77 run/materiality gate declaration changed")
    if gates.get("candidate_prediction_domain_coverage") != 1.0 or gates.get("over_70_mps_count") != 0:
        raise fail("Phase77 domain/speed gate declaration changed")
    matrix = freeze.get("matrix", {})
    expected_matrix = {
        "candidate_runs_per_route": 2,
        "control_runs_per_route": 1,
        "native_invocations": 12,
        "raw_device_gnss_reads": 12,
        "raw_device_imu_reads": 12,
        "broadcast_nav_reads": 12,
        "base_rinex_reads": 12,
        "truth_reads": 0,
        "solver_rerun_after_truth": False,
        "candidate_artifacts_not_reused_from_phase73": True,
        "new_output_root_required": True,
    }
    if any(matrix.get(key) != value for key, value in expected_matrix.items()):
        raise fail("Phase77 matrix/read contract changed")
    if relative(DEFAULT_OUTPUT) != matrix.get("output_root"):
        raise fail("Phase77 output root contract changed")
    if gates.get("base_matched_adopted_factor_fraction") != "informational only; do not re-apply the historical 0.80 gate because the exact Phase73 population is frozen by identity":
        raise fail("Phase77 matched-factor gate was reintroduced")
    if gates.get("base_finite_correction_fraction") != "informational only; do not re-apply the historical 0.99 gate because the exact Phase73 population is frozen by identity":
        raise fail("Phase77 finite-correction threshold was reintroduced")

    if not MANIFEST.is_file():
        raise fail("Phase77 structural manifest is missing")
    manifest = load_json(MANIFEST, "Phase77 structural manifest")
    if manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256:
        raise fail("Phase77 manifest freeze pin changed")
    if manifest.get("binary", {}).get("sha256") != BINARY_SHA256:
        raise fail("Phase77 manifest binary pin changed")
    if manifest.get("source_commit") != SOURCE_COMMIT:
        raise fail("Phase77 manifest source commit changed")
    if manifest.get("routes") != list(ROUTES):
        raise fail("Phase77 manifest route order changed")
    manifest_matrix = manifest.get("matrix", {})
    if manifest_matrix.get("truth_reads") != 0 or manifest_matrix.get("native_invocations") != 12:
        raise fail("Phase77 manifest truth/run contract changed")
    evaluator_pin = manifest.get("evaluator", {})
    if evaluator_pin.get("path") != relative(EVALUATOR) or evaluator_pin.get("sha256") != sha256(EVALUATOR):
        raise fail("Phase77 evaluator manifest pin changed")
    return freeze


def input_paths(route: str) -> dict[str, Path]:
    if route not in INPUTS or route not in BASE_INPUTS:
        raise fail(f"unknown route: {route}")
    return {name: ROOT / path for name, path in INPUTS[route].items()} | {"base.obs": ROOT / BASE_INPUTS[route]}


def verify_inputs(route: str) -> dict[str, Any]:
    reports: dict[str, Any] = {}
    for name, path in input_paths(route).items():
        digest = sha256(path)
        expected = BASE_INPUT_HASHES[route]["sha256"] if name == "base.obs" else RAW_INPUT_HASHES[route][name]
        if digest != expected:
            raise fail(f"input hash mismatch: {route}/{name}")
        expected_bytes = BASE_INPUT_HASHES[route]["bytes"] if name == "base.obs" else path.stat().st_size
        if path.stat().st_size != expected_bytes:
            raise fail(f"input byte-size mismatch: {route}/{name}")
        reports[name] = {"path": relative(path), "bytes": path.stat().st_size, "sha256": digest}
    return reports


def read_prediction(path: Path, dataset_id: str) -> list[tuple[int, float, float]]:
    return P73.read_prediction(path, dataset_id)


def speed_report(rows: list[tuple[int, float, float]]) -> dict[str, Any]:
    return P73.speed_report(rows)


def _validate_base(summary: dict[str, Any], route: str) -> tuple[dict[str, Any], dict[str, Any]]:
    P73._validate_common_summary(summary, route)
    base, miss = P73._validate_base(summary, route)
    return base, miss


def _validate_signal_bias(summary: dict[str, Any], route: str) -> dict[str, Any]:
    if summary.get("native_signal_bias_states") is not True:
        raise fail(f"signal-bias option telemetry is not enabled: {route}")
    epochs = summary.get("epochs")
    if not isinstance(epochs, dict):
        raise fail(f"signal-bias epoch telemetry missing: {route}")
    try:
        factors = int(epochs["receiver_signal_bias_factors"])
        states = int(epochs["receiver_signal_bias_states"])
    except (KeyError, TypeError, ValueError) as exc:
        raise fail(f"signal-bias factor/state telemetry missing: {route}") from exc
    if factors < 1 or states < 1 or factors < states:
        raise fail(f"signal-bias factor/state materiality failed: {route}")
    estimates = summary.get("receiver_signal_bias_estimates_m")
    if not isinstance(estimates, dict) or len(estimates) != states:
        raise fail(f"signal-bias estimate map does not match state count: {route}")
    for key, value in estimates.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            raise fail(f"nonfinite signal-bias estimate: {route}/{key}")
    return {"factors": factors, "states": states, "estimates_m": estimates}


def _validate_control_no_bias(summary: dict[str, Any], route: str) -> None:
    if summary.get("native_signal_bias_states") is not False:
        raise fail(f"control signal-bias option is not disabled: {route}")
    epochs = summary.get("epochs")
    estimates = summary.get("receiver_signal_bias_estimates_m")
    if not isinstance(epochs, dict) or int(epochs.get("receiver_signal_bias_factors", -1)) != 0 or int(epochs.get("receiver_signal_bias_states", -1)) != 0:
        raise fail(f"control signal-bias telemetry is nonzero: {route}")
    if estimates not in ({}, None):
        raise fail(f"control signal-bias estimates leaked: {route}")


def _miss_identity(route: str, miss: dict[str, Any]) -> dict[str, Any]:
    expected = _expected_miss_telemetry(route)
    fields = tuple(expected)
    mismatches: dict[str, dict[str, Any]] = {}
    for key in fields:
        actual = miss.get(key)
        if actual != expected[key]:
            mismatches[key] = {"actual": actual, "expected": expected[key]}
    return {"passed": not mismatches, "mismatches": mismatches, "fields": list(fields)}


def _population_contract(summary: dict[str, Any]) -> dict[str, Any]:
    epochs = summary.get("epochs")
    tdcp = summary.get("tdcp_contract")
    graph = summary.get("graph")
    quality = summary.get("upstream_observable_quality")
    if not all(isinstance(value, dict) for value in (epochs, tdcp, graph, quality)):
        raise fail("population telemetry block missing")
    assert isinstance(epochs, dict) and isinstance(tdcp, dict) and isinstance(graph, dict) and isinstance(quality, dict)
    return {
        "pseudorange_factors": epochs.get("pseudorange_factors"),
        "tdcp_factors_built": tdcp.get("factors_built"),
        "tdcp_factors_inserted": tdcp.get("factors_inserted"),
        "undifferenced_doppler_factors_inserted": quality.get("doppler_graph_factors"),
        "imu_intervals": graph.get("imu_intervals"),
        "output_epochs": epochs.get("output"),
        "over_70_mps_count": 0,
    }


def validate_summary(path: Path, route: str, candidate: bool) -> dict[str, Any]:
    summary = load_json(path, "native summary")
    base, miss = _validate_base(summary, route)
    miss_identity = _miss_identity(route, miss)
    if candidate:
        signal = _validate_signal_bias(summary, route)
    else:
        _validate_control_no_bias(summary, route)
        signal = {"factors": 0, "states": 0, "estimates_m": {}}
    return {
        "summary": summary,
        "base": base,
        "source_miss_mask": miss,
        "phase73_miss_mask_identity": miss_identity,
        "signal_bias": signal,
    }


def artifact_report(submission: Path, summary: Path, route: str, candidate: bool) -> dict[str, Any]:
    rows = read_prediction(submission, route)
    speed = speed_report(rows)
    if not speed["finite"] or speed["over_70_mps_count"] != 0:
        raise fail(f"speed gate failed: {route}")
    diagnostics = validate_summary(summary, route, candidate)
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
        "summary_payload": diagnostics["summary"],
        "prediction_keys": [row[0] for row in rows],
        "speed": speed,
        "base_telemetry": diagnostics["base"],
        "source_miss_mask_telemetry": diagnostics["source_miss_mask"],
        "phase73_miss_mask_identity": diagnostics["phase73_miss_mask_identity"],
        "signal_bias": diagnostics["signal_bias"],
        "population": _population_contract(diagnostics["summary"]),
    }


def native_command(route: str, run_dir: Path, candidate: bool) -> list[str]:
    paths = input_paths(route)
    command = [
        str(BINARY.relative_to(ROOT)),
        "--android-gnss", relative(paths["device_gnss.csv"]),
        "--android-imu", relative(paths["device_imu.csv"]),
        "--nav", relative(paths["brdc.nav"]),
        "--out", relative(run_dir / "submission.csv"),
        "--summary-json", relative(run_dir / "summary.json"),
        "--dataset-id", route,
        *BASE_FLAGS,
        BASE_COMP_FLAG,
        BASE_RINEX_FLAG,
        relative(paths["base.obs"]),
        BASE_SHA_FLAG,
        BASE_INPUT_HASHES[route]["sha256"],
        MISS_MASK_FLAG,
    ]
    if candidate:
        command.append(SIGNAL_BIAS_FLAG)
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
    report: dict[str, Any] = {
        "variant": variant,
        "run": run_number,
        "candidate": candidate,
        "return_code": process.returncode,
        "wall_seconds": time.perf_counter() - started,
        "command": command,
        "log": {"path": relative(run_dir / "run.log"), "sha256": sha256(run_dir / "run.log")},
    }
    if process.returncode != 0:
        raise fail(f"native process returned {process.returncode}: {route}/{variant}/run{run_number}")
    submission = run_dir / "submission.csv"
    summary = run_dir / "summary.json"
    if not submission.is_file() or not summary.is_file():
        raise fail(f"native artifacts missing: {route}/{variant}/run{run_number}")
    report.update(artifact_report(submission, summary, route, candidate))
    return report


def phase43_control(route: str) -> dict[str, Any]:
    base = PHASE43_ROOT / route / "run1"
    submission = base / "submission.csv"
    summary = base / "summary.json"
    if not submission.is_file() or not summary.is_file():
        raise fail(f"missing Phase43 control artifact: {route}")
    expected = PHASE43_CONTROL[route]
    actual = {"submission": sha256(submission), "summary": sha256(summary)}
    if actual != expected:
        raise fail(f"Phase43 control artifact hash changed: {route}")
    rows = read_prediction(submission, route)
    return {
        "path": relative(base),
        "submission_artifact": {"bytes": submission.stat().st_size, "sha256": actual["submission"], "rows": len(rows)},
        "summary_artifact": {"bytes": summary.stat().st_size, "sha256": actual["summary"]},
        "prediction_keys": [row[0] for row in rows],
    }


def _public_report(report: dict[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in report.items() if key != "prediction_keys"}


def _delta(candidate: dict[str, Any], control: dict[str, Any]) -> dict[str, Any]:
    values: dict[str, Any] = {}
    for key, value in candidate.items():
        other = control.get(key)
        if isinstance(value, (int, float)) and isinstance(other, (int, float)):
            values[key] = value - other
        else:
            values[key] = {"candidate": value, "control": other, "equal": value == other}
    return values


def _route_case_error(route: str, variant: str, run_number: int, exc: Exception) -> dict[str, Any]:
    return {
        "variant": variant,
        "run": run_number,
        "candidate": variant == "candidate",
        "error_type": type(exc).__name__,
        "error": str(exc),
    }


def run_matrix(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    verify_freeze()
    output_root = output_root.resolve()
    reject_forbidden(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty structural output: {output_root}")
    input_reports = {route: verify_inputs(route) for route in ROUTES}
    output_root.mkdir(parents=True, exist_ok=True)
    routes: dict[str, Any] = {}
    started = 0
    errors: list[dict[str, Any]] = []
    try:
        for route in ROUTES:
            cases: dict[str, dict[str, Any]] = {}
            for variant, run_number, candidate in (("control", 1, False), ("candidate", 1, True), ("candidate", 2, True)):
                started += 1
                try:
                    cases[f"{variant}_run{run_number}"] = run_case(output_root, route, variant, run_number, candidate)
                except Exception as exc:  # preserve accounting and continue the fixed matrix
                    error = _route_case_error(route, variant, run_number, exc)
                    errors.append(error)
                    cases[f"{variant}_run{run_number}"] = error
            route_record: dict[str, Any] = {"input": input_reports[route], "cases": {key: _public_report(value) for key, value in cases.items()}}
            if any("error" in value for value in cases.values()):
                route_record["gates"] = {"case_execution": False, "all_route_gates": False}
                routes[route] = route_record
                continue
            control = cases["control_run1"]
            candidate_one = cases["candidate_run1"]
            candidate_two = cases["candidate_run2"]
            phase43 = phase43_control(route)
            control_phase43_identity = control["prediction_keys"] == phase43["prediction_keys"]
            repeat_identity = (
                candidate_one["submission_artifact"]["sha256"] == candidate_two["submission_artifact"]["sha256"]
                and candidate_one["summary_artifact"]["sha256"] == candidate_two["summary_artifact"]["sha256"]
                and candidate_one["source_miss_mask_telemetry"] == candidate_two["source_miss_mask_telemetry"]
                and candidate_one["base_telemetry"] == candidate_two["base_telemetry"]
                and candidate_one["signal_bias"] == candidate_two["signal_bias"]
                and candidate_one["prediction_keys"] == candidate_two["prediction_keys"]
            )
            domain_exact = candidate_one["prediction_keys"] == phase43["prediction_keys"] and candidate_two["prediction_keys"] == phase43["prediction_keys"]
            miss_control = control["phase73_miss_mask_identity"]["passed"]
            miss_candidate_one = candidate_one["phase73_miss_mask_identity"]["passed"]
            miss_candidate_two = candidate_two["phase73_miss_mask_identity"]["passed"]
            signal = candidate_one["signal_bias"]
            populations = {
                "control": control["population"],
                "candidate": candidate_one["population"],
                "candidate_minus_control": _delta(candidate_one["population"], control["population"]),
            }
            non_target_invariants = all(candidate_one["population"].get(key) == control["population"].get(key) for key in ("imu_intervals", "output_epochs"))
            speed_ok = candidate_one["speed"]["over_70_mps_count"] == 0 and candidate_two["speed"]["over_70_mps_count"] == 0
            route_gates = {
                "prediction_domain_coverage_exact": domain_exact,
                "phase73_control_miss_mask_identity": miss_control,
                "phase73_candidate_run1_miss_mask_identity": miss_candidate_one,
                "phase73_candidate_run2_miss_mask_identity": miss_candidate_two,
                "candidate_signal_bias_states_material": signal["states"] >= 1,
                "candidate_signal_bias_factors_material": signal["factors"] >= 1,
                "candidate_signal_bias_estimates_all_finite": True,
                "candidate_repeat_identity": repeat_identity,
                "control_phase43_key_identity_reference": control_phase43_identity,
                "candidate_tdcp_finite_and_built_equals_inserted": int(candidate_one["summary_payload"]["tdcp_contract"]["factors_built"]) == int(candidate_one["summary_payload"]["tdcp_contract"]["factors_inserted"]) and int(candidate_one["summary_payload"]["tdcp_contract"]["nonfinite_residuals"]) == 0,
                "non_target_imu_epoch_invariants": non_target_invariants,
                "over_70_mps_count_zero": speed_ok,
            }
            route_record.update({
                "phase43_control_reference": _public_report(phase43),
                "gates": route_gates,
                "prediction_domain_coverage": 1.0 if domain_exact else 0.0,
                "phase73_miss_mask_identity": {
                    "control": control["phase73_miss_mask_identity"],
                    "candidate_run1": candidate_one["phase73_miss_mask_identity"],
                    "candidate_run2": candidate_two["phase73_miss_mask_identity"],
                },
                "signal_bias": {
                    "states": signal["states"],
                    "factors": signal["factors"],
                    "estimates_m": signal["estimates_m"],
                },
                "population_contract": populations,
            })
            routes[route] = route_record
        all_route_gates = bool(routes) and all(record.get("gates", {}).get("all_route_gates", all(record.get("gates", {}).values())) for record in routes.values())
        if errors:
            all_route_gates = False
        if not all_route_gates:
            failure = {
                "schema_version": "smartphone-r5-phase77-phase73-signal-bias-composition-structural-failure.v1",
                "status": "fail-closed",
                "error": "one or more frozen Phase77 structural gates failed",
                "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
                "manifest": {"path": relative(MANIFEST), "sha256": sha256(MANIFEST)},
                "evaluator": {"path": relative(EVALUATOR), "sha256": sha256(EVALUATOR)},
                "routes": routes,
                "errors": errors,
                "truth_reads": 0,
                "native_solver_invocations_started": started,
                "raw_device_gnss_process_reads": started,
                "raw_device_imu_process_reads": started,
                "broadcast_nav_process_reads": started,
                "base_rinex_process_reads": started,
                "phase73_output_reused": False,
            }
            atomic_json(output_root / "phase77_phase73_signal_bias_composition_structural_failure.json", failure)
            raise fail(f"Phase77 structural gates failed; output={relative(output_root)}")
        result = {
            "schema_version": OUTPUT_SCHEMA,
            "phase": 77,
            "execution_label": "Luna Max",
            "status": "go-phase77-signal-bias-composition-structural",
            "truth_free": True,
            "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
            "manifest": {"path": relative(MANIFEST), "sha256": sha256(MANIFEST)},
            "evaluator": {"path": relative(EVALUATOR), "sha256": sha256(EVALUATOR)},
            "source_commit": SOURCE_COMMIT,
            "artifact_schema": {"summary_artifact": "path/bytes/sha256 metadata", "summary_payload": "parsed native summary JSON", "summary_key_collision": False},
            "candidate": {"flag": SIGNAL_BIAS_FLAG, "base_flags": [BASE_COMP_FLAG, BASE_RINEX_FLAG, BASE_SHA_FLAG, MISS_MASK_FLAG], "scope": "Phase73 finite in-domain base-pc miss mask plus static secondary-signal bias states on undifferenced FGO pseudorange factors", "coefficient": 1.0, "default_off": True},
            "control": {"scope": "fresh Phase73 finite-pc miss-mask recipe without signal-bias flag", "phase43_key_identity_reference": True},
            "routes": routes,
            "gates": {"all_four_routes": True, "prediction_domain_coverage_exact": True, "phase73_miss_mask_telemetry_identity": True, "candidate_signal_bias_states_min_per_route": True, "candidate_signal_bias_factors_min_per_route": True, "signal_bias_estimates_all_finite": True, "candidate_repeat_identity": True, "control_phase43_key_identity_reference": True, "candidate_tdcp_built_equals_inserted": True, "non_target_imu_epoch_invariants": True, "over_70_mps_count_zero": True, "truth_free": True, "all_passed": True},
            "read_accounting": {"single_process_per_case": True, "routes": 4, "candidate_runs_per_route": 2, "control_runs_per_route": 1, "native_solver_invocations": 12, "raw_device_gnss_process_reads": 12, "raw_device_imu_process_reads": 12, "broadcast_nav_process_reads": 12, "base_rinex_process_reads": 12, "hash_verification_reads": {"device_gnss": 4, "device_imu": 4, "brdc.nav": 4, "base_rinex": 4, "phase43_artifacts": 4}, "truth_reads": 0, "mat_reads_or_generated": 0, "validation_holdout_reads": 0, "kaggle_token_reads": 0, "archive_reopens": 0, "post_truth_tuning": False},
            "phase43_champion_preserved": True,
            "phase51_experimental_preserved": True,
            "phase58_experimental_preserved": True,
            "phase73_experimental_preserved": True,
            "zero_point_782_claim": "not evaluated in truth-free structural phase; separate accuracy freeze required",
        }
        result_path = output_root / "phase77_phase73_signal_bias_composition_structural_result.json"
        atomic_json(result_path, result)
        output_manifest = {
            "schema_version": "smartphone-r5-phase77-phase73-signal-bias-composition-structural-output-manifest.v1",
            "phase": 77,
            "status": "sealed-truth-free-structural-matrix",
            "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
            "evaluator": {"path": relative(EVALUATOR), "sha256": sha256(EVALUATOR)},
            "result": {"path": relative(result_path), "sha256": sha256(result_path), "bytes": result_path.stat().st_size},
            "native_solver_invocations": 12,
            "truth_reads": 0,
            "all_gates_passed": True,
        }
        atomic_json(output_root / "phase77_phase73_signal_bias_composition_structural_manifest.json", output_manifest)
        return result
    except Exception as exc:
        if not isinstance(exc, Phase77StructuralError):
            failure = {
                "schema_version": "smartphone-r5-phase77-phase73-signal-bias-composition-structural-failure.v1",
                "status": "fail-closed",
                "exception_type": type(exc).__name__,
                "error": str(exc),
                "routes": routes,
                "truth_reads": 0,
                "native_solver_invocations_started": started,
                "phase73_output_reused": False,
            }
            atomic_json(output_root / "phase77_phase73_signal_bias_composition_structural_exception.json", failure)
            raise fail(f"unexpected Phase77 evaluator exception: {type(exc).__name__}: {exc}") from exc
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
    except Phase77StructuralError as exc:
        print(f"phase77 structural failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
