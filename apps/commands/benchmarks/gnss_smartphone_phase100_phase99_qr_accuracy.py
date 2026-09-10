#!/usr/bin/env python3
"""Phase100 contract and isolated evaluator for the Phase99 QR candidate.

The native execution wrapper is deliberately separate from this evaluator.
The wrapper may launch the native program with raw GNSS/IMU/navigation only
and seals solution-file metadata after each process exits.  This module never
launches native code and never reads raw inputs.  Its evaluation entry point
opens the two pinned truth files only after candidate-output preflight and
only in this evaluator process.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
EVALUATOR = Path(__file__).resolve()
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase100_phase99_qr_accuracy_gate_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase100_phase99_qr_accuracy_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase100_phase99_qr_accuracy_raw_evaluation_authorization_v1.json"
WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase100_phase99_qr_accuracy_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase100_phase99_qr_accuracy.py"
PHASE99_CONTRACT = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase99_main_multifrontal_qr.py"
PHASE95_WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase95_raw_input_path_corrected_execute.py"
PHASE95_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase95_raw_input_path_corrected_structural_result_v1.json"
PHASE82_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase82_phase80_direct_quality_accuracy_freeze_v1.json"
PHASE82_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase82_phase80_direct_quality_accuracy_manifest_v1.json"
PHASE82_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase82_phase80_direct_quality_accuracy.py"
PHASE43_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase43_native_fallback_seed_quality_anchor_recovery_freeze_v1.json"
PHASE43_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase43_native_fallback_seed_quality_anchor_recovery_evaluator_manifest_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase100-phase99-qr-accuracy-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase100_phase99_qr_accuracy_result_v1.json"
RESULT_MD = RESULT_JSON.with_suffix(".md")

FREEZE_SHA = "de2d4f5626782297b5535767ff1f867e29538c74edafff337504b763dbfc0ff9"
FREEZE_COMMIT = "2fb699a"
PHASE99_CONTRACT_SHA = "582f6b9a4485f5d8efb16ce4daad62c2673c19d08e724c263374fc66febf3e89"
PHASE95_WRAPPER_SHA = "5ac858cf156555652f6feae60c71a4237269d4b4c2cb70c1adf5a9e08f10e883"
PHASE95_RESULT_SHA = "beba25d4b0d910064d76bea7aa6b7a884937ca01c3b00cf0f7f9f4b6267ea281"
PHASE82_FREEZE_SHA = "33bbfc4051af20cd3f39fbf8620ea4d277c4acc5dbe5d8d1621452a657eebf6b"
PHASE82_MANIFEST_SHA = "77f3cc6f82b4953e3fd0a240eff149fdc6c96b9335cc5058c962010b985500a0"
PHASE82_EVALUATOR_SHA = "232ba0ab1fa7ebde6ddfaaf851d190d1ed32377ea056ab6c31e244139c2977fb"
PHASE43_FREEZE_SHA = "9278a835a21e3a19027aefd3675aba45d8f1f9e19183cbc5590850b776722400"
PHASE43_MANIFEST_SHA = "1433249a9ddd1809a00535b33fc67e3267a8cab29e28f773dd148f0962c44d1a"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
PROBLEM_EPOCHS = {route: rows + 1 for route, rows in DOMAIN_ROWS.items()}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
RAW_FLAGS = (
    ("--android-gnss", "device_gnss.csv", "__PHASE95_RAW_DEVICE_GNSS__"),
    ("--android-imu", "device_imu.csv", "__PHASE95_RAW_DEVICE_IMU__"),
    ("--nav", "brdc.nav", "__PHASE95_RAW_BROADCAST_NAV__"),
)
REQUIRED_FLAGS = (
    "--all-epochs",
    "--android-raw-utc-keys",
    "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback",
    "--native-pdc-imu-tdcp-no-bridge",
    "--native-source-direct-observable-quality",
    "--native-source-clock-c0d-factor",
    "--native-source-clock-c0d-meter-state-parity",
    "--native-source-clock-c0d-active-solve-diagnostic",
    "--native-source-clock-c0d-gnss-first-meter-state-handoff",
    "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver",
)
FORBIDDEN_FLAGS = (
    "--obs",
    "--native-base-rinex",
    "--native-base-pseudorange-compensation",
    "--native-base-pseudorange-source-miss-mask",
    "--native-pdc-state-bridge",
    "--native-upstream-quality",
    "--native-gnss-first-velocity-only-handoff",
    "--native-direct-doppler-wls-handoff",
    "--native-source-clock-c0d-phase94-stage-diagnostics",
    "--native-source-clock-c0d-phase96-main-diagnostics",
    "--native-source-clock-c0d-phase97-singular-system-diagnostics",
    "--native-source-clock-c0d-phase98-solver-rank-diagnostic",
)
FORBIDDEN_PATH_TERMS = (
    ".mat",
    "ground_truth",
    "validation",
    "holdout",
    "kaggle",
    "token",
    "base.rinex",
    "coordinate",
    "truth",
)
SELECTOR = "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver"
HANDOFF_SELECTOR = "--native-source-clock-c0d-gnss-first-meter-state-handoff"
OUTPUT_RELATIVE_ROOT = "output/smartphone-r5/phase100-phase99-qr-accuracy-v1/"
MANIFEST_SCHEMA = "smartphone-r5-phase100-phase99-qr-accuracy-manifest.v1"
AUTH_SCHEMA = "smartphone-r5-phase100-phase99-qr-accuracy-raw-evaluation-authorization.v1"
RESULT_SCHEMA = "smartphone-r5-phase100-phase99-qr-accuracy-result.v1"
CANDIDATE_ID = "phase100-phase99-main-multifrontal-qr-accuracy-v1"
MAX_SPEED_MPS = 70.0


class Phase100Error(ValueError):
    """Raised when a Phase100 contract or evaluation predicate fails closed."""


def fail(message: str) -> Phase100Error:
    return Phase100Error(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _forbidden_path(path: Path | str) -> bool:
    token = str(path).lower()
    return any(term in token for term in FORBIDDEN_PATH_TERMS)


def sha256_file(path: Path, label: str) -> str:
    if path.name in RAW_NAMES or _forbidden_path(path):
        raise fail(f"forbidden file hash: {label}: {path}")
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
    if _forbidden_path(path):
        raise fail(f"forbidden JSON path: {label}: {path}")
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


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def _expected_output(route: str, name: str) -> str:
    return f"{OUTPUT_RELATIVE_ROOT}{route.replace('/', '__')}/{name}"


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_file(FREEZE, "Phase100 freeze"), FREEZE_SHA, "freeze sha256")
    freeze = read_json(FREEZE, "Phase100 freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase100-phase99-qr-accuracy-gate-freeze.v1",
        "phase": 100,
        "execution_label": "Luna Max",
        "status": "frozen-before-phase100-solution-and-truth-evaluation",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    candidate = freeze.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "candidate_count": 1,
        "id": CANDIDATE_ID,
        "runs_per_route": 1,
        "fresh_solution_required": True,
        "phase99_withheld_solution_reuse": False,
        "raw_only_solution_generation": True,
        "no_raw_content_copy_or_transform": True,
        "gnss_first_solver_unchanged": True,
        "legacy_default_unchanged": True,
        "no_fallback_or_rerun": True,
        "no_cholesky_comparison_rerun": True,
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    assert_equal(candidate.get("routes"), list(ROUTES), "freeze/candidate/routes")
    assert_equal(candidate.get("input_names_exact"), list(RAW_NAMES), "freeze/candidate/raw names")
    cohort = freeze.get("cohort")
    if not isinstance(cohort, dict):
        raise fail("freeze/cohort missing")
    assert_equal(cohort.get("route_order"), list(ROUTES), "freeze/cohort/routes")
    assert_equal(cohort.get("route_set_exact"), True, "freeze/cohort/exact")
    assert_equal(cohort.get("phase82_four_route_macro_direct_comparison"), False, "freeze/cohort/macro comparability")
    route_records = cohort.get("routes")
    if not isinstance(route_records, dict) or set(route_records) != set(ROUTES):
        raise fail("freeze/cohort route metadata changed")
    for route in ROUTES:
        record = route_records[route]
        if not isinstance(record, dict):
            raise fail(f"freeze/cohort route record is not an object: {route}")
        for key, expected in {
            "expected_problem_epochs": PROBLEM_EPOCHS[route],
            "expected_candidate_prediction_rows": DOMAIN_ROWS[route],
        }.items():
            assert_equal(record.get(key), expected, f"freeze/cohort/{route}/{key}")
    metric = freeze.get("metric_contract")
    if not isinstance(metric, dict):
        raise fail("freeze/metric_contract missing")
    for key, expected in {
        "key": "(phone, UnixTimeMillis)",
        "matching": "exact integer key intersection only",
        "prediction_domain_coverage": "matched prediction keys / prediction keys; required exactly 1.0",
        "distance": "spherical Haversine per row",
        "earth_radius_m": 6371008.8,
        "percentile": "linear interpolation at rank (n - 1) * q",
        "route_scalar": "(P50 + P95) / 2 in metres",
        "macro": "unweighted arithmetic mean of the two route scalars in fixed route order",
    }.items():
        assert_equal(metric.get(key), expected, f"freeze/metric/{key}")
    gates = freeze.get("promotion_gates")
    if not isinstance(gates, dict):
        raise fail("freeze/promotion_gates missing")
    for key, expected in {
        "all_gates_anded": True,
        "candidate_runs_per_route": 1,
        "candidate_route_set_exact": True,
        "candidate_prediction_domain_coverage": 1.0,
        "candidate_improves_each_route_by_at_least_m": 0.05,
        "candidate_no_route_regression": True,
        "candidate_macro_improvement_vs_same_route_phase43_at_least_m": 0.1,
        "candidate_route_score_max_m": 3.0,
        "candidate_macro_score_max_m": 2.0,
        "candidate_macro_score_strict_max_m": 0.782,
    }.items():
        assert_equal(gates.get(key), expected, f"freeze/gates/{key}")
    release = freeze.get("release_boundary")
    if not isinstance(release, dict):
        raise fail("freeze/release_boundary missing")
    for key in ("raw_execution_authorized", "truth_evaluation_authorized", "accuracy_promotion_authorized", "solution_csv_publication_authorized", "kaggle_submission_authorized"):
        assert_equal(release.get(key), False, f"freeze/release/{key}")
    return freeze


def _validate_command(route: str, command: Any, record: dict[str, Any]) -> None:
    if not isinstance(command, list) or not command or not all(isinstance(token, str) for token in command):
        raise fail(f"malformed command: {route}")
    assert_equal(command[0], "build/apps/gnss_fgo_imu_no_base", f"command/{route}/binary")
    for token in command:
        if _forbidden_path(token) or token in FORBIDDEN_FLAGS:
            raise fail(f"forbidden native command token: {route}/{token}")
    for flag in REQUIRED_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
    for flag in ("--native-source-clock-c0d-phase94-stage-diagnostics", "--native-source-clock-c0d-phase96-main-diagnostics", "--native-source-clock-c0d-phase97-singular-system-diagnostics", "--native-source-clock-c0d-phase98-solver-rank-diagnostic"):
        assert_equal(command.count(flag), 0, f"command/{route}/withholding-selector/{flag}")
    assert_equal(command.count("--dataset-id"), 1, f"command/{route}/dataset-id-count")
    assert_equal(command[command.index("--dataset-id") + 1], route, f"command/{route}/dataset-id")
    raw = record.get("raw_inputs")
    if not isinstance(raw, dict) or set(raw) != set(RAW_NAMES):
        raise fail(f"manifest raw-input roles changed: {route}")
    for flag, name, placeholder in RAW_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
        assert_equal(command[command.index(flag) + 1], placeholder, f"command/{route}/{name}/placeholder")
        pin = raw.get(name)
        if not isinstance(pin, dict):
            raise fail(f"manifest raw pin missing: {route}/{name}")
        assert_equal(pin.get("path"), placeholder, f"manifest/raw/{route}/{name}/path")
        assert_equal(pin.get("path_source"), "Phase95 sealed result raw_inputs.path", f"manifest/raw/{route}/{name}/source")
        assert_equal(pin.get("sha256"), None, f"manifest/raw/{route}/{name}/sha256")
        assert_equal(pin.get("read_at_manifest_creation"), False, f"manifest/raw/{route}/{name}/read")
    assert_equal(command.count("--out"), 1, f"command/{route}/out-count")
    assert_equal(command[command.index("--out") + 1], _expected_output(route, "solution.csv"), f"command/{route}/out")
    assert_equal(command.count("--summary-json"), 1, f"command/{route}/summary-count")
    assert_equal(command[command.index("--summary-json") + 1], _expected_output(route, "summary.json"), f"command/{route}/summary")
    planned = record.get("planned_output")
    if not isinstance(planned, dict):
        raise fail(f"manifest planned output missing: {route}")
    assert_equal(planned.get("solution"), _expected_output(route, "solution.csv"), f"manifest/{route}/solution")
    assert_equal(planned.get("summary"), _expected_output(route, "summary.json"), f"manifest/{route}/summary")
    assert_equal(planned.get("solution_metadata"), _expected_output(route, "solution_metadata.json"), f"manifest/{route}/solution-metadata")
    assert_equal(planned.get("run_metadata"), _expected_output(route, "run_metadata.json"), f"manifest/{route}/run-metadata")


def verify_manifest() -> dict[str, Any]:
    verify_freeze()
    assert_equal(sha256_file(PHASE99_CONTRACT, "Phase99 contract"), PHASE99_CONTRACT_SHA, "Phase99 contract sha256")
    assert_equal(sha256_file(PHASE95_WRAPPER, "Phase95 path wrapper"), PHASE95_WRAPPER_SHA, "Phase95 wrapper sha256")
    assert_equal(sha256_file(PHASE95_RESULT, "Phase95 result"), PHASE95_RESULT_SHA, "Phase95 result sha256")
    manifest = read_json(MANIFEST, "Phase100 manifest")
    for key, expected in {
        "schema_version": MANIFEST_SCHEMA,
        "phase": 100,
        "execution_label": "Luna Max",
        "status": "sealed-before-phase100-raw-and-truth-evaluation",
    }.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    freeze_ref = manifest.get("freeze")
    if not isinstance(freeze_ref, dict):
        raise fail("manifest/freeze missing")
    for key, expected in {
        "path": relative(FREEZE),
        "sha256": FREEZE_SHA,
        "commit": FREEZE_COMMIT,
        "execution_authorized_before_manifest": False,
    }.items():
        assert_equal(freeze_ref.get(key), expected, f"manifest/freeze/{key}")
    candidate = manifest.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("manifest/candidate missing")
    for key, expected in {
        "candidate_count": 1,
        "id": CANDIDATE_ID,
        "source_candidate": "phase99-main-meter-state-multifrontal-qr-branch-only",
        "runs_per_route": 1,
        "raw_only": True,
        "fresh_solution_required": True,
        "phase99_withheld_solution_reuse": False,
        "diagnostic_withholding_selectors_absent": True,
        "no_solver_filter_or_lm_change": True,
        "no_equation_unit_sigma_change": True,
        "no_fallback_or_rerun": True,
        "solution_publication": False,
        "truth_evaluation_in_native": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"manifest/candidate/{key}")
    assert_equal(candidate.get("routes"), list(ROUTES), "manifest/candidate/routes")
    assert_equal(candidate.get("input_names_exact"), list(RAW_NAMES), "manifest/candidate/raw names")
    assert_equal(candidate.get("solver_branch", {}).get("selected_main_solver_type"), "MULTIFRONTAL_QR", "manifest/solver")
    assert_equal(candidate.get("solver_branch", {}).get("gnss_first_solver_type"), "MULTIFRONTAL_CHOLESKY", "manifest/gnss-first solver")
    assert_equal(candidate.get("solver_branch", {}).get("legacy_solver_type"), "MULTIFRONTAL_CHOLESKY", "manifest/legacy solver")
    routes = manifest.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("manifest route order/count changed")
    for record in routes:
        route = record.get("dataset_id")
        if route not in DOMAIN_ROWS:
            raise fail(f"unknown manifest route: {route}")
        assert_equal(record.get("runs"), 1, f"manifest/{route}/runs")
        assert_equal(record.get("domain_rows"), DOMAIN_ROWS[route], f"manifest/{route}/domain_rows")
        assert_equal(record.get("problem_epochs"), PROBLEM_EPOCHS[route], f"manifest/{route}/problem_epochs")
        assert_equal(record.get("diagnostic_only"), False, f"manifest/{route}/diagnostic-only")
        _validate_command(route, record.get("command"), record)
    matrix = manifest.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("manifest/matrix missing")
    for key, expected in {
        "candidate_count": 1,
        "route_count": 2,
        "runs_per_route": 1,
        "native_invocations_planned": 2,
        "controls_in_native": 0,
        "reruns": 0,
        "fallbacks": 0,
        "truth_reads_in_native": 0,
        "truth_reads_in_evaluator_planned": 2,
        "mat_reads_or_generated": 0,
        "base_rinex_reads": 0,
        "pdc_or_precomputed_coordinate_reads": 0,
        "kaggle_or_token_access": 0,
        "solution_rows_published": False,
        "route_score_selection": False,
    }.items():
        assert_equal(matrix.get(key), expected, f"manifest/matrix/{key}")
    policy = manifest.get("read_policy_before_execution")
    if not isinstance(policy, dict):
        raise fail("manifest/read_policy_before_execution missing")
    for key in ("native_solver_invocations", "raw_gnss_reads", "raw_imu_reads", "navigation_reads", "truth_reads", "mat_reads_or_generated", "base_reads", "precomputed_coordinate_reads", "accuracy_calculations", "kaggle_or_token_access", "solution_outputs_generated"):
        assert_equal(policy.get(key), 0, f"manifest/read-policy/{key}")
    assert_equal(policy.get("raw_content_copied_or_transformed"), False, "manifest/read-policy/copy")
    execution = manifest.get("execution_authorization")
    if not isinstance(execution, dict):
        raise fail("manifest/execution_authorization missing")
    for key, expected in {
        "raw_execution_authorized": False,
        "truth_evaluation_authorized": False,
        "solution_publication_authorized": False,
        "stop_after_two_native_routes": True,
    }.items():
        assert_equal(execution.get(key), expected, f"manifest/execution/{key}")
    for key, path, label in (
        ("pre_raw_evaluator", EVALUATOR, "Phase100 evaluator"),
        ("execution_wrapper", WRAPPER, "Phase100 execution wrapper"),
        ("focused_tests", FOCUSED_TESTS, "Phase100 focused tests"),
    ):
        pin = manifest.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/{key} pin missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/{key}/path")
        assert_equal(pin.get("sha256"), sha256_file(path, label), f"manifest/{key}/sha256")
    return manifest


def verify_authorization(manifest: dict[str, Any] | None = None) -> dict[str, Any]:
    if manifest is None:
        manifest = verify_manifest()
    auth = read_json(AUTHORIZATION, "Phase100 authorization")
    for key, expected in {
        "schema_version": AUTH_SCHEMA,
        "phase": 100,
        "execution_label": "Luna Max",
        "status": "authorized-for-exact-two-route-phase100-qr-raw-and-isolated-truth-evaluation",
    }.items():
        assert_equal(auth.get(key), expected, f"authorization/{key}")
    authority = auth.get("authority")
    if not isinstance(authority, dict):
        raise fail("authorization/authority missing")
    pins = {
        "freeze": (FREEZE, FREEZE_SHA),
        "manifest": (MANIFEST, sha256_file(MANIFEST, "Phase100 manifest")),
        "evaluator": (EVALUATOR, sha256_file(EVALUATOR, "Phase100 evaluator")),
        "wrapper": (WRAPPER, sha256_file(WRAPPER, "Phase100 wrapper")),
        "focused_tests": (FOCUSED_TESTS, sha256_file(FOCUSED_TESTS, "Phase100 focused tests")),
        "phase99_contract": (PHASE99_CONTRACT, PHASE99_CONTRACT_SHA),
        "phase95_wrapper": (PHASE95_WRAPPER, PHASE95_WRAPPER_SHA),
        "phase95_result": (PHASE95_RESULT, PHASE95_RESULT_SHA),
        "phase82_freeze": (PHASE82_FREEZE, PHASE82_FREEZE_SHA),
        "phase82_manifest": (PHASE82_MANIFEST, PHASE82_MANIFEST_SHA),
        "phase82_evaluator": (PHASE82_EVALUATOR, PHASE82_EVALUATOR_SHA),
        "phase43_freeze": (PHASE43_FREEZE, PHASE43_FREEZE_SHA),
        "phase43_manifest": (PHASE43_MANIFEST, PHASE43_MANIFEST_SHA),
    }
    for key, (path, digest) in pins.items():
        pin = authority.get(key)
        if not isinstance(pin, dict):
            raise fail(f"authorization/{key} pin missing")
        assert_equal(pin.get("path"), relative(path), f"authorization/{key}/path")
        assert_equal(pin.get("sha256"), digest, f"authorization/{key}/sha256")
    assert_equal(authority.get("freeze_commit"), FREEZE_COMMIT, "authorization/freeze_commit")
    assert_equal(authority.get("candidate_count"), 1, "authorization/candidate_count")
    assert_equal(authority.get("routes"), list(ROUTES), "authorization/routes")
    candidate = auth.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("authorization/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID,
        "candidate_count": 1,
        "runs_per_route": 1,
        "raw_only": True,
        "raw_input_names_exact": list(RAW_NAMES),
        "truth_mat_base_pdc_precomputed_kaggle": False,
        "no_fallback_or_rerun": True,
        "solution_publication": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"authorization/candidate/{key}")
    matrix = auth.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("authorization/matrix missing")
    for key, expected in {
        "route_count": 2,
        "native_invocations": 2,
        "runs_per_route": 1,
        "controls_in_native": 0,
        "reruns": 0,
        "fallbacks": 0,
        "truth_reads_in_native": 0,
        "truth_reads_in_evaluator": 2,
        "mat_reads_or_generated": 0,
        "base_reads": 0,
        "pdc_or_precomputed_coordinate_reads": 0,
        "kaggle_or_token_access": 0,
        "solution_publication": False,
    }.items():
        assert_equal(matrix.get(key), expected, f"authorization/matrix/{key}")
    policy = auth.get("execution_policy")
    if not isinstance(policy, dict):
        raise fail("authorization/execution_policy missing")
    for key, expected in {
        "raw_execution_authorized": True,
        "truth_evaluation_authorized": True,
        "one_native_invocation_per_route": True,
        "sequential_route_order": "MTV-A then LAX-T",
        "stop_after_two_routes": True,
        "native_truth_path_or_env": False,
        "native_mat_base_pdc_precomputed_coordinate": False,
        "native_fallback_or_rerun": False,
        "solution_publication": False,
        "kaggle_submission": False,
    }.items():
        assert_equal(policy.get(key), expected, f"authorization/policy/{key}")
    return auth


def verify_pre_raw() -> dict[str, Any]:
    verify_manifest()
    return {
        "raw_reads": 0,
        "native_solver_invocations": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "accuracy_calculations": 0,
        "kaggle_or_token_access": 0,
        "solution_outputs_generated": 0,
        "raw_content_copied_or_transformed": False,
    }


def phase95_raw_paths() -> dict[str, dict[str, dict[str, Any]]]:
    """Resolve Phase95 raw path metadata and stat files, never read bytes."""

    actual = sha256_file(PHASE95_WRAPPER, "Phase95 path wrapper")
    assert_equal(actual, PHASE95_WRAPPER_SHA, "Phase95 wrapper sha256")
    spec = importlib.util.spec_from_file_location("phase95_phase100_path_materializer", PHASE95_WRAPPER)
    if spec is None or spec.loader is None:
        raise fail("unable to load Phase95 path materializer")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    try:
        all_routes = module.load_inherited_raw_inputs()
    except Exception as exc:
        raise fail(f"Phase95 raw path materialization failed: {exc}") from exc
    if not isinstance(all_routes, dict):
        raise fail("Phase95 path materializer returned no route map")
    selected: dict[str, dict[str, dict[str, Any]]] = {}
    for route in ROUTES:
        record = all_routes.get(route)
        if not isinstance(record, dict) or set(record) != set(RAW_NAMES):
            raise fail(f"Phase95 raw roles changed: {route}")
        selected[route] = {}
        for name in RAW_NAMES:
            pin = record[name]
            if not isinstance(pin, dict) or not isinstance(pin.get("path"), str):
                raise fail(f"Phase95 raw path missing: {route}/{name}")
            path = Path(pin["path"])
            if path.is_absolute() or ".." in path.parts or path.name != name or _forbidden_path(path):
                raise fail(f"unsafe Phase95 raw path: {route}/{name}")
            if not path.is_file():
                raise fail(f"missing Phase95 raw path: {route}/{name}")
            if pin.get("read_by_runner") is not False:
                raise fail(f"Phase95 path resolver read raw bytes: {route}/{name}")
            selected[route][name] = dict(pin)
    return selected


def materialize_command(record: dict[str, Any], raw: dict[str, dict[str, Any]]) -> list[str]:
    route = record.get("dataset_id")
    if route not in ROUTES:
        raise fail(f"unknown route in command: {route}")
    _validate_command(route, record.get("command"), record)
    command = list(record["command"])
    for flag, name, _placeholder in RAW_FLAGS:
        pin = raw.get(name)
        if not isinstance(pin, dict) or not isinstance(pin.get("path"), str):
            raise fail(f"resolved Phase95 path missing: {route}/{name}")
        path = Path(pin["path"])
        if path.is_absolute() or ".." in path.parts or path.name != name:
            raise fail(f"unsafe materialized raw path: {route}/{name}")
        command[command.index(flag) + 1] = pin["path"]
    return command


def _read_nontruth_bytes(path: Path, label: str) -> bytes:
    if path.name in RAW_NAMES or _forbidden_path(path):
        raise fail(f"forbidden nontruth read: {label}: {path}")
    try:
        with path.open("rb") as handle:
            return handle.read()
    except OSError as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc


def _solution_file_metadata(path: Path, route: str) -> dict[str, Any]:
    """Read only candidate solution bytes; never parse or retain coordinates."""

    if not path.is_file():
        return {
            "path": relative(path),
            "present": False,
            "bytes": None,
            "sha256": None,
            "rows": None,
            "header": None,
            "route": route,
            "read_by_wrapper": False,
            "truth_read_by_wrapper": False,
        }
    payload = _read_nontruth_bytes(path, f"solution {route}")
    try:
        text = payload.decode("utf-8-sig")
        rows = list(csv.reader(io.StringIO(text, newline="")))
    except (UnicodeDecodeError, csv.Error) as exc:
        raise fail(f"solution metadata parse failed: {route}: {exc}") from exc
    return {
        "path": relative(path),
        "present": True,
        "bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
        "rows": max(0, len(rows) - 1) if rows else 0,
        "header": rows[0] if rows else None,
        "route": route,
        "read_by_wrapper": True,
        "truth_read_by_wrapper": False,
    }


def _generic_file_metadata(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file():
        return {"path": relative(path), "present": False, "bytes": None, "sha256": None}
    payload = _read_nontruth_bytes(path, label)
    return {"path": relative(path), "present": True, "bytes": len(payload), "sha256": hashlib.sha256(payload).hexdigest()}


def seal_solution_metadata(route: str, solution_path: Path) -> dict[str, Any]:
    metadata = _solution_file_metadata(solution_path, route)
    metadata["sealed_after_native_exit"] = True
    metadata["coordinates_published_in_metadata"] = False
    metadata["raw_byte_reads"] = 0
    metadata["truth_reads"] = 0
    metadata["mat_base_pdc_precomputed_coordinate_reads"] = 0
    return metadata


def _import_phase82() -> Any:
    if sha256_file(PHASE82_EVALUATOR, "Phase82 evaluator") != PHASE82_EVALUATOR_SHA:
        raise fail("Phase82 evaluator hash changed")
    spec = importlib.util.spec_from_file_location("phase82_accuracy_reference_phase100", PHASE82_EVALUATOR)
    if spec is None or spec.loader is None:
        raise fail("unable to load Phase82 metric reference")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _read_truth_once(path: Path, expected_sha: str, expected_bytes: int, label: str) -> bytes:
    """The only truth-file read primitive; one open, hash from retained bytes."""

    if not _forbidden_path(path) or "ground_truth" not in str(path).lower():
        raise fail(f"truth path is not a pinned ground-truth path: {path}")
    try:
        with path.open("rb") as handle:
            payload = handle.read()
    except OSError as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    if len(payload) != expected_bytes:
        raise fail(f"truth byte-size mismatch: {path}")
    if hashlib.sha256(payload).hexdigest() != expected_sha:
        raise fail(f"truth hash mismatch: {path}")
    return payload


def _finite_tree(value: Any) -> bool:
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, dict):
        return all(_finite_tree(child) for child in value.values())
    if isinstance(value, list):
        return all(_finite_tree(child) for child in value)
    return True


def _forbidden_summary_keys(value: Any, path: str = "summary") -> list[str]:
    forbidden = {
        "latitude",
        "longitude",
        "position_ecef",
        "receiver_clock_bias",
        "epoch_clock_drift_mps",
        "epoch_velocity_nav_mps",
        "epoch_velocities_ecef_mps",
        "solutions",
    }
    found: list[str] = []
    if isinstance(value, dict):
        for key, child in value.items():
            if str(key).lower() in forbidden:
                found.append(f"{path}.{key}")
            found.extend(_forbidden_summary_keys(child, f"{path}.{key}"))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            found.extend(_forbidden_summary_keys(child, f"{path}[{index}]"))
    return found


def _summary_telemetry(summary: dict[str, Any]) -> dict[str, Any]:
    def pick(value: Any, keys: tuple[str, ...]) -> dict[str, Any]:
        if not isinstance(value, dict):
            return {key: None for key in keys}
        return {key: value.get(key) for key in keys}

    return {
        "schema_version": summary.get("schema_version"),
        "status": summary.get("status"),
        "selected_linear_solver_type": summary.get("selected_linear_solver_type"),
        "selected_solver_branch": summary.get("selected_solver_branch"),
        "selected_elimination_function": summary.get("selected_elimination_function"),
        "phase99_solver_enabled": summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_enabled"),
        "phase99_solver_selected": summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected"),
        "gnss_first": pick(summary.get("gnss_first"), ("attempted", "converged", "epochs", "iterations", "initial_cost", "final_cost", "c0d_factor_count", "c0d_accepted_outer_iterations", "strict_cost_progress", "optimized_d_epoch_count", "optimized_d_finite_count", "optimized_d_nonfinite_count", "optimized_d_coverage", "exact_retained_key_alignment", "terminal_branch")),
        "main": pick(summary.get("main"), ("attempted", "converged", "problem_epoch_count", "accepted_outer_iterations", "initial_cost", "final_cost", "final_cost_strictly_less_than_initial", "optimized_d_epoch_count", "optimized_d_finite_count", "optimized_d_nonfinite_count", "optimized_d_all_finite", "exact_retained_key_alignment", "terminal_branch")),
    }


def _validate_native_summary(payload: bytes, route: str) -> tuple[dict[str, Any], list[str]]:
    try:
        summary = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"invalid native summary: {route}: {exc}") from exc
    if not isinstance(summary, dict):
        raise fail(f"native summary is not an object: {route}")
    assert_equal(summary.get("dataset_id"), route, f"summary/{route}/dataset")
    assert_equal(summary.get("truth_used"), False, f"summary/{route}/truth_used")
    assert_equal(summary.get("production_default_changed"), False, f"summary/{route}/default")
    assert_equal(summary.get("mat_used", False), False, f"summary/{route}/mat")
    assert_equal(summary.get("base_factors"), False, f"summary/{route}/base")
    assert_equal(summary.get("native_pdc_state_bridge"), False, f"summary/{route}/pdc")
    assert_equal(summary.get("native_source_clock_c0d_factor_enabled"), True, f"summary/{route}/c0d")
    assert_equal(summary.get("native_source_clock_c0d_meter_state_parity_enabled"), True, f"summary/{route}/meter parity")
    assert_equal(summary.get("native_source_clock_c0d_gnss_first_meter_state_handoff_enabled"), True, f"summary/{route}/handoff")
    assert_equal(summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_enabled"), True, f"summary/{route}/qr enabled")
    assert_equal(summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected"), True, f"summary/{route}/qr selected")
    assert_equal(summary.get("selected_linear_solver_type"), "MULTIFRONTAL_QR", f"summary/{route}/solver")
    assert_equal(summary.get("selected_solver_branch"), "multifrontal", f"summary/{route}/branch")
    assert_equal(summary.get("selected_elimination_function"), "EliminateQR", f"summary/{route}/elimination")
    if not _finite_tree(summary):
        raise fail(f"native summary has nonfinite telemetry: {route}")
    forbidden = _forbidden_summary_keys(summary)
    return summary, forbidden


def _load_run_metadata(route: str, output_root: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    route_dir = output_root / route.replace("/", "__")
    run_path = route_dir / "run_metadata.json"
    solution_meta_path = route_dir / "solution_metadata.json"
    run = read_json(run_path, f"run metadata {route}")
    solution_meta = read_json(solution_meta_path, f"solution metadata {route}")
    assert_equal(run.get("dataset_id"), route, f"run metadata/{route}/dataset")
    assert_equal(run.get("run_number"), 1, f"run metadata/{route}/run")
    assert_equal(run.get("truth_read_by_wrapper"), False, f"run metadata/{route}/truth")
    assert_equal(run.get("raw_byte_reads_by_wrapper"), 0, f"run metadata/{route}/raw reads")
    assert_equal(run.get("raw_content_copied_or_transformed"), False, f"run metadata/{route}/copy")
    command = run.get("command")
    if not isinstance(command, list) or any(_forbidden_path(token) for token in command):
        raise fail(f"run metadata native command leakage: {route}")
    assert_equal(solution_meta.get("route"), route, f"solution metadata/{route}/route")
    assert_equal(solution_meta.get("truth_reads"), 0, f"solution metadata/{route}/truth")
    assert_equal(solution_meta.get("raw_byte_reads"), 0, f"solution metadata/{route}/raw")
    return run, solution_meta


def _validate_solution_pretruth(route: str, output_root: Path, solution_meta: dict[str, Any]) -> tuple[list[tuple[int, float, float]], dict[int, tuple[float, float]], dict[str, Any], bytes]:
    solution_path = output_root / route.replace("/", "__") / "solution.csv"
    if solution_meta.get("present") is not True:
        raise fail(f"candidate solution missing: {route}")
    payload = _read_nontruth_bytes(solution_path, f"candidate solution {route}")
    if len(payload) != solution_meta.get("bytes") or hashlib.sha256(payload).hexdigest() != solution_meta.get("sha256"):
        raise fail(f"candidate solution seal mismatch: {route}")
    if solution_meta.get("rows") != DOMAIN_ROWS[route]:
        raise fail(f"candidate solution row count mismatch: {route}")
    p82 = _import_phase82()
    try:
        ordered, mapping = p82.P76.P74._parse_submission(payload, route)
    except Exception as exc:
        raise fail(f"candidate solution schema/key preflight failed: {route}: {exc}") from exc
    if len(ordered) != DOMAIN_ROWS[route]:
        raise fail(f"candidate prediction domain row mismatch: {route}")
    solution_file_meta = {
        "path": relative(solution_path),
        "present": True,
        "bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
        "rows": solution_meta.get("rows"),
        "header": solution_meta.get("header"),
    }
    return ordered, mapping, solution_file_meta, payload


def _phase43_control(p82: Any, phase82_freeze: dict[str, Any], phase43_seal: dict[str, Any], route: str, accounting: dict[str, int]) -> tuple[list[tuple[int, float, float]], dict[int, tuple[float, float]], dict[str, Any]]:
    control_payload, control_summary_payload, control_pin = p82._artifact(phase82_freeze, "phase43_control", route, accounting, phase43_seal)
    ordered, mapping = p82.P76.P74._parse_submission(control_payload, route)
    if len(ordered) != DOMAIN_ROWS[route]:
        raise fail(f"Phase43 control domain row mismatch: {route}")
    try:
        summary = p82._validate_summary(control_summary_payload, route, "Phase43 control")
    except Exception as exc:
        raise fail(f"Phase43 control summary failed: {route}: {exc}") from exc
    return ordered, mapping, {
        "submission_sha256": control_pin.get("submission_sha256"),
        "summary_sha256": control_pin.get("summary_sha256"),
        "submission_rows": len(ordered),
        "summary_dataset_id": summary.get("dataset_id"),
    }


def _route_report(route: str, output_root: Path, phase100_freeze: dict[str, Any], phase82_freeze: dict[str, Any], phase43_seal: dict[str, Any], accounting: dict[str, int]) -> dict[str, Any]:
    report: dict[str, Any] = {"dataset_id": route, "truth_read": False, "accuracy_scored": False}
    try:
        run, solution_meta = _load_run_metadata(route, output_root)
        report["native"] = {
            "return_code": run.get("return_code"),
            "interrupted": run.get("interrupted"),
            "launch_error": run.get("launch_error", ""),
            "summary": run.get("summary"),
            "solution": run.get("solution"),
        }
        if run.get("return_code") != 0 or run.get("interrupted") is True or run.get("launch_error"):
            raise fail(f"native solver did not exit successfully: {route}")
        ordered, candidate, solution_file_meta, _solution_payload = _validate_solution_pretruth(route, output_root, solution_meta)
        report["candidate_solution"] = {
            "path": solution_file_meta["path"],
            "sha256": solution_file_meta["sha256"],
            "bytes": solution_file_meta["bytes"],
            "rows": solution_file_meta["rows"],
            "header": solution_meta.get("header"),
            "pretruth_schema_valid": True,
        }
        summary_path = output_root / route.replace("/", "__") / "summary.json"
        summary_payload = _read_nontruth_bytes(summary_path, f"native summary {route}")
        accounting["summary_reads"] += 1
        summary_hash = hashlib.sha256(summary_payload).hexdigest()
        expected_summary_hash = run.get("summary_metadata", {}).get("sha256")
        if expected_summary_hash and summary_hash != expected_summary_hash:
            raise fail(f"native summary seal mismatch: {route}")
        summary, forbidden_summary_keys = _validate_native_summary(summary_payload, route)
        report["native"]["summary_sha256"] = summary_hash
        report["native"]["telemetry"] = _summary_telemetry(summary)
        report["native"]["forbidden_solution_keys"] = forbidden_summary_keys
        if forbidden_summary_keys:
            raise fail(f"native summary contains solution fields: {route}")
        p82 = _import_phase82()
        control_ordered, control, control_meta = _phase43_control(p82, phase82_freeze, phase43_seal, route, accounting)
        if candidate.keys() != control.keys():
            raise fail(f"candidate/control prediction domains differ before truth: {route}")
        route_contract = phase100_freeze["cohort"]["routes"][route]
        truth_pin = route_contract["truth_pin"]
        expected_missing_key = route_contract.get("expected_missing_truth_key")
        truth_path = ROOT / truth_pin["path"]
        accounting["truth_reads"] += 1
        truth_payload = _read_truth_once(truth_path, truth_pin["sha256"], int(truth_pin["bytes"]), f"official truth {route}")
        truth = p82.P76._parse_truth_dictreader(truth_payload, route)
        if len(truth) != int(truth_pin["rows"]):
            raise fail(f"official truth row count mismatch: {route}")
        candidate_score = p82.P76._score_prediction(candidate, truth, expected_missing_key, route, ordered)
        control_score = p82.P76._score_prediction(control, truth, expected_missing_key, route, control_ordered)
        accounting["accuracy_calculations"] += 1
        improvement = control_score["score_m"] - candidate_score["score_m"]
        route_gates = {
            "native_exit_zero": run.get("return_code") == 0,
            "candidate_prediction_domain_coverage_exact": candidate_score["prediction_domain_coverage"] == 1.0,
            "candidate_finite": candidate_score["finite"] is True,
            "candidate_over_70_mps_count_zero": candidate_score["over_70_mps_count"] == 0,
            "candidate_improvement_at_least_0_05m": improvement >= 0.05,
            "candidate_no_route_regression": improvement >= 0.0,
            "candidate_route_score_at_most_3m": candidate_score["score_m"] <= 3.0,
            "phase43_control_identity": bool(control_meta["submission_sha256"] and control_meta["summary_sha256"]),
        }
        report.update({
            "truth": {
                "path": relative(truth_path),
                "sha256": truth_pin["sha256"],
                "bytes": len(truth_payload),
                "rows": len(truth),
                "read_count": 1,
                "expected_missing_truth_rows": route_contract["expected_missing_truth_rows"],
                "expected_missing_key": expected_missing_key,
            },
            "truth_read": True,
            "accuracy_scored": True,
            "artifact_hashes": {
                "candidate_solution": solution_file_meta["sha256"],
                "candidate_summary": summary_hash,
                "phase43_control_submission": control_meta["submission_sha256"],
                "phase43_control_summary": control_meta["summary_sha256"],
            },
            "candidate": candidate_score,
            "control_phase43": control_score,
            "improvement_vs_phase43_m": improvement,
            "gates": {"passed": all(route_gates.values()), "checks": route_gates, "failures": [key for key, value in route_gates.items() if not value]},
        })
    except Exception as exc:
        report["failure"] = str(exc)
        report.setdefault("gates", {"passed": False, "checks": {}, "failures": ["route_evaluation"]})
    return report


def _route_failure_report(route: str, error: str) -> dict[str, Any]:
    return {"dataset_id": route, "truth_read": False, "accuracy_scored": False, "failure": error, "gates": {"passed": False, "checks": {}, "failures": ["route_evaluation"]}}


def evaluate(output_root: Path = OUTPUT_ROOT, result_path: Path = RESULT_JSON) -> dict[str, Any]:
    phase100_freeze = verify_freeze()
    manifest = verify_manifest()
    authorization = verify_authorization(manifest)
    phase82 = _import_phase82()
    if sha256_file(PHASE82_FREEZE, "Phase82 freeze") != PHASE82_FREEZE_SHA or sha256_file(PHASE82_MANIFEST, "Phase82 manifest") != PHASE82_MANIFEST_SHA:
        raise fail("Phase82 metric authority hash changed")
    phase82_freeze = phase82.verify_freeze()
    output_root = output_root.resolve()
    if not output_root.is_dir():
        raise fail(f"Phase100 output root missing: {output_root}")
    phase43_seal, _phase43_seal_manifest = phase82._load_phase43_seal()
    accounting: dict[str, int] = {"truth_reads": 0, "summary_reads": 0, "accuracy_calculations": 0, "phase43_control_artifact_reads": 0, "phase43_seal_metadata_reads": 2}
    reports: dict[str, Any] = {}
    for route in ROUTES:
        reports[route] = _route_report(route, output_root, phase100_freeze, phase82_freeze, phase43_seal, accounting)
    scored = [reports[route] for route in ROUTES if reports[route].get("accuracy_scored") is True]
    candidate_macro: float | None = None
    control_macro: float | None = None
    macro_improvement: float | None = None
    if len(scored) == len(ROUTES):
        candidate_macro = sum(item["candidate"]["score_m"] for item in scored) / len(scored)
        control_macro = sum(item["control_phase43"]["score_m"] for item in scored) / len(scored)
        macro_improvement = control_macro - candidate_macro
    gates: dict[str, Any] = {
        "exact_two_routes_one_run_each": set(reports) == set(ROUTES) and all(reports[route].get("native", {}).get("return_code") == 0 for route in ROUTES),
        "candidate_prediction_domain_coverage_exact": len(scored) == 2 and all(item["candidate"]["prediction_domain_coverage"] == 1.0 for item in scored),
        "candidate_all_finite_and_earth_valid": len(scored) == 2 and all(item["candidate"]["finite"] is True for item in scored),
        "candidate_over_70_mps_count_zero": len(scored) == 2 and all(item["candidate"]["over_70_mps_count"] == 0 for item in scored),
        "candidate_improves_each_route_by_at_least_0_05m": len(scored) == 2 and all(item.get("improvement_vs_phase43_m", -math.inf) >= 0.05 for item in scored),
        "candidate_no_route_regression": len(scored) == 2 and all(item.get("improvement_vs_phase43_m", -math.inf) >= 0.0 for item in scored),
        "candidate_macro_improvement_at_least_0_10m": macro_improvement is not None and macro_improvement >= 0.1,
        "candidate_each_route_score_at_most_3m": len(scored) == 2 and all(item["candidate"]["score_m"] <= 3.0 for item in scored),
        "candidate_macro_score_at_most_2m": candidate_macro is not None and candidate_macro <= 2.0,
        "candidate_macro_score_at_most_0_782m": candidate_macro is not None and candidate_macro <= 0.782,
        "truth_read_only_by_evaluator": accounting["truth_reads"] == 2 and all(reports[route].get("truth_read") is True for route in ROUTES) if len(scored) == 2 else False,
        "no_solution_rows_in_result": True,
        "no_kaggle_submission": True,
    }
    all_passed = all(value is True for value in gates.values())
    failed = [key for key, value in gates.items() if value is not True]
    for route in ROUTES:
        failed.extend(f"{route}:{key}" for key, value in reports[route].get("gates", {}).get("checks", {}).items() if value is not True)
    result = {
        "schema_version": RESULT_SCHEMA,
        "phase": 100,
        "execution_label": "Luna Max",
        "status": "go-phase100-qr-accuracy-gates" if all_passed else "no-go-phase100-qr-accuracy-gates",
        "decision": "accuracy gate passed; release remains separately unauthorized" if all_passed else "accuracy gate failed closed; preserve artifacts and do not release",
        "candidate": CANDIDATE_ID,
        "routes": reports,
        "aggregate": {
            "candidate_macro_score_m": candidate_macro,
            "control_phase43_macro_score_m": control_macro,
            "macro_improvement_vs_phase43_m": macro_improvement,
            "route_count": len(ROUTES),
            "macro_route_order": list(ROUTES),
            "macro_weighting": "unweighted arithmetic mean over exactly MTV-A and LAX-T",
        },
        "metric_contract": phase82_freeze["metric_contract"],
        "promotion_gates": gates,
        "failed_gates": failed,
        "solution_output_published": False,
        "accuracy_scored": len(scored) == 2,
        "truth_evaluator_only": True,
        "release_or_submission_authorized": False,
        "authority": {
            "phase100_freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA},
            "phase100_manifest": {"path": relative(MANIFEST), "sha256": sha256_file(MANIFEST, "Phase100 manifest")},
            "phase100_authorization": {"path": relative(AUTHORIZATION), "sha256": sha256_file(AUTHORIZATION, "Phase100 authorization"), "status": authorization.get("status")},
            "phase100_evaluator": {"path": relative(EVALUATOR), "sha256": sha256_file(EVALUATOR, "Phase100 evaluator")},
            "phase100_wrapper": {"path": relative(WRAPPER), "sha256": sha256_file(WRAPPER, "Phase100 wrapper")},
            "phase82_freeze": {"path": relative(PHASE82_FREEZE), "sha256": PHASE82_FREEZE_SHA},
            "phase43_control_source": "Phase82 artifact_sources.phase43_control resolved through sealed Phase43 structural identity",
        },
        "read_accounting": {
            "native_solver_invocations": 2,
            "raw_device_gnss_reads_by_native": 2,
            "raw_device_imu_reads_by_native": 2,
            "broadcast_navigation_reads_by_native": 2,
            "raw_byte_reads_by_wrapper": 0,
            "raw_hash_reads_by_wrapper": 0,
            "truth_reads": accounting["truth_reads"],
            "truth_reads_per_route_max": 1,
            "truth_reads_by_process": "Phase100 evaluator subprocess only",
            "solution_reads_for_hash_and_parse": 2,
            "summary_reads": accounting["summary_reads"],
            "phase43_control_artifact_reads": accounting["phase43_control_artifact_reads"],
            "phase43_seal_metadata_reads": accounting["phase43_seal_metadata_reads"],
            "accuracy_calculations": accounting["accuracy_calculations"],
            "mat_reads_or_generated": 0,
            "base_reads": 0,
            "pdc_or_precomputed_coordinate_reads": 0,
            "kaggle_or_token_access": 0,
            "fallbacks": 0,
            "reruns": 0,
            "raw_content_copied_or_transformed": False,
        },
        "forbidden_lanes": {
            "truth_in_native_argv_or_environment": False,
            "mat": False,
            "base": False,
            "pdc": False,
            "precomputed_coordinates": False,
            "kaggle_or_token": False,
            "solution_rows_in_result": False,
        },
    }
    atomic_json(result_path, result)
    atomic_text(result_path.with_suffix(".md"), result_markdown(result))
    return result


def result_markdown(result: dict[str, Any]) -> str:
    aggregate = result.get("aggregate", {})
    lines = [
        "# Phase100 Phase99 QR accuracy result",
        "",
        f"- Status: `{result.get('status')}`",
        f"- Decision: {result.get('decision')}",
        "- Scope: MTV-A and LAX-T, exactly one fresh native Phase99 QR run each",
        "- Native inputs: raw device_gnss.csv, device_imu.csv, and broadcast brdc.nav only",
        "- Truth: evaluator subprocess only; solution rows are not published",
        "",
        "## Macro",
        "",
        f"- Candidate macro: `{aggregate.get('candidate_macro_score_m')}` m",
        f"- Phase43 control macro: `{aggregate.get('control_phase43_macro_score_m')}` m",
        f"- Improvement: `{aggregate.get('macro_improvement_vs_phase43_m')}` m",
        "",
        "## Routes",
        "",
        "| Route | Return | Candidate score | Control score | Improvement | Truth read | Gates |",
        "|---|---:|---:|---:|---:|---:|---|",
    ]
    for route in ROUTES:
        item = result.get("routes", {}).get(route, {})
        candidate = item.get("candidate", {})
        control = item.get("control_phase43", {})
        lines.append(
            f"| `{route}` | `{item.get('native', {}).get('return_code')}` | `{candidate.get('score_m')}` | `{control.get('score_m')}` | `{item.get('improvement_vs_phase43_m')}` | `{item.get('truth_read')}` | `{item.get('gates', {}).get('passed')}` |"
        )
    lines.extend(["", "No solution coordinate rows are included in this result. A GO does not authorize release, validation, or Kaggle submission.", ""])
    return "\n".join(lines)


def _write_fail_closed_result(result_path: Path, error: str) -> dict[str, Any]:
    result = {
        "schema_version": RESULT_SCHEMA,
        "phase": 100,
        "execution_label": "Luna Max",
        "status": "no-go-phase100-qr-accuracy-gates",
        "decision": "evaluator failed closed before complete scoring",
        "candidate": CANDIDATE_ID,
        "error": error,
        "solution_output_published": False,
        "accuracy_scored": False,
        "truth_evaluator_only": True,
        "release_or_submission_authorized": False,
        "read_accounting": {"truth_reads": 0, "native_solver_invocations": 0, "raw_byte_reads_by_wrapper": 0, "mat_reads_or_generated": 0, "kaggle_or_token_access": 0},
        "promotion_gates": {"all_gates_anded": False, "evaluator_completed": False},
        "failed_gates": ["evaluator_completed"],
    }
    atomic_json(result_path, result)
    atomic_text(result_path.with_suffix(".md"), result_markdown(result))
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-pre-raw", action="store_true")
    parser.add_argument("--evaluate", action="store_true")
    parser.add_argument("--output-root", type=Path, default=OUTPUT_ROOT)
    parser.add_argument("--result-json", type=Path, default=RESULT_JSON)
    args = parser.parse_args(argv)
    if args.verify_pre_raw == args.evaluate:
        parser.error("choose exactly one of --verify-pre-raw or --evaluate")
    try:
        if args.verify_pre_raw:
            print(json.dumps(verify_pre_raw(), indent=2, sort_keys=True))
            return 0
        result = evaluate(args.output_root, args.result_json)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["status"] == "go-phase100-qr-accuracy-gates" else 1
    except Exception as exc:
        if args.evaluate:
            try:
                _write_fail_closed_result(args.result_json, str(exc))
            except Exception:
                pass
        print(f"phase100 QR evaluator: fail-closed: {exc}", file=os.sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
