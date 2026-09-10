#!/usr/bin/env python3
"""Phase108 truth-only evaluator for the immutable Phase107 solutions.

Phase107 already completed the only native/raw/base runs.  This module never
launches native code and never opens a raw or base input.  Before authorization
it reads only sealed metadata and source.  In the authorized evaluator process
it opens each opaque candidate CSV once, performs the pinned schema/finite
preflight, then opens each official truth CSV exactly once and scores it with
the sealed Phase82 metric implementation.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
EVALUATOR = Path(__file__).resolve()
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase108_phase107_solution_accuracy_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase108_phase107_solution_accuracy_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase108_phase107_solution_accuracy_authorization_v1.json"
WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase108_phase107_solution_accuracy_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase108_phase107_solution_accuracy.py"
PHASE107_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase107_raw_base_source_parity_result_v1.json"
PHASE107_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase107_raw_base_source_parity_manifest_v1.json"
PHASE107_AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase107_raw_base_source_parity_authorization_v1.json"
PHASE82_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase82_phase80_direct_quality_accuracy.py"
PHASE82_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase82_phase80_direct_quality_accuracy_result_v1.json"
PHASE103_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase103_phase102_truth_only_evaluator_correction_result_v1.json"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase108_phase107_solution_accuracy_result_v1.json"

AUDIT_SHA = "adc77bfa634c5507eb5f16d7de803e0d972d8ff475f3bbc46aa6981bdf6b029e"
FREEZE_SHA = "e503dac7508f6b6466173c5f0c651a04fa9dc24222aa3ce5ae22332fbb16dde1"
PHASE107_RESULT_SHA = "e7eb3f1d2670672414744a100dc1e2fc3e0b019ae7efc0dbfef1518300ce8d6e"
PHASE107_MANIFEST_SHA = "b46a4bfa23ac1604d9e51487734be18262f027612eede40fdbcba15fa75c2a9c"
PHASE107_AUTHORIZATION_SHA = "ba975dd93f11cd64fdaec6cc6d57a080367fc2e6a04ee58513cb3a0b0542eb56"
PHASE82_EVALUATOR_SHA = "232ba0ab1fa7ebde6ddfaaf851d190d1ed32377ea056ab6c31e244139c2977fb"
PHASE82_RESULT_SHA = "39c5a38b7e3e61fda0306582fffb961dad3e6fb0bd49fcfe70e66674e2c90873"
PHASE103_RESULT_SHA = "1a909a9b91ea90517fac3f138faa363fd5a601e33e4361b2017e4873e8873688"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
TRUTH_ROWS = {ROUTES[0]: 2159, ROUTES[1]: 1465}
EXPECTED_MISSING = {ROUTES[0]: [ROUTES[0], 1615921153434], ROUTES[1]: None}
CANDIDATE_ID = "phase108-phase107-withheld-solution-truth-only-accuracy-v1"
MANIFEST_SCHEMA = "smartphone-r5-phase108-phase107-solution-accuracy-manifest.v1"
AUTHORIZATION_SCHEMA = "smartphone-r5-phase108-phase107-solution-accuracy-authorization.v1"
RESULT_SCHEMA = "smartphone-r5-phase108-phase107-solution-accuracy-result.v1"
EARTH_RADIUS_M = 6371008.8
MAX_SPEED_MPS = 70.0


class Phase108Error(ValueError):
    """Raised when the Phase108 contract fails closed."""


def fail(message: str) -> Phase108Error:
    return Phase108Error(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def sha256_file(path: Path, label: str) -> str:
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
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


def _safe_repo_path(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value or Path(value).is_absolute() or ".." in Path(value).parts:
        raise fail(f"unsafe {label} path")
    path = (ROOT / value).resolve()
    try:
        path.relative_to(ROOT)
    except ValueError as exc:
        raise fail(f"{label} path escapes repository") from exc
    return path


def _candidate_path(value: Any, route: str) -> Path:
    path = _safe_repo_path(value, f"candidate {route}")
    if path.name != "withheld_solution_output.csv" or not str(path).startswith(str(ROOT / "output/smartphone-r5/phase107-raw-base-source-parity-v1")):
        raise fail(f"candidate path is not the sealed Phase107 withheld output: {route}")
    return path


def _truth_path(value: Any, route: str) -> Path:
    path = _safe_repo_path(value, f"truth {route}")
    if path.name != "ground_truth.csv" or "/truth/" not in str(path):
        raise fail(f"truth path is not a pinned ground-truth file: {route}")
    return path


def _expected_metric() -> dict[str, Any]:
    return {
        "candidate_submission_header": ["phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"],
        "truth_reader": "CSV DictReader over required field names; optional columns allowed",
        "required_truth_fields": ["UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"],
        "truth_phone_policy": "optional phone must equal declared route; absent phone is assigned route",
        "key": "(phone, UnixTimeMillis)",
        "timestamp_type": "exact integer",
        "matching": "exact integer key intersection only",
        "duplicate_policy": "duplicate prediction or truth keys fail closed",
        "extra_prediction_policy": "prediction keys absent from truth fail closed",
        "prediction_domain_coverage": "matched prediction keys / prediction keys; required exactly 1.0",
        "missing_truth_policy": "only the exact pinned leading warm-up truth key may be absent; no interpolation, nearest, edge hold, extrapolation, or fill",
        "distance": "spherical Haversine per row",
        "earth_radius_m": EARTH_RADIUS_M,
        "percentile": "linear interpolation at rank (n - 1) * q",
        "route_scalar": "(P50 + P95) / 2 in metres",
        "macro": "unweighted arithmetic mean over exactly MTV-A then LAX-T",
        "continuity": "prediction Haversine transition speed; over_70_mps_count must equal 0",
        "finite_and_earth_valid": "all coordinates and derived metric values finite; latitude in [-90,90] and longitude in [-180,180]",
    }


def _finite_tree(value: Any) -> bool:
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, dict):
        return all(_finite_tree(child) for child in value.values())
    if isinstance(value, (list, tuple)):
        return all(_finite_tree(child) for child in value)
    return True


def _verify_pinned_json(path: Path, expected_sha: str, label: str) -> dict[str, Any]:
    assert_equal(sha256_file(path, label), expected_sha, f"{label} SHA-256")
    return read_json(path, label)


def verify_freeze() -> dict[str, Any]:
    freeze = _verify_pinned_json(FREEZE, FREEZE_SHA, "Phase108 freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase108-phase107-solution-accuracy-freeze.v1",
        "phase": 108,
        "status": "frozen-before-phase108-truth-only-evaluation",
        "execution_label": "Luna Max",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    authority = freeze.get("authority", {})
    audit = authority.get("audit", {})
    assert_equal(audit.get("sha256"), AUDIT_SHA, "audit pin")
    assert_equal(sha256_file(_safe_repo_path(audit.get("path"), "audit"), "Phase108 audit"), AUDIT_SHA, "audit hash")
    result = authority.get("phase107_result", {})
    assert_equal(result.get("sha256"), PHASE107_RESULT_SHA, "Phase107 result pin")
    sealed_result = _verify_pinned_json(PHASE107_RESULT, PHASE107_RESULT_SHA, "Phase107 result")
    for key, expected in (("status", "go-phase107-raw-base-structural"), ("accuracy_scored", False), ("solution_output_published", False)):
        assert_equal(sealed_result.get(key), expected, f"Phase107 result/{key}")
    assert_equal(sealed_result.get("read_accounting", {}).get("truth_reads"), 0, "Phase107 result/truth reads")
    for path, digest, label in (
        (PHASE107_MANIFEST, PHASE107_MANIFEST_SHA, "Phase107 manifest"),
        (PHASE107_AUTHORIZATION, PHASE107_AUTHORIZATION_SHA, "Phase107 authorization"),
        (PHASE82_EVALUATOR, PHASE82_EVALUATOR_SHA, "Phase82 evaluator"),
        (PHASE82_RESULT, PHASE82_RESULT_SHA, "Phase82 result"),
        (PHASE103_RESULT, PHASE103_RESULT_SHA, "Phase103 result"),
    ):
        if sha256_file(path, label) != digest:
            raise fail(f"{label} hash changed")
    candidate = freeze.get("candidate", {})
    assert_equal(candidate.get("id"), CANDIDATE_ID, "candidate id")
    assert_equal(candidate.get("candidate_count"), 1, "candidate count")
    assert_equal(candidate.get("routes"), list(ROUTES), "candidate route order")
    for key, expected in {
        "performed_before_truth": True,
        "opaque_bytes_only": True,
        "coordinate_interpretation": False,
        "csv_parse": False,
        "row_values_interpreted": False,
        "candidate_files_copied_or_transformed": False,
    }.items():
        assert_equal(candidate.get("metadata_hash_seal", {}).get(key), expected, f"candidate metadata/{key}")
    route_meta = candidate.get("routes_metadata", {})
    assert_equal(set(route_meta), set(ROUTES), "candidate route metadata set")
    for route in ROUTES:
        pin = route_meta[route]
        _candidate_path(pin.get("path"), route)
        if not isinstance(pin.get("sha256"), str) or len(pin["sha256"]) != 64 or isinstance(pin.get("bytes"), bool) or not isinstance(pin.get("bytes"), int) or pin["bytes"] <= 0:
            raise fail(f"candidate opaque seal malformed: {route}")
        assert_equal(pin.get("expected_prediction_rows"), DOMAIN_ROWS[route], f"candidate expected rows/{route}")
        assert_equal(pin.get("opaque_metadata_seal"), True, f"candidate opaque seal/{route}")
    truth = freeze.get("truth_cohort", {})
    assert_equal(truth.get("route_order"), list(ROUTES), "truth route order")
    for key, expected in (("read_by_solver", False), ("read_by_audit", False), ("read_by_freeze", False), ("read_by_evaluator_only", True)):
        assert_equal(truth.get(key), expected, f"truth cohort/{key}")
    truth_routes = truth.get("routes", {})
    assert_equal(set(truth_routes), set(ROUTES), "truth route set")
    for route in ROUTES:
        pin = truth_routes[route]
        _truth_path(pin.get("path"), route)
        assert_equal(pin.get("sha256"), {ROUTES[0]: "7c84ed6a80b1bbb08c0ffad57493513833b9d5474e22a43c5a44da82824ee22d", ROUTES[1]: "29e0861dd1ecb8865c10adab69396d98ed96618e8877b09d04aa8d671edf79e8"}[route], f"truth hash/{route}")
        assert_equal(pin.get("rows"), TRUTH_ROWS[route], f"truth rows/{route}")
        assert_equal(pin.get("expected_missing_truth_key"), EXPECTED_MISSING[route], f"truth missing key/{route}")
    assert_equal(freeze.get("metric_contract"), _expected_metric(), "metric contract")
    baselines = freeze.get("comparison_baselines", {})
    assert_equal(baselines.get("phase82_same_route", {}).get("route_scores_m"), {ROUTES[0]: 1.1139384500152307, ROUTES[1]: 0.9389644134001871}, "Phase82 baselines")
    assert_equal(baselines.get("phase103_no_base_c7", {}).get("route_scores_m"), {ROUTES[0]: 1.139793072101309, ROUTES[1]: 3.310820065300743}, "Phase103 baselines")
    assert_equal(baselines.get("phase82_same_route", {}).get("macro_m"), 1.0264514317077089, "Phase82 macro")
    assert_equal(baselines.get("phase103_no_base_c7", {}).get("macro_m"), 2.225306568701026, "Phase103 macro")
    gates = freeze.get("promotion_gates", {})
    for key, expected in {
        "all_gates_anded": True,
        "candidate_prediction_domain_coverage_exact": 1.0,
        "candidate_all_finite_and_earth_valid": True,
        "candidate_over_70_mps_count_zero": True,
        "candidate_route_score_max_m": 3.0,
        "candidate_macro_score_max_m": 2.0,
        "candidate_macro_score_strict_max_m": 0.782,
        "candidate_no_route_regression_vs_phase82_same_route": True,
        "candidate_no_route_regression_vs_phase103_no_base_c7": True,
        "strict_0_782_is_explicit_and_gate": True,
        "truth_read_only_by_one_evaluator_subprocess": True,
        "solution_rows_absent_from_result": True,
        "no_solver_raw_or_base_rerun": True,
        "no_mat_precomputed_phone_coordinates_pdc_or_kaggle": True,
    }.items():
        assert_equal(gates.get(key), expected, f"promotion gate/{key}")
    accounting = freeze.get("read_accounting", {})
    audit_accounting = accounting.get("audit", {})
    for key in ("native_solver_invocations", "raw_gnss_imu_navigation_reads", "base_rinex_reads", "truth_reads", "candidate_coordinate_interpretations", "mat_precomputed_phone_coordinate_pdc_reads", "accuracy_calculations", "reruns", "fallbacks", "kaggle_or_token_access"):
        assert_equal(audit_accounting.get(key), 0, f"audit accounting/{key}")
    return freeze


def verify_manifest() -> dict[str, Any]:
    freeze = verify_freeze()
    manifest = read_json(MANIFEST, "Phase108 manifest")
    for key, expected in {"schema_version": MANIFEST_SCHEMA, "phase": 108, "status": "sealed-before-phase108-truth-only-evaluation", "execution_label": "Luna Max"}.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    authority = manifest.get("authority", {})
    assert_equal(authority.get("freeze", {}).get("path"), relative(FREEZE), "manifest freeze path")
    assert_equal(authority.get("freeze", {}).get("sha256"), FREEZE_SHA, "manifest freeze hash")
    for label, path in (("evaluator", EVALUATOR), ("wrapper", WRAPPER), ("focused_tests", FOCUSED_TESTS)):
        pin = authority.get(label, {})
        assert_equal(pin.get("path"), relative(path), f"manifest {label} path")
        assert_equal(pin.get("sha256"), sha256_file(path, f"Phase108 {label}"), f"manifest {label} hash")
    candidate = manifest.get("candidate", {})
    assert_equal(candidate.get("id"), CANDIDATE_ID, "manifest candidate id")
    assert_equal(candidate.get("candidate_count"), 1, "manifest candidate count")
    assert_equal(candidate.get("routes"), list(ROUTES), "manifest candidate routes")
    assert_equal(candidate.get("metadata_hash_seal_precedes_truth"), True, "manifest candidate seal boundary")
    assert_equal(candidate.get("solution_publication"), False, "manifest solution publication")
    assert_equal(candidate.get("route_metadata"), freeze["candidate"]["routes_metadata"], "manifest candidate route metadata")
    truth_cohort = manifest.get("truth_cohort", {})
    assert_equal(truth_cohort.get("route_order"), list(ROUTES), "manifest truth route order")
    assert_equal(truth_cohort.get("routes"), freeze["truth_cohort"]["routes"], "manifest truth pins")
    for key, expected in (("read_by_solver", False), ("read_by_manifest", False), ("read_by_evaluator_only", True)):
        assert_equal(truth_cohort.get(key), expected, f"manifest truth cohort/{key}")
    assert_equal(manifest.get("metric_contract_source"), {"freeze_path": relative(FREEZE), "freeze_sha256": FREEZE_SHA, "phase82_evaluator_sha256": PHASE82_EVALUATOR_SHA}, "manifest metric source")
    planned = manifest.get("read_accounting", {}).get("planned_truth_evaluation", {})
    for key, expected in (("native_solver_invocations", 0), ("raw_gnss_imu_navigation_reads", 0), ("base_rinex_reads", 0), ("truth_reads", 2), ("candidate_solution_reads_for_hash_and_parse", 2), ("accuracy_calculations", 2), ("reruns", 0), ("fallbacks", 0), ("mat_precomputed_phone_coordinate_pdc_reads", 0), ("kaggle_or_token_access", 0)):
        assert_equal(planned.get(key), expected, f"manifest planned accounting/{key}")
    boundary = manifest.get("truth_evaluator_boundary", {})
    for key, expected in (("evaluator_process_count", 1), ("candidate_metadata_hash_seal_precedes_truth", True), ("candidate_parse_precedes_truth", True), ("truth_reads", 2), ("truth_reads_per_route", 1), ("native_after_truth", False), ("result_content", "metrics, gate booleans, sealed hashes, and accounting only; no solution or truth coordinate rows")):
        assert_equal(boundary.get(key), expected, f"manifest boundary/{key}")
    return manifest


def verify_pre_truth() -> dict[str, Any]:
    verify_manifest()
    return {
        "status": "pre-truth-verified",
        "native_solver_invocations": 0,
        "raw_reads": 0,
        "base_reads": 0,
        "truth_reads": 0,
        "candidate_coordinate_interpretations": 0,
        "accuracy_calculations": 0,
        "mat_precomputed_phone_coordinate_pdc_reads": 0,
        "kaggle_or_token_access": 0,
        "native_rerun": False,
    }


def verify_authorization() -> dict[str, Any]:
    manifest = verify_manifest()
    auth = read_json(AUTHORIZATION, "Phase108 authorization")
    for key, expected in {"schema_version": AUTHORIZATION_SCHEMA, "phase": 108, "execution_label": "Luna Max", "status": "authorized-for-phase108-truth-only-accuracy-evaluation"}.items():
        assert_equal(auth.get(key), expected, f"authorization/{key}")
    authority = auth.get("authority", {})
    pins = {
        "freeze": (FREEZE, FREEZE_SHA),
        "manifest": (MANIFEST, sha256_file(MANIFEST, "Phase108 manifest")),
        "evaluator": (EVALUATOR, sha256_file(EVALUATOR, "Phase108 evaluator")),
        "wrapper": (WRAPPER, sha256_file(WRAPPER, "Phase108 wrapper")),
        "focused_tests": (FOCUSED_TESTS, sha256_file(FOCUSED_TESTS, "Phase108 focused tests")),
        "phase107_result": (PHASE107_RESULT, PHASE107_RESULT_SHA),
        "phase107_manifest": (PHASE107_MANIFEST, PHASE107_MANIFEST_SHA),
        "phase107_authorization": (PHASE107_AUTHORIZATION, PHASE107_AUTHORIZATION_SHA),
    }
    for label, (path, digest) in pins.items():
        pin = authority.get(label, {})
        assert_equal(pin.get("path"), relative(path), f"authorization/{label} path")
        assert_equal(pin.get("sha256"), digest, f"authorization/{label} hash")
    assert_equal(authority.get("routes"), list(ROUTES), "authorization routes")
    candidate = auth.get("candidate", {})
    for key, expected in (("id", CANDIDATE_ID), ("candidate_count", 1), ("solver_rerun", False), ("raw_or_base_rerun", False), ("truth_reads", 2), ("truth_reads_per_route", 1), ("solution_publication", False), ("no_fallback_or_rerun", True)):
        assert_equal(candidate.get(key), expected, f"authorization candidate/{key}")
    policy = auth.get("execution_policy", {})
    for key, expected in (("truth_only_evaluation_authorized", True), ("native_solver_invocations", 0), ("raw_reads", 0), ("base_reads", 0), ("truth_reads_per_route", 1), ("native_after_truth", False), ("solution_publication", False), ("mat_precomputed_phone_coordinate_pdc_reads", 0), ("kaggle_or_token_access", 0)):
        assert_equal(policy.get(key), expected, f"authorization policy/{key}")
    assert_equal(manifest.get("authority", {}).get("freeze", {}).get("sha256"), FREEZE_SHA, "authorization manifest identity")
    return auth


def _import_phase82() -> Any:
    assert_equal(sha256_file(PHASE82_EVALUATOR, "Phase82 evaluator"), PHASE82_EVALUATOR_SHA, "Phase82 evaluator hash")
    spec = importlib.util.spec_from_file_location("phase82_metric_reference_phase108", PHASE82_EVALUATOR)
    if spec is None or spec.loader is None:
        raise fail("unable to load Phase82 metric reference")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _read_candidate_once(path: Path, pin: dict[str, Any], route: str, p82: Any) -> tuple[list[tuple[int, float, float]], dict[int, tuple[float, float]], dict[str, Any]]:
    try:
        with path.open("rb") as handle:
            payload = handle.read()
    except OSError as exc:
        raise fail(f"failed to read candidate {route}: {exc}") from exc
    digest = hashlib.sha256(payload).hexdigest()
    if len(payload) != pin.get("bytes") or digest != pin.get("sha256"):
        raise fail(f"candidate opaque seal mismatch: {route}")
    try:
        ordered, mapping = p82.P76.P74._parse_submission(payload, route)
    except Exception as exc:
        raise fail(f"candidate schema preflight failed: {route}: {exc}") from exc
    if len(ordered) != DOMAIN_ROWS[route] or [item[0] for item in ordered] != sorted(item[0] for item in ordered) or len(mapping) != len(ordered):
        raise fail(f"candidate exact timestamp alignment failed: {route}")
    if any(not all(math.isfinite(float(value)) for value in item[1:]) or not -90.0 <= item[1] <= 90.0 or not -180.0 <= item[2] <= 180.0 for item in ordered):
        raise fail(f"candidate finite/Earth-valid preflight failed: {route}")
    return ordered, mapping, {"path": relative(path), "bytes": len(payload), "sha256": digest, "header": ["phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"], "rows": len(ordered), "read_count": 1, "coordinate_rows_omitted": True}


def _read_truth_once(path: Path, pin: dict[str, Any], route: str, p82: Any) -> tuple[dict[int, tuple[float, float]], dict[str, Any]]:
    try:
        with path.open("rb") as handle:
            payload = handle.read()
    except OSError as exc:
        raise fail(f"failed to read official truth {route}: {exc}") from exc
    digest = hashlib.sha256(payload).hexdigest()
    if len(payload) != pin.get("bytes") or digest != pin.get("sha256"):
        raise fail(f"truth pin mismatch: {route}")
    try:
        truth = p82.P76._parse_truth_dictreader(payload, route)
    except Exception as exc:
        raise fail(f"truth schema failed: {route}: {exc}") from exc
    if len(truth) != pin.get("rows"):
        raise fail(f"truth row count mismatch: {route}")
    return truth, {"path": relative(path), "bytes": len(payload), "sha256": digest, "rows": len(truth), "read_count": 1, "coordinate_rows_omitted": True}


def _sealed_baselines(freeze: dict[str, Any]) -> dict[str, dict[str, float]]:
    phase82 = _verify_pinned_json(PHASE82_RESULT, PHASE82_RESULT_SHA, "Phase82 result")
    phase103 = _verify_pinned_json(PHASE103_RESULT, PHASE103_RESULT_SHA, "Phase103 result")
    values = {
        "phase82": {route: float(phase82["routes"][route]["candidate"]["score_m"]) for route in ROUTES},
        "phase103": {route: float(phase103["routes"][route]["candidate"]["score_m"]) for route in ROUTES},
    }
    expected = freeze["comparison_baselines"]
    assert_equal(values["phase82"], expected["phase82_same_route"]["route_scores_m"], "sealed Phase82 baseline values")
    assert_equal(values["phase103"], expected["phase103_no_base_c7"]["route_scores_m"], "sealed Phase103 baseline values")
    return values


def _route_report(route: str, freeze: dict[str, Any], p82: Any, baselines: dict[str, dict[str, float]], accounting: dict[str, int]) -> dict[str, Any]:
    report: dict[str, Any] = {"dataset_id": route, "truth_read": False, "accuracy_scored": False}
    try:
        candidate_pin = freeze["candidate"]["routes_metadata"][route]
        candidate_path = _candidate_path(candidate_pin["path"], route)
        ordered, candidate, candidate_meta = _read_candidate_once(candidate_path, candidate_pin, route, p82)
        accounting["candidate_solution_reads"] += 1
        report["candidate_solution"] = candidate_meta
        truth_pin = freeze["truth_cohort"]["routes"][route]
        truth_path = _truth_path(truth_pin["path"], route)
        accounting["truth_reads"] += 1
        truth, truth_meta = _read_truth_once(truth_path, truth_pin, route, p82)
        report["truth"] = truth_meta
        report["truth_read"] = True
        score = p82.P76._score_prediction(candidate, truth, EXPECTED_MISSING[route], route, ordered)
        accounting["accuracy_calculations"] += 1
        phase82_score = baselines["phase82"][route]
        phase103_score = baselines["phase103"][route]
        checks = {
            "candidate_schema_and_exact_alignment": len(ordered) == DOMAIN_ROWS[route] and len(candidate) == DOMAIN_ROWS[route],
            "candidate_prediction_domain_coverage_exact": score.get("prediction_domain_coverage") == 1.0,
            "candidate_finite_and_earth_valid": score.get("finite") is True,
            "candidate_over_70_mps_count_zero": score.get("over_70_mps_count") == 0,
            "candidate_route_score_at_most_3m": score.get("score_m", math.inf) <= 3.0,
            "candidate_no_route_regression_vs_phase82": score.get("score_m", math.inf) <= phase82_score,
            "candidate_no_route_regression_vs_phase103_no_base_c7": score.get("score_m", math.inf) <= phase103_score,
        }
        report.update({
            "accuracy_scored": True,
            "candidate": score,
            "baseline_phase82_same_route": {"score_m": phase82_score},
            "baseline_phase103_no_base_c7": {"score_m": phase103_score},
            "gates": {"passed": all(checks.values()), "checks": checks, "failures": [key for key, value in checks.items() if value is not True]},
        })
    except Exception as exc:
        report["failure"] = str(exc)
        report.setdefault("gates", {"passed": False, "checks": {}, "failures": ["route_evaluation"]})
    return report


def _result_markdown(result: dict[str, Any]) -> str:
    aggregate = result.get("aggregate", {})
    lines = [
        "# Phase108 Phase107 withheld-solution truth-only accuracy result",
        "",
        f"- Status: `{result.get('status')}`",
        f"- Decision: {result.get('decision')}",
        "- Native/raw/base processing: not rerun; existing Phase107 outputs reused",
        "- Truth: one official file read per route by this evaluator subprocess",
        "- Candidate/truth coordinate rows: omitted from this result",
        "",
        "## Aggregate",
        "",
        f"- Candidate macro: `{aggregate.get('candidate_macro_score_m')}` m",
        f"- Phase82 same-route macro: `{aggregate.get('phase82_same_route_macro_m')}` m",
        f"- Phase103 no-base C7 macro: `{aggregate.get('phase103_no_base_c7_macro_m')}` m",
        f"- Strict `0.782 m` gate: `{result.get('strict_0_782_gate', {}).get('passed')}`",
        "",
        "## Routes",
        "",
        "| Route | Candidate (m) | Phase82 (m) | Phase103 C7 (m) | Truth read | Gates |",
        "|---|---:|---:|---:|---:|---|",
    ]
    for route in ROUTES:
        item = result.get("routes", {}).get(route, {})
        lines.append(f"| `{route}` | `{item.get('candidate', {}).get('score_m')}` | `{item.get('baseline_phase82_same_route', {}).get('score_m')}` | `{item.get('baseline_phase103_no_base_c7', {}).get('score_m')}` | `{item.get('truth_read')}` | `{item.get('gates', {}).get('passed')}` |")
    lines.extend(["", "Failure is fail-closed. GO, if any, does not authorize release, validation, or Kaggle submission.", ""])
    return "\n".join(lines)


def evaluate(result_path: Path = RESULT_JSON) -> dict[str, Any]:
    freeze = verify_freeze()
    verify_manifest()
    verify_authorization()
    p82 = _import_phase82()
    baselines = _sealed_baselines(freeze)
    accounting: dict[str, int] = {"candidate_solution_reads": 0, "truth_reads": 0, "accuracy_calculations": 0}
    reports = {route: _route_report(route, freeze, p82, baselines, accounting) for route in ROUTES}
    scored = [reports[route] for route in ROUTES if reports[route].get("accuracy_scored") is True]
    candidate_macro = sum(item["candidate"]["score_m"] for item in scored) / 2.0 if len(scored) == 2 else None
    phase82_macro = sum(baselines["phase82"].values()) / 2.0
    phase103_macro = sum(baselines["phase103"].values()) / 2.0
    gates: dict[str, Any] = {
        "exact_two_routes_one_evaluation_each": len(reports) == 2 and len(scored) == 2 and accounting["truth_reads"] == 2,
        "candidate_output_schema_and_exact_alignment": len(scored) == 2 and all(report["gates"]["checks"].get("candidate_schema_and_exact_alignment") is True for report in scored),
        "candidate_prediction_domain_coverage_exact": len(scored) == 2 and all(report["candidate"].get("prediction_domain_coverage") == 1.0 for report in scored),
        "candidate_all_finite_and_earth_valid": len(scored) == 2 and all(report["candidate"].get("finite") is True for report in scored),
        "candidate_over_70_mps_count_zero": len(scored) == 2 and all(report["candidate"].get("over_70_mps_count") == 0 for report in scored),
        "candidate_each_route_score_at_most_3m": len(scored) == 2 and all(report["candidate"]["score_m"] <= 3.0 for report in scored),
        "candidate_no_route_regression_vs_phase82_same_route": len(scored) == 2 and all(report["candidate"]["score_m"] <= baselines["phase82"][route] for route, report in reports.items() if report.get("accuracy_scored")),
        "candidate_no_route_regression_vs_phase103_no_base_c7": len(scored) == 2 and all(report["candidate"]["score_m"] <= baselines["phase103"][route] for route, report in reports.items() if report.get("accuracy_scored")),
        "candidate_macro_score_at_most_2m": candidate_macro is not None and candidate_macro <= 2.0,
        "candidate_macro_score_strict_at_most_0_782m": candidate_macro is not None and candidate_macro <= 0.782,
        "truth_read_only_by_one_evaluator_subprocess": accounting["truth_reads"] == 2 and all(reports[route].get("truth_read") is True for route in ROUTES),
        "no_solution_rows_in_result": True,
        "no_solver_raw_base_mat_precomputed_pdc_kaggle": True,
    }
    failed = [key for key, value in gates.items() if value is not True]
    result = {
        "schema_version": RESULT_SCHEMA,
        "phase": 108,
        "execution_label": "Luna Max",
        "status": "go-phase108-phase107-truth-only-accuracy-gates" if all(gates.values()) else "no-go-phase108-phase107-truth-only-accuracy-gates",
        "decision": "truth-only accuracy gate passed; release remains separately unauthorized" if all(gates.values()) else "truth-only accuracy gate failed closed; preserve artifacts and do not rerun",
        "candidate": CANDIDATE_ID,
        "accuracy_scored": len(scored) == 2,
        "routes": reports,
        "aggregate": {"candidate_macro_score_m": candidate_macro, "phase82_same_route_macro_m": phase82_macro, "phase103_no_base_c7_macro_m": phase103_macro, "route_count": len(ROUTES), "macro_route_order": list(ROUTES), "macro_weighting": "unweighted arithmetic mean over exactly MTV-A then LAX-T"},
        "comparison_baselines": {"phase82_same_route": baselines["phase82"], "phase103_no_base_c7": baselines["phase103"]},
        "metric_contract": freeze["metric_contract"],
        "promotion_gates": gates,
        "strict_0_782_gate": {"threshold_m": 0.782, "candidate_macro_score_m": candidate_macro, "passed": gates["candidate_macro_score_strict_at_most_0_782m"]},
        "failed_gates": failed,
        "read_accounting": {"native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0, "base_rinex_reads": 0, "candidate_solution_reads_for_hash_and_parse": accounting["candidate_solution_reads"], "truth_reads": accounting["truth_reads"], "truth_reads_per_route": 1, "accuracy_calculations": accounting["accuracy_calculations"], "reruns": 0, "fallbacks": 0, "mat_precomputed_phone_coordinate_pdc_reads": 0, "kaggle_or_token_access": 0, "truth_reads_by_process": "Phase108 truth-only evaluator subprocess"},
        "forbidden_lanes": {"native_solver_rerun": False, "raw_or_base_rerun": False, "mat": False, "precomputed_phone_coordinates": False, "pdc": False, "kaggle_or_token": False, "solution_rows_in_result": False},
        "solution_output_published": False,
        "release_or_submission_authorized": False,
        "authority": {"phase108_freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA}, "phase108_manifest": {"path": relative(MANIFEST), "sha256": sha256_file(MANIFEST, "Phase108 manifest")}, "phase108_authorization": {"path": relative(AUTHORIZATION), "sha256": sha256_file(AUTHORIZATION, "Phase108 authorization")}, "phase107_result": {"path": relative(PHASE107_RESULT), "sha256": PHASE107_RESULT_SHA}, "evaluator": {"path": relative(EVALUATOR), "sha256": sha256_file(EVALUATOR, "Phase108 evaluator")}},
    }
    atomic_json(result_path, result)
    atomic_text(result_path.with_suffix(".md"), _result_markdown(result))
    return result


def _write_fail_closed_result(error: str) -> dict[str, Any]:
    result = {"schema_version": RESULT_SCHEMA, "phase": 108, "execution_label": "Luna Max", "status": "no-go-phase108-phase107-truth-only-accuracy-gates", "decision": "truth-only evaluator failed closed; preserve artifacts and do not rerun", "candidate": CANDIDATE_ID, "error": error, "accuracy_scored": False, "strict_0_782_gate": {"threshold_m": 0.782, "passed": False}, "read_accounting": {"native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0, "base_rinex_reads": 0, "truth_reads": 0, "reruns": 0, "fallbacks": 0, "mat_precomputed_phone_coordinate_pdc_reads": 0, "kaggle_or_token_access": 0}, "forbidden_lanes": {"native_solver_rerun": False, "raw_or_base_rerun": False, "mat": False, "precomputed_phone_coordinates": False, "pdc": False, "kaggle_or_token": False, "solution_rows_in_result": False}, "solution_output_published": False, "release_or_submission_authorized": False}
    atomic_json(RESULT_JSON, result)
    atomic_text(RESULT_JSON.with_suffix(".md"), _result_markdown(result))
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-pre-truth", action="store_true")
    parser.add_argument("--evaluate", action="store_true")
    parser.add_argument("--result-json", type=Path, default=RESULT_JSON)
    args = parser.parse_args(argv)
    if args.verify_pre_truth == args.evaluate:
        parser.error("choose exactly one of --verify-pre-truth or --evaluate")
    try:
        if args.verify_pre_truth:
            print(json.dumps(verify_pre_truth(), indent=2, sort_keys=True))
            return 0
        result = evaluate(args.result_json)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result.get("status", "").startswith("go-") else 1
    except Exception as exc:
        if args.evaluate:
            try:
                _write_fail_closed_result(str(exc))
            except Exception:
                pass
        print(f"phase108 truth-only evaluator: fail-closed: {exc}", file=os.sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
