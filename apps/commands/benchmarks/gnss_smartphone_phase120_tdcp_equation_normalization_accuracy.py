#!/usr/bin/env python3
"""Launch-free contract and isolated truth-only evaluator for Phase120.

The launch-free modes inspect source and sealed metadata only.  They never
probe or open a Phase120 candidate file or an official truth file.  The
optional ``--evaluate`` mode is intentionally gated by a future independent
truth-only authorization and is the only path that materializes two candidate
and two truth payloads.  It delegates parsing and scoring to the pinned
Phase118/Phase82/Phase76 implementation without changing metric semantics.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
EVALUATOR = Path(__file__).resolve()
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase120_tdcp_equation_normalization_accuracy_freeze_v1.json"
AUDIT = ROOT / "docs/use_cases/records/smartphone_r5_phase120_tdcp_equation_normalization_accuracy_audit_v1.md"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase120_tdcp_equation_normalization_accuracy_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase120_tdcp_equation_normalization_accuracy_truth_authorization_v1.json"
STRUCTURAL_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase120_tdcp_equation_normalization_structural_result_v1.json"
PHASE118_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase118_tdcp_robust_k_accuracy.py"
PHASE118_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_accuracy_manifest_v1.json"
PHASE118_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_accuracy_gate_freeze_v1.json"
PHASE118_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_accuracy_result_v1.json"
PHASE82_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase82_phase80_direct_quality_accuracy.py"
PHASE82_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase82_phase80_direct_quality_accuracy_result_v1.json"
PHASE76_PARSER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase76_phase75_accuracy_integrity_recovery.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase120_tdcp_equation_normalization_accuracy.py"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase120_tdcp_equation_normalization_accuracy_result_v1.json"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
TRUTH_ROWS = {ROUTES[0]: 2159, ROUTES[1]: 1465}
EXPECTED_MISSING = {ROUTES[0]: [ROUTES[0], 1615921153434], ROUTES[1]: None}
EARTH_RADIUS_M = 6371008.8
MAX_SPEED_MPS = 70.0
STRICT_PROMOTION_THRESHOLD_M = 0.782
CANDIDATE_ID = "phase120-official-tdcp-resl-atmosphere-cancellation-truth-only-accuracy-v1"
RESULT_SCHEMA = "smartphone-r5-phase120-tdcp-equation-normalization-accuracy-result.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase120-tdcp-equation-normalization-accuracy-manifest.v1"
AUTHORIZATION_SCHEMA = "smartphone-r5-phase120-tdcp-equation-normalization-accuracy-truth-authorization.v1"

FREEZE_COMMIT = "e7e11eeb344d6cbb133e26e636b268ef25b5c956"
FREEZE_SHA256 = "f9b3746477a107d849cc3e957382d340c2a26d1677bf01edf879aef294bb4f2f"
AUDIT_COMMIT = "dda00518fa9e12a87239ff6fa174eb01e065771f"
AUDIT_SHA256 = "62b076eaef8977244521fb3da2f51df7e25745edb6ce205334c12cf06880e13b"
STRUCTURAL_COMMIT = "6a050b1041c29a484aa791a39a0edcd533640f94"
STRUCTURAL_SHA256 = "4c783ed063cc4f1bb3530826a40607eab6d8aa3c9716a8a8bedc8abebcdb48f5"
PHASE118_EVALUATOR_COMMIT = "c29e1700922260a77f047db8c82692f9d2b13603"
PHASE118_EVALUATOR_SHA256 = "a1b2fac0a5615e47a11e7e4b171ced77d12818b66c598a531a4c48d2bf7ff829"
PHASE118_MANIFEST_SHA256 = "b33b276c32bbbb5e2a8778a9259cd9e8daf92d874e67aa01dc27a20fb46b186b"
PHASE118_FREEZE_SHA256 = "860a7f382199958f8dd2c43ba697d4c5cde1bc550d5747028e4c6f73be346760"
PHASE118_RESULT_SHA256 = "918597aab19f462a2aadfff606f461ce47f48f2fdcd997e5fdeb89ab459a0164"
PHASE82_EVALUATOR_COMMIT = "be45879297a2a00a8574c77b71be0a5b721ff8c9"
PHASE82_EVALUATOR_SHA256 = "232ba0ab1fa7ebde6ddfaaf851d190d1ed32377ea056ab6c31e244139c2977fb"
PHASE82_RESULT_SHA256 = "39c5a38b7e3e61fda0306582fffb961dad3e6fb0bd49fcfe70e66674e2c90873"
PHASE76_PARSER_COMMIT = "7a8e77ce03f6f4354cf2f541098a53628faa23d5"
PHASE76_PARSER_SHA256 = "513e5970ee58846963f077b9960944e5fbf5f3535e5be24ca6e72493785f7761"
PHASE112_RESULT_SHA256 = "46d8d836758ac1a88b95ec2c1a0c0080cedfb753c12411ed74e4c2501549c57e"


class Phase120AccuracyError(ValueError):
    """Raised when the immutable Phase120 truth-only contract fails closed."""


def fail(message: str) -> Phase120AccuracyError:
    return Phase120AccuracyError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def _payload_path(path: Path) -> bool:
    return path.name in {"ground_truth.csv", "withheld_solution_output.csv", "device_gnss.csv", "device_imu.csv", "brdc.nav", "base.obs"}


def sha256_file(path: Path, label: str) -> str:
    """Hash source/sealed metadata only; payload guard precedes filesystem probe."""
    if _payload_path(path):
        raise fail(f"payload hash forbidden before truth authorization: {label}")
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
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


def _relative_metadata(path_text: Any, basename: str, label: str, fragment: str | None = None) -> str:
    """Validate metadata path syntax without resolving or probing the path."""
    if not isinstance(path_text, str) or not path_text:
        raise fail(f"missing {label} path metadata")
    path = Path(path_text)
    if path.is_absolute() or ".." in path.parts or path.name != basename:
        raise fail(f"unsafe {label} path metadata: {path_text}")
    if fragment is not None and fragment not in path_text:
        raise fail(f"{label} path outside pinned root: {path_text}")
    return path_text


def _pin(container: dict[str, Any], key: str, path: Path, expected: str, label: str | None = None) -> None:
    item = container.get(key)
    name = label or key
    if not isinstance(item, dict):
        raise fail(f"missing authority pin: {name}")
    assert_equal(item.get("path"), relative(path), f"authority/{name}/path")
    assert_equal(item.get("sha256"), expected, f"authority/{name}/sha256")


def metric_contract() -> dict[str, Any]:
    return {
        "source": "Phase102/103/112 truth-only contract and pinned Phase82/Phase76 scorer",
        "candidate_submission_header": ["phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"],
        "truth_reader": "CSV DictReader over required field names; optional columns allowed",
        "required_truth_fields": ["UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"],
        "truth_phone_policy": "optional phone must equal declared route; absent phone is assigned the declared route",
        "key": "(phone, UnixTimeMillis)",
        "timestamp_type": "exact integer",
        "matching": "exact integer key intersection only",
        "duplicate_policy": "duplicate prediction or truth keys fail closed",
        "extra_prediction_policy": "prediction keys absent from truth fail closed",
        "prediction_domain_coverage": "matched prediction keys / prediction keys; required exactly 1.0",
        "truth_row_coverage": "informational only; pinned warm-up exclusion is enforced",
        "missing_truth_policy": "only the exact pinned leading warm-up truth key may be absent; no interpolation, nearest, edge hold, extrapolation, or fill",
        "distance": "spherical Haversine per row",
        "earth_radius_m": EARTH_RADIUS_M,
        "percentile": "linear interpolation at rank (n - 1) * q",
        "route_scalar": "(P50 + P95) / 2 in metres",
        "macro": "unweighted arithmetic mean over exactly MTV-A then LAX-T",
        "continuity": "prediction Haversine transition speed; over_70_mps_count must equal 0",
        "finite_and_earth_valid": "all candidate coordinates and derived metric values finite; latitude in [-90,90] and longitude in [-180,180]",
        "strict_promotion_comparator": "candidate_macro_score_m < 0.782",
        "strict_promotion_threshold_m": STRICT_PROMOTION_THRESHOLD_M,
        "equality_at_threshold": "fails; strict less-than is required",
    }


def _verify_metric_contract(metric: Any, label: str) -> None:
    assert_equal(metric, metric_contract(), label)


def _verify_zero_accounting(accounting: Any, label: str) -> None:
    if not isinstance(accounting, dict):
        raise fail(f"{label} read accounting missing")
    for key in (
        "native_solver_invocations", "raw_gnss_imu_navigation_reads", "raw_base_rinex_reads",
        "truth_reads", "candidate_solution_payload_reads", "candidate_coordinate_interpretations",
        "accuracy_calculations", "mat_precomputed_phone_coordinate_pdc_reads", "kaggle_or_token_access",
        "reruns", "fallbacks",
    ):
        assert_equal(accounting.get(key), 0, f"{label}/{key}")


def verify_structural_result() -> dict[str, Any]:
    assert_equal(sha256_file(STRUCTURAL_RESULT, "Phase120 structural result"), STRUCTURAL_SHA256, "structural/sha256")
    result = read_json(STRUCTURAL_RESULT, "Phase120 structural result")
    for key, expected in {
        "schema_version": "smartphone-r5-phase120-tdcp-equation-normalization-structural-result.v1",
        "phase": 120,
        "status": "go-phase120-official-tdcp-resl-atmosphere-cancellation-structural",
        "accuracy_scored": False,
        "solution_output_published": False,
        "truth_free": True,
        "promotion_authorized": False,
    }.items():
        assert_equal(result.get(key), expected, f"structural/{key}")
    assert_equal(result.get("read_accounting", {}).get("truth_reads"), 0, "structural/truth_reads")
    gates = result.get("gates")
    if not isinstance(gates, dict) or gates.get("all_structural_gates_passed") is not True or gates.get("route_order_exact") is not True:
        raise fail("Phase120 structural result is not structural GO")
    routes = result.get("routes")
    if not isinstance(routes, dict) or list(routes) != list(ROUTES):
        raise fail("Phase120 structural route set/order changed")
    for route in ROUTES:
        item = routes[route]
        if not isinstance(item, dict) or item.get("return_code") != 0 or item.get("truth_used") is not False:
            raise fail(f"Phase120 structural route policy changed: {route}")
        if item.get("accuracy_scored") is not False or item.get("solution_output_published") is not False:
            raise fail(f"Phase120 structural route was published/scored: {route}")
        seals = item.get("solution_hash_seal")
        if not isinstance(seals, dict) or seals.get("published") is not False or seals.get("coordinate_rows_omitted") is not True:
            raise fail(f"Phase120 opaque candidate seal missing: {route}")
        _relative_metadata(seals.get("path"), "withheld_solution_output.csv", f"candidate {route}", "/phase120-tdcp-equation-normalization-v1/")
        if seals.get("rows") != DOMAIN_ROWS[route] or seals.get("row_count_matches_domain") is not True:
            raise fail(f"Phase120 candidate row seal mismatch: {route}")
        if not isinstance(seals.get("sha256"), str) or len(seals["sha256"]) != 64 or not isinstance(seals.get("bytes"), int):
            raise fail(f"Phase120 candidate opaque digest malformed: {route}")
    return result


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_file(FREEZE, "Phase120 accuracy freeze"), FREEZE_SHA256, "freeze/sha256")
    freeze = read_json(FREEZE, "Phase120 accuracy freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase120-tdcp-equation-normalization-accuracy-freeze.v1",
        "phase": 120,
        "execution_label": "Luna Max",
        "status": "frozen-before-phase120-truth-only-accuracy-evaluation",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    authority = freeze.get("authority")
    if not isinstance(authority, dict):
        raise fail("freeze/authority missing")
    _pin(authority, "audit", AUDIT, AUDIT_SHA256)
    assert_equal(sha256_file(AUDIT, "Phase120 accuracy audit"), AUDIT_SHA256, "audit/sha256")
    structural = authority.get("phase120_structural_result")
    if not isinstance(structural, dict):
        raise fail("freeze/phase120 structural result authority missing")
    for key, expected in {"commit": STRUCTURAL_COMMIT, "path": relative(STRUCTURAL_RESULT), "sha256": STRUCTURAL_SHA256}.items():
        assert_equal(structural.get(key), expected, f"freeze/structural/{key}")
    candidate = freeze.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID, "candidate_count": 1, "routes": list(ROUTES), "route_order_fixed": True,
        "runs_per_route": 1, "solver_rerun": False, "raw_or_base_rerun": False, "tdcp_rerun": False,
        "solution_repaired_or_replaced": False, "solution_publication": False,
        "metadata_hash_seal_precedes_truth": True, "coordinate_interpretation_before_truth": False,
        "candidate_files_copied_or_transformed": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    offset = candidate.get("phase120_offset_boundary")
    if not isinstance(offset, dict):
        raise fail("freeze/candidate/phase120_offset_boundary missing")
    for key, expected in {"required_already_applied_in_structural_run": True, "evaluator_must_not_apply_again": True, "double_application": "fail-closed"}.items():
        assert_equal(offset.get(key), expected, f"freeze/offset/{key}")
    for route in ROUTES:
        metadata = candidate.get("routes_metadata", {}).get(route)
        if not isinstance(metadata, dict):
            raise fail(f"freeze candidate metadata missing: {route}")
        _relative_metadata(metadata.get("path"), "withheld_solution_output.csv", f"candidate {route}", "/phase120-tdcp-equation-normalization-v1/")
        for key, expected in {"expected_prediction_rows": DOMAIN_ROWS[route], "expected_problem_epochs": DOMAIN_ROWS[route] + 1, "opaque_metadata_seal": True}.items():
            assert_equal(metadata.get(key), expected, f"freeze/candidate/{route}/{key}")
        if not isinstance(metadata.get("sha256"), str) or len(metadata["sha256"]) != 64 or not isinstance(metadata.get("bytes"), int):
            raise fail(f"freeze candidate seal malformed: {route}")

    truth = freeze.get("truth_cohort")
    if not isinstance(truth, dict):
        raise fail("freeze/truth_cohort missing")
    for key, expected in {"route_order": list(ROUTES), "read_by_solver": False, "read_by_audit": False, "read_by_freeze": False, "read_by_manifest": False, "read_by_evaluator_only": True}.items():
        assert_equal(truth.get(key), expected, f"freeze/truth_cohort/{key}")
    for route in ROUTES:
        item = truth.get("routes", {}).get(route)
        if not isinstance(item, dict):
            raise fail(f"freeze truth metadata missing: {route}")
        _relative_metadata(item.get("path"), "ground_truth.csv", f"truth {route}", "/truth/")
        for key, expected in {"rows": TRUTH_ROWS[route]}.items():
            assert_equal(item.get(key), expected, f"freeze/truth/{route}/{key}")
        if not isinstance(item.get("sha256"), str) or len(item["sha256"]) != 64 or not isinstance(item.get("bytes"), int):
            raise fail(f"freeze truth metadata seal malformed: {route}")
    _verify_metric_contract(freeze.get("metric_contract"), "freeze/metric_contract")
    _verify_metric_contract(read_json(PHASE118_MANIFEST, "Phase118 metric manifest").get("metric_contract"), "Phase118/metric_contract")
    baselines = freeze.get("comparison_baselines")
    if not isinstance(baselines, dict):
        raise fail("freeze/comparison_baselines missing")
    assert_equal(baselines["phase112_same_pipeline"]["macro_m"], 0.8318381724000121, "freeze/Phase112/macro")
    assert_equal(baselines["phase118_same_recipe"]["macro_m"], 0.814198150322117, "freeze/Phase118/macro")
    gates = freeze.get("promotion_gates")
    if not isinstance(gates, dict):
        raise fail("freeze/promotion_gates missing")
    for key, expected in {
        "all_gates_anded": True, "exact_two_existing_routes_one_evaluation_each": True,
        "candidate_output_schema_and_exact_alignment": True, "candidate_prediction_domain_coverage_exact": 1.0,
        "candidate_all_finite_and_earth_valid": True, "candidate_over_70_mps_count_zero": True,
        "candidate_no_route_regression_vs_phase112": True, "strict_less_than_is_explicit_and_gate": True,
        "strict_macro_comparator": "candidate_macro_score_m < 0.782", "strict_macro_threshold_m": 0.782,
        "equality_at_threshold_passes": False, "pixel5_offset_already_exactly_once_no_evaluator_reapplication": True,
        "truth_read_only_by_one_evaluator_subprocess": True, "solution_rows_absent_from_result": True,
        "no_solver_raw_base_tdcp_mat_precomputed_phone_coordinates_pdc_or_kaggle": True,
    }.items():
        assert_equal(gates.get(key), expected, f"freeze/promotion_gates/{key}")
    accounting = freeze.get("read_accounting", {})
    _verify_zero_accounting(accounting.get("audit_before_freeze"), "freeze/audit_before_freeze")
    planned = accounting.get("planned_truth_evaluation")
    if not isinstance(planned, dict):
        raise fail("freeze/planned_truth_evaluation missing")
    for key, expected in {"native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0, "raw_base_rinex_reads": 0, "candidate_solution_reads_for_hash_and_parse": 2, "candidate_coordinate_interpretations": 2, "truth_reads": 2, "truth_reads_per_route": 1, "accuracy_calculations": 2, "mat_precomputed_phone_coordinate_pdc_reads": 0, "kaggle_or_token_access": 0, "reruns": 0, "fallbacks": 0}.items():
        assert_equal(planned.get(key), expected, f"freeze/planned_truth_evaluation/{key}")
    boundary = freeze.get("truth_evaluator_boundary")
    if not isinstance(boundary, dict):
        raise fail("freeze/truth_evaluator_boundary missing")
    for key, expected in {"authorization_required_before_path_materialization": True, "candidate_metadata_hash_seal_precedes_truth": True, "candidate_parse_precedes_truth": True, "evaluator_process_count": 1, "candidate_solution_reads_for_hash_and_parse": 2, "truth_reads": 2, "truth_reads_per_route": 1, "accuracy_calculations": 2, "native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0, "raw_base_rinex_reads": 0, "truth_path_or_bytes_in_native": False, "native_after_truth": False, "no_solution_rows_in_result": True, "no_truth_coordinate_rows_in_result": True, "release_or_submission": False}.items():
        assert_equal(boundary.get(key), expected, f"freeze/truth_evaluator_boundary/{key}")
    leakage = freeze.get("leakage_guard")
    if not isinstance(leakage, dict):
        raise fail("freeze/leakage_guard missing")
    for key in ("native_solver_rerun", "raw_gnss_imu_navigation_rerun", "raw_base_rinex_rerun", "mat_reads_or_generated", "precomputed_phone_coordinate_reads", "pdc_reads", "kaggle_or_token_access", "truth_reads_before_authorization", "accuracy_selection_before_truth", "post_truth_tuning", "solution_publication", "offset_reapplication", "repair_or_fallback"):
        assert_equal(leakage.get(key), False, f"freeze/leakage_guard/{key}")
    return freeze


def verify_manifest() -> dict[str, Any]:
    freeze = verify_freeze()
    structural = verify_structural_result()
    for path, expected, label in (
        (PHASE118_EVALUATOR, PHASE118_EVALUATOR_SHA256, "Phase118 evaluator"),
        (PHASE118_MANIFEST, PHASE118_MANIFEST_SHA256, "Phase118 accuracy manifest"),
        (PHASE118_FREEZE, PHASE118_FREEZE_SHA256, "Phase118 accuracy freeze"),
        (PHASE118_RESULT, PHASE118_RESULT_SHA256, "Phase118 accuracy result"),
        (PHASE82_EVALUATOR, PHASE82_EVALUATOR_SHA256, "Phase82 scorer"),
        (PHASE82_RESULT, PHASE82_RESULT_SHA256, "Phase82 result"),
        (PHASE76_PARSER, PHASE76_PARSER_SHA256, "Phase76 parser"),
    ):
        assert_equal(sha256_file(path, label), expected, f"{label}/sha256")
    manifest = read_json(MANIFEST, "Phase120 truth accuracy manifest")
    for key, expected in {"schema_version": MANIFEST_SCHEMA, "phase": 120, "execution_label": "Luna Max", "status": "sealed-before-phase120-truth-only-accuracy-evaluation"}.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    assert_equal(manifest.get("routes"), list(ROUTES), "manifest/routes")
    authority = manifest.get("authority")
    if not isinstance(authority, dict):
        raise fail("manifest/authority missing")
    pins = {
        "accuracy_freeze": (FREEZE, FREEZE_SHA256), "accuracy_audit": (AUDIT, AUDIT_SHA256),
        "phase120_structural_result": (STRUCTURAL_RESULT, STRUCTURAL_SHA256),
        "phase118_accuracy_evaluator": (PHASE118_EVALUATOR, PHASE118_EVALUATOR_SHA256),
        "phase118_accuracy_manifest": (PHASE118_MANIFEST, PHASE118_MANIFEST_SHA256),
        "phase118_accuracy_freeze": (PHASE118_FREEZE, PHASE118_FREEZE_SHA256),
        "phase118_accuracy_result": (PHASE118_RESULT, PHASE118_RESULT_SHA256),
        "phase82_metric_reference": (PHASE82_EVALUATOR, PHASE82_EVALUATOR_SHA256),
        "phase82_result": (PHASE82_RESULT, PHASE82_RESULT_SHA256),
        "phase76_parser_reference": (PHASE76_PARSER, PHASE76_PARSER_SHA256),
    }
    for key, (path, expected) in pins.items():
        _pin(authority, key, path, expected)
        assert_equal(sha256_file(path, f"manifest/{key}"), expected, f"manifest/{key}/sha256")
    for key, path in (("evaluator", EVALUATOR), ("focused_tests", FOCUSED_TESTS)):
        item = authority.get(key)
        if not isinstance(item, dict):
            raise fail(f"manifest/{key} pin missing")
        assert_equal(item.get("path"), relative(path), f"manifest/{key}/path")
        expected = item.get("sha256")
        if not isinstance(expected, str) or len(expected) != 64:
            raise fail(f"manifest/{key}/sha256 missing")
        assert_equal(sha256_file(path, f"manifest/{key}"), expected, f"manifest/{key}/sha256")
    assert_equal(manifest.get("freeze"), {"path": relative(FREEZE), "commit": FREEZE_COMMIT, "sha256": FREEZE_SHA256}, "manifest/freeze")
    assert_equal(manifest.get("structural_result"), {"path": relative(STRUCTURAL_RESULT), "commit": STRUCTURAL_COMMIT, "sha256": STRUCTURAL_SHA256}, "manifest/structural_result")
    assert_equal(manifest.get("candidate"), freeze["candidate"], "manifest/candidate")
    assert_equal(manifest.get("truth_cohort"), freeze["truth_cohort"], "manifest/truth_cohort")
    _verify_metric_contract(manifest.get("metric_contract"), "manifest/metric_contract")
    assert_equal(manifest.get("comparison_baselines"), freeze["comparison_baselines"], "manifest/comparison_baselines")
    assert_equal(manifest.get("promotion_gates"), {"strict_macro_comparator": "candidate_macro_score_m < 0.782", "strict_macro_threshold_m": 0.782, "equality_at_threshold_passes": False, "exact_route_order": list(ROUTES), "route_evaluations": 2, "truth_reads_per_route": 1, "no_repair_or_rerun_or_fallback": True, "pixel5_offset_already_applied_exactly_once": True, "evaluator_must_not_apply_offset": True, "solution_rows_in_result": False}, "manifest/promotion_gates")
    _verify_zero_accounting(manifest.get("read_accounting_before_authorization"), "manifest/read_accounting_before_authorization")
    boundary = manifest.get("truth_evaluator_boundary")
    if not isinstance(boundary, dict):
        raise fail("manifest/truth_evaluator_boundary missing")
    for key, expected in {"authorization_required_before_path_materialization": True, "candidate_metadata_hash_seal_precedes_truth": True, "candidate_parse_precedes_truth": True, "evaluator_process_count": 1, "candidate_solution_reads_for_hash_and_parse": 2, "truth_reads": 2, "truth_reads_per_route": 1, "accuracy_calculations": 2, "native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0, "raw_base_rinex_reads": 0, "no_solution_rows_in_result": True, "no_truth_coordinate_rows_in_result": True, "release_or_submission": False}.items():
        assert_equal(boundary.get(key), expected, f"manifest/truth_evaluator_boundary/{key}")
    requirements = manifest.get("authorization_requirements")
    if not isinstance(requirements, dict):
        raise fail("manifest/authorization_requirements missing")
    for key, expected in {
        "authorization_file": relative(AUTHORIZATION),
        "schema_version": AUTHORIZATION_SCHEMA,
        "status": "authorized-for-phase120-truth-only-accuracy-evaluation",
        "independent_commit_required": True,
        "candidate_and_truth_paths_materialized_only_after_auth": True,
        "route_order": list(ROUTES),
        "exact_candidate_reads": 2,
        "exact_truth_reads": 2,
        "exact_accuracy_calculations": 2,
        "native_solver_invocations": 0,
        "raw_reads": 0,
        "base_reads": 0,
        "reruns": 0,
        "fallbacks": 0,
        "solution_publication": False,
        "mat_precomputed_phone_coordinate_pdc_reads": 0,
        "kaggle_or_token_access": 0,
    }.items():
        assert_equal(requirements.get(key), expected, f"manifest/authorization_requirements/{key}")
    return manifest


def verify_pre_truth() -> dict[str, Any]:
    manifest = verify_manifest()
    return {"status": "pre-truth-verified", "phase": 120, "execution_label": "Luna Max", "route_order": list(ROUTES), "candidate_count": manifest["candidate"]["candidate_count"], "candidate_paths_materialized": 0, "truth_paths_materialized": 0, "native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0, "raw_base_rinex_reads": 0, "truth_reads": 0, "candidate_solution_reads": 0, "candidate_coordinate_interpretations": 0, "accuracy_calculations": 0, "mat_precomputed_phone_coordinate_pdc_reads": 0, "kaggle_or_token_access": 0, "reruns": 0, "fallbacks": 0, "raw_execution_or_solver_authorized": False, "truth_only_authorized": False, "solution_output_published": False}


def load_phase118() -> Any:
    """Load only the pinned Phase118 evaluator source; no payload is opened."""
    assert_equal(sha256_file(PHASE118_EVALUATOR, "Phase118 evaluator"), PHASE118_EVALUATOR_SHA256, "Phase118 evaluator/sha256")
    spec = importlib.util.spec_from_file_location("phase118_accuracy_reference_phase120", PHASE118_EVALUATOR)
    if spec is None or spec.loader is None:
        raise fail("unable to load Phase118 evaluator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _verify_authority_pin(container: dict[str, Any], key: str, path: Path, expected: str) -> None:
    _pin(container, key, path, expected)
    assert_equal(sha256_file(path, f"authorization/{key}"), expected, f"authorization/{key}/sha256")


def verify_authorization() -> dict[str, Any]:
    """Verify an independent authorization without opening candidate/truth payloads."""
    verify_manifest()
    auth = read_json(AUTHORIZATION, "Phase120 truth-only authorization")
    for key, expected in {
        "schema_version": AUTHORIZATION_SCHEMA,
        "phase": 120,
        "execution_label": "Luna Max",
        "status": "authorized-for-phase120-truth-only-accuracy-evaluation",
    }.items():
        assert_equal(auth.get(key), expected, f"authorization/{key}")
    authority = auth.get("authority")
    if not isinstance(authority, dict):
        raise fail("authorization authority missing")
    _verify_authority_pin(authority, "accuracy_freeze", FREEZE, FREEZE_SHA256)
    _verify_authority_pin(authority, "accuracy_manifest", MANIFEST, sha256_file(MANIFEST, "Phase120 accuracy manifest"))
    _verify_authority_pin(authority, "accuracy_audit", AUDIT, AUDIT_SHA256)
    _verify_authority_pin(authority, "accuracy_evaluator", EVALUATOR, sha256_file(EVALUATOR, "Phase120 accuracy evaluator"))
    _verify_authority_pin(authority, "focused_tests", FOCUSED_TESTS, sha256_file(FOCUSED_TESTS, "Phase120 focused tests"))
    _verify_authority_pin(authority, "phase120_structural_result", STRUCTURAL_RESULT, STRUCTURAL_SHA256)
    _verify_authority_pin(authority, "phase118_accuracy_evaluator", PHASE118_EVALUATOR, PHASE118_EVALUATOR_SHA256)
    _verify_authority_pin(authority, "phase118_accuracy_manifest", PHASE118_MANIFEST, PHASE118_MANIFEST_SHA256)
    _verify_authority_pin(authority, "phase118_accuracy_freeze", PHASE118_FREEZE, PHASE118_FREEZE_SHA256)
    _verify_authority_pin(authority, "phase118_accuracy_result", PHASE118_RESULT, PHASE118_RESULT_SHA256)
    _verify_authority_pin(authority, "phase82_metric_reference", PHASE82_EVALUATOR, PHASE82_EVALUATOR_SHA256)
    _verify_authority_pin(authority, "phase82_result", PHASE82_RESULT, PHASE82_RESULT_SHA256)
    _verify_authority_pin(authority, "phase76_parser_reference", PHASE76_PARSER, PHASE76_PARSER_SHA256)
    _verify_authority_pin(authority, "phase112_accuracy_result", ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_accuracy_result_v1.json", PHASE112_RESULT_SHA256)
    assert_equal(auth.get("routes"), list(ROUTES), "authorization/routes")
    candidate = auth.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("authorization candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID,
        "candidate_count": 1,
        "route_order": list(ROUTES),
        "runs_per_route": 1,
        "candidate_solution_reads_for_hash_and_parse": 2,
        "truth_reads": 2,
        "truth_reads_per_route": 1,
        "accuracy_calculations": 2,
        "solver_rerun": False,
        "raw_or_base_rerun": False,
        "solution_publication": False,
        "no_repair_or_rerun_or_fallback": True,
    }.items():
        assert_equal(candidate.get(key), expected, f"authorization/candidate/{key}")
    policy = auth.get("execution_policy")
    if not isinstance(policy, dict):
        raise fail("authorization execution policy missing")
    for key, expected in {
        "truth_only_evaluation_authorized": True,
        "native_solver_invocations": 0,
        "raw_reads": 0,
        "base_reads": 0,
        "candidate_reads_per_route": 1,
        "truth_reads_per_route": 1,
        "accuracy_calculations": 2,
        "native_after_truth": False,
        "solution_publication": False,
        "strict_macro_comparator": "candidate_macro_score_m < 0.782",
        "strict_macro_threshold_m": STRICT_PROMOTION_THRESHOLD_M,
        "mat_precomputed_phone_coordinate_pdc_reads": 0,
        "kaggle_or_token_access": 0,
        "reruns": 0,
        "fallbacks": 0,
    }.items():
        assert_equal(policy.get(key), expected, f"authorization/execution_policy/{key}")
    assert_equal(auth.get("manifest_sha256"), sha256_file(MANIFEST, "Phase120 accuracy manifest"), "authorization/manifest_sha256")
    return auth


def strict_macro_gate(value: Any) -> bool:
    return finite(value) and float(value) < STRICT_PROMOTION_THRESHOLD_M


def materialize_candidate_path(structural: dict[str, Any], route: str, *, authorized: bool) -> Path:
    if not authorized:
        raise fail("candidate path materialization requires independent truth authorization")
    if route not in ROUTES:
        raise fail(f"unknown route: {route}")
    text = structural["routes"][route]["solution_hash_seal"]["path"]
    _relative_metadata(text, "withheld_solution_output.csv", f"candidate {route}", "/phase120-tdcp-equation-normalization-v1/")
    return ROOT / text


def materialize_truth_path(freeze: dict[str, Any], route: str, *, authorized: bool) -> Path:
    if not authorized:
        raise fail("truth path materialization requires independent truth authorization")
    if route not in ROUTES:
        raise fail(f"unknown route: {route}")
    text = freeze["truth_cohort"]["routes"][route]["path"]
    _relative_metadata(text, "ground_truth.csv", f"truth {route}", "/truth/")
    return ROOT / text


def read_candidate_once(path: Path, seal: dict[str, Any], route: str, p82: Any) -> tuple[list[tuple[int, float, float]], dict[int, tuple[float, float]], dict[str, Any]]:
    payload = path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != seal.get("sha256") or len(payload) != seal.get("bytes"):
        raise fail(f"Phase120 candidate opaque seal mismatch: {route}")
    ordered, mapping = p82.P76.P74._parse_submission(payload, route)
    if len(ordered) != DOMAIN_ROWS[route] or len(mapping) != len(ordered) or [item[0] for item in ordered] != sorted(item[0] for item in ordered):
        raise fail(f"Phase120 candidate exact alignment failed: {route}")
    if any(not all(math.isfinite(float(value)) for value in item[1:]) or not -90.0 <= item[1] <= 90.0 or not -180.0 <= item[2] <= 180.0 for item in ordered):
        raise fail(f"Phase120 candidate finite/Earth-valid preflight failed: {route}")
    return ordered, mapping, {"path": relative(path), "bytes": len(payload), "sha256": digest, "header": "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees", "rows": len(ordered), "read_count": 1, "coordinate_rows_omitted": True}


def read_truth_once(path: Path, pin: dict[str, Any], route: str, p82: Any) -> tuple[dict[int, tuple[float, float]], dict[str, Any]]:
    payload = path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != pin.get("sha256") or len(payload) != pin.get("bytes"):
        raise fail(f"official truth seal mismatch: {route}")
    truth = p82.P76._parse_truth_dictreader(payload, route)
    if len(truth) != pin.get("rows"):
        raise fail(f"official truth row count mismatch: {route}")
    return truth, {"path": relative(path), "bytes": len(payload), "sha256": digest, "rows": len(truth), "read_count": 1, "coordinate_rows_omitted": True}


def sealed_baselines() -> dict[str, Any]:
    phase112 = read_json(ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_accuracy_result_v1.json", "Phase112 sealed accuracy aggregate")
    assert_equal(sha256_file(ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_accuracy_result_v1.json", "Phase112 sealed accuracy aggregate"), PHASE112_RESULT_SHA256, "Phase112 aggregate/sha256")
    phase118 = read_json(PHASE118_RESULT, "Phase118 sealed accuracy aggregate")
    assert_equal(phase118.get("aggregate", {}).get("candidate_macro_score_m"), 0.814198150322117, "Phase118 aggregate/macro")
    try:
        phase112_routes = {route: float(phase112["routes"][route]["candidate"]["score_m"]) for route in ROUTES}
        phase112_macro = float(phase112["aggregate"]["candidate_macro_score_m"])
    except (KeyError, TypeError, ValueError) as exc:
        raise fail(f"sealed baseline aggregate malformed: {exc}") from exc
    assert_equal(phase112_routes, {ROUTES[0]: 0.997685253035948, ROUTES[1]: 0.6659910917640763}, "Phase112 sealed routes")
    assert_equal(phase112_macro, 0.8318381724000121, "Phase112 sealed macro")
    return {"phase112": phase112_routes, "phase112_macro": phase112_macro, "phase118_macro": 0.814198150322117}


def evaluate(result_path: Path = RESULT_JSON) -> dict[str, Any]:
    """Future truth-only lane: auth first, exactly two candidate/truth reads."""
    manifest = verify_manifest()
    # This file is intentionally absent until a separate authorization commit.
    verify_authorization()
    freeze = verify_freeze()
    structural = verify_structural_result()
    p118 = load_phase118()
    p82 = p118.load_phase82()
    baselines = sealed_baselines()
    accounting = {"candidate_paths_materialized": 0, "truth_paths_materialized": 0, "candidate_solution_reads": 0, "candidate_coordinate_interpretations": 0, "truth_reads": 0, "accuracy_calculations": 0}
    reports: dict[str, Any] = {}
    for route in ROUTES:
        candidate_path = materialize_candidate_path(structural, route, authorized=True)
        accounting["candidate_paths_materialized"] += 1
        ordered, candidate, candidate_meta = read_candidate_once(candidate_path, freeze["candidate"]["routes_metadata"][route], route, p82)
        accounting["candidate_solution_reads"] += 1
        accounting["candidate_coordinate_interpretations"] += 1
        truth_path = materialize_truth_path(freeze, route, authorized=True)
        accounting["truth_paths_materialized"] += 1
        truth, truth_meta = read_truth_once(truth_path, freeze["truth_cohort"]["routes"][route], route, p82)
        accounting["truth_reads"] += 1
        score = p82.P76._score_prediction(candidate, truth, EXPECTED_MISSING[route], route, ordered)
        accounting["accuracy_calculations"] += 1
        phase112_score = baselines["phase112"][route]
        checks = {"candidate_schema_and_exact_alignment": len(ordered) == DOMAIN_ROWS[route], "candidate_prediction_domain_coverage_exact": score.get("prediction_domain_coverage") == 1.0, "candidate_finite_and_earth_valid": score.get("finite") is True, "candidate_over_70_mps_count_zero": score.get("over_70_mps_count") == 0, "candidate_route_score_finite": finite(score.get("score_m")), "candidate_route_score_at_most_3m": finite(score.get("score_m")) and score.get("score_m") <= 3.0, "candidate_no_route_regression_vs_phase112": finite(score.get("score_m")) and score.get("score_m") <= phase112_score}
        reports[route] = {"dataset_id": route, "candidate_solution": candidate_meta, "truth": truth_meta, "truth_read": True, "accuracy_scored": True, "candidate": score, "baseline_phase112_same_pipeline": {"score_m": phase112_score}, "gates": {"passed": all(checks.values()), "checks": checks, "failures": [key for key, value in checks.items() if value is not True]}}
    candidate_macro = sum(reports[route]["candidate"]["score_m"] for route in ROUTES) / 2.0
    gates = {"exact_two_routes_one_evaluation_each": accounting == {**accounting, "candidate_paths_materialized": 2, "truth_paths_materialized": 2, "candidate_solution_reads": 2, "candidate_coordinate_interpretations": 2, "truth_reads": 2, "accuracy_calculations": 2}, "candidate_output_schema_and_exact_alignment": all(reports[route]["gates"]["checks"]["candidate_schema_and_exact_alignment"] for route in ROUTES), "candidate_prediction_domain_coverage_exact": all(reports[route]["candidate"].get("prediction_domain_coverage") == 1.0 for route in ROUTES), "candidate_all_finite_and_earth_valid": all(reports[route]["candidate"].get("finite") is True for route in ROUTES), "candidate_over_70_mps_count_zero": all(reports[route]["candidate"].get("over_70_mps_count") == 0 for route in ROUTES), "candidate_route_scores_finite": all(finite(reports[route]["candidate"].get("score_m")) for route in ROUTES), "candidate_each_route_score_at_most_3m": all(reports[route]["candidate"]["score_m"] <= 3.0 for route in ROUTES), "candidate_no_route_regression_vs_phase112": all(reports[route]["candidate"]["score_m"] <= baselines["phase112"][route] for route in ROUTES), "candidate_macro_score_strict_less_than_0_782m": strict_macro_gate(candidate_macro), "truth_read_only_by_one_evaluator_process": accounting["truth_reads"] == 2, "no_solution_rows_in_result": True, "no_solver_raw_base_tdcp_mat_precomputed_pdc_kaggle": True}
    failed = [key for key, value in gates.items() if value is not True]
    result = {"schema_version": RESULT_SCHEMA, "phase": 120, "execution_label": "Luna Max", "status": "go-phase120-truth-only-accuracy" if not failed else "no-go-phase120-truth-only-accuracy", "decision": "truth-only accuracy evaluated; release remains separately unauthorized" if not failed else "truth-only accuracy gate failed closed; preserve artifacts and do not rerun", "candidate": CANDIDATE_ID, "accuracy_scored": True, "routes": reports, "aggregate": {"candidate_macro_score_m": candidate_macro, "phase112_same_pipeline_macro_m": baselines["phase112_macro"], "phase118_same_recipe_macro_m": baselines["phase118_macro"], "route_count": 2, "macro_route_order": list(ROUTES), "macro_weighting": "unweighted arithmetic mean over exactly MTV-A then LAX-T"}, "metric_contract": manifest["metric_contract"], "promotion_gates": gates, "strict_0_782_gate": {"comparator": "candidate_macro_score_m < 0.782", "threshold_m": STRICT_PROMOTION_THRESHOLD_M, "candidate_macro_score_m": candidate_macro, "passed": gates["candidate_macro_score_strict_less_than_0_782m"]}, "failed_gates": failed, "read_accounting": {"native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0, "raw_base_rinex_reads": 0, "candidate_solution_reads_for_hash_and_parse": accounting["candidate_solution_reads"], "candidate_coordinate_interpretations": accounting["candidate_coordinate_interpretations"], "truth_reads": accounting["truth_reads"], "truth_reads_per_route": 1, "accuracy_calculations": accounting["accuracy_calculations"], "candidate_paths_materialized_after_authorization": accounting["candidate_paths_materialized"], "truth_paths_materialized_after_authorization": accounting["truth_paths_materialized"], "reruns": 0, "fallbacks": 0, "mat_precomputed_phone_coordinate_pdc_reads": 0, "kaggle_or_token_access": 0, "truth_reads_by_process": "one Phase120 truth-only evaluator process"}, "forbidden_lanes": {"native_solver_rerun": False, "raw_or_base_rerun": False, "tdcp_rerun": False, "mat": False, "precomputed_phone_coordinates": False, "pdc": False, "kaggle_or_token": False, "solution_rows_in_result": False}, "solution_output_published": False, "release_or_submission_authorized": False}
    atomic_json(result_path, result)
    atomic_text(result_path.with_suffix(".md"), "# Phase120 truth-only accuracy result\n\nCoordinates are omitted; release remains unauthorized.\n")
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--verify-manifest", action="store_true")
    parser.add_argument("--verify-pre-truth", action="store_true")
    parser.add_argument("--evaluate", action="store_true")
    parser.add_argument("--result-json", type=Path, default=RESULT_JSON)
    args = parser.parse_args(argv)
    modes = [args.verify_freeze, args.verify_manifest, args.verify_pre_truth, args.evaluate]
    if sum(bool(mode) for mode in modes) != 1:
        parser.error("choose exactly one verification/evaluation mode")
    try:
        if args.verify_freeze:
            verify_freeze()
        elif args.verify_manifest:
            verify_manifest()
        elif args.verify_pre_truth:
            print(json.dumps(verify_pre_truth(), indent=2, sort_keys=True))
        else:
            result = evaluate(args.result_json)
            print(json.dumps(result, indent=2, sort_keys=True))
            return 0 if result.get("status", "").startswith("go-") else 1
        return 0
    except Exception as exc:
        print(f"phase120 truth-only evaluator: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
