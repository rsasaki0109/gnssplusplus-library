#!/usr/bin/env python3
"""Launch-free contract and isolated truth-only evaluator for Phase134.

The verification modes read tracked source and sealed metadata only.  They do
not stat, hash, or open a candidate solution, official truth, raw input, or
native output payload.  The optional ``--evaluate`` mode is deliberately
gated by a future independent truth-only authorization; only that mode may
materialize the two opaque candidate files and two official truth files.

The metric implementation is delegated to the pinned Phase118/Phase82/Phase76
helpers after authorization.  This lane never launches a solver, changes a
candidate, reapplies the Pixel5 offset, or publishes solution rows.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[3]
EVALUATOR = Path(__file__).resolve()
AUDIT = ROOT / "docs/use_cases/records/smartphone_r5_phase134_native_summary_bridge_accuracy_audit_v1.md"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase134_native_summary_bridge_accuracy_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase134_native_summary_bridge_accuracy_manifest_v1.json"
PRE_TRUTH = ROOT / "docs/use_cases/records/smartphone_r5_phase134_native_summary_bridge_accuracy_pre_truth_audit_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase134_native_summary_bridge_accuracy_truth_authorization_v1.json"
STRUCTURAL_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase134_native_summary_bridge_structural_result_v1.json"
PHASE120_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase120_tdcp_equation_normalization_accuracy.py"
PHASE120_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase120_tdcp_equation_normalization_accuracy_manifest_v1.json"
PHASE120_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase120_tdcp_equation_normalization_accuracy_freeze_v1.json"
PHASE120_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase120_tdcp_equation_normalization_accuracy_result_v1.json"
PHASE118_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase118_tdcp_robust_k_accuracy.py"
PHASE118_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_accuracy_manifest_v1.json"
PHASE118_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_accuracy_gate_freeze_v1.json"
PHASE118_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_accuracy_result_v1.json"
PHASE82_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase82_phase80_direct_quality_accuracy.py"
PHASE82_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase82_phase80_direct_quality_accuracy_result_v1.json"
PHASE76_PARSER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase76_phase75_accuracy_integrity_recovery.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase134_native_summary_bridge_accuracy.py"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase134_native_summary_bridge_accuracy_result_v1.json"

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
CANDIDATE_ID = "phase134-native-phase131-summary-diagnostics-bridge-v1"
RESULT_SCHEMA = "smartphone-r5-phase134-native-summary-bridge-accuracy-result.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase134-native-summary-bridge-accuracy-manifest.v1"
AUTHORIZATION_SCHEMA = "smartphone-r5-phase134-native-summary-bridge-accuracy-truth-authorization.v1"

AUDIT_COMMIT = "32f7de632c6e11bb6b6d70c962df164ef33202b6"
AUDIT_SHA256 = "7eedb6e97aa9e97faa87ef237ed461d9b9a2f2308820854786162e22ab711077"
FREEZE_COMMIT = "75a3037114595ebe339fb0317ec50bfa45419cae"
STRUCTURAL_COMMIT = "f658d52ce666882c1516d9ede8d396885d8adbd6"
STRUCTURAL_SHA256 = "426adb4d951d226a4e99e36813e0873fd0b10ec47a6b62a10f2df4d67327f0fa"
PHASE120_EVALUATOR_COMMIT = "7e91c25e10b53f252b978542d3ec59f57d845a2a"
PHASE120_EVALUATOR_SHA256 = "22c0f3eb6c3624d81a3e7662828a1eb299a464c352250c16c101ae64eadab0cf"
PHASE120_MANIFEST_SHA256 = "48bcc206060a62f55b8f1094b6f68bf3a43f619aff44b77b25bebf2a05c6856d"
PHASE120_FREEZE_SHA256 = "f9b3746477a107d849cc3e957382d340c2a26d1677bf01edf879aef294bb4f2f"
PHASE120_RESULT_SHA256 = "608e5a1711c458240d4be7e481e93d8ba76d4d7677720f76033c9262d92a1dc2"
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
PHASE112_RESULT_COMMIT = "0151bd070f8029413f8f2a17a28fe982dddb735f"
PHASE112_RESULT_SHA256 = "46d8d836758ac1a88b95ec2c1a0c0080cedfb753c12411ed74e4c2501549c57e"


class Phase134AccuracyError(ValueError):
    """A truth-only contract violation; all failures are closed."""


def fail(message: str) -> Phase134AccuracyError:
    return Phase134AccuracyError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def assert_true(value: Any, label: str) -> None:
    if value is not True:
        raise fail(f"{label}: expected true, got {value!r}")


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def _payload_path(path: Path) -> bool:
    return path.name in {
        "opaque_solution_output.csv", "ground_truth.csv", "device_gnss.csv",
        "device_imu.csv", "brdc.nav", "base.obs",
    }


def sha256_file(path: Path, label: str) -> str:
    """Hash static artifacts only; reject payloads before filesystem probing."""
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


def atomic_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def _relative_metadata(path_text: Any, basename: str, label: str, fragment: str) -> str:
    if not isinstance(path_text, str) or not path_text:
        raise fail(f"missing {label} path metadata")
    path = Path(path_text)
    if path.is_absolute() or ".." in path.parts or path.name != basename or fragment not in path_text:
        raise fail(f"unsafe {label} path metadata: {path_text}")
    return path_text


def _pin(container: Mapping[str, Any], key: str, path: Path, expected: str, label: str | None = None) -> None:
    item = container.get(key)
    name = label or key
    if not isinstance(item, Mapping):
        raise fail(f"missing authority pin: {name}")
    assert_equal(item.get("path"), relative(path), f"authority/{name}/path")
    assert_equal(item.get("sha256"), expected, f"authority/{name}/sha256")


def metric_contract() -> dict[str, Any]:
    return {
        "source": "Phase118/120 truth-only contract and pinned Phase82/Phase76 scorer; no semantic change",
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


def _verify_metric_contract(value: Any, label: str) -> None:
    assert_equal(value, metric_contract(), label)


def _verify_zero_accounting(value: Any, label: str) -> None:
    if not isinstance(value, Mapping):
        raise fail(f"{label} missing")
    for key in (
        "candidate_paths_materialized", "truth_paths_materialized", "candidate_solution_payload_reads",
        "candidate_coordinate_interpretations", "truth_reads", "accuracy_calculations",
        "native_solver_invocations", "raw_gnss_imu_navigation_reads", "raw_base_rinex_reads",
        "mat_precomputed_phone_coordinate_pdc_reads", "kaggle_or_token_access", "reruns", "fallbacks",
    ):
        assert_equal(value.get(key), 0, f"{label}/{key}")
    assert_equal(value.get("raw_content_copied_or_transformed"), False, f"{label}/raw_content_copied_or_transformed")
    assert_equal(value.get("solution_rows_in_result"), False, f"{label}/solution_rows_in_result")
    assert_equal(value.get("solution_output_published"), False, f"{label}/solution_output_published")


def _verify_structural_accounting(value: Any, label: str) -> None:
    """Validate Phase134's already-sealed structural counter schema."""
    if not isinstance(value, Mapping):
        raise fail(f"{label} missing")
    for key, expected in {
        "raw_device_gnss_reads": 2, "raw_device_imu_reads": 2,
        "broadcast_navigation_reads": 2, "raw_base_rinex_payload_reads": 2,
        "raw_base_header_reads": 2, "raw_base_hash_reads": 2,
        "native_command_constructions": 2, "native_solver_invocations": 2,
        "solution_opaque_hash_reads": 2, "solution_coordinate_row_reads": 0,
        "solution_coordinate_interpretations": 0, "truth_reads": 0,
        "accuracy_calculations": 0, "mat_pdc_precomputed_coordinate_reads": 0,
        "kaggle_or_token_access": 0, "route_reruns": 0, "fallbacks": 0,
        "repairs": 0,
    }.items():
        assert_equal(value.get(key), expected, f"{label}/{key}")
    assert_equal(value.get("raw_content_copied_or_transformed"), False, f"{label}/raw_content_copied_or_transformed")


def verify_structural_result() -> dict[str, Any]:
    assert_equal(sha256_file(STRUCTURAL_RESULT, "Phase134 structural result"), STRUCTURAL_SHA256, "structural/sha256")
    result = read_json(STRUCTURAL_RESULT, "Phase134 structural result")
    for key, expected in {
        "schema_version": "smartphone-r5-phase134-native-summary-bridge-structural-result.v1",
        "phase": 134,
        "status": "go-phase134-native-summary-bridge-structural",
        "accuracy_scored": False,
        "solution_output_published": False,
        "truth_free": True,
        "promotion_authorized": False,
    }.items():
        assert_equal(result.get(key), expected, f"structural/{key}")
    _verify_structural_accounting(result.get("read_accounting"), "structural/read_accounting")
    matrix = result.get("matrix")
    if not isinstance(matrix, Mapping):
        raise fail("Phase134 structural matrix metadata missing")
    for key, expected in {
        "candidate_count": 1, "route_order": ["MTV-A", "LAX-T"],
        "runs_per_route": 1, "native_solver_invocations": 2,
        "truth_reads": 0, "accuracy_calculations": 0, "solution_rows_opened": 0,
    }.items():
        assert_equal(matrix.get(key), expected, f"structural/matrix/{key}")
    routes = result.get("routes")
    if not isinstance(routes, Mapping) or list(routes) != list(ROUTES):
        raise fail("Phase134 structural route set/order changed")
    candidate = result.get("candidate")
    if not isinstance(candidate, Mapping):
        raise fail("Phase134 structural candidate metadata missing")
    for key, expected in {
        "id": CANDIDATE_ID, "phase118": True, "phase126": True,
        "phase127": True, "phase128": True, "phase129": True, "phase131": True,
        "phase130_native_argv": False, "phase117_dynamic_sigma": False,
        "phase120_atmosphere": False, "fixed_tdcp_sigma": 0.03,
        "official_huber_k": 0.5, "main_solver": "MULTIFRONTAL_QR", "solution_opaque": True,
    }.items():
        assert_equal(candidate.get(key), expected, f"structural/candidate/{key}")
    for route in ROUTES:
        item = routes.get(route)
        if not isinstance(item, Mapping):
            raise fail(f"structural route missing: {route}")
        for key, expected in {
            "return_code": 0, "solver_launched": True, "solution_hash_only": True,
            "solution_opened": False, "solution_published": False, "truth_used": False,
            "accuracy_scored": False, "solution_present": True,
        }.items():
            assert_equal(item.get(key), expected, f"structural/{route}/{key}")
        digest = item.get("solution_hash_sealed")
        if not isinstance(digest, str) or len(digest) != 64:
            raise fail(f"structural/{route}/solution_hash_sealed malformed")
        expected_epochs = DOMAIN_ROWS[route] + 1
        assert_equal(item.get("solution_rows_sealed"), expected_epochs, f"structural/{route}/solution_rows_sealed")
        gates = item.get("gates")
        if not isinstance(gates, Mapping) or not gates or not all(value is True for value in gates.values()):
            raise fail(f"structural/{route}/gates are not all true")
    return result


def verify_freeze() -> dict[str, Any]:
    if not AUDIT.is_file() or not FREEZE.is_file():
        raise fail("Phase134 accuracy audit/freeze is missing")
    assert_equal(sha256_file(AUDIT, "Phase134 accuracy audit"), AUDIT_SHA256, "audit/sha256")
    freeze = read_json(FREEZE, "Phase134 accuracy freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase134-native-summary-bridge-accuracy-freeze.v1",
        "phase": 134,
        "execution_label": "Luna Max",
        "status": "frozen-before-phase134-truth-only-accuracy-evaluation",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    authority = freeze.get("authority")
    if not isinstance(authority, Mapping):
        raise fail("freeze/authority missing")
    _pin(authority, "accuracy_audit", AUDIT, AUDIT_SHA256)
    assert_equal(authority.get("accuracy_audit", {}).get("commit"), AUDIT_COMMIT, "freeze/audit/commit")
    _pin(authority, "phase134_structural_result", STRUCTURAL_RESULT, STRUCTURAL_SHA256)
    assert_equal(authority.get("phase134_structural_result", {}).get("commit"), STRUCTURAL_COMMIT, "freeze/structural/commit")
    _pin(authority, "phase120_metric_evaluator", PHASE120_EVALUATOR, PHASE120_EVALUATOR_SHA256)
    _pin(authority, "phase120_metric_manifest", PHASE120_MANIFEST, PHASE120_MANIFEST_SHA256)
    _pin(authority, "phase118_metric_evaluator", PHASE118_EVALUATOR, PHASE118_EVALUATOR_SHA256)
    _pin(authority, "phase118_metric_manifest", PHASE118_MANIFEST, PHASE118_MANIFEST_SHA256)
    _pin(authority, "phase82_scorer", PHASE82_EVALUATOR, PHASE82_EVALUATOR_SHA256)
    _pin(authority, "phase76_truth_parser", PHASE76_PARSER, PHASE76_PARSER_SHA256)
    _pin(authority, "phase112_sealed_baseline", ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_accuracy_result_v1.json", PHASE112_RESULT_SHA256)
    _pin(authority, "phase118_sealed_baseline", PHASE118_RESULT, PHASE118_RESULT_SHA256)
    _pin(authority, "phase120_sealed_baseline", PHASE120_RESULT, PHASE120_RESULT_SHA256)
    candidate = freeze.get("candidate")
    if not isinstance(candidate, Mapping):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID, "candidate_count": 1, "route_order_fixed": True,
        "runs_per_route": 1, "solver_rerun": False, "raw_or_base_rerun": False,
        "candidate_files_copied_or_transformed": False, "solution_repaired_or_replaced": False,
        "solution_publication": False, "coordinate_interpretation_before_truth": False,
        "metadata_hash_seal_precedes_truth": True,
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    assert_equal(candidate.get("routes"), list(ROUTES), "freeze/candidate/routes")
    offset = candidate.get("phase134_offset_boundary")
    if not isinstance(offset, Mapping):
        raise fail("freeze/candidate/phase134_offset_boundary missing")
    for key, expected in {
        "required_already_applied_in_structural_run": True,
        "evaluator_must_not_apply_again": True,
        "double_application": "fail-closed",
        "official_norm_m": 0.316227766,
        "native_corrected_epochs_must_equal_problem_epochs": True,
    }.items():
        assert_equal(offset.get(key), expected, f"freeze/offset/{key}")
    metadata = candidate.get("routes_metadata")
    if not isinstance(metadata, Mapping):
        raise fail("freeze/candidate/routes_metadata missing")
    structural = verify_structural_result()
    for route in ROUTES:
        item = metadata.get(route)
        if not isinstance(item, Mapping):
            raise fail(f"freeze candidate metadata missing: {route}")
        _relative_metadata(item.get("path"), "opaque_solution_output.csv", f"candidate {route}", "/phase134-native-summary-bridge-v1/")
        structural_item = structural["routes"][route]
        for key, expected in {
            "sha256": structural_item["solution_hash_sealed"],
            "expected_prediction_rows": DOMAIN_ROWS[route],
            "expected_problem_epochs": DOMAIN_ROWS[route] + 1,
            "solution_rows_sealed": DOMAIN_ROWS[route] + 1,
            "opaque_metadata_seal": True,
            "bytes": None,
            "bytes_not_probed_before_authorization": True,
        }.items():
            assert_equal(item.get(key), expected, f"freeze/candidate/{route}/{key}")
    truth = freeze.get("truth_cohort")
    if not isinstance(truth, Mapping):
        raise fail("freeze/truth_cohort missing")
    for key, expected in {
        "route_order": list(ROUTES), "read_by_solver": False, "read_by_audit": False,
        "read_by_freeze": False, "read_by_manifest": False, "read_by_evaluator_only": True,
    }.items():
        assert_equal(truth.get(key), expected, f"freeze/truth_cohort/{key}")
    truth_routes = truth.get("routes")
    if not isinstance(truth_routes, Mapping):
        raise fail("freeze/truth_cohort/routes missing")
    for route in ROUTES:
        item = truth_routes.get(route)
        if not isinstance(item, Mapping):
            raise fail(f"freeze truth metadata missing: {route}")
        _relative_metadata(item.get("path"), "ground_truth.csv", f"truth {route}", "/truth/")
        assert_equal(item.get("rows"), TRUTH_ROWS[route], f"freeze/truth/{route}/rows")
        if not isinstance(item.get("sha256"), str) or len(item["sha256"]) != 64:
            raise fail(f"freeze truth hash malformed: {route}")
    _verify_metric_contract(freeze.get("metric_contract"), "freeze/metric_contract")
    baselines = freeze.get("comparison_baselines")
    if not isinstance(baselines, Mapping):
        raise fail("freeze/comparison_baselines missing")
    assert_equal(baselines["phase112_same_pipeline"]["macro_m"], 0.8318381724000121, "freeze/Phase112/macro")
    assert_equal(baselines["phase118_champion"]["macro_m"], 0.814198150322117, "freeze/Phase118/macro")
    assert_equal(baselines["phase120"]["macro_m"], 0.8170323380544604, "freeze/Phase120/macro")
    gates = freeze.get("promotion_gates")
    if not isinstance(gates, Mapping):
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
    boundary = freeze.get("truth_evaluator_boundary")
    if not isinstance(boundary, Mapping):
        raise fail("freeze/truth_evaluator_boundary missing")
    for key, expected in {
        "authorization_required_before_path_materialization": True,
        "candidate_metadata_hash_seal_precedes_truth": True, "candidate_parse_precedes_truth": True,
        "evaluator_process_count": 1, "candidate_solution_reads_for_hash_and_parse": 2,
        "truth_reads": 2, "truth_reads_per_route": 1, "accuracy_calculations": 2,
        "native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0, "truth_path_or_bytes_in_native": False,
        "native_after_truth": False, "no_solution_rows_in_result": True,
        "no_truth_coordinate_rows_in_result": True, "release_or_submission": False,
    }.items():
        assert_equal(boundary.get(key), expected, f"freeze/truth_evaluator_boundary/{key}")
    _verify_zero_accounting(freeze.get("read_accounting", {}).get("audit_before_freeze"), "freeze/audit_before_freeze")
    planned = freeze.get("read_accounting", {}).get("planned_truth_evaluation")
    if not isinstance(planned, Mapping):
        raise fail("freeze/planned_truth_evaluation missing")
    for key, expected in {
        "candidate_paths_materialized": 2, "truth_paths_materialized": 2,
        "candidate_solution_reads_for_hash_and_parse": 2, "candidate_coordinate_interpretations": 2,
        "truth_reads": 2, "truth_reads_per_route": 1, "accuracy_calculations": 2,
        "native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0, "raw_base_rinex_reads": 0,
        "mat_precomputed_phone_coordinate_pdc_reads": 0, "kaggle_or_token_access": 0,
        "reruns": 0, "fallbacks": 0,
    }.items():
        assert_equal(planned.get(key), expected, f"freeze/planned/{key}")
    return freeze


def verify_manifest() -> dict[str, Any]:
    freeze = verify_freeze()
    manifest = read_json(MANIFEST, "Phase134 accuracy manifest")
    for key, expected in {
        "schema_version": MANIFEST_SCHEMA, "phase": 134,
        "execution_label": "Luna Max",
        "status": "sealed-before-phase134-truth-only-accuracy-evaluation",
    }.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    assert_equal(manifest.get("routes"), list(ROUTES), "manifest/routes")
    _verify_metric_contract(manifest.get("metric_contract"), "manifest/metric_contract")
    assert_equal(manifest.get("candidate"), freeze["candidate"], "manifest/candidate")
    assert_equal(manifest.get("truth_cohort"), freeze["truth_cohort"], "manifest/truth_cohort")
    assert_equal(manifest.get("comparison_baselines"), freeze["comparison_baselines"], "manifest/comparison_baselines")
    assert_equal(manifest.get("freeze"), {
        "path": relative(FREEZE), "commit": FREEZE_COMMIT,
        "sha256": sha256_file(FREEZE, "Phase134 accuracy freeze"),
    }, "manifest/freeze")
    assert_equal(manifest.get("structural_result"), {
        "path": relative(STRUCTURAL_RESULT), "commit": STRUCTURAL_COMMIT, "sha256": STRUCTURAL_SHA256,
    }, "manifest/structural_result")
    authority = manifest.get("authority")
    if not isinstance(authority, Mapping):
        raise fail("manifest/authority missing")
    pins = {
        "accuracy_audit": (AUDIT, AUDIT_SHA256),
        "accuracy_freeze": (FREEZE, sha256_file(FREEZE, "Phase134 accuracy freeze")),
        "phase134_structural_result": (STRUCTURAL_RESULT, STRUCTURAL_SHA256),
        "phase120_metric_evaluator": (PHASE120_EVALUATOR, PHASE120_EVALUATOR_SHA256),
        "phase120_metric_manifest": (PHASE120_MANIFEST, PHASE120_MANIFEST_SHA256),
        "phase118_metric_evaluator": (PHASE118_EVALUATOR, PHASE118_EVALUATOR_SHA256),
        "phase118_metric_manifest": (PHASE118_MANIFEST, PHASE118_MANIFEST_SHA256),
        "phase82_scorer": (PHASE82_EVALUATOR, PHASE82_EVALUATOR_SHA256),
        "phase76_truth_parser": (PHASE76_PARSER, PHASE76_PARSER_SHA256),
        "phase112_sealed_baseline": (ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_accuracy_result_v1.json", PHASE112_RESULT_SHA256),
        "phase118_sealed_baseline": (PHASE118_RESULT, PHASE118_RESULT_SHA256),
        "phase120_sealed_baseline": (PHASE120_RESULT, PHASE120_RESULT_SHA256),
    }
    for key, (path, expected) in pins.items():
        _pin(authority, key, path, expected)
        assert_equal(sha256_file(path, f"manifest/{key}"), expected, f"manifest/{key}/sha256")
    for key, path in (("evaluator", EVALUATOR), ("focused_tests", FOCUSED_TESTS)):
        item = authority.get(key)
        if not isinstance(item, Mapping):
            raise fail(f"manifest/{key} pin missing")
        assert_equal(item.get("path"), relative(path), f"manifest/{key}/path")
        digest = item.get("sha256")
        if not isinstance(digest, str) or len(digest) != 64:
            raise fail(f"manifest/{key}/sha256 malformed")
        assert_equal(sha256_file(path, f"manifest/{key}"), digest, f"manifest/{key}/sha256")
    _verify_zero_accounting(manifest.get("read_accounting_before_authorization"), "manifest/read_accounting_before_authorization")
    requirements = manifest.get("authorization_requirements")
    if not isinstance(requirements, Mapping):
        raise fail("manifest/authorization_requirements missing")
    for key, expected in {
        "authorization_file": relative(AUTHORIZATION), "schema_version": AUTHORIZATION_SCHEMA,
        "status": "authorized-for-phase134-truth-only-accuracy-evaluation",
        "independent_commit_required": True, "candidate_and_truth_paths_materialized_only_after_auth": True,
        "route_order": list(ROUTES), "exact_candidate_reads": 2, "exact_truth_reads": 2,
        "exact_accuracy_calculations": 2, "native_solver_invocations": 0, "raw_reads": 0,
        "base_reads": 0, "reruns": 0, "fallbacks": 0, "solution_publication": False,
        "mat_precomputed_phone_coordinate_pdc_reads": 0, "kaggle_or_token_access": 0,
    }.items():
        assert_equal(requirements.get(key), expected, f"manifest/authorization/{key}")
    return manifest


def verify_pre_truth() -> dict[str, Any]:
    manifest = verify_manifest()
    result = {
        "status": "pre-truth-verified", "phase": 134, "execution_label": "Luna Max",
        "route_order": list(ROUTES), "candidate_count": manifest["candidate"]["candidate_count"],
        "candidate_paths_materialized": 0, "truth_paths_materialized": 0,
        "native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0,
        "raw_base_rinex_reads": 0, "truth_reads": 0, "candidate_solution_reads": 0,
        "candidate_coordinate_interpretations": 0, "accuracy_calculations": 0,
        "mat_precomputed_phone_coordinate_pdc_reads": 0, "kaggle_or_token_access": 0,
        "reruns": 0, "fallbacks": 0, "raw_execution_or_solver_authorized": False,
        "truth_only_authorized": False, "solution_output_published": False,
    }
    return result


def _verify_auth_pin(container: Mapping[str, Any], key: str, path: Path, expected: str) -> None:
    _pin(container, key, path, expected)
    assert_equal(sha256_file(path, f"authorization/{key}"), expected, f"authorization/{key}/sha256")


def verify_authorization() -> dict[str, Any]:
    """Verify independent authorization without opening candidate or truth."""
    manifest = verify_manifest()
    auth = read_json(AUTHORIZATION, "Phase134 truth-only authorization")
    for key, expected in {
        "schema_version": AUTHORIZATION_SCHEMA, "phase": 134,
        "execution_label": "Luna Max",
        "status": "authorized-for-phase134-truth-only-accuracy-evaluation",
    }.items():
        assert_equal(auth.get(key), expected, f"authorization/{key}")
    authority = auth.get("authority")
    if not isinstance(authority, Mapping):
        raise fail("authorization/authority missing")
    for key, path in (("accuracy_audit", AUDIT), ("accuracy_freeze", FREEZE),
                      ("accuracy_manifest", MANIFEST), ("phase134_structural_result", STRUCTURAL_RESULT),
                      ("accuracy_evaluator", EVALUATOR), ("focused_tests", FOCUSED_TESTS),
                      ("phase120_metric_evaluator", PHASE120_EVALUATOR), ("phase118_metric_evaluator", PHASE118_EVALUATOR),
                      ("phase82_scorer", PHASE82_EVALUATOR), ("phase76_truth_parser", PHASE76_PARSER)):
        item = authority.get(key)
        if not isinstance(item, Mapping):
            raise fail(f"authorization/{key} pin missing")
        expected = item.get("sha256")
        if not isinstance(expected, str) or len(expected) != 64:
            raise fail(f"authorization/{key} hash malformed")
        _verify_auth_pin(authority, key, path, expected)
    assert_equal(auth.get("routes"), list(ROUTES), "authorization/routes")
    policy = auth.get("execution_policy")
    if not isinstance(policy, Mapping):
        raise fail("authorization/execution_policy missing")
    for key, expected in {
        "truth_only_evaluation_authorized": True, "native_solver_invocations": 0,
        "raw_reads": 0, "base_reads": 0, "candidate_reads_per_route": 1,
        "truth_reads_per_route": 1, "accuracy_calculations": 2,
        "native_after_truth": False, "solution_publication": False,
        "strict_macro_comparator": "candidate_macro_score_m < 0.782",
        "strict_macro_threshold_m": STRICT_PROMOTION_THRESHOLD_M,
        "mat_precomputed_phone_coordinate_pdc_reads": 0, "kaggle_or_token_access": 0,
        "reruns": 0, "fallbacks": 0,
    }.items():
        assert_equal(policy.get(key), expected, f"authorization/policy/{key}")
    assert_equal(auth.get("manifest_sha256"), sha256_file(MANIFEST, "Phase134 accuracy manifest"), "authorization/manifest_sha256")
    return auth


def materialize_candidate_path(freeze: Mapping[str, Any], route: str, *, authorized: bool) -> Path:
    if not authorized:
        raise fail("candidate path materialization requires independent truth authorization")
    if route not in ROUTES:
        raise fail(f"unknown route: {route}")
    text = freeze["candidate"]["routes_metadata"][route]["path"]
    _relative_metadata(text, "opaque_solution_output.csv", f"candidate {route}", "/phase134-native-summary-bridge-v1/")
    return ROOT / text


def materialize_truth_path(freeze: Mapping[str, Any], route: str, *, authorized: bool) -> Path:
    if not authorized:
        raise fail("truth path materialization requires independent truth authorization")
    if route not in ROUTES:
        raise fail(f"unknown route: {route}")
    text = freeze["truth_cohort"]["routes"][route]["path"]
    _relative_metadata(text, "ground_truth.csv", f"truth {route}", "/truth/")
    return ROOT / text


def load_phase118() -> Any:
    assert_equal(sha256_file(PHASE118_EVALUATOR, "Phase118 evaluator"), PHASE118_EVALUATOR_SHA256, "Phase118 evaluator/sha256")
    spec = importlib.util.spec_from_file_location("phase118_accuracy_reference_phase134", PHASE118_EVALUATOR)
    if spec is None or spec.loader is None:
        raise fail("unable to load Phase118 evaluator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def read_candidate_once(path: Path, seal: Mapping[str, Any], route: str, p82: Any) -> tuple[list[tuple[int, float, float]], dict[int, tuple[float, float]], dict[str, Any]]:
    payload = path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != seal.get("sha256"):
        raise fail(f"Phase134 candidate opaque seal mismatch: {route}")
    ordered, mapping = p82.P76.P74._parse_submission(payload, route)
    if len(ordered) != DOMAIN_ROWS[route] or len(mapping) != len(ordered) or [item[0] for item in ordered] != sorted(item[0] for item in ordered):
        raise fail(f"Phase134 candidate exact alignment failed: {route}")
    if any(not all(math.isfinite(float(value)) for value in item[1:]) or not -90.0 <= item[1] <= 90.0 or not -180.0 <= item[2] <= 180.0 for item in ordered):
        raise fail(f"Phase134 candidate finite/Earth-valid preflight failed: {route}")
    return ordered, mapping, {
        "path": relative(path), "bytes": len(payload), "sha256": digest,
        "header": "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees",
        "rows": len(ordered), "read_count": 1, "coordinate_rows_omitted": True,
    }


def read_truth_once(path: Path, pin: Mapping[str, Any], route: str, p82: Any) -> tuple[dict[int, tuple[float, float]], dict[str, Any]]:
    payload = path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != pin.get("sha256") or len(payload) != pin.get("bytes"):
        raise fail(f"official truth seal mismatch: {route}")
    truth = p82.P76._parse_truth_dictreader(payload, route)
    if len(truth) != pin.get("rows"):
        raise fail(f"official truth row count mismatch: {route}")
    return truth, {"path": relative(path), "bytes": len(payload), "sha256": digest, "rows": len(truth), "read_count": 1, "coordinate_rows_omitted": True}


def strict_macro_gate(value: Any) -> bool:
    return finite(value) and float(value) < STRICT_PROMOTION_THRESHOLD_M


def evaluate(result_path: Path = RESULT_JSON) -> dict[str, Any]:
    """Future isolated lane; authorization precedes exactly two payload reads."""
    manifest = verify_manifest()
    verify_authorization()
    freeze = verify_freeze()
    structural = verify_structural_result()
    p118 = load_phase118()
    p82 = p118.load_phase82()
    baseline = manifest["comparison_baselines"]["phase112_same_pipeline"]["route_scores_m"]
    accounting = {
        "candidate_paths_materialized": 0, "truth_paths_materialized": 0,
        "candidate_solution_reads": 0, "candidate_coordinate_interpretations": 0,
        "truth_reads": 0, "accuracy_calculations": 0,
    }
    reports: dict[str, Any] = {}
    for route in ROUTES:
        candidate_path = materialize_candidate_path(freeze, route, authorized=True)
        accounting["candidate_paths_materialized"] += 1
        ordered, candidate, candidate_meta = read_candidate_once(candidate_path, freeze["candidate"]["routes_metadata"][route], route, p82)
        accounting["candidate_solution_reads"] += 1
        accounting["candidate_coordinate_interpretations"] += 1
        truth_path = materialize_truth_path(freeze, route, authorized=True)
        accounting["truth_paths_materialized"] += 1
        truth, truth_meta = read_truth_once(truth_path, freeze["truth_cohort"]["routes"][route], route, p82)
        accounting["truth_reads"] += 1
        score = p82.P76._score_prediction(candidate, truth, freeze["truth_cohort"]["routes"][route].get("expected_missing_truth_key"), route, ordered)
        accounting["accuracy_calculations"] += 1
        phase112_score = float(baseline[route])
        checks = {
            "candidate_schema_and_exact_alignment": len(ordered) == DOMAIN_ROWS[route],
            "candidate_prediction_domain_coverage_exact": score.get("prediction_domain_coverage") == 1.0,
            "candidate_finite_and_earth_valid": score.get("finite") is True,
            "candidate_over_70_mps_count_zero": score.get("over_70_mps_count") == 0,
            "candidate_route_score_finite": finite(score.get("score_m")),
            "candidate_route_score_at_most_3m": finite(score.get("score_m")) and score.get("score_m") <= 3.0,
            "candidate_no_route_regression_vs_phase112": finite(score.get("score_m")) and score.get("score_m") <= phase112_score,
        }
        reports[route] = {
            "dataset_id": route, "candidate_solution": candidate_meta, "truth": truth_meta,
            "truth_read": True, "accuracy_scored": True, "candidate": score,
            "baseline_phase112_same_pipeline": {"score_m": phase112_score},
            "gates": {"passed": all(checks.values()), "checks": checks,
                       "failures": [key for key, value in checks.items() if value is not True]},
        }
    candidate_macro = sum(reports[route]["candidate"]["score_m"] for route in ROUTES) / 2.0
    gates = {
        "exact_two_routes_one_evaluation_each": accounting == {
            "candidate_paths_materialized": 2, "truth_paths_materialized": 2,
            "candidate_solution_reads": 2, "candidate_coordinate_interpretations": 2,
            "truth_reads": 2, "accuracy_calculations": 2,
        },
        "candidate_output_schema_and_exact_alignment": all(reports[route]["gates"]["checks"]["candidate_schema_and_exact_alignment"] for route in ROUTES),
        "candidate_prediction_domain_coverage_exact": all(reports[route]["candidate"].get("prediction_domain_coverage") == 1.0 for route in ROUTES),
        "candidate_all_finite_and_earth_valid": all(reports[route]["candidate"].get("finite") is True for route in ROUTES),
        "candidate_over_70_mps_count_zero": all(reports[route]["candidate"].get("over_70_mps_count") == 0 for route in ROUTES),
        "candidate_route_scores_finite": all(finite(reports[route]["candidate"].get("score_m")) for route in ROUTES),
        "candidate_each_route_score_at_most_3m": all(reports[route]["candidate"]["score_m"] <= 3.0 for route in ROUTES),
        "candidate_no_route_regression_vs_phase112": all(reports[route]["candidate"]["score_m"] <= float(baseline[route]) for route in ROUTES),
        "candidate_macro_score_strict_less_than_0_782m": strict_macro_gate(candidate_macro),
        "truth_read_only_by_one_evaluator_process": accounting["truth_reads"] == 2,
        "no_solution_rows_in_result": True,
        "no_solver_raw_base_tdcp_mat_precomputed_pdc_kaggle": True,
    }
    failed = [key for key, value in gates.items() if value is not True]
    result = {
        "schema_version": RESULT_SCHEMA, "phase": 134, "execution_label": "Luna Max",
        "status": "go-phase134-truth-only-accuracy" if not failed else "no-go-phase134-truth-only-accuracy",
        "decision": "truth-only accuracy evaluated; release remains separately unauthorized" if not failed else "truth-only accuracy gate failed closed; preserve artifacts and do not rerun",
        "candidate": CANDIDATE_ID, "accuracy_scored": True, "routes": reports,
        "aggregate": {
            "candidate_macro_score_m": candidate_macro,
            "phase112_same_pipeline_macro_m": manifest["comparison_baselines"]["phase112_same_pipeline"]["macro_m"],
            "phase118_champion_macro_m": manifest["comparison_baselines"]["phase118_champion"]["macro_m"],
            "phase120_macro_m": manifest["comparison_baselines"]["phase120"]["macro_m"],
            "route_count": 2, "macro_route_order": list(ROUTES),
            "macro_weighting": "unweighted arithmetic mean over exactly MTV-A then LAX-T",
        },
        "metric_contract": manifest["metric_contract"], "promotion_gates": gates,
        "strict_0_782_gate": {"comparator": "candidate_macro_score_m < 0.782", "threshold_m": STRICT_PROMOTION_THRESHOLD_M,
                               "candidate_macro_score_m": candidate_macro, "passed": gates["candidate_macro_score_strict_less_than_0_782m"]},
        "failed_gates": failed, "read_accounting": {
            "native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0, "raw_base_rinex_reads": 0,
            "candidate_solution_reads_for_hash_and_parse": accounting["candidate_solution_reads"],
            "candidate_coordinate_interpretations": accounting["candidate_coordinate_interpretations"],
            "truth_reads": accounting["truth_reads"], "truth_reads_per_route": 1,
            "accuracy_calculations": accounting["accuracy_calculations"],
            "candidate_paths_materialized_after_authorization": accounting["candidate_paths_materialized"],
            "truth_paths_materialized_after_authorization": accounting["truth_paths_materialized"],
            "reruns": 0, "fallbacks": 0, "mat_precomputed_phone_coordinate_pdc_reads": 0,
            "kaggle_or_token_access": 0, "truth_reads_by_process": "one Phase134 truth-only evaluator process",
        },
        "forbidden_lanes": {"native_solver_rerun": False, "raw_or_base_rerun": False, "mat": False,
                            "precomputed_phone_coordinates": False, "pdc": False, "kaggle_or_token": False,
                            "solution_rows_in_result": False},
        "solution_output_published": False, "release_or_submission_authorized": False,
    }
    atomic_json(result_path, result)
    atomic_text(result_path.with_suffix(".md"), "# Phase134 truth-only accuracy result\n\nCoordinates are omitted; release remains unauthorized.\n")
    return result


def main(argv: list[str] | None = None) -> int:
    import argparse

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
        print(f"phase134 truth-only evaluator: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
