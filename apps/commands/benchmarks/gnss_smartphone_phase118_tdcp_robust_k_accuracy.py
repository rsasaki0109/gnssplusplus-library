#!/usr/bin/env python3
"""Launch-free contract and truth-only evaluator for the sealed Phase118 output.

The verification path reads only tracked source and sealed metadata.  It does
not stat, hash, or open a candidate solution or an official truth file before
an independent authorization file has been supplied.  The later ``--evaluate``
path is deliberately the only path that materializes those two kinds of
payload paths; it reads one opaque candidate and one official truth file per
route, in the fixed MTV-A then LAX-T order, and writes metrics without rows.

No native solver is launched by this module.  The evaluator is therefore safe
to qualify and hash before the isolated truth-only authorization is created.
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
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_accuracy_gate_freeze_v1.json"
AUDIT = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_accuracy_gate_audit_v1.md"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_accuracy_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_accuracy_truth_authorization_v1.json"
STRUCTURAL_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_structural_result_v1.json"
PHASE112_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_accuracy_result_v1.json"
PHASE112_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_accuracy_manifest_v1.json"
PHASE112_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase112_main_output_offset_accuracy.py"
PHASE117_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_accuracy_manifest_v1.json"
PHASE117_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase117_tdcp_weighting_accuracy.py"
PHASE82_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase82_phase80_direct_quality_accuracy.py"
PHASE82_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase82_phase80_direct_quality_accuracy_result_v1.json"
PHASE76_PARSER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase76_phase75_accuracy_integrity_recovery.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase118_tdcp_robust_k_accuracy.py"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_accuracy_result_v1.json"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
TRUTH_ROWS = {ROUTES[0]: 2159, ROUTES[1]: 1465}
EXPECTED_MISSING = {
    ROUTES[0]: [ROUTES[0], 1615921153434],
    ROUTES[1]: None,
}
EARTH_RADIUS_M = 6371008.8
MAX_SPEED_MPS = 70.0
STRICT_PROMOTION_THRESHOLD_M = 0.782
CANDIDATE_ID = "phase118-official-tdcp-huber-k-mapping-truth-only-accuracy-v1"
RESULT_SCHEMA = "smartphone-r5-phase118-tdcp-robust-k-accuracy-result.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase118-tdcp-robust-k-accuracy-manifest.v1"
AUTHORIZATION_SCHEMA = "smartphone-r5-phase118-tdcp-robust-k-accuracy-truth-authorization.v1"

# These are immutable inputs to this contract.  In particular, the candidate
# and truth payload hashes below are metadata seals copied from the Phase118
# freeze; they are never used to touch a payload on the launch-free path.
FREEZE_COMMIT = "ef8d11bc4fa64e2ed6cd5fface133979655953ff"
FREEZE_SHA256 = "860a7f382199958f8dd2c43ba697d4c5cde1bc550d5747028e4c6f73be346760"
AUDIT_COMMIT = "2bcb67385bb1c4df0a750c4401084532c73b96fa"
AUDIT_SHA256 = "dc4ae55d365ef743b3958cecbceb54c6cf5d211a72d749059bda709e107532f6"
STRUCTURAL_COMMIT = "b4ea80d96d7156f9fd62cc554f2deeedcc550cb0"
STRUCTURAL_SHA256 = "88e8799050fd396eeb14e83d4f5923339279f46d0219a385e303d3cb50731538"
PHASE112_RESULT_SHA256 = "46d8d836758ac1a88b95ec2c1a0c0080cedfb753c12411ed74e4c2501549c57e"
PHASE112_MANIFEST_SHA256 = "29aac6bd43f56183142c84219a7924d6172733a73385a5725a9e3e3013566768"
PHASE112_EVALUATOR_SHA256 = "d34c7df1b48be999141013018a78f5b61e70f831afc2977871e0ac756741b6cc"
PHASE117_MANIFEST_SHA256 = "cbba73a941d2cd46c5d9a6cfacfa55404c4e716b4c05191fa5d5e8fd3140789a"
PHASE117_EVALUATOR_SHA256 = "88e8020052a0bdaed3f7262dedc46600f6608a84a7bcdabe82ac009b8db82ebf"
PHASE82_EVALUATOR_SHA256 = "232ba0ab1fa7ebde6ddfaaf851d190d1ed32377ea056ab6c31e244139c2977fb"
PHASE82_RESULT_SHA256 = "39c5a38b7e3e61fda0306582fffb961dad3e6fb0bd49fcfe70e66674e2c90873"
PHASE76_PARSER_SHA256 = "513e5970ee58846963f077b9960944e5fbf5f3535e5be24ca6e72493785f7761"


class Phase118AccuracyError(ValueError):
    """Raised when the immutable Phase118 truth-only contract fails closed."""


def fail(message: str) -> Phase118AccuracyError:
    return Phase118AccuracyError(message)


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
    name = path.name
    return name == "ground_truth.csv" or name == "withheld_solution_output.csv" or name in {
        "device_gnss.csv", "device_imu.csv", "brdc.nav", "base.obs",
    }


def sha256_file(path: Path, label: str) -> str:
    """Hash only an authorized contract/source/metadata file.

    The payload guard is intentionally before ``is_file`` so pre-truth
    verification cannot even probe candidate, truth, or raw input members.
    """
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
    """Validate a path string without resolving or probing it on disk."""
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


def _verify_metric_contract(metric: Any, label: str) -> None:
    if not isinstance(metric, dict):
        raise fail(f"{label} metric contract missing")
    expected = {
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
    for key, value in expected.items():
        assert_equal(metric.get(key), value, f"{label}/{key}")


def _verify_zero_accounting(accounting: Any, label: str) -> None:
    if not isinstance(accounting, dict):
        raise fail(f"{label} read accounting missing")
    for key in (
        "native_solver_invocations",
        "raw_gnss_imu_navigation_reads",
        "raw_base_rinex_reads",
        "truth_reads",
        "candidate_solution_payload_reads",
        "candidate_coordinate_interpretations",
        "accuracy_calculations",
        "mat_precomputed_phone_coordinate_pdc_reads",
        "kaggle_or_token_access",
        "reruns",
        "fallbacks",
    ):
        assert_equal(accounting.get(key), 0, f"{label}/{key}")


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_file(FREEZE, "Phase118 accuracy freeze"), FREEZE_SHA256, "freeze/sha256")
    freeze = read_json(FREEZE, "Phase118 accuracy freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase118-tdcp-robust-k-accuracy-gate-freeze.v1",
        "phase": 118,
        "status": "frozen-before-phase118-truth-only-accuracy-evaluation",
        "execution_label": "Luna Max",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")

    authority = freeze.get("authority")
    if not isinstance(authority, dict):
        raise fail("freeze/authority missing")
    _pin(authority, "accuracy_audit", AUDIT, AUDIT_SHA256)
    assert_equal(sha256_file(AUDIT, "Phase118 accuracy audit"), AUDIT_SHA256, "audit/sha256")
    structural_pin = authority.get("phase118_structural_result")
    if not isinstance(structural_pin, dict):
        raise fail("freeze structural result authority missing")
    assert_equal(structural_pin.get("commit"), STRUCTURAL_COMMIT, "freeze/structural/commit")
    assert_equal(structural_pin.get("path"), relative(STRUCTURAL_RESULT), "freeze/structural/path")
    assert_equal(structural_pin.get("sha256"), STRUCTURAL_SHA256, "freeze/structural/sha256")

    candidate = freeze.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID,
        "candidate_count": 1,
        "routes": list(ROUTES),
        "route_order_fixed": True,
        "runs_per_route": 1,
        "solver_rerun": False,
        "raw_or_base_rerun": False,
        "tdcp_rerun": False,
        "solution_repaired_or_replaced": False,
        "solution_publication": False,
        "metadata_hash_seal_precedes_truth": True,
        "coordinate_interpretation_before_truth": False,
        "candidate_files_copied_or_transformed": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    offset = candidate.get("phase118_offset_boundary")
    if not isinstance(offset, dict):
        raise fail("freeze/candidate/phase118_offset_boundary missing")
    for key, expected in {
        "required_already_applied_in_structural_run": True,
        "evaluator_must_not_apply_again": True,
        "double_application": "fail-closed",
        "selector": "--native-upstream-position-offset",
    }.items():
        assert_equal(offset.get(key), expected, f"freeze/offset/{key}")
    for route in ROUTES:
        metadata = candidate.get("routes_metadata", {}).get(route)
        if not isinstance(metadata, dict):
            raise fail(f"freeze candidate metadata missing: {route}")
        _relative_metadata(metadata.get("path"), "withheld_solution_output.csv", f"candidate {route}", "/phase118-tdcp-robust-k-v1/")
        assert_equal(metadata.get("expected_prediction_rows"), DOMAIN_ROWS[route], f"candidate rows/{route}")
        assert_equal(metadata.get("expected_problem_epochs"), DOMAIN_ROWS[route] + 1, f"candidate epochs/{route}")
        if not metadata.get("opaque_metadata_seal") or not isinstance(metadata.get("sha256"), str) or len(metadata["sha256"]) != 64:
            raise fail(f"candidate opaque seal malformed: {route}")

    truth = freeze.get("truth_cohort")
    if not isinstance(truth, dict):
        raise fail("freeze/truth_cohort missing")
    for key, expected in {
        "route_order": list(ROUTES),
        "read_by_solver": False,
        "read_by_audit": False,
        "read_by_freeze": False,
        "read_by_manifest": False,
        "read_by_evaluator_only": True,
    }.items():
        assert_equal(truth.get(key), expected, f"freeze/truth_cohort/{key}")
    for route in ROUTES:
        item = truth.get("routes", {}).get(route)
        if not isinstance(item, dict):
            raise fail(f"freeze truth metadata missing: {route}")
        _relative_metadata(item.get("path"), "ground_truth.csv", f"truth {route}", "/truth/")
        assert_equal(item.get("rows"), TRUTH_ROWS[route], f"truth rows/{route}")
        assert_equal(item.get("expected_missing_truth_key"), EXPECTED_MISSING[route], f"truth missing key/{route}")
        if not isinstance(item.get("sha256"), str) or len(item["sha256"]) != 64 or not isinstance(item.get("bytes"), int):
            raise fail(f"truth metadata seal malformed: {route}")

    _verify_metric_contract(freeze.get("metric_contract"), "freeze/metric_contract")
    baselines = freeze.get("comparison_baselines")
    if not isinstance(baselines, dict):
        raise fail("freeze comparison baselines missing")
    phase112 = baselines.get("phase112_same_pipeline")
    if not isinstance(phase112, dict):
        raise fail("Phase112 baseline missing")
    assert_equal(phase112.get("macro_m"), 0.8318381724000121, "Phase112 baseline/macro")
    assert_equal(phase112.get("route_scores_m"), {
        ROUTES[0]: 0.9976852530,
        ROUTES[1]: 0.6659910918,
    }, "Phase112 baseline/routes")
    phase82 = baselines.get("phase82_same_route")
    if not isinstance(phase82, dict):
        raise fail("Phase82 baseline missing")
    assert_equal(phase82.get("macro_m"), 1.026451431707709, "Phase82 baseline/macro")

    gates = freeze.get("promotion_gates")
    if not isinstance(gates, dict):
        raise fail("freeze promotion gates missing")
    for key in (
        "all_gates_anded",
        "exact_two_existing_routes_one_evaluation_each",
        "candidate_output_schema_and_exact_alignment",
        "candidate_all_finite_and_earth_valid",
        "candidate_over_70_mps_count_zero",
        "candidate_no_route_regression_vs_phase112",
        "candidate_no_route_regression_vs_phase82_same_route",
        "strict_less_than_is_explicit_and_gate",
        "pixel5_offset_already_exactly_once_no_evaluator_reapplication",
        "truth_read_only_by_one_evaluator_subprocess",
        "solution_rows_absent_from_result",
        "no_solver_raw_base_tdcp_mat_precomputed_phone_coordinates_pdc_or_kaggle",
    ):
        assert_equal(gates.get(key), True, f"freeze/promotion_gates/{key}")
    assert_equal(gates.get("candidate_macro_score_strict_less_than_m"), STRICT_PROMOTION_THRESHOLD_M, "freeze/promotion_gates/threshold")

    leakage = freeze.get("leakage_guard")
    if not isinstance(leakage, dict):
        raise fail("freeze leakage guard missing")
    for key in (
        "native_solver_rerun", "raw_gnss_imu_navigation_rerun", "raw_base_rinex_rerun",
        "mat_reads_or_generated", "precomputed_phone_coordinate_reads", "pdc_reads",
        "kaggle_or_token_access", "truth_reads_before_authorization", "accuracy_selection_before_truth",
        "post_truth_tuning", "solution_publication", "offset_reapplication",
    ):
        assert_equal(leakage.get(key), False, f"freeze/leakage_guard/{key}")

    accounting = freeze.get("read_accounting", {})
    _verify_zero_accounting(accounting.get("audit_before_freeze"), "freeze/audit_before_freeze")
    planned = accounting.get("planned_truth_evaluation")
    if not isinstance(planned, dict):
        raise fail("freeze planned truth accounting missing")
    for key, expected in {
        "native_solver_invocations": 0,
        "raw_gnss_imu_navigation_reads": 0,
        "raw_base_rinex_reads": 0,
        "candidate_solution_reads_for_hash_and_parse": 2,
        "truth_reads": 2,
        "truth_reads_per_route": 1,
        "accuracy_calculations": 2,
        "reruns": 0,
        "fallbacks": 0,
        "mat_precomputed_phone_coordinate_pdc_reads": 0,
        "kaggle_or_token_access": 0,
    }.items():
        assert_equal(planned.get(key), expected, f"freeze/planned_truth_evaluation/{key}")

    boundary = freeze.get("truth_evaluator_boundary")
    if not isinstance(boundary, dict):
        raise fail("freeze truth evaluator boundary missing")
    for key, expected in {
        "evaluator_process_count": 1,
        "candidate_metadata_hash_seal_precedes_truth": True,
        "candidate_parse_precedes_truth": True,
        "truth_reads": 2,
        "truth_reads_per_route": 1,
        "native_after_truth": False,
        "truth_path_or_bytes_in_native": False,
        "release_or_submission": False,
    }.items():
        assert_equal(boundary.get(key), expected, f"freeze/truth_evaluator_boundary/{key}")
    return freeze


def verify_structural_result(freeze: dict[str, Any] | None = None) -> dict[str, Any]:
    if freeze is None:
        freeze = verify_freeze()
    assert_equal(sha256_file(STRUCTURAL_RESULT, "Phase118 structural result"), STRUCTURAL_SHA256, "structural/sha256")
    result = read_json(STRUCTURAL_RESULT, "Phase118 structural result")
    for key, expected in {
        "schema_version": "smartphone-r5-phase118-tdcp-robust-k-structural-result.v1",
        "phase": 118,
        "status": "go-phase118-official-tdcp-huber-k-structural",
        "accuracy_scored": False,
        "solution_output_published": False,
        "truth_free": True,
        "promotion_authorized": False,
    }.items():
        assert_equal(result.get(key), expected, f"structural/{key}")
    if result.get("read_accounting", {}).get("truth_reads") != 0:
        raise fail("Phase118 structural result already read truth")
    gates = result.get("gates")
    if not isinstance(gates, dict) or gates.get("all_structural_gates_passed") is not True or gates.get("route_order_exact") is not True:
        raise fail("Phase118 structural result is not structural GO")
    routes = result.get("routes")
    if not isinstance(routes, dict) or set(routes) != set(ROUTES):
        raise fail("Phase118 structural route set changed")
    for route in ROUTES:
        item = routes[route]
        if not isinstance(item, dict) or item.get("return_code") != 0 or item.get("truth_used") is not False or item.get("solution_output_published") is not False:
            raise fail(f"Phase118 structural route policy changed: {route}")
        if item.get("accuracy_scored") is not False:
            raise fail(f"Phase118 structural route was accuracy scored: {route}")
        seal = item.get("solution_hash_seal")
        expected = freeze["candidate"]["routes_metadata"][route]
        if not isinstance(seal, dict):
            raise fail(f"structural candidate seal missing: {route}")
        for key, expected_value in {
            "path": expected["path"],
            "sha256": expected["sha256"],
            "bytes": expected["bytes"],
            "rows": expected["expected_prediction_rows"],
            "published": False,
            "coordinate_rows_omitted": True,
            "row_count_matches_domain": True,
        }.items():
            assert_equal(seal.get(key), expected_value, f"structural/candidate_seal/{route}/{key}")
        _relative_metadata(seal.get("path"), "withheld_solution_output.csv", f"structural candidate {route}", "/phase118-tdcp-robust-k-v1/")
        offset = item.get("telemetry", {}).get("upstream_position_offset")
        if not isinstance(offset, dict) or offset.get("applied") is not True:
            raise fail(f"Pixel5 offset boundary changed: {route}")
    return result


def _verify_sealed_reference(path: Path, expected: str, label: str) -> None:
    assert_equal(sha256_file(path, label), expected, f"{label}/sha256")


def verify_manifest() -> dict[str, Any]:
    freeze = verify_freeze()
    structural = verify_structural_result(freeze)
    for path, expected, label in (
        (PHASE112_RESULT, PHASE112_RESULT_SHA256, "Phase112 accuracy aggregate"),
        (PHASE112_MANIFEST, PHASE112_MANIFEST_SHA256, "Phase112 accuracy manifest"),
        (PHASE112_EVALUATOR, PHASE112_EVALUATOR_SHA256, "Phase112 accuracy evaluator"),
        (PHASE117_MANIFEST, PHASE117_MANIFEST_SHA256, "Phase117 accuracy manifest"),
        (PHASE117_EVALUATOR, PHASE117_EVALUATOR_SHA256, "Phase117 accuracy evaluator"),
        (PHASE82_EVALUATOR, PHASE82_EVALUATOR_SHA256, "Phase82 metric evaluator"),
        (PHASE82_RESULT, PHASE82_RESULT_SHA256, "Phase82 accuracy aggregate"),
        (PHASE76_PARSER, PHASE76_PARSER_SHA256, "Phase76 parser"),
    ):
        _verify_sealed_reference(path, expected, label)

    manifest = read_json(MANIFEST, "Phase118 truth accuracy manifest")
    for key, expected in {
        "schema_version": MANIFEST_SCHEMA,
        "phase": 118,
        "execution_label": "Luna Max",
        "status": "sealed-before-phase118-truth-only-accuracy-evaluation",
    }.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    assert_equal(manifest.get("routes"), list(ROUTES), "manifest/routes")
    authority = manifest.get("authority")
    if not isinstance(authority, dict):
        raise fail("manifest authority missing")
    for key, path, expected in (
        ("accuracy_freeze", FREEZE, FREEZE_SHA256),
        ("accuracy_audit", AUDIT, AUDIT_SHA256),
        ("phase118_structural_result", STRUCTURAL_RESULT, STRUCTURAL_SHA256),
        ("phase112_accuracy_result", PHASE112_RESULT, PHASE112_RESULT_SHA256),
        ("phase112_accuracy_manifest", PHASE112_MANIFEST, PHASE112_MANIFEST_SHA256),
        ("phase112_accuracy_evaluator", PHASE112_EVALUATOR, PHASE112_EVALUATOR_SHA256),
        ("phase117_accuracy_manifest", PHASE117_MANIFEST, PHASE117_MANIFEST_SHA256),
        ("phase117_accuracy_evaluator", PHASE117_EVALUATOR, PHASE117_EVALUATOR_SHA256),
        ("phase82_metric_reference", PHASE82_EVALUATOR, PHASE82_EVALUATOR_SHA256),
        ("phase82_result", PHASE82_RESULT, PHASE82_RESULT_SHA256),
        ("phase76_parser_reference", PHASE76_PARSER, PHASE76_PARSER_SHA256),
    ):
        _pin(authority, key, path, expected)
        _verify_sealed_reference(path, expected, key)
    for key, path in (("evaluator", EVALUATOR), ("focused_tests", FOCUSED_TESTS)):
        _pin(authority, key, path, sha256_file(path, f"manifest/{key}"))

    for key, expected in {
        "path": relative(FREEZE),
        "commit": FREEZE_COMMIT,
        "sha256": FREEZE_SHA256,
    }.items():
        assert_equal(manifest.get("freeze", {}).get(key), expected, f"manifest/freeze/{key}")
    assert_equal(manifest.get("structural_result", {}).get("commit"), STRUCTURAL_COMMIT, "manifest/structural_result/commit")
    assert_equal(manifest.get("structural_result", {}).get("sha256"), STRUCTURAL_SHA256, "manifest/structural_result/sha256")

    candidate = manifest.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("manifest candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID,
        "candidate_count": 1,
        "route_order": list(ROUTES),
        "route_order_fixed": True,
        "runs_per_route": 1,
        "solver_rerun": False,
        "raw_or_base_rerun": False,
        "tdcp_rerun": False,
        "solution_repaired_or_replaced": False,
        "solution_publication": False,
        "metadata_hash_seal_precedes_truth": True,
        "coordinate_interpretation_before_truth": False,
        "candidate_files_copied_or_transformed": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"manifest/candidate/{key}")
    assert_equal(candidate.get("phase118_offset_boundary"), freeze["candidate"]["phase118_offset_boundary"], "manifest/candidate/offset_boundary")
    for route in ROUTES:
        expected = freeze["candidate"]["routes_metadata"][route]
        item = candidate.get("routes_metadata", {}).get(route)
        if not isinstance(item, dict):
            raise fail(f"manifest candidate metadata missing: {route}")
        for key in ("path", "bytes", "sha256", "expected_prediction_rows", "expected_problem_epochs", "opaque_metadata_seal"):
            assert_equal(item.get(key), expected.get(key), f"manifest/candidate/{route}/{key}")
        _relative_metadata(item.get("path"), "withheld_solution_output.csv", f"manifest candidate {route}", "/phase118-tdcp-robust-k-v1/")

    truth = manifest.get("truth_cohort")
    if not isinstance(truth, dict):
        raise fail("manifest truth cohort missing")
    assert_equal(truth, freeze["truth_cohort"], "manifest/truth_cohort")
    for route in ROUTES:
        _relative_metadata(truth["routes"][route]["path"], "ground_truth.csv", f"manifest truth {route}", "/truth/")

    _verify_metric_contract(manifest.get("metric_contract"), "manifest/metric_contract")
    assert_equal(manifest.get("metric_contract"), freeze["metric_contract"], "manifest/metric_contract/parity")
    assert_equal(manifest.get("comparison_baselines"), freeze["comparison_baselines"], "manifest/comparison_baselines")
    gates = manifest.get("promotion_gates")
    if not isinstance(gates, dict):
        raise fail("manifest promotion gates missing")
    for key, expected in {
        "strict_macro_comparator": "candidate_macro_score_m < 0.782",
        "strict_macro_threshold_m": STRICT_PROMOTION_THRESHOLD_M,
        "equality_at_threshold_passes": False,
        "exact_route_order": list(ROUTES),
        "route_evaluations": 2,
        "truth_reads_per_route": 1,
        "no_repair_or_rerun_or_fallback": True,
        "pixel5_offset_already_applied_exactly_once": True,
        "evaluator_must_not_apply_offset": True,
        "solution_rows_in_result": False,
    }.items():
        assert_equal(gates.get(key), expected, f"manifest/promotion_gates/{key}")

    accounting = manifest.get("read_accounting_before_authorization")
    _verify_zero_accounting(accounting, "manifest/read_accounting_before_authorization")
    for key, expected in {
        "candidate_paths_materialized": 0,
        "truth_paths_materialized": 0,
        "native_solver_invocations": 0,
        "raw_gnss_imu_navigation_reads": 0,
        "raw_base_rinex_reads": 0,
        "truth_reads": 0,
        "candidate_solution_payload_reads": 0,
        "accuracy_calculations": 0,
        "mat_precomputed_phone_coordinate_pdc_reads": 0,
        "kaggle_or_token_access": 0,
        "reruns": 0,
        "fallbacks": 0,
    }.items():
        assert_equal(accounting.get(key), expected, f"manifest/read_accounting_before_authorization/{key}")

    boundary = manifest.get("truth_evaluator_boundary")
    if not isinstance(boundary, dict):
        raise fail("manifest truth evaluator boundary missing")
    for key, expected in {
        "authorization_required_before_path_materialization": True,
        "candidate_metadata_hash_seal_precedes_truth": True,
        "candidate_parse_precedes_truth": True,
        "evaluator_process_count": 1,
        "candidate_solution_reads_for_hash_and_parse": 2,
        "truth_reads": 2,
        "truth_reads_per_route": 1,
        "accuracy_calculations": 2,
        "native_solver_invocations": 0,
        "native_after_truth": False,
        "no_solution_rows_in_result": True,
        "no_truth_coordinate_rows_in_result": True,
        "release_or_submission": False,
    }.items():
        assert_equal(boundary.get(key), expected, f"manifest/truth_evaluator_boundary/{key}")
    for key in (
        "native_solver_rerun", "raw_gnss_imu_navigation_rerun", "raw_base_rinex_rerun",
        "mat_reads_or_generated", "precomputed_phone_coordinate_reads", "pdc_reads",
        "kaggle_or_token_access", "truth_reads_before_authorization", "solution_publication",
        "offset_reapplication",
    ):
        assert_equal(manifest.get("leakage_guard", {}).get(key), False, f"manifest/leakage_guard/{key}")
    return manifest


def verify_pre_truth() -> dict[str, Any]:
    """Verify the contract without materializing candidate/truth payload paths."""
    manifest = verify_manifest()
    return {
        "status": "pre-truth-verified",
        "phase": 118,
        "execution_label": "Luna Max",
        "route_order": list(ROUTES),
        "candidate_count": manifest["candidate"]["candidate_count"],
        "candidate_paths_materialized": 0,
        "truth_paths_materialized": 0,
        "native_solver_invocations": 0,
        "raw_gnss_imu_navigation_reads": 0,
        "raw_base_rinex_reads": 0,
        "truth_reads": 0,
        "candidate_solution_reads": 0,
        "candidate_coordinate_interpretations": 0,
        "accuracy_calculations": 0,
        "mat_precomputed_phone_coordinate_pdc_reads": 0,
        "kaggle_or_token_access": 0,
        "reruns": 0,
        "fallbacks": 0,
        "raw_execution_or_solver_authorized": False,
        "truth_only_authorized": False,
        "solution_output_published": False,
    }


def _verify_pin(container: dict[str, Any], key: str, path: Path, expected: str) -> None:
    _pin(container, key, path, expected)
    _verify_sealed_reference(path, expected, f"authorization/{key}")


def verify_authorization() -> dict[str, Any]:
    """Verify an independently-created auth file without reading payloads."""
    manifest = verify_manifest()
    auth = read_json(AUTHORIZATION, "Phase118 truth-only authorization")
    for key, expected in {
        "schema_version": AUTHORIZATION_SCHEMA,
        "phase": 118,
        "execution_label": "Luna Max",
        "status": "authorized-for-phase118-truth-only-accuracy-evaluation",
    }.items():
        assert_equal(auth.get(key), expected, f"authorization/{key}")
    authority = auth.get("authority")
    if not isinstance(authority, dict):
        raise fail("authorization authority missing")
    _verify_pin(authority, "accuracy_freeze", FREEZE, FREEZE_SHA256)
    _verify_pin(authority, "accuracy_manifest", MANIFEST, sha256_file(MANIFEST, "Phase118 accuracy manifest"))
    _verify_pin(authority, "accuracy_evaluator", EVALUATOR, sha256_file(EVALUATOR, "Phase118 accuracy evaluator"))
    _verify_pin(authority, "focused_tests", FOCUSED_TESTS, sha256_file(FOCUSED_TESTS, "Phase118 focused tests"))
    _verify_pin(authority, "phase118_structural_result", STRUCTURAL_RESULT, STRUCTURAL_SHA256)
    _verify_pin(authority, "phase112_accuracy_result", PHASE112_RESULT, PHASE112_RESULT_SHA256)
    _verify_pin(authority, "phase82_metric_reference", PHASE82_EVALUATOR, PHASE82_EVALUATOR_SHA256)
    _verify_pin(authority, "phase82_result", PHASE82_RESULT, PHASE82_RESULT_SHA256)
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
    assert_equal(auth.get("manifest_sha256"), sha256_file(MANIFEST, "Phase118 accuracy manifest"), "authorization/manifest_sha256")
    return auth


def load_phase82() -> Any:
    """Load only the pinned scorer source; no dataset file is opened here."""
    _verify_sealed_reference(PHASE82_EVALUATOR, PHASE82_EVALUATOR_SHA256, "Phase82 metric evaluator")
    spec = importlib.util.spec_from_file_location("phase82_metric_reference_phase118", PHASE82_EVALUATOR)
    if spec is None or spec.loader is None:
        raise fail("unable to load Phase82 metric evaluator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def materialize_candidate_path(structural: dict[str, Any], route: str, *, authorized: bool) -> Path:
    """Create a candidate payload path only after auth verification succeeds."""
    if not authorized:
        raise fail("candidate path materialization requires independent truth authorization")
    if route not in ROUTES:
        raise fail(f"unknown route: {route}")
    text = structural["routes"][route]["solution_hash_seal"]["path"]
    _relative_metadata(text, "withheld_solution_output.csv", f"candidate {route}", "/phase118-tdcp-robust-k-v1/")
    return ROOT / text


def materialize_truth_path(freeze: dict[str, Any], route: str, *, authorized: bool) -> Path:
    """Create an official truth payload path only after auth verification."""
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
        raise fail(f"Phase118 candidate opaque seal mismatch: {route}")
    try:
        ordered, mapping = p82.P76.P74._parse_submission(payload, route)
    except Exception as exc:
        raise fail(f"Phase118 candidate schema failed: {route}: {exc}") from exc
    if len(ordered) != DOMAIN_ROWS[route] or len(mapping) != len(ordered) or [item[0] for item in ordered] != sorted(item[0] for item in ordered):
        raise fail(f"Phase118 candidate exact alignment failed: {route}")
    if any(not all(math.isfinite(float(value)) for value in item[1:]) or not -90.0 <= item[1] <= 90.0 or not -180.0 <= item[2] <= 180.0 for item in ordered):
        raise fail(f"Phase118 candidate finite/Earth-valid preflight failed: {route}")
    return ordered, mapping, {
        "path": relative(path),
        "bytes": len(payload),
        "sha256": digest,
        "header": "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees",
        "rows": len(ordered),
        "read_count": 1,
        "coordinate_rows_omitted": True,
    }


def read_truth_once(path: Path, pin: dict[str, Any], route: str, p82: Any) -> tuple[dict[int, tuple[float, float]], dict[str, Any]]:
    payload = path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != pin.get("sha256") or len(payload) != pin.get("bytes"):
        raise fail(f"official truth seal mismatch: {route}")
    try:
        truth = p82.P76._parse_truth_dictreader(payload, route)
    except Exception as exc:
        raise fail(f"official truth schema failed: {route}: {exc}") from exc
    if len(truth) != pin.get("rows"):
        raise fail(f"official truth row count mismatch: {route}")
    return truth, {
        "path": relative(path),
        "bytes": len(payload),
        "sha256": digest,
        "rows": len(truth),
        "read_count": 1,
        "coordinate_rows_omitted": True,
    }


def sealed_baselines() -> dict[str, Any]:
    """Read sealed aggregate metadata only; never derive a new accuracy value."""
    phase112 = read_json(PHASE112_RESULT, "Phase112 sealed accuracy aggregate")
    _verify_sealed_reference(PHASE112_RESULT, PHASE112_RESULT_SHA256, "Phase112 sealed accuracy aggregate")
    phase82 = read_json(PHASE82_RESULT, "Phase82 sealed accuracy aggregate")
    _verify_sealed_reference(PHASE82_RESULT, PHASE82_RESULT_SHA256, "Phase82 sealed accuracy aggregate")
    try:
        phase112_routes = {route: float(phase112["routes"][route]["candidate"]["score_m"]) for route in ROUTES}
        phase112_macro = float(phase112["aggregate"]["candidate_macro_score_m"])
        phase82_macro = float(phase82["aggregate"]["candidate_macro_score_m"])
    except (KeyError, TypeError, ValueError) as exc:
        raise fail(f"sealed baseline aggregate malformed: {exc}") from exc
    assert_equal(phase112_routes, {ROUTES[0]: 0.997685253035948, ROUTES[1]: 0.6659910917640763}, "Phase112 sealed route scores")
    assert_equal(phase112_macro, 0.8318381724000121, "Phase112 sealed macro")
    assert_equal(phase82_macro, 1.7643320853515223, "Phase82 sealed aggregate candidate macro")
    return {"phase112": phase112_routes, "phase112_macro": phase112_macro, "phase82_macro": phase82_macro}


def strict_macro_gate(candidate_macro_score_m: Any) -> bool:
    """The promotion comparison is intentionally strict: equality fails."""
    return finite(candidate_macro_score_m) and float(candidate_macro_score_m) < STRICT_PROMOTION_THRESHOLD_M


def route_report(route: str, structural: dict[str, Any], freeze: dict[str, Any], p82: Any, baselines: dict[str, Any], accounting: dict[str, int]) -> dict[str, Any]:
    report: dict[str, Any] = {"dataset_id": route, "truth_read": False, "accuracy_scored": False}
    try:
        seal = structural["routes"][route]["solution_hash_seal"]
        # ``authorized=True`` is reachable only from evaluate(), immediately
        # after verify_authorization() has returned successfully.
        candidate_file = materialize_candidate_path(structural, route, authorized=True)
        accounting["candidate_paths_materialized"] += 1
        ordered, candidate, candidate_meta = read_candidate_once(candidate_file, seal, route, p82)
        accounting["candidate_solution_reads"] += 1
        accounting["candidate_coordinate_interpretations"] += 1
        report["candidate_solution"] = candidate_meta
        truth_file = materialize_truth_path(freeze, route, authorized=True)
        accounting["truth_paths_materialized"] += 1
        truth, truth_meta = read_truth_once(truth_file, freeze["truth_cohort"]["routes"][route], route, p82)
        accounting["truth_reads"] += 1
        report["truth"] = truth_meta
        report["truth_read"] = True
        score = p82.P76._score_prediction(candidate, truth, EXPECTED_MISSING[route], route, ordered)
        accounting["accuracy_calculations"] += 1
        phase112_score = baselines["phase112"][route]
        checks = {
            "candidate_schema_and_exact_alignment": len(ordered) == DOMAIN_ROWS[route],
            "candidate_prediction_domain_coverage_exact": score.get("prediction_domain_coverage") == 1.0,
            "candidate_finite_and_earth_valid": score.get("finite") is True,
            "candidate_over_70_mps_count_zero": score.get("over_70_mps_count") == 0,
            "candidate_route_score_finite": finite(score.get("score_m")),
            "candidate_route_score_at_most_3m": finite(score.get("score_m")) and score.get("score_m") <= 3.0,
            "candidate_no_route_regression_vs_phase112": finite(score.get("score_m")) and score.get("score_m") <= phase112_score,
        }
        report.update({
            "accuracy_scored": True,
            "candidate": score,
            "baseline_phase112_same_pipeline": {"score_m": phase112_score},
            "gates": {"passed": all(checks.values()), "checks": checks, "failures": [key for key, value in checks.items() if value is not True]},
        })
    except Exception as exc:
        report["failure"] = str(exc)
        report.setdefault("gates", {"passed": False, "checks": {}, "failures": ["route_evaluation"]})
    return report


def render_markdown(result: dict[str, Any]) -> str:
    aggregate = result.get("aggregate", {})
    lines = [
        "# Phase118 official TDCP Huber-k truth-only accuracy result",
        "",
        f"- status: `{result.get('status')}`",
        "- Native solver/raw/base/TDCP: not rerun; immutable Phase118 solutions reused",
        "- Truth: one official file read per route by the authorized evaluator process",
        "- Candidate/truth coordinate rows: omitted from this result",
        "",
        f"- Candidate macro: `{aggregate.get('candidate_macro_score_m')}` m",
        f"- Phase112 same-pipeline macro: `{aggregate.get('phase112_same_pipeline_macro_m')}` m",
        f"- Strict `<0.782 m` gate: `{result.get('strict_0_782_gate', {}).get('passed')}`",
        "",
        "| Route | Candidate (m) | Phase112 (m) | Truth read | Gates |",
        "|---|---:|---:|---:|---|",
    ]
    for route in ROUTES:
        item = result.get("routes", {}).get(route, {})
        lines.append(
            f"| `{route}` | `{item.get('candidate', {}).get('score_m')}` | "
            f"`{item.get('baseline_phase112_same_pipeline', {}).get('score_m')}` | "
            f"`{item.get('truth_read')}` | `{item.get('gates', {}).get('passed')}` |"
        )
    lines.extend(["", "Accuracy GO would not authorize release, validation, or Kaggle submission.", ""])
    return "\n".join(lines)


def evaluate(result_path: Path = RESULT_JSON) -> dict[str, Any]:
    """Run exactly the authorized two-route truth-only evaluation."""
    manifest = verify_manifest()
    verify_authorization()
    freeze = verify_freeze()
    structural = verify_structural_result(freeze)
    p82 = load_phase82()
    baselines = sealed_baselines()
    accounting = {
        "candidate_paths_materialized": 0,
        "truth_paths_materialized": 0,
        "candidate_solution_reads": 0,
        "candidate_coordinate_interpretations": 0,
        "truth_reads": 0,
        "accuracy_calculations": 0,
    }
    reports = {route: route_report(route, structural, freeze, p82, baselines, accounting) for route in ROUTES}
    scored = [reports[route] for route in ROUTES if reports[route].get("accuracy_scored") is True]
    candidate_macro = sum(item["candidate"]["score_m"] for item in scored) / 2.0 if len(scored) == len(ROUTES) else None
    route_gate_passed = len(scored) == len(ROUTES) and all(reports[route].get("gates", {}).get("passed") is True for route in ROUTES)
    gates = {
        "exact_two_routes_one_evaluation_each": len(scored) == 2 and accounting["candidate_solution_reads"] == 2 and accounting["truth_reads"] == 2 and accounting["accuracy_calculations"] == 2,
        "candidate_output_schema_and_exact_alignment": route_gate_passed and all(reports[route]["gates"]["checks"].get("candidate_schema_and_exact_alignment") is True for route in ROUTES),
        "candidate_prediction_domain_coverage_exact": route_gate_passed and all(reports[route]["candidate"].get("prediction_domain_coverage") == 1.0 for route in ROUTES),
        "candidate_all_finite_and_earth_valid": route_gate_passed and all(reports[route]["candidate"].get("finite") is True for route in ROUTES),
        "candidate_over_70_mps_count_zero": route_gate_passed and all(reports[route]["candidate"].get("over_70_mps_count") == 0 for route in ROUTES),
        "candidate_route_scores_finite": route_gate_passed and all(finite(reports[route]["candidate"].get("score_m")) for route in ROUTES),
        "candidate_each_route_score_at_most_3m": route_gate_passed and all(reports[route]["candidate"].get("score_m", math.inf) <= 3.0 for route in ROUTES),
        "candidate_no_route_regression_vs_phase112": route_gate_passed and all(reports[route]["candidate"]["score_m"] <= baselines["phase112"][route] for route in ROUTES),
        "candidate_macro_score_strict_less_than_0_782m": strict_macro_gate(candidate_macro),
        "truth_read_only_by_one_evaluator_process": accounting["truth_reads"] == 2 and all(reports[route].get("truth_read") is True for route in ROUTES),
        "no_solution_rows_in_result": True,
        "no_solver_raw_base_tdcp_mat_precomputed_pdc_kaggle": True,
    }
    failed = [key for key, value in gates.items() if value is not True]
    result = {
        "schema_version": RESULT_SCHEMA,
        "phase": 118,
        "execution_label": "Luna Max",
        "status": "go-phase118-tdcp-robust-k-truth-only-accuracy" if not failed else "no-go-phase118-tdcp-robust-k-truth-only-accuracy",
        "decision": "truth-only accuracy evaluated; release remains separately unauthorized" if not failed else "truth-only accuracy gate failed closed; preserve artifacts and do not rerun",
        "candidate": CANDIDATE_ID,
        "accuracy_scored": len(scored) == 2,
        "routes": reports,
        "aggregate": {
            "candidate_macro_score_m": candidate_macro,
            "phase112_same_pipeline_macro_m": baselines["phase112_macro"],
            "phase82_same_route_macro_m": baselines["phase82_macro"],
            "route_count": 2,
            "macro_route_order": list(ROUTES),
            "macro_weighting": "unweighted arithmetic mean over exactly MTV-A then LAX-T",
        },
        "metric_contract": manifest["metric_contract"],
        "promotion_gates": gates,
        "strict_0_782_gate": {
            "comparator": "candidate_macro_score_m < 0.782",
            "threshold_m": STRICT_PROMOTION_THRESHOLD_M,
            "candidate_macro_score_m": candidate_macro,
            "passed": gates["candidate_macro_score_strict_less_than_0_782m"],
        },
        "failed_gates": failed,
        "read_accounting": {
            "native_solver_invocations": 0,
            "raw_gnss_imu_navigation_reads": 0,
            "raw_base_rinex_reads": 0,
            "candidate_solution_reads_for_hash_and_parse": accounting["candidate_solution_reads"],
            "candidate_coordinate_interpretations": accounting["candidate_coordinate_interpretations"],
            "truth_reads": accounting["truth_reads"],
            "truth_reads_per_route": 1,
            "accuracy_calculations": accounting["accuracy_calculations"],
            "candidate_paths_materialized_after_authorization": accounting["candidate_paths_materialized"],
            "truth_paths_materialized_after_authorization": accounting["truth_paths_materialized"],
            "reruns": 0,
            "fallbacks": 0,
            "mat_precomputed_phone_coordinate_pdc_reads": 0,
            "kaggle_or_token_access": 0,
            "truth_reads_by_process": "one Phase118 truth-only evaluator process",
        },
        "forbidden_lanes": {
            "native_solver_rerun": False,
            "raw_or_base_rerun": False,
            "tdcp_rerun": False,
            "mat": False,
            "precomputed_phone_coordinates": False,
            "pdc": False,
            "kaggle_or_token": False,
            "solution_rows_in_result": False,
        },
        "solution_output_published": False,
        "release_or_submission_authorized": False,
        "authority": {
            "accuracy_freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
            "accuracy_manifest": {"path": relative(MANIFEST), "sha256": sha256_file(MANIFEST, "Phase118 accuracy manifest")},
            "accuracy_authorization": {"path": relative(AUTHORIZATION), "sha256": sha256_file(AUTHORIZATION, "Phase118 truth authorization")},
            "evaluator": {"path": relative(EVALUATOR), "sha256": sha256_file(EVALUATOR, "Phase118 accuracy evaluator")},
        },
    }
    atomic_json(result_path, result)
    atomic_text(result_path.with_suffix(".md"), render_markdown(result))
    return result


def fail_closed(error: str, result_path: Path = RESULT_JSON) -> dict[str, Any]:
    result = {
        "schema_version": RESULT_SCHEMA,
        "phase": 118,
        "execution_label": "Luna Max",
        "status": "no-go-phase118-tdcp-robust-k-truth-only-accuracy",
        "decision": "truth-only evaluator failed closed; preserve artifacts and do not rerun",
        "candidate": CANDIDATE_ID,
        "error": error,
        "accuracy_scored": False,
        "strict_0_782_gate": {"comparator": "candidate_macro_score_m < 0.782", "threshold_m": STRICT_PROMOTION_THRESHOLD_M, "passed": False},
        "read_accounting": {
            "native_solver_invocations": 0,
            "raw_gnss_imu_navigation_reads": 0,
            "raw_base_rinex_reads": 0,
            "candidate_solution_reads_for_hash_and_parse": 0,
            "candidate_coordinate_interpretations": 0,
            "truth_reads": 0,
            "accuracy_calculations": 0,
            "reruns": 0,
            "fallbacks": 0,
            "mat_precomputed_phone_coordinate_pdc_reads": 0,
            "kaggle_or_token_access": 0,
        },
        "forbidden_lanes": {
            "native_solver_rerun": False,
            "raw_or_base_rerun": False,
            "tdcp_rerun": False,
            "mat": False,
            "precomputed_phone_coordinates": False,
            "pdc": False,
            "kaggle_or_token": False,
            "solution_rows_in_result": False,
        },
        "solution_output_published": False,
        "release_or_submission_authorized": False,
    }
    atomic_json(result_path, result)
    atomic_text(result_path.with_suffix(".md"), render_markdown(result))
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
        if args.evaluate:
            try:
                fail_closed(str(exc), args.result_json)
            except Exception:
                pass
        print(f"phase118 truth-only evaluator: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
