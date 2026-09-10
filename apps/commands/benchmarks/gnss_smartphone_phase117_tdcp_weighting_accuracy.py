#!/usr/bin/env python3
"""Truth-only evaluator for the sealed Phase117 TDCP candidate.

This contract never launches the native solver and never opens raw phone or
base inputs.  Before the independent truth authorization it reads only sealed
JSON/MD metadata and source pins.  After authorization one evaluator process
opens each opaque candidate once for hash/schema preflight and each official
truth file once for the pinned Phase82 metric.  Result artifacts contain no
solution or truth coordinate rows.
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
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_accuracy_freeze_v1.json"
AUDIT = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_accuracy_audit_v1.md"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_accuracy_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_accuracy_truth_authorization_v1.json"
STRUCTURAL_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_structural_result_v1.json"
PHASE117_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_manifest_v1.json"
PHASE117_RAW_AUTH = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_raw_authorization_v1.json"
PHASE117_CONTRACT = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase117_tdcp_weighting.py"
PHASE117_WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase117_tdcp_weighting_execute.py"
PHASE117_SOURCE_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_source_parity_freeze_v1.json"
PHASE112_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_accuracy_result_v1.json"
PHASE112_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_accuracy_manifest_v1.json"
PHASE112_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase112_main_output_offset_accuracy.py"
PHASE112_TESTS = ROOT / "tests/test_smartphone_phase112_main_output_offset_accuracy.py"
PHASE82_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase82_phase80_direct_quality_accuracy.py"
PHASE82_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase82_phase80_direct_quality_accuracy_result_v1.json"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase117_tdcp_weighting_accuracy.py"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_accuracy_result_v1.json"
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

FREEZE_SHA = "0257bb38489e83898786b8d456d10773071e4ea7248a0e06c9d9766a4d7c2f24"
AUDIT_SHA = "3203cdd64aea2052d0970a18a4bf0b1c6399f25948cddf5e43a4baf0132f05d6"
STRUCTURAL_RESULT_SHA = "2517dcc805146dc34e790c78a392caf000c79eaba837fe099b6d522c147e9cf2"
PHASE117_MANIFEST_SHA = "676f5653a3f3ed95302da2eaec03d38e5757334076cf33134a4df5f8b0a682ec"
PHASE117_RAW_AUTH_SHA = "5bccd316f942ea7bb8bf2e0e69da627b92fd2dd76f1af4d75ad628e053fba202"
PHASE117_CONTRACT_SHA = "02083e7e642a73d05f2652a7786f8a1e769a6714a7a28dbcc7fe2fe39257717c"
PHASE117_WRAPPER_SHA = "07d7d9643bd449976bab85f2fde6d0dd16b77dbd9c5abbac2b4f0f706eb1e1c1"
PHASE117_SOURCE_FREEZE_SHA = "aba967b2ffab7ded84daa232ab948848d49497a70561262d1077200584b92dea"
PHASE112_RESULT_SHA = "46d8d836758ac1a88b95ec2c1a0c0080cedfb753c12411ed74e4c2501549c57e"
PHASE112_MANIFEST_SHA = "29aac6bd43f56183142c84219a7924d6172733a73385a5725a9e3e3013566768"
PHASE112_EVALUATOR_SHA = "d34c7df1b48be999141013018a78f5b61e70f831afc2977871e0ac756741b6cc"
PHASE112_TESTS_SHA = "414c501cc23283aa42f73b3cd7802cc0946b58494ac7fef1417945af2b554176"
PHASE82_EVALUATOR_SHA = "232ba0ab1fa7ebde6ddfaaf851d190d1ed32377ea056ab6c31e244139c2977fb"
PHASE82_RESULT_SHA = "39c5a38b7e3e61fda0306582fffb961dad3e6fb0bd49fcfe70e66674e2c90873"
RESULT_SCHEMA = "smartphone-r5-phase117-tdcp-weighting-accuracy-result.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase117-tdcp-weighting-accuracy-manifest.v1"
AUTHORIZATION_SCHEMA = "smartphone-r5-phase117-tdcp-weighting-accuracy-truth-authorization.v1"
CANDIDATE_ID = "phase117-official-tdcp-snr-type-weighting-truth-only-accuracy-v1"


class Phase117AccuracyError(ValueError):
    """Raised when the Phase117 truth-only contract fails closed."""


def fail(message: str) -> Phase117AccuracyError:
    return Phase117AccuracyError(message)


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


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def finite_tree(value: Any) -> bool:
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, list):
        return all(finite_tree(item) for item in value)
    if isinstance(value, dict):
        return all(finite_tree(item) for item in value.values())
    return True


def safe_relative(path_text: Any, basename: str, label: str) -> Path:
    if not isinstance(path_text, str):
        raise fail(f"missing {label} path")
    path = Path(path_text)
    if path.is_absolute() or ".." in path.parts or path.name != basename:
        raise fail(f"unsafe {label} path: {path_text}")
    return path


def _pin(authority: dict[str, Any], key: str, path: Path, expected: str) -> None:
    item = authority.get(key)
    if not isinstance(item, dict):
        raise fail(f"missing authority pin: {key}")
    assert_equal(item.get("path"), relative(path), f"authority/{key}/path")
    assert_equal(item.get("sha256"), expected, f"authority/{key}/sha256")


def verify_freeze() -> dict[str, Any]:
    freeze = read_json(FREEZE, "Phase117 truth accuracy freeze")
    assert_equal(sha256_file(FREEZE, "Phase117 truth accuracy freeze"), FREEZE_SHA, "freeze sha256")
    for key, expected in {
        "schema_version": "smartphone-r5-phase117-tdcp-weighting-accuracy-freeze.v1",
        "phase": 117,
        "status": "frozen-before-phase117-truth-only-accuracy-evaluation",
        "execution_label": "Luna Max",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    authority = freeze.get("authority")
    if not isinstance(authority, dict):
        raise fail("freeze authority missing")
    audit_pin = authority.get("audit")
    if not isinstance(audit_pin, dict) or audit_pin.get("sha256") != AUDIT_SHA or audit_pin.get("commit") != "6f524423e0cd80d23a526f23b7b50d548e51755a":
        raise fail("Phase117 accuracy audit pin changed")
    assert_equal(sha256_file(AUDIT, "Phase117 accuracy audit"), AUDIT_SHA, "audit sha256")
    _pin(authority, "phase117_structural_result", STRUCTURAL_RESULT, STRUCTURAL_RESULT_SHA)
    _pin(authority, "phase117_manifest", PHASE117_MANIFEST, PHASE117_MANIFEST_SHA)
    _pin(authority, "phase117_raw_authorization", PHASE117_RAW_AUTH, PHASE117_RAW_AUTH_SHA)
    _pin(authority, "phase117_contract", PHASE117_CONTRACT, PHASE117_CONTRACT_SHA)
    _pin(authority, "phase117_execution_wrapper", PHASE117_WRAPPER, PHASE117_WRAPPER_SHA)
    _pin(authority, "phase117_freeze", PHASE117_SOURCE_FREEZE, PHASE117_SOURCE_FREEZE_SHA)
    _pin(authority, "phase112_accuracy_result", PHASE112_RESULT, PHASE112_RESULT_SHA)
    _pin(authority, "phase112_accuracy_manifest", PHASE112_MANIFEST, PHASE112_MANIFEST_SHA)
    _pin(authority, "phase112_accuracy_evaluator", PHASE112_EVALUATOR, PHASE112_EVALUATOR_SHA)
    _pin(authority, "phase112_focused_tests", PHASE112_TESTS, PHASE112_TESTS_SHA)
    _pin(authority, "phase82_metric_reference", PHASE82_EVALUATOR, PHASE82_EVALUATOR_SHA)
    _pin(authority, "phase82_result", PHASE82_RESULT, PHASE82_RESULT_SHA)
    structural = read_json(STRUCTURAL_RESULT, "Phase117 structural result")
    for key, expected in {
        "phase": 117,
        "status": "go-phase117-tdcp-weighting-structural",
        "accuracy_scored": False,
        "solution_output_published": False,
    }.items():
        assert_equal(structural.get(key), expected, f"structural/{key}")
    if structural.get("read_accounting", {}).get("truth_reads") != 0:
        raise fail("Phase117 structural result already read truth")
    if structural.get("gates", {}).get("all_structural_gates_passed") is not True:
        raise fail("Phase117 structural result is not GO")
    routes = structural.get("routes")
    if not isinstance(routes, dict) or set(routes) != set(ROUTES):
        raise fail("Phase117 structural route set changed")
    candidate = freeze.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("freeze candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID,
        "candidate_count": 1,
        "route_order_fixed": True,
        "runs_per_route": 1,
        "solver_rerun": False,
        "raw_or_base_rerun": False,
        "tdcp_rerun": False,
        "solution_repaired_or_replaced": False,
        "solution_publication": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"candidate/{key}")
    for route in ROUTES:
        seal = routes[route].get("solution_hash_seal")
        metadata = candidate.get("routes_metadata", {}).get(route)
        if not isinstance(seal, dict) or not isinstance(metadata, dict):
            raise fail(f"missing opaque candidate seal: {route}")
        if seal.get("sha256") != metadata.get("sha256") or seal.get("bytes") != metadata.get("bytes") or seal.get("rows") != metadata.get("expected_prediction_rows"):
            raise fail(f"candidate seal mismatch: {route}")
        safe_relative(metadata.get("path"), "withheld_solution_output.csv", f"candidate {route}")
        if seal.get("published") is not False or seal.get("coordinate_rows_omitted") is not True or seal.get("row_count_matches_domain") is not True:
            raise fail(f"candidate publication/alignment boundary changed: {route}")
        if routes[route].get("return_code") != 0 or any(value is not True for value in routes[route].get("gates", {}).values()):
            raise fail(f"Phase117 structural gate changed: {route}")
    truth = freeze.get("truth_cohort")
    if not isinstance(truth, dict):
        raise fail("freeze truth cohort missing")
    for key, expected in {"route_order": list(ROUTES), "read_by_solver": False, "read_by_audit": False, "read_by_freeze": False, "read_by_manifest": False, "read_by_evaluator_only": True}.items():
        assert_equal(truth.get(key), expected, f"truth_cohort/{key}")
    for route in ROUTES:
        item = truth.get("routes", {}).get(route)
        if not isinstance(item, dict):
            raise fail(f"truth metadata missing: {route}")
        safe_relative(item.get("path"), "ground_truth.csv", f"truth {route}")
        if "/truth/" not in str(item.get("path")) or item.get("rows") != TRUTH_ROWS[route] or len(item.get("sha256", "")) != 64:
            raise fail(f"truth metadata malformed: {route}")
    metric = freeze.get("metric_contract")
    if not isinstance(metric, dict):
        raise fail("metric contract missing")
    for key, expected in {
        "matching": "exact integer key intersection only",
        "earth_radius_m": EARTH_RADIUS_M,
        "route_scalar": "(P50 + P95) / 2 in metres",
        "macro": "unweighted arithmetic mean over exactly MTV-A then LAX-T",
        "strict_macro_gate_m": 0.782,
        "prediction_domain_coverage": "matched prediction keys / prediction keys; required exactly 1.0",
    }.items():
        assert_equal(metric.get(key), expected, f"metric/{key}")
    boundary = freeze.get("truth_evaluator_boundary")
    for key, expected in {"evaluator_process_count": 1, "candidate_metadata_hash_seal_precedes_truth": True, "candidate_parse_precedes_truth": True, "truth_reads": 2, "truth_reads_per_route": 1, "native_after_truth": False, "result_content": "metrics, gate booleans, sealed hashes, and accounting only; no solution or truth coordinate rows"}.items():
        assert_equal(boundary.get(key), expected, f"truth_evaluator_boundary/{key}")
    return freeze


def verify_structural_result() -> dict[str, Any]:
    freeze = verify_freeze()
    structural = read_json(STRUCTURAL_RESULT, "Phase117 structural result")
    for route in ROUTES:
        seal = structural["routes"][route]["solution_hash_seal"]
        metadata = freeze["candidate"]["routes_metadata"][route]
        assert_equal(seal.get("sha256"), metadata.get("sha256"), f"structural candidate sha/{route}")
        assert_equal(seal.get("bytes"), metadata.get("bytes"), f"structural candidate bytes/{route}")
    return structural


def verify_manifest() -> dict[str, Any]:
    freeze = verify_freeze()
    manifest = read_json(MANIFEST, "Phase117 truth accuracy manifest")
    for key, expected in {"schema_version": MANIFEST_SCHEMA, "phase": 117, "execution_label": "Luna Max", "status": "sealed-before-phase117-truth-only-accuracy-evaluation"}.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    authority = manifest.get("authority")
    if not isinstance(authority, dict):
        raise fail("manifest authority missing")
    _pin(authority, "accuracy_freeze", FREEZE, FREEZE_SHA)
    _pin(authority, "accuracy_audit", AUDIT, AUDIT_SHA)
    _pin(authority, "phase117_structural_result", STRUCTURAL_RESULT, STRUCTURAL_RESULT_SHA)
    _pin(authority, "phase117_manifest", PHASE117_MANIFEST, PHASE117_MANIFEST_SHA)
    _pin(authority, "phase117_raw_authorization", PHASE117_RAW_AUTH, PHASE117_RAW_AUTH_SHA)
    _pin(authority, "phase117_contract", PHASE117_CONTRACT, PHASE117_CONTRACT_SHA)
    _pin(authority, "phase117_execution_wrapper", PHASE117_WRAPPER, PHASE117_WRAPPER_SHA)
    _pin(authority, "phase117_source_freeze", PHASE117_SOURCE_FREEZE, PHASE117_SOURCE_FREEZE_SHA)
    _pin(authority, "phase112_accuracy_result", PHASE112_RESULT, PHASE112_RESULT_SHA)
    _pin(authority, "phase112_accuracy_manifest", PHASE112_MANIFEST, PHASE112_MANIFEST_SHA)
    _pin(authority, "phase112_accuracy_evaluator", PHASE112_EVALUATOR, PHASE112_EVALUATOR_SHA)
    _pin(authority, "phase82_evaluator", PHASE82_EVALUATOR, PHASE82_EVALUATOR_SHA)
    _pin(authority, "phase82_result", PHASE82_RESULT, PHASE82_RESULT_SHA)
    _pin(authority, "evaluator", EVALUATOR, sha256_file(EVALUATOR, "Phase117 accuracy evaluator"))
    _pin(authority, "focused_tests", FOCUSED_TESTS, sha256_file(FOCUSED_TESTS, "Phase117 accuracy focused tests"))
    assert_equal(manifest.get("routes"), list(ROUTES), "manifest routes")
    candidate = manifest.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("manifest candidate missing")
    for key, expected in {"id": CANDIDATE_ID, "candidate_count": 1, "solver_rerun": False, "raw_or_base_rerun": False, "tdcp_rerun": False, "solution_publication": False}.items():
        assert_equal(candidate.get(key), expected, f"manifest candidate/{key}")
    truth = manifest.get("truth_cohort")
    if not isinstance(truth, dict) or truth.get("read_by_solver") is not False or truth.get("read_by_manifest") is not False or truth.get("read_by_evaluator_only") is not True:
        raise fail("manifest truth boundary changed")
    for route in ROUTES:
        item = truth.get("routes", {}).get(route)
        expected = freeze["truth_cohort"]["routes"][route]
        if not isinstance(item, dict):
            raise fail(f"manifest truth metadata missing: {route}")
        for key in ("path", "sha256", "bytes", "rows", "expected_missing_truth_rows", "expected_missing_truth_key"):
            assert_equal(item.get(key), expected.get(key), f"manifest truth/{route}/{key}")
    metric = manifest.get("metric_contract")
    if not isinstance(metric, dict) or metric.get("strict_macro_gate_m") != 0.782 or metric.get("earth_radius_m") != EARTH_RADIUS_M:
        raise fail("manifest metric contract changed")
    accounting = manifest.get("read_accounting_before_authorization")
    if not isinstance(accounting, dict):
        raise fail("manifest accounting missing")
    for key in ("native_solver_invocations", "raw_gnss_imu_navigation_reads", "base_rinex_reads", "truth_reads", "candidate_solution_reads", "accuracy_calculations", "mat_precomputed_phone_coordinate_pdc_reads", "kaggle_or_token_access", "reruns", "fallbacks"):
        assert_equal(accounting.get(key), 0, f"manifest accounting/{key}")
    return manifest


def verify_pre_truth() -> dict[str, Any]:
    verify_manifest()
    return {
        "status": "pre-truth-verified",
        "native_solver_invocations": 0,
        "raw_reads": 0,
        "base_reads": 0,
        "truth_reads": 0,
        "candidate_solution_reads": 0,
        "candidate_coordinate_interpretations": 0,
        "accuracy_calculations": 0,
        "mat_precomputed_phone_coordinate_pdc_reads": 0,
        "kaggle_or_token_access": 0,
        "native_rerun": False,
    }


def verify_authorization() -> dict[str, Any]:
    verify_manifest()
    auth = read_json(AUTHORIZATION, "Phase117 truth-only authorization")
    for key, expected in {"schema_version": AUTHORIZATION_SCHEMA, "phase": 117, "execution_label": "Luna Max", "status": "authorized-for-phase117-truth-only-accuracy-evaluation"}.items():
        assert_equal(auth.get(key), expected, f"authorization/{key}")
    authority = auth.get("authority")
    if not isinstance(authority, dict):
        raise fail("authorization authority missing")
    _pin(authority, "accuracy_freeze", FREEZE, FREEZE_SHA)
    _pin(authority, "accuracy_manifest", MANIFEST, sha256_file(MANIFEST, "Phase117 accuracy manifest"))
    _pin(authority, "accuracy_evaluator", EVALUATOR, sha256_file(EVALUATOR, "Phase117 accuracy evaluator"))
    _pin(authority, "focused_tests", FOCUSED_TESTS, sha256_file(FOCUSED_TESTS, "Phase117 accuracy focused tests"))
    _pin(authority, "phase117_structural_result", STRUCTURAL_RESULT, STRUCTURAL_RESULT_SHA)
    _pin(authority, "phase117_raw_authorization", PHASE117_RAW_AUTH, PHASE117_RAW_AUTH_SHA)
    _pin(authority, "phase112_accuracy_result", PHASE112_RESULT, PHASE112_RESULT_SHA)
    _pin(authority, "phase82_evaluator", PHASE82_EVALUATOR, PHASE82_EVALUATOR_SHA)
    _pin(authority, "phase82_result", PHASE82_RESULT, PHASE82_RESULT_SHA)
    assert_equal(auth.get("routes"), list(ROUTES), "authorization routes")
    candidate = auth.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("authorization candidate missing")
    for key, expected in {"id": CANDIDATE_ID, "candidate_count": 1, "solver_rerun": False, "raw_or_base_rerun": False, "truth_reads": 2, "truth_reads_per_route": 1, "accuracy_calculations": 2, "solution_publication": False, "no_fallback_or_rerun": True}.items():
        assert_equal(candidate.get(key), expected, f"authorization candidate/{key}")
    policy = auth.get("execution_policy")
    if not isinstance(policy, dict):
        raise fail("authorization execution policy missing")
    for key, expected in {"truth_only_evaluation_authorized": True, "native_solver_invocations": 0, "raw_reads": 0, "base_reads": 0, "truth_reads_per_route": 1, "native_after_truth": False, "solution_publication": False, "strict_macro_gate_m": 0.782, "mat_precomputed_phone_coordinate_pdc_reads": 0, "kaggle_or_token_access": 0}.items():
        assert_equal(policy.get(key), expected, f"authorization policy/{key}")
    return auth


def load_phase82() -> Any:
    if sha256_file(PHASE82_EVALUATOR, "Phase82 metric evaluator") != PHASE82_EVALUATOR_SHA:
        raise fail("Phase82 metric evaluator hash changed")
    spec = importlib.util.spec_from_file_location("phase82_metric_reference_phase117", PHASE82_EVALUATOR)
    if spec is None or spec.loader is None:
        raise fail("unable to load Phase82 metric evaluator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def candidate_path(structural: dict[str, Any], route: str) -> Path:
    path = safe_relative(structural["routes"][route]["solution_hash_seal"].get("path"), "withheld_solution_output.csv", f"candidate {route}")
    if "/phase117-tdcp-weighting-v1/" not in str(path):
        raise fail(f"candidate path outside Phase117 output root: {route}")
    return ROOT / path


def truth_path(freeze: dict[str, Any], route: str) -> Path:
    path = safe_relative(freeze["truth_cohort"]["routes"][route].get("path"), "ground_truth.csv", f"truth {route}")
    if "/truth/" not in str(path):
        raise fail(f"truth path outside pinned truth cohort: {route}")
    return ROOT / path


def read_candidate_once(path: Path, seal: dict[str, Any], route: str, p82: Any) -> tuple[list[tuple[int, float, float]], dict[int, tuple[float, float]], dict[str, Any]]:
    payload = path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != seal.get("sha256") or len(payload) != seal.get("bytes"):
        raise fail(f"Phase117 candidate opaque seal mismatch: {route}")
    try:
        ordered, mapping = p82.P76.P74._parse_submission(payload, route)
    except Exception as exc:
        raise fail(f"Phase117 candidate schema failed: {route}: {exc}") from exc
    if len(ordered) != DOMAIN_ROWS[route] or len(mapping) != len(ordered) or [item[0] for item in ordered] != sorted(item[0] for item in ordered):
        raise fail(f"Phase117 candidate exact alignment failed: {route}")
    if any(not all(math.isfinite(float(value)) for value in item[1:]) or not -90.0 <= item[1] <= 90.0 or not -180.0 <= item[2] <= 180.0 for item in ordered):
        raise fail(f"Phase117 candidate finite/Earth-valid preflight failed: {route}")
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


def sealed_baselines() -> dict[str, dict[str, float]]:
    phase112 = read_json(PHASE112_RESULT, "Phase112 sealed accuracy aggregate")
    if sha256_file(PHASE112_RESULT, "Phase112 sealed accuracy aggregate") != PHASE112_RESULT_SHA:
        raise fail("Phase112 sealed accuracy aggregate hash changed")
    phase82 = read_json(PHASE82_RESULT, "Phase82 sealed accuracy aggregate")
    if sha256_file(PHASE82_RESULT, "Phase82 sealed accuracy aggregate") != PHASE82_RESULT_SHA:
        raise fail("Phase82 sealed accuracy aggregate hash changed")
    try:
        return {
            "phase112": {route: float(phase112["routes"][route]["candidate"]["score_m"]) for route in ROUTES},
            "phase82": {route: float(phase82["routes"][route]["candidate"]["score_m"]) for route in ROUTES},
            "phase112_macro": float(phase112["aggregate"]["candidate_macro_score_m"]),
        }
    except (KeyError, TypeError, ValueError) as exc:
        raise fail(f"sealed baseline aggregate malformed: {exc}") from exc


def route_report(route: str, structural: dict[str, Any], freeze: dict[str, Any], p82: Any, baselines: dict[str, dict[str, float]], accounting: dict[str, int]) -> dict[str, Any]:
    report: dict[str, Any] = {"dataset_id": route, "truth_read": False, "accuracy_scored": False}
    try:
        seal = structural["routes"][route]["solution_hash_seal"]
        ordered, candidate, candidate_meta = read_candidate_once(candidate_path(structural, route), seal, route, p82)
        accounting["candidate_solution_reads"] += 1
        report["candidate_solution"] = candidate_meta
        truth, truth_meta = read_truth_once(truth_path(freeze, route), freeze["truth_cohort"]["routes"][route], route, p82)
        accounting["truth_reads"] += 1
        report["truth"] = truth_meta
        report["truth_read"] = True
        score = p82.P76._score_prediction(candidate, truth, EXPECTED_MISSING[route], route, ordered)
        accounting["accuracy_calculations"] += 1
        phase112_score = baselines["phase112"][route]
        phase82_score = baselines["phase82"][route]
        checks = {
            "candidate_schema_and_exact_alignment": len(ordered) == DOMAIN_ROWS[route],
            "candidate_prediction_domain_coverage_exact": score.get("prediction_domain_coverage") == 1.0,
            "candidate_finite_and_earth_valid": score.get("finite") is True,
            "candidate_over_70_mps_count_zero": score.get("over_70_mps_count") == 0,
            "candidate_route_score_at_most_3m": score.get("score_m", math.inf) <= 3.0,
            "candidate_no_route_regression_vs_phase112": score.get("score_m", math.inf) <= phase112_score,
            "candidate_no_route_regression_vs_phase82_same_route": score.get("score_m", math.inf) <= phase82_score,
        }
        report.update({
            "accuracy_scored": True,
            "candidate": score,
            "baseline_phase112_same_pipeline": {"score_m": phase112_score},
            "baseline_phase82_same_route": {"score_m": phase82_score},
            "gates": {"passed": all(checks.values()), "checks": checks, "failures": [key for key, value in checks.items() if value is not True]},
        })
    except Exception as exc:
        report["failure"] = str(exc)
        report.setdefault("gates", {"passed": False, "checks": {}, "failures": ["route_evaluation"]})
    return report


def render_markdown(result: dict[str, Any]) -> str:
    aggregate = result.get("aggregate", {})
    lines = [
        "# Phase117 official TDCP SNR/type weighting truth-only accuracy result",
        "",
        f"- status: `{result.get('status')}`",
        "- Native solver/raw/base/TDCP: not rerun; Phase117 sealed solutions reused",
        "- Truth: one official file read per route by the authorized evaluator subprocess",
        "- Solution/truth coordinate rows: omitted from this result",
        "",
        f"- Candidate macro: `{aggregate.get('candidate_macro_score_m')}` m",
        f"- Phase112 same-pipeline macro: `{aggregate.get('phase112_same_pipeline_macro_m')}` m",
        f"- Phase82 same-route macro: `{aggregate.get('phase82_same_route_macro_m')}` m",
        f"- Strict `0.782 m` gate: `{result.get('strict_0_782_gate', {}).get('passed')}`",
        "",
        "| Route | Candidate (m) | Phase112 (m) | Phase82 (m) | Truth read | Gates |",
        "|---|---:|---:|---:|---:|---|",
    ]
    for route in ROUTES:
        item = result.get("routes", {}).get(route, {})
        lines.append(
            f"| `{route}` | `{item.get('candidate', {}).get('score_m')}` | "
            f"`{item.get('baseline_phase112_same_pipeline', {}).get('score_m')}` | "
            f"`{item.get('baseline_phase82_same_route', {}).get('score_m')}` | "
            f"`{item.get('truth_read')}` | `{item.get('gates', {}).get('passed')}` |"
        )
    lines.extend(["", "Accuracy GO would not authorize release, validation, or Kaggle submission.", ""])
    return "\n".join(lines)


def evaluate(result_path: Path = RESULT_JSON) -> dict[str, Any]:
    manifest = verify_manifest()
    verify_authorization()
    freeze = verify_freeze()
    structural = verify_structural_result()
    p82 = load_phase82()
    baselines = sealed_baselines()
    accounting = {"candidate_solution_reads": 0, "truth_reads": 0, "accuracy_calculations": 0}
    reports = {route: route_report(route, structural, freeze, p82, baselines, accounting) for route in ROUTES}
    scored = [reports[route] for route in ROUTES if reports[route].get("accuracy_scored") is True]
    candidate_macro = sum(item["candidate"]["score_m"] for item in scored) / 2.0 if len(scored) == 2 else None
    phase112_macro = baselines["phase112_macro"]
    phase82_macro = sum(baselines["phase82"].values()) / 2.0
    gates = {
        "exact_two_routes_one_evaluation_each": len(scored) == 2 and accounting["truth_reads"] == 2,
        "candidate_output_schema_and_exact_alignment": len(scored) == 2 and all(report["gates"]["checks"].get("candidate_schema_and_exact_alignment") is True for report in scored),
        "candidate_prediction_domain_coverage_exact": len(scored) == 2 and all(report["candidate"].get("prediction_domain_coverage") == 1.0 for report in scored),
        "candidate_all_finite_and_earth_valid": len(scored) == 2 and all(report["candidate"].get("finite") is True for report in scored),
        "candidate_over_70_mps_count_zero": len(scored) == 2 and all(report["candidate"].get("over_70_mps_count") == 0 for report in scored),
        "candidate_each_route_score_at_most_3m": len(scored) == 2 and all(report["candidate"].get("score_m", math.inf) <= 3.0 for report in scored),
        "candidate_no_route_regression_vs_phase112": len(scored) == 2 and all(reports[route]["candidate"]["score_m"] <= baselines["phase112"][route] for route in ROUTES),
        "candidate_no_route_regression_vs_phase82_same_route": len(scored) == 2 and all(reports[route]["candidate"]["score_m"] <= baselines["phase82"][route] for route in ROUTES),
        "candidate_macro_score_at_most_2m": candidate_macro is not None and candidate_macro <= 2.0,
        "candidate_macro_score_strict_at_most_0_782m": candidate_macro is not None and candidate_macro <= 0.782,
        "truth_read_only_by_one_evaluator_subprocess": accounting["truth_reads"] == 2 and all(reports[route].get("truth_read") is True for route in ROUTES),
        "no_solution_rows_in_result": True,
        "no_solver_raw_base_tdcp_mat_precomputed_pdc_kaggle": True,
    }
    failed = [key for key, value in gates.items() if value is not True]
    result = {
        "schema_version": RESULT_SCHEMA,
        "phase": 117,
        "execution_label": "Luna Max",
        "status": "go-phase117-tdcp-weighting-truth-only-accuracy" if not failed else "no-go-phase117-tdcp-weighting-truth-only-accuracy",
        "decision": "truth-only accuracy evaluated; release remains separately unauthorized" if not failed else "truth-only accuracy gate failed closed; preserve artifacts and do not rerun",
        "candidate": CANDIDATE_ID,
        "accuracy_scored": len(scored) == 2,
        "routes": reports,
        "aggregate": {
            "candidate_macro_score_m": candidate_macro,
            "phase112_same_pipeline_macro_m": phase112_macro,
            "phase82_same_route_macro_m": phase82_macro,
            "route_count": 2,
            "macro_route_order": list(ROUTES),
            "macro_weighting": "unweighted arithmetic mean over exactly MTV-A then LAX-T",
        },
        "metric_contract": manifest["metric_contract"],
        "promotion_gates": gates,
        "strict_0_782_gate": {"threshold_m": 0.782, "candidate_macro_score_m": candidate_macro, "passed": gates["candidate_macro_score_strict_at_most_0_782m"]},
        "failed_gates": failed,
        "read_accounting": {
            "native_solver_invocations": 0,
            "raw_gnss_imu_navigation_reads": 0,
            "base_rinex_reads": 0,
            "candidate_solution_reads_for_hash_and_parse": accounting["candidate_solution_reads"],
            "truth_reads": accounting["truth_reads"],
            "truth_reads_per_route": 1,
            "accuracy_calculations": accounting["accuracy_calculations"],
            "reruns": 0,
            "fallbacks": 0,
            "mat_precomputed_phone_coordinate_pdc_reads": 0,
            "kaggle_or_token_access": 0,
            "truth_reads_by_process": "Phase117 truth-only evaluator subprocess",
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
            "accuracy_freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA},
            "accuracy_manifest": {"path": relative(MANIFEST), "sha256": sha256_file(MANIFEST, "Phase117 accuracy manifest")},
            "accuracy_authorization": {"path": relative(AUTHORIZATION), "sha256": sha256_file(AUTHORIZATION, "Phase117 truth authorization")},
            "evaluator": {"path": relative(EVALUATOR), "sha256": sha256_file(EVALUATOR, "Phase117 accuracy evaluator")},
        },
    }
    atomic_json(result_path, result)
    atomic_text(result_path.with_suffix(".md"), render_markdown(result))
    return result


def fail_closed(error: str, result_path: Path = RESULT_JSON) -> dict[str, Any]:
    result = {
        "schema_version": RESULT_SCHEMA,
        "phase": 117,
        "execution_label": "Luna Max",
        "status": "no-go-phase117-tdcp-weighting-truth-only-accuracy",
        "decision": "truth-only evaluator failed closed; preserve artifacts and do not rerun",
        "candidate": CANDIDATE_ID,
        "error": error,
        "accuracy_scored": False,
        "strict_0_782_gate": {"threshold_m": 0.782, "passed": False},
        "read_accounting": {"native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0, "base_rinex_reads": 0, "truth_reads": 0, "candidate_solution_reads_for_hash_and_parse": 0, "accuracy_calculations": 0, "reruns": 0, "fallbacks": 0, "mat_precomputed_phone_coordinate_pdc_reads": 0, "kaggle_or_token_access": 0},
        "forbidden_lanes": {"native_solver_rerun": False, "raw_or_base_rerun": False, "tdcp_rerun": False, "mat": False, "precomputed_phone_coordinates": False, "pdc": False, "kaggle_or_token": False, "solution_rows_in_result": False},
        "solution_output_published": False,
        "release_or_submission_authorized": False,
    }
    atomic_json(result_path, result)
    atomic_text(result_path.with_suffix(".md"), render_markdown(result))
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
                fail_closed(str(exc), args.result_json)
            except Exception:
                pass
        print(f"phase117 truth-only evaluator: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
