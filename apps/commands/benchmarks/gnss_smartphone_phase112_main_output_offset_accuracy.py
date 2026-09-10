#!/usr/bin/env python3
"""Truth-only evaluator for the sealed Phase112 output-offset solutions.

This contract never launches the native solver and never opens raw phone or
base inputs.  Before the truth authorization it reads only sealed result,
freeze, source, and path/hash metadata.  The authorized evaluator process
opens each opaque Phase112 solution once and each official truth file once,
then delegates parsing and scoring to the pinned Phase82 metric code.
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
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
EVALUATOR = Path(__file__).resolve()
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_accuracy_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_accuracy_truth_authorization_v1.json"
STRUCTURAL_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_structural_result_v1.json"
PHASE108_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase108_phase107_solution_accuracy_freeze_v1.json"
PHASE108_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase108_phase107_solution_accuracy_result_v1.json"
PHASE82_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase82_phase80_direct_quality_accuracy.py"
PHASE82_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase82_phase80_direct_quality_accuracy_result_v1.json"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_accuracy_result_v1.json"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
EXPECTED_MISSING = {ROUTES[0]: [ROUTES[0], 1615921153434], ROUTES[1]: None}
PHASE108_FREEZE_SHA = "e503dac7508f6b6466173c5f0c651a04fa9dc24222aa3ce5ae22332fbb16dde1"
PHASE108_RESULT_SHA = "9f9a2528745c06ba06e774139f56892bcea0ef208a3d064e4b70042e48f29a9d"
PHASE82_EVALUATOR_SHA = "232ba0ab1fa7ebde6ddfaaf851d190d1ed32377ea056ab6c31e244139c2977fb"
PHASE82_RESULT_SHA = "39c5a38b7e3e61fda0306582fffb961dad3e6fb0bd49fcfe70e66674e2c90873"
STRUCTURAL_RESULT_SHA = "087b1bf51b4e584b86a7b558560df4ceb78e19b01e572f2d60aeef4c3d759ba0"
STRUCTURAL_COMMIT = "dfa9c3a186c9d8633dc572192b513b584f18800a"
CANDIDATE_ID = "phase112-main-output-upstream-position-offset-v1"
MANIFEST_SCHEMA = "smartphone-r5-phase112-main-output-offset-accuracy-manifest.v1"
AUTHORIZATION_SCHEMA = "smartphone-r5-phase112-main-output-offset-truth-authorization.v1"
RESULT_SCHEMA = "smartphone-r5-phase112-main-output-offset-accuracy-result.v1"
EARTH_RADIUS_M = 6371008.8
MAX_SPEED_MPS = 70.0


class Phase112AccuracyError(ValueError):
    """Raised when the truth-only contract fails closed."""


def fail(message: str) -> Phase112AccuracyError:
    return Phase112AccuracyError(message)


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


def verify_phase108_freeze() -> dict[str, Any]:
    freeze = read_json(PHASE108_FREEZE, "Phase108 metric freeze")
    assert_equal(sha256_file(PHASE108_FREEZE, "Phase108 metric freeze"), PHASE108_FREEZE_SHA, "Phase108 freeze sha256")
    for key, expected in {
        "schema_version": "smartphone-r5-phase108-phase107-solution-accuracy-freeze.v1",
        "phase": 108, "status": "frozen-before-phase108-truth-only-evaluation",
        "execution_label": "Luna Max",
    }.items():
        assert_equal(freeze.get(key), expected, f"Phase108 freeze/{key}")
    metric = freeze.get("metric_contract")
    if not isinstance(metric, dict):
        raise fail("Phase108 metric contract missing")
    for key, expected in {
        "earth_radius_m": EARTH_RADIUS_M,
        "route_scalar": "(P50 + P95) / 2 in metres",
        "macro": "unweighted arithmetic mean over exactly MTV-A then LAX-T",
        "matching": "exact integer key intersection only",
        "prediction_domain_coverage": "matched prediction keys / prediction keys; required exactly 1.0",
    }.items():
        assert_equal(metric.get(key), expected, f"metric/{key}")
    truth = freeze.get("truth_cohort")
    if not isinstance(truth, dict) or truth.get("read_by_solver") is not False or truth.get("read_by_audit") is not False or truth.get("read_by_freeze") is not False or truth.get("read_by_evaluator_only") is not True:
        raise fail("Phase108 truth cohort boundary changed")
    for route in ROUTES:
        pin = truth.get("routes", {}).get(route)
        if not isinstance(pin, dict) or not isinstance(pin.get("path"), str):
            raise fail(f"missing sealed truth metadata: {route}")
        if pin.get("sha256") not in {
            "2021-03-16-18-59-us-ca-mtv-a/pixel5": "7c84ed6a80b1bbb08c0ffad57493513833b9d5474e22a43c5a44da82824ee22d",
            "2022-04-01-18-22-us-ca-lax-t/pixel5": "29e0861dd1ecb8865c10adab69396d98ed96618e8877b09d04aa8d671edf79e8",
        }[route]:
            raise fail(f"truth pin changed: {route}")
        assert_equal(pin.get("rows"), 2159 if route == ROUTES[0] else 1465, f"truth rows/{route}")
    return freeze


def verify_structural_result() -> dict[str, Any]:
    assert_equal(sha256_file(STRUCTURAL_RESULT, "Phase112 structural result"), STRUCTURAL_RESULT_SHA, "structural result sha256")
    result = read_json(STRUCTURAL_RESULT, "Phase112 structural result")
    assert_equal(result.get("schema_version"), "smartphone-r5-phase112-main-output-offset-structural-result.v1", "structural schema")
    assert_equal(result.get("status"), "go-phase112-main-output-offset-structural", "structural status")
    assert_equal(result.get("accuracy_scored"), False, "structural accuracy flag")
    assert_equal(result.get("solution_output_published"), False, "structural publication flag")
    if result.get("read_accounting", {}).get("truth_reads") != 0:
        raise fail("structural result already read truth")
    routes = result.get("routes")
    if not isinstance(routes, dict) or set(routes) != set(ROUTES):
        raise fail("structural route set changed")
    for route in ROUTES:
        seal = routes[route].get("solution_hash_seal")
        if not isinstance(seal, dict) or seal.get("present") is not True or seal.get("published") is not False or seal.get("coordinate_rows_omitted") is not True:
            raise fail(f"opaque solution seal missing: {route}")
        if not isinstance(seal.get("sha256"), str) or len(seal["sha256"]) != 64 or seal.get("rows") != DOMAIN_ROWS[route]:
            raise fail(f"opaque solution seal malformed: {route}")
    return result


def verify_manifest() -> dict[str, Any]:
    freeze = verify_phase108_freeze()
    structural = verify_structural_result()
    if sha256_file(PHASE108_RESULT, "Phase108 sealed aggregate") != PHASE108_RESULT_SHA:
        raise fail("Phase108 sealed aggregate hash changed")
    if sha256_file(PHASE82_EVALUATOR, "Phase82 metric evaluator") != PHASE82_EVALUATOR_SHA:
        raise fail("Phase82 metric source hash changed")
    if sha256_file(PHASE82_RESULT, "Phase82 sealed aggregate") != PHASE82_RESULT_SHA:
        raise fail("Phase82 sealed aggregate hash changed")
    manifest = read_json(MANIFEST, "Phase112 truth manifest")
    for key, expected in {"schema_version": MANIFEST_SCHEMA, "phase": 112, "execution_label": "Luna Max", "status": "sealed-before-phase112-truth-only-evaluation"}.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    authority = manifest.get("authority")
    if not isinstance(authority, dict):
        raise fail("truth manifest authority missing")
    for key, path, expected in (
        ("phase112_structural_result", STRUCTURAL_RESULT, STRUCTURAL_RESULT_SHA),
        ("phase108_freeze", PHASE108_FREEZE, PHASE108_FREEZE_SHA),
        ("phase108_result", PHASE108_RESULT, PHASE108_RESULT_SHA),
        ("phase82_evaluator", PHASE82_EVALUATOR, PHASE82_EVALUATOR_SHA),
        ("phase82_result", PHASE82_RESULT, PHASE82_RESULT_SHA),
    ):
        pin = authority.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest authority missing: {key}")
        assert_equal(pin.get("path"), relative(path), f"manifest authority/{key}/path")
        assert_equal(pin.get("sha256"), expected, f"manifest authority/{key}/sha256")
    for key, path in (
        ("evaluator", EVALUATOR),
        ("focused_tests", ROOT / "tests/test_smartphone_phase112_main_output_offset_accuracy.py"),
    ):
        pin = manifest.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/{key} pin missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/{key}/path")
        assert_equal(pin.get("sha256"), sha256_file(path, f"manifest/{key}"), f"manifest/{key}/sha256")
    for key, expected in {"structural_commit": STRUCTURAL_COMMIT, "candidate_id": CANDIDATE_ID, "native_solver_invocations": 0, "truth_reads_before_auth": 0, "solution_rows_in_result": False}.items():
        assert_equal(manifest.get("boundary", {}).get(key), expected, f"manifest boundary/{key}")
    assert_equal(manifest.get("routes"), list(ROUTES), "manifest routes")
    truth = manifest.get("truth_cohort")
    if not isinstance(truth, dict):
        raise fail("manifest truth cohort missing")
    assert_equal(truth.get("read_by_solver"), False, "manifest truth solver read")
    assert_equal(truth.get("read_by_manifest"), False, "manifest truth manifest read")
    assert_equal(truth.get("read_by_evaluator_only"), True, "manifest truth evaluator boundary")
    for route in ROUTES:
        pin = truth.get("routes", {}).get(route)
        if not isinstance(pin, dict):
            raise fail(f"manifest truth pin missing: {route}")
        safe = Path(pin.get("path", ""))
        if safe.is_absolute() or ".." in safe.parts or safe.name != "ground_truth.csv" or "/truth/" not in str(safe):
            raise fail(f"unsafe truth path metadata: {route}")
        expected = freeze["truth_cohort"]["routes"][route]
        for key in ("path", "sha256", "bytes", "rows", "expected_missing_truth_key"):
            assert_equal(pin.get(key), expected.get(key) if key != "expected_missing_truth_key" else expected.get(key), f"manifest truth/{route}/{key}")
    metric = manifest.get("metric_contract")
    if not isinstance(metric, dict) or metric.get("source") != "Phase108 metric contract + Phase82 evaluator" or metric.get("strict_0_782_threshold_m") != 0.782:
        raise fail("truth metric contract changed")
    accounting = manifest.get("read_accounting_before_authorization")
    if not isinstance(accounting, dict):
        raise fail("truth manifest accounting missing")
    for key in ("native_solver_invocations", "raw_gnss_imu_navigation_reads", "base_rinex_reads", "truth_reads", "candidate_solution_reads", "accuracy_calculations", "mat_reads_or_generated", "precomputed_phone_coordinate_pdc_reads", "kaggle_or_token_access", "reruns", "fallbacks"):
        assert_equal(accounting.get(key), 0, f"manifest accounting/{key}")
    return manifest


def verify_pre_truth() -> dict[str, Any]:
    verify_manifest()
    return {
        "status": "pre-truth-verified", "native_solver_invocations": 0,
        "raw_reads": 0, "base_reads": 0, "truth_reads": 0,
        "candidate_solution_reads": 0, "candidate_coordinate_interpretations": 0,
        "accuracy_calculations": 0, "mat_precomputed_phone_coordinate_pdc_reads": 0,
        "kaggle_or_token_access": 0, "native_rerun": False,
    }


def verify_authorization() -> dict[str, Any]:
    manifest = verify_manifest()
    auth = read_json(AUTHORIZATION, "Phase112 truth authorization")
    for key, expected in {"schema_version": AUTHORIZATION_SCHEMA, "phase": 112, "execution_label": "Luna Max", "status": "authorized-for-phase112-truth-only-accuracy-evaluation"}.items():
        assert_equal(auth.get(key), expected, f"authorization/{key}")
    authority = auth.get("authority")
    if not isinstance(authority, dict):
        raise fail("truth authorization authority missing")
    pins = {
        "phase112_structural_result": (STRUCTURAL_RESULT, STRUCTURAL_RESULT_SHA),
        "phase112_manifest": (MANIFEST, sha256_file(MANIFEST, "Phase112 truth manifest")),
        "phase112_evaluator": (EVALUATOR, sha256_file(EVALUATOR, "Phase112 truth evaluator")),
        "phase112_focused_tests": (ROOT / "tests/test_smartphone_phase112_main_output_offset_accuracy.py", None),
        "phase108_freeze": (PHASE108_FREEZE, PHASE108_FREEZE_SHA),
        "phase108_result": (PHASE108_RESULT, PHASE108_RESULT_SHA),
        "phase82_evaluator": (PHASE82_EVALUATOR, PHASE82_EVALUATOR_SHA),
    }
    for key, (path, expected) in pins.items():
        pin = authority.get(key)
        if not isinstance(pin, dict):
            raise fail(f"truth authorization pin missing: {key}")
        assert_equal(pin.get("path"), relative(path), f"authorization/{key}/path")
        if expected is None:
            expected = sha256_file(path, f"truth focused tests")
        assert_equal(pin.get("sha256"), expected, f"authorization/{key}/sha256")
    assert_equal(auth.get("routes"), list(ROUTES), "truth authorization routes")
    candidate = auth.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("truth authorization candidate missing")
    for key, expected in {"id": CANDIDATE_ID, "candidate_count": 1, "solver_rerun": False, "raw_or_base_rerun": False, "truth_reads": 2, "truth_reads_per_route": 1, "solution_publication": False, "accuracy_calculations": 2, "no_fallback_or_rerun": True}.items():
        assert_equal(candidate.get(key), expected, f"authorization candidate/{key}")
    policy = auth.get("execution_policy")
    if not isinstance(policy, dict):
        raise fail("truth authorization policy missing")
    for key, expected in {"truth_only_evaluation_authorized": True, "native_solver_invocations": 0, "raw_reads": 0, "base_reads": 0, "truth_reads_per_route": 1, "native_after_truth": False, "solution_publication": False, "strict_0_782_threshold_m": 0.782, "mat_precomputed_phone_coordinate_pdc_reads": 0, "kaggle_or_token_access": 0}.items():
        assert_equal(policy.get(key), expected, f"authorization policy/{key}")
    return auth


def _load_phase82() -> Any:
    if sha256_file(PHASE82_EVALUATOR, "Phase82 metric evaluator") != PHASE82_EVALUATOR_SHA:
        raise fail("Phase82 evaluator hash changed")
    spec = importlib.util.spec_from_file_location("phase82_metric_reference_phase112", PHASE82_EVALUATOR)
    if spec is None or spec.loader is None:
        raise fail("unable to load Phase82 metric evaluator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _candidate_path(structural: dict[str, Any], route: str) -> Path:
    value = structural["routes"][route]["solution_hash_seal"]["path"]
    path = Path(value)
    if path.is_absolute() or ".." in path.parts or path.name != "withheld_solution_output.csv" or "/phase112-main-output-offset-v1/" not in str(path):
        raise fail(f"unsafe Phase112 solution path: {route}")
    return ROOT / path


def _truth_path(freeze: dict[str, Any], route: str) -> Path:
    value = freeze["truth_cohort"]["routes"][route]["path"]
    path = Path(value)
    if path.is_absolute() or ".." in path.parts or path.name != "ground_truth.csv" or "/truth/" not in str(path):
        raise fail(f"unsafe official truth path: {route}")
    return ROOT / path


def _read_candidate_once(path: Path, seal: dict[str, Any], route: str, p82: Any) -> tuple[list[tuple[int, float, float]], dict[int, tuple[float, float]], dict[str, Any]]:
    payload = path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != seal.get("sha256") or len(payload) != seal.get("bytes"):
        raise fail(f"Phase112 solution opaque seal mismatch: {route}")
    try:
        ordered, mapping = p82.P76.P74._parse_submission(payload, route)
    except Exception as exc:
        raise fail(f"Phase112 solution schema failed: {route}: {exc}") from exc
    if len(ordered) != DOMAIN_ROWS[route] or len(mapping) != len(ordered) or [item[0] for item in ordered] != sorted(item[0] for item in ordered):
        raise fail(f"Phase112 solution exact alignment failed: {route}")
    if any(not all(math.isfinite(float(value)) for value in item[1:]) or not -90.0 <= item[1] <= 90.0 or not -180.0 <= item[2] <= 180.0 for item in ordered):
        raise fail(f"Phase112 solution finite/Earth-valid preflight failed: {route}")
    return ordered, mapping, {"path": relative(path), "bytes": len(payload), "sha256": digest, "header": "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees", "rows": len(ordered), "read_count": 1, "coordinate_rows_omitted": True}


def _read_truth_once(path: Path, pin: dict[str, Any], route: str, p82: Any) -> tuple[dict[int, tuple[float, float]], dict[str, Any]]:
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
    return truth, {"path": relative(path), "bytes": len(payload), "sha256": digest, "rows": len(truth), "read_count": 1, "coordinate_rows_omitted": True}


def _sealed_baselines() -> dict[str, dict[str, float]]:
    phase108 = read_json(PHASE108_RESULT, "Phase108 sealed aggregate")
    if phase108.get("aggregate", {}).get("candidate_macro_score_m") is None:
        raise fail("Phase108 aggregate missing")
    phase82 = read_json(PHASE82_RESULT, "Phase82 sealed aggregate")
    return {
        "phase82": {route: float(phase82["routes"][route]["candidate"]["score_m"]) for route in ROUTES},
        "phase108": {route: float(phase108["routes"][route]["candidate"]["score_m"]) for route in ROUTES},
    }


def _route_report(route: str, structural: dict[str, Any], freeze: dict[str, Any], p82: Any, baselines: dict[str, dict[str, float]], accounting: dict[str, int]) -> dict[str, Any]:
    report: dict[str, Any] = {"dataset_id": route, "truth_read": False, "accuracy_scored": False}
    try:
        seal = structural["routes"][route]["solution_hash_seal"]
        ordered, candidate, candidate_meta = _read_candidate_once(_candidate_path(structural, route), seal, route, p82)
        accounting["candidate_solution_reads"] += 1
        report["candidate_solution"] = candidate_meta
        truth, truth_meta = _read_truth_once(_truth_path(freeze, route), freeze["truth_cohort"]["routes"][route], route, p82)
        accounting["truth_reads"] += 1
        report["truth"] = truth_meta
        report["truth_read"] = True
        score = p82.P76._score_prediction(candidate, truth, EXPECTED_MISSING[route], route, ordered)
        accounting["accuracy_calculations"] += 1
        phase82_score = baselines["phase82"][route]
        phase108_score = baselines["phase108"][route]
        checks = {
            "candidate_schema_and_exact_alignment": len(ordered) == DOMAIN_ROWS[route],
            "candidate_prediction_domain_coverage_exact": score.get("prediction_domain_coverage") == 1.0,
            "candidate_finite_and_earth_valid": score.get("finite") is True,
            "candidate_over_70_mps_count_zero": score.get("over_70_mps_count") == 0,
            "candidate_route_score_at_most_3m": score.get("score_m", math.inf) <= 3.0,
            "candidate_no_route_regression_vs_phase82": score.get("score_m", math.inf) <= phase82_score,
            "candidate_no_route_regression_vs_phase108": score.get("score_m", math.inf) <= phase108_score,
        }
        report.update({
            "accuracy_scored": True,
            "candidate": score,
            "baseline_phase82_same_route": {"score_m": phase82_score},
            "baseline_phase108_offsetless_qr": {"score_m": phase108_score},
            "gates": {"passed": all(checks.values()), "checks": checks, "failures": [key for key, value in checks.items() if value is not True]},
        })
    except Exception as exc:
        report["failure"] = str(exc)
        report.setdefault("gates", {"passed": False, "checks": {}, "failures": ["route_evaluation"]})
    return report


def _markdown(result: dict[str, Any]) -> str:
    aggregate = result.get("aggregate", {})
    lines = [
        "# Phase112 main output offset truth-only accuracy result", "",
        f"- status: `{result.get('status')}`",
        "- Native solver/raw/base: not rerun; Phase112 sealed solutions reused",
        "- Truth: one official file read per route by the authorized evaluator subprocess",
        "- Solution/truth coordinate rows: omitted from this result", "",
        f"- Candidate macro: `{aggregate.get('candidate_macro_score_m')}` m",
        f"- Phase82 same-route macro: `{aggregate.get('phase82_same_route_macro_m')}` m",
        f"- Phase108 offsetless QR macro: `{aggregate.get('phase108_offsetless_qr_macro_m')}` m",
        f"- Strict `0.782 m` gate: `{result.get('strict_0_782_gate', {}).get('passed')}`", "",
        "| Route | Candidate (m) | Phase82 (m) | Phase108 QR (m) | Truth read | Gates |",
        "|---|---:|---:|---:|---:|---|",
    ]
    for route in ROUTES:
        item = result.get("routes", {}).get(route, {})
        lines.append(f"| `{route}` | `{item.get('candidate', {}).get('score_m')}` | `{item.get('baseline_phase82_same_route', {}).get('score_m')}` | `{item.get('baseline_phase108_offsetless_qr', {}).get('score_m')}` | `{item.get('truth_read')}` | `{item.get('gates', {}).get('passed')}` |")
    lines.extend(["", "Failure is fail-closed; accuracy GO does not authorize release, publication, or Kaggle submission.", ""])
    return "\n".join(lines)


def evaluate(result_path: Path = RESULT_JSON) -> dict[str, Any]:
    manifest = verify_manifest()
    verify_authorization()
    freeze = verify_phase108_freeze()
    structural = verify_structural_result()
    p82 = _load_phase82()
    baselines = _sealed_baselines()
    accounting = {"candidate_solution_reads": 0, "truth_reads": 0, "accuracy_calculations": 0}
    reports = {route: _route_report(route, structural, freeze, p82, baselines, accounting) for route in ROUTES}
    scored = [reports[route] for route in ROUTES if reports[route].get("accuracy_scored") is True]
    candidate_macro = sum(item["candidate"]["score_m"] for item in scored) / 2.0 if len(scored) == 2 else None
    phase82_macro = sum(baselines["phase82"].values()) / 2.0
    phase108_macro = sum(baselines["phase108"].values()) / 2.0
    gates = {
        "exact_two_routes_one_evaluation_each": len(scored) == 2 and accounting["truth_reads"] == 2,
        "candidate_output_schema_and_exact_alignment": len(scored) == 2 and all(report["gates"]["checks"].get("candidate_schema_and_exact_alignment") is True for report in scored),
        "candidate_prediction_domain_coverage_exact": len(scored) == 2 and all(report["candidate"].get("prediction_domain_coverage") == 1.0 for report in scored),
        "candidate_all_finite_and_earth_valid": len(scored) == 2 and all(report["candidate"].get("finite") is True for report in scored),
        "candidate_over_70_mps_count_zero": len(scored) == 2 and all(report["candidate"].get("over_70_mps_count") == 0 for report in scored),
        "candidate_each_route_score_at_most_3m": len(scored) == 2 and all(report["candidate"].get("score_m", math.inf) <= 3.0 for report in scored),
        "candidate_no_route_regression_vs_phase82_same_route": len(scored) == 2 and all(reports[route]["candidate"]["score_m"] <= baselines["phase82"][route] for route in ROUTES),
        "candidate_no_route_regression_vs_phase108_offsetless_qr": len(scored) == 2 and all(reports[route]["candidate"]["score_m"] <= baselines["phase108"][route] for route in ROUTES),
        "candidate_macro_score_at_most_2m": candidate_macro is not None and candidate_macro <= 2.0,
        "candidate_macro_score_strict_at_most_0_782m": candidate_macro is not None and candidate_macro <= 0.782,
        "truth_read_only_by_one_evaluator_subprocess": accounting["truth_reads"] == 2 and all(reports[route].get("truth_read") is True for route in ROUTES),
        "no_solution_rows_in_result": True,
        "no_solver_raw_base_mat_precomputed_pdc_kaggle": True,
    }
    failed = [key for key, value in gates.items() if value is not True]
    result = {
        "schema_version": RESULT_SCHEMA, "phase": 112, "execution_label": "Luna Max",
        "status": "go-phase112-main-output-offset-truth-only-accuracy" if all(gates.values()) else "no-go-phase112-main-output-offset-truth-only-accuracy",
        "decision": "truth-only accuracy evaluated; release remains separately unauthorized" if all(gates.values()) else "truth-only accuracy gate failed closed; preserve artifacts and do not rerun",
        "candidate": CANDIDATE_ID, "accuracy_scored": len(scored) == 2, "routes": reports,
        "aggregate": {"candidate_macro_score_m": candidate_macro, "phase82_same_route_macro_m": phase82_macro, "phase108_offsetless_qr_macro_m": phase108_macro, "route_count": 2, "macro_route_order": list(ROUTES), "macro_weighting": "unweighted arithmetic mean over exactly MTV-A then LAX-T"},
        "metric_contract": manifest["metric_contract"], "promotion_gates": gates,
        "strict_0_782_gate": {"threshold_m": 0.782, "candidate_macro_score_m": candidate_macro, "passed": gates["candidate_macro_score_strict_at_most_0_782m"]},
        "failed_gates": failed,
        "read_accounting": {"native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0, "base_rinex_reads": 0, "candidate_solution_reads_for_hash_and_parse": accounting["candidate_solution_reads"], "truth_reads": accounting["truth_reads"], "truth_reads_per_route": 1, "accuracy_calculations": accounting["accuracy_calculations"], "reruns": 0, "fallbacks": 0, "mat_precomputed_phone_coordinate_pdc_reads": 0, "kaggle_or_token_access": 0, "truth_reads_by_process": "Phase112 truth-only evaluator subprocess"},
        "forbidden_lanes": {"native_solver_rerun": False, "raw_or_base_rerun": False, "mat": False, "precomputed_phone_coordinates": False, "pdc": False, "kaggle_or_token": False, "solution_rows_in_result": False},
        "solution_output_published": False, "release_or_submission_authorized": False,
        "authority": {"phase112_structural_result": {"path": relative(STRUCTURAL_RESULT), "sha256": STRUCTURAL_RESULT_SHA}, "phase112_manifest": {"path": relative(MANIFEST), "sha256": sha256_file(MANIFEST, "Phase112 truth manifest")}, "phase112_authorization": {"path": relative(AUTHORIZATION), "sha256": sha256_file(AUTHORIZATION, "Phase112 truth authorization")}, "evaluator": {"path": relative(EVALUATOR), "sha256": sha256_file(EVALUATOR, "Phase112 truth evaluator")}},
    }
    atomic_json(result_path, result)
    atomic_text(result_path.with_suffix(".md"), _markdown(result))
    return result


def _fail_closed(error: str, result_path: Path = RESULT_JSON) -> dict[str, Any]:
    result = {"schema_version": RESULT_SCHEMA, "phase": 112, "execution_label": "Luna Max", "status": "no-go-phase112-main-output-offset-truth-only-accuracy", "decision": "truth-only evaluator failed closed; preserve artifacts and do not rerun", "candidate": CANDIDATE_ID, "error": error, "accuracy_scored": False, "strict_0_782_gate": {"threshold_m": 0.782, "passed": False}, "read_accounting": {"native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0, "base_rinex_reads": 0, "truth_reads": 0, "candidate_solution_reads_for_hash_and_parse": 0, "accuracy_calculations": 0, "reruns": 0, "fallbacks": 0, "mat_precomputed_phone_coordinate_pdc_reads": 0, "kaggle_or_token_access": 0}, "forbidden_lanes": {"native_solver_rerun": False, "raw_or_base_rerun": False, "mat": False, "precomputed_phone_coordinates": False, "pdc": False, "kaggle_or_token": False, "solution_rows_in_result": False}, "solution_output_published": False, "release_or_submission_authorized": False}
    atomic_json(result_path, result)
    atomic_text(result_path.with_suffix(".md"), _markdown(result))
    return result


def main(argv: list[str] | None = None) -> int:
    import argparse
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
                _fail_closed(str(exc), args.result_json)
            except Exception:
                pass
        print(f"phase112 truth-only evaluator: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
