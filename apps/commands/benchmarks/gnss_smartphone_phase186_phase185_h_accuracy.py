#!/usr/bin/env python3
"""Single-route H accuracy evaluator for the sealed Phase185 output.

Verification reads only source and sealed metadata.  ``--evaluate`` is the
separate truth-authorized mode: it reads the immutable Phase185 CSV once and
the pinned H development truth once, then delegates parsing, exact-key joins,
Haversine distances, and linear P50/P95 scoring to the pinned Phase76/Phase74
helpers.  The result contains aggregate metrics only.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[3]
EVALUATOR = Path(__file__).resolve()
TESTS = ROOT / "tests/test_smartphone_phase186_phase185_h_accuracy.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase186_phase185_h_accuracy_manifest_v1.json"
RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase186_phase185_h_accuracy_result_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase186_phase185_h_accuracy_authorization_v1.json"
PHASE185_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase185_h_native_source_tdcp_huber_result_v1.json"
PHASE185_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase185_h_native_source_tdcp_huber_manifest_v1.json"
PHASE183_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase183_phase182_accuracy_result_v1.json"
PHASE183_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase183_phase182_accuracy_manifest_v1.json"
P76_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase76_phase75_accuracy_integrity_recovery.py"
P74_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase74_phase73_miss_mask_accuracy.py"

ROUTE = "2021-08-24-20-32-us-ca-mtv-h/pixel5"
LABEL = "MTV-H"
MODELED_EPOCHS = 3140
CANDIDATE_ROWS = 3139
TRUTH_ROWS = 3139
EARTH_RADIUS_M = 6371008.8
STRICT_THRESHOLD_M = 0.782
BASELINE_PHASE183_M = 1.2874762859947766

PHASE185_RESULT_SHA = "a7b5667ec9f7f060eab953aac6977f71aef44d2e737712c04e128a33f668f044"
PHASE185_RESULT_COMMIT = "e9bc385d4984077e059a928a73072576694fa23c"
PHASE185_MANIFEST_SHA = "bf4a8dfc61555f91d133ac94ba47747091be6bd5fb3f2dde019f3cb979e5d973"
PHASE185_MANIFEST_COMMIT = "0d9dba0f3b4c2e10c5ae512928b76e6fb1b37820"
PHASE183_RESULT_SHA = "b6dc71f2591e036cad9242834f707657c152ba7c99cc034f4f3f66221cd79898"
PHASE183_RESULT_COMMIT = "3449e8850a7f8b2dbd6f87b9a3298ac4e7fd3e4d"
PHASE183_MANIFEST_SHA = "4484ec4213885c1dcb8c2c2a732cd6c9dcf042330411aabb1491f96822f0d3e6"
PHASE183_MANIFEST_COMMIT = "559d7c3d44c96e3f2c82d3c939d1be18a8833ecf"
P76_SHA = "513e5970ee58846963f077b9960944e5fbf5f3535e5be24ca6e72493785f7761"
P76_COMMIT = "7a8e77ce03f6f4354cf2f541098a53628faa23d5"
P74_SHA = "bf07ff7ccb9f9efce5a759f2fabe57b7dc0c24ac5acdaf57c6ad9fc88a3fd761"

SCHEMA = "smartphone-r5-phase186-phase185-h-accuracy-manifest.v1"
RESULT_SCHEMA = "smartphone-r5-phase186-phase185-h-accuracy-result.v1"


class Phase186Error(ValueError):
    """Raised when the Phase186 evaluation contract fails closed."""


def fail(message: str) -> Phase186Error:
    return Phase186Error(message)


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def _no_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise fail(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_no_duplicates)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, Phase186Error) as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def static_hash(path: Path, label: str) -> str:
    lowered = str(path).lower()
    forbidden = {
        "opaque_solution_output.csv", "ground_truth.csv", "device_gnss.csv",
        "device_imu.csv", "brdc.nav", "base.obs",
    }
    if path.name.lower() in forbidden or "/truth/" in lowered or ".mat" in lowered or ".pdc" in lowered or "kaggle" in lowered:
        raise fail(f"payload hash forbidden before authorization: {label}")
    if not path.is_file():
        raise fail(f"missing static artifact: {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_helpers() -> Any:
    if static_hash(P76_PATH, "Phase76 parser") != P76_SHA:
        raise fail("Phase76 parser hash changed")
    if static_hash(P74_PATH, "Phase74 metric helper") != P74_SHA:
        raise fail("Phase74 metric helper hash changed")
    spec = importlib.util.spec_from_file_location("phase76_helpers_phase186", P76_PATH)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load Phase76 parser: {P76_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def safe_deferred_path(value: Any, basename: str, fragment: str, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise fail(f"missing {label} path")
    path = Path(value)
    if path.is_absolute() or ".." in path.parts or path.name != basename or fragment not in value:
        raise fail(f"unsafe {label} path: {value!r}")
    return value


def _pin_record(path: Path, expected_sha: str, label: str) -> dict[str, Any]:
    value = read_json(path, label)
    if static_hash(path, label) != expected_sha:
        raise fail(f"{label} hash changed")
    return value


def verify_metadata() -> dict[str, Any]:
    """Verify all provenance without opening candidate or truth payloads."""
    phase185 = _pin_record(PHASE185_RESULT, PHASE185_RESULT_SHA, "Phase185 result")
    phase185_manifest = _pin_record(PHASE185_MANIFEST, PHASE185_MANIFEST_SHA, "Phase185 manifest")
    phase183 = _pin_record(PHASE183_RESULT, PHASE183_RESULT_SHA, "Phase183 result")
    phase183_manifest = _pin_record(PHASE183_MANIFEST, PHASE183_MANIFEST_SHA, "Phase183 manifest")
    if phase185.get("schema_version") != "smartphone-r5-phase185-h-native-source-tdcp-huber-result.v1" or phase185.get("status") != "completed-structural-not-accuracy":
        raise fail("Phase185 result identity/status changed")
    if phase185.get("execution", {}).get("native_return_code") != 0:
        raise fail("Phase185 native return code is not zero")
    recipe = phase185.get("recipe", {})
    if recipe.get("phase184_source_tdcp_huber_k") is not True or recipe.get("phase184_tdcp_setting_type") != "Street" or recipe.get("phase184_tdcp_huber_k") != 0.2:
        raise fail("Phase184 selector/type mapping is not pinned")
    output = phase185.get("output", {})
    candidate_path = safe_deferred_path(output.get("opaque_solution_path"), "opaque_solution_output.csv", "/phase185-h-native-source-tdcp-huber-v1/", "candidate")
    candidate = {
        "source": "Phase185 opaque H native output; evaluator input only",
        "path": candidate_path,
        "sha256": output.get("opaque_solution_sha256"),
        "bytes": output.get("opaque_solution_bytes"),
        "rows": output.get("published_data_rows"),
        "modeled_epochs": output.get("raw_epoch_keys"),
        "warmup_epoch_excluded": output.get("warmup_epoch_excluded"),
        "pixel5_offset_reapplication": False,
        "opaque_only": True,
    }
    if candidate["sha256"] is None or len(candidate["sha256"]) != 64 or candidate["bytes"] != 251174 or candidate["rows"] != CANDIDATE_ROWS or candidate["modeled_epochs"] != MODELED_EPOCHS or candidate["warmup_epoch_excluded"] is not True:
        raise fail("Phase185 candidate metadata changed")
    if phase185_manifest.get("selector_contract", {}).get("pixel5_offset_reapplication") is not False:
        raise fail("Phase185 Pixel5 offset policy changed")
    if phase185.get("native_summary", {}).get("output_contract_finite_coordinates") is not True:
        raise fail("Phase185 finite-output gate is not sealed")

    truth = phase183.get("truth", {})
    truth_path = safe_deferred_path(truth.get("path", phase183_manifest.get("truth_reference", {}).get("path")), "ground_truth.csv", "/truth/", "H truth")
    truth_ref = {
        "source": "Phase183-pinned H development truth metadata; official archive provenance",
        "path": truth_path,
        "sha256": truth.get("sha256"),
        "bytes": truth.get("bytes"),
        "rows": truth.get("rows"),
        "archive_member": phase183_manifest.get("truth_reference", {}).get("archive_member"),
        "archive_crc32": phase183_manifest.get("truth_reference", {}).get("archive_crc32"),
        "role": truth.get("role"),
    }
    if truth_ref["sha256"] != "a55f452e611426693677fbeacde227fc80e5f03f6040d04aef9d6a2baf08d249" or truth_ref["bytes"] != 305432 or truth_ref["rows"] != TRUTH_ROWS:
        raise fail("H truth metadata pin changed")
    if "not heldout" not in str(truth_ref["role"]).lower() or "leaderboard" not in str(truth_ref["role"]).lower():
        raise fail("H truth role is not explicitly development/non-heldout")
    if phase183_manifest.get("truth_reference", {}).get("prior_phase44_materialization") is not True:
        raise fail("H truth prior-materialization provenance changed")

    if phase183.get("dataset_id") != ROUTE or phase183.get("metrics", {}).get("route_score_m") != BASELINE_PHASE183_M:
        raise fail("Phase183 H baseline metadata changed")
    return {
        "candidate": candidate,
        "truth": truth_ref,
        "phase185_result": {"path": str(PHASE185_RESULT.relative_to(ROOT)), "sha256": PHASE185_RESULT_SHA, "commit": PHASE185_RESULT_COMMIT},
        "phase185_manifest": {"path": str(PHASE185_MANIFEST.relative_to(ROOT)), "sha256": PHASE185_MANIFEST_SHA, "commit": PHASE185_MANIFEST_COMMIT},
        "phase183_result": {"path": str(PHASE183_RESULT.relative_to(ROOT)), "sha256": PHASE183_RESULT_SHA, "commit": PHASE183_RESULT_COMMIT},
        "phase183_manifest": {"path": str(PHASE183_MANIFEST.relative_to(ROOT)), "sha256": PHASE183_MANIFEST_SHA, "commit": PHASE183_MANIFEST_COMMIT},
    }


def metric_contract() -> dict[str, Any]:
    return {
        "candidate_parser": "Phase74 _parse_submission",
        "truth_parser": "Phase76 _parse_truth_dictreader (CSV DictReader; required fields by name; optional columns allowed)",
        "join_and_score": "Phase76 _score_prediction delegating exact-key/Haversine/percentile helpers",
        "key": "(phone, UnixTimeMillis)",
        "matching": "exact integer key intersection only",
        "duplicate_policy": "duplicate prediction/truth keys or duplicate truth header fields fail closed",
        "earth_radius_m": EARTH_RADIUS_M,
        "distance": "spherical Haversine per row",
        "percentile": "linear interpolation at rank (n - 1) * q",
        "route_scalar": "(P50 + P95) / 2 in metres",
        "strict_promotion_comparator": "candidate_route_score_m < 0.782",
        "strict_promotion_threshold_m": STRICT_THRESHOLD_M,
        "pixel5_offset_reapplication": False,
        "missing_truth_policy": "no missing truth keys; no interpolation, nearest, edge hold, extrapolation, or fill",
    }


def verify_manifest() -> dict[str, Any]:
    manifest = read_json(MANIFEST, "Phase186 manifest")
    expected = {
        "schema_version": SCHEMA,
        "phase": 186,
        "execution_label": "Luna Max",
        "status": "frozen-before-phase186-truth-read",
        "route_order": [ROUTE],
    }
    for key, value in expected.items():
        if manifest.get(key) != value:
            raise fail(f"manifest/{key} changed")
    metadata = verify_metadata()
    if manifest.get("candidate_reference") != metadata["candidate"]:
        raise fail("manifest candidate reference changed")
    if manifest.get("truth_reference") != metadata["truth"]:
        raise fail("manifest truth reference changed")
    if manifest.get("metric_contract") != metric_contract():
        raise fail("manifest metric contract changed")
    authority = manifest.get("authority", {})
    static_pins = {
        "phase185_result": (PHASE185_RESULT, PHASE185_RESULT_SHA, PHASE185_RESULT_COMMIT),
        "phase185_manifest": (PHASE185_MANIFEST, PHASE185_MANIFEST_SHA, PHASE185_MANIFEST_COMMIT),
        "phase183_result": (PHASE183_RESULT, PHASE183_RESULT_SHA, PHASE183_RESULT_COMMIT),
        "phase183_manifest": (PHASE183_MANIFEST, PHASE183_MANIFEST_SHA, PHASE183_MANIFEST_COMMIT),
        "phase76_parser": (P76_PATH, P76_SHA, P76_COMMIT),
        "phase74_metric_helper": (P74_PATH, P74_SHA, None),
        "evaluator": (EVALUATOR, authority.get("evaluator", {}).get("sha256"), None),
        "focused_tests": (TESTS, authority.get("focused_tests", {}).get("sha256"), None),
    }
    for name, (path, expected_sha, commit) in static_pins.items():
        item = authority.get(name)
        if not isinstance(item, Mapping) or item.get("path") != str(path.relative_to(ROOT)):
            raise fail(f"authority/{name} path changed")
        if not isinstance(expected_sha, str) or len(expected_sha) != 64 or static_hash(path, f"authority/{name}") != expected_sha or item.get("sha256") != expected_sha:
            raise fail(f"authority/{name} hash changed")
        if commit is not None and item.get("commit") != commit:
            raise fail(f"authority/{name} commit changed")
    if manifest.get("comparison_baseline") != {"phase183_h_route_score_m": BASELINE_PHASE183_M, "target_strict_less_than_m": STRICT_THRESHOLD_M, "role": "H development/train diagnostic; not heldout or leaderboard proof"}:
        raise fail("manifest baseline/role changed")
    if manifest.get("read_accounting_before_truth") != {
        "candidate_payload_reads": 0, "truth_payload_reads": 0,
        "candidate_coordinate_interpretations": 0, "accuracy_calculations": 0,
        "native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0,
        "mat_pdc_precomputed_coordinate_reads": 0, "kaggle_or_token_access": 0,
        "reruns": 0, "fallbacks": 0, "repairs": 0,
    }:
        raise fail("manifest pre-truth accounting changed")
    policy = manifest.get("execution_policy", {})
    for key, value in {
        "candidate_reads": 1, "truth_reads": 1, "accuracy_calculations": 1,
        "native_solver_invocations": 0, "raw_reads": 0,
        "mat_pdc_precomputed_coordinate_reads": 0, "kaggle_or_token_access": 0,
        "reruns": 0, "fallbacks": 0, "repairs": 0,
        "solution_publication": False, "candidate_output_evaluation_only": True,
    }.items():
        if policy.get(key) != value:
            raise fail(f"manifest execution policy/{key} changed")
    return {"manifest": manifest, "metadata": metadata}


def verify_pre_truth() -> dict[str, Any]:
    verify_manifest()
    return {
        "candidate_payload_reads": 0,
        "truth_payload_reads": 0,
        "candidate_coordinate_interpretations": 0,
        "accuracy_calculations": 0,
        "native_solver_invocations": 0,
        "raw_gnss_imu_navigation_reads": 0,
        "mat_pdc_precomputed_coordinate_reads": 0,
        "kaggle_or_token_access": 0,
        "reruns": 0,
        "fallbacks": 0,
        "repairs": 0,
        "truth_only_authorized": False,
    }


def verify_payload_seal(payload: bytes, pin: Mapping[str, Any], label: str) -> None:
    if len(payload) != pin.get("bytes") or hashlib.sha256(payload).hexdigest() != pin.get("sha256"):
        raise fail(f"{label} SHA-256/byte seal mismatch")


def ensure_fresh_output(path: Path) -> None:
    if path.exists():
        if path.is_dir() and any(path.iterdir()):
            raise fail(f"refusing to overwrite nonempty result directory: {path}")
        if path.is_file() or (path.is_dir() and not any(path.iterdir())):
            raise fail(f"refusing to overwrite existing result path: {path}")


def read_candidate_once(path: Path, pin: Mapping[str, Any], helpers: Any) -> tuple[list[tuple[int, float, float]], dict[int, tuple[float, float]]]:
    payload = path.read_bytes()
    verify_payload_seal(payload, pin, "candidate")
    ordered, mapping = helpers.P74._parse_submission(payload, ROUTE)
    if len(ordered) != CANDIDATE_ROWS or len(mapping) != CANDIDATE_ROWS:
        raise fail("candidate row alignment mismatch")
    return ordered, mapping


def read_truth_once(path: Path, pin: Mapping[str, Any], helpers: Any) -> dict[int, tuple[float, float]]:
    payload = path.read_bytes()
    verify_payload_seal(payload, pin, "truth")
    truth = helpers._parse_truth_dictreader(payload, ROUTE)
    if len(truth) != TRUTH_ROWS:
        raise fail("truth row count mismatch")
    return truth


def route_gates(ordered_count: int, score: Mapping[str, Any]) -> dict[str, bool]:
    return {
        "candidate_exact_alignment": ordered_count == CANDIDATE_ROWS,
        "candidate_prediction_domain_exact": score.get("prediction_domain_coverage") == 1.0,
        "truth_domain_exact": score.get("truth_row_coverage") == 1.0 and score.get("matched_rows") == TRUTH_ROWS,
        "candidate_all_finite_and_earth_valid": score.get("finite") is True,
        "velocity_gate_over_70_mps_zero": score.get("over_70_mps_count") == 0,
        "route_score_finite": finite(score.get("score_m")),
    }


def strict_route_gate(value: Any) -> bool:
    return finite(value) and float(value) < STRICT_THRESHOLD_M


def evaluate(authorization_path: Path = AUTHORIZATION, result_path: Path = RESULT) -> dict[str, Any]:
    frozen = verify_manifest()
    authorization = read_json(authorization_path, "Phase186 authorization")
    if authorization.get("schema_version") != "smartphone-r5-phase186-phase185-h-accuracy-authorization.v1" or authorization.get("manifest_schema") != SCHEMA or authorization.get("allow_truth_read") is not True:
        raise fail("truth authorization is missing or mismatched")
    manifest_sha = static_hash(MANIFEST, "Phase186 manifest")
    if authorization.get("manifest_sha256") != manifest_sha:
        raise fail("truth authorization does not pin the current Phase186 manifest")
    if authorization.get("candidate_sha256") != frozen["metadata"]["candidate"]["sha256"] or authorization.get("truth_sha256") != frozen["metadata"]["truth"]["sha256"]:
        raise fail("truth authorization candidate/truth pins do not match the frozen manifest")
    ensure_fresh_output(result_path)
    helpers = load_helpers()
    candidate_pin = frozen["metadata"]["candidate"]
    truth_pin = frozen["metadata"]["truth"]
    candidate_path = ROOT / candidate_pin["path"]
    truth_path = ROOT / truth_pin["path"]
    ordered, candidate = read_candidate_once(candidate_path, candidate_pin, helpers)
    truth = read_truth_once(truth_path, truth_pin, helpers)
    score = helpers._score_prediction(candidate, truth, None, ROUTE, ordered)
    gates = route_gates(len(ordered), score)
    result = {
        "schema_version": RESULT_SCHEMA,
        "phase": 186,
        "execution_label": "Luna Max",
        "status": "completed-h-development-accuracy-diagnostic",
        "manifest": str(MANIFEST.relative_to(ROOT)),
        "candidate": {"path": candidate_pin["path"], "sha256": candidate_pin["sha256"], "bytes": candidate_pin["bytes"], "rows": CANDIDATE_ROWS},
        "truth": {"path": truth_pin["path"], "sha256": truth_pin["sha256"], "bytes": truth_pin["bytes"], "rows": TRUTH_ROWS, "role": truth_pin["role"]},
        "metric": {"route": LABEL, "score_m": score["score_m"], "p50_m": score["p50_m"], "p95_m": score["p95_m"], "matched_rows": score["matched_rows"], "prediction_domain_coverage": score["prediction_domain_coverage"], "truth_domain_coverage": score["truth_row_coverage"], "over_70_mps_count": score["over_70_mps_count"], "finite": score["finite"]},
        "gates": {**gates, "strict_score_less_than_0_782": strict_route_gate(score["score_m"]), "pixel5_offset_not_reapplied": True},
        "read_accounting": {"candidate_reads": 1, "truth_reads": 1, "accuracy_calculations": 1, "native_solver_invocations": 0, "raw_reads": 0, "mat_pdc_precomputed_coordinate_reads": 0, "kaggle_or_token_access": 0, "reruns": 0, "fallbacks": 0, "repairs": 0, "coordinate_rows_in_result": False},
        "comparison": {"phase183_h_route_score_m": BASELINE_PHASE183_M, "target_strict_less_than_m": STRICT_THRESHOLD_M, "role": "H development/train diagnostic; not heldout or leaderboard proof"},
    }
    result_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with result_path.open("x", encoding="utf-8", newline="") as handle:
            handle.write(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n")
    except FileExistsError as exc:
        raise fail(f"refusing to overwrite result: {result_path}") from exc
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--verify", action="store_true", help="verify source/metadata without payload reads")
    mode.add_argument("--evaluate", action="store_true", help="run the separately authorized one-route truth evaluation")
    parser.add_argument("--authorization", type=Path, default=AUTHORIZATION)
    parser.add_argument("--result", type=Path, default=RESULT)
    args = parser.parse_args(argv)
    try:
        if args.verify:
            print(json.dumps(verify_pre_truth(), sort_keys=True))
        else:
            print(json.dumps(evaluate(args.authorization, args.result), sort_keys=True))
        return 0
    except Phase186Error as exc:
        print(f"phase186: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
