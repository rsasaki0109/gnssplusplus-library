#!/usr/bin/env python3
"""Single-route H scorer for a frozen Phase188 native output.

``--verify`` reads only the frozen manifest and pinned source metadata.
``--evaluate`` is the separately authorized one-pass mode: it
reads the immutable candidate once and the pinned H development truth once,
then delegates parsing, exact-key joining, Haversine distance, and linear
P50/P95 scoring to the sealed Phase76/Phase74 helpers.  The result contains
aggregate metrics only; candidate coordinate rows are never serialized.
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
P76_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase76_phase75_accuracy_integrity_recovery.py"
P74_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase74_phase73_miss_mask_accuracy.py"

ROUTE = "2021-08-24-20-32-us-ca-mtv-h/pixel5"
LABEL = "MTV-H"
MODELED_EPOCHS = 3140
PUBLISHED_ROWS = 3139
TRUTH_ROWS = 3139
EARTH_RADIUS_M = 6371008.8
STRICT_THRESHOLD_M = 0.782
PHASE182_BASELINE_M = 1.2874762859947766
P76_SHA = "513e5970ee58846963f077b9960944e5fbf5f3535e5be24ca6e72493785f7761"
P74_SHA = "bf07ff7ccb9f9efce5a759f2fabe57b7dc0c24ac5acdaf57c6ad9fc88a3fd761"
P76_COMMIT = "7a8e77ce03f6f4354cf2f541098a53628faa23d5"

MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase189_phase188_h_accuracy_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase189_phase188_h_accuracy_authorization_v1.json"
RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase189_phase188_h_accuracy_result_v1.json"
SCHEMA = "smartphone-r5-phase189-phase188-h-accuracy-manifest.v1"
RESULT_SCHEMA = "smartphone-r5-phase189-phase188-h-accuracy-result.v1"


class Phase189Error(ValueError):
    """Raised when the frozen single-route evaluation contract fails closed."""


def fail(message: str) -> Phase189Error:
    return Phase189Error(message)


def finite(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


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
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, Phase189Error) as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def static_hash(path: Path, label: str) -> str:
    lowered = str(path).lower()
    if path.name.lower() in {
        "opaque_solution_output.csv", "ground_truth.csv", "device_gnss.csv",
        "device_imu.csv", "brdc.nav", "base.obs",
    } or "/truth/" in lowered or ".mat" in lowered or ".pdc" in lowered or "kaggle" in lowered:
        raise fail(f"payload hash forbidden before authorization: {label}")
    if not path.is_file():
        raise fail(f"missing static artifact {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_helpers() -> tuple[Any, Any]:
    if static_hash(P76_PATH, "Phase76 parser") != P76_SHA:
        raise fail("Phase76 parser hash changed")
    if static_hash(P74_PATH, "Phase74 metric helper") != P74_SHA:
        raise fail("Phase74 metric helper hash changed")
    p74_spec = importlib.util.spec_from_file_location("phase74_helpers_phase189", P74_PATH)
    p76_spec = importlib.util.spec_from_file_location("phase76_helpers_phase189", P76_PATH)
    if p74_spec is None or p74_spec.loader is None or p76_spec is None or p76_spec.loader is None:
        raise fail("unable to load pinned Phase74/Phase76 helpers")
    p74 = importlib.util.module_from_spec(p74_spec)
    p74_spec.loader.exec_module(p74)
    p76 = importlib.util.module_from_spec(p76_spec)
    p76_spec.loader.exec_module(p76)
    return p76, p74


def safe_payload_path(value: Any, basename: str, fragment: str, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise fail(f"missing {label} path")
    path = Path(value)
    if path.is_absolute() or ".." in path.parts or path.name != basename or fragment not in value:
        raise fail(f"unsafe {label} path: {value!r}")
    return value


def verify_manifest(path: Path = MANIFEST) -> dict[str, Any]:
    manifest = read_json(path, "Phase189 manifest")
    expected = {
        "schema_version": SCHEMA,
        "phase": 189,
        "execution_label": "Luna Max",
        "status": "frozen-before-phase189-truth-read",
        "route": ROUTE,
        "modeled_epochs": MODELED_EPOCHS,
        "published_rows": PUBLISHED_ROWS,
        "truth_rows": TRUTH_ROWS,
    }
    for key, value in expected.items():
        if manifest.get(key) != value:
            raise fail(f"manifest/{key} changed")
    candidate = manifest.get("candidate")
    truth = manifest.get("truth")
    if not isinstance(candidate, Mapping) or not isinstance(truth, Mapping):
        raise fail("manifest candidate/truth metadata missing")
    safe_payload_path(candidate.get("path"), "opaque_solution_output.csv", "/phase188-h-native-upstream-stop-v1/", "candidate")
    safe_payload_path(truth.get("path"), "ground_truth.csv", "/truth/", "truth")
    for pin, rows, label in ((candidate, PUBLISHED_ROWS, "candidate"), (truth, TRUTH_ROWS, "truth")):
        if not isinstance(pin.get("sha256"), str) or len(pin["sha256"]) != 64:
            raise fail(f"{label} SHA-256 pin missing")
        if pin.get("rows") != rows or not isinstance(pin.get("bytes"), int) or pin["bytes"] <= 0:
            raise fail(f"{label} row/byte pin invalid")
    if candidate.get("modeled_epochs") != MODELED_EPOCHS or candidate.get("warmup_epoch_excluded") is not True:
        raise fail("candidate modeled/warmup contract changed")
    if candidate.get("interpolated_epochs") != 0 or candidate.get("edge_hold_epochs") != 0 or candidate.get("unresolved_epochs") != 0:
        raise fail("candidate interpolation/hold contract changed")
    if candidate.get("pixel5_offset_reapplication") is not False:
        raise fail("candidate Pixel5 offset policy changed")
    if truth.get("role") != "development/train; not heldout or leaderboard proof":
        raise fail("truth role is not the pinned development role")
    metric = manifest.get("metric_contract")
    if metric != {
        "candidate_parser": "Phase74 _parse_submission",
        "truth_parser": "Phase76 _parse_truth_dictreader",
        "join": "exact (phone, UnixTimeMillis) keys; no interpolation/hold/fill",
        "distance": "Phase74 spherical Haversine, radius 6371008.8 m",
        "percentile": "Phase74 linear rank (n - 1) * q",
        "route_scalar": "(P50 + P95) / 2 in metres",
        "pixel5_offset_reapplication": False,
    }:
        raise fail("metric contract changed")
    authority = manifest.get("authority")
    if not isinstance(authority, Mapping):
        raise fail("manifest authority missing")
    for name, path_obj, expected_sha in (
        ("evaluator", EVALUATOR, authority.get("evaluator", {}).get("sha256") if isinstance(authority.get("evaluator"), Mapping) else None),
        ("focused_tests", ROOT / "tests/test_smartphone_phase189_phase188_h_accuracy.py", authority.get("focused_tests", {}).get("sha256") if isinstance(authority.get("focused_tests"), Mapping) else None),
        ("phase76_parser", P76_PATH, P76_SHA),
        ("phase74_metric_helper", P74_PATH, P74_SHA),
    ):
        item = authority.get(name)
        if not isinstance(item, Mapping) or item.get("path") != str(path_obj.relative_to(ROOT)):
            raise fail(f"authority/{name} path changed")
        if not isinstance(expected_sha, str) or static_hash(path_obj, f"authority/{name}") != expected_sha or item.get("sha256") != expected_sha:
            raise fail(f"authority/{name} hash changed")
        if name == "phase76_parser" and item.get("commit") != P76_COMMIT:
            raise fail("authority/phase76_parser commit changed")
    if manifest.get("comparison_baseline") != {
        "phase182_h_route_score_m": PHASE182_BASELINE_M,
        "strict_target_m": STRICT_THRESHOLD_M,
        "role": "H development/train diagnostic; not heldout or leaderboard proof",
    }:
        raise fail("comparison baseline changed")
    return manifest


def verify_payload(payload: bytes, pin: Mapping[str, Any], label: str) -> None:
    if len(payload) != pin.get("bytes") or hashlib.sha256(payload).hexdigest() != pin.get("sha256"):
        raise fail(f"{label} SHA-256/byte seal mismatch")


def ensure_fresh(path: Path) -> None:
    if path.exists():
        raise fail(f"refusing to overwrite existing result: {path}")


def route_gates(score: Mapping[str, Any], candidate_rows: int, truth_rows: int) -> dict[str, bool]:
    return {
        "candidate_exact_alignment": score.get("prediction_rows") == candidate_rows,
        "candidate_prediction_domain_exact": score.get("prediction_domain_coverage") == 1.0,
        "truth_domain_exact": score.get("truth_row_coverage") == 1.0 and score.get("matched_rows") == truth_rows,
        "candidate_all_finite_and_earth_valid": score.get("finite") is True,
        "velocity_gate_over_70_mps_zero": score.get("over_70_mps_count") == 0,
        "route_score_finite": finite(score.get("score_m")),
        "strict_route_score_less_than_0_782_m": finite(score.get("score_m")) and float(score["score_m"]) < STRICT_THRESHOLD_M,
    }


def score_payloads(
    candidate_path: Path,
    candidate_pin: Mapping[str, Any],
    truth_path: Path,
    truth_pin: Mapping[str, Any],
    p76: Any,
    p74: Any,
    *,
    route: str,
    result_path: Path | None = None,
) -> dict[str, Any]:
    """Read candidate once, then truth once, and return aggregate score only."""
    if result_path is not None:
        ensure_fresh(result_path)
    candidate_payload = candidate_path.read_bytes()
    verify_payload(candidate_payload, candidate_pin, "candidate")
    try:
        ordered, candidate = p74._parse_submission(candidate_payload, route)
    except Exception as exc:
        raise fail(f"candidate parser rejected payload: {exc}") from exc
    if len(ordered) != int(candidate_pin["rows"]) or len(candidate) != len(ordered):
        raise fail("candidate exact alignment failed")
    truth_payload = truth_path.read_bytes()
    verify_payload(truth_payload, truth_pin, "truth")
    try:
        truth = p76._parse_truth_dictreader(truth_payload, route)
    except Exception as exc:
        raise fail(f"truth parser rejected payload: {exc}") from exc
    if len(truth) != int(truth_pin["rows"]):
        raise fail("truth row count mismatch")
    try:
        score = p76._score_prediction(candidate, truth, None, route, ordered)
    except Exception as exc:
        raise fail(f"exact-key scorer rejected payloads: {exc}") from exc
    gates = route_gates(score, int(candidate_pin["rows"]), int(truth_pin["rows"]))
    return {
        "route": route,
        "candidate": {"path": candidate_pin["path"], "sha256": candidate_pin["sha256"], "bytes": candidate_pin["bytes"], "rows": len(ordered), "read_count": 1},
        "truth": {"path": truth_pin["path"], "sha256": truth_pin["sha256"], "bytes": truth_pin["bytes"], "rows": len(truth), "read_count": 1},
        "metric": {
            "p50_m": score["p50_m"],
            "p95_m": score["p95_m"],
            "route_score_m": score["score_m"],
            "matched_rows": score["matched_rows"],
            "prediction_domain_coverage": score["prediction_domain_coverage"],
            "truth_domain_coverage": score["truth_row_coverage"],
            "over_70_mps_count": score["over_70_mps_count"],
            "finite": score["finite"],
        },
        "gates": gates,
        "read_accounting": {
            "candidate_payload_reads": 1,
            "truth_payload_reads": 1,
            "score_calculations": 1,
            "interpolated_epochs": 0,
            "edge_hold_epochs": 0,
            "pixel5_offset_reapplications": 0,
        },
    }


def verify_pre_truth(manifest_path: Path = MANIFEST) -> dict[str, Any]:
    manifest = verify_manifest(manifest_path)
    return {
        "manifest": str(manifest_path.relative_to(ROOT)),
        "candidate_payload_reads": 0,
        "truth_payload_reads": 0,
        "accuracy_calculations": 0,
        "native_solver_invocations": 0,
        "raw_gnss_imu_navigation_reads": 0,
        "mat_pdc_precomputed_coordinate_reads": 0,
        "kaggle_or_token_access": 0,
        "candidate": {"path": manifest["candidate"]["path"], "sha256": manifest["candidate"]["sha256"]},
        "truth": {"path": manifest["truth"]["path"], "sha256": manifest["truth"]["sha256"]},
    }


def evaluate(
    manifest_path: Path = MANIFEST,
    authorization_path: Path = AUTHORIZATION,
    result_path: Path = RESULT,
) -> dict[str, Any]:
    manifest = verify_manifest(manifest_path)
    authorization = read_json(authorization_path, "Phase189 authorization")
    if authorization.get("schema_version") != "smartphone-r5-phase189-phase188-h-accuracy-authorization.v1" or authorization.get("allow_truth_read") is not True:
        raise fail("truth authorization is missing or mismatched")
    if authorization.get("manifest_sha256") != static_hash(manifest_path, "Phase189 manifest"):
        raise fail("authorization does not pin the current manifest")
    if authorization.get("candidate_sha256") != manifest["candidate"]["sha256"] or authorization.get("truth_sha256") != manifest["truth"]["sha256"]:
        raise fail("authorization candidate/truth pins do not match manifest")
    ensure_fresh(result_path)
    p76, p74 = load_helpers()
    candidate_path = ROOT / manifest["candidate"]["path"]
    truth_path = ROOT / manifest["truth"]["path"]
    score = score_payloads(candidate_path, manifest["candidate"], truth_path, manifest["truth"], p76, p74, route=manifest["route"], result_path=result_path)
    result = {
        "schema_version": RESULT_SCHEMA,
        "phase": 189,
        "execution_label": "Luna Max",
        "status": "completed-h-development-accuracy-diagnostic",
        "manifest": str(manifest_path.relative_to(ROOT)),
        "comparison_baseline_phase182_h_route_score_m": PHASE182_BASELINE_M,
        "metric_contract": manifest["metric_contract"],
        **score,
        "coordinate_rows_published": False,
    }
    try:
        with result_path.open("x", encoding="utf-8") as handle:
            handle.write(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n")
    except FileExistsError as exc:
        raise fail(f"refusing to overwrite existing result: {result_path}") from exc
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--verify", action="store_true")
    mode.add_argument("--evaluate", action="store_true")
    parser.add_argument("--manifest", type=Path, default=MANIFEST)
    parser.add_argument("--authorization", type=Path, default=AUTHORIZATION)
    parser.add_argument("--result", type=Path, default=RESULT)
    args = parser.parse_args(argv)
    try:
        value = verify_pre_truth(args.manifest) if args.verify else evaluate(args.manifest, args.authorization, args.result)
        print(json.dumps(value, sort_keys=True))
        return 0
    except Phase189Error as exc:
        print(f"phase189: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
