#!/usr/bin/env python3
"""Pre-truth H evaluator for the frozen Phase202 native candidate.

``--verify`` validates only the Phase203 manifest and static Phase202/source
metadata.  ``--evaluate`` is authorization-gated and delegates the unchanged
Phase189 exact-key, Haversine, and percentile kernel through the already
frozen Phase199 thin wrapper; it is intentionally not run in this phase.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[3]
EVALUATOR = Path(__file__).resolve()
PHASE199_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase199_phase198_h_accuracy.py"
PHASE196_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase196_phase195_h_accuracy.py"
PHASE189_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase189_phase188_h_accuracy.py"
P76_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase76_phase75_accuracy_integrity_recovery.py"
P74_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase74_phase73_miss_mask_accuracy.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase203_phase202_h_accuracy.py"

ROUTE = "2021-08-24-20-32-us-ca-mtv-h/pixel5"
MODELED_EPOCHS = 3140
PUBLISHED_ROWS = 3139
TRUTH_ROWS = 3139
EARTH_RADIUS_M = 6371008.8
STRICT_THRESHOLD_M = 0.782
PHASE199_BASELINE_M = 1.2751561666667786
PHASE193_BASELINE_M = 1.2780192442543101
PHASE182_BASELINE_M = 1.2874762859947766

PHASE199_SHA = "67e5591f9156e800a205cc7ad40af1ad09d2b1d52686466d0cf3c9ef61178ef4"
PHASE196_SHA = "372b68f2e1b4a83ab17ac9af0c16f172af1b71201983bd786469586f0454368a"
PHASE189_SHA = "525a436707ab7de0202b1130bb6daed4e001686b8396a24b23ed15c669a60a83"
P76_SHA = "513e5970ee58846963f077b9960944e5fbf5f3535e5be24ca6e72493785f7761"
P74_SHA = "bf07ff7ccb9f9efce5a759f2fabe57b7dc0c24ac5acdaf57c6ad9fc88a3fd761"
PHASE202_RESULT_SHA = "3ac02f76104e705cfc02180a444e9040c281944d690ea199a48ccd1e9db113a9"
PHASE202_MANIFEST_SHA = "38428a384b0dc5ca382b4cfbc4951bed2f259dc194ade6c8348506f5321531ee"
PHASE202_CANDIDATE_SHA = "1330b5a758d7a4022279cffcc70c02e8fd286658e79e07f4ca36ed4585083a9f"
PHASE202_CANDIDATE_BYTES = 251174

MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase203_phase202_h_accuracy_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase203_phase202_h_accuracy_authorization_v1.json"
RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase203_phase202_h_accuracy_result_v1.json"
SCHEMA = "smartphone-r5-phase203-phase202-h-accuracy-manifest.v1"
RESULT_SCHEMA = "smartphone-r5-phase203-phase202-h-accuracy-result.v1"


class Phase203Error(ValueError):
    """Raised when the frozen Phase203 contract fails closed."""


def fail(message: str) -> Phase203Error:
    return Phase203Error(message)


def _no_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise fail(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"), object_pairs_hook=_no_duplicates
        )
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, Phase203Error) as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def static_hash(path: Path, label: str) -> str:
    lowered = str(path).lower()
    if path.name.lower() in {
        "opaque_solution_output.csv",
        "ground_truth.csv",
        "device_gnss.csv",
        "device_imu.csv",
        "brdc.nav",
        "base.obs",
    } or "/truth/" in lowered or ".mat" in lowered or ".pdc" in lowered or "kaggle" in lowered:
        raise fail(f"payload hash forbidden before authorization: {label}")
    if not path.is_file():
        raise fail(f"missing static artifact {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def safe_payload_path(value: Any, basename: str, fragment: str, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise fail(f"missing {label} path")
    path = Path(value)
    if path.is_absolute() or ".." in path.parts or path.name != basename or fragment not in value:
        raise fail(f"unsafe {label} path: {value!r}")
    return value


def authority_entry(
    authority: Mapping[str, Any],
    name: str,
    path: Path,
    expected_sha: str | None = None,
) -> None:
    item = authority.get(name)
    if not isinstance(item, Mapping) or item.get("path") != str(path.relative_to(ROOT)):
        raise fail(f"authority/{name} path changed")
    pinned = item.get("sha256")
    if not isinstance(pinned, str) or (expected_sha is not None and pinned != expected_sha):
        raise fail(f"authority/{name} hash pin changed")
    if static_hash(path, f"authority/{name}") != pinned:
        raise fail(f"authority/{name} hash changed")


def verify_phase202_metadata(authority: Mapping[str, Any]) -> None:
    result_path = ROOT / "docs/use_cases/records/smartphone_r5_phase202_h_native_phase201_imu_schedule_result_v1.json"
    manifest_path = ROOT / "docs/use_cases/records/smartphone_r5_phase202_h_native_phase201_imu_schedule_manifest_v1.json"
    result = read_json(result_path, "Phase202 result metadata")
    prior_manifest = read_json(manifest_path, "Phase202 manifest metadata")
    manifest_meta = result.get("manifest")
    if not isinstance(manifest_meta, Mapping) or manifest_meta.get("sha256") != PHASE202_MANIFEST_SHA:
        raise fail("Phase202 result does not pin the frozen manifest")
    if manifest_meta.get("implementation_commit") != "bb75275f6819ca92be361efdc1d5fea6c9d669cd":
        raise fail("Phase202 implementation commit changed")
    execution = result.get("execution")
    policy = result.get("accuracy_and_payload_policy")
    if not isinstance(execution, Mapping) or execution.get("return_code") != 0:
        raise fail("Phase202 native completion metadata changed")
    if not isinstance(policy, Mapping) or policy.get("truth_used") is not False or policy.get("accuracy_calculated") is not False:
        raise fail("Phase202 truth/accuracy boundary changed")
    candidate = result.get("candidate_identity")
    if not isinstance(candidate, Mapping) or candidate.get("sha256") != PHASE202_CANDIDATE_SHA or candidate.get("bytes") != PHASE202_CANDIDATE_BYTES or candidate.get("published_rows") != PUBLISHED_ROWS:
        raise fail("Phase202 candidate seal changed")
    schedule = result.get("phase201_schedule")
    if not isinstance(schedule, Mapping) or schedule.get("configuration_valid") is not True or schedule.get("intervals") != 3139:
        raise fail("Phase202 Phase201 schedule metadata changed")
    if prior_manifest.get("implementation_commit") != "bb75275f6819ca92be361efdc1d5fea6c9d669cd":
        raise fail("Phase202 manifest implementation pin changed")


def load_phase199() -> Any:
    if static_hash(PHASE199_PATH, "Phase199 evaluator") != PHASE199_SHA:
        raise fail("Phase199 evaluator hash changed")
    spec = importlib.util.spec_from_file_location("phase199_accuracy_phase203", PHASE199_PATH)
    if spec is None or spec.loader is None:
        raise fail("unable to load Phase199 evaluator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def verify_manifest(path: Path = MANIFEST) -> dict[str, Any]:
    manifest = read_json(path, "Phase203 manifest")
    expected = {
        "schema_version": SCHEMA,
        "phase": 203,
        "execution_label": "Luna Max",
        "status": "frozen-before-phase203-truth-read",
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
    safe_payload_path(
        candidate.get("path"),
        "opaque_solution_output.csv",
        "/phase202-h-native-phase201-imu-schedule-v1/",
        "candidate",
    )
    safe_payload_path(truth.get("path"), "ground_truth.csv", "/truth/", "truth")
    if candidate.get("sha256") != PHASE202_CANDIDATE_SHA or candidate.get("bytes") != PHASE202_CANDIDATE_BYTES or candidate.get("rows") != PUBLISHED_ROWS:
        raise fail("candidate is not the sealed Phase202 output")
    if candidate.get("modeled_epochs") != MODELED_EPOCHS or candidate.get("warmup_epoch_excluded") is not True:
        raise fail("candidate modeled/warmup contract changed")
    if any(candidate.get(key) != 0 for key in ("interpolated_epochs", "edge_hold_epochs", "unresolved_epochs")):
        raise fail("candidate interpolation/hold contract changed")
    if candidate.get("pixel5_offset_reapplication") is not False or candidate.get("opaque_only") is not True:
        raise fail("candidate publication/offset policy changed")
    if truth.get("sha256") != "a55f452e611426693677fbeacde227fc80e5f03f6040d04aef9d6a2baf08d249" or truth.get("rows") != TRUTH_ROWS or truth.get("bytes") != 305432:
        raise fail("truth is not the pinned H development truth")
    if truth.get("archive_member") != "dataset_2023/train/2021-08-24-20-32-us-ca-mtv-h/pixel5/ground_truth.csv" or truth.get("archive_crc32") != "fe212a4d":
        raise fail("truth archive membership changed")
    if truth.get("role") != "development/train; not heldout or leaderboard proof":
        raise fail("truth role changed")

    metric = manifest.get("metric_contract")
    expected_metric = {
        "candidate_parser": "Phase74 _parse_submission",
        "truth_parser": "Phase76 _parse_truth_dictreader (CSV DictReader; required fields by name; optional columns allowed)",
        "join": "exact (phone, UnixTimeMillis) keys; no interpolation/hold/fill",
        "distance": "Phase74 spherical Haversine, radius 6371008.8 m",
        "percentile": "Phase74 linear rank (n - 1) * q",
        "route_scalar": "(P50 + P95) / 2 in metres",
        "pixel5_offset_reapplication": False,
    }
    if metric != expected_metric:
        raise fail("metric contract changed")

    source_contract = manifest.get("source_contract")
    expected_source = {
        "phase201_source_inclusive_forward_imu_schedule": True,
        "phase197_source_utc_fallback_imu_offset": True,
        "phase194_source_utc_fallback_imu_noise": False,
        "configured_offset_ms": -20,
        "effective_offset_ms": -20,
        "raw_utc_keys_unchanged": True,
        "relative_pairing_unchanged": True,
        "main_generic_doppler_factors": 0,
        "stage_ecef_doppler_factors": 66685,
    }
    if source_contract != expected_source:
        raise fail("Phase202 source contract changed")

    authority = manifest.get("authority")
    if not isinstance(authority, Mapping):
        raise fail("manifest authority missing")
    authority_entry(authority, "evaluator", EVALUATOR)
    authority_entry(authority, "focused_tests", FOCUSED_TESTS)
    authority_entry(authority, "phase199_evaluator", PHASE199_PATH, PHASE199_SHA)
    authority_entry(authority, "phase196_thin_wrapper", PHASE196_PATH, PHASE196_SHA)
    authority_entry(authority, "phase189_kernel", PHASE189_PATH, PHASE189_SHA)
    authority_entry(authority, "phase76_parser", P76_PATH, P76_SHA)
    authority_entry(authority, "phase74_metric_helper", P74_PATH, P74_SHA)
    authority_entry(
        authority,
        "phase202_result",
        ROOT / "docs/use_cases/records/smartphone_r5_phase202_h_native_phase201_imu_schedule_result_v1.json",
        PHASE202_RESULT_SHA,
    )
    authority_entry(
        authority,
        "phase202_manifest",
        ROOT / "docs/use_cases/records/smartphone_r5_phase202_h_native_phase201_imu_schedule_manifest_v1.json",
        PHASE202_MANIFEST_SHA,
    )
    verify_phase202_metadata(authority)

    baseline = manifest.get("comparison_baseline")
    expected_baseline = {
        "phase199_h_route_score_m": PHASE199_BASELINE_M,
        "phase193_h_route_score_m": PHASE193_BASELINE_M,
        "phase182_h_route_score_m": PHASE182_BASELINE_M,
        "strict_target_m": STRICT_THRESHOLD_M,
        "role": "H development/train diagnostic; not heldout or leaderboard proof",
    }
    if baseline != expected_baseline:
        raise fail("comparison baseline changed")
    return manifest


def score_payloads(
    candidate_path: Path,
    candidate_pin: Mapping[str, Any],
    truth_path: Path,
    truth_pin: Mapping[str, Any],
    *,
    route: str,
    result_path: Path | None = None,
) -> dict[str, Any]:
    """Delegate to the unchanged Phase199 -> Phase189 scoring kernel."""
    kernel = load_phase199()
    try:
        return kernel.score_payloads(
            candidate_path,
            candidate_pin,
            truth_path,
            truth_pin,
            route=route,
            result_path=result_path,
        )
    except Exception as exc:
        if isinstance(exc, Phase203Error):
            raise
        raise fail(f"Phase199/189 scoring kernel rejected payloads: {exc}") from exc


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
    authorization = read_json(authorization_path, "Phase203 authorization")
    if authorization.get("schema_version") != "smartphone-r5-phase203-phase202-h-accuracy-authorization.v1" or authorization.get("allow_truth_read") is not True:
        raise fail("truth authorization is missing or mismatched")
    if authorization.get("manifest_sha256") != static_hash(manifest_path, "Phase203 manifest"):
        raise fail("authorization does not pin the current manifest")
    if authorization.get("candidate_sha256") != manifest["candidate"]["sha256"] or authorization.get("truth_sha256") != manifest["truth"]["sha256"]:
        raise fail("authorization candidate/truth pins do not match manifest")
    if result_path.exists():
        raise fail(f"refusing to overwrite existing result: {result_path}")
    candidate_pin = manifest["candidate"]
    truth_pin = manifest["truth"]
    score = score_payloads(
        ROOT / candidate_pin["path"],
        candidate_pin,
        ROOT / truth_pin["path"],
        truth_pin,
        route=manifest["route"],
        result_path=result_path,
    )
    result = {
        "schema_version": RESULT_SCHEMA,
        "phase": 203,
        "execution_label": "Luna Max",
        "status": "completed-h-development-accuracy-diagnostic",
        "manifest": str(manifest_path.relative_to(ROOT)),
        "comparison_baseline_phase199_h_route_score_m": PHASE199_BASELINE_M,
        "comparison_baseline_phase193_h_route_score_m": PHASE193_BASELINE_M,
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
    except Phase203Error as exc:
        print(f"phase203: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
