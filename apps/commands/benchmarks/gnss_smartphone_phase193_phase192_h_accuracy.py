#!/usr/bin/env python3
"""Pre-truth H evaluator for the frozen Phase192 native output.

``--verify`` validates only the frozen Phase193 metadata and source pins.
``--evaluate`` is intentionally authorization-gated: after the root reviews
the manifest, it reads the immutable Phase192 candidate once and the pinned H
development truth once, then delegates parsing, exact-key joining, Haversine
distance, and linear P50/P95 scoring to the sealed Phase189 kernel and its
Phase76/Phase74 helpers.  Only aggregate metrics are written.
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
KERNEL_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase189_phase188_h_accuracy.py"
P76_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase76_phase75_accuracy_integrity_recovery.py"
P74_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase74_phase73_miss_mask_accuracy.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase193_phase192_h_accuracy.py"

ROUTE = "2021-08-24-20-32-us-ca-mtv-h/pixel5"
LABEL = "MTV-H"
MODELED_EPOCHS = 3140
PUBLISHED_ROWS = 3139
TRUTH_ROWS = 3139
EARTH_RADIUS_M = 6371008.8
STRICT_THRESHOLD_M = 0.782
PHASE182_BASELINE_M = 1.2874762859947766
PHASE192_RESULT_SHA = "3f91629b22cf3bdc346b2835301bb4511917f3af665a51542559a72bee9b36db"
PHASE192_MANIFEST_SHA = "8fec8c1d866cf0e7e2327cac44c2a9cd4c24fd3ea20dc7b66a8b463ffb62c0cf"
KERNEL_SHA = "525a436707ab7de0202b1130bb6daed4e001686b8396a24b23ed15c669a60a83"
P76_SHA = "513e5970ee58846963f077b9960944e5fbf5f3535e5be24ca6e72493785f7761"
P74_SHA = "bf07ff7ccb9f9efce5a759f2fabe57b7dc0c24ac5acdaf57c6ad9fc88a3fd761"
P76_COMMIT = "7a8e77ce03f6f4354cf2f541098a53628faa23d5"
KERNEL_COMMIT = "5f456010c78cbb80cf5614f421d5c8b0d38aee3a"

MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase193_phase192_h_accuracy_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase193_phase192_h_accuracy_authorization_v1.json"
RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase193_phase192_h_accuracy_result_v1.json"
SCHEMA = "smartphone-r5-phase193-phase192-h-accuracy-manifest.v1"
RESULT_SCHEMA = "smartphone-r5-phase193-phase192-h-accuracy-result.v1"


class Phase193Error(ValueError):
    """Raised when the frozen Phase193 contract fails closed."""


def fail(message: str) -> Phase193Error:
    return Phase193Error(message)


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
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, Phase193Error) as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def static_hash(path: Path, label: str) -> str:
    """Hash source/metadata only; payload hashes are forbidden in verify mode."""
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


_KERNEL: Any | None = None


def load_kernel() -> Any:
    global _KERNEL
    if _KERNEL is not None:
        return _KERNEL
    if static_hash(KERNEL_PATH, "Phase189 kernel") != KERNEL_SHA:
        raise fail("Phase189 kernel hash changed")
    spec = importlib.util.spec_from_file_location("phase189_kernel_phase193", KERNEL_PATH)
    if spec is None or spec.loader is None:
        raise fail("unable to load Phase189 kernel")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _KERNEL = module
    return module


def load_helpers() -> tuple[Any, Any]:
    """Load the exact Phase76 parser and Phase74 metric helper via the kernel."""
    return load_kernel().load_helpers()


def safe_payload_path(value: Any, basename: str, fragment: str, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise fail(f"missing {label} path")
    path = Path(value)
    if path.is_absolute() or ".." in path.parts or path.name != basename or fragment not in value:
        raise fail(f"unsafe {label} path: {value!r}")
    return value


def _authority_entry(authority: Mapping[str, Any], name: str, path: Path, expected_sha: str) -> None:
    item = authority.get(name)
    if not isinstance(item, Mapping):
        raise fail(f"authority/{name} missing")
    if item.get("path") != str(path.relative_to(ROOT)):
        raise fail(f"authority/{name} path changed")
    if item.get("sha256") != expected_sha or static_hash(path, f"authority/{name}") != expected_sha:
        raise fail(f"authority/{name} hash changed")


def verify_manifest(path: Path = MANIFEST) -> dict[str, Any]:
    manifest = read_json(path, "Phase193 manifest")
    expected = {
        "schema_version": SCHEMA,
        "phase": 193,
        "execution_label": "Luna Max",
        "status": "frozen-before-phase193-truth-read",
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
        "/phase192-h-native-ecef-doppler-gnss-first-v1/",
        "candidate",
    )
    safe_payload_path(truth.get("path"), "ground_truth.csv", "/truth/", "truth")
    for pin, rows, label in ((candidate, PUBLISHED_ROWS, "candidate"), (truth, TRUTH_ROWS, "truth")):
        if not isinstance(pin.get("sha256"), str) or len(pin["sha256"]) != 64:
            raise fail(f"{label} SHA-256 pin missing")
        if pin.get("rows") != rows or not isinstance(pin.get("bytes"), int) or pin["bytes"] <= 0:
            raise fail(f"{label} row/byte pin invalid")
    if candidate.get("sha256") != "8080b373a441b99ffdd545fefc943b4b9fe07f316e6597164945c05ddbaddcc4":
        raise fail("candidate is not the sealed Phase192 output")
    if candidate.get("modeled_epochs") != MODELED_EPOCHS or candidate.get("warmup_epoch_excluded") is not True:
        raise fail("candidate modeled/warmup contract changed")
    if candidate.get("interpolated_epochs") != 0 or candidate.get("edge_hold_epochs") != 0 or candidate.get("unresolved_epochs") != 0:
        raise fail("candidate interpolation/hold contract changed")
    if candidate.get("pixel5_offset_reapplication") is not False:
        raise fail("candidate Pixel5 offset policy changed")
    if candidate.get("opaque_only") is not True:
        raise fail("candidate coordinate publication policy changed")
    if truth.get("sha256") != "a55f452e611426693677fbeacde227fc80e5f03f6040d04aef9d6a2baf08d249":
        raise fail("truth is not the pinned H development truth")
    if truth.get("archive_member") != "dataset_2023/train/2021-08-24-20-32-us-ca-mtv-h/pixel5/ground_truth.csv":
        raise fail("truth archive member changed")
    if truth.get("archive_crc32") != "fe212a4d":
        raise fail("truth archive CRC changed")
    if truth.get("role") != "development/train; not heldout or leaderboard proof":
        raise fail("truth role is not the pinned development role")

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

    authority = manifest.get("authority")
    if not isinstance(authority, Mapping):
        raise fail("manifest authority missing")
    _authority_entry(authority, "evaluator", EVALUATOR, static_hash(EVALUATOR, "Phase193 evaluator"))
    _authority_entry(authority, "focused_tests", FOCUSED_TESTS, static_hash(FOCUSED_TESTS, "Phase193 focused tests"))
    _authority_entry(authority, "phase189_kernel", KERNEL_PATH, KERNEL_SHA)
    _authority_entry(authority, "phase76_parser", P76_PATH, P76_SHA)
    _authority_entry(authority, "phase74_metric_helper", P74_PATH, P74_SHA)
    for name, relative, expected_sha in (
        (
            "phase192_result",
            ROOT / "docs/use_cases/records/smartphone_r5_phase192_h_native_ecef_doppler_gnss_first_result_v1.json",
            PHASE192_RESULT_SHA,
        ),
        (
            "phase192_manifest",
            ROOT / "docs/use_cases/records/smartphone_r5_phase192_h_native_ecef_doppler_gnss_first_manifest_v1.json",
            PHASE192_MANIFEST_SHA,
        ),
    ):
        _authority_entry(authority, name, relative, expected_sha)

    comparison = manifest.get("comparison_baseline")
    if comparison != {
        "phase182_h_route_score_m": PHASE182_BASELINE_M,
        "strict_target_m": STRICT_THRESHOLD_M,
        "role": "H development/train diagnostic; not heldout or leaderboard proof",
    }:
        raise fail("comparison baseline changed")
    return manifest


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
    """Delegate the exact Phase189 kernel; this wrapper changes no metric math."""
    kernel = load_kernel()
    try:
        return kernel.score_payloads(
            candidate_path,
            candidate_pin,
            truth_path,
            truth_pin,
            p76,
            p74,
            route=route,
            result_path=result_path,
        )
    except Exception as exc:
        if isinstance(exc, Phase193Error):
            raise
        raise fail(f"Phase189 kernel rejected payloads: {exc}") from exc


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
    authorization = read_json(authorization_path, "Phase193 authorization")
    if (
        authorization.get("schema_version") != "smartphone-r5-phase193-phase192-h-accuracy-authorization.v1"
        or authorization.get("allow_truth_read") is not True
    ):
        raise fail("truth authorization is missing or mismatched")
    if authorization.get("manifest_sha256") != static_hash(manifest_path, "Phase193 manifest"):
        raise fail("authorization does not pin the current manifest")
    if authorization.get("candidate_sha256") != manifest["candidate"]["sha256"] or authorization.get("truth_sha256") != manifest["truth"]["sha256"]:
        raise fail("authorization candidate/truth pins do not match manifest")
    if result_path.exists():
        raise fail(f"refusing to overwrite existing result: {result_path}")
    p76, p74 = load_helpers()
    candidate_path = ROOT / manifest["candidate"]["path"]
    truth_path = ROOT / manifest["truth"]["path"]
    score = score_payloads(candidate_path, manifest["candidate"], truth_path, manifest["truth"], p76, p74, route=manifest["route"], result_path=result_path)
    result = {
        "schema_version": RESULT_SCHEMA,
        "phase": 193,
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
    except Phase193Error as exc:
        print(f"phase193: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
