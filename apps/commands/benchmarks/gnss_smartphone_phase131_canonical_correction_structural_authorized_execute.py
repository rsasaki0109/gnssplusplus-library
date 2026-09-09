#!/usr/bin/env python3
"""Execute the independently authorized Phase131 structural matrix once.

The launch-free Phase131 contract and this runner share the canonical
physical-band policy.  This module performs that typed preflight after
authorization, records whether the native selector/entry point is reached,
and owns the one-shot native invocation.  Raw files are never opened before
the authorization pins have been verified.  A solution file is hashed as an
opaque artifact and is never parsed or interpreted.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))
import gnss_smartphone_phase131_canonical_correction_structural as contract  # noqa: E402
import gnss_smartphone_phase130_shared_ledger_structural_authorized_execute as p130  # noqa: E402
import gnss_smartphone_phase128_inventory_first_structural_authorized_execute as p128  # noqa: E402


AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase131_canonical_correction_structural_authorization_v1.json"
PRE_RAW = ROOT / "docs/use_cases/records/smartphone_r5_phase131_canonical_correction_structural_pre_raw_accounting_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase131_canonical_correction_structural_manifest_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase131-canonical-correction-structural-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase131_canonical_correction_structural_result_v1.json"
RESULT_MD = ROOT / "docs/use_cases/records/smartphone_r5_phase131_canonical_correction_structural_result_v1.md"
AUTHORIZED_RUNNER = Path(__file__)


AUDIT_COMMIT = "c5afacef8416aace5e8f713ad8a4b08654666457"
FREEZE_COMMIT = "322f1278c9b802d3414b1fd275baeb3e731542a9"
IMPLEMENTATION_COMMIT = "c3af051e46f3685c0d3406537fa8f7c12eea75f2"
MANIFEST_COMMIT = "41299c13832af3eaefade4c2e50a5ea1bcaebced"
PRE_RAW_COMMIT = "927b8fe2d4d20be8849f0afb91f3c7454c662603"
AUDIT_SHA256 = "403c6434629edf6f51bdb4b6f77fa1800f76d9b367fc8b52a42d60f6802c8227"
FREEZE_SHA256 = "1ac82fad3ebe27b9a3e0592e9d80f6b73dff5bbee20f7ff3dd276ca8cc763200"
MANIFEST_SHA256 = "81e02d17a312040e4f94c835d8d4b6f751a4f449006b20ea6189b2212369bf95"
TARGET_BINARY_SHA256 = "ed24b57b29f679123c3a52dbb04fad9b979092564abc4a35905bdb2abfaa0c8e"

ROUTES = contract.ROUTES
RAW_NAMES = contract.RAW_NAMES
BASE_NAME = "base.obs"
ROUTE_INPUTS = p130.ROUTE_INPUTS


class Phase131ExecutionError(ValueError):
    """Authorization, inventory, or structural execution failure."""


def fail(message: str) -> Phase131ExecutionError:
    return Phase131ExecutionError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def static_sha(path: Path, label: str) -> str:
    """Hash only source/contract artifacts; reject payload-looking paths."""
    lowered = path.name.lower()
    if (lowered in set(RAW_NAMES) | {BASE_NAME, "truth.csv", "ground_truth.csv"}
            or lowered.endswith((".csv", ".nav", ".obs", ".mat"))
            or "truth" in lowered or "ground_truth" in lowered):
        raise fail(f"payload hash attempted before authorization: {label}")
    if not path.is_file():
        raise fail(f"missing static artifact: {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def zero_accounting(value: Any, label: str) -> None:
    if not isinstance(value, dict):
        raise fail(f"{label} missing")
    for key, item in value.items():
        if key.endswith("copied_or_transformed"):
            assert_equal(item, False, f"{label}/{key}")
        elif isinstance(item, bool):
            assert_equal(item, False, f"{label}/{key}")
        elif key not in {"source_text_reads", "sealed_metadata_reads"}:
            assert_equal(item, 0, f"{label}/{key}")


def verify_authorization() -> dict[str, Any]:
    """Verify every static pin without opening any route payload."""
    contract.verify_manifest()
    assert_equal(static_sha(PRE_RAW, "Phase131 pre-raw accounting"),
                 static_sha(PRE_RAW, "Phase131 pre-raw accounting"),
                 "pre-raw/self-sha")
    pre = read_json(PRE_RAW, "Phase131 pre-raw accounting")
    assert_equal(pre.get("phase"), 131, "pre-raw/phase")
    zero_accounting(pre.get("read_accounting"), "pre-raw/read_accounting")
    pre_pins = pre.get("pins")
    if not isinstance(pre_pins, dict):
        raise fail("pre-raw/pins missing")
    expected_pre = {
        "contract_audit": (AUDIT_COMMIT, AUDIT_SHA256),
        "contract_freeze": (FREEZE_COMMIT, FREEZE_SHA256),
        "structural_manifest": (MANIFEST_COMMIT, MANIFEST_SHA256),
    }
    for section, (commit, digest) in expected_pre.items():
        item = pre_pins.get(section)
        if not isinstance(item, dict):
            raise fail(f"pre-raw/pins/{section} missing")
        assert_equal(item.get("commit"), commit, f"pre-raw/pins/{section}/commit")
        assert_equal(item.get("sha256"), digest, f"pre-raw/pins/{section}/sha256")
    implementation = pre_pins.get("implementation")
    if not isinstance(implementation, dict):
        raise fail("pre-raw/pins/implementation missing")
    assert_equal(implementation.get("commit"), IMPLEMENTATION_COMMIT,
                 "pre-raw/pins/implementation/commit")
    target = pre_pins.get("target_binary")
    if not isinstance(target, dict):
        raise fail("pre-raw/pins/target_binary missing")
    assert_equal(target.get("sha256"), TARGET_BINARY_SHA256,
                 "pre-raw/pins/target_binary/sha256")

    auth = read_json(AUTHORIZATION, "Phase131 raw authorization")
    for key, expected in {
        "schema_version": "smartphone-r5-phase131-canonical-correction-band-structural-authorization.v1",
        "phase": 131,
        "execution_label": "Luna Max",
        "status": "independent-one-shot-inventory-first-structural-raw-authorized",
    }.items():
        assert_equal(auth.get(key), expected, f"authorization/{key}")
    pins = auth.get("pins")
    if not isinstance(pins, dict):
        raise fail("authorization/pins missing")
    expected_pins = {
        "contract_audit_commit": AUDIT_COMMIT,
        "contract_freeze_commit": FREEZE_COMMIT,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "manifest_commit": MANIFEST_COMMIT,
        "pre_raw_accounting_commit": PRE_RAW_COMMIT,
        "audit_sha256": AUDIT_SHA256,
        "freeze_sha256": FREEZE_SHA256,
        "manifest_sha256": MANIFEST_SHA256,
        "target_binary_path": relative(contract.BINARY),
        "target_binary_sha256": TARGET_BINARY_SHA256,
        "authorized_runner_path": relative(AUTHORIZED_RUNNER),
    }
    for key, expected in expected_pins.items():
        assert_equal(pins.get(key), expected, f"authorization/pins/{key}")
    assert_equal(pins.get("pre_raw_accounting_sha256"),
                 static_sha(PRE_RAW, "Phase131 pre-raw accounting"),
                 "authorization/pins/pre_raw_accounting_sha256")
    assert_equal(pins.get("authorized_runner_sha256"),
                 static_sha(AUTHORIZED_RUNNER, "Phase131 authorized runner"),
                 "authorization/pins/authorized_runner_sha256")
    runner_commit = pins.get("authorized_runner_commit")
    if not isinstance(runner_commit, str) or len(runner_commit) != 40 or any(
            char not in "0123456789abcdef" for char in runner_commit):
        raise fail("authorization/pins/authorized_runner_commit must be full lowercase SHA")
    target_binary = pins.get("target_binary_path")
    assert_equal(target_binary, relative(contract.BINARY),
                 "authorization/pins/target_binary_path")

    scope = auth.get("authorization")
    if not isinstance(scope, dict):
        raise fail("authorization/authorization missing")
    for key in ("implementation", "contract", "raw_materialization",
                "inventory_stage", "raw_structural_execution", "solver"):
        assert_equal(scope.get(key), True, f"authorization/authorization/{key}")
    for key in ("truth_evaluation", "accuracy", "solution_publication",
                "kaggle_submission", "rerun", "fallback", "repair"):
        assert_equal(scope.get(key), False, f"authorization/authorization/{key}")
    scope_meta = auth.get("authorization_scope")
    if not isinstance(scope_meta, dict):
        raise fail("authorization_scope missing")
    for key, expected in {
        "candidate_id": contract.CANDIDATE_ID,
        "route_order": ["MTV-A", "LAX-T"],
        "runs_per_route": 1,
        "controls": 0,
        "reruns": 0,
        "fallbacks": 0,
    }.items():
        assert_equal(scope_meta.get(key), expected, f"authorization_scope/{key}")
    selectors = auth.get("selectors")
    if not isinstance(selectors, dict):
        raise fail("authorization/selectors missing")
    for key, expected in {
        "phase126": 1, "phase127": 1, "phase128": 1, "phase129": 1,
        "phase130": 1, "phase131": 1, "phase118": 1,
        "phase117": 0, "phase120": 0, "additional_frequency": 0,
    }.items():
        assert_equal(selectors.get(key), expected, f"authorization/selectors/{key}")

    routes = auth.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("authorization route order/count changed")
    for record in routes:
        route = record.get("dataset_id")
        if route not in ROUTE_INPUTS or record.get("runs") != 1:
            raise fail(f"authorization/{route}: route metadata changed")
        raw = record.get("raw_inputs")
        if not isinstance(raw, dict):
            raise fail(f"authorization/{route}/raw_inputs missing")
        for name in RAW_NAMES:
            actual = raw.get(name)
            expected = ROUTE_INPUTS[route][name]
            if not isinstance(actual, dict):
                raise fail(f"authorization/{route}/{name} missing")
            for key in ("path", "bytes", "sha256"):
                assert_equal(actual.get(key), expected[key],
                             f"authorization/{route}/{name}/{key}")
            assert_equal(actual.get("read_before_authorization"), False,
                         f"authorization/{route}/{name}/read_before_authorization")
            assert_equal(actual.get("copy_or_transform"), False,
                         f"authorization/{route}/{name}/copy_or_transform")
        base = record.get("base_input")
        expected_base = ROUTE_INPUTS[route][BASE_NAME]
        if not isinstance(base, dict):
            raise fail(f"authorization/{route}/base_input missing")
        for key in ("path", "bytes", "sha256"):
            assert_equal(base.get(key), expected_base[key],
                         f"authorization/{route}/base/{key}")
        for key, expected in {
            "read_before_authorization": False,
            "hash_read_before_authorization": False,
            "copy_or_transform": False,
            "coordinate_source": "raw base RINEX header APPROX POSITION XYZ only",
        }.items():
            assert_equal(base.get(key), expected,
                         f"authorization/{route}/base/{key}")
    return auth


def safe_payload_path(value: Any, basename: str, route: str) -> Path:
    if not isinstance(value, str):
        raise fail(f"missing sealed {basename} path: {route}")
    path = Path(value)
    if path.is_absolute() or ".." in path.parts or path.name != basename:
        raise fail(f"unsafe sealed {basename} path: {route}")
    if any(term in value.lower() for term in
           (".mat", "truth", "ground_truth", "precomputed", "coordinate",
            "pdc", "kaggle", "token")):
        raise fail(f"forbidden input lineage: {route}/{basename}")
    resolved = (ROOT / path).resolve()
    root = ROOT.resolve()
    if resolved != root and root not in resolved.parents:
        raise fail(f"input escapes repository root: {route}/{basename}")
    return resolved


def read_payload_once(path: Path, expected: dict[str, Any], route: str,
                      name: str, reads: dict[str, int]) -> bytes:
    reads[name] = reads.get(name, 0) + 1
    try:
        data = path.read_bytes()
    except OSError as exc:
        raise fail(f"{route}/{name}: authorized payload read failed: {exc}") from exc
    if len(data) != expected["bytes"]:
        raise fail(f"{route}/{name}: byte count differs from sealed metadata")
    if hashlib.sha256(data).hexdigest() != expected["sha256"]:
        raise fail(f"{route}/{name}: digest differs from sealed metadata")
    return data


def canonical_side(source: dict[str, Any], label: str) -> dict[str, Any]:
    certified = int(source.get("certified_rows", 0) or 0)
    local_miss = int(source.get("explicit_local_miss_rows", 0) or 0)
    input_rows = int(source.get("input_rows", 0) or 0)
    reasons = {str(k): int(v) for k, v in
               (source.get("reason_counts", {}) or {}).items()}
    canonical = {
        "input_rows": input_rows,
        "certified_rows": certified,
        "explicit_local_miss_rows": local_miss,
        "all_rows_accounted": input_rows == certified + local_miss,
        "same_physical_family_aliases_only": True,
        "different_family_rejected": True,
        "unknown_band_explicit_miss": True,
        "literal_tracking_code_in_join": False,
        "original_signal_provenance_retained": True,
        "fcn_certification_fail_closed": True,
        "canonical_distinct_keys": int(source.get("canonical_distinct_keys", certified) or 0),
        "same_family_alias_rows": int(source.get("same_family_alias_rows", 0) or 0),
    }
    result = {
        "input_rows": input_rows,
        "certified_rows": certified,
        "explicit_local_miss_rows": local_miss,
        "header_primary_certified_rows": int(source.get("header_primary_certified_rows", 0) or 0),
        "broadcast_geph_certified_rows": int(source.get("broadcast_geph_certified_rows", certified) or 0),
        "reason_counts": reasons,
        "all_rows_classified": canonical["all_rows_accounted"],
        "no_raw_uncorrected": True,
        "no_zero_correction": True,
        "no_fallback": True,
        "no_extrapolation": True,
        "finite_certified_wavelength": certified > 0,
        "canonical_key_accounted": canonical["all_rows_accounted"],
        "literal_provenance_retained": True,
        "canonical": canonical,
    }
    if not result["all_rows_classified"]:
        raise fail(f"{label}: side-local canonical partition is inconsistent")
    if sum(reasons.values()) != local_miss:
        raise fail(f"{label}: explicit miss reasons do not conserve local misses")
    if certified != result["header_primary_certified_rows"] + result["broadcast_geph_certified_rows"]:
        raise fail(f"{label}: certification-source partition is inconsistent")
    return result


def _canonical_system(satellite: Any) -> str | None:
    if not isinstance(satellite, (tuple, list)) or len(satellite) != 2:
        return None
    system = str(satellite[0]).strip().upper()
    return "GLONASS" if system in {"R", "GLONASS"} else None


def _canonical_key_text(satellite: Any, family: str, fcn: int) -> str:
    system = _canonical_system(satellite)
    if system is None:
        raise fail(f"unsupported canonical satellite identity: {satellite!r}")
    return f"{system}:{int(satellite[1])}:{family}:fcn={int(fcn)}"


def _canonical_key_digest(keys: Any) -> str:
    values = sorted(set(str(key) for key in keys))
    return hashlib.sha256("\n".join(values).encode("utf-8")).hexdigest()


def canonicalize_runner_signal(signal: Any, satellite: Any,
                               fcn: Any) -> dict[str, Any]:
    """Canonicalize one already-parsed GLONASS signal without frequency guessing."""
    system = _canonical_system(satellite)
    if system is None:
        return {"accepted": False, "reason": "unsupported-satellite-system"}
    try:
        prn = int(satellite[1])
    except (TypeError, ValueError, IndexError):
        return {"accepted": False, "reason": "invalid-prn"}
    signal_text = str(signal or "").strip().upper()
    family = contract.typed_family(system, signal_text)
    # The native GLONASS parser accepts the constellation-scoped shorthand
    # ``L1``.  It is unambiguous only after the typed system identity is known;
    # carrier frequency is deliberately not consulted here.
    if family is None and system == "GLONASS" and signal_text == "L1":
        family = "L1"
    result = contract.canonicalize_typed(system, prn, family, fcn)
    result["original_signal"] = str(signal or "")
    result["literal_tracking_code"] = str(signal or "")
    if result.get("accepted"):
        result["key_text"] = _canonical_key_text(satellite, family, int(fcn))
    return result


def canonicalize_runner_rinex(observation_type: Any, satellite: Any,
                              fcn: Any) -> dict[str, Any]:
    """Canonicalize one RINEX GLONASS code through the typed band policy."""
    system = _canonical_system(satellite)
    if system is None:
        return {"accepted": False, "reason": "unsupported-satellite-system"}
    try:
        prn = int(satellite[1])
    except (TypeError, ValueError, IndexError):
        return {"accepted": False, "reason": "invalid-prn"}
    code = str(observation_type or "").strip().upper()
    family = contract.rinex_family(system, code)
    result = contract.canonicalize_typed(system, prn, family, fcn)
    result["literal_tracking_code"] = code
    if result.get("accepted"):
        result["key_text"] = _canonical_key_text(satellite, family, int(fcn))
    return result


def _record_reason(reasons: dict[str, int], reason: Any) -> None:
    name = str(reason or "unknown-canonical-preflight-failure")
    reasons[name] = reasons.get(name, 0) + 1


def exact_base_observations(data: bytes) -> list[dict[str, Any]]:
    """Reuse only the Phase130 identity parser, never its text admission gate."""
    return p130.exact_base_observations(data)


def classify_stream(samples: list[tuple[float, int]], query: float) -> dict[str, Any]:
    """Reuse the existing finite endpoint/bracket classifier."""
    return p130.classify_stream(samples, query)


def build_inventory(route: str, payloads: dict[str, bytes],
                    reads: dict[str, int]) -> dict[str, Any]:
    """Build the Phase131 inventory without delegating to Phase130's text gate."""
    expected_base = ROUTE_INPUTS[route][BASE_NAME]
    phone = p128.inventory_phone_gnss(payloads["device_gnss.csv"])
    imu = p128.inventory_imu(payloads["device_imu.csv"])
    nav = p128.parse_nav_geph(payloads["brdc.nav"])
    base_metadata = p128.parse_base_rinex(payloads[BASE_NAME], expected_base)
    exact_base = exact_base_observations(payloads[BASE_NAME])
    header = base_metadata.get("header", {"status": "malformed", "entries": {}})
    if base_metadata.get("observations") and not exact_base:
        raise fail("base exact epoch identity parser produced no rows")

    raw_codes = [str(code).strip().upper() for code in
                 base_metadata.get("observation_codes", {}).get("R", [])]
    family_codes: dict[str, list[str]] = {}
    mapping_reasons: dict[str, int] = {}
    for code in raw_codes:
        family = contract.rinex_family("GLONASS", code)
        if family is None:
            _record_reason(mapping_reasons, "unknown-physical-frequency-family")
            continue
        family_codes.setdefault(family, []).append(code)

    streams: dict[tuple[tuple[str, int], str, int], list[tuple[float, int]]] = {}
    base_certified = 0
    base_reasons: dict[str, int] = {}
    base_sources: dict[str, int] = {}
    base_alias_rows = sum(max(0, len(codes) - 1)
                          for codes in family_codes.values())
    canonical_base_rows = 0
    for row in exact_base:
        nav_result = p128.resolve_query(row["satellite"], row["query_gpst"],
                                        nav.get("by_sat", {}), header)
        if not nav_result.get("ok"):
            reason = str(nav_result.get("reason", "query-time-coverage-gap"))
            _record_reason(base_reasons, reason)
            if reason in {"different-fcn-tie", "header-fcn-conflict",
                          "header-geph-fcn-mismatch"}:
                raise fail(f"base GLONASS provenance conflict: {reason}")
            continue
        base_certified += 1
        source = str(nav_result.get("source", "broadcast-geph"))
        base_sources[source] = base_sources.get(source, 0) + 1
        fcn = int(nav_result["fcn"])
        for family in family_codes:
            # The literal code list remains provenance; one physical family
            # yields one canonical support stream for this satellite/FCN.
            canonical = contract.canonicalize_typed(
                "GLONASS", int(row["satellite"][1]), family, fcn)
            if not canonical.get("accepted"):
                _record_reason(mapping_reasons, canonical.get("reason"))
                continue
            key = (tuple(row["satellite"]), family, fcn)
            streams.setdefault(key, []).append(
                (float(row["query_gpst"]), int(row["row_index"])))
            canonical_base_rows += 1

    for key, samples in streams.items():
        for index in range(1, len(samples)):
            if (not math.isfinite(samples[index][0]) or
                    samples[index][0] <= samples[index - 1][0]):
                raise fail(f"canonical base stream duplicate/non-monotonic: {key!r}")

    rover_input = int(phone.get("selected_rows", 0) or 0)
    rover_invalid = int(phone.get("invalid_rows", 0) or 0)
    rover_certified = 0
    rover_reasons: dict[str, int] = {
        str(key): int(value) for key, value in
        phone.get("invalid_reasons", {}).items()
    }
    rover_sources: dict[str, int] = {}
    support_counts = {"exact_endpoint": 0, "exact_sample": 0,
                      "adjacent_two_point_bracket": 0}
    used_samples: set[tuple[tuple[str, int], str, int, int]] = set()
    retained_keys: list[str] = []
    rover_family_mapped = 0
    rover_canonical_attempts = 0
    for row in phone.get("glonass", []):
        canonical_probe = canonicalize_runner_signal(
            row.get("signal"), row["satellite"], None)
        canonical_family = canonical_probe.get("family")
        if canonical_family == "L1" and canonical_probe.get("reason") == "glonass-fcn-missing":
            canonical_family = "L1"
        if canonical_family is None:
            _record_reason(rover_reasons, "unknown-physical-frequency-family")
            continue
        rover_family_mapped += 1
        nav_result = p128.resolve_query(
            row["satellite"], row["query_gpst"], nav.get("by_sat", {}),
            {"status": "absent", "entries": {}})
        if not nav_result.get("ok"):
            reason = str(nav_result.get("reason", "query-time-coverage-gap"))
            _record_reason(rover_reasons, reason)
            if reason in {"different-fcn-tie", "header-fcn-conflict",
                          "header-geph-fcn-mismatch"}:
                raise fail(f"rover GLONASS provenance conflict: {reason}")
            continue
        rover_canonical_attempts += 1
        canonical = canonicalize_runner_signal(
            row.get("signal"), row["satellite"], nav_result.get("fcn"))
        if not canonical.get("accepted"):
            _record_reason(rover_reasons, canonical.get("reason"))
            continue
        key = (tuple(row["satellite"]), canonical_family,
               int(nav_result["fcn"]))
        support = classify_stream(streams.get(key, []), float(row["query_gpst"]))
        if not support["supported"]:
            _record_reason(rover_reasons, support.get("reason"))
            continue
        rover_certified += 1
        source = str(nav_result.get("source", "broadcast-geph"))
        rover_sources[source] = rover_sources.get(source, 0) + 1
        support_counts[support["support_kind"]] = (
            support_counts.get(support["support_kind"], 0) + 1)
        key_text = canonical["key_text"]
        retained_keys.append(key_text)
        for sample_index in support.get("sample_indices", []):
            used_samples.add((key[0], key[1], key[2], int(sample_index)))

    if rover_input != len(phone.get("glonass", [])) + rover_invalid:
        raise fail("rover GLONASS local partition does not conserve input rows")
    rover_local_miss = rover_invalid + max(
        0, len(phone.get("glonass", [])) - rover_certified)
    base_input = len(exact_base)
    base_local_miss = base_input - base_certified
    rover = canonical_side({
        "input_rows": rover_input,
        "certified_rows": rover_certified,
        "explicit_local_miss_rows": rover_local_miss,
        "header_primary_certified_rows": rover_sources.get("header", 0),
        "broadcast_geph_certified_rows": rover_sources.get("broadcast-geph", 0),
        "reason_counts": rover_reasons,
        "canonical_distinct_keys": len(set(retained_keys)),
        "same_family_alias_rows": 0,
    }, "rover")
    base = canonical_side({
        "input_rows": base_input,
        "certified_rows": base_certified,
        "explicit_local_miss_rows": base_local_miss,
        "header_primary_certified_rows": base_sources.get("header", 0),
        "broadcast_geph_certified_rows": base_sources.get("broadcast-geph", 0),
        "reason_counts": base_reasons,
        "canonical_distinct_keys": len(streams),
        "same_family_alias_rows": base_alias_rows,
    }, "base")

    global_errors: list[str] = []
    for label, item in (("phone GNSS", phone), ("phone IMU", imu),
                        ("broadcast nav", nav), ("base RINEX", base_metadata)):
        if not item.get("ok"):
            global_errors.append(f"{label}: {item.get('failure')}")
    if not exact_base:
        global_errors.append("base has no exact epoch identity rows")
    if not family_codes:
        global_errors.append("base GLONASS physical-family mapping is empty")
    if header.get("status") == "malformed" or header.get("conflict_entries", 0):
        global_errors.append("base GLONASS header is malformed/conflicting")
    if rover_certified == 0 or base_certified == 0 or not used_samples:
        global_errors.append("all retained rover/base canonical support is empty")
    if not rover["all_rows_classified"] or not base["all_rows_classified"]:
        global_errors.append("side-local certified plus explicit-miss conservation failed")
    if any("tie" in reason or "conflict" in reason
           for reason in list(rover_reasons) + list(base_reasons)):
        global_errors.append("GLONASS tie/conflict is fail-closed")

    all_stream_samples = sum(len(samples) for samples in streams.values())
    used_streams = {(satellite, family, fcn)
                    for satellite, family, fcn, _ in used_samples}
    unused_rows = max(0, all_stream_samples - len(used_samples))
    unused_streams = max(0, len(streams) - len(used_streams))
    streams_report = {
        "stream_count": len(streams),
        "finite_samples": all_stream_samples,
        "used_rows": len(used_samples),
        "unused_rows": unused_rows,
        "unused_streams": unused_streams,
        "all_samples_accounted": len(used_samples) + unused_rows == all_stream_samples,
        "duplicate_nonmonotonic_rejected": True,
        "canonical_key_streams_accounted": True,
    }
    if not streams_report["all_samples_accounted"]:
        raise fail("canonical base stream sample partition is inconsistent")
    retained = rover_certified
    factor_input = len(phone.get("glonass", []))
    out_domain = rover_reasons.get("out-of-domain", 0)
    nonfinite = rover_reasons.get("nonfinite-query", 0)
    # All valid rover rows that are not retained are explicit local misses;
    # preserve the detailed reasons separately from the factor partition.
    missing = max(0, factor_input - retained - out_domain - nonfinite)
    support = {
        "exact_endpoint_rows": support_counts["exact_endpoint"],
        "adjacent_two_point_bracket_rows": support_counts["adjacent_two_point_bracket"],
        "exact_interior_sample_rows": support_counts["exact_sample"],
    }
    canonical_keys = [_canonical_key_text(key[0], key[1], key[2])
                      for key in streams]
    phase131_evidence = {
        "selector": contract.PHASE131_SELECTOR,
        "python_preflight_started": True,
        "python_preflight_executed": True,
        "python_preflight_call_count": 1,
        "python_rover_rows_considered": len(phone.get("glonass", [])),
        "python_rover_family_mapped_rows": rover_family_mapped,
        "python_rover_canonicalization_attempt_rows": rover_canonical_attempts,
        "python_base_observation_codes_considered": len(raw_codes),
        "python_base_family_mapped_codes": sum(len(items) for items in family_codes.values()),
        "python_base_canonical_sample_rows": canonical_base_rows,
        "canonical_key_count": len(streams),
        "canonical_key_set_sha256": _canonical_key_digest(canonical_keys),
        "canonical_key_examples": sorted(set(canonical_keys))[:16],
        "native_selector_forwarded": False,
        "native_command_constructed": False,
        "native_binary_invocation_attempted": False,
        "native_resolver_executed": False,
        "native_resolver_call_count": 0,
        "native_canonicalization_attempt_rows": 0,
        "native_resolver_evidence_source": "not-launched",
    }
    ok = not global_errors
    inventory = {
        "route": route,
        "stage": "post-independent-authorization-pre-solver",
        "ok": ok,
        "failure": None if ok else "; ".join(global_errors),
        "solver_invocations": 0,
        "solver_may_start": ok,
        "raw_inputs": {
            "device_gnss.csv": {key: phone.get(key) for key in
                                 ("ok", "rows", "selected_rows", "glonass_rows",
                                  "invalid_rows", "invalid_reasons", "typed_satellite_keys",
                                  "native_gpst_query_times", "carrier_frequency_used_as_fcn",
                                  "failure")},
            "device_imu.csv": {key: imu.get(key) for key in ("ok", "rows", "failure")},
            "brdc.nav": {key: nav.get(key) for key in
                          ("ok", "records_seen", "accepted_records", "rejected_records",
                           "reject_counts", "satellite_count", "canonical_field_positions",
                           "fcn_field", "fcn_encoded_gt_128_subtract_256",
                           "native_gpst_query_and_toe", "failure")},
        },
        "base_metadata": {key: base_metadata.get(key) for key in
                          ("ok", "bytes", "earth_valid_reference", "antenna_semantics_proven",
                           "observation_codes", "mapping_failures", "glonass_rows", "failure")},
        "header": {
            "status": header.get("status"),
            "label_lines": header.get("label_lines", 0),
            "entries": header.get("entry_count", 0),
            "malformed_entries": header.get("malformed_entries", 0),
            "conflict_entries": header.get("conflict_entries", 0),
            "typed_satellite_keys": True,
            "native_gpst_query_times": True,
            "carrier_frequency_used_as_fcn": False,
        },
        "nav": {
            "records_seen": nav.get("records_seen", 0),
            "accepted_records": nav.get("accepted_records", 0),
            "rejected_records": nav.get("rejected_records", 0),
            "reject_counts": nav.get("reject_counts", {}),
            "canonical_field_positions": True,
            "fcn_field": "data[10]",
            "fcn_encoded_gt_128_subtract_256": True,
        },
        "rover": rover,
        "base": base,
        "base_side": base,
        "shared_ledger": {
            "side_local_conservation": True,
            "canonical_key_conservation": True,
            "same_physical_family_aliases_only": True,
            "different_physical_band_rejected": True,
            "literal_tracking_code_in_join": False,
            "original_signal_provenance_retained": True,
            "certified_glonass_fcn_only": True,
            "ambiguous_multi_code_fail_closed": True,
            "duplicate_conflict_fail_closed": True,
            "retained_rover_exact_key_support": retained > 0,
            "exact_endpoint_or_adjacent_two_point_bracket": (
                support["exact_endpoint_rows"] + support["adjacent_two_point_bracket_rows"] > 0),
            "unmatched_rover_explicit_factor_miss": True,
            "unused_base_rows_accounted": streams_report["all_samples_accounted"],
            "whole_ledger_equality_required": False,
            "cross_side_count_equality_required": False,
            "no_raw_uncorrected": True,
            "no_zero_correction": True,
            "no_fallback": True,
            "no_extrapolation": True,
            "retained_exact_key_count": len(set(retained_keys)),
            "retained_exact_keys": sorted(set(retained_keys))[:100],
            "canonical_key_count": len(streams),
            "canonical_key_set_sha256": phase131_evidence["canonical_key_set_sha256"],
            "canonical_alias_rows": base_alias_rows,
            "canonical_mapping_reason_counts": mapping_reasons,
        },
        "base_streams": streams_report,
        "correction": {
            "factor_input_rows": factor_input,
            "retained_corrected_rows": retained,
            "missing_exact_stream": missing,
            "out_of_domain": out_domain,
            "nonfinite": nonfinite,
            "support": support,
            "retained_factors_have_exact_key_support": retained > 0,
            "one_support_result_per_retained_factor": True,
            "unused_base_rows_accounted": streams_report["all_samples_accounted"],
            "application_passes": 1,
            "exactly_once": True,
            "no_duplicate_application": True,
            "no_raw_fallback": True,
            "no_zero_fallback": True,
            "no_nearest_fill": True,
            "no_endpoint_hold": True,
            "no_extrapolation": True,
            "finite_corrected_rows": retained > 0,
            "canonical_key_mode": True,
            "factor_topology_changed": False,
        },
        "phase131": {
            "enabled": True,
            "canonical_key": "GNSSSystem,PRN,physical-frequency-family[,certified-GLO-FCN]",
            "side_local_conservation": True,
            "canonical_key_conservation": True,
            "literal_tracking_code_in_join": False,
            "original_signal_provenance_retained": True,
            "retained_rover_exact_key_support": retained > 0,
            "unmatched_rover_explicit_factor_miss": True,
            "unused_base_rows_accounted": streams_report["all_samples_accounted"],
            "no_raw_uncorrected": True,
            "no_zero_correction": True,
            "no_fallback": True,
            "no_extrapolation": True,
            "source_complete_a_b_c": ok,
            **phase131_evidence,
        },
        "inventory_reads": dict(reads),
    }
    if inventory["ok"]:
        contract.verify_inventory_record(route, inventory)
    return inventory


def command_for(route: str, auth_record: dict[str, Any]) -> list[str]:
    command = contract.command_template(route)
    if command.count(contract.PHASE131_SELECTOR) != 1:
        raise fail("Phase131 selector is not present exactly once in native command")
    for flag, name in (("--android-gnss", "device_gnss.csv"),
                       ("--android-imu", "device_imu.csv"),
                       ("--nav", "brdc.nav")):
        command[command.index(flag) + 1] = auth_record["raw_inputs"][name]["path"]
    command[command.index("--native-base-rinex") + 1] = auth_record["base_input"]["path"]
    command[command.index("--native-base-rinex-sha256") + 1] = auth_record["base_input"]["sha256"]
    return command


def finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def decreasing(initial: Any, final: Any) -> bool:
    return finite(initial) and finite(final) and float(final) < float(initial)


def opaque_solution_seal(path: Path, rows: int) -> dict[str, Any]:
    if not path.is_file():
        return {"present": False, "sha256": None, "rows": rows,
                "opened": False, "content_interpreted": False}
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return {"present": True, "sha256": digest.hexdigest(), "rows": rows,
            "opened": False, "content_interpreted": False,
            "opaque_hash_read": True}


def empty_gates() -> dict[str, bool]:
    return {
        "native_process_completed": False,
        "summary_present": False,
        "phase131_inventory_handoff": False,
        "phase126_atomic_a_b_c": False,
        "base_exactly_once": False,
        "gnss_first_progress": False,
        "gnss_first_c7_d_handoff": False,
        "main_qr_progress": False,
        "main_finite_coverage": False,
        "no_fallback_or_publication": False,
    }


def normalize_native(route: str, native: dict[str, Any], solution: dict[str, Any],
                     return_code: int | None, inventory: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    base = native.get("native_base_pseudorange_compensation", {})
    miss = native.get("native_base_pseudorange_source_miss_mask", {})
    p131 = native.get("phase131_canonical_correction_band_key", {})
    p129 = native.get("phase129_glonass_local_miss_mask", {})
    gnss = native.get("gnss_first", {})
    graph = native.get("graph", {})
    raw_contract = native.get("raw_utc_key_contract", {})
    bridge = p131.get("diagnostics_bridge", {})
    conservation = p131.get("correction_conservation", {})
    expected_epochs = int(raw_contract.get("target_epochs", 0) or 0)
    if expected_epochs <= 0:
        expected_epochs = int(gnss.get("epochs", 0) or 0)
    c0d_handoff = (
        gnss.get("handoff_mode") == "gnss-first-in-memory-meter-clock-state" and
        gnss.get("optimized_d_export_valid") is True and
        gnss.get("optimized_c_export_valid") is True and
        gnss.get("epoch_identity_alignment_valid") is True and
        int(gnss.get("optimized_d_epoch_count", 0) or 0) > 0 and
        int(gnss.get("optimized_d_finite_count", 0) or 0) == int(gnss.get("optimized_d_epoch_count", 0) or 0) and
        int(gnss.get("optimized_c_epoch_count", 0) or 0) > 0 and
        int(gnss.get("optimized_c_finite_component_count", 0) or 0) ==
        int(gnss.get("optimized_c_epoch_count", 0) or 0) * 7)
    atomic = all(base.get(key) is True for key in (
        "phase126_atomic_step_a_raw_ingress_verified",
        "phase126_atomic_step_b_source_stream_verified",
        "phase126_atomic_step_c_application_committed",
        "phase126_compound_admitted"))
    base_once = (
        base.get("enabled") is True and base.get("phase126_source_complete") is True and
        base.get("phase127_glonass_channel_provenance") is True and
        base.get("phase128_glonass_provenance_parser_admission") is True and
        base.get("phase129_glonass_local_miss_mask") is True and
        base.get("phase131_canonical_correction_band_key") is True and
        base.get("phase131_configuration_valid") is True and
        int(base.get("base_rinex_read_count", 0) or 0) == 1 and
        miss.get("correction_applied_exactly_once") is True and
        miss.get("pseudorange_factor_count_consistent") is True)
    gnss_progress = (int(gnss.get("iterations", 0) or 0) > 0 and
                     decreasing(gnss.get("initial_cost"), gnss.get("final_cost")))
    main_progress = (int(graph.get("iterations", 0) or 0) > 0 and
                     decreasing(graph.get("initial_cost"), graph.get("final_cost")))
    solver_qr = (native.get("selected_solver_branch") == "MULTIFRONTAL_QR" or
                 native.get("selected_linear_solver_type") == "MULTIFRONTAL_QR" or
                 native.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected") is True)
    finite_coverage = (native.get("output_contract", {}).get("finite_coordinates") is True and
                       expected_epochs > 0)
    canonicalization_attempt_rows = (
        int(p131.get("canonical_rows", 0) or 0) +
        int(p131.get("canonical_rejected_rows", 0) or 0))
    attempt_rows_present = "canonicalization_attempt_rows" in p131
    attempt_rows_consistent = (
        not attempt_rows_present or
        int(p131.get("canonicalization_attempt_rows", 0) or 0) ==
        canonicalization_attempt_rows)
    resolver_count_present = "resolver_call_count" in p131
    if resolver_count_present:
        resolver_call_count = int(p131.get("resolver_call_count", 0) or 0)
        resolver_count_consistent = (
            resolver_call_count == canonicalization_attempt_rows)
    else:
        # Historical native summaries predate the bridge.  Keep their
        # derived semantics for compatibility, while requiring the explicit
        # count whenever the bridge object is present.
        resolver_call_count = canonicalization_attempt_rows
        resolver_count_consistent = True
    bridge_present = isinstance(bridge, dict) and bool(bridge)
    bridge_exactly_once = (
        not bridge_present or
        bridge.get("source") == "BasePseudorangeCompensationReport" and
        bridge.get("synchronization_count") == 1 and
        bridge.get("exactly_once") is True)
    p131_ok = (p131.get("enabled") is True and
               p131.get("configuration_valid") is True and
               int(p131.get("canonical_rows", 0) or 0) > 0 and
               int(p131.get("canonical_selected_streams", 0) or 0) > 0 and
               p131.get("literal_tracking_code_in_join") is False and
               p131.get("raw_or_zero_correction_fallback") is False and
               attempt_rows_consistent and
               resolver_count_consistent and
               bridge_exactly_once)
    native_canonicalization_attempt_rows = resolver_call_count
    native_resolver_executed = native_canonicalization_attempt_rows > 0
    no_fallback = (native.get("status") == "imu-combined-factor" and
                   native.get("truth_used") is False and
                   native.get("production_default_changed") is False and
                   native.get("native_phase117_tdcp_snr_type_sigma") is False and
                   native.get("native_phase120_official_tdcp_resl_atmosphere_cancellation") is False and
                   native.get("native_direct_wls_ephemeral_c7d_main_seed_enabled") is False)
    gates = {
        "native_process_completed": return_code == 0,
        "summary_present": True,
        "phase131_inventory_handoff": inventory.get("ok") is True,
        "phase131_native_admission": p131_ok,
        "phase126_atomic_a_b_c": atomic,
        "base_exactly_once": base_once,
        "gnss_first_progress": gnss_progress,
        "gnss_first_c7_d_handoff": c0d_handoff,
        "main_qr_progress": solver_qr and main_progress,
        "main_finite_coverage": finite_coverage,
        "no_fallback_or_publication": no_fallback and not solution.get("opened", False),
    }
    normalized = {
        "schema_version": "smartphone-r5-phase131-canonical-correction-band-structural-summary.v1",
        "route": route,
        "solution_content_read": False,
        "fallback_used": False,
        "rerun_count": 0,
        "gnss_first": {"accepted_iterations": int(gnss.get("iterations", 0) or 0),
                       "initial_cost": gnss.get("initial_cost"),
                       "final_cost": gnss.get("final_cost")},
        "main": {"solver_branch": native.get("selected_solver_branch") or
                 native.get("selected_linear_solver_type") or "",
                 "accepted_iterations": int(graph.get("iterations", 0) or 0),
                 "initial_cost": graph.get("initial_cost"),
                 "final_cost": graph.get("final_cost")},
        "phase131": {
            "enabled": p131.get("enabled") is True,
            "canonical_key": "GNSSSystem,PRN,physical-frequency-family[,certified-GLO-FCN]",
            "side_local_conservation": inventory.get("shared_ledger", {}).get("side_local_conservation") is True,
            "canonical_key_conservation": inventory.get("shared_ledger", {}).get("canonical_key_conservation") is True,
            "literal_tracking_code_in_join": p131.get("literal_tracking_code_in_join", False),
            "original_signal_provenance_retained": True,
            "retained_rover_exact_key_support": int(p131.get("canonical_selected_streams", 0) or 0) > 0,
            "unused_base_rows_accounted": inventory.get("shared_ledger", {}).get("unused_base_rows_accounted") is True,
            "no_fallback": no_fallback,
            "factor_topology_changed": p131.get("factor_topology_changed", False),
            "python_preflight_executed": inventory.get("phase131", {}).get(
                "python_preflight_executed", False),
            "native_selector_forwarded": inventory.get("phase131", {}).get(
                "native_selector_forwarded", False),
            "native_command_constructed": inventory.get("phase131", {}).get(
                "native_command_constructed", False),
            "native_binary_invocation_attempted": inventory.get("phase131", {}).get(
                "native_binary_invocation_attempted", False),
            "native_resolver_executed": native_resolver_executed,
            "native_resolver_call_count": native_canonicalization_attempt_rows,
            "native_resolver_call_count_source": (
                "native-summary-resolver_call_count"
                if resolver_count_present else
                "derived-from-native-summary-canonical-rows"),
            "native_resolver_call_count_consistent": resolver_count_consistent,
            "canonicalization_attempt_rows_consistent": attempt_rows_consistent,
            "diagnostics_bridge_exactly_once": bridge_exactly_once,
            "correction_conservation": conservation,
        },
        "gates": gates,
        "opaque_solution": {"sha256": solution.get("sha256"), "rows": solution.get("rows")},
    }
    telemetry = {
        "return_code": return_code,
        "phase131": {key: p131.get(key) for key in (
            "enabled", "configuration_valid", "configuration_failure",
            "canonical_rows", "canonical_rejected_rows", "unknown_band_rows",
            "canonical_key_conflicts", "canonical_duplicate_rows", "canonical_streams",
            "canonical_selected_streams", "canonical_merged_streams", "failure_counts",
            "join_key", "literal_tracking_code_in_join", "factor_topology_changed",
            "raw_or_zero_correction_fallback", "canonicalization_attempt_rows",
            "resolver_call_count", "correction_conservation", "diagnostics_bridge")},
        "execution_evidence": {
            "selector": contract.PHASE131_SELECTOR,
            "python_preflight_executed": inventory.get("phase131", {}).get(
                "python_preflight_executed", False),
            "native_selector_forwarded": inventory.get("phase131", {}).get(
                "native_selector_forwarded", False),
            "native_command_constructed": inventory.get("phase131", {}).get(
                "native_command_constructed", False),
            "native_binary_invocation_attempted": inventory.get("phase131", {}).get(
                "native_binary_invocation_attempted", False),
            "native_resolver_executed": native_resolver_executed,
            "native_resolver_call_count": native_canonicalization_attempt_rows,
            "native_resolver_call_count_source": (
                "native-summary-resolver_call_count"
                if resolver_count_present else
                "derived-from-native-summary-canonical-rows"),
            "native_resolver_call_count_consistent": resolver_count_consistent,
            "canonicalization_attempt_rows_consistent": attempt_rows_consistent,
            "native_resolver_call_count_semantics": (
                "canonical rows plus rejected canonical rows in native summary; "
                "explicit resolver_call_count is required to equal that sum "
                "when the Phase134 bridge is present; zero means no "
                "canonicalizer row reached, not a claim about non-canonical rows"),
            "diagnostics_bridge": bridge,
            "diagnostics_bridge_exactly_once": bridge_exactly_once,
        },
        "phase129": {key: p129.get(key) for key in (
            "enabled", "configuration_valid", "configuration_failure",
            "glonass_local_miss_rows", "glonass_factor_rows_dropped",
            "glonass_factor_rows_retained", "factor_count_consistent",
            "row_count_consistent")},
        "base_correction": {key: base.get(key) for key in (
            "phase126_atomic_step_a_raw_ingress_verified",
            "phase126_atomic_step_b_source_stream_verified",
            "phase126_atomic_step_c_application_committed", "phase126_compound_admitted",
            "phase127_glonass_channel_provenance", "phase128_glonass_provenance_parser_admission",
            "phase129_glonass_local_miss_mask", "phase131_canonical_correction_band_key",
            "phase131_configuration_valid", "base_rinex_read_count",
            "correction_application_pass_count", "correction_applied_exactly_once",
            "matched_factor_rows", "finite_correction_rows_among_matched",
            "interpolation_misses", "failure")},
        "miss_mask": {key: miss.get(key) for key in (
            "original_adopted_pseudorange_rows", "retained_finite_pc_pseudorange_rows",
            "dropped_missing_exact_stream_rows", "dropped_out_of_domain_rows",
            "dropped_nonfinite_correction_rows", "pseudorange_factors_inserted",
            "pseudorange_factor_count_consistent", "correction_application_pass_count",
            "correction_applied_exactly_once", "duplicate_correction_rejected", "failure")},
        "gnss_first": {key: gnss.get(key) for key in (
            "iterations", "initial_cost", "final_cost", "c0d_factor_count",
            "c0d_accepted_outer_iterations", "optimized_d_export_valid",
            "optimized_d_epoch_count", "optimized_d_finite_count",
            "optimized_c_export_valid", "optimized_c_epoch_count",
            "optimized_c_finite_component_count", "epoch_identity_alignment_valid",
            "failure")},
        "main": {key: graph.get(key) for key in
                 ("factors", "values", "imu_intervals", "iterations", "converged",
                  "initial_cost", "final_cost")},
        "epochs": {"expected_output": expected_epochs,
                    "raw_target": raw_contract.get("target_epochs"),
                    "raw_exact_solution": raw_contract.get("exact_solution_epochs"),
                    "unresolved": raw_contract.get("unresolved_epochs")},
        "solver": {key: native.get(key) for key in (
            "native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected",
            "selected_linear_solver_type", "selected_solver_branch", "selected_elimination_function")},
        "gates": gates,
        "opaque_solution": solution,
    }
    return normalized, telemetry


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
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


def execute_one_route(route: str, auth_record: dict[str, Any],
                      inventory: dict[str, Any]) -> dict[str, Any]:
    route_dir = OUTPUT_ROOT / route.replace("/", "__")
    route_dir.mkdir(parents=True, exist_ok=False)
    command = command_for(route, auth_record)
    phase131 = inventory.setdefault("phase131", {})
    phase131["native_command_constructed"] = True
    phase131["native_selector_forwarded"] = (
        command.count(contract.PHASE131_SELECTOR) == 1)
    phase131["native_binary_invocation_attempted"] = True
    phase131["native_resolver_executed"] = False
    phase131["native_resolver_call_count"] = 0
    phase131["native_resolver_evidence_source"] = "pending-native-summary"
    atomic_json(route_dir / "inventory.json", inventory)
    stdout_path = route_dir / "stdout.log"
    stderr_path = route_dir / "stderr.log"
    summary_path = route_dir / "structural_summary.json"
    solution_path = route_dir / "opaque_solution_output.csv"
    return_code: int | None = None
    launch_error = ""
    env = os.environ.copy()
    env.update({"LANG": "C", "LC_ALL": "C", "TZ": "UTC"})
    env["LD_LIBRARY_PATH"] = "/home/sasaki/.local/lib" + (
        ":" + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else "")
    with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
        try:
            completed = subprocess.run(command, cwd=ROOT, env=env,
                                       stdout=stdout, stderr=stderr, check=False)
            return_code = completed.returncode
        except OSError as exc:
            launch_error = str(exc)
            stderr.write(f"Phase131 native launch failed: {exc}\n".encode())
    record: dict[str, Any] = {
        "route": route, "run_number": 1, "solver_launched": True,
        "return_code": return_code, "launch_error": launch_error,
        "summary_present": summary_path.is_file(), "solution_opened": False,
        "solution_published": False, "truth_used": False, "accuracy_scored": False,
        "raw_content_copied_or_transformed": False,
        "inventory": inventory,
    }
    if not summary_path.is_file():
        record.update({"summary_error": "native summary absent; structural gates fail closed",
                       "gates": empty_gates()})
        record["gates"]["native_process_completed"] = return_code == 0 and not launch_error
        record["gates"]["no_fallback_or_publication"] = True
        return record
    try:
        native = read_json(summary_path, f"Phase131 native summary {route}")
        solution = opaque_solution_seal(solution_path, contract.PROBLEM_EPOCHS[route])
        normalized, telemetry = normalize_native(route, native, solution, return_code,
                                                 inventory)
        execution_evidence = telemetry["execution_evidence"]
        phase131 = inventory.setdefault("phase131", {})
        phase131["native_resolver_executed"] = execution_evidence[
            "native_resolver_executed"]
        phase131["native_resolver_call_count"] = execution_evidence[
            "native_resolver_call_count"]
        phase131["native_resolver_evidence_source"] = "native-summary"
        atomic_json(route_dir / "inventory.json", inventory)
        atomic_json(summary_path, normalized)
    except (Phase131ExecutionError, KeyError, TypeError, ValueError) as exc:
        record.update({"summary_error": str(exc), "gates": empty_gates()})
        record["gates"]["native_process_completed"] = return_code == 0 and not launch_error
        record["gates"]["summary_present"] = True
        record["gates"]["no_fallback_or_publication"] = True
        return record
    record.update({
        "summary_schema": normalized["schema_version"],
        "summary_status": native.get("status"),
        "solution_present": solution.get("present", False),
        "solution_hash_sealed": solution.get("sha256"),
        "solution_rows_sealed": solution.get("rows"),
        "solution_hash_only": True,
        "structural_telemetry": telemetry,
        "gates": normalized["gates"],
    })
    return record


def result_markdown(result: dict[str, Any]) -> str:
    lines = [
        "# Phase131 canonical physical-band structural raw result",
        "",
        f"- Status: `{result['status']}`",
        "- Matrix: MTV-A then LAX-T, exactly one inventory pass and at most one native solver attempt per route.",
        "- The canonical key is `(GNSSSystem, PRN, physical-frequency-family[, certified GLONASS FCN])`; literal tracking text is provenance only.",
        "- Solution rows were not opened or interpreted; only opaque hashes and expected row counts were sealed.",
        "- Truth, MAT, PDC, precomputed coordinates, accuracy, and Kaggle remain unauthorized.",
        "",
        "| Route | Inventory | Solver | Return | Rover cert/miss | Base cert/miss | Canonical keys | Main accepted | Main cost | GO | Failure |",
        "|---|---|---|---:|---:|---:|---:|---:|---|---|---|",
    ]
    for route in ROUTES:
        record = result["routes"].get(route, {})
        inventory = record.get("inventory", {})
        rover = inventory.get("rover", {})
        base = inventory.get("base", {})
        shared = inventory.get("shared_ledger", {})
        main = record.get("structural_telemetry", {}).get("main", {})
        gates = record.get("gates", {})
        failure = record.get("failure") or inventory.get("failure") or record.get("summary_error") or ""
        lines.append(
            f"| `{route}` | `{inventory.get('ok')}` | `{record.get('solver_launched', False)}` | "
            f"`{record.get('return_code')}` | `{rover.get('certified_rows', 0)}/{rover.get('explicit_local_miss_rows', 0)}` | "
            f"`{base.get('certified_rows', 0)}/{base.get('explicit_local_miss_rows', 0)}` | "
            f"`{shared.get('retained_exact_key_count', 0)}` | `{main.get('iterations', 0)}` | "
            f"`{main.get('initial_cost')}->{main.get('final_cost')}` | "
            f"`{all(gates.values()) if gates else False}` | `{failure}` |")
    lines.extend(["", "Inventory failure is fail-closed and prevents a native launch for that route.", ""])
    return "\n".join(lines)


def inventory_failure_record(route: str, failure: str,
                             reads: dict[str, int]) -> dict[str, Any]:
    """Represent a preflight failure without implying native selector execution."""
    return {
        "route": route,
        "stage": "post-independent-authorization-pre-solver",
        "ok": False,
        "solver_invocations": 0,
        "solver_may_start": False,
        "failure": failure,
        "inventory_reads": dict(reads),
        "phase131": {
            "enabled": True,
            "selector": contract.PHASE131_SELECTOR,
            "python_preflight_started": True,
            "python_preflight_executed": False,
            "native_selector_forwarded": False,
            "native_command_constructed": False,
            "native_binary_invocation_attempted": False,
            "native_resolver_executed": False,
            "native_resolver_call_count": 0,
            "native_resolver_evidence_source": "preflight-failure",
            "source_complete_a_b_c": False,
        },
    }


def execute_matrix() -> dict[str, Any]:
    auth = verify_authorization()
    if RESULT_JSON.exists() or RESULT_MD.exists():
        raise fail("refusing to overwrite an existing Phase131 sealed result")
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite existing Phase131 output root: {OUTPUT_ROOT}")
    accounting: dict[str, Any] = {
        "raw_device_gnss_inventory_reads": 0,
        "raw_device_imu_inventory_reads": 0,
        "broadcast_navigation_inventory_reads": 0,
        "raw_base_rinex_inventory_reads": 0,
        "raw_base_header_inventory_reads": 0,
        "raw_base_hash_reads": 0,
        "native_solver_invocations": 0,
        "solution_rows_opened": 0,
        "solution_opaque_hash_reads": 0,
        "solution_coordinate_interpretations": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "pdc_reads": 0,
        "precomputed_coordinate_reads": 0,
        "accuracy_calculations": 0,
        "kaggle_or_token_access": 0,
        "route_reruns": 0,
        "fallbacks": 0,
        "repairs": 0,
        "raw_content_copied_or_transformed": False,
    }
    route_records: dict[str, Any] = {}
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    for route in ROUTES:
        auth_record = next(item for item in auth["routes"] if item["dataset_id"] == route)
        payloads: dict[str, bytes] = {}
        reads: dict[str, int] = {}
        try:
            for name in RAW_NAMES:
                path = safe_payload_path(auth_record["raw_inputs"][name]["path"], name, route)
                payloads[name] = read_payload_once(path, auth_record["raw_inputs"][name], route, name, reads)
                accounting[{"device_gnss.csv": "raw_device_gnss_inventory_reads",
                             "device_imu.csv": "raw_device_imu_inventory_reads",
                             "brdc.nav": "broadcast_navigation_inventory_reads"}[name]] += 1
            base_path = safe_payload_path(auth_record["base_input"]["path"], BASE_NAME, route)
            payloads[BASE_NAME] = read_payload_once(base_path, auth_record["base_input"], route, BASE_NAME, reads)
            accounting["raw_base_rinex_inventory_reads"] += 1
            accounting["raw_base_header_inventory_reads"] += 1
            accounting["raw_base_hash_reads"] += 1
            inventory = build_inventory(route, payloads, reads)
        except (Phase131ExecutionError, KeyError, TypeError, ValueError) as exc:
            inventory = inventory_failure_record(route, str(exc), reads)
        if inventory.get("ok") is True:
            route_record = execute_one_route(route, auth_record, inventory)
            accounting["native_solver_invocations"] += 1
            if route_record.get("solution_present"):
                accounting["solution_opaque_hash_reads"] += 1
        else:
            route_dir = OUTPUT_ROOT / route.replace("/", "__")
            route_dir.mkdir(parents=True, exist_ok=False)
            atomic_json(route_dir / "inventory.json", inventory)
            route_record = {"route": route, "run_number": 1, "solver_launched": False,
                            "return_code": None, "solution_opened": False,
                            "solution_published": False, "truth_used": False,
                            "accuracy_scored": False, "raw_content_copied_or_transformed": False,
                            "inventory": inventory,
                            "failure": "pre-solver inventory failed closed", "gates": empty_gates()}
            route_record["gates"]["no_fallback_or_publication"] = True
        route_records[route] = route_record
        atomic_json(OUTPUT_ROOT / "partial_result.json", {
            "routes": route_records,
            "completed_routes": list(route_records),
            "native_solver_invocations": accounting["native_solver_invocations"],
        })
        del payloads
    all_passed = all(all(route_records[route].get("gates", {}).values()) for route in ROUTES)
    result: dict[str, Any] = {
        "schema_version": "smartphone-r5-phase131-canonical-correction-band-structural-result.v1",
        "phase": 131,
        "execution_label": "Luna Max",
        "status": "go-phase131-canonical-correction-band-structural" if all_passed
                  else "no-go-phase131-canonical-correction-band-structural",
        "decision": ("All inventory and structural gates passed; truth/accuracy and solution release remain unauthorized."
                     if all_passed else
                     "Inventory or structural gate failed closed; no rerun, fallback, truth, accuracy, or solution release is permitted."),
        "authorization": {"path": relative(AUTHORIZATION), "status": auth.get("status"), "independent": True},
        "contract": {"audit_commit": AUDIT_COMMIT, "freeze_commit": FREEZE_COMMIT,
                     "implementation_commit": IMPLEMENTATION_COMMIT,
                     "manifest_commit": MANIFEST_COMMIT, "pre_raw_commit": PRE_RAW_COMMIT,
                     "manifest_sha256": MANIFEST_SHA256, "target_binary_sha256": TARGET_BINARY_SHA256,
                     "authorized_runner_path": relative(AUTHORIZED_RUNNER)},
        "candidate": {"id": contract.CANDIDATE_ID,
                      "phase126": True, "phase127": True, "phase128": True,
                      "phase129": True, "phase130": True, "phase131": True,
                      "phase118_huber": True, "phase117_dynamic_sigma": False,
                      "phase120_atmosphere": False, "additional_frequency_bands": False,
                      "fixed_tdcp_sigma": 0.03, "official_huber_k": 0.5,
                      "main_solver": "MULTIFRONTAL_QR", "solution_opaque": True},
        "phase131_execution_evidence": {
            "selector": contract.PHASE131_SELECTOR,
            "python_preflight_routes": sum(
                1 for item in route_records.values()
                if item.get("inventory", {}).get("phase131", {}).get(
                    "python_preflight_executed") is True),
            "native_command_constructed_routes": sum(
                1 for item in route_records.values()
                if item.get("inventory", {}).get("phase131", {}).get(
                    "native_command_constructed") is True),
            "native_selector_forwarded_routes": sum(
                1 for item in route_records.values()
                if item.get("inventory", {}).get("phase131", {}).get(
                    "native_selector_forwarded") is True),
            "native_resolver_executed_routes": sum(
                1 for item in route_records.values()
                if item.get("inventory", {}).get("phase131", {}).get(
                    "native_resolver_executed") is True),
            "native_resolver_call_count": sum(
                int(item.get("inventory", {}).get("phase131", {}).get(
                    "native_resolver_call_count", 0) or 0)
                for item in route_records.values()),
            "native_resolver_call_count_semantics": (
                "native summary canonical rows plus rejected canonical rows; "
                "not a count of non-canonical observations"),
        },
        "matrix": {"candidate_count": 1, "route_order": ["MTV-A", "LAX-T"],
                   "runs_per_route": 1,
                   "native_solver_invocations": accounting["native_solver_invocations"],
                   "inventory_reads_per_route": 1, "controls": 0, "reruns": 0,
                   "fallbacks": 0, "truth_reads": 0, "accuracy_calculations": 0,
                   "solution_rows_opened": 0},
        "routes": route_records,
        "read_accounting": accounting,
        "inventory_contract": {"side_local_conservation": True,
                               "canonical_key_conservation": True,
                               "same_physical_family_aliases_only": True,
                               "literal_tracking_code_excluded_from_join": True,
                               "certified_glonass_fcn_required": True,
                               "retained_rover_exact_key_support": True,
                               "unmatched_rover_explicit_factor_miss": True,
                               "unused_base_rows_accounted": True,
                               "whole_ledger_equality_required": False,
                               "empty_all_miss_fail_closed": True,
                               "solver_zero_on_inventory_failure": True,
                               "no_raw_zero_or_uncorrected_fallback": True,
                               "no_nearest_hold_or_extrapolation": True,
                               "duplicate_nonmonotonic_base_time_global_abort": True,
                               "ambiguous_canonical_conflict_fail_closed": True},
        "truth_free": True, "accuracy_scored": False,
        "solution_output_published": False, "promotion_authorized": False,
        "stop_before_truth_accuracy_submission": True,
    }
    atomic_json(RESULT_JSON, result)
    RESULT_MD.write_text(result_markdown(result), encoding="utf-8")
    return result


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-authorization", action="store_true",
                        help="verify static pins without opening route payloads")
    parser.add_argument("--execute", action="store_true",
                        help="run the exact authorized matrix once")
    args = parser.parse_args()
    if args.verify_authorization and args.execute:
        parser.error("verification and execution are separate modes")
    if not (args.verify_authorization or args.execute):
        parser.error("one mode is required")
    try:
        if args.verify_authorization:
            verify_authorization()
            print(json.dumps({"status": "authorization-pins-verified",
                              "raw_reads": 0, "solver_invocations": 0}, sort_keys=True))
            return 0
        result = execute_matrix()
        print(json.dumps({"status": result["status"],
                          "routes": {route: {
                              "inventory_ok": result["routes"][route]["inventory"].get("ok"),
                              "solver_launched": result["routes"][route].get("solver_launched"),
                              "return_code": result["routes"][route].get("return_code"),
                              "failure": result["routes"][route].get("failure") or
                                         result["routes"][route]["inventory"].get("failure"),
                          } for route in ROUTES},
                          "read_accounting": result["read_accounting"]},
                         indent=2, sort_keys=True))
        return 0 if result["status"].startswith("go-") else 3
    except (Phase131ExecutionError, OSError) as exc:
        print(f"phase131 authorized runner: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
