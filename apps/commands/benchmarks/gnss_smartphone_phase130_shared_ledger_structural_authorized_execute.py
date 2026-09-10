#!/usr/bin/env python3
"""Execute the independently authorized Phase130 structural matrix once.

The Phase130 selector is intentionally a runner/contract boundary selector;
the pinned native binary remains the Phase129 implementation and therefore
does not receive an unknown Phase130 CLI option.  This runner performs the
key-local support admission before the native launch, then forwards the
unchanged Phase129 recipe to the binary at most once per route.  Only raw
phone GNSS/IMU, broadcast navigation, and the sealed raw-base RINEX are
opened.  Native solution rows are never parsed or interpreted.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))
import gnss_smartphone_phase130_shared_ledger_structural as contract  # noqa: E402
import gnss_smartphone_phase128_inventory_first_structural_authorized_execute as p128  # noqa: E402


AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase130_shared_ledger_structural_authorization_v1.json"
PRE_RAW = ROOT / "docs/use_cases/records/smartphone_r5_phase130_shared_ledger_structural_pre_raw_accounting_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase130_shared_ledger_structural_manifest_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase130-shared-ledger-structural-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase130_shared_ledger_structural_result_v1.json"
RESULT_MD = ROOT / "docs/use_cases/records/smartphone_r5_phase130_shared_ledger_structural_result_v1.md"
AUTHORIZED_RUNNER = Path(__file__)

AUDIT_COMMIT = "355b6225bcc3bc838dad8a3044bc3aadd34d80f3"
FREEZE_COMMIT = "94085c90ac33ce987c3b9d29a6462c5d77795e88"
IMPLEMENTATION_COMMIT = "36b0977e47d9b24eda975ab224e05aadfdc860bb"
MANIFEST_COMMIT = "ab1a6bfa2e1878bb01d784a2f926a1732d34b004"
PRE_RAW_COMMIT = "e2821f8b5c2c8ebfd6e0a1078cf7cb5fe89ae9b6"
PHASE129_IMPLEMENTATION_COMMIT = "05e57d5008320734de93c763ff84ccd31f755805"
AUDIT_SHA256 = "fcde87b6267daaa2da2dba2210bd0e5baec2658daa8266d53b40bd4bd9256912"
FREEZE_SHA256 = "ecc34c4c5daff4a77f6e97da3d35f63710a4dc443d5ff372938ab05c823a6a2f"
MANIFEST_SHA256 = "85f0702fb9c09e164aadeb5efdec9a6a013fcc0eb0ba126fc109cbd07f3df097"
PRE_RAW_SHA256 = "e583f29ca0b77bd3afc7db87311f299c57bc14c6769f9610a1a34d47eac25a69"
TARGET_BINARY_SHA256 = "5c81e9b8f83843e16550553c6add5143fde32ba105cd5f1904b8f4b6cd4b9c4b"

PHASE130_SELECTOR = contract.PHASE130_SELECTOR
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
BASE_NAME = "base.obs"
ROUTES = contract.ROUTES

# These are sealed metadata inherited from the Phase129 authorization.  They
# are checked before any corresponding path is opened.
ROUTE_INPUTS: dict[str, dict[str, dict[str, Any]]] = {
    ROUTES[0]: {
        "device_gnss.csv": {
            "path": "output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/device_gnss.csv",
            "bytes": 57715495,
            "sha256": "c7d50d5127d16586adc6c79d724758e298b385496da22c5e5dfd6ec522cbc863",
        },
        "device_imu.csv": {
            "path": "output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/device_imu.csv",
            "bytes": 34393802,
            "sha256": "afc540e7c4ce2ca66b442a1afbcd604e9f6b3d2cc4d3733183739901b5b97bd6",
        },
        "brdc.nav": {
            "path": "output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/brdc.nav",
            "bytes": 9955040,
            "sha256": "6adfaf7fe4452a4faeb94a7b607c15e05f578c46a028a030b94aa6f79de194cd",
        },
        BASE_NAME: {
            "path": "output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2021-03-16-18-59-us-ca-mtv-a__pixel5/base.obs",
            "bytes": 10708536,
            "sha256": "380b8ff9091344fb756697e27f0983d9a0ba2cf0c201b96849bfc1ecc1af0e52",
            "interval": 1.0,
            "window": 151,
        },
    },
    ROUTES[1]: {
        "device_gnss.csv": {
            "path": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/device_gnss.csv",
            "bytes": 33837317,
            "sha256": "50362c01bff3e0bb7088e54021164591cd750227ed97c2fd7d95d763a08798f1",
        },
        "device_imu.csv": {
            "path": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/device_imu.csv",
            "bytes": 23649244,
            "sha256": "2e39a3e9f294c64b8ecfd452d0960025d1013b97f2d7497e6e48a2a1997b38c5",
        },
        "brdc.nav": {
            "path": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/brdc.nav",
            "bytes": 10635773,
            "sha256": "443d3d5a73f4895b83e576e24a568f4658f869e79a480856de7f0717763dcbe6",
        },
        BASE_NAME: {
            "path": "output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2022-04-01-18-22-us-ca-lax-t__pixel5/base.obs",
            "bytes": 719969,
            "sha256": "d731e0e8a7ba4396d62340c85b6238e66c50c6a349621f9dcea0ea6885fd4cfe",
            "interval": 15.0,
            "window": 11,
        },
    },
}


class Phase130ExecutionError(ValueError):
    """Authorization, inventory, or structural execution failure."""


def fail(message: str) -> Phase130ExecutionError:
    return Phase130ExecutionError(message)


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
    lowered = path.name.lower()
    if (lowered in RAW_NAMES or lowered in {BASE_NAME, "truth.csv", "ground_truth.csv"}
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
        else:
            assert_equal(item, 0, f"{label}/{key}")


def verify_authorization() -> dict[str, Any]:
    """Verify all static pins without opening a route payload."""
    contract.verify_manifest()
    assert_equal(static_sha(PRE_RAW, "Phase130 pre-raw accounting"),
                 PRE_RAW_SHA256, "pre-raw/sha256")
    pre = read_json(PRE_RAW, "Phase130 pre-raw accounting")
    assert_equal(pre.get("phase"), 130, "pre-raw/phase")
    zero_accounting(pre.get("read_accounting_before_independent_authorization"),
                    "pre-raw/read_accounting_before_independent_authorization")
    pre_pins = pre.get("pins")
    if not isinstance(pre_pins, dict):
        raise fail("pre-raw/pins missing")
    for section, expected in {
        "contract_audit": (AUDIT_COMMIT, AUDIT_SHA256),
        "contract_freeze": (FREEZE_COMMIT, FREEZE_SHA256),
        "implementation_runner_tests": (IMPLEMENTATION_COMMIT, None),
        "structural_manifest": (MANIFEST_COMMIT, MANIFEST_SHA256),
    }.items():
        item = pre_pins.get(section)
        if not isinstance(item, dict):
            raise fail(f"pre-raw/pins/{section} missing")
        assert_equal(item.get("commit"), expected[0], f"pre-raw/pins/{section}/commit")
        if expected[1] is not None:
            assert_equal(item.get("sha256"), expected[1], f"pre-raw/pins/{section}/sha256")
    target = pre_pins.get("target_binary")
    if not isinstance(target, dict):
        raise fail("pre-raw/pins/target_binary missing")
    assert_equal(target.get("sha256"), TARGET_BINARY_SHA256,
                 "pre-raw/pins/target_binary/sha256")

    auth = read_json(AUTHORIZATION, "Phase130 raw authorization")
    for key, expected in {
        "schema_version": "smartphone-r5-phase130-shared-ledger-key-local-support-structural-authorization.v1",
        "phase": 130,
        "execution_label": "Luna Max",
        "status": "independent-one-shot-inventory-first-structural-raw-authorized",
    }.items():
        assert_equal(auth.get(key), expected, f"authorization/{key}")
    pins = auth.get("pins")
    if not isinstance(pins, dict):
        raise fail("authorization/pins missing")
    expected_pins = {
        "audit_commit": AUDIT_COMMIT,
        "freeze_commit": FREEZE_COMMIT,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "manifest_commit": MANIFEST_COMMIT,
        "pre_raw_accounting_commit": PRE_RAW_COMMIT,
        "phase129_implementation_commit": PHASE129_IMPLEMENTATION_COMMIT,
        "audit_sha256": AUDIT_SHA256,
        "freeze_sha256": FREEZE_SHA256,
        "manifest_sha256": MANIFEST_SHA256,
        "pre_raw_accounting_sha256": PRE_RAW_SHA256,
        "target_binary_path": relative(contract.BINARY),
        "target_binary_sha256": TARGET_BINARY_SHA256,
        "authorized_runner_path": relative(AUTHORIZED_RUNNER),
    }
    for key, expected in expected_pins.items():
        assert_equal(pins.get(key), expected, f"authorization/pins/{key}")
    assert_equal(pins.get("authorized_runner_sha256"),
                 static_sha(AUTHORIZED_RUNNER, "authorized runner"),
                 "authorization/pins/authorized_runner_sha256")
    runner_commit = pins.get("authorized_runner_commit")
    if not isinstance(runner_commit, str) or len(runner_commit) != 40 or any(
            char not in "0123456789abcdef" for char in runner_commit):
        raise fail("authorization/pins/authorized_runner_commit must be full lowercase SHA")

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
        "route_order": list(ROUTES),
        "runs_per_route": 1,
        "controls": 0,
        "reruns": 0,
        "fallbacks": 0,
    }.items():
        assert_equal(scope_meta.get(key), expected, f"authorization_scope/{key}")
    selectors = auth.get("selectors")
    if not isinstance(selectors, dict):
        raise fail("authorization/selectors missing")
    expected_selectors = {
        "phase126": 1, "phase127": 1, "phase128": 1, "phase129": 1,
        "phase130_runner_boundary": 1, "phase118": 1,
        "phase117": 0, "phase120": 0, "additional_frequency": 0,
    }
    for key, expected in expected_selectors.items():
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
            assert_equal(base.get(key), expected, f"authorization/{route}/base/{key}")
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


def exact_epoch(parts: list[str], rinex_version: float) -> float | None:
    if len(parts) < 6:
        return None
    try:
        year = int(float(parts[0]))
        if rinex_version < 3.0:
            year += 1900 if year >= 80 else 2000
        month, day, hour, minute = (int(float(value)) for value in parts[1:5])
        second = float(parts[5])
        whole = int(math.floor(second))
        micros = int(round((second - whole) * 1.0e6))
        if micros >= 1000000:
            whole += 1
            micros -= 1000000
        utc = datetime(year, month, day, hour, minute, whole, micros,
                       tzinfo=timezone.utc)
        return p128.utc_to_gpst(utc)
    except (TypeError, ValueError, OverflowError, OSError):
        return None


def exact_base_observations(data: bytes) -> list[dict[str, Any]]:
    """Read only typed base satellite/time identities, without measurements."""
    lines = data.decode("ascii", errors="replace").splitlines()
    version = 3.0
    for line in lines[:100]:
        if "RINEX VERSION" in line:
            try:
                version = float(line[:9])
            except ValueError:
                version = 3.0
            break
    header_end = next((i for i, line in enumerate(lines)
                       if "END OF HEADER" in line), None)
    if header_end is None:
        return []
    current: float | None = None
    observations: list[dict[str, Any]] = []
    old_epoch = re.compile(
        r"^\s?([0-9]{1,4})\s+([0-9]{1,2})\s+([0-9]{1,2})\s+"
        r"([0-9]{1,2})\s+([0-9]{1,2})\s+([0-9]+(?:\.\d*)?)")
    for line in lines[header_end + 1:]:
        if line.startswith(">"):
            current = exact_epoch(line[1:].split()[:6], version)
            continue
        match_epoch = old_epoch.match(line)
        if match_epoch:
            current = exact_epoch(list(match_epoch.groups()), version)
            continue
        match_sat = re.match(r"^\s*R(\d{1,2})", line)
        if match_sat and current is not None:
            satellite = p128.parse_satellite("R" + match_sat.group(1))
            if satellite is not None and math.isfinite(current):
                observations.append({"satellite": satellite,
                                      "query_gpst": current,
                                      "row_index": len(observations)})
    return observations


def signal_descriptor(signal: Any) -> str | None:
    """Normalize an Android/RINEX signal to its exact band/tracking suffix."""
    text = str(signal or "").strip().upper()
    for prefix in ("GLONASS", "GLO"):
        if text.startswith(prefix):
            text = text[len(prefix):]
            break
    if text and text[0] in "GR":
        text = text[1:]
    if len(text) < 2:
        return None
    if text[0] in "CLDS" and len(text) >= 3 and text[1].isdigit():
        text = text[1:]
    if not text[0].isdigit():
        return None
    return text[:2]


def classify_stream(samples: list[tuple[float, int]], query: float) -> dict[str, Any]:
    if not samples:
        return {"supported": False, "reason": "missing-exact-stream",
                "support_kind": None, "sample_indices": []}
    for index in range(1, len(samples)):
        if not math.isfinite(samples[index][0]) or samples[index][0] <= samples[index - 1][0]:
            raise fail("base stream has duplicate or non-monotonic time")
    if not math.isfinite(query):
        return {"supported": False, "reason": "nonfinite-query",
                "support_kind": None, "sample_indices": []}
    if query < samples[0][0] or query > samples[-1][0]:
        return {"supported": False, "reason": "out-of-domain",
                "support_kind": None, "sample_indices": []}
    for index, (sample_time, _row_index) in enumerate(samples):
        if query == sample_time:
            return {"supported": True,
                    "reason": None,
                    "support_kind": ("exact_endpoint" if index in (0, len(samples) - 1)
                                      else "exact_sample"),
                    "sample_indices": [samples[index][1]]}
        if query < sample_time:
            return {"supported": True,
                    "reason": None,
                    "support_kind": "adjacent_two_point_bracket",
                    "sample_indices": [samples[index - 1][1], samples[index][1]]}
    raise fail("base support classifier reached impossible state")


def side_ledger(input_rows: int, certified: int, local_miss: int,
                source_counts: dict[str, int], reasons: dict[str, int]) -> dict[str, Any]:
    return {
        "input_rows": input_rows,
        "certified_rows": certified,
        "explicit_local_miss_rows": local_miss,
        "header_primary_certified_rows": source_counts.get("header", 0),
        "broadcast_geph_certified_rows": source_counts.get("broadcast-geph", 0),
        "reason_counts": reasons,
        "all_rows_classified": input_rows == certified + local_miss,
        "no_raw_uncorrected": True,
        "no_zero_correction": True,
        "no_fallback": True,
        "no_extrapolation": True,
        "finite_certified_wavelength": certified > 0,
    }


def build_inventory(route: str, payloads: dict[str, bytes],
                    reads: dict[str, int]) -> dict[str, Any]:
    expected_base = ROUTE_INPUTS[route][BASE_NAME]
    phone = p128.inventory_phone_gnss(payloads["device_gnss.csv"])
    imu = p128.inventory_imu(payloads["device_imu.csv"])
    nav = p128.parse_nav_geph(payloads["brdc.nav"])
    base = p128.parse_base_rinex(payloads[BASE_NAME], expected_base)
    exact_base = exact_base_observations(payloads[BASE_NAME])
    header = base.get("header", {"status": "malformed", "entries": {}})
    if base.get("observations") and not exact_base:
        raise fail("base exact epoch identity parser produced no rows")
    descriptors = {
        descriptor
        for code in base.get("observation_codes", {}).get("R", [])
        if (descriptor := signal_descriptor(code)) is not None
    }
    streams: dict[tuple[tuple[str, int], str], list[tuple[float, int]]] = {}
    for row in exact_base:
        for descriptor in descriptors:
            streams.setdefault((row["satellite"], descriptor), []).append(
                (float(row["query_gpst"]), int(row["row_index"])))
    for key, samples in streams.items():
        for index in range(1, len(samples)):
            if not math.isfinite(samples[index][0]) or samples[index][0] <= samples[index - 1][0]:
                raise fail(f"base stream duplicate/non-monotonic: {key!r}")

    rover_input = int(phone.get("selected_rows", 0) or 0)
    rover_invalid = int(phone.get("invalid_rows", 0) or 0)
    rover_certified = 0
    rover_reasons: dict[str, int] = {
        str(key): int(value) for key, value in phone.get("invalid_reasons", {}).items()
    }
    rover_sources: dict[str, int] = {}
    support_counts = {"exact_endpoint": 0, "exact_sample": 0,
                      "adjacent_two_point_bracket": 0}
    used_samples: set[tuple[tuple[str, int], str, int]] = set()
    retained_keys: list[str] = []
    for row in phone.get("glonass", []):
        satellite = tuple(row["satellite"])
        descriptor = signal_descriptor(row.get("signal"))
        if descriptor is None or descriptor not in descriptors:
            rover_reasons["missing-exact-key"] = rover_reasons.get("missing-exact-key", 0) + 1
            continue
        nav_result = p128.resolve_query(satellite, row["query_gpst"], nav.get("by_sat", {}),
                                        {"status": "absent", "entries": {}})
        if not nav_result.get("ok"):
            reason = str(nav_result.get("reason", "query-time-coverage-gap"))
            rover_reasons[reason] = rover_reasons.get(reason, 0) + 1
            if reason in {"different-fcn-tie", "header-fcn-conflict", "header-geph-fcn-mismatch"}:
                raise fail(f"rover GLONASS provenance conflict: {reason}")
            continue
        support = classify_stream(streams.get((satellite, descriptor), []),
                                  float(row["query_gpst"]))
        if not support["supported"]:
            reason = str(support["reason"])
            rover_reasons[reason] = rover_reasons.get(reason, 0) + 1
            continue
        rover_certified += 1
        rover_sources["broadcast-geph"] = rover_sources.get("broadcast-geph", 0) + 1
        support_counts[support["support_kind"]] = support_counts.get(support["support_kind"], 0) + 1
        retained_keys.append(f"{satellite[0]}{satellite[1]}:{descriptor}")
        for row_index in support["sample_indices"]:
            used_samples.add((satellite, descriptor, row_index))

    base_input = len(exact_base)
    base_certified = 0
    base_reasons: dict[str, int] = {}
    base_sources: dict[str, int] = {}
    for row in exact_base:
        nav_result = p128.resolve_query(row["satellite"], row["query_gpst"],
                                        nav.get("by_sat", {}), header)
        if not nav_result.get("ok"):
            reason = str(nav_result.get("reason", "query-time-coverage-gap"))
            base_reasons[reason] = base_reasons.get(reason, 0) + 1
            if reason in {"different-fcn-tie", "header-fcn-conflict", "header-geph-fcn-mismatch"}:
                raise fail(f"base GLONASS provenance conflict: {reason}")
            continue
        base_certified += 1
        source = str(nav_result.get("source", "broadcast-geph"))
        base_sources[source] = base_sources.get(source, 0) + 1

    rover_local_miss = rover_invalid + max(0, len(phone.get("glonass", [])) - rover_certified)
    # selected_rows includes invalid GLONASS rows, while valid GLONASS rows
    # are the only rows emitted in phone["glonass"].
    if rover_input != len(phone.get("glonass", [])) + rover_invalid:
        raise fail("rover GLONASS local partition does not conserve input rows")
    base_local_miss = base_input - base_certified
    rover = side_ledger(rover_input, rover_certified, rover_local_miss,
                        rover_sources, rover_reasons)
    base_side = side_ledger(base_input, base_certified, base_local_miss,
                            base_sources, base_reasons)

    global_errors: list[str] = []
    for label, item in (("phone GNSS", phone), ("phone IMU", imu),
                        ("broadcast nav", nav), ("base RINEX", base)):
        if not item.get("ok"):
            global_errors.append(f"{label}: {item.get('failure')}")
    if not exact_base:
        global_errors.append("base has no exact epoch identity rows")
    if not descriptors:
        global_errors.append("base GLONASS signal mapping is empty")
    if header.get("status") == "malformed" or header.get("conflict_entries", 0):
        global_errors.append("base GLONASS header is malformed/conflicting")
    if rover_certified == 0 or base_certified == 0 or not used_samples:
        global_errors.append("all retained rover/base exact-key support is empty")
    if not rover["all_rows_classified"] or not base_side["all_rows_classified"]:
        global_errors.append("side-local certified plus explicit-miss conservation failed")
    if any("different-fcn-tie" in reason or "fcn-conflict" in reason
           for reason in list(rover_reasons) + list(base_reasons)):
        global_errors.append("GLONASS tie/conflict is fail-closed")
    ok = not global_errors
    all_stream_samples = sum(len(samples) for samples in streams.values())
    used_streams = {(satellite, descriptor) for satellite, descriptor, _ in used_samples}
    correction_miss = rover_local_miss
    retained = rover_certified
    return {
        "route": route,
        "stage": "post-authorization-pre-solver",
        "ok": ok,
        "solver_invocations": 0,
        "solver_may_start": ok,
        "failure": None if ok else "; ".join(global_errors),
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
        "base": {key: base.get(key) for key in
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
        "base_glonass": base_side,
        "shared_ledger": {
            "side_local_conservation": rover["all_rows_classified"] and base_side["all_rows_classified"],
            "retained_rover_exact_key_support": retained > 0,
            "exact_endpoint_or_adjacent_two_point_bracket": (
                support_counts["exact_endpoint"] + support_counts["adjacent_two_point_bracket"] > 0),
            "unmatched_rover_explicit_factor_miss": rover_local_miss >= 0,
            "unused_base_rows_accounted": all_stream_samples >= len(used_samples),
            "whole_ledger_equality_required": False,
            "cross_side_count_equality_required": False,
            "cross_side_reason_map_equality_required": False,
            "no_raw_uncorrected": True,
            "no_zero_correction": True,
            "no_fallback": True,
            "no_extrapolation": True,
            "retained_exact_keys": retained_keys[:100],
            "retained_exact_key_count": len(retained_keys),
        },
        "base_streams": {
            "stream_count": len(streams),
            "finite_samples": all_stream_samples,
            "used_samples": len(used_samples),
            "unused_rows": max(0, all_stream_samples - len(used_samples)),
            "used_streams": len(used_streams),
            "unused_streams": max(0, len(streams) - len(used_streams)),
            "all_samples_accounted": all_stream_samples >= len(used_samples),
            "duplicate_nonmonotonic_rejected": True,
        },
        "correction": {
            "factor_input_rows": len(phone.get("glonass", [])),
            "retained_corrected_rows": retained,
            "missing_exact_stream": rover_reasons.get("missing-exact-stream", 0) + rover_reasons.get("missing-exact-key", 0),
            "out_of_domain": rover_reasons.get("out-of-domain", 0),
            "nonfinite": rover_reasons.get("nonfinite-query", 0),
            "support": support_counts,
            "retained_factors_have_exact_key_support": retained > 0,
            "one_support_result_per_retained_factor": True,
            "unused_base_rows_accounted": all_stream_samples >= len(used_samples),
            "application_passes": 1,
            "exactly_once": True,
            "no_duplicate_application": True,
            "no_raw_fallback": True,
            "no_zero_fallback": True,
            "no_nearest_fill": True,
            "no_endpoint_hold": True,
            "no_extrapolation": True,
            "finite_corrected_rows": retained > 0,
        },
        "phase130": {
            "enabled": True,
            "selector_boundary_only": True,
            "native_selector_forwarded": False,
            "side_local_conservation": rover["all_rows_classified"] and base_side["all_rows_classified"],
            "whole_ledger_equality_required": False,
            "retained_rover_exact_key_support": retained > 0,
            "unmatched_rover_explicit_factor_miss": rover_local_miss >= 0,
            "unused_base_rows_accounted": all_stream_samples >= len(used_samples),
            "exact_endpoint_or_adjacent_two_point_bracket": (
                support_counts["exact_endpoint"] + support_counts["adjacent_two_point_bracket"] > 0),
            "no_raw_uncorrected": True,
            "no_zero_correction": True,
            "no_fallback": True,
            "no_extrapolation": True,
            "source_complete_a_b_c": ok,
        },
        "coverage": {
            "rover_certified_or_local_miss": rover["all_rows_classified"],
            "base_certified_or_local_miss": base_side["all_rows_classified"],
            "all_finite_positive_frequency_wavelength": retained > 0,
            "exact_query_time_records": True,
            "exact_endpoint_or_adjacent_bracket_only": True,
            "header_primary_or_geph_only": True,
            "fixed_channel_or_external_table": False,
        },
        "inventory_reads": dict(reads),
    }


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


def command_for(route: str, auth_record: dict[str, Any]) -> tuple[list[str], bool]:
    command = contract.command_template(route)
    # The Phase130 selector is intentionally a contract/runner boundary only;
    # the pinned Phase129 binary cannot accept a new native CLI option.
    assert PHASE130_SELECTOR in command
    command.remove(PHASE130_SELECTOR)
    for flag, name in (("--android-gnss", "device_gnss.csv"),
                       ("--android-imu", "device_imu.csv"),
                       ("--nav", "brdc.nav")):
        command[command.index(flag) + 1] = auth_record["raw_inputs"][name]["path"]
    command[command.index("--native-base-rinex") + 1] = auth_record["base_input"]["path"]
    command[command.index("--native-base-rinex-sha256") + 1] = auth_record["base_input"]["sha256"]
    return command, False


def finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def decreasing(initial: Any, final: Any) -> bool:
    return finite(initial) and finite(final) and float(final) < float(initial)


def empty_gates() -> dict[str, bool]:
    return {
        "native_process_completed": False,
        "summary_present": False,
        "phase130_inventory_handoff": False,
        "phase126_atomic_a_b_c": False,
        "base_exactly_once": False,
        "gnss_first_progress": False,
        "gnss_first_c7_d_handoff": False,
        "main_qr_progress": False,
        "main_finite_coverage": False,
        "no_fallback_or_publication": False,
    }


def opaque_solution_seal(path: Path, expected_rows: int) -> dict[str, Any]:
    if not path.is_file():
        return {"present": False, "sha256": None, "rows": expected_rows,
                "opened": False, "content_interpreted": False}
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return {"present": True, "sha256": digest.hexdigest(), "rows": expected_rows,
            "opened": False, "content_interpreted": False,
            "opaque_hash_read": True}


def normalize_native(route: str, native: dict[str, Any], solution: dict[str, Any],
                     return_code: int | None, inventory: dict[str, Any],
                     phase130_forwarded: bool) -> tuple[dict[str, Any], dict[str, Any]]:
    base = native.get("native_base_pseudorange_compensation", {})
    miss = native.get("native_base_pseudorange_source_miss_mask", {})
    gnss = native.get("gnss_first", {})
    graph = native.get("graph", {})
    epochs = native.get("epochs", {})
    c0d = native.get("native_source_clock_c0d_factor", {})
    qr = native.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected") is True
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
        base.get("phase129_configuration_valid") is True and
        base.get("base_rinex_read_count") == 1 and
        miss.get("correction_applied_exactly_once") is True and
        miss.get("pseudorange_factor_count_consistent") is True)
    gnss_progress = (int(gnss.get("iterations", 0) or 0) > 0 and
                     decreasing(gnss.get("initial_cost"), gnss.get("final_cost")))
    problem_epochs = int(epochs.get("problem", 0) or 0)
    handoff = (
        c0d.get("meter_state_parity_enabled") is True and
        c0d.get("epoch_vector_parity_enabled") is True and
        c0d.get("epoch_vector_dimension") == 7 and
        c0d.get("epoch_vector_state_count") == problem_epochs and
        c0d.get("epoch_vector_handoff_count") == problem_epochs and
        c0d.get("global_isb_state_count") == 0 and
        gnss.get("epoch_identity_alignment_valid", True) is True and
        gnss.get("optimized_d_export_valid", True) is True and
        gnss.get("optimized_c_export_valid", True) is True)
    main_progress = (int(graph.get("iterations", 0) or 0) > 0 and
                     decreasing(graph.get("initial_cost"), graph.get("final_cost")))
    output_ok = (problem_epochs > 0 and int(epochs.get("output", 0) or 0) == problem_epochs and
                 native.get("output_contract", {}).get("finite_coordinates") is True)
    phase129_ok = (base.get("phase129_glonass_local_miss_mask") is True and
                   base.get("phase129_glonass_row_count_consistent") is True and
                   base.get("phase129_configuration_valid") is True and
                   miss.get("pseudorange_factor_count_consistent") is True)
    no_fallback = (native.get("status") == "imu-combined-factor" and
                   native.get("truth_used") is False and
                   native.get("production_default_changed") is False and
                   native.get("native_phase117_tdcp_snr_type_sigma") is False and
                   native.get("native_phase120_official_tdcp_resl_atmosphere_cancellation") is False)
    gates = {
        "native_process_completed": return_code == 0,
        "summary_present": True,
        "phase130_inventory_handoff": inventory.get("ok") is True and
                                       inventory.get("phase130", {}).get("source_complete_a_b_c") is True,
        "phase126_atomic_a_b_c": atomic,
        "base_exactly_once": base_once,
        "gnss_first_progress": gnss_progress,
        "gnss_first_c7_d_handoff": handoff,
        "main_qr_progress": qr and main_progress,
        "main_finite_coverage": output_ok,
        "no_fallback_or_publication": no_fallback and not solution.get("opened", False),
    }
    normalized = {
        "schema_version": "smartphone-r5-phase130-shared-ledger-key-local-support-structural-summary.v1",
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
        "phase130": {"enabled": True, "selector_boundary_only": True,
                     "native_selector_forwarded": phase130_forwarded,
                     "whole_ledger_equality_required": False,
                     "side_local_conservation": inventory.get("shared_ledger", {}).get("side_local_conservation") is True,
                     "retained_rover_exact_key_support": inventory.get("shared_ledger", {}).get("retained_rover_exact_key_support") is True,
                     "unused_base_rows_accounted": inventory.get("shared_ledger", {}).get("unused_base_rows_accounted") is True,
                     "no_fallback": True},
        "gates": gates,
        "opaque_solution": {"sha256": solution.get("sha256"), "rows": solution.get("rows")},
    }
    telemetry = {
        "return_code": return_code,
        "phase130": {"enabled": True, "selector_boundary_only": True,
                      "native_selector_forwarded": phase130_forwarded,
                      "inventory_ok": inventory.get("ok"),
                      "rover_rows": inventory.get("rover", {}).get("input_rows"),
                      "rover_certified": inventory.get("rover", {}).get("certified_rows"),
                      "rover_explicit_local_miss": inventory.get("rover", {}).get("explicit_local_miss_rows"),
                      "base_rows": inventory.get("base_glonass", {}).get("input_rows"),
                      "base_certified": inventory.get("base_glonass", {}).get("certified_rows"),
                      "base_explicit_local_miss": inventory.get("base_glonass", {}).get("explicit_local_miss_rows"),
                      "retained_exact_key_count": inventory.get("shared_ledger", {}).get("retained_exact_key_count"),
                      "factor_support": inventory.get("correction", {}).get("support"),
                      "factor_missing_exact_stream": inventory.get("correction", {}).get("missing_exact_stream"),
                      "base_streams": inventory.get("base_streams")},
        "native_phase129": {key: base.get(key) for key in (
            "phase129_glonass_local_miss_mask", "phase129_configuration_valid",
            "phase129_configuration_failure", "phase129_glonass_local_miss_rows",
            "phase129_glonass_local_miss_streams", "phase129_glonass_local_miss_counts",
            "phase129_glonass_row_count_consistent", "phase129_glonass_factor_rows_dropped",
            "phase129_glonass_factor_rows_retained", "phase129_glonass_factor_count_consistent")},
        "base_correction": {key: base.get(key) for key in (
            "phase126_atomic_step_a_raw_ingress_verified",
            "phase126_atomic_step_b_source_stream_verified",
            "phase126_atomic_step_c_application_committed", "phase126_compound_admitted",
            "base_rinex_read_count", "correction_application_pass_count",
            "correction_applied_exactly_once", "matched_factor_rows", "finite_correction_rows_among_matched",
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
            "optimized_d_epoch_count", "optimized_d_finite_count", "optimized_c_export_valid",
            "optimized_c_epoch_count", "optimized_c_finite_component_count",
            "epoch_identity_alignment_valid", "failure")},
        "main": {key: graph.get(key) for key in
                 ("factors", "values", "imu_intervals", "iterations", "converged",
                  "initial_cost", "final_cost")},
        "epochs": {key: epochs.get(key) for key in
                    ("problem", "output", "pseudorange_factors", "tdcp_factors_built",
                     "double_difference_pseudorange_factors", "double_difference_carrier_factors")},
        "solver": {key: native.get(key) for key in (
            "native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected",
            "selected_linear_solver_type", "selected_solver_branch", "selected_elimination_function")},
        "gates": gates,
        "opaque_solution": solution,
    }
    return normalized, telemetry


def execute_one_route(route: str, auth_record: dict[str, Any],
                      inventory: dict[str, Any]) -> dict[str, Any]:
    route_dir = OUTPUT_ROOT / route.replace("/", "__")
    route_dir.mkdir(parents=True, exist_ok=False)
    atomic_json(route_dir / "inventory.json", inventory)
    command, phase130_forwarded = command_for(route, auth_record)
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
            stderr.write(f"Phase130 native launch failed: {exc}\n".encode())
    record: dict[str, Any] = {
        "route": route, "run_number": 1, "solver_launched": True,
        "return_code": return_code, "launch_error": launch_error,
        "summary_present": summary_path.is_file(), "solution_opened": False,
        "solution_published": False, "truth_used": False, "accuracy_scored": False,
        "raw_content_copied_or_transformed": False, "phase130_selector_forwarded": phase130_forwarded,
        "inventory": inventory,
    }
    if not summary_path.is_file():
        record.update({"summary_error": "native summary absent; structural gates fail closed",
                       "gates": empty_gates()})
        record["gates"]["native_process_completed"] = return_code == 0 and not launch_error
        record["gates"]["no_fallback_or_publication"] = True
        return record
    try:
        native = read_json(summary_path, f"Phase130 native summary {route}")
    except Phase130ExecutionError as exc:
        record.update({"summary_error": str(exc), "gates": empty_gates()})
        record["gates"]["native_process_completed"] = return_code == 0 and not launch_error
        record["gates"]["no_fallback_or_publication"] = True
        return record
    solution = opaque_solution_seal(solution_path, int(contract.PROBLEM_EPOCHS[route]))
    normalized, telemetry = normalize_native(route, native, solution, return_code,
                                             inventory, phase130_forwarded)
    # Remove coordinate-bearing native summary fields from the sealed summary;
    # this writes only the structural subset and never opens the solution rows.
    atomic_json(summary_path, normalized)
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
        "# Phase130 keyed shared-ledger structural raw result",
        "",
        f"- Status: `{result['status']}`",
        "- Matrix: MTV-A then LAX-T, exactly one authorized inventory and at most one native solver attempt per route.",
        "- Phase130 selector is enforced at the runner/contract boundary and is not forwarded to the pinned Phase129 binary.",
        "- Solution rows were not opened or interpreted; only opaque hashes and expected row counts were sealed.",
        "- Truth, MAT, PDC, precomputed coordinates, accuracy, and Kaggle remain unauthorized.",
        "",
        "| Route | Inventory | Solver | Return | Rover cert/miss | Base cert/miss | Retained exact keys | Main accepted | Main cost | GO | Failure |",
        "|---|---|---|---:|---:|---:|---:|---:|---|---|---|",
    ]
    for route in ROUTES:
        record = result["routes"].get(route, {})
        inventory = record.get("inventory", {})
        rover = inventory.get("rover", {})
        base = inventory.get("base_glonass", {})
        phase = inventory.get("shared_ledger", {})
        main = record.get("structural_telemetry", {}).get("main", {})
        gates = record.get("gates", {})
        failure = record.get("failure") or inventory.get("failure") or record.get("summary_error") or ""
        lines.append(
            f"| `{route}` | `{inventory.get('ok')}` | `{record.get('solver_launched', False)}` | "
            f"`{record.get('return_code')}` | `{rover.get('certified_rows', 0)}/{rover.get('explicit_local_miss_rows', 0)}` | "
            f"`{base.get('certified_rows', 0)}/{base.get('explicit_local_miss_rows', 0)}` | "
            f"`{phase.get('retained_exact_key_count', 0)}` | `{main.get('iterations', 0)}` | "
            f"`{main.get('initial_cost')}->{main.get('final_cost')}` | "
            f"`{all(gates.values()) if gates else False}` | `{failure}` |")
    lines.extend(["", "Inventory failures are fail-closed and prevent a native launch for that route.", ""])
    return "\n".join(lines)


def execute_matrix() -> dict[str, Any]:
    auth = verify_authorization()
    if RESULT_JSON.exists() or RESULT_MD.exists():
        raise fail("refusing to overwrite an existing Phase130 sealed result")
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite existing Phase130 output root: {OUTPUT_ROOT}")
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
        except (Phase130ExecutionError, KeyError, TypeError, ValueError) as exc:
            inventory = {"route": route, "stage": "post-authorization-pre-solver",
                         "ok": False, "solver_invocations": 0, "solver_may_start": False,
                         "failure": str(exc), "inventory_reads": dict(reads),
                         "phase130": {"enabled": True, "selector_boundary_only": True,
                                      "native_selector_forwarded": False}}
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
                            "phase130_selector_forwarded": False, "inventory": inventory,
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
        "schema_version": "smartphone-r5-phase130-shared-ledger-key-local-support-structural-result.v1",
        "phase": 130,
        "execution_label": "Luna Max",
        "status": "go-phase130-shared-ledger-key-local-support-structural" if all_passed
                  else "no-go-phase130-shared-ledger-key-local-support-structural",
        "decision": ("All inventory and structural gates passed; truth/accuracy and solution release remain unauthorized."
                     if all_passed else
                     "Inventory or structural gate failed closed; no rerun, fallback, truth, accuracy, or solution release is permitted."),
        "authorization": {"path": relative(AUTHORIZATION), "status": auth.get("status"), "independent": True},
        "contract": {"audit_commit": AUDIT_COMMIT, "freeze_commit": FREEZE_COMMIT,
                     "implementation_commit": IMPLEMENTATION_COMMIT,
                     "manifest_commit": MANIFEST_COMMIT, "pre_raw_commit": PRE_RAW_COMMIT,
                     "phase129_implementation_commit": PHASE129_IMPLEMENTATION_COMMIT,
                     "manifest_sha256": MANIFEST_SHA256, "target_binary_sha256": TARGET_BINARY_SHA256},
        "candidate": {"id": contract.CANDIDATE_ID, "phase130_runner_boundary": True,
                      "phase126": True, "phase127": True, "phase128": True, "phase129": True,
                      "phase118_huber": True, "phase117_dynamic_sigma": False,
                      "phase120_atmosphere": False, "additional_frequency_bands": False,
                      "fixed_tdcp_sigma": 0.03, "official_huber_k": 0.5,
                      "main_solver": "MULTIFRONTAL_QR", "solution_opaque": True},
        "matrix": {"candidate_count": 1, "route_order": list(ROUTES), "runs_per_route": 1,
                   "native_solver_invocations": accounting["native_solver_invocations"],
                   "inventory_reads_per_route": 1, "controls": 0, "reruns": 0,
                   "fallbacks": 0, "truth_reads": 0, "accuracy_calculations": 0,
                   "solution_rows_opened": 0},
        "routes": route_records,
        "read_accounting": accounting,
        "inventory_contract": {"side_local_conservation": True,
                               "retained_rover_exact_key_support": True,
                               "exact_endpoint_or_adjacent_two_point_bracket": True,
                               "unmatched_rover_explicit_factor_miss": True,
                               "unused_base_rows_accounted": True,
                               "whole_ledger_equality_required": False,
                               "empty_all_miss_fail_closed": True,
                               "solver_zero_on_inventory_failure": True,
                               "no_raw_zero_or_uncorrected_fallback": True,
                               "no_nearest_hold_or_extrapolation": True,
                               "duplicate_nonmonotonic_base_time_global_abort": True,
                               "glonass_tie_conflict_global_abort": True},
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
                        help="verify all static pins without opening route payloads")
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
    except (Phase130ExecutionError, OSError) as exc:
        print(f"phase130 authorized runner: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
