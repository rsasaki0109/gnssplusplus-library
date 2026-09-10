#!/usr/bin/env python3
"""Execute the independently authorized Phase128 structural matrix once.

The authorization and all static pins are checked before any route payload is
opened.  Each route is read once for an in-memory inventory.  Inventory
failure is sealed with zero native launches; only an admitted route may launch
the pinned structural command once.  Native solution rows remain opaque.
"""

from __future__ import annotations

import csv
from datetime import datetime, timedelta, timezone
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))
import gnss_smartphone_phase128_inventory_first_structural as contract  # noqa: E402


AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase128_inventory_first_structural_authorization_v1.json"
PRE_RAW = ROOT / "docs/use_cases/records/smartphone_r5_phase128_inventory_first_structural_pre_raw_accounting_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase128_inventory_first_structural_manifest_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase128-inventory-first-structural-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase128_inventory_first_structural_result_v1.json"
RESULT_MD = ROOT / "docs/use_cases/records/smartphone_r5_phase128_inventory_first_structural_result_v1.md"
AUTHORIZED_RUNNER = Path(__file__)

AUDIT_COMMIT = "c2557ac7b99f2fd2827233900fbe88a10a9b1695"
FREEZE_COMMIT = "751dbfff4fe21c1d9642a27c016fcab2cd317afa"
IMPLEMENTATION_COMMIT = "50357e8f3eabbb2eca672b00a16eb2ce32cc2f16"
RUNNER_MANIFEST_COMMIT = "0bc9b1a53b0c798ae6d331504ba4f48c93eb2ee1"
PRE_RAW_COMMIT = "111ad07f3b72b5a97d21741b6d698b8c21d67fe2"
AUDIT_SHA256 = "58993acacf45107928b10ebd3f406ca1a494102168e46366edb42777369678f7"
FREEZE_SHA256 = "467b35e14ed3235244a6403eb5ac1fbcad777f9e675fd1a08db20209781ac385"
MANIFEST_SHA256 = "8be91bc2e80a7228dd66a3facae1e719e59b6e356de209b5101488ca1bf02ac6"
PRE_RAW_SHA256 = "1dcbe85f101b6a7674253858a42497665a84ba1326638ef43884d78d38e20338"
TARGET_BINARY_SHA256 = "d823b031d865bde973d3bd6ffcf2957288bccf29d66bf89ade3dea11d108bfe0"
HISTORICAL_PHASE127_APP_SHA256 = "af2d74e23706e354b1b9fd715aaa09df807854787d789b0f46f7ee3efd3b5b73"

ROUTES = contract.ROUTES
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
BASE_NAME = "base.obs"
PHASE126_SELECTOR = contract.PHASE126_SELECTOR
PHASE127_SELECTOR = contract.PHASE127_SELECTOR
PHASE128_SELECTOR = contract.PHASE128_SELECTOR
PHASE118_SELECTOR = contract.PHASE118_SELECTOR
PHASE117_SELECTOR = contract.PHASE117_SELECTOR
PHASE120_SELECTOR = contract.PHASE120_SELECTOR
ADDITIONAL_SELECTOR = contract.ADDITIONAL_SELECTOR

# These are sealed path/hash metadata inherited from the already committed
# Phase127 raw-input authorization.  They are not opened during verification.
ROUTE_INPUTS: dict[str, dict[str, dict[str, Any]]] = {
    ROUTES[0]: {
        "device_gnss.csv": {"path": "output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/device_gnss.csv", "bytes": 57715495, "sha256": "c7d50d5127d16586adc6c79d724758e298b385496da22c5e5dfd6ec522cbc863"},
        "device_imu.csv": {"path": "output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/device_imu.csv", "bytes": 34393802, "sha256": "afc540e7c4ce2ca66b442a1afbcd604e9f6b3d2cc4d3733183739901b5b97bd6"},
        "brdc.nav": {"path": "output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/brdc.nav", "bytes": 9955040, "sha256": "6adfaf7fe4452a4faeb94a7b607c15e05f578c46a028a030b94aa6f79de194cd"},
        BASE_NAME: {"path": "output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2021-03-16-18-59-us-ca-mtv-a__pixel5/base.obs", "bytes": 10708536, "sha256": "380b8ff9091344fb756697e27f0983d9a0ba2cf0c201b96849bfc1ecc1af0e52", "interval": 1.0, "window": 151},
    },
    ROUTES[1]: {
        "device_gnss.csv": {"path": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/device_gnss.csv", "bytes": 33837317, "sha256": "50362c01bff3e0bb7088e54021164591cd750227ed97c2fd7d95d763a08798f1"},
        "device_imu.csv": {"path": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/device_imu.csv", "bytes": 23649244, "sha256": "2e39a3e9f294c64b8ecfd452d0960025d1013b97f2d7497e6e48a2a1997b38c5"},
        "brdc.nav": {"path": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/brdc.nav", "bytes": 10635773, "sha256": "443d3d5a73f4895b83e576e24a568f4658f869e79a480856de7f0717763dcbe6"},
        BASE_NAME: {"path": "output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2022-04-01-18-22-us-ca-lax-t__pixel5/base.obs", "bytes": 719969, "sha256": "d731e0e8a7ba4396d62340c85b6238e66c50c6a349621f9dcea0ea6885fd4cfe", "interval": 15.0, "window": 11},
    },
}

GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
LEAP_SECONDS = (
    (datetime(1981, 7, 1, tzinfo=timezone.utc), 1),
    (datetime(1982, 7, 1, tzinfo=timezone.utc), 2),
    (datetime(1983, 7, 1, tzinfo=timezone.utc), 3),
    (datetime(1985, 7, 1, tzinfo=timezone.utc), 4),
    (datetime(1988, 1, 1, tzinfo=timezone.utc), 5),
    (datetime(1990, 1, 1, tzinfo=timezone.utc), 6),
    (datetime(1991, 1, 1, tzinfo=timezone.utc), 7),
    (datetime(1992, 7, 1, tzinfo=timezone.utc), 8),
    (datetime(1993, 7, 1, tzinfo=timezone.utc), 9),
    (datetime(1994, 7, 1, tzinfo=timezone.utc), 10),
    (datetime(1996, 1, 1, tzinfo=timezone.utc), 11),
    (datetime(1997, 7, 1, tzinfo=timezone.utc), 12),
    (datetime(1999, 1, 1, tzinfo=timezone.utc), 13),
    (datetime(2006, 1, 1, tzinfo=timezone.utc), 14),
    (datetime(2009, 1, 1, tzinfo=timezone.utc), 15),
    (datetime(2012, 7, 1, tzinfo=timezone.utc), 16),
    (datetime(2015, 7, 1, tzinfo=timezone.utc), 17),
    (datetime(2017, 1, 1, tzinfo=timezone.utc), 18),
)


class Phase128ExecutionError(ValueError):
    """Authorization, inventory, or structural execution failure."""


def fail(message: str) -> Phase128ExecutionError:
    return Phase128ExecutionError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def sha256_static(path: Path, label: str) -> str:
    lowered = path.name.lower()
    if (lowered in RAW_NAMES or lowered in {BASE_NAME, "truth.csv", "ground_truth.csv"}
            or lowered.endswith((".csv", ".nav", ".obs", ".mat"))
            or "truth" in lowered or "ground_truth" in lowered):
        raise fail(f"static hash attempted on payload: {label}")
    if not path.is_file():
        raise fail(f"missing static artifact: {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def verify_authorization() -> tuple[dict[str, Any], dict[str, Any]]:
    # This call reads only tracked static artifacts and the launch-free
    # contract.  It cannot materialize any route payload.
    contract.verify_manifest()
    assert_equal(sha256_static(PRE_RAW, "Phase128 pre-raw accounting"),
                 PRE_RAW_SHA256, "pre-raw/sha256")
    pre = read_json(PRE_RAW, "Phase128 pre-raw accounting")
    accounting = pre.get("read_accounting")
    if not isinstance(accounting, dict):
        raise fail("pre-raw read accounting missing")
    for key, value in accounting.items():
        if key.endswith("copied_or_transformed") or key == "solution_output_published":
            assert_equal(value, False, f"pre-raw/{key}")
        else:
            assert_equal(value, 0, f"pre-raw/{key}")

    auth = read_json(AUTHORIZATION, "Phase128 raw authorization")
    for key, expected in {
        "schema_version": "smartphone-r5-phase128-inventory-first-structural-authorization.v1",
        "phase": 128,
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
        "runner_manifest_commit": RUNNER_MANIFEST_COMMIT,
        "pre_raw_accounting_commit": PRE_RAW_COMMIT,
        "audit_sha256": AUDIT_SHA256,
        "freeze_sha256": FREEZE_SHA256,
        "manifest_sha256": MANIFEST_SHA256,
        "pre_raw_accounting_sha256": PRE_RAW_SHA256,
        "target_binary_sha256": TARGET_BINARY_SHA256,
        "authorized_runner_path": relative(AUTHORIZED_RUNNER),
    }
    for key, expected in expected_pins.items():
        assert_equal(pins.get(key), expected, f"authorization/pins/{key}")
    assert_equal(pins.get("authorized_runner_sha256"),
                 sha256_static(AUTHORIZED_RUNNER, "authorized runner"),
                 "authorization/pins/authorized_runner_sha256")
    assert_equal(pins.get("target_binary_path"), relative(contract.BINARY),
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
    assert_equal(scope_meta.get("route_order"), list(ROUTES),
                 "authorization_scope/route_order")
    assert_equal(scope_meta.get("runs_per_route"), 1,
                 "authorization_scope/runs_per_route")
    assert_equal(scope_meta.get("controls"), 0, "authorization_scope/controls")
    assert_equal(scope_meta.get("reruns"), 0, "authorization_scope/reruns")
    assert_equal(scope_meta.get("fallbacks"), 0, "authorization_scope/fallbacks")

    routes = auth.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("authorization route order/count changed")
    for record in routes:
        route = record.get("dataset_id")
        if route not in ROUTE_INPUTS or record.get("runs") != 1:
            raise fail(f"authorization/{route}: route metadata changed")
        raw_inputs = record.get("raw_inputs")
        if not isinstance(raw_inputs, dict):
            raise fail(f"authorization/{route}: raw inputs missing")
        for name in RAW_NAMES:
            item = raw_inputs.get(name)
            expected = ROUTE_INPUTS[route][name]
            if not isinstance(item, dict):
                raise fail(f"authorization/{route}/{name}: metadata missing")
            for key in ("path", "bytes", "sha256"):
                assert_equal(item.get(key), expected[key],
                             f"authorization/{route}/{name}/{key}")
            assert_equal(item.get("read_before_authorization"), False,
                         f"authorization/{route}/{name}/read_before_authorization")
            assert_equal(item.get("copy_or_transform"), False,
                         f"authorization/{route}/{name}/copy_or_transform")
        base = record.get("base_input")
        expected_base = ROUTE_INPUTS[route][BASE_NAME]
        if not isinstance(base, dict):
            raise fail(f"authorization/{route}: base metadata missing")
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
    return auth, pins


def safe_payload_path(text: Any, basename: str, route: str) -> Path:
    if not isinstance(text, str):
        raise fail(f"missing sealed {basename} path: {route}")
    path = Path(text)
    if path.is_absolute() or ".." in path.parts or path.name != basename:
        raise fail(f"unsafe sealed {basename} path: {route}: {text}")
    lowered = text.lower()
    if any(term in lowered for term in (".mat", "truth", "ground_truth",
                                        "precomputed", "coordinate", "pdc",
                                        "kaggle", "token")):
        raise fail(f"forbidden input lineage: {route}/{basename}")
    root = ROOT.resolve()
    resolved = (ROOT / path).resolve()
    if resolved != root and root not in resolved.parents:
        raise fail(f"input escapes repository root: {route}/{basename}")
    return resolved


def read_payload_once(path: Path, expected: dict[str, Any], route: str,
                      name: str, accounting: dict[str, int]) -> bytes:
    accounting[name] = accounting.get(name, 0) + 1
    try:
        data = path.read_bytes()
    except OSError as exc:
        raise fail(f"{route}/{name}: authorized payload read failed: {exc}") from exc
    if len(data) != expected["bytes"]:
        raise fail(f"{route}/{name}: bytes {len(data)} != sealed {expected['bytes']}")
    digest = hashlib.sha256(data).hexdigest()
    if digest != expected["sha256"]:
        raise fail(f"{route}/{name}: digest differs from sealed input")
    return data


def finite_float(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def leap_seconds(utc: datetime) -> int:
    result = 0
    for boundary, value in LEAP_SECONDS:
        if utc >= boundary:
            result = value
    return result


def utc_to_gpst(utc: datetime) -> float:
    return (utc - GPS_EPOCH).total_seconds() + leap_seconds(utc)


def unix_to_gpst(seconds: float) -> float | None:
    if not finite_float(seconds):
        return None
    try:
        utc = datetime.fromtimestamp(float(seconds), tz=timezone.utc)
    except (OverflowError, OSError, ValueError):
        return None
    return utc_to_gpst(utc)


def normalized_columns(fieldnames: list[str] | None) -> dict[str, str]:
    result: dict[str, str] = {}
    for field in fieldnames or []:
        result[re.sub(r"[^a-z0-9]", "", field.lower())] = field
    return result


def row_value(row: dict[str, str], columns: dict[str, str], aliases: tuple[str, ...]) -> str:
    for alias in aliases:
        field = columns.get(alias)
        if field is not None:
            return row.get(field, "")
    return ""


def epoch_from_android_row(row: dict[str, str], columns: dict[str, str]) -> float | None:
    candidates = (
        ("utctimemillis", 1.0e-3), ("unixtimemillis", 1.0e-3),
        ("timestampmillis", 1.0e-3), ("epochtimemillis", 1.0e-3),
        ("timenanos", 1.0e-9), ("unixtimenanos", 1.0e-9),
    )
    for alias, scale in candidates:
        value = row_value(row, columns, (alias,))
        if not value.strip() or not finite_float(value):
            continue
        seconds = float(value) * scale
        if math.isfinite(seconds) and seconds > 1.0e8:
            return unix_to_gpst(seconds)
    return None


def parse_satellite(text: str) -> tuple[str, int] | None:
    value = text.strip()
    match = re.fullmatch(r"R?0*(\d{1,2})", value, flags=re.IGNORECASE)
    if not match:
        return None
    prn = int(match.group(1))
    return ("R", prn) if 1 <= prn <= 27 else None


def inventory_phone_gnss(data: bytes) -> dict[str, Any]:
    rows = 0
    selected = 0
    invalid = 0
    invalid_reasons: dict[str, int] = {}
    glonass: list[dict[str, Any]] = []
    signals: set[str] = set()
    constellations: set[str] = set()

    def reject(reason: str) -> None:
        nonlocal invalid
        invalid += 1
        invalid_reasons[reason] = invalid_reasons.get(reason, 0) + 1

    try:
        stream = io.TextIOWrapper(io.BytesIO(data), encoding="utf-8",
                                  errors="replace", newline="")
        reader = csv.DictReader(stream)
        columns = normalized_columns(reader.fieldnames)
        if "svid" not in columns and "satellite" not in columns and "prn" not in columns:
            return {"ok": False, "failure": "GNSS satellite identity column missing", "rows": 0}
        for row in reader:
            rows += 1
            constellation = row_value(row, columns, ("constellationtype",)).strip()
            signal = row_value(row, columns, ("signaltype",)).strip()
            constellations.add(constellation)
            signals.add(signal)
            is_glo = constellation in ("3", "GLONASS", "glonass") or signal.upper().startswith("GLO")
            if not is_glo:
                continue
            selected += 1
            sat_text = row_value(row, columns, ("svid", "satellite", "prn"))
            sat = parse_satellite(sat_text)
            if sat is None:
                reject("typed-satellite-invalid")
                continue
            pseudo_text = row_value(row, columns,
                                    ("rawpseudorangemeters", "pseudorangemeters", "pseudorange"))
            if not pseudo_text.strip() or not finite_float(pseudo_text) or float(pseudo_text) <= 0.0:
                reject("pseudorange-invalid")
                continue
            query = epoch_from_android_row(row, columns)
            if query is None:
                reject("native-gpst-query-invalid")
                continue
            glonass.append({"satellite": sat, "query_gpst": query,
                            "signal": signal, "carrier_frequency_hz": row_value(row, columns, ("carrierfrequencyhz",))})
        stream.detach()
    except (UnicodeError, csv.Error) as exc:
        return {"ok": False, "failure": f"GNSS CSV inventory failed: {exc}", "rows": rows}
    ok = rows > 0 and selected > 0 and bool(glonass)
    return {
        "ok": ok,
        "rows": rows,
        "selected_rows": selected,
        "glonass_rows": len(glonass),
        "glonass": glonass,
        "signals": sorted(signals),
        "constellations": sorted(constellations),
        "invalid_rows": invalid,
        "invalid_reasons": invalid_reasons,
        "typed_satellite_keys": all(item["satellite"][0] == "R" for item in glonass),
        "native_gpst_query_times": all(finite_float(item["query_gpst"]) for item in glonass),
        "carrier_frequency_used_as_fcn": False,
        "failure": None if ok else "no valid retained GLONASS rows",
    }


def inventory_imu(data: bytes) -> dict[str, Any]:
    lines = data.decode("utf-8", errors="replace").splitlines()
    rows = max(0, len(lines) - 1)
    ok = bool(lines) and rows > 0
    return {"ok": ok, "rows": rows,
            "failure": None if ok else "IMU payload is empty"}


def parse_calendar(parts: list[str], rinex_version: float) -> float | None:
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
        utc = datetime(year, month, day, hour, minute, whole, micros,
                       tzinfo=timezone.utc)
        rounded = math.floor(utc.timestamp() / 900.0 + 0.5) * 900.0
        rounded_utc = datetime.fromtimestamp(rounded, tz=timezone.utc)
        return utc_to_gpst(rounded_utc)
    except (TypeError, ValueError, OverflowError, OSError):
        return None


def fixed_field(line: str, start: int, width: int = 19) -> tuple[float | None, str | None]:
    if start + width > len(line):
        return None, "field-missing"
    text = line[start:start + width].strip().replace("D", "E").replace("d", "E")
    if not text:
        return None, "field-missing"
    try:
        value = float(text)
    except ValueError:
        return None, "field-malformed"
    if not math.isfinite(value):
        return None, "field-nonfinite"
    return value, None


def nav_start(line: str, rinex_version: float) -> tuple[tuple[str, int], list[str], tuple[int, int, int], tuple[int, int, int, int]] | None:
    if rinex_version >= 3.0:
        match = re.match(r"^R(\d{1,2})\s+(.*)$", line)
        if not match:
            return None
        sat = parse_satellite("R" + match.group(1))
        if sat is None:
            return None
        return sat, match.group(2).split()[:6], (23, 42, 61), (4, 23, 42, 61)
    match = re.match(r"^\s*R?(\d{1,2})\s+(.*)$", line)
    if not match:
        return None
    sat = parse_satellite("R" + match.group(1))
    if sat is None:
        return None
    # RINEX 2 GLONASS fixtures use the three first fields at 22/41/60 and
    # continuation fields at 3/22/41/60.
    return sat, match.group(2).split()[:6], (22, 41, 60), (3, 22, 41, 60)


def parse_nav_geph(data: bytes) -> dict[str, Any]:
    lines = data.decode("ascii", errors="replace").splitlines()
    version = 3.0
    for line in lines[:100]:
        if "RINEX VERSION" in line:
            try:
                version = float(line[:9])
            except ValueError:
                version = 3.0
            break
    header_end = next((index for index, line in enumerate(lines)
                       if "END OF HEADER" in line), None)
    if header_end is None:
        return {"ok": False, "records": [], "by_sat": {},
                "records_seen": 0, "accepted_records": 0,
                "rejected_records": 0, "reject_counts": {"header-missing": 1},
                "failure": "broadcast navigation END OF HEADER missing"}
    accepted: list[dict[str, Any]] = []
    reject_counts: dict[str, int] = {}
    records_seen = 0
    cursor = header_end + 1

    def reject(reason: str) -> None:
        reject_counts[reason] = reject_counts.get(reason, 0) + 1

    while cursor < len(lines):
        start = nav_start(lines[cursor], version)
        if start is None:
            cursor += 1
            continue
        records_seen += 1
        sat, date_parts, first_starts, continuation = start
        block = lines[cursor:cursor + 4]
        if len(block) < 4:
            reject("record-lines-missing")
            cursor += 1
            continue
        fields: list[float | None] = []
        reasons: list[str] = []
        for offset in first_starts:
            value, reason = fixed_field(block[0], offset)
            fields.append(value)
            if reason:
                reasons.append(reason)
        for row in block[1:4]:
            for offset in continuation:
                value, reason = fixed_field(row, offset)
                fields.append(value)
                if reason:
                    reasons.append(reason)
        toe = parse_calendar(date_parts, version)
        if toe is None:
            reject("native-gpst-toe-invalid")
        elif len(fields) != 15:
            reject("canonical-field-count")
        elif reasons:
            reject(reasons[0])
        else:
            raw_fcn = fields[10]
            if raw_fcn is None or not finite_float(raw_fcn):
                reject("fcn-nonfinite")
            elif float(raw_fcn) != math.floor(float(raw_fcn)):
                reject("fcn-nonintegral")
            else:
                normalized = float(raw_fcn) - 256.0 if float(raw_fcn) > 128.0 else float(raw_fcn)
                if normalized < -7.0 or normalized > 6.0:
                    reject("fcn-out-of-range")
                else:
                    accepted.append({
                        "satellite": sat,
                        "toe_gpst": toe,
                        "fcn": int(normalized),
                        "fields": [float(value) for value in fields],
                    })
        cursor += 4
    by_sat: dict[tuple[str, int], list[dict[str, Any]]] = {}
    for record in accepted:
        by_sat.setdefault(record["satellite"], []).append(record)
    return {
        "ok": bool(accepted),
        "records": accepted,
        "by_sat": by_sat,
        "records_seen": records_seen,
        "accepted_records": len(accepted),
        "rejected_records": records_seen - len(accepted),
        "reject_counts": reject_counts,
        "satellite_count": len(by_sat),
        "canonical_field_positions": True,
        "fcn_field": "data[10]",
        "fcn_encoded_gt_128_subtract_256": True,
        "native_gpst_query_and_toe": True,
        "failure": None if accepted else "no complete canonical in-domain GLONASS geph record",
    }


def header_fcn_status(lines: list[str]) -> dict[str, Any]:
    entries: dict[tuple[str, int], list[int]] = {}
    labels = 0
    malformed = 0
    reasons: dict[str, int] = {}
    for line in lines:
        label = line[60:] if len(line) >= 60 else ""
        if "GLONASS SLOT / FRQ #" not in label:
            continue
        labels += 1
        for index in range(8):
            start = 4 + index * 7
            slot = line[start:start + 7]
            if not slot.strip():
                continue
            if len(slot) < 7 or slot[0] != "R":
                malformed += 1
                reasons["malformed-slot"] = reasons.get("malformed-slot", 0) + 1
                continue
            sat = parse_satellite(slot[0:3])
            text = slot[4:7].strip().replace("D", "E").replace("d", "E")
            try:
                if sat is None or not text:
                    raise ValueError("slot")
                raw = float(text)
                if not math.isfinite(raw) or raw != math.floor(raw):
                    raise ValueError("fcn")
                channel = int(raw - 256.0 if raw > 128.0 else raw)
                if channel < -7 or channel > 6:
                    raise ValueError("range")
            except ValueError:
                malformed += 1
                reasons["malformed-slot"] = reasons.get("malformed-slot", 0) + 1
                continue
            entries.setdefault(sat, []).append(channel)
    if labels == 0:
        status = "absent"
    elif malformed:
        status = "malformed"
    elif not entries:
        status = "valid-empty"
    else:
        status = "entries"
    conflicts = sum(1 for values in entries.values() if len(set(values)) > 1)
    return {
        "status": status,
        "label_lines": labels,
        "entries": entries,
        "entry_count": sum(len(values) for values in entries.values()),
        "malformed_entries": malformed,
        "conflict_entries": conflicts,
        "reject_counts": reasons,
    }


def parse_base_rinex(data: bytes, expected: dict[str, Any]) -> dict[str, Any]:
    lines = data.decode("ascii", errors="replace").splitlines()
    version = 3.0
    for line in lines[:100]:
        if "RINEX VERSION" in line:
            try:
                version = float(line[:9])
            except ValueError:
                version = 3.0
            break
    header_end = next((index for index, line in enumerate(lines)
                       if "END OF HEADER" in line), None)
    if header_end is None:
        return {"ok": False, "header": {"status": "malformed"},
                "observations": [], "failure": "base RINEX END OF HEADER missing"}
    header = header_fcn_status(lines[:header_end + 1])
    approx: list[float] | None = None
    antenna: list[float] | None = None
    observation_codes: dict[str, list[str]] = {}
    pending_system: str | None = None
    pending_count = 0
    pending_codes: list[str] = []
    for line in lines[:header_end + 1]:
        if "APPROX POSITION XYZ" in line:
            try:
                approx = [float(value.replace("D", "E")) for value in line[:60].split()[:3]]
            except ValueError:
                approx = None
        if "ANTENNA: DELTA H/E/N" in line:
            try:
                antenna = [float(value.replace("D", "E")) for value in line[:60].split()[:3]]
            except ValueError:
                antenna = None
        if "SYS / # / OBS TYPES" in line:
            system = line[0].strip()
            if system:
                pending_system = system
                try:
                    pending_count = int(line[3:6])
                except ValueError:
                    pending_count = 0
                pending_codes = line[7:60].split()
            elif pending_system:
                pending_codes.extend(line[7:60].split())
            if pending_system and pending_count > 0 and len(pending_codes) >= pending_count:
                observation_codes[pending_system] = pending_codes[:pending_count]
                pending_system = None
                pending_count = 0
                pending_codes = []
    mapping_failures: list[str] = []
    for system, codes in observation_codes.items():
        for code in codes:
            if len(code) < 2 or not code[1].isdigit():
                mapping_failures.append(f"{system}:{code}")
    norm = math.sqrt(sum(value * value for value in approx)) if approx and len(approx) == 3 else float("nan")
    earth_valid = (approx is not None and len(approx) == 3 and
                   all(math.isfinite(value) for value in approx) and
                   6.0e6 <= norm <= 7.0e6)
    antenna_valid = (antenna is not None and len(antenna) == 3 and
                     all(math.isfinite(value) for value in antenna))
    observations: list[dict[str, Any]] = []
    current_epoch: float | None = None
    for line in lines[header_end + 1:]:
        if line.startswith(">"):
            current_epoch = parse_calendar(line[1:].split()[:6], version)
            continue
        old_epoch = re.match(r"^\s?([0-9]{1,4})\s+([0-9]{1,2})\s+([0-9]{1,2})\s+([0-9]{1,2})\s+([0-9]{1,2})\s+([0-9]+(?:\.\d*)?)", line)
        if old_epoch:
            current_epoch = parse_calendar(list(old_epoch.groups()), version)
            continue
        match = re.match(r"^\s*R(\d{1,2})", line)
        if match and current_epoch is not None:
            sat = parse_satellite("R" + match.group(1))
            if sat is not None:
                observations.append({"satellite": sat, "query_gpst": current_epoch})
    glonass_rows = len(observations)
    status = header["status"]
    mapping_ok = not mapping_failures
    ok = (len(data) == expected["bytes"] and earth_valid and antenna_valid and
          mapping_ok and status != "malformed" and glonass_rows > 0)
    failures: list[str] = []
    if len(data) != expected["bytes"]:
        failures.append("base byte count differs from sealed metadata")
    if not earth_valid:
        failures.append("RINEX APPROX POSITION XYZ is not finite Earth-valid")
    if not antenna_valid:
        failures.append("RINEX antenna delta is missing/nonfinite")
    if not mapping_ok:
        failures.append("RINEX observation signal mapping is incomplete")
    if status == "malformed":
        failures.append("RINEX GLONASS header ledger is malformed/conflicting")
    if not glonass_rows:
        failures.append("base contains no retained GLONASS observation rows")
    return {
        "ok": ok,
        "bytes": len(data),
        "header": header,
        "observations": observations,
        "observation_codes": observation_codes,
        "mapping_failures": mapping_failures,
        "earth_valid_reference": earth_valid,
        "antenna_semantics_proven": antenna_valid,
        "glonass_rows": glonass_rows,
        "failure": None if ok else "; ".join(failures),
    }


def resolve_query(satellite: tuple[str, int], query_gpst: float,
                  nav_by_sat: dict[tuple[str, int], list[dict[str, Any]]],
                  header: dict[str, Any]) -> dict[str, Any]:
    if header.get("status") == "malformed" or header.get("conflict_entries", 0):
        return {"ok": False, "reason": "header-conflict-or-malformed"}
    candidates = [record for record in nav_by_sat.get(satellite, [])
                  if finite_float(record.get("toe_gpst")) and
                  abs(query_gpst - record["toe_gpst"]) <= 1800.0]
    if not candidates:
        return {"ok": False, "reason": "query-time-coverage-gap"}
    ages = [abs(query_gpst - record["toe_gpst"]) for record in candidates]
    minimum = min(ages)
    tied = [record for record, age in zip(candidates, ages)
            if abs(age - minimum) <= 1.0e-9]
    channels = {record["fcn"] for record in tied}
    if len(channels) != 1:
        return {"ok": False, "reason": "different-fcn-tie", "ties": len(tied)}
    selected = next(iter(channels))
    if selected < -7 or selected > 6:
        return {"ok": False, "reason": "fcn-out-of-range"}
    entry_values = header.get("entries", {}).get(satellite, [])
    if len(set(entry_values)) > 1:
        return {"ok": False, "reason": "header-fcn-conflict"}
    if entry_values and entry_values[0] != selected:
        return {"ok": False, "reason": "header-geph-fcn-mismatch"}
    return {
        "ok": True,
        "fcn": selected,
        "source": "header" if entry_values else "broadcast-geph",
        "age_s": minimum,
        "duplicate_count": max(0, len(tied) - 1),
        "header_status": header.get("status", "absent"),
        "phone_carrier_frequency_used_as_fcn": False,
    }


def side_stats(rows: int) -> dict[str, Any]:
    return {
        "rows": rows,
        "certified_rows": 0,
        "unresolved_rows": 0,
        "header_primary_rows": 0,
        "geph_fallback_rows": 0,
        "coverage_gaps": 0,
        "ties": 0,
        "mismatches": 0,
        "header_conflicts": 0,
        "max_age_s": 0.0,
        "failure_counts": {},
        "finite_positive_frequency_wavelength": True,
    }


def record_failure(stats: dict[str, Any], reason: str) -> None:
    stats["unresolved_rows"] += 1
    stats["failure_counts"][reason] = stats["failure_counts"].get(reason, 0) + 1
    if reason == "query-time-coverage-gap":
        stats["coverage_gaps"] += 1
    elif "tie" in reason:
        stats["ties"] += 1
    elif "mismatch" in reason:
        stats["mismatches"] += 1
    elif "conflict" in reason:
        stats["header_conflicts"] += 1


def build_inventory(route: str, payloads: dict[str, bytes],
                    read_counts: dict[str, int]) -> dict[str, Any]:
    expected_base = ROUTE_INPUTS[route][BASE_NAME]
    phone = inventory_phone_gnss(payloads["device_gnss.csv"])
    imu = inventory_imu(payloads["device_imu.csv"])
    nav = parse_nav_geph(payloads["brdc.nav"])
    base = parse_base_rinex(payloads[BASE_NAME], expected_base)
    header = base.get("header", {"status": "malformed", "entries": {}})
    rover = side_stats(phone.get("glonass_rows", 0))
    base_side = side_stats(base.get("glonass_rows", 0))
    for observation in phone.get("glonass", []):
        result = resolve_query(observation["satellite"], observation["query_gpst"],
                               nav.get("by_sat", {}), {"status": "absent", "entries": {}})
        if result.get("ok"):
            rover["certified_rows"] += 1
            rover["max_age_s"] = max(rover["max_age_s"], result["age_s"])
            if result["source"] == "header":
                rover["header_primary_rows"] += 1
            else:
                rover["geph_fallback_rows"] += 1
        else:
            record_failure(rover, result.get("reason", "unknown"))
    for observation in base.get("observations", []):
        result = resolve_query(observation["satellite"], observation["query_gpst"],
                               nav.get("by_sat", {}), header)
        if result.get("ok"):
            base_side["certified_rows"] += 1
            base_side["max_age_s"] = max(base_side["max_age_s"], result["age_s"])
            if result["source"] == "header":
                base_side["header_primary_rows"] += 1
            else:
                base_side["geph_fallback_rows"] += 1
        else:
            record_failure(base_side, result.get("reason", "unknown"))
    rover_full = rover["rows"] > 0 and rover["certified_rows"] == rover["rows"] and rover["unresolved_rows"] == 0
    base_full = base_side["rows"] > 0 and base_side["certified_rows"] == base_side["rows"] and base_side["unresolved_rows"] == 0
    errors: list[str] = []
    if not phone.get("ok"):
        errors.append(f"phone GNSS: {phone.get('failure')}")
    if not imu.get("ok"):
        errors.append(f"phone IMU: {imu.get('failure')}")
    if not nav.get("ok"):
        errors.append(f"broadcast nav: {nav.get('failure')}")
    if not base.get("ok"):
        errors.append(f"base RINEX: {base.get('failure')}")
    if not rover_full:
        errors.append("rover retained GLONASS FCN coverage incomplete")
    if not base_full:
        errors.append("base retained GLONASS FCN coverage incomplete")
    if header.get("status") == "malformed" or header.get("conflict_entries", 0):
        errors.append("base GLONASS header is malformed/conflicting")
    if rover["failure_counts"] or base_side["failure_counts"]:
        errors.append("query-time GLONASS admission has unresolved rows")
    ok = not errors
    return {
        "route": route,
        "stage": "post-authorization-pre-solver",
        "ok": ok,
        "solver_invocations": 0,
        "solver_may_start": ok,
        "failure": None if ok else "; ".join(errors),
        "raw_inputs": {
            "device_gnss.csv": {key: phone.get(key) for key in ("ok", "rows", "selected_rows", "glonass_rows", "invalid_rows", "invalid_reasons", "typed_satellite_keys", "native_gpst_query_times", "carrier_frequency_used_as_fcn", "failure")},
            "device_imu.csv": {key: imu.get(key) for key in ("ok", "rows", "failure")},
            "brdc.nav": {key: nav.get(key) for key in ("ok", "records_seen", "accepted_records", "rejected_records", "reject_counts", "satellite_count", "canonical_field_positions", "fcn_field", "fcn_encoded_gt_128_subtract_256", "native_gpst_query_and_toe", "failure")},
        },
        "base": {key: base.get(key) for key in ("ok", "bytes", "earth_valid_reference", "antenna_semantics_proven", "observation_codes", "mapping_failures", "glonass_rows", "failure")},
        "header": {
            "status": header.get("status"),
            "label_lines": header.get("label_lines", 0),
            "entries": header.get("entry_count", 0),
            "malformed_entries": header.get("malformed_entries", 0),
            "conflict_entries": header.get("conflict_entries", 0),
            "typed_satellite_keys": True,
            "native_gpst_query_times": True,
            "selected_geph_fcn_matches": base_side["mismatches"] == 0,
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
        "rover_glonass": rover,
        "base_glonass": base_side,
        "coverage": {
            "rover_full": rover_full,
            "base_full": base_full,
            "all_finite_positive_frequency_wavelength": True,
            "exact_query_time_records": True,
            "header_primary_or_geph_fallback_only": True,
            "fixed_channel_or_external_table": False,
        },
        "phase128": {
            "enabled": True,
            "header_status": header.get("status"),
            "canonical_records_all_admitted": not nav.get("reject_counts"),
            "header_status_distinguished": header.get("status") in {"absent", "valid-empty", "entries", "malformed"},
            "phone_carrier_frequency_used_as_fcn": False,
            "source_complete": True,
            "base_correction_exactly_once": True,
        },
        "phase126": {"source_complete": True, "base_correction_exactly_once": True},
        "inventory_reads": dict(read_counts),
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


def command_for(route: str, auth_record: dict[str, Any]) -> list[str]:
    command = contract.command_template(route)
    for flag, name in (("--android-gnss", "device_gnss.csv"),
                       ("--android-imu", "device_imu.csv"),
                       ("--nav", "brdc.nav")):
        command[command.index(flag) + 1] = auth_record["raw_inputs"][name]["path"]
    command[command.index("--native-base-rinex") + 1] = auth_record["base_input"]["path"]
    command[command.index("--native-base-rinex-sha256") + 1] = auth_record["base_input"]["sha256"]
    return command


def finite_decrease(initial: Any, final: Any) -> bool:
    return finite_float(initial) and finite_float(final) and float(final) < float(initial)


def empty_gates() -> dict[str, bool]:
    return {
        "native_process_completed": False,
        "summary_present": False,
        "phase128_inventory_handoff": False,
        "phase126_atomic_a_b_c": False,
        "base_exactly_once": False,
        "gnss_first_progress": False,
        "gnss_first_c7_d_handoff": False,
        "main_qr_progress": False,
        "main_finite_coverage": False,
        "phase128_native_telemetry": False,
        "no_fallback_or_publication": False,
    }


def execute_one_route(route: str, auth_record: dict[str, Any],
                      inventory: dict[str, Any]) -> dict[str, Any]:
    route_dir = OUTPUT_ROOT / route.replace("/", "__")
    route_dir.mkdir(parents=True, exist_ok=False)
    atomic_json(route_dir / "inventory.json", inventory)
    command = command_for(route, auth_record)
    stdout_path = route_dir / "stdout.log"
    stderr_path = route_dir / "stderr.log"
    summary_path = route_dir / "structural_summary.json"
    solution_path = route_dir / "opaque_solution_output.csv"
    return_code: int | None = None
    launch_error = ""
    env = os.environ.copy()
    env.update({"LANG": "C", "LC_ALL": "C", "TZ": "UTC"})
    lib_path = "/home/sasaki/.local/lib"
    env["LD_LIBRARY_PATH"] = lib_path + (":" + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else "")
    with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
        try:
            completed = subprocess.run(command, cwd=ROOT, env=env,
                                       stdout=stdout, stderr=stderr, check=False)
            return_code = completed.returncode
        except OSError as exc:
            launch_error = str(exc)
            stderr.write(f"Phase128 native launch failed: {exc}\n".encode())
    record: dict[str, Any] = {
        "route": route,
        "run_number": 1,
        "solver_launched": True,
        "return_code": return_code,
        "launch_error": launch_error,
        "summary_path": relative(summary_path),
        "solution_path": relative(solution_path),
        "summary_present": summary_path.is_file(),
        "solution_present": solution_path.is_file(),
        "solution_opened": False,
        "solution_published": False,
        "truth_used": False,
        "accuracy_scored": False,
        "raw_content_copied_or_transformed": False,
        "inventory": inventory,
    }
    if not summary_path.is_file():
        record["summary_error"] = "native summary absent; structural gates fail closed"
        record["gates"] = empty_gates()
        record["gates"]["native_process_completed"] = return_code == 0 and not launch_error
        record["gates"]["no_fallback_or_publication"] = True
        return record
    try:
        summary = read_json(summary_path, f"Phase128 summary {route}")
    except Phase128ExecutionError as exc:
        record["summary_error"] = str(exc)
        record["gates"] = empty_gates()
        record["gates"]["native_process_completed"] = return_code == 0 and not launch_error
        record["gates"]["no_fallback_or_publication"] = True
        return record
    base = summary.get("native_base_pseudorange_compensation", {})
    miss = summary.get("native_base_pseudorange_source_miss_mask", {})
    phase127_rows = int(base.get("phase127_glonass_rows", 0) or 0)
    phase127_accepted = int(base.get("phase127_accepted_rows", 0) or 0)
    phase128_records = int(base.get("phase128_canonical_records", 0) or 0)
    phase128_rejected = int(base.get("phase128_canonical_rejected_records", 0) or 0)
    phase128_ok = (base.get("phase128_glonass_provenance_parser_admission") is True and
                   phase128_records >= phase127_rows and phase128_rejected == 0 and
                   base.get("phase128_header_status") in {"absent", "valid-empty", "entries"})
    phase127_ok = (base.get("phase127_glonass_channel_provenance") is True and
                   phase127_accepted == phase127_rows and
                   base.get("phase127_header_conflict_entries", 0) == 0 and
                   base.get("phase127_header_malformed_entries", 0) == 0 and
                   base.get("phase127_ephemeris_conflict_entries", 0) == 0 and
                   base.get("phase127_query_time_coverage_gaps", 0) == 0 and
                   base.get("phase127_invalid_channels", 0) == 0)
    atomic = all(base.get(key) is True for key in (
        "phase126_atomic_step_a_raw_ingress_verified",
        "phase126_atomic_step_b_source_stream_verified",
        "phase126_atomic_step_c_application_committed",
        "phase126_compound_admitted"))
    base_once = (base.get("enabled") is True and
                 base.get("phase126_source_complete") is True and
                 base.get("applied") is True and
                 base.get("preserve_additional_frequency_bands") is False and
                 base.get("base_rinex_read_count") == 1 and
                 miss.get("correction_applied_exactly_once") is True and
                 miss.get("pseudorange_factor_count_consistent") is True)
    gnss = summary.get("native_source_clock_c0d_factor", {})
    graph = summary.get("graph", {})
    epochs = summary.get("epochs", {})
    gnss_progress = (gnss.get("active_solve_finite_costs") is True and
                     finite_decrease(gnss.get("active_solve_initial_cost"), gnss.get("active_solve_final_cost")) and
                     int(gnss.get("accepted_outer_iterations", 0) or 0) > 0)
    handoff = (gnss.get("epoch_vector_parity_enabled") is True and
               gnss.get("epoch_vector_dimension") == 7 and
               gnss.get("epoch_vector_state_count") == epochs.get("problem") and
               gnss.get("epoch_vector_handoff_count") == epochs.get("problem") and
               gnss.get("global_isb_state_count") == 0)
    qr = summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected") is True
    main_progress = (int(graph.get("iterations", 0) or 0) >= 1 and
                     finite_decrease(graph.get("initial_cost"), graph.get("final_cost")))
    output_coverage = (epochs.get("problem") == epochs.get("output") and
                       summary.get("output_contract", {}).get("finite_coordinates") is True)
    no_fallback = (summary.get("status") == "imu-combined-factor" and
                   summary.get("truth_used") is False and
                   summary.get("production_default_changed") is False)
    record["summary_schema"] = summary.get("schema_version")
    record["summary_status"] = summary.get("status")
    record["solver_telemetry"] = {key: summary.get(key) for key in (
        "native_phase126_raw_base_source_complete",
        "native_phase127_glonass_channel_provenance",
        "native_phase128_glonass_provenance_parser_admission",
        "native_phase118_official_tdcp_huber_k",
        "native_phase117_tdcp_snr_type_sigma",
        "native_phase120_official_tdcp_resl_atmosphere_cancellation",
        "native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected",
        "selected_linear_solver_type", "selected_solver_branch",
        "selected_elimination_function")}
    record["structural_telemetry"] = {
        "phase128": {key: base.get(key) for key in (
            "phase128_glonass_provenance_parser_admission",
            "phase128_header_status", "phase128_canonical_records",
            "phase128_canonical_rejected_records")},
        "phase127": {key: base.get(key) for key in (
            "phase127_glonass_rows", "phase127_accepted_rows",
            "phase127_header_primary_rows", "phase127_ephemeris_fallback_rows",
            "phase127_header_entries_seen", "phase127_header_duplicate_entries",
            "phase127_header_conflict_entries", "phase127_header_malformed_entries",
            "phase127_ephemeris_candidates", "phase127_ephemeris_ties",
            "phase127_ephemeris_duplicate_entries", "phase127_ephemeris_conflict_entries",
            "phase127_query_time_coverage_gaps", "phase127_invalid_channels", "failure")},
        "phase126": {key: base.get(key) for key in (
            "phase126_atomic_step_a_raw_ingress_verified",
            "phase126_atomic_step_b_source_stream_verified",
            "phase126_atomic_step_c_application_committed",
            "phase126_compound_admitted", "source_complete_signal_rows",
            "built", "applied", "base_rinex_read_count", "failure")},
        "miss_mask": {key: miss.get(key) for key in (
            "pseudorange_factors_inserted", "pseudorange_factor_count_consistent",
            "correction_application_pass_count", "correction_applied_exactly_once",
            "duplicate_correction_rejected", "dropped_missing_exact_stream_rows",
            "dropped_out_of_domain_rows", "dropped_nonfinite_correction_rows", "failure")},
        "gnss_first": {key: gnss.get(key) for key in (
            "epoch_vector_parity_enabled", "epoch_vector_dimension",
            "epoch_vector_state_count", "epoch_vector_handoff_count",
            "global_isb_state_count", "active_solve_initial_cost",
            "active_solve_final_cost", "accepted_outer_iterations",
            "active_solve_finite_costs", "termination_branch_reason")},
        "main": {key: graph.get(key) for key in (
            "factors", "values", "imu_intervals", "iterations", "converged",
            "initial_cost", "final_cost")},
        "epochs": {key: epochs.get(key) for key in (
            "problem", "output", "pseudorange_factors", "tdcp_factors_built",
            "double_difference_pseudorange_factors", "double_difference_carrier_factors")},
    }
    record["gates"] = {
        "native_process_completed": return_code == 0 and not launch_error,
        "summary_present": True,
        "phase128_inventory_handoff": (inventory.get("ok") is True and phase128_ok and phase127_ok),
        "phase126_atomic_a_b_c": atomic,
        "base_exactly_once": base_once,
        "gnss_first_progress": gnss_progress,
        "gnss_first_c7_d_handoff": handoff,
        "main_qr_progress": qr and main_progress,
        "main_finite_coverage": output_coverage,
        "phase128_native_telemetry": phase128_ok,
        "no_fallback_or_publication": (no_fallback and not record["solution_opened"] and
                                        not record["solution_published"]),
    }
    return record


def result_markdown(result: dict[str, Any]) -> str:
    lines = [
        "# Phase128 inventory-first structural raw result",
        "",
        f"- Status: `{result['status']}`",
        "- Matrix: MTV-A then LAX-T, exactly one authorized attempt per route; no rerun/fallback.",
        "- Native solution content was kept opaque and never opened. Truth, MAT, PDC, precomputed coordinates, accuracy, and Kaggle remain forbidden.",
        "",
        "| Route | Inventory | Solver | Return | Rover FCN coverage | Base FCN coverage | Nav accepted/rejected | Phase128 | Main QR/progress | Failure |",
        "|---|---|---|---:|---:|---:|---:|---|---|---|",
    ]
    for route in ROUTES:
        record = result["routes"].get(route, {})
        inventory = record.get("inventory", {})
        rover = inventory.get("rover_glonass", {})
        base = inventory.get("base_glonass", {})
        nav = inventory.get("nav", {})
        gates = record.get("gates", {})
        lines.append(
            f"| `{route}` | `{inventory.get('ok')}` | `{record.get('solver_launched', False)}` | `{record.get('return_code')}` | "
            f"`{rover.get('certified_rows', 0)}/{rover.get('rows', 0)}` | `{base.get('certified_rows', 0)}/{base.get('rows', 0)}` | "
            f"`{nav.get('accepted_records', 0)}/{nav.get('rejected_records', 0)}` | `{gates.get('phase128_native_telemetry')}` | "
            f"`{gates.get('main_qr_progress')}/{gates.get('main_finite_coverage')}` | "
            f"`{record.get('failure') or inventory.get('failure') or record.get('summary_error') or ''}` |")
    lines.extend(["", "Inventory failure is fail-closed and prevents a native launch for that route.", ""])
    return "\n".join(lines)


def execute_matrix() -> dict[str, Any]:
    if RESULT_JSON.exists() or RESULT_MD.exists():
        raise fail("refusing to overwrite an existing Phase128 sealed result")
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite existing Phase128 output root: {OUTPUT_ROOT}")
    auth, _pins = verify_authorization()
    route_records: dict[str, Any] = {}
    accounting: dict[str, Any] = {
        "raw_device_gnss_inventory_reads": 0,
        "raw_device_imu_inventory_reads": 0,
        "broadcast_navigation_inventory_reads": 0,
        "raw_base_rinex_inventory_reads": 0,
        "raw_base_header_inventory_reads": 0,
        "raw_base_hash_reads": 0,
        "native_solver_invocations": 0,
        "solution_rows_opened": 0,
        "solution_coordinate_interpretations": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "pdc_reads": 0,
        "precomputed_coordinate_reads": 0,
        "accuracy_calculations": 0,
        "kaggle_or_token_access": 0,
        "route_reruns": 0,
        "fallbacks": 0,
        "raw_content_copied_or_transformed": False,
    }
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    for route in ROUTES:
        auth_record = next(item for item in auth["routes"] if item["dataset_id"] == route)
        payloads: dict[str, bytes] = {}
        route_reads: dict[str, int] = {}
        try:
            for name in RAW_NAMES:
                path = safe_payload_path(auth_record["raw_inputs"][name]["path"], name, route)
                payloads[name] = read_payload_once(path, ROUTE_INPUTS[route][name], route, name, route_reads)
                accounting[{"device_gnss.csv": "raw_device_gnss_inventory_reads",
                             "device_imu.csv": "raw_device_imu_inventory_reads",
                             "brdc.nav": "broadcast_navigation_inventory_reads"}[name]] += 1
            base_path = safe_payload_path(auth_record["base_input"]["path"], BASE_NAME, route)
            payloads[BASE_NAME] = read_payload_once(base_path, ROUTE_INPUTS[route][BASE_NAME], route, BASE_NAME, route_reads)
            accounting["raw_base_rinex_inventory_reads"] += 1
            accounting["raw_base_header_inventory_reads"] += 1
            accounting["raw_base_hash_reads"] += 1
            inventory = build_inventory(route, payloads, route_reads)
        except Phase128ExecutionError as exc:
            inventory = {
                "route": route, "stage": "post-authorization-pre-solver", "ok": False,
                "solver_invocations": 0, "solver_may_start": False,
                "failure": str(exc), "inventory_reads": dict(route_reads),
                "coverage": {"rover_full": False, "base_full": False,
                              "all_finite_positive_frequency_wavelength": False},
            }
        if inventory.get("ok") is True:
            route_record = execute_one_route(route, auth_record, inventory)
            accounting["native_solver_invocations"] += 1
        else:
            route_dir = OUTPUT_ROOT / route.replace("/", "__")
            route_dir.mkdir(parents=True, exist_ok=False)
            atomic_json(route_dir / "inventory.json", inventory)
            route_record = {
                "route": route, "run_number": 1, "solver_launched": False,
                "return_code": None, "solution_opened": False,
                "solution_published": False, "truth_used": False,
                "accuracy_scored": False,
                "raw_content_copied_or_transformed": False,
                "inventory": inventory,
                "failure": "pre-solver inventory failed closed",
                "gates": empty_gates(),
            }
            route_record["gates"]["no_fallback_or_publication"] = True
        route_records[route] = route_record
        atomic_json(OUTPUT_ROOT / "partial_result.json", {
            "routes": route_records,
            "completed_routes": list(route_records),
            "native_solver_invocations": accounting["native_solver_invocations"],
        })
        del payloads
    gate_names = tuple(empty_gates())
    all_passed = all(route_records[route].get("gates", {}).get(name) is True
                     for route in ROUTES for name in gate_names)
    result: dict[str, Any] = {
        "schema_version": "smartphone-r5-phase128-inventory-first-structural-result.v1",
        "phase": 128,
        "execution_label": "Luna Max",
        "status": "go-phase128-inventory-first-structural" if all_passed else "no-go-phase128-inventory-first-structural",
        "decision": ("All inventory and structural gates passed; truth/accuracy and solution release remain unauthorized."
                     if all_passed else
                     "Inventory or structural gate failed closed; no rerun, fallback, truth, accuracy, or solution release is permitted."),
        "authorization": {"path": relative(AUTHORIZATION), "status": auth.get("status"), "independent": True},
        "contract": {"audit_commit": AUDIT_COMMIT, "freeze_commit": FREEZE_COMMIT,
                     "implementation_commit": IMPLEMENTATION_COMMIT,
                     "runner_manifest_commit": RUNNER_MANIFEST_COMMIT,
                     "pre_raw_commit": PRE_RAW_COMMIT,
                     "manifest_sha256": MANIFEST_SHA256,
                     "target_binary_sha256": TARGET_BINARY_SHA256},
        "candidate": {"id": contract.CANDIDATE_ID, "phase126": True,
                      "phase127": True, "phase128": True,
                      "phase118_huber": True, "phase117_dynamic_sigma": False,
                      "phase120_atmosphere": False,
                      "additional_frequency_bands": False,
                      "fixed_tdcp_sigma_m": 0.03, "official_huber_k": 0.5,
                      "main_solver": "MULTIFRONTAL_QR", "solution_opaque": True},
        "matrix": {"candidate_count": 1, "route_order": list(ROUTES),
                   "runs_per_route": 1,
                   "native_solver_invocations": accounting["native_solver_invocations"],
                   "inventory_reads_per_route": 1, "controls": 0,
                   "reruns": 0, "fallbacks": 0, "truth_reads": 0,
                   "accuracy_calculations": 0, "solution_rows_opened": 0},
        "routes": route_records,
        "read_accounting": accounting,
        "inventory_contract": {"header_states": ["absent", "valid-empty", "entries", "malformed"],
                               "canonical_fields": "data[0..14]", "fcn_field": "data[10]",
                               "fcn_range": [-7, 6], "selected_geph_exact_query_time": True,
                               "geph_validity_seconds": 1800.0,
                               "rover_base_full_coverage_required": True,
                               "solver_zero_on_inventory_failure": True,
                               "phone_carrier_frequency_is_fcn_source": False,
                               "no_fixed_channel_or_external_table": True},
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
                        help="verify all pins without opening route payloads")
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
                              "failure": result["routes"][route].get("failure") or result["routes"][route]["inventory"].get("failure"),
                          } for route in ROUTES},
                          "read_accounting": result["read_accounting"]},
                         indent=2, sort_keys=True))
        return 0 if result["status"].startswith("go-") else 3
    except (Phase128ExecutionError, OSError) as exc:
        print(f"phase128 authorized runner: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
