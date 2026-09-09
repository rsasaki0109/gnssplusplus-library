#!/usr/bin/env python3
"""Truth-free Phase68 diagnosis of Phase67 base matching misses.

This evaluator reads each pinned raw Android GNSS CSV and base RINEX member
once, in one process, and parses the bytes in memory.  It does not invoke the
native solver or read navigation, IMU, truth, MAT, validation, or prior-phase
outputs.  The result is descriptive only: the Phase67 coverage gate is not
changed and no correction is implemented.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field
from datetime import datetime, timezone
import hashlib
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase68_base_matching_taxonomy_freeze_v1.json"
FREEZE_SHA256 = "f409f7038add71cd0a41df9278fc99651d637dd3697c10913c9da1fbb8c53b43"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase68_base_matching_taxonomy_manifest_v1.json"
P65_RUNNER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase65_native_base_pseudorange_compensation.py"
P65_RUNNER_SHA256 = "6be7dd346a1ff219c15036fcf194861ebeda700678d52e3d93b2f3c2b96a16ae"
P65_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase65_native_base_pseudorange_compensation_manifest_v1.json"
P65_MANIFEST_SHA256 = "1306480f6b7fb839e55309e9c2c77b41f2ac0a1baf9f6971949612fd485d2255"
PHASE67_FAILURE = ROOT / "docs/use_cases/records/smartphone_r5_phase67_phase66_structural_integrity_recovery_failure_v1.json"
PHASE67_FAILURE_SHA256 = "291efc6619fb68cd724c4f81f3a73535b13ef79606177be9bb98ce5c31e9d46a"
ANDROID_LOADER_SOURCE = ROOT / "src/io/android_raw_gnss.cpp"
ANDROID_LOADER_SHA256 = "1bb9369e7db651e53fddab2a92c2f217512667bb349f70aa52934aeb80cbf387"
RINEX_LOADER_SOURCE = ROOT / "src/io/rinex.cpp"
RINEX_LOADER_SHA256 = "92f683e34b4052f8d105f69fc4bf4397ca990be3b5a6b220181a56423f465533"
SIGNAL_POLICY_SOURCE = ROOT / "include/libgnss++/core/signal_policy.hpp"
SIGNAL_POLICY_SHA256 = "d3f8edbdd785f0292c2b8a1596248c00a2b5e9284c09de439e5c7340c1dbca99"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase68-base-matching-taxonomy-v1"
TIME_TOLERANCE_S = 1.0e-6
NANOSECONDS_PER_SECOND = 1.0e9
SECONDS_PER_WEEK = 604800.0
HALF_WEEK = 302400.0
SPEED_OF_LIGHT = 299792458.0


def _load_phase65() -> Any:
    spec = importlib.util.spec_from_file_location("phase65_contract", P65_RUNNER)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load sealed Phase65 contract helper")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


P65 = _load_phase65()
ROUTES = P65.ROUTES


class Phase68Error(ValueError):
    """Raised when the Phase68 immutable diagnostic contract fails."""


def fail(message: str) -> Phase68Error:
    return Phase68Error(message)


def normalise_header(value: str) -> str:
    return "".join(character.lower() for character in value.strip() if character.isalnum())


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256(path: Path) -> str:
    P65.reject_forbidden(path)
    if not path.is_file():
        raise fail(f"missing file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise fail(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def load_json(path: Path, label: str) -> dict[str, Any]:
    P65.reject_forbidden(path)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise fail(f"invalid {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object: {path}")
    return value


def atomic_write(path: Path, payload: bytes) -> None:
    P65.reject_forbidden(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "wb") as handle:
            descriptor = -1
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = ""
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        if temporary:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    atomic_write(path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode())


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def read_once(path: Path, expected_sha256: str, expected_bytes: int | None = None) -> bytes:
    """Read one input file once, hashing the same in-memory bytes."""
    P65.reject_forbidden(path)
    try:
        with path.open("rb") as handle:
            payload = handle.read()
    except OSError as exc:
        raise fail(f"failed to read pinned input {path}: {exc}") from exc
    actual = sha256_bytes(payload)
    if actual != expected_sha256:
        raise fail(f"input hash mismatch: {path}: {actual} != {expected_sha256}")
    if expected_bytes is not None and len(payload) != expected_bytes:
        raise fail(f"input byte count mismatch: {path}: {len(payload)} != {expected_bytes}")
    return payload


def verify_freeze() -> dict[str, Any]:
    if sha256(FREEZE) != FREEZE_SHA256:
        raise fail("Phase68 freeze hash changed")
    freeze = load_json(FREEZE, "Phase68 freeze")
    if freeze.get("status") != "frozen-before-phase68-raw-base-read":
        raise fail("Phase68 freeze status changed")
    authority = freeze.get("authority", {})
    if authority.get("base_commit") != "7f0b869":
        raise fail("Phase68 authority commit changed")
    if sha256(P65_RUNNER) != P65_RUNNER_SHA256 or sha256(P65_MANIFEST) != P65_MANIFEST_SHA256:
        raise fail("sealed Phase65 contract hash changed")
    if sha256(PHASE67_FAILURE) != PHASE67_FAILURE_SHA256:
        raise fail("sealed Phase67 failure record changed")
    for path, expected in (
        (ANDROID_LOADER_SOURCE, ANDROID_LOADER_SHA256),
        (RINEX_LOADER_SOURCE, RINEX_LOADER_SHA256),
        (SIGNAL_POLICY_SOURCE, SIGNAL_POLICY_SHA256),
    ):
        if sha256(path) != expected:
            raise fail(f"source contract hash changed: {path}")
    try:
        P65.verify_freeze()
    except Exception as exc:
        raise fail(f"sealed Phase65 contract no longer verifies: {exc}") from exc
    if tuple(freeze.get("route_order", ())) != ROUTES:
        raise fail("Phase68 route order changed")
    raw_inputs = freeze.get("raw_inputs", {})
    base_inputs = freeze.get("base_inputs", {})
    if set(raw_inputs) != set(ROUTES) or set(base_inputs) != set(ROUTES):
        raise fail("Phase68 route input pins changed")
    read_contract = freeze.get("read_contract", {})
    expected_read_contract = {
        "single_process": True,
        "raw_device_gnss_reads": 4,
        "base_rinex_reads": 4,
        "nav_reads": 0,
        "imu_reads": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "validation_holdout_reads": 0,
        "archive_reopens": 0,
        "solver_invocations": 0,
        "old_phase65_or_phase66_output_reuse": False,
    }
    for key, value in expected_read_contract.items():
        if read_contract.get(key) != value:
            raise fail(f"Phase68 read contract changed: {key}")
    if not MANIFEST.is_file():
        raise fail("Phase68 manifest is missing")
    manifest = load_json(MANIFEST, "Phase68 manifest")
    if manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256:
        raise fail("Phase68 manifest freeze pin changed")
    if manifest.get("evaluator", {}).get("sha256") != sha256(Path(__file__)):
        raise fail("Phase68 evaluator hash pin changed")
    if manifest.get("routes") != list(ROUTES):
        raise fail("Phase68 manifest routes changed")
    if manifest.get("read_accounting", {}).get("truth_reads") != 0:
        raise fail("Phase68 manifest truth accounting changed")
    if manifest.get("read_accounting", {}).get("old_phase65_or_phase66_output_reuse") is not False:
        raise fail("Phase68 manifest prior-output policy changed")
    return freeze


def finite_float(value: str | None, *, required: bool = False) -> float:
    token = "" if value is None else value.strip()
    if not token:
        if required:
            raise fail("required numeric field is blank")
        return math.nan
    try:
        parsed = float(token)
    except ValueError as exc:
        raise fail(f"invalid numeric field: {token!r}") from exc
    if not math.isfinite(parsed):
        if required:
            raise fail(f"required numeric field is non-finite: {token!r}")
        return math.nan
    return parsed


def integer(value: str | None, *, required: bool = True) -> int:
    token = "" if value is None else value.strip()
    if not token:
        if required:
            raise fail("required integer field is blank")
        return 0
    try:
        return int(token)
    except ValueError as exc:
        raise fail(f"invalid integer field: {token!r}") from exc


def close_frequency(actual: float, expected: float, tolerance: float = 1000.0) -> bool:
    return math.isfinite(actual) and abs(actual - expected) <= tolerance


@dataclass(frozen=True)
class SignalInfo:
    system: str
    signal: str
    canonical_frequency: str
    band: int
    channel: int | None = None


def parse_android_signal(token: str, constellation: int, frequency_hz: float) -> SignalInfo | None:
    name = token.strip().upper()
    if constellation == 1:
        if (name in {"GPS_L1_CA", "GPS_L1C", "L1"} or not name) and close_frequency(frequency_hz, 1575.42e6):
            return SignalInfo("GPS", "GPS_L1CA", "GPS_L1", 1)
        if (name in {"GPS_L5_Q", "GPS_L5", "L5"} or not name) and close_frequency(frequency_hz, 1176.45e6):
            return SignalInfo("GPS", "GPS_L5", "GPS_L5", 5)
    if constellation == 3:
        if name in {"GLO_G1_CA", "GLO_G1C", "GLO_L1", "L1"} or not name:
            channel_value = (frequency_hz - 1602.0e6) / 0.5625e6
            channel = int(round(channel_value))
            if -7 <= channel <= 6 and close_frequency(frequency_hz, 1602.0e6 + channel * 0.5625e6):
                return SignalInfo("GLONASS", "GLO_L1CA", f"GLO_L1_CH{channel}", 1, channel)
    if constellation == 6:
        if (name in {"GAL_E1_C_P", "GAL_E1_C", "GAL_E1", "L1"} or not name) and close_frequency(frequency_hz, 1575.42e6):
            return SignalInfo("Galileo", "GAL_E1", "GAL_E1", 1)
        if (name in {"GAL_E5A_Q", "GAL_E5A", "GAL_E5", "L5"} or not name) and close_frequency(frequency_hz, 1176.45e6):
            return SignalInfo("Galileo", "GAL_E5A", "GAL_E5A", 5)
    if constellation == 5:
        if (name in {"BDS_B1I", "BDS_B1_I", "BDS_B1", "B1I"} or not name) and close_frequency(frequency_hz, 1561.098e6):
            return SignalInfo("BeiDou", "BDS_B1I", "BDS_B1I", 2)
        if (name in {"BDS_B1C", "BDS_B1_C"} or not name) and close_frequency(frequency_hz, 1575.42e6):
            return SignalInfo("BeiDou", "BDS_B1C", "BDS_B1C", 1)
        if (name in {"BDS_B2A", "BDS_B2A_Q", "BDS_B2A_I", "L5"} or not name) and close_frequency(frequency_hz, 1176.45e6):
            return SignalInfo("BeiDou", "BDS_B2A", "BDS_B2A", 5)
    return None


def glonass_tow_to_gps(glonass_tow: float, gps_tow_reference: float) -> float:
    day_of_week = math.floor(gps_tow_reference / 86400.0)
    gpst = glonass_tow + day_of_week * 86400.0 - 3.0 * 3600.0 + 18.0
    day_offset = day_of_week - math.floor(gpst / 86400.0)
    return gpst + day_offset * 86400.0


def raw_pseudorange(
    time_nanos: int,
    base_full_bias_nanos: int,
    bias_nanos: float,
    time_offset_nanos: float,
    received_sv_time_nanos: int,
    system: str,
) -> float:
    gps_seconds = (time_nanos - base_full_bias_nanos) / NANOSECONDS_PER_SECOND
    week = math.floor(gps_seconds / SECONDS_PER_WEEK)
    tow_rx = gps_seconds - week * SECONDS_PER_WEEK - bias_nanos / NANOSECONDS_PER_SECOND
    tow_rx -= time_offset_nanos / NANOSECONDS_PER_SECOND
    tow_tx = received_sv_time_nanos / NANOSECONDS_PER_SECOND
    if system == "GLONASS":
        tow_tx = glonass_tow_to_gps(tow_tx, tow_rx)
    elif system == "BeiDou":
        tow_tx += 14.0
    delta = tow_rx - tow_tx
    while delta > HALF_WEEK:
        delta -= SECONDS_PER_WEEK
    while delta < -HALF_WEEK:
        delta += SECONDS_PER_WEEK
    return delta * SPEED_OF_LIGHT


def raw_epoch_time(time_nanos: int, base_full_bias_nanos: int, bias_nanos: float) -> float:
    gps_seconds = (time_nanos - base_full_bias_nanos) / NANOSECONDS_PER_SECOND
    week = math.floor(gps_seconds / SECONDS_PER_WEEK)
    return week * SECONDS_PER_WEEK + gps_seconds - week * SECONDS_PER_WEEK - bias_nanos / NANOSECONDS_PER_SECOND


def _raw_row_value(row: dict[str, str], names: Iterable[str]) -> str:
    for name in names:
        if name in row:
            return row[name]
    return ""


@dataclass
class RawAdoptedRow:
    time_s: float
    utc_millis: int
    system: str
    svid: int
    signal: str
    canonical_frequency: str
    frequency_hz: float


@dataclass
class RawReport:
    input_rows: int = 0
    raw_rows: int = 0
    adopted_rows: list[RawAdoptedRow] = field(default_factory=list)
    unsupported_signal_rows: int = 0
    invalid_timing_rows: int = 0
    invalid_quality_rows: int = 0
    invalid_pseudorange_rows: int = 0
    masked_code_rows: int = 0
    duplicate_epoch_signal_rows: int = 0
    signal_census: dict[str, int] = field(default_factory=dict)
    canonical_census: dict[str, int] = field(default_factory=dict)
    time_min_s: float = math.nan
    time_max_s: float = math.nan


def parse_raw_csv(payload: bytes) -> RawReport:
    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise fail(f"raw GNSS is not UTF-8 CSV: {exc}") from exc
    reader = csv.reader(io.StringIO(text))
    try:
        header = next(reader)
    except StopIteration as exc:
        raise fail("raw GNSS has no header") from exc
    names = [normalise_header(value) for value in header]
    if not all(names) or len(set(names)) != len(names):
        raise fail("raw GNSS header has empty or duplicate normalized fields")
    required = {
        "messagetype", "utctimemillis", "timenanos", "fullbiasnanos", "biasnanos",
        "receivedsvtimenanos", "svid", "constellationtype", "pseudorangeratemeterspersecond",
        "accumulateddeltarangestate", "accumulateddeltarangemeters", "carrierfrequencyhz",
    }
    missing = sorted(required - set(names))
    if missing:
        raise fail(f"raw GNSS header lacks required fields: {missing}")
    report = RawReport()
    base_full_bias_nanos = 0
    previous_time_nanos = 0
    previous_utc = -1
    previous_hcdc = 0
    have_previous = False
    selected_keys: set[tuple[int, int, str, int]] = set()
    for row_values in reader:
        if not row_values or all(not value.strip() for value in row_values):
            continue
        report.input_rows += 1
        if len(row_values) != len(names):
            raise fail(f"raw GNSS row {report.input_rows + 1} has wrong field count")
        row = dict(zip(names, row_values))
        if _raw_row_value(row, ("messagetype",)).strip() != "Raw":
            continue
        report.raw_rows += 1
        utc_millis = integer(_raw_row_value(row, ("utctimemillis",)))
        time_nanos = integer(_raw_row_value(row, ("timenanos",)))
        full_bias_nanos = integer(_raw_row_value(row, ("fullbiasnanos",)))
        bias_nanos = finite_float(_raw_row_value(row, ("biasnanos",)), required=True)
        received_sv_time_nanos = integer(_raw_row_value(row, ("receivedsvtimenanos",)))
        svid = integer(_raw_row_value(row, ("svid",)))
        constellation = integer(_raw_row_value(row, ("constellationtype",)))
        frequency_hz = finite_float(_raw_row_value(row, ("carrierfrequencyhz",)), required=True)
        time_offset_nanos = finite_float(_raw_row_value(row, ("timeoffsetnanos",)), required=False)
        if not math.isfinite(time_offset_nanos):
            time_offset_nanos = 0.0
        hcdc = integer(_raw_row_value(row, ("hardwareclockdiscontinuitycount",)), required=False)
        state_token = _raw_row_value(row, ("state",)).strip()
        state_present = bool(state_token)
        state = integer(state_token, required=False) if state_present else 0
        multipath_token = _raw_row_value(row, ("multipathindicator",)).strip()
        multipath_present = bool(multipath_token)
        multipath = integer(multipath_token, required=False) if multipath_present else 0
        cn0 = finite_float(_raw_row_value(row, ("cn0dbhz",)), required=False)
        bias_uncertainty = finite_float(_raw_row_value(row, ("biasuncertaintynanos",)), required=False)
        if time_nanos == 0 or received_sv_time_nanos < 10_000_000_000:
            report.invalid_timing_rows += 1
            continue
        if math.isfinite(bias_uncertainty) and bias_uncertainty > 1.0e4:
            report.invalid_quality_rows += 1
            continue
        if utc_millis < 0 or (have_previous and utc_millis < previous_utc):
            raise fail("raw GNSS utcTimeMillis is not monotonic")
        if not have_previous:
            base_full_bias_nanos = full_bias_nanos
            previous_time_nanos = time_nanos
            previous_hcdc = hcdc
            have_previous = True
        elif abs(time_nanos - previous_time_nanos) > 1_000_000_000:
            base_full_bias_nanos = full_bias_nanos
        # Hardware clock discontinuity is retained for source accounting but
        # does not reset base_full_bias in the pinned Android loader contract.
        previous_hcdc = hcdc
        previous_time_nanos = time_nanos
        previous_utc = utc_millis
        info = parse_android_signal(_raw_row_value(row, ("signaltype",)), constellation, frequency_hz)
        if info is None:
            report.unsupported_signal_rows += 1
            continue
        max_svid = 32 if info.system == "GPS" else (24 if info.system == "GLONASS" else (63 if info.system == "BeiDou" else 36))
        if not (1 <= svid <= max_svid):
            report.unsupported_signal_rows += 1
            continue
        pseudorange_m = raw_pseudorange(
            time_nanos, base_full_bias_nanos, bias_nanos, time_offset_nanos,
            received_sv_time_nanos, info.system,
        )
        if not math.isfinite(pseudorange_m) or pseudorange_m <= 0.0:
            report.invalid_pseudorange_rows += 1
            continue
        code_lock_mask = (1 << 0) | (1 << 10)
        transmit_mask = (1 << 7) | (1 << 15) if info.system == "GLONASS" else (1 << 3) | (1 << 14)
        status_invalid = state_present and ((state & code_lock_mask) == 0 or (state & transmit_mask) == 0)
        low_snr = math.isfinite(cn0) and cn0 < 20.0
        code_masked = low_snr or (multipath_present and multipath == 1) or pseudorange_m < 1.0e7 or pseudorange_m > 4.0e7 or status_invalid
        if code_masked:
            report.masked_code_rows += 1
            continue
        key = (utc_millis, svid, info.signal, constellation)
        if key in selected_keys:
            report.duplicate_epoch_signal_rows += 1
            continue
        selected_keys.add(key)
        time_s = raw_epoch_time(time_nanos, base_full_bias_nanos, bias_nanos)
        adopted = RawAdoptedRow(time_s, utc_millis, info.system, svid, info.signal, info.canonical_frequency, frequency_hz)
        report.adopted_rows.append(adopted)
        report.signal_census[info.signal] = report.signal_census.get(info.signal, 0) + 1
        report.canonical_census[info.canonical_frequency] = report.canonical_census.get(info.canonical_frequency, 0) + 1
    if report.adopted_rows:
        report.time_min_s = min(row.time_s for row in report.adopted_rows)
        report.time_max_s = max(row.time_s for row in report.adopted_rows)
    return report


def system_from_rinex_char(value: str) -> str | None:
    return {"G": "GPS", "R": "GLONASS", "E": "Galileo", "C": "BeiDou", "J": "QZSS", "I": "NavIC"}.get(value)


def rinex_signal(system: str, obs_type: str, channel: int | None) -> SignalInfo | None:
    if len(obs_type) < 2 or not obs_type[1].isdigit():
        return None
    band = int(obs_type[1])
    tracking = obs_type[2] if len(obs_type) >= 3 else ""
    if system == "GPS":
        mapping = {1: ("GPS_L1CA", "GPS_L1"), 2: ("GPS_L2C", "GPS_L2"), 5: ("GPS_L5", "GPS_L5")}
    elif system == "GLONASS":
        mapping = {1: ("GLO_L1P" if tracking == "P" else "GLO_L1CA", f"GLO_L1_CH{channel}" if channel is not None else "GLO_L1"), 2: ("GLO_L2P" if tracking == "P" else "GLO_L2CA", f"GLO_L2_CH{channel}" if channel is not None else "GLO_L2")}
    elif system == "Galileo":
        mapping = {1: ("GAL_E1", "GAL_E1"), 5: ("GAL_E5A", "GAL_E5A"), 7: ("GAL_E5B", "GAL_E5B"), 8: ("GAL_E5B", "GAL_E5B"), 6: ("GAL_E6", "GAL_E6")}
    elif system == "BeiDou":
        mapping = {1: ("BDS_B1C", "BDS_B1C"), 2: ("BDS_B1I", "BDS_B1I"), 5: ("BDS_B2A", "BDS_B2A"), 6: ("BDS_B3I", "BDS_B3I"), 7: ("BDS_B2I", "BDS_B2I"), 8: ("BDS_B2A", "BDS_B2A")}
    elif system == "QZSS":
        mapping = {1: ("QZS_L1CA", "QZS_L1"), 2: ("QZS_L2C", "QZS_L2"), 5: ("QZS_L5", "QZS_L5")}
    else:
        return None
    selected = mapping.get(band)
    if selected is None:
        return None
    return SignalInfo(system, selected[0], selected[1], band, channel)


def civil_to_gps_seconds(year: int, month: int, day: int, hour: int, minute: int, second: float) -> float:
    whole_second = int(math.floor(second))
    fraction = second - whole_second
    date = datetime(year, month, day, hour, minute, whole_second, tzinfo=timezone.utc)
    gps_epoch = datetime(1980, 1, 6, tzinfo=timezone.utc)
    return (date - gps_epoch).total_seconds() + fraction


def parse_epoch_v3(line: str) -> tuple[float, int, int, int, int, int, float] | None:
    parts = line[1:].split()
    if len(parts) < 8:
        return None
    try:
        year, month, day, hour, minute = (int(parts[index]) for index in range(5))
        second = float(parts[5])
        flag = int(parts[6])
        satellites = int(parts[7])
    except ValueError:
        return None
    return civil_to_gps_seconds(year, month, day, hour, minute, second), flag, satellites, year, month, day, second


def parse_epoch_v2(line: str) -> tuple[float, int, int] | None:
    if len(line) < 32 or line[0] == ">":
        return None
    try:
        year = int(line[1:3])
        year += 2000 if year < 80 else 1900
        month = int(line[4:6])
        day = int(line[7:9])
        hour = int(line[10:12])
        minute = int(line[13:15])
        second = float(line[15:26])
        flag = int(line[28:29].strip() or "0")
        satellites = int(line[29:32])
    except ValueError:
        return None
    return civil_to_gps_seconds(year, month, day, hour, minute, second), flag, satellites


@dataclass
class BaseIndex:
    base_rows: int = 0
    finite_code_rows: int = 0
    selected_rows: int = 0
    exact_streams: dict[tuple[str, int, str], list[float]] = field(default_factory=dict)
    frequency_streams: dict[tuple[str, int, str], list[float]] = field(default_factory=dict)
    satellite_streams: dict[tuple[str, int], set[str]] = field(default_factory=dict)
    signal_census: dict[str, int] = field(default_factory=dict)
    canonical_census: dict[str, int] = field(default_factory=dict)
    duplicate_epoch_frequency_rows: int = 0
    duplicate_epoch_frequency_events: int = 0
    time_min_s: float = math.nan
    time_max_s: float = math.nan
    version: float = math.nan
    observation_types: dict[str, list[str]] = field(default_factory=dict)
    glonass_channels: dict[int, int] = field(default_factory=dict)


def parse_header(lines: list[str]) -> tuple[int, float, dict[str, list[str]], list[str], dict[int, int]]:
    version = math.nan
    system_types: dict[str, list[str]] = {}
    global_types: list[str] = []
    glonass_channels: dict[int, int] = {}
    end_header = -1
    active_system: str | None = None
    expected_system_count = 0
    for index, line in enumerate(lines):
        label = line[60:].strip() if len(line) >= 60 else ""
        if index == 0:
            try:
                version = float(line[:20].strip())
            except ValueError:
                raise fail("RINEX header version is invalid")
        if label == "SYS / # / OBS TYPES":
            sys_char = line[0:1].strip()
            # RINEX 3 continuation lines leave the system character blank.
            # Keep appending to the system declared by the first line rather
            # than creating a spurious empty-system observation list.
            if sys_char and (active_system != sys_char or expected_system_count == 0):
                active_system = sys_char
                expected_system_count = int(line[3:6].strip() or "0")
                system_types.setdefault(sys_char, [])
            # Observation fields occupy columns 8..60 (zero-based 7..59);
            # exclude the column-60 label from the final three-character
            # slice on otherwise padded header lines.
            values = [
                line[position:position + 3].strip()
                for position in range(7, max(7, min(len(line), 60) - 2), 4)
            ]
            target_system = sys_char or active_system
            if target_system is None:
                raise fail("RINEX observation-type continuation has no active system")
            system_types.setdefault(target_system, []).extend(value for value in values if value)
            if len(system_types.get(target_system, [])) >= expected_system_count:
                active_system = None
                expected_system_count = 0
        elif label == "# / TYPES OF OBSERV":
            count = int(line[:6].strip() or "0")
            values = [line[position:position + 6].strip() for position in range(6, min(len(line), 60), 6)]
            global_types.extend(value for value in values if value)
            if count and len(global_types) > count:
                global_types = global_types[:count]
        elif label == "GLONASS SLOT / FRQ #":
            for match in re.finditer(r"R\s*(\d{1,2})\s+([+-]?\d+)", line[:60]):
                glonass_channels[int(match.group(1))] = int(match.group(2))
        elif label == "END OF HEADER":
            end_header = index
            break
    if end_header < 0:
        raise fail("RINEX END OF HEADER is missing")
    return end_header, version, system_types, global_types, glonass_channels


def parse_float_field(value: str) -> float:
    token = value.strip().replace("D", "E")
    if not token or token in {"0.000", "0.0", "0"}:
        return math.nan
    try:
        parsed = float(token)
    except ValueError:
        return math.nan
    return parsed if math.isfinite(parsed) else math.nan


def observation_value(record_lines: list[str], obs_index: int, rinex_version: float) -> float:
    if rinex_version >= 3.0:
        first = record_lines[0][3:] if record_lines else ""
        continuation = "".join(line for line in record_lines[1:])
        fields = first + continuation
    else:
        fields = "".join(record_lines)
    start = obs_index * 16
    return parse_float_field(fields[start:start + 14])


def selected_band_entries(entries: list[tuple[str, SignalInfo, float]], system: str) -> list[tuple[str, SignalInfo, float]]:
    # This is the source policy's primary/secondary selection, with one code
    # observation per selected band. Additional RINEX bands are not emitted by
    # the native reader unless preserve_additional_frequency_bands is enabled;
    # Phase65 leaves that option at its default false.
    primary_priority = {
        "GPS": {1: 0},
        "GLONASS": {1: 0},
        "Galileo": {1: 0},
        "BeiDou": {2: 0, 1: 1},
        "QZSS": {1: 0},
    }.get(system, {})
    secondary_priority = {
        "GPS": {2: 0, 5: 1},
        "GLONASS": {2: 0},
        "Galileo": {5: 0, 7: 1, 8: 1, 6: 2},
        "BeiDou": {7: 0, 6: 1, 5: 2, 8: 2},
        "QZSS": {2: 0, 5: 1},
    }.get(system, {})
    selected: list[tuple[str, SignalInfo, float]] = []
    for priorities in (primary_priority, secondary_priority):
        candidates = [entry for entry in entries if entry[1].band in priorities]
        if candidates:
            candidates.sort(key=lambda entry: priorities[entry[1].band])
            selected.append(candidates[0])
    return selected


def add_base_epoch(index: BaseIndex, epoch_time: float, satellite_entries: list[tuple[str, int, str, SignalInfo, float]]) -> None:
    if not satellite_entries:
        return
    grouped: dict[tuple[str, int], list[tuple[str, SignalInfo, float]]] = {}
    for system, svid, obs_type, signal, value in satellite_entries:
        index.base_rows += 1
        if not math.isfinite(value) or value <= 0.0:
            continue
        index.finite_code_rows += 1
        index.signal_census[signal.signal] = index.signal_census.get(signal.signal, 0) + 1
        index.canonical_census[signal.canonical_frequency] = index.canonical_census.get(signal.canonical_frequency, 0) + 1
        grouped.setdefault((system, svid), []).append((obs_type, signal, value))
    for (system, svid), entries in grouped.items():
        by_frequency: dict[str, int] = {}
        for _, signal, _ in entries:
            by_frequency[signal.canonical_frequency] = by_frequency.get(signal.canonical_frequency, 0) + 1
        for canonical, count in by_frequency.items():
            if count > 1:
                index.duplicate_epoch_frequency_events += 1
                index.duplicate_epoch_frequency_rows += count - 1
        for obs_type, signal, _ in selected_band_entries(entries, system):
            index.selected_rows += 1
            exact_key = (system, svid, signal.signal)
            freq_key = (system, svid, signal.canonical_frequency)
            index.exact_streams.setdefault(exact_key, []).append(epoch_time)
            index.frequency_streams.setdefault(freq_key, []).append(epoch_time)
            index.satellite_streams.setdefault((system, svid), set()).add(signal.canonical_frequency)


def parse_rinex(payload: bytes) -> BaseIndex:
    try:
        text = payload.decode("ascii")
    except UnicodeDecodeError as exc:
        raise fail(f"base RINEX is not ASCII: {exc}") from exc
    lines = text.splitlines()
    end_header, version, system_types, global_types, glonass_channels = parse_header(lines)
    index = BaseIndex(version=version, observation_types=system_types, glonass_channels=glonass_channels)
    for values in index.exact_streams.values():
        values.clear()
    cursor = end_header + 1
    if version >= 3.0:
        while cursor < len(lines):
            line = lines[cursor]
            if not line.startswith(">"):
                cursor += 1
                continue
            parsed = parse_epoch_v3(line)
            if parsed is None:
                raise fail(f"invalid RINEX 3 epoch line at {cursor + 1}")
            epoch_time, flag, satellite_count, *_ = parsed
            cursor += 1
            if flag >= 2:
                # Event records carry event lines rather than satellite rows.
                cursor += satellite_count
                continue
            for _ in range(satellite_count):
                if cursor >= len(lines):
                    raise fail("RINEX 3 ended inside an epoch")
                first = lines[cursor]
                cursor += 1
                sat_token = first[:3]
                system = system_from_rinex_char(sat_token[:1])
                if system is None:
                    raise fail(f"unsupported RINEX system {sat_token[:1]!r}")
                try:
                    svid = int(sat_token[1:3])
                except ValueError as exc:
                    raise fail(f"invalid RINEX satellite token {sat_token!r}") from exc
                obs_types = system_types.get(sat_token[:1], global_types)
                line_count = max(1, (len(obs_types) + 4) // 5)
                record_lines = [first]
                for _ in range(line_count - 1):
                    if cursor >= len(lines):
                        raise fail("RINEX 3 ended inside a satellite record")
                    record_lines.append(lines[cursor])
                    cursor += 1
                entries: list[tuple[str, int, str, SignalInfo, float]] = []
                channel = glonass_channels.get(svid) if system == "GLONASS" else None
                for obs_index, obs_type in enumerate(obs_types):
                    if not obs_type or obs_type[0] not in {"C", "P"}:
                        continue
                    value = observation_value(record_lines, obs_index, version)
                    signal = rinex_signal(system, obs_type, channel)
                    if signal is not None:
                        entries.append((system, svid, obs_type, signal, value))
                add_base_epoch(index, epoch_time, entries)
    else:
        while cursor < len(lines):
            line = lines[cursor]
            parsed = parse_epoch_v2(line)
            if parsed is None:
                cursor += 1
                continue
            epoch_time, flag, satellite_count = parsed
            cursor += 1
            if flag >= 2:
                cursor += satellite_count
                continue
            satellites: list[tuple[str, int]] = []
            sat_text = line[32:]
            while len(satellites) < satellite_count:
                for offset in range(0, len(sat_text), 3):
                    token = sat_text[offset:offset + 3]
                    if len(token) < 3:
                        continue
                    system = system_from_rinex_char(token[:1])
                    if system is None:
                        continue
                    try:
                        satellites.append((system, int(token[1:3])))
                    except ValueError:
                        continue
                    if len(satellites) == satellite_count:
                        break
                if len(satellites) < satellite_count:
                    if cursor >= len(lines):
                        raise fail("RINEX 2 ended inside satellite list")
                    sat_text = lines[cursor]
                    cursor += 1
                else:
                    break
            obs_types = global_types
            line_count = max(1, (len(obs_types) + 4) // 5)
            for system, svid in satellites:
                if cursor + line_count > len(lines):
                    raise fail("RINEX 2 ended inside a satellite record")
                record_lines = lines[cursor:cursor + line_count]
                cursor += line_count
                entries = []
                channel = glonass_channels.get(svid) if system == "GLONASS" else None
                for obs_index, obs_type in enumerate(obs_types):
                    if not obs_type or obs_type[0] not in {"C", "P"}:
                        continue
                    value = observation_value(record_lines, obs_index, version)
                    signal = rinex_signal(system, obs_type, channel)
                    if signal is not None:
                        entries.append((system, svid, obs_type, signal, value))
                add_base_epoch(index, epoch_time, entries)
    all_times = [time for values in index.exact_streams.values() for time in values]
    if all_times:
        index.time_min_s = min(all_times)
        index.time_max_s = max(all_times)
    return index


def stream_domain(times: list[float], value: float) -> bool:
    return bool(times) and times[0] - TIME_TOLERANCE_S <= value <= times[-1] + TIME_TOLERANCE_S


def classify_rows(raw: RawReport, base: BaseIndex) -> dict[str, Any]:
    counts = {
        "exact_signal_in_domain": 0,
        "same_frequency_variant_in_domain": 0,
        "out_of_domain_time": 0,
        "missing_frequency_stream": 0,
        "missing_satellite_stream": 0,
    }
    detail: dict[str, dict[str, int]] = {}
    out_domain_exact = 0
    out_domain_frequency = 0
    for row in raw.adopted_rows:
        satellite = (row.system, row.svid)
        exact_times = base.exact_streams.get((row.system, row.svid, row.signal), [])
        frequency_times = base.frequency_streams.get((row.system, row.svid, row.canonical_frequency), [])
        if exact_times:
            if stream_domain(exact_times, row.time_s):
                category = "exact_signal_in_domain"
            else:
                category = "out_of_domain_time"
                out_domain_exact += 1
        elif frequency_times:
            if stream_domain(frequency_times, row.time_s):
                category = "same_frequency_variant_in_domain"
            else:
                category = "out_of_domain_time"
                out_domain_frequency += 1
        elif satellite in base.satellite_streams:
            category = "missing_frequency_stream"
        else:
            category = "missing_satellite_stream"
        counts[category] += 1
        signal_detail = detail.setdefault(row.signal, {key: 0 for key in counts})
        signal_detail[category] += 1
    total = len(raw.adopted_rows)
    rates = {key: (value / total if total else math.nan) for key, value in counts.items()}
    return {
        "adopted_rows": total,
        "counts": counts,
        "rates": rates,
        "by_exact_signal": detail,
        "out_of_domain_exact_stream_rows": out_domain_exact,
        "out_of_domain_frequency_stream_rows": out_domain_frequency,
        "sum_check": sum(counts.values()) == total,
    }


def route_report(route: str, raw_payload: bytes, base_payload: bytes) -> dict[str, Any]:
    raw = parse_raw_csv(raw_payload)
    base = parse_rinex(base_payload)
    classification = classify_rows(raw, base)
    return {
        "raw": {
            "adopted_rows_definition": (
                "diagnostic source-filtered raw pseudorange proxy; not asserted "
                "equal to the native adopted FGO factors"
            ),
            "input_rows": raw.input_rows,
            "raw_rows": raw.raw_rows,
            "adopted_rows": len(raw.adopted_rows),
            "unsupported_signal_rows": raw.unsupported_signal_rows,
            "invalid_timing_rows": raw.invalid_timing_rows,
            "invalid_quality_rows": raw.invalid_quality_rows,
            "invalid_pseudorange_rows": raw.invalid_pseudorange_rows,
            "masked_code_rows": raw.masked_code_rows,
            "duplicate_epoch_signal_rows": raw.duplicate_epoch_signal_rows,
            "signal_census": dict(sorted(raw.signal_census.items())),
            "canonical_frequency_census": dict(sorted(raw.canonical_census.items())),
            "time_min_s": raw.time_min_s,
            "time_max_s": raw.time_max_s,
        },
        "base": {
            "selected_rows_definition": (
                "diagnostic finite RINEX code stream under the pinned source "
                "mapping; not asserted equal to native adopted FGO factors"
            ),
            "rinex_version": base.version,
            "base_rows": base.base_rows,
            "finite_code_rows": base.finite_code_rows,
            "selected_rows": base.selected_rows,
            "exact_stream_count": len(base.exact_streams),
            "canonical_frequency_stream_count": len(base.frequency_streams),
            "satellite_stream_count": len(base.satellite_streams),
            "signal_census": dict(sorted(base.signal_census.items())),
            "canonical_frequency_census": dict(sorted(base.canonical_census.items())),
            "duplicate_epoch_frequency_rows": base.duplicate_epoch_frequency_rows,
            "duplicate_epoch_frequency_events": base.duplicate_epoch_frequency_events,
            "time_min_s": base.time_min_s,
            "time_max_s": base.time_max_s,
            "glonass_channels": {str(key): value for key, value in sorted(base.glonass_channels.items())},
        },
        "classification": classification,
    }


def run_audit(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    freeze = verify_freeze()
    output_root = output_root.resolve()
    P65.reject_forbidden(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty Phase68 output: {output_root}")
    reports: dict[str, Any] = {}
    read_counts = {"raw_device_gnss": 0, "base_rinex": 0}
    output_root.mkdir(parents=True, exist_ok=True)
    try:
        for route in ROUTES:
            raw_pin = freeze["raw_inputs"][route]
            base_pin = freeze["base_inputs"][route]
            raw_path = ROOT / raw_pin["device_gnss.csv"]
            base_path = ROOT / base_pin["path"]
            raw_payload = read_once(raw_path, raw_pin["sha256"])
            read_counts["raw_device_gnss"] += 1
            base_payload = read_once(base_path, base_pin["sha256"], int(base_pin["bytes"]))
            read_counts["base_rinex"] += 1
            reports[route] = route_report(route, raw_payload, base_payload)
        result = {
            "schema_version": "smartphone-r5-phase68-base-matching-taxonomy-result.v1",
            "phase": 68,
            "execution_label": "Luna Max",
            "status": "sealed-truth-free-matching-taxonomy",
            "truth_free": True,
            "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
            "manifest": {"path": relative(MANIFEST), "sha256": sha256(MANIFEST)},
            "source_pins": {
                "phase65_source_commit": "f8cdf3b",
                "phase65_runner_sha256": P65_RUNNER_SHA256,
                "android_loader_sha256": ANDROID_LOADER_SHA256,
                "rinex_loader_sha256": RINEX_LOADER_SHA256,
                "signal_policy_sha256": SIGNAL_POLICY_SHA256,
            },
            "routes": reports,
            "native_adopted_fgo_equality_claim": False,
            "proxy_scope_note": (
                "Raw adopted and base selected counts are diagnostic proxies. "
                "This result does not claim equality to native adopted FGO "
                "factor populations."
            ),
            "aggregate": {
                "adopted_rows": sum(report["classification"]["adopted_rows"] for report in reports.values()),
                "classification_counts": {
                    key: sum(report["classification"]["counts"][key] for report in reports.values())
                    for key in next(iter(reports.values()))["classification"]["counts"]
                } if reports else {},
                "duplicate_epoch_frequency_events": sum(report["base"]["duplicate_epoch_frequency_events"] for report in reports.values()),
                "duplicate_epoch_frequency_rows": sum(report["base"]["duplicate_epoch_frequency_rows"] for report in reports.values()),
                "classification_sum_checks": all(report["classification"]["sum_check"] for report in reports.values()),
            },
            "read_accounting": {
                "single_process": True,
                "raw_device_gnss_reads": read_counts["raw_device_gnss"],
                "base_rinex_reads": read_counts["base_rinex"],
                "nav_reads": 0,
                "imu_reads": 0,
                "solver_invocations": 0,
                "truth_reads": 0,
                "mat_reads_or_generated": 0,
                "validation_holdout_reads": 0,
                "archive_reopens": 0,
                "old_phase65_or_phase66_output_reuse": False,
            },
            "gate_policy": {
                "phase67_coverage_gate_unchanged": True,
                "native_correction_authorized": False,
                "canonicalization_authorized": False,
                "interpolation_change_authorized": False,
                "zero_point_782": "not evaluated",
            },
        }
        result_path = output_root / "phase68_base_matching_taxonomy_result.json"
        atomic_json(result_path, result)
        output_manifest = {
            "schema_version": "smartphone-r5-phase68-base-matching-taxonomy-output-manifest.v1",
            "phase": 68,
            "status": "sealed-truth-free-diagnostic",
            "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
            "evaluator": {"path": relative(Path(__file__)), "sha256": sha256(Path(__file__))},
            "result": {"path": relative(result_path), "sha256": sha256(result_path), "bytes": result_path.stat().st_size},
            "raw_device_gnss_reads": read_counts["raw_device_gnss"],
            "base_rinex_reads": read_counts["base_rinex"],
            "solver_invocations": 0,
            "truth_reads": 0,
            "all_classification_sum_checks": result["aggregate"]["classification_sum_checks"],
        }
        atomic_json(output_root / "phase68_base_matching_taxonomy_output_manifest.json", output_manifest)
        return result
    except Phase68Error as exc:
        atomic_json(
            output_root / "phase68_base_matching_taxonomy_failure.json",
            {
                "schema_version": "smartphone-r5-phase68-base-matching-taxonomy-failure.v1",
                "status": "fail-closed",
                "error": str(exc),
                "read_accounting": {**read_counts, "truth_reads": 0, "solver_invocations": 0},
                "partial_output_reuse": False,
            },
        )
        raise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--run-audit", action="store_true")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        if args.verify_freeze:
            verify_freeze()
        if args.run_audit:
            result = run_audit(args.output_root)
            print(json.dumps({"status": result["status"], "routes": len(result["routes"]), "adopted_rows": result["aggregate"]["adopted_rows"], "truth_reads": result["read_accounting"]["truth_reads"]}, sort_keys=True))
        elif not args.verify_freeze:
            parser.error("one of --verify-freeze or --run-audit is required")
        return 0
    except Phase68Error as exc:
        print(f"phase68 matching taxonomy failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
