#!/usr/bin/env python3
"""Truth-free audit of satellite-specific Android raw code residuals.

Phase47 deliberately stays below the navigation/trajectory boundary.  It
opens the four frozen Pixel5 ``device_gnss.csv`` files once each, reconstructs
the Phase25 raw pseudorange in integer/Decimal arithmetic, and compares
same-epoch dual-frequency observables plus carrier-minus-code arcs.  The
command does not read truth, a navigation file, a solver output, or any
coordinate field, and it never fits or applies a correction.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field
from decimal import Decimal, InvalidOperation
import hashlib
import io
import json
import math
import os
from pathlib import Path
import statistics
import sys
import tempfile
from typing import Any, Iterable, Sequence


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase47_pixel5_raw_code_multipath_freeze_v1.json"
FREEZE_SHA256 = "bd9c3b0649b2068286de44bfd36171d09b46ff3e6e760d02067e225f5a13d5f9"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase47_pixel5_raw_code_multipath_evaluator_manifest_v1.json"
MANIFEST_SHA256 = ""
VERIFIED_MANIFEST_SHA256 = ""
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase47-pixel5-raw-code-multipath-v1"

SCHEMA = "smartphone-r5-phase47-pixel5-raw-code-multipath.v1"
FREEZE_SCHEMA = "smartphone-r5-phase47-pixel5-raw-code-multipath-freeze.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase47-pixel5-raw-code-multipath-manifest.v1"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)

SPEED_OF_LIGHT_MPS = 299_792_458.0
NS_PER_SECOND = Decimal("1000000000")
SECONDS_PER_WEEK = Decimal("604800")
NS_PER_WEEK = Decimal("604800000000000")
HALF_WEEK_SECONDS = Decimal("302400")
TIME_GAP_NS = 1_000_000_000
CMC_GAP_NS = 1_500_000_000
MIN_PSEUDORANGE_M = 10_000_000.0
MAX_PSEUDORANGE_M = 40_000_000.0
MIN_FLAGGED_P95_M = 2.0
FLAGGED_CLEAN_RATIO = 1.5
MIN_CLEAN_RETENTION = 0.80
MIN_NONCOMMON_SPREAD_M = 0.1

REQUIRED_COLUMNS = (
    "MessageType",
    "utcTimeMillis",
    "TimeNanos",
    "FullBiasNanos",
    "BiasNanos",
    "ReceivedSvTimeNanos",
    "Svid",
    "ConstellationType",
    "SignalType",
    "CarrierFrequencyHz",
)
OPTIONAL_NUMERIC_COLUMNS = (
    "TimeOffsetNanos",
    "HardwareClockDiscontinuityCount",
    "State",
    "MultipathIndicator",
    "AccumulatedDeltaRangeState",
    "AccumulatedDeltaRangeMeters",
    "ReceivedSvTimeUncertaintyNanos",
    "Cn0DbHz",
    "PseudorangeRateMetersPerSecond",
    "PseudorangeRateUncertaintyMetersPerSecond",
    "BiasUncertaintyNanos",
    "DriftNanosPerSecond",
    "DriftUncertaintyNanosPerSecond",
    "TimeUncertaintyNanos",
)

CONSTELLATION_NAMES = {
    "1": "GPS",
    "3": "GLONASS",
    "5": "BEIDOU",
    "6": "GALILEO",
    "GPS": "GPS",
    "GLONASS": "GLONASS",
    "GLO": "GLONASS",
    "BEIDOU": "BEIDOU",
    "BDS": "BEIDOU",
    "GALILEO": "GALILEO",
    "GAL": "GALILEO",
}

SIGNAL_ALIASES = {
    "GPS_L1_CA": ("GPS", "GPS_L1CA"),
    "GPS_L1C": ("GPS", "GPS_L1CA"),
    "GPS_L1CA": ("GPS", "GPS_L1CA"),
    "L1": ("", "L1"),
    "GPS_L5_Q": ("GPS", "GPS_L5"),
    "GPS_L5": ("GPS", "GPS_L5"),
    "L5": ("", "L5"),
    "GLO_G1_CA": ("GLONASS", "GLO_G1CA"),
    "GLO_G1C": ("GLONASS", "GLO_G1CA"),
    "GLO_L1": ("GLONASS", "GLO_G1CA"),
    "GAL_E1_C_P": ("GALILEO", "GAL_E1"),
    "GAL_E1_C": ("GALILEO", "GAL_E1"),
    "GAL_E1": ("GALILEO", "GAL_E1"),
    "GAL_E5A_Q": ("GALILEO", "GAL_E5A"),
    "GAL_E5A": ("GALILEO", "GAL_E5A"),
    "GAL_E5": ("GALILEO", "GAL_E5A"),
    "BDS_B1I": ("BEIDOU", "BDS_B1I"),
    "BDS_B1_I": ("BEIDOU", "BDS_B1I"),
    "BDS_B1": ("BEIDOU", "BDS_B1I"),
    "B1I": ("BEIDOU", "BDS_B1I"),
    "BDS_B1C": ("BEIDOU", "BDS_B1C"),
    "BDS_B1_C": ("BEIDOU", "BDS_B1C"),
    "BDS_B2A": ("BEIDOU", "BDS_B2A"),
    "BDS_B2A_Q": ("BEIDOU", "BDS_B2A"),
    "BDS_B2A_I": ("BEIDOU", "BDS_B2A"),
}

SIGNAL_FREQUENCIES_HZ = {
    "GPS_L1CA": 1_575_420_000.0,
    "GPS_L5": 1_176_450_000.0,
    "GAL_E1": 1_575_420_000.0,
    "GAL_E5A": 1_176_450_000.0,
    "BDS_B1I": 1_561_098_000.0,
    "BDS_B2A": 1_176_450_000.0,
}
PAIR_MAP = {
    ("GPS", "GPS_L1CA", "GPS_L5"): "GPS_L1CA-GPS_L5",
    ("GALILEO", "GAL_E1", "GAL_E5A"): "GAL_E1-GAL_E5A",
    ("BEIDOU", "BDS_B1I", "BDS_B2A"): "BDS_B1I-BDS_B2A",
}
PAIR_PRIMARY = {value: key[1] for key, value in PAIR_MAP.items()}
PAIR_SECONDARY = {value: key[2] for key, value in PAIR_MAP.items()}


class Phase47Error(ValueError):
    """Raised when the sealed raw-only audit contract cannot be proved."""


def _fail(message: str) -> Phase47Error:
    return Phase47Error(message)


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _reject_path(path: Path | str) -> None:
    lowered = str(path).lower()
    forbidden = (
        ".mat",
        "ground_truth",
        "validation",
        "holdout",
        "precomputed",
        "device_wls",
        "svposition",
        "svelevation",
        "kaggle",
        "token",
        "archive",
    )
    if any(token in lowered for token in forbidden):
        raise _fail(f"forbidden Phase47 input/output path: {path}")


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _read_bytes_once(path: Path, label: str, expected_sha256: str | None = None) -> tuple[bytes, str]:
    _reject_path(path)
    if not path.is_file():
        raise _fail(f"missing {label}: {path}")
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise _fail(f"failed to read {label}: {path}: {exc}") from exc
    digest = _sha256_bytes(payload)
    if expected_sha256 is not None and digest != expected_sha256:
        raise _fail(f"{label} hash mismatch: {digest} != {expected_sha256}")
    return payload, digest


def _load_json_once(path: Path, label: str, expected_sha256: str | None = None) -> tuple[dict[str, Any], str, int]:
    payload, digest = _read_bytes_once(path, label, expected_sha256)
    try:
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise _fail(f"invalid {label}") from exc
    if not isinstance(value, dict):
        raise _fail(f"{label} must be a JSON object")
    return value, digest, len(payload)


def _json_bytes(value: Any) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")


def _atomic_write(path: Path, payload: bytes) -> None:
    _reject_path(path)
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


def _atomic_json(path: Path, value: Any) -> bytes:
    payload = _json_bytes(value)
    _atomic_write(path, payload)
    return payload


def _parse_int(value: Any, label: str, required: bool = True) -> int | None:
    text = str(value).strip()
    if not text:
        if required:
            raise _fail(f"empty integer {label}")
        return None
    try:
        decimal = Decimal(text)
    except InvalidOperation as exc:
        raise _fail(f"invalid integer {label}: {text!r}") from exc
    if not decimal.is_finite():
        if not required:
            return None
        raise _fail(f"non-finite integer {label}: {text!r}")
    if decimal != decimal.to_integral_value():
        raise _fail(f"non-integral integer {label}: {text!r}")
    return int(decimal)


def _parse_decimal(value: Any, label: str, required: bool = True) -> Decimal | None:
    text = str(value).strip()
    if not text:
        if required:
            raise _fail(f"empty numeric {label}")
        return None
    try:
        decimal = Decimal(text)
    except InvalidOperation as exc:
        raise _fail(f"invalid numeric {label}: {text!r}") from exc
    if not decimal.is_finite():
        if not required:
            return None
        raise _fail(f"non-finite numeric {label}: {text!r}")
    return decimal


def _to_float(value: Decimal | int | float) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise _fail("non-finite numeric result")
    return result


def _median(values: Iterable[float]) -> float:
    data = [float(value) for value in values]
    if not data:
        return 0.0
    result = float(statistics.median(data))
    if not math.isfinite(result):
        raise _fail("non-finite median")
    return result


def _percentile(values: Iterable[float], fraction: float) -> float:
    data = sorted(float(value) for value in values)
    if not data:
        return 0.0
    rank = fraction * (len(data) - 1)
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return data[lower]
    alpha = rank - lower
    return data[lower] + alpha * (data[upper] - data[lower])


def _mad(values: Iterable[float], center: float | None = None) -> float:
    data = [float(value) for value in values]
    if not data:
        return 0.0
    if center is None:
        center = _median(data)
    return _median(abs(value - center) for value in data)


def _distribution(values: Iterable[float], center: float | None = None) -> dict[str, Any]:
    data = [float(value) for value in values]
    if not data:
        return {"count": 0, "median": 0.0, "mad": 0.0, "p50_abs": 0.0, "p95_abs": 0.0, "max_abs": 0.0}
    if center is None:
        center = _median(data)
    absolute = [abs(value - center) for value in data]
    return {
        "count": len(data),
        "median": _median(data),
        "mad": _mad(data, center),
        "p50_abs": _percentile(absolute, 0.50),
        "p95_abs": _percentile(absolute, 0.95),
        "max_abs": max(absolute),
    }


def _distribution_centered(values: Iterable[float]) -> dict[str, Any]:
    data = [float(value) for value in values]
    return _distribution(data, 0.0)


def _bucket(value: Decimal | None, kind: str) -> str:
    if value is None:
        return "missing"
    number = _to_float(value)
    if kind == "cn0":
        return "<20" if number < 20.0 else ("20_to_30" if number < 30.0 else ">=30")
    if kind == "multipath":
        return "0" if number == 0 else ("1" if number == 1 else "other")
    if kind == "state":
        return "valid" if number >= 0 else "invalid"
    if kind == "sv_uncert":
        return "<=10" if number <= 10.0 else ("10_to_100" if number <= 100.0 else ">100")
    if kind == "rate_uncert":
        return "<=0.5" if number <= 0.5 else ("0.5_to_2" if number <= 2.0 else ">2")
    raise _fail(f"unknown bucket kind {kind}")


def _constellation(value: Any) -> str:
    text = str(value).strip().upper()
    return CONSTELLATION_NAMES.get(text, text or "UNKNOWN")


def _signal(value: Any, constellation: str, frequency_hz: Decimal | None) -> tuple[str, str]:
    token = str(value).strip().upper()
    mapped = SIGNAL_ALIASES.get(token)
    if mapped is not None and mapped[1] not in {"L1", "L5"}:
        system, name = mapped
        if not system:
            system = constellation
        return system, name
    frequency = _to_float(frequency_hz) if frequency_hz is not None else 0.0
    candidates: tuple[tuple[str, str, float], ...] = (
        ("GPS", "GPS_L1CA", SIGNAL_FREQUENCIES_HZ["GPS_L1CA"]),
        ("GPS", "GPS_L5", SIGNAL_FREQUENCIES_HZ["GPS_L5"]),
        ("GALILEO", "GAL_E1", SIGNAL_FREQUENCIES_HZ["GAL_E1"]),
        ("GALILEO", "GAL_E5A", SIGNAL_FREQUENCIES_HZ["GAL_E5A"]),
        ("BEIDOU", "BDS_B1I", SIGNAL_FREQUENCIES_HZ["BDS_B1I"]),
        ("BEIDOU", "BDS_B2A", SIGNAL_FREQUENCIES_HZ["BDS_B2A"]),
    )
    for system, name, nominal in candidates:
        if system == constellation and abs(frequency - nominal) <= 1000.0:
            return system, name
    return constellation, token or "UNKNOWN"


@dataclass(slots=True)
class RawObservation:
    row_number: int
    utc_ms: int
    time_ns: int
    full_bias_ns: int
    bias_ns: Decimal
    time_offset_ns: Decimal
    received_sv_time_ns: Decimal
    hcdc: int
    state: int | None
    multipath: int | None
    adr_state: int | None
    adr_m: Decimal | None
    sv_time_uncertainty_ns: Decimal | None
    cn0_dbhz: Decimal | None
    pseudorange_rate_mps: Decimal | None
    pseudorange_rate_uncertainty_mps: Decimal | None
    drift_nsps: Decimal | None
    bias_uncertainty_ns: Decimal | None
    drift_uncertainty_nsps: Decimal | None
    time_uncertainty_ns: Decimal | None
    svid: int
    system: str
    signal: str
    carrier_hz: Decimal
    segment: int = -1
    pseudorange_m: float | None = None
    code_masked: bool = False
    mask_reasons: tuple[str, ...] = ()
    cmc_arc: int | None = None

    @property
    def key(self) -> tuple[str, int, str]:
        return self.system, self.svid, self.signal

    @property
    def epoch_key(self) -> tuple[int, str, int, str]:
        return self.utc_ms, self.system, self.svid, self.signal


@dataclass(slots=True)
class Epoch:
    key_ms: int
    rows: list[RawObservation] = field(default_factory=list)
    segment: int = -1
    segment_base_full_bias_ns: int | None = None


def _status_invalid(row: RawObservation) -> bool:
    if row.state is None:
        return False
    code_mask = (1 << 0) | (1 << 10)
    transmit_mask = ((1 << 7) | (1 << 15)) if row.system == "GLONASS" else ((1 << 3) | (1 << 14))
    return (row.state & code_mask) == 0 or (row.state & transmit_mask) == 0


def _adr_valid(row: RawObservation) -> bool:
    if row.adr_state is None:
        return False
    return (row.adr_state & ((1 << 1) | (1 << 2))) == 0 and (row.adr_state & 1) != 0


def _mask_row(row: RawObservation) -> tuple[bool, tuple[str, ...]]:
    reasons: list[str] = []
    if row.multipath == 1:
        reasons.append("multipath")
    if row.cn0_dbhz is not None and _to_float(row.cn0_dbhz) < 20.0:
        reasons.append("low_cnr")
    if _status_invalid(row):
        reasons.append("invalid_state")
    if row.pseudorange_m is None or row.pseudorange_m < MIN_PSEUDORANGE_M or row.pseudorange_m > MAX_PSEUDORANGE_M:
        reasons.append("code_range")
    return bool(reasons), tuple(reasons)


def _raw_pseudorange(row: RawObservation, base_full_bias_ns: int) -> float | None:
    """Phase25 long-double equation, retained as Decimal until final float."""

    clock_ns = Decimal(row.time_ns - base_full_bias_ns)
    gps_seconds = clock_ns / NS_PER_SECOND
    week = int((gps_seconds / SECONDS_PER_WEEK).to_integral_value(rounding="ROUND_FLOOR"))
    tow_rx = (clock_ns - Decimal(week) * NS_PER_WEEK - row.bias_ns - row.time_offset_ns) / NS_PER_SECOND
    tow_tx = row.received_sv_time_ns / NS_PER_SECOND
    if row.system == "GLONASS":
        # The four-route pair contract is GPS/Galileo/BeiDou.  Keep the
        # Phase25 day-of-week conversion for complete raw diagnostics.
        day = (tow_rx / Decimal("86400")).to_integral_value(rounding="ROUND_FLOOR")
        tow_tx = tow_tx + day * Decimal("86400") - Decimal("10800") + Decimal("18")
        day_offset = day - (tow_tx / Decimal("86400")).to_integral_value(rounding="ROUND_FLOOR")
        tow_tx += day_offset * Decimal("86400")
    elif row.system == "BEIDOU":
        tow_tx += Decimal("14")
    delta = tow_rx - tow_tx
    while delta > HALF_WEEK_SECONDS:
        delta -= SECONDS_PER_WEEK
    while delta < -HALF_WEEK_SECONDS:
        delta += SECONDS_PER_WEEK
    value = _to_float(delta * Decimal(str(SPEED_OF_LIGHT_MPS)))
    return value if math.isfinite(value) and value > 0.0 else None


def _parse_raw_payload(payload: bytes) -> tuple[list[Epoch], dict[str, Any]]:
    try:
        text = payload.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise _fail("device_gnss.csv is not UTF-8") from exc
    reader = csv.DictReader(io.StringIO(text, newline=""))
    if reader.fieldnames is None:
        raise _fail("device_gnss.csv has no header")
    fieldnames = [str(name).strip() for name in reader.fieldnames]
    missing = [name for name in REQUIRED_COLUMNS if name not in fieldnames]
    if missing:
        raise _fail(f"missing required raw columns: {missing}")
    optional_present = {name: name in fieldnames for name in OPTIONAL_NUMERIC_COLUMNS}
    optional_counts = {name: 0 for name in OPTIONAL_NUMERIC_COLUMNS}
    epochs_by_key: dict[int, Epoch] = {}
    order: list[int] = []
    raw_rows = 0
    non_raw_rows = 0
    unsupported_rows = 0
    stream_last_key: int | None = None
    stream_closed_keys: set[int] = set()
    repeated_epoch_key_count = 0
    nonmonotonic_epoch_key_count = 0
    for row_number, source in enumerate(reader, start=2):
        if str(source.get("MessageType", "")).strip() != "Raw":
            non_raw_rows += 1
            continue
        raw_rows += 1

        def integer(name: str, required: bool = True, default: int | None = None) -> int | None:
            value = _parse_int(source.get(name, ""), f"{name} row {row_number}", required=required)
            return default if value is None and default is not None else value

        def decimal(name: str, required: bool = True, default: Decimal | None = None) -> Decimal | None:
            value = _parse_decimal(source.get(name, ""), f"{name} row {row_number}", required=required)
            if value is not None:
                optional_counts[name] = optional_counts.get(name, 0) + 1
            return default if value is None and default is not None else value

        utc_ms = integer("utcTimeMillis")
        time_ns = integer("TimeNanos")
        full_bias_ns = integer("FullBiasNanos")
        received_sv_time_ns = integer("ReceivedSvTimeNanos")
        svid = integer("Svid")
        assert utc_ms is not None and time_ns is not None and full_bias_ns is not None
        assert received_sv_time_ns is not None and svid is not None
        if stream_last_key != utc_ms:
            if utc_ms in stream_closed_keys:
                repeated_epoch_key_count += 1
            if stream_last_key is not None and utc_ms < stream_last_key:
                nonmonotonic_epoch_key_count += 1
            if stream_last_key is not None:
                stream_closed_keys.add(stream_last_key)
            stream_last_key = utc_ms
        bias_ns = decimal("BiasNanos", required=False, default=Decimal(0))
        frequency_hz = decimal("CarrierFrequencyHz")
        assert bias_ns is not None and frequency_hz is not None
        hcdc = integer("HardwareClockDiscontinuityCount", required=False, default=0)
        assert hcdc is not None
        constellation = _constellation(source.get("ConstellationType", ""))
        system, signal = _signal(source.get("SignalType", ""), constellation, frequency_hz)
        if system == "UNKNOWN" or signal == "UNKNOWN":
            unsupported_rows += 1
        row = RawObservation(
            row_number=row_number,
            utc_ms=utc_ms,
            time_ns=time_ns,
            full_bias_ns=full_bias_ns,
            bias_ns=bias_ns,
            time_offset_ns=decimal("TimeOffsetNanos", required=False, default=Decimal(0)) or Decimal(0),
            received_sv_time_ns=Decimal(received_sv_time_ns),
            hcdc=hcdc,
            state=integer("State", required=False),
            multipath=integer("MultipathIndicator", required=False),
            adr_state=integer("AccumulatedDeltaRangeState", required=False),
            adr_m=decimal("AccumulatedDeltaRangeMeters", required=False),
            sv_time_uncertainty_ns=decimal("ReceivedSvTimeUncertaintyNanos", required=False),
            cn0_dbhz=decimal("Cn0DbHz", required=False),
            pseudorange_rate_mps=decimal("PseudorangeRateMetersPerSecond", required=False),
            pseudorange_rate_uncertainty_mps=decimal("PseudorangeRateUncertaintyMetersPerSecond", required=False),
            drift_nsps=decimal("DriftNanosPerSecond", required=False),
            bias_uncertainty_ns=decimal("BiasUncertaintyNanos", required=False),
            drift_uncertainty_nsps=decimal("DriftUncertaintyNanosPerSecond", required=False),
            time_uncertainty_ns=decimal("TimeUncertaintyNanos", required=False),
            svid=svid,
            system=system,
            signal=signal,
            carrier_hz=frequency_hz,
        )
        if utc_ms not in epochs_by_key:
            epochs_by_key[utc_ms] = Epoch(key_ms=utc_ms)
            order.append(utc_ms)
        epochs_by_key[utc_ms].rows.append(row)
    if raw_rows == 0:
        raise _fail("no MessageType=Raw rows")
    epochs = [epochs_by_key[key] for key in order]
    metadata = {
        "header_columns": fieldnames,
        "optional_present": optional_present,
        "optional_nonempty_counts": optional_counts,
        "raw_rows": raw_rows,
        "non_raw_rows": non_raw_rows,
        "unsupported_signal_rows": unsupported_rows,
        "epoch_count": len(epochs),
        "epoch_domain_coverage": 1.0,
        "repeated_epoch_key_count": repeated_epoch_key_count,
        "nonmonotonic_epoch_key_count": nonmonotonic_epoch_key_count,
        "first_seen_utc_keys": len(order),
    }
    return epochs, metadata


def _epoch_median_time_ns(epoch: Epoch) -> Decimal:
    values = sorted(Decimal(row.time_ns) for row in epoch.rows)
    middle = len(values) // 2
    if len(values) % 2:
        return values[middle]
    return (values[middle - 1] + values[middle]) / Decimal(2)


def _assign_segments(epochs: Sequence[Epoch]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Assign the Phase25 segment base and record raw clock-boundary events."""

    assignments: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []
    previous_epoch: Epoch | None = None
    previous_hcdc: int | None = None
    previous_full_bias: int | None = None
    segment = -1
    for epoch_index, epoch in enumerate(epochs):
        hcdc_values = sorted({row.hcdc for row in epoch.rows})
        full_bias_values = sorted({row.full_bias_ns for row in epoch.rows})
        hcdc = hcdc_values[0]
        gap_ns = None
        if previous_epoch is not None:
            gap_ns = _to_float(_epoch_median_time_ns(epoch) - _epoch_median_time_ns(previous_epoch))
        hcdc_change = previous_hcdc is not None and hcdc != previous_hcdc
        gap_boundary = gap_ns is not None and gap_ns > TIME_GAP_NS
        explicit_boundary = previous_epoch is None or hcdc_change or gap_boundary
        if explicit_boundary:
            segment += 1
            epoch.segment = segment
            epoch.segment_base_full_bias_ns = epoch.rows[0].full_bias_ns
        else:
            epoch.segment = segment
            assert previous_epoch is not None and previous_epoch.segment_base_full_bias_ns is not None
            epoch.segment_base_full_bias_ns = previous_epoch.segment_base_full_bias_ns
        assert epoch.segment_base_full_bias_ns is not None
        for row in epoch.rows:
            row.segment = segment
            row.pseudorange_m = _raw_pseudorange(row, epoch.segment_base_full_bias_ns)
            row.code_masked, row.mask_reasons = _mask_row(row)
        if hcdc_change:
            events.append({
                "kind": "hardware_clock_discontinuity",
                "utcTimeMillis": epoch.key_ms,
                "epoch_index": epoch_index,
                "previous_hcdc": previous_hcdc,
                "hcdc": hcdc,
                "captured_by_segment_boundary": True,
            })
        if gap_boundary:
            events.append({
                "kind": "time_nanos_gap_gt_1s",
                "utcTimeMillis": epoch.key_ms,
                "epoch_index": epoch_index,
                "gap_ns": gap_ns,
                "captured_by_segment_boundary": True,
            })
        if (previous_full_bias is not None and full_bias_values != [previous_full_bias]) or len(full_bias_values) > 1:
            captured = explicit_boundary and len(full_bias_values) == 1 and full_bias_values[0] == epoch.segment_base_full_bias_ns
            events.append({
                "kind": "full_bias_change",
                "utcTimeMillis": epoch.key_ms,
                "epoch_index": epoch_index,
                "previous_full_bias_ns": previous_full_bias,
                "full_bias_values_ns": full_bias_values,
                "captured_by_segment_boundary": bool(captured),
            })
        if len(hcdc_values) > 1:
            events.append({
                "kind": "same_epoch_hcdc_disagreement",
                "utcTimeMillis": epoch.key_ms,
                "epoch_index": epoch_index,
                "hcdc_values": hcdc_values,
                "captured_by_segment_boundary": False,
            })
        tow_values = [
            _to_float(Decimal(row.time_ns - epoch.segment_base_full_bias_ns) - row.bias_ns - row.time_offset_ns)
            for row in epoch.rows
        ]
        row_tow_values = [_to_float(Decimal(row.time_ns - row.full_bias_ns) - row.bias_ns - row.time_offset_ns) for row in epoch.rows]
        base_row_differences = [a - b for a, b in zip(tow_values, row_tow_values)]
        if any(abs(value) > 0.0 for value in base_row_differences):
            events.append({
                "kind": "base_vs_row_full_bias_difference",
                "utcTimeMillis": epoch.key_ms,
                "epoch_index": epoch_index,
                "max_abs_difference_ns": max(abs(value) for value in base_row_differences),
                "at_explicit_boundary": bool(explicit_boundary),
            })
        previous_epoch = epoch
        previous_hcdc = hcdc
        previous_full_bias = epoch.rows[0].full_bias_ns
        assignments.append({
            "epoch_index": epoch_index,
            "utcTimeMillis": epoch.key_ms,
            "segment": segment,
            "segment_base_FullBiasNanos": epoch.segment_base_full_bias_ns,
            "full_bias_values_ns": full_bias_values,
            "hcdc_values": hcdc_values,
            "time_nanos_gap_ns": gap_ns,
            "explicit_segment_boundary": bool(explicit_boundary),
            "base_vs_row_tow_difference_ns": _median(base_row_differences),
            "base_vs_row_tow_spread_ns": max(base_row_differences) - min(base_row_differences) if base_row_differences else 0.0,
            "receiver_tow_spread_ns": max(tow_values) - min(tow_values) if tow_values else 0.0,
        })
    return assignments, events


def _summary(values: Sequence[float], center: float | None = None) -> dict[str, Any]:
    return _distribution(values, center)


def _system_signal_key(row: RawObservation) -> str:
    return f"{row.system}:{row.signal}"


def _pair_rows(epochs: Sequence[Epoch]) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    pairs: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []
    duplicate_count = 0
    pair_counts: dict[str, dict[str, Any]] = {}
    for epoch_index, epoch in enumerate(epochs):
        by_sat: dict[tuple[str, int], dict[str, RawObservation]] = {}
        for row in epoch.rows:
            if row.pseudorange_m is None or row.signal in {"UNKNOWN", "L1", "L5"}:
                continue
            key = (row.system, row.svid)
            signal_map = by_sat.setdefault(key, {})
            if row.signal in signal_map:
                duplicate_count += 1
                events.append({
                    "kind": "duplicate_signal_same_epoch",
                    "utcTimeMillis": epoch.key_ms,
                    "epoch_index": epoch_index,
                    "system": row.system,
                    "svid": row.svid,
                    "signal": row.signal,
                    "row_number": row.row_number,
                })
                continue
            signal_map[row.signal] = row
        for (system, svid), signal_map in by_sat.items():
            for (pair_system, primary, secondary), pair_name in PAIR_MAP.items():
                if system != pair_system or primary not in signal_map or secondary not in signal_map:
                    continue
                left = signal_map[primary]
                right = signal_map[secondary]
                difference = float(left.pseudorange_m - right.pseudorange_m)
                f_left = _to_float(left.carrier_hz)
                f_right = _to_float(right.carrier_hz)
                if f_left <= 0.0:
                    f_left = SIGNAL_FREQUENCIES_HZ[primary]
                if f_right <= 0.0:
                    f_right = SIGNAL_FREQUENCIES_HZ[secondary]
                denom = 1.0 - (f_left / f_right) ** 2
                ionosphere = difference / denom if abs(denom) > 1e-12 else 0.0
                flagged = bool(left.code_masked or right.code_masked)
                reasons = sorted(set(left.mask_reasons + right.mask_reasons))
                item = {
                    "epoch_index": epoch_index,
                    "utcTimeMillis": epoch.key_ms,
                    "system": system,
                    "pair": pair_name,
                    "svid": svid,
                    "primary_signal": primary,
                    "secondary_signal": secondary,
                    "code_difference_m": difference,
                    "frequency_squared_ionosphere_m": ionosphere,
                    "flagged_by_existing_adapter_mask": flagged,
                    "mask_reasons": reasons,
                    "clean": not flagged,
                }
                pairs.append(item)
                state = pair_counts.setdefault(pair_name, {"epochs": 0, "satellites": set(), "clean": 0, "flagged": 0})
                state["epochs"] += 1
                state["satellites"].add(svid)
                state["clean" if not flagged else "flagged"] += 1
    for state in pair_counts.values():
        state["satellites"] = sorted(state["satellites"])
    return pairs, events, {"pair_counts": pair_counts, "duplicate_signal_count": duplicate_count}


def _pair_group_summary(pairs: Sequence[dict[str, Any]]) -> dict[str, Any]:
    groups: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for item in pairs:
        groups.setdefault((item["pair"], "all"), []).append(item)
        groups.setdefault((item["pair"], "clean" if item["clean"] else "flagged"), []).append(item)
    report: dict[str, Any] = {}
    for (pair_name, bucket), items in sorted(groups.items()):
        values = [float(item["code_difference_m"]) for item in items]
        iono = [float(item["frequency_squared_ionosphere_m"]) for item in items]
        report[f"{pair_name}:{bucket}"] = {
            "count": len(items),
            "satellites": sorted({int(item["svid"]) for item in items}),
            "code_difference_m": _distribution(values),
            "frequency_squared_ionosphere_m": _distribution(iono),
        }
    return report


def _pair_residuals_by_satellite(pairs: Sequence[dict[str, Any]]) -> dict[tuple[str, str, int], list[float]]:
    grouped: dict[tuple[str, str, int], list[float]] = {}
    for item in pairs:
        grouped.setdefault((str(item["pair"]), str(item["system"]), int(item["svid"])), []).append(float(item["code_difference_m"]))
    return grouped


def _same_epoch_noncommon(pairs: Sequence[dict[str, Any]]) -> dict[str, Any]:
    grouped: dict[tuple[int, str], list[float]] = {}
    for item in pairs:
        grouped.setdefault((int(item["epoch_index"]), str(item["pair"])), []).append(float(item["code_difference_m"]))
    centered_spreads: list[float] = []
    noncommon_values: list[float] = []
    epoch_count = 0
    for values in grouped.values():
        if len(values) < 2:
            continue
        epoch_count += 1
        center = _median(values)
        centered = [value - center for value in values]
        centered_spreads.append(max(centered) - min(centered))
        noncommon_values.extend(abs(value) for value in centered)
    return {
        "comparable_epoch_count": epoch_count,
        "within_epoch_spread_m": _distribution_centered(centered_spreads),
        "noncommon_abs_m": _distribution_centered(noncommon_values),
        "evidence_above_0_1m": bool(centered_spreads and _percentile(centered_spreads, 0.50) >= MIN_NONCOMMON_SPREAD_M),
    }


def _cmc_rows(epochs: Sequence[Epoch]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Build median-centered CMC observations and raw-only arc events."""

    by_key: dict[tuple[str, int, str], list[RawObservation]] = {}
    for epoch in epochs:
        for row in epoch.rows:
            by_key.setdefault(row.key, []).append(row)
    cmc: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []
    for (system, svid, signal), rows in sorted(by_key.items()):
        rows.sort(key=lambda row: (row.utc_ms, row.row_number))
        arc = -1
        previous: RawObservation | None = None
        previous_valid = False
        for row in rows:
            current_valid = row.pseudorange_m is not None and row.adr_m is not None and _adr_valid(row)
            gap_ns = None if previous is None else row.time_ns - previous.time_ns
            hcdc_change = previous is not None and row.hcdc != previous.hcdc
            gap_reset = gap_ns is not None and gap_ns > CMC_GAP_NS
            slip = row.adr_state is not None and (row.adr_state & ((1 << 1) | (1 << 2))) != 0
            reset = previous is None or hcdc_change or gap_reset or slip or not current_valid or not previous_valid
            if reset:
                arc += 1
            row.cmc_arc = arc
            if hcdc_change:
                events.append({"kind": "cmc_hcdc_arc_reset", "utcTimeMillis": row.utc_ms, "system": system, "svid": svid, "signal": signal, "arc": arc})
            if gap_reset:
                events.append({"kind": "cmc_gap_arc_reset", "utcTimeMillis": row.utc_ms, "system": system, "svid": svid, "signal": signal, "gap_ns": gap_ns, "arc": arc})
            if slip:
                events.append({"kind": "cmc_cycle_slip_arc_reset", "utcTimeMillis": row.utc_ms, "system": system, "svid": svid, "signal": signal, "adr_state": row.adr_state, "arc": arc})
            if current_valid:
                assert row.pseudorange_m is not None and row.adr_m is not None
                cmc_value = row.pseudorange_m - _to_float(row.adr_m)
                cmc.append({
                    "utcTimeMillis": row.utc_ms,
                    "row_number": row.row_number,
                    "epoch_key": row.utc_ms,
                    "system": system,
                    "svid": svid,
                    "signal": signal,
                    "arc": arc,
                    "cmc_m": cmc_value,
                    "clean": not row.code_masked,
                    "mask_reasons": list(row.mask_reasons),
                    "cn0_bucket": _bucket(row.cn0_dbhz, "cn0"),
                    "multipath_bucket": _bucket(Decimal(row.multipath) if row.multipath is not None else None, "multipath"),
                    "state_bucket": "missing" if row.state is None else ("invalid" if _status_invalid(row) else "valid"),
                    "sv_uncertainty_bucket": _bucket(row.sv_time_uncertainty_ns, "sv_uncert"),
                    "rate_uncertainty_bucket": _bucket(row.pseudorange_rate_uncertainty_mps, "rate_uncert"),
                    "adr_bucket": "valid" if _adr_valid(row) else ("cycle_slip" if slip else "invalid"),
                })
            if current_valid and previous_valid and previous is not None and previous.cmc_arc == arc:
                assert previous.pseudorange_m is not None and previous.adr_m is not None
                previous_cmc = previous.pseudorange_m - _to_float(previous.adr_m)
                jump = cmc_value - previous_cmc
                delta_time_s = (row.time_ns - previous.time_ns) / 1e9
                drift_mps = jump / delta_time_s if delta_time_s > 0.0 else 0.0
                if abs(jump) >= 2.0:
                    events.append({
                        "kind": "cmc_adjacent_jump",
                        "utcTimeMillis": row.utc_ms,
                        "system": system,
                        "svid": svid,
                        "signal": signal,
                        "arc": arc,
                        "jump_m": jump,
                        "drift_mps": drift_mps,
                    })
            previous = row
            previous_valid = current_valid
    return cmc, events


def _cmc_summary(cmc: Sequence[dict[str, Any]]) -> dict[str, Any]:
    grouped: dict[str, list[float]] = {}
    grouped_clean: dict[str, list[float]] = {}
    grouped_flagged: dict[str, list[float]] = {}
    centered_all: list[float] = []
    centered_by_group: dict[str, list[float]] = {}
    for item in cmc:
        key = f"{item['system']}:{item['signal']}"
        grouped.setdefault(key, []).append(float(item["cmc_m"]))
        (grouped_clean if item["clean"] else grouped_flagged).setdefault(key, []).append(float(item["cmc_m"]))
    arc_groups: dict[tuple[str, int, str, int], list[float]] = {}
    for item in cmc:
        arc_groups.setdefault((item["system"], int(item["svid"]), item["signal"], int(item["arc"])), []).append(float(item["cmc_m"]))
    for values in arc_groups.values():
        center = _median(values)
        centered = [value - center for value in values]
        centered_all.extend(centered)
    groups: dict[str, Any] = {}
    for key, values in sorted(grouped.items()):
        group_centered: list[float] = []
        for arc_key, values in arc_groups.items():
            if f"{arc_key[0]}:{arc_key[2]}" == key:
                center = _median(values)
                group_centered.extend(value - center for value in values)
        groups[key] = {
            "count": len(values),
            "satellites": sorted({int(item["svid"]) for item in cmc if f"{item['system']}:{item['signal']}" == key}),
            "arc_count": len({(int(item["svid"]), int(item["arc"])) for item in cmc if f"{item['system']}:{item['signal']}" == key}),
            "raw_cmc_m": _distribution(values),
            "arc_median_centered_cmc_m": _distribution_centered(group_centered),
            "clean_cmc_m": _distribution(grouped_clean.get(key, [])),
            "flagged_cmc_m": _distribution(grouped_flagged.get(key, [])),
        }
    return {
        "count": len(cmc),
        "groups": groups,
        "arc_median_centered_cmc_m": _distribution_centered(centered_all),
        "arc_count": len(arc_groups),
    }


def _bucket_counts(rows: Sequence[RawObservation], pairs: Sequence[dict[str, Any]], cmc: Sequence[dict[str, Any]]) -> dict[str, Any]:
    raw_dimensions = {
        "cn0_db_hz": {name: 0 for name in ("missing", "<20", "20_to_30", ">=30")},
        "multipath_indicator": {name: 0 for name in ("missing", "0", "1", "other")},
        "android_state": {name: 0 for name in ("missing", "valid", "invalid")},
        "received_sv_time_uncertainty_ns": {name: 0 for name in ("missing", "<=10", "10_to_100", ">100")},
        "pseudorange_rate_uncertainty_mps": {name: 0 for name in ("missing", "<=0.5", "0.5_to_2", ">2")},
        "adr_state": {name: 0 for name in ("missing", "valid", "cycle_slip", "invalid")},
    }
    for row in rows:
        raw_dimensions["cn0_db_hz"][_bucket(row.cn0_dbhz, "cn0")] += 1
        raw_dimensions["multipath_indicator"][_bucket(Decimal(row.multipath) if row.multipath is not None else None, "multipath")] += 1
        if row.state is None:
            state_name = "missing"
        else:
            state_name = "invalid" if _status_invalid(row) else "valid"
        raw_dimensions["android_state"][state_name] += 1
        raw_dimensions["received_sv_time_uncertainty_ns"][_bucket(row.sv_time_uncertainty_ns, "sv_uncert")] += 1
        raw_dimensions["pseudorange_rate_uncertainty_mps"][_bucket(row.pseudorange_rate_uncertainty_mps, "rate_uncert")] += 1
        if row.adr_state is None:
            adr_name = "missing"
        elif row.adr_state & ((1 << 1) | (1 << 2)):
            adr_name = "cycle_slip"
        else:
            adr_name = "valid" if row.adr_state & 1 else "invalid"
        raw_dimensions["adr_state"][adr_name] += 1
    pair_dimensions: dict[str, Any] = {}
    for dimension in ("cn0_bucket", "multipath_bucket", "state_bucket", "sv_uncertainty_bucket", "rate_uncertainty_bucket"):
        by_bucket: dict[str, list[float]] = {}
        for item in cmc:
            by_bucket.setdefault(str(item[dimension]), []).append(float(item["cmc_m"]))
        pair_dimensions[dimension] = {bucket: {"count": len(values), "cmc_m": _distribution(values)} for bucket, values in sorted(by_bucket.items())}
    pair_dimensions["existing_mask_clean_vs_flagged"] = {
        "clean": {"count": sum(1 for item in pairs if item["clean"]), "code_difference_m": _distribution([float(item["code_difference_m"]) for item in pairs if item["clean"]])},
        "flagged": {"count": sum(1 for item in pairs if not item["clean"]), "code_difference_m": _distribution([float(item["code_difference_m"]) for item in pairs if not item["clean"]])},
    }
    return {"raw_rows": raw_dimensions, "cmc_buckets": pair_dimensions}


def _time_shape(items: Sequence[dict[str, Any]], value_name: str) -> dict[str, Any]:
    ordered = sorted(items, key=lambda item: (int(item["utcTimeMillis"]), int(item.get("epoch_index", 0))))
    values = [float(item[value_name]) for item in ordered]
    if not values:
        return {"count": 0, "prefix_25": {}, "tail_25": {}, "quartiles": [], "normalized_slope": 0.0}
    quarter = max(1, int(math.ceil(len(values) * 0.25)))
    prefix = values[:quarter]
    tail = values[-quarter:]
    quartiles = []
    for index in range(4):
        start = (index * len(values)) // 4
        end = ((index + 1) * len(values)) // 4
        if end > start:
            quartiles.append({"index": index + 1, **_distribution(values[start:end])})
    x = [index / max(1, len(values) - 1) for index in range(len(values))]
    x_mean = statistics.fmean(x)
    y_mean = statistics.fmean(values)
    denominator = sum((value - x_mean) ** 2 for value in x)
    slope = 0.0 if denominator == 0.0 else sum((a - x_mean) * (b - y_mean) for a, b in zip(x, values)) / denominator
    return {
        "count": len(values),
        "prefix_25": _distribution(prefix),
        "tail_25": _distribution(tail),
        "quartiles": quartiles,
        "normalized_slope_m_per_route": slope,
    }


def _pair_factor_metrics(pairs: Sequence[dict[str, Any]]) -> dict[str, Any]:
    by_pair: dict[str, list[dict[str, Any]]] = {}
    for item in pairs:
        by_pair.setdefault(str(item["pair"]), []).append(item)
    report: dict[str, Any] = {}
    for pair_name, items in sorted(by_pair.items()):
        all_values = [float(item["code_difference_m"]) for item in items]
        clean_values = [float(item["code_difference_m"]) for item in items if item["clean"]]
        flagged_values = [float(item["code_difference_m"]) for item in items if not item["clean"]]
        all_center = _median(all_values)
        clean_p95 = _percentile([abs(value - all_center) for value in clean_values], 0.95) if clean_values else 0.0
        flagged_p95 = _percentile([abs(value - all_center) for value in flagged_values], 0.95) if flagged_values else 0.0
        ratio = flagged_p95 / clean_p95 if clean_p95 > 0.0 else (float("inf") if flagged_p95 > 0.0 else 0.0)
        sat_medians = []
        by_sat: dict[int, list[float]] = {}
        for item in items:
            by_sat.setdefault(int(item["svid"]), []).append(float(item["code_difference_m"]))
        for svid, values in sorted(by_sat.items()):
            sat_medians.append({"svid": svid, "median_m": _median(values), "count": len(values)})
        report[pair_name] = {
            "count": len(items),
            "satellites": sorted(by_sat),
            "all_code_difference_m": _distribution(all_values),
            "clean_count": len(clean_values),
            "flagged_count": len(flagged_values),
            "clean_p95_abs_about_all_median": clean_p95,
            "flagged_p95_abs_about_all_median": flagged_p95,
            "flagged_to_clean_p95_ratio": ratio if math.isfinite(ratio) else None,
            "flagged_excess_p95_m": flagged_p95 - clean_p95,
            "satellite_medians": sat_medians,
            "satellite_median_range_m": (max(item["median_m"] for item in sat_medians) - min(item["median_m"] for item in sat_medians)) if sat_medians else 0.0,
            "time_shape": _time_shape(items, "code_difference_m"),
        }
    return report


def _pair_persistence(pairs: Sequence[dict[str, Any]], threshold_m: float = MIN_FLAGGED_P95_M) -> dict[str, Any]:
    grouped: dict[tuple[str, int], list[dict[str, Any]]] = {}
    for item in pairs:
        grouped.setdefault((str(item["pair"]), int(item["svid"])), []).append(item)
    rows = []
    for (pair_name, svid), items in sorted(grouped.items()):
        center = _median(float(item["code_difference_m"]) for item in items)
        ordered = sorted(items, key=lambda item: (int(item["utcTimeMillis"]), int(item["epoch_index"])))
        current = 0
        longest = 0
        outlier_count = 0
        for item in ordered:
            if abs(float(item["code_difference_m"]) - center) >= threshold_m:
                current += 1
                outlier_count += 1
                longest = max(longest, current)
            else:
                current = 0
        rows.append({"pair": pair_name, "svid": svid, "outlier_count": outlier_count, "longest_consecutive_outlier_run": longest})
    return {"satellite_groups": rows, "persistent_group_count": sum(1 for row in rows if row["longest_consecutive_outlier_run"] >= 3)}


def _route_summary(route: str, epochs: Sequence[Epoch], metadata: dict[str, Any], file_size: int, digest: str) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    assignments, segment_events = _assign_segments(epochs)
    all_rows = [row for epoch in epochs for row in epoch.rows]
    pairs, pair_events, pair_metadata = _pair_rows(epochs)
    cmc, cmc_events = _cmc_rows(epochs)
    events = segment_events + pair_events + cmc_events
    pair_factors = _pair_factor_metrics(pairs)
    canonical_values = [float(item["code_difference_m"]) for item in pairs]
    canonical_metric = 0.0
    canonical_pair = None
    for pair_name, metric in pair_factors.items():
        excess = float(metric["flagged_excess_p95_m"])
        if canonical_pair is None or excess > canonical_metric:
            canonical_pair = pair_name
            canonical_metric = excess
    pair_coverage = {}
    for pair_name, state in sorted(pair_metadata["pair_counts"].items()):
        pair_coverage[pair_name] = {
            "pair_epochs": int(state["epochs"]),
            "dual_frequency_satellites": len(state["satellites"]),
            "satellite_ids": state["satellites"],
            "clean_pair_epochs": int(state["clean"]),
            "flagged_pair_epochs": int(state["flagged"]),
        }
    by_signal: dict[str, list[float]] = {}
    by_satellite: dict[str, list[float]] = {}
    for row in all_rows:
        if row.pseudorange_m is not None:
            by_signal.setdefault(_system_signal_key(row), []).append(row.pseudorange_m)
            by_satellite.setdefault(f"{row.system}:{row.svid}", []).append(row.pseudorange_m)
    signal_report = {
        key: {"count": len(values), "satellites": sorted({int(row.svid) for row in all_rows if _system_signal_key(row) == key}), "pseudorange_m": _distribution(values)}
        for key, values in sorted(by_signal.items())
    }
    satellite_report = {key: {"count": len(values), "pseudorange_m": _distribution(values)} for key, values in sorted(by_satellite.items())}
    assignment_spreads_m = [float(item["receiver_tow_spread_ns"]) * SPEED_OF_LIGHT_MPS / 1e9 for item in assignments]
    existing_clean_rows = sum(1 for row in all_rows if not row.code_masked)
    finite_rows = sum(1 for row in all_rows if row.pseudorange_m is not None)
    pair_total = len(pairs)
    pair_clean = sum(1 for item in pairs if item["clean"])
    row_report = {
        "raw": len(all_rows),
        "non_raw": int(metadata["non_raw_rows"]),
        "epochs": len(epochs),
        "raw_row_coverage": 1.0,
        "pseudorange_finite_rows": finite_rows,
        "unsupported_signal_rows": int(metadata["unsupported_signal_rows"]),
        "after_existing_gate_clean_rows": existing_clean_rows,
        "after_existing_gate_retention": existing_clean_rows / len(all_rows) if all_rows else 0.0,
        "pair_clean_retention": pair_clean / pair_total if pair_total else 0.0,
        "pair_clean_rows": pair_clean,
        "pair_rows": pair_total,
        "duplicate_signal_rows": int(pair_metadata["duplicate_signal_count"]),
        "repeated_epoch_key_count": int(metadata["repeated_epoch_key_count"]),
        "nonmonotonic_epoch_key_count": int(metadata["nonmonotonic_epoch_key_count"]),
    }
    segment_report = {
        "segment_count": (max((item["segment"] for item in assignments), default=-1) + 1),
        "assignments": assignments[:20],
        "assignment_truncated": len(assignments) > 20,
        "base_vs_row_tow_difference_ns": _distribution([float(item["base_vs_row_tow_difference_ns"]) for item in assignments], 0.0),
        "receiver_tow_spread_m": _distribution(assignment_spreads_m, 0.0),
        "full_bias_change_events": sum(1 for event in segment_events if event["kind"] == "full_bias_change"),
        "uncaptured_full_bias_change_events": sum(1 for event in segment_events if event["kind"] == "full_bias_change" and not event["captured_by_segment_boundary"]),
        "hcdc_change_events": sum(1 for event in segment_events if event["kind"] == "hardware_clock_discontinuity"),
        "time_gap_events": sum(1 for event in segment_events if event["kind"] == "time_nanos_gap_gt_1s"),
    }
    factor_values = [float(metric["flagged_excess_p95_m"]) for metric in pair_factors.values()]
    canonical_metric = max(factor_values, default=0.0)
    pair_rows = {
        "coverage": pair_coverage,
        "groups": pair_factors,
        "all_pairs": _pair_group_summary(pairs),
        "same_epoch_noncommon": _same_epoch_noncommon(pairs),
        "persistence": _pair_persistence(pairs),
        "canonical_factor_metric_m": canonical_metric,
        "canonical_pair": canonical_pair,
        "code_difference_overall_m": _distribution(canonical_values),
        "frequency_squared_ionosphere_overall_m": _distribution([float(item["frequency_squared_ionosphere_m"]) for item in pairs]),
    }
    report = {
        "route": route,
        "input": {"path": str(metadata["path"]), "file_size": file_size, "sha256": digest},
        "rows": row_report,
        "pseudorange": {"signals": signal_report, "satellites": satellite_report},
        "pair_diagnostics": pair_rows,
        "cmc_diagnostics": _cmc_summary(cmc),
        "quality_buckets": _bucket_counts(all_rows, pairs, cmc),
        "segment_contract": segment_report,
        "events": {"count": len(events)},
        "gate_observations": {
            "all_finite": finite_rows == len(all_rows),
            "dual_frequency_satellites_max": max((item["dual_frequency_satellites"] for item in pair_coverage.values()), default=0),
            "pair_epochs_max": max((item["pair_epochs"] for item in pair_coverage.values()), default=0),
            "canonical_factor_metric_m": canonical_metric,
            "same_epoch_noncommon_evidence": bool(pair_rows["same_epoch_noncommon"]["evidence_above_0_1m"]),
            "clean_retention": row_report["pair_clean_retention"],
        },
        "_pairs": pairs,
        "_cmc": cmc,
        "_events": events,
    }
    return report, events


def _verify_freeze(path: Path = FREEZE) -> dict[str, Any]:
    freeze, digest, _ = _load_json_once(path, "Phase47 freeze", FREEZE_SHA256 or None)
    if FREEZE_SHA256 and digest != FREEZE_SHA256:
        raise _fail(f"Phase47 freeze hash changed: {digest} != {FREEZE_SHA256}")
    if freeze.get("schema_version") != FREEZE_SCHEMA or freeze.get("status") != "frozen-before-phase47-raw-read":
        raise _fail("Phase47 freeze schema/status mismatch")
    cohort = freeze.get("cohort")
    if not isinstance(cohort, dict) or cohort.get("route_order") != list(ROUTES) or cohort.get("route_count_required", 4) != 4:
        raise _fail("Phase47 cohort route order/count changed")
    inputs = freeze.get("exact_raw_inputs")
    if not isinstance(inputs, dict) or tuple(inputs) != ROUTES:
        raise _fail("Phase47 exact raw input map changed")
    for route in ROUTES:
        item = inputs.get(route)
        if not isinstance(item, dict) or not item.get("path") or len(str(item.get("sha256", ""))) != 64 or int(item.get("file_size", 0)) <= 0:
            raise _fail(f"Phase47 raw input pin missing for {route}")
    reads = freeze.get("input_policy")
    if not isinstance(reads, dict):
        raise _fail("Phase47 input policy missing")
    required_reads = {
        "raw_device_gnss_read_count_per_route": 1,
        "single_evaluator_process": True,
        "truth_read_count": 0,
        "phase45_truth_payload_used": False,
        "phase46_payload_used": False,
        "brdc_nav_used": False,
        "archive_reopen_count": 0,
        "rematerialization_count": 0,
        "solver_rerun": False,
        "correction_fit_or_application": False,
        "validation_holdout_access": False,
        "mat_read_or_generated": False,
        "device_wls_coordinates_used": False,
        "precomputed_inference_coordinates_used": False,
    }
    if any(reads.get(key) != value for key, value in required_reads.items()):
        raise _fail("Phase47 raw-only read policy is not closed")
    equations = freeze.get("fixed_raw_equations")
    if not isinstance(equations, dict) or "long-double-equivalent" not in str(equations.get("raw_pseudorange", "")):
        raise _fail("Phase47 raw pseudorange equation is not frozen")
    pairs = freeze.get("signal_pairs")
    if not isinstance(pairs, dict) or pairs.get("minimum_dual_frequency_satellites_per_route") != 3 or pairs.get("minimum_pair_epochs_per_route") != 100:
        raise _fail("Phase47 pair coverage gate changed")
    gates = freeze.get("numeric_gates")
    if not isinstance(gates, dict):
        raise _fail("Phase47 numeric gates missing")
    expected = {
        ("route_count", "required"): 4,
        ("dual_frequency_coverage", "min_satellites_per_route"): 3,
        ("dual_frequency_coverage", "min_pair_epochs_per_route"): 100,
        ("route_metric_direction", "minimum_absolute_route_median_m"): 0.5,
        ("route_center_dispersion", "max_route_median_mad_m"): 2.0,
        ("flagged_vs_clean", "flagged_p95_min_m"): 2.0,
        ("flagged_vs_clean", "flagged_to_clean_p95_ratio_min"): 1.5,
        ("flagged_vs_clean", "clean_retention_min"): 0.8,
        ("noncommon_not_gauge_or_iono", "same_epoch_noncommon_spread_min_m"): 0.1,
    }
    for (section, key), value in expected.items():
        if gates.get(section, {}).get(key) != value:
            raise _fail(f"Phase47 gate changed: {section}.{key}")
    if gates.get("loo_stability", {}).get("all_leave_one_route_out_same_decision") is not True or gates.get("current_fgo_impact", {}).get("required") is not True:
        raise _fail("Phase47 LOO/FGO gates are not closed")
    pre = freeze.get("pre_read_assertions")
    if not isinstance(pre, dict) or any(value is not False for value in pre.values()):
        raise _fail("Phase47 pre-read assertions are not all false")
    return freeze


def _verify_manifest(freeze: dict[str, Any], path: Path = MANIFEST) -> dict[str, Any]:
    global VERIFIED_MANIFEST_SHA256
    manifest, digest, _ = _load_json_once(path, "Phase47 evaluator manifest", MANIFEST_SHA256 or None)
    if MANIFEST_SHA256 and digest != MANIFEST_SHA256:
        raise _fail(f"Phase47 evaluator manifest hash changed: {digest} != {MANIFEST_SHA256}")
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("status") != "evaluator-frozen-before-phase47-raw-read":
        raise _fail("Phase47 evaluator manifest schema/status mismatch")
    freeze_pin = manifest.get("freeze")
    if not isinstance(freeze_pin, dict) or freeze_pin.get("path") != _relative(FREEZE) or freeze_pin.get("sha256") != FREEZE_SHA256:
        raise _fail("Phase47 freeze pin mismatch in evaluator manifest")
    evaluator = manifest.get("evaluator")
    if not isinstance(evaluator, dict) or evaluator.get("operation") != "audit" or evaluator.get("native_solver_invoked") is not False or evaluator.get("single_process") is not True:
        raise _fail("Phase47 evaluator permits solver or multi-process execution")
    cohort = manifest.get("cohort")
    if not isinstance(cohort, dict) or cohort.get("route_order") != list(ROUTES) or cohort.get("raw_device_gnss_reads_per_route") != 1 or cohort.get("truth_reads_per_route") != 0:
        raise _fail("Phase47 evaluator cohort/read accounting changed")
    forbidden = manifest.get("forbidden")
    if not isinstance(forbidden, list) or not all(token in forbidden for token in ("ground_truth.csv", "solver/native binary rerun", "correction implementation", "precomputed inference coordinates")):
        raise _fail("Phase47 forbidden-input policy missing")
    for key in ("source", "test"):
        pin = evaluator.get(key)
        if not isinstance(pin, dict) or not pin.get("path") or len(str(pin.get("sha256", ""))) != 64:
            raise _fail(f"Phase47 {key} source pin missing")
        pin_path = ROOT / str(pin["path"])
        _reject_path(pin_path)
        if not pin_path.is_file():
            raise _fail(f"Phase47 {key} file missing: {pin_path}")
        actual = _sha256_bytes(pin_path.read_bytes())
        if actual != pin["sha256"]:
            raise _fail(f"Phase47 {key} hash mismatch: {actual} != {pin['sha256']}")
    VERIFIED_MANIFEST_SHA256 = digest
    return manifest


def _static_signal_contract(freeze: dict[str, Any]) -> dict[str, Any]:
    """Audit the adapter/FGO adoption contract from pinned source bytes."""

    pins = freeze["authority_pins"]["source_contracts"]
    contents: dict[str, str] = {}
    read_hashes: dict[str, str] = {}
    for name, item in pins.items():
        path = ROOT / str(item["path"])
        payload, digest = _read_bytes_once(path, f"Phase47 static source {name}", str(item["sha256"]))
        contents[name] = payload.decode("utf-8")
        read_hashes[name] = digest
    adapter = contents["android_raw_gnss_cpp"]
    fgo = contents["fgo_problems_cpp"]
    expected_signals = {
        "GPS_L1CA": "GPS_L1_CA",
        "GPS_L5": "GPS_L5_Q",
        "GAL_E1": "GAL_E1_C_P",
        "GAL_E5A": "GAL_E5A_Q",
        "BDS_B1I": "BDS_B1I",
        "BDS_B2A": "BDS_B2A",
    }
    parser_evidence = {signal: (token in adapter) for signal, token in expected_signals.items()}
    fgo_evidence = {
        "iterates_epoch_observations": "for (const auto& observation : epoch.observations)" in fgo,
        "checks_has_pseudorange": "observation.has_pseudorange" in fgo,
        "constructs_pseudorange_factor": "PseudorangeFactor factor" in fgo,
        "uses_signal_eligibility": "isEligibleFgoSignal" in fgo,
    }
    adopted = all(parser_evidence.values()) and all(fgo_evidence.values())
    return {
        "source_hashes": read_hashes,
        "adapter_parser_signal_evidence": parser_evidence,
        "existing_adapter_mask_evidence": {
            "multipath_indicator": "raw_multipath_masked" in adapter and "MultipathIndicator" in adapter,
            "low_cnr": "raw_snr_masked" in adapter and "Cn0DbHz" in adapter,
            "state": "status_code_invalid" in adapter and "PSTATE_CODE_LOCK" in adapter,
        },
        "fgo_observation_path_evidence": fgo_evidence,
        "all_declared_pair_signals_adopted_by_current_fgo": adopted,
        "interpretation": "A signal is considered adopted only through the generic current FGO observation path after adapter parsing; an unused parser token cannot support a causal claim.",
    }


def _gate(name: str, observed: Any, required: Any, passed: bool, detail: str) -> dict[str, Any]:
    return {"name": name, "observed": observed, "required": required, "passed": bool(passed), "detail": detail}


def _canonical_pair(routes: dict[str, dict[str, Any]]) -> str | None:
    pair_names = sorted({name for report in routes.values() for name in report["pair_diagnostics"]["groups"]})
    best: tuple[float, str] | None = None
    for name in pair_names:
        values = [float(report["pair_diagnostics"]["groups"].get(name, {}).get("flagged_excess_p95_m", 0.0)) for report in routes.values()]
        candidate = (sum(values), name)
        if best is None or candidate > best:
            best = candidate
    return best[1] if best else None


def _loo_for_pair(routes: dict[str, dict[str, Any]], pair_name: str | None) -> dict[str, Any]:
    if pair_name is None:
        return {"folds": [], "stable": False, "selected_pair": None}
    folds: list[dict[str, Any]] = []
    for omitted in ROUTES:
        retained = [report["pair_diagnostics"]["groups"].get(pair_name, {}) for route, report in routes.items() if route != omitted]
        flagged = sum(int(item.get("flagged_count", 0)) for item in retained)
        clean = sum(int(item.get("clean_count", 0)) for item in retained)
        excess = _median(float(item.get("flagged_excess_p95_m", 0.0)) for item in retained) if retained else 0.0
        p95 = _median(float(item.get("flagged_p95_abs_about_all_median", 0.0)) for item in retained) if retained else 0.0
        clean_p95 = _median(float(item.get("clean_p95_abs_about_all_median", 0.0)) for item in retained) if retained else 0.0
        ratio = p95 / clean_p95 if clean_p95 > 0.0 else (float("inf") if p95 > 0.0 else 0.0)
        decision = flagged > 0 and p95 >= MIN_FLAGGED_P95_M and ratio >= FLAGGED_CLEAN_RATIO
        folds.append({"omitted_route": omitted, "pair": pair_name, "flagged_count": flagged, "clean_count": clean, "flagged_p95_m": p95, "clean_p95_m": clean_p95, "ratio": ratio if math.isfinite(ratio) else None, "excess_m": excess, "decision": decision})
    return {"folds": folds, "stable": bool(folds) and len({bool(item["decision"]) for item in folds}) == 1, "selected_pair": pair_name}


def _satellite_loo_stable(routes: dict[str, dict[str, Any]], pair_name: str | None) -> bool:
    if pair_name is None:
        return False
    directions: list[int] = []
    for report in routes.values():
        group = report["pair_diagnostics"]["groups"].get(pair_name, {})
        medians = [float(item["median_m"]) for item in group.get("satellite_medians", [])]
        if len(medians) < 3:
            return False
        for index in range(len(medians)):
            remain = medians[:index] + medians[index + 1:]
            directions.append(1 if _median(remain) >= 0.0 else -1)
    return bool(directions) and len(set(directions)) == 1


def _aggregate(routes: dict[str, dict[str, Any]], static_contract: dict[str, Any]) -> dict[str, Any]:
    route_metric = {route: float(report["pair_diagnostics"]["canonical_factor_metric_m"]) for route, report in routes.items()}
    pair_name = _canonical_pair(routes)
    pair_medians: dict[str, list[float]] = {}
    for report in routes.values():
        for name, group in report["pair_diagnostics"]["groups"].items():
            pair_medians.setdefault(name, []).append(float(group.get("all_code_difference_m", {}).get("median", 0.0)))
    pairwise = []
    names = list(routes)
    for i, left in enumerate(names):
        for right in names[i + 1:]:
            pairwise.append({"route_a": left, "route_b": right, "canonical_metric_distance_m": abs(route_metric[left] - route_metric[right])})
    return {
        "route_count": len(routes),
        "canonical_factor_pair": pair_name,
        "route_canonical_metric_m": route_metric,
        "route_canonical_metric_median_m": _median(route_metric.values()),
        "route_canonical_metric_mad_m": _mad(route_metric.values()),
        "route_canonical_metric_range_m": (max(route_metric.values()) - min(route_metric.values())) if route_metric else 0.0,
        "pairwise_route_median_distances": pairwise,
        "pair_code_difference_route_medians_m": {name: [_median(values)] for name, values in sorted(pair_medians.items())},
        "static_signal_contract": static_contract,
        "phase13_policy": "Pixel7pro single-route upstream-quality candidate regressed in Phase13; it is not re-proposed here.",
    }


def _strip_internal(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: _strip_internal(item) for key, item in value.items() if not str(key).startswith("_")}
    if isinstance(value, list):
        return [_strip_internal(item) for item in value]
    return value


def _audit(freeze: dict[str, Any], manifest: dict[str, Any], output_root: Path) -> dict[str, Any]:
    _reject_path(output_root)
    if output_root.exists():
        raise _fail(f"Phase47 one-shot output already exists: {output_root}")
    static_contract = _static_signal_contract(freeze)
    input_map = freeze["exact_raw_inputs"]
    route_reports: dict[str, dict[str, Any]] = {}
    all_events: list[dict[str, Any]] = []
    read_accounting: dict[str, Any] = {
        "single_process": True,
        "raw_device_gnss_reads": {},
        "raw_device_gnss_read_count_total": 0,
        "truth_reads": 0,
        "phase45_truth_payload_reads": 0,
        "phase46_payload_reads": 0,
        "brdc_nav_reads": 0,
        "archive_reopens": 0,
        "rematerializations": 0,
        "solver_runs": 0,
        "source_static_reads": len(static_contract["source_hashes"]),
    }
    for route in ROUTES:
        item = input_map[route]
        path = ROOT / str(item["path"])
        payload, digest = _read_bytes_once(path, f"raw device_gnss {route}", str(item["sha256"]))
        if len(payload) != int(item["file_size"]):
            raise _fail(f"raw device_gnss byte count mismatch for {route}")
        epochs, metadata = _parse_raw_payload(payload)
        del payload
        metadata["path"] = path
        report, events = _route_summary(route, epochs, metadata, int(item["file_size"]), digest)
        report["input"]["expected_sha256"] = str(item["sha256"])
        report["input"]["expected_file_size"] = int(item["file_size"])
        route_reports[route] = report
        for event in events:
            event["route"] = route
        all_events.extend(events)
        read_accounting["raw_device_gnss_reads"][route] = 1
        read_accounting["raw_device_gnss_read_count_total"] += 1

    aggregate = _aggregate(route_reports, static_contract)
    canonical_pair = aggregate["canonical_factor_pair"]
    canonical_metrics = [float(report["pair_diagnostics"]["canonical_factor_metric_m"]) for report in route_reports.values()]
    positive_direction = bool(canonical_metrics) and all(abs(value) >= 0.5 for value in canonical_metrics) and len({value >= 0.0 for value in canonical_metrics}) == 1
    pair_coverages = [
        max((int(item["dual_frequency_satellites"]) for item in report["pair_diagnostics"]["coverage"].values()), default=0)
        for report in route_reports.values()
    ]
    pair_epoch_coverages = [
        max((int(item["pair_epochs"]) for item in report["pair_diagnostics"]["coverage"].values()), default=0)
        for report in route_reports.values()
    ]
    pair_groups = [report["pair_diagnostics"]["groups"].get(canonical_pair, {}) for report in route_reports.values()] if canonical_pair else []
    flagged_p95_ok = bool(pair_groups) and all(float(item.get("flagged_p95_abs_about_all_median", 0.0)) >= MIN_FLAGGED_P95_M for item in pair_groups)
    ratio_ok = bool(pair_groups) and all(float(item.get("flagged_to_clean_p95_ratio") or 0.0) >= FLAGGED_CLEAN_RATIO for item in pair_groups)
    pair_clean_retention = []
    for report in route_reports.values():
        coverage = report["pair_diagnostics"]["coverage"]
        total = sum(int(item["pair_epochs"]) for item in coverage.values())
        clean = sum(int(item["clean_pair_epochs"]) for item in coverage.values())
        pair_clean_retention.append(clean / total if total else 0.0)
    clean_retention_ok = bool(pair_clean_retention) and min(pair_clean_retention) >= MIN_CLEAN_RETENTION
    loo = _loo_for_pair(route_reports, canonical_pair)
    noncommon_ok = all(bool(report["pair_diagnostics"]["same_epoch_noncommon"]["evidence_above_0_1m"]) for report in route_reports.values())
    route_mad_ok = float(aggregate["route_canonical_metric_mad_m"]) <= 2.0
    satellite_dispersion = [
        float(group.get("satellite_median_range_m", 0.0))
        for group in pair_groups
    ]
    satellite_dispersion_ok = bool(satellite_dispersion) and all(value >= 0.5 for value in satellite_dispersion)
    satellite_loo_ok = _satellite_loo_stable(route_reports, canonical_pair)
    fgo_impact = bool(static_contract["all_declared_pair_signals_adopted_by_current_fgo"]) and all(
        int(report["pair_diagnostics"]["coverage"].get(canonical_pair, {}).get("clean_pair_epochs", 0)) > 0
        for report in route_reports.values()
    ) if canonical_pair else False
    gate_rows: list[dict[str, Any]] = [
        _gate("route_count", len(route_reports), 4, len(route_reports) == 4 and tuple(route_reports) == ROUTES, "exact four pinned Pixel5 routes"),
        _gate("raw_input_identity", all(report["input"]["sha256"] == report["input"]["expected_sha256"] and report["input"]["file_size"] == report["input"]["expected_file_size"] for report in route_reports.values()), True, all(report["input"]["sha256"] == report["input"]["expected_sha256"] and report["input"]["file_size"] == report["input"]["expected_file_size"] for report in route_reports.values()), "byte count and SHA-256 checked on the one read"),
        _gate("raw_domain_coverage", min((float(report["rows"]["raw_row_coverage"]) for report in route_reports.values()), default=0.0), 1.0, all(report["rows"]["raw_row_coverage"] == 1.0 for report in route_reports.values()), "every raw row counted once"),
        _gate("all_finite_raw_pseudorange", all(bool(report["gate_observations"]["all_finite"]) for report in route_reports.values()), True, all(bool(report["gate_observations"]["all_finite"]) for report in route_reports.values()), "all eligible raw rows produce finite in-contract P"),
        _gate("dual_frequency_coverage_satellites", min(pair_coverages, default=0), 3, min(pair_coverages, default=0) >= 3, "at least three pair-supported satellites per route"),
        _gate("dual_frequency_coverage_epochs", min(pair_epoch_coverages, default=0), 100, min(pair_epoch_coverages, default=0) >= 100, "at least 100 valid same-epoch pair epochs per route"),
        _gate("route_metric_same_direction", canonical_metrics, "same sign and abs >=0.5m", positive_direction, "flagged-minus-clean p95 excess route metric"),
        _gate("route_center_dispersion", aggregate["route_canonical_metric_mad_m"], 2.0, route_mad_ok, "route metric MAD is bounded by the frozen route-center gate"),
        _gate("flagged_p95_material", [float(item.get("flagged_p95_abs_about_all_median", 0.0)) for item in pair_groups], 2.0, flagged_p95_ok, "flagged pair residual p95"),
        _gate("flagged_clean_p95_ratio", [item.get("flagged_to_clean_p95_ratio") for item in pair_groups], 1.5, ratio_ok, "flagged versus clean p95 ratio"),
        _gate("clean_retention", min(pair_clean_retention, default=0.0), 0.8, clean_retention_ok, "post-existing-mask pair retention"),
        _gate("leave_one_route_out_stability", loo["folds"], True, bool(loo["stable"]), "same pair/threshold decision in every LOO fold"),
        _gate("current_fgo_used_rows_impacted", fgo_impact, True, fgo_impact, "static FGO adoption and eligible pair rows in all routes"),
        _gate("same_epoch_noncommon_mode", [bool(report["pair_diagnostics"]["same_epoch_noncommon"]["evidence_above_0_1m"]) for report in route_reports.values()], True, noncommon_ok, "non-common satellite spread over frozen 0.1m floor"),
        _gate("satellite_dispersion", satellite_dispersion, 0.5, satellite_dispersion_ok, "satellite median dispersion for signal-bias interpretation"),
        _gate("leave_one_satellite_out_stability", satellite_loo_ok, True, satellite_loo_ok, "satellite-out direction stability"),
    ]
    passed = all(bool(row["passed"]) for row in gate_rows)
    if passed:
        strongest = "Satellite-specific flagged code residual is directionally consistent, materially worse than clean, non-common-mode, route/satellite stable, and statically adopted by FGO."
        next_factor = None
        status = "go-raw-code-multipath-factor-auditable"
        deployable = "raw-only existing-mask bucket mechanism; no correction implementation authorized"
    elif not canonical_pair or min(pair_coverages, default=0) < 3 or min(pair_epoch_coverages, default=0) < 100:
        strongest = "The frozen dual-frequency raw-code population is insufficient for a four-route satellite-specific identifiability claim."
        next_factor = "raw Android per-satellite ReceivedSvTimeUncertaintyNanos/code-tracking residual"
        status = "no-go-raw-code-multipath-not-identifiable-or-coverage-failure"
        deployable = None
    elif not flagged_p95_ok or not ratio_ok:
        strongest = "Existing adapter quality flags do not separate a materially worse satellite-specific code population from clean pair residuals under the frozen 2m/1.5x gate."
        next_factor = "raw Android per-satellite ReceivedSvTimeUncertaintyNanos/code-tracking residual"
        status = "no-go-raw-code-multipath-not-materially-identifiable"
        deployable = None
    elif not noncommon_ok:
        strongest = "The pair residual structure is common-mode or ionosphere-confounded rather than a reproducible satellite-specific multipath signature."
        next_factor = "raw Android per-satellite ReceivedSvTimeUncertaintyNanos/code-tracking residual"
        status = "no-go-raw-code-multipath-common-mode-or-ionosphere-confounded"
        deployable = None
    else:
        strongest = "The satellite-specific code metric is not stable under the frozen route/satellite leave-out gates or is not demonstrably used by current FGO."
        next_factor = "raw Android per-satellite ReceivedSvTimeUncertaintyNanos/code-tracking residual"
        status = "no-go-raw-code-multipath-not-stable-or-not-adopted"
        deployable = None

    result = {
        "schema_version": SCHEMA,
        "phase": 47,
        "status": status,
        "mode": "raw-device-gnss-code-propagation-multipath-audit-only",
        "freeze": {"path": _relative(FREEZE), "sha256": FREEZE_SHA256 or _sha256_bytes(_json_bytes(freeze))},
        "evaluator_manifest": {"path": _relative(MANIFEST), "sha256": MANIFEST_SHA256 or VERIFIED_MANIFEST_SHA256},
        "read_accounting": read_accounting,
        "routes": {route: _strip_internal(report) for route, report in route_reports.items()},
        "aggregate": _strip_internal(aggregate),
        "gates": {"all_passed": passed, "rows": gate_rows},
        "loo_diagnostic": loo,
        "decision": {
            "audit_only": True,
            "correction_authorized": False,
            "solver_rerun": False,
            "truth_derived_bias_or_correction": False,
            "strongest_finding": strongest,
            "next_single_raw_physical_factor": next_factor,
            "deployable_mechanism_if_go": deployable,
            "zero_point_782_reachability": "not evaluated without truth",
            "phase13_upstream_quality_candidate_reproposed": False,
            "existing_multipath_indicator_mask_reimplemented": False,
        },
        "events": {"count": len(all_events), "table": "phase47_pixel5_raw_code_multipath.events.json"},
    }
    output_root.mkdir(parents=True, exist_ok=True)
    route_table = {"schema_version": SCHEMA + ".routes", "routes": result["routes"]}
    event_table = {"schema_version": SCHEMA + ".events", "events": _strip_internal(all_events)}
    routes_path = output_root / "phase47_pixel5_raw_code_multipath.routes.json"
    events_path = output_root / "phase47_pixel5_raw_code_multipath.events.json"
    routes_bytes = _atomic_json(routes_path, route_table)
    events_bytes = _atomic_json(events_path, event_table)
    result["output_artifacts"] = {
        _relative(routes_path): {"bytes": len(routes_bytes), "sha256": _sha256_bytes(routes_bytes)},
        _relative(events_path): {"bytes": len(events_bytes), "sha256": _sha256_bytes(events_bytes)},
    }
    result_path = output_root / "phase47_pixel5_raw_code_multipath.json"
    result_bytes = _atomic_json(result_path, result)
    artifacts = {
        _relative(result_path): {"bytes": len(result_bytes), "sha256": _sha256_bytes(result_bytes)},
        _relative(routes_path): {"bytes": len(routes_bytes), "sha256": _sha256_bytes(routes_bytes)},
        _relative(events_path): {"bytes": len(events_bytes), "sha256": _sha256_bytes(events_bytes)},
    }
    output_manifest = {
        "schema_version": SCHEMA + ".output-manifest",
        "status": "atomic-publish-complete",
        "phase": 47,
        "freeze_sha256": result["freeze"]["sha256"],
        "evaluator_manifest_sha256": result["evaluator_manifest"]["sha256"],
        "read_accounting": read_accounting,
        "artifacts": artifacts,
    }
    output_manifest_path = output_root / "phase47_pixel5_raw_code_multipath.manifest.json"
    manifest_bytes = _atomic_json(output_manifest_path, output_manifest)
    result["output_artifacts"][ _relative(output_manifest_path)] = {"bytes": len(manifest_bytes), "sha256": _sha256_bytes(manifest_bytes)}
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        freeze = _verify_freeze()
        manifest = _verify_manifest(freeze)
        _audit(freeze, manifest, args.output)
    except Phase47Error as exc:
        print(f"phase47: {exc}", file=sys.stderr)
        return 2
    print(f"phase47: completed raw code/multipath audit at {_relative(args.output)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
