#!/usr/bin/env python3
"""Truth-free Phase55 audit of Android ADR uncertainty.

This evaluator reads each frozen Pixel5 raw ``device_gnss.csv`` exactly once
in one process.  It forms the existing ordinary TDCP closure residual and
tests its relationship with the source-exact per-row
``AccumulatedDeltaRangeUncertaintyMeters`` value.  The uncertainty pair is
``sqrt(u_prev_m**2 + u_current_m**2)`` with coefficient one and no cap.  Raw
input is the only physical input; no truth, navigation, coordinates, solver,
trajectory, or prior metric payload is opened.
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
import tempfile
from typing import Any, Iterable, Sequence


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase55_pixel5_adr_uncertainty_freeze_v1.json"
FREEZE_SHA256 = "cc303962265d5814f613a06db793e181ebaf5a9a2727d7903ef31ee65741cf2a"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase55_pixel5_adr_uncertainty_evaluator_manifest_v1.json"
MANIFEST_SHA256 = ""
VERIFIED_MANIFEST_SHA256 = ""
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase55-pixel5-adr-uncertainty-v1"

SCHEMA = "smartphone-r5-phase55-pixel5-adr-uncertainty.v1"
FREEZE_SCHEMA = "smartphone-r5-phase55-pixel5-adr-uncertainty-freeze.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase55-pixel5-adr-uncertainty-manifest.v1"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)

SPEED_OF_LIGHT_MPS = 299_792_458.0
NS_PER_SECOND = Decimal("1000000000")
NS_PER_WEEK = Decimal("604800000000000")
SECONDS_PER_WEEK = Decimal("604800")
HALF_WEEK = Decimal("302400")
TIME_GAP_NS = 1_000_000_000
PAIR_MAX_NS = 1_500_000_000
MIN_P = 10_000_000.0
MAX_P = 40_000_000.0

ADR_VALID = 1 << 0
ADR_RESET = 1 << 1
ADR_CYCLE_SLIP = 1 << 2
ADR_HALF_CYCLE_RESOLVED = 1 << 3
ADR_HALF_CYCLE_REPORTED = 1 << 4

MIN_ORDINARY_PAIRS = 1_000
MIN_UNCERTAINTY_PAIRS = 1_000
MIN_SATELLITES = 5
MIN_CURRENT_TDCP_OVERLAP = 100
MIN_UNCERTAINTY_FRACTION = 0.30
MIN_POPULATED_BINS = 3
MIN_PAIRS_PER_BIN = 50
MIN_SPEARMAN = 0.20
MIN_QUANTILE_EXCESS_M = 0.02
MIN_QUANTILE_RATIO = 1.25
MIN_LOO_EXCESS_M = 0.01
MIN_LOO_RATIO = 1.10
MIN_CALIBRATION_COVERAGE = 0.05
MAX_CALIBRATION_COVERAGE = 0.95
MIN_NORMALIZED_MEDIAN = 0.05
MAX_NORMALIZED_MEDIAN = 20.0

FIXED_BINS = (
    ("<=0.01m", None, 0.01),
    ("0.01_to_0.1m", 0.01, 0.1),
    ("0.1_to_1m", 0.1, 1.0),
    (">1m", 1.0, None),
)

REQUIRED_COLUMNS = (
    "MessageType", "utcTimeMillis", "TimeNanos", "FullBiasNanos", "BiasNanos",
    "ReceivedSvTimeNanos", "Svid", "ConstellationType", "SignalType",
    "CarrierFrequencyHz", "PseudorangeRateMetersPerSecond",
    "AccumulatedDeltaRangeState", "AccumulatedDeltaRangeMeters",
)
OPTIONAL_COLUMNS = (
    "TimeOffsetNanos", "HardwareClockDiscontinuityCount", "State",
    "MultipathIndicator", "Cn0DbHz", "PseudorangeRateUncertaintyMetersPerSecond",
    "AccumulatedDeltaRangeUncertaintyMeters", "BiasUncertaintyNanos",
    "DriftNanosPerSecond", "TimeUncertaintyNanos", "CarrierCycles", "CarrierPhase",
    "ReceivedSvTimeUncertaintyNanos", "FullInterSignalBiasNanos",
    "SatelliteInterSignalBiasNanos",
)

CONSTELLATIONS = {
    "1": "GPS", "3": "GLONASS", "5": "BEIDOU", "6": "GALILEO",
    "GPS": "GPS", "GLONASS": "GLONASS", "GLO": "GLONASS",
    "BEIDOU": "BEIDOU", "BDS": "BEIDOU", "GALILEO": "GALILEO", "GAL": "GALILEO",
}
SIGNAL_MAP = {
    "GPS_L1_CA": "GPS_L1CA", "GPS_L1C": "GPS_L1CA", "GPS_L1CA": "GPS_L1CA",
    "GPS_L5_Q": "GPS_L5", "GPS_L5": "GPS_L5",
    "GLO_G1_CA": "GLO_G1CA", "GLO_G1C": "GLO_G1CA", "GLO_L1": "GLO_G1CA",
    "GAL_E1_C_P": "GAL_E1", "GAL_E1_C": "GAL_E1", "GAL_E1": "GAL_E1",
    "GAL_E5A_Q": "GAL_E5A", "GAL_E5A": "GAL_E5A", "GAL_E5": "GAL_E5A",
    "BDS_B1I": "BDS_B1I", "BDS_B1_I": "BDS_B1I", "BDS_B1": "BDS_B1I", "B1I": "BDS_B1I",
    "BDS_B1C": "BDS_B1C", "BDS_B1_C": "BDS_B1C",
}
FREQUENCIES = {
    "GPS_L1CA": 1_575_420_000.0, "GPS_L5": 1_176_450_000.0,
    "GLO_G1CA": 1_602_000_000.0, "GAL_E1": 1_575_420_000.0,
    "GAL_E5A": 1_176_450_000.0, "BDS_B1I": 1_561_098_000.0,
    "BDS_B2A": 1_176_450_000.0,
}


class Phase55Error(ValueError):
    """Raised when the immutable Phase55 contract is violated."""


def _fail(message: str) -> Phase55Error:
    return Phase55Error(message)


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _reject_path(path: Path | str) -> None:
    lowered = str(path).lower()
    forbidden = (
        ".mat", "ground_truth", "/truth", "validation", "holdout", "precomputed",
        "device_wls", "svposition", "svelevation", "archive", "kaggle", "token",
    )
    if any(token in lowered for token in forbidden):
        raise _fail(f"forbidden Phase55 input/output path: {path}")


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _read_bytes_once(path: Path, label: str, expected_sha256: str | None = None) -> tuple[bytes, str]:
    _reject_path(path)
    if not path.is_file():
        raise _fail(f"missing {label}: {path}")
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise _fail(f"failed to read {label}: {exc}") from exc
    digest = _sha256_bytes(payload)
    if expected_sha256 is not None and digest != expected_sha256:
        raise _fail(f"{label} hash mismatch: {digest} != {expected_sha256}")
    return payload, digest


def _load_json_once(path: Path, label: str, expected_sha256: str | None = None) -> tuple[dict[str, Any], str]:
    payload, digest = _read_bytes_once(path, label, expected_sha256)
    try:
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise _fail(f"invalid {label}") from exc
    if not isinstance(value, dict):
        raise _fail(f"{label} must be a JSON object")
    return value, digest


def _json_bytes(value: Any) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()


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
        raise _fail(f"invalid integer {label}") from exc
    if not decimal.is_finite():
        if required:
            raise _fail(f"non-finite integer {label}")
        return None
    if decimal != decimal.to_integral_value():
        raise _fail(f"non-integral integer {label}")
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
        raise _fail(f"invalid numeric {label}") from exc
    if not decimal.is_finite():
        if required:
            raise _fail(f"non-finite numeric {label}")
        return None
    return decimal


def _float(value: Decimal | int | float) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise _fail("non-finite result")
    return result


def _median(values: Iterable[float]) -> float:
    data = [float(value) for value in values]
    return float(statistics.median(data)) if data else 0.0


def _percentile(values: Iterable[float], fraction: float) -> float:
    data = sorted(float(value) for value in values)
    if not data:
        return 0.0
    rank = fraction * (len(data) - 1)
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    return data[lower] if lower == upper else data[lower] + (rank - lower) * (data[upper] - data[lower])


def _mad(values: Iterable[float], center: float | None = None) -> float:
    data = [float(value) for value in values]
    if not data:
        return 0.0
    actual = _median(data) if center is None else center
    return _median(abs(value - actual) for value in data)


def _distribution(values: Iterable[float], center: float = 0.0) -> dict[str, Any]:
    data = [float(value) for value in values if math.isfinite(float(value))]
    absolute = [abs(value - center) for value in data]
    return {
        "count": len(data), "median": _median(data), "mad": _mad(data, center),
        "p50_abs": _percentile(absolute, 0.50), "p95_abs": _percentile(absolute, 0.95),
        "max_abs": max(absolute) if absolute else 0.0,
    }


def _rank(values: Sequence[float]) -> list[float]:
    ordered = sorted(enumerate(values), key=lambda item: item[1])
    ranks = [0.0] * len(values)
    index = 0
    while index < len(ordered):
        end = index + 1
        while end < len(ordered) and ordered[end][1] == ordered[index][1]:
            end += 1
        rank = (index + end - 1) / 2.0 + 1.0
        for position in range(index, end):
            ranks[ordered[position][0]] = rank
        index = end
    return ranks


def _spearman(x: Sequence[float], y: Sequence[float]) -> float:
    if len(x) != len(y) or len(x) < 2:
        return 0.0
    rx, ry = _rank(x), _rank(y)
    mx, my = statistics.fmean(rx), statistics.fmean(ry)
    dx = math.sqrt(sum((value - mx) ** 2 for value in rx))
    dy = math.sqrt(sum((value - my) ** 2 for value in ry))
    if dx == 0.0 or dy == 0.0:
        return 0.0
    return sum((a - mx) * (b - my) for a, b in zip(rx, ry)) / (dx * dy)


def _normalise_header(value: str) -> str:
    return "".join(character.lower() for character in str(value).strip() if character.isalnum())


def _system(value: Any) -> str:
    text = str(value).strip().upper()
    return CONSTELLATIONS.get(text, text or "UNKNOWN")


def _signal(value: Any, system: str, frequency: Decimal | None) -> str:
    token = str(value).strip().upper().replace(" ", "_")
    if token in SIGNAL_MAP:
        return SIGNAL_MAP[token]
    freq = _float(frequency) if frequency is not None else 0.0
    for name, nominal in FREQUENCIES.items():
        expected = "GPS" if name.startswith("GPS") else ("GALILEO" if name.startswith("GAL") else ("GLONASS" if name.startswith("GLO") else "BEIDOU"))
        if expected == system and abs(freq - nominal) <= 1000.0:
            return name
    return token or "UNKNOWN"


def _adr_class(value: int | None) -> str:
    if value is None:
        return "missing"
    if not (value & ADR_VALID):
        return "invalid"
    if value & ADR_RESET:
        return "reset"
    if value & ADR_CYCLE_SLIP:
        return "cycle_slip"
    if value & ADR_HALF_CYCLE_RESOLVED and value & ADR_HALF_CYCLE_REPORTED:
        return "valid_resolved_reported"
    if value & ADR_HALF_CYCLE_RESOLVED:
        return "valid_resolved"
    if value & ADR_HALF_CYCLE_REPORTED:
        return "valid_reported_unresolved"
    return "valid_unresolved"


@dataclass(slots=True)
class Row:
    row_number: int
    utc_ms: int
    time_ns: int
    full_bias_ns: int
    bias_ns: Decimal
    offset_ns: Decimal
    received_sv_time_ns: Decimal
    rate_mps: Decimal | None
    rate_uncertainty_mps: Decimal | None
    adr_state: int | None
    adr_m: Decimal | None
    adr_uncertainty_m: Decimal | None
    state: int | None
    multipath: int | None
    cn0: Decimal | None
    hcdc: int
    svid: int
    system: str
    signal: str
    frequency_hz: Decimal
    segment: int = -1
    pseudorange_m: float | None = None
    code_masked: bool = False
    adr_masked: bool = False

    @property
    def sat_signal(self) -> tuple[str, int, str]:
        return self.system, self.svid, self.signal


def _status_valid(row: Row) -> bool:
    if row.state is None:
        return False
    code_mask = (1 << 0) | (1 << 10)
    transmit_mask = ((1 << 7) | (1 << 15)) if row.system == "GLONASS" else ((1 << 3) | (1 << 14))
    return (row.state & code_mask) == code_mask and (row.state & transmit_mask) == transmit_mask


def _adr_unflagged(row: Row) -> bool:
    return row.adr_state is not None and bool(row.adr_state & ADR_VALID) and not bool(row.adr_state & (ADR_RESET | ADR_CYCLE_SLIP))


def _raw_pseudorange(row: Row, base_full_bias_ns: int) -> float | None:
    clock_ns = Decimal(row.time_ns - base_full_bias_ns)
    gps_seconds = clock_ns / NS_PER_SECOND
    week = int((gps_seconds / SECONDS_PER_WEEK).to_integral_value(rounding="ROUND_FLOOR"))
    tow_rx = (clock_ns - Decimal(week) * NS_PER_WEEK - row.bias_ns - row.offset_ns) / NS_PER_SECOND
    tow_tx = row.received_sv_time_ns / NS_PER_SECOND
    if row.system == "BEIDOU":
        tow_tx += Decimal(14)
    elif row.system == "GLONASS":
        day = (tow_rx / Decimal(86400)).to_integral_value(rounding="ROUND_FLOOR")
        tow_tx = tow_tx + day * Decimal(86400) - Decimal(10800) + Decimal(18)
        day_offset = day - (tow_tx / Decimal(86400)).to_integral_value(rounding="ROUND_FLOOR")
        tow_tx += day_offset * Decimal(86400)
    delta = tow_rx - tow_tx
    while delta > HALF_WEEK:
        delta -= SECONDS_PER_WEEK
    while delta < -HALF_WEEK:
        delta += SECONDS_PER_WEEK
    value = _float(delta * Decimal(str(SPEED_OF_LIGHT_MPS)))
    return value if MIN_P <= value <= MAX_P else None


def _parse_payload(payload: bytes) -> tuple[list[Row], dict[str, Any]]:
    try:
        text = payload.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise _fail("raw CSV is not UTF-8") from exc
    reader = csv.DictReader(io.StringIO(text, newline=""))
    if reader.fieldnames is None:
        raise _fail("raw CSV has no header")
    header = [str(value).strip() for value in reader.fieldnames]
    fields = {_normalise_header(name): name for name in header}
    missing = [name for name in REQUIRED_COLUMNS if _normalise_header(name) not in fields]
    if missing:
        raise _fail(f"missing required columns: {missing}")
    uncertainty_field = fields.get(_normalise_header("AccumulatedDeltaRangeUncertaintyMeters"))
    rows: list[Row] = []
    raw_count = non_raw_count = unsupported = 0
    epoch_keys: list[int] = []
    epoch_seen: set[int] = set()
    repeated_epoch_keys = nonmonotonic_epochs = 0
    previous_epoch: int | None = None
    for number, source in enumerate(reader, start=2):
        if str(source.get(fields["messagetype"], "")).strip() != "Raw":
            non_raw_count += 1
            continue
        raw_count += 1

        def integer(name: str, required: bool = True, default: int | None = None) -> int | None:
            key = fields.get(_normalise_header(name))
            if key is None:
                if required:
                    raise _fail(f"missing field {name}")
                return default
            value = _parse_int(source.get(key, ""), f"{name} row {number}", required)
            return default if value is None and default is not None else value

        def decimal(name: str, required: bool = True, default: Decimal | None = None) -> Decimal | None:
            key = fields.get(_normalise_header(name))
            if key is None:
                if required:
                    raise _fail(f"missing field {name}")
                return default
            value = _parse_decimal(source.get(key, ""), f"{name} row {number}", required)
            return default if value is None and default is not None else value

        utc = integer("utcTimeMillis")
        time_ns = integer("TimeNanos")
        full_bias = integer("FullBiasNanos")
        received = integer("ReceivedSvTimeNanos")
        svid = integer("Svid")
        constellation = integer("ConstellationType")
        adr_state = integer("AccumulatedDeltaRangeState")
        assert utc is not None and time_ns is not None and full_bias is not None and received is not None and svid is not None and constellation is not None and adr_state is not None
        frequency = decimal("CarrierFrequencyHz")
        rate = decimal("PseudorangeRateMetersPerSecond")
        adr_m = decimal("AccumulatedDeltaRangeMeters")
        bias = decimal("BiasNanos", required=False, default=Decimal(0)) or Decimal(0)
        offset = decimal("TimeOffsetNanos", required=False, default=Decimal(0)) or Decimal(0)
        adr_uncertainty = decimal("AccumulatedDeltaRangeUncertaintyMeters", required=False)
        rate_uncertainty = decimal("PseudorangeRateUncertaintyMetersPerSecond", required=False)
        state = integer("State", required=False)
        multipath = integer("MultipathIndicator", required=False)
        cn0 = decimal("Cn0DbHz", required=False)
        hcdc = integer("HardwareClockDiscontinuityCount", required=False, default=0) or 0
        assert frequency is not None and rate is not None
        if previous_epoch != utc:
            if utc in epoch_seen:
                repeated_epoch_keys += 1
            if previous_epoch is not None and utc < previous_epoch:
                nonmonotonic_epochs += 1
            epoch_seen.add(utc)
            epoch_keys.append(utc)
            previous_epoch = utc
        system = _system(source.get(fields["constellationtype"], ""))
        signal = _signal(source.get(fields["signaltype"], ""), system, frequency)
        if system == "UNKNOWN" or signal == "UNKNOWN":
            unsupported += 1
        rows.append(Row(
            row_number=number, utc_ms=utc, time_ns=time_ns, full_bias_ns=full_bias,
            bias_ns=bias, offset_ns=offset, received_sv_time_ns=Decimal(received),
            rate_mps=rate, rate_uncertainty_mps=rate_uncertainty,
            adr_state=adr_state, adr_m=adr_m, adr_uncertainty_m=adr_uncertainty,
            state=state, multipath=multipath, cn0=cn0, hcdc=hcdc, svid=svid,
            system=system, signal=signal, frequency_hz=frequency,
        ))
    if raw_count == 0:
        raise _fail("no Raw rows")
    return rows, {
        "header_columns": header,
        "normalised_header_columns": sorted(fields),
        "raw_rows": raw_count,
        "non_raw_rows": non_raw_count,
        "unsupported_signal_rows": unsupported,
        "epoch_count": len(epoch_keys),
        "repeated_epoch_key_count": repeated_epoch_keys,
        "nonmonotonic_epoch_key_count": nonmonotonic_epochs,
        "adr_uncertainty_field": uncertainty_field,
        "adr_uncertainty_field_present": uncertainty_field is not None,
        "optional_field_presence": {name: _normalise_header(name) in fields for name in OPTIONAL_COLUMNS},
    }


def _assign_segments(rows: Sequence[Row]) -> list[dict[str, Any]]:
    by_epoch: dict[int, list[Row]] = {}
    order: list[int] = []
    for row in rows:
        if row.utc_ms not in by_epoch:
            by_epoch[row.utc_ms] = []
            order.append(row.utc_ms)
        by_epoch[row.utc_ms].append(row)
    assignments: list[dict[str, Any]] = []
    previous_rows: list[Row] | None = None
    previous_hcdc: int | None = None
    segment = -1
    base_full_bias_ns: int | None = None
    for epoch_index, key in enumerate(order):
        current_rows = by_epoch[key]
        hcdc = current_rows[0].hcdc
        current_time = _median(row.time_ns for row in current_rows)
        previous_time = _median(row.time_ns for row in previous_rows) if previous_rows else None
        gap_ns = None if previous_time is None else current_time - previous_time
        hcdc_change = previous_hcdc is not None and hcdc != previous_hcdc
        gap_change = gap_ns is not None and abs(gap_ns) > TIME_GAP_NS
        boundary = previous_rows is None or hcdc_change or gap_change
        if boundary:
            segment += 1
            base_full_bias_ns = current_rows[0].full_bias_ns
        assert base_full_bias_ns is not None
        for row in current_rows:
            row.segment = segment
            row.pseudorange_m = _raw_pseudorange(row, base_full_bias_ns)
            reasons: list[str] = []
            if row.multipath == 1:
                reasons.append("multipath")
            if row.cn0 is not None and _float(row.cn0) < 20.0:
                reasons.append("low_cnr")
            if not _status_valid(row):
                reasons.append("invalid_state")
            if row.pseudorange_m is None:
                reasons.append("code_range")
            row.code_masked = bool(reasons)
            row.adr_masked = not _adr_unflagged(row) or row.adr_m is None
        assignments.append({
            "epoch_index": epoch_index, "utcTimeMillis": key, "segment": segment,
            "hcdc": hcdc, "segment_base_FullBiasNanos": base_full_bias_ns,
            "time_nanos_gap_ns": gap_ns, "explicit_boundary": boundary,
        })
        previous_rows, previous_hcdc = current_rows, hcdc
    return assignments


def _pair_reason(previous: Row, current: Row, residual: float | None) -> str:
    dt_ns = current.time_ns - previous.time_ns
    if dt_ns <= 0:
        return "nonpositive_dt"
    if dt_ns > PAIR_MAX_NS:
        return "gap"
    if current.hcdc != previous.hcdc or current.segment != previous.segment:
        return "hcdc_or_segment_boundary"
    if ((previous.adr_state is not None and previous.adr_state & ADR_RESET)
            or (current.adr_state is not None and current.adr_state & ADR_RESET)):
        return "adr_reset"
    if ((previous.adr_state is not None and previous.adr_state & ADR_CYCLE_SLIP)
            or (current.adr_state is not None and current.adr_state & ADR_CYCLE_SLIP)):
        return "cycle_slip"
    if not _adr_unflagged(previous) or not _adr_unflagged(current):
        return "invalid_adr_state"
    if previous.code_masked or current.code_masked:
        return "existing_mask"
    if residual is None or not math.isfinite(residual):
        return "nonfinite_closure"
    if previous.adr_uncertainty_m is None or current.adr_uncertainty_m is None:
        return "uncertainty_missing"
    if not _finite_positive(previous.adr_uncertainty_m) or not _finite_positive(current.adr_uncertainty_m):
        return "uncertainty_nonpositive_or_nonfinite"
    return "ordinary_uncertainty"


def _finite_positive(value: Decimal | float | None) -> bool:
    if value is None:
        return False
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(parsed) and parsed > 0.0


def _pair_items(rows: Sequence[Row]) -> list[dict[str, Any]]:
    by_key: dict[tuple[str, int, str], list[Row]] = {}
    for row in rows:
        by_key.setdefault(row.sat_signal, []).append(row)
    pairs: list[dict[str, Any]] = []
    for key, key_rows in sorted(by_key.items()):
        key_rows.sort(key=lambda row: (row.utc_ms, row.row_number))
        for index, (previous, current) in enumerate(zip(key_rows, key_rows[1:])):
            dt_s = (current.time_ns - previous.time_ns) / 1.0e9
            residual: float | None = None
            if previous.rate_mps is not None and current.rate_mps is not None and previous.adr_m is not None and current.adr_m is not None:
                residual = (float(current.adr_m) - float(previous.adr_m)) - 0.5 * (float(previous.rate_mps) + float(current.rate_mps)) * dt_s
            reason = _pair_reason(previous, current, residual)
            ordinary = reason in {"ordinary_uncertainty", "uncertainty_missing", "uncertainty_nonpositive_or_nonfinite"}
            uncertainty_eligible = reason == "ordinary_uncertainty"
            uncertainty_pair = math.hypot(float(previous.adr_uncertainty_m), float(current.adr_uncertainty_m)) if uncertainty_eligible else None
            item: dict[str, Any] = {
                "key": key, "key_index": index, "utcTimeMillis": current.utc_ms,
                "previous_utcTimeMillis": previous.utc_ms, "system": current.system,
                "svid": current.svid, "signal": current.signal,
                "group": f"{current.system}:{current.signal}:{int(round(float(current.frequency_hz)))}Hz",
                "state_group": _adr_class(current.adr_state), "segment": current.segment,
                "hcdc": current.hcdc, "reason": reason, "ordinary": ordinary,
                "uncertainty_eligible": uncertainty_eligible, "residual_m": residual,
                "dt_s": dt_s, "adr_state_previous": previous.adr_state,
                "adr_state_current": current.adr_state,
                "adr_uncertainty_previous_m": float(previous.adr_uncertainty_m) if previous.adr_uncertainty_m is not None else None,
                "adr_uncertainty_current_m": float(current.adr_uncertainty_m) if current.adr_uncertainty_m is not None else None,
                "u_pair_m": uncertainty_pair,
            }
            pairs.append(item)
    centers: dict[tuple[int, int], list[float]] = {}
    for item in pairs:
        if item["ordinary"] and item["residual_m"] is not None and math.isfinite(float(item["residual_m"])):
            centers.setdefault((int(item["utcTimeMillis"]), int(item["hcdc"])), []).append(float(item["residual_m"]))
    for item in pairs:
        residual = item.get("residual_m")
        if residual is None or not math.isfinite(float(residual)):
            item["center_m"] = None
            item["centered_residual_m"] = None
            item["abs_centered_residual_m"] = None
            continue
        group = (int(item["utcTimeMillis"]), int(item["hcdc"]))
        center = _median(centers.get(group, [float(residual)]))
        centered = float(residual) - center
        item["center_m"] = center
        item["centered_residual_m"] = centered
        item["abs_centered_residual_m"] = abs(centered)
        if item["uncertainty_eligible"]:
            item["normalized_abs_residual"] = abs(centered) / float(item["u_pair_m"])
            item["within_one_sigma"] = abs(centered) <= float(item["u_pair_m"])
        else:
            item["normalized_abs_residual"] = None
            item["within_one_sigma"] = None
    return pairs


def _fixed_bin(value: float | None) -> str:
    if value is None or not math.isfinite(value) or value <= 0.0:
        return "unavailable"
    if value <= 0.01:
        return "<=0.01m"
    if value <= 0.1:
        return "0.01_to_0.1m"
    if value <= 1.0:
        return "0.1_to_1m"
    return ">1m"


def _finite_items(items: Sequence[dict[str, Any]], key: str) -> list[float]:
    values: list[float] = []
    for item in items:
        value = item.get(key)
        if value is None:
            continue
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(parsed):
            values.append(parsed)
    return values


def _summary(items: Sequence[dict[str, Any]]) -> dict[str, Any]:
    residuals = _finite_items(items, "abs_centered_residual_m")
    us = _finite_items(items, "u_pair_m")
    normalized = _finite_items(items, "normalized_abs_residual")
    within = [bool(item["within_one_sigma"]) for item in items if item.get("within_one_sigma") is not None]
    return {
        "count": len(items),
        "u_pair_m": _distribution(us, 0.0),
        "abs_centered_residual_m": _distribution(residuals, 0.0),
        "normalized_abs_residual": _distribution(normalized, 0.0),
        "within_one_sigma_fraction": sum(within) / len(within) if within else None,
    }


def _quantile_groups(items: Sequence[dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    ordered = sorted(items, key=lambda item: (float(item["u_pair_m"]), int(item["utcTimeMillis"]), int(item["svid"])))
    count = len(ordered)
    groups: dict[str, list[dict[str, Any]]] = {"q1": [], "q2": [], "q3": [], "q4": []}
    if not ordered:
        return groups
    for index, item in enumerate(ordered):
        group = min(3, (index * 4) // count)
        groups[f"q{group + 1}"].append(item)
    return groups


def _group_report(items: Sequence[dict[str, Any]], key: str) -> dict[str, Any]:
    groups: dict[str, list[dict[str, Any]]] = {}
    for item in items:
        groups.setdefault(str(item[key]), []).append(item)
    report: dict[str, Any] = {}
    for name, group in sorted(groups.items()):
        eligible = [item for item in group if item["uncertainty_eligible"]]
        report[name] = {
            "count": len(group),
            "uncertainty_count": len(eligible),
            "summary": _summary(eligible),
            "spearman_u_pair_vs_abs_residual": _spearman(_finite_items(eligible, "u_pair_m"), _finite_items(eligible, "abs_centered_residual_m")),
            "state_classes": sorted({str(item["state_group"]) for item in group}),
        }
    return report


def _route_report(route: str, rows: Sequence[Row], metadata: dict[str, Any], path: Path, digest: str, byte_size: int) -> dict[str, Any]:
    assignments = _assign_segments(rows)
    pairs = _pair_items(rows)
    ordinary = [item for item in pairs if item["ordinary"]]
    uncertainty = [item for item in pairs if item["uncertainty_eligible"]]
    unavailable = [item for item in ordinary if not item["uncertainty_eligible"]]
    satellites = sorted({f"{item['system']}:{item['svid']}" for item in ordinary})
    signal_groups = sorted({str(item["group"]) for item in ordinary})
    state_groups = sorted({str(item["state_group"]) for item in ordinary})
    fixed: dict[str, list[dict[str, Any]]] = {name: [] for name, _, _ in FIXED_BINS}
    for item in uncertainty:
        fixed[_fixed_bin(float(item["u_pair_m"]))].append(item)
    fixed_stats: dict[str, dict[str, Any]] = {}
    for name, _, _ in FIXED_BINS:
        values = fixed[name]
        summary = _summary(values)
        fixed_stats[name] = {
            **summary,
            "median_abs_centered_residual_m": summary["abs_centered_residual_m"]["median"],
            "p95_abs_centered_residual_m": summary["abs_centered_residual_m"]["p95_abs"],
            "within_one_sigma_fraction": summary["within_one_sigma_fraction"],
        }
    quantiles = _quantile_groups(uncertainty)
    quantile_stats = {name: _summary(values) for name, values in quantiles.items()}
    q1, q4 = quantile_stats["q1"], quantile_stats["q4"]
    q1_p95 = float(q1["abs_centered_residual_m"]["p95_abs"])
    q4_p95 = float(q4["abs_centered_residual_m"]["p95_abs"])
    q_ratio = q4_p95 / q1_p95 if q1_p95 > 0.0 else None
    bin_medians = [float(fixed_stats[name]["median_abs_centered_residual_m"]) for name, _, _ in FIXED_BINS if fixed[name]]
    bin_medians_non_decreasing = all(left <= right for left, right in zip(bin_medians, bin_medians[1:]))
    eligible_us = _finite_items(uncertainty, "u_pair_m")
    eligible_residuals = _finite_items(uncertainty, "abs_centered_residual_m")
    normalized = _finite_items(uncertainty, "normalized_abs_residual")
    ordinary_residuals = _finite_items(ordinary, "abs_centered_residual_m")
    reason_counts: dict[str, int] = {}
    for item in pairs:
        reason_counts[item["reason"]] = reason_counts.get(item["reason"], 0) + 1
    fixed_populated = [name for name, _, _ in FIXED_BINS if fixed[name]]
    calibration_values = [fixed_stats[name]["within_one_sigma_fraction"] for name in fixed_populated]
    calibration_pass = bool(calibration_values) and all(
        value is not None and MIN_CALIBRATION_COVERAGE <= float(value) <= MAX_CALIBRATION_COVERAGE
        for value in calibration_values
    )
    signal_report = _group_report(ordinary, "group")
    satellite_report = _group_report(ordinary, "svid")
    state_report = _group_report(ordinary, "state_group")
    correlation = _spearman(eligible_us, eligible_residuals)
    return {
        "route": route,
        "input": {"path": _relative(path), "bytes": byte_size, "sha256": digest},
        "headers": {
            "columns": metadata["header_columns"],
            "adr_uncertainty_field": metadata["adr_uncertainty_field"],
            "adr_uncertainty_field_present": metadata["adr_uncertainty_field_present"],
            "optional_field_presence": metadata["optional_field_presence"],
        },
        "rows": {
            "raw": metadata["raw_rows"], "non_raw": metadata["non_raw_rows"],
            "epochs": metadata["epoch_count"], "pair_count": len(pairs),
            "ordinary_pair_count": len(ordinary), "uncertainty_pair_count": len(uncertainty),
            "uncertainty_pair_fraction_of_ordinary": len(uncertainty) / len(ordinary) if ordinary else 0.0,
            "ordinary_without_uncertainty_count": len(unavailable),
            "satellite_count": len(satellites), "satellites": satellites,
            "signal_group_count": len(signal_groups), "signal_groups": signal_groups,
            "state_group_count": len(state_groups), "state_groups": state_groups,
            "unsupported_signal_rows": metadata["unsupported_signal_rows"],
            "repeated_epoch_key_count": metadata["repeated_epoch_key_count"],
            "nonmonotonic_epoch_key_count": metadata["nonmonotonic_epoch_key_count"],
        },
        "closure": {
            "ordinary_signed_residual_m": _distribution(_finite_items(ordinary, "residual_m"), 0.0),
            "ordinary_abs_centered_residual_m": _distribution(ordinary_residuals, 0.0),
        },
        "uncertainty": {
            "pair_u_m": _distribution(eligible_us, 0.0),
            "normalized_abs_centered_residual": _distribution(normalized, 0.0),
            "fixed_bins": fixed_stats,
            "quantiles": quantile_stats,
            "quantile_q4_vs_q1": {
                "q1_p95_abs_centered_residual_m": q1_p95,
                "q4_p95_abs_centered_residual_m": q4_p95,
                "p95_excess_m": q4_p95 - q1_p95,
                "p95_ratio": q_ratio,
            },
        },
        "groups": {
            "signal_frequency": signal_report,
            "satellite": satellite_report,
            "state": state_report,
        },
        "segments": {
            "count": len({int(item["segment"]) for item in assignments}),
            "boundaries": sum(1 for item in assignments if item["explicit_boundary"]),
            "assignments": assignments,
        },
        "pair_reasons": reason_counts,
        "gate_observations": {
            "all_core_finite": all(
                item.get(key) is not None and math.isfinite(float(item[key]))
                for item in ordinary
                for key in ("residual_m", "abs_centered_residual_m")
            ),
            "ordinary_pairs": len(ordinary),
            "uncertainty_pairs": len(uncertainty),
            "uncertainty_pair_fraction": len(uncertainty) / len(ordinary) if ordinary else 0.0,
            "satellite_count": len(satellites),
            "current_tdcp_overlap": len(ordinary),
            "uncertainty_field_present": metadata["adr_uncertainty_field_present"],
            "populated_fixed_bins": len(fixed_populated),
            "minimum_populated_bin_count": min((len(fixed[name]) for name in fixed_populated), default=0),
            "spearman_u_pair_vs_abs_centered_residual": correlation,
            "q4_vs_q1_p95_excess_m": q4_p95 - q1_p95,
            "q4_vs_q1_p95_ratio": q_ratio,
            "fixed_bin_median_abs_residual_non_decreasing": bin_medians_non_decreasing,
            "calibration_per_bin_fraction_within_u_pair": calibration_pass,
            "normalized_abs_residual_median": _median(normalized),
            "same_signal_direction": all(
                float(group["spearman_u_pair_vs_abs_residual"]) >= 0.0
                for group in signal_report.values() if int(group["uncertainty_count"]) >= 2
            ),
            "signal_group_count": len(signal_groups),
            "satellite_group_count": len(satellites),
            "header_and_finite_uncertainty_coverage": bool(metadata["adr_uncertainty_field_present"] and uncertainty),
        },
        "_pairs": pairs,
    }


def _static_contract(freeze: dict[str, Any]) -> dict[str, Any]:
    contents: dict[str, str] = {}
    hashes: dict[str, str] = {}
    for name, pin in freeze["authority_pins"]["source_contracts"].items():
        payload, digest = _read_bytes_once(ROOT / pin["path"], f"Phase55 static source {name}", pin["sha256"])
        contents[name] = payload.decode("utf-8")
        hashes[name] = digest
    adapter = contents["android_raw_gnss_cpp"]
    observation = contents["observation_header"]
    fgo = contents["fgo_problems_cpp"]
    return {
        "source_hashes": hashes,
        "adapter_parses_adr": "accumulateddeltarangemeters" in adapter.lower(),
        "adapter_parses_adr_state": "accumulateddeltarangestate" in adapter.lower(),
        "adapter_parses_adr_uncertainty": "accumulateddeltarangeuncertaintymeters" in adapter.lower(),
        "observation_retains_adr_uncertainty": "accumulateddeltarangeuncertainty" in observation.lower(),
        "fgo_consumes_adr_uncertainty_for_tdcp_sigma": "accumulateddeltarangeuncertainty" in fgo.lower(),
        "fgo_uses_fixed_tdcp_sigma": "config_.tdcp_sigma_m" in fgo,
        "tdcp_contract_present": "tdcp_contract::evaluateAdjacentPair" in fgo and "max_tdcp_gap_s" in fgo,
        "interpretation": "Current source parses ADR state and meters but does not parse or retain AccumulatedDeltaRangeUncertaintyMeters and does not consume it for TDCP sigma; implementation is therefore authorized only in a separately sealed stage after all raw identifiability gates pass.",
    }


def _verify_freeze(path: Path = FREEZE) -> dict[str, Any]:
    freeze, digest = _load_json_once(path, "Phase55 freeze", FREEZE_SHA256 or None)
    if digest != FREEZE_SHA256:
        raise _fail(f"Phase55 freeze hash changed: {digest} != {FREEZE_SHA256}")
    if freeze.get("schema_version") != FREEZE_SCHEMA or freeze.get("status") != "frozen-before-phase55-raw-read":
        raise _fail("Phase55 freeze schema/status mismatch")
    if tuple(freeze.get("cohort", {}).get("route_order", [])) != ROUTES:
        raise _fail("Phase55 route order changed")
    inputs = freeze.get("exact_raw_inputs")
    if not isinstance(inputs, dict) or tuple(inputs) != ROUTES:
        raise _fail("Phase55 exact raw input map changed")
    for route in ROUTES:
        item = inputs[route]
        if len(str(item.get("sha256", ""))) != 64 or int(item.get("file_size", 0)) <= 0:
            raise _fail(f"Phase55 raw hash/size missing for {route}")
    policy = freeze.get("input_policy", {})
    expected_policy = {
        "single_process": True, "raw_device_gnss_read_count_per_route": 1,
        "raw_device_imu_read_count": 0, "truth_read_count": 0,
        "phase53_or_phase54_metric_payload_reads": 0, "brdc_nav_reads": 0,
        "solver_reruns": 0, "trajectory_reruns": 0, "correction_fit_or_application": False,
        "validation_holdout_access": False, "archive_reopens": 0,
        "rematerialization_count": 0, "kaggle_or_token_access": False,
        "mat_reads_or_generated": False, "device_wls_or_precomputed_coordinates": False,
        "SvPosition_or_SvElevation": False,
    }
    if any(policy.get(key) != value for key, value in expected_policy.items()):
        raise _fail("Phase55 read policy changed")
    gates = freeze.get("numeric_gates", {})
    expected_gates = {
        "route_count_required": 4, "min_ordinary_pairs_per_route": MIN_ORDINARY_PAIRS,
        "min_satellites_per_route": MIN_SATELLITES, "min_current_tdcp_overlap_pairs_per_route": MIN_CURRENT_TDCP_OVERLAP,
        "min_positive_uncertainty_pairs_per_route": MIN_UNCERTAINTY_PAIRS,
        "min_positive_uncertainty_fraction_of_ordinary": MIN_UNCERTAINTY_FRACTION,
    }
    for key, value in expected_gates.items():
        if gates.get(key) != value:
            raise _fail(f"Phase55 gate changed: {key}")
    fixed = gates.get("fixed_bins_m", {})
    expected_definitions = [
        name.removesuffix("m") for name, _, _ in FIXED_BINS
    ]
    if fixed.get("definitions") != expected_definitions or fixed.get("min_populated_bins_per_route") != MIN_POPULATED_BINS or fixed.get("min_pairs_per_populated_bin") != MIN_PAIRS_PER_BIN or fixed.get("median_abs_residual_non_decreasing") is not True or fixed.get("allow_exact_equality") is not True:
        raise _fail("Phase55 fixed-bin gate changed")
    relation = gates.get("routewise_relation", {})
    for key, value in {
        "spearman_u_pair_vs_abs_centered_residual_min": MIN_SPEARMAN,
        "q4_vs_q1_p95_excess_min_m": MIN_QUANTILE_EXCESS_M,
        "q4_vs_q1_p95_ratio_min": MIN_QUANTILE_RATIO,
        "min_signal_groups_per_route": 1,
        "min_satellite_groups_per_route": MIN_SATELLITES,
    }.items():
        if relation.get(key) != value:
            raise _fail(f"Phase55 relation gate changed: {key}")
    calibration = gates.get("calibration", {})
    for key, value in {
        "per_bin_abs_residual_le_u_pair_fraction_min": MIN_CALIBRATION_COVERAGE,
        "per_bin_abs_residual_le_u_pair_fraction_max": MAX_CALIBRATION_COVERAGE,
        "global_normalized_abs_residual_median_min": MIN_NORMALIZED_MEDIAN,
        "global_normalized_abs_residual_median_max": MAX_NORMALIZED_MEDIAN,
    }.items():
        if calibration.get(key) != value:
            raise _fail(f"Phase55 calibration gate changed: {key}")
    loo = gates.get("loo", {})
    for key, value in {
        "q4_vs_q1_p95_excess_min_m": MIN_LOO_EXCESS_M,
        "q4_vs_q1_p95_ratio_min": MIN_LOO_RATIO,
        "fixed_quartile_mapping_direction_stable": True,
        "all_folds_positive_direction": True,
    }.items():
        if loo.get(key) != value:
            raise _fail(f"Phase55 LOO gate changed: {key}")
    assertions = freeze.get("pre_read_assertions", {})
    if not isinstance(assertions, dict) or any(value is not False for value in assertions.values()):
        raise _fail("Phase55 pre-read assertions are not closed")
    return freeze


def _verify_manifest(freeze: dict[str, Any], path: Path = MANIFEST) -> dict[str, Any]:
    global VERIFIED_MANIFEST_SHA256
    manifest, digest = _load_json_once(path, "Phase55 evaluator manifest", MANIFEST_SHA256 or None)
    if MANIFEST_SHA256 and digest != MANIFEST_SHA256:
        raise _fail(f"Phase55 manifest hash changed: {digest} != {MANIFEST_SHA256}")
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("status") != "evaluator-frozen-before-phase55-raw-read":
        raise _fail("Phase55 manifest schema/status mismatch")
    if manifest.get("freeze", {}).get("path") != _relative(FREEZE) or manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256:
        raise _fail("Phase55 manifest freeze pin mismatch")
    evaluator = manifest.get("evaluator", {})
    if evaluator.get("operation") != "audit" or evaluator.get("native_solver_invoked") is not False or evaluator.get("single_process") is not True:
        raise _fail("Phase55 manifest permits solver/multi-process execution")
    if evaluator.get("phase55_raw_reads_before_gate") != 0 or evaluator.get("correction_before_gate") is not False:
        raise _fail("Phase55 manifest permits early raw/correction access")
    cohort = manifest.get("cohort", {})
    if tuple(cohort.get("route_order", [])) != ROUTES or cohort.get("raw_device_gnss_reads_per_route") != 1 or cohort.get("truth_reads_per_route") != 0:
        raise _fail("Phase55 manifest route/read policy mismatch")
    required_forbidden = ("truth", "validation", "MAT", "navigation", "solver", "coordinates", "IMU", "Kaggle", "token", "prior metric payload")
    forbidden = manifest.get("forbidden", [])
    if not isinstance(forbidden, list) or not all(token in forbidden for token in required_forbidden):
        raise _fail("Phase55 forbidden policy missing")
    for name in ("source", "test", "cmake"):
        pin = evaluator.get(name, {})
        pin_path = ROOT / str(pin.get("path", ""))
        if not pin_path.is_file() or len(str(pin.get("sha256", ""))) != 64:
            raise _fail(f"Phase55 {name} pin missing")
        actual = _sha256_bytes(pin_path.read_bytes())
        if actual != pin["sha256"]:
            raise _fail(f"Phase55 {name} hash mismatch")
    VERIFIED_MANIFEST_SHA256 = digest
    return manifest


def _aggregate_uncertainty(items: Sequence[dict[str, Any]]) -> dict[str, Any]:
    eligible = [item for item in items if item.get("uncertainty_eligible")]
    groups = _quantile_groups(eligible)
    q1 = _summary(groups["q1"])
    q4 = _summary(groups["q4"])
    q1_p95 = float(q1["abs_centered_residual_m"]["p95_abs"])
    q4_p95 = float(q4["abs_centered_residual_m"]["p95_abs"])
    ratio = q4_p95 / q1_p95 if q1_p95 > 0.0 else None
    us = _finite_items(eligible, "u_pair_m")
    residuals = _finite_items(eligible, "abs_centered_residual_m")
    return {
        "count": len(eligible),
        "spearman": _spearman(us, residuals),
        "q1_p95_abs_m": q1_p95,
        "q4_p95_abs_m": q4_p95,
        "q4_q1_excess_m": q4_p95 - q1_p95,
        "q4_q1_ratio": ratio,
        "positive_direction": bool(_spearman(us, residuals) >= 0.0 and q4_p95 - q1_p95 >= 0.0),
    }


def _loo(reports: dict[str, dict[str, Any]]) -> dict[str, Any]:
    folds: list[dict[str, Any]] = []
    for omitted in ROUTES:
        items = [item for route, report in reports.items() if route != omitted for item in report["_pairs"] if item.get("uncertainty_eligible")]
        aggregate = _aggregate_uncertainty(items)
        aggregate["omitted_route"] = omitted
        aggregate["fixed_positive_material"] = bool(
            float(aggregate["q4_q1_excess_m"]) >= MIN_LOO_EXCESS_M
            and aggregate["q4_q1_ratio"] is not None
            and float(aggregate["q4_q1_ratio"]) >= MIN_LOO_RATIO
        )
        folds.append(aggregate)
    directions = [bool(item["fixed_positive_material"]) for item in folds]
    return {
        "folds": folds,
        "fixed_rule": f"q4/q1 p95 excess >= {MIN_LOO_EXCESS_M} m and ratio >= {MIN_LOO_RATIO}",
        "direction_stable": bool(directions) and len(set(directions)) == 1,
        "all_positive_material": bool(directions) and all(directions),
    }


def _presentation_integrity(reports: dict[str, dict[str, Any]], aggregate: dict[str, Any], loo: dict[str, Any], events_count: int) -> dict[str, Any]:
    pair_reasons: dict[str, bool] = {}
    state_groups: dict[str, bool] = {}
    signal_groups: dict[str, bool] = {}
    satellite_groups: dict[str, bool] = {}
    bin_groups: dict[str, bool] = {}
    quantile_groups: dict[str, bool] = {}
    header_map: dict[str, bool] = {}
    for route, report in reports.items():
        pair_reasons[route] = sum(int(value) for value in report["pair_reasons"].values()) == int(report["rows"]["pair_count"])
        state_groups[route] = sum(int(value["count"]) for value in report["groups"]["state"].values()) == int(report["rows"]["ordinary_pair_count"])
        signal_groups[route] = sum(int(value["count"]) for value in report["groups"]["signal_frequency"].values()) == int(report["rows"]["ordinary_pair_count"])
        satellite_groups[route] = sum(int(value["count"]) for value in report["groups"]["satellite"].values()) == int(report["rows"]["ordinary_pair_count"])
        bin_groups[route] = sum(int(value["count"]) for value in report["uncertainty"]["fixed_bins"].values()) == int(report["rows"]["uncertainty_pair_count"])
        quantile_groups[route] = sum(int(value["count"]) for value in report["uncertainty"]["quantiles"].values()) == int(report["rows"]["uncertainty_pair_count"])
        header_map[route] = isinstance(report["headers"].get("columns"), list) and isinstance(report["headers"].get("optional_field_presence"), dict)
    medians = aggregate["route_median_abs_uncertainty_residual_m"]
    retained = len(medians) == len(ROUTES) and tuple(medians) == ROUTES
    values = list(medians.values())
    aggregate_exact = aggregate["route_median_aggregate_m"] == _median(values) and aggregate["route_median_mad_m"] == _mad(values)
    return {
        "pair_reason_counts_sum": pair_reasons,
        "pair_reason_counts_sum_all": all(pair_reasons.values()),
        "state_group_counts_sum": state_groups,
        "state_group_counts_sum_all": all(state_groups.values()),
        "signal_group_counts_sum": signal_groups,
        "signal_group_counts_sum_all": all(signal_groups.values()),
        "satellite_group_counts_sum": satellite_groups,
        "satellite_group_counts_sum_all": all(satellite_groups.values()),
        "fixed_bin_counts_sum": bin_groups,
        "fixed_bin_counts_sum_all": all(bin_groups.values()),
        "quantile_counts_sum": quantile_groups,
        "quantile_counts_sum_all": all(quantile_groups.values()),
        "four_route_medians_retained": retained,
        "aggregate_recomputed_exact": aggregate_exact,
        "loo_route_count_exact": len(loo.get("folds", [])) == len(ROUTES) and tuple(item.get("omitted_route") for item in loo.get("folds", [])) == ROUTES,
        "event_count_exact": events_count == 0,
        "header_field_presence_route_map_exact": len(header_map) == len(ROUTES) and tuple(header_map) == ROUTES and all(header_map.values()),
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
        raise _fail(f"Phase55 output already exists: {output_root}")
    static = _static_contract(freeze)
    reports: dict[str, dict[str, Any]] = {}
    reads: dict[str, Any] = {
        "single_process": True, "raw_device_gnss_reads": {}, "raw_device_gnss_read_count_total": 0,
        "raw_device_imu_reads": 0, "truth_reads": 0, "phase53_or_phase54_metric_payload_reads": 0,
        "brdc_nav_reads": 0, "solver_reruns": 0, "trajectory_reruns": 0,
        "correction_implementations": 0, "validation_holdout_reads": 0, "archive_reopens": 0,
        "rematerializations": 0, "kaggle_token_access": 0, "mat_reads_or_generated": 0,
        "device_wls_or_precomputed_coordinates": 0, "SvPosition_or_SvElevation": 0,
        "source_static_reads": len(static["source_hashes"]),
    }
    for route in ROUTES:
        pin = freeze["exact_raw_inputs"][route]
        path = ROOT / pin["path"]
        payload, digest = _read_bytes_once(path, f"Phase55 raw device_gnss {route}", pin["sha256"])
        if len(payload) != int(pin["file_size"]):
            raise _fail(f"Phase55 raw byte count mismatch: {route}")
        rows, metadata = _parse_payload(payload)
        byte_size = len(payload)
        del payload
        reports[route] = _route_report(route, rows, metadata, path, digest, byte_size)
        reads["raw_device_gnss_reads"][route] = 1
        reads["raw_device_gnss_read_count_total"] += 1
        del rows
    # Route center is the median of all uncertainty-eligible absolute closure residuals,
    # not a quantile proxy.  Keep this explicit to prevent stale loop-variable summaries.
    route_medians = {
        route: _median(_finite_items(report["_pairs"], "abs_centered_residual_m"))
        for route, report in reports.items()
    }
    aggregate = {
        "route_count": len(reports),
        "route_median_abs_uncertainty_residual_m": route_medians,
        "route_median_aggregate_m": _median(route_medians.values()),
        "route_median_mad_m": _mad(route_medians.values()),
        "pairwise_route_median_distances_m": [
            {"route_a": left, "route_b": right, "distance_m": abs(route_medians[left] - route_medians[right])}
            for index, left in enumerate(ROUTES) for right in ROUTES[index + 1:]
        ],
    }
    loo = _loo(reports)
    events_count = 0
    integrity = _presentation_integrity(reports, aggregate, loo, events_count)
    observations = {route: report["gate_observations"] for route, report in reports.items()}
    ordinary_pass = all(
        int(value["ordinary_pairs"]) >= MIN_ORDINARY_PAIRS
        and int(value["satellite_count"]) >= MIN_SATELLITES
        and int(value["current_tdcp_overlap"]) >= MIN_CURRENT_TDCP_OVERLAP
        and bool(value["all_core_finite"])
        for value in observations.values()
    )
    uncertainty_pass = all(
        int(value["uncertainty_pairs"]) >= MIN_UNCERTAINTY_PAIRS
        and float(value["uncertainty_pair_fraction"]) >= MIN_UNCERTAINTY_FRACTION
        and bool(value["uncertainty_field_present"])
        for value in observations.values()
    )
    bins_pass = all(
        int(value["populated_fixed_bins"]) >= MIN_POPULATED_BINS
        and int(value["minimum_populated_bin_count"]) >= MIN_PAIRS_PER_BIN
        and bool(value["fixed_bin_median_abs_residual_non_decreasing"])
        for value in observations.values()
    )
    relation_pass = all(
        float(value["spearman_u_pair_vs_abs_centered_residual"]) >= MIN_SPEARMAN
        and float(value["q4_vs_q1_p95_excess_m"]) >= MIN_QUANTILE_EXCESS_M
        and value["q4_vs_q1_p95_ratio"] is not None
        and float(value["q4_vs_q1_p95_ratio"]) >= MIN_QUANTILE_RATIO
        and bool(value["same_signal_direction"])
        for value in observations.values()
    )
    calibration_pass = all(
        bool(value["calibration_per_bin_fraction_within_u_pair"])
        and MIN_NORMALIZED_MEDIAN <= float(value["normalized_abs_residual_median"]) <= MAX_NORMALIZED_MEDIAN
        for value in observations.values()
    )
    loo_pass = bool(loo["direction_stable"] and loo["all_positive_material"])
    integrity_pass = all(
        bool(integrity[key]) for key in (
            "pair_reason_counts_sum_all", "state_group_counts_sum_all", "signal_group_counts_sum_all",
            "satellite_group_counts_sum_all", "fixed_bin_counts_sum_all", "quantile_counts_sum_all",
            "four_route_medians_retained", "aggregate_recomputed_exact", "loo_route_count_exact",
            "event_count_exact", "header_field_presence_route_map_exact",
        )
    )
    gate_rows = {
        "route_count": len(reports) == 4 and tuple(reports) == ROUTES,
        "raw_input_integrity": all(
            int(report["rows"]["repeated_epoch_key_count"]) == 0
            and int(report["rows"]["nonmonotonic_epoch_key_count"]) == 0
            and bool(report["gate_observations"]["all_core_finite"])
            for report in reports.values()
        ),
        "ordinary_coverage": ordinary_pass,
        "uncertainty_coverage": uncertainty_pass,
        "fixed_bins": bins_pass,
        "routewise_relation": relation_pass,
        "calibration": calibration_pass,
        "loo": loo_pass,
        "presentation_integrity": integrity_pass,
    }
    all_passed = all(gate_rows.values())
    if all_passed:
        status = "go-native-tdcp-adr-uncertainty-floor-implementation-eligibility"
        strongest = "All frozen raw-only ADR uncertainty identifiability gates passed; a separate native implementation seal is authorized, but no correction or truth read is performed in Phase55."
        next_factor = None
    else:
        status = "no-go-adr-uncertainty-not-identifiable"
        failed = [key for key, value in gate_rows.items() if not value]
        strongest = "ADR uncertainty identifiability failed frozen gate(s): " + ", ".join(failed)
        next_factor = freeze["next_single_raw_physical_factor_if_no_go"]
    route_output = {route: _strip_internal(report) for route, report in reports.items()}
    result = {
        "schema_version": SCHEMA, "phase": 55, "execution_label": "Luna Max", "status": status,
        "factor": "raw Android per-satellite AccumulatedDeltaRangeUncertaintyMeters",
        "audit_only": True, "correction_or_solver_change": False,
        "freeze": {"path": _relative(FREEZE), "sha256": FREEZE_SHA256},
        "evaluator_manifest": {"path": _relative(MANIFEST), "sha256": VERIFIED_MANIFEST_SHA256},
        "fixed_contract": freeze["fixed_observable"],
        "static_contract": static,
        "routes": route_output, "aggregate": aggregate, "loo": loo,
        "gates": {"all_passed": all_passed, "observed": gate_rows, "thresholds": freeze["numeric_gates"]},
        "presentation_integrity": integrity,
        "read_accounting": reads,
        "decision": {
            "audit_only": True, "correction_authorized": all_passed,
            "implementation_stage_authorized": all_passed,
            "strongest_finding": strongest,
            "next_single_raw_physical_factor": next_factor,
            "phase43_champion_preserved": True, "phase51_experimental_preserved": True,
            "zero_point_782": "not evaluated without truth",
            "truth_or_navigation_or_coordinate_or_solver_input": False,
        },
    }
    output_root.mkdir(parents=True, exist_ok=False)
    result_path = output_root / "phase55_pixel5_adr_uncertainty.json"
    routes_path = output_root / "phase55_pixel5_adr_uncertainty.routes.json"
    manifest_path = output_root / "phase55_pixel5_adr_uncertainty.manifest.json"
    result_payload = _atomic_json(result_path, result)
    routes_payload = _atomic_json(routes_path, {"schema_version": SCHEMA + ".routes", "phase": 55, "route_order": list(ROUTES), "routes": route_output})
    output_manifest = {
        "schema_version": SCHEMA + ".output-manifest", "status": "atomic-publish-complete", "phase": 55,
        "freeze_sha256": FREEZE_SHA256, "evaluator_manifest_sha256": VERIFIED_MANIFEST_SHA256,
        "read_accounting": reads,
        "artifacts": {
            "result": {"path": _relative(result_path), "bytes": len(result_payload), "sha256": _sha256_bytes(result_payload)},
            "routes": {"path": _relative(routes_path), "bytes": len(routes_payload), "sha256": _sha256_bytes(routes_payload)},
        },
    }
    _atomic_json(manifest_path, output_manifest)
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true", help="verify freeze/manifest without raw reads")
    parser.add_argument("--audit", action="store_true", help="run one-shot four-route raw audit")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    if args.verify_freeze == args.audit:
        parser.error("choose exactly one of --verify-freeze or --audit")
    try:
        freeze = _verify_freeze()
        manifest = _verify_manifest(freeze)
        if args.verify_freeze:
            print("phase55 freeze/evaluator manifest: verified without raw/truth reads")
            return 0
        result = _audit(freeze, manifest, args.output)
        print(json.dumps({
            "status": result["status"], "all_gates_passed": result["gates"]["all_passed"],
            "raw_reads": result["read_accounting"]["raw_device_gnss_read_count_total"],
            "next_factor_if_no_go": result["decision"]["next_single_raw_physical_factor"],
        }, sort_keys=True))
        return 0
    except Phase55Error as exc:
        print(f"phase55 fail-closed: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
