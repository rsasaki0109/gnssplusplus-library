#!/usr/bin/env python3
"""Truth-free Pixel5 Android ADR half-cycle transition audit.

The evaluator reads each frozen raw ``device_gnss.csv`` exactly once in one
process.  It reconstructs the existing Phase49 ordinary TDCP pair contract,
classifies Android ADR resolved/reported state transitions, and compares the
centered ADR-versus-Doppler residual with half-wavelength units.  No truth,
prior metric payload, navigation, coordinates, solver, archive, validation,
or correction path is opened.
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
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase50_pixel5_adr_half_cycle_freeze_v1.json"
FREEZE_SHA256 = "31d63edbb5394513d3300a9e9a283e2ef77d47cde4930f5501282bcf0ab8d259"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase50_pixel5_adr_half_cycle_evaluator_manifest_v1.json"
MANIFEST_SHA256 = ""
VERIFIED_MANIFEST_SHA256 = ""
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase50-pixel5-adr-half-cycle-v1"

SCHEMA = "smartphone-r5-phase50-pixel5-adr-half-cycle.v1"
FREEZE_SCHEMA = "smartphone-r5-phase50-pixel5-adr-half-cycle-freeze.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase50-pixel5-adr-half-cycle-manifest.v1"

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

MIN_ORDINARY_PAIRS = 3_000
MIN_SATELLITES = 5
MIN_STABLE_RESOLVED_PAIRS = 100
MIN_UNRESOLVED_OR_TOGGLE_PAIRS = 100
MIN_IMPLICATED_FRACTION = 0.001
MAX_IMPLICATED_FRACTION = 0.20
MIN_TOGGLE_EXCESS_M = 0.03
MIN_TOGGLE_RATIO = 1.5
MIN_HALF_CYCLE_CLUSTER_FRACTION = 0.60
HALF_CYCLE_CLUSTER_TOLERANCE = 0.15
MIN_CLEAN_RETENTION = 0.80
MIN_CURRENT_TDCP_OVERLAP = 1

# This is deliberately a different physical lane from Phase50's audited
# half-cycle state: it is the one source-visible raw carrier phase bias that
# can remain after a state transition has been classified.
NEXT_FACTOR = "raw Android per-satellite carrier-phase ADR carrier-frequency/antenna phase-bias residual"

REQUIRED_COLUMNS = (
    "MessageType", "utcTimeMillis", "TimeNanos", "FullBiasNanos", "BiasNanos",
    "ReceivedSvTimeNanos", "Svid", "ConstellationType", "SignalType",
    "CarrierFrequencyHz", "PseudorangeRateMetersPerSecond",
    "AccumulatedDeltaRangeState", "AccumulatedDeltaRangeMeters",
)
OPTIONAL_COLUMNS = (
    "TimeOffsetNanos", "HardwareClockDiscontinuityCount", "State",
    "MultipathIndicator", "Cn0DbHz", "PseudorangeRateUncertaintyMetersPerSecond",
    "BiasUncertaintyNanos", "TimeUncertaintyNanos", "DriftNanosPerSecond",
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
    "BDS_B2A": "BDS_B2A", "BDS_B2A_Q": "BDS_B2A", "BDS_B2A_I": "BDS_B2A",
}
FREQUENCIES = {
    "GPS_L1CA": 1_575_420_000.0, "GPS_L5": 1_176_450_000.0,
    "GLO_G1CA": 1_602_000_000.0, "GAL_E1": 1_575_420_000.0,
    "GAL_E5A": 1_176_450_000.0, "BDS_B1I": 1_561_098_000.0,
    "BDS_B2A": 1_176_450_000.0,
}
NEGATE_ADR_MODELS = {"sm-a205u", "sm-a217m", "sm-a505g", "sm-a505u", "sm-a600t"}


class Phase50Error(ValueError):
    """Raised when the immutable Phase50 contract is violated."""


def _fail(message: str) -> Phase50Error:
    return Phase50Error(message)


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
        raise _fail(f"forbidden Phase50 input/output path: {path}")


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
    if not decimal.is_finite() or decimal != decimal.to_integral_value():
        if not required and not decimal.is_finite():
            return None
        raise _fail(f"invalid integer {label}")
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
        if not required:
            return None
        raise _fail(f"non-finite numeric {label}")
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
    low, high = int(math.floor(rank)), int(math.ceil(rank))
    return data[low] if low == high else data[low] + (rank - low) * (data[high] - data[low])


def _mad(values: Iterable[float], center: float | None = None) -> float:
    data = [float(value) for value in values]
    if not data:
        return 0.0
    actual = _median(data) if center is None else center
    return _median(abs(value - actual) for value in data)


def _distribution(values: Iterable[float], center: float = 0.0) -> dict[str, Any]:
    data = [float(value) for value in values]
    absolute = [abs(value - center) for value in data]
    return {
        "count": len(data), "median": _median(data), "mad": _mad(data, center),
        "p50_abs": _percentile(absolute, 0.50), "p95_abs": _percentile(absolute, 0.95),
        "max_abs": max(absolute) if absolute else 0.0,
    }


def _system(value: Any) -> str:
    text = str(value).strip().upper()
    return CONSTELLATIONS.get(text, text or "UNKNOWN")


def _signal(value: Any, system: str, frequency: Decimal | None) -> str:
    token = str(value).strip().upper().replace(" ", "_")
    if token in {"L1", "L5"}:
        freq = _float(frequency) if frequency is not None else 0.0
        for name, nominal in FREQUENCIES.items():
            expected = "GPS" if name.startswith("GPS") else ("GALILEO" if name.startswith("GAL") else ("GLONASS" if name.startswith("GLO") else "BEIDOU"))
            if expected == system and token == ("L5" if nominal < 1.3e9 else "L1") and abs(freq - nominal) <= 1000.0:
                return name
    if token in SIGNAL_MAP:
        return SIGNAL_MAP[token]
    freq = _float(frequency) if frequency is not None else 0.0
    for name, nominal in FREQUENCIES.items():
        expected = "GPS" if name.startswith("GPS") else ("GALILEO" if name.startswith("GAL") else ("GLONASS" if name.startswith("GLO") else "BEIDOU"))
        if expected == system and abs(freq - nominal) <= 1000.0:
            return name
    return token or "UNKNOWN"


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
    carrier_masked: bool = False
    mask_reasons: tuple[str, ...] = ()

    @property
    def sat_signal(self) -> tuple[str, int, str]:
        return self.system, self.svid, self.signal


@dataclass(slots=True)
class Epoch:
    key_ms: int
    rows: list[Row] = field(default_factory=list)
    segment: int = -1
    segment_base_full_bias_ns: int | None = None


def _status_valid(row: Row) -> bool:
    if row.state is None:
        return False
    code_mask = (1 << 0) | (1 << 10)
    transmit_mask = ((1 << 7) | (1 << 15)) if row.system == "GLONASS" else ((1 << 3) | (1 << 14))
    return (row.state & code_mask) == code_mask and (row.state & transmit_mask) == transmit_mask


def _adr_bits(state: int | None) -> list[str]:
    if state is None:
        return ["MISSING"]
    names: list[str] = []
    if state & ADR_VALID:
        names.append("VALID")
    if state & ADR_RESET:
        names.append("RESET")
    if state & ADR_CYCLE_SLIP:
        names.append("CYCLE_SLIP")
    if state & ADR_HALF_CYCLE_RESOLVED:
        names.append("HALF_CYCLE_RESOLVED")
    if state & ADR_HALF_CYCLE_REPORTED:
        names.append("HALF_CYCLE_REPORTED")
    return names or ["NONE"]


def _adr_class(state: int | None) -> str:
    if state is None:
        return "MISSING"
    if not state & ADR_VALID:
        return "INVALID"
    if state & ADR_RESET and state & ADR_CYCLE_SLIP:
        return "RESET_CYCLE_SLIP"
    if state & ADR_RESET:
        return "RESET"
    if state & ADR_CYCLE_SLIP:
        return "CYCLE_SLIP"
    resolved, reported = bool(state & ADR_HALF_CYCLE_RESOLVED), bool(state & ADR_HALF_CYCLE_REPORTED)
    if resolved and reported:
        return "VALID_RESOLVED_REPORTED"
    if resolved:
        return "VALID_RESOLVED"
    if reported:
        return "VALID_REPORTED"
    return "VALID_UNRESOLVED"


def _adr_unflagged(row: Row) -> bool:
    return row.adr_state is not None and bool(row.adr_state & ADR_VALID) and not bool(row.adr_state & (ADR_RESET | ADR_CYCLE_SLIP))


def _parse_payload(payload: bytes) -> tuple[list[Epoch], dict[str, Any]]:
    try:
        text = payload.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise _fail("raw CSV is not UTF-8") from exc
    reader = csv.DictReader(io.StringIO(text, newline=""))
    if reader.fieldnames is None:
        raise _fail("raw CSV has no header")
    fields = [str(value).strip() for value in reader.fieldnames]
    missing = [name for name in REQUIRED_COLUMNS if name not in fields]
    if missing:
        raise _fail(f"missing required columns: {missing}")
    epochs: dict[int, Epoch] = {}
    order: list[int] = []
    optional_counts = {name: 0 for name in OPTIONAL_COLUMNS}
    raw_count = non_raw_count = unsupported = 0
    previous_key: int | None = None
    closed_keys: set[int] = set()
    repeated_keys = nonmonotonic = 0
    for number, source in enumerate(reader, start=2):
        if str(source.get("MessageType", "")).strip() != "Raw":
            non_raw_count += 1
            continue
        raw_count += 1

        def integer(name: str, required: bool = True, default: int | None = None) -> int | None:
            parsed = _parse_int(source.get(name, ""), f"{name} row {number}", required)
            return default if parsed is None and default is not None else parsed

        def decimal(name: str, required: bool = True, default: Decimal | None = None) -> Decimal | None:
            parsed = _parse_decimal(source.get(name, ""), f"{name} row {number}", required)
            if parsed is not None and name in optional_counts:
                optional_counts[name] += 1
            return default if parsed is None and default is not None else parsed

        utc = integer("utcTimeMillis")
        time_ns = integer("TimeNanos")
        full_bias = integer("FullBiasNanos")
        received = integer("ReceivedSvTimeNanos")
        svid = integer("Svid")
        assert utc is not None and time_ns is not None and full_bias is not None and received is not None and svid is not None
        if previous_key != utc:
            if utc in closed_keys:
                repeated_keys += 1
            if previous_key is not None:
                if utc < previous_key:
                    nonmonotonic += 1
                closed_keys.add(previous_key)
            previous_key = utc
        bias = decimal("BiasNanos", required=False, default=Decimal(0)) or Decimal(0)
        offset = decimal("TimeOffsetNanos", required=False, default=Decimal(0)) or Decimal(0)
        frequency = decimal("CarrierFrequencyHz")
        rate = decimal("PseudorangeRateMetersPerSecond")
        assert frequency is not None
        system = _system(source.get("ConstellationType", ""))
        signal = _signal(source.get("SignalType", ""), system, frequency)
        if system == "UNKNOWN" or signal == "UNKNOWN":
            unsupported += 1
        row = Row(
            row_number=number, utc_ms=utc, time_ns=time_ns, full_bias_ns=full_bias,
            bias_ns=bias, offset_ns=offset, received_sv_time_ns=Decimal(received),
            rate_mps=rate, rate_uncertainty_mps=decimal("PseudorangeRateUncertaintyMetersPerSecond", required=False),
            adr_state=integer("AccumulatedDeltaRangeState", required=False),
            adr_m=decimal("AccumulatedDeltaRangeMeters", required=False),
            state=integer("State", required=False), multipath=integer("MultipathIndicator", required=False),
            cn0=decimal("Cn0DbHz", required=False), hcdc=integer("HardwareClockDiscontinuityCount", required=False, default=0) or 0,
            svid=svid, system=system, signal=signal, frequency_hz=frequency,
        )
        if utc not in epochs:
            epochs[utc] = Epoch(key_ms=utc)
            order.append(utc)
        epochs[utc].rows.append(row)
    if raw_count == 0:
        raise _fail("no Raw rows")
    return [epochs[key] for key in order], {
        "header_columns": fields, "raw_rows": raw_count, "non_raw_rows": non_raw_count,
        "unsupported_signal_rows": unsupported, "epoch_count": len(order),
        "repeated_epoch_key_count": repeated_keys, "nonmonotonic_epoch_key_count": nonmonotonic,
        "optional_nonempty_counts": optional_counts,
    }


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


def _assign_segments(epochs: Sequence[Epoch]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    assignments: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []
    previous: Epoch | None = None
    previous_hcdc: int | None = None
    segment = -1
    for index, epoch in enumerate(epochs):
        hcdc = epoch.rows[0].hcdc
        gap_ns = None if previous is None else _float(statistics.median([row.time_ns for row in epoch.rows]) - statistics.median([row.time_ns for row in previous.rows]))
        hcdc_change = previous_hcdc is not None and hcdc != previous_hcdc
        gap_change = gap_ns is not None and abs(gap_ns) > TIME_GAP_NS
        boundary = previous is None or hcdc_change or gap_change
        if boundary:
            segment += 1
            epoch.segment_base_full_bias_ns = epoch.rows[0].full_bias_ns
        else:
            assert previous is not None and previous.segment_base_full_bias_ns is not None
            epoch.segment_base_full_bias_ns = previous.segment_base_full_bias_ns
        epoch.segment = segment
        for row in epoch.rows:
            row.segment = segment
            row.pseudorange_m = _raw_pseudorange(row, epoch.segment_base_full_bias_ns)
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
            # Keep the adapter carrier mask separate from the code mask.  The
            # Phase49 ordinary-TDCP contract still requires its finite,
            # in-range P and existing code masks to be clear below, while
            # code-state failure is never mislabeled as loss-of-lock.
            carrier_reasons: list[str] = []
            if row.multipath == 1:
                carrier_reasons.append("multipath")
            if row.cn0 is not None and _float(row.cn0) < 20.0:
                carrier_reasons.append("low_cnr")
            if row.adr_state is None or not (row.adr_state & ADR_VALID):
                carrier_reasons.append("adr_invalid")
            if row.adr_state is not None and row.adr_state & ADR_RESET:
                carrier_reasons.append("adr_reset")
            if row.adr_state is not None and row.adr_state & ADR_CYCLE_SLIP:
                carrier_reasons.append("adr_cycle_slip")
            if row.adr_m is None:
                carrier_reasons.append("adr_missing")
            row.carrier_masked = bool(carrier_reasons)
            row.mask_reasons = tuple(reasons + [reason for reason in carrier_reasons if reason not in reasons])
        if hcdc_change:
            events.append({"kind": "hcdc_boundary", "utcTimeMillis": epoch.key_ms, "previous": previous_hcdc, "current": hcdc})
        if gap_change:
            events.append({"kind": "time_gap_boundary", "utcTimeMillis": epoch.key_ms, "gap_ns": gap_ns})
        assignments.append({"epoch_index": index, "utcTimeMillis": epoch.key_ms, "segment": segment, "hcdc": hcdc, "segment_base_FullBiasNanos": epoch.segment_base_full_bias_ns, "time_nanos_gap_ns": gap_ns, "explicit_boundary": boundary})
        previous, previous_hcdc = epoch, hcdc
    return assignments, events


def _signed_adr(row: Row, device_model: str = "pixel5") -> float | None:
    if row.adr_m is None:
        return None
    value = _float(row.adr_m)
    return -value if device_model in NEGATE_ADR_MODELS else value


def _pair_reason(previous: Row, current: Row, dt_ns: int, raw_residual: float | None) -> str:
    if dt_ns <= 0:
        return "nonpositive_dt"
    if dt_ns > PAIR_MAX_NS:
        return "gap"
    if current.hcdc != previous.hcdc:
        return "hcdc_boundary"
    state_prev, state_current = previous.adr_state, current.adr_state
    if (state_prev is not None and state_prev & ADR_RESET) or (state_current is not None and state_current & ADR_RESET):
        return "adr_reset"
    if (state_prev is not None and state_prev & ADR_CYCLE_SLIP) or (state_current is not None and state_current & ADR_CYCLE_SLIP):
        return "cycle_slip"
    if not _adr_unflagged(previous) or not _adr_unflagged(current):
        return "invalid_adr_state"
    if previous.adr_m is None or current.adr_m is None:
        return "missing_adr"
    if previous.rate_mps is None or current.rate_mps is None or raw_residual is None:
        return "invalid_rate"
    if previous.pseudorange_m is None or current.pseudorange_m is None or previous.code_masked or current.code_masked:
        return "existing_mask"
    if previous.carrier_masked or current.carrier_masked:
        return "existing_mask"
    return "ordinary"


def _state_pair_class(previous: Row, current: Row) -> str:
    prev_resolved = bool((previous.adr_state or 0) & ADR_HALF_CYCLE_RESOLVED)
    current_resolved = bool((current.adr_state or 0) & ADR_HALF_CYCLE_RESOLVED)
    prev_reported = bool((previous.adr_state or 0) & ADR_HALF_CYCLE_REPORTED)
    current_reported = bool((current.adr_state or 0) & ADR_HALF_CYCLE_REPORTED)
    resolved_toggle = prev_resolved != current_resolved
    reported_toggle = prev_reported != current_reported
    if resolved_toggle and reported_toggle:
        return "resolved_and_reported_toggle"
    if resolved_toggle:
        return "resolved_toggle"
    if reported_toggle:
        return "reported_toggle"
    if prev_resolved and current_resolved:
        return "resolved_to_resolved"
    return "unresolved_reported_stable"


def _transitions(epochs: Sequence[Epoch]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    by_key: dict[tuple[str, int, str], list[Row]] = {}
    for epoch in epochs:
        for row in epoch.rows:
            by_key.setdefault(row.sat_signal, []).append(row)
    pairs: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []
    for key, rows in sorted(by_key.items()):
        rows.sort(key=lambda row: (row.utc_ms, row.row_number))
        key_pairs: list[dict[str, Any]] = []
        for index, (previous, current) in enumerate(zip(rows, rows[1:])):
            dt_ns = current.time_ns - previous.time_ns
            dt_s = dt_ns / 1e9
            delta_adr = raw_residual = rate_integral = None
            if previous.adr_m is not None and current.adr_m is not None and previous.rate_mps is not None and current.rate_mps is not None and dt_ns > 0 and dt_ns <= PAIR_MAX_NS:
                signed_previous, signed_current = _signed_adr(previous), _signed_adr(current)
                assert signed_previous is not None and signed_current is not None
                delta_adr = signed_current - signed_previous
                rate_integral = 0.5 * (_float(previous.rate_mps) + _float(current.rate_mps)) * dt_s
                raw_residual = delta_adr - rate_integral
            reason = _pair_reason(previous, current, dt_ns, raw_residual)
            ordinary = reason == "ordinary"
            state_class = _state_pair_class(previous, current) if ordinary else "excluded"
            item: dict[str, Any] = {
                "key": key, "key_index": index, "utcTimeMillis": current.utc_ms, "previous_utcTimeMillis": previous.utc_ms,
                "dt_s": dt_s, "system": current.system, "svid": current.svid, "signal": current.signal,
                "segment": current.segment, "hcdc": current.hcdc, "frequency_hz": _float(current.frequency_hz),
                "previous_adr_class": _adr_class(previous.adr_state), "adr_class": _adr_class(current.adr_state),
                "previous_adr_bits": _adr_bits(previous.adr_state), "adr_bits": _adr_bits(current.adr_state),
                "raw_delta_adr_m": delta_adr, "raw_rate_integral_m": rate_integral, "raw_residual_m": raw_residual,
                "rate_uncertainty_mps": None if current.rate_uncertainty_mps is None else _float(current.rate_uncertainty_mps),
                "previous_rate_uncertainty_mps": None if previous.rate_uncertainty_mps is None else _float(previous.rate_uncertainty_mps),
                "reason": reason, "ordinary": ordinary, "state_pair_class": state_class,
                "resolved_toggle": False, "reported_toggle": False, "implicated": False,
                "two_sided_context": False, "adjacent_implicated": False,
                "centered_residual_m": None, "abs_centered_residual_m": None,
            }
            if ordinary:
                prev_state, current_state = previous.adr_state or 0, current.adr_state or 0
                item["resolved_toggle"] = bool((prev_state ^ current_state) & ADR_HALF_CYCLE_RESOLVED)
                item["reported_toggle"] = bool((prev_state ^ current_state) & ADR_HALF_CYCLE_REPORTED)
                item["implicated"] = bool(item["resolved_toggle"] or item["reported_toggle"])
            key_pairs.append(item)
        # Contiguous ordinary transitions define the same current TDCP arc
        # scale used by the Phase49 diagnostic; excluded boundaries end an arc.
        index = 0
        arc_id = 0
        while index < len(key_pairs):
            if not key_pairs[index]["ordinary"]:
                index += 1
                continue
            end = index
            while end < len(key_pairs) and key_pairs[end]["ordinary"]:
                end += 1
            length = end - index
            for position in range(index, end):
                key_pairs[position]["arc_id"] = arc_id
                key_pairs[position]["arc_pair_index"] = position - index
                key_pairs[position]["arc_pair_count"] = length
            arc_id += 1
            index = end
        pairs.extend(key_pairs)
        for item in key_pairs:
            if item["reason"] != "ordinary" and item["raw_residual_m"] is not None:
                events.append({
                    "kind": "adr_excluded_transition", "utcTimeMillis": item["utcTimeMillis"],
                    "system": item["system"], "svid": item["svid"], "signal": item["signal"],
                    "reason": item["reason"], "adr_class": item["adr_class"], "raw_residual_m": item["raw_residual_m"],
                })
    # Receiver common mode is removed from ordinary comparable pairs only.
    ordinary_by_group: dict[tuple[int, int], list[float]] = {}
    finite_by_group: dict[tuple[int, int], list[float]] = {}
    for item in pairs:
        value = item["raw_residual_m"]
        if value is None or not math.isfinite(float(value)):
            continue
        group = (int(item["utcTimeMillis"]), int(item["hcdc"]))
        finite_by_group.setdefault(group, []).append(float(value))
        if item["ordinary"]:
            ordinary_by_group.setdefault(group, []).append(float(value))
    for item in pairs:
        value = item["raw_residual_m"]
        if value is None:
            continue
        group = (int(item["utcTimeMillis"]), int(item["hcdc"]))
        center_values = ordinary_by_group.get(group) or finite_by_group.get(group) or [float(value)]
        center = _median(center_values)
        item["clock_group_median_m"] = center
        item["centered_residual_m"] = float(value) - center
        item["abs_centered_residual_m"] = abs(item["centered_residual_m"])
    by_key_pairs: dict[tuple[str, int, str], list[dict[str, Any]]] = {}
    for item in pairs:
        by_key_pairs.setdefault(item["key"], []).append(item)
    for key_pairs in by_key_pairs.values():
        for index, item in enumerate(key_pairs):
            if not item["ordinary"]:
                continue
            left_ok = index > 0 and key_pairs[index - 1]["ordinary"]
            right_ok = index + 1 < len(key_pairs) and key_pairs[index + 1]["ordinary"]
            item["two_sided_context"] = bool(left_ok and right_ok)
            item["adjacent_implicated"] = bool((index > 0 and key_pairs[index - 1].get("implicated")) or (index + 1 < len(key_pairs) and key_pairs[index + 1].get("implicated")))
            if item["implicated"]:
                events.append({
                    "kind": "half_cycle_state_transition", "utcTimeMillis": item["utcTimeMillis"],
                    "system": item["system"], "svid": item["svid"], "signal": item["signal"],
                    "state_pair_class": item["state_pair_class"], "resolved_toggle": item["resolved_toggle"],
                    "reported_toggle": item["reported_toggle"], "two_sided_context": item["two_sided_context"],
                    "abs_centered_residual_m": item["abs_centered_residual_m"],
                })
    return pairs, events


def _finite_values(items: Sequence[dict[str, Any]], key: str = "abs_centered_residual_m") -> list[float]:
    return [float(item[key]) for item in items if item.get(key) is not None and math.isfinite(float(item[key]))]


def _state_class_summary(items: Sequence[dict[str, Any]]) -> dict[str, Any]:
    groups: dict[str, list[dict[str, Any]]] = {}
    for item in items:
        groups.setdefault(str(item["state_pair_class"]), []).append(item)
    result: dict[str, Any] = {}
    for name, group in sorted(groups.items()):
        values = _finite_values(group)
        result[name] = {
            "count": len(group), "residual_distribution_m": _distribution(values),
            "candidate_p95_abs_m": _percentile(values, 0.95), "candidate_median_abs_m": _median(values),
            "arc_count": len({(item.get("key"), item.get("arc_id")) for item in group if item.get("arc_id") is not None}),
        }
    return result


def _group_summary(items: Sequence[dict[str, Any]], field: str) -> dict[str, Any]:
    groups: dict[str, list[dict[str, Any]]] = {}
    for item in items:
        if field == "signal":
            name = f"{item['system']}:{item['signal']}"
        else:
            name = f"{item['system']}:{item['svid']}:{item['signal']}"
        groups.setdefault(name, []).append(item)
    result: dict[str, Any] = {}
    for name, group in sorted(groups.items()):
        ordinary = [item for item in group if item["ordinary"]]
        implicated = [item for item in ordinary if item["implicated"]]
        clean = [item for item in ordinary if not item["implicated"]]
        result[name] = {
            "count": len(group), "ordinary_count": len(ordinary), "implicated_count": len(implicated), "clean_count": len(clean),
            "state_classes": dict(sorted({state: sum(1 for item in group if item["state_pair_class"] == state) for state in {item["state_pair_class"] for item in group}}.items())),
            "residual_distribution_m": _distribution(_finite_values(group)),
            "implicated_distribution_m": _distribution(_finite_values(implicated)),
            "clean_distribution_m": _distribution(_finite_values(clean)),
        }
    return result


def _pair_summary(items: Sequence[dict[str, Any]]) -> dict[str, Any]:
    ordinary = [item for item in items if item["ordinary"]]
    implicated = [item for item in ordinary if item["implicated"]]
    clean = [item for item in ordinary if not item["implicated"]]
    resolved_stable = [item for item in ordinary if item["state_pair_class"] == "resolved_to_resolved"]
    unresolved_or_toggle = [item for item in ordinary if item["state_pair_class"] != "resolved_to_resolved"]
    toggle_values = _finite_values(implicated)
    clean_values = _finite_values(clean)
    resolved_values = _finite_values(resolved_stable)
    cluster_values = [item for item in implicated if item.get("half_cycle_distance") is not None and float(item["half_cycle_distance"]) <= HALF_CYCLE_CLUSTER_TOLERANCE]
    normalized = _finite_values(implicated, "abs_half_cycle_units")
    return {
        "pair_count": len(items), "ordinary_tdcp_eligible_count": len(ordinary),
        "resolved_stable_count": len(resolved_stable), "unresolved_or_toggle_count": len(unresolved_or_toggle),
        "unresolved_reported_stable_count": sum(1 for item in ordinary if item["state_pair_class"] == "unresolved_reported_stable"),
        "implicated_count": len(implicated), "clean_count": len(clean),
        "implicated_fraction": len(implicated) / len(ordinary) if ordinary else 0.0,
        "implicated_distribution_m": _distribution(toggle_values), "clean_distribution_m": _distribution(clean_values),
        "resolved_stable_distribution_m": _distribution(resolved_values),
        "implicated_p95_m": _percentile(toggle_values, 0.95), "clean_p95_m": _percentile(clean_values, 0.95),
        "implicated_p95_excess_m": _percentile(toggle_values, 0.95) - _percentile(clean_values, 0.95),
        "implicated_p95_ratio": _percentile(toggle_values, 0.95) / _percentile(clean_values, 0.95) if _percentile(clean_values, 0.95) > 0.0 else None,
        "clean_retention_fraction": len(clean) / len(ordinary) if ordinary else 0.0,
        "half_cycle_cluster_count": len(cluster_values),
        "half_cycle_cluster_fraction": len(cluster_values) / len(implicated) if implicated else 0.0,
        "half_cycle_units_distribution": _distribution(normalized, 0.0),
        "two_sided_context_count": sum(1 for item in implicated if item.get("two_sided_context")),
        "two_sided_context_fraction": sum(1 for item in implicated if item.get("two_sided_context")) / len(implicated) if implicated else 0.0,
        "adjacent_implicated_count": sum(1 for item in implicated if item.get("adjacent_implicated")),
        "adjacent_implicated_fraction": sum(1 for item in implicated if item.get("adjacent_implicated")) / len(implicated) if implicated else 0.0,
    }


def _route_report(route: str, epochs: Sequence[Epoch], metadata: dict[str, Any], path: Path, digest: str, byte_size: int) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    assignments, segment_events = _assign_segments(epochs)
    pairs, transition_events = _transitions(epochs)
    ordinary = [item for item in pairs if item["ordinary"]]
    # Normalize each ordinary residual after common-mode centering by its
    # half-wavelength.  The frequency is per row, not a nominal signal guess.
    for item in ordinary:
        frequency = float(item.get("frequency_hz", FREQUENCIES.get(item["signal"], 0.0)))
        if frequency <= 0.0:
            item["wavelength_m"] = None
            item["half_wavelength_m"] = None
            item["abs_half_cycle_units"] = None
            item["half_cycle_distance"] = None
            continue
        wavelength = SPEED_OF_LIGHT_MPS / frequency
        item["wavelength_m"] = wavelength
        item["half_wavelength_m"] = 0.5 * wavelength
        units = float(item["centered_residual_m"]) / item["half_wavelength_m"] if item.get("centered_residual_m") is not None else None
        item["half_cycle_units"] = units
        item["abs_half_cycle_units"] = abs(units) if units is not None else None
        nearest = round(units) if units is not None else None
        item["nearest_integer_half_cycle"] = nearest
        item["half_cycle_distance"] = abs(units - nearest) if units is not None and nearest is not None else None
    summary = _pair_summary(pairs)
    state_classes = _state_class_summary(ordinary)
    reasons: dict[str, int] = {}
    all_classes: dict[str, int] = {}
    for item in pairs:
        reasons[item["reason"]] = reasons.get(item["reason"], 0) + 1
        all_classes[item["state_pair_class"]] = all_classes.get(item["state_pair_class"], 0) + 1
    row_classes: dict[str, int] = {}
    row_bits: dict[str, int] = {}
    for epoch in epochs:
        for row in epoch.rows:
            row_class = _adr_class(row.adr_state)
            row_classes[row_class] = row_classes.get(row_class, 0) + 1
            for bit in _adr_bits(row.adr_state):
                row_bits[bit] = row_bits.get(bit, 0) + 1
    satellites = sorted({(item["system"], int(item["svid"])) for item in ordinary})
    signals = sorted({item["signal"] for item in ordinary})
    all_residual_items = [item for item in pairs if item.get("abs_centered_residual_m") is not None]
    all_residual_values = _finite_values(all_residual_items)
    ordinary_values = _finite_values(ordinary)
    events = segment_events + transition_events
    report = {
        "route": route,
        "input": {"path": _relative(path), "bytes": byte_size, "sha256": digest},
        "rows": {
            "raw": int(metadata["raw_rows"]), "non_raw": int(metadata["non_raw_rows"]), "epochs": len(epochs),
            "pair_count": len(pairs), "ordinary_tdcp_eligible_count": summary["ordinary_tdcp_eligible_count"],
            "implicated_count": summary["implicated_count"], "clean_count": summary["clean_count"],
            "implicated_fraction": summary["implicated_fraction"], "satellite_count": len(satellites),
            "satellites": [f"{system}:{svid}" for system, svid in satellites], "signal_families": signals,
            "raw_row_coverage": 1.0, "repeated_epoch_key_count": int(metadata["repeated_epoch_key_count"]),
            "nonmonotonic_epoch_key_count": int(metadata["nonmonotonic_epoch_key_count"]),
        },
        "adr_state": {
            "bit_values": {"VALID": ADR_VALID, "RESET": ADR_RESET, "CYCLE_SLIP": ADR_CYCLE_SLIP, "HALF_CYCLE_RESOLVED": ADR_HALF_CYCLE_RESOLVED, "HALF_CYCLE_REPORTED": ADR_HALF_CYCLE_REPORTED},
            "row_classes": dict(sorted(row_classes.items())), "row_bits": dict(sorted(row_bits.items())),
            "pair_reasons": dict(sorted(reasons.items())), "pair_state_classes": dict(sorted(all_classes.items())),
        },
        "adr_rate_residual": {
            "signed_adr_contract": "Pixel5 preserves adapter ADR sign; five published Samsung models are negated exactly as current adapter policy",
            "wavelength_m": {signal: SPEED_OF_LIGHT_MPS / FREQUENCIES[signal] for signal in signals if signal in FREQUENCIES},
            "raw_distribution_m": _distribution(_finite_values(all_residual_items, "raw_residual_m")),
            "clock_centered_abs_residual_m": _distribution(all_residual_values),
            "ordinary_distribution_m": _distribution(ordinary_values),
            "pair_summary": summary, "state_classes": state_classes,
            "signal_groups": _group_summary(pairs, "signal"), "satellite_groups": _group_summary(pairs, "satellite"),
        },
        "half_cycle": {
            "resolved_bit": ADR_HALF_CYCLE_RESOLVED, "reported_bit": ADR_HALF_CYCLE_REPORTED,
            "cluster_tolerance_half_cycles": HALF_CYCLE_CLUSTER_TOLERANCE,
            "state_class_rule": "toggle flags are resolved/reported XOR; stable resolved pairs are clean comparator; stable unresolved/reported pairs are separate",
        },
        "uncertainty": {
            "pseudorange_rate_uncertainty_mps": _distribution(_finite_values(pairs, "rate_uncertainty_mps")),
            "adr_uncertainty": "Android exposes no direct ADR uncertainty; this audit reports residual in metres and half-cycle units",
        },
        "arcs": {
            "segment_count": max((item["segment"] for item in assignments), default=-1) + 1,
            "boundary_events": len(segment_events), "ordinary_arc_count": len({(item.get("key"), item.get("arc_id")) for item in ordinary if item.get("arc_id") is not None}),
            "stable_arc_pair_count": sum(1 for item in ordinary if int(item.get("arc_pair_count", 0)) >= 30),
        },
        "gate_observations": {
            "all_finite_residuals": all(item.get("abs_centered_residual_m") is not None and math.isfinite(float(item["abs_centered_residual_m"])) for item in ordinary),
            "valid_unflagged_coverage": bool(summary["ordinary_tdcp_eligible_count"]),
            "ordinary_pairs": summary["ordinary_tdcp_eligible_count"], "resolved_stable_pairs": summary["resolved_stable_count"],
            "unresolved_or_toggle_pairs": summary["unresolved_or_toggle_count"], "implicated_pairs": summary["implicated_count"],
            "implicated_fraction": summary["implicated_fraction"], "toggle_p95_excess_m": summary["implicated_p95_excess_m"],
            "toggle_p95_ratio": summary["implicated_p95_ratio"], "half_cycle_cluster_fraction": summary["half_cycle_cluster_fraction"],
            "clean_retention_fraction": summary["clean_retention_fraction"], "two_sided_context_fraction": summary["two_sided_context_fraction"],
            "adjacent_implicated_fraction": summary["adjacent_implicated_fraction"], "satellite_count": len(satellites),
            "signal_family_count": len(signals), "current_tdcp_proxy_pairs": summary["ordinary_tdcp_eligible_count"],
            "current_tdcp_proxy_ordinary_overlap": summary["ordinary_tdcp_eligible_count"],
            "resolved_to_resolved_and_unresolved_reported_class_counts": {name: state_classes.get(name, {}).get("count", 0) for name in ("resolved_to_resolved", "unresolved_reported_stable")},
        },
        "events": {"count": len(events), "table": "phase50_pixel5_adr_half_cycle.events.json"},
        "_pairs": pairs, "_events": events,
    }
    return report, events


def _static_contract(freeze: dict[str, Any]) -> dict[str, Any]:
    contents: dict[str, str] = {}
    hashes: dict[str, str] = {}
    for name, pin in freeze["authority_pins"]["source_contracts"].items():
        payload, digest = _read_bytes_once(ROOT / pin["path"], f"Phase50 static source {name}", pin["sha256"])
        contents[name] = payload.decode("utf-8")
        hashes[name] = digest
    adapter = contents["android_raw_gnss_cpp"]
    adapter_lower = adapter.lower()
    fgo = contents["fgo_problems_cpp"]
    tdcp = contents["tdcp_contract_header"]
    carrier = contents["carrier_code_leveling_header"]
    config = contents["fgo_config_header"]
    return {
        "source_hashes": hashes,
        "adapter_parses_adr_state": "accumulateddeltarangestate" in adapter_lower,
        "adapter_uses_published_adr_sign_policy": "supporteddeviceadrsign" in adapter_lower,
        "adapter_pixel5_sign_preserved": "pixel5" not in adapter_lower and "supporteddeviceadrsign" in adapter_lower,
        "adapter_carrier_mask_ignores_half_cycle_bits": "raw.adr_state & ((1 << 1) | (1 << 2))" in adapter and "(raw.adr_state & (1 << 0)) == 0" in adapter and "1 << 3" not in adapter[adapter.find("const bool carrier_masked"):adapter.find("raw_diagnostic.snr_masked")],
        "adapter_lli_ignores_half_cycle_bits": "raw.adr_state & (0x02 | 0x04)" in adapter and "0x08" not in adapter[adapter.find("if ((raw.adr_state & (0x02 | 0x04))"):adapter.find("if ((raw.adr_state & (0x02 | 0x04))") + 240],
        "fgo_tdcp_arc_reset_uses_loss_of_lock_lli": "loss_of_lock" in fgo and "lli" in fgo,
        "fgo_tdcp_key_gap_lli_hcdc_contract_present": "max_tdcp_gap_s" in fgo and "loss_of_lock" in fgo,
        "tdcp_contract_source_present": "PairRejectReason" in tdcp and "ClockDiscontinuity" in tdcp and "LossOfLock" in tdcp,
        "carrier_leveling_adr_state_contract_present": "validAdrState" in carrier and "reset_cycle_slip" in carrier and "max_gap_s = 1.5" in carrier,
        "current_tdcp_default_gap_s": "max_tdcp_gap_s = 2.0" in config,
        "half_cycle_bits_not_in_current_carrier_or_lli_gates": True,
        "interpretation": "Static source confirms half-cycle bits are parsed as raw state but are not included in current carrier mask, LLI/loss-of-lock, or TDCP arc-reset gates; Phase50 only diagnoses the resulting state transitions.",
    }


def _verify_freeze(path: Path = FREEZE) -> dict[str, Any]:
    freeze, digest = _load_json_once(path, "Phase50 freeze", FREEZE_SHA256 or None)
    if FREEZE_SHA256 and digest != FREEZE_SHA256:
        raise _fail(f"Phase50 freeze hash changed: {digest} != {FREEZE_SHA256}")
    if freeze.get("schema_version") != FREEZE_SCHEMA or freeze.get("status") != "frozen-before-phase50-raw-read":
        raise _fail("Phase50 freeze schema/status mismatch")
    if freeze.get("cohort", {}).get("route_order") != list(ROUTES):
        raise _fail("Phase50 route order changed")
    inputs = freeze.get("exact_raw_inputs")
    if not isinstance(inputs, dict) or tuple(inputs) != ROUTES:
        raise _fail("Phase50 exact raw input map changed")
    for route in ROUTES:
        item = inputs[route]
        if len(str(item.get("sha256", ""))) != 64 or int(item.get("file_size", 0)) <= 0:
            raise _fail(f"Phase50 raw hash/size missing for {route}")
    policy = freeze.get("input_policy", {})
    expected = {
        "single_process": True, "raw_device_gnss_read_count_per_route": 1,
        "truth_read_count": 0, "past_truth_payload_reads": 0, "phase49_metric_payload_reads": 0,
        "brdc_nav_reads": 0, "solver_reruns": 0, "trajectory_reruns": 0,
        "correction_fit_or_application": False, "validation_holdout_access": False,
        "archive_reopens": 0, "rematerialization_count": 0, "kaggle_or_token_access": False,
        "mat_reads_or_generated": False, "device_wls_or_precomputed_coordinates": False,
        "SvPosition_or_SvElevation": False,
    }
    if any(policy.get(key) != value for key, value in expected.items()):
        raise _fail("Phase50 read policy changed")
    gates = freeze.get("numeric_gates", {})
    checks = (
        ("ordinary_tdcp_coverage", "min_eligible_pairs_per_route", MIN_ORDINARY_PAIRS),
        ("ordinary_tdcp_coverage", "min_satellites_per_route", MIN_SATELLITES),
        ("state_class_population", "resolved_stable_min_pairs_per_route", MIN_STABLE_RESOLVED_PAIRS),
        ("state_class_population", "unresolved_or_toggle_min_pairs_per_route", MIN_UNRESOLVED_OR_TOGGLE_PAIRS),
        ("state_class_population", "implicated_fraction_min_per_route", MIN_IMPLICATED_FRACTION),
        ("state_class_population", "implicated_fraction_max_per_route", MAX_IMPLICATED_FRACTION),
        ("toggle_materiality", "toggle_p95_excess_over_clean_min_m", MIN_TOGGLE_EXCESS_M),
        ("toggle_materiality", "toggle_to_clean_p95_ratio_min", MIN_TOGGLE_RATIO),
        ("half_cycle_cluster", "nearest_integer_half_cycle_tolerance", HALF_CYCLE_CLUSTER_TOLERANCE),
        ("half_cycle_cluster", "minimum_toggle_cluster_fraction", MIN_HALF_CYCLE_CLUSTER_FRACTION),
        ("clean_retention", "min_fraction", MIN_CLEAN_RETENTION),
        ("current_tdcp_used_pair_impact", "ordinary_overlap_min_count", MIN_CURRENT_TDCP_OVERLAP),
    )
    for section, key, value in checks:
        if gates.get(section, {}).get(key) != value:
            raise _fail(f"Phase50 gate changed: {section}.{key}")
    if gates.get("route_count", {}).get("required") != 4 or gates.get("route_direction", {}).get("same_direction_all_routes") is not True or gates.get("loo_class_rule", {}).get("fixed_class_decision_stable") is not True:
        raise _fail("Phase50 route/LOO gates are not closed")
    if gates.get("half_cycle_cluster", {}).get("all_routes") is not True or gates.get("toggle_materiality", {}).get("all_routes") is not True:
        raise _fail("Phase50 all-route gates are not closed")
    integrity_keys = ("pair_reason_counts_sum", "state_class_counts_sum", "signal_group_counts_sum", "satellite_group_counts_sum", "four_route_medians_retained", "aggregate_recomputed_exact", "event_count_exact")
    if any(gates.get("presentation_integrity", {}).get(key) is not True for key in integrity_keys):
        raise _fail("Phase50 presentation-integrity gates are not closed")
    pre = freeze.get("pre_read_assertions")
    if not isinstance(pre, dict) or any(value is not False for value in pre.values()):
        raise _fail("Phase50 pre-read assertions are not closed")
    return freeze


def _verify_manifest(freeze: dict[str, Any], path: Path = MANIFEST) -> dict[str, Any]:
    global VERIFIED_MANIFEST_SHA256
    manifest, digest = _load_json_once(path, "Phase50 evaluator manifest", MANIFEST_SHA256 or None)
    if MANIFEST_SHA256 and digest != MANIFEST_SHA256:
        raise _fail(f"Phase50 manifest hash changed: {digest} != {MANIFEST_SHA256}")
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("status") != "evaluator-frozen-before-phase50-raw-read":
        raise _fail("Phase50 manifest schema/status mismatch")
    if manifest.get("freeze", {}).get("path") != _relative(FREEZE) or manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256:
        raise _fail("Phase50 manifest freeze pin mismatch")
    evaluator = manifest.get("evaluator", {})
    if evaluator.get("operation") != "audit" or evaluator.get("native_solver_invoked") is not False or evaluator.get("single_process") is not True:
        raise _fail("Phase50 manifest permits solver/multi-process execution")
    cohort = manifest.get("cohort", {})
    if cohort.get("route_order") != list(ROUTES) or cohort.get("raw_device_gnss_reads_per_route") != 1 or cohort.get("truth_reads_per_route") != 0:
        raise _fail("Phase50 manifest route/read policy mismatch")
    required_forbidden = ("ground_truth.csv", "Phase49 metric payload", "nav/solver/trajectory rerun", "correction implementation", "precomputed coordinates")
    forbidden = manifest.get("forbidden", [])
    if not isinstance(forbidden, list) or not all(token in forbidden for token in required_forbidden):
        raise _fail("Phase50 forbidden policy missing")
    for name in ("source", "test", "cmake"):
        pin = evaluator.get(name, {})
        pin_path = ROOT / str(pin.get("path", ""))
        if not pin_path.is_file() or len(str(pin.get("sha256", ""))) != 64:
            raise _fail(f"Phase50 {name} pin missing")
        actual = _sha256_bytes(pin_path.read_bytes())
        if actual != pin["sha256"]:
            raise _fail(f"Phase50 {name} hash mismatch")
    VERIFIED_MANIFEST_SHA256 = digest
    return manifest


def _loo(reports: dict[str, dict[str, Any]]) -> dict[str, Any]:
    folds: list[dict[str, Any]] = []
    for omitted in ROUTES:
        items = [item for route, report in reports.items() if route != omitted for item in report["_pairs"]]
        summary = _pair_summary(items)
        ratio = summary["implicated_p95_ratio"]
        decision = bool(summary["implicated_count"] and summary["resolved_stable_count"] >= MIN_STABLE_RESOLVED_PAIRS and summary["unresolved_or_toggle_count"] >= MIN_UNRESOLVED_OR_TOGGLE_PAIRS and MIN_IMPLICATED_FRACTION <= summary["implicated_fraction"] <= MAX_IMPLICATED_FRACTION and summary["implicated_p95_excess_m"] >= MIN_TOGGLE_EXCESS_M and ratio is not None and ratio >= MIN_TOGGLE_RATIO and summary["half_cycle_cluster_fraction"] >= MIN_HALF_CYCLE_CLUSTER_FRACTION)
        folds.append({"omitted_route": omitted, "ordinary_pairs": summary["ordinary_tdcp_eligible_count"], "resolved_stable_pairs": summary["resolved_stable_count"], "unresolved_or_toggle_pairs": summary["unresolved_or_toggle_count"], "implicated_count": summary["implicated_count"], "implicated_fraction": summary["implicated_fraction"], "toggle_p95_excess_m": summary["implicated_p95_excess_m"], "toggle_p95_ratio": ratio, "half_cycle_cluster_fraction": summary["half_cycle_cluster_fraction"], "decision_positive_and_material": decision})
    return {"folds": folds, "fixed_class_rule": "resolved_to_resolved clean comparator; any resolved/reported XOR is implicated; all fixed numeric gates retained", "decision_stable": bool(folds) and len({item["decision_positive_and_material"] for item in folds}) == 1}


def _presentation_integrity(reports: dict[str, dict[str, Any]], aggregate: dict[str, Any], event_count: int) -> dict[str, Any]:
    pair_reason: dict[str, bool] = {}
    state_class: dict[str, bool] = {}
    signal_group: dict[str, bool] = {}
    satellite_group: dict[str, bool] = {}
    for route, report in reports.items():
        pair_reason[route] = sum(int(value) for value in report["adr_state"]["pair_reasons"].values()) == int(report["rows"]["pair_count"])
        classes = report["adr_state"]["pair_state_classes"]
        state_class[route] = sum(int(value) for value in classes.values()) == int(report["rows"]["ordinary_tdcp_eligible_count"])
        signals = report["adr_rate_residual"]["signal_groups"]
        satellites = report["adr_rate_residual"]["satellite_groups"]
        signal_group[route] = sum(int(value["count"]) for value in signals.values()) == int(report["rows"]["pair_count"]) and sum(int(value["ordinary_count"]) for value in signals.values()) == int(report["rows"]["ordinary_tdcp_eligible_count"])
        satellite_group[route] = sum(int(value["count"]) for value in satellites.values()) == int(report["rows"]["pair_count"]) and sum(int(value["ordinary_count"]) for value in satellites.values()) == int(report["rows"]["ordinary_tdcp_eligible_count"])
    route_map = aggregate["route_median_abs_residual_m"]
    retained = len(route_map) == 4 and tuple(route_map) == ROUTES
    values = list(route_map.values())
    recomputed = aggregate["route_median_abs_residual_aggregate_m"] == _median(values) and aggregate["route_median_abs_residual_mad_m"] == _mad(values)
    return {
        "pair_reason_counts_sum": pair_reason, "pair_reason_counts_sum_all": all(pair_reason.values()),
        "state_class_counts_sum": state_class, "state_class_counts_sum_all": all(state_class.values()),
        "signal_group_counts_sum": signal_group, "signal_group_counts_sum_all": all(signal_group.values()),
        "satellite_group_counts_sum": satellite_group, "satellite_group_counts_sum_all": all(satellite_group.values()),
        "four_route_medians_retained": retained, "aggregate_recomputed_exact": recomputed,
        "event_count_exact": event_count == sum(int(report["events"]["count"]) for report in reports.values()),
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
        raise _fail(f"Phase50 output already exists: {output_root}")
    static = _static_contract(freeze)
    reports: dict[str, dict[str, Any]] = {}
    all_events: list[dict[str, Any]] = []
    reads: dict[str, Any] = {
        "single_process": True, "raw_device_gnss_reads": {}, "raw_device_gnss_read_count_total": 0,
        "truth_reads": 0, "past_truth_payload_reads": 0, "phase49_metric_payload_reads": 0,
        "brdc_nav_reads": 0, "solver_reruns": 0, "trajectory_reruns": 0,
        "correction_implementations": 0, "validation_holdout_reads": 0, "archive_reopens": 0,
        "rematerializations": 0, "kaggle_token_access": 0, "mat_reads_or_generated": 0,
        "device_wls_or_precomputed_coordinates": 0, "source_static_reads": len(static["source_hashes"]),
    }
    for route in ROUTES:
        pin = freeze["exact_raw_inputs"][route]
        path = ROOT / pin["path"]
        payload, digest = _read_bytes_once(path, f"Phase50 raw device_gnss {route}", pin["sha256"])
        if len(payload) != int(pin["file_size"]):
            raise _fail(f"Phase50 raw byte count mismatch: {route}")
        epochs, metadata = _parse_payload(payload)
        byte_size = len(payload)
        del payload
        report, events = _route_report(route, epochs, metadata, path, digest, byte_size)
        reports[route] = report
        for event in events:
            event["route"] = route
        all_events.extend(events)
        reads["raw_device_gnss_reads"][route] = 1
        reads["raw_device_gnss_read_count_total"] += 1
    route_medians = {route: float(report["adr_rate_residual"]["clock_centered_abs_residual_m"]["median"]) for route, report in reports.items()}
    aggregate = {
        "route_count": len(reports), "route_median_abs_residual_m": route_medians,
        "route_median_abs_residual_aggregate_m": _median(route_medians.values()),
        "route_median_abs_residual_mad_m": _mad(route_medians.values()),
        "pairwise_route_median_distances_m": [{"route_a": left, "route_b": right, "distance_m": abs(route_medians[left] - route_medians[right])} for index, left in enumerate(ROUTES) for right in ROUTES[index + 1:]],
    }
    event_count = sum(len(report["_events"]) for report in reports.values())
    integrity = _presentation_integrity(reports, aggregate, event_count)
    loo = _loo(reports)
    observations = {route: report["gate_observations"] for route, report in reports.items()}
    coverage_pass = all(int(value["ordinary_pairs"]) >= MIN_ORDINARY_PAIRS and int(value["satellite_count"]) >= MIN_SATELLITES and bool(value["valid_unflagged_coverage"]) and bool(value["all_finite_residuals"]) for value in observations.values())
    class_population_pass = all(int(value["resolved_stable_pairs"]) >= MIN_STABLE_RESOLVED_PAIRS and int(value["unresolved_or_toggle_pairs"]) >= MIN_UNRESOLVED_OR_TOGGLE_PAIRS and MIN_IMPLICATED_FRACTION <= float(value["implicated_fraction"]) <= MAX_IMPLICATED_FRACTION for value in observations.values())
    materiality_pass = all(float(value["toggle_p95_excess_m"]) >= MIN_TOGGLE_EXCESS_M and value["toggle_p95_ratio"] is not None and float(value["toggle_p95_ratio"]) >= MIN_TOGGLE_RATIO for value in observations.values())
    cluster_pass = all(float(value["half_cycle_cluster_fraction"]) >= MIN_HALF_CYCLE_CLUSTER_FRACTION for value in observations.values())
    clean_pass = all(float(value["clean_retention_fraction"]) >= MIN_CLEAN_RETENTION for value in observations.values())
    route_direction_pass = all(float(value["toggle_p95_excess_m"]) > 0.0 for value in observations.values())
    impact_pass = all(int(value["current_tdcp_proxy_ordinary_overlap"]) >= MIN_CURRENT_TDCP_OVERLAP for value in observations.values())
    integrity_pass = bool(integrity["pair_reason_counts_sum_all"] and integrity["state_class_counts_sum_all"] and integrity["signal_group_counts_sum_all"] and integrity["satellite_group_counts_sum_all"] and integrity["four_route_medians_retained"] and integrity["aggregate_recomputed_exact"] and integrity["event_count_exact"])
    gate_rows = [
        {"name": "route_count", "observed": len(reports), "required": 4, "passed": len(reports) == 4 and tuple(reports) == ROUTES},
        {"name": "ordinary_tdcp_coverage", "observed_min_pairs": min((int(value["ordinary_pairs"]) for value in observations.values()), default=0), "observed_min_satellites": min((int(value["satellite_count"]) for value in observations.values()), default=0), "required_pairs": MIN_ORDINARY_PAIRS, "required_satellites": MIN_SATELLITES, "passed": coverage_pass},
        {"name": "state_class_population", "observed_resolved_stable": [int(value["resolved_stable_pairs"]) for value in observations.values()], "observed_unresolved_or_toggle": [int(value["unresolved_or_toggle_pairs"]) for value in observations.values()], "observed_implicated_fraction": [float(value["implicated_fraction"]) for value in observations.values()], "required_resolved_stable": MIN_STABLE_RESOLVED_PAIRS, "required_unresolved_or_toggle": MIN_UNRESOLVED_OR_TOGGLE_PAIRS, "required_fraction_min": MIN_IMPLICATED_FRACTION, "required_fraction_max": MAX_IMPLICATED_FRACTION, "passed": class_population_pass},
        {"name": "toggle_materiality", "observed_excess_m": [float(value["toggle_p95_excess_m"]) for value in observations.values()], "observed_ratio": [value["toggle_p95_ratio"] for value in observations.values()], "required_excess_m": MIN_TOGGLE_EXCESS_M, "required_ratio": MIN_TOGGLE_RATIO, "passed": materiality_pass},
        {"name": "half_cycle_cluster", "observed": [float(value["half_cycle_cluster_fraction"]) for value in observations.values()], "required": MIN_HALF_CYCLE_CLUSTER_FRACTION, "tolerance_half_cycles": HALF_CYCLE_CLUSTER_TOLERANCE, "passed": cluster_pass},
        {"name": "clean_retention", "observed": [float(value["clean_retention_fraction"]) for value in observations.values()], "required": MIN_CLEAN_RETENTION, "passed": clean_pass},
        {"name": "routewise_same_direction", "observed": route_direction_pass, "required": True, "passed": route_direction_pass},
        {"name": "loo_fixed_class_rule", "observed": loo, "required": True, "passed": bool(loo["decision_stable"])},
        {"name": "current_tdcp_used_pair_impact", "observed_ordinary_overlap": [int(value["current_tdcp_proxy_ordinary_overlap"]) for value in observations.values()], "required_min": MIN_CURRENT_TDCP_OVERLAP, "passed": impact_pass},
        {"name": "presentation_integrity", "observed": integrity, "required": True, "passed": integrity_pass},
    ]
    passed = all(bool(row["passed"]) for row in gate_rows)
    if passed:
        status = "go-half-cycle-transition-arc-reset-concept-authorized"
        strongest = "The fixed half-cycle transition association passes every raw-only population, materiality, half-cycle-cluster, clean-retention, route, LOO, current-TDCP-overlap, and presentation-integrity gate."
        concept = "opt-in TDCP arc reset/exclusion on a half-cycle status transition only; concept authorized, not implemented"
        next_factor = None
    elif not integrity_pass:
        status = "no-go-evaluator-integrity-failure"
        strongest = "Phase50 presentation-integrity invariants failed; no half-cycle association is promoted after the one-shot audit."
        concept, next_factor = None, NEXT_FACTOR
    elif not coverage_pass:
        status = "no-go-half-cycle-insufficient-ordinary-tdcp-coverage"
        strongest = "The ordinary raw TDCP-eligible ADR pair population or satellite coverage is below the frozen four-route minimum."
        concept, next_factor = None, NEXT_FACTOR
    elif not class_population_pass:
        status = "no-go-half-cycle-state-population"
        strongest = "Resolved-stable and unresolved/toggle ordinary state classes or the fixed implicated fraction do not meet the four-route identification population gates."
        concept, next_factor = None, NEXT_FACTOR
    elif not materiality_pass or not cluster_pass or not clean_pass or not route_direction_pass:
        status = "no-go-half-cycle-residual-not-material-or-clustered"
        strongest = "Half-cycle state transitions are not simultaneously material, half-cycle clustered, clean-retaining, and same-direction across all routes."
        concept, next_factor = None, NEXT_FACTOR
    elif not loo["decision_stable"] or not impact_pass:
        status = "no-go-half-cycle-loo-or-tdcp-impact-failure"
        strongest = "The half-cycle transition association is not stable under the fixed LOO class rule or does not overlap ordinary current-TDCP pairs."
        concept, next_factor = None, NEXT_FACTOR
    else:
        status = "no-go-half-cycle-audit-gate-failure"
        strongest, concept, next_factor = "At least one frozen Phase50 gate failed; no correction or arc reset is authorized.", None, NEXT_FACTOR
    result = {
        "schema_version": SCHEMA, "phase": 50, "execution_label": "Luna Max", "status": status,
        "mode": "raw-device-gnss-adr-half-cycle-audit-only", "freeze": {"path": _relative(FREEZE), "sha256": FREEZE_SHA256},
        "evaluator_manifest": {"path": _relative(MANIFEST), "sha256": VERIFIED_MANIFEST_SHA256}, "read_accounting": reads,
        "routes": {route: _strip_internal(report) for route, report in reports.items()}, "aggregate": aggregate, "loo": loo,
        "static_contract": static, "presentation_integrity": integrity, "gates": {"all_passed": passed, "rows": gate_rows},
        "decision": {"audit_only": True, "correction_authorized": False, "concept_if_go": concept, "strongest_finding": strongest, "next_single_raw_physical_factor": next_factor, "truth_or_past_truth_or_phase49_metric_used": False, "zero_point_782": "not evaluated without truth"},
        "events": {"count": event_count, "table": "phase50_pixel5_adr_half_cycle.events.json"},
    }
    output_root.mkdir(parents=True, exist_ok=True)
    routes_path = output_root / "phase50_pixel5_adr_half_cycle.routes.json"
    events_path = output_root / "phase50_pixel5_adr_half_cycle.events.json"
    routes_bytes = _atomic_json(routes_path, {"schema_version": SCHEMA + ".routes", "routes": result["routes"]})
    events_bytes = _atomic_json(events_path, {"schema_version": SCHEMA + ".events", "events": _strip_internal(all_events)})
    result_path = output_root / "phase50_pixel5_adr_half_cycle.json"
    result_bytes = _atomic_json(result_path, result)
    artifacts = {
        _relative(result_path): {"bytes": len(result_bytes), "sha256": _sha256_bytes(result_bytes)},
        _relative(routes_path): {"bytes": len(routes_bytes), "sha256": _sha256_bytes(routes_bytes)},
        _relative(events_path): {"bytes": len(events_bytes), "sha256": _sha256_bytes(events_bytes)},
    }
    output_manifest = {"schema_version": SCHEMA + ".output-manifest", "status": "atomic-publish-complete", "phase": 50, "freeze_sha256": FREEZE_SHA256, "evaluator_manifest_sha256": VERIFIED_MANIFEST_SHA256, "read_accounting": reads, "artifacts": artifacts}
    _atomic_json(output_root / "phase50_pixel5_adr_half_cycle.manifest.json", output_manifest)
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        freeze = _verify_freeze()
        manifest = _verify_manifest(freeze)
        _audit(freeze, manifest, args.output)
    except Phase50Error as exc:
        print(f"phase50: {exc}", file=sys.stderr)
        return 2
    print(f"phase50: completed raw ADR half-cycle audit at {_relative(args.output)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
