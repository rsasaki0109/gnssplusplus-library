#!/usr/bin/env python3
"""Truth-free Phase53 audit of Android carrier-frequency/phase-bias residuals.

The evaluator has one deliberately narrow input boundary: one byte-buffer read
of each of the four frozen Pixel5 ``device_gnss.csv`` files in one process.
It uses only raw Android timing, ADR, carrier frequency, Doppler/range-rate,
and optional raw phase/bias fields.  It never opens truth, navigation,
coordinates, solver output, or a prior metric payload.

The primary relation is a carrier-frequency-aware ADR/rate residual.  A
constant-frequency ADR/rate relation is computed from the same in-memory pairs
as the control.  Missing Android antenna/phase-bias fields are reported as
missing, never replaced with zero; absence is a hard no-implementation result.
"""

from __future__ import annotations

import argparse
import ast
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
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase53_pixel5_carrier_frequency_antenna_phase_bias_freeze_v1.json"
FREEZE_SHA256 = "26a9c893e0697a5987a2deef3b12cd85dbff8556f05fa07ecd7e380c199c766c"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase53_pixel5_carrier_frequency_antenna_phase_bias_evaluator_manifest_v1.json"
MANIFEST_SHA256 = ""
VERIFIED_MANIFEST_SHA256 = ""
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase53-pixel5-carrier-frequency-antenna-phase-bias-v1"

SCHEMA = "smartphone-r5-phase53-pixel5-carrier-frequency-antenna-phase-bias.v1"
FREEZE_SCHEMA = "smartphone-r5-phase53-pixel5-carrier-frequency-antenna-phase-bias-freeze.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase53-pixel5-carrier-frequency-antenna-phase-bias-manifest.v1"

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

MIN_ORDINARY_PAIRS = 3_000
MIN_SATELLITES = 5
MIN_CURRENT_TDCP_OVERLAP = 100
MIN_FREQUENCY_GROUPS = 2
MIN_WITHIN_SIGNAL_GROUPS = 2
MIN_DIRECT_PHASE_PAIRS = 100
MIN_FREQUENCY_OFFSET_P95_PPM = 1.0
MIN_LEAKAGE_P95_M = 0.03
MIN_FREQUENCY_EXCESS_M = 0.02
MIN_SATELLITE_MAD_M = 0.01
MIN_SPEARMAN = 0.35
MIN_LOO_EXCESS_M = 0.02
MIN_CANDIDATE_FRACTION = 0.001
MAX_CANDIDATE_FRACTION = 0.20
MIN_CLEAN_RETENTION = 0.80
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
    "AccumulatedDeltaRangeUncertaintyMeters", "CarrierCycles", "CarrierPhase",
    "CarrierPhaseUncertainty", "FullInterSignalBiasNanos",
    "SatelliteInterSignalBiasNanos", "FullInterSignalBiasUncertaintyNanos",
    "SatelliteInterSignalBiasUncertaintyNanos",
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


class Phase53Error(ValueError):
    """Raised when the immutable Phase53 contract is violated."""


def _fail(message: str) -> Phase53Error:
    return Phase53Error(message)


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
        raise _fail(f"forbidden Phase53 input/output path: {path}")


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
    low, high = int(math.floor(rank)), int(math.ceil(rank))
    return data[low] if low == high else data[low] + (rank - low) * (data[high] - data[low])


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
    direct_phase_cycles: Decimal | None = None
    direct_carrier_cycles: Decimal | None = None
    phase_uncertainty: Decimal | None = None
    antenna_fields: dict[str, Decimal] = field(default_factory=dict)
    segment: int = -1
    pseudorange_m: float | None = None
    code_masked: bool = False
    carrier_masked: bool = False

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


def _nominal_frequency(row: Row) -> float:
    return float(FREQUENCIES.get(row.signal, float(row.frequency_hz)))


def _signed_adr(row: Row, device_model: str = "pixel5") -> float | None:
    if row.adr_m is None:
        return None
    value = _float(row.adr_m)
    return -value if device_model in NEGATE_ADR_MODELS else value


def _optional_value(source: dict[str, str], fields: dict[str, str], name: str,
                    row_number: int) -> Decimal | None:
    key = fields.get(_normalise_header(name))
    if key is None:
        return None
    return _parse_decimal(source.get(key, ""), f"{name} row {row_number}", required=False)


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
    direct_phase_names = [name for name in header if _normalise_header(name) in {
        "carrierphase", "carriercycles", "carrierphaseuncertainty",
        "accumulateddeltarangeuncertaintymeters",
    }]
    antenna_names = [
        name for name in header
        if "antenna" in _normalise_header(name)
        and any(token in _normalise_header(name) for token in ("phase", "bias", "pco", "pcv"))
    ]
    intersignal_names = [
        name for name in header
        if "intersignalbias" in _normalise_header(name)
        or "interfrequencybias" in _normalise_header(name)
    ]
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
            parsed = _parse_int(source.get(key, ""), f"{name} row {number}", required)
            return default if parsed is None and default is not None else parsed

        def decimal(name: str, required: bool = True) -> Decimal | None:
            key = fields.get(_normalise_header(name))
            if key is None:
                if required:
                    raise _fail(f"missing field {name}")
                return None
            return _parse_decimal(source.get(key, ""), f"{name} row {number}", required)

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
        bias = decimal("BiasNanos", required=False) or Decimal(0)
        offset = decimal("TimeOffsetNanos", required=False) or Decimal(0)
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
        direct_phase = _optional_value(source, fields, "CarrierPhase", number)
        direct_cycles = _optional_value(source, fields, "CarrierCycles", number)
        phase_uncertainty = _optional_value(source, fields, "CarrierPhaseUncertainty", number)
        if phase_uncertainty is None:
            phase_uncertainty = _optional_value(source, fields, "AccumulatedDeltaRangeUncertaintyMeters", number)
        optional_antenna: dict[str, Decimal] = {}
        for name in antenna_names:
            value = _parse_decimal(source.get(name, ""), f"{name} row {number}", required=False)
            if value is not None:
                optional_antenna[name] = value
        state = integer("State", required=False)
        multipath = integer("MultipathIndicator", required=False)
        cn0 = decimal("Cn0DbHz", required=False)
        hcdc = integer("HardwareClockDiscontinuityCount", required=False, default=0) or 0
        row = Row(
            row_number=number, utc_ms=utc, time_ns=time_ns, full_bias_ns=full_bias,
            bias_ns=bias, offset_ns=offset, received_sv_time_ns=Decimal(received),
            rate_mps=rate, adr_state=adr_state, adr_m=adr_m, state=state,
            multipath=multipath, cn0=cn0, hcdc=hcdc, svid=svid,
            system=system, signal=signal, frequency_hz=frequency,
            direct_phase_cycles=direct_phase, direct_carrier_cycles=direct_cycles,
            phase_uncertainty=phase_uncertainty, antenna_fields=optional_antenna,
        )
        rows.append(row)
    if raw_count == 0:
        raise _fail("no Raw rows")
    return rows, {
        "header_columns": header,
        "normalised_header_columns": sorted(fields),
        "raw_rows": raw_count,
        "non_raw_rows": non_raw_count,
        "unsupported_signal_rows": unsupported,
        "epoch_count": len(epoch_keys),
        "epoch_keys": epoch_keys,
        "repeated_epoch_key_count": repeated_epoch_keys,
        "nonmonotonic_epoch_key_count": nonmonotonic_epochs,
        "direct_phase_fields": direct_phase_names,
        "antenna_phase_bias_fields": antenna_names,
        "inter_signal_bias_fields": intersignal_names,
        "optional_field_presence": {
            name: _normalise_header(name) in fields for name in OPTIONAL_COLUMNS
        },
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
            carrier_reasons = list(reasons)
            if not _adr_unflagged(row):
                carrier_reasons.append("adr_invalid_or_boundary")
            if row.adr_m is None:
                carrier_reasons.append("adr_missing")
            row.carrier_masked = bool(carrier_reasons)
        assignments.append({
            "epoch_index": epoch_index, "utcTimeMillis": key, "segment": segment,
            "hcdc": hcdc, "segment_base_FullBiasNanos": base_full_bias_ns,
            "time_nanos_gap_ns": gap_ns, "explicit_boundary": boundary,
        })
        previous_rows, previous_hcdc = current_rows, hcdc
    return assignments
