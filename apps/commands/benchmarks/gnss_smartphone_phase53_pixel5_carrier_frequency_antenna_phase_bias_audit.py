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
FREEZE_SHA256 = "618b351764876fb0eafd3c2431076acf0ac8fd43670ea1a5e33f8d7ee43dce1a"
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
NEXT_FACTOR = "raw Android per-satellite accumulated-delta-range uncertainty (AccumulatedDeltaRangeUncertaintyMeters)"

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


def _frequency_group(row: Row) -> str:
    """Use the parsed signal plus nominal RF frequency as a stable group key."""
    return f"{row.system}:{row.signal}:{int(round(_nominal_frequency(row)))}Hz"


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
        "carrierphase", "carriercycles",
    }]
    phase_uncertainty_names = [name for name in header if _normalise_header(name) in {
        "carrierphaseuncertainty", "accumulateddeltarangeuncertaintymeters",
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
        "phase_uncertainty_fields": phase_uncertainty_names,
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


def _pair_components(previous: Row, current: Row) -> dict[str, float] | None:
    """Return frequency-aware and constant-frequency components for one pair."""
    if previous.rate_mps is None or current.rate_mps is None:
        return None
    if previous.adr_m is None or current.adr_m is None:
        return None
    if not math.isfinite(_float(previous.frequency_hz)) or not math.isfinite(_float(current.frequency_hz)):
        return None
    dt_s = (current.time_ns - previous.time_ns) / 1.0e9
    if not (0.0 < dt_s <= 1.5):
        return None
    signed_previous, signed_current = _signed_adr(previous), _signed_adr(current)
    if signed_previous is None or signed_current is None:
        return None
    f_previous, f_current = _float(previous.frequency_hz), _float(current.frequency_hz)
    f_ref = 0.5 * (f_previous + f_current)
    if not math.isfinite(f_ref) or f_ref <= 0.0:
        return None
    delta_adr_m = signed_current - signed_previous
    rate_integral_m = 0.5 * (_float(previous.rate_mps) + _float(current.rate_mps)) * dt_s
    phi_previous = signed_previous * f_previous / SPEED_OF_LIGHT_MPS
    phi_current = signed_current * f_current / SPEED_OF_LIGHT_MPS
    delta_phase_m = (phi_current - phi_previous) * SPEED_OF_LIGHT_MPS / f_ref
    frequency_residual_m = delta_phase_m - rate_integral_m
    control_residual_m = delta_adr_m - rate_integral_m
    leakage_m = delta_phase_m - delta_adr_m
    nominal = _nominal_frequency(current)
    offset_previous_ppm = (f_previous - _nominal_frequency(previous)) / _nominal_frequency(previous) * 1.0e6
    offset_current_ppm = (f_current - nominal) / nominal * 1.0e6
    return {
        "dt_s": dt_s,
        "delta_adr_m": delta_adr_m,
        "rate_integral_m": rate_integral_m,
        "delta_phase_m": delta_phase_m,
        "frequency_residual_m": frequency_residual_m,
        "control_residual_m": control_residual_m,
        "frequency_leakage_m": leakage_m,
        "frequency_offset_previous_ppm": offset_previous_ppm,
        "frequency_offset_current_ppm": offset_current_ppm,
        "frequency_hz_previous": f_previous,
        "frequency_hz_current": f_current,
        "frequency_hz_reference": f_ref,
    }


def _pair_reason(previous: Row, current: Row, components: dict[str, float] | None) -> str:
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
    if previous.carrier_masked or current.carrier_masked:
        return "existing_mask"
    if components is None:
        return "nonfinite_relation"
    return "ordinary"


def _direct_phase_residual(row: Row) -> float | None:
    """Return a source-sign-consistent direct phase diagnostic when possible.

    Android documents ``AccumulatedDeltaRange = -k * carrier phase``.  The
    deprecated ``CarrierPhase`` is only fractional, so this value is retained
    as a diagnostic and never treated as a full ambiguity.  ``CarrierCycles``
    is kept separate because its export sign/epoch convention is source-owned.
    """
    signed = _signed_adr(row)
    if signed is None:
        return None
    phi_range = signed * _float(row.frequency_hz) / SPEED_OF_LIGHT_MPS
    if row.direct_phase_cycles is not None:
        return _float(row.direct_phase_cycles) + phi_range
    if row.direct_carrier_cycles is not None:
        return _float(row.direct_carrier_cycles) + phi_range
    return None


def _finite_items(items: Sequence[dict[str, Any]], key: str) -> list[float]:
    values: list[float] = []
    for item in items:
        value = item.get(key)
        if value is None:
            continue
        try:
            value_float = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(value_float):
            values.append(value_float)
    return values


def _pair_stats(items: Sequence[dict[str, Any]], key: str) -> dict[str, Any]:
    values = _finite_items(items, key)
    return _distribution(values, 0.0)


def _group_stats(items: Sequence[dict[str, Any]], key_name: str, value_key: str) -> dict[str, dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for item in items:
        key = str(item.get(key_name, "UNKNOWN"))
        grouped.setdefault(key, []).append(item)
    return {
        key: {"count": len(group), "residual": _pair_stats(group, value_key)}
        for key, group in sorted(grouped.items())
    }


def _center_pairs(pairs: Sequence[dict[str, Any]]) -> None:
    """Center each relation on endpoint UTC/HCDC using ordinary pairs only."""
    for value_key, center_key, centered_key in (
        ("frequency_residual_m", "frequency_center_m", "frequency_centered_residual_m"),
        ("control_residual_m", "control_center_m", "control_centered_residual_m"),
        ("frequency_leakage_m", "leakage_center_m", "frequency_centered_leakage_m"),
    ):
        centers: dict[tuple[int, int], list[float]] = {}
        for item in pairs:
            value = item.get(value_key)
            if item.get("ordinary") and value is not None and math.isfinite(float(value)):
                centers.setdefault((int(item["utcTimeMillis"]), int(item["hcdc"])), []).append(float(value))
        for item in pairs:
            value = item.get(value_key)
            if value is None or not math.isfinite(float(value)):
                continue
            group = (int(item["utcTimeMillis"]), int(item["hcdc"]))
            center = _median(centers.get(group, [float(value)]))
            item[center_key] = center
            item[centered_key] = float(value) - center
            item[f"abs_{centered_key}"] = abs(float(value) - center)


def _make_pairs(rows: Sequence[Row]) -> list[dict[str, Any]]:
    by_key: dict[tuple[str, int, str], list[Row]] = {}
    for row in rows:
        by_key.setdefault(row.sat_signal, []).append(row)
    pairs: list[dict[str, Any]] = []
    for key, key_rows in sorted(by_key.items()):
        key_rows.sort(key=lambda row: (row.utc_ms, row.row_number))
        for index, (previous, current) in enumerate(zip(key_rows, key_rows[1:])):
            components = _pair_components(previous, current)
            reason = _pair_reason(previous, current, components)
            item: dict[str, Any] = {
                "key": key,
                "key_index": index,
                "utcTimeMillis": current.utc_ms,
                "previous_utcTimeMillis": previous.utc_ms,
                "system": current.system,
                "svid": current.svid,
                "signal": current.signal,
                "group": _frequency_group(current),
                "segment": current.segment,
                "hcdc": current.hcdc,
                "reason": reason,
                "ordinary": reason == "ordinary",
                "components": components,
                "direct_phase_previous": _direct_phase_residual(previous),
                "direct_phase_current": _direct_phase_residual(current),
                "antenna_field_previous": dict(previous.antenna_fields),
                "antenna_field_current": dict(current.antenna_fields),
            }
            if components is not None:
                item.update(components)
            pairs.append(item)
    _center_pairs(pairs)
    for item in pairs:
        if not item.get("ordinary"):
            item["candidate"] = False
            continue
        abs_frequency = abs(float(item.get("frequency_offset_current_ppm", 0.0)))
        abs_residual = float(item.get("abs_frequency_centered_residual_m", 0.0))
        item["candidate"] = bool(abs_frequency >= MIN_FREQUENCY_OFFSET_P95_PPM and abs_residual >= MIN_LEAKAGE_P95_M)
        item["clean"] = not item["candidate"]
    return pairs


def _route_report(route: str, rows: Sequence[Row], metadata: dict[str, Any],
                  path: Path, digest: str, byte_size: int) -> dict[str, Any]:
    assignments = _assign_segments(rows)
    pairs = _make_pairs(rows)
    ordinary = [item for item in pairs if item.get("ordinary")]
    candidates = [item for item in ordinary if item.get("candidate")]
    clean = [item for item in ordinary if not item.get("candidate")]
    satellites = sorted({f"{item['system']}:{item['svid']}" for item in ordinary})
    groups = sorted({str(item["group"]) for item in ordinary})
    frequencies = [_float(row.frequency_hz) for row in rows if math.isfinite(_float(row.frequency_hz))]
    offsets = [
        (value - _nominal_frequency(row)) / _nominal_frequency(row) * 1.0e6
        for row in rows
        for value in [_float(row.frequency_hz)]
        if math.isfinite(value) and _nominal_frequency(row) > 0.0
    ]
    ordinary_offsets = [float(item.get("frequency_offset_current_ppm", 0.0)) for item in ordinary]
    centered = _finite_items(ordinary, "frequency_centered_residual_m")
    centered_abs = _finite_items(ordinary, "abs_frequency_centered_residual_m")
    control_centered_abs = _finite_items(ordinary, "abs_control_centered_residual_m")
    leakage_abs = _finite_items(ordinary, "abs_frequency_centered_leakage_m")
    candidate_abs = _finite_items(candidates, "abs_frequency_centered_residual_m")
    clean_abs = _finite_items(clean, "abs_frequency_centered_residual_m")
    frequency_abs = [abs(value) for value in offsets if math.isfinite(value)]
    satellite_groups: dict[str, list[dict[str, Any]]] = {}
    signal_groups: dict[str, list[dict[str, Any]]] = {}
    for item in ordinary:
        satellite_groups.setdefault(f"{item['system']}:{item['svid']}", []).append(item)
        signal_groups.setdefault(str(item["group"]), []).append(item)
    sat_medians = {
        key: _median(_finite_items(values, "abs_frequency_centered_residual_m"))
        for key, values in sorted(satellite_groups.items())
    }
    group_medians = {
        key: _median(_finite_items(values, "abs_frequency_centered_residual_m"))
        for key, values in sorted(signal_groups.items())
    }
    correlation_x = [abs(float(item.get("frequency_offset_current_ppm", 0.0))) for item in ordinary]
    correlation_y = [abs(float(item.get("frequency_centered_residual_m", 0.0))) for item in ordinary]
    correlation = _spearman(correlation_x, correlation_y)
    direct_phase_values = [
        float(value)
        for row in rows
        for value in [_direct_phase_residual(row)]
        if value is not None and math.isfinite(float(value))
    ]
    antenna_value_count = sum(len(row.antenna_fields) for row in rows)
    reason_counts: dict[str, int] = {}
    state_groups: dict[str, int] = {}
    for item in pairs:
        reason = str(item["reason"])
        reason_counts[reason] = reason_counts.get(reason, 0) + 1
        state_groups[reason] = state_groups.get(reason, 0) + 1
    candidate_fraction = len(candidates) / len(ordinary) if ordinary else 0.0
    return {
        "route": route,
        "input": {"path": _relative(path), "bytes": byte_size, "sha256": digest},
        "headers": {
            "columns": metadata["header_columns"],
            "direct_phase_fields": metadata["direct_phase_fields"],
            "phase_uncertainty_fields": metadata["phase_uncertainty_fields"],
            "antenna_phase_bias_fields": metadata["antenna_phase_bias_fields"],
            "inter_signal_bias_fields": metadata["inter_signal_bias_fields"],
            "optional_field_presence": metadata["optional_field_presence"],
        },
        "rows": {
            "raw": metadata["raw_rows"],
            "non_raw": metadata["non_raw_rows"],
            "epochs": metadata["epoch_count"],
            "pair_count": len(pairs),
            "ordinary_pair_count": len(ordinary),
            "candidate_count": len(candidates),
            "candidate_fraction": candidate_fraction,
            "clean_count": len(clean),
            "satellite_count": len(satellites),
            "satellites": satellites,
            "signal_frequency_group_count": len(groups),
            "signal_frequency_groups": groups,
            "unsupported_signal_rows": metadata["unsupported_signal_rows"],
            "repeated_epoch_key_count": metadata["repeated_epoch_key_count"],
            "nonmonotonic_epoch_key_count": metadata["nonmonotonic_epoch_key_count"],
        },
        "frequency": {
            "raw_hz": _distribution(frequencies, _median(frequencies)),
            "offset_ppm": _distribution(offsets, 0.0),
            "ordinary_offset_ppm": _distribution(ordinary_offsets, 0.0),
            "nominal_constants": {key: value for key, value in sorted(FREQUENCIES.items()) if any(row.signal == key for row in rows)},
        },
        "residuals": {
            "frequency_aware_centered_m": _distribution(centered, 0.0),
            "frequency_aware_abs_m": _distribution(centered_abs, 0.0),
            "constant_frequency_control_centered_m": _distribution(_finite_items(ordinary, "control_centered_residual_m"), 0.0),
            "constant_frequency_control_abs_m": _distribution(control_centered_abs, 0.0),
            "frequency_leakage_centered_m": _distribution(_finite_items(ordinary, "frequency_centered_leakage_m"), 0.0),
            "frequency_leakage_abs_m": _distribution(leakage_abs, 0.0),
            "candidate_abs_m": _distribution(candidate_abs, 0.0),
            "clean_abs_m": _distribution(clean_abs, 0.0),
            "spearman_abs_frequency_vs_abs_residual": correlation,
        },
        "groups": {
            "signal_frequency": {
                key: {
                    "count": len(values),
                    "median_abs_centered_residual_m": _median(_finite_items(values, "abs_frequency_centered_residual_m")),
                    "p95_abs_centered_residual_m": _percentile(_finite_items(values, "abs_frequency_centered_residual_m"), 0.95),
                    "median_frequency_offset_ppm": _median(_finite_items(values, "frequency_offset_current_ppm")),
                }
                for key, values in sorted(signal_groups.items())
            },
            "satellite": {
                key: {
                    "count": len(values),
                    "median_abs_centered_residual_m": sat_medians[key],
                    "p95_abs_centered_residual_m": _percentile(_finite_items(values, "abs_frequency_centered_residual_m"), 0.95),
                }
                for key, values in sorted(satellite_groups.items())
            },
            "state": {
                key: {"count": count}
                for key, count in sorted(state_groups.items())
            },
        },
        "direct_phase_bias": {
            "fields": metadata["direct_phase_fields"],
            "finite_value_count": len(direct_phase_values),
            "centered_relation_cycles": _distribution(direct_phase_values, _median(direct_phase_values)),
            "interpretation": "diagnostic only; CarrierPhase is fractional and direct fields are never promoted to full ambiguity",
        },
        "antenna_phase_bias": {
            "fields": metadata["antenna_phase_bias_fields"],
            "finite_value_count": antenna_value_count,
            "available": bool(metadata["antenna_phase_bias_fields"] and antenna_value_count),
            "interpretation": "missing fields are unavailable, not zero",
        },
        "segments": {
            "count": len({int(item["segment"]) for item in assignments}),
            "boundaries": sum(1 for item in assignments if item["explicit_boundary"]),
            "assignments": assignments,
        },
        "pair_reasons": reason_counts,
        "gate_observations": {
            "all_core_finite": all(
                item.get("frequency_residual_m") is not None
                and math.isfinite(float(item["frequency_residual_m"]))
                for item in ordinary
            ),
            "ordinary_pairs": len(ordinary),
            "satellite_count": len(satellites),
            "signal_frequency_group_count": len(groups),
            "within_signal_groups": len(groups),
            "current_tdcp_overlap": len(ordinary),
            "direct_phase_pairs": sum(
                1 for item in ordinary
                if item.get("direct_phase_previous") is not None and item.get("direct_phase_current") is not None
            ),
            "antenna_fields_available": bool(metadata["antenna_phase_bias_fields"] and antenna_value_count),
            "frequency_offset_p95_abs_ppm": _percentile(frequency_abs, 0.95),
            "frequency_leakage_p95_abs_m": _percentile(leakage_abs, 0.95),
            "frequency_residual_p95_abs_m": _percentile(centered_abs, 0.95),
            "control_residual_p95_abs_m": _percentile(control_centered_abs, 0.95),
            "frequency_vs_control_p95_excess_m": _percentile(centered_abs, 0.95) - _percentile(control_centered_abs, 0.95),
            "satellite_median_mad_m": _mad(sat_medians.values()),
            "spearman": correlation,
            "candidate_fraction": candidate_fraction,
            "clean_retention_fraction": len(clean) / len(ordinary) if ordinary else 0.0,
            "frequency_offset_signed_median_ppm": _median(offsets),
        },
        "_pairs": pairs,
    }


def _static_contract(freeze: dict[str, Any]) -> dict[str, Any]:
    contents: dict[str, str] = {}
    hashes: dict[str, str] = {}
    for name, pin in freeze["authority_pins"]["source_contracts"].items():
        payload, digest = _read_bytes_once(ROOT / pin["path"], f"Phase53 static source {name}", pin["sha256"])
        contents[name] = payload.decode("utf-8")
        hashes[name] = digest
    adapter = contents["android_raw_gnss_cpp"]
    adapter_header = contents["android_raw_gnss_header"]
    observation = contents["observation_header"]
    fgo = contents["fgo_problems_cpp"]
    tdcp = contents["tdcp_contract_header"]
    carrier = contents["carrier_code_leveling_header"]
    return {
        "source_hashes": hashes,
        "adapter_parses_carrier_frequency": "carrierfrequencyhz" in adapter.lower(),
        "adapter_parses_accumulated_delta_range": "accumulateddeltarangemeters" in adapter.lower(),
        "adapter_preserves_pixel5_adr_sign": "supportedDeviceAdrSign" in adapter and "pixel5" not in adapter.lower(),
        "adapter_has_no_android_antenna_phase_field_parser": "antenna" not in adapter.lower() and "phasecenter" not in adapter.lower(),
        "adapter_header_has_no_android_antenna_phase_field": "antenna" not in adapter_header.lower() and "phasecenter" not in adapter_header.lower(),
        "observation_exposes_antenna_pco_default_only": "antenna_pco" in observation and "source_carrier_frequency_hz" in observation,
        "fgo_consumes_carrier_frequency_for_wavelength": "source_carrier_frequency_hz" in fgo and "wavelength" in fgo,
        "fgo_tdcp_contract_present": "tdcp_contract::evaluateAdjacentPair" in fgo and "max_tdcp_gap_s" in fgo,
        "tdcp_source_has_loss_of_lock_and_clock_contract": "LossOfLock" in tdcp and "ClockDiscontinuity" in tdcp,
        "carrier_leveling_uses_adr_state_and_gap": "validAdrState" in carrier and "max_gap_s = 1.5" in carrier,
        "interpretation": "Static source confirms raw frequency/ADR are present in the adapter and current FGO/TDCP uses the derived wavelength, while no Android antenna phase-center/bias parser is present. Runtime header audit remains authoritative for raw field availability.",
    }


def _verify_freeze(path: Path = FREEZE) -> dict[str, Any]:
    freeze, digest = _load_json_once(path, "Phase53 freeze", FREEZE_SHA256 or None)
    if FREEZE_SHA256 and digest != FREEZE_SHA256:
        raise _fail(f"Phase53 freeze hash changed: {digest} != {FREEZE_SHA256}")
    if freeze.get("schema_version") != FREEZE_SCHEMA or freeze.get("status") != "frozen-before-phase53-raw-read":
        raise _fail("Phase53 freeze schema/status mismatch")
    if freeze.get("cohort", {}).get("route_order") != list(ROUTES):
        raise _fail("Phase53 route order changed")
    inputs = freeze.get("exact_raw_inputs")
    if not isinstance(inputs, dict) or tuple(inputs) != ROUTES:
        raise _fail("Phase53 exact raw input map changed")
    for route in ROUTES:
        item = inputs[route]
        if len(str(item.get("sha256", ""))) != 64 or int(item.get("file_size", 0)) <= 0:
            raise _fail(f"Phase53 raw hash/size missing for {route}")
    policy = freeze.get("input_policy", {})
    expected_policy = {
        "single_process": True,
        "raw_device_gnss_read_count_per_route": 1,
        "raw_device_imu_read_count": 0,
        "truth_read_count": 0,
        "past_truth_payload_reads": 0,
        "phase52_metric_payload_reads": 0,
        "brdc_nav_reads": 0,
        "solver_reruns": 0,
        "trajectory_reruns": 0,
        "correction_fit_or_application": False,
        "validation_holdout_access": False,
        "archive_reopens": 0,
        "rematerialization_count": 0,
        "kaggle_or_token_access": False,
        "mat_reads_or_generated": False,
        "device_wls_or_precomputed_coordinates": False,
        "SvPosition_or_SvElevation": False,
    }
    if any(policy.get(key) != value for key, value in expected_policy.items()):
        raise _fail("Phase53 read policy changed")
    gates = freeze.get("numeric_gates", {})
    checks = (
        ("ordinary_pair_coverage", "min_eligible_pairs_per_route", MIN_ORDINARY_PAIRS),
        ("ordinary_pair_coverage", "min_satellites_per_route", MIN_SATELLITES),
        ("ordinary_pair_coverage", "current_tdcp_used_pair_overlap_min_per_route", MIN_CURRENT_TDCP_OVERLAP),
        ("frequency_group_coverage", "min_distinct_system_signal_frequency_groups_per_route", MIN_FREQUENCY_GROUPS),
        ("frequency_group_coverage", "within_signal_direction_min_groups_per_route", MIN_WITHIN_SIGNAL_GROUPS),
        ("direct_source_availability", "direct_phase_field_pair_min_per_route_if_present", MIN_DIRECT_PHASE_PAIRS),
        ("frequency_materiality", "abs_frequency_offset_p95_min_ppm", MIN_FREQUENCY_OFFSET_P95_PPM),
        ("frequency_materiality", "frequency_leakage_p95_min_m", MIN_LEAKAGE_P95_M),
        ("frequency_materiality", "frequency_aware_vs_control_p95_excess_min_m", MIN_FREQUENCY_EXCESS_M),
        ("frequency_materiality", "non_common_satellite_median_mad_min_m", MIN_SATELLITE_MAD_M),
        ("relation_identifiability", "abs_frequency_offset_vs_abs_centered_residual_spearman_min", MIN_SPEARMAN),
        ("relation_identifiability", "loo_effect_excess_min_m", MIN_LOO_EXCESS_M),
        ("retention_and_integrity", "clean_retention_min_fraction", MIN_CLEAN_RETENTION),
    )
    for section, key, value in checks:
        if gates.get(section, {}).get(key) != value:
            raise _fail(f"Phase53 gate changed: {section}.{key}")
    if gates.get("frequency_group_coverage", {}).get("min_satellite_groups_per_route") != MIN_SATELLITES:
        raise _fail("Phase53 satellite group gate changed")
    if gates.get("direct_source_availability", {}).get("antenna_phase_center_or_bias_field_required_for_native_antenna_correction") is not True:
        raise _fail("Phase53 antenna source gate is not closed")
    if gates.get("relation_identifiability", {}).get("routewise_spearman_required") is not True or gates.get("relation_identifiability", {}).get("loo_fixed_rule_direction_stable") is not True or gates.get("relation_identifiability", {}).get("signal_within_family_direction_required") is not True:
        raise _fail("Phase53 relation gates are not closed")
    for key in ("exact_sha256_and_bytes", "all_core_fields_finite", "nonmonotonic_epochs", "duplicate_epoch_keys"):
        if key == "nonmonotonic_epochs" or key == "duplicate_epoch_keys":
            if gates.get("raw_input_integrity", {}).get(key) != 0:
                raise _fail(f"Phase53 raw integrity gate changed: {key}")
        elif gates.get("raw_input_integrity", {}).get(key) is not True:
            raise _fail(f"Phase53 raw integrity gate changed: {key}")
    integrity = gates.get("retention_and_integrity", {})
    for key in ("all_residuals_finite", "pair_reason_counts_sum", "state_group_counts_sum", "signal_frequency_group_counts_sum", "satellite_group_counts_sum", "four_route_medians_retained", "aggregate_recomputed_exact", "loo_route_count_exact", "header_field_presence_route_map_exact"):
        if integrity.get(key) is not True:
            raise _fail(f"Phase53 presentation gate changed: {key}")
    assertions = freeze.get("pre_read_assertions")
    if not isinstance(assertions, dict) or any(value is not False for value in assertions.values()):
        raise _fail("Phase53 pre-read assertions are not closed")
    return freeze


def _verify_manifest(freeze: dict[str, Any], path: Path = MANIFEST) -> dict[str, Any]:
    global VERIFIED_MANIFEST_SHA256
    manifest, digest = _load_json_once(path, "Phase53 evaluator manifest", MANIFEST_SHA256 or None)
    if MANIFEST_SHA256 and digest != MANIFEST_SHA256:
        raise _fail(f"Phase53 manifest hash changed: {digest} != {MANIFEST_SHA256}")
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("status") != "evaluator-frozen-before-phase53-raw-read":
        raise _fail("Phase53 manifest schema/status mismatch")
    if manifest.get("freeze", {}).get("path") != _relative(FREEZE) or manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256:
        raise _fail("Phase53 manifest freeze pin mismatch")
    evaluator = manifest.get("evaluator", {})
    if evaluator.get("operation") != "audit" or evaluator.get("native_solver_invoked") is not False or evaluator.get("single_process") is not True:
        raise _fail("Phase53 manifest permits solver/multi-process execution")
    cohort = manifest.get("cohort", {})
    if cohort.get("route_order") != list(ROUTES) or cohort.get("raw_device_gnss_reads_per_route") != 1 or cohort.get("truth_reads_per_route") != 0:
        raise _fail("Phase53 manifest route/read policy mismatch")
    required_forbidden = ("ground truth", "prior metric payload", "navigation", "solver/trajectory", "correction implementation", "precomputed coordinates")
    forbidden = manifest.get("forbidden", [])
    if not isinstance(forbidden, list) or not all(token in forbidden for token in required_forbidden):
        raise _fail("Phase53 forbidden policy missing")
    for name in ("source", "test", "cmake"):
        pin = evaluator.get(name, {})
        pin_path = ROOT / str(pin.get("path", ""))
        if not pin_path.is_file() or len(str(pin.get("sha256", ""))) != 64:
            raise _fail(f"Phase53 {name} pin missing")
        actual = _sha256_bytes(pin_path.read_bytes())
        if actual != pin["sha256"]:
            raise _fail(f"Phase53 {name} hash mismatch")
    VERIFIED_MANIFEST_SHA256 = digest
    return manifest


def _aggregate_values(items: Sequence[dict[str, Any]]) -> dict[str, float]:
    frequency = _finite_items(items, "frequency_centered_residual_m")
    control = _finite_items(items, "control_centered_residual_m")
    leakage = _finite_items(items, "frequency_centered_leakage_m")
    return {
        "frequency_p95_abs_m": _percentile([abs(value) for value in frequency], 0.95),
        "control_p95_abs_m": _percentile([abs(value) for value in control], 0.95),
        "frequency_leakage_p95_abs_m": _percentile([abs(value) for value in leakage], 0.95),
    }


def _loo(reports: dict[str, dict[str, Any]]) -> dict[str, Any]:
    folds: list[dict[str, Any]] = []
    for omitted in ROUTES:
        items = [
            item for route, report in reports.items() if route != omitted
            for item in report["_pairs"] if item.get("ordinary")
        ]
        aggregate = _aggregate_values(items)
        excess = aggregate["frequency_p95_abs_m"] - aggregate["control_p95_abs_m"]
        folds.append({
            "omitted_route": omitted,
            "ordinary_pairs": len(items),
            **aggregate,
            "frequency_vs_control_p95_excess_m": excess,
            "positive_material_effect": bool(excess >= MIN_FREQUENCY_EXCESS_M and aggregate["frequency_leakage_p95_abs_m"] >= MIN_LEAKAGE_P95_M),
        })
    decisions = [bool(item["positive_material_effect"]) for item in folds]
    return {
        "folds": folds,
        "fixed_rule": f"frequency-aware p95 excess >= {MIN_FREQUENCY_EXCESS_M} m and leakage p95 >= {MIN_LEAKAGE_P95_M} m",
        "direction_stable": bool(decisions) and len(set(decisions)) == 1,
        "all_positive_material": bool(decisions) and all(decisions),
    }


def _same_nonzero_direction(values: Iterable[float]) -> bool:
    signs = {1 if float(value) > 0.0 else -1 if float(value) < 0.0 else 0 for value in values}
    signs.discard(0)
    return len(signs) == 1 and bool(signs)


def _presentation_integrity(reports: dict[str, dict[str, Any]], aggregate: dict[str, Any], loo: dict[str, Any]) -> dict[str, Any]:
    pair_reason_sum: dict[str, bool] = {}
    signal_group_sum: dict[str, bool] = {}
    satellite_group_sum: dict[str, bool] = {}
    state_group_sum: dict[str, bool] = {}
    header_map: dict[str, bool] = {}
    for route, report in reports.items():
        pair_reason_sum[route] = sum(int(value) for value in report["pair_reasons"].values()) == int(report["rows"]["pair_count"])
        signal_group_sum[route] = sum(int(value["count"]) for value in report["groups"]["signal_frequency"].values()) == int(report["rows"]["ordinary_pair_count"])
        satellite_group_sum[route] = sum(int(value["count"]) for value in report["groups"]["satellite"].values()) == int(report["rows"]["ordinary_pair_count"])
        state_group_sum[route] = sum(int(value["count"]) for value in report["groups"]["state"].values()) == int(report["rows"]["pair_count"])
        header_map[route] = isinstance(report["headers"].get("columns"), list) and isinstance(report["headers"].get("optional_field_presence"), dict)
    route_medians = aggregate["route_median_abs_frequency_residual_m"]
    retained = len(route_medians) == len(ROUTES) and tuple(route_medians) == ROUTES
    median_values = list(route_medians.values())
    recomputed = (
        aggregate["route_median_abs_frequency_residual_aggregate_m"] == _median(median_values)
        and aggregate["route_median_abs_frequency_residual_mad_m"] == _mad(median_values)
    )
    return {
        "pair_reason_counts_sum": pair_reason_sum,
        "pair_reason_counts_sum_all": all(pair_reason_sum.values()),
        "signal_frequency_group_counts_sum": signal_group_sum,
        "signal_frequency_group_counts_sum_all": all(signal_group_sum.values()),
        "satellite_group_counts_sum": satellite_group_sum,
        "satellite_group_counts_sum_all": all(satellite_group_sum.values()),
        "state_group_counts_sum": state_group_sum,
        "state_group_counts_sum_all": all(state_group_sum.values()),
        "four_route_medians_retained": retained,
        "aggregate_recomputed_exact": recomputed,
        "loo_route_count_exact": len(loo.get("folds", [])) == len(ROUTES) and tuple(item.get("omitted_route") for item in loo.get("folds", [])) == ROUTES,
        "header_field_presence_route_map_exact": len(header_map) == len(ROUTES) and tuple(header_map) == ROUTES and all(header_map.values()),
    }


def _strip_internal(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: _strip_internal(item) for key, item in value.items() if not str(key).startswith("_")}
    if isinstance(value, list):
        return [_strip_internal(item) for item in value]
    return value


def _failure_reason(gates: dict[str, bool], reports: dict[str, dict[str, Any]]) -> str:
    if not gates["presentation_integrity"]:
        return "presentation-integrity invariants failed; no frequency/phase association is promoted"
    if not gates["antenna_source"]:
        missing = [route for route, report in reports.items() if not report["gate_observations"]["antenna_fields_available"]]
        return "Android raw headers expose no finite antenna phase-center/bias field on routes: " + ", ".join(missing)
    if not gates["frequency_groups"]:
        return "the four Pixel5 routes do not provide the frozen minimum of two distinct system/signal/frequency groups per route"
    if not gates["frequency_materiality"]:
        return "carrier-frequency leakage/residual is below the frozen non-common-mode materiality gates"
    if not gates["relation_identifiability"]:
        return "frequency/residual association is not routewise stable under the frozen Spearman/LOO/non-common-mode gates"
    if not gates["ordinary_coverage"]:
        return "ordinary raw ADR pair or satellite coverage is below the frozen minimum"
    return "at least one frozen Phase53 identifiability gate failed"


def _audit(freeze: dict[str, Any], manifest: dict[str, Any], output_root: Path) -> dict[str, Any]:
    _reject_path(output_root)
    if output_root.exists():
        raise _fail(f"Phase53 output already exists: {output_root}")
    static = _static_contract(freeze)
    reports: dict[str, dict[str, Any]] = {}
    reads: dict[str, Any] = {
        "single_process": True,
        "raw_device_gnss_reads": {},
        "raw_device_gnss_read_count_total": 0,
        "raw_device_imu_reads": 0,
        "truth_reads": 0,
        "past_truth_payload_reads": 0,
        "phase52_metric_payload_reads": 0,
        "phase50_metric_payload_reads": 0,
        "phase49_metric_payload_reads": 0,
        "brdc_nav_reads": 0,
        "solver_reruns": 0,
        "trajectory_reruns": 0,
        "correction_implementations": 0,
        "validation_holdout_reads": 0,
        "archive_reopens": 0,
        "rematerializations": 0,
        "kaggle_token_access": 0,
        "mat_reads_or_generated": 0,
        "device_wls_or_precomputed_coordinates": 0,
        "SvPosition_or_SvElevation": 0,
        "source_static_reads": len(static["source_hashes"]),
    }
    for route in ROUTES:
        pin = freeze["exact_raw_inputs"][route]
        path = ROOT / pin["path"]
        payload, digest = _read_bytes_once(path, f"Phase53 raw device_gnss {route}", pin["sha256"])
        if len(payload) != int(pin["file_size"]):
            raise _fail(f"Phase53 raw byte count mismatch: {route}")
        rows, metadata = _parse_payload(payload)
        byte_size = len(payload)
        del payload
        report = _route_report(route, rows, metadata, path, digest, byte_size)
        reports[route] = report
        reads["raw_device_gnss_reads"][route] = 1
        reads["raw_device_gnss_read_count_total"] += 1
        del rows

    route_medians = {
        route: float(report["residuals"]["frequency_aware_abs_m"]["median"])
        for route, report in reports.items()
    }
    control_medians = {
        route: float(report["residuals"]["constant_frequency_control_abs_m"]["median"])
        for route, report in reports.items()
    }
    leakage_medians = {
        route: float(report["residuals"]["frequency_leakage_abs_m"]["median"])
        for route, report in reports.items()
    }
    aggregate = {
        "route_count": len(reports),
        "route_median_abs_frequency_residual_m": route_medians,
        "route_median_abs_control_residual_m": control_medians,
        "route_median_abs_frequency_leakage_m": leakage_medians,
        "route_median_abs_frequency_residual_aggregate_m": _median(route_medians.values()),
        "route_median_abs_frequency_residual_mad_m": _mad(route_medians.values()),
        "pairwise_route_median_distances_m": [
            {
                "route_a": left,
                "route_b": right,
                "frequency_residual_distance_m": abs(route_medians[left] - route_medians[right]),
                "control_residual_distance_m": abs(control_medians[left] - control_medians[right]),
            }
            for index, left in enumerate(ROUTES) for right in ROUTES[index + 1:]
        ],
    }
    loo = _loo(reports)
    integrity = _presentation_integrity(reports, aggregate, loo)
    observations = {route: report["gate_observations"] for route, report in reports.items()}
    ordinary_pass = all(
        int(value["ordinary_pairs"]) >= MIN_ORDINARY_PAIRS
        and int(value["satellite_count"]) >= MIN_SATELLITES
        and int(value["current_tdcp_overlap"]) >= MIN_CURRENT_TDCP_OVERLAP
        and bool(value["all_core_finite"])
        for value in observations.values()
    )
    frequency_groups_pass = all(
        int(value["signal_frequency_group_count"]) >= MIN_FREQUENCY_GROUPS
        and int(value["within_signal_groups"]) >= MIN_WITHIN_SIGNAL_GROUPS
        for value in observations.values()
    )
    direct_phase_pass = all(
        (not report["headers"]["direct_phase_fields"] or int(report["gate_observations"]["direct_phase_pairs"]) >= MIN_DIRECT_PHASE_PAIRS)
        for report in reports.values()
    )
    antenna_source_pass = all(bool(value["antenna_fields_available"]) for value in observations.values())
    frequency_materiality_pass = all(
        float(value["frequency_offset_p95_abs_ppm"]) >= MIN_FREQUENCY_OFFSET_P95_PPM
        and float(value["frequency_leakage_p95_abs_m"]) >= MIN_LEAKAGE_P95_M
        and float(value["frequency_vs_control_p95_excess_m"]) >= MIN_FREQUENCY_EXCESS_M
        and float(value["satellite_median_mad_m"]) >= MIN_SATELLITE_MAD_M
        for value in observations.values()
    )
    route_direction_pass = _same_nonzero_direction(
        float(value["frequency_offset_signed_median_ppm"]) for value in observations.values()
    )
    signal_direction_pass = all(
        _same_nonzero_direction(
            float(item["median_frequency_offset_ppm"])
            for item in report["groups"]["signal_frequency"].values()
        )
        and len(report["groups"]["signal_frequency"]) >= MIN_WITHIN_SIGNAL_GROUPS
        for report in reports.values()
    )
    relation_pass = (
        all(float(value["spearman"]) >= MIN_SPEARMAN for value in observations.values())
        and route_direction_pass
        and signal_direction_pass
        and bool(loo["direction_stable"])
        and bool(loo["all_positive_material"])
        and all(float(value["satellite_median_mad_m"]) >= MIN_SATELLITE_MAD_M for value in observations.values())
    )
    retention_pass = all(float(value["clean_retention_fraction"]) >= MIN_CLEAN_RETENTION for value in observations.values())
    integrity_pass = bool(
        integrity["pair_reason_counts_sum_all"]
        and integrity["signal_frequency_group_counts_sum_all"]
        and integrity["satellite_group_counts_sum_all"]
        and integrity["state_group_counts_sum_all"]
        and integrity["four_route_medians_retained"]
        and integrity["aggregate_recomputed_exact"]
        and integrity["loo_route_count_exact"]
        and integrity["header_field_presence_route_map_exact"]
    )
    gate_rows = {
        "route_count": len(reports) == 4 and tuple(reports) == ROUTES,
        "raw_input_integrity": all(
            int(report["rows"]["repeated_epoch_key_count"]) == 0
            and int(report["rows"]["nonmonotonic_epoch_key_count"]) == 0
            and int(report["rows"]["unsupported_signal_rows"]) == 0
            for report in reports.values()
        ),
        "ordinary_coverage": ordinary_pass,
        "frequency_groups": frequency_groups_pass,
        "direct_phase_source": direct_phase_pass,
        "antenna_source": antenna_source_pass,
        "frequency_materiality": frequency_materiality_pass,
        "relation_identifiability": relation_pass,
        "retention": retention_pass,
        "presentation_integrity": integrity_pass,
    }
    all_passed = all(gate_rows.values())
    if all_passed:
        status = "go-native-correction-eligibility"
        strongest = "All frozen raw-only frequency/phase-bias identifiability gates passed; a separate native implementation seal is authorized, but no truth or accuracy read is performed in Phase53."
        concept = "one opt-in native C++ FGO/TDCP correction design may be sealed in a separate stage; not implemented by this audit"
    else:
        status = "no-go-carrier-frequency-antenna-phase-bias-not-identifiable"
        strongest = _failure_reason(gate_rows, reports)
        concept = None
    events: list[dict[str, Any]] = []
    for route, report in reports.items():
        for item in report["_pairs"]:
            if item.get("candidate"):
                event = {key: _strip_internal(value) for key, value in item.items() if key not in {"key", "components", "antenna_field_previous", "antenna_field_current"}}
                event["route"] = route
                events.append(event)
    route_output = {route: _strip_internal(report) for route, report in reports.items()}
    result = {
        "schema_version": SCHEMA,
        "phase": 53,
        "execution_label": "Luna Max",
        "status": status,
        "factor": "raw Android per-satellite carrier-phase ADR carrier-frequency/antenna phase-bias residual",
        "audit_only": True,
        "correction_or_solver_change": False,
        "freeze": {"path": _relative(FREEZE), "sha256": FREEZE_SHA256},
        "evaluator_manifest": {"path": _relative(MANIFEST), "sha256": VERIFIED_MANIFEST_SHA256},
        "authority": {
            "phase52_policy_path": freeze["authority_pins"]["phase52_result"]["path"],
            "phase52_policy_sha256": freeze["authority_pins"]["phase52_result"]["sha256"],
            "phase52_metric_payload_used": False,
            "phase50_metric_payload_used": False,
            "phase49_metric_payload_used": False,
        },
        "fixed_contract": freeze["fixed_physical_contract"],
        "routes": route_output,
        "aggregate": aggregate,
        "loo": loo,
        "gates": {
            "all_passed": all_passed,
            "observed": gate_rows,
            "thresholds": freeze["numeric_gates"],
        },
        "presentation_integrity": integrity,
        "events": {"count": len(events), "table": "phase53_pixel5_carrier_frequency_antenna_phase_bias.events.json"},
        "read_accounting": reads,
        "decision": {
            "audit_only": True,
            "correction_authorized": all_passed,
            "concept_if_go": concept,
            "strongest_finding": strongest,
            "next_single_raw_physical_factor": NEXT_FACTOR if not all_passed else None,
            "zero_point_782": "not evaluated without truth",
            "truth_or_navigation_or_coordinate_or_solver_input": False,
        },
    }
    output_root.mkdir(parents=True, exist_ok=False)
    routes_path = output_root / "phase53_pixel5_carrier_frequency_antenna_phase_bias.routes.json"
    events_path = output_root / "phase53_pixel5_carrier_frequency_antenna_phase_bias.events.json"
    result_path = output_root / "phase53_pixel5_carrier_frequency_antenna_phase_bias.json"
    routes_payload = _atomic_json(routes_path, {"schema_version": SCHEMA + ".routes", "phase": 53, "route_order": list(ROUTES), "routes": route_output})
    events_payload = _atomic_json(events_path, {"schema_version": SCHEMA + ".events", "phase": 53, "count": len(events), "events": events})
    result_payload = _atomic_json(result_path, result)
    artifact_manifest = {
        "schema_version": SCHEMA + ".output-manifest",
        "status": "atomic-publish-complete",
        "phase": 53,
        "freeze_sha256": FREEZE_SHA256,
        "evaluator_manifest_sha256": VERIFIED_MANIFEST_SHA256,
        "read_accounting": reads,
        "artifacts": {
            "result": {"path": _relative(result_path), "bytes": len(result_payload), "sha256": _sha256_bytes(result_payload)},
            "routes": {"path": _relative(routes_path), "bytes": len(routes_payload), "sha256": _sha256_bytes(routes_payload)},
            "events": {"path": _relative(events_path), "bytes": len(events_payload), "sha256": _sha256_bytes(events_payload)},
        },
    }
    manifest_path = output_root / "phase53_pixel5_carrier_frequency_antenna_phase_bias.manifest.json"
    manifest_payload = _atomic_json(manifest_path, artifact_manifest)
    # The result is intentionally immutable after its first atomic publish; the
    # output manifest is the authoritative hash ledger for all four artifacts.
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true", help="verify the closed freeze and manifest without opening raw data")
    parser.add_argument("--audit", action="store_true", help="run the one-shot four-route raw audit")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    if args.verify_freeze == args.audit:
        parser.error("choose exactly one of --verify-freeze or --audit")
    try:
        freeze = _verify_freeze()
        manifest = _verify_manifest(freeze)
        if args.verify_freeze:
            print("phase53 freeze/evaluator manifest: verified without raw/truth reads")
            return 0
        result = _audit(freeze, manifest, args.output)
        print(json.dumps({
            "status": result["status"],
            "all_gates_passed": result["gates"]["all_passed"],
            "raw_reads": result["read_accounting"]["raw_device_gnss_read_count_total"],
            "next_factor_if_no_go": result["decision"]["next_single_raw_physical_factor"],
        }, sort_keys=True))
        return 0
    except Phase53Error as exc:
        print(f"phase53 fail-closed: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
