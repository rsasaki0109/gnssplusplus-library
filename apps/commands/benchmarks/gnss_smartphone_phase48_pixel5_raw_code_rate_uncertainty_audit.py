#!/usr/bin/env python3
"""Truth-free audit of per-satellite raw code-rate/uncertainty residuals.

The command reads each frozen Pixel5 ``device_gnss.csv`` exactly once in one
process.  It reconstructs Phase25 raw P, forms a same-satellite code
increment minus trapezoidal Android range-rate residual, centers receiver
common mode per endpoint epoch/HCDC group, and audits association with
``ReceivedSvTimeUncertaintyNanos``.  No truth, navigation, trajectory,
solver, coordinate, archive, or Phase47 metric payload is read.
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
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase48_pixel5_raw_code_rate_uncertainty_freeze_v1.json"
FREEZE_SHA256 = "e8bbfddda9670b492c3a938109c92333fd8432c564c7a6837874d9a11194b454"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase48_pixel5_raw_code_rate_uncertainty_evaluator_manifest_v1.json"
MANIFEST_SHA256 = ""
VERIFIED_MANIFEST_SHA256 = ""
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase48-pixel5-raw-code-rate-uncertainty-v1"

SCHEMA = "smartphone-r5-phase48-pixel5-raw-code-rate-uncertainty.v1"
FREEZE_SCHEMA = "smartphone-r5-phase48-pixel5-raw-code-rate-uncertainty-freeze.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase48-pixel5-raw-code-rate-uncertainty-manifest.v1"

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
TRANSITION_MAX_NS = 1_500_000_000
MIN_P = 10_000_000.0
MAX_P = 40_000_000.0
MIN_RESIDUALS = 1000
MIN_SATELLITES = 3
MIN_BUCKETS = 3
MIN_SPEARMAN = 0.35
MIN_WITHIN_SIGNAL_SPEARMAN = 0.20
HIGH_LOW_EXCESS_M = 2.0
HIGH_LOW_RATIO = 1.5
MIN_LOW_BASE_FRACTION = 0.30

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
    "PseudorangeRateMetersPerSecond",
)
OPTIONAL_COLUMNS = (
    "TimeOffsetNanos",
    "HardwareClockDiscontinuityCount",
    "State",
    "MultipathIndicator",
    "Cn0DbHz",
    "ReceivedSvTimeUncertaintyNanos",
    "PseudorangeRateUncertaintyMetersPerSecond",
    "AccumulatedDeltaRangeState",
    "AccumulatedDeltaRangeMeters",
    "DriftNanosPerSecond",
    "BiasUncertaintyNanos",
    "TimeUncertaintyNanos",
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
    "GAL_E1": 1_575_420_000.0, "GAL_E5A": 1_176_450_000.0,
    "BDS_B1I": 1_561_098_000.0, "BDS_B2A": 1_176_450_000.0,
}


class Phase48Error(ValueError):
    """Raised when the immutable Phase48 audit contract is violated."""


def _fail(message: str) -> Phase48Error:
    return Phase48Error(message)


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _reject_path(path: Path | str) -> None:
    lowered = str(path).lower()
    forbidden = (".mat", "ground_truth", "validation", "holdout", "precomputed", "device_wls", "svposition", "svelevation", "archive", "kaggle", "token")
    if any(token in lowered for token in forbidden):
        raise _fail(f"forbidden Phase48 input/output path: {path}")


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
        if not required:
            return None
        raise _fail(f"non-finite integer {label}")
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
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return data[lower]
    return data[lower] + (rank - lower) * (data[upper] - data[lower])


def _mad(values: Iterable[float], center: float | None = None) -> float:
    data = [float(value) for value in values]
    if not data:
        return 0.0
    return _median(abs(value - (_median(data) if center is None else center)) for value in data)


def _distribution(values: Iterable[float], center: float | None = None) -> dict[str, Any]:
    data = [float(value) for value in values]
    if not data:
        return {"count": 0, "median": 0.0, "mad": 0.0, "p50_abs": 0.0, "p95_abs": 0.0, "max_abs": 0.0}
    actual_center = _median(data) if center is None else center
    absolute = [abs(value - actual_center) for value in data]
    return {"count": len(data), "median": _median(data), "mad": _mad(data, actual_center), "p50_abs": _percentile(absolute, 0.50), "p95_abs": _percentile(absolute, 0.95), "max_abs": max(absolute)}


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
    den_x = math.sqrt(sum((value - mx) ** 2 for value in rx))
    den_y = math.sqrt(sum((value - my) ** 2 for value in ry))
    if den_x == 0.0 or den_y == 0.0:
        return 0.0
    return sum((a - mx) * (b - my) for a, b in zip(rx, ry)) / (den_x * den_y)


def _system(value: Any) -> str:
    text = str(value).strip().upper()
    return CONSTELLATIONS.get(text, text or "UNKNOWN")


def _signal(value: Any, system: str, frequency: Decimal | None) -> str:
    token = str(value).strip().upper()
    if token in {"L1", "L5"}:
        freq = _float(frequency) if frequency is not None else 0.0
        candidates = [("GPS_L1CA", 1_575_420_000.0), ("GPS_L5", 1_176_450_000.0), ("GAL_E1", 1_575_420_000.0), ("GAL_E5A", 1_176_450_000.0), ("BDS_B1I", 1_561_098_000.0), ("BDS_B2A", 1_176_450_000.0)]
        for name, nominal in candidates:
            expected = "GPS" if name.startswith("GPS") else ("GALILEO" if name.startswith("GAL") else "BEIDOU")
            if expected == system and abs(freq - nominal) <= 1000.0:
                return name
    if token in SIGNAL_MAP:
        return SIGNAL_MAP[token]
    freq = _float(frequency) if frequency is not None else 0.0
    for name, nominal in FREQUENCIES.items():
        expected = "GPS" if name.startswith("GPS") else ("GALILEO" if name.startswith("GAL") else "BEIDOU")
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
    sv_uncertainty_ns: Decimal | None
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
    epochs_map: dict[int, Epoch] = {}
    order: list[int] = []
    optional_counts = {name: 0 for name in OPTIONAL_COLUMNS}
    raw_count = 0
    non_raw_count = 0
    unsupported = 0
    previous_key: int | None = None
    closed_keys: set[int] = set()
    repeated_keys = 0
    nonmonotonic = 0
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
            row_number=number,
            utc_ms=utc,
            time_ns=time_ns,
            full_bias_ns=full_bias,
            bias_ns=bias,
            offset_ns=offset,
            received_sv_time_ns=Decimal(received),
            rate_mps=rate,
            rate_uncertainty_mps=decimal("PseudorangeRateUncertaintyMetersPerSecond", required=False),
            sv_uncertainty_ns=decimal("ReceivedSvTimeUncertaintyNanos", required=False),
            state=integer("State", required=False),
            multipath=integer("MultipathIndicator", required=False),
            cn0=decimal("Cn0DbHz", required=False),
            hcdc=integer("HardwareClockDiscontinuityCount", required=False, default=0) or 0,
            svid=svid,
            system=system,
            signal=signal,
            frequency_hz=frequency,
        )
        if utc not in epochs_map:
            epochs_map[utc] = Epoch(key_ms=utc)
            order.append(utc)
        epochs_map[utc].rows.append(row)
    if raw_count == 0:
        raise _fail("no Raw rows")
    return [epochs_map[key] for key in order], {
        "header_columns": fields,
        "raw_rows": raw_count,
        "non_raw_rows": non_raw_count,
        "unsupported_signal_rows": unsupported,
        "epoch_count": len(order),
        "raw_row_coverage": 1.0,
        "repeated_epoch_key_count": repeated_keys,
        "nonmonotonic_epoch_key_count": nonmonotonic,
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
    return value if value > 0.0 and math.isfinite(value) else None


def _mask(row: Row) -> tuple[bool, tuple[str, ...]]:
    reasons: list[str] = []
    if row.multipath == 1:
        reasons.append("multipath")
    if row.cn0 is not None and _float(row.cn0) < 20.0:
        reasons.append("low_cnr")
    if not _status_valid(row):
        reasons.append("invalid_state")
    if row.pseudorange_m is None or row.pseudorange_m < MIN_P or row.pseudorange_m > MAX_P:
        reasons.append("code_range")
    return bool(reasons), tuple(reasons)


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
        gap_change = gap_ns is not None and gap_ns > TIME_GAP_NS
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
            row.code_masked, row.mask_reasons = _mask(row)
        if hcdc_change:
            events.append({"kind": "hcdc_boundary", "utcTimeMillis": epoch.key_ms, "previous": previous_hcdc, "current": hcdc})
        if gap_change:
            events.append({"kind": "time_gap_boundary", "utcTimeMillis": epoch.key_ms, "gap_ns": gap_ns})
        assignments.append({"epoch_index": index, "utcTimeMillis": epoch.key_ms, "segment": segment, "hcdc": hcdc, "segment_base_FullBiasNanos": epoch.segment_base_full_bias_ns, "time_nanos_gap_ns": gap_ns, "explicit_boundary": boundary})
        previous = epoch
        previous_hcdc = hcdc
    return assignments, events


def _transitions(epochs: Sequence[Epoch]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    by_key: dict[tuple[str, int, str], list[Row]] = {}
    for epoch in epochs:
        for row in epoch.rows:
            by_key.setdefault(row.sat_signal, []).append(row)
    transitions: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []
    for key, rows in sorted(by_key.items()):
        rows.sort(key=lambda row: (row.utc_ms, row.row_number))
        for previous, current in zip(rows, rows[1:]):
            dt_ns = current.time_ns - previous.time_ns
            dt = dt_ns / 1e9
            if dt_ns <= 0 or dt_ns > TRANSITION_MAX_NS:
                continue
            if current.hcdc != previous.hcdc or current.rate_mps is None or previous.rate_mps is None:
                continue
            if current.pseudorange_m is None or previous.pseudorange_m is None or current.code_masked or previous.code_masked:
                continue
            raw_residual = (current.pseudorange_m - previous.pseudorange_m) - 0.5 * (_float(previous.rate_mps) + _float(current.rate_mps)) * dt
            if not math.isfinite(raw_residual):
                continue
            transitions.append({
                "utcTimeMillis": current.utc_ms,
                "previous_utcTimeMillis": previous.utc_ms,
                "dt_s": dt,
                "system": current.system,
                "svid": current.svid,
                "signal": current.signal,
                "hcdc": current.hcdc,
                "raw_residual_m": raw_residual,
                "sv_uncertainty_ns": _float(current.sv_uncertainty_ns) if current.sv_uncertainty_ns is not None else None,
                "rate_uncertainty_mps": _float(current.rate_uncertainty_mps) if current.rate_uncertainty_mps is not None else None,
                "cn0": _float(current.cn0) if current.cn0 is not None else None,
                "multipath": current.multipath,
                "mask_reasons": list(current.mask_reasons),
            })
    by_clock: dict[tuple[int, int], list[float]] = {}
    for item in transitions:
        by_clock.setdefault((int(item["utcTimeMillis"]), int(item["hcdc"])), []).append(float(item["raw_residual_m"]))
    for item in transitions:
        center = _median(by_clock[(int(item["utcTimeMillis"]), int(item["hcdc"]))])
        item["clock_group_median_m"] = center
        item["centered_residual_m"] = float(item["raw_residual_m"]) - center
        item["abs_centered_residual_m"] = abs(item["centered_residual_m"])
        if item["abs_centered_residual_m"] >= 2.0:
            events.append({"kind": "code_rate_residual_outlier", "utcTimeMillis": item["utcTimeMillis"], "system": item["system"], "svid": item["svid"], "signal": item["signal"], "abs_centered_residual_m": item["abs_centered_residual_m"]})
    return transitions, events


def _uncertainty_bucket(value: float | None) -> str:
    if value is None:
        return "missing"
    if value <= 10.0:
        return "<=10ns"
    if value <= 100.0:
        return "10_to_100ns"
    return ">100ns"


def _log_bucket(value: float | None) -> str:
    if value is None:
        return "missing"
    if value < 1.0:
        return "<1ns"
    if value < 3.0:
        return "1_to_3ns"
    if value < 10.0:
        return "3_to_10ns"
    if value < 30.0:
        return "10_to_30ns"
    if value <= 100.0:
        return "30_to_100ns"
    return ">100ns"


def _bucket_report(items: Sequence[dict[str, Any]]) -> dict[str, Any]:
    ordered = ("<=10ns", "10_to_100ns", ">100ns")
    buckets: dict[str, list[float]] = {name: [] for name in ("missing",) + ordered}
    log_buckets: dict[str, list[float]] = {name: [] for name in ("missing", "<1ns", "1_to_3ns", "3_to_10ns", "10_to_30ns", "30_to_100ns", ">100ns")}
    normalized: list[float] = []
    for item in items:
        uncertainty = item.get("sv_uncertainty_ns")
        value = float(item["abs_centered_residual_m"])
        buckets[_uncertainty_bucket(uncertainty)].append(value)
        log_buckets[_log_bucket(uncertainty)].append(value)
        if uncertainty is not None and uncertainty > 0.0:
            sigma = SPEED_OF_LIGHT_MPS * float(uncertainty) / 1e9
            item["uncertainty_sigma_m"] = sigma
            item["normalized_abs_residual_sigma"] = value / sigma if sigma > 0.0 else None
            if sigma > 0.0:
                normalized.append(item["normalized_abs_residual_sigma"])
        else:
            item["uncertainty_sigma_m"] = None
            item["normalized_abs_residual_sigma"] = None
    def summarize(mapping: dict[str, list[float]]) -> dict[str, Any]:
        return {name: {"count": len(values), "median_m": _median(values), "p95_m": _percentile(values, 0.95), "max_m": max(values) if values else 0.0} for name, values in mapping.items()}
    present = [item for item in items if item.get("sv_uncertainty_ns") is not None]
    values = [float(item["sv_uncertainty_ns"]) for item in present]
    residuals = [float(item["abs_centered_residual_m"]) for item in present]
    # The frozen materiality gate is explicitly ``>10 ns``.  Keep the two
    # ordered buckets in the presentation, but pool both of them for the
    # high-vs-low comparison; using only ``>100 ns`` here would silently test
    # a different gate than the one sealed before the raw read.
    high_values = buckets["10_to_100ns"] + buckets[">100ns"]
    low_p95 = _percentile(buckets["<=10ns"], 0.95)
    high_p95 = _percentile(high_values, 0.95)
    return {
        "ordered": summarize(buckets),
        "log_bins": summarize(log_buckets),
        "uncertainty_quartiles_ns": {"q25": _percentile(values, 0.25), "q50": _percentile(values, 0.50), "q75": _percentile(values, 0.75), "count": len(values)},
        "normalized_abs_residual_sigma": _distribution(normalized, 0.0),
        "spearman_uncertainty_abs_centered_residual": _spearman(values, residuals),
        "high_low": {
            "high_definition": ">10ns",
            "low_count": len(buckets["<=10ns"]),
            "high_count": len(high_values),
            "low_p95_m": low_p95,
            "high_p95_m": high_p95,
            "p95_excess_m": high_p95 - low_p95,
            "p95_ratio": (high_p95 / low_p95) if low_p95 > 0.0 else None,
        },
        "transition_count": len(items),
        "finite_uncertainty_count": len(present),
        "uncertainty_bucket_count_populated": sum(1 for name in ordered if buckets[name]),
    }


def _group_summary(items: Sequence[dict[str, Any]]) -> dict[str, Any]:
    groups: dict[str, list[dict[str, Any]]] = {}
    for item in items:
        groups.setdefault(f"{item['system']}:{item['signal']}", []).append(item)
    report: dict[str, Any] = {}
    for name, group in sorted(groups.items()):
        present = [item for item in group if item.get("sv_uncertainty_ns") is not None]
        x = [float(item["sv_uncertainty_ns"]) for item in present]
        y = [float(item["abs_centered_residual_m"]) for item in present]
        report[name] = {
            "count": len(group),
            "satellites": sorted({int(item["svid"]) for item in group}),
            "uncertainty_count": len(present),
            "spearman": _spearman(x, y),
            "residual_distribution_m": _distribution([float(item["abs_centered_residual_m"]) for item in group], 0.0),
            "uncertainty_median_ns": _median(x),
        }
    return report


def _route_report(route: str, epochs: Sequence[Epoch], metadata: dict[str, Any], path: Path, digest: str) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    assignments, boundary_events = _assign_segments(epochs)
    transitions, residual_events = _transitions(epochs)
    bucket = _bucket_report(transitions)
    signal_groups = _group_summary(transitions)
    all_uncertainties = [float(item["sv_uncertainty_ns"]) for item in transitions if item.get("sv_uncertainty_ns") is not None]
    residual_values = [float(item["abs_centered_residual_m"]) for item in transitions]
    satellites = sorted({(item["system"], int(item["svid"])) for item in transitions})
    rows = {
        "raw": int(metadata["raw_rows"]),
        "non_raw": int(metadata["non_raw_rows"]),
        "epochs": len(epochs),
        "raw_row_coverage": 1.0,
        "transition_count": len(transitions),
        "transition_satellite_count": len(satellites),
        "transition_svids": [f"{system}:{svid}" for system, svid in satellites],
        "existing_mask_retention": len(transitions) / int(metadata["raw_rows"]) if metadata["raw_rows"] else 0.0,
        "uncertainty_present_fraction": len(all_uncertainties) / len(transitions) if transitions else 0.0,
        "repeated_epoch_key_count": int(metadata["repeated_epoch_key_count"]),
        "nonmonotonic_epoch_key_count": int(metadata["nonmonotonic_epoch_key_count"]),
    }
    gate_observations = {
        "all_finite_residuals": all(math.isfinite(value) for value in residual_values),
        "residual_count": len(transitions),
        "satellite_count": len(satellites),
        "populated_uncertainty_buckets": int(bucket["uncertainty_bucket_count_populated"]),
        "spearman": float(bucket["spearman_uncertainty_abs_centered_residual"]),
        "high_low_excess_m": float(bucket["high_low"]["p95_excess_m"]),
        "high_low_ratio": bucket["high_low"]["p95_ratio"],
        "low_base_fraction": len([item for item in transitions if item.get("sv_uncertainty_ns") is not None and float(item["sv_uncertainty_ns"]) <= 10.0]) / len(transitions) if transitions else 0.0,
        "within_signal_groups_spearman_ge_0_2": sum(1 for item in signal_groups.values() if item["uncertainty_count"] > 1 and item["spearman"] >= MIN_WITHIN_SIGNAL_SPEARMAN),
    }
    report = {
        "route": route,
        "input": {"path": _relative(path), "bytes": int(path.stat().st_size), "sha256": digest},
        "rows": rows,
        "code_rate_residual": {
            "raw_centered_residual_m": _distribution([float(item["raw_residual_m"]) for item in transitions], 0.0),
            "clock_centered_abs_residual_m": _distribution(residual_values, 0.0),
            "spearman_and_buckets": bucket,
            "system_signal_groups": signal_groups,
        },
        "uncertainty": {
            "received_sv_time_uncertainty_to_metres": _distribution([SPEED_OF_LIGHT_MPS * value / 1e9 for value in all_uncertainties], 0.0),
            "pseudorange_rate_uncertainty_mps": _distribution([float(item["rate_uncertainty_mps"]) for item in transitions if item.get("rate_uncertainty_mps") is not None], 0.0),
        },
        "segment_contract": {"segment_count": max((item["segment"] for item in assignments), default=-1) + 1, "boundary_events": len(boundary_events)},
        "gate_observations": gate_observations,
        "events": {"count": len(boundary_events) + len(residual_events)},
        "_transitions": transitions,
        "_events": boundary_events + residual_events,
    }
    return report, boundary_events + residual_events


def _static_contract(freeze: dict[str, Any]) -> dict[str, Any]:
    pins = freeze["authority_pins"]["source_contracts"]
    contents: dict[str, str] = {}
    hashes: dict[str, str] = {}
    for name, pin in pins.items():
        payload, digest = _read_bytes_once(ROOT / pin["path"], f"Phase48 static source {name}", pin["sha256"])
        contents[name] = payload.decode("utf-8")
        hashes[name] = digest
    adapter = contents["android_raw_gnss_cpp"]
    header = contents["android_raw_gnss_header"]
    observation = contents["observation_header"]
    fgo = contents["fgo_problems_cpp"]
    field = "ReceivedSvTimeUncertaintyNanos"
    return {
        "source_hashes": hashes,
        "adapter_parses_received_sv_time_uncertainty": field in adapter or field.lower() in adapter.lower(),
        "adapter_parses_pseudorange_rate_uncertainty": "PseudorangeRateUncertainty" in adapter,
        "observation_retains_received_sv_time_uncertainty": field in observation or field.lower() in observation.lower(),
        "fgo_consumes_received_sv_time_uncertainty_as_sigma": field in fgo or field.lower() in fgo.lower(),
        "fgo_consumes_pseudorange_rate_uncertainty_as_sigma": "PseudorangeRateUncertainty" in fgo,
        "existing_adapter_masks_pinned": all(token in adapter for token in ("MultipathIndicator", "Cn0DbHz", "raw_code_masked")),
        "interpretation": "The field is deployable only if adapter parsing and current FGO sigma consumption are both statically present; raw audit association alone cannot authorize a correction.",
    }


def _verify_freeze(path: Path = FREEZE) -> dict[str, Any]:
    freeze, digest = _load_json_once(path, "Phase48 freeze", FREEZE_SHA256 or None)
    if FREEZE_SHA256 and digest != FREEZE_SHA256:
        raise _fail(f"Phase48 freeze hash changed: {digest} != {FREEZE_SHA256}")
    if freeze.get("schema_version") != FREEZE_SCHEMA or freeze.get("status") != "frozen-before-phase48-raw-read":
        raise _fail("Phase48 freeze schema/status mismatch")
    if freeze.get("cohort", {}).get("route_order") != list(ROUTES):
        raise _fail("Phase48 route order changed")
    inputs = freeze.get("exact_raw_inputs")
    if not isinstance(inputs, dict) or tuple(inputs) != ROUTES:
        raise _fail("Phase48 exact raw input map changed")
    for route in ROUTES:
        item = inputs[route]
        if len(str(item.get("sha256", ""))) != 64 or int(item.get("file_size", 0)) <= 0:
            raise _fail(f"Phase48 raw hash/size missing for {route}")
    policy = freeze.get("input_policy", {})
    expected = {"single_process": True, "raw_device_gnss_read_count_per_route": 1, "truth_read_count": 0, "phase45_payload_reads": 0, "phase47_metric_payload_reads": 0, "brdc_nav_reads": 0, "solver_reruns": 0, "trajectory_reruns": 0, "correction_fit_or_application": False, "validation_holdout_access": False, "archive_reopens": 0, "rematerialization_count": 0, "kaggle_or_token_access": False, "mat_reads_or_generated": False, "device_wls_or_precomputed_coordinates": False, "SvPosition_or_SvElevation": False}
    if any(policy.get(key) != value for key, value in expected.items()):
        raise _fail("Phase48 read policy changed")
    gates = freeze.get("numeric_gates", {})
    expected_gates = [("residual_coverage", "min_residuals_per_route", 1000), ("residual_coverage", "min_satellites_per_route", 3), ("uncertainty_population", "min_populated_ordered_buckets_per_route", 3), ("spearman", "min_each_route", 0.35), ("high_vs_low", "p95_excess_min_m", 2.0), ("high_vs_low", "p95_ratio_min", 1.5), ("low_base_retention", "minimum_low_or_base_population_fraction", 0.30), ("composition_independence", "within_signal_families_min_per_route", 2), ("composition_independence", "within_signal_spearman_min", 0.20)]
    for section, key, value in expected_gates:
        if gates.get(section, {}).get(key) != value:
            raise _fail(f"Phase48 gate changed: {section}.{key}")
    if gates.get("bucket_monotonicity", {}).get("allow_exact_equality") is not True or gates.get("loo_mapping_stability", {}).get("all_leave_one_route_out_direction_stable") is not True or gates.get("presentation_integrity", {}).get("aggregate_recomputed_exact") is not True:
        raise _fail("Phase48 monotonicity/LOO/integrity gates are not closed")
    pre = freeze.get("pre_read_assertions")
    if not isinstance(pre, dict) or any(value is not False for value in pre.values()):
        raise _fail("Phase48 pre-read assertions are not closed")
    return freeze


def _verify_manifest(freeze: dict[str, Any], path: Path = MANIFEST) -> dict[str, Any]:
    global VERIFIED_MANIFEST_SHA256
    manifest, digest = _load_json_once(path, "Phase48 evaluator manifest", MANIFEST_SHA256 or None)
    if MANIFEST_SHA256 and digest != MANIFEST_SHA256:
        raise _fail(f"Phase48 manifest hash changed: {digest} != {MANIFEST_SHA256}")
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("status") != "evaluator-frozen-before-phase48-raw-read":
        raise _fail("Phase48 manifest schema/status mismatch")
    if manifest.get("freeze", {}).get("path") != _relative(FREEZE) or manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256:
        raise _fail("Phase48 manifest freeze pin mismatch")
    evaluator = manifest.get("evaluator", {})
    if evaluator.get("operation") != "audit" or evaluator.get("native_solver_invoked") is not False or evaluator.get("single_process") is not True:
        raise _fail("Phase48 manifest permits solver/multi-process execution")
    cohort = manifest.get("cohort", {})
    if cohort.get("route_order") != list(ROUTES) or cohort.get("raw_device_gnss_reads_per_route") != 1 or cohort.get("truth_reads_per_route") != 0:
        raise _fail("Phase48 manifest route/read policy mismatch")
    forbidden = manifest.get("forbidden", [])
    required_forbidden = ("ground_truth.csv", "Phase47 metric payload", "nav/solver/trajectory rerun", "correction implementation", "precomputed coordinates")
    if not isinstance(forbidden, list) or not all(token in forbidden for token in required_forbidden):
        raise _fail("Phase48 forbidden policy missing")
    for name in ("source", "test", "cmake"):
        pin = evaluator.get(name, {})
        pin_path = ROOT / str(pin.get("path", ""))
        if not pin_path.is_file() or len(str(pin.get("sha256", ""))) != 64:
            raise _fail(f"Phase48 {name} pin missing")
        actual = _sha256_bytes(pin_path.read_bytes())
        if actual != pin["sha256"]:
            raise _fail(f"Phase48 {name} hash mismatch")
    VERIFIED_MANIFEST_SHA256 = digest
    return manifest


def _loo(routes: dict[str, dict[str, Any]]) -> dict[str, Any]:
    folds: list[dict[str, Any]] = []
    for omitted in ROUTES:
        items = [item for route, report in routes.items() if route != omitted for item in report["_transitions"]]
        bucket = _bucket_report(items)
        high_low = bucket["high_low"]
        direction = high_low["p95_excess_m"] >= 0.0
        folds.append({"omitted_route": omitted, "transition_count": len(items), "spearman": bucket["spearman_uncertainty_abs_centered_residual"], "high_low_excess_m": high_low["p95_excess_m"], "high_low_ratio": high_low["p95_ratio"], "direction_positive": direction})
    return {"folds": folds, "direction_stable": bool(folds) and len({bool(item["direction_positive"]) for item in folds}) == 1}


def _presentation_integrity(routes: dict[str, dict[str, Any]], aggregate: dict[str, Any]) -> dict[str, Any]:
    route_counts = {route: int(report["rows"]["transition_count"]) for route, report in routes.items()}
    group_sum = {}
    for route, report in routes.items():
        group_sum[route] = sum(int(item["count"]) for item in report["code_rate_residual"]["spearman_and_buckets"]["ordered"].values()) == route_counts[route]
    retained = len(aggregate["route_median_abs_residual_m"]) == 4 and tuple(aggregate["route_median_abs_residual_m"]) == ROUTES
    recomputed_values = list(aggregate["route_median_abs_residual_m"].values())
    recomputed = aggregate["route_median_abs_residual_aggregate_m"] == _median(recomputed_values) and aggregate["route_median_abs_residual_mad_m"] == _mad(recomputed_values)
    return {"group_counts_sum": group_sum, "group_counts_sum_all": all(group_sum.values()), "four_route_medians_retained": retained, "aggregate_recomputed_exact": recomputed}


def _strip_internal(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: _strip_internal(item) for key, item in value.items() if not str(key).startswith("_")}
    if isinstance(value, list):
        return [_strip_internal(item) for item in value]
    return value


def _audit(freeze: dict[str, Any], manifest: dict[str, Any], output_root: Path) -> dict[str, Any]:
    _reject_path(output_root)
    if output_root.exists():
        raise _fail(f"Phase48 output already exists: {output_root}")
    static = _static_contract(freeze)
    reports: dict[str, dict[str, Any]] = {}
    all_events: list[dict[str, Any]] = []
    reads: dict[str, Any] = {"single_process": True, "raw_device_gnss_reads": {}, "raw_device_gnss_read_count_total": 0, "truth_reads": 0, "phase45_payload_reads": 0, "phase47_metric_payload_reads": 0, "brdc_nav_reads": 0, "solver_reruns": 0, "trajectory_reruns": 0, "correction_implementations": 0, "archive_reopens": 0, "rematerializations": 0, "validation_holdout_reads": 0, "kaggle_token_access": 0, "mat_reads_or_generated": 0, "device_wls_or_precomputed_coordinates": 0, "source_static_reads": len(static["source_hashes"])}
    for route in ROUTES:
        pin = freeze["exact_raw_inputs"][route]
        path = ROOT / pin["path"]
        payload, digest = _read_bytes_once(path, f"Phase48 raw device_gnss {route}", pin["sha256"])
        if len(payload) != int(pin["file_size"]):
            raise _fail(f"Phase48 raw byte count mismatch: {route}")
        epochs, metadata = _parse_payload(payload)
        del payload
        report, events = _route_report(route, epochs, metadata, path, digest)
        reports[route] = report
        for event in events:
            event["route"] = route
        all_events.extend(events)
        reads["raw_device_gnss_reads"][route] = 1
        reads["raw_device_gnss_read_count_total"] += 1
    route_medians = {route: float(report["code_rate_residual"]["clock_centered_abs_residual_m"]["median"]) for route, report in reports.items()}
    aggregate = {
        "route_count": len(reports),
        "route_median_abs_residual_m": route_medians,
        "route_median_abs_residual_aggregate_m": _median(route_medians.values()),
        "route_median_abs_residual_mad_m": _mad(route_medians.values()),
        "pairwise_route_median_distances_m": [{"route_a": left, "route_b": right, "distance_m": abs(route_medians[left] - route_medians[right])} for index, left in enumerate(ROUTES) for right in ROUTES[index + 1:]],
    }
    integrity = _presentation_integrity(reports, aggregate)
    loo = _loo(reports)
    min_residuals = min((report["gate_observations"]["residual_count"] for report in reports.values()), default=0)
    min_sats = min((report["gate_observations"]["satellite_count"] for report in reports.values()), default=0)
    min_buckets = min((report["gate_observations"]["populated_uncertainty_buckets"] for report in reports.values()), default=0)
    spearmans = [float(report["gate_observations"]["spearman"]) for report in reports.values()]
    high_low_excess = [float(report["gate_observations"]["high_low_excess_m"]) for report in reports.values()]
    high_low_ratios = [report["gate_observations"]["high_low_ratio"] for report in reports.values()]
    low_base = [float(report["gate_observations"]["low_base_fraction"]) for report in reports.values()]
    within_signal = [int(report["gate_observations"]["within_signal_groups_spearman_ge_0_2"]) for report in reports.values()]
    monotonicity: dict[str, Any] = {}
    for route, report in reports.items():
        ordered = report["code_rate_residual"]["spearman_and_buckets"]["ordered"]
        available = [ordered[name] for name in ("<=10ns", "10_to_100ns", ">100ns") if ordered[name]["count"] > 0]
        monotonicity[route] = {"median_non_decreasing": all(left["median_m"] <= right["median_m"] for left, right in zip(available, available[1:])), "p95_non_decreasing": all(left["p95_m"] <= right["p95_m"] for left, right in zip(available, available[1:])), "bucket_count": len(available)}
    monotonic_pass = all(item["median_non_decreasing"] and item["p95_non_decreasing"] and item["bucket_count"] >= MIN_BUCKETS for item in monotonicity.values())
    static_impact = bool(static["adapter_parses_received_sv_time_uncertainty"] and static["fgo_consumes_received_sv_time_uncertainty_as_sigma"])
    direction_pass = all(value >= MIN_SPEARMAN for value in spearmans)
    high_low_pass = all(value >= HIGH_LOW_EXCESS_M for value in high_low_excess) and all(value is not None and float(value) >= HIGH_LOW_RATIO for value in high_low_ratios)
    coverage_pass = min_residuals >= MIN_RESIDUALS and min_sats >= MIN_SATELLITES
    uncertainty_pass = min_buckets >= MIN_BUCKETS
    low_base_pass = min(low_base, default=0.0) >= MIN_LOW_BASE_FRACTION
    composition_pass = min(within_signal, default=0) >= 2
    gate_rows = [
        {"name": "route_count", "observed": len(reports), "required": 4, "passed": len(reports) == 4 and tuple(reports) == ROUTES},
        {"name": "residual_coverage", "observed_min_residuals": min_residuals, "observed_min_satellites": min_sats, "required_residuals": MIN_RESIDUALS, "required_satellites": MIN_SATELLITES, "passed": coverage_pass},
        {"name": "uncertainty_population", "observed_min_populated_buckets": min_buckets, "required": MIN_BUCKETS, "passed": uncertainty_pass},
        {"name": "spearman_each_route", "observed": spearmans, "required": MIN_SPEARMAN, "passed": direction_pass},
        {"name": "bucket_monotonicity", "observed": monotonicity, "required": "median and p95 non-decreasing", "passed": monotonic_pass},
        {"name": "high_vs_low_p95", "observed_excess_m": high_low_excess, "observed_ratio": high_low_ratios, "required_excess_m": HIGH_LOW_EXCESS_M, "required_ratio": HIGH_LOW_RATIO, "passed": high_low_pass},
        {"name": "low_base_retention", "observed_min_fraction": min(low_base, default=0.0), "required": MIN_LOW_BASE_FRACTION, "passed": low_base_pass},
        {"name": "current_fgo_used_rows_impacted", "observed": static_impact, "required": True, "passed": static_impact},
        {"name": "loo_mapping_direction", "observed": loo, "required": True, "passed": bool(loo["direction_stable"])},
        {"name": "within_signal_composition_independence", "observed_groups": within_signal, "required": 2, "passed": composition_pass},
        {"name": "presentation_integrity", "observed": integrity, "required": True, "passed": bool(integrity["group_counts_sum_all"] and integrity["four_route_medians_retained"] and integrity["aggregate_recomputed_exact"])},
    ]
    passed = all(bool(row["passed"]) for row in gate_rows)
    if passed:
        status = "go-uncertainty-sigma-floor-concept-authorized"
        strongest = "The uncertainty-to-code-rate association passes every predeclared raw-only route, LOO, composition, FGO-adoption, and presentation-integrity gate."
        next_factor = None
        concept = "opt-in sigma floor max(existing_sigma,c*ReceivedSvTimeUncertaintyNanos); concept only, not implemented"
    elif not static_impact:
        status = "no-go-uncertainty-field-not-adopted-by-current-fgo"
        strongest = "ReceivedSvTimeUncertaintyNanos is observable in raw diagnostics but the pinned adapter/current FGO source does not parse and consume it as an estimator sigma, so no deployable effect is authorized."
        next_factor = "raw Android per-satellite carrier-phase ADR cycle-slip/lock-loss residual"
        concept = None
    elif not coverage_pass or not uncertainty_pass:
        status = "no-go-uncertainty-insufficient-raw-coverage"
        strongest = "The raw code-rate transition population or uncertainty bucket coverage is insufficient for four-route identification under the frozen minimums."
        next_factor = "raw Android per-satellite carrier-phase ADR cycle-slip/lock-loss residual"
        concept = None
    elif not direction_pass or not monotonic_pass or not high_low_pass:
        status = "no-go-uncertainty-not-monotonic-or-material"
        strongest = "ReceivedSvTimeUncertaintyNanos does not provide a stable monotonic and materially separated raw code-rate residual across the frozen routes."
        next_factor = "raw Android per-satellite carrier-phase ADR cycle-slip/lock-loss residual"
        concept = None
    elif not integrity["group_counts_sum_all"] or not integrity["four_route_medians_retained"] or not integrity["aggregate_recomputed_exact"]:
        status = "no-go-evaluator-integrity-failure"
        strongest = "The evaluator presentation-integrity invariants failed, so no raw association is promoted after the one-shot audit."
        next_factor = "raw Android per-satellite carrier-phase ADR cycle-slip/lock-loss residual"
        concept = None
    else:
        status = "no-go-uncertainty-composition-or-loo-failure"
        strongest = "The uncertainty association is not stable under the frozen leave-one-route-out or within-signal composition gates."
        next_factor = "raw Android per-satellite carrier-phase ADR cycle-slip/lock-loss residual"
        concept = None
    result = {
        "schema_version": SCHEMA,
        "phase": 48,
        "status": status,
        "mode": "raw-device-gnss-code-rate-uncertainty-audit-only",
        "freeze": {"path": _relative(FREEZE), "sha256": FREEZE_SHA256},
        "evaluator_manifest": {"path": _relative(MANIFEST), "sha256": VERIFIED_MANIFEST_SHA256},
        "read_accounting": reads,
        "routes": {route: _strip_internal(report) for route, report in reports.items()},
        "aggregate": aggregate,
        "loo": loo,
        "static_contract": static,
        "presentation_integrity": integrity,
        "gates": {"all_passed": passed, "rows": gate_rows},
        "decision": {"audit_only": True, "correction_authorized": False, "sigma_floor_concept_if_go": concept, "strongest_finding": strongest, "next_single_raw_physical_factor": next_factor, "truth_or_phase47_metric_used": False, "zero_point_782": "not evaluated without truth"},
        "events": {"count": len(all_events), "table": "phase48_pixel5_raw_code_rate_uncertainty.events.json"},
    }
    output_root.mkdir(parents=True, exist_ok=True)
    route_table = {"schema_version": SCHEMA + ".routes", "routes": result["routes"]}
    event_table = {"schema_version": SCHEMA + ".events", "events": _strip_internal(all_events)}
    routes_path = output_root / "phase48_pixel5_raw_code_rate_uncertainty.routes.json"
    events_path = output_root / "phase48_pixel5_raw_code_rate_uncertainty.events.json"
    routes_bytes = _atomic_json(routes_path, route_table)
    events_bytes = _atomic_json(events_path, event_table)
    result["output_artifacts"] = {_relative(routes_path): {"bytes": len(routes_bytes), "sha256": _sha256_bytes(routes_bytes)}, _relative(events_path): {"bytes": len(events_bytes), "sha256": _sha256_bytes(events_bytes)}}
    result_path = output_root / "phase48_pixel5_raw_code_rate_uncertainty.json"
    result_bytes = _atomic_json(result_path, result)
    artifacts = {_relative(result_path): {"bytes": len(result_bytes), "sha256": _sha256_bytes(result_bytes)}, _relative(routes_path): {"bytes": len(routes_bytes), "sha256": _sha256_bytes(routes_bytes)}, _relative(events_path): {"bytes": len(events_bytes), "sha256": _sha256_bytes(events_bytes)}}
    output_manifest = {"schema_version": SCHEMA + ".output-manifest", "status": "atomic-publish-complete", "phase": 48, "freeze_sha256": FREEZE_SHA256, "evaluator_manifest_sha256": VERIFIED_MANIFEST_SHA256, "read_accounting": reads, "artifacts": artifacts}
    manifest_path = output_root / "phase48_pixel5_raw_code_rate_uncertainty.manifest.json"
    manifest_bytes = _atomic_json(manifest_path, output_manifest)
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        freeze = _verify_freeze()
        manifest = _verify_manifest(freeze)
        _audit(freeze, manifest, args.output)
    except Phase48Error as exc:
        print(f"phase48: {exc}", file=sys.stderr)
        return 2
    print(f"phase48: completed raw code-rate uncertainty audit at {_relative(args.output)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
