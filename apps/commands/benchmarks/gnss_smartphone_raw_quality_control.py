#!/usr/bin/env python3
"""Truth-free raw-observation quality audit and robust SPP candidate.

This command is deliberately separate from the promoted Android handset WLS
lane.  It audits the observables which are visible before truth is opened and,
when an observation RINEX and broadcast navigation file are supplied, runs a
single physically predeclared native SPP configuration with Huber weighting.
The candidate is an experimental fallback only: a malformed or incomplete
native result is replaced by the exact supplied handset-WLS POS, without
interpolation or synthesized epochs.

No truth path is accepted by this interface.  The evaluator which scores this
candidate opens frozen development truth only after all reports and POS files
have been sealed.
"""

from __future__ import annotations

import argparse
import csv
from collections import Counter, defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import shutil
import subprocess
import sys
import tempfile
import time
from typing import Any, Iterable

import numpy as np

# Direct invocation and dispatcher invocation have different ``sys.path``
# roots.  Make the shared command support package explicit before importing
# it; this does not consult any dataset or truth file.
_COMMANDS_DIR = Path(__file__).resolve().parents[1]
if str(_COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(_COMMANDS_DIR))
from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-raw-quality-control.v1"
MANIFEST_SCHEMA_VERSION = "smartphone-r5-raw-quality-control-manifest.v1"
POSITION_SCHEMA_VERSION = "smartphone-r5-raw-quality-control-position.v1"
LEAP_SECONDS = 18
POSITION_MATCH_TOLERANCE_MS = 100
GAP_THRESHOLD_MS = 1_500
EARTH_RADIUS_MIN_M = 6_000_000.0
EARTH_RADIUS_MAX_M = 7_500_000.0
SUPPORTED_SIGNALS = {
    "GPS_L1_CA": {"constellation": "GPS", "frequency_hz": 1_575_420_000.0},
    "GAL_E1_C_P": {"constellation": "Galileo", "frequency_hz": 1_575_420_000.0},
}
RAW_FIELDS = (
    "MessageType",
    "utcTimeMillis",
    "HardwareClockDiscontinuityCount",
    "Svid",
    "SignalType",
    "RawPseudorangeMeters",
    "RawPseudorangeUncertaintyMeters",
    "Cn0DbHz",
    "AccumulatedDeltaRangeState",
    "CarrierFrequencyHz",
    "SvPositionXEcefMeters",
    "SvPositionYEcefMeters",
    "SvPositionZEcefMeters",
    "WlsPositionXEcefMeters",
    "WlsPositionYEcefMeters",
    "WlsPositionZEcefMeters",
    "ReceivedSvTimeUncertaintyNanos",
    "PseudorangeRateUncertaintyMetersPerSecond",
    "AccumulatedDeltaRangeUncertaintyMeters",
)
SENTINEL_LIMITS = {
    "RawPseudorangeUncertaintyMeters": 1.0e6,
    "ReceivedSvTimeUncertaintyNanos": 1.0e8,
    "PseudorangeRateUncertaintyMetersPerSecond": 1.0e6,
    "AccumulatedDeltaRangeUncertaintyMeters": 1.0e6,
}
ROBUST_PARAMETERS = {
    "robust_weighting": True,
    "robust_threshold_sigma": 3.0,
    "robust_min_weight_factor": 0.05,
    "outlier_detection": True,
    "outlier_threshold_sigma": 3.0,
    "raim_fde": True,
    "raim_fde_min_improvement_ratio": 0.25,
    "raim_fde_min_improvement_m": 1.0,
    "variance_model": True,
    "snr_reference_dbhz": 45.0,
    "elevation_mask_deg": 15.0,
    "snr_mask_dbhz": 0.0,
    "galileo_e1_hatch_window_s": 30,
    "leap_seconds": LEAP_SECONDS,
}


class QualityControlError(ValueError):
    """Raised when the truth-free input or artifact contract is invalid."""


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise QualityControlError(f"missing file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_write(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: str | None = None
    descriptor: int | None = None
    try:
        descriptor, temporary = tempfile.mkstemp(
            prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
        )
        with os.fdopen(descriptor, "wb") as handle:
            descriptor = None
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = None
        if hasattr(os, "O_DIRECTORY"):
            directory = os.open(str(path.parent), os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
    except OSError as exc:
        raise QualityControlError(f"atomic publish failed for {path}: {exc}") from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)
        if temporary is not None:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    _atomic_write(path, (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode())


def _number(raw: Any) -> float | None:
    if raw is None:
        return None
    token = str(raw).strip()
    if not token:
        return None
    try:
        value = float(token)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _integer(raw: Any, field: str, row_number: int) -> int:
    token = str(raw or "").strip()
    if not token:
        raise QualityControlError(f"row {row_number}: {field} is empty")
    try:
        value = float(token)
    except ValueError as exc:
        raise QualityControlError(f"row {row_number}: {field} is not integral") from exc
    if not math.isfinite(value) or not value.is_integer():
        raise QualityControlError(f"row {row_number}: {field} is not integral")
    return int(value)


def _percentile(values: Iterable[float], fraction: float) -> float | None:
    ordered = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not ordered:
        return None
    rank = (len(ordered) - 1) * fraction
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - rank) + ordered[upper] * (rank - lower)


def _distribution(values: Iterable[float]) -> dict[str, Any]:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    return {
        "count": len(finite),
        "p50": _percentile(finite, 0.50),
        "p95": _percentile(finite, 0.95),
        "max": max(finite) if finite else None,
    }


def _valid_quality_number(row: dict[str, str], field: str) -> float | None:
    value = _number(row.get(field))
    if value is None or value <= 0.0:
        return None
    sentinel_limit = SENTINEL_LIMITS.get(field)
    if sentinel_limit is not None and value >= sentinel_limit:
        return None
    return value


def _ecef(row: dict[str, str], prefix: str) -> tuple[float, float, float] | None:
    values = tuple(_number(row.get(f"{prefix}{axis}EcefMeters")) for axis in "XYZ")
    if any(value is None for value in values):
        return None
    result = tuple(float(value) for value in values)  # type: ignore[arg-type]
    if not all(math.isfinite(value) for value in result):
        return None
    return result


def _earth_plausible(ecef: tuple[float, float, float]) -> bool:
    norm = math.sqrt(sum(value * value for value in ecef))
    return EARTH_RADIUS_MIN_M <= norm <= EARTH_RADIUS_MAX_M


def _geometry(rows: list[dict[str, str]], wls_ecef: tuple[float, float, float] | None) -> tuple[int | None, float | None, int]:
    if wls_ecef is None:
        return None, None, 0
    design: list[list[float]] = []
    for row in rows:
        if row.get("SignalType", "").strip() not in SUPPORTED_SIGNALS:
            continue
        satellite = _ecef(row, "SvPosition")
        if satellite is None:
            continue
        delta = np.asarray(satellite, dtype=float) - np.asarray(wls_ecef, dtype=float)
        distance = float(np.linalg.norm(delta))
        if not math.isfinite(distance) or distance <= 1.0:
            continue
        line_of_sight = delta / distance
        design.append([-float(line_of_sight[0]), -float(line_of_sight[1]), -float(line_of_sight[2]), 1.0])
    if len(design) < 4:
        return None, None, len(design)
    matrix = np.asarray(design, dtype=float)
    try:
        rank = int(np.linalg.matrix_rank(matrix))
        if rank < 4:
            return rank, None, len(design)
        normal_pseudoinverse = np.linalg.pinv(matrix.T @ matrix)
        gdop = math.sqrt(float(np.trace(normal_pseudoinverse)))
    except (np.linalg.LinAlgError, ValueError, FloatingPointError):
        return rank if "rank" in locals() else None, None, len(design)
    return rank, gdop if math.isfinite(gdop) else None, len(design)


def _read_epochs(path: Path) -> list[tuple[int, list[dict[str, str]]]]:
    if not path.is_file():
        raise QualityControlError(f"missing device GNSS CSV: {path}")
    try:
        handle = path.open(encoding="utf-8-sig", newline="")
    except OSError as exc:
        raise QualityControlError(f"failed to open device GNSS CSV: {path}") from exc
    try:
        reader = csv.DictReader(handle)
        fields = list(reader.fieldnames or ())
        if len(fields) != len(set(fields)):
            raise QualityControlError("device GNSS CSV has duplicate fields")
        missing = [field for field in RAW_FIELDS if field not in fields]
        if missing:
            raise QualityControlError(
                "device GNSS CSV missing required fields: " + ", ".join(missing)
            )
        epochs: list[tuple[int, list[dict[str, str]]]] = []
        current_timestamp: int | None = None
        current_rows: list[dict[str, str]] = []
        previous_timestamp: int | None = None
        saw_row = False
        for row_number, raw in enumerate(reader, start=2):
            row = {key: (value or "") for key, value in raw.items()}
            saw_row = True
            if row.get("MessageType") != "Raw":
                raise QualityControlError(f"row {row_number}: MessageType must be Raw")
            timestamp = _integer(row.get("utcTimeMillis"), "utcTimeMillis", row_number)
            if timestamp < 0:
                raise QualityControlError(f"row {row_number}: utcTimeMillis is negative")
            if previous_timestamp is not None and timestamp < previous_timestamp:
                raise QualityControlError(f"row {row_number}: utcTimeMillis moved backwards")
            if current_timestamp is None:
                current_timestamp = timestamp
            elif timestamp != current_timestamp:
                epochs.append((current_timestamp, current_rows))
                current_timestamp = timestamp
                current_rows = []
            current_rows.append(row)
            previous_timestamp = timestamp
        if current_timestamp is not None:
            epochs.append((current_timestamp, current_rows))
        if not saw_row or not epochs:
            raise QualityControlError(f"device GNSS CSV has no Raw rows: {path}")
        return epochs
    finally:
        handle.close()


def _epoch_quality(timestamp: int, rows: list[dict[str, str]], previous_timestamp: int | None) -> tuple[dict[str, Any], dict[str, list[float]]]:
    supported = [row for row in rows if row.get("SignalType", "").strip() in SUPPORTED_SIGNALS]
    wls_values = [value for row in rows if (value := _ecef(row, "WlsPosition")) is not None]
    wls_ecef = wls_values[0] if wls_values else None
    wls_spread_m = None
    if wls_values:
        wls_spread_m = max(
            max(value[index] for value in wls_values) - min(value[index] for value in wls_values)
            for index in range(3)
        )
    raw_residuals: list[tuple[str, float]] = []
    raw_uncertainties: list[float] = []
    cn0: list[float] = []
    received_uncertainties: list[float] = []
    rate_uncertainties: list[float] = []
    adr_uncertainties: list[float] = []
    adr_states: list[int] = []
    signal_counts: Counter[str] = Counter()
    constellation_counts: Counter[str] = Counter()
    frequencies: Counter[str] = Counter()
    residual_sources = 0
    geometry_rows = 0
    for row in supported:
        signal = row.get("SignalType", "").strip()
        info = SUPPORTED_SIGNALS[signal]
        signal_counts[signal] += 1
        constellation_counts[str(info["constellation"])] += 1
        frequency = _number(row.get("CarrierFrequencyHz"))
        if frequency is not None and frequency > 0.0:
            frequencies[str(int(round(frequency)))] += 1
        uncertainty = _valid_quality_number(row, "RawPseudorangeUncertaintyMeters")
        if uncertainty is not None:
            raw_uncertainties.append(uncertainty)
        value = _valid_quality_number(row, "Cn0DbHz")
        if value is not None:
            cn0.append(value)
        for field, destination in (
            ("ReceivedSvTimeUncertaintyNanos", received_uncertainties),
            ("PseudorangeRateUncertaintyMetersPerSecond", rate_uncertainties),
            ("AccumulatedDeltaRangeUncertaintyMeters", adr_uncertainties),
        ):
            quality_value = _valid_quality_number(row, field)
            if quality_value is not None:
                destination.append(quality_value)
        state = _number(row.get("AccumulatedDeltaRangeState"))
        if state is not None and state.is_integer():
            adr_states.append(int(state))
        raw_code = _number(row.get("RawPseudorangeMeters"))
        satellite = _ecef(row, "SvPosition")
        if raw_code is not None and raw_code > 0.0 and satellite is not None and wls_ecef is not None:
            range_m = math.sqrt(sum((satellite[index] - wls_ecef[index]) ** 2 for index in range(3)))
            if math.isfinite(range_m) and range_m > 1.0:
                raw_residuals.append((signal, raw_code - range_m))
                residual_sources += 1
        if satellite is not None and wls_ecef is not None:
            delta = math.sqrt(sum((satellite[index] - wls_ecef[index]) ** 2 for index in range(3)))
            if math.isfinite(delta) and delta > 1.0:
                geometry_rows += 1
    centered_by_signal: dict[str, float] = {}
    grouped: defaultdict[str, list[float]] = defaultdict(list)
    for signal, value in raw_residuals:
        grouped[signal].append(value)
    for signal, values in grouped.items():
        center = _percentile(values, 0.50)
        if center is not None:
            centered_by_signal[signal] = center
    centered_abs: list[float] = []
    standardized_abs: list[float] = []
    for row in supported:
        signal = row.get("SignalType", "").strip()
        raw_code = _number(row.get("RawPseudorangeMeters"))
        satellite = _ecef(row, "SvPosition")
        if raw_code is None or raw_code <= 0.0 or satellite is None or wls_ecef is None:
            continue
        range_m = math.sqrt(sum((satellite[index] - wls_ecef[index]) ** 2 for index in range(3)))
        if not math.isfinite(range_m) or range_m <= 1.0:
            continue
        centered = raw_code - range_m - centered_by_signal.get(signal, 0.0)
        if not math.isfinite(centered):
            continue
        absolute = abs(centered)
        centered_abs.append(absolute)
        uncertainty = _valid_quality_number(row, "RawPseudorangeUncertaintyMeters")
        standardized_abs.append(absolute / max(uncertainty or 1.0, 1.0))
    geometry_rank, gdop_proxy, geometry_count = _geometry(supported, wls_ecef)
    clock_values = set()
    for row in rows:
        clock = _number(row.get("HardwareClockDiscontinuityCount"))
        if clock is not None and clock.is_integer():
            clock_values.add(int(clock))
    if len(clock_values) > 1:
        raise QualityControlError(f"epoch {timestamp}: inconsistent clock discontinuity count")
    clock_count = next(iter(clock_values)) if clock_values else None
    epoch_gap_s = (timestamp - previous_timestamp) / 1000.0 if previous_timestamp is not None else None
    feature = {
        "timestamp_ms": timestamp,
        "row_count": len(rows),
        "supported_row_count": len(supported),
        "unsupported_row_count": len(rows) - len(supported),
        "signal_counts": dict(sorted(signal_counts.items())),
        "constellation_counts": dict(sorted(constellation_counts.items())),
        "frequencies_hz": dict(sorted(frequencies.items())),
        "raw_residual_proxy_abs_p95_m": _percentile(centered_abs, 0.95),
        "standardized_abs_residual_p95_sigma": _percentile(standardized_abs, 0.95),
        "standardized_abs_residual_tail_gt3_count": sum(value > 3.0 for value in standardized_abs),
        "standardized_abs_residual_tail_gt5_count": sum(value > 5.0 for value in standardized_abs),
        "raw_pseudorange_uncertainty_p95_m": _percentile(raw_uncertainties, 0.95),
        "cn0_median_dbhz": _percentile(cn0, 0.50),
        "received_sv_time_uncertainty_p95_ns": _percentile(received_uncertainties, 0.95),
        "pseudorange_rate_uncertainty_p95_mps": _percentile(rate_uncertainties, 0.95),
        "adr_uncertainty_p95_m": _percentile(adr_uncertainties, 0.95),
        "adr_valid_fraction": (sum(bool(state & 1) for state in adr_states) / len(adr_states)) if adr_states else None,
        "adr_reset_or_cycle_slip_count": sum(bool(state & 0b110) for state in adr_states),
        "clock_discontinuity_count": clock_count,
        "epoch_gap_s": epoch_gap_s,
        "geometry_row_count": geometry_count,
        "geometry_rank": geometry_rank,
        "gdop_proxy": gdop_proxy,
        "wls_ecef_available": wls_ecef is not None,
        "wls_ecef_earth_plausible": _earth_plausible(wls_ecef) if wls_ecef is not None else None,
        "wls_ecef_spread_m": wls_spread_m,
        "residual_source_count": residual_sources,
    }
    return feature, {
        "residual_abs_m": centered_abs,
        "standardized_abs_sigma": standardized_abs,
        "raw_uncertainty_m": raw_uncertainties,
        "cn0_dbhz": cn0,
        "received_sv_uncertainty_ns": received_uncertainties,
        "pseudorange_rate_uncertainty_mps": rate_uncertainties,
        "adr_uncertainty_m": adr_uncertainties,
    }


def build_observable_audit(device_gnss: Path) -> dict[str, Any]:
    """Build a deterministic, truth-free raw quality report."""

    epochs = _read_epochs(device_gnss)
    previous_timestamp: int | None = None
    clock_transitions = 0
    previous_clock: int | None = None
    distributions: defaultdict[str, list[float]] = defaultdict(list)
    signal_distributions: defaultdict[str, defaultdict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    epoch_features: list[dict[str, Any]] = []
    supported_rows = 0
    total_rows = 0
    unsupported_counts: Counter[str] = Counter()
    supported_counts: Counter[str] = Counter()
    gap_count = 0
    max_gap_ms = 0
    residual_tail_gt3 = 0
    residual_tail_gt5 = 0
    for timestamp, rows in epochs:
        feature, values = _epoch_quality(timestamp, rows, previous_timestamp)
        epoch_features.append(feature)
        total_rows += len(rows)
        supported_rows += int(feature["supported_row_count"])
        for row in rows:
            signal = row.get("SignalType", "").strip() or "<empty>"
            if signal in SUPPORTED_SIGNALS:
                supported_counts[signal] += 1
            else:
                unsupported_counts[signal] += 1
        for key, numbers in values.items():
            distributions[key].extend(numbers)
        for signal, raw_count in feature["signal_counts"].items():
            signal_distributions[signal]["row_count"].append(float(raw_count))
        residual_tail_gt3 += int(feature["standardized_abs_residual_tail_gt3_count"])
        residual_tail_gt5 += int(feature["standardized_abs_residual_tail_gt5_count"])
        if feature["epoch_gap_s"] is not None:
            gap_ms = timestamp - (previous_timestamp or timestamp)
            if gap_ms > GAP_THRESHOLD_MS:
                gap_count += 1
                max_gap_ms = max(max_gap_ms, gap_ms)
        clock = feature["clock_discontinuity_count"]
        if clock is not None and previous_clock is not None and clock != previous_clock:
            clock_transitions += 1
        if clock is not None:
            previous_clock = clock
        previous_timestamp = timestamp
    per_signal: dict[str, Any] = {}
    for signal in sorted(SUPPORTED_SIGNALS):
        rows_for_signal = [
            row
            for timestamp, rows in epochs
            for row in rows
            if row.get("SignalType", "").strip() == signal
        ]
        local: dict[str, list[float]] = defaultdict(list)
        for row in rows_for_signal:
            raw_code = _number(row.get("RawPseudorangeMeters"))
            sat = _ecef(row, "SvPosition")
            wls = _ecef(row, "WlsPosition")
            if raw_code is not None and sat is not None and wls is not None:
                distance = math.sqrt(sum((sat[index] - wls[index]) ** 2 for index in range(3)))
                if math.isfinite(distance) and distance > 1.0:
                    local["raw_range_proxy_m"].append(raw_code - distance)
            uncertainty = _valid_quality_number(row, "RawPseudorangeUncertaintyMeters")
            cn0 = _valid_quality_number(row, "Cn0DbHz")
            if uncertainty is not None:
                local["raw_uncertainty_m"].append(uncertainty)
            if cn0 is not None:
                local["cn0_dbhz"].append(cn0)
        per_signal[signal] = {
            "rows": len(rows_for_signal),
            "distributions": {key: _distribution(numbers) for key, numbers in sorted(local.items())},
        }
    return {
        "schema_version": SCHEMA_VERSION,
        "decision": "truth-free-raw-observable-audit",
        "truth_free": True,
        "truth_path": None,
        "inputs": {"device_gnss": {"path": str(device_gnss), "sha256": _sha256(device_gnss)}},
        "contract": {
            "supported_signals": sorted(SUPPORTED_SIGNALS),
            "signal_frequency_policy_hz": {key: value["frequency_hz"] for key, value in sorted(SUPPORTED_SIGNALS.items())},
            "sentinel_policy": "empty, nonfinite, nonpositive quality values and documented Android large sentinels are unavailable, never numeric",
            "innovation_proxy": "RawPseudorangeMeters minus satellite-to-handset-WLS range, centered by supported SignalType within each epoch",
            "standardization": "absolute centered proxy divided by max(finite RawPseudorangeUncertaintyMeters, 1 m); unavailable uncertainty uses the fixed 1 m floor",
            "geometry": "finite supported satellite ECEF line-of-sight design [−los_x,−los_y,−los_z,1], rank and sqrt(trace(pinv(G'G))) GDOP proxy",
            "gap_threshold_ms": GAP_THRESHOLD_MS,
            "truth_access": "not accepted by this command; no labels influence quality features",
        },
        "populations": {
            "rows": total_rows,
            "epochs": len(epochs),
            "supported_rows": supported_rows,
            "unsupported_rows": total_rows - supported_rows,
            "supported_signal_rows": dict(sorted(supported_counts.items())),
            "unsupported_signal_rows": dict(sorted(unsupported_counts.items())),
            "clock_transition_count": clock_transitions,
            "timestamp_gap_count_gt_1500ms": gap_count,
            "max_timestamp_gap_s": max_gap_ms / 1000.0,
            "first_timestamp_ms": epochs[0][0],
            "last_timestamp_ms": epochs[-1][0],
        },
        "distributions": {key: _distribution(numbers) for key, numbers in sorted(distributions.items())},
        "tail_counts": {
            "standardized_abs_residual_gt3": residual_tail_gt3,
            "standardized_abs_residual_gt5": residual_tail_gt5,
        },
        "per_signal": per_signal,
        "epoch_features": epoch_features,
    }


def write_observable_audit(device_gnss: Path, output_dir: Path, *, dataset_id: str | None = None) -> dict[str, Any]:
    report = build_observable_audit(device_gnss)
    report["dataset_id"] = dataset_id
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / "quality_report.json"
    _atomic_json(report_path, report)
    manifest = {
        "schema_version": MANIFEST_SCHEMA_VERSION,
        "truth_free": True,
        "truth_path": None,
        "dataset_id": dataset_id,
        "input": {"path": str(device_gnss), "sha256": _sha256(device_gnss)},
        "report": {"path": str(report_path), "sha256": _sha256(report_path), "bytes": report_path.stat().st_size},
    }
    manifest_path = output_dir / "quality_manifest.json"
    _atomic_json(manifest_path, manifest)
    return {"report": report, "report_path": report_path, "manifest_path": manifest_path, "manifest": manifest}


def _position_lines(path: Path) -> tuple[list[str], dict[int, str]]:
    if not path.is_file():
        raise QualityControlError(f"missing POS file: {path}")
    headers: list[str] = []
    data: dict[int, str] = {}
    try:
        with path.open(encoding="ascii") as handle:
            for line_number, raw_line in enumerate(handle, start=1):
                stripped = raw_line.strip()
                if not stripped or stripped.startswith("%") or stripped.startswith("#"):
                    headers.append(raw_line)
                    continue
                fields = stripped.split()
                if len(fields) < 11:
                    raise QualityControlError(f"POS line {line_number}: expected at least 11 fields")
                try:
                    timestamp = smoother._position_timestamp(fields[0], fields[1], LEAP_SECONDS, line_number)
                    # _read_positions performs all finite ECEF/geodetic checks.
                    float_values = [float(fields[index]) for index in range(2, 8)]
                except (ValueError, smoother.SmootherError) as exc:
                    raise QualityControlError(f"POS line {line_number}: invalid numeric value") from exc
                if not all(math.isfinite(value) for value in float_values):
                    raise QualityControlError(f"POS line {line_number}: nonfinite coordinate")
                if timestamp in data:
                    raise QualityControlError(f"POS has duplicate timestamp {timestamp}")
                data[timestamp] = " ".join(fields) + "\n"
    except OSError as exc:
        raise QualityControlError(f"failed to read POS: {path}") from exc
    if not data:
        raise QualityControlError(f"POS has no data rows: {path}")
    smoother._read_positions(path, LEAP_SECONDS)
    return headers, data


def _rewrite_position_timestamp(line: str, reference_line: str) -> str:
    fields = line.strip().split()
    reference = reference_line.strip().split()
    fields[0] = reference[0]
    fields[1] = reference[1]
    return " ".join(fields) + "\n"


def _merge_positions(
    robust_position: Path,
    fallback_position: Path,
    *,
    tolerance_ms: int = POSITION_MATCH_TOLERANCE_MS,
) -> tuple[bytes, dict[str, int]]:
    """Merge robust rows onto exact fallback/device keys, fail closed on ambiguity."""

    fallback_headers, fallback_rows = _position_lines(fallback_position)
    fallback_order = list(fallback_rows)
    robust_headers, robust_rows = _position_lines(robust_position)
    robust_mapped: dict[int, str] = {}
    for robust_timestamp, line in robust_rows.items():
        nearest = min(fallback_order, key=lambda value: abs(value - robust_timestamp))
        if abs(nearest - robust_timestamp) > tolerance_ms:
            raise QualityControlError(
                f"robust POS timestamp {robust_timestamp} is outside {tolerance_ms} ms fallback tolerance"
            )
        if nearest in robust_mapped:
            raise QualityControlError(f"robust POS maps multiple rows to {nearest}")
        robust_mapped[nearest] = _rewrite_position_timestamp(line, fallback_rows[nearest])
    if not robust_mapped:
        raise QualityControlError("robust POS has no mapped rows")
    headers = robust_headers if robust_headers else fallback_headers
    content = "".join(headers)
    robust_count = 0
    fallback_count = 0
    for timestamp in fallback_order:
        if timestamp in robust_mapped:
            content += robust_mapped[timestamp]
            robust_count += 1
        else:
            content += fallback_rows[timestamp]
            fallback_count += 1
    return content.encode("ascii"), {"robust": robust_count, "fallback": fallback_count, "total": len(fallback_order)}


def _resource_rss_kb() -> int:
    return int(resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss)


def run_robust_candidate(
    device_gnss: Path,
    obs: Path,
    nav: Path,
    fallback_position: Path,
    output_dir: Path,
    *,
    dataset_id: str | None = None,
    binary: Path | None = None,
) -> dict[str, Any]:
    """Run fixed robust SPP and atomically publish a POS with exact fallback."""

    for path in (device_gnss, obs, nav, fallback_position):
        if not path.is_file():
            raise QualityControlError(f"missing candidate input: {path}")
    output_dir.mkdir(parents=True, exist_ok=True)
    audit = write_observable_audit(device_gnss, output_dir, dataset_id=dataset_id)
    solver_binary = binary or (ROOT / "build" / "apps" / "gnss_spp")
    if not solver_binary.is_file() or not os.access(solver_binary, os.X_OK):
        raise QualityControlError(f"native SPP binary is unavailable or not executable: {solver_binary}")
    started = time.perf_counter()
    return_code: int | None = None
    solver_stdout = ""
    solver_stderr = ""
    robust_source: dict[str, int] = {"robust": 0, "fallback": 0, "total": 0}
    solver_summary: dict[str, Any] | None = None
    robust_valid = False
    with tempfile.TemporaryDirectory(prefix="raw-quality-spp-", dir=str(output_dir)) as staging_name:
        staging = Path(staging_name)
        robust_path = staging / "robust_spp.pos"
        summary_path = staging / "robust_spp_summary.json"
        timing_path = staging / "robust_spp_timing.csv"
        command = [
            str(solver_binary),
            "--obs", str(obs),
            "--nav", str(nav),
            "--out", str(robust_path),
            "--summary-json", str(summary_path),
            "--timing-csv", str(timing_path),
            "--robust-weighting",
            "--robust-threshold-sigma", "3",
            "--robust-min-weight", "0.05",
            "--quiet",
        ]
        try:
            completed = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=300,
            )
            return_code = int(completed.returncode)
            solver_stdout = completed.stdout[-2000:]
            solver_stderr = completed.stderr[-2000:]
        except (OSError, subprocess.SubprocessError) as exc:
            solver_stderr = str(exc)
            return_code = None
        if summary_path.is_file():
            try:
                value = json.loads(summary_path.read_text(encoding="utf-8"))
                if isinstance(value, dict):
                    solver_summary = value
            except (OSError, json.JSONDecodeError):
                solver_summary = None
        if return_code == 0 and robust_path.is_file():
            try:
                merged_content, robust_source = _merge_positions(robust_path, fallback_position)
                robust_valid = True
            except QualityControlError:
                robust_valid = False
        if not robust_valid:
            merged_content = fallback_position.read_bytes()
            fallback_rows = _position_lines(fallback_position)[1]
            robust_source = {"robust": 0, "fallback": len(fallback_rows), "total": len(fallback_rows)}
        _atomic_write(output_dir / "robust_spp_or_wls.pos", merged_content)
        if summary_path.is_file():
            _atomic_write(output_dir / "robust_spp_summary.json", summary_path.read_bytes())
        else:
            _atomic_json(output_dir / "robust_spp_summary.json", {"solver_summary_available": False})
        if timing_path.is_file():
            _atomic_write(output_dir / "robust_spp_timing.csv", timing_path.read_bytes())
        else:
            _atomic_write(output_dir / "robust_spp_timing.csv", b"")
    elapsed = time.perf_counter() - started
    candidate_summary = {
        "schema_version": POSITION_SCHEMA_VERSION,
        "decision": "robust-spp-with-exact-wls-fallback" if robust_valid else "exact-wls-fallback",
        "truth_free": True,
        "truth_path": None,
        "dataset_id": dataset_id,
        "algorithm": {
            "id": "innovation_huber_native_spp_v1",
            "parameters": ROBUST_PARAMETERS,
            "fallback_policy": "if native output is invalid, incomplete, nonfinite, ambiguous, or process-failed, copy exact fallback POS; no extrapolation",
        },
        "inputs": {
            "device_gnss": {"path": str(device_gnss), "sha256": _sha256(device_gnss)},
            "obs": {"path": str(obs), "sha256": _sha256(obs)},
            "nav": {"path": str(nav), "sha256": _sha256(nav)},
            "fallback_position": {"path": str(fallback_position), "sha256": _sha256(fallback_position)},
        },
        "solver": {
            "binary": {"path": str(solver_binary), "sha256": _sha256(solver_binary)},
            "command": command,
            "return_code": return_code,
            "stdout_tail": solver_stdout,
            "stderr_tail": solver_stderr,
            "summary": solver_summary,
        },
        "source_counts": robust_source,
        "performance": {"wall_s": elapsed, "child_max_rss_kb": _resource_rss_kb()},
        "artifacts": {
            "quality_report": {"path": str(output_dir / "quality_report.json"), "sha256": _sha256(output_dir / "quality_report.json")},
            "position": {"path": str(output_dir / "robust_spp_or_wls.pos"), "sha256": _sha256(output_dir / "robust_spp_or_wls.pos")},
            "solver_summary": {"path": str(output_dir / "robust_spp_summary.json"), "sha256": _sha256(output_dir / "robust_spp_summary.json")},
            "timing": {"path": str(output_dir / "robust_spp_timing.csv"), "sha256": _sha256(output_dir / "robust_spp_timing.csv")},
        },
    }
    summary_path = output_dir / "robust_spp_candidate.json"
    _atomic_json(summary_path, candidate_summary)
    manifest = {
        "schema_version": MANIFEST_SCHEMA_VERSION,
        "truth_free": True,
        "truth_path": None,
        "dataset_id": dataset_id,
        "candidate": {"path": str(summary_path), "sha256": _sha256(summary_path)},
        "quality_manifest": {"path": str(audit["manifest_path"]), "sha256": _sha256(audit["manifest_path"])},
        "position": {"path": str(output_dir / "robust_spp_or_wls.pos"), "sha256": _sha256(output_dir / "robust_spp_or_wls.pos")},
        "inputs": candidate_summary["inputs"],
        "atomic_publish": True,
    }
    manifest_path = output_dir / "candidate_manifest.json"
    _atomic_json(manifest_path, manifest)
    return {
        "candidate": candidate_summary,
        "manifest": manifest,
        "manifest_path": manifest_path,
        "position_path": output_dir / "robust_spp_or_wls.pos",
        "quality_report_path": audit["report_path"],
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-raw-quality-control")
    )
    parser.add_argument("--device-gnss", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--position", type=Path, help="Optional WLS POS used only for audit metadata")
    parser.add_argument("--obs", type=Path)
    parser.add_argument("--nav", type=Path)
    parser.add_argument("--fallback-position", type=Path)
    parser.add_argument("--dataset-id")
    parser.add_argument("--binary", type=Path)
    args = parser.parse_args(argv)
    candidate_values = (args.obs, args.nav, args.fallback_position)
    if any(value is not None for value in candidate_values) and not all(value is not None for value in candidate_values):
        parser.error("--obs, --nav, and --fallback-position must be supplied together")
    if args.position is not None and args.fallback_position is None and not args.position.is_file():
        parser.error("--position does not exist")
    return args


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.obs is None:
            result = write_observable_audit(args.device_gnss, args.output_dir, dataset_id=args.dataset_id)
            print(f"Raw smartphone quality audit complete: {result['report_path']}")
        else:
            result = run_robust_candidate(
                args.device_gnss,
                args.obs,
                args.nav,
                args.fallback_position,
                args.output_dir,
                dataset_id=args.dataset_id,
                binary=args.binary,
            )
            print(f"Raw smartphone quality candidate complete: {result['manifest_path']}")
        return 0
    except (QualityControlError, smoother.SmootherError, ValueError) as exc:
        print(f"Raw smartphone quality control failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(run())
