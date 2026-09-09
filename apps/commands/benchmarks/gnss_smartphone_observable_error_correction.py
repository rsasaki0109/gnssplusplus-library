#!/usr/bin/env python3
"""Truth-free observable-feature residual correction for handset WLS.

The inference lane reads only ``device_gnss.csv`` and a sealed model.  It
never accepts a ground-truth path.  Training is intentionally implemented in
the separate evaluation command so that a truth-free production/submission
process cannot accidentally consume labels.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
from typing import Any, Iterable

import numpy as np

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_wls as wls  # noqa: E402


SCHEMA_VERSION = "smartphone-observable-error-correction.v1"
MODEL_SCHEMA_VERSION = "smartphone-observable-error-correction-model.v1"
FEATURE_MANIFEST_SCHEMA_VERSION = "smartphone-observable-error-correction-features.v1"
MODEL_ID = "observable_ridge_residual_v1"
MODEL_ALPHA = 100.0
DEFAULT_SKIP_EPOCHS = 1
DEFAULT_LEAP_SECONDS = 18
_INTEGER_RE = re.compile(r"^[+-]?\d+$")
_FLOAT_RE = re.compile(
    r"^[+-]?(?:(?:\d+(?:\.\d*)?)|(?:\.\d+))(?:[eE][+-]?\d+)?$"
)

FEATURE_NAMES = (
    "wls_ecef_x_m",
    "wls_ecef_y_m",
    "wls_ecef_z_m",
    "wls_ecef_delta_prev_x_m",
    "wls_ecef_delta_prev_y_m",
    "wls_ecef_delta_prev_z_m",
    "epoch_dt_s",
    "raw_row_count",
    "eligible_gps_l1_count",
    "eligible_galileo_e1_count",
    "cn0_mean_dbhz",
    "cn0_median_dbhz",
    "cn0_std_dbhz",
    "cn0_max_dbhz",
    "pseudorange_rate_uncertainty_mps_median",
    "adr_valid_count",
    "satellite_geometry_trace_m2",
    "satellite_geometry_min_eigenvalue_m2",
    "satellite_geometry_max_eigenvalue_m2",
    "clock_discontinuity_count",
    "time_of_day_sin",
    "time_of_day_cos",
)

RAW_FEATURE_FIELDS = (
    "utcTimeMillis",
    "HardwareClockDiscontinuityCount",
    "Svid",
    "State",
    "Cn0DbHz",
    "PseudorangeRateUncertaintyMetersPerSecond",
    "AccumulatedDeltaRangeState",
    "ConstellationType",
    "SignalType",
    "SvPositionXEcefMeters",
    "SvPositionYEcefMeters",
    "SvPositionZEcefMeters",
    "WlsPositionXEcefMeters",
    "WlsPositionYEcefMeters",
    "WlsPositionZEcefMeters",
)


class ObservableCorrectionError(ValueError):
    """Raised when a truth-free observable correction contract is invalid."""


@dataclass(frozen=True)
class ObservableFeatureRow:
    timestamp_ms: int
    ecef: np.ndarray
    features: np.ndarray
    source_line: int
    raw_row_count: int
    satellite_count: int


@dataclass(frozen=True)
class FeatureExtraction:
    rows: tuple[ObservableFeatureRow, ...]
    input_rows: int
    input_epochs: int
    selected_epochs: int
    skipped_epochs: int
    timestamp_gap_count: int
    max_timestamp_gap_s: float
    device_sha256: str


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise ObservableCorrectionError(f"missing file: {path}")
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
        raise ObservableCorrectionError(f"atomic publish failed for {path}: {exc}") from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)
        if temporary is not None:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    _atomic_write(path, (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def _parse_int(raw: str | None, field: str, line: int) -> int:
    token = (raw or "").strip()
    if not _INTEGER_RE.fullmatch(token):
        raise ObservableCorrectionError(f"row {line}: {field} must be an integer")
    try:
        return int(token)
    except ValueError as exc:
        raise ObservableCorrectionError(f"row {line}: {field} is not representable") from exc


def _parse_optional_float(raw: str | None, field: str, line: int) -> float | None:
    token = (raw or "").strip()
    if not token:
        return None
    if not _FLOAT_RE.fullmatch(token):
        raise ObservableCorrectionError(f"row {line}: {field} must be finite")
    try:
        value = float(token)
    except ValueError as exc:
        raise ObservableCorrectionError(f"row {line}: {field} must be finite") from exc
    if not math.isfinite(value):
        raise ObservableCorrectionError(f"row {line}: {field} must be finite")
    return value


def _raw_epoch_rows(device_gnss: Path) -> tuple[dict[int, list[dict[str, str]]], int]:
    if not device_gnss.is_file():
        raise ObservableCorrectionError(f"missing device GNSS CSV: {device_gnss}")
    grouped: dict[int, list[dict[str, str]]] = {}
    input_rows = 0
    previous_timestamp: int | None = None
    try:
        with device_gnss.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            fields = list(reader.fieldnames or ())
            if len(fields) != len(set(fields)):
                raise ObservableCorrectionError("device GNSS CSV has duplicate fields")
            missing = [field for field in RAW_FEATURE_FIELDS if field not in fields]
            if missing:
                raise ObservableCorrectionError(
                    "device GNSS CSV missing feature fields: " + ", ".join(missing)
                )
            for line, row in enumerate(reader, start=2):
                input_rows += 1
                if None in row:
                    raise ObservableCorrectionError(f"row {line}: extra CSV fields")
                if row.get("MessageType") != "Raw":
                    raise ObservableCorrectionError(f"row {line}: MessageType must be Raw")
                timestamp = _parse_int(row.get("utcTimeMillis"), "utcTimeMillis", line)
                if timestamp < 0:
                    raise ObservableCorrectionError(f"row {line}: timestamp must be non-negative")
                if previous_timestamp is not None and timestamp < previous_timestamp:
                    raise ObservableCorrectionError(f"row {line}: timestamp moved backwards")
                previous_timestamp = timestamp
                _parse_int(row.get("Svid"), "Svid", line)
                _parse_int(
                    row.get("HardwareClockDiscontinuityCount"),
                    "HardwareClockDiscontinuityCount",
                    line,
                )
                for field in (
                    "Cn0DbHz",
                    "PseudorangeRateUncertaintyMetersPerSecond",
                    "AccumulatedDeltaRangeMeters",
                    "SvPositionXEcefMeters",
                    "SvPositionYEcefMeters",
                    "SvPositionZEcefMeters",
                    "WlsPositionXEcefMeters",
                    "WlsPositionYEcefMeters",
                    "WlsPositionZEcefMeters",
                ):
                    _parse_optional_float(row.get(field), field, line)
                _parse_int(row.get("AccumulatedDeltaRangeState"), "AccumulatedDeltaRangeState", line)
                grouped.setdefault(timestamp, []).append(row)
    except OSError as exc:
        raise ObservableCorrectionError(f"failed to read device GNSS CSV: {device_gnss}") from exc
    if not grouped:
        raise ObservableCorrectionError("device GNSS CSV contains no raw epochs")
    return grouped, input_rows


def _median_or_zero(values: Iterable[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return float(np.median(finite)) if finite else 0.0


def _mean_std_max_or_zero(values: Iterable[float]) -> tuple[float, float, float]:
    finite = np.asarray([value for value in values if math.isfinite(value)], dtype=float)
    if finite.size == 0:
        return 0.0, 0.0, 0.0
    return float(np.mean(finite)), float(np.std(finite)), float(np.max(finite))


def _geometry_features(rows: list[dict[str, str]], line: int) -> tuple[float, float, float]:
    points: list[tuple[float, float, float]] = []
    for row in rows:
        values = tuple(
            _parse_optional_float(row.get(field), field, line)
            for field in (
                "SvPositionXEcefMeters",
                "SvPositionYEcefMeters",
                "SvPositionZEcefMeters",
            )
        )
        if all(value is not None for value in values):
            points.append(tuple(float(value) for value in values))  # type: ignore[arg-type]
    if len(points) < 2:
        return 0.0, 0.0, 0.0
    matrix = np.asarray(points, dtype=float)
    centered = matrix - np.mean(matrix, axis=0)
    covariance = (centered.T @ centered) / float(len(points))
    try:
        eigenvalues = np.linalg.eigvalsh(covariance)
    except np.linalg.LinAlgError as exc:
        raise ObservableCorrectionError(f"row {line}: satellite geometry is not solvable") from exc
    if not np.isfinite(eigenvalues).all():
        raise ObservableCorrectionError(f"row {line}: satellite geometry is non-finite")
    eigenvalues = np.maximum(eigenvalues, 0.0)
    return float(np.sum(eigenvalues)), float(eigenvalues[0]), float(eigenvalues[-1])


def _signal_counts(rows: list[dict[str, str]], line: int) -> tuple[int, int]:
    gps_l1 = 0
    galileo_e1 = 0
    for row in rows:
        signal = (row.get("SignalType") or "").strip().upper()
        constellation = (row.get("ConstellationType") or "").strip()
        if signal == "GPS_L1_CA" or ("GPS" in signal and "L1" in signal):
            gps_l1 += 1
        elif constellation == "1" and signal:
            gps_l1 += 1
        if signal == "GALILEO_E1_C" or ("GALILEO" in signal and "E1" in signal):
            galileo_e1 += 1
        elif constellation == "6" and signal:
            galileo_e1 += 1
    return gps_l1, galileo_e1


def extract_features(
    device_gnss: Path,
    *,
    skip_epochs: int = DEFAULT_SKIP_EPOCHS,
    consistency_tolerance_m: float = wls.DEFAULT_CONSISTENCY_TOLERANCE_M,
    leap_seconds: int = DEFAULT_LEAP_SECONDS,
) -> FeatureExtraction:
    """Extract fixed observable features without reading truth."""

    if skip_epochs < 0:
        raise ObservableCorrectionError("skip_epochs must be non-negative")
    if not math.isfinite(consistency_tolerance_m) or consistency_tolerance_m < 0.0:
        raise ObservableCorrectionError("consistency tolerance must be finite and non-negative")
    if leap_seconds < 0:
        raise ObservableCorrectionError("leap_seconds must be non-negative")
    grouped, input_rows = _raw_epoch_rows(device_gnss)
    extraction = wls.extract_epochs(
        device_gnss,
        consistency_tolerance_m=consistency_tolerance_m,
    )
    selected = wls.select_epochs(extraction, skip_epochs, -1)
    selected_timestamps = [epoch.timestamp_ms for epoch in selected]
    if not selected_timestamps:
        raise ObservableCorrectionError("skip_epochs remove every WLS epoch")
    by_timestamp = {epoch.timestamp_ms: epoch for epoch in selected}
    all_timestamps = list(extraction.epoch_timestamps)
    previous_ecef: np.ndarray | None = None
    previous_timestamp: int | None = None
    rows: list[ObservableFeatureRow] = []
    for timestamp in selected_timestamps:
        raw_rows = grouped.get(timestamp)
        if not raw_rows:
            raise ObservableCorrectionError(f"WLS epoch {timestamp} has no raw feature rows")
        epoch = by_timestamp[timestamp]
        ecef = np.asarray(epoch.ecef, dtype=float)
        if ecef.shape != (3,) or not np.isfinite(ecef).all():
            raise ObservableCorrectionError(f"epoch {timestamp}: WLS ECEF is non-finite")
        line = epoch.first_source_row
        clock_counts = [
            _parse_int(row.get("HardwareClockDiscontinuityCount"), "HardwareClockDiscontinuityCount", line)
            for row in raw_rows
        ]
        if len(set(clock_counts)) != 1:
            raise ObservableCorrectionError(f"epoch {timestamp}: clock count is inconsistent")
        cn0 = [
            value
            for row in raw_rows
            if (value := _parse_optional_float(row.get("Cn0DbHz"), "Cn0DbHz", line)) is not None
        ]
        rate_uncertainty = [
            value
            for row in raw_rows
            if (
                value := _parse_optional_float(
                    row.get("PseudorangeRateUncertaintyMetersPerSecond"),
                    "PseudorangeRateUncertaintyMetersPerSecond",
                    line,
                )
            )
            is not None
        ]
        adr_valid = 0
        for row in raw_rows:
            state = _parse_int(row.get("AccumulatedDeltaRangeState"), "AccumulatedDeltaRangeState", line)
            if state != 0:
                adr_valid += 1
        gps_l1, galileo_e1 = _signal_counts(raw_rows, line)
        cn0_mean, cn0_std, cn0_max = _mean_std_max_or_zero(cn0)
        geometry_trace, geometry_min, geometry_max = _geometry_features(raw_rows, line)
        if previous_ecef is None or previous_timestamp is None:
            delta = np.zeros(3, dtype=float)
            dt = 0.0
        else:
            dt_ms = timestamp - previous_timestamp
            if dt_ms <= 0:
                raise ObservableCorrectionError(f"epoch {timestamp}: timestamps are not increasing")
            delta = ecef - previous_ecef
            dt = dt_ms / 1000.0
        day_ms = 86_400_000.0
        phase = 2.0 * math.pi * ((timestamp % int(day_ms)) / day_ms)
        feature_values = np.asarray(
            [
                float(ecef[0]),
                float(ecef[1]),
                float(ecef[2]),
                float(delta[0]),
                float(delta[1]),
                float(delta[2]),
                dt,
                float(len(raw_rows)),
                float(gps_l1),
                float(galileo_e1),
                cn0_mean,
                _median_or_zero(cn0),
                cn0_std,
                cn0_max,
                _median_or_zero(rate_uncertainty),
                float(adr_valid),
                geometry_trace,
                geometry_min,
                geometry_max,
                float(clock_counts[0]),
                math.sin(phase),
                math.cos(phase),
            ],
            dtype=float,
        )
        if feature_values.shape != (len(FEATURE_NAMES),) or not np.isfinite(feature_values).all():
            raise ObservableCorrectionError(f"epoch {timestamp}: observable feature is non-finite")
        rows.append(
            ObservableFeatureRow(
                timestamp_ms=timestamp,
                ecef=ecef,
                features=feature_values,
                source_line=line,
                raw_row_count=len(raw_rows),
                satellite_count=epoch.svid_count,
            )
        )
        previous_ecef = ecef
        previous_timestamp = timestamp
    if len(all_timestamps) > 1:
        gaps = [
            (right - left) / 1000.0
            for left, right in zip(all_timestamps, all_timestamps[1:])
        ]
        timestamp_gap_count = sum(gap * 1000.0 > wls.TIMESTAMP_GAP_THRESHOLD_MS for gap in gaps)
        max_gap = max(gaps)
    else:
        timestamp_gap_count = 0
        max_gap = 0.0
    return FeatureExtraction(
        rows=tuple(rows),
        input_rows=input_rows,
        input_epochs=len(all_timestamps),
        selected_epochs=len(rows),
        skipped_epochs=skip_epochs,
        timestamp_gap_count=timestamp_gap_count,
        max_timestamp_gap_s=float(max_gap),
        device_sha256=_sha256(device_gnss),
    )


def _feature_matrix(rows: Iterable[ObservableFeatureRow]) -> np.ndarray:
    values = np.asarray([row.features for row in rows], dtype=float)
    if values.ndim != 2 or values.shape[1] != len(FEATURE_NAMES) or not np.isfinite(values).all():
        raise ObservableCorrectionError("feature matrix has invalid shape or non-finite value")
    return values


def fit_ridge_model(
    features: np.ndarray,
    targets: np.ndarray,
    *,
    alpha: float = MODEL_ALPHA,
) -> dict[str, Any]:
    """Fit the frozen standardized multi-output ridge model.

    This function is called only by the truth-scored evaluator.  The
    inference CLI never exposes a target/training argument.
    """

    x = np.asarray(features, dtype=float)
    y = np.asarray(targets, dtype=float)
    if x.ndim != 2 or x.shape[1] != len(FEATURE_NAMES):
        raise ObservableCorrectionError("training features have invalid shape")
    if y.ndim != 2 or y.shape != (x.shape[0], 3):
        raise ObservableCorrectionError("training targets must be an Nx3 matrix")
    if x.shape[0] == 0 or not np.isfinite(x).all() or not np.isfinite(y).all():
        raise ObservableCorrectionError("training data is empty or non-finite")
    if not math.isfinite(alpha) or alpha <= 0.0:
        raise ObservableCorrectionError("ridge alpha must be finite and positive")
    mean = np.mean(x, axis=0)
    scale = np.std(x, axis=0)
    scale = np.where(np.isfinite(scale) & (scale > 1.0e-12), scale, 1.0)
    standardized = (x - mean) / scale
    target_mean = np.mean(y, axis=0)
    centered_target = y - target_mean
    normal = standardized.T @ standardized
    normal.flat[:: normal.shape[0] + 1] += alpha
    try:
        coefficients = np.linalg.solve(normal, standardized.T @ centered_target).T
    except np.linalg.LinAlgError as exc:
        raise ObservableCorrectionError("ridge normal equation is not solvable") from exc
    if not np.isfinite(coefficients).all() or not np.isfinite(target_mean).all():
        raise ObservableCorrectionError("ridge model is non-finite")
    return {
        "schema_version": MODEL_SCHEMA_VERSION,
        "model_id": MODEL_ID,
        "alpha": float(alpha),
        "feature_names": list(FEATURE_NAMES),
        "feature_mean": [float(value) for value in mean],
        "feature_scale": [float(value) for value in scale],
        "coefficient_ecef_residual_m": [
            [float(value) for value in row] for row in coefficients
        ],
        "intercept_ecef_residual_m": [float(value) for value in target_mean],
        "fit_rows": int(x.shape[0]),
    }


def _validate_model(model: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if not isinstance(model, dict) or model.get("schema_version") != MODEL_SCHEMA_VERSION:
        raise ObservableCorrectionError("observable correction model schema is invalid")
    if model.get("model_id") != MODEL_ID:
        raise ObservableCorrectionError("observable correction model id is invalid")
    if model.get("feature_names") != list(FEATURE_NAMES):
        raise ObservableCorrectionError("observable correction feature order differs")
    alpha = float(model.get("alpha", 0.0))
    if not math.isfinite(alpha) or alpha != MODEL_ALPHA:
        raise ObservableCorrectionError("observable correction alpha differs from frozen value")
    mean = np.asarray(model.get("feature_mean"), dtype=float)
    scale = np.asarray(model.get("feature_scale"), dtype=float)
    coefficients = np.asarray(model.get("coefficient_ecef_residual_m"), dtype=float)
    intercept = np.asarray(model.get("intercept_ecef_residual_m"), dtype=float)
    if (
        mean.shape != (len(FEATURE_NAMES),)
        or scale.shape != (len(FEATURE_NAMES),)
        or coefficients.shape != (3, len(FEATURE_NAMES))
        or intercept.shape != (3,)
        or not all(np.isfinite(value).all() for value in (mean, scale, coefficients, intercept))
        or np.any(scale <= 0.0)
    ):
        raise ObservableCorrectionError("observable correction model arrays are invalid")
    return mean, scale, coefficients, intercept


def apply_model(
    rows: Iterable[ObservableFeatureRow], model: dict[str, Any]
) -> tuple[list[np.ndarray], dict[str, int]]:
    """Apply a sealed model, failing closed to the base ECEF per epoch."""

    mean, scale, coefficients, intercept = _validate_model(model)
    row_list = list(rows)
    matrix = _feature_matrix(row_list)
    corrected: list[np.ndarray] = []
    fallback_count = 0
    for row, vector in zip(row_list, matrix):
        candidate = row.ecef + intercept + coefficients @ ((vector - mean) / scale)
        if not np.isfinite(candidate).all():
            candidate = row.ecef.copy()
            fallback_count += 1
        else:
            norm = float(np.linalg.norm(candidate))
            if not math.isfinite(norm) or not wls.ECEF_NORM_MIN_M <= norm <= wls.ECEF_NORM_MAX_M:
                candidate = row.ecef.copy()
                fallback_count += 1
        corrected.append(np.asarray(candidate, dtype=float))
    return corrected, {"model_rows": len(row_list), "base_fallback_rows": fallback_count}


def write_feature_artifact(
    extraction: FeatureExtraction,
    path: Path,
    *,
    dataset_id: str,
    device_gnss: Path | None = None,
) -> dict[str, Any]:
    """Publish a deterministic feature CSV and manifest payload."""

    lines = ["UnixTimeMillis,BaseWlsXEcefMeters,BaseWlsYEcefMeters,BaseWlsZEcefMeters," + ",".join(FEATURE_NAMES)]
    for row in extraction.rows:
        values = [
            str(row.timestamp_ms),
            *(f"{float(value):.9f}" for value in row.ecef),
            *(f"{float(value):.12g}" for value in row.features),
        ]
        lines.append(",".join(values))
    _atomic_write(path, ("\n".join(lines) + "\n").encode("ascii"))
    return {
        "schema_version": FEATURE_MANIFEST_SCHEMA_VERSION,
        "truth_free": True,
        "dataset_id": dataset_id,
        "input": {
            "path": str(device_gnss) if device_gnss is not None else "device_gnss.csv",
            "sha256": extraction.device_sha256,
        },
        "features": list(FEATURE_NAMES),
        "input_rows": extraction.input_rows,
        "input_epochs": extraction.input_epochs,
        "selected_epochs": extraction.selected_epochs,
        "skipped_epochs": extraction.skipped_epochs,
        "timestamp_gap_count": extraction.timestamp_gap_count,
        "max_timestamp_gap_s": extraction.max_timestamp_gap_s,
        "artifact": {"path": str(path), "sha256": _sha256(path), "bytes": path.stat().st_size},
    }


def write_correction_outputs(
    extraction: FeatureExtraction,
    corrected_ecef: Iterable[np.ndarray],
    output_dir: Path,
    *,
    device_gnss: Path,
    dataset_id: str,
    model: dict[str, Any],
    fallback_counts: dict[str, int],
    profile: Path | None = None,
) -> dict[str, Any]:
    """Atomically publish corrected POS, trajectory, summary, and manifest."""

    corrected = [np.asarray(value, dtype=float) for value in corrected_ecef]
    if len(corrected) != len(extraction.rows):
        raise ObservableCorrectionError("corrected row count differs from feature rows")
    if any(value.shape != (3,) or not np.isfinite(value).all() for value in corrected):
        raise ObservableCorrectionError("corrected output contains non-finite ECEF")
    if len({row.timestamp_ms for row in extraction.rows}) != len(extraction.rows):
        raise ObservableCorrectionError("corrected output contains duplicate timestamps")
    selected_epochs = tuple(
        wls.WlsEpoch(
            timestamp_ms=row.timestamp_ms,
            clock_discontinuity_count=0,
            ecef=tuple(float(value) for value in corrected[index]),
            rows=row.raw_row_count,
            finite_rows=row.raw_row_count,
            svid_count=row.satellite_count,
            first_source_row=row.source_line,
            last_source_row=row.source_line + row.raw_row_count - 1,
        )
        for index, row in enumerate(extraction.rows)
    )
    positions = wls.epochs_to_positions(selected_epochs, DEFAULT_LEAP_SECONDS)
    final_paths = {
        "position": output_dir / "corrected.pos",
        "trajectory": output_dir / "trajectory.csv",
        "summary": output_dir / "correction_summary.json",
        "manifest": output_dir / "correction_manifest.json",
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".observable-correction-", dir=str(output_dir)) as temp_name:
        temporary_dir = Path(temp_name)
        temporary_paths = {key: temporary_dir / path.name for key, path in final_paths.items()}
        wls._write_pos(temporary_paths["position"], positions)
        trajectory_lines = [
            "UnixTimeMillis,X_ECEF_m,Y_ECEF_m,Z_ECEF_m,LatitudeDegrees,LongitudeDegrees,AltitudeMeters,source"
        ]
        for position in positions:
            trajectory_lines.append(
                f"{position.timestamp_ms},{position.ecef[0]:.9f},{position.ecef[1]:.9f},{position.ecef[2]:.9f},"
                f"{position.latitude:.12f},{position.longitude:.12f},{position.height:.6f},observable_ridge_residual"
            )
        _atomic_write(temporary_paths["trajectory"], ("\n".join(trajectory_lines) + "\n").encode("ascii"))
        artifacts = {
            name: {"path": str(final_paths[name]), "sha256": _sha256(temporary_paths[name]), "bytes": temporary_paths[name].stat().st_size}
            for name in ("position", "trajectory")
        }
        summary = {
            "schema_version": SCHEMA_VERSION,
            "truth_free": True,
            "dataset_id": dataset_id,
            "base_lane": "android-handset-wls-ecef",
            "candidate": {"model_id": MODEL_ID, "alpha": MODEL_ALPHA, "feature_count": len(FEATURE_NAMES)},
            "input": {"device_gnss": {"path": str(device_gnss), "sha256": _sha256(device_gnss)}, "ground_truth": None},
            "selection": {"skip_epochs": extraction.skipped_epochs, "selected_epochs": extraction.selected_epochs},
            "fallback_counts": dict(fallback_counts),
            "artifacts": artifacts,
        }
        _atomic_json(temporary_paths["summary"], summary)
        artifacts["summary"] = {"path": str(final_paths["summary"]), "sha256": _sha256(temporary_paths["summary"]), "bytes": temporary_paths["summary"].stat().st_size}
        manifest = {
            "schema_version": SCHEMA_VERSION + "-manifest",
            "truth_free": True,
            "truth_used": False,
            "dataset_id": dataset_id,
            "model": {"schema_version": model.get("schema_version"), "model_id": model.get("model_id"), "alpha": model.get("alpha"), "feature_names": model.get("feature_names")},
            "inputs": {
                "device_gnss": {"path": str(device_gnss), "sha256": _sha256(device_gnss)},
                "ground_truth": None,
                "profile": {"path": str(profile), "sha256": _sha256(profile)} if profile is not None else None,
            },
            "contract": {
                "same_base_wls_epoch_keys": True,
                "no_interpolation": True,
                "nonfinite_policy": "per-epoch fallback to base WLS",
                "feature_names": list(FEATURE_NAMES),
                "raw_feature_fields": list(RAW_FEATURE_FIELDS),
            },
            "fallback_counts": dict(fallback_counts),
            "artifacts": artifacts,
        }
        _atomic_json(temporary_paths["manifest"], manifest)
        artifacts["manifest"] = {"path": str(final_paths["manifest"]), "sha256": _sha256(temporary_paths["manifest"]), "bytes": temporary_paths["manifest"].stat().st_size}
        for key, target in final_paths.items():
            os.replace(temporary_paths[key], target)
    return manifest


def _load_model(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ObservableCorrectionError(f"invalid model JSON: {path}") from exc
    _validate_model(payload)
    return payload


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-observable-error-correction")
    )
    parser.add_argument("--device-gnss", type=Path, required=True)
    parser.add_argument("--model-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--dataset-id", required=True)
    parser.add_argument("--skip-epochs", type=int, default=DEFAULT_SKIP_EPOCHS)
    parser.add_argument("--profile", type=Path)
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        model = _load_model(args.model_json)
        extraction = extract_features(args.device_gnss, skip_epochs=args.skip_epochs)
        feature_path = args.output_dir / "observable_features.csv"
        feature_manifest = write_feature_artifact(extraction, feature_path, dataset_id=args.dataset_id)
        corrected, fallback_counts = apply_model(extraction.rows, model)
        manifest = write_correction_outputs(
            extraction,
            corrected,
            args.output_dir,
            device_gnss=args.device_gnss,
            dataset_id=args.dataset_id,
            model=model,
            fallback_counts=fallback_counts,
            profile=args.profile,
        )
        manifest["feature_artifact"] = feature_manifest
        _atomic_json(args.output_dir / "correction_manifest.json", manifest)
    except (ObservableCorrectionError, ValueError) as exc:
        print(f"Observable correction lane failed: {exc}", file=sys.stderr)
        return 2
    print(f"Observable correction lane complete: {args.output_dir / 'corrected.pos'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
