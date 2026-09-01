#!/usr/bin/env python3
"""Truth-free Doppler-assisted postprocess for Android handset WLS.

The Android log exposes satellite ECEF position/velocity and pseudorange rate.
This module estimates receiver velocity from those observables and uses it only
for a bounded one-step position prediction/update around the handset WLS fix.
It is deliberately an experimental lane: malformed observations, clock
transitions, long gaps, rank deficiency, excessive speed, or a large position
innovation fall back to the exact WLS coordinate and reset the prediction
chain.  No truth or leaderboard data is accepted by this command.
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
import time
from typing import Any, Iterable

import numpy as np

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_kaggle as kaggle  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402


SCHEMA_VERSION = "smartphone-doppler-position.v1"
MANIFEST_SCHEMA_VERSION = "smartphone-doppler-position-manifest.v1"
DEFAULT_SKIP_EPOCHS = 1
DEFAULT_MIN_SATELLITES = 4
DEFAULT_ROBUST_PASSES = 2
DEFAULT_ROBUST_RESIDUAL_GATE_MPS = 3.0
DEFAULT_MAX_FIT_RMS_MPS = 3.0
DEFAULT_MAX_RECEIVER_SPEED_MPS = 70.0
DEFAULT_MAX_DT_S = 1.5
DEFAULT_INNOVATION_GATE_M = 30.0
DEFAULT_POSITION_MEASUREMENT_FLOOR_M = 1.0
DEFAULT_VELOCITY_PROCESS_FLOOR_MPS = 1.0
DEFAULT_UNCERTAINTY_FLOOR_MPS = 0.10
REQUIRED_FIELDS = (
    "MessageType",
    "utcTimeMillis",
    "HardwareClockDiscontinuityCount",
    "ConstellationType",
    "Svid",
    "Cn0DbHz",
    "PseudorangeRateMetersPerSecond",
    "PseudorangeRateUncertaintyMetersPerSecond",
    "SvPositionXEcefMeters",
    "SvPositionYEcefMeters",
    "SvPositionZEcefMeters",
    "SvVelocityXEcefMetersPerSecond",
    "SvVelocityYEcefMetersPerSecond",
    "SvVelocityZEcefMetersPerSecond",
)
_INTEGER_RE = re.compile(r"^[+-]?\d+$")


class DopplerPositionError(ValueError):
    """Raised when an input or output contract cannot be proven."""


@dataclass(frozen=True)
class DopplerConfig:
    min_satellites: int = DEFAULT_MIN_SATELLITES
    robust_passes: int = DEFAULT_ROBUST_PASSES
    robust_residual_gate_mps: float = DEFAULT_ROBUST_RESIDUAL_GATE_MPS
    max_fit_rms_mps: float = DEFAULT_MAX_FIT_RMS_MPS
    max_receiver_speed_mps: float = DEFAULT_MAX_RECEIVER_SPEED_MPS
    max_dt_s: float = DEFAULT_MAX_DT_S
    innovation_gate_m: float = DEFAULT_INNOVATION_GATE_M
    position_measurement_floor_m: float = DEFAULT_POSITION_MEASUREMENT_FLOOR_M
    velocity_process_floor_mps: float = DEFAULT_VELOCITY_PROCESS_FLOOR_MPS
    uncertainty_floor_mps: float = DEFAULT_UNCERTAINTY_FLOOR_MPS

    def validate(self) -> None:
        if not isinstance(self.min_satellites, int) or self.min_satellites < 4:
            raise DopplerPositionError("min_satellites must be an integer >= 4")
        if not isinstance(self.robust_passes, int) or not 1 <= self.robust_passes <= 4:
            raise DopplerPositionError("robust_passes must be in [1, 4]")
        values = {
            "robust_residual_gate_mps": self.robust_residual_gate_mps,
            "max_fit_rms_mps": self.max_fit_rms_mps,
            "max_receiver_speed_mps": self.max_receiver_speed_mps,
            "max_dt_s": self.max_dt_s,
            "innovation_gate_m": self.innovation_gate_m,
            "position_measurement_floor_m": self.position_measurement_floor_m,
            "velocity_process_floor_mps": self.velocity_process_floor_mps,
            "uncertainty_floor_mps": self.uncertainty_floor_mps,
        }
        for name, value in values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise DopplerPositionError(f"{name} must be finite and positive")

    def as_dict(self) -> dict[str, Any]:
        return {
            "min_satellites": self.min_satellites,
            "robust_passes": self.robust_passes,
            "robust_residual_gate_mps": self.robust_residual_gate_mps,
            "max_fit_rms_mps": self.max_fit_rms_mps,
            "max_receiver_speed_mps": self.max_receiver_speed_mps,
            "max_dt_s": self.max_dt_s,
            "innovation_gate_m": self.innovation_gate_m,
            "position_measurement_floor_m": self.position_measurement_floor_m,
            "velocity_process_floor_mps": self.velocity_process_floor_mps,
            "uncertainty_floor_mps": self.uncertainty_floor_mps,
        }


@dataclass(frozen=True)
class _Observation:
    key: tuple[str, str]
    cn0: float
    rate_mps: float
    uncertainty_mps: float
    satellite_ecef: np.ndarray
    satellite_velocity_ecef_mps: np.ndarray


@dataclass(frozen=True)
class DopplerEstimate:
    timestamp_ms: int
    clock_discontinuity_count: int | None
    velocity_ecef_mps: np.ndarray | None
    clock_range_rate_mps: float | None
    covariance_ecef: np.ndarray | None
    rms_mps: float | None
    satellite_count: int
    inlier_count: int
    valid: bool
    reason: str | None

    def as_dict(self) -> dict[str, Any]:
        return {
            "timestamp_ms": self.timestamp_ms,
            "clock_discontinuity_count": self.clock_discontinuity_count,
            "velocity_ecef_mps": (
                [float(value) for value in self.velocity_ecef_mps]
                if self.velocity_ecef_mps is not None
                else None
            ),
            "clock_range_rate_mps": self.clock_range_rate_mps,
            "rms_mps": self.rms_mps,
            "satellite_count": self.satellite_count,
            "inlier_count": self.inlier_count,
            "valid": self.valid,
            "reason": self.reason,
        }


@dataclass(frozen=True)
class DopplerPositionResult:
    rows: tuple[smoother.PositionRow, ...]
    estimates: tuple[DopplerEstimate, ...]
    sources: tuple[str, ...]
    source_counts: dict[str, int]
    reason_counts: dict[str, int]
    invalid_observation_rows: int
    duplicate_observation_rows: int
    clock_transition_count: int
    maximum_innovation_m: float
    maximum_speed_mps: float
    wall_time_s: float


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise DopplerPositionError(f"missing file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_write(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor: int | None = None
    temporary: str | None = None
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
        raise DopplerPositionError(f"atomic publish failed for {path}: {exc}") from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)
        if temporary is not None:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def _json_bytes(payload: dict[str, Any]) -> bytes:
    return (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")


def _parse_int(raw: str | None, field: str, line: int) -> int:
    token = (raw or "").strip()
    if not _INTEGER_RE.fullmatch(token):
        raise DopplerPositionError(
            f"row {line}: {field} must be an integer"
        )
    try:
        return int(token)
    except ValueError as exc:
        raise DopplerPositionError(f"row {line}: invalid {field}") from exc


def _parse_float(raw: str | None, field: str, line: int) -> float:
    try:
        value = float((raw or "").strip())
    except ValueError as exc:
        raise DopplerPositionError(f"row {line}: {field} must be finite") from exc
    if not math.isfinite(value):
        raise DopplerPositionError(f"row {line}: {field} must be finite")
    return value


def _raw_observations(
    path: Path,
) -> tuple[dict[int, tuple[int | None, tuple[_Observation, ...]]], int, int, int]:
    """Read grouped raw rows, skipping only rows unusable for Doppler."""

    if not path.is_file():
        raise DopplerPositionError(f"missing device GNSS CSV: {path}")
    grouped: dict[int, dict[tuple[str, str], tuple[float, _Observation]]] = {}
    clocks: dict[int, int | None] = {}
    invalid_rows = 0
    duplicate_rows = 0
    input_rows = 0
    previous_timestamp: int | None = None
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            fields = list(reader.fieldnames or ())
            missing = [field for field in REQUIRED_FIELDS if field not in fields]
            if missing:
                raise DopplerPositionError(
                    "device GNSS CSV missing fields: " + ", ".join(missing)
                )
            if len(fields) != len(set(fields)):
                raise DopplerPositionError("device GNSS CSV has duplicate fields")
            for line, raw in enumerate(reader, start=2):
                input_rows += 1
                if None in raw:
                    raise DopplerPositionError(
                        f"row {line}: more values than CSV header fields"
                    )
                if raw.get("MessageType") != "Raw":
                    raise DopplerPositionError(f"row {line}: MessageType must be Raw")
                timestamp = _parse_int(raw.get("utcTimeMillis"), "utcTimeMillis", line)
                if timestamp < 0:
                    raise DopplerPositionError(f"row {line}: timestamp must be non-negative")
                if previous_timestamp is not None and timestamp < previous_timestamp:
                    raise DopplerPositionError(
                        f"row {line}: utcTimeMillis moved backwards"
                    )
                previous_timestamp = timestamp
                clock = _parse_int(
                    raw.get("HardwareClockDiscontinuityCount"),
                    "HardwareClockDiscontinuityCount",
                    line,
                )
                if clock < 0:
                    raise DopplerPositionError(f"row {line}: clock count must be non-negative")
                if timestamp in clocks and clocks[timestamp] != clock:
                    raise DopplerPositionError(
                        f"epoch {timestamp}: inconsistent hardware clock count"
                    )
                clocks[timestamp] = clock
                if timestamp not in grouped:
                    grouped[timestamp] = {}
                try:
                    constellation = (raw.get("ConstellationType") or "").strip()
                    svid = (raw.get("Svid") or "").strip()
                    if not constellation or not _INTEGER_RE.fullmatch(svid):
                        raise ValueError("invalid satellite key")
                    cn0 = _parse_float(raw.get("Cn0DbHz"), "Cn0DbHz", line)
                    rate = _parse_float(
                        raw.get("PseudorangeRateMetersPerSecond"),
                        "PseudorangeRateMetersPerSecond",
                        line,
                    )
                    uncertainty = _parse_float(
                        raw.get("PseudorangeRateUncertaintyMetersPerSecond"),
                        "PseudorangeRateUncertaintyMetersPerSecond",
                        line,
                    )
                    if uncertainty <= 0.0:
                        raise ValueError("pseudorange-rate uncertainty must be positive")
                    sat = np.array(
                        [
                            _parse_float(raw.get(field), field, line)
                            for field in (
                                "SvPositionXEcefMeters",
                                "SvPositionYEcefMeters",
                                "SvPositionZEcefMeters",
                            )
                        ],
                        dtype=float,
                    )
                    sat_velocity = np.array(
                        [
                            _parse_float(raw.get(field), field, line)
                            for field in (
                                "SvVelocityXEcefMetersPerSecond",
                                "SvVelocityYEcefMetersPerSecond",
                                "SvVelocityZEcefMetersPerSecond",
                            )
                        ],
                        dtype=float,
                    )
                    if not np.isfinite(np.r_[sat, sat_velocity]).all():
                        raise ValueError("non-finite satellite state")
                    observation = _Observation(
                        (constellation, svid),
                        cn0,
                        rate,
                        uncertainty,
                        sat,
                        sat_velocity,
                    )
                    previous = grouped[timestamp].get(observation.key)
                    if previous is not None:
                        duplicate_rows += 1
                    if previous is None or cn0 > previous[0]:
                        grouped[timestamp][observation.key] = (cn0, observation)
                except (DopplerPositionError, ValueError, OverflowError):
                    invalid_rows += 1
    except OSError as exc:
        raise DopplerPositionError(f"failed to read device GNSS CSV: {path}") from exc
    output = {
        timestamp: (clocks.get(timestamp), tuple(item[1] for item in sorted(rows.values(), key=lambda item: item[1].key)))
        for timestamp, rows in grouped.items()
    }
    return output, input_rows, invalid_rows, duplicate_rows


def _solve_weighted(
    matrix: np.ndarray,
    values: np.ndarray,
    uncertainties: np.ndarray,
    mask: np.ndarray,
    config: DopplerConfig,
) -> tuple[np.ndarray, np.ndarray, float, int] | None:
    if int(mask.sum()) < config.min_satellites:
        return None
    selected_matrix = matrix[mask]
    selected_values = values[mask]
    selected_uncertainties = np.maximum(
        uncertainties[mask], config.uncertainty_floor_mps
    )
    weights = 1.0 / np.square(selected_uncertainties)
    if not np.isfinite(weights).all():
        return None
    weighted_matrix = selected_matrix * np.sqrt(weights)[:, None]
    weighted_values = selected_values * np.sqrt(weights)
    try:
        solution, _, rank, _ = np.linalg.lstsq(
            weighted_matrix, weighted_values, rcond=None
        )
    except np.linalg.LinAlgError:
        return None
    if rank < 4 or solution.shape != (4,) or not np.isfinite(solution).all():
        return None
    residuals = selected_matrix @ solution - selected_values
    rms = float(np.sqrt(np.mean(np.square(residuals))))
    if not math.isfinite(rms):
        return None
    normal = weighted_matrix.T @ weighted_matrix
    try:
        normal_inverse = np.linalg.pinv(normal, rcond=1.0e-12)
    except np.linalg.LinAlgError:
        return None
    degrees = max(int(mask.sum()) - 4, 1)
    weighted_sse = float(np.sum(np.square(residuals) * weights))
    residual_variance = max(weighted_sse / degrees, config.velocity_process_floor_mps**2)
    covariance = residual_variance * normal_inverse
    if covariance.shape != (4, 4) or not np.isfinite(covariance).all():
        return None
    return solution, covariance, rms, int(mask.sum())


def _recover_single_outlier_consensus(
    matrix: np.ndarray,
    values: np.ndarray,
    uncertainties: np.ndarray,
    config: DopplerConfig,
) -> np.ndarray | None:
    """Find a fixed-gate consensus when one gross row contaminated the fit."""

    if len(matrix) <= config.min_satellites:
        return None
    best: np.ndarray | None = None
    best_count = config.min_satellites - 1
    best_speed = math.inf
    for excluded in range(len(matrix)):
        trial = np.ones(len(matrix), dtype=bool)
        trial[excluded] = False
        solved = _solve_weighted(matrix, values, uncertainties, trial, config)
        if solved is None:
            continue
        residuals = matrix @ solved[0] - values
        consensus = np.abs(residuals) <= config.robust_residual_gate_mps
        count = int(consensus.sum())
        speed = float(np.linalg.norm(solved[0][:3]))
        if count >= config.min_satellites and (
            count > best_count
            or (count == best_count and speed < best_speed)
        ):
            best = consensus
            best_count = count
            best_speed = speed
    return best


def estimate_doppler(
    timestamp_ms: int,
    base_ecef: np.ndarray,
    clock_count: int | None,
    observations: Iterable[_Observation],
    config: DopplerConfig,
) -> DopplerEstimate:
    """Estimate receiver velocity using a fixed robust weighted LS contract."""

    obs = tuple(observations)
    if len(obs) < config.min_satellites:
        return DopplerEstimate(
            timestamp_ms, clock_count, None, None, None, None, len(obs), 0, False, "insufficient_satellites"
        )
    matrix_rows: list[np.ndarray] = []
    values: list[float] = []
    uncertainties: list[float] = []
    for observation in obs:
        line_of_sight = observation.satellite_ecef - base_ecef
        norm = float(np.linalg.norm(line_of_sight))
        if not math.isfinite(norm) or norm < 1.0e6:
            continue
        line_of_sight = line_of_sight / norm
        # d(rho)/dt = LOS dot (v_satellite - v_receiver) + clock range rate.
        matrix_rows.append(np.r_[-line_of_sight, 1.0])
        values.append(
            observation.rate_mps
            - float(line_of_sight @ observation.satellite_velocity_ecef_mps)
        )
        uncertainties.append(observation.uncertainty_mps)
    if len(matrix_rows) < config.min_satellites:
        return DopplerEstimate(
            timestamp_ms, clock_count, None, None, None, None, len(obs), 0, False, "invalid_satellite_geometry"
        )
    matrix = np.asarray(matrix_rows, dtype=float)
    values_array = np.asarray(values, dtype=float)
    uncertainty_array = np.asarray(uncertainties, dtype=float)
    if not np.isfinite(matrix).all() or not np.isfinite(values_array).all():
        return DopplerEstimate(
            timestamp_ms, clock_count, None, None, None, None, len(obs), 0, False, "nonfinite_observation"
        )
    mask = np.ones(len(matrix), dtype=bool)
    solved: tuple[np.ndarray, np.ndarray, float, int] | None = None
    for _ in range(config.robust_passes):
        solved = _solve_weighted(matrix, values_array, uncertainty_array, mask, config)
        if solved is None:
            return DopplerEstimate(
                timestamp_ms, clock_count, None, None, None, None, len(obs), int(mask.sum()), False, "rank_deficient"
            )
        residuals = matrix @ solved[0] - values_array
        new_mask = np.abs(residuals) <= config.robust_residual_gate_mps
        if int(new_mask.sum()) < config.min_satellites:
            recovered = _recover_single_outlier_consensus(
                matrix, values_array, uncertainty_array, config
            )
            if recovered is None:
                return DopplerEstimate(
                    timestamp_ms, clock_count, None, None, None, None, len(obs), int(new_mask.sum()), False, "robust_inlier_shortage"
                )
            new_mask = recovered
        if np.array_equal(new_mask, mask):
            break
        mask = new_mask
    solved = _solve_weighted(matrix, values_array, uncertainty_array, mask, config)
    if solved is None:
        return DopplerEstimate(
            timestamp_ms, clock_count, None, None, None, None, len(obs), int(mask.sum()), False, "rank_deficient"
        )
    solution, covariance, rms, inliers = solved
    velocity = solution[:3]
    speed = float(np.linalg.norm(velocity))
    if rms > config.max_fit_rms_mps:
        reason = "fit_rms_exceeded"
        valid = False
    elif not math.isfinite(speed) or speed > config.max_receiver_speed_mps:
        reason = "receiver_speed_exceeded"
        valid = False
    else:
        reason = None
        valid = True
    return DopplerEstimate(
        timestamp_ms,
        clock_count,
        velocity if valid else None,
        float(solution[3]) if valid else None,
        covariance[:3, :3] if valid else None,
        rms,
        len(obs),
        inliers,
        valid,
        reason,
    )


def _position_with_ecef(row: smoother.PositionRow, ecef: np.ndarray) -> smoother.PositionRow:
    if ecef.shape != (3,) or not np.isfinite(ecef).all():
        raise DopplerPositionError("candidate ECEF is non-finite")
    latitude_rad, longitude_rad, height = smoother._wgs84_ecef_to_geodetic(ecef)
    return smoother.PositionRow(
        row.week,
        row.tow,
        row.timestamp_ms,
        ecef.astype(float, copy=True),
        math.degrees(latitude_rad),
        math.degrees(longitude_rad),
        height,
        row.status,
        row.satellites,
        row.pdop,
        row.ratio,
        row.fixed_ambiguities,
        row.iterations,
        row.source_line,
    )


def build_candidate(
    positions: list[smoother.PositionRow],
    device_epochs: list[int],
    raw_epochs: dict[int, tuple[int | None, tuple[_Observation, ...]]],
    config: DopplerConfig,
) -> DopplerPositionResult:
    config.validate()
    if not positions or not device_epochs:
        raise DopplerPositionError("positions and device epochs must be non-empty")
    position_by_timestamp = {row.timestamp_ms: row for row in positions}
    unknown = sorted(set(position_by_timestamp) - set(device_epochs))
    if unknown:
        raise DopplerPositionError(
            f"position timestamp is not an exact device epoch: {unknown[0]}"
        )
    observations = [position_by_timestamp.get(timestamp) for timestamp in device_epochs]
    if any(row is None for row in observations):
        raise DopplerPositionError(
            "base WLS position is missing a selected device epoch"
        )
    started = time.perf_counter()
    estimates: list[DopplerEstimate] = []
    result_rows: list[smoother.PositionRow] = []
    sources: list[str] = []
    source_counts: dict[str, int] = {}
    reason_counts: dict[str, int] = {}
    clock_transition_count = 0
    maximum_innovation = 0.0
    maximum_speed = 0.0
    previous_clock: int | None = None
    previous_timestamp: int | None = None
    previous_ecef: np.ndarray | None = None

    def count(mapping: dict[str, int], key: str) -> None:
        mapping[key] = mapping.get(key, 0) + 1

    for index, timestamp in enumerate(device_epochs):
        base = observations[index]
        assert base is not None
        clock, obs = raw_epochs.get(timestamp, (None, tuple()))
        estimate = estimate_doppler(timestamp, base.ecef, clock, obs, config)
        estimates.append(estimate)
        if estimate.reason:
            count(reason_counts, estimate.reason)
        if estimate.valid and estimate.velocity_ecef_mps is not None:
            maximum_speed = max(
                maximum_speed, float(np.linalg.norm(estimate.velocity_ecef_mps))
            )
        reset = False
        if previous_timestamp is None or previous_ecef is None:
            candidate_ecef = base.ecef
            source = "wls_anchor"
            reset = True
        else:
            dt = (timestamp - previous_timestamp) / 1000.0
            if clock is not None and previous_clock is not None and clock != previous_clock:
                clock_transition_count += 1
                reset = True
                candidate_ecef = base.ecef
                source = "wls_clock_reset"
            elif not math.isfinite(dt) or dt <= 0.0 or dt > config.max_dt_s:
                reset = True
                candidate_ecef = base.ecef
                source = "wls_gap_reset"
            elif not estimate.valid or estimate.velocity_ecef_mps is None or estimate.covariance_ecef is None:
                reset = True
                candidate_ecef = base.ecef
                source = "wls_doppler_fallback"
            else:
                predicted = previous_ecef + estimate.velocity_ecef_mps * dt
                innovation = base.ecef - predicted
                innovation_norm = float(np.linalg.norm(innovation))
                maximum_innovation = max(maximum_innovation, innovation_norm)
                if not math.isfinite(innovation_norm):
                    reset = True
                    candidate_ecef = base.ecef
                    source = "wls_nonfinite_innovation"
                elif innovation_norm > config.innovation_gate_m:
                    reset = True
                    candidate_ecef = base.ecef
                    source = "wls_innovation_reset"
                else:
                    velocity_variance = float(
                        np.trace(estimate.covariance_ecef) / 3.0
                    )
                    if not math.isfinite(velocity_variance) or velocity_variance < 0.0:
                        reset = True
                        candidate_ecef = base.ecef
                        source = "wls_nonfinite_covariance"
                    else:
                        process_sigma = max(
                            config.velocity_process_floor_mps,
                            math.sqrt(velocity_variance),
                        ) * dt
                        measurement_sigma = smoother._measurement_sigma(
                            base, config.position_measurement_floor_m
                        )
                        predicted_variance = max(process_sigma * process_sigma, 1.0e-12)
                        measurement_variance = max(measurement_sigma**2, 1.0e-12)
                        gain = predicted_variance / (
                            predicted_variance + measurement_variance
                        )
                        if not math.isfinite(gain) or not 0.0 < gain <= 1.0:
                            reset = True
                            candidate_ecef = base.ecef
                            source = "wls_nonfinite_gain"
                        else:
                            candidate_ecef = predicted + gain * innovation
                            source = "doppler_position_update"
        result_row = _position_with_ecef(base, candidate_ecef)
        if not np.isfinite(result_row.ecef).all():
            raise DopplerPositionError(f"epoch {timestamp}: candidate is non-finite")
        result_rows.append(result_row)
        sources.append(source)
        count(source_counts, source)
        previous_ecef = result_row.ecef
        previous_timestamp = timestamp
        previous_clock = clock
        if reset and source != "wls_anchor":
            # The current finite WLS observation is the new prediction anchor.
            previous_ecef = base.ecef.astype(float, copy=True)
    return DopplerPositionResult(
        tuple(result_rows),
        tuple(estimates),
        tuple(sources),
        dict(sorted(source_counts.items())),
        dict(sorted(reason_counts.items())),
        0,
        0,
        clock_transition_count,
        maximum_innovation,
        maximum_speed,
        time.perf_counter() - started,
    )


def _write_pos(path: Path, rows: Iterable[smoother.PositionRow]) -> None:
    lines = [
        "% LibGNSS++ truth-free Doppler-assisted smartphone WLS position",
        "% Columns: GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) Status Satellites PDOP Ratio FixedAmbiguities Iterations",
        "% Candidate uses bounded pseudorange-rate receiver velocity prediction/update; no truth input.",
    ]
    for row in rows:
        lines.append(
            f"{row.week:d} {row.tow:.6f} "
            f"{row.ecef[0]:.6f} {row.ecef[1]:.6f} {row.ecef[2]:.6f} "
            f"{row.latitude:.9f} {row.longitude:.9f} {row.height:.6f} "
            f"{row.status:d} {row.satellites:d} {row.pdop:.6f} "
            f"{row.ratio:.6f} {row.fixed_ambiguities:d} {row.iterations:d}"
        )
    _atomic_write(path, ("\n".join(lines) + "\n").encode("ascii"))


def _write_trajectory_csv(path: Path, result: DopplerPositionResult) -> None:
    lines = [
        "UnixTimeMillis,LatitudeDegrees,LongitudeDegrees,AltitudeMeters,"
        "X_ECEF_m,Y_ECEF_m,Z_ECEF_m,source,doppler_valid,doppler_rms_mps,"
        "doppler_speed_mps,doppler_satellite_count,doppler_inlier_count,"
        "doppler_reason"
    ]
    for row, estimate, source in zip(result.rows, result.estimates, result.sources):
        speed = (
            float(np.linalg.norm(estimate.velocity_ecef_mps))
            if estimate.velocity_ecef_mps is not None
            else None
        )
        lines.append(
            f"{row.timestamp_ms},{row.latitude:.9f},{row.longitude:.9f},{row.height:.6f},"
            f"{row.ecef[0]:.6f},{row.ecef[1]:.6f},{row.ecef[2]:.6f},"
            f"{source},"
            f"{int(estimate.valid)},{'' if estimate.rms_mps is None else f'{estimate.rms_mps:.9f}'},"
            f"{'' if speed is None else f'{speed:.9f}'},{estimate.satellite_count},"
            f"{estimate.inlier_count},{estimate.reason or ''}"
        )
    _atomic_write(path, ("\n".join(lines) + "\n").encode("utf-8"))


def _summary(result: DopplerPositionResult, config: DopplerConfig) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "configuration": config.as_dict(),
        "epochs": len(result.rows),
        "source_counts": result.source_counts,
        "reason_counts": result.reason_counts,
        "invalid_observation_rows": result.invalid_observation_rows,
        "duplicate_observation_rows": result.duplicate_observation_rows,
        "clock_transition_count": result.clock_transition_count,
        "maximum_innovation_m": result.maximum_innovation_m,
        "maximum_speed_mps": result.maximum_speed_mps,
        "wall_time_s": result.wall_time_s,
        "truth_used": False,
    }


def write_outputs(
    result: DopplerPositionResult,
    config: DopplerConfig,
    output_dir: Path,
    *,
    position_path: Path,
    device_gnss_path: Path,
    skip_epochs: int,
    phone: str | None = None,
    dataset_id: str | None = None,
    submission_output: Path | None = None,
) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    position_output = output_dir / "doppler.pos"
    trajectory_output = output_dir / "doppler_trajectory.csv"
    summary_output = output_dir / "doppler_summary.json"
    _write_pos(position_output, result.rows)
    _write_trajectory_csv(trajectory_output, result)
    summary = _summary(result, config)
    _atomic_write(summary_output, _json_bytes(summary))
    submission_manifest: dict[str, Any] | None = None
    if submission_output is not None:
        if not phone:
            raise DopplerPositionError("phone is required for submission output")
        submission_manifest = kaggle.generate_submission(
            position_output,
            submission_output,
            phone,
            device_gnss_path=device_gnss_path,
            dataset_id=dataset_id,
            skip_epochs=skip_epochs,
            gps_utc_leap_seconds=smoother.DEFAULT_GPS_UTC_LEAP_SECONDS,
        )
    manifest = {
        "schema_version": MANIFEST_SCHEMA_VERSION,
        "truth_used": False,
        "position_input": {"path": str(position_path), "sha256": _sha256(position_path)},
        "device_gnss_input": {"path": str(device_gnss_path), "sha256": _sha256(device_gnss_path)},
        "configuration": config.as_dict(),
        "selection": {
            "phone": phone,
            "dataset_id": dataset_id,
            "skip_epochs": skip_epochs,
            "position_rows": len(result.rows),
        },
        "algorithm": {
            "equation": "d(rho)/dt = LOS dot (v_satellite-v_receiver) + clock_range_rate",
            "matrix": "A=[-LOS,1], b=PseudorangeRate-LOS dot satellite_velocity",
            "weight": "inverse square finite PseudorangeRateUncertainty with 0.10 m/s floor",
            "robust_screen": "two fixed passes, absolute residual <= 3.0 m/s",
            "update": "prediction + covariance-derived scalar gain * (WLS-prediction)",
            "fail_closed": "invalid Doppler, clock transition, long gap, rank deficiency, excessive speed, nonfinite covariance/gain, or innovation > 30 m uses WLS and resets",
        },
        "runtime": {
            "source_counts": result.source_counts,
            "reason_counts": result.reason_counts,
            "invalid_observation_rows": result.invalid_observation_rows,
            "duplicate_observation_rows": result.duplicate_observation_rows,
            "clock_transition_count": result.clock_transition_count,
            "maximum_innovation_m": result.maximum_innovation_m,
            "maximum_speed_mps": result.maximum_speed_mps,
            "wall_time_s": result.wall_time_s,
        },
        "artifacts": {
            "position": {
                "path": str(position_output),
                "sha256": _sha256(position_output),
                "bytes": position_output.stat().st_size,
            },
            "trajectory": {
                "path": str(trajectory_output),
                "sha256": _sha256(trajectory_output),
                "bytes": trajectory_output.stat().st_size,
            },
            "summary": {
                "path": str(summary_output),
                "sha256": _sha256(summary_output),
                "bytes": summary_output.stat().st_size,
            },
            "submission": submission_manifest,
        },
    }
    manifest_output = output_dir / "doppler_manifest.json"
    _atomic_write(manifest_output, _json_bytes(manifest))
    manifest["artifacts"]["manifest"] = {
        "path": str(manifest_output),
        "sha256": _sha256(manifest_output),
        "bytes": manifest_output.stat().st_size,
    }
    return manifest


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-doppler-position")
    )
    parser.add_argument("--position", type=Path, required=True)
    parser.add_argument("--device-gnss", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--phone")
    parser.add_argument("--dataset-id")
    parser.add_argument("--submission-output", type=Path)
    parser.add_argument("--skip-epochs", type=int, default=DEFAULT_SKIP_EPOCHS)
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.skip_epochs < 0:
            raise DopplerPositionError("skip-epochs must be non-negative")
        config = DopplerConfig()
        config.validate()
        positions = smoother._read_positions(
            args.position, smoother.DEFAULT_GPS_UTC_LEAP_SECONDS
        )
        device_epochs = smoother._read_device_epochs(args.device_gnss, args.skip_epochs)
        raw_epochs, _, invalid_rows, duplicate_rows = _raw_observations(args.device_gnss)
        result = build_candidate(positions, device_epochs, raw_epochs, config)
        result = DopplerPositionResult(
            result.rows,
            result.estimates,
            result.sources,
            result.source_counts,
            result.reason_counts,
            invalid_rows,
            duplicate_rows,
            result.clock_transition_count,
            result.maximum_innovation_m,
            result.maximum_speed_mps,
            result.wall_time_s,
        )
        manifest = write_outputs(
            result,
            config,
            args.output_dir,
            position_path=args.position,
            device_gnss_path=args.device_gnss,
            skip_epochs=args.skip_epochs,
            phone=args.phone,
            dataset_id=args.dataset_id,
            submission_output=args.submission_output,
        )
    except (DopplerPositionError, OSError, ValueError) as exc:
        raise SystemExit(f"smartphone Doppler position failed: {exc}") from exc
    print(f"Doppler position: {manifest['artifacts']['position']['path']}")
    print(f"Doppler manifest: {manifest['artifacts']['manifest']['path']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
