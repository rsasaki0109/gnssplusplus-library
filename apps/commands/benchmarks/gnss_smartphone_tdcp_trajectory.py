#!/usr/bin/env python3
"""Truth-free carrier-phase displacement constrained smartphone trajectory.

The Android raw log already contains accumulated delta range (ADR), satellite
ECEF coordinates, and a receiver WLS position for each raw epoch.  This
command uses only those fields to estimate a time-differenced carrier-phase
receiver displacement and blends it with the displacement of the frozen base
position lane.  It deliberately has no ground-truth argument and is an
experimental, development-only post-process; production SPP/RTK defaults are
not changed.

The implementation is deliberately conservative.  A pair is used only when
the Android ADR state, receiver clock continuity, geometry, robust residual,
and displacement checks all pass.  Every failed pair falls back to the exact
finite base position for that epoch and resets the displacement chain.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import sys
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


SCHEMA_VERSION = "smartphone-r5-tdcp-trajectory.v1"
MANIFEST_SCHEMA_VERSION = "smartphone-r5-tdcp-trajectory-manifest.v1"
GPS_SIGNAL = "GPS_L1_CA"
GALILEO_SIGNAL = "GAL_E1_C_P"
SUPPORTED_SIGNALS = (GPS_SIGNAL, GALILEO_SIGNAL)
GPS_CONSTELLATION = 1
GALILEO_CONSTELLATION = 6
E1_FREQUENCY_HZ = 1_575_420_000.0
ADR_VALID_BIT = 1
ADR_RESET_BIT = 2
ADR_CYCLE_SLIP_BIT = 4
MAX_PAIR_GAP_MS = 1500
MIN_INLIERS = 4
RESIDUAL_GATE_M = 0.50
MAX_ROBUST_PASSES = 2
MAX_SOLUTION_RMS_M = 0.50
MAX_DISPLACEMENT_M = 100.0
TDCP_WEIGHT = 0.50
BASE_WEIGHT = 0.50
LEAP_SECONDS = 18


class TdcpError(ValueError):
    """Raised when an input or artifact contract cannot be satisfied."""


@dataclass(frozen=True)
class RawObservation:
    timestamp_ms: int
    clock_discontinuity_count: int
    constellation: int
    svid: int
    signal: str
    adr_state: int
    adr_m: float
    cn0_db_hz: float
    satellite_ecef: np.ndarray


@dataclass(frozen=True)
class TdcpSolution:
    delta_ecef: np.ndarray | None
    pair_count: int
    inlier_count: int
    rejected_count: int
    rms_m: float | None
    reason: str


@dataclass(frozen=True)
class TdcpTrajectoryResult:
    base_rows: list[smoother.PositionRow]
    candidate_ecef: list[np.ndarray]
    sources: list[str]
    pair_stats: list[dict[str, Any]]
    observation_stats: dict[str, Any]
    elapsed_s: float


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise TdcpError(f"missing input: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_write(path: Path, content: bytes) -> None:
    # Reuse the already audited fsync + replace contract used by the
    # truth-free trajectory tools.
    smoother._atomic_write(path, content)


def _json_bytes(payload: dict[str, Any]) -> bytes:
    return (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")


def _finite_float(raw: str | None, field: str) -> float | None:
    if raw is None or not raw.strip():
        return None
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _integer(raw: str | None, field: str) -> int | None:
    if raw is None or not raw.strip():
        return None
    try:
        return int(raw)
    except (TypeError, ValueError):
        return None


def _signal_contract(row: dict[str, str]) -> tuple[int, str] | None:
    signal = (row.get("SignalType") or "").strip()
    constellation = _integer(row.get("ConstellationType"), "ConstellationType")
    if signal == GPS_SIGNAL and constellation == GPS_CONSTELLATION:
        expected = GPS_CONSTELLATION
    elif signal == GALILEO_SIGNAL and constellation == GALILEO_CONSTELLATION:
        expected = GALILEO_CONSTELLATION
    else:
        return None
    frequency = _finite_float(row.get("CarrierFrequencyHz"), "CarrierFrequencyHz")
    # Empty frequency is accepted because older Android logs omit it for some
    # otherwise correctly mapped rows.  A present frequency must be the E1/L1
    # carrier; silently accepting another band would violate the frozen lane.
    if frequency is not None and abs(frequency - E1_FREQUENCY_HZ) > 1_000_000.0:
        return None
    return expected, signal


def read_observations(path: Path) -> tuple[dict[int, dict[tuple[int, int], RawObservation]], dict[str, Any]]:
    """Read eligible ADR/satellite rows grouped by exact raw epoch.

    Unsupported, incomplete, or non-finite observations are omitted from the
    candidate rather than made into a synthetic measurement.  Structural CSV
    errors remain fatal so a malformed source cannot silently produce a
    misleading artifact.
    """

    required = {
        "MessageType",
        "utcTimeMillis",
        "HardwareClockDiscontinuityCount",
        "Svid",
        "AccumulatedDeltaRangeState",
        "AccumulatedDeltaRangeMeters",
        "CarrierFrequencyHz",
        "ConstellationType",
        "SignalType",
        "Cn0DbHz",
        "SvPositionXEcefMeters",
        "SvPositionYEcefMeters",
        "SvPositionZEcefMeters",
    }
    epochs: dict[int, dict[tuple[int, int], RawObservation]] = {}
    stats: dict[str, Any] = {
        "input_rows": 0,
        "input_epochs": 0,
        "eligible_rows": 0,
        "eligible_epochs": 0,
        "duplicate_lower_cn0_rows": 0,
        "unsupported_signal_rows": 0,
        "invalid_numeric_rows": 0,
        "invalid_adr_state_rows": 0,
        "nonfinite_satellite_rows": 0,
        "clock_values": {},
        "signals": {},
    }
    last_timestamp: int | None = None
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            fields = set(reader.fieldnames or ())
            missing = sorted(required - fields)
            if missing:
                raise TdcpError(
                    f"device GNSS CSV missing required fields: {', '.join(missing)}"
                )
            if len(fields) != len(reader.fieldnames or ()):
                raise TdcpError("device GNSS CSV has duplicate header fields")
            for line_number, raw in enumerate(reader, start=2):
                stats["input_rows"] += 1
                if None in raw:
                    raise TdcpError(f"device row {line_number}: more values than header fields")
                row = {key: (value or "") for key, value in raw.items()}
                if row.get("MessageType") != "Raw":
                    raise TdcpError(f"device row {line_number}: MessageType must be Raw")
                timestamp = _integer(row.get("utcTimeMillis"), "utcTimeMillis")
                if timestamp is None or timestamp < 0:
                    raise TdcpError(f"device row {line_number}: invalid utcTimeMillis")
                if last_timestamp is not None and timestamp < last_timestamp:
                    raise TdcpError(f"device row {line_number}: utcTimeMillis moved backwards")
                last_timestamp = timestamp
                if timestamp not in epochs:
                    epochs[timestamp] = {}
                clock = _integer(row.get("HardwareClockDiscontinuityCount"), "clock")
                contract = _signal_contract(row)
                if clock is None or contract is None:
                    if contract is None:
                        stats["unsupported_signal_rows"] += 1
                    else:
                        stats["invalid_numeric_rows"] += 1
                    continue
                constellation, signal = contract
                svid = _integer(row.get("Svid"), "Svid")
                adr_state = _integer(row.get("AccumulatedDeltaRangeState"), "ADR state")
                adr_m = _finite_float(row.get("AccumulatedDeltaRangeMeters"), "ADR")
                cn0 = _finite_float(row.get("Cn0DbHz"), "Cn0")
                satellite_values = tuple(
                    _finite_float(row.get(field), field)
                    for field in (
                        "SvPositionXEcefMeters",
                        "SvPositionYEcefMeters",
                        "SvPositionZEcefMeters",
                    )
                )
                if svid is None or svid <= 0 or adr_m is None or cn0 is None:
                    stats["invalid_numeric_rows"] += 1
                    continue
                if adr_state is None or not (adr_state & ADR_VALID_BIT) or (
                    adr_state & (ADR_RESET_BIT | ADR_CYCLE_SLIP_BIT)
                ):
                    stats["invalid_adr_state_rows"] += 1
                    continue
                if any(value is None for value in satellite_values):
                    stats["nonfinite_satellite_rows"] += 1
                    continue
                satellite = np.asarray(satellite_values, dtype=float)
                if not np.isfinite(satellite).all() or np.linalg.norm(satellite) < 1.0e6:
                    stats["nonfinite_satellite_rows"] += 1
                    continue
                key = (constellation, svid)
                observation = RawObservation(
                    timestamp,
                    clock,
                    constellation,
                    svid,
                    signal,
                    adr_state,
                    adr_m,
                    cn0,
                    satellite,
                )
                previous = epochs[timestamp].get(key)
                if previous is not None:
                    if observation.cn0_db_hz <= previous.cn0_db_hz:
                        stats["duplicate_lower_cn0_rows"] += 1
                        continue
                    stats["duplicate_lower_cn0_rows"] += 1
                epochs[timestamp][key] = observation
                stats["eligible_rows"] += 1
                stats["signals"][signal] = stats["signals"].get(signal, 0) + 1
                stats["clock_values"][str(clock)] = stats["clock_values"].get(str(clock), 0) + 1
    except OSError as exc:
        raise TdcpError(f"failed to read device GNSS CSV: {path}") from exc
    stats["input_epochs"] = len(epochs)
    stats["eligible_epochs"] = sum(bool(values) for values in epochs.values())
    return epochs, stats


def _eligible_pair(previous: RawObservation, current: RawObservation) -> bool:
    return (
        previous.adr_state & ADR_VALID_BIT
        and current.adr_state & ADR_VALID_BIT
        and not previous.adr_state & (ADR_RESET_BIT | ADR_CYCLE_SLIP_BIT)
        and not current.adr_state & (ADR_RESET_BIT | ADR_CYCLE_SLIP_BIT)
        and previous.clock_discontinuity_count == current.clock_discontinuity_count
    )


def _solve_least_squares(matrix: np.ndarray, values: np.ndarray) -> np.ndarray | None:
    try:
        solution, _, rank, _ = np.linalg.lstsq(matrix, values, rcond=None)
    except (np.linalg.LinAlgError, ValueError):
        return None
    if int(rank) < 4 or solution.shape != (4,) or not np.isfinite(solution).all():
        return None
    return solution


def solve_tdcp(
    previous: dict[tuple[int, int], RawObservation] | None,
    current: dict[tuple[int, int], RawObservation] | None,
    previous_timestamp_ms: int,
    current_timestamp_ms: int,
    previous_base_ecef: np.ndarray,
) -> TdcpSolution:
    """Solve the frozen four-state TDCP equation for one consecutive pair."""

    if previous is None or current is None:
        return TdcpSolution(None, 0, 0, 0, None, "missing_observation_epoch")
    gap = current_timestamp_ms - previous_timestamp_ms
    if gap <= 0 or gap > MAX_PAIR_GAP_MS:
        return TdcpSolution(None, 0, 0, 0, None, "pair_gap_or_order")
    if previous_base_ecef.shape != (3,) or not np.isfinite(previous_base_ecef).all():
        return TdcpSolution(None, 0, 0, 0, None, "nonfinite_base_position")

    matrix_rows: list[np.ndarray] = []
    measurements: list[float] = []
    common = sorted(set(previous).intersection(current))
    for key in common:
        old = previous[key]
        new = current[key]
        if not _eligible_pair(old, new):
            continue
        line_of_sight_vector = old.satellite_ecef - previous_base_ecef
        norm = float(np.linalg.norm(line_of_sight_vector))
        if not math.isfinite(norm) or norm < 1.0e6:
            continue
        line_of_sight = line_of_sight_vector / norm
        satellite_delta = new.satellite_ecef - old.satellite_ecef
        measurement = new.adr_m - old.adr_m - float(np.dot(line_of_sight, satellite_delta))
        if not math.isfinite(measurement):
            continue
        matrix_rows.append(
            np.asarray(
                (-line_of_sight[0], -line_of_sight[1], -line_of_sight[2], 1.0),
                dtype=float,
            )
        )
        measurements.append(measurement)
    pair_count = len(matrix_rows)
    if pair_count < MIN_INLIERS:
        return TdcpSolution(None, pair_count, 0, 0, None, "insufficient_pairs")

    matrix = np.asarray(matrix_rows, dtype=float)
    values = np.asarray(measurements, dtype=float)
    inlier_mask = np.ones(pair_count, dtype=bool)
    solution: np.ndarray | None = None
    for _ in range(MAX_ROBUST_PASSES):
        solution = _solve_least_squares(matrix[inlier_mask], values[inlier_mask])
        if solution is None:
            return TdcpSolution(None, pair_count, int(inlier_mask.sum()), pair_count - int(inlier_mask.sum()), None, "rank_deficient")
        residuals = matrix @ solution - values
        center = float(np.median(residuals))
        mad = float(np.median(np.abs(residuals - center)))
        threshold = max(RESIDUAL_GATE_M, 6.0 * 1.4826 * mad)
        next_mask = np.isfinite(residuals) & (np.abs(residuals - center) <= threshold)
        if int(next_mask.sum()) < MIN_INLIERS:
            return TdcpSolution(None, pair_count, int(next_mask.sum()), pair_count - int(next_mask.sum()), None, "insufficient_robust_inliers")
        if np.array_equal(next_mask, inlier_mask):
            inlier_mask = next_mask
            break
        inlier_mask = next_mask
    if solution is None:
        return TdcpSolution(None, pair_count, 0, pair_count, None, "solver_failure")
    solution = _solve_least_squares(matrix[inlier_mask], values[inlier_mask])
    if solution is None:
        return TdcpSolution(None, pair_count, int(inlier_mask.sum()), pair_count - int(inlier_mask.sum()), None, "rank_deficient")
    residuals = matrix[inlier_mask] @ solution - values[inlier_mask]
    rms = float(math.sqrt(float(np.mean(np.square(residuals))))) if residuals.size else math.inf
    displacement = solution[:3]
    displacement_norm = float(np.linalg.norm(displacement))
    if not math.isfinite(rms) or rms > MAX_SOLUTION_RMS_M:
        return TdcpSolution(None, pair_count, int(inlier_mask.sum()), pair_count - int(inlier_mask.sum()), rms, "rms_gate")
    if not math.isfinite(displacement_norm) or displacement_norm > MAX_DISPLACEMENT_M:
        return TdcpSolution(None, pair_count, int(inlier_mask.sum()), pair_count - int(inlier_mask.sum()), rms, "displacement_gate")
    return TdcpSolution(
        displacement.copy(),
        pair_count,
        int(inlier_mask.sum()),
        pair_count - int(inlier_mask.sum()),
        rms,
        "accepted",
    )


def build_trajectory(
    base_position_path: Path,
    device_gnss_path: Path,
    *,
    skip_epochs: int = 0,
) -> TdcpTrajectoryResult:
    """Build a TDCP/base blend using no truth or external score."""

    started = time.perf_counter()
    if skip_epochs < 0:
        raise TdcpError("skip_epochs must be non-negative")
    base_rows = smoother._read_positions(base_position_path, LEAP_SECONDS)
    device_epochs = smoother._read_device_epochs(device_gnss_path, skip_epochs)
    observations, observation_stats = read_observations(device_gnss_path)
    base_by_timestamp = {row.timestamp_ms: row for row in base_rows}
    selected_rows = [row for row in base_rows if row.timestamp_ms in set(device_epochs)]
    if not selected_rows:
        raise TdcpError("base position has no exact selected device epoch keys")
    if len(selected_rows) != len(base_rows):
        # A partial base lane would make availability depend on the candidate
        # rather than on the frozen lane.  Keep the postprocess fail-closed.
        raise TdcpError("base position timestamps are not all exact selected device epochs")
    if any(row.timestamp_ms not in observations for row in selected_rows):
        # Missing observation epochs are valid fallback conditions, not fatal.
        pass

    candidate_ecef: list[np.ndarray] = []
    sources: list[str] = []
    pair_stats: list[dict[str, Any]] = []
    previous_base: smoother.PositionRow | None = None
    previous_candidate: np.ndarray | None = None
    accepted = 0
    fallback_counts: dict[str, int] = {}
    for index, row in enumerate(selected_rows):
        base = np.asarray(row.ecef, dtype=float)
        if base.shape != (3,) or not np.isfinite(base).all():
            raise TdcpError(f"base position epoch {row.timestamp_ms} is non-finite")
        if index == 0:
            candidate = base.copy()
            source = "anchor"
            solution = TdcpSolution(None, 0, 0, 0, None, "anchor")
        else:
            assert previous_base is not None and previous_candidate is not None
            solution = solve_tdcp(
                observations.get(previous_base.timestamp_ms),
                observations.get(row.timestamp_ms),
                previous_base.timestamp_ms,
                row.timestamp_ms,
                np.asarray(previous_base.ecef, dtype=float),
            )
            if solution.delta_ecef is None:
                candidate = base.copy()
                source = f"base_fallback_{solution.reason}"
                fallback_counts[solution.reason] = fallback_counts.get(solution.reason, 0) + 1
            else:
                base_delta = base - np.asarray(previous_base.ecef, dtype=float)
                candidate = previous_candidate + TDCP_WEIGHT * solution.delta_ecef + BASE_WEIGHT * base_delta
                if not np.isfinite(candidate).all():
                    candidate = base.copy()
                    source = "base_fallback_nonfinite_blend"
                    fallback_counts["nonfinite_blend"] = fallback_counts.get("nonfinite_blend", 0) + 1
                    solution = TdcpSolution(None, solution.pair_count, solution.inlier_count, solution.rejected_count, solution.rms_m, "nonfinite_blend")
                else:
                    source = "tdcp_blend"
                    accepted += 1
        candidate_ecef.append(candidate)
        sources.append(source)
        pair_stats.append(
            {
                "timestamp_ms": row.timestamp_ms,
                "previous_timestamp_ms": previous_base.timestamp_ms if previous_base is not None else None,
                "gap_ms": row.timestamp_ms - previous_base.timestamp_ms if previous_base is not None else None,
                "pair_count": solution.pair_count,
                "inlier_count": solution.inlier_count,
                "rejected_count": solution.rejected_count,
                "rms_m": solution.rms_m,
                "reason": solution.reason,
                "source": source,
            }
        )
        previous_base = row
        previous_candidate = candidate

    observation_stats["selected_base_epochs"] = len(selected_rows)
    observation_stats["accepted_tdcp_epochs"] = accepted
    observation_stats["fallback_counts"] = dict(sorted(fallback_counts.items()))
    observation_stats["source_counts"] = {
        source: sources.count(source) for source in sorted(set(sources))
    }
    return TdcpTrajectoryResult(
        selected_rows,
        candidate_ecef,
        sources,
        pair_stats,
        observation_stats,
        time.perf_counter() - started,
    )


def _write_pos(path: Path, result: TdcpTrajectoryResult) -> None:
    lines = [
        "% LibGNSS++ truth-free TDCP/base displacement blend",
        "% Columns: GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) Status Satellites PDOP Ratio FixedAmbiguities Iterations",
        "% Candidate parameters are frozen in smartphone_r5_tdcp_trajectory_selection.json",
    ]
    for row, ecef in zip(result.base_rows, result.candidate_ecef):
        latitude, longitude, height = smoother._wgs84_ecef_to_geodetic(ecef)
        lines.append(
            f"{row.week:d} {row.tow:.9f} {ecef[0]:.9f} {ecef[1]:.9f} {ecef[2]:.9f} "
            f"{math.degrees(latitude):.9f} {math.degrees(longitude):.9f} {height:.6f} "
            f"{row.status:d} {row.satellites:d} {row.pdop:.6f} {row.ratio:.6f} "
            f"{row.fixed_ambiguities:d} {row.iterations:d}"
        )
    _atomic_write(path, ("\n".join(lines) + "\n").encode("ascii"))


def _write_trajectory(path: Path, result: TdcpTrajectoryResult) -> None:
    output = []
    buffer = __import__("io").StringIO(newline="")
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerow(
        (
            "UnixTimeMillis",
            "LatitudeDegrees",
            "LongitudeDegrees",
            "AltitudeMeters",
            "Xmeters",
            "Ymeters",
            "Zmeters",
            "source",
            "pair_count",
            "inlier_count",
            "rejected_count",
            "rms_m",
            "reason",
        )
    )
    for ecef, stats in zip(result.candidate_ecef, result.pair_stats):
        latitude, longitude, height = smoother._wgs84_ecef_to_geodetic(ecef)
        writer.writerow(
            (
                stats["timestamp_ms"],
                f"{math.degrees(latitude):.12f}",
                f"{math.degrees(longitude):.12f}",
                f"{height:.6f}",
                f"{ecef[0]:.9f}",
                f"{ecef[1]:.9f}",
                f"{ecef[2]:.9f}",
                stats["source"],
                stats["pair_count"],
                stats["inlier_count"],
                stats["rejected_count"],
                "" if stats["rms_m"] is None else f"{stats['rms_m']:.9f}",
                stats["reason"],
            )
        )
    _atomic_write(path, buffer.getvalue().encode("utf-8"))


def _candidate_parameters() -> dict[str, Any]:
    return {
        "source_signals": list(SUPPORTED_SIGNALS),
        "adr_state": {
            "valid_bit": ADR_VALID_BIT,
            "reset_bit": ADR_RESET_BIT,
            "cycle_slip_bit": ADR_CYCLE_SLIP_BIT,
        },
        "max_pair_gap_ms": MAX_PAIR_GAP_MS,
        "minimum_inliers": MIN_INLIERS,
        "residual_gate_m": RESIDUAL_GATE_M,
        "maximum_robust_passes": MAX_ROBUST_PASSES,
        "maximum_solution_rms_m": MAX_SOLUTION_RMS_M,
        "maximum_displacement_m": MAX_DISPLACEMENT_M,
        "tdcp_weight": TDCP_WEIGHT,
        "base_position_delta_weight": BASE_WEIGHT,
        "equation": "delta_ADR - LOS_previous dot delta_satellite = -LOS_previous dot delta_receiver + receiver_clock_range_change",
        "fallback": "exact finite base position and chain reset on continuity/state/geometry/inlier/RMS/displacement failure",
        "truth_free": True,
    }


def write_outputs(
    result: TdcpTrajectoryResult,
    *,
    base_position_path: Path,
    device_gnss_path: Path,
    output_dir: Path,
    phone: str | None = None,
    dataset_id: str | None = None,
    submission_output: Path | None = None,
    skip_epochs: int = 0,
) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    position_output = output_dir / "tdcp_blended.pos"
    trajectory_output = output_dir / "tdcp_trajectory.csv"
    summary_output = output_dir / "tdcp_summary.json"
    manifest_output = output_dir / "tdcp_manifest.json"
    _write_pos(position_output, result)
    _write_trajectory(trajectory_output, result)
    submission_manifest: dict[str, Any] | None = None
    if submission_output is not None:
        if not phone:
            raise TdcpError("--phone is required when --submission-output is used")
        submission_manifest = kaggle.generate_submission(
            position_output,
            submission_output,
            phone,
            device_gnss_path=device_gnss_path,
            dataset_id=dataset_id,
            skip_epochs=skip_epochs,
            gps_utc_leap_seconds=LEAP_SECONDS,
        )
    summary = {
        "schema_version": SCHEMA_VERSION,
        "decision": "truth-free-tdcp-blend",
        "candidate": _candidate_parameters(),
        "elapsed_s": result.elapsed_s,
        "observations": result.observation_stats,
        "pair_counts": {
            "total_pairs": max(0, len(result.pair_stats) - 1),
            "accepted": sum(stat["reason"] == "accepted" for stat in result.pair_stats),
            "fallback": sum(stat["reason"] not in ("accepted", "anchor") for stat in result.pair_stats),
        },
        "inputs": {
            "base_position": {"path": str(base_position_path), "sha256": _sha256(base_position_path)},
            "device_gnss": {"path": str(device_gnss_path), "sha256": _sha256(device_gnss_path)},
            "ground_truth": None,
        },
        "artifacts": {
            "position": {"path": str(position_output), "sha256": _sha256(position_output), "rows": len(result.base_rows)},
            "trajectory": {"path": str(trajectory_output), "sha256": _sha256(trajectory_output), "rows": len(result.base_rows)},
            "submission": (
                {"path": str(submission_output), "sha256": _sha256(submission_output), "manifest": submission_manifest}
                if submission_output is not None
                else None
            ),
        },
        "integrity": {
            "truth_used": False,
            "public_private_score_used": False,
            "external_mutation": False,
            "production_default_changed": False,
        },
    }
    _atomic_write(summary_output, _json_bytes(summary))
    manifest = {
        "schema_version": MANIFEST_SCHEMA_VERSION,
        "candidate": _candidate_parameters(),
        "inputs": summary["inputs"],
        "artifacts": {
            "position": {"path": str(position_output), "sha256": _sha256(position_output)},
            "trajectory": {"path": str(trajectory_output), "sha256": _sha256(trajectory_output)},
            "summary": {"path": str(summary_output), "sha256": _sha256(summary_output)},
            "submission": (
                {"path": str(submission_output), "sha256": _sha256(submission_output)}
                if submission_output is not None
                else None
            ),
        },
        "truth_free_contract": {
            "truth_path": None,
            "no_truth_argument": True,
            "atomic_publish": True,
            "nonfinite_policy": "fail-closed",
        },
        "observations": result.observation_stats,
    }
    _atomic_write(manifest_output, _json_bytes(manifest))
    return manifest


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="smartphone-tdcp-trajectory")
    parser.add_argument("--base-position", type=Path, required=True)
    parser.add_argument("--device-gnss", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--phone")
    parser.add_argument("--dataset-id")
    parser.add_argument("--skip-epochs", type=int, default=0)
    parser.add_argument("--submission-output", type=Path)
    # No ground-truth option is intentionally defined.  argparse therefore
    # fails closed if a caller attempts to pass one to this truth-free lane.
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        result = build_trajectory(
            args.base_position,
            args.device_gnss,
            skip_epochs=args.skip_epochs,
        )
        manifest = write_outputs(
            result,
            base_position_path=args.base_position,
            device_gnss_path=args.device_gnss,
            output_dir=args.output_dir,
            phone=args.phone,
            dataset_id=args.dataset_id,
            submission_output=args.submission_output,
            skip_epochs=args.skip_epochs,
        )
    except (OSError, TdcpError, smoother.SmootherError, ValueError) as exc:
        print(f"smartphone TDCP trajectory failed: {exc}", file=sys.stderr)
        return 1
    print(f"Smartphone TDCP trajectory: {manifest['artifacts']['position']['path']}")
    print(f"TDCP manifest: {args.output_dir / 'tdcp_manifest.json'}")
    return 0


def main() -> int:
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
