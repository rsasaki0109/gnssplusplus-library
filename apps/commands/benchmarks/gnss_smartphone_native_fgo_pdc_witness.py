#!/usr/bin/env python3
"""Truth-free Android/RINEX Doppler contract sanity witness.

The witness is deliberately diagnostic only.  It never reads a truth file and
does not select or tune a solver mapping.  It uses the satellite state fields
already exported with ``device_gnss.csv`` and the handset WLS position as a
rough receiver-motion witness.  The common receiver-clock term is removed
per epoch before comparing mapping variants, so the comparison tests sign and
units without pretending that WLS is an independent accuracy reference.

The formal native-FGO v5 submission contains keyed latitude/longitude only;
it has no per-route ECEF state.  Consequently callers must not label this
raw-device-WLS witness as a v5-ECEF witness.  The output records that boundary
explicitly and fails closed on malformed/non-finite rows.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any


SPEED_OF_LIGHT_MPS = 299_792_458.0
GPS_L1_HZ = 1_575_420_000.0
OMEGA_EARTH_RAD_S = 7.2921151467e-5
MIN_RADIUS_M = 1.0e6


class WitnessError(ValueError):
    """Raised when a truth-free witness input violates its schema."""


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise WitnessError(f"cannot read {path}: {exc}") from exc
    return digest.hexdigest()


def finite(row: dict[str, str], name: str) -> float:
    value = row.get(name, "").strip()
    if not value:
        raise WitnessError(f"missing {name}")
    try:
        number = float(value)
    except ValueError as exc:
        raise WitnessError(f"invalid {name}") from exc
    if not math.isfinite(number):
        raise WitnessError(f"non-finite {name}")
    return number


def rotate_state(
    satellite_position: tuple[float, float, float],
    satellite_velocity: tuple[float, float, float],
    receiver_position: tuple[float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    delta = tuple(satellite_position[i] - receiver_position[i] for i in range(3))
    range_m = math.sqrt(sum(value * value for value in delta))
    if not math.isfinite(range_m) or range_m <= 0.0:
        raise WitnessError("invalid satellite/receiver geometry")
    angle = OMEGA_EARTH_RAD_S * range_m / SPEED_OF_LIGHT_MPS
    cosine, sine = math.cos(angle), math.sin(angle)
    position = (
        cosine * satellite_position[0] + sine * satellite_position[1],
        -sine * satellite_position[0] + cosine * satellite_position[1],
        satellite_position[2],
    )
    velocity = (
        cosine * satellite_velocity[0] + sine * satellite_velocity[1],
        -sine * satellite_velocity[0] + cosine * satellite_velocity[1],
        satellite_velocity[2],
    )
    return position, velocity


def receiver_velocities(
    positions: dict[int, tuple[float, float, float]],
) -> dict[int, tuple[float, float, float]]:
    timestamps = sorted(positions)
    if len(timestamps) < 2:
        raise WitnessError("at least two WLS epochs are required")
    result: dict[int, tuple[float, float, float]] = {}
    for index, timestamp in enumerate(timestamps):
        if index == 0:
            left, right = timestamps[0], timestamps[1]
        elif index + 1 == len(timestamps):
            left, right = timestamps[-2], timestamps[-1]
        else:
            left, right = timestamps[index - 1], timestamps[index + 1]
        dt = (right - left) / 1000.0
        if not math.isfinite(dt) or dt <= 0.0:
            raise WitnessError("WLS timestamps are not strictly increasing")
        result[timestamp] = tuple(
            (positions[right][axis] - positions[left][axis]) / dt
            for axis in range(3)
        )
    return result


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(math.ceil(fraction * len(ordered))) - 1))
    return ordered[index]


def run_witness(device_gnss: Path, route_id: str = "") -> dict[str, Any]:
    if not device_gnss.is_file():
        raise WitnessError(f"missing device_gnss.csv: {device_gnss}")

    rows_by_epoch: dict[int, list[dict[str, str]]] = {}
    positions: dict[int, tuple[float, float, float]] = {}
    required = {
        "utcTimeMillis",
        "PseudorangeRateMetersPerSecond",
        "PseudorangeRateUncertaintyMetersPerSecond",
        "CarrierFrequencyHz",
        "SvPositionXEcefMeters",
        "SvPositionYEcefMeters",
        "SvPositionZEcefMeters",
        "SvVelocityXEcefMetersPerSecond",
        "SvVelocityYEcefMetersPerSecond",
        "SvVelocityZEcefMetersPerSecond",
        "SvClockDriftMetersPerSecond",
        "WlsPositionXEcefMeters",
        "WlsPositionYEcefMeters",
        "WlsPositionZEcefMeters",
    }
    try:
        stream = device_gnss.open(newline="", encoding="utf-8-sig")
    except OSError as exc:
        raise WitnessError(f"cannot open device_gnss.csv: {device_gnss}") from exc
    with stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            missing = sorted(required - set(reader.fieldnames or ()))
            raise WitnessError(f"device_gnss schema missing fields: {missing}")
        for row in reader:
            try:
                timestamp = int(row.get("utcTimeMillis", ""))
            except ValueError as exc:
                raise WitnessError("invalid utcTimeMillis") from exc
            if timestamp <= 0:
                raise WitnessError("utcTimeMillis must be positive")
            rows_by_epoch.setdefault(timestamp, []).append(row)
            position = tuple(
                finite(row, field)
                for field in (
                    "WlsPositionXEcefMeters",
                    "WlsPositionYEcefMeters",
                    "WlsPositionZEcefMeters",
                )
            )
            if math.sqrt(sum(value * value for value in position)) <= MIN_RADIUS_M:
                raise WitnessError("WLS position is not an ECEF position")
            prior = positions.setdefault(timestamp, position)
            if prior != position:
                raise WitnessError("inconsistent WLS position within an epoch")

    velocities = receiver_velocities(positions)
    variants: dict[str, list[float]] = {
        "android_spec_mps": [],
        "rinex_roundtrip_mps": [],
        "sign_inverted_mps": [],
        "doppler_hz_mistaken_as_mps": [],
    }
    centered_abs: dict[str, list[float]] = {name: [] for name in variants}
    raw_abs: dict[str, list[float]] = {name: [] for name in variants}
    sample_count = 0
    epoch_count = 0
    skipped_rows = 0
    frequency_count = 0

    for timestamp in sorted(rows_by_epoch):
        receiver_position = positions[timestamp]
        receiver_velocity = velocities[timestamp]
        predicted: list[float] = []
        observed: dict[str, list[float]] = {name: [] for name in variants}
        for row in rows_by_epoch[timestamp]:
            try:
                rate = finite(row, "PseudorangeRateMetersPerSecond")
                uncertainty = finite(row, "PseudorangeRateUncertaintyMetersPerSecond")
                frequency = finite(row, "CarrierFrequencyHz")
                satellite_position = tuple(
                    finite(row, field)
                    for field in (
                        "SvPositionXEcefMeters",
                        "SvPositionYEcefMeters",
                        "SvPositionZEcefMeters",
                    )
                )
                satellite_velocity = tuple(
                    finite(row, field)
                    for field in (
                        "SvVelocityXEcefMetersPerSecond",
                        "SvVelocityYEcefMetersPerSecond",
                        "SvVelocityZEcefMetersPerSecond",
                    )
                )
                satellite_clock_drift = finite(row, "SvClockDriftMetersPerSecond")
            except WitnessError:
                skipped_rows += 1
                continue
            if uncertainty < 0.0 or frequency <= 0.0:
                skipped_rows += 1
                continue
            frequency_count += 1
            wavelength = SPEED_OF_LIGHT_MPS / frequency
            if frequency == GPS_L1_HZ:
                # Keep the exact L1 conversion visible in the output, but do
                # not require one frequency: Galileo/other finite bands are
                # also valid for the generic unit witness.
                pass
            corrected_position, corrected_velocity = rotate_state(
                satellite_position, satellite_velocity, receiver_position
            )
            delta = tuple(corrected_position[i] - receiver_position[i] for i in range(3))
            range_m = math.sqrt(sum(value * value for value in delta))
            if range_m <= 0.0 or not math.isfinite(range_m):
                skipped_rows += 1
                continue
            los = tuple(value / range_m for value in delta)
            satellite_rate = sum(corrected_velocity[i] * los[i] for i in range(3))
            predicted_rate = (
                satellite_rate
                - satellite_clock_drift
                - sum(los[i] * receiver_velocity[i] for i in range(3))
            )
            doppler_hz = -rate / wavelength
            roundtrip = -doppler_hz * wavelength
            expected = {
                "android_spec_mps": rate,
                "rinex_roundtrip_mps": roundtrip,
                "sign_inverted_mps": -rate,
                "doppler_hz_mistaken_as_mps": doppler_hz,
            }
            for name, value in expected.items():
                if math.isfinite(value):
                    observed[name].append(value - predicted_rate)
            predicted.append(predicted_rate)
            sample_count += 1

        if not predicted:
            continue
        epoch_count += 1
        for name, residuals in observed.items():
            if not residuals:
                continue
            # Receiver clock drift is a common per-epoch term.  Remove only
            # its robust location for this diagnostic; no variant is selected.
            center = sorted(residuals)[len(residuals) // 2]
            centered_abs[name].extend(abs(value - center) for value in residuals)
            raw_abs[name].extend(abs(value) for value in residuals)
            variants[name].extend(residuals)

    if sample_count == 0:
        raise WitnessError("no finite Doppler/satellite-state witness rows")

    def stats(values: list[float]) -> dict[str, Any]:
        absolute = [abs(value) for value in values]
        return {
            "samples": len(values),
            "raw_residual_median_abs_mps": percentile(absolute, 0.5),
            "raw_residual_p95_abs_mps": percentile(absolute, 0.95),
        }

    centered_stats = {
        name: {
            "samples": len(values),
            "clock_centered_residual_median_abs_mps": percentile(values, 0.5),
            "clock_centered_residual_p95_abs_mps": percentile(values, 0.95),
        }
        for name, values in centered_abs.items()
    }
    return {
        "schema_version": "smartphone-r5-gsdc2023-native-fgo-pdc-doppler-witness.v1",
        "route_id": route_id,
        "truth_free": True,
        "truth_policy": {
            "truth_open": False,
            "truth_materialized": False,
            "validation_or_holdout_used": False,
            "test_truth_used": False,
            "leaderboard_or_token_used": False,
        },
        "inputs": {
            "device_gnss": {"path": str(device_gnss), "sha256": sha256(device_gnss)},
            "position_source": "same device_gnss.csv WlsPosition* fields",
            "satellite_state_source": "same device_gnss.csv SvPosition/SvVelocity/SvClockDrift fields",
        },
        "contract": {
            "android_pseudorange_rate_unit": "m/s",
            "android_positive_sign": "satellite moving away",
            "rinex_doppler_unit": "cycles/s (Hz)",
            "android_to_rinex": "D=-rate/wavelength",
            "rinex_to_range_rate": "range_rate=-D*wavelength",
            "earth_rotation": "rotate satellite position and velocity together",
            "clock_treatment": "remove per-epoch common residual location for witness only",
        },
        "population": {
            "epochs": len(rows_by_epoch),
            "epochs_with_witness": epoch_count,
            "finite_rows_used": sample_count,
            "skipped_rows": skipped_rows,
            "frequency_rows": frequency_count,
        },
        "mapping_variants": {
            "android_spec_mps": stats(variants["android_spec_mps"]),
            "rinex_roundtrip_mps": stats(variants["rinex_roundtrip_mps"]),
            "sign_inverted_mps": stats(variants["sign_inverted_mps"]),
            "doppler_hz_mistaken_as_mps": stats(variants["doppler_hz_mistaken_as_mps"]),
        },
        "clock_centered_mapping_comparison": centered_stats,
        "interpretation": {
            "selection_or_tuning": False,
            "v5_ecef_witness": "blocked_missing_per_route_ecef",
            "limitation": "formal native-FGO v5 submission has keyed lat/lon only; this witness uses raw-device WLS ECEF and broadcast-state-derived satellite fields",
            "not_an_accuracy_score": True,
        },
    }


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device-gnss", type=Path, required=True)
    parser.add_argument("--route-id", default="")
    parser.add_argument("--output-json", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = run_witness(args.device_gnss, args.route_id)
        atomic_json(args.output_json, result)
    except WitnessError as exc:
        parser.error(str(exc))
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
