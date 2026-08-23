#!/usr/bin/env python3
"""Validate and normalize Google Smartphone Decimeter Challenge GNSS CSVs."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timedelta
import hashlib
import json
import math
import os
from pathlib import Path
from statistics import median


SCHEMA_VERSION = "smartphone-gnss-adapter.v1"
DEVICE_REQUIRED = (
    "MessageType",
    "utcTimeMillis",
    "TimeNanos",
    "FullBiasNanos",
    "HardwareClockDiscontinuityCount",
    "Svid",
    "State",
    "ReceivedSvTimeNanos",
    "ReceivedSvTimeUncertaintyNanos",
    "Cn0DbHz",
    "PseudorangeRateMetersPerSecond",
    "AccumulatedDeltaRangeState",
    "AccumulatedDeltaRangeMeters",
    "AccumulatedDeltaRangeUncertaintyMeters",
    "CarrierFrequencyHz",
    "ConstellationType",
    "SignalType",
    "ArrivalTimeNanosSinceGpsEpoch",
    "RawPseudorangeMeters",
    "RawPseudorangeUncertaintyMeters",
    "WlsPositionXEcefMeters",
    "WlsPositionYEcefMeters",
    "WlsPositionZEcefMeters",
)
TRUTH_REQUIRED = (
    "MessageType",
    "Provider",
    "LatitudeDegrees",
    "LongitudeDegrees",
    "AltitudeMeters",
    "UnixTimeMillis",
)
GPS_EPOCH = datetime(1980, 1, 6)
GPS_L1_HZ = 1_575_420_000.0
SPEED_OF_LIGHT_MPS = 299_792_458.0


def fail(message: str) -> "NoReturn":
    raise SystemExit(message)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def integer(token: str, field: str, row_number: int) -> int:
    try:
        value = float(token)
    except (TypeError, ValueError):
        fail(f"row {row_number}: invalid {field}: {token!r}")
    if not math.isfinite(value) or not value.is_integer():
        fail(f"row {row_number}: {field} must be a finite integer")
    return int(value)


def finite(token: str, field: str, row_number: int) -> float:
    try:
        value = float(token)
    except (TypeError, ValueError):
        fail(f"row {row_number}: invalid {field}: {token!r}")
    if not math.isfinite(value):
        fail(f"row {row_number}: {field} must be finite")
    return value


def read_csv(path: Path, required: tuple[str, ...]) -> tuple[list[str], list[dict[str, str]]]:
    if not path.is_file():
        fail(f"missing input: {path}")
    with path.open(encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        fields = list(reader.fieldnames or ())
        if len(fields) != len(set(fields)):
            fail(f"duplicate CSV fields in {path}")
        missing = [field for field in required if field not in fields]
        if missing:
            fail(f"missing required fields in {path}: {', '.join(missing)}")
        rows = list(reader)
    if not rows:
        fail(f"no data rows in {path}")
    return fields, rows


def ecef_to_geodetic(x: float, y: float, z: float) -> tuple[float, float, float]:
    a = 6378137.0
    e2 = 6.6943799901413165e-3
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - e2))
    height = 0.0
    for _ in range(10):
        sin_lat = math.sin(lat)
        radius = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
        height = p / max(math.cos(lat), 1e-15) - radius
        next_lat = math.atan2(z, p * (1.0 - e2 * radius / (radius + height)))
        if abs(next_lat - lat) < 1e-14:
            lat = next_lat
            break
        lat = next_lat
    return math.degrees(lat), math.degrees(lon), height


def horizontal_error_m(lat: float, lon: float, truth_lat: float, truth_lon: float) -> float:
    radius = 6378137.0
    dlat = math.radians(lat - truth_lat)
    dlon = math.radians(lon - truth_lon)
    mean_lat = math.radians((lat + truth_lat) / 2.0)
    return radius * math.hypot(dlat, math.cos(mean_lat) * dlon)


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - index) + ordered[upper] * (index - lower)


def rinex_header_line(content: str, label: str) -> str:
    return f"{content[:60]:<60}{label}\n"


def rinex_value(value: float | None) -> str:
    return " " * 16 if value is None else f"{value:14.3f}  "


def write_gps_l1_rinex(
    path: Path,
    epochs: dict[int, list[dict[str, str]]],
    selected_timestamps: list[int],
) -> dict[str, object]:
    converted: list[tuple[float, list[tuple[int, float, float | None, float, float]]]] = []
    source_rows = 0
    for timestamp in selected_timestamps:
        by_satellite: dict[int, tuple[float, float | None, float, float]] = {}
        arrival_times: list[float] = []
        for row in epochs[timestamp]:
            if row["SignalType"].strip() != "GPS_L1_CA":
                continue
            pseudorange_token = row["RawPseudorangeMeters"].strip()
            arrival_token = row["ArrivalTimeNanosSinceGpsEpoch"].strip()
            if not pseudorange_token or not arrival_token:
                continue
            pseudorange = finite(pseudorange_token, "RawPseudorangeMeters", 0)
            arrival_seconds = finite(arrival_token, "ArrivalTimeNanosSinceGpsEpoch", 0) / 1e9
            prn = integer(row["Svid"], "Svid", 0)
            if not 1 <= prn <= 32:
                fail(f"epoch {timestamp}: invalid GPS PRN {prn}")
            wavelength = SPEED_OF_LIGHT_MPS / GPS_L1_HZ
            rate = finite(row["PseudorangeRateMetersPerSecond"], "PseudorangeRateMetersPerSecond", 0)
            doppler = -rate / wavelength
            cn0 = finite(row["Cn0DbHz"], "Cn0DbHz", 0)
            adr_token = row["AccumulatedDeltaRangeMeters"].strip()
            adr_state = integer(row["AccumulatedDeltaRangeState"], "AccumulatedDeltaRangeState", 0)
            carrier = None
            if adr_token and adr_state & 1:
                adr_m = finite(adr_token, "AccumulatedDeltaRangeMeters", 0)
                if abs(adr_m) < 1e9:
                    carrier = adr_m / wavelength
            if prn in by_satellite:
                fail(f"epoch {timestamp}: duplicate GPS_L1_CA observation for G{prn:02d}")
            by_satellite[prn] = (pseudorange, carrier, doppler, cn0)
            arrival_times.append(arrival_seconds)
            source_rows += 1
        if not by_satellite:
            continue
        if max(arrival_times) - min(arrival_times) > 1e-3:
            fail(f"epoch {timestamp}: inconsistent GPS arrival times")
        converted.append((median(arrival_times), [(prn, *values) for prn, values in sorted(by_satellite.items())]))

    if not converted:
        fail("no supported GPS_L1_CA observations with raw pseudoranges")
    first_source_row = epochs[selected_timestamps[0]][0]
    approximate_position = tuple(
        finite(first_source_row[field], field, 0)
        for field in (
            "WlsPositionXEcefMeters",
            "WlsPositionYEcefMeters",
            "WlsPositionZEcefMeters",
        )
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii", newline="") as handle:
        handle.write(rinex_header_line("     3.04           OBSERVATION DATA    G", "RINEX VERSION / TYPE"))
        handle.write(rinex_header_line("gnss smartphone-adap libgnss++", "PGM / RUN BY / DATE"))
        handle.write(rinex_header_line("GSDC GPS_L1_CA; GPST from ArrivalTimeNanosSinceGpsEpoch", "COMMENT"))
        handle.write(
            rinex_header_line(
                "".join(f"{coordinate:14.4f}" for coordinate in approximate_position),
                "APPROX POSITION XYZ",
            )
        )
        handle.write(rinex_header_line("G    4 C1C L1C D1C S1C", "SYS / # / OBS TYPES"))
        handle.write(rinex_header_line("GPS", "TIME SYSTEM ID"))
        handle.write(rinex_header_line("", "END OF HEADER"))
        for gps_seconds, observations in converted:
            stamp = GPS_EPOCH + timedelta(seconds=gps_seconds)
            second = stamp.second + stamp.microsecond / 1e6
            handle.write(
                f"> {stamp.year:04d} {stamp.month:02d} {stamp.day:02d} {stamp.hour:02d} "
                f"{stamp.minute:02d} {second:011.7f}  0{len(observations):3d}\n"
            )
            for prn, pseudorange, carrier, doppler, cn0 in observations:
                handle.write(
                    f"G{prn:02d}{rinex_value(pseudorange)}{rinex_value(carrier)}"
                    f"{rinex_value(doppler)}{rinex_value(cn0)}\n"
                )
    return {
        "path": str(path),
        "signal_mapping": {"GPS_L1_CA": ["C1C", "L1C", "D1C", "S1C"]},
        "source_rows": source_rows,
        "epochs": len(converted),
        "time_system": "GPST",
        "initial_position_source": "device_wls_ecef_first_selected_epoch",
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--device-gnss", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--dataset-id", required=True)
    parser.add_argument("--device-model", required=True)
    parser.add_argument("--source-url", required=True)
    parser.add_argument("--source-terms", required=True)
    parser.add_argument("--role", choices=("development", "holdout"), required=True)
    parser.add_argument("--skip-epochs", type=int, default=0)
    parser.add_argument("--max-epochs", type=int, default=-1)
    parser.add_argument(
        "--allow-missing-truth",
        action="store_true",
        help="Inventory-only mode; accuracy evidence remains unavailable.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.skip_epochs < 0:
        fail("--skip-epochs must be zero or greater")
    if args.max_epochs == 0 or args.max_epochs < -1:
        fail("--max-epochs must be -1 or a positive integer")

    fields, rows = read_csv(args.device_gnss, DEVICE_REQUIRED)
    _, truth_rows = read_csv(args.ground_truth, TRUTH_REQUIRED)

    epochs: dict[int, list[dict[str, str]]] = {}
    last_timestamp: int | None = None
    for row_number, row in enumerate(rows, start=2):
        if row["MessageType"] != "Raw":
            fail(f"row {row_number}: MessageType must be Raw")
        timestamp = integer(row["utcTimeMillis"], "utcTimeMillis", row_number)
        if last_timestamp is not None and timestamp < last_timestamp:
            fail(f"row {row_number}: utcTimeMillis moved backwards")
        last_timestamp = timestamp
        epochs.setdefault(timestamp, []).append(row)

    if args.skip_epochs >= len(epochs):
        fail("--skip-epochs removes every input epoch")
    selected_timestamps = list(epochs)[args.skip_epochs :]
    if args.max_epochs > 0:
        selected_timestamps = selected_timestamps[: args.max_epochs]
    selected = set(selected_timestamps)
    selected_rows = [row for row in rows if integer(row["utcTimeMillis"], "utcTimeMillis", 0) in selected]

    clock_counts: list[int] = []
    signals: dict[str, int] = {}
    constellations: dict[str, int] = {}
    unmapped_signal_rows = 0
    receiver_positions: dict[int, tuple[float, float, float]] = {}
    for timestamp in selected_timestamps:
        epoch_rows = epochs[timestamp]
        counts = {
            integer(row["HardwareClockDiscontinuityCount"], "HardwareClockDiscontinuityCount", 0)
            for row in epoch_rows
        }
        if len(counts) != 1:
            fail(f"epoch {timestamp}: inconsistent hardware clock discontinuity count")
        count = counts.pop()
        if clock_counts and count < clock_counts[-1]:
            fail(f"epoch {timestamp}: hardware clock discontinuity count moved backwards")
        clock_counts.append(count)

        positions = []
        for row in epoch_rows:
            signal = row["SignalType"].strip()
            if not signal:
                signal = "<unmapped>"
                unmapped_signal_rows += 1
            signals[signal] = signals.get(signal, 0) + 1
            constellation = row["ConstellationType"].strip()
            constellations[constellation] = constellations.get(constellation, 0) + 1
            positions.append(
                tuple(
                    finite(row[field], field, 0)
                    for field in (
                        "WlsPositionXEcefMeters",
                        "WlsPositionYEcefMeters",
                        "WlsPositionZEcefMeters",
                    )
                )
            )
        for axis in range(3):
            if max(position[axis] for position in positions) - min(position[axis] for position in positions) > 1e-3:
                fail(f"epoch {timestamp}: inconsistent WLS ECEF position")
        receiver_positions[timestamp] = positions[0]

    truth: dict[int, tuple[float, float, float]] = {}
    last_truth_timestamp: int | None = None
    for row_number, row in enumerate(truth_rows, start=2):
        if row["MessageType"] != "Fix" or row["Provider"] != "GT":
            fail(f"truth row {row_number}: expected MessageType=Fix and Provider=GT")
        timestamp = integer(row["UnixTimeMillis"], "UnixTimeMillis", row_number)
        if last_truth_timestamp is not None and timestamp <= last_truth_timestamp:
            fail(f"truth row {row_number}: timestamps must be strictly increasing")
        last_truth_timestamp = timestamp
        truth[timestamp] = (
            finite(row["LatitudeDegrees"], "LatitudeDegrees", row_number),
            finite(row["LongitudeDegrees"], "LongitudeDegrees", row_number),
            finite(row["AltitudeMeters"], "AltitudeMeters", row_number),
        )

    missing_truth = [timestamp for timestamp in selected_timestamps if timestamp not in truth]
    if missing_truth and not args.allow_missing_truth:
        fail(f"missing truth for {len(missing_truth)} selected epochs")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    normalized_path = args.output_dir / "observations.csv"
    with normalized_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(selected_rows)

    rinex_path = args.output_dir / "rover.obs"
    rinex_summary = write_gps_l1_rinex(rinex_path, epochs, selected_timestamps)

    receiver_path = args.output_dir / "receiver_wls.csv"
    reference_path = args.output_dir / "reference.csv"
    errors_h: list[float] = []
    errors_v: list[float] = []
    with receiver_path.open("w", encoding="utf-8", newline="") as receiver_handle, reference_path.open(
        "w", encoding="utf-8", newline=""
    ) as reference_handle:
        receiver_writer = csv.writer(receiver_handle, lineterminator="\n")
        reference_writer = csv.writer(reference_handle, lineterminator="\n")
        receiver_writer.writerow(("utc_time_millis", "latitude_deg", "longitude_deg", "height_m", "source"))
        reference_writer.writerow(("utc_time_millis", "latitude_deg", "longitude_deg", "height_m", "source"))
        for timestamp in selected_timestamps:
            lat, lon, height = ecef_to_geodetic(*receiver_positions[timestamp])
            receiver_writer.writerow((timestamp, f"{lat:.12f}", f"{lon:.12f}", f"{height:.4f}", "device_wls_ecef"))
            if timestamp in truth:
                truth_lat, truth_lon, truth_height = truth[timestamp]
                reference_writer.writerow((timestamp, truth_lat, truth_lon, truth_height, "independent_ground_truth"))
                errors_h.append(horizontal_error_m(lat, lon, truth_lat, truth_lon))
                errors_v.append(abs(height - truth_height))

    transitions = sum(current != previous for previous, current in zip(clock_counts, clock_counts[1:]))
    summary = {
        "schema_version": SCHEMA_VERSION,
        "decision": "adapter-pass" if not missing_truth else "inventory-only",
        "dataset": {
            "id": args.dataset_id,
            "role": args.role,
            "device_model": args.device_model,
            "source_url": args.source_url,
            "source_terms": args.source_terms,
        },
        "inputs": {
            "device_gnss": {"path": str(args.device_gnss), "sha256": sha256(args.device_gnss)},
            "ground_truth": {"path": str(args.ground_truth), "sha256": sha256(args.ground_truth)},
        },
        "observations": {
            "rows": len(selected_rows),
            "epochs": len(selected_timestamps),
            "input_epochs": len(epochs),
            "explicitly_skipped_epochs": args.skip_epochs,
            "first_utc_time_millis": selected_timestamps[0],
            "last_utc_time_millis": selected_timestamps[-1],
            "duration_seconds": (selected_timestamps[-1] - selected_timestamps[0]) / 1000.0,
            "signal_rows": dict(sorted(signals.items())),
            "unmapped_signal_rows": unmapped_signal_rows,
            "unmapped_signal_policy": "preserved-but-excluded-from-solver-input",
            "constellation_rows": dict(sorted(constellations.items())),
            "hardware_clock_discontinuity_transitions": transitions,
            "hardware_clock_discontinuity_counts": sorted(set(clock_counts)),
        },
        "truth": {
            "matched_epochs": len(errors_h),
            "missing_epochs": len(missing_truth),
            "coverage_ratio": len(errors_h) / len(selected_timestamps),
            "independent": True,
        },
        "device_wls_score": {
            "horizontal_median_m": median(errors_h) if errors_h else None,
            "horizontal_p95_m": percentile(errors_h, 0.95),
            "vertical_median_abs_m": median(errors_v) if errors_v else None,
            "accuracy_claim": "baseline-only",
        },
        "native_observation_adapter": rinex_summary,
        "artifacts": {
            "normalized_observations": str(normalized_path),
            "receiver_wls": str(receiver_path),
            "reference": str(reference_path),
            "rinex_observations": str(rinex_path),
        },
    }
    summary_path = args.output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Smartphone GNSS adapter complete: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
