#!/usr/bin/env python3
"""Adapt a frozen MARS-LVIG ROS1 bag into auditable R6 flight inputs."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timedelta
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from typing import Iterable


SCHEMA_VERSION = "uav-mars-adapter.v1"
GPS_EPOCH = datetime(1980, 1, 6)
ROSBAG_MAGIC = b"#ROSBAG V2.0\n"
MCAP_MAGIC = b"\x89MCAP0\r\n"
FREQUENCY_TOLERANCE_HZ = 2_000.0
SYSTEMS = (
    (1, 32, "G", 0),
    (33, 59, "R", 32),
    (60, 97, "E", 59),
    (98, 160, "C", 97),
)
BANDS = {
    "G": ((1_575_420_000.0, "1C"), (1_227_600_000.0, "2S")),
    "E": ((1_575_420_000.0, "1C"), (1_207_140_000.0, "7Q")),
    "C": ((1_561_098_000.0, "2I"), (1_207_140_000.0, "7I")),
}


def fail(message: str) -> "NoReturn":
    raise SystemExit(message)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path, label: str) -> dict[str, object]:
    if not path.is_file():
        fail(f"missing {label}: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        fail(f"invalid {label}: {exc}")
    if not isinstance(payload, dict):
        fail(f"{label} must contain a JSON object")
    return payload


def finite(value: object, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        fail(f"{label} must be numeric")
    if not math.isfinite(number):
        fail(f"{label} must be finite")
    return number


def timestamp_ns_from_header(message: object, label: str) -> int:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        fail(f"{label} has no ROS header timestamp")
    seconds = int(getattr(stamp, "sec", -1))
    nanoseconds = int(getattr(stamp, "nanosec", -1))
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        fail(f"{label} has an invalid ROS timestamp")
    return seconds * 1_000_000_000 + nanoseconds


def satellite_identifier(sat: int) -> tuple[str, int]:
    for lower, upper, system, offset in SYSTEMS:
        if lower <= sat <= upper:
            return system, sat - offset
    fail(f"unsupported gnss_comm satellite id: {sat}")


def signal_band(system: str, frequency: float) -> str | None:
    if system == "R":
        if 1_598_062_500.0 <= frequency <= 1_605_375_000.0:
            return "1C"
        if 1_242_937_500.0 <= frequency <= 1_248_625_000.0:
            return "2C"
        return None
    for nominal, band in BANDS.get(system, ()):
        if abs(frequency - nominal) <= FREQUENCY_TOLERANCE_HZ:
            return band
    return None


def rinex_header_line(content: str, label: str) -> str:
    return f"{content[:60]:<60}{label}\n"


def rinex_field(value: float | None, lli: int = 0) -> str:
    if value is None:
        return " " * 16
    return f"{value:14.3f}{lli if lli else ' '} "


def ecef_from_geodetic(latitude_deg: float, longitude_deg: float, height_m: float) -> tuple[float, float, float]:
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    semi_major = 6_378_137.0
    eccentricity_squared = 6.6943799901413165e-3
    sin_latitude = math.sin(latitude)
    prime_vertical = semi_major / math.sqrt(1.0 - eccentricity_squared * sin_latitude**2)
    return (
        (prime_vertical + height_m) * math.cos(latitude) * math.cos(longitude),
        (prime_vertical + height_m) * math.cos(latitude) * math.sin(longitude),
        (prime_vertical * (1.0 - eccentricity_squared) + height_m) * sin_latitude,
    )


def choose_connection(connections: Iterable[object], topic: str | None, suffix: str, label: str) -> object:
    candidates = [connection for connection in connections if str(connection.msgtype).endswith(suffix)]
    if topic is not None:
        candidates = [connection for connection in candidates if connection.topic == topic]
    if len(candidates) != 1:
        names = ", ".join(f"{item.topic} ({item.msgtype})" for item in candidates) or "none"
        fail(f"expected exactly one {label} topic, found: {names}; pass an explicit topic")
    return candidates[0]


def write_rinex_observations(
    path: Path,
    messages: list[object],
    approximate_position: tuple[float, float, float],
) -> dict[str, object]:
    if not messages:
        fail("raw GNSS measurement topic is empty")
    mapped_signals: dict[str, int] = {}
    unmapped_signals: dict[str, int] = {}
    previous_time: tuple[int, float] | None = None
    with path.open("w", encoding="ascii", newline="") as output:
        output.write(rinex_header_line("     3.04           OBSERVATION DATA    M", "RINEX VERSION / TYPE"))
        output.write(rinex_header_line("gnss uav-mars-adapt libgnss++", "PGM / RUN BY / DATE"))
        output.write(rinex_header_line("MARS-LVIG gnss_comm raw observations; source values preserved", "COMMENT"))
        output.write(
            rinex_header_line(
                "".join(f"{coordinate:14.4f}" for coordinate in approximate_position),
                "APPROX POSITION XYZ",
            )
        )
        for system, bands in (
            ("G", ("1C", "2S")),
            ("R", ("1C", "2C")),
            ("E", ("1C", "7Q")),
            ("C", ("2I", "7I")),
        ):
            types = [f"{kind}{band}" for band in bands for kind in "CLDS"]
            output.write(rinex_header_line(f"{system}  {len(types):3d} " + " ".join(types), "SYS / # / OBS TYPES"))
        output.write(rinex_header_line("GPS", "TIME SYSTEM ID"))
        output.write(rinex_header_line("", "END OF HEADER"))
        written_epochs = 0
        written_satellites = 0
        for epoch_index, message in enumerate(messages):
            observations = list(getattr(message, "meas", ()))
            if not observations:
                fail(f"GNSS epoch {epoch_index} contains no observations")
            week = int(getattr(observations[0].time, "week"))
            tow = finite(getattr(observations[0].time, "tow"), f"GNSS epoch {epoch_index} TOW")
            if week <= 0 or not 0.0 <= tow < 604_800.0:
                fail(f"GNSS epoch {epoch_index} has invalid GPST week/TOW")
            if previous_time is not None and (week, tow) <= previous_time:
                fail(f"GNSS epoch {epoch_index} time is not strictly increasing")
            previous_time = (week, tow)
            converted: list[tuple[str, list[str]]] = []
            seen_satellites: set[str] = set()
            for observation in observations:
                obs_week = int(getattr(observation.time, "week"))
                obs_tow = finite(getattr(observation.time, "tow"), "observation TOW")
                if obs_week != week or abs(obs_tow - tow) > 1e-3:
                    fail(f"GNSS epoch {epoch_index} contains inconsistent observation times")
                system, prn = satellite_identifier(int(observation.sat))
                identifier = f"{system}{prn:02d}"
                if identifier in seen_satellites:
                    fail(f"GNSS epoch {epoch_index} contains duplicate satellite {identifier}")
                seen_satellites.add(identifier)
                arrays = {
                    name: list(getattr(observation, name, ()))
                    for name in ("freqs", "CN0", "LLI", "psr", "cp", "dopp", "status")
                }
                lengths = {len(values) for values in arrays.values()}
                if len(lengths) != 1:
                    fail(f"GNSS epoch {epoch_index} {identifier} has inconsistent signal arrays")
                by_band: dict[str, tuple[float | None, float | None, float | None, float | None, int]] = {}
                for signal_index, frequency_value in enumerate(arrays["freqs"]):
                    frequency = finite(frequency_value, "GNSS signal frequency")
                    band = signal_band(system, frequency)
                    key = f"{system}:{frequency:.3f}"
                    if band is None:
                        unmapped_signals[key] = unmapped_signals.get(key, 0) + 1
                        continue
                    if band in by_band:
                        fail(f"GNSS epoch {epoch_index} {identifier} has duplicate mapped band {band}")
                    status = int(arrays["status"][signal_index])
                    pseudorange = finite(arrays["psr"][signal_index], "pseudorange") if status & 0x01 else None
                    carrier = finite(arrays["cp"][signal_index], "carrier phase") if status & 0x02 else None
                    doppler = finite(arrays["dopp"][signal_index], "Doppler")
                    cn0 = finite(arrays["CN0"][signal_index], "C/N0")
                    lli = int(arrays["LLI"][signal_index])
                    by_band[band] = (pseudorange, carrier, doppler, cn0, lli)
                    mapped_key = f"{system}:{band}"
                    mapped_signals[mapped_key] = mapped_signals.get(mapped_key, 0) + 1
                bands = {"G": ("1C", "2S"), "R": ("1C", "2C"), "E": ("1C", "7Q"), "C": ("2I", "7I")}[system]
                fields: list[str] = []
                for band in bands:
                    values = by_band.get(band)
                    if values is None:
                        fields.extend([" " * 16] * 4)
                    else:
                        pseudorange, carrier, doppler, cn0, lli = values
                        fields.extend(
                            (
                                rinex_field(pseudorange),
                                rinex_field(carrier, lli),
                                rinex_field(doppler),
                                rinex_field(cn0),
                            )
                        )
                if any(field.strip() for field in fields):
                    converted.append((identifier, fields))
            if not converted:
                fail(f"GNSS epoch {epoch_index} has no supported observations")
            stamp = GPS_EPOCH + timedelta(weeks=week, seconds=tow)
            seconds = stamp.second + stamp.microsecond / 1e6
            output.write(
                f"> {stamp.year:04d} {stamp.month:02d} {stamp.day:02d} {stamp.hour:02d} "
                f"{stamp.minute:02d} {seconds:011.7f}  0{len(converted):3d}\n"
            )
            for identifier, fields in sorted(converted):
                output.write(identifier + "".join(fields) + "\n")
            written_epochs += 1
            written_satellites += len(converted)
    return {
        "path": str(path),
        "sha256": sha256(path),
        "epochs": written_epochs,
        "satellite_records": written_satellites,
        "mapped_signal_rows": mapped_signals,
        "unmapped_signal_rows": unmapped_signals,
        "time_system": "GPST",
        "initial_position_source": "raw_receiver_pvt_first_valid_epoch",
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--acquire-summary", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--role", choices=("development", "holdout"), required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--measurement-topic")
    parser.add_argument("--pvt-topic")
    parser.add_argument("--imu-topic")
    parser.add_argument("--truth-topic")
    parser.add_argument("--attitude-topic")
    parser.add_argument("--rtk-yaw-topic")
    parser.add_argument("--rtk-position-status-topic")
    parser.add_argument("--rtk-yaw-status-topic")
    parser.add_argument("--max-epochs", type=int, default=-1)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.max_epochs == 0 or args.max_epochs < -1:
        fail("--max-epochs must be -1 or a positive integer")
    profile = load_json(args.profile, "R6 profile")
    if profile.get("schema_version") != "uav-r6-profile.v1":
        fail("R6 profile schema is not uav-r6-profile.v1")
    acquire = load_json(args.acquire_summary, "acquire summary")
    if acquire.get("schema_version") != "uav-mars-acquire.v1":
        fail("acquire summary schema is not uav-mars-acquire.v1")
    acquire_dataset = dict(acquire.get("dataset", {}))
    if acquire_dataset.get("role") != args.role:
        fail("acquire summary role does not match --role")
    datasets = profile.get("datasets")
    expected_dataset = dict(datasets.get(args.role, {})) if isinstance(datasets, dict) else {}
    if acquire_dataset.get("id") != expected_dataset.get("id"):
        fail("acquire summary dataset does not match the frozen R6 profile")
    bag_contract = dict(acquire.get("bag", {}))
    bag_bytes = args.bag.stat().st_size
    bag_sha256 = sha256(args.bag)
    if bag_bytes != int(bag_contract.get("bytes", -1)) or bag_sha256 != bag_contract.get("sha256"):
        fail("bag size/hash does not match acquire summary")
    container = str(expected_dataset.get("container", "ros1_bag"))
    expected_magic = {"ros1_bag": ROSBAG_MAGIC, "mcap": MCAP_MAGIC}.get(container)
    if expected_magic is None:
        fail(f"unsupported R6 container: {container}")
    with args.bag.open("rb") as handle:
        if handle.read(len(expected_magic)) != expected_magic:
            fail(f"input is not the frozen {container} container")
    frame_contract = profile.get("frame_contract")
    if not isinstance(frame_contract, dict) or frame_contract.get("vehicle_nhc") is not False:
        fail("R6 frame contract must explicitly disable vehicle NHC")
    lever_keys = ("raw_gnss_antenna_phase_center_in_lidar_m", "rtk_truth_antenna_phase_center_in_lidar_m")
    for key in lever_keys:
        vector = frame_contract.get(key)
        if not isinstance(vector, list) or len(vector) != 3 or not all(math.isfinite(float(value)) for value in vector):
            fail(f"R6 frame contract has invalid {key}")
    topic_contract = profile.get("topic_contract")
    if not isinstance(topic_contract, dict):
        fail("R6 profile has no topic_contract")
    args.measurement_topic = args.measurement_topic or str(topic_contract.get("measurement", ""))
    args.pvt_topic = args.pvt_topic or str(topic_contract.get("raw_receiver_pvt", ""))
    args.imu_topic = args.imu_topic or str(topic_contract.get("imu", ""))
    args.truth_topic = args.truth_topic or str(topic_contract.get("rtk_truth", ""))
    args.attitude_topic = args.attitude_topic or str(topic_contract.get("attitude", ""))
    args.rtk_yaw_topic = args.rtk_yaw_topic or str(topic_contract.get("rtk_yaw", ""))
    args.rtk_position_status_topic = args.rtk_position_status_topic or str(topic_contract.get("rtk_position_status", ""))
    args.rtk_yaw_status_topic = args.rtk_yaw_status_topic or str(topic_contract.get("rtk_yaw_status", ""))
    if not all(
        (
            args.measurement_topic,
            args.pvt_topic,
            args.imu_topic,
            args.truth_topic,
            args.attitude_topic,
            args.rtk_yaw_topic,
            args.rtk_position_status_topic,
            args.rtk_yaw_status_topic,
        )
    ):
        fail("R6 topic contract is incomplete")
    try:
        from rosbags.highlevel import AnyReader
    except ImportError:
        fail("uav-mars-adapter requires the Python 'rosbags' package")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with AnyReader([args.bag]) as reader:
        connections = list(reader.connections)
        inventory = [
            {"topic": item.topic, "message_type": item.msgtype, "message_count": item.msgcount}
            for item in connections
        ]
        actual_topic_counts = {item.topic: item.msgcount for item in connections}
        expected_topic_counts = expected_dataset.get("expected_topics")
        if not isinstance(expected_topic_counts, dict):
            fail(f"R6 {args.role} dataset has no expected_topics contract")
        for topic, expected_count in expected_topic_counts.items():
            if actual_topic_counts.get(str(topic)) != int(expected_count):
                fail(
                    f"topic count mismatch for {topic}: expected {expected_count}, "
                    f"got {actual_topic_counts.get(str(topic))}"
                )
        measurement_connection = choose_connection(connections, args.measurement_topic, "GnssMeasMsg", "raw GNSS measurement")
        pvt_connection = choose_connection(connections, args.pvt_topic, "GnssPVTSolnMsg", "raw receiver PVT")
        imu_connection = choose_connection(connections, args.imu_topic, "sensor_msgs/msg/Imu", "IMU")
        truth_connection = choose_connection(connections, args.truth_topic, "sensor_msgs/msg/NavSatFix", "RTK truth")
        attitude_connection = choose_connection(connections, args.attitude_topic, "geometry_msgs/msg/QuaternionStamped", "attitude")
        yaw_connection = choose_connection(connections, args.rtk_yaw_topic, "std_msgs/msg/Int16", "RTK yaw")
        position_status_connection = choose_connection(
            connections, args.rtk_position_status_topic, "std_msgs/msg/UInt8", "RTK position status"
        )
        yaw_status_connection = choose_connection(
            connections, args.rtk_yaw_status_topic, "std_msgs/msg/UInt8", "RTK yaw status"
        )
        selected = [
            measurement_connection, pvt_connection, imu_connection, truth_connection,
            attitude_connection, yaw_connection, position_status_connection, yaw_status_connection,
        ]
        decoded: dict[int, list[tuple[int, object]]] = {connection.id: [] for connection in selected}
        for connection, bag_timestamp, rawdata in reader.messages(connections=selected):
            if connection.id == measurement_connection.id and args.max_epochs > 0 and len(decoded[connection.id]) >= args.max_epochs:
                break
            decoded[connection.id].append((bag_timestamp, reader.deserialize(rawdata, connection.msgtype)))
    pvt_messages = [message for _, message in decoded[pvt_connection.id]]
    valid_pvt = next(
        (
            message
            for message in pvt_messages
            if bool(getattr(message, "valid_fix", False))
            and all(
                math.isfinite(float(getattr(message, field)))
                for field in ("latitude", "longitude", "altitude")
            )
        ),
        None,
    )
    if valid_pvt is None:
        fail("raw receiver PVT topic has no valid finite fix for solver initialization")
    approximate_position = ecef_from_geodetic(valid_pvt.latitude, valid_pvt.longitude, valid_pvt.altitude)
    observations_path = args.output_dir / "uav_observations.rnx"
    observations = write_rinex_observations(
        observations_path,
        [message for _, message in decoded[measurement_connection.id]],
        approximate_position,
    )

    imu_path = args.output_dir / "imu.csv"
    with imu_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("unix_time_ns", "frame_id", "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"))
        previous = -1
        for index, (_, message) in enumerate(decoded[imu_connection.id]):
            stamp = timestamp_ns_from_header(message, f"IMU message {index}")
            frame_id = str(message.header.frame_id).strip()
            if not frame_id:
                fail(f"IMU message {index} has an undefined frame_id")
            if stamp <= previous:
                fail(f"IMU message {index} timestamp is not strictly increasing")
            previous = stamp
            acceleration = message.linear_acceleration
            angular = message.angular_velocity
            values = tuple(finite(value, f"IMU message {index}") for value in (acceleration.x, acceleration.y, acceleration.z, angular.x, angular.y, angular.z))
            writer.writerow((stamp, frame_id, *values))

    truth_path = args.output_dir / "rtk_truth.csv"
    with truth_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("unix_time_ns", "frame_id", "latitude_deg", "longitude_deg", "ellipsoidal_height_m", "status"))
        previous = -1
        for index, (_, message) in enumerate(decoded[truth_connection.id]):
            stamp = timestamp_ns_from_header(message, f"truth message {index}")
            if stamp <= previous:
                fail(f"truth message {index} timestamp is not strictly increasing")
            previous = stamp
            frame_id = str(message.header.frame_id).strip()
            if not frame_id:
                frame_id = str(frame_contract.get("rtk_truth_frame", "")).strip()
            if not frame_id:
                fail(f"truth message {index} has an undefined frame_id and the profile supplies no semantic frame")
            values = tuple(finite(getattr(message, field), f"truth message {index} {field}") for field in ("latitude", "longitude", "altitude"))
            writer.writerow((stamp, frame_id, *values, int(message.status.status)))

    attitude_path = args.output_dir / "attitude.csv"
    with attitude_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("unix_time_ns", "frame_id", "qx", "qy", "qz", "qw"))
        previous = -1
        for index, (_, message) in enumerate(decoded[attitude_connection.id]):
            stamp = timestamp_ns_from_header(message, f"attitude message {index}")
            if stamp <= previous:
                fail(f"attitude message {index} timestamp is not strictly increasing")
            previous = stamp
            frame_id = str(message.header.frame_id).strip()
            if not frame_id:
                fail(f"attitude message {index} has an undefined frame_id")
            quaternion = message.quaternion
            values = tuple(finite(value, f"attitude message {index}") for value in (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
            norm = math.sqrt(sum(value * value for value in values))
            if abs(norm - 1.0) > 1e-3:
                fail(f"attitude message {index} quaternion norm is {norm}, expected 1")
            writer.writerow((stamp, frame_id, *values))

    yaw_path = args.output_dir / "rtk_yaw.csv"
    with yaw_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("unix_time_ns", "yaw_degrees", "timestamp_source"))
        previous = -1
        for index, (bag_timestamp, message) in enumerate(decoded[yaw_connection.id]):
            stamp = int(bag_timestamp)
            if stamp <= previous:
                fail(f"RTK yaw message {index} bag timestamp is not strictly increasing")
            previous = stamp
            yaw_degrees = int(message.data)
            if not 0 <= yaw_degrees < 360:
                fail(f"RTK yaw message {index} is outside the observed/documented degree range")
            writer.writerow((stamp, yaw_degrees, "rosbag_record_time"))

    status_path = args.output_dir / "rtk_status.csv"
    with status_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("kind", "unix_time_ns", "status", "timestamp_source"))
        for kind, connection in (("position", position_status_connection), ("yaw", yaw_status_connection)):
            previous = -1
            for index, (bag_timestamp, message) in enumerate(decoded[connection.id]):
                stamp = int(bag_timestamp)
                if stamp <= previous:
                    fail(f"RTK {kind} status message {index} bag timestamp is not strictly increasing")
                previous = stamp
                writer.writerow((kind, stamp, int(message.data), "rosbag_record_time"))

    artifacts = {}
    for label, path in (
        ("imu", imu_path), ("truth", truth_path), ("attitude", attitude_path),
        ("rtk_yaw", yaw_path), ("rtk_status", status_path),
    ):
        artifacts[label] = {"path": str(path), "sha256": sha256(path), "rows": sum(1 for _ in path.open(encoding="utf-8")) - 1}
        if artifacts[label]["rows"] <= 0:
            fail(f"{label} export is empty")
    payload = {
        "schema_version": SCHEMA_VERSION,
        "dataset": {"id": acquire_dataset.get("id"), "role": args.role},
        "profile": {"path": str(args.profile), "sha256": sha256(args.profile)},
        "acquire_summary": {"path": str(args.acquire_summary), "sha256": sha256(args.acquire_summary)},
        "source_bag": {"path": str(args.bag), "bytes": bag_bytes, "sha256": bag_sha256, "container": container},
        "topics": {
            "measurement": measurement_connection.topic,
            "raw_receiver_pvt": pvt_connection.topic,
            "imu": imu_connection.topic,
            "rtk_truth": truth_connection.topic,
            "attitude": attitude_connection.topic,
            "rtk_yaw": yaw_connection.topic,
            "rtk_position_status": position_status_connection.topic,
            "rtk_yaw_status": yaw_status_connection.topic,
        },
        "topic_inventory": inventory,
        "observations": observations,
        "artifacts": artifacts,
        "frame_contract": frame_contract,
        "vehicle_nhc": False,
        "bounded": args.max_epochs > 0,
        "command": [os.environ.get("GNSS_CLI_NAME", "uav-mars-adapter"), *sys.argv[1:]],
    }
    summary = args.output_dir / "adapter_summary.json"
    summary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
