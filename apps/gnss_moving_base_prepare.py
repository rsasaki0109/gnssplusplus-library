#!/usr/bin/env python3
"""Prepare replayable moving-base artifacts from ROS2 bags with u-blox topics."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import shutil
import struct
import sys
import tempfile
from typing import Iterable
import zipfile


ROOT_DIR = Path(__file__).resolve().parent.parent
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:  # pragma: no cover - exercised only on non-ROS systems.
    rosbag2_py = None
    deserialize_message = None
    get_message = None


@dataclass
class RawxEpoch:
    week: int
    tow_s: float


@dataclass
class NavPvtSample:
    i_tow_ms: int
    year: int
    month: int
    day: int
    lat_deg: float
    lon_deg: float
    height_m: float


@dataclass
class NavRelPosSample:
    i_tow_ms: int
    north_m: float
    east_m: float
    down_m: float
    heading_deg: float | None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--input", type=Path, required=True, help="ROS2 bag directory or Zenodo zip.")
    parser.add_argument(
        "--rover-ubx-out",
        type=Path,
        default=ROOT_DIR / "output/moving_base_rover.ubx",
        help="Output UBX file for rover NAV-PVT/RXM-RAWX.",
    )
    parser.add_argument(
        "--base-ubx-out",
        type=Path,
        default=ROOT_DIR / "output/moving_base_base.ubx",
        help="Output UBX file for base NAV-PVT/RXM-RAWX.",
    )
    parser.add_argument(
        "--reference-csv",
        type=Path,
        default=ROOT_DIR / "output/moving_base_reference.csv",
        help="Output reference CSV for gnss moving-base-signoff.",
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=ROOT_DIR / "output/moving_base_prepare_summary.json",
        help="Output JSON summary.",
    )
    parser.add_argument("--max-epochs", type=int, default=-1, help="Optional RAWX epoch limit per receiver.")
    parser.add_argument(
        "--match-tolerance-ms",
        type=int,
        default=250,
        help="Maximum iTOW mismatch when matching NavPVT/NavRELPOSNED to RAWX epochs.",
    )
    parser.add_argument("--rover-rawx-topic", default="/rover/rxmrawx")
    parser.add_argument("--base-rawx-topic", default="/base/rxmrawx")
    parser.add_argument("--rover-navpvt-topic", default="/rover/navpvt")
    parser.add_argument("--base-navpvt-topic", default="/base/navpvt")
    parser.add_argument("--rover-relpos-topic", default="/rover/navrelposned")
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress non-summary prints.",
    )
    return parser.parse_args()


def require_ros2_support() -> None:
    if rosbag2_py is None or deserialize_message is None or get_message is None:
        raise SystemExit(
            "moving-base-prepare requires ROS2 Python support "
            "(rosbag2_py, rclpy, rosidl_runtime_py, ublox_msgs)."
        )


def ensure_input_exists(path: Path) -> None:
    if not path.exists():
        raise SystemExit(f"Missing moving-base bag input: {path}")


def gps_epoch_date_from_rawx(epoch: RawxEpoch) -> str:
    # GPS epoch is 1980-01-06, but the exact day is easier to recover from NavPVT.
    # This helper is used only as a fallback.
    import datetime as dt

    gps_epoch = dt.datetime(1980, 1, 6)
    value = gps_epoch + dt.timedelta(weeks=epoch.week, seconds=epoch.tow_s)
    return value.date().isoformat()


def geodetic_to_ecef(lat_deg: float, lon_deg: float, height_m: float) -> tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + height_m) * cos_lat * cos_lon
    y = (n + height_m) * cos_lat * sin_lon
    z = (n * (1.0 - WGS84_E2) + height_m) * sin_lat
    return x, y, z


def ned_to_ecef_delta(north_m: float, east_m: float, down_m: float, lat_deg: float, lon_deg: float) -> tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    up_m = -down_m
    dx = -sin_lat * cos_lon * north_m - sin_lon * east_m + cos_lat * cos_lon * up_m
    dy = -sin_lat * sin_lon * north_m + cos_lon * east_m + cos_lat * sin_lon * up_m
    dz = cos_lat * north_m + sin_lat * up_m
    return dx, dy, dz


def ubx_checksum(body: bytes) -> tuple[int, int]:
    ck_a = 0
    ck_b = 0
    for value in body:
        ck_a = (ck_a + value) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def build_ubx_frame(message_class: int, message_id: int, payload: bytes) -> bytes:
    body = bytes((message_class & 0xFF, message_id & 0xFF)) + struct.pack("<H", len(payload)) + payload
    ck_a, ck_b = ubx_checksum(body)
    return b"\xB5\x62" + body + bytes((ck_a, ck_b))


def clamp_u8(value: int) -> int:
    return int(value) & 0xFF


def pack_nav_pvt(message: object) -> bytes:
    payload = bytearray(92)
    struct.pack_into("<I", payload, 0, int(message.i_tow))
    struct.pack_into("<H", payload, 4, int(message.year))
    payload[6] = clamp_u8(message.month)
    payload[7] = clamp_u8(message.day)
    payload[8] = clamp_u8(message.hour)
    payload[9] = clamp_u8(message.min)
    payload[10] = clamp_u8(message.sec)
    payload[11] = clamp_u8(message.valid)
    struct.pack_into("<I", payload, 12, int(message.t_acc))
    struct.pack_into("<i", payload, 16, int(message.nano))
    payload[20] = clamp_u8(message.fix_type)
    payload[21] = clamp_u8(message.flags)
    payload[22] = clamp_u8(message.flags2)
    payload[23] = clamp_u8(message.num_sv)
    struct.pack_into("<i", payload, 24, int(message.lon))
    struct.pack_into("<i", payload, 28, int(message.lat))
    struct.pack_into("<i", payload, 32, int(message.height))
    struct.pack_into("<i", payload, 36, int(message.h_msl))
    struct.pack_into("<I", payload, 40, int(message.h_acc))
    struct.pack_into("<I", payload, 44, int(message.v_acc))
    struct.pack_into("<i", payload, 48, int(message.vel_n))
    struct.pack_into("<i", payload, 52, int(message.vel_e))
    struct.pack_into("<i", payload, 56, int(message.vel_d))
    struct.pack_into("<i", payload, 60, int(message.g_speed))
    struct.pack_into("<i", payload, 64, int(message.heading))
    struct.pack_into("<I", payload, 68, int(message.s_acc))
    struct.pack_into("<I", payload, 72, int(message.head_acc))
    struct.pack_into("<H", payload, 76, int(message.p_dop))
    payload[78:84] = bytes(int(value) & 0xFF for value in message.reserved1)
    struct.pack_into("<i", payload, 84, int(message.head_veh))
    struct.pack_into("<h", payload, 88, int(message.mag_dec))
    struct.pack_into("<H", payload, 90, int(message.mag_acc))
    return build_ubx_frame(0x01, 0x07, bytes(payload))


def pack_rawx(message: object) -> bytes:
    measurements = list(message.meas)
    payload = bytearray(16 + 32 * len(measurements))
    struct.pack_into(
        "<dHbBBB",
        payload,
        0,
        float(message.rcv_tow),
        int(message.week),
        int(message.leap_s),
        len(measurements),
        clamp_u8(message.rec_stat),
        clamp_u8(message.version),
    )
    payload[14:16] = bytes(int(value) & 0xFF for value in message.reserved1)
    for index, meas in enumerate(measurements):
        base = 16 + index * 32
        struct.pack_into(
            "<ddfBBBBHBBBBBB",
            payload,
            base,
            float(meas.pr_mes),
            float(meas.cp_mes),
            float(meas.do_mes),
            clamp_u8(meas.gnss_id),
            clamp_u8(meas.sv_id),
            clamp_u8(meas.reserved0),
            clamp_u8(meas.freq_id),
            int(meas.locktime),
            clamp_u8(meas.cno),
            clamp_u8(meas.pr_stdev),
            clamp_u8(meas.cp_stdev),
            clamp_u8(meas.do_stdev),
            clamp_u8(meas.trk_stat),
            clamp_u8(meas.reserved1),
        )
    return build_ubx_frame(0x02, 0x15, bytes(payload))


def navpvt_sample(message: object) -> NavPvtSample:
    return NavPvtSample(
        i_tow_ms=int(message.i_tow),
        year=int(message.year),
        month=int(message.month),
        day=int(message.day),
        lat_deg=float(message.lat) * 1e-7,
        lon_deg=float(message.lon) * 1e-7,
        height_m=float(message.height) * 1e-3,
    )


def relpos_sample(message: object) -> NavRelPosSample:
    north_m = float(message.rel_pos_n) * 1e-2 + float(message.rel_pos_hpn) * 1e-4
    east_m = float(message.rel_pos_e) * 1e-2 + float(message.rel_pos_hpe) * 1e-4
    down_m = float(message.rel_pos_d) * 1e-2 + float(message.rel_pos_hpd) * 1e-4
    heading_deg = float(message.rel_pos_heading) * 1e-5 if int(message.rel_pos_heading) != 0 else None
    return NavRelPosSample(
        i_tow_ms=int(message.i_tow),
        north_m=north_m,
        east_m=east_m,
        down_m=down_m,
        heading_deg=heading_deg,
    )


def rawx_epoch(message: object) -> RawxEpoch:
    return RawxEpoch(week=int(message.week), tow_s=float(message.rcv_tow))


def nearest_sample_index(samples: list[object], target_i_tow_ms: int, tolerance_ms: int, key) -> int | None:
    best_index: int | None = None
    best_delta: int | None = None
    for index, sample in enumerate(samples):
        delta = abs(int(key(sample)) - target_i_tow_ms)
        if best_delta is None or delta < best_delta:
            best_delta = delta
            best_index = index
    if best_delta is None or best_delta > tolerance_ms:
        return None
    return best_index


class PreparedBag:
    def __init__(self, input_path: Path) -> None:
        self.input_path = input_path
        self._temp_dir: tempfile.TemporaryDirectory[str] | None = None
        self.bag_dir = self._resolve_bag_dir(input_path)

    def _resolve_bag_dir(self, input_path: Path) -> Path:
        if input_path.is_dir():
            metadata = input_path / "metadata.yaml"
            if metadata.exists():
                return input_path
            for child in input_path.iterdir():
                if child.is_dir() and (child / "metadata.yaml").exists():
                    return child
            raise SystemExit(f"No ROS2 bag directory with metadata.yaml found under {input_path}")
        if input_path.suffix.lower() != ".zip":
            raise SystemExit(f"Unsupported moving-base bag input: {input_path}")
        self._temp_dir = tempfile.TemporaryDirectory(prefix="gnss_moving_base_bag_")
        with zipfile.ZipFile(input_path) as archive:
            archive.extractall(self._temp_dir.name)
        temp_root = Path(self._temp_dir.name)
        for child in temp_root.iterdir():
            if child.is_dir() and (child / "metadata.yaml").exists():
                return child
        raise SystemExit(f"Failed to extract a ROS2 bag directory from {input_path}")

    def close(self) -> None:
        if self._temp_dir is not None:
            self._temp_dir.cleanup()
            self._temp_dir = None


def collect_messages(args: argparse.Namespace, bag_dir: Path) -> tuple[bytes, bytes, list[RawxEpoch], list[NavPvtSample], list[NavRelPosSample], dict[str, int], str | None]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    topic_types = {meta.name: meta.type for meta in reader.get_all_topics_and_types()}
    wanted_topics = {
        args.rover_rawx_topic,
        args.base_rawx_topic,
        args.rover_navpvt_topic,
        args.base_navpvt_topic,
        args.rover_relpos_topic,
    }
    missing = [topic for topic in wanted_topics if topic not in topic_types]
    if missing:
        raise SystemExit(f"Missing required ROS2 bag topics: {', '.join(sorted(missing))}")

    topic_messages = {topic: get_message(topic_types[topic]) for topic in wanted_topics}
    counts = {topic: 0 for topic in wanted_topics}
    rover_frames = bytearray()
    base_frames = bytearray()
    rover_epochs: list[RawxEpoch] = []
    base_nav: list[NavPvtSample] = []
    relpos: list[NavRelPosSample] = []
    date_text: str | None = None

    rover_rawx_count = 0
    base_rawx_count = 0

    while reader.has_next():
        topic, data, _timestamp = reader.read_next()
        if topic not in wanted_topics:
            continue
        if args.max_epochs >= 0:
            if topic == args.rover_rawx_topic and rover_rawx_count >= args.max_epochs:
                continue
            if topic == args.base_rawx_topic and base_rawx_count >= args.max_epochs:
                continue
        message = deserialize_message(data, topic_messages[topic])
        counts[topic] += 1
        if topic == args.rover_navpvt_topic:
            rover_frames.extend(pack_nav_pvt(message))
            sample = navpvt_sample(message)
            if date_text is None:
                date_text = f"{sample.year:04d}-{sample.month:02d}-{sample.day:02d}"
        elif topic == args.base_navpvt_topic:
            base_frames.extend(pack_nav_pvt(message))
            sample = navpvt_sample(message)
            base_nav.append(sample)
            if date_text is None:
                date_text = f"{sample.year:04d}-{sample.month:02d}-{sample.day:02d}"
        elif topic == args.rover_rawx_topic:
            rover_frames.extend(pack_rawx(message))
            rover_epochs.append(rawx_epoch(message))
            rover_rawx_count += 1
        elif topic == args.base_rawx_topic:
            base_frames.extend(pack_rawx(message))
            base_rawx_count += 1
        elif topic == args.rover_relpos_topic:
            relpos.append(relpos_sample(message))

    if not rover_epochs:
        raise SystemExit(f"No RAWX epochs found on {args.rover_rawx_topic}")
    if not base_nav:
        raise SystemExit(f"No NavPVT messages found on {args.base_navpvt_topic}")
    if not relpos:
        raise SystemExit(f"No NavRELPOSNED messages found on {args.rover_relpos_topic}")
    return bytes(rover_frames), bytes(base_frames), rover_epochs, base_nav, relpos, counts, date_text


def write_reference_csv(
    rover_epochs: list[RawxEpoch],
    base_nav: list[NavPvtSample],
    relpos: list[NavRelPosSample],
    tolerance_ms: int,
    output_path: Path,
) -> dict[str, object]:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    matched_rows = 0
    skipped_no_base = 0
    skipped_no_rel = 0
    first_week = rover_epochs[0].week
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "gps_week",
                "gps_tow_s",
                "base_ecef_x_m",
                "base_ecef_y_m",
                "base_ecef_z_m",
                "rover_ecef_x_m",
                "rover_ecef_y_m",
                "rover_ecef_z_m",
                "baseline_n_m",
                "baseline_e_m",
                "baseline_d_m",
                "heading_deg",
            ]
        )
        for epoch in rover_epochs:
            target_i_tow_ms = int(round(epoch.tow_s * 1000.0))
            base_index = nearest_sample_index(base_nav, target_i_tow_ms, tolerance_ms, lambda item: item.i_tow_ms)
            if base_index is None:
                skipped_no_base += 1
                continue
            rel_index = nearest_sample_index(relpos, target_i_tow_ms, tolerance_ms, lambda item: item.i_tow_ms)
            if rel_index is None:
                skipped_no_rel += 1
                continue
            base = base_nav[base_index]
            rel = relpos[rel_index]
            base_x, base_y, base_z = geodetic_to_ecef(base.lat_deg, base.lon_deg, base.height_m)
            dx, dy, dz = ned_to_ecef_delta(rel.north_m, rel.east_m, rel.down_m, base.lat_deg, base.lon_deg)
            writer.writerow(
                [
                    first_week,
                    f"{epoch.tow_s:.3f}",
                    f"{base_x:.6f}",
                    f"{base_y:.6f}",
                    f"{base_z:.6f}",
                    f"{base_x + dx:.6f}",
                    f"{base_y + dy:.6f}",
                    f"{base_z + dz:.6f}",
                    f"{rel.north_m:.4f}",
                    f"{rel.east_m:.4f}",
                    f"{rel.down_m:.4f}",
                    "" if rel.heading_deg is None else f"{rel.heading_deg:.5f}",
                ]
            )
            matched_rows += 1

    if matched_rows == 0:
        raise SystemExit("Failed to match rover RAWX epochs to base NavPVT / NavRELPOSNED reference rows")
    return {
        "matched_reference_rows": matched_rows,
        "skipped_missing_base_nav": skipped_no_base,
        "skipped_missing_relpos": skipped_no_rel,
    }


def main() -> int:
    args = parse_args()
    require_ros2_support()
    ensure_input_exists(args.input)
    prepared_bag = PreparedBag(args.input)
    try:
        rover_frames, base_frames, rover_epochs, base_nav, relpos, counts, date_text = collect_messages(args, prepared_bag.bag_dir)
    finally:
        prepared_bag.close()

    args.rover_ubx_out.parent.mkdir(parents=True, exist_ok=True)
    args.base_ubx_out.parent.mkdir(parents=True, exist_ok=True)
    args.rover_ubx_out.write_bytes(rover_frames)
    args.base_ubx_out.write_bytes(base_frames)

    reference_metrics = write_reference_csv(
        rover_epochs,
        base_nav,
        relpos,
        args.match_tolerance_ms,
        args.reference_csv,
    )

    summary = {
        "input": str(args.input),
        "bag_dir": str(prepared_bag.bag_dir),
        "rover_ubx_out": str(args.rover_ubx_out),
        "base_ubx_out": str(args.base_ubx_out),
        "reference_csv": str(args.reference_csv),
        "summary_json": str(args.summary_json),
        "gps_week": rover_epochs[0].week,
        "date": date_text or gps_epoch_date_from_rawx(rover_epochs[0]),
        "rover_epochs": len(rover_epochs),
        "rover_ubx_bytes": len(rover_frames),
        "base_ubx_bytes": len(base_frames),
        "topics": counts,
        **reference_metrics,
    }
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    if not args.quiet:
        print("Prepared moving-base bag artifacts.")
        print(f"  rover_ubx: {args.rover_ubx_out}")
        print(f"  base_ubx: {args.base_ubx_out}")
        print(f"  reference_csv: {args.reference_csv}")
        print(f"  summary_json: {args.summary_json}")
        print(f"  rover_epochs: {summary['rover_epochs']}")
        print(f"  matched_reference_rows: {summary['matched_reference_rows']}")
        print(f"  date: {summary['date']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
