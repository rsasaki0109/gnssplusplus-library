#!/usr/bin/env python3
"""CLI regression tests for the dispatcher-backed native tools."""

from __future__ import annotations

import os
import binascii
import csv
import json
import socket
import gzip
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import math
import sqlite3
import zlib
import zipfile
import http.server
import importlib.util
from functools import partial
from pathlib import Path
from urllib import request

if os.name != "nt":
    import pty


ROOT_DIR = Path(__file__).resolve().parents[2]
DISPATCHER = ROOT_DIR / "apps" / "gnss.py"
SCRIPTS_DIR = ROOT_DIR / "scripts"

STATIC_DATA_FILES = (
    "data/rover_static.obs",
    "data/navigation_static.nav",
)
SHORT_BASELINE_DATA_FILES = (
    "data/short_baseline/TSK200JPN_R_20240010000_01D_30S_MO.rnx",
    "data/short_baseline/TSKB00JPN_R_20240010000_01D_30S_MO.rnx",
    "data/short_baseline/BRDC00IGS_R_20240010000_01D_MN.rnx",
)
KINEMATIC_DATA_FILES = (
    "data/rover_kinematic.obs",
    "data/base_kinematic.obs",
    "data/navigation_kinematic.nav",
)
ODAIBA_DATA_FILES = (
    "data/driving/Tokyo_Data/Odaiba/rover_trimble.obs",
    "data/driving/Tokyo_Data/Odaiba/base_trimble.obs",
    "data/driving/Tokyo_Data/Odaiba/base.nav",
    "data/driving/Tokyo_Data/Odaiba/reference.csv",
)
DEFAULT_PPC_DATASET_ROOT = Path("/tmp/PPC-Dataset-data/PPC-Dataset")
DEFAULT_RTKLIB_BIN = Path("rtklib_v2_ws/RTKLIB/rnx2rtkp")

sys.path.insert(0, str(SCRIPTS_DIR))

import generate_driving_comparison as driving_comparison  # noqa: E402


def repo_data_exists(*relative_paths: str) -> bool:
    return all((ROOT_DIR / relative_path).exists() for relative_path in relative_paths)


def ros2_bag_support_available() -> bool:
    required = ("rosbag2_py", "rclpy", "rosidl_runtime_py", "ublox_msgs")
    return all(importlib.util.find_spec(name) is not None for name in required)


def build_synthetic_sqlite_rosbag(
    bag_dir: Path,
    *,
    include_raw_binary: bool = True,
    include_raw: bool = True,
    include_fix: bool = True,
) -> None:
    bag_dir.mkdir(parents=True, exist_ok=True)
    db_path = bag_dir / "synthetic_0.db3"
    topic_specs = []
    if include_raw_binary:
        topic_specs.append(("/gnss/raw_binary", "std_msgs/msg/UInt8MultiArray", [1_000_000_000, 1_200_000_000, 1_400_000_000]))
    if include_raw:
        topic_specs.append(("/gnss/raw", "gnss_raw_driver/msg/GnssRawEpoch", [1_000_000_000, 1_200_000_000, 1_400_000_000]))
    if include_fix:
        topic_specs.append(("/gnss/fix", "sensor_msgs/msg/NavSatFix", [1_000_000_000, 1_200_000_000, 4_000_000_000]))

    with sqlite3.connect(db_path) as connection:
        connection.execute(
            "CREATE TABLE topics("
            "id INTEGER PRIMARY KEY, "
            "name TEXT NOT NULL, "
            "type TEXT NOT NULL, "
            "serialization_format TEXT NOT NULL, "
            "offered_qos_profiles TEXT NOT NULL)"
        )
        connection.execute(
            "CREATE TABLE messages("
            "id INTEGER PRIMARY KEY, "
            "topic_id INTEGER NOT NULL, "
            "timestamp INTEGER NOT NULL, "
            "data BLOB NOT NULL)"
        )
        message_id = 1
        for topic_id, (name, msg_type, timestamps) in enumerate(topic_specs, start=1):
            connection.execute(
                "INSERT INTO topics(id, name, type, serialization_format, offered_qos_profiles) VALUES (?, ?, ?, ?, ?)",
                (topic_id, name, msg_type, "cdr", ""),
            )
            for timestamp in timestamps:
                connection.execute(
                    "INSERT INTO messages(id, topic_id, timestamp, data) VALUES (?, ?, ?, ?)",
                    (message_id, topic_id, timestamp, b"\x00\x01\x02"),
                )
                message_id += 1
        connection.commit()

    bag_dir.joinpath("metadata.yaml").write_text(
        "\n".join(
            [
                "rosbag2_bagfile_information:",
                "  version: 5",
                "  storage_identifier: sqlite3",
                f"  message_count: {message_id - 1}",
                "  duration:",
                "    nanoseconds: 3000000000",
                "  starting_time:",
                "    nanoseconds_since_epoch: 1000000000",
            ]
        )
        + "\n",
        encoding="utf-8",
    )


def build_synthetic_mcap_metadata_rosbag(bag_dir: Path) -> None:
    bag_dir.mkdir(parents=True, exist_ok=True)
    bag_dir.joinpath("synthetic_0.mcap").write_bytes(b"\x89MCAP\r\n")
    bag_dir.joinpath("metadata.yaml").write_text(
        "\n".join(
            [
                "rosbag2_bagfile_information:",
                "  version: 8",
                "  storage_identifier: mcap",
                "  relative_file_paths:",
                "    - synthetic_0.mcap",
                "  duration:",
                "    nanoseconds: 4000000000",
                "  starting_time:",
                "    nanoseconds_since_epoch: 1000000000",
                "  message_count: 7",
                "  topics_with_message_count:",
                "    - topic_metadata:",
                "        name: /gnss/raw_binary",
                "        type: std_msgs/msg/UInt8MultiArray",
                "        serialization_format: cdr",
                "        offered_qos_profiles: \"\"",
                "      message_count: 2",
                "    - topic_metadata:",
                "        name: /gnss/raw",
                "        type: gnss_raw_driver/msg/GnssRawEpoch",
                "        serialization_format: cdr",
                "        offered_qos_profiles: \"\"",
                "      message_count: 2",
                "    - topic_metadata:",
                "        name: /gnss/fix",
                "        type: sensor_msgs/msg/NavSatFix",
                "        serialization_format: cdr",
                "        offered_qos_profiles: \"\"",
                "      message_count: 3",
            ]
        )
        + "\n",
        encoding="utf-8",
    )


def write_fake_mcap_reader_package(package_root: Path) -> None:
    mcap_dir = package_root / "mcap"
    mcap_dir.mkdir(parents=True, exist_ok=True)
    mcap_dir.joinpath("__init__.py").write_text("", encoding="utf-8")
    mcap_dir.joinpath("reader.py").write_text(
        "\n".join(
            [
                "import json",
                "from types import SimpleNamespace",
                "",
                "class FakeReader:",
                "    def __init__(self, stream):",
                "        self._messages = json.loads(stream.read().decode('utf-8'))",
                "",
                "    def iter_messages(self, topics=None, start_time=None, end_time=None, log_time_order=True, reverse=False):",
                "        messages = list(self._messages)",
                "        if topics is not None:",
                "            wanted = {topics} if isinstance(topics, str) else set(topics)",
                "            messages = [item for item in messages if item['topic'] in wanted]",
                "        if log_time_order:",
                "            messages.sort(key=lambda item: item['log_time'], reverse=reverse)",
                "        for item in messages:",
                "            log_time = item['log_time']",
                "            if start_time is not None and log_time < start_time:",
                "                continue",
                "            if end_time is not None and log_time >= end_time:",
                "                continue",
                "            schema = SimpleNamespace(name=item.get('schema')) if item.get('schema') else None",
                "            channel = SimpleNamespace(topic=item['topic'], message_encoding=item.get('encoding', 'cdr'))",
                "            message = SimpleNamespace(log_time=log_time, data=b'x' * int(item.get('data_size', 0)))",
                "            yield schema, channel, message",
                "",
                "def make_reader(stream, *args, **kwargs):",
                "    return FakeReader(stream)",
            ]
        )
        + "\n",
        encoding="utf-8",
    )


def write_fake_mcap_messages(bag_dir: Path) -> None:
    messages = [
        {"topic": "/gnss/raw_binary", "schema": "std_msgs/msg/UInt8MultiArray", "log_time": 1_000_000_000, "data_size": 12},
        {"topic": "/gnss/raw", "schema": "gnss_raw_driver/msg/GnssRawEpoch", "log_time": 1_000_000_000, "data_size": 32},
        {"topic": "/gnss/fix", "schema": "sensor_msgs/msg/NavSatFix", "log_time": 1_000_000_000, "data_size": 48},
        {"topic": "/gnss/raw_binary", "schema": "std_msgs/msg/UInt8MultiArray", "log_time": 1_200_000_000, "data_size": 12},
        {"topic": "/gnss/raw", "schema": "gnss_raw_driver/msg/GnssRawEpoch", "log_time": 1_200_000_000, "data_size": 32},
        {"topic": "/gnss/fix", "schema": "sensor_msgs/msg/NavSatFix", "log_time": 1_200_000_000, "data_size": 48},
        {"topic": "/gnss/fix", "schema": "sensor_msgs/msg/NavSatFix", "log_time": 4_000_000_000, "data_size": 48},
    ]
    bag_dir.joinpath("synthetic_0.mcap").write_text(json.dumps(messages), encoding="utf-8")


def write_pty_payload(
    master_fd: int,
    payloads: list[bytes],
    *,
    initial_delay_s: float,
    between_delay_s: float,
    final_delay_s: float,
) -> None:
    try:
        time.sleep(initial_delay_s)
        for index, payload in enumerate(payloads):
            try:
                os.write(master_fd, payload)
            except OSError:
                break
            if index + 1 < len(payloads):
                time.sleep(between_delay_s)
        time.sleep(final_delay_s)
    finally:
        try:
            os.close(master_fd)
        except OSError:
            pass


def build_synthetic_moving_base_rosbag(bag_dir: Path) -> None:
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from ublox_msgs.msg import NavPVT, NavRELPOSNED9, RxmRAWX, RxmRAWXMeas

    bag_dir.parent.mkdir(parents=True, exist_ok=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topics = {
        "/rover/navpvt": "ublox_msgs/msg/NavPVT",
        "/base/navpvt": "ublox_msgs/msg/NavPVT",
        "/rover/rxmrawx": "ublox_msgs/msg/RxmRAWX",
        "/base/rxmrawx": "ublox_msgs/msg/RxmRAWX",
        "/rover/navrelposned": "ublox_msgs/msg/NavRELPOSNED9",
    }
    for topic, msg_type in topics.items():
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                0,
                name=topic,
                type=msg_type,
                serialization_format="cdr",
                offered_qos_profiles=[],
            )
        )

    def make_navpvt(itow_ms: int, lat_deg: float, lon_deg: float, height_m: float) -> NavPVT:
        msg = NavPVT()
        msg.i_tow = itow_ms
        msg.year = 2023
        msg.month = 6
        msg.day = 14
        msg.hour = 12
        msg.min = 0
        msg.sec = int((itow_ms // 1000) % 60)
        msg.valid = 0x37
        msg.fix_type = 3
        msg.flags = 0x83
        msg.flags2 = 0
        msg.num_sv = 18
        msg.lon = int(round(lon_deg * 1e7))
        msg.lat = int(round(lat_deg * 1e7))
        msg.height = int(round(height_m * 1000.0))
        msg.h_msl = msg.height
        msg.h_acc = 100
        msg.v_acc = 100
        msg.s_acc = 50
        msg.head_acc = 100000
        msg.p_dop = 150
        return msg

    def make_rawx(itow_s: float, week: int, sv_id: int, sig_id: int = 0) -> RxmRAWX:
        meas = RxmRAWXMeas()
        meas.pr_mes = 20200000.0 + sv_id * 1000.0
        meas.cp_mes = 110000000.0 + sv_id * 10000.0
        meas.do_mes = -1200.0 + sv_id
        meas.gnss_id = 0
        meas.sv_id = sv_id
        meas.reserved0 = sig_id
        meas.freq_id = 0
        meas.locktime = 100
        meas.cno = 45
        meas.pr_stdev = 3
        meas.cp_stdev = 1
        meas.do_stdev = 5
        meas.trk_stat = 0x03
        meas.reserved1 = 0

        msg = RxmRAWX()
        msg.rcv_tow = itow_s
        msg.week = week
        msg.leap_s = 18
        msg.num_meas = 1
        msg.rec_stat = 1
        msg.version = 1
        msg.reserved1 = [0, 0]
        msg.meas = [meas]
        return msg

    def make_relpos(itow_ms: int, north_m: float, east_m: float, down_m: float) -> NavRELPOSNED9:
        msg = NavRELPOSNED9()
        msg.version = 1
        msg.i_tow = itow_ms
        msg.rel_pos_n = int(math.trunc(north_m * 100.0))
        msg.rel_pos_e = int(math.trunc(east_m * 100.0))
        msg.rel_pos_d = int(math.trunc(down_m * 100.0))
        msg.rel_pos_hpn = int(round((north_m * 10000.0) - msg.rel_pos_n * 100.0))
        msg.rel_pos_hpe = int(round((east_m * 10000.0) - msg.rel_pos_e * 100.0))
        msg.rel_pos_hpd = int(round((down_m * 10000.0) - msg.rel_pos_d * 100.0))
        msg.rel_pos_length = int(round(math.sqrt(north_m * north_m + east_m * east_m + down_m * down_m) * 100.0))
        msg.rel_pos_hp_length = 0
        msg.rel_pos_heading = int(round((math.degrees(math.atan2(east_m, north_m)) % 360.0) * 1e5))
        msg.acc_n = 100
        msg.acc_e = 100
        msg.acc_d = 100
        msg.acc_length = 100
        msg.acc_heading = 100
        msg.flags = 1
        return msg

    epochs = [
        (2200, 345600.000, 35.0000000, 139.0000000, 50.0, 1.2, 2.3, -0.4),
        (2200, 345601.000, 35.0000003, 139.0000002, 50.1, 1.4, 2.6, -0.5),
    ]
    timestamp_ns = 1_000_000_000
    for week, tow_s, base_lat, base_lon, base_h, north_m, east_m, down_m in epochs:
        itow_ms = int(round(tow_s * 1000.0))
        writer.write("/base/navpvt", serialize_message(make_navpvt(itow_ms, base_lat, base_lon, base_h)), timestamp_ns)
        writer.write(
            "/rover/navpvt",
            serialize_message(make_navpvt(itow_ms, base_lat + 1e-6, base_lon + 1e-6, base_h)),
            timestamp_ns + 1,
        )
        writer.write("/rover/navrelposned", serialize_message(make_relpos(itow_ms, north_m, east_m, down_m)), timestamp_ns + 2)
        writer.write("/base/rxmrawx", serialize_message(make_rawx(tow_s, week, 3)), timestamp_ns + 3)
        writer.write("/rover/rxmrawx", serialize_message(make_rawx(tow_s, week, 3)), timestamp_ns + 4)
        timestamp_ns += 1_000_000_000


def ppc_dataset_root() -> Path:
    env_value = os.environ.get("GNSSPP_PPC_DATASET_ROOT")
    if env_value:
        return Path(env_value)
    return DEFAULT_PPC_DATASET_ROOT


def rtklib_binary() -> Path:
    env_value = os.environ.get("GNSSPP_RTKLIB_BIN")
    if env_value:
        return Path(env_value)
    return DEFAULT_RTKLIB_BIN


def find_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def wait_for_file(path: Path, timeout_s: float = 5.0) -> str:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if path.exists():
            return path.read_text(encoding="utf-8").strip()
        time.sleep(0.05)
    raise TimeoutError(f"timed out waiting for {path}")


class QuietSimpleHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, format: str, *args: object) -> None:
        del format, args


def start_static_http_server(directory: Path) -> tuple[http.server.ThreadingHTTPServer, threading.Thread, int]:
    handler = partial(QuietSimpleHTTPRequestHandler, directory=str(directory))
    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread, int(server.server_address[1])


def crc24q(data: bytes) -> int:
    table = (
        0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC, 0x9F7F17,
        0xA18139, 0x27CDC2, 0x2B5434, 0xAD18CF, 0x3267D8, 0xB42B23, 0xB8B2D5, 0x3EFE2E,
        0xC54E89, 0x430272, 0x4F9B84, 0xC9D77F, 0x56A868, 0xD0E493, 0xDC7D65, 0x5A319E,
        0x64CFB0, 0xE2834B, 0xEE1ABD, 0x685646, 0xF72951, 0x7165AA, 0x7DFC5C, 0xFBB0A7,
        0x0CD1E9, 0x8A9D12, 0x8604E4, 0x00481F, 0x9F3708, 0x197BF3, 0x15E205, 0x93AEFE,
        0xAD50D0, 0x2B1C2B, 0x2785DD, 0xA1C926, 0x3EB631, 0xB8FACA, 0xB4633C, 0x322FC7,
        0xC99F60, 0x4FD39B, 0x434A6D, 0xC50696, 0x5A7981, 0xDC357A, 0xD0AC8C, 0x56E077,
        0x681E59, 0xEE52A2, 0xE2CB54, 0x6487AF, 0xFBF8B8, 0x7DB443, 0x712DB5, 0xF7614E,
        0x19A3D2, 0x9FEF29, 0x9376DF, 0x153A24, 0x8A4533, 0x0C09C8, 0x00903E, 0x86DCC5,
        0xB822EB, 0x3E6E10, 0x32F7E6, 0xB4BB1D, 0x2BC40A, 0xAD88F1, 0xA11107, 0x275DFC,
        0xDCED5B, 0x5AA1A0, 0x563856, 0xD074AD, 0x4F0BBA, 0xC94741, 0xC5DEB7, 0x43924C,
        0x7D6C62, 0xFB2099, 0xF7B96F, 0x71F594, 0xEE8A83, 0x68C678, 0x645F8E, 0xE21375,
        0x15723B, 0x933EC0, 0x9FA736, 0x19EBCD, 0x8694DA, 0x00D821, 0x0C41D7, 0x8A0D2C,
        0xB4F302, 0x32BFF9, 0x3E260F, 0xB86AF4, 0x2715E3, 0xA15918, 0xADC0EE, 0x2B8C15,
        0xD03CB2, 0x567049, 0x5AE9BF, 0xDCA544, 0x43DA53, 0xC596A8, 0xC90F5E, 0x4F43A5,
        0x71BD8B, 0xF7F170, 0xFB6886, 0x7D247D, 0xE25B6A, 0x641791, 0x688E67, 0xEEC29C,
        0x3347A4, 0xB50B5F, 0xB992A9, 0x3FDE52, 0xA0A145, 0x26EDBE, 0x2A7448, 0xAC38B3,
        0x92C69D, 0x148A66, 0x181390, 0x9E5F6B, 0x01207C, 0x876C87, 0x8BF571, 0x0DB98A,
        0xF6092D, 0x7045D6, 0x7CDC20, 0xFA90DB, 0x65EFCC, 0xE3A337, 0xEF3AC1, 0x69763A,
        0x578814, 0xD1C4EF, 0xDD5D19, 0x5B11E2, 0xC46EF5, 0x42220E, 0x4EBBF8, 0xC8F703,
        0x3F964D, 0xB9DAB6, 0xB54340, 0x330FBB, 0xAC70AC, 0x2A3C57, 0x26A5A1, 0xA0E95A,
        0x9E1774, 0x185B8F, 0x14C279, 0x928E82, 0x0DF195, 0x8BBD6E, 0x872498, 0x016863,
        0xFAD8C4, 0x7C943F, 0x700DC9, 0xF64132, 0x693E25, 0xEF72DE, 0xE3EB28, 0x65A7D3,
        0x5B59FD, 0xDD1506, 0xD18CF0, 0x57C00B, 0xC8BF1C, 0x4EF3E7, 0x426A11, 0xC426EA,
        0x2AE476, 0xACA88D, 0xA0317B, 0x267D80, 0xB90297, 0x3F4E6C, 0x33D79A, 0xB59B61,
        0x8B654F, 0x0D29B4, 0x01B042, 0x87FCB9, 0x1883AE, 0x9ECF55, 0x9256A3, 0x141A58,
        0xEFAAFF, 0x69E604, 0x657FF2, 0xE33309, 0x7C4C1E, 0xFA00E5, 0xF69913, 0x70D5E8,
        0x4E2BC6, 0xC8673D, 0xC4FECB, 0x42B230, 0xDDCD27, 0x5B81DC, 0x57182A, 0xD154D1,
        0x26359F, 0xA07964, 0xACE092, 0x2AAC69, 0xB5D37E, 0x339F85, 0x3F0673, 0xB94A88,
        0x87B4A6, 0x01F85D, 0x0D61AB, 0x8B2D50, 0x145247, 0x921EBC, 0x9E874A, 0x18CBB1,
        0xE37B16, 0x6537ED, 0x69AE1B, 0xEFE2E0, 0x709DF7, 0xF6D10C, 0xFA48FA, 0x7C0401,
        0x42FA2F, 0xC4B6D4, 0xC82F22, 0x4E63D9, 0xD11CCE, 0x575035, 0x5BC9C3, 0xDD8538,
    )
    crc = 0
    for byte in data:
        index = ((crc >> 16) ^ byte) & 0xFF
        crc = ((crc << 8) ^ table[index]) & 0xFFFFFFFF
    return crc & 0xFFFFFF


def set_unsigned_bits(payload: bytearray, pos: int, length: int, value: int) -> None:
    for i in range(length):
        bit_index = pos + length - 1 - i
        byte_index = bit_index // 8
        bit_in_byte = 7 - (bit_index % 8)
        mask = 1 << bit_in_byte
        if (value >> i) & 1:
            payload[byte_index] |= mask
        else:
            payload[byte_index] &= ~mask & 0xFF


def set_signed_bits(payload: bytearray, pos: int, length: int, value: int) -> None:
    masked = value & ((1 << length) - 1)
    set_unsigned_bits(payload, pos, length, masked)


def set_sign_magnitude_bits(payload: bytearray, pos: int, length: int, value: int) -> None:
    magnitude_bits = length - 1
    magnitude = abs(value)
    if magnitude >= (1 << magnitude_bits):
        raise ValueError(f"value {value} does not fit in {length} sign-magnitude bits")
    set_unsigned_bits(payload, pos, 1, 1 if value < 0 else 0)
    set_unsigned_bits(payload, pos + 1, magnitude_bits, magnitude)


def read_unsigned_bits(payload: bytes | bytearray, pos: int, length: int) -> int:
    value = 0
    for offset in range(length):
        absolute_bit = pos + offset
        byte_index = absolute_bit // 8
        bit_in_byte = 7 - (absolute_bit % 8)
        value = (value << 1) | ((payload[byte_index] >> bit_in_byte) & 1)
    return value


def build_rtcm1005(x_m: float, y_m: float, z_m: float) -> bytes:
    payload = bytearray(19)
    bit = 0
    set_unsigned_bits(payload, bit, 12, 1005)
    bit += 12
    set_unsigned_bits(payload, bit, 12, 42)
    bit += 12
    set_unsigned_bits(payload, bit, 6, 0)
    bit += 6
    for _ in range(4):
        set_unsigned_bits(payload, bit, 1, 1)
        bit += 1
    set_signed_bits(payload, bit, 38, round(x_m * 10000.0))
    bit += 38
    set_unsigned_bits(payload, bit, 1, 0)
    bit += 1
    set_unsigned_bits(payload, bit, 1, 0)
    bit += 1
    set_signed_bits(payload, bit, 38, round(y_m * 10000.0))
    bit += 38
    set_unsigned_bits(payload, bit, 2, 0)
    bit += 2
    set_signed_bits(payload, bit, 38, round(z_m * 10000.0))

    frame = bytearray([0xD3, 0x00, len(payload)])
    frame.extend(payload)
    crc = crc24q(frame)
    frame.extend(((crc >> 16) & 0xFF, (crc >> 8) & 0xFF, crc & 0xFF))
    return bytes(frame)


def build_rtcm1060(prn: int, tow_seconds: int) -> bytes:
    total_bits = 68 + 205
    payload = bytearray((total_bits + 7) // 8)
    bit = 0
    set_unsigned_bits(payload, bit, 12, 1060)
    bit += 12
    set_unsigned_bits(payload, bit, 20, tow_seconds)
    bit += 20
    set_unsigned_bits(payload, bit, 4, 2)
    bit += 4
    set_unsigned_bits(payload, bit, 1, 0)
    bit += 1
    set_unsigned_bits(payload, bit, 1, 1)
    bit += 1
    set_unsigned_bits(payload, bit, 4, 7)
    bit += 4
    set_unsigned_bits(payload, bit, 16, 21)
    bit += 16
    set_unsigned_bits(payload, bit, 4, 3)
    bit += 4
    set_unsigned_bits(payload, bit, 6, 1)
    bit += 6
    set_unsigned_bits(payload, bit, 6, prn)
    bit += 6
    set_unsigned_bits(payload, bit, 8, 0)
    bit += 8
    set_signed_bits(payload, bit, 22, 0)
    bit += 22
    set_signed_bits(payload, bit, 20, 0)
    bit += 20
    set_signed_bits(payload, bit, 20, 0)
    bit += 20
    set_signed_bits(payload, bit, 21, 0)
    bit += 21
    set_signed_bits(payload, bit, 19, 0)
    bit += 19
    set_signed_bits(payload, bit, 19, 0)
    bit += 19
    set_signed_bits(payload, bit, 22, 0)
    bit += 22
    set_signed_bits(payload, bit, 21, 0)
    bit += 21
    set_signed_bits(payload, bit, 27, 0)

    frame = bytearray([0xD3, 0x00, len(payload)])
    frame.extend(payload)
    crc = crc24q(frame)
    frame.extend(((crc >> 16) & 0xFF, (crc >> 8) & 0xFF, crc & 0xFF))
    return bytes(frame)


def build_rtcm1062(prn: int, tow_seconds: int, high_rate_units: int = 2500) -> bytes:
    total_bits = 67 + 28
    payload = bytearray((total_bits + 7) // 8)
    bit = 0
    set_unsigned_bits(payload, bit, 12, 1062)
    bit += 12
    set_unsigned_bits(payload, bit, 20, tow_seconds)
    bit += 20
    set_unsigned_bits(payload, bit, 4, 2)
    bit += 4
    set_unsigned_bits(payload, bit, 1, 0)
    bit += 1
    set_unsigned_bits(payload, bit, 4, 7)
    bit += 4
    set_unsigned_bits(payload, bit, 16, 21)
    bit += 16
    set_unsigned_bits(payload, bit, 4, 3)
    bit += 4
    set_unsigned_bits(payload, bit, 6, 1)
    bit += 6
    set_unsigned_bits(payload, bit, 6, prn)
    bit += 6
    set_signed_bits(payload, bit, 22, high_rate_units)

    frame = bytearray([0xD3, 0x00, len(payload)])
    frame.extend(payload)
    crc = crc24q(frame)
    frame.extend(((crc >> 16) & 0xFF, (crc >> 8) & 0xFF, crc & 0xFF))
    return bytes(frame)


def build_rtcm1059(prn: int, tow_seconds: int, signal_id: int = 2, bias_centimeters: int = -12) -> bytes:
    total_bits = 67 + 6 + 5 + 19
    payload = bytearray((total_bits + 7) // 8)
    bit = 0
    set_unsigned_bits(payload, bit, 12, 1059)
    bit += 12
    set_unsigned_bits(payload, bit, 20, tow_seconds)
    bit += 20
    set_unsigned_bits(payload, bit, 4, 2)
    bit += 4
    set_unsigned_bits(payload, bit, 1, 0)
    bit += 1
    set_unsigned_bits(payload, bit, 4, 7)
    bit += 4
    set_unsigned_bits(payload, bit, 16, 21)
    bit += 16
    set_unsigned_bits(payload, bit, 4, 3)
    bit += 4
    set_unsigned_bits(payload, bit, 6, 1)
    bit += 6
    set_unsigned_bits(payload, bit, 6, prn)
    bit += 6
    set_unsigned_bits(payload, bit, 5, 1)
    bit += 5
    set_unsigned_bits(payload, bit, 5, signal_id)
    bit += 5
    set_signed_bits(payload, bit, 14, bias_centimeters)

    frame = bytearray([0xD3, 0x00, len(payload)])
    frame.extend(payload)
    crc = crc24q(frame)
    frame.extend(((crc >> 16) & 0xFF, (crc >> 8) & 0xFF, crc & 0xFF))
    return bytes(frame)


def build_ubx_message(message_class: int, message_id: int, payload: bytes) -> bytes:
    message = bytearray([0xB5, 0x62, message_class, message_id])
    message.extend(struct.pack("<H", len(payload)))
    message.extend(payload)
    ck_a = 0
    ck_b = 0
    for byte in message[2:]:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    message.extend((ck_a, ck_b))
    return bytes(message)


def checksum8(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value & 0xFF


def build_binex_frame(record_id: int, payload: bytes) -> bytes:
    if len(payload) >= 128:
        raise ValueError("test BINEX helper only supports short payloads")
    checksum_input = bytes((record_id, len(payload))) + payload
    return bytes((0xE2, record_id, len(payload))) + payload + bytes((checksum8(checksum_input),))


def build_binex_metadata_frame() -> bytes:
    return build_binex_frame(0x00, bytes((0x08,)) + b"TSKB")


def build_binex_nav_frame() -> bytes:
    return build_binex_frame(0x01, bytes((0x06, 199)) + b"\xAA\x55\x00\x01")


def build_binex_proto_frame() -> bytes:
    return build_binex_frame(0x7F, bytes((0x05,)) + b"\x10\x20\x30\x40")


def build_nav_pvt_message() -> bytes:
    payload = bytearray(92)
    struct.pack_into("<I", payload, 0, 345600000)
    payload[20] = 3
    payload[21] = 0x01 | 0x02 | (0x02 << 6)
    payload[23] = 18
    struct.pack_into("<i", payload, 24, 1391234567)
    struct.pack_into("<i", payload, 28, 356543210)
    struct.pack_into("<i", payload, 32, 12345)
    struct.pack_into("<I", payload, 40, 1500)
    struct.pack_into("<I", payload, 44, 2300)
    return build_ubx_message(0x01, 0x07, payload)


def build_rawx_message(measurements=None) -> bytes:
    if measurements is None:
        measurements = [
            (20200000.25, 110000.5, -1234.5, 0, 12, 0, 500, 45, 0x03),
        ]

    payload = bytearray()
    payload.extend(struct.pack("<d", 345600.125))
    payload.extend(struct.pack("<H", 2200))
    payload.extend(bytes([18, len(measurements), 0x01, 0x01, 0x00, 0x00]))

    for pseudorange, carrier_phase, doppler, gnss_id, sv_id, sig_id, locktime, cno, trk_stat in measurements:
        payload.extend(struct.pack("<d", pseudorange))
        payload.extend(struct.pack("<d", carrier_phase))
        payload.extend(struct.pack("<f", doppler))
        payload.extend(bytes([gnss_id, sv_id, sig_id, 0]))
        payload.extend(struct.pack("<H", locktime))
        payload.extend(bytes([cno, 0, 0, 0, trk_stat, 0]))

    return build_ubx_message(0x02, 0x15, payload)


def build_mixed_rawx_message() -> bytes:
    return build_rawx_message(
        [
            (20200000.25, 110000.5, -1234.5, 0, 12, 0, 500, 45, 0x03),
            (21400000.75, 120000.25, -432.5, 2, 5, 0, 480, 42, 0x03),
            (22300000.50, 130000.75, 125.0, 6, 7, 2, 460, 41, 0x03),
            (23400000.00, 140000.125, -55.0, 3, 19, 0, 440, 40, 0x03),
            (24500000.25, 150000.875, 8.0, 5, 3, 0, 420, 39, 0x03),
        ]
    )


def build_sfrbx_message() -> bytes:
    payload = bytearray(
        [
            0x02,  # version
            0x03,  # numWords
            0x01,  # channel
            0x00,
            0x00,  # gnssId = GPS
            0x0C,  # svId = 12
            0x00,
            0x00,  # freqId
        ]
    )
    payload.extend(struct.pack("<I", 0x8B0000AA))
    payload.extend(struct.pack("<I", 0x00000500))
    payload.extend(struct.pack("<I", 0xCAFEBABE))
    return build_ubx_message(0x02, 0x13, payload)


def build_nmea_sentence(body: str) -> str:
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return f"${body}*{checksum:02X}\r\n"


def build_nmea_gga_sentence() -> str:
    return build_nmea_sentence(
        "GPGGA,123519,4807.038,N,01131.000,E,4,12,0.8,545.4,M,46.9,M,,"
    )


def build_nmea_rmc_sentence() -> str:
    return build_nmea_sentence(
        "GPRMC,123520,A,4807.038,N,01131.000,E,5.5,84.4,230394,,,A"
    )


def build_novatel_ascii_record(header: str, body: str) -> str:
    content = f"{header};{body}"
    checksum = zlib.crc32(content.encode("ascii")) & 0xFFFFFFFF
    return f"#{content}*{checksum:08X}\r\n"


def build_novatel_bestpos_record() -> str:
    header = "BESTPOSA,COM1,0,0.0,FINESTEERING,2200,345600.000,02000000,0000,0000"
    body = "SOL_COMPUTED,NARROW_INT,35.1234567,139.7654321,45.600,0.000,WGS84,0.010,0.020,0.030,\"\",0.0,0.0,18,18,18,18,00,00,00,00"
    return build_novatel_ascii_record(header, body)


def build_novatel_bestvel_record() -> str:
    header = "BESTVELA,COM1,0,0.0,FINESTEERING,2200,345600.000,02000000,0000,0000"
    body = "SOL_COMPUTED,DOPPLER_VELOCITY,0,0,5.500,84.400"
    return build_novatel_ascii_record(header, body)


def build_novatel_binary_frame(message_id: int, gps_week: int, gps_tow_ms: int, body: bytes) -> bytes:
    header = bytearray([0xAA, 0x44, 0x12, 0x1C])
    header.extend(struct.pack("<H", message_id))
    header.extend(struct.pack("<B", 0))
    header.extend(struct.pack("<B", 0))
    header.extend(struct.pack("<H", len(body)))
    header.extend(struct.pack("<H", 0))
    header.extend(struct.pack("<B", 0))
    header.extend(struct.pack("<B", 0))
    header.extend(struct.pack("<H", gps_week))
    header.extend(struct.pack("<I", gps_tow_ms))
    header.extend(struct.pack("<I", 0))
    header.extend(struct.pack("<H", 0))
    header.extend(struct.pack("<H", 0))
    frame = bytes(header) + body
    crc = zlib.crc32(frame) & 0xFFFFFFFF
    return frame + struct.pack("<I", crc)


def build_novatel_bestpos_binary_record() -> bytes:
    body = bytearray()
    body.extend(struct.pack("<I", 0))   # SOL_COMPUTED
    body.extend(struct.pack("<I", 50))  # NARROW_INT
    body.extend(struct.pack("<d", 35.1234567))
    body.extend(struct.pack("<d", 139.7654321))
    body.extend(struct.pack("<d", 45.6))
    body.extend(struct.pack("<f", 0.0))
    body.extend(struct.pack("<I", 61))
    body.extend(struct.pack("<f", 0.01))
    body.extend(struct.pack("<f", 0.02))
    body.extend(struct.pack("<f", 0.03))
    body.extend(b"0000")
    body.extend(struct.pack("<f", 0.0))
    body.extend(struct.pack("<f", 0.0))
    body.extend(bytes([18, 18, 18, 18, 0, 0, 0, 0]))
    return build_novatel_binary_frame(42, 2200, 345600000, bytes(body))


def build_novatel_bestvel_binary_record() -> bytes:
    body = bytearray()
    body.extend(struct.pack("<I", 0))   # SOL_COMPUTED
    body.extend(struct.pack("<I", 74))  # DOPPLER_VELOCITY
    body.extend(struct.pack("<f", 0.0))
    body.extend(struct.pack("<f", 0.0))
    body.extend(struct.pack("<d", 5.5))
    body.extend(struct.pack("<d", 84.4))
    body.extend(struct.pack("<d", 0.2))
    body.extend(struct.pack("<f", 0.0))
    return build_novatel_binary_frame(99, 2200, 345600000, bytes(body))


def build_sbp_frame(message_type: int, sender_id: int, payload: bytes) -> bytes:
    header = struct.pack("<HHB", message_type, sender_id, len(payload))
    crc = binascii.crc_hqx(header + payload, 0) & 0xFFFF
    return bytes([0x55]) + header + payload + struct.pack("<H", crc)


def build_sbp_gps_time_frame() -> bytes:
    payload = struct.pack("<IHiB", 345600123, 2200, -250, 0x01)
    return build_sbp_frame(0x0102, 66, payload)


def build_sbp_pos_llh_frame() -> bytes:
    payload = struct.pack(
        "<IdddHHBB",
        345600123,
        35.1234567,
        139.7654321,
        42.1,
        25,
        40,
        18,
        0x04,
    )
    return build_sbp_frame(0x020A, 66, payload)


def build_sbp_vel_ned_frame() -> bytes:
    payload = struct.pack(
        "<IiiiHHBB",
        345600123,
        1250,
        -500,
        125,
        50,
        75,
        18,
        0x04,
    )
    return build_sbp_frame(0x020E, 66, payload)


def build_sbf_frame(block_number: int, revision: int, payload: bytes) -> bytes:
    raw_id = (revision << 13) | (block_number & 0x1FFF)
    padded_payload = payload
    padding = (-((8 + len(payload)) % 4)) % 4
    if padding:
        padded_payload += bytes(padding)
    length = 8 + len(padded_payload)
    body = struct.pack("<HH", raw_id, length) + padded_payload
    crc = binascii.crc_hqx(body, 0) & 0xFFFF
    return b"$@" + struct.pack("<H", crc) + body


def build_sbf_pvt_geodetic_frame() -> bytes:
    payload = bytearray()
    payload.extend(struct.pack("<IH", 345600123, 2200))
    payload.extend(struct.pack("<BB", 10, 0))
    payload.extend(struct.pack("<ddd", math.radians(35.1234567), math.radians(139.7654321), 42.1))
    payload.extend(struct.pack("<f", 38.4))
    payload.extend(struct.pack("<f", 1.25))
    payload.extend(struct.pack("<f", -0.50))
    payload.extend(struct.pack("<f", 0.125))
    payload.extend(struct.pack("<f", 84.4))
    payload.extend(struct.pack("<d", 0.0))
    payload.extend(struct.pack("<f", 0.0))
    payload.extend(struct.pack("<BBBB", 0, 0, 18, 0))
    payload.extend(struct.pack("<HH", 65535, 0))
    payload.extend(struct.pack("<I", 0))
    payload.extend(struct.pack("<BB", 0, 0))
    payload.extend(struct.pack("<H", 0))
    payload.extend(struct.pack("<H", 0))
    payload.extend(struct.pack("<H", 25))
    payload.extend(struct.pack("<H", 40))
    payload.extend(struct.pack("<B", 0))
    return build_sbf_frame(4007, 2, bytes(payload))


def build_sbf_lband_tracker_frame() -> bytes:
    payload = bytearray()
    payload.extend(struct.pack("<IHBB", 345600123, 2200, 1, 24))
    payload.extend(
        struct.pack(
            "<IHHfHhbBBBHBx",
            1545260000,
            1200,
            42,
            1.25,
            4567,
            -123,
            -8,
            0,
            3,
            108,
            3600,
            1,
        )
    )
    return build_sbf_frame(4201, 3, bytes(payload))


def build_sbf_p2pp_status_frame() -> bytes:
    payload = struct.pack("<IHBBBBBB", 345600123, 2200, 1, 4, 1, 2, 2 << 1, 1)
    return build_sbf_frame(4238, 0, payload)


QZSS_L6_FRAME_BYTES = 250
QZSS_L6_HEADER_BITS = 49
QZSS_L6_DATA_PART_BITS = 1695
QZSS_L6_SUBFRAME_BITS = QZSS_L6_DATA_PART_BITS * 5


def build_qzss_l6_frame(
    *,
    prn: int = 199,
    facility_id: int = 0,
    subframe_start: bool = True,
    alert: bool = False,
    data_part: bytes = b"CLAS-L6-PAYLOAD",
) -> bytes:
    frame = bytearray(QZSS_L6_FRAME_BYTES)
    set_unsigned_bits(frame, 0, 32, 0x1ACFFC1D)
    set_unsigned_bits(frame, 32, 8, prn)
    message_type_id = (0b101 << 5) | ((facility_id & 0x3) << 3) | (0 << 1) | (1 if subframe_start else 0)
    set_unsigned_bits(frame, 40, 8, message_type_id)
    set_unsigned_bits(frame, 48, 1, 1 if alert else 0)
    for byte_index, value in enumerate(data_part):
        absolute_bit = QZSS_L6_HEADER_BITS + byte_index * 8
        if absolute_bit + 8 > QZSS_L6_HEADER_BITS + QZSS_L6_DATA_PART_BITS:
            break
        set_unsigned_bits(frame, absolute_bit, 8, value)
    return bytes(frame)


def copy_bits(
    source: bytes | bytearray,
    source_bit_offset: int,
    target: bytearray,
    target_bit_offset: int,
    bit_length: int,
) -> None:
    for bit_index in range(bit_length):
        value = read_unsigned_bits(source, source_bit_offset + bit_index, 1)
        set_unsigned_bits(target, target_bit_offset + bit_index, 1, value)


def encode_cssr_satellite_mask(prn: int, prn_base: int = 1) -> int:
    return 1 << (39 - (prn - prn_base))


def build_qzss_cssr_mask_message(
    *,
    tow: int,
    iod: int,
    prn: int = 3,
    prns: tuple[int, ...] | None = None,
    sync: bool = True,
    sigmask: int = 0x8000,
    system_id: int = 0,
    prn_base: int = 1,
) -> tuple[bytes, int]:
    satellites = prns if prns is not None else (prn,)
    signal_slots = [index for index in range(16) if ((sigmask >> (15 - index)) & 1) != 0]
    nsig = len(signal_slots)
    payload = bytearray(32)
    bit = 0
    set_unsigned_bits(payload, bit, 12, 4073)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 1)
    bit += 4
    set_unsigned_bits(payload, bit, 20, tow)
    bit += 20
    set_unsigned_bits(payload, bit, 4, 0)
    bit += 4
    set_unsigned_bits(payload, bit, 1, 1 if sync else 0)
    bit += 1
    set_unsigned_bits(payload, bit, 4, iod)
    bit += 4
    set_unsigned_bits(payload, bit, 4, 1)
    bit += 4
    set_unsigned_bits(payload, bit, 4, system_id)
    bit += 4
    svmask = 0
    for satellite_prn in satellites:
        svmask |= encode_cssr_satellite_mask(satellite_prn, prn_base)
    set_unsigned_bits(payload, bit, 40, svmask)
    bit += 40
    set_unsigned_bits(payload, bit, 16, sigmask)
    bit += 16
    set_unsigned_bits(payload, bit, 1, 1)
    bit += 1
    full_cellmask = (1 << nsig) - 1
    for _ in satellites:
        set_unsigned_bits(payload, bit, nsig, full_cellmask)
        bit += nsig
    return bytes(payload), bit


def build_qzss_cssr_combined_message(
    *,
    tow_delta: int,
    iod: int,
    prn: int = 3,
    sync: bool = False,
    network_id: int = 1,
    dx: float = 0.0,
    dy: float = 0.0,
    dz: float = 0.0,
    dclock_m: float = 0.025,
) -> tuple[bytes, int]:
    del prn
    payload = bytearray(32)
    bit = 0
    set_unsigned_bits(payload, bit, 12, 4073)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 11)
    bit += 4
    set_unsigned_bits(payload, bit, 12, tow_delta)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 0)
    bit += 4
    set_unsigned_bits(payload, bit, 1, 1 if sync else 0)
    bit += 1
    set_unsigned_bits(payload, bit, 4, iod)
    bit += 4
    set_unsigned_bits(payload, bit, 1, 1)
    bit += 1
    set_unsigned_bits(payload, bit, 1, 1)
    bit += 1
    set_unsigned_bits(payload, bit, 1, 1)
    bit += 1
    set_unsigned_bits(payload, bit, 5, network_id)
    bit += 5
    set_unsigned_bits(payload, bit, 1, 1)
    bit += 1
    set_unsigned_bits(payload, bit, 8, 12)
    bit += 8
    set_signed_bits(payload, bit, 15, round(dx / 0.0016))
    bit += 15
    set_signed_bits(payload, bit, 13, round(dy / 0.0064))
    bit += 13
    set_signed_bits(payload, bit, 13, round(dz / 0.0064))
    bit += 13
    set_signed_bits(payload, bit, 15, round(dclock_m / 0.0016))
    bit += 15
    return bytes(payload), bit


def build_qzss_cssr_orbit_message(
    *,
    tow_delta: int,
    iod: int,
    dx: float = 0.0,
    dy: float = 0.0,
    dz: float = 0.0,
    sync: bool = True,
) -> tuple[bytes, int]:
    payload = bytearray(32)
    bit = 0
    set_unsigned_bits(payload, bit, 12, 4073)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 2)
    bit += 4
    set_unsigned_bits(payload, bit, 12, tow_delta)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 0)
    bit += 4
    set_unsigned_bits(payload, bit, 1, 1 if sync else 0)
    bit += 1
    set_unsigned_bits(payload, bit, 4, iod)
    bit += 4
    set_unsigned_bits(payload, bit, 8, 12)
    bit += 8
    set_signed_bits(payload, bit, 15, round(dx / 0.0016))
    bit += 15
    set_signed_bits(payload, bit, 13, round(dy / 0.0064))
    bit += 13
    set_signed_bits(payload, bit, 13, round(dz / 0.0064))
    bit += 13
    return bytes(payload), bit


def build_qzss_cssr_clock_message(
    *,
    tow_delta: int,
    iod: int,
    dclock_m: float = 0.025,
    sync: bool = False,
) -> tuple[bytes, int]:
    payload = bytearray(32)
    bit = 0
    set_unsigned_bits(payload, bit, 12, 4073)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 3)
    bit += 4
    set_unsigned_bits(payload, bit, 12, tow_delta)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 0)
    bit += 4
    set_unsigned_bits(payload, bit, 1, 1 if sync else 0)
    bit += 1
    set_unsigned_bits(payload, bit, 4, iod)
    bit += 4
    set_signed_bits(payload, bit, 15, round(dclock_m / 0.0016))
    bit += 15
    return bytes(payload), bit


def build_qzss_cssr_code_bias_message(
    *,
    tow_delta: int,
    iod: int,
    bias_m: float = -0.12,
    biases_m: tuple[float, ...] | None = None,
    entry_count: int = 1,
    sync: bool = True,
) -> tuple[bytes, int]:
    bias_values = biases_m if biases_m is not None else (bias_m,)
    entry_count = max(entry_count, len(bias_values))
    payload = bytearray(32)
    bit = 0
    set_unsigned_bits(payload, bit, 12, 4073)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 4)
    bit += 4
    set_unsigned_bits(payload, bit, 12, tow_delta)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 0)
    bit += 4
    set_unsigned_bits(payload, bit, 1, 1 if sync else 0)
    bit += 1
    set_unsigned_bits(payload, bit, 4, iod)
    bit += 4
    for index in range(entry_count):
        entry_bias_m = bias_values[index] if index < len(bias_values) else bias_values[-1]
        set_signed_bits(payload, bit, 11, round(entry_bias_m / 0.02))
        bit += 11
    return bytes(payload), bit


def build_qzss_cssr_phase_bias_message(
    *,
    tow_delta: int,
    iod: int,
    phase_bias_m: float = 0.015,
    phase_biases_m: tuple[float, ...] | None = None,
    satellite_count: int | None = None,
    entry_count: int = 1,
    sync: bool = True,
) -> tuple[bytes, int]:
    phase_bias_values = phase_biases_m if phase_biases_m is not None else (phase_bias_m,)
    if satellite_count is not None:
        entry_count = max(entry_count, satellite_count)
    entry_count = max(entry_count, len(phase_bias_values))
    payload = bytearray(40)
    bit = 0
    set_unsigned_bits(payload, bit, 12, 4073)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 5)
    bit += 4
    set_unsigned_bits(payload, bit, 12, tow_delta)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 0)
    bit += 4
    set_unsigned_bits(payload, bit, 1, 1 if sync else 0)
    bit += 1
    set_unsigned_bits(payload, bit, 4, iod)
    bit += 4
    for index in range(entry_count):
        bias_m = phase_bias_values[index] if index < len(phase_bias_values) else phase_bias_values[-1]
        set_signed_bits(payload, bit, 15, round(bias_m / 0.001))
        bit += 15
        set_unsigned_bits(payload, bit, 2, 0)
        bit += 2
    return bytes(payload), bit


def build_qzss_cssr_code_phase_bias_message(
    *,
    tow_delta: int,
    iod: int,
    code_bias_m: float = -0.12,
    phase_bias_m: float = 0.015,
    code_biases_m: tuple[float, ...] | None = None,
    phase_biases_m: tuple[float, ...] | None = None,
    satellite_count: int = 1,
    entries_per_selected_satellite: int = 1,
    sync: bool = True,
    network_bias: bool = False,
    code_bias_exists: bool = True,
    phase_bias_exists: bool = True,
    selected_mask: int | None = None,
) -> tuple[bytes, int]:
    code_bias_values = code_biases_m if code_biases_m is not None else (code_bias_m,)
    phase_bias_values = phase_biases_m if phase_biases_m is not None else (phase_bias_m,)
    satellite_count = max(satellite_count, len(code_bias_values), len(phase_bias_values))
    if selected_mask is None:
        selected_mask = (1 << satellite_count) - 1
    payload = bytearray(40)
    bit = 0
    set_unsigned_bits(payload, bit, 12, 4073)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 6)
    bit += 4
    set_unsigned_bits(payload, bit, 12, tow_delta)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 0)
    bit += 4
    set_unsigned_bits(payload, bit, 1, 1 if sync else 0)
    bit += 1
    set_unsigned_bits(payload, bit, 4, iod)
    bit += 4
    set_unsigned_bits(payload, bit, 1, 1 if code_bias_exists else 0)
    bit += 1
    set_unsigned_bits(payload, bit, 1, 1 if phase_bias_exists else 0)
    bit += 1
    set_unsigned_bits(payload, bit, 1, 1 if network_bias else 0)
    bit += 1
    if network_bias:
        set_unsigned_bits(payload, bit, 5, 1)
        bit += 5
        set_unsigned_bits(payload, bit, satellite_count, selected_mask)
        bit += satellite_count
    selected_count = sum(
        1
        for index in range(satellite_count)
        if ((selected_mask >> (satellite_count - 1 - index)) & 1) != 0
    )
    total_entries = selected_count * max(1, entries_per_selected_satellite)
    for index in range(total_entries):
        if code_bias_exists:
            bias_m = code_bias_values[index] if index < len(code_bias_values) else code_bias_values[-1]
            set_signed_bits(payload, bit, 11, round(bias_m / 0.02))
            bit += 11
        if phase_bias_exists:
            bias_m = phase_bias_values[index] if index < len(phase_bias_values) else phase_bias_values[-1]
            set_signed_bits(payload, bit, 15, round(bias_m / 0.001))
            bit += 15
            set_unsigned_bits(payload, bit, 2, 0)
            bit += 2
    return bytes(payload), bit


def build_qzss_cssr_ura_message(
    *,
    tow_delta: int,
    iod: int,
    ura_index: int = 9,
    sync: bool = False,
) -> tuple[bytes, int]:
    payload = bytearray(32)
    bit = 0
    set_unsigned_bits(payload, bit, 12, 4073)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 7)
    bit += 4
    set_unsigned_bits(payload, bit, 12, tow_delta)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 0)
    bit += 4
    set_unsigned_bits(payload, bit, 1, 1 if sync else 0)
    bit += 1
    set_unsigned_bits(payload, bit, 4, iod)
    bit += 4
    set_unsigned_bits(payload, bit, 6, ura_index)
    bit += 6
    return bytes(payload), bit


def build_qzss_cssr_atmos_message(
    *,
    tow_delta: int,
    iod: int,
    sync: bool = False,
    network_id: int = 1,
    trop_avail: int = 3,
    stec_avail: int = 3,
    grid_count: int = 1,
    selected_satellites: int = 1,
    trop_quality: int = 0,
    trop_type: int = 0,
    trop_t00_m: float = 0.0,
    trop_t01_m_per_deg: float = 0.0,
    trop_t10_m_per_deg: float = 0.0,
    trop_t11_m_per_deg2: float = 0.0,
    trop_residual_size: int = 0,
    trop_offset_m: float = 0.0,
    trop_residuals_m: tuple[float, ...] | None = None,
    stec_quality: int = 0,
    stec_type: int = 0,
    stec_c00_tecu: float = 0.0,
    stec_c01_tecu_per_deg: float = 0.0,
    stec_c10_tecu_per_deg: float = 0.0,
    stec_c11_tecu_per_deg2: float = 0.0,
    stec_c02_tecu_per_deg2: float = 0.0,
    stec_c20_tecu_per_deg2: float = 0.0,
    stec_residual_size: int = 0,
    stec_residuals_tecu: tuple[float, ...] | None = None,
) -> tuple[bytes, int]:
    payload = bytearray(256)
    bit = 0
    set_unsigned_bits(payload, bit, 12, 4073)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 12)
    bit += 4
    set_unsigned_bits(payload, bit, 12, tow_delta)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 0)
    bit += 4
    set_unsigned_bits(payload, bit, 1, 1 if sync else 0)
    bit += 1
    set_unsigned_bits(payload, bit, 4, iod)
    bit += 4

    set_unsigned_bits(payload, bit, 2, trop_avail)
    bit += 2
    set_unsigned_bits(payload, bit, 2, stec_avail)
    bit += 2
    set_unsigned_bits(payload, bit, 5, network_id)
    bit += 5
    set_unsigned_bits(payload, bit, 6, grid_count)
    bit += 6

    if trop_avail != 0:
        set_unsigned_bits(payload, bit, 6, trop_quality)
        bit += 6
    if (trop_avail & 0x01) != 0:
        set_unsigned_bits(payload, bit, 2, trop_type)
        bit += 2
        set_signed_bits(payload, bit, 9, round(trop_t00_m / 0.004))
        bit += 9
        if trop_type > 0:
            set_signed_bits(payload, bit, 7, round(trop_t01_m_per_deg / 0.002))
            bit += 7
            set_signed_bits(payload, bit, 7, round(trop_t10_m_per_deg / 0.002))
            bit += 7
        if trop_type > 1:
            set_signed_bits(payload, bit, 7, round(trop_t11_m_per_deg2 / 0.001))
            bit += 7
    if (trop_avail & 0x02) != 0:
        residuals_m = trop_residuals_m if trop_residuals_m is not None else tuple(0.0 for _ in range(grid_count))
        set_unsigned_bits(payload, bit, 1, trop_residual_size)
        bit += 1
        set_unsigned_bits(payload, bit, 4, round(trop_offset_m / 0.02))
        bit += 4
        trop_bits = 6 if trop_residual_size == 0 else 8
        for grid_index in range(grid_count):
            residual_m = residuals_m[grid_index] if grid_index < len(residuals_m) else 0.0
            set_signed_bits(payload, bit, trop_bits, round(residual_m / 0.004))
            bit += trop_bits

    if stec_avail != 0:
        for _ in range(selected_satellites):
            set_unsigned_bits(payload, bit, 1, 1)
            bit += 1
        if selected_satellites == 0:
            set_unsigned_bits(payload, bit, 1, 0)
            bit += 1
        for _ in range(selected_satellites):
            set_unsigned_bits(payload, bit, 6, stec_quality)
            bit += 6
            if (stec_avail & 0x01) != 0:
                set_unsigned_bits(payload, bit, 2, stec_type)
                bit += 2
                set_signed_bits(payload, bit, 14, round(stec_c00_tecu / 0.05))
                bit += 14
                if stec_type > 0:
                    set_signed_bits(payload, bit, 12, round(stec_c01_tecu_per_deg / 0.02))
                    bit += 12
                    set_signed_bits(payload, bit, 12, round(stec_c10_tecu_per_deg / 0.02))
                    bit += 12
                if stec_type > 1:
                    set_signed_bits(payload, bit, 10, round(stec_c11_tecu_per_deg2 / 0.02))
                    bit += 10
                if stec_type > 2:
                    set_signed_bits(payload, bit, 8, round(stec_c02_tecu_per_deg2 / 0.005))
                    bit += 8
                    set_signed_bits(payload, bit, 8, round(stec_c20_tecu_per_deg2 / 0.005))
                    bit += 8
            if (stec_avail & 0x02) != 0:
                residuals_tecu = (
                    stec_residuals_tecu
                    if stec_residuals_tecu is not None
                    else tuple(0.0 for _ in range(grid_count))
                )
                set_unsigned_bits(payload, bit, 2, stec_residual_size)
                bit += 2
                stec_bits = (4, 4, 5, 7)[stec_residual_size]
                stec_scale = (0.04, 0.12, 0.16, 0.24)[stec_residual_size]
                for _ in range(grid_count):
                    residual_tecu = residuals_tecu[_] if _ < len(residuals_tecu) else 0.0
                    set_signed_bits(payload, bit, stec_bits, round(residual_tecu / stec_scale))
                    bit += stec_bits
    return bytes(payload), bit


def build_qzss_cssr_gridded_message(
    *,
    tow_delta: int,
    iod: int,
    sync: bool = True,
    network_id: int = 1,
    trop_type: int = 1,
    stec_residual_range: int = 0,
    selected_satellites: int = 1,
    trop_quality: int = 0,
    grid_count: int = 1,
    trop_hs_residuals_m: tuple[float, ...] | None = None,
    trop_wet_residuals_m: tuple[float, ...] | None = None,
    stec_residuals_tecu: tuple[float, ...] | None = None,
) -> tuple[bytes, int]:
    payload = bytearray(256)
    bit = 0
    set_unsigned_bits(payload, bit, 12, 4073)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 9)
    bit += 4
    set_unsigned_bits(payload, bit, 12, tow_delta)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 0)
    bit += 4
    set_unsigned_bits(payload, bit, 1, 1 if sync else 0)
    bit += 1
    set_unsigned_bits(payload, bit, 4, iod)
    bit += 4

    set_unsigned_bits(payload, bit, 2, trop_type)
    bit += 2
    set_unsigned_bits(payload, bit, 1, stec_residual_range)
    bit += 1
    set_unsigned_bits(payload, bit, 5, network_id)
    bit += 5
    for _ in range(selected_satellites):
        set_unsigned_bits(payload, bit, 1, 1)
        bit += 1
    if selected_satellites == 0:
        set_unsigned_bits(payload, bit, 1, 0)
        bit += 1
    set_unsigned_bits(payload, bit, 6, trop_quality)
    bit += 6
    set_unsigned_bits(payload, bit, 6, grid_count)
    bit += 6

    hs_residuals = (
        trop_hs_residuals_m if trop_hs_residuals_m is not None else tuple(0.0 for _ in range(grid_count))
    )
    wet_residuals = (
        trop_wet_residuals_m if trop_wet_residuals_m is not None else tuple(0.0 for _ in range(grid_count))
    )
    stec_values = (
        stec_residuals_tecu if stec_residuals_tecu is not None else tuple(0.0 for _ in range(grid_count))
    )
    stec_bits = 7 if stec_residual_range == 0 else 16
    for grid_index in range(grid_count):
        hs_value = hs_residuals[grid_index] if grid_index < len(hs_residuals) else 0.0
        wet_value = wet_residuals[grid_index] if grid_index < len(wet_residuals) else 0.0
        set_signed_bits(payload, bit, 9, round(hs_value / 0.004))
        bit += 9
        set_signed_bits(payload, bit, 8, round(wet_value / 0.004))
        bit += 8
        for _ in range(selected_satellites):
            stec_value = stec_values[grid_index] if grid_index < len(stec_values) else 0.0
            set_signed_bits(payload, bit, stec_bits, round(stec_value / 0.04))
            bit += stec_bits
    return bytes(payload), bit


def build_qzss_cssr_stec_message(
    *,
    tow_delta: int,
    iod: int,
    sync: bool = True,
    network_id: int = 1,
    selected_satellites: int = 1,
    stec_quality: int = 0,
    stec_type: int = 0,
    stec_c00_tecu: float = 0.0,
    stec_c01_tecu_per_deg: float = 0.0,
    stec_c10_tecu_per_deg: float = 0.0,
    stec_c11_tecu_per_deg2: float = 0.0,
    stec_c02_tecu_per_deg2: float = 0.0,
    stec_c20_tecu_per_deg2: float = 0.0,
) -> tuple[bytes, int]:
    payload = bytearray(128)
    bit = 0
    set_unsigned_bits(payload, bit, 12, 4073)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 8)
    bit += 4
    set_unsigned_bits(payload, bit, 12, tow_delta)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 0)
    bit += 4
    set_unsigned_bits(payload, bit, 1, 1 if sync else 0)
    bit += 1
    set_unsigned_bits(payload, bit, 4, iod)
    bit += 4
    set_unsigned_bits(payload, bit, 2, stec_type)
    bit += 2
    set_unsigned_bits(payload, bit, 5, network_id)
    bit += 5
    for _ in range(selected_satellites):
        set_unsigned_bits(payload, bit, 1, 1)
        bit += 1
    if selected_satellites == 0:
        set_unsigned_bits(payload, bit, 1, 0)
        bit += 1
    for _ in range(selected_satellites):
        set_unsigned_bits(payload, bit, 6, stec_quality)
        bit += 6
        set_signed_bits(payload, bit, 14, round(stec_c00_tecu / 0.05))
        bit += 14
        if stec_type > 0:
            set_signed_bits(payload, bit, 12, round(stec_c01_tecu_per_deg / 0.02))
            bit += 12
            set_signed_bits(payload, bit, 12, round(stec_c10_tecu_per_deg / 0.02))
            bit += 12
        if stec_type > 1:
            set_signed_bits(payload, bit, 10, round(stec_c11_tecu_per_deg2 / 0.02))
            bit += 10
        if stec_type > 2:
            set_signed_bits(payload, bit, 8, round(stec_c02_tecu_per_deg2 / 0.005))
            bit += 8
            set_signed_bits(payload, bit, 8, round(stec_c20_tecu_per_deg2 / 0.005))
            bit += 8
    return bytes(payload), bit


def build_qzss_cssr_service_info_message(
    *,
    sync: bool,
    info_counter: int,
    payload_bytes: bytes,
) -> tuple[bytes, int]:
    if len(payload_bytes) % 5 != 0 or not payload_bytes:
        raise ValueError("service info payload must be a non-empty multiple of 5 bytes")
    data_size = len(payload_bytes) // 5 - 1
    payload = bytearray(32)
    bit = 0
    set_unsigned_bits(payload, bit, 12, 4073)
    bit += 12
    set_unsigned_bits(payload, bit, 4, 10)
    bit += 4
    set_unsigned_bits(payload, bit, 1, 1 if sync else 0)
    bit += 1
    set_unsigned_bits(payload, bit, 3, info_counter)
    bit += 3
    set_unsigned_bits(payload, bit, 2, data_size)
    bit += 2
    for value in payload_bytes:
        set_unsigned_bits(payload, bit, 8, value)
        bit += 8
    return bytes(payload), bit


def build_qzss_l6_subframe_stream(
    messages: list[tuple[bytes, int]],
    *,
    prn: int = 199,
    facility_id: int = 0,
) -> bytes:
    subframe_bits = bytearray((QZSS_L6_SUBFRAME_BITS + 7) // 8)
    bit_offset = 0
    for payload, payload_bits in messages:
        copy_bits(payload, 0, subframe_bits, bit_offset, payload_bits)
        bit_offset += payload_bits
    frames = []
    for frame_index in range(5):
        frame = bytearray(
            build_qzss_l6_frame(
                prn=prn,
                facility_id=facility_id,
                subframe_start=frame_index == 0,
                data_part=b"",
            )
        )
        copy_bits(
            subframe_bits,
            frame_index * QZSS_L6_DATA_PART_BITS,
            frame,
            QZSS_L6_HEADER_BITS,
            QZSS_L6_DATA_PART_BITS,
        )
        frames.append(bytes(frame))
    return b"".join(frames)


def build_single_grid_residuals(
    grid_count: int,
    selected_index: int,
    selected_value: float,
) -> tuple[float, ...]:
    residuals = [0.0] * grid_count
    if 0 <= selected_index < grid_count:
        residuals[selected_index] = selected_value
    return tuple(residuals)


def build_gps_lnav_sfrbx_message(subframe_id: int, sv_id: int = 12, week: int = 2200) -> bytes:
    payload_bits = bytearray(30)
    tow_seconds = 345600
    toc_seconds = 345600
    toes_seconds = 345600
    iodc = 77
    iode = 77

    set_unsigned_bits(payload_bits, 24, 17, tow_seconds // 6)
    set_unsigned_bits(payload_bits, 43, 3, subframe_id)

    if subframe_id == 1:
        bit = 48
        set_unsigned_bits(payload_bits, bit, 10, week % 1024)
        bit += 10
        set_unsigned_bits(payload_bits, bit, 2, 0)
        bit += 2
        set_unsigned_bits(payload_bits, bit, 4, 0)
        bit += 4
        set_unsigned_bits(payload_bits, bit, 6, 0)
        bit += 6
        set_unsigned_bits(payload_bits, bit, 2, iodc >> 8)
        bit += 2
        set_unsigned_bits(payload_bits, bit, 1, 0)
        bit += 1 + 87
        set_signed_bits(payload_bits, bit, 8, 0)
        bit += 8
        set_unsigned_bits(payload_bits, bit, 8, iodc & 0xFF)
        bit += 8
        set_unsigned_bits(payload_bits, bit, 16, toc_seconds // 16)
        bit += 16
        set_signed_bits(payload_bits, bit, 8, 0)
        bit += 8
        set_signed_bits(payload_bits, bit, 16, 0)
        bit += 16
        set_signed_bits(payload_bits, bit, 22, 0)

    elif subframe_id == 2:
        bit = 48
        set_unsigned_bits(payload_bits, bit, 8, iode)
        bit += 8
        set_signed_bits(payload_bits, bit, 16, 0)
        bit += 16
        set_signed_bits(payload_bits, bit, 16, 0)
        bit += 16
        set_signed_bits(payload_bits, bit, 32, 0)
        bit += 32
        set_signed_bits(payload_bits, bit, 16, 0)
        bit += 16
        set_unsigned_bits(payload_bits, bit, 32, round(0.01 / (2.0 ** -33)))
        bit += 32
        set_signed_bits(payload_bits, bit, 16, 0)
        bit += 16
        set_unsigned_bits(payload_bits, bit, 32, round(5153.79589081 / (2.0 ** -19)))
        bit += 32
        set_unsigned_bits(payload_bits, bit, 16, toes_seconds // 16)

    elif subframe_id == 3:
        bit = 48
        set_signed_bits(payload_bits, bit, 16, 0)
        bit += 16
        set_signed_bits(payload_bits, bit, 32, round(0.25 / ((2.0 ** -31) * math.pi)))
        bit += 32
        set_signed_bits(payload_bits, bit, 16, 0)
        bit += 16
        set_signed_bits(payload_bits, bit, 32, round(0.94 / ((2.0 ** -31) * math.pi)))
        bit += 32
        set_signed_bits(payload_bits, bit, 16, 0)
        bit += 16
        set_signed_bits(payload_bits, bit, 32, round(0.50 / ((2.0 ** -31) * math.pi)))
        bit += 32
        set_signed_bits(payload_bits, bit, 24, 0)
        bit += 24
        set_unsigned_bits(payload_bits, bit, 8, iode)
        bit += 8
        set_signed_bits(payload_bits, bit, 14, 0)

    else:
        raise ValueError(f"unsupported GPS subframe id: {subframe_id}")

    payload = bytearray(
        [
            0x02,  # version
            0x0A,  # numWords
            0x01,  # channel
            0x00,
            0x00,  # gnssId = GPS
            sv_id,
            0x00,
            0x00,  # freqId
        ]
    )
    for word_index in range(10):
        word_without_parity = read_unsigned_bits(payload_bits, word_index * 24, 24)
        payload.extend(struct.pack("<I", word_without_parity << 6))
    return build_ubx_message(0x02, 0x13, payload)


def build_glonass_sfrbx_message(string_number: int, sv_id: int = 7, frequency_id: int = 8) -> bytes:
    string_bits = bytearray(10)

    if string_number == 1:
        set_unsigned_bits(string_bits, 1, 4, 1)
        set_unsigned_bits(string_bits, 7, 2, 0)
        set_unsigned_bits(string_bits, 9, 5, 3)
        set_unsigned_bits(string_bits, 14, 6, 0)
        set_unsigned_bits(string_bits, 20, 1, 0)
        set_sign_magnitude_bits(string_bits, 21, 24, 0)
        set_sign_magnitude_bits(string_bits, 45, 5, 0)
        set_sign_magnitude_bits(
            string_bits, 50, 27, int(round(19100000.0 / ((2.0 ** -11) * 1e3)))
        )

    elif string_number == 2:
        set_unsigned_bits(string_bits, 1, 4, 2)
        set_unsigned_bits(string_bits, 5, 3, 0)
        set_unsigned_bits(string_bits, 8, 1, 0)
        set_unsigned_bits(string_bits, 9, 7, 12)
        set_sign_magnitude_bits(string_bits, 21, 24, 0)
        set_sign_magnitude_bits(string_bits, 45, 5, 0)
        set_sign_magnitude_bits(
            string_bits, 50, 27, -int(round(13400000.0 / ((2.0 ** -11) * 1e3)))
        )

    elif string_number == 3:
        set_unsigned_bits(string_bits, 1, 4, 3)
        set_unsigned_bits(string_bits, 5, 1, 0)
        set_sign_magnitude_bits(string_bits, 6, 11, 0)
        set_unsigned_bits(string_bits, 18, 2, 0)
        set_unsigned_bits(string_bits, 20, 1, 0)
        set_sign_magnitude_bits(string_bits, 21, 24, 0)
        set_sign_magnitude_bits(string_bits, 45, 5, 0)
        set_sign_magnitude_bits(
            string_bits, 50, 27, int(round(21200000.0 / ((2.0 ** -11) * 1e3)))
        )

    elif string_number == 4:
        set_unsigned_bits(string_bits, 1, 4, 4)
        set_sign_magnitude_bits(string_bits, 5, 22, 0)
        set_sign_magnitude_bits(string_bits, 27, 5, 0)
        set_unsigned_bits(string_bits, 32, 5, 9)
        set_unsigned_bits(string_bits, 51, 1, 0)
        set_unsigned_bits(string_bits, 52, 4, 3)
        set_unsigned_bits(string_bits, 59, 11, 0)
        set_unsigned_bits(string_bits, 70, 5, sv_id)
        set_unsigned_bits(string_bits, 75, 2, 0)

    else:
        raise ValueError(f"unsupported GLONASS string number: {string_number}")

    frame = bytearray(16)
    frame[:10] = string_bits
    frame[12] = 0x34
    frame[13] = 0x12

    payload = bytearray(
        [
            0x02,  # version
            0x04,  # numWords
            0x01,  # channel
            0x00,
            0x06,  # gnssId = GLONASS
            sv_id,
            0x00,
            frequency_id,
        ]
    )
    for word_index in range(4):
        word = int.from_bytes(frame[word_index * 4:(word_index + 1) * 4], "big")
        payload.extend(struct.pack("<I", word))
    return build_ubx_message(0x02, 0x13, payload)


def build_beidou_d1_sfrbx_message(subframe_id: int, sv_id: int = 12, bdt_week: int = 844) -> bytes:
    subframe_bits = bytearray(38)
    sow = 345600 + (subframe_id - 1) * 6
    toc_bdt = 345600
    toes_bdt = 345600
    iodc = 9
    iode = 17

    set_unsigned_bits(subframe_bits, 15, 3, subframe_id)
    set_unsigned_bits(subframe_bits, 18, 8, sow >> 12)
    set_unsigned_bits(subframe_bits, 30, 12, sow & 0xFFF)

    if subframe_id == 1:
        set_unsigned_bits(subframe_bits, 42, 1, 0)
        set_unsigned_bits(subframe_bits, 43, 5, iodc)
        set_unsigned_bits(subframe_bits, 48, 4, 3)
        set_unsigned_bits(subframe_bits, 60, 13, bdt_week)
        toc_raw = toc_bdt // 8
        set_unsigned_bits(subframe_bits, 73, 9, toc_raw >> 8)
        set_unsigned_bits(subframe_bits, 90, 8, toc_raw & 0xFF)
        tgd1_units = -120
        tgd2_units = 230
        set_signed_bits(subframe_bits, 98, 10, tgd1_units)
        set_signed_bits(subframe_bits, 108, 4, tgd2_units >> 6)
        set_unsigned_bits(subframe_bits, 120, 6, tgd2_units & 0x3F)
        set_signed_bits(subframe_bits, 214, 11, 0)
        af0_units = round(2.5e-4 / (2.0 ** -33))
        af1_units = round(-4.0e-12 / (2.0 ** -50))
        set_signed_bits(subframe_bits, 225, 7, af0_units >> 17)
        set_unsigned_bits(subframe_bits, 240, 17, af0_units & 0x1FFFF)
        set_signed_bits(subframe_bits, 257, 5, af1_units >> 17)
        set_unsigned_bits(subframe_bits, 270, 17, af1_units & 0x1FFFF)
        set_unsigned_bits(subframe_bits, 287, 5, iode)

    elif subframe_id == 2:
        set_signed_bits(subframe_bits, 42, 10, 0)
        set_unsigned_bits(subframe_bits, 60, 6, 0)
        set_signed_bits(subframe_bits, 66, 16, 0)
        set_unsigned_bits(subframe_bits, 90, 2, 0)
        m0_units = round(0.24 / ((2.0 ** -31) * math.pi))
        set_signed_bits(subframe_bits, 92, 20, m0_units >> 12)
        set_unsigned_bits(subframe_bits, 120, 12, m0_units & 0xFFF)
        e_units = round(0.012 / (2.0 ** -33))
        set_unsigned_bits(subframe_bits, 132, 10, e_units >> 22)
        set_unsigned_bits(subframe_bits, 150, 22, e_units & 0x3FFFFF)
        set_signed_bits(subframe_bits, 180, 18, 0)
        set_signed_bits(subframe_bits, 198, 4, 0)
        set_unsigned_bits(subframe_bits, 210, 14, 0)
        set_signed_bits(subframe_bits, 224, 8, 0)
        set_unsigned_bits(subframe_bits, 240, 10, 0)
        sqrt_a_units = round(5282.6256 / (2.0 ** -19))
        set_unsigned_bits(subframe_bits, 250, 12, sqrt_a_units >> 20)
        set_unsigned_bits(subframe_bits, 270, 20, sqrt_a_units & 0xFFFFF)
        toe_raw = toes_bdt // 8
        set_unsigned_bits(subframe_bits, 290, 2, toe_raw >> 15)

    elif subframe_id == 3:
        toe_raw = toes_bdt // 8
        set_unsigned_bits(subframe_bits, 42, 10, toe_raw >> 5)
        set_unsigned_bits(subframe_bits, 60, 5, toe_raw & 0x1F)
        i0_units = round(0.97 / ((2.0 ** -31) * math.pi))
        set_signed_bits(subframe_bits, 65, 17, i0_units >> 15)
        set_unsigned_bits(subframe_bits, 90, 15, i0_units & 0x7FFF)
        set_signed_bits(subframe_bits, 105, 7, 0)
        set_unsigned_bits(subframe_bits, 120, 11, 0)
        omega_dot_units = round(-7.5e-9 / ((2.0 ** -43) * math.pi))
        set_signed_bits(subframe_bits, 131, 11, omega_dot_units >> 13)
        set_unsigned_bits(subframe_bits, 150, 13, omega_dot_units & 0x1FFF)
        set_signed_bits(subframe_bits, 163, 9, 0)
        set_unsigned_bits(subframe_bits, 180, 9, 0)
        idot_units = round(9.0e-11 / ((2.0 ** -43) * math.pi))
        set_signed_bits(subframe_bits, 189, 13, idot_units >> 1)
        set_unsigned_bits(subframe_bits, 210, 1, idot_units & 0x1)
        omega0_units = round(1.25 / ((2.0 ** -31) * math.pi))
        set_signed_bits(subframe_bits, 211, 21, omega0_units >> 11)
        set_unsigned_bits(subframe_bits, 240, 11, omega0_units & 0x7FF)
        omega_units = round(0.61 / ((2.0 ** -31) * math.pi))
        set_signed_bits(subframe_bits, 251, 11, omega_units >> 21)
        set_unsigned_bits(subframe_bits, 270, 21, omega_units & 0x1FFFFF)

    else:
        raise ValueError(f"unsupported BeiDou D1 subframe id: {subframe_id}")

    payload = bytearray(
        [
            0x02,  # version
            0x0A,  # numWords
            0x01,  # channel
            0x00,
            0x03,  # gnssId = BeiDou
            sv_id,
            0x00,
            0x00,  # freqId
        ]
    )
    for word_index in range(10):
        word = read_unsigned_bits(subframe_bits, word_index * 30, 30)
        payload.extend(struct.pack("<I", word))
    return build_ubx_message(0x02, 0x13, payload)


def build_beidou_d2_sfrbx_message(page_id: int, sv_id: int = 3, bdt_week: int = 844) -> bytes:
    if page_id not in {1, 3, 4, 5, 6, 7, 8, 9, 10}:
        raise ValueError(f"unsupported BeiDou D2 page id: {page_id}")

    page_bits = bytearray(38)
    sow_lookup = {
        1: 345600,
        3: 345606,
        4: 345609,
        5: 345612,
        6: 345615,
        7: 345618,
        8: 345621,
        9: 345624,
        10: 345627,
    }
    sow = sow_lookup[page_id]
    toc_bdt = 345600
    toes_bdt = 345600

    set_unsigned_bits(page_bits, 15, 3, 1)
    set_unsigned_bits(page_bits, 18, 8, sow >> 12)
    set_unsigned_bits(page_bits, 30, 12, sow & 0xFFF)
    set_unsigned_bits(page_bits, 42, 4, page_id)

    if page_id == 1:
        set_unsigned_bits(page_bits, 46, 1, 0)
        set_unsigned_bits(page_bits, 47, 5, 9)
        set_unsigned_bits(page_bits, 60, 4, 3)
        set_unsigned_bits(page_bits, 64, 13, bdt_week)
        toc_raw = toc_bdt // 8
        set_unsigned_bits(page_bits, 77, 5, toc_raw >> 12)
        set_unsigned_bits(page_bits, 90, 12, toc_raw & 0xFFF)
        set_signed_bits(page_bits, 102, 10, -120)
        set_signed_bits(page_bits, 120, 10, 230)

    elif page_id == 3:
        set_signed_bits(page_bits, 100, 12, 0)
        set_unsigned_bits(page_bits, 120, 12, 0)
        set_signed_bits(page_bits, 132, 4, 0)

    elif page_id == 4:
        set_unsigned_bits(page_bits, 46, 6, 0)
        set_unsigned_bits(page_bits, 60, 12, 0)
        set_signed_bits(page_bits, 72, 10, 0)
        set_unsigned_bits(page_bits, 90, 1, 0)
        set_unsigned_bits(page_bits, 91, 5, 17)
        set_signed_bits(page_bits, 96, 16, 0)
        set_signed_bits(page_bits, 120, 14, 0)

    elif page_id == 5:
        set_unsigned_bits(page_bits, 46, 4, 0)
        set_signed_bits(page_bits, 50, 2, 0)
        set_unsigned_bits(page_bits, 60, 22, 0)
        set_unsigned_bits(page_bits, 90, 8, 0)
        set_signed_bits(page_bits, 98, 14, 0)
        set_unsigned_bits(page_bits, 120, 4, 0)
        set_signed_bits(page_bits, 124, 10, 0)

    elif page_id == 6:
        set_unsigned_bits(page_bits, 46, 6, 0)
        set_unsigned_bits(page_bits, 60, 16, 0)
        sqrt_a_units = round(5282.6256 / (2.0 ** -19))
        set_unsigned_bits(page_bits, 76, 6, sqrt_a_units >> 26)
        set_unsigned_bits(page_bits, 90, 22, (sqrt_a_units >> 4) & 0x3FFFFF)
        set_unsigned_bits(page_bits, 120, 4, sqrt_a_units & 0xF)
        set_signed_bits(page_bits, 124, 10, 0)

    elif page_id == 7:
        set_unsigned_bits(page_bits, 46, 6, 0)
        set_unsigned_bits(page_bits, 60, 2, 0)
        set_signed_bits(page_bits, 62, 18, 0)
        toe_raw = toes_bdt // 8
        set_unsigned_bits(page_bits, 80, 2, toe_raw >> 15)
        set_unsigned_bits(page_bits, 90, 15, toe_raw & 0x7FFF)
        set_signed_bits(page_bits, 105, 7, 0)
        set_unsigned_bits(page_bits, 120, 14, 0)

    elif page_id == 8:
        set_unsigned_bits(page_bits, 46, 6, 0)
        set_unsigned_bits(page_bits, 60, 5, 0)
        set_signed_bits(page_bits, 65, 17, 0)
        set_unsigned_bits(page_bits, 90, 1, 0)
        set_signed_bits(page_bits, 91, 18, 0)
        set_signed_bits(page_bits, 109, 3, 0)
        set_unsigned_bits(page_bits, 120, 16, 0)

    elif page_id == 9:
        set_unsigned_bits(page_bits, 46, 5, 0)
        set_signed_bits(page_bits, 51, 1, 0)
        set_unsigned_bits(page_bits, 60, 22, 0)
        set_unsigned_bits(page_bits, 90, 9, 0)
        set_signed_bits(page_bits, 99, 13, 0)
        set_unsigned_bits(page_bits, 120, 14, 0)

    elif page_id == 10:
        set_unsigned_bits(page_bits, 46, 5, 0)
        set_signed_bits(page_bits, 51, 1, 0)
        set_unsigned_bits(page_bits, 60, 13, 0)

    payload = bytearray(
        [
            0x02,  # version
            0x0A,  # numWords
            0x01,  # channel
            0x00,
            0x03,  # gnssId = BeiDou
            sv_id,
            0x00,
            0x00,  # freqId
        ]
    )
    for word_index in range(10):
        word = read_unsigned_bits(page_bits, word_index * 30, 30)
        payload.extend(struct.pack("<I", word))
    return build_ubx_message(0x02, 0x13, payload)


def build_galileo_inav_sfrbx_message(
    word_type: int,
    sv_id: int = 5,
    gst_week: int = 1176,
) -> bytes:
    if word_type < 0 or word_type > 5:
        raise ValueError(f"unsupported Galileo word type: {word_type}")

    word = bytearray(16)
    iod_nav = 44
    tow = 345600
    toes = 345600
    toc = 345600

    if word_type == 0:
        set_unsigned_bits(word, 0, 6, 0)
        set_unsigned_bits(word, 6, 2, 2)
        set_unsigned_bits(word, 96, 12, gst_week)
        set_unsigned_bits(word, 108, 20, tow)

    elif word_type == 1:
        set_unsigned_bits(word, 0, 6, 1)
        set_unsigned_bits(word, 6, 10, iod_nav)
        set_unsigned_bits(word, 16, 14, toes // 60)
        set_signed_bits(word, 30, 32, round(0.37 / ((2.0 ** -31) * math.pi)))
        set_unsigned_bits(word, 62, 32, round(0.015 / (2.0 ** -33)))
        set_unsigned_bits(word, 94, 32, round(5440.58820343 / (2.0 ** -19)))

    elif word_type == 2:
        set_unsigned_bits(word, 0, 6, 2)
        set_unsigned_bits(word, 6, 10, iod_nav)
        set_signed_bits(word, 16, 32, round(1.9 / ((2.0 ** -31) * math.pi)))
        set_signed_bits(word, 48, 32, round(0.98 / ((2.0 ** -31) * math.pi)))
        set_signed_bits(word, 80, 32, round(0.73 / ((2.0 ** -31) * math.pi)))
        set_signed_bits(word, 112, 14, round(8.0e-11 / ((2.0 ** -43) * math.pi)))

    elif word_type == 3:
        set_unsigned_bits(word, 0, 6, 3)
        set_unsigned_bits(word, 6, 10, iod_nav)
        set_signed_bits(word, 16, 24, round(-5.9e-9 / ((2.0 ** -43) * math.pi)))
        set_signed_bits(word, 40, 16, round(3.5e-9 / ((2.0 ** -43) * math.pi)))
        set_signed_bits(word, 56, 16, round(1.0e-6 / (2.0 ** -29)))
        set_signed_bits(word, 72, 16, round(-1.5e-6 / (2.0 ** -29)))
        set_signed_bits(word, 88, 16, round(210.0 / (2.0 ** -5)))
        set_signed_bits(word, 104, 16, round(120.0 / (2.0 ** -5)))
        set_unsigned_bits(word, 120, 8, 3)

    elif word_type == 4:
        set_unsigned_bits(word, 0, 6, 4)
        set_unsigned_bits(word, 6, 10, iod_nav)
        set_unsigned_bits(word, 16, 6, sv_id)
        set_signed_bits(word, 22, 16, round(2.4e-8 / (2.0 ** -29)))
        set_signed_bits(word, 38, 16, round(-3.1e-8 / (2.0 ** -29)))
        set_unsigned_bits(word, 54, 14, toc // 60)
        set_signed_bits(word, 68, 31, round(1.7e-4 / (2.0 ** -34)))
        set_signed_bits(word, 99, 21, round(-3.0e-12 / (2.0 ** -46)))
        set_signed_bits(word, 120, 6, 0)

    else:
        set_unsigned_bits(word, 0, 6, 5)
        set_signed_bits(word, 47, 10, round(-4.5e-9 / (2.0 ** -32)))
        set_signed_bits(word, 57, 10, round(1.1e-8 / (2.0 ** -32)))
        set_unsigned_bits(word, 67, 2, 0)
        set_unsigned_bits(word, 69, 2, 0)
        set_unsigned_bits(word, 71, 1, 0)
        set_unsigned_bits(word, 72, 1, 0)

    frame = bytearray(32)
    set_unsigned_bits(frame, 0, 1, 0)
    set_unsigned_bits(frame, 1, 1, 0)
    for index in range(14):
        set_unsigned_bits(frame, 2 + index * 8, 8, word[index])
    set_unsigned_bits(frame, 128, 1, 1)
    set_unsigned_bits(frame, 129, 1, 0)
    for index in range(2):
        set_unsigned_bits(frame, 130 + index * 8, 8, word[14 + index])

    payload = bytearray(
        [
            0x02,  # version
            0x08,  # numWords
            0x01,  # channel
            0x00,
            0x02,  # gnssId = Galileo
            sv_id,
            0x00,
            0x00,  # freqId
        ]
    )
    for word_index in range(8):
        raw_word = int.from_bytes(frame[word_index * 4:(word_index + 1) * 4], "big")
        payload.extend(struct.pack("<I", raw_word))
    return build_ubx_message(0x02, 0x13, payload)


def build_gsof_record(record_type: int, payload: bytes) -> bytes:
    if len(payload) > 255:
        raise ValueError("GSOF record payload too long")
    return bytes([record_type, len(payload)]) + payload


def build_gsof_time_record(
    *,
    gps_week: int = 2200,
    gps_tow_ms: int = 345600123,
    sv_used: int = 18,
    position_flags_1: int = 0x21,
    position_flags_2: int = 0x04,
    init_count: int = 7,
) -> bytes:
    return build_gsof_record(
        1,
        struct.pack(">IHBBBB", gps_tow_ms, gps_week, sv_used, position_flags_1, position_flags_2, init_count),
    )


def build_gsof_llh_record(
    *,
    latitude_deg: float = 35.1234567,
    longitude_deg: float = 139.9876543,
    height_m: float = 42.1,
) -> bytes:
    return build_gsof_record(
        2,
        struct.pack(">ddd", math.radians(latitude_deg), math.radians(longitude_deg), height_m),
    )


def build_gsof_velocity_record(
    *,
    flags: int = 0x03,
    horizontal_speed_mps: float = 1.25,
    heading_deg: float = 90.0,
    vertical_speed_mps: float = -0.125,
    local_heading_deg: float | None = 91.5,
) -> bytes:
    payload = struct.pack(
        ">Bfff",
        flags,
        horizontal_speed_mps,
        math.radians(heading_deg),
        vertical_speed_mps,
    )
    if local_heading_deg is not None:
        payload += struct.pack(">f", math.radians(local_heading_deg))
    return build_gsof_record(8, payload)


def build_gsof_genout_packet(
    records: list[bytes],
    *,
    transmission_number: int = 1,
    page_index: int = 0,
    max_page_index: int = 0,
) -> bytes:
    data = bytes([transmission_number & 0xFF, page_index & 0xFF, max_page_index & 0xFF]) + b"".join(records)
    if len(data) > 255:
        raise ValueError("GSOF packet data too long")
    frame = bytearray([0x02, 0x00, 0x40, len(data)])
    frame.extend(data)
    frame.append(sum(frame[1:]) & 0xFF)
    frame.append(0x03)
    return bytes(frame)


def build_skytraq_frame(message_id: int, body: bytes) -> bytes:
    payload = bytes([message_id]) + body
    checksum = 0
    for value in payload:
        checksum ^= value
    return b"\xA0\xA1" + len(payload).to_bytes(2, "big") + payload + bytes([checksum & 0xFF, 0x0D, 0x0A])


def build_skytraq_epoch_message(*, iod: int = 7, week: int = 2200, tow_ms: int = 345600123) -> bytes:
    return build_skytraq_frame(
        0xDC,
        bytes([iod]) + week.to_bytes(2, "big") + tow_ms.to_bytes(4, "big"),
    )


def build_skytraq_raw_message(*, iod: int = 7, nsat: int = 12) -> bytes:
    return build_skytraq_frame(0xDD, bytes([iod, nsat]))


def build_skytraq_rawx_message(
    *,
    version: int = 1,
    iod: int = 7,
    week: int = 2200,
    tow_ms: int = 345600123,
    period_ms: int = 1000,
    nsat: int = 14,
) -> bytes:
    return build_skytraq_frame(
        0xE5,
        (
            bytes([version, iod])
            + week.to_bytes(2, "big")
            + tow_ms.to_bytes(4, "big")
            + period_ms.to_bytes(2, "big")
            + b"\x00\x00"
            + bytes([nsat])
        ),
    )


def build_skytraq_ack_message(message_id: int = 0x1E) -> bytes:
    return build_skytraq_frame(0x83, bytes([message_id]))


def build_skytraq_nack_message(message_id: int = 0x09) -> bytes:
    return build_skytraq_frame(0x84, bytes([message_id]))


def geodetic_to_ecef(latitude_rad: float, longitude_rad: float, height_m: float) -> tuple[float, float, float]:
    a = 6378137.0
    e2 = 0.00669437999014
    sin_lat = math.sin(latitude_rad)
    cos_lat = math.cos(latitude_rad)
    n = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    return (
        (n + height_m) * cos_lat * math.cos(longitude_rad),
        (n + height_m) * cos_lat * math.sin(longitude_rad),
        (n * (1.0 - e2) + height_m) * sin_lat,
    )


def enu_to_ecef(enu: tuple[float, float, float], latitude_rad: float, longitude_rad: float) -> tuple[float, float, float]:
    east, north, up = enu
    sin_lat = math.sin(latitude_rad)
    cos_lat = math.cos(latitude_rad)
    sin_lon = math.sin(longitude_rad)
    cos_lon = math.cos(longitude_rad)
    return (
        -sin_lon * east - sin_lat * cos_lon * north + cos_lat * cos_lon * up,
        cos_lon * east - sin_lat * sin_lon * north + cos_lat * sin_lon * up,
        cos_lat * north + sin_lat * up,
    )


def geodist(satellite_position: tuple[float, float, float], receiver_position: tuple[float, float, float]) -> float:
    dx = satellite_position[0] - receiver_position[0]
    dy = satellite_position[1] - receiver_position[1]
    dz = satellite_position[2] - receiver_position[2]
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)
    return distance + 7.2921151467e-5 * (
        satellite_position[0] * receiver_position[1] - satellite_position[1] * receiver_position[0]
    ) / 299792458.0


def interpolate_niell_coefficient(abs_lat_deg: float, coefficients: tuple[float, ...]) -> float:
    latitude_grid_deg = (15.0, 30.0, 45.0, 60.0, 75.0)
    if abs_lat_deg <= latitude_grid_deg[0]:
        return coefficients[0]
    if abs_lat_deg >= latitude_grid_deg[-1]:
        return coefficients[-1]
    for index in range(len(latitude_grid_deg) - 1):
        if abs_lat_deg <= latitude_grid_deg[index + 1]:
            span = latitude_grid_deg[index + 1] - latitude_grid_deg[index]
            weight = (abs_lat_deg - latitude_grid_deg[index]) / span
            return coefficients[index] * (1.0 - weight) + coefficients[index + 1] * weight
    return coefficients[-1]


def niell_mapping_continued_fraction(sin_elevation: float, a: float, b: float, c: float) -> float:
    numerator = 1.0 + a / (1.0 + b / (1.0 + c))
    denominator = sin_elevation + a / (sin_elevation + b / (sin_elevation + c))
    return numerator / denominator


def estimate_zenith_troposphere_climatology(
    latitude_rad: float,
    height_m: float,
    day_of_year: int,
) -> tuple[float, float, float, float, float]:
    mean_pressure_hpa = (1013.25, 1017.25, 1015.75, 1011.75, 1013.00)
    amp_pressure_hpa = (0.0, -3.75, -2.25, -1.75, -0.50)
    mean_temperature_k = (299.65, 294.15, 283.15, 272.15, 263.65)
    amp_temperature_k = (0.0, 7.00, 11.00, 15.00, 14.50)
    mean_water_vapor_hpa = (26.31, 21.79, 11.66, 6.78, 4.11)
    amp_water_vapor_hpa = (0.0, 8.85, 7.24, 5.36, 3.39)
    mean_beta = (6.30e-3, 6.05e-3, 5.58e-3, 5.39e-3, 4.53e-3)
    amp_beta = (0.0, 0.25e-3, 0.32e-3, 0.81e-3, 0.62e-3)
    mean_lambda = (2.77, 3.15, 2.57, 1.81, 1.55)
    amp_lambda = (0.0, 0.33, 0.46, 0.74, 0.30)
    gravity = 9.80665
    gas_constant_dry = 287.054

    abs_lat_deg = max(0.0, min(abs(latitude_rad) * 180.0 / math.pi, 90.0))
    seasonal_phase = math.cos(
        2.0 * math.pi * (float(day_of_year) - 28.0) / 365.25 +
        (math.pi if latitude_rad < 0.0 else 0.0)
    )

    def seasonal(mean_values: tuple[float, ...], amplitude_values: tuple[float, ...]) -> float:
        return (
            interpolate_niell_coefficient(abs_lat_deg, mean_values) -
            interpolate_niell_coefficient(abs_lat_deg, amplitude_values) * seasonal_phase
        )

    pressure0_hpa = seasonal(mean_pressure_hpa, amp_pressure_hpa)
    temperature0_k = seasonal(mean_temperature_k, amp_temperature_k)
    water_vapor0_hpa = seasonal(mean_water_vapor_hpa, amp_water_vapor_hpa)
    beta = seasonal(mean_beta, amp_beta)
    lam = seasonal(mean_lambda, amp_lambda)

    clamped_height_m = max(height_m, 0.0)
    temperature_scale = max(1.0 - beta * clamped_height_m / temperature0_k, 1e-3)
    exponent = gravity / (gas_constant_dry * beta)

    pressure_hpa = pressure0_hpa * math.pow(temperature_scale, exponent)
    temperature_k = temperature0_k - beta * clamped_height_m
    water_vapor_pressure_hpa = water_vapor0_hpa * math.pow(
        temperature_scale,
        exponent * (lam + 1.0) - 1.0,
    )
    hydrostatic_delay_m = 0.0022768 * pressure_hpa / (
        1.0 - 0.00266 * math.cos(2.0 * latitude_rad) - 0.00028 * clamped_height_m * 1e-3
    )
    wet_delay_m = 0.002277 * (1255.0 / temperature_k + 0.05) * water_vapor_pressure_hpa
    return (
        hydrostatic_delay_m,
        wet_delay_m,
        pressure_hpa,
        temperature_k,
        water_vapor_pressure_hpa,
    )


def modeled_ppp_trop_delay(
    latitude_rad: float,
    height_m: float,
    elevation_rad: float,
    day_of_year: int,
) -> float:
    mean_a = (1.2769934e-3, 1.2683230e-3, 1.2465397e-3, 1.2196049e-3, 1.2045996e-3)
    mean_b = (2.9153695e-3, 2.9152299e-3, 2.9288445e-3, 2.9022565e-3, 2.9024912e-3)
    mean_c = (62.610505e-3, 62.837393e-3, 63.721774e-3, 63.824265e-3, 64.258455e-3)
    amp_a = (0.0, 1.2709626e-5, 2.6523662e-5, 3.4000452e-5, 4.1202191e-5)
    amp_b = (0.0, 2.1414979e-5, 3.0160779e-5, 7.2562722e-5, 11.723375e-5)
    amp_c = (0.0, 9.0128400e-5, 4.3497037e-5, 84.795348e-5, 170.37206e-5)
    wet_a = (5.8021897e-4, 5.6794847e-4, 5.8118019e-4, 5.9727542e-4, 6.1641693e-4)
    wet_b = (1.4275268e-3, 1.5138625e-3, 1.4572752e-3, 1.5007428e-3, 1.7599082e-3)
    wet_c = (4.3472961e-2, 4.6729510e-2, 4.3908931e-2, 4.4626982e-2, 5.4736038e-2)
    height_a = 2.53e-5
    height_b = 5.49e-3
    height_c = 1.14e-3

    abs_lat_deg = max(0.0, min(abs(latitude_rad) * 180.0 / math.pi, 90.0))
    seasonal_phase = math.cos(
        2.0 * math.pi * (float(day_of_year) - 28.0) / 365.25 +
        (math.pi if latitude_rad < 0.0 else 0.0)
    )
    sin_elevation = max(math.sin(elevation_rad), 0.05)

    hydro_a = interpolate_niell_coefficient(abs_lat_deg, mean_a) - (
        interpolate_niell_coefficient(abs_lat_deg, amp_a) * seasonal_phase
    )
    hydro_b = interpolate_niell_coefficient(abs_lat_deg, mean_b) - (
        interpolate_niell_coefficient(abs_lat_deg, amp_b) * seasonal_phase
    )
    hydro_c = interpolate_niell_coefficient(abs_lat_deg, mean_c) - (
        interpolate_niell_coefficient(abs_lat_deg, amp_c) * seasonal_phase
    )
    hydro_mapping = niell_mapping_continued_fraction(sin_elevation, hydro_a, hydro_b, hydro_c)
    height_km = max(height_m, 0.0) * 1e-3
    if height_km > 0.0:
        hydro_mapping += (
            1.0 / sin_elevation -
            niell_mapping_continued_fraction(sin_elevation, height_a, height_b, height_c)
        ) * height_km

    wet_mapping = niell_mapping_continued_fraction(
        sin_elevation,
        interpolate_niell_coefficient(abs_lat_deg, wet_a),
        interpolate_niell_coefficient(abs_lat_deg, wet_b),
        interpolate_niell_coefficient(abs_lat_deg, wet_c),
    )
    (
        hydrostatic_delay_m,
        wet_delay_m,
        _pressure_hpa,
        _temperature_k,
        _water_vapor_pressure_hpa,
    ) = estimate_zenith_troposphere_climatology(latitude_rad, height_m, day_of_year)
    return hydro_mapping * hydrostatic_delay_m + wet_mapping * wet_delay_m


def format_rinex_header_line(content: str, label: str) -> str:
    return f"{content:<60}{label}\n"


def build_synthetic_receiver_antex_text(antenna_type: str = "TEST-ANT") -> str:
    return "".join(
        (
            format_rinex_header_line("     1.4            M                                       ", "ANTEX VERSION / SYST"),
            format_rinex_header_line("", "START OF ANTENNA"),
            format_rinex_header_line(f"{antenna_type:<20}{'':<20}", "TYPE / SERIAL NO"),
            format_rinex_header_line("   G01", "START OF FREQUENCY"),
            format_rinex_header_line(f"{15.0:10.1f}{-20.0:10.1f}{120.0:10.1f}", "NORTH / EAST / UP"),
            format_rinex_header_line("", "END OF FREQUENCY"),
            format_rinex_header_line("   G02", "START OF FREQUENCY"),
            format_rinex_header_line(f"{10.0:10.1f}{-15.0:10.1f}{105.0:10.1f}", "NORTH / EAST / UP"),
            format_rinex_header_line("", "END OF FREQUENCY"),
            format_rinex_header_line("", "END OF ANTENNA"),
            format_rinex_header_line("", "END OF FILE"),
        )
    )


def build_synthetic_blq_text(
    station_name: str = "TESTMARK",
    up_amplitude_m: float = 0.008,
    west_amplitude_m: float = 0.003,
    south_amplitude_m: float = 0.002,
) -> str:
    def row_text(first_value: float) -> str:
        return "".join(f"{value:10.6f}" for value in [first_value] + [0.0] * 10)

    return "\n".join(
        [
            "$$ Synthetic BLQ coefficients",
            station_name,
            row_text(up_amplitude_m),
            row_text(west_amplitude_m),
            row_text(south_amplitude_m),
            row_text(0.0),
            row_text(0.0),
            row_text(0.0),
            "",
        ]
    )


def build_synthetic_ionex_text() -> str:
    return "".join(
        (
            "     1.0           I                   G                   IONEX VERSION / TYPE\n",
            "   600                                                      INTERVAL\n",
            "    -1                                                      EXPONENT\n",
            "   30.0   40.0   10.0                                       LAT1 / LAT2 / DLAT\n",
            "  130.0  140.0   10.0                                       LON1 / LON2 / DLON\n",
            "  450.0  450.0    0.0                                       HGT1 / HGT2 / DHGT\n",
            "                                                            END OF HEADER\n",
            "    1                                                      START OF TEC MAP\n",
            " 2026     3    26     1     0     0                        EPOCH OF CURRENT MAP\n",
            "   30.0  130.0  140.0   10.0  450.0                        LAT/LON1/LON2/DLON/H\n",
            "    1    2\n",
            "   40.0  130.0  140.0   10.0  450.0                        LAT/LON1/LON2/DLON/H\n",
            "    3    4\n",
            "                                                            END OF TEC MAP\n",
            "    2                                                      START OF TEC MAP\n",
            " 2026     3    26     1    10     0                        EPOCH OF CURRENT MAP\n",
            "   30.0  130.0  140.0   10.0  450.0                        LAT/LON1/LON2/DLON/H\n",
            "    2    3\n",
            "   40.0  130.0  140.0   10.0  450.0                        LAT/LON1/LON2/DLON/H\n",
            "    4    5\n",
            "                                                            END OF TEC MAP\n",
        )
    )


def build_synthetic_dcb_text() -> str:
    return "\n".join(
        [
            "%=BIA 1.00 TEST TEST 2026:085:00000 TEST",
            "+BIAS/SOLUTION",
            "*BIAS SVN PRN STATION OBS1 OBS2 BEGIN END UNIT EST STDDEV",
            " OSB G01 C1C C1C 2026:085:00000 2026:086:00000 ns 0.100 0.010",
            " OSB G01 C2W C2W 2026:085:00000 2026:086:00000 ns -0.050 0.010",
            " OSB G02 C1C C1C 2026:085:00000 2026:086:00000 ns 0.080 0.010",
            " OSB G02 C2W C2W 2026:085:00000 2026:086:00000 ns -0.040 0.010",
            "-BIAS/SOLUTION",
            "",
        ]
    )


def build_synthetic_ppp_inputs(
    temp_root: Path,
    *,
    include_antenna_header: bool = False,
) -> tuple[Path, Path, Path, tuple[float, float, float]]:
    latitude = math.radians(35.0)
    longitude = math.radians(139.0)
    true_position = geodetic_to_ecef(latitude, longitude, 45.0)
    approx_position = (
        true_position[0] + 8.0,
        true_position[1] - 5.0,
        true_position[2] + 3.0,
    )

    look_angles = [
        (0.0, 55.0),
        (60.0, 48.0),
        (120.0, 62.0),
        (180.0, 43.0),
        (240.0, 68.0),
        (300.0, 37.0),
    ]
    satellites: list[tuple[int, tuple[float, float, float]]] = []
    range_m = 26_500_000.0
    for index, (azimuth_deg, elevation_deg) in enumerate(look_angles, start=1):
        azimuth = math.radians(azimuth_deg)
        elevation = math.radians(elevation_deg)
        horizontal = range_m * math.cos(elevation)
        enu = (
            horizontal * math.sin(azimuth),
            horizontal * math.cos(azimuth),
            range_m * math.sin(elevation),
        )
        ecef_delta = enu_to_ecef(enu, latitude, longitude)
        satellites.append(
            (
                index,
                (
                    true_position[0] + ecef_delta[0],
                    true_position[1] + ecef_delta[1],
                    true_position[2] + ecef_delta[2],
                ),
            )
        )

    obs_path = temp_root / "synthetic_ppp.obs"
    sp3_path = temp_root / "synthetic_ppp.sp3"
    clk_path = temp_root / "synthetic_ppp.clk"

    epoch_times = []
    for epoch_index in range(8):
        total_seconds = 30.0 * epoch_index
        minute = int(total_seconds // 60.0)
        second = total_seconds - minute * 60.0
        epoch_times.append(
            f"> 2026 03 26 01 {minute:02d} {second:010.7f}  0{len(satellites):3d}\n"
        )
    obs_lines = [
        format_rinex_header_line("     3.04           O                   M", "RINEX VERSION / TYPE"),
        format_rinex_header_line("libgnss++           tests               20260326 010000 UTC", "PGM / RUN BY / DATE"),
        format_rinex_header_line("TESTMARK", "MARKER NAME"),
        format_rinex_header_line(
            f"{approx_position[0]:14.4f}{approx_position[1]:14.4f}{approx_position[2]:14.4f}",
            "APPROX POSITION XYZ",
        ),
    ]
    if include_antenna_header:
        obs_lines.extend(
            [
                format_rinex_header_line(f"{'12345':<20}{'TEST-ANT':<20}", "ANT # / TYPE"),
                format_rinex_header_line(
                    f"{1.2340:14.4f}{0.1230:14.4f}{-0.4560:14.4f}",
                    "ANTENNA: DELTA H/E/N",
                ),
            ]
        )
    obs_lines.extend(
        [
        format_rinex_header_line(f"G  {4:3d} C1C L1C C2W L2W", "SYS / # / OBS TYPES"),
        format_rinex_header_line("", "END OF HEADER"),
        ]
    )
    for epoch_line in epoch_times:
        obs_lines.append(epoch_line)
        for prn, satellite_position in satellites:
            dx = satellite_position[0] - true_position[0]
            dy = satellite_position[1] - true_position[1]
            dz = satellite_position[2] - true_position[2]
            east, north, up = (
                -math.sin(longitude) * dx + math.cos(longitude) * dy,
                -math.sin(latitude) * math.cos(longitude) * dx
                - math.sin(latitude) * math.sin(longitude) * dy
                + math.cos(latitude) * dz,
                math.cos(latitude) * math.cos(longitude) * dx
                + math.cos(latitude) * math.sin(longitude) * dy
                + math.sin(latitude) * dz,
            )
            elevation = math.atan2(up, math.hypot(east, north))
            trop_delay = modeled_ppp_trop_delay(latitude, 45.0, elevation, 85)
            pseudorange = geodist(satellite_position, true_position)
            l1_cycles = (pseudorange + trop_delay) / (299792458.0 / 1575.42e6)
            l2_cycles = (pseudorange + trop_delay) / (299792458.0 / 1227.60e6)
            obs_lines.append(
                f"G{prn:02d}"
                f"{pseudorange + trop_delay:14.3f}  "
                f"{l1_cycles:14.3f}  "
                f"{pseudorange + trop_delay:14.3f}  "
                f"{l2_cycles:14.3f}  \n"
            )
    obs_path.write_text("".join(obs_lines), encoding="ascii")

    sp3_lines = [
        "*  2026 03 26 01 00 00.00000000\n",
    ]
    for prn, satellite_position in satellites:
        sp3_lines.append(
            f"PG{prn:02d} {satellite_position[0] / 1000.0:14.6f} {satellite_position[1] / 1000.0:14.6f} "
            f"{satellite_position[2] / 1000.0:14.6f} {0.0:14.6f}\n"
        )
    sp3_lines.append("*  2026 03 26 01 10 00.00000000\n")
    for prn, satellite_position in satellites:
        sp3_lines.append(
            f"PG{prn:02d} {satellite_position[0] / 1000.0:14.6f} {satellite_position[1] / 1000.0:14.6f} "
            f"{satellite_position[2] / 1000.0:14.6f} {0.0:14.6f}\n"
        )
    sp3_path.write_text("".join(sp3_lines), encoding="ascii")

    clk_lines = [
        "     3.00           C                   RINEX VERSION / TYPE\n",
        "END OF HEADER\n",
    ]
    for timestamp in ("2026 03 26 01 00 00.00000000", "2026 03 26 01 10 00.00000000"):
        for prn, _ in satellites:
            clk_lines.append(
                f"AS G{prn:02d} {timestamp}  2  0.000000000000E+00  1.000000000000E-12\n"
            )
    clk_path.write_text("".join(clk_lines), encoding="ascii")

    return obs_path, sp3_path, clk_path, true_position


def build_synthetic_ppp_inputs_with_cycle_slip(
    temp_root: Path,
) -> tuple[Path, Path, Path, tuple[float, float, float]]:
    obs_path, sp3_path, clk_path, true_position = build_synthetic_ppp_inputs(temp_root)
    lines = obs_path.read_text(encoding="ascii").splitlines(keepends=True)
    patched_lines: list[str] = []
    epoch_index = -1
    for line in lines:
        if line.startswith(">"):
            epoch_index += 1
            patched_lines.append(line)
            continue
        if line.startswith("G01") and epoch_index >= 2:
            fields = line.split()
            l2_cycles = float(fields[4]) + 20.0
            patched_lines.append(
                f"{fields[0]}"
                f"{float(fields[1]):14.3f}  "
                f"{float(fields[2]):14.3f}  "
                f"{float(fields[3]):14.3f}  "
                f"{l2_cycles:14.3f}  \n"
            )
            continue
        patched_lines.append(line)
    obs_path.write_text("".join(patched_lines), encoding="ascii")
    return obs_path, sp3_path, clk_path, true_position


def write_reference_csv(
    path: Path,
    rows: list[tuple[int, float, float, float, float]],
) -> None:
    with path.open("w", newline="", encoding="ascii") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "gps_tow_s",
                "gps_week",
                "lat_deg",
                "lon_deg",
                "height_m",
                "ecef_x_m",
                "ecef_y_m",
                "ecef_z_m",
            ]
        )
        for week, tow, lat, lon, height in rows:
            x, y, z = driving_comparison.llh_to_ecef(lat, lon, height)
            writer.writerow([tow, week, lat, lon, height, x, y, z])


def write_libgnss_pos(
    path: Path,
    rows: list[tuple[int, float, float, float, float, int, int, float]],
) -> None:
    with path.open("w", encoding="ascii") as handle:
        handle.write("% LibGNSS++ Position Solution\n")
        for week, tow, lat, lon, height, status, nsat, pdop in rows:
            x, y, z = driving_comparison.llh_to_ecef(lat, lon, height)
            handle.write(
                f"{week} {tow:.3f} {x:.4f} {y:.4f} {z:.4f} "
                f"{lat:.9f} {lon:.9f} {height:.4f} {status} {nsat} {pdop:.1f}\n"
            )


def write_rtklib_pos(
    path: Path,
    rows: list[tuple[int, float, float, float, float, int, int]],
) -> None:
    with path.open("w", encoding="ascii") as handle:
        handle.write("% synthetic rtklib solution\n")
        for week, tow, lat, lon, height, quality, nsat in rows:
            handle.write(
                f"{week} {tow:.3f} {lat:.9f} {lon:.9f} {height:.4f} {quality} {nsat}\n"
            )


def ros2_solution_node_exists() -> bool:
    build_dir = ROOT_DIR / "build"
    return any(path.is_file() for path in build_dir.rglob("gnss_solution_node"))


def build_synthetic_ppp_inputs_with_atmos(
    temp_root: Path,
) -> tuple[Path, Path, Path, Path, tuple[float, float, float]]:
    latitude = math.radians(35.0)
    longitude = math.radians(139.0)
    true_position = geodetic_to_ecef(latitude, longitude, 45.0)
    approx_position = (
        true_position[0] + 15.0,
        true_position[1] - 10.0,
        true_position[2] + 6.0,
    )

    look_angles = [
        (0.0, 55.0),
        (60.0, 48.0),
        (120.0, 62.0),
        (180.0, 43.0),
        (240.0, 68.0),
        (300.0, 37.0),
    ]
    satellites: list[tuple[int, tuple[float, float, float], float]] = []
    range_m = 26_500_000.0
    for index, (azimuth_deg, elevation_deg) in enumerate(look_angles, start=1):
        azimuth = math.radians(azimuth_deg)
        elevation = math.radians(elevation_deg)
        horizontal = range_m * math.cos(elevation)
        enu = (
            horizontal * math.sin(azimuth),
            horizontal * math.cos(azimuth),
            range_m * math.sin(elevation),
        )
        ecef_delta = enu_to_ecef(enu, latitude, longitude)
        satellites.append(
            (
                index,
                (
                    true_position[0] + ecef_delta[0],
                    true_position[1] + ecef_delta[1],
                    true_position[2] + ecef_delta[2],
                ),
                elevation,
            )
        )

    obs_path = temp_root / "synthetic_ppp_atmos.obs"
    sp3_path = temp_root / "synthetic_ppp_atmos.sp3"
    clk_path = temp_root / "synthetic_ppp_atmos.clk"
    ssr_path = temp_root / "synthetic_ppp_atmos.csv"

    epoch_times = []
    for epoch_index in range(4):
        total_seconds = 30.0 * epoch_index
        minute = int(total_seconds // 60.0)
        second = total_seconds - minute * 60.0
        epoch_times.append(
            f"> 2026 03 26 03 {minute:02d} {second:010.7f}  0{len(satellites):3d}\n"
        )
    obs_lines = [
        format_rinex_header_line("     3.04           O                   M", "RINEX VERSION / TYPE"),
        format_rinex_header_line("libgnss++           tests               20260326 030000 UTC", "PGM / RUN BY / DATE"),
        format_rinex_header_line(
            f"{approx_position[0]:14.4f}{approx_position[1]:14.4f}{approx_position[2]:14.4f}",
            "APPROX POSITION XYZ",
        ),
        format_rinex_header_line(f"G  {4:3d} C1C L1C C2W L2W", "SYS / # / OBS TYPES"),
        format_rinex_header_line("", "END OF HEADER"),
    ]
    ssr_lines = ["# week,tow,sat,dx,dy,dz,dclock_m[,atmos_<name>=<value>...]\n"]
    base_week = 2411
    base_tow = 356400.0
    for epoch_index, epoch_line in enumerate(epoch_times):
        obs_lines.append(epoch_line)
        tow = base_tow + 30.0 * epoch_index
        for prn, satellite_position, elevation in satellites:
            trop_delay = modeled_ppp_trop_delay(latitude, 45.0, elevation, 85) + 8.0 * (
                1.001 / math.sqrt(0.002001 + max(math.sin(elevation), 0.1) ** 2)
            )
            stec_tecu = 12.0 + 0.5 * prn
            iono_l1 = 40.3e16 * stec_tecu / (1575.42e6 ** 2)
            iono_l2 = 40.3e16 * stec_tecu / (1227.60e6 ** 2)
            pseudorange = geodist(satellite_position, true_position)
            l1_m = pseudorange + trop_delay - iono_l1
            l2_m = pseudorange + trop_delay - iono_l2
            obs_lines.append(
                f"G{prn:02d}"
                f"{pseudorange + trop_delay + iono_l1:14.3f}  "
                f"{l1_m / (299792458.0 / 1575.42e6):14.3f}  "
                f"{pseudorange + trop_delay + iono_l2:14.3f}  "
                f"{l2_m / (299792458.0 / 1227.60e6):14.3f}  \n"
            )
            ssr_lines.append(
                f"{base_week},{tow:.1f},G{prn:02d},0.0,0.0,0.0,0.0,"
                f"atmos_trop_t00_m=8.000000,"
                f"atmos_stec_c00_tecu:G{prn:02d}={stec_tecu:.6f}\n"
            )
    obs_path.write_text("".join(obs_lines), encoding="ascii")
    sp3_lines = [
        "*  2026 03 26 03 00 00.00000000\n",
    ]
    for prn, satellite_position, _ in satellites:
        sp3_lines.append(
            f"PG{prn:02d} {satellite_position[0] / 1000.0:14.6f} {satellite_position[1] / 1000.0:14.6f} "
            f"{satellite_position[2] / 1000.0:14.6f} {0.0:14.6f}\n"
        )
    sp3_lines.append("*  2026 03 26 03 10 00.00000000\n")
    for prn, satellite_position, _ in satellites:
        sp3_lines.append(
            f"PG{prn:02d} {satellite_position[0] / 1000.0:14.6f} {satellite_position[1] / 1000.0:14.6f} "
            f"{satellite_position[2] / 1000.0:14.6f} {0.0:14.6f}\n"
        )
    sp3_path.write_text("".join(sp3_lines), encoding="ascii")

    clk_lines = [
        "     3.00           C                   RINEX VERSION / TYPE\n",
        "END OF HEADER\n",
    ]
    for timestamp in ("2026 03 26 03 00 00.00000000", "2026 03 26 03 10 00.00000000"):
        for prn, _, _ in satellites:
            clk_lines.append(
                f"AS G{prn:02d} {timestamp}  2  0.000000000000E+00  1.000000000000E-12\n"
            )
    clk_path.write_text("".join(clk_lines), encoding="ascii")
    ssr_path.write_text("".join(ssr_lines), encoding="ascii")
    return obs_path, sp3_path, clk_path, ssr_path, true_position


def build_synthetic_ppp_inputs_with_grid_polynomial_atmos(
    temp_root: Path,
) -> tuple[Path, Path, Path, Path, tuple[float, float, float]]:
    latitude = math.radians(35.0)
    longitude = math.radians(139.0)
    true_position = geodetic_to_ecef(latitude, longitude, 45.0)
    approx_position = (
        true_position[0] + 15.0,
        true_position[1] - 10.0,
        true_position[2] + 6.0,
    )

    look_angles = [
        (0.0, 55.0),
        (60.0, 48.0),
        (120.0, 62.0),
        (180.0, 43.0),
        (240.0, 68.0),
        (300.0, 37.0),
    ]
    satellites: list[tuple[int, tuple[float, float, float], float]] = []
    range_m = 26_500_000.0
    for index, (azimuth_deg, elevation_deg) in enumerate(look_angles, start=1):
        azimuth = math.radians(azimuth_deg)
        elevation = math.radians(elevation_deg)
        horizontal = range_m * math.cos(elevation)
        enu = (
            horizontal * math.sin(azimuth),
            horizontal * math.cos(azimuth),
            range_m * math.sin(elevation),
        )
        ecef_delta = enu_to_ecef(enu, latitude, longitude)
        satellites.append(
            (
                index,
                (
                    true_position[0] + ecef_delta[0],
                    true_position[1] + ecef_delta[1],
                    true_position[2] + ecef_delta[2],
                ),
                elevation,
            )
        )

    obs_path = temp_root / "synthetic_ppp_grid_poly.obs"
    sp3_path = temp_root / "synthetic_ppp_grid_poly.sp3"
    clk_path = temp_root / "synthetic_ppp_grid_poly.clk"
    ssr_path = temp_root / "synthetic_ppp_grid_poly.csv"

    epoch_times = []
    for epoch_index in range(4):
        total_seconds = 30.0 * epoch_index
        minute = int(total_seconds // 60.0)
        second = total_seconds - minute * 60.0
        epoch_times.append(
            f"> 2026 03 26 03 {minute:02d} {second:010.7f}  0{len(satellites):3d}\n"
        )
    obs_lines = [
        format_rinex_header_line("     3.04           O                   M", "RINEX VERSION / TYPE"),
        format_rinex_header_line("libgnss++           tests               20260326 030000 UTC", "PGM / RUN BY / DATE"),
        format_rinex_header_line(
            f"{approx_position[0]:14.4f}{approx_position[1]:14.4f}{approx_position[2]:14.4f}",
            "APPROX POSITION XYZ",
        ),
        format_rinex_header_line(f"G  {4:3d} C1C L1C C2W L2W", "SYS / # / OBS TYPES"),
        format_rinex_header_line("", "END OF HEADER"),
    ]
    ssr_lines = ["# week,tow,sat,dx,dy,dz,dclock_m[,atmos_<name>=<value>...]\n"]
    base_week = 2411
    base_tow = 356400.0
    for epoch_index, epoch_line in enumerate(epoch_times):
        obs_lines.append(epoch_line)
        tow = base_tow + 30.0 * epoch_index
        for prn, satellite_position, elevation in satellites:
            trop_residual = 8.0 * (1.001 / math.sqrt(0.002001 + max(math.sin(elevation), 0.1) ** 2))
            trop_delay = modeled_ppp_trop_delay(latitude, 45.0, elevation, 85) + trop_residual
            stec_tecu = 12.0 + 0.5 * prn
            iono_l1 = 40.3e16 * stec_tecu / (1575.42e6 ** 2)
            iono_l2 = 40.3e16 * stec_tecu / (1227.60e6 ** 2)
            pseudorange = geodist(satellite_position, true_position)
            l1_m = pseudorange + trop_delay - iono_l1
            l2_m = pseudorange + trop_delay - iono_l2
            obs_lines.append(
                f"G{prn:02d}"
                f"{pseudorange + trop_delay + iono_l1:14.3f}  "
                f"{l1_m / (299792458.0 / 1575.42e6):14.3f}  "
                f"{pseudorange + trop_delay + iono_l2:14.3f}  "
                f"{l2_m / (299792458.0 / 1227.60e6):14.3f}  \n"
            )
            ssr_lines.append(
                f"{base_week},{tow:.1f},G{prn:02d},0.0,0.0,0.0,0.0,"
                f"atmos_network_id=7,atmos_grid_count=22,"
                f"atmos_trop_t00_m=8.000000,"
                f"atmos_stec_type:G{prn:02d}=1,"
                f"atmos_stec_c00_tecu:G{prn:02d}={stec_tecu:.6f},"
                f"atmos_stec_c01_tecu_per_deg:G{prn:02d}=0.000000,"
                f"atmos_stec_c10_tecu_per_deg:G{prn:02d}=0.000000\n"
            )
    obs_path.write_text("".join(obs_lines), encoding="ascii")
    sp3_lines = [
        "*  2026 03 26 03 00 00.00000000\n",
    ]
    for prn, satellite_position, _ in satellites:
        sp3_lines.append(
            f"PG{prn:02d} {satellite_position[0] / 1000.0:14.6f} {satellite_position[1] / 1000.0:14.6f} "
            f"{satellite_position[2] / 1000.0:14.6f} {0.0:14.6f}\n"
        )
    sp3_lines.append("*  2026 03 26 03 10 00.00000000\n")
    for prn, satellite_position, _ in satellites:
        sp3_lines.append(
            f"PG{prn:02d} {satellite_position[0] / 1000.0:14.6f} {satellite_position[1] / 1000.0:14.6f} "
            f"{satellite_position[2] / 1000.0:14.6f} {0.0:14.6f}\n"
        )
    sp3_path.write_text("".join(sp3_lines), encoding="ascii")

    clk_lines = [
        "     3.00           C                   RINEX VERSION / TYPE\n",
        "END OF HEADER\n",
    ]
    for timestamp in ("2026 03 26 03 00 00.00000000", "2026 03 26 03 10 00.00000000"):
        for prn, _, _ in satellites:
            clk_lines.append(
                f"AS G{prn:02d} {timestamp}  2  0.000000000000E+00  1.000000000000E-12\n"
            )
    clk_path.write_text("".join(clk_lines), encoding="ascii")
    ssr_path.write_text("".join(ssr_lines), encoding="ascii")
    return obs_path, sp3_path, clk_path, ssr_path, true_position




class _CLIToolsShared(unittest.TestCase):
    STATIC_DATA_TESTS = {
        "test_spp_cli_processes_real_static_sample",
        "test_visibility_cli_writes_csv_and_summary_for_static_data",
        "test_nav_products_cli_generates_sp3_and_clk_from_static_sample",
        "test_ppp_cli_processes_real_static_sample_with_generated_products",
        "test_ppp_cli_runs_real_static_slice_with_generated_products_and_ar_enabled",
        "test_ppp_cli_accepts_ssr_corrections_csv",
        "test_ppp_cli_accepts_rtcm_ssr_corrections",
        "test_ppp_cli_accepts_ntrip_rtcm_ssr_corrections",
        "test_ppp_static_signoff_cli_writes_summary_and_passes_thresholds",
        "test_ppp_static_signoff_cli_supports_real_data_ar_signoff",
        "test_clas_ppp_cli_writes_summary_for_named_profile",
        "test_clas_ppp_cli_accepts_compact_sampled_corrections",
        "test_clas_ppp_cli_accepts_direct_qzss_l6_corrections",
        "test_clas_ppp_cli_accepts_direct_qzss_l6_orbit_clock_corrections",
        "test_clas_ppp_cli_accepts_direct_qzss_l6_code_bias_and_ura",
        "test_clas_ppp_cli_accepts_direct_qzss_l6_with_atmos_inventory",
        "test_clas_ppp_cli_accepts_direct_qzss_l6_with_stec_inventory",
        "test_clas_ppp_cli_accepts_direct_qzss_l6_gridded_corrections",
        "test_clas_ppp_cli_accepts_direct_qzss_l6_code_phase_bias",
        "test_clas_ppp_cli_accepts_direct_qzss_l6_code_bias_composition_policy",
        "test_clas_ppp_cli_accepts_direct_qzss_l6_code_bias_bank_policy",
        "test_clas_ppp_cli_accepts_direct_qzss_l6_bias_row_materialization",
        "test_clas_ppp_cli_accepts_direct_qzss_l6_row_construction_policy",
        "test_clas_ppp_cli_reports_applied_atmospheric_corrections_for_direct_qzss_l6",
        "test_clas_ppp_cli_uses_nearest_clas_grid_residuals_for_direct_qzss_l6",
        "test_ppp_products_signoff_cli_runs_static_profile_with_local_templates",
        "test_ppp_products_signoff_cli_runs_ppc_profile_with_existing_solution",
        "test_ppp_products_signoff_cli_reads_ppc_config_toml",
        "test_ppp_products_signoff_cli_runs_ppc_profile_with_malib_bin",
    }
    SHORT_BASELINE_DATA_TESTS = {
        "test_solve_short_baseline_cli_reaches_fixed_solution",
        "test_short_baseline_signoff_cli_writes_summary_and_passes_thresholds",
    }
    KINEMATIC_DATA_TESTS = {
        "test_solve_cli_supports_estimated_iono_mode",
        "test_solve_cli_supports_moving_base_mode",
        "test_rtk_kinematic_signoff_cli_writes_summary_and_passes_thresholds",
        "test_ppp_kinematic_signoff_cli_writes_summary_and_passes_thresholds",
        "test_ppp_kinematic_signoff_cli_can_fetch_precise_products",
        "test_ppp_products_signoff_cli_runs_kinematic_profile_with_malib_bin",
        "test_replay_solves_bundled_rinex_sequence",
        "test_replay_supports_moving_base_mode",
        "test_solve_accepts_low_cost_preset_and_hold_knobs",
    }
    STATIC_DATA_TESTS |= {
        "test_ppp_static_signoff_cli_can_fetch_precise_products",
    }
    ODAIBA_DATA_TESTS = {
        "test_solve_odaiba_slice_uses_glonass_and_beidou_in_rtk",
    }
    def setUp(self) -> None:
        name = self._testMethodName
        if name in self.STATIC_DATA_TESTS and not repo_data_exists(*STATIC_DATA_FILES):
            self.skipTest("repo static test data is not available")
        if name in self.SHORT_BASELINE_DATA_TESTS and not repo_data_exists(*SHORT_BASELINE_DATA_FILES):
            self.skipTest("repo short-baseline test data is not available")
        if name in self.KINEMATIC_DATA_TESTS and not repo_data_exists(*KINEMATIC_DATA_FILES):
            self.skipTest("repo kinematic test data is not available")
        if name in self.ODAIBA_DATA_TESTS and not repo_data_exists(*ODAIBA_DATA_FILES):
            self.skipTest("repo Odaiba test data is not available")
    def run_gnss(
        self,
        *args: str,
        extra_env: dict[str, str] | None = None,
    ) -> subprocess.CompletedProcess[str]:
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        if extra_env is not None:
            env.update(extra_env)
        return subprocess.run(
            [sys.executable, str(DISPATCHER), *args],
            cwd=ROOT_DIR,
            env=env,
            text=True,
            capture_output=True,
            check=False,
        )
    def read_pos_records(self, path: Path) -> list[dict[str, float | int]]:
        records: list[dict[str, float | int]] = []
        for line in path.read_text(encoding="ascii").splitlines():
            if not line or line.startswith("%"):
                continue
            parts = line.split()
            records.append(
                {
                    "tow": float(parts[1]),
                    "x": float(parts[2]),
                    "y": float(parts[3]),
                    "z": float(parts[4]),
                    "status": int(parts[8]),
                    "satellites": int(parts[9]),
                }
            )
        return records
