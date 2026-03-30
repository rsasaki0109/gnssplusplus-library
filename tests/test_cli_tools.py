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
import zlib
import http.server
import importlib.util
from functools import partial
from pathlib import Path
from urllib import request

if os.name != "nt":
    import pty


ROOT_DIR = Path(__file__).resolve().parents[1]
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
DEFAULT_RTKLIB_BIN = Path("/media/sasaki/aiueo/ai_coding_ws/rtklib_v2_ws/RTKLIB/rnx2rtkp")

sys.path.insert(0, str(SCRIPTS_DIR))

import generate_driving_comparison as driving_comparison  # noqa: E402


def repo_data_exists(*relative_paths: str) -> bool:
    return all((ROOT_DIR / relative_path).exists() for relative_path in relative_paths)


def ros2_bag_support_available() -> bool:
    required = ("rosbag2_py", "rclpy", "rosidl_runtime_py", "ublox_msgs")
    return all(importlib.util.find_spec(name) is not None for name in required)


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
    sync: bool = True,
    sigmask: int = 0x8000,
) -> tuple[bytes, int]:
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
    set_unsigned_bits(payload, bit, 4, 0)
    bit += 4
    set_unsigned_bits(payload, bit, 40, encode_cssr_satellite_mask(prn))
    bit += 40
    set_unsigned_bits(payload, bit, 16, sigmask)
    bit += 16
    set_unsigned_bits(payload, bit, 1, 1)
    bit += 1
    set_unsigned_bits(payload, bit, 1, 1)
    bit += 1
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
    sync: bool = True,
) -> tuple[bytes, int]:
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
    set_signed_bits(payload, bit, 11, round(bias_m / 0.02))
    bit += 11
    return bytes(payload), bit


def build_qzss_cssr_phase_bias_message(
    *,
    tow_delta: int,
    iod: int,
    phase_bias_m: float = 0.015,
    sync: bool = True,
) -> tuple[bytes, int]:
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
    set_signed_bits(payload, bit, 15, round(phase_bias_m / 0.001))
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
    sync: bool = True,
    network_bias: bool = False,
) -> tuple[bytes, int]:
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
    set_unsigned_bits(payload, bit, 1, 1)
    bit += 1
    set_unsigned_bits(payload, bit, 1, 1)
    bit += 1
    set_unsigned_bits(payload, bit, 1, 1 if network_bias else 0)
    bit += 1
    if network_bias:
        set_unsigned_bits(payload, bit, 5, 1)
        bit += 5
        set_unsigned_bits(payload, bit, 1, 1)
        bit += 1
    set_signed_bits(payload, bit, 11, round(code_bias_m / 0.02))
    bit += 11
    set_signed_bits(payload, bit, 15, round(phase_bias_m / 0.001))
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
            trop_delay = modeled_ppp_trop_delay(latitude, 45.0, elevation, 85) + 2.3 * (
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
                f"atmos_trop_t00_m={2.3 * (1.001 / math.sqrt(0.002001 + max(math.sin(elevation), 0.1) ** 2)):.6f},"
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

    # CLAS network 7 grid no. 11 from the official clas_grid.def.
    dlat_deg = 35.0 - 34.77
    dlon_deg = 139.0 - 139.37

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
    stec_c01 = 40.0
    for epoch_index, epoch_line in enumerate(epoch_times):
        obs_lines.append(epoch_line)
        tow = base_tow + 30.0 * epoch_index
        for prn, satellite_position, elevation in satellites:
            trop_residual = 2.3 * (1.001 / math.sqrt(0.002001 + max(math.sin(elevation), 0.1) ** 2))
            trop_delay = modeled_ppp_trop_delay(latitude, 45.0, elevation, 85) + trop_residual
            stec_tecu = 12.0 + 0.5 * prn
            stec_c10 = (stec_tecu - stec_c01 * dlat_deg) / dlon_deg
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
                f"atmos_trop_t00_m={trop_residual:.6f},"
                f"atmos_stec_type:G{prn:02d}=1,"
                f"atmos_stec_c01_tecu_per_deg:G{prn:02d}={stec_c01:.6f},"
                f"atmos_stec_c10_tecu_per_deg:G{prn:02d}={stec_c10:.6f}\n"
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


class CLIToolsTest(unittest.TestCase):
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
        "test_clas_ppp_cli_reports_applied_atmospheric_corrections_for_direct_qzss_l6",
        "test_clas_ppp_cli_uses_nearest_clas_grid_residuals_for_direct_qzss_l6",
    }
    SHORT_BASELINE_DATA_TESTS = {
        "test_solve_short_baseline_cli_reaches_fixed_solution",
        "test_short_baseline_signoff_cli_writes_summary_and_passes_thresholds",
    }
    KINEMATIC_DATA_TESTS = {
        "test_solve_cli_supports_estimated_iono_mode",
        "test_rtk_kinematic_signoff_cli_writes_summary_and_passes_thresholds",
        "test_ppp_kinematic_signoff_cli_writes_summary_and_passes_thresholds",
        "test_replay_solves_bundled_rinex_sequence",
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

    def test_rinex_info_reports_observation_header_and_epoch_count(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rinex_info_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, _, _, _ = build_synthetic_ppp_inputs(temp_root)

            result = self.run_gnss("rinex-info", "--count-records", str(obs_path))

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("type: observation", result.stdout)
            self.assertIn("marker: TESTMARK", result.stdout)
            self.assertIn("epoch count: 8", result.stdout)
            self.assertIn("total observation records: 96", result.stdout)

    def test_fetch_products_cli_copies_and_decompresses_local_template(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_fetch_products_local_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            source_dir = temp_root / "source"
            source_dir.mkdir()
            compressed_path = source_dir / "2024002.sp3.gz"
            with gzip.open(compressed_path, "wb") as stream:
                stream.write(b"TEST SP3\n")

            cache_dir = temp_root / "cache"
            summary_path = temp_root / "products.json"
            product_template = str(source_dir / "{yyyy}{doy}.sp3.gz")

            result = self.run_gnss(
                "fetch-products",
                "--date",
                "2024-01-02",
                "--product",
                f"sp3={product_template}",
                "--cache-dir",
                str(cache_dir),
                "--summary-json",
                str(summary_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(result.stdout)
            self.assertEqual(payload["status_counts"], {"copied": 1})
            self.assertEqual(set(payload["products"]), {"sp3"})
            destination = Path(payload["products"]["sp3"])
            self.assertTrue(destination.exists())
            self.assertEqual(destination.read_bytes(), b"TEST SP3\n")
            self.assertFalse(destination.name.endswith(".gz"))
            self.assertEqual(json.loads(summary_path.read_text(encoding="utf-8"))["products"]["sp3"], str(destination))

            cached = self.run_gnss(
                "fetch-products",
                "--date",
                "2024-01-02",
                "--product",
                f"sp3={product_template}",
                "--cache-dir",
                str(cache_dir),
            )
            self.assertEqual(cached.returncode, 0, msg=cached.stderr)
            cached_payload = json.loads(cached.stdout)
            self.assertEqual(cached_payload["results"][0]["status"], "cached")

    def test_fetch_products_cli_downloads_http_template(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_fetch_products_http_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            source_dir = temp_root / "source"
            source_dir.mkdir()
            compressed_path = source_dir / "24002.clk.gz"
            with gzip.open(compressed_path, "wb") as stream:
                stream.write(b"TEST CLK\n")

            server, thread, port = start_static_http_server(source_dir)
            try:
                cache_dir = temp_root / "cache"
                summary_path = temp_root / "products.json"
                result = self.run_gnss(
                    "fetch-products",
                    "--date",
                    "2024-01-02",
                    "--product",
                    f"clk=http://127.0.0.1:{port}/{{yy}}{{doy}}.clk.gz",
                    "--cache-dir",
                    str(cache_dir),
                    "--summary-json",
                    str(summary_path),
                )
            finally:
                server.shutdown()
                thread.join(timeout=2.0)
                server.server_close()

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(result.stdout)
            self.assertEqual(payload["status_counts"], {"downloaded": 1})
            destination = Path(payload["products"]["clk"])
            self.assertTrue(destination.exists())
            self.assertEqual(destination.read_bytes(), b"TEST CLK\n")
            self.assertEqual(payload["results"][0]["resolved_source"], f"http://127.0.0.1:{port}/24002.clk.gz")
            self.assertEqual(json.loads(summary_path.read_text(encoding="utf-8"))["products"]["clk"], str(destination))

    def test_fetch_products_cli_lists_presets_and_supports_dry_run(self) -> None:
        presets = self.run_gnss("fetch-products", "--list-presets")
        self.assertEqual(presets.returncode, 0, msg=presets.stderr)
        self.assertIn("igs-final", presets.stdout)
        self.assertIn("ionex", presets.stdout)
        self.assertIn("dcb", presets.stdout)
        self.assertIn("brdc-nav", presets.stdout)

        with tempfile.TemporaryDirectory(prefix="gnss_fetch_products_dry_run_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            cache_dir = temp_root / "cache"
            summary_path = temp_root / "products.json"
            dry_run = self.run_gnss(
                "fetch-products",
                "--date",
                "2024-01-02",
                "--preset",
                "igs-final",
                "--preset",
                "ionex",
                "--preset",
                "dcb",
                "--cache-dir",
                str(cache_dir),
                "--summary-json",
                str(summary_path),
                "--dry-run",
            )
            self.assertEqual(dry_run.returncode, 0, msg=dry_run.stderr)
            payload = json.loads(dry_run.stdout)
            self.assertTrue(payload["dry_run"])
            self.assertEqual(payload["presets"], ["igs-final", "ionex", "dcb"])
            self.assertEqual(set(payload["products"]), {"sp3", "clk", "ionex", "dcb"})
            self.assertEqual(payload["status_counts"], {"dry-run": 4})
            self.assertIn("COD0OPSFIN_20240020000_01D_05M_ORB.SP3.gz", payload["results"][0]["resolved_source"])
            self.assertIn("CAS0MGXRAP_20240020000_01D_01D_DCB.BSX.gz", json.dumps(payload))
            self.assertEqual(
                json.loads(summary_path.read_text(encoding="utf-8"))["products"]["ionex"],
                payload["products"]["ionex"],
            )

    def test_ionex_info_reports_header_and_map_counts(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ionex_info_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            ionex_path = temp_root / "sample.ionex"
            summary_path = temp_root / "sample_ionex.json"
            ionex_path.write_text(
                "\n".join(
                    [
                        "     1.0           I                   G                   IONEX VERSION / TYPE",
                        "  2024     1     2     0     0     0                        EPOCH OF FIRST MAP",
                        "  2024     1     2     1     0     0                        EPOCH OF LAST MAP",
                        "    3600                                                      INTERVAL",
                        "       2                                                    # OF MAPS IN FILE",
                        "       2                                                    MAP DIMENSION",
                        "  6371.0                                                    BASE RADIUS",
                        "     10.0                                                   ELEVATION CUTOFF",
                        "COSZ                                                        MAPPING FUNCTION",
                        "      -1                                                    EXPONENT",
                        "   -87.5    87.5     2.5                                    LAT1 / LAT2 / DLAT",
                        "  -180.0   180.0     5.0                                    LON1 / LON2 / DLON",
                        "   450.0   450.0     0.0                                    HGT1 / HGT2 / DHGT",
                        "G01     1.234     0.100                                     PRN / BIAS / RMS",
                        "                                                            END OF HEADER",
                        "     1                                                      START OF TEC MAP",
                        "  2024     1     2     0     0     0                        EPOCH OF CURRENT MAP",
                        "     1                                                      END OF TEC MAP",
                        "     2                                                      START OF TEC MAP",
                        "  2024     1     2     1     0     0                        EPOCH OF CURRENT MAP",
                        "     2                                                      END OF TEC MAP",
                        "     1                                                      START OF RMS MAP",
                        "     1                                                      END OF RMS MAP",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            result = self.run_gnss(
                "ionex-info",
                "--input",
                str(ionex_path),
                "--summary-json",
                str(summary_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("format: IONEX", result.stdout)
            self.assertIn("maps: 2", result.stdout)
            self.assertIn("rms maps: 1", result.stdout)
            self.assertIn("aux dcb entries: 1", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["map_count"], 2)
            self.assertEqual(payload["rms_map_count"], 1)
            self.assertEqual(payload["aux_dcb_count"], 1)
            self.assertEqual(payload["system"], "G")

    def test_dcb_info_reports_bias_sinex_entries(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_dcb_info_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            dcb_path = temp_root / "sample.BSX"
            summary_path = temp_root / "sample_dcb.json"
            dcb_path.write_text(
                "\n".join(
                    [
                        "%=BIA 1.00 TEST TEST 2024:002:00000 TEST",
                        "+BIAS/SOLUTION",
                        "*BIAS SVN PRN STATION OBS1 OBS2 BEGIN END UNIT EST STDDEV",
                        " DSB G01 C1C C2W 2024:002:00000 2024:003:00000 ns 1.234 0.100",
                        " OSB E11 C1C C5Q 2024:002:00000 2024:003:00000 ns 0.321 0.050",
                        "-BIAS/SOLUTION",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            result = self.run_gnss(
                "dcb-info",
                "--input",
                str(dcb_path),
                "--summary-json",
                str(summary_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("format: SINEX_BIAS", result.stdout)
            self.assertIn("entries: 2", result.stdout)
            self.assertIn("bias types: DSB, OSB", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["entry_count"], 2)
            self.assertEqual(payload["bias_types"], ["DSB", "OSB"])
            self.assertEqual(payload["systems"], ["E", "G"])

    def test_spp_cli_processes_real_static_sample(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_spp_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            output_path = temp_root / "spp.pos"

            result = self.run_gnss(
                "spp",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--out",
                str(output_path),
                "--max-epochs",
                "10",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Processed epochs:", result.stdout)
            self.assertIn("Valid solutions:", result.stdout)
            self.assertTrue(output_path.exists())
            self.assertIn("LibGNSS++ Position Solution", output_path.read_text(encoding="ascii"))

    def test_visibility_cli_writes_csv_and_summary_for_static_data(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_visibility_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            csv_path = temp_root / "visibility.csv"
            summary_path = temp_root / "visibility.json"

            result = self.run_gnss(
                "visibility",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--csv",
                str(csv_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "5",
                "--min-elevation-deg",
                "5",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Visibility summary:", result.stdout)
            self.assertIn("rows written:", result.stdout)
            self.assertTrue(csv_path.exists())
            self.assertTrue(summary_path.exists())

            csv_lines = csv_path.read_text(encoding="utf-8").splitlines()
            self.assertGreater(len(csv_lines), 1)
            self.assertEqual(
                csv_lines[0],
                "epoch_index,week,tow,satellite,system,signal,azimuth_deg,elevation_deg,snr_dbhz,"
                "has_pseudorange,has_carrier_phase,has_doppler",
            )
            first_row = csv_lines[1].split(",")
            self.assertEqual(len(first_row), 12)
            self.assertTrue(first_row[3].startswith(("G", "R", "E", "C", "J", "S", "I")))

            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["epochs_processed"], 5)
            self.assertGreater(payload["rows_written"], 0)
            self.assertGreater(payload["unique_satellites"], 0)
            self.assertIn("GPS", payload["rows_per_system"])

    def test_visibility_plot_generates_png_from_visibility_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_visibility_plot_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            csv_path = temp_root / "visibility.csv"
            png_path = temp_root / "visibility.png"
            csv_path.write_text(
                "\n".join(
                    [
                        "epoch_index,week,tow,satellite,system,signal,azimuth_deg,elevation_deg,snr_dbhz,has_pseudorange,has_carrier_phase,has_doppler",
                        "1,2200,100.0,G01,GPS,GPS_L1CA,45.0,30.0,42.0,1,1,0",
                        "1,2200,100.0,G02,GPS,GPS_L1CA,120.0,55.0,47.0,1,1,0",
                        "2,2200,130.0,E11,Galileo,GAL_E1,250.0,20.0,38.0,1,1,0",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            result = self.run_gnss("visibility-plot", str(csv_path), str(png_path))

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Saved:", result.stdout)
            self.assertTrue(png_path.exists())
            self.assertGreater(png_path.stat().st_size, 0)

    def test_stats_reports_solution_status_breakdown(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_stats_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            pos_path = temp_root / "stats.pos"
            write_libgnss_pos(
                pos_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 4, 10, 1.0),
                    (2200, 345630.0, 35.000001, 139.000001, 10.1, 3, 9, 1.3),
                    (2200, 345660.0, 35.000002, 139.000002, 10.2, 1, 8, 1.8),
                    (2200, 345690.0, 35.0000005, 139.0000005, 10.0, 4, 11, 0.9),
                ],
            )

            result = self.run_gnss("stats", str(pos_path))

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("総エポック数", result.stdout)
            self.assertIn("Fix解", result.stdout)
            self.assertIn("Float解", result.stdout)
            self.assertIn("SPP解", result.stdout)

    def test_compare_generates_png_for_synthetic_solutions(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_compare_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            lib_path = temp_root / "compare.pos"
            rtklib_path = temp_root / "compare_rtklib.pos"
            output_png = temp_root / "compare_comparison.png"
            write_libgnss_pos(
                lib_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 4, 10, 1.0),
                    (2200, 345630.0, 35.000001, 139.000002, 10.0, 4, 10, 1.0),
                    (2200, 345660.0, 35.000002, 139.000004, 10.1, 3, 9, 1.2),
                ],
            )
            write_rtklib_pos(
                rtklib_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 1, 10),
                    (2200, 345630.0, 35.0000012, 139.0000021, 10.1, 1, 10),
                    (2200, 345660.0, 35.0000024, 139.0000040, 10.2, 2, 9),
                ],
            )

            result = self.run_gnss("compare", str(lib_path), str(rtklib_path))

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("LibGNSS++:", result.stdout)
            self.assertIn("RTKLIB   :", result.stdout)
            self.assertTrue(output_png.exists())

    def test_plot_generates_single_and_comparison_pngs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_plot_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            lib_path = temp_root / "plot.pos"
            rtklib_path = temp_root / "plot_rtklib.pos"
            write_libgnss_pos(
                lib_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 4, 10, 1.0),
                    (2200, 345630.0, 35.000001, 139.000001, 10.0, 3, 9, 1.2),
                    (2200, 345660.0, 35.000002, 139.000002, 10.1, 1, 8, 1.6),
                ],
            )
            write_rtklib_pos(
                rtklib_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 1, 10),
                    (2200, 345630.0, 35.0000015, 139.0000010, 10.0, 2, 9),
                    (2200, 345660.0, 35.0000025, 139.0000021, 10.2, 5, 8),
                ],
            )

            result = self.run_gnss("plot", str(lib_path), str(rtklib_path))

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue((temp_root / "plot_plot.png").exists())
            self.assertTrue((temp_root / "plot_vs_rtklib.png").exists())

    def test_trackplot_generates_png(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_trackplot_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            lib_path = temp_root / "track.pos"
            rtklib_path = temp_root / "track_rtklib.pos"
            output_png = temp_root / "track.png"
            write_libgnss_pos(
                lib_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 4, 10, 1.0),
                    (2200, 345630.0, 35.000001, 139.000001, 10.1, 3, 9, 1.2),
                    (2200, 345660.0, 35.000002, 139.000002, 10.2, 1, 8, 1.6),
                ],
            )
            write_rtklib_pos(
                rtklib_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 1, 10),
                    (2200, 345630.0, 35.0000011, 139.0000011, 10.0, 2, 9),
                    (2200, 345660.0, 35.0000022, 139.0000021, 10.1, 4, 8),
                ],
            )

            result = self.run_gnss("trackplot", str(lib_path), str(rtklib_path), str(output_png))

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Saved:", result.stdout)
            self.assertTrue(output_png.exists())

    def test_rtklib2pos_converts_solution_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rtklib2pos_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "input_rtklib.pos"
            output_path = temp_root / "converted.pos"
            input_path.write_text(
                "\n".join(
                    [
                        "% synthetic rtklib solution",
                        "2026/03/27 00:00:00.000 35.000000000 139.000000000 10.0000 1 10",
                        "2026/03/27 00:00:30.000 35.000001000 139.000001000 10.1000 2 9",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            result = self.run_gnss("rtklib2pos", str(input_path), str(output_path))

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Converted 2 position solutions", result.stdout)
            exported = output_path.read_text(encoding="utf-8")
            self.assertIn("Converted from RTKLIB POS format", exported)
            self.assertIn(" 4 10 0.0", exported)
            self.assertIn(" 3 9 0.0", exported)

    def test_pos2kml_exports_track_and_sample_points(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_pos2kml_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "track.pos"
            output_path = temp_root / "track.kml"
            write_libgnss_pos(
                input_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 4, 10, 1.0),
                    (2200, 345630.0, 35.000001, 139.000001, 10.1, 3, 9, 1.2),
                    (2200, 345660.0, 35.000002, 139.000002, 10.2, 1, 8, 1.6),
                ],
            )

            result = self.run_gnss(
                "pos2kml",
                str(input_path),
                str(output_path),
                "--sample-points",
                "1",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Saved:", result.stdout)
            self.assertIn("Epochs: 3", result.stdout)
            exported = output_path.read_text(encoding="utf-8")
            self.assertIn("<LineString>", exported)
            self.assertIn("FIXED", exported)

    def test_driving_compare_generates_pngs_from_synthetic_inputs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_driving_compare_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            lib_path = temp_root / "lib.pos"
            rtklib_path = temp_root / "rtklib.pos"
            reference_csv = temp_root / "reference.csv"
            output_png = temp_root / "comparison.png"
            rtklib_2d = temp_root / "rtklib_2d.png"
            lib_2d = temp_root / "lib_2d.png"
            reference_rows = [
                (2200, 345600.0, 35.0, 139.0, 10.0),
                (2200, 345630.0, 35.000001, 139.000001, 10.0),
                (2200, 345660.0, 35.000002, 139.000002, 10.0),
                (2200, 345690.0, 35.000003, 139.000003, 10.0),
            ]
            write_reference_csv(reference_csv, reference_rows)
            write_libgnss_pos(
                lib_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 4, 10, 1.0),
                    (2200, 345630.0, 35.0000011, 139.0000011, 10.0, 3, 9, 1.2),
                    (2200, 345660.0, 35.0000020, 139.0000021, 10.1, 1, 8, 1.5),
                    (2200, 345690.0, 35.0000030, 139.0000030, 10.0, 4, 10, 1.0),
                ],
            )
            write_rtklib_pos(
                rtklib_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 1, 10),
                    (2200, 345630.0, 35.0000012, 139.0000010, 10.0, 2, 9),
                    (2200, 345660.0, 35.0000022, 139.0000022, 10.0, 4, 8),
                    (2200, 345690.0, 35.0000031, 139.0000031, 10.0, 1, 10),
                ],
            )

            result = self.run_gnss(
                "driving-compare",
                "--lib-pos",
                str(lib_path),
                "--rtklib-pos",
                str(rtklib_path),
                "--reference-csv",
                str(reference_csv),
                "--output",
                str(output_png),
                "--rtklib-2d-output",
                str(rtklib_2d),
                "--lib-2d-output",
                str(lib_2d),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_png.exists())
            self.assertTrue(rtklib_2d.exists())
            self.assertTrue(lib_2d.exists())

    def test_scorecard_and_social_card_generate_pngs_from_synthetic_inputs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_cards_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            lib_path = temp_root / "lib.pos"
            rtklib_path = temp_root / "rtklib.pos"
            reference_csv = temp_root / "reference.csv"
            scorecard_png = temp_root / "scorecard.png"
            social_png = temp_root / "social.png"
            reference_rows = [
                (2200, 345600.0, 35.0, 139.0, 10.0),
                (2200, 345630.0, 35.000001, 139.000001, 10.0),
                (2200, 345660.0, 35.000002, 139.000002, 10.0),
            ]
            write_reference_csv(reference_csv, reference_rows)
            write_libgnss_pos(
                lib_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 4, 10, 1.0),
                    (2200, 345630.0, 35.0000011, 139.0000010, 10.0, 4, 10, 1.0),
                    (2200, 345660.0, 35.0000021, 139.0000020, 10.0, 4, 10, 1.0),
                ],
            )
            write_rtklib_pos(
                rtklib_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 1, 10),
                    (2200, 345630.0, 35.0000014, 139.0000014, 10.0, 1, 10),
                    (2200, 345660.0, 35.0000026, 139.0000026, 10.0, 1, 10),
                ],
            )

            scorecard_result = self.run_gnss(
                "scorecard",
                "--lib-pos",
                str(lib_path),
                "--rtklib-pos",
                str(rtklib_path),
                "--reference-csv",
                str(reference_csv),
                "--output",
                str(scorecard_png),
            )
            self.assertEqual(scorecard_result.returncode, 0, msg=scorecard_result.stderr)
            self.assertTrue(scorecard_png.exists())

            social_result = self.run_gnss(
                "social-card",
                "--lib-pos",
                str(lib_path),
                "--rtklib-pos",
                str(rtklib_path),
                "--reference-csv",
                str(reference_csv),
                "--output",
                str(social_png),
            )
            self.assertEqual(social_result.returncode, 0, msg=social_result.stderr)
            self.assertIn("Saved:", social_result.stdout)
            self.assertTrue(social_png.exists())

    def test_odaiba_benchmark_help_is_available(self) -> None:
        result = self.run_gnss("odaiba-benchmark", "--help")
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("--summary-json", result.stdout)
        self.assertIn("--require-all-epochs-min", result.stdout)

    def test_odaiba_scan_help_is_available(self) -> None:
        result = self.run_gnss("odaiba-scan", "--help")
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("--window-size", result.stdout)
        self.assertIn("--output-csv", result.stdout)

    def test_ros2_solution_node_runs_via_dispatcher_when_built(self) -> None:
        if not ros2_solution_node_exists():
            self.skipTest("gnss_solution_node is not built in this environment")
        with tempfile.TemporaryDirectory(prefix="gnss_ros2_dispatcher_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            solution_path = temp_root / "sample.pos"
            solution_path.write_text(
                "% synthetic solution\n"
                "1316 518400.0 -3978242.0 3382841.0 3649903.0 35.0 139.0 10.0 4 9 1.0\n"
                "1316 518430.0 -3978243.0 3382840.0 3649902.0 35.0 139.0 10.0 6 10 2.5\n",
                encoding="ascii",
            )
            result = self.run_gnss(
                "ros2-solution-node",
                "--ros-args",
                "-p",
                f"solution_file:={solution_path}",
                "-p",
                "publish_period_ms:=1",
                "-p",
                "max_messages:=2",
            )
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("published_messages=2", result.stdout + result.stderr)

    def test_ppp_cli_processes_synthetic_precise_products(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, true_position = build_synthetic_ppp_inputs(temp_root)
            out_path = temp_root / "ppp_solution.pos"

            result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--no-estimate-troposphere",
                "--out",
                str(out_path),
                "--max-epochs",
                "8",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("PPP summary:", result.stdout)
            self.assertIn("PPP float solutions: 8", result.stdout)
            self.assertIn("PPP fixed solutions: 0", result.stdout)
            self.assertIn("fallback solutions: 0", result.stdout)
            self.assertIn("mode: static", result.stdout)

            records = self.read_pos_records(out_path)
            self.assertEqual(len(records), 8)
            self.assertTrue(all(record["status"] == 5 for record in records))
            self.assertTrue(all(record["satellites"] >= 6 for record in records))

            last_record = records[-1]
            error = math.sqrt(
                (last_record["x"] - true_position[0]) ** 2
                + (last_record["y"] - true_position[1]) ** 2
                + (last_record["z"] - true_position[2]) ** 2
            )
            self.assertLess(error, 1.0)

    def test_ppp_cli_accepts_ionex_and_dcb_products(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_bias_products_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, _true_position = build_synthetic_ppp_inputs(temp_root)
            ionex_path = temp_root / "synthetic.ionex"
            dcb_path = temp_root / "synthetic.bsx"
            out_path = temp_root / "ppp_with_bias_products.pos"
            summary_path = temp_root / "ppp_with_bias_products.json"
            ionex_path.write_text(build_synthetic_ionex_text(), encoding="ascii")
            dcb_path.write_text(build_synthetic_dcb_text(), encoding="ascii")

            result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--ionex",
                str(ionex_path),
                "--dcb",
                str(dcb_path),
                "--no-estimate-troposphere",
                "--out",
                str(out_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "4",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("ionex maps: 2", result.stdout)
            self.assertIn("dcb entries: 4", result.stdout)
            self.assertNotIn("ionex corrections: 0", result.stdout)
            self.assertNotIn("dcb corrections: 0", result.stdout)
            self.assertTrue(out_path.exists())
            self.assertTrue(summary_path.exists())
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertTrue(payload["ionex_loaded"])
            self.assertTrue(payload["dcb_loaded"])
            self.assertGreater(payload["ionex_corrections"], 0)
            self.assertGreater(payload["dcb_corrections"], 0)
            self.assertEqual(payload["valid_solutions"], 4)
            records = self.read_pos_records(out_path)
            self.assertEqual(len(records), 4)

    def test_ppp_cli_supports_receiver_antex_offsets(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_antex_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, true_position = build_synthetic_ppp_inputs(
                temp_root,
                include_antenna_header=True,
            )
            antex_path = temp_root / "receiver.atx"
            antex_path.write_text(build_synthetic_receiver_antex_text(), encoding="ascii")
            base_out_path = temp_root / "ppp_base.pos"
            antex_out_path = temp_root / "ppp_antex.pos"

            base_result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--no-estimate-troposphere",
                "--out",
                str(base_out_path),
                "--max-epochs",
                "8",
            )
            antex_result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--antex",
                str(antex_path),
                "--no-estimate-troposphere",
                "--out",
                str(antex_out_path),
                "--max-epochs",
                "8",
            )

            self.assertEqual(base_result.returncode, 0, msg=base_result.stderr)
            self.assertEqual(antex_result.returncode, 0, msg=antex_result.stderr)
            self.assertIn("PPP float solutions: 8", antex_result.stdout)

            base_records = self.read_pos_records(base_out_path)
            antex_records = self.read_pos_records(antex_out_path)
            self.assertEqual(len(base_records), 8)
            self.assertEqual(len(antex_records), 8)
            self.assertTrue(all(record["status"] == 5 for record in base_records))
            self.assertTrue(all(record["status"] == 5 for record in antex_records))

            base_last = base_records[-1]
            antex_last = antex_records[-1]
            base_error = math.sqrt(
                (base_last["x"] - true_position[0]) ** 2
                + (base_last["y"] - true_position[1]) ** 2
                + (base_last["z"] - true_position[2]) ** 2
            )
            antex_error = math.sqrt(
                (antex_last["x"] - true_position[0]) ** 2
                + (antex_last["y"] - true_position[1]) ** 2
                + (antex_last["z"] - true_position[2]) ** 2
            )
            solution_delta = math.sqrt(
                (antex_last["x"] - base_last["x"]) ** 2
                + (antex_last["y"] - base_last["y"]) ** 2
                + (antex_last["z"] - base_last["z"]) ** 2
            )

            self.assertLess(base_error, 1.5)
            self.assertLess(antex_error, 1.5)
            self.assertGreater(solution_delta, 1e-4)
            self.assertLess(solution_delta, 1.0)

    def test_ppp_cli_supports_ocean_loading_coefficients(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_blq_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, true_position = build_synthetic_ppp_inputs(temp_root)
            blq_path = temp_root / "site.blq"
            blq_path.write_text(build_synthetic_blq_text(), encoding="ascii")
            base_out_path = temp_root / "ppp_base.pos"
            blq_out_path = temp_root / "ppp_blq.pos"

            base_result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--no-estimate-troposphere",
                "--out",
                str(base_out_path),
                "--max-epochs",
                "8",
            )
            blq_result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--blq",
                str(blq_path),
                "--no-estimate-troposphere",
                "--out",
                str(blq_out_path),
                "--max-epochs",
                "8",
            )

            self.assertEqual(base_result.returncode, 0, msg=base_result.stderr)
            self.assertEqual(blq_result.returncode, 0, msg=blq_result.stderr)
            base_records = self.read_pos_records(base_out_path)
            blq_records = self.read_pos_records(blq_out_path)
            self.assertEqual(len(base_records), 8)
            self.assertEqual(len(blq_records), 8)

            base_last = base_records[-1]
            blq_last = blq_records[-1]
            base_error = math.sqrt(
                (base_last["x"] - true_position[0]) ** 2
                + (base_last["y"] - true_position[1]) ** 2
                + (base_last["z"] - true_position[2]) ** 2
            )
            blq_error = math.sqrt(
                (blq_last["x"] - true_position[0]) ** 2
                + (blq_last["y"] - true_position[1]) ** 2
                + (blq_last["z"] - true_position[2]) ** 2
            )
            solution_delta = math.sqrt(
                (blq_last["x"] - base_last["x"]) ** 2
                + (blq_last["y"] - base_last["y"]) ** 2
                + (blq_last["z"] - base_last["z"]) ** 2
            )

            self.assertLess(base_error, 1.5)
            self.assertLess(blq_error, 1.5)
            self.assertGreater(solution_delta, 1e-5)
            self.assertLess(solution_delta, 0.5)

    def test_ppp_cli_supports_kinematic_mode(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_kinematic_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, _ = build_synthetic_ppp_inputs(temp_root)
            out_path = temp_root / "ppp_kinematic_solution.pos"

            result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--no-estimate-troposphere",
                "--out",
                str(out_path),
                "--max-epochs",
                "4",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("mode: kinematic", result.stdout)
            self.assertIn("PPP float solutions: 4", result.stdout)
            records = self.read_pos_records(out_path)
            self.assertEqual(len(records), 4)
            self.assertTrue(all(record["status"] == 5 for record in records))

    def test_ppp_cli_can_enable_ambiguity_resolution(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_ar_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, _ = build_synthetic_ppp_inputs(temp_root)
            out_path = temp_root / "ppp_ar_solution.pos"

            result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--enable-ar",
                "--convergence-min-epochs",
                "4",
                "--ar-ratio-threshold",
                "2.0",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--no-estimate-troposphere",
                "--out",
                str(out_path),
                "--max-epochs",
                "8",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("ambiguity resolution: on", result.stdout)
            self.assertIn("PPP fixed solutions:", result.stdout)
            records = self.read_pos_records(out_path)
            self.assertEqual(len(records), 8)
            self.assertTrue(all(record["status"] in (5, 6) for record in records))

    def test_stream_relays_rtcm_frames(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_stream_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "input.rtcm3"
            output_path = temp_root / "relay.rtcm3"
            frame = build_rtcm1005(3875000.125, 332500.25, 5025000.75)
            input_path.write_bytes(frame)

            result = self.run_gnss(
                "stream",
                "--input",
                str(input_path),
                "--output",
                str(output_path),
                "--limit",
                "1",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Reference Station ARP", result.stdout)
            self.assertIn("summary: messages=1", result.stdout)
            self.assertEqual(output_path.read_bytes(), frame)

    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_stream_reads_rtcm_frame_from_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        frame = build_rtcm1005(14.0, 28.0, 42.0)

        def writer() -> None:
            time.sleep(0.05)
            os.write(master_fd, frame)
            time.sleep(0.1)
            os.close(master_fd)

        thread = threading.Thread(target=writer)
        thread.start()
        try:
            result = self.run_gnss(
                "stream",
                "--input",
                f"serial://{slave_path}?baud=115200",
                "--limit",
                "1",
            )
        finally:
            thread.join()

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("Reference Station ARP", result.stdout)
        self.assertIn("summary: messages=1", result.stdout)

    @unittest.skipIf(os.name == "nt", "TCP source test is POSIX-only in the current build")
    def test_stream_reads_rtcm_frame_from_tcp_source(self) -> None:
        frame = build_rtcm1005(14.5, 29.0, 43.5)

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.bind(("127.0.0.1", 0))
            server.listen(1)
            server.settimeout(2.0)
            port = server.getsockname()[1]

            def writer() -> None:
                conn, _ = server.accept()
                with conn:
                    conn.sendall(frame)

            thread = threading.Thread(target=writer)
            thread.start()
            try:
                result = self.run_gnss(
                    "stream",
                    "--input",
                    f"tcp://127.0.0.1:{port}",
                    "--limit",
                    "1",
                )
            finally:
                thread.join()

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("Reference Station ARP", result.stdout)
        self.assertIn("summary: messages=1", result.stdout)

    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_stream_relays_rtcm_frame_to_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        frame = build_rtcm1005(15.0, 30.0, 45.0)

        with tempfile.TemporaryDirectory(prefix="gnss_stream_serial_out_test_") as temp_dir:
            input_path = Path(temp_dir) / "input.rtcm3"
            input_path.write_bytes(frame)
            received = bytearray()

            def reader() -> None:
                deadline = time.time() + 2.0
                try:
                    while len(received) < len(frame) and time.time() < deadline:
                        try:
                            chunk = os.read(master_fd, 1024)
                        except OSError as exc:
                            if exc.errno == 5:
                                time.sleep(0.02)
                                continue
                            raise
                        if not chunk:
                            time.sleep(0.02)
                            continue
                        received.extend(chunk)
                finally:
                    os.close(master_fd)

            thread = threading.Thread(target=reader)
            thread.start()
            try:
                result = self.run_gnss(
                    "stream",
                    "--input",
                    str(input_path),
                    "--output",
                    f"serial://{slave_path}?baud=115200",
                    "--limit",
                    "1",
                    "--quiet",
                )
            finally:
                thread.join()

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: messages=1", result.stdout)
            self.assertEqual(bytes(received), frame)

    @unittest.skipIf(os.name == "nt", "TCP sink relay is POSIX-only in the current build")
    def test_stream_relays_rtcm_frame_to_tcp_sink(self) -> None:
        frame = build_rtcm1005(16.0, 32.0, 48.0)

        with tempfile.TemporaryDirectory(prefix="gnss_stream_tcp_out_test_") as temp_dir:
            input_path = Path(temp_dir) / "input.rtcm3"
            input_path.write_bytes(frame)

            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                server.bind(("127.0.0.1", 0))
                server.listen(1)
                server.settimeout(2.0)
                port = server.getsockname()[1]
                received = bytearray()

                def reader() -> None:
                    conn, _ = server.accept()
                    with conn:
                        conn.settimeout(2.0)
                        while len(received) < len(frame):
                            chunk = conn.recv(1024)
                            if not chunk:
                                break
                            received.extend(chunk)

                thread = threading.Thread(target=reader)
                thread.start()
                try:
                    result = self.run_gnss(
                        "stream",
                        "--input",
                        str(input_path),
                        "--output",
                        f"tcp://127.0.0.1:{port}",
                        "--limit",
                        "1",
                        "--quiet",
                    )
                finally:
                    thread.join()

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: messages=1", result.stdout)
            self.assertEqual(bytes(received), frame)

    def test_ubx_info_decodes_and_exports_rawx(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ubx_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.ubx"
            output_path = temp_root / "session.obs"
            input_path.write_bytes(
                build_nav_pvt_message() + build_sfrbx_message() + build_mixed_rawx_message()
            )

            result = self.run_gnss(
                "ubx-info",
                "--input",
                str(input_path),
                "--decode-nav",
                "--decode-observations",
                "--obs-rinex-out",
                str(output_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("UBX-NAV-PVT", result.stdout)
            self.assertIn("UBX-RXM-SFRBX", result.stdout)
            self.assertIn("UBX-RXM-RAWX", result.stdout)
            self.assertIn("nav: fix_type=3", result.stdout)
            self.assertIn("subframe: system=GPS sv=12 words=3 kind=GPS_LNAV frame_id=5", result.stdout)
            self.assertIn("obs: week=2200", result.stdout)
            self.assertIn("summary: processed_messages=3", result.stdout)
            self.assertIn("sfrbx=1", result.stdout)
            self.assertTrue(output_path.exists())
            self.assertGreater(output_path.stat().st_size, 0)
            exported = output_path.read_text(encoding="ascii")
            self.assertIn("RINEX VERSION / TYPE", exported)
            self.assertIn("G12", exported)
            self.assertIn("E05", exported)
            self.assertIn("R07", exported)
            self.assertIn("C19", exported)
            self.assertIn("J03", exported)

    def test_convert_converts_ubx_into_observation_rinex(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_convert_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.ubx"
            output_path = temp_root / "converted.obs"
            input_path.write_bytes(build_nav_pvt_message() + build_mixed_rawx_message())

            result = self.run_gnss(
                "convert",
                "--format",
                "ubx",
                "--input",
                str(input_path),
                "--obs-out",
                str(output_path),
                "--quiet",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: processed_messages=2", result.stdout)
            self.assertIn("exported_obs_epochs=1", result.stdout)
            self.assertTrue(output_path.exists())
            exported = output_path.read_text(encoding="ascii")
            self.assertIn("RINEX VERSION / TYPE", exported)
            self.assertIn("G12", exported)
            self.assertIn("E05", exported)
            self.assertIn("R07", exported)
            self.assertIn("C19", exported)
            self.assertIn("J03", exported)

    def test_nmea_info_decodes_gga_and_rmc_from_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_nmea_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.nmea"
            input_path.write_text(
                build_nmea_gga_sentence() + build_nmea_rmc_sentence(),
                encoding="ascii",
            )

            result = self.run_gnss(
                "nmea-info",
                "--input",
                str(input_path),
                "--decode-gga",
                "--decode-rmc",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("gga: time=123519", result.stdout)
            self.assertIn("quality=4", result.stdout)
            self.assertIn("rmc: time=123520", result.stdout)
            self.assertIn("status=A", result.stdout)
            self.assertIn(
                "summary: sentences=2 valid=2 gga=1 rmc=1 valid_positions=2 checksum_errors=0",
                result.stdout,
            )

    def test_novatel_info_decodes_bestpos_and_bestvel_from_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_novatel_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.log"
            input_path.write_text(
                build_novatel_bestpos_record() + build_novatel_bestvel_record(),
                encoding="ascii",
            )

            result = self.run_gnss(
                "novatel-info",
                "--input",
                str(input_path),
                "--decode-bestpos",
                "--decode-bestvel",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("bestpos: week=2200 tow=345600.000", result.stdout)
            self.assertIn("type=NARROW_INT", result.stdout)
            self.assertIn("bestvel: week=2200 tow=345600.000", result.stdout)
            self.assertIn("speed_mps=5.500", result.stdout)
            self.assertIn(
                "summary: records=2 valid=2 bestpos=1 bestvel=1 valid_positions=1 checksum_errors=0",
                result.stdout,
            )

    def test_novatel_info_decodes_binary_bestpos_and_bestvel_from_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_novatel_binary_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.bin"
            input_path.write_bytes(
                build_novatel_bestpos_binary_record() + build_novatel_bestvel_binary_record()
            )

            result = self.run_gnss(
                "novatel-info",
                "--input",
                str(input_path),
                "--decode-bestpos",
                "--decode-bestvel",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("bestpos: week=2200 tow=345600.000", result.stdout)
            self.assertIn("type=NARROW_INT", result.stdout)
            self.assertIn("bestvel: week=2200 tow=345600.000", result.stdout)
            self.assertIn("type=DOPPLER_VELOCITY", result.stdout)
            self.assertIn(
                "summary: records=2 valid=2 bestpos=1 bestvel=1 valid_positions=1 checksum_errors=0",
                result.stdout,
            )

    def test_sbp_info_decodes_gps_time_pos_llh_and_vel_ned_from_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_sbp_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.sbp"
            input_path.write_bytes(
                build_sbp_gps_time_frame() + build_sbp_pos_llh_frame() + build_sbp_vel_ned_frame()
            )

            result = self.run_gnss(
                "sbp-info",
                "--input",
                str(input_path),
                "--decode-time",
                "--decode-pos",
                "--decode-vel",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("gps_time: sender=66 week=2200 tow_ms=345600123", result.stdout)
            self.assertIn("ns_residual=-250", result.stdout)
            self.assertIn("pos_llh: sender=66 tow_ms=345600123", result.stdout)
            self.assertIn("lat=35.1234567", result.stdout)
            self.assertIn("height=42.100m", result.stdout)
            self.assertIn("vel_ned: sender=66 tow_ms=345600123", result.stdout)
            self.assertIn("n=1.250 e=-0.500 d=0.125mps", result.stdout)
            self.assertIn(
                "summary: frames=3 valid=3 gps_time=1 pos_llh=1 vel_ned=1 valid_positions=1 crc_errors=0",
                result.stdout,
            )

    def test_sbf_info_decodes_pvt_lband_and_p2pp_from_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_sbf_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.sbf"
            input_path.write_bytes(
                build_sbf_pvt_geodetic_frame()
                + build_sbf_lband_tracker_frame()
                + build_sbf_p2pp_status_frame()
            )

            result = self.run_gnss(
                "sbf-info",
                "--input",
                str(input_path),
                "--decode-pvt",
                "--decode-lband",
                "--decode-p2pp",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("pvt_geodetic: week=2200 tow_ms=345600123 mode=PPP", result.stdout)
            self.assertIn("lat=35.1234567", result.stdout)
            self.assertIn("lband_tracker: week=2200 tow_ms=345600123", result.stdout)
            self.assertIn("locked=1", result.stdout)
            self.assertIn("service=42", result.stdout)
            self.assertIn("p2pp_status: week=2200 tow_ms=345600123", result.stdout)
            self.assertIn("status=Connected", result.stdout)
            self.assertIn(
                "summary: frames=3 valid=3 pvt_geodetic=1 lband_tracker=1 p2pp_status=1 valid_positions=1 crc_errors=0",
                result.stdout,
            )

    def test_trimble_info_decodes_time_llh_and_velocity_from_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_trimble_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.gsof"
            input_path.write_bytes(
                build_gsof_genout_packet(
                    [
                        build_gsof_time_record(),
                        build_gsof_llh_record(),
                        build_gsof_velocity_record(),
                    ]
                )
            )

            result = self.run_gnss(
                "trimble-info",
                "--input",
                str(input_path),
                "--decode-time",
                "--decode-llh",
                "--decode-vel",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("time: week=2200 tow=345600.123 svs=18", result.stdout)
            self.assertIn("llh: lat=35.1234567 lon=139.9876543 height=42.100m", result.stdout)
            self.assertIn("velocity: flags=0x03 horiz=1.250mps heading=90.00deg vertical=-0.125mps", result.stdout)
            self.assertIn(
                "summary: packets=1 valid=1 time=1 llh=1 velocity=1 valid_positions=1 checksum_errors=0",
                result.stdout,
            )

    def test_skytraq_info_decodes_epoch_rawx_and_ack_from_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_skytraq_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.stq"
            input_path.write_bytes(
                build_skytraq_epoch_message()
                + build_skytraq_rawx_message()
                + build_skytraq_ack_message()
            )

            result = self.run_gnss(
                "skytraq-info",
                "--input",
                str(input_path),
                "--decode-epoch",
                "--decode-raw",
                "--decode-ack",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("epoch: iod=7 week=2200 tow=345600.123", result.stdout)
            self.assertIn("rawx: version=1 iod=7 week=2200 tow=345600.123 period=1.000 nsat=14", result.stdout)
            self.assertIn("ack: msg=0x1E", result.stdout)
            self.assertIn(
                "summary: frames=3 valid=3 epoch=1 raw=0 rawx=1 ack=1 nack=0 checksum_errors=0",
                result.stdout,
            )

    def test_binex_info_decodes_metadata_nav_and_proto_from_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_binex_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.bnx"
            input_path.write_bytes(
                build_binex_metadata_frame() + build_binex_nav_frame() + build_binex_proto_frame()
            )

            result = self.run_gnss(
                "binex-info",
                "--input",
                str(input_path),
                "--decode-metadata",
                "--decode-nav",
                "--decode-proto",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("metadata: subrecord=0x08 payload_bytes=4", result.stdout)
            self.assertIn("nav: subrecord=0x06 payload_bytes=5", result.stdout)
            self.assertIn("proto: subrecord=0x05 payload_bytes=4", result.stdout)
            self.assertIn(
                "summary: frames=3 valid=3 metadata=1 nav=1 proto=1 checksum_errors=0",
                result.stdout,
            )

    def test_qzss_l6_info_decodes_frames_and_extracts_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6.bin"
            extract_path = temp_root / "session_l6.csv"
            input_path.write_bytes(
                build_qzss_l6_frame(
                    prn=199,
                    facility_id=0,
                    subframe_start=True,
                    data_part=b"CLAS-L6-START",
                )
                + build_qzss_l6_frame(
                    prn=200,
                    facility_id=2,
                    subframe_start=False,
                    alert=True,
                    data_part=b"CLAS-L6-NEXT",
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--show-preview",
                "--extract-data-parts",
                str(extract_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("l6_frame: index=1 prn=199 vendor=5 facility=Hitachi-Ota", result.stdout)
            self.assertIn("subframe_start=1", result.stdout)
            self.assertIn("preview=CLAS-L6-START", result.stdout)
            self.assertIn("l6_frame: index=2 prn=200 vendor=5 facility=Kobe", result.stdout)
            self.assertIn("alert=1", result.stdout)
            self.assertIn("extracted: frames=2", result.stdout)
            self.assertIn(
                "summary: frames=2 valid=2 clas_vendor=2 subframe_starts=1 alerts=1 subframes=0 prns=199,200",
                result.stdout,
            )

            extracted = extract_path.read_text(encoding="ascii")
            self.assertIn("frame_index,prn,vendor_id,facility_id,reserved_bits,subframe_start,alert,data_part_bits,data_part_hex,rs_parity_hex", extracted)
            self.assertIn("1,199,5,0,0,1,0,1695,", extracted)
            self.assertIn("2,200,5,2,0,0,1,1695,", extracted)

    def test_qzss_l6_info_assembles_five_part_subframe(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_subframe_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_subframe.bin"
            subframe_path = temp_root / "session_l6_subframes.csv"
            input_path.write_bytes(
                b"".join(
                    build_qzss_l6_frame(
                        prn=199,
                        facility_id=0,
                        subframe_start=index == 0,
                        data_part=f"SUBFRAME-{index}".encode("ascii"),
                    )
                    for index in range(5)
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--limit",
                "5",
                "--extract-subframes",
                str(subframe_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("l6_subframe: index=1 prn=199 vendor=5 facility=Hitachi-Ota frames=5", result.stdout)
            self.assertIn("assembled: subframes=1", result.stdout)
            self.assertIn(
                "summary: frames=5 valid=5 clas_vendor=5 subframe_starts=1 alerts=0 subframes=1 prns=199",
                result.stdout,
            )

            exported = subframe_path.read_text(encoding="ascii")
            self.assertIn(
                "subframe_index,prn,vendor_id,facility_id,frame_count,alert_frames,data_bits,first_frame_index,last_frame_index,subframe_hex",
                exported,
            )
            self.assertIn("1,199,5,0,5,0,8475,1,5,", exported)

    def test_qzss_l6_info_extracts_compact_cssr_messages_and_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_compact_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_compact.bin"
            messages_path = temp_root / "session_compact_messages.csv"
            corrections_path = temp_root / "session_compact_corrections.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_combined_message(
                            tow_delta=0,
                            iod=3,
                            prn=3,
                            sync=False,
                            network_id=1,
                            dclock_m=0.025,
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--limit",
                "5",
                "--gps-week",
                "1316",
                "--extract-compact-messages",
                str(messages_path),
                "--extract-compact-corrections",
                str(corrections_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("cssr_message: subframe=1 index=1 subtype=1 tow=518400", result.stdout)
            self.assertIn("cssr_message: subframe=1 index=2 subtype=11 tow=518400", result.stdout)
            self.assertIn("decoded: compact_messages=2", result.stdout)
            self.assertIn("extracted: compact_corrections=1", result.stdout)
            messages_csv = messages_path.read_text(encoding="ascii")
            self.assertIn(
                "subframe_index,message_index,ctype,subtype,tow,udi_seconds,sync,iod,message_bits,correction_count,detail",
                messages_csv,
            )
            self.assertIn("1,1,4073,1,518400,1.0,1,3,", messages_csv)
            self.assertIn("1,2,4073,11,518400,1.0,0,3,", messages_csv)
            corrections_csv = corrections_path.read_text(encoding="ascii")
            self.assertIn("# week,tow,system,prn,dx,dy,dz,dclock_m,high_rate_clock_m", corrections_csv)
            self.assertIn("1316,518400.000,G,3,0.000000,0.000000,0.000000,0.025600,0.000000", corrections_csv)

    def test_qzss_l6_info_extracts_separate_orbit_clock_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_orbit_clock_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_orbit_clock.bin"
            corrections_path = temp_root / "session_orbit_clock_corrections.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_orbit_message(
                            tow_delta=0,
                            iod=3,
                            dx=0.016,
                            dy=0.0128,
                            dz=-0.0128,
                            sync=True,
                        ),
                        build_qzss_cssr_clock_message(tow_delta=0, iod=3, dclock_m=0.0256, sync=False),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--limit",
                "5",
                "--gps-week",
                "1316",
                "--extract-compact-corrections",
                str(corrections_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("cssr_message: subframe=1 index=2 subtype=2 tow=518400", result.stdout)
            self.assertIn("cssr_message: subframe=1 index=3 subtype=3 tow=518400", result.stdout)
            corrections_csv = corrections_path.read_text(encoding="ascii")
            self.assertIn("1316,518400.000,G,3,0.016000,0.012800,-0.012800,0.025600,0.000000", corrections_csv)

    def test_qzss_l6_info_extracts_code_bias_and_ura_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_cbias_ura_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_cbias_ura.bin"
            corrections_path = temp_root / "session_cbias_ura_corrections.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(
                            tow_delta=0,
                            iod=3,
                            bias_m=-0.12,
                            sync=True,
                        ),
                        build_qzss_cssr_ura_message(
                            tow_delta=0,
                            iod=3,
                            ura_index=9,
                            sync=True,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0,
                            iod=3,
                            prn=3,
                            dclock_m=0.0256,
                            sync=False,
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--limit",
                "5",
                "--gps-week",
                "1316",
                "--extract-compact-corrections",
                str(corrections_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("cssr_message: subframe=1 index=2 subtype=4 tow=518400", result.stdout)
            self.assertIn("cssr_message: subframe=1 index=3 subtype=7 tow=518400", result.stdout)
            corrections_csv = corrections_path.read_text(encoding="ascii")
            self.assertIn("ura_sigma_m=0.002750", corrections_csv)
            self.assertIn("cbias:2=-0.120000", corrections_csv)

    def test_qzss_l6_info_extracts_atmos_inventory(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_atmos_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_atmos.bin"
            messages_path = temp_root / "session_atmos_messages.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_atmos_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            network_id=1,
                            trop_avail=3,
                            stec_avail=3,
                            grid_count=1,
                            selected_satellites=1,
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--limit",
                "5",
                "--extract-compact-messages",
                str(messages_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("cssr_message: subframe=1 index=2 subtype=12 tow=518400", result.stdout)
            self.assertIn("detail=network=1 grids=1 trop=3 stec=3 sats=1", result.stdout)
            messages_csv = messages_path.read_text(encoding="ascii")
            self.assertIn("1,2,4073,12,518400,1.0,0,3,", messages_csv)
            self.assertIn("network=1 grids=1 trop=3 stec=3 sats=1", messages_csv)

    def test_qzss_l6_info_extracts_stec_inventory(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_stec_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_stec.bin"
            messages_path = temp_root / "session_stec_messages.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_stec_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            network_id=7,
                            stec_type=3,
                            selected_satellites=1,
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--limit",
                "5",
                "--extract-compact-messages",
                str(messages_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("cssr_message: subframe=1 index=2 subtype=8 tow=518400", result.stdout)
            self.assertIn("detail=network=7 stec_type=3 sats=1", result.stdout)
            messages_csv = messages_path.read_text(encoding="ascii")
            self.assertIn("1,2,4073,8,518400,1.0,0,3,", messages_csv)
            self.assertIn("network=7 stec_type=3 sats=1", messages_csv)

    def test_qzss_l6_info_extracts_service_info_packets(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_service_info_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_service_info.bin"
            messages_path = temp_root / "session_service_messages.csv"
            packets_path = temp_root / "session_service_packets.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_service_info_message(
                            sync=True,
                            info_counter=0,
                            payload_bytes=b"\x01\x02\x03\x04\x05",
                        ),
                        build_qzss_cssr_service_info_message(
                            sync=False,
                            info_counter=1,
                            payload_bytes=b"\x06\x07\x08\x09\x0a",
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-messages",
                str(messages_path),
                "--extract-service-info",
                str(packets_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("cssr_message: subframe=1 index=1 subtype=10 tow=0", result.stdout)
            self.assertIn("cssr_message: subframe=1 index=2 subtype=10 tow=0", result.stdout)
            self.assertIn("extracted: service_info_packets=1", result.stdout)
            messages_csv = messages_path.read_text(encoding="ascii")
            self.assertIn(",10,0,0.0,1,0,", messages_csv)
            self.assertIn(",10,0,0.0,0,1,", messages_csv)
            packets_csv = packets_path.read_text(encoding="ascii")
            self.assertIn("packet_index,first_subframe_index,last_subframe_index,chunk_count,total_bits,packet_hex", packets_csv)
            self.assertIn("1,1,1,2,80,0102030405060708090a", packets_csv)

    def test_qzss_l6_info_exports_atmos_metadata_on_compact_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_atmos_corr_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_atmos_corr.bin"
            corrections_path = temp_root / "session_atmos_corr.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_atmos_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_id=1,
                            trop_avail=3,
                            stec_avail=3,
                            grid_count=1,
                            trop_quality=9,
                            trop_type=2,
                            trop_t00_m=0.12,
                            trop_t01_m_per_deg=0.01,
                            trop_t10_m_per_deg=-0.02,
                            trop_t11_m_per_deg2=0.003,
                            trop_residual_size=0,
                            trop_offset_m=0.08,
                            trop_residuals_m=(0.02,),
                            stec_quality=17,
                            stec_type=3,
                            stec_c00_tecu=1.5,
                            stec_c01_tecu_per_deg=0.12,
                            stec_c10_tecu_per_deg=-0.10,
                            stec_c11_tecu_per_deg2=0.06,
                            stec_c02_tecu_per_deg2=0.025,
                            stec_c20_tecu_per_deg2=-0.015,
                            stec_residual_size=2,
                            stec_residuals_tecu=(0.32,),
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(corrections_path),
                "--gps-week",
                "2200",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            corrections_csv = corrections_path.read_text(encoding="ascii")
            self.assertIn("atmos_network_id=1", corrections_csv)
            self.assertIn("atmos_trop_avail=3", corrections_csv)
            self.assertIn("atmos_stec_avail=3", corrections_csv)
            self.assertIn("atmos_grid_count=1", corrections_csv)
            self.assertIn("atmos_trop_quality=9", corrections_csv)
            self.assertIn("atmos_trop_type=2", corrections_csv)
            self.assertIn("atmos_trop_t00_m=0.120000", corrections_csv)
            self.assertIn("atmos_trop_t01_m_per_deg=0.010000", corrections_csv)
            self.assertIn("atmos_trop_t10_m_per_deg=-0.020000", corrections_csv)
            self.assertIn("atmos_trop_t11_m_per_deg2=0.003000", corrections_csv)
            self.assertIn("atmos_trop_offset_m=0.080000", corrections_csv)
            self.assertIn("atmos_trop_residuals_m=0.020000", corrections_csv)
            self.assertIn("atmos_stec_quality:G03=17", corrections_csv)
            self.assertIn("atmos_stec_type:G03=3", corrections_csv)
            self.assertIn("atmos_stec_c00_tecu:G03=1.500000", corrections_csv)
            self.assertIn("atmos_stec_c01_tecu_per_deg:G03=0.120000", corrections_csv)
            self.assertIn("atmos_stec_c10_tecu_per_deg:G03=-0.100000", corrections_csv)
            self.assertIn("atmos_stec_c11_tecu_per_deg2:G03=0.060000", corrections_csv)
            self.assertIn("atmos_stec_c02_tecu_per_deg2:G03=0.025000", corrections_csv)
            self.assertIn("atmos_stec_c20_tecu_per_deg2:G03=-0.015000", corrections_csv)
            self.assertIn("atmos_stec_residual_size:G03=2", corrections_csv)
            self.assertIn("atmos_stec_residuals_tecu:G03=0.320000", corrections_csv)

    def test_qzss_l6_info_exports_stec_metadata_on_compact_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_stec_corr_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_stec_corr.bin"
            corrections_path = temp_root / "session_stec_corr.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_stec_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_id=7,
                            stec_quality=17,
                            stec_type=3,
                            stec_c00_tecu=1.5,
                            stec_c01_tecu_per_deg=0.12,
                            stec_c10_tecu_per_deg=-0.10,
                            stec_c11_tecu_per_deg2=0.06,
                            stec_c02_tecu_per_deg2=0.025,
                            stec_c20_tecu_per_deg2=-0.015,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(corrections_path),
                "--gps-week",
                "2200",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            corrections_csv = corrections_path.read_text(encoding="ascii")
            self.assertIn("atmos_network_id=7", corrections_csv)
            self.assertIn("atmos_trop_avail=0", corrections_csv)
            self.assertIn("atmos_stec_avail=1", corrections_csv)
            self.assertIn("atmos_grid_count=0", corrections_csv)
            self.assertIn("atmos_stec_quality:G03=17", corrections_csv)
            self.assertIn("atmos_stec_type:G03=3", corrections_csv)
            self.assertIn("atmos_stec_c00_tecu:G03=1.500000", corrections_csv)
            self.assertIn("atmos_stec_c01_tecu_per_deg:G03=0.120000", corrections_csv)
            self.assertIn("atmos_stec_c10_tecu_per_deg:G03=-0.100000", corrections_csv)
            self.assertIn("atmos_stec_c11_tecu_per_deg2:G03=0.060000", corrections_csv)
            self.assertIn("atmos_stec_c02_tecu_per_deg2:G03=0.025000", corrections_csv)
            self.assertIn("atmos_stec_c20_tecu_per_deg2:G03=-0.015000", corrections_csv)

    def test_qzss_l6_info_exports_gridded_metadata_on_compact_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_gridded_corr_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_gridded_corr.bin"
            corrections_path = temp_root / "session_gridded_corr.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_gridded_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_id=7,
                            trop_type=1,
                            trop_quality=9,
                            grid_count=2,
                            selected_satellites=1,
                            trop_hs_residuals_m=(0.100, 0.200),
                            trop_wet_residuals_m=(-0.020, 0.032),
                            stec_residuals_tecu=(0.40, -0.28),
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(corrections_path),
                "--gps-week",
                "2200",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            corrections_csv = corrections_path.read_text(encoding="ascii")
            self.assertIn("atmos_network_id=7", corrections_csv)
            self.assertIn("atmos_trop_type=1", corrections_csv)
            self.assertIn("atmos_trop_quality=9", corrections_csv)
            self.assertIn("atmos_grid_count=2", corrections_csv)
            self.assertIn("atmos_trop_hs_residuals_m=0.100000;0.200000", corrections_csv)
            self.assertIn("atmos_trop_wet_residuals_m=-0.020000;0.032000", corrections_csv)
            self.assertIn("atmos_stec_residual_range=0", corrections_csv)
            self.assertIn("atmos_stec_residuals_tecu:G03=0.400000;-0.280000", corrections_csv)

    def test_qzss_l6_info_extracts_code_phase_bias_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_pbias_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_pbias.bin"
            messages_path = temp_root / "session_pbias_messages.csv"
            corrections_path = temp_root / "session_pbias_corrections.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0, iod=3, code_bias_m=-0.12, phase_bias_m=0.015, sync=True
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-messages",
                str(messages_path),
                "--extract-compact-corrections",
                str(corrections_path),
                "--gps-week",
                "2200",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("cssr_message: subframe=1 index=2 subtype=6 tow=518400", result.stdout)
            self.assertIn("mapped_code=1 mapped_phase=1", result.stdout)
            corrections_csv = corrections_path.read_text(encoding="ascii")
            self.assertIn("cbias:2=-0.120000", corrections_csv)
            self.assertIn("pbias:2=0.015000", corrections_csv)
            messages_csv = messages_path.read_text(encoding="ascii")
            self.assertIn(",6,518400,", messages_csv)

    def test_qzss_l6_info_extracts_phase_bias_only_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_phase_bias_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_phase_bias.bin"
            messages_path = temp_root / "session_phase_bias_messages.csv"
            corrections_path = temp_root / "session_phase_bias_corrections.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_phase_bias_message(
                            tow_delta=0, iod=3, phase_bias_m=0.015, sync=True
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-messages",
                str(messages_path),
                "--extract-compact-corrections",
                str(corrections_path),
                "--gps-week",
                "2200",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("cssr_message: subframe=1 index=2 subtype=5 tow=518400", result.stdout)
            self.assertIn("mapped_phase=1", result.stdout)
            corrections_csv = corrections_path.read_text(encoding="ascii")
            self.assertIn("pbias:2=0.015000", corrections_csv)
            messages_csv = messages_path.read_text(encoding="ascii")
            self.assertIn(",5,518400,", messages_csv)

    def test_solve_odaiba_slice_uses_glonass_and_beidou_in_rtk(self) -> None:
        rover = ROOT_DIR / "data/driving/Tokyo_Data/Odaiba/rover_trimble.obs"
        base = ROOT_DIR / "data/driving/Tokyo_Data/Odaiba/base_trimble.obs"
        nav = ROOT_DIR / "data/driving/Tokyo_Data/Odaiba/base.nav"
        reference_csv = ROOT_DIR / "data/driving/Tokyo_Data/Odaiba/reference.csv"

        self.assertTrue(rover.exists())
        self.assertTrue(base.exists())
        self.assertTrue(nav.exists())
        self.assertTrue(reference_csv.exists())

        with tempfile.TemporaryDirectory(prefix="gnss_solve_odaiba_test_") as temp_dir:
            temp_root = Path(temp_dir)
            cases = {
                "default": [],
                "no_glonass": ["--no-glonass"],
                "no_beidou": ["--no-beidou"],
                "gps_gal_qzss_only": ["--no-glonass", "--no-beidou"],
            }
            case_records: dict[str, list[dict[str, float | int]]] = {}

            for name, extra_args in cases.items():
                output_path = temp_root / f"{name}.pos"
                result = self.run_gnss(
                    "solve",
                    "--rover",
                    str(rover),
                    "--base",
                    str(base),
                    "--nav",
                    str(nav),
                    "--out",
                    str(output_path),
                    "--no-kml",
                    "--skip-epochs",
                    "0",
                    "--max-epochs",
                    "5",
                    *extra_args,
                )

                self.assertEqual(result.returncode, 0, msg=result.stderr)
                self.assertIn("valid solutions: 5", result.stdout)
                self.assertTrue(output_path.exists())

                records = self.read_pos_records(output_path)
                self.assertEqual(len(records), 5)
                self.assertTrue(all(int(record["status"]) > 0 for record in records))
                case_records[name] = records

            def average_satellites(name: str) -> float:
                records = case_records[name]
                return sum(int(record["satellites"]) for record in records) / len(records)

            default_avg = average_satellites("default")
            no_glonass_avg = average_satellites("no_glonass")
            no_beidou_avg = average_satellites("no_beidou")
            gps_gal_qzss_avg = average_satellites("gps_gal_qzss_only")

            self.assertGreater(default_avg, no_glonass_avg)
            self.assertGreater(default_avg, no_beidou_avg)
            self.assertGreater(no_glonass_avg, gps_gal_qzss_avg)
            self.assertGreater(no_beidou_avg, gps_gal_qzss_avg)

            reference = driving_comparison.read_reference_csv(reference_csv)
            default_epochs = driving_comparison.read_libgnss_pos(temp_root / "default.pos")
            default_matched = driving_comparison.match_to_reference(default_epochs, reference, 0.11)
            default_summary = driving_comparison.summarize(
                default_matched,
                fixed_status=4,
                label="odaiba default slice",
            )

            self.assertEqual(default_summary["epochs"], 5)
            self.assertLess(default_summary["p95_h_m"], 1.0)
            self.assertLess(default_summary["max_h_m"], 1.0)

    def test_solve_short_baseline_cli_reaches_fixed_solution(self) -> None:
        rover = ROOT_DIR / "data/short_baseline/TSK200JPN_R_20240010000_01D_30S_MO.rnx"
        base = ROOT_DIR / "data/short_baseline/TSKB00JPN_R_20240010000_01D_30S_MO.rnx"
        nav = ROOT_DIR / "data/short_baseline/BRDC00IGS_R_20240010000_01D_MN.rnx"

        self.assertTrue(rover.exists())
        self.assertTrue(base.exists())
        self.assertTrue(nav.exists())

        with tempfile.TemporaryDirectory(prefix="gnss_solve_short_baseline_test_") as temp_dir:
            output_path = Path(temp_dir) / "short_baseline.pos"
            result = self.run_gnss(
                "solve",
                "--rover",
                str(rover),
                "--base",
                str(base),
                "--nav",
                str(nav),
                "--out",
                str(output_path),
                "--no-kml",
                "--max-epochs",
                "10",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("valid solutions: 10", result.stdout)
            self.assertIn("fixed solutions:", result.stdout)
            self.assertTrue(output_path.exists())

            records = self.read_pos_records(output_path)
            self.assertEqual(len(records), 10)
            self.assertGreaterEqual(
                sum(int(record["status"]) == 4 for record in records),
                1,
            )
            self.assertGreaterEqual(
                sum(int(record["satellites"]) for record in records) / len(records),
                10.0,
            )

    def test_solve_cli_supports_estimated_iono_mode(self) -> None:
        rover = ROOT_DIR / "data/rover_kinematic.obs"
        base = ROOT_DIR / "data/base_kinematic.obs"
        nav = ROOT_DIR / "data/navigation_kinematic.nav"

        self.assertTrue(rover.exists())
        self.assertTrue(base.exists())
        self.assertTrue(nav.exists())

        with tempfile.TemporaryDirectory(prefix="gnss_solve_iono_est_") as temp_dir:
            output_path = Path(temp_dir) / "iono_est.pos"
            result = self.run_gnss(
                "solve",
                "--rover",
                str(rover),
                "--base",
                str(base),
                "--nav",
                str(nav),
                "--out",
                str(output_path),
                "--no-kml",
                "--mode",
                "kinematic",
                "--iono",
                "est",
                "--max-epochs",
                "5",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("iono: est", result.stdout)
            self.assertIn("valid solutions: 5", result.stdout)
            self.assertTrue(output_path.exists())

            records = self.read_pos_records(output_path)
            self.assertEqual(len(records), 5)
            self.assertTrue(all(int(record["status"]) > 0 for record in records))

    def test_solve_cli_supports_moving_base_mode(self) -> None:
        rover = ROOT_DIR / "data/rover_kinematic.obs"
        base = ROOT_DIR / "data/base_kinematic.obs"
        nav = ROOT_DIR / "data/navigation_kinematic.nav"

        self.assertTrue(rover.exists())
        self.assertTrue(base.exists())
        self.assertTrue(nav.exists())

        with tempfile.TemporaryDirectory(prefix="gnss_solve_moving_base_") as temp_dir:
            output_path = Path(temp_dir) / "moving_base.pos"
            result = self.run_gnss(
                "solve",
                "--rover",
                str(rover),
                "--base",
                str(base),
                "--nav",
                str(nav),
                "--out",
                str(output_path),
                "--no-kml",
                "--mode",
                "moving-base",
                "--max-epochs",
                "5",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("mode: moving-base", result.stdout)
            self.assertIn("valid solutions: 5", result.stdout)
            self.assertTrue(output_path.exists())

    def test_short_baseline_signoff_cli_writes_summary_and_passes_thresholds(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_short_baseline_signoff_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            output_path = temp_root / "short_baseline.pos"
            summary_path = temp_root / "short_baseline_summary.json"

            result = self.run_gnss(
                "short-baseline-signoff",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "120",
                "--require-fix-rate-min",
                "95",
                "--require-mean-error-max",
                "0.15",
                "--require-max-error-max",
                "0.60",
                "--require-mean-sats-min",
                "14",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Finished short-baseline sign-off.", result.stdout)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())

            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "Tsukuba short_baseline")
            self.assertEqual(payload["epochs"], 120)
            self.assertGreaterEqual(payload["fix_rate_pct"], 95.0)
            self.assertLessEqual(payload["mean_position_error_m"], 0.15)
            self.assertLessEqual(payload["max_position_error_m"], 0.60)
            self.assertGreaterEqual(payload["mean_satellites"], 14.0)

    def test_rtk_kinematic_signoff_cli_writes_summary_and_passes_thresholds(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rtk_kinematic_signoff_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            output_path = temp_root / "rtk_kinematic.pos"
            summary_path = temp_root / "rtk_kinematic_summary.json"

            result = self.run_gnss(
                "rtk-kinematic-signoff",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "120",
                "--require-valid-epochs-min",
                "120",
                "--require-fix-rate-min",
                "95",
                "--require-mean-error-max",
                "3.0",
                "--require-max-error-max",
                "3.0",
                "--require-mean-sats-min",
                "25",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Finished RTK kinematic sign-off.", result.stdout)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())

            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "sample mixed-GNSS kinematic RTK")
            self.assertEqual(payload["epochs"], 120)
            self.assertGreaterEqual(payload["fix_rate_pct"], 95.0)
            self.assertLessEqual(payload["mean_position_error_m"], 3.0)
            self.assertLessEqual(payload["max_position_error_m"], 3.0)
            self.assertGreaterEqual(payload["mean_satellites"], 25.0)

    def test_ppp_static_signoff_cli_writes_summary_and_passes_thresholds(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_static_signoff_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            output_path = temp_root / "ppp_static.pos"
            summary_path = temp_root / "ppp_static_summary.json"

            result = self.run_gnss(
                "ppp-static-signoff",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "120",
                "--require-valid-epochs-min",
                "120",
                "--require-mean-error-max",
                "1.5",
                "--require-max-error-max",
                "1.5",
                "--require-mean-sats-min",
                "6.0",
                "--require-ppp-solution-rate-min",
                "100.0",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Finished PPP static sign-off.", result.stdout)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())

            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "sample static PPP")
            self.assertEqual(payload["epochs"], 120)
            self.assertEqual(payload["fallback_epochs"], 0)
            self.assertLessEqual(payload["mean_position_error_m"], 1.5)
            self.assertLessEqual(payload["max_position_error_m"], 1.5)
            self.assertGreaterEqual(payload["mean_satellites"], 6.0)
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)

    def test_ppp_static_signoff_cli_supports_real_data_ar_signoff(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_static_signoff_ar_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            output_path = temp_root / "ppp_static_ar.pos"
            summary_path = temp_root / "ppp_static_ar_summary.json"

            result = self.run_gnss(
                "ppp-static-signoff",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "120",
                "--enable-ar",
                "--generate-products",
                "--ar-ratio-threshold",
                "1.5",
                "--require-valid-epochs-min",
                "120",
                "--require-mean-error-max",
                "5.0",
                "--require-max-error-max",
                "6.0",
                "--require-ppp-solution-rate-min",
                "100.0",
                "--require-ppp-fixed-epochs-min",
                "1",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Finished PPP static sign-off.", result.stdout)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())

            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "sample static PPP")
            self.assertEqual(payload["epochs"], 120)
            self.assertEqual(payload["fallback_epochs"], 0)
            self.assertLessEqual(payload["mean_position_error_m"], 5.0)
            self.assertLessEqual(payload["max_position_error_m"], 6.0)
            self.assertGreaterEqual(payload["ppp_fixed_epochs"], 1)
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)
            self.assertTrue(payload["ambiguity_resolution_enabled"])
            self.assertEqual(payload["ar_ratio_threshold"], 1.5)

    def test_ppp_static_signoff_cli_can_fetch_precise_products(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_static_signoff_fetch_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            generated_sp3 = temp_root / "generated.sp3"
            generated_clk = temp_root / "generated.clk"
            products_dir = temp_root / "products"
            products_dir.mkdir()
            output_path = temp_root / "ppp_static_fetch.pos"
            summary_path = temp_root / "ppp_static_fetch_summary.json"

            nav_products = self.run_gnss(
                "nav-products",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3-out",
                str(generated_sp3),
                "--clk-out",
                str(generated_clk),
                "--max-epochs",
                "30",
            )
            self.assertEqual(nav_products.returncode, 0, msg=nav_products.stderr)

            with gzip.open(products_dir / "2024002.sp3.gz", "wb") as stream:
                stream.write(generated_sp3.read_bytes())
            with gzip.open(products_dir / "2024002.clk.gz", "wb") as stream:
                stream.write(generated_clk.read_bytes())
            with gzip.open(products_dir / "2024002.ionex.gz", "wb") as stream:
                stream.write(build_synthetic_ionex_text().encode("ascii"))
            with gzip.open(products_dir / "2024002.bsx.gz", "wb") as stream:
                stream.write(build_synthetic_dcb_text().encode("ascii"))

            result = self.run_gnss(
                "ppp-static-signoff",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "30",
                "--fetch-products",
                "--product-date",
                "2024-01-02",
                "--product",
                f"sp3={products_dir / '{yyyy}{doy}.sp3.gz'}",
                "--product",
                f"clk={products_dir / '{yyyy}{doy}.clk.gz'}",
                "--product",
                f"ionex={products_dir / '{yyyy}{doy}.ionex.gz'}",
                "--product",
                f"dcb={products_dir / '{yyyy}{doy}.bsx.gz'}",
                "--require-valid-epochs-min",
                "30",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertTrue(payload["fetch_products"])
            self.assertEqual(payload["fetched_product_date"], "2024-01-02")
            self.assertEqual(set(payload["fetched_products"]), {"sp3", "clk", "ionex", "dcb"})
            self.assertTrue(str(payload["ionex"]).endswith("2024002.ionex"))
            self.assertTrue(str(payload["dcb"]).endswith("2024002.bsx"))
            self.assertGreaterEqual(payload["ionex_corrections"], 0)
            self.assertGreaterEqual(payload["dcb_corrections"], 0)
            self.assertIn("ppp_run_summary", payload)

    def test_ppp_kinematic_signoff_cli_writes_summary_and_passes_thresholds(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_kinematic_signoff_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            output_path = temp_root / "ppp_kinematic.pos"
            reference_path = temp_root / "ppp_kinematic_reference.pos"
            summary_path = temp_root / "ppp_kinematic_summary.json"

            result = self.run_gnss(
                "ppp-kinematic-signoff",
                "--out",
                str(output_path),
                "--reference-pos",
                str(reference_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "60",
                "--require-common-epoch-pairs-min",
                "60",
                "--require-reference-fix-rate-min",
                "90",
                "--require-converged",
                "--require-convergence-time-max",
                "300",
                "--require-mean-sats-min",
                "18",
                "--require-mean-error-max",
                "7",
                "--require-p95-error-max",
                "7",
                "--require-max-error-max",
                "7",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Finished PPP kinematic sign-off.", result.stdout)
            self.assertTrue(output_path.exists())
            self.assertTrue(reference_path.exists())
            self.assertTrue(summary_path.exists())

            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "sample kinematic PPP")
            self.assertEqual(payload["epochs"], 60)
            self.assertEqual(payload["common_epoch_pairs"], 60)
            self.assertTrue(payload["ppp_converged"])
            self.assertLessEqual(payload["ppp_convergence_time_s"], 300.0)
            self.assertEqual(payload["fallback_epochs"], 0)
            self.assertGreaterEqual(payload["reference_fix_rate_pct"], 90.0)
            self.assertLessEqual(payload["mean_position_error_m"], 7.0)
            self.assertLessEqual(payload["p95_position_error_m"], 7.0)
            self.assertLessEqual(payload["max_position_error_m"], 7.0)

    def test_ppp_kinematic_signoff_cli_can_fetch_precise_products(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_kinematic_signoff_fetch_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            generated_sp3 = temp_root / "generated.sp3"
            generated_clk = temp_root / "generated.clk"
            products_dir = temp_root / "products"
            products_dir.mkdir()
            output_path = temp_root / "ppp_kinematic_fetch.pos"
            reference_path = temp_root / "ppp_kinematic_fetch_reference.pos"
            summary_path = temp_root / "ppp_kinematic_fetch_summary.json"

            nav_products = self.run_gnss(
                "nav-products",
                "--obs",
                str(ROOT_DIR / "data/rover_kinematic.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_kinematic.nav"),
                "--sp3-out",
                str(generated_sp3),
                "--clk-out",
                str(generated_clk),
                "--max-epochs",
                "20",
            )
            self.assertEqual(nav_products.returncode, 0, msg=nav_products.stderr)

            with gzip.open(products_dir / "2024002.sp3.gz", "wb") as stream:
                stream.write(generated_sp3.read_bytes())
            with gzip.open(products_dir / "2024002.clk.gz", "wb") as stream:
                stream.write(generated_clk.read_bytes())
            with gzip.open(products_dir / "2024002.ionex.gz", "wb") as stream:
                stream.write(build_synthetic_ionex_text().encode("ascii"))
            with gzip.open(products_dir / "2024002.bsx.gz", "wb") as stream:
                stream.write(build_synthetic_dcb_text().encode("ascii"))

            result = self.run_gnss(
                "ppp-kinematic-signoff",
                "--obs",
                str(ROOT_DIR / "data/rover_kinematic.obs"),
                "--base",
                str(ROOT_DIR / "data/base_kinematic.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_kinematic.nav"),
                "--out",
                str(output_path),
                "--reference-pos",
                str(reference_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "20",
                "--fetch-products",
                "--product-date",
                "2024-01-02",
                "--product",
                f"sp3={products_dir / '{yyyy}{doy}.sp3.gz'}",
                "--product",
                f"clk={products_dir / '{yyyy}{doy}.clk.gz'}",
                "--product",
                f"ionex={products_dir / '{yyyy}{doy}.ionex.gz'}",
                "--product",
                f"dcb={products_dir / '{yyyy}{doy}.bsx.gz'}",
                "--require-common-epoch-pairs-min",
                "20",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertTrue(payload["fetch_products"])
            self.assertEqual(payload["fetched_product_date"], "2024-01-02")
            self.assertEqual(set(payload["fetched_products"]), {"sp3", "clk", "ionex", "dcb"})
            self.assertTrue(str(payload["ionex"]).endswith("2024002.ionex"))
            self.assertTrue(str(payload["dcb"]).endswith("2024002.bsx"))
            self.assertGreaterEqual(payload["ionex_corrections"], 0)
            self.assertGreaterEqual(payload["dcb_corrections"], 0)
            self.assertIn("ppp_run_summary", payload)
            self.assertGreaterEqual(payload["mean_satellites"], 18.0)
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)

    def test_ppp_products_signoff_cli_runs_static_profile_with_local_templates(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_products_signoff_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            generated_sp3 = temp_root / "generated.sp3"
            generated_clk = temp_root / "generated.clk"
            products_dir = temp_root / "products"
            products_dir.mkdir()
            output_path = temp_root / "ppp_products.pos"
            summary_path = temp_root / "ppp_products_summary.json"

            nav_products = self.run_gnss(
                "nav-products",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3-out",
                str(generated_sp3),
                "--clk-out",
                str(generated_clk),
                "--max-epochs",
                "20",
            )
            self.assertEqual(nav_products.returncode, 0, msg=nav_products.stderr)

            with gzip.open(products_dir / "2024002.sp3.gz", "wb") as stream:
                stream.write(generated_sp3.read_bytes())
            with gzip.open(products_dir / "2024002.clk.gz", "wb") as stream:
                stream.write(generated_clk.read_bytes())
            with gzip.open(products_dir / "2024002.ionex.gz", "wb") as stream:
                stream.write(build_synthetic_ionex_text().encode("ascii"))
            with gzip.open(products_dir / "2024002.bsx.gz", "wb") as stream:
                stream.write(build_synthetic_dcb_text().encode("ascii"))

            result = self.run_gnss(
                "ppp-products-signoff",
                "--profile",
                "static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "20",
                "--product-date",
                "2024-01-02",
                "--product",
                f"sp3={products_dir / '{yyyy}{doy}.sp3.gz'}",
                "--product",
                f"clk={products_dir / '{yyyy}{doy}.clk.gz'}",
                "--product",
                f"ionex={products_dir / '{yyyy}{doy}.ionex.gz'}",
                "--product",
                f"dcb={products_dir / '{yyyy}{doy}.bsx.gz'}",
                "--require-valid-epochs-min",
                "20",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["products_signoff_profile"], "static")
            self.assertEqual(payload["product_presets"], [])
            self.assertEqual(len(payload["product_specs"]), 4)
            self.assertTrue(payload["fetch_products"])
            self.assertEqual(payload["fetched_product_date"], "2024-01-02")

    def test_ppc_demo_cli_summarizes_existing_solution_against_reference_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_demo_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            run_dir = temp_root / "tokyo" / "run1"
            run_dir.mkdir(parents=True)
            solution_path = temp_root / "ppc_demo.pos"
            rtklib_path = temp_root / "ppc_demo_rtklib.pos"
            summary_path = temp_root / "ppc_demo_summary.json"
            reference_csv = run_dir / "reference.csv"

            reference_rows = [
                (2300, 1000.0, 35.1000000, 139.1000000, 42.0),
                (2300, 1000.2, 35.1000100, 139.1000200, 42.2),
                (2300, 1000.4, 35.1000200, 139.1000400, 42.4),
            ]
            reference_lines = [
                "gps_week,gps_tow_s,lat_deg,lon_deg,height_m,roll_deg,pitch_deg,yaw_deg"
            ]
            for week, tow, lat, lon, height in reference_rows:
                reference_lines.append(
                    f"{week},{tow:.3f},{lat:.7f},{lon:.7f},{height:.3f},0.0,0.0,0.0"
                )
            reference_csv.write_text("\n".join(reference_lines) + "\n", encoding="ascii")

            with solution_path.open("w", encoding="ascii") as handle:
                handle.write("% synthetic ppc demo solution\n")
                for week, tow, lat, lon, height, status, satellites in (
                    (2300, 1000.0, 35.1000002, 139.1000001, 42.1, 4, 12),
                    (2300, 1000.2, 35.1000099, 139.1000202, 42.3, 4, 13),
                    (2300, 1000.4, 35.1000197, 139.1000398, 42.5, 3, 11),
                ):
                    ecef = driving_comparison.llh_to_ecef(lat, lon, height)
                    handle.write(
                        f"{week} {tow:.3f} {ecef[0]:.6f} {ecef[1]:.6f} {ecef[2]:.6f} "
                        f"{lat:.9f} {lon:.9f} {height:.4f} {status} {satellites} 1.0\n"
                    )
            write_rtklib_pos(
                rtklib_path,
                [
                    (2300, 1000.0, 35.1000004, 139.1000003, 42.2, 1, 12),
                    (2300, 1000.2, 35.1000103, 139.1000205, 42.4, 1, 13),
                    (2300, 1000.4, 35.1000205, 139.1000404, 42.6, 2, 11),
                ],
            )

            result = self.run_gnss(
                "ppc-demo",
                "--run-dir",
                str(run_dir),
                "--solver",
                "rtk",
                "--use-existing-solution",
                "--solver-wall-time-s",
                "0.5",
                "--out",
                str(solution_path),
                "--rtklib-pos",
                str(rtklib_path),
                "--use-existing-rtklib-solution",
                "--rtklib-solver-wall-time-s",
                "0.1",
                "--summary-json",
                str(summary_path),
                "--require-valid-epochs-min",
                "3",
                "--require-matched-epochs-min",
                "3",
                "--require-fix-rate-min",
                "60",
                "--require-median-h-max",
                "0.2",
                "--require-p95-h-max",
                "0.2",
                "--require-max-h-max",
                "0.2",
                "--require-p95-up-max",
                "0.2",
                "--require-mean-sats-min",
                "11",
                "--require-solver-wall-time-max",
                "1.0",
                "--require-realtime-factor-min",
                "0.5",
                "--require-effective-epoch-rate-min",
                "5.0",
                "--require-lib-fix-rate-vs-rtklib-min-delta",
                "0.0",
                "--require-lib-median-h-vs-rtklib-max-delta",
                "0.0",
                "--require-lib-p95-h-vs-rtklib-max-delta",
                "0.0",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Finished PPC-Dataset demo.", result.stdout)
            self.assertTrue(summary_path.exists())

            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "PPC-Dataset tokyo run1")
            self.assertEqual(payload["solver"], "rtk")
            self.assertEqual(payload["valid_epochs"], 3)
            self.assertEqual(payload["matched_epochs"], 3)
            self.assertEqual(payload["fixed_epochs"], 2)
            self.assertGreaterEqual(payload["fix_rate_pct"], 60.0)
            self.assertLessEqual(payload["p95_h_m"], 0.2)
            self.assertLessEqual(payload["p95_abs_up_m"], 0.2)
            self.assertGreaterEqual(payload["mean_satellites"], 11.0)
            self.assertEqual(payload["solver_wall_time_s"], 0.5)
            self.assertEqual(payload["solution_span_s"], 0.4)
            self.assertEqual(payload["realtime_factor"], 0.8)
            self.assertEqual(payload["effective_epoch_rate_hz"], 6.0)
            self.assertIn("rtklib", payload)
            self.assertEqual(payload["rtklib"]["matched_epochs"], 3)
            self.assertEqual(payload["rtklib"]["solver_wall_time_s"], 0.1)
            self.assertAlmostEqual(payload["rtklib"]["realtime_factor"], 4.0, places=5)
            self.assertIn("delta_vs_rtklib", payload)
            self.assertIn("rtklib performance: wall=0.1 s", result.stdout)
            self.assertIn("performance: wall=0.5 s, span=0.4 s, rtf=0.8, rate=6.0 Hz", result.stdout)

    def test_ppc_rtk_signoff_cli_wraps_profile_with_existing_solutions(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_rtk_signoff_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            run_dir = temp_root / "tokyo" / "run1"
            run_dir.mkdir(parents=True)
            solution_path = temp_root / "ppc_signoff.pos"
            rtklib_path = temp_root / "ppc_signoff_rtklib.pos"
            summary_path = temp_root / "ppc_signoff_summary.json"
            reference_csv = run_dir / "reference.csv"

            reference_rows = [
                (2300, 1000.0, 35.1000000, 139.1000000, 42.0),
                (2300, 1000.2, 35.1000100, 139.1000200, 42.2),
                (2300, 1000.4, 35.1000200, 139.1000400, 42.4),
            ]
            reference_lines = [
                "gps_week,gps_tow_s,lat_deg,lon_deg,height_m,roll_deg,pitch_deg,yaw_deg"
            ]
            for week, tow, lat, lon, height in reference_rows:
                reference_lines.append(
                    f"{week},{tow:.3f},{lat:.7f},{lon:.7f},{height:.3f},0.0,0.0,0.0"
                )
            reference_csv.write_text("\n".join(reference_lines) + "\n", encoding="ascii")

            with solution_path.open("w", encoding="ascii") as handle:
                handle.write("% synthetic ppc signoff solution\n")
                for week, tow, lat, lon, height, status, satellites in (
                    (2300, 1000.0, 35.1000002, 139.1000001, 42.1, 4, 12),
                    (2300, 1000.2, 35.1000099, 139.1000202, 42.3, 4, 13),
                    (2300, 1000.4, 35.1000197, 139.1000398, 42.5, 3, 11),
                ):
                    ecef = driving_comparison.llh_to_ecef(lat, lon, height)
                    handle.write(
                        f"{week} {tow:.3f} {ecef[0]:.6f} {ecef[1]:.6f} {ecef[2]:.6f} "
                        f"{lat:.9f} {lon:.9f} {height:.4f} {status} {satellites} 1.0\n"
                    )
            write_rtklib_pos(
                rtklib_path,
                [
                    (2300, 1000.0, 35.1000004, 139.1000003, 42.2, 1, 12),
                    (2300, 1000.2, 35.1000103, 139.1000205, 42.4, 1, 13),
                    (2300, 1000.4, 35.1000205, 139.1000404, 42.6, 2, 11),
                ],
            )

            result = self.run_gnss(
                "ppc-rtk-signoff",
                "--run-dir",
                str(run_dir),
                "--city",
                "tokyo",
                "--use-existing-solution",
                "--solver-wall-time-s",
                "0.5",
                "--out",
                str(solution_path),
                "--rtklib-pos",
                str(rtklib_path),
                "--use-existing-rtklib-solution",
                "--rtklib-solver-wall-time-s",
                "0.1",
                "--summary-json",
                str(summary_path),
                "--require-valid-epochs-min",
                "3",
                "--require-matched-epochs-min",
                "3",
                "--require-fix-rate-min",
                "60",
                "--require-median-h-max",
                "0.2",
                "--require-p95-h-max",
                "0.2",
                "--require-max-h-max",
                "0.2",
                "--require-p95-up-max",
                "0.2",
                "--require-mean-sats-min",
                "11",
                "--require-solver-wall-time-max",
                "1.0",
                "--require-realtime-factor-min",
                "0.5",
                "--require-effective-epoch-rate-min",
                "5.0",
                "--require-lib-fix-rate-vs-rtklib-min-delta",
                "0.0",
                "--require-lib-median-h-vs-rtklib-max-delta",
                "0.0",
                "--require-lib-p95-h-vs-rtklib-max-delta",
                "0.0",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Finished PPC RTK sign-off.", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["signoff_profile"], "ppc-rtk-tokyo")
            self.assertTrue(payload["rtklib_comparison_enabled"])
            self.assertIn("signoff_thresholds", payload)
            self.assertEqual(payload["signoff_thresholds"]["require_fix_rate_min"], 60.0)
            self.assertIn("delta_vs_rtklib", payload)

    def test_ppc_demo_cli_tokyo_rtk_realdataset_signoff_if_present(self) -> None:
        dataset_root = ppc_dataset_root()
        run_dir = dataset_root / "tokyo" / "run1"
        required_files = (
            run_dir / "rover.obs",
            run_dir / "base.obs",
            run_dir / "base.nav",
            run_dir / "reference.csv",
        )
        if not all(path.exists() for path in required_files):
            self.skipTest("PPC-Dataset tokyo/run1 is not available")

        with tempfile.TemporaryDirectory(prefix="gnss_ppc_tokyo_rtk_") as temp_dir:
            temp_root = Path(temp_dir)
            solution_path = temp_root / "tokyo_run1_rtk.pos"
            summary_path = temp_root / "tokyo_run1_rtk_summary.json"

            result = self.run_gnss(
                "ppc-demo",
                "--run-dir",
                str(run_dir),
                "--solver",
                "rtk",
                "--max-epochs",
                "120",
                "--out",
                str(solution_path),
                "--summary-json",
                str(summary_path),
                "--require-valid-epochs-min",
                "100",
                "--require-matched-epochs-min",
                "100",
                "--require-fix-rate-min",
                "90",
                "--require-median-h-max",
                "0.10",
                "--require-p95-h-max",
                "0.20",
                "--require-max-h-max",
                "0.50",
                "--require-p95-up-max",
                "0.60",
                "--require-mean-sats-min",
                "18",
                "--require-solver-wall-time-max",
                "10.0",
                "--require-realtime-factor-min",
                "1.0",
                "--require-effective-epoch-rate-min",
                "5.0",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Finished PPC-Dataset demo.", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "PPC-Dataset tokyo run1")
            self.assertEqual(payload["solver"], "rtk")
            self.assertGreaterEqual(payload["valid_epochs"], 100)
            self.assertGreaterEqual(payload["matched_epochs"], 100)
            self.assertGreaterEqual(payload["fix_rate_pct"], 90.0)
            self.assertLessEqual(payload["median_h_m"], 0.10)
            self.assertLessEqual(payload["p95_h_m"], 0.20)
            self.assertLessEqual(payload["max_h_m"], 0.50)
            self.assertLessEqual(payload["p95_abs_up_m"], 0.60)
            self.assertGreaterEqual(payload["mean_satellites"], 18.0)
            self.assertLessEqual(payload["solver_wall_time_s"], 10.0)
            self.assertGreaterEqual(payload["realtime_factor"], 1.0)
            self.assertGreaterEqual(payload["effective_epoch_rate_hz"], 5.0)

    def test_ppc_demo_cli_tokyo_rtk_realdataset_tracks_rtklib_if_present(self) -> None:
        dataset_root = ppc_dataset_root()
        run_dir = dataset_root / "tokyo" / "run1"
        rtklib_bin = rtklib_binary()
        required_files = (
            run_dir / "rover.obs",
            run_dir / "base.obs",
            run_dir / "base.nav",
            run_dir / "reference.csv",
        )
        if not all(path.exists() for path in required_files):
            self.skipTest("PPC-Dataset tokyo/run1 is not available")
        if not rtklib_bin.exists():
            self.skipTest("RTKLIB rnx2rtkp is not available")

        with tempfile.TemporaryDirectory(prefix="gnss_ppc_tokyo_rtk_rtklib_") as temp_dir:
            temp_root = Path(temp_dir)
            solution_path = temp_root / "tokyo_run1_rtk.pos"
            summary_path = temp_root / "tokyo_run1_rtk_summary.json"

            result = self.run_gnss(
                "ppc-demo",
                "--run-dir",
                str(run_dir),
                "--solver",
                "rtk",
                "--max-epochs",
                "120",
                "--out",
                str(solution_path),
                "--summary-json",
                str(summary_path),
                "--rtklib-bin",
                str(rtklib_bin),
                "--require-valid-epochs-min",
                "100",
                "--require-matched-epochs-min",
                "100",
                "--require-realtime-factor-min",
                "1.0",
                "--require-lib-fix-rate-vs-rtklib-min-delta",
                "0.0",
                "--require-lib-median-h-vs-rtklib-max-delta",
                "0.01",
                "--require-lib-p95-h-vs-rtklib-max-delta",
                "0.02",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertIn("rtklib", payload)
            self.assertIn("delta_vs_rtklib", payload)
            self.assertGreaterEqual(payload["realtime_factor"], 1.0)
            self.assertGreaterEqual(
                payload["delta_vs_rtklib"]["fix_rate_pct"],
                0.0,
            )
            self.assertLessEqual(
                payload["delta_vs_rtklib"]["median_h_m"],
                0.01,
            )
            self.assertLessEqual(
                payload["delta_vs_rtklib"]["p95_h_m"],
                0.02,
            )

    def test_ppc_rtk_signoff_cli_tokyo_realdataset_if_present(self) -> None:
        dataset_root = ppc_dataset_root()
        run_dir = dataset_root / "tokyo" / "run1"
        rtklib_bin = rtklib_binary()
        required_files = (
            run_dir / "rover.obs",
            run_dir / "base.obs",
            run_dir / "base.nav",
            run_dir / "reference.csv",
        )
        if not all(path.exists() for path in required_files):
            self.skipTest("PPC-Dataset tokyo/run1 is not available")
        if not rtklib_bin.exists():
            self.skipTest("RTKLIB rnx2rtkp is not available")

        with tempfile.TemporaryDirectory(prefix="gnss_ppc_tokyo_signoff_") as temp_dir:
            temp_root = Path(temp_dir)
            summary_path = temp_root / "tokyo_run1_signoff_summary.json"
            result = self.run_gnss(
                "ppc-rtk-signoff",
                "--dataset-root",
                str(dataset_root),
                "--city",
                "tokyo",
                "--max-epochs",
                "120",
                "--summary-json",
                str(summary_path),
                "--rtklib-bin",
                str(rtklib_bin),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["signoff_profile"], "ppc-rtk-tokyo")
            self.assertTrue(payload["rtklib_comparison_enabled"])
            self.assertEqual(payload["tuning_profile"]["preset"], "low-cost")
            self.assertTrue(payload["tuning_profile"]["arfilter"])
            self.assertGreaterEqual(payload["fix_rate_pct"], 95.0)
            self.assertGreaterEqual(payload["delta_vs_rtklib"]["fix_rate_pct"], 0.0)
            self.assertLessEqual(payload["delta_vs_rtklib"]["median_h_m"], 0.01)
            self.assertLessEqual(payload["delta_vs_rtklib"]["p95_h_m"], 0.02)

    def test_ppc_rtk_signoff_cli_nagoya_realdataset_if_present(self) -> None:
        dataset_root = ppc_dataset_root()
        run_dir = dataset_root / "nagoya" / "run1"
        required_files = (
            run_dir / "rover.obs",
            run_dir / "base.obs",
            run_dir / "base.nav",
            run_dir / "reference.csv",
        )
        if not all(path.exists() for path in required_files):
            self.skipTest("PPC-Dataset nagoya/run1 is not available")

        with tempfile.TemporaryDirectory(prefix="gnss_ppc_nagoya_signoff_") as temp_dir:
            temp_root = Path(temp_dir)
            summary_path = temp_root / "nagoya_run1_signoff_summary.json"
            result = self.run_gnss(
                "ppc-rtk-signoff",
                "--dataset-root",
                str(dataset_root),
                "--city",
                "nagoya",
                "--max-epochs",
                "120",
                "--summary-json",
                str(summary_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["signoff_profile"], "ppc-rtk-nagoya")
            self.assertFalse(payload["rtklib_comparison_enabled"])
            self.assertEqual(payload["tuning_profile"]["preset"], "low-cost")
            self.assertFalse(payload["tuning_profile"]["arfilter"])
            self.assertGreaterEqual(payload["fix_rate_pct"], 95.0)
            self.assertLessEqual(payload["median_h_m"], 0.12)
            self.assertLessEqual(payload["p95_h_m"], 0.12)

    def test_nav_products_cli_generates_sp3_and_clk_from_static_sample(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_nav_products_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            sp3_path = temp_root / "static_products.sp3"
            clk_path = temp_root / "static_products.clk"

            result = self.run_gnss(
                "nav-products",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3-out",
                str(sp3_path),
                "--clk-out",
                str(clk_path),
                "--max-epochs",
                "4",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("epochs written: 4", result.stdout)
            self.assertTrue(sp3_path.exists())
            self.assertTrue(clk_path.exists())

            sp3_text = sp3_path.read_text(encoding="ascii")
            clk_text = clk_path.read_text(encoding="ascii")
            self.assertIn("*  2005 04 02 00 00 00.00000000", sp3_text)
            self.assertIn("PG03", sp3_text)
            self.assertIn("PG07", sp3_text)
            self.assertIn("AS G03 2005 04 02 00 00 00.00000000", clk_text)
            self.assertIn("AS G07 2005 04 02 00 00 00.00000000", clk_text)

    def test_ppp_cli_processes_real_static_sample_with_generated_products(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_real_static_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            sp3_path = temp_root / "static_products.sp3"
            clk_path = temp_root / "static_products.clk"
            output_path = temp_root / "ppp_real_static.pos"

            nav_products = self.run_gnss(
                "nav-products",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3-out",
                str(sp3_path),
                "--clk-out",
                str(clk_path),
                "--max-epochs",
                "120",
            )
            self.assertEqual(nav_products.returncode, 0, msg=nav_products.stderr)

            result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--out",
                str(output_path),
                "--max-epochs",
                "120",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertIn("valid solutions: 120", result.stdout)
            self.assertIn("PPP float solutions: 120", result.stdout)
            self.assertIn("PPP fixed solutions: 0", result.stdout)
            self.assertIn("fallback solutions: 0", result.stdout)
            self.assertIn("PPP solution rate (%): 100", result.stdout)

    def test_ppp_cli_runs_real_static_slice_with_generated_products_and_ar_enabled(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_real_static_ar_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            sp3_path = temp_root / "static_products.sp3"
            clk_path = temp_root / "static_products.clk"
            output_path = temp_root / "ppp_real_static_ar.pos"

            nav_products = self.run_gnss(
                "nav-products",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3-out",
                str(sp3_path),
                "--clk-out",
                str(clk_path),
                "--max-epochs",
                "20",
            )
            self.assertEqual(nav_products.returncode, 0, msg=nav_products.stderr)

            result = self.run_gnss(
                "ppp",
                "--static",
                "--enable-ar",
                "--convergence-min-epochs",
                "4",
                "--ar-ratio-threshold",
                "1.5",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--out",
                str(output_path),
                "--max-epochs",
                "20",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertIn("valid solutions: 20", result.stdout)
            self.assertIn("ambiguity resolution: on", result.stdout)
            self.assertIn("PPP fixed solutions:", result.stdout)
            self.assertIn("PPP solution rate (%):", result.stdout)

    def test_ppp_cli_accepts_ssr_corrections_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_ssr_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            ssr_path = temp_root / "corrections.csv"
            output_path = temp_root / "ppp_ssr.pos"
            ssr_path.write_text(
                "\n".join(
                    [
                        "# week,tow,sat,dx,dy,dz,dclock_m",
                        "1316,518400.0,G03,0.0,0.0,0.0,0.0",
                        "1316,518430.0,G03,0.0,0.0,0.0,0.0",
                        "1316,518460.0,G03,0.0,0.0,0.0,0.0",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--ssr",
                str(ssr_path),
                "--out",
                str(output_path),
                "--max-epochs",
                "3",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertIn("SSR corrections: on", result.stdout)
            self.assertIn("valid solutions: 3", result.stdout)

    def test_ppp_cli_reports_applied_atmospheric_corrections_from_sampled_ssr(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_ssr_atmos_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, ssr_path, true_position = build_synthetic_ppp_inputs_with_atmos(
                temp_root
            )
            baseline_path = temp_root / "ppp_ssr_atmos_baseline.pos"
            output_path = temp_root / "ppp_ssr_atmos.pos"

            baseline_result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--no-estimate-troposphere",
                "--out",
                str(baseline_path),
                "--max-epochs",
                "4",
            )

            result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--ssr",
                str(ssr_path),
                "--no-estimate-troposphere",
                "--out",
                str(output_path),
                "--max-epochs",
                "4",
            )

            self.assertEqual(baseline_result.returncode, 0, msg=baseline_result.stderr)
            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(baseline_path.exists())
            self.assertTrue(output_path.exists())
            self.assertIn("SSR corrections: on", result.stdout)
            self.assertIn("atmospheric trop corrections:", result.stdout)
            self.assertIn("atmospheric ionosphere corrections:", result.stdout)
            trop_line = next(
                line for line in result.stdout.splitlines() if "atmospheric trop corrections:" in line
            )
            iono_line = next(
                line
                for line in result.stdout.splitlines()
                if "atmospheric ionosphere corrections:" in line
            )
            self.assertGreater(int(trop_line.rsplit(":", 1)[1].strip()), 0)
            self.assertGreater(int(iono_line.rsplit(":", 1)[1].strip()), 0)

            baseline_records = self.read_pos_records(baseline_path)
            records = self.read_pos_records(output_path)
            self.assertEqual(len(baseline_records), 4)
            self.assertEqual(len(records), 4)
            baseline_last = baseline_records[-1]
            last_record = records[-1]
            baseline_error = math.sqrt(
                (baseline_last["x"] - true_position[0]) ** 2
                + (baseline_last["y"] - true_position[1]) ** 2
                + (baseline_last["z"] - true_position[2]) ** 2
            )
            error = math.sqrt(
                (last_record["x"] - true_position[0]) ** 2
                + (last_record["y"] - true_position[1]) ** 2
                + (last_record["z"] - true_position[2]) ** 2
            )
            self.assertLess(error, baseline_error)

    def test_ppp_cli_applies_grid_polynomial_ionosphere_from_sampled_ssr(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_ssr_grid_poly_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, ssr_path, true_position = (
                build_synthetic_ppp_inputs_with_grid_polynomial_atmos(temp_root)
            )
            baseline_path = temp_root / "ppp_ssr_grid_poly_baseline.pos"
            output_path = temp_root / "ppp_ssr_grid_poly.pos"

            baseline_result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--no-estimate-troposphere",
                "--out",
                str(baseline_path),
                "--max-epochs",
                "4",
            )

            result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--ssr",
                str(ssr_path),
                "--no-estimate-troposphere",
                "--out",
                str(output_path),
                "--max-epochs",
                "4",
            )

            self.assertEqual(baseline_result.returncode, 0, msg=baseline_result.stderr)
            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(baseline_path.exists())
            self.assertTrue(output_path.exists())
            self.assertIn("SSR corrections: on", result.stdout)
            self.assertIn("atmospheric ionosphere corrections:", result.stdout)

            iono_count_line = next(
                line
                for line in result.stdout.splitlines()
                if "atmospheric ionosphere corrections:" in line
            )
            self.assertGreater(int(iono_count_line.rsplit(":", 1)[1].strip()), 0)

            baseline_records = self.read_pos_records(baseline_path)
            records = self.read_pos_records(output_path)
            self.assertEqual(len(baseline_records), 4)
            self.assertEqual(len(records), 4)
            baseline_last = baseline_records[-1]
            last_record = records[-1]
            baseline_error = math.sqrt(
                (baseline_last["x"] - true_position[0]) ** 2
                + (baseline_last["y"] - true_position[1]) ** 2
                + (baseline_last["z"] - true_position[2]) ** 2
            )
            error = math.sqrt(
                (last_record["x"] - true_position[0]) ** 2
                + (last_record["y"] - true_position[1]) ** 2
                + (last_record["z"] - true_position[2]) ** 2
            )
            self.assertLess(error, baseline_error)

    def test_ppp_cli_accepts_rtcm_ssr_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_ssr_rtcm_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            ssr_path = temp_root / "corrections.rtcm3"
            output_path = temp_root / "ppp_ssr_rtcm.pos"
            ssr_path.write_bytes(
                build_rtcm1060(3, 518400) +
                build_rtcm1059(3, 518400) +
                build_rtcm1062(3, 518400)
            )

            result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--ssr-rtcm",
                str(ssr_path),
                "--out",
                str(output_path),
                "--max-epochs",
                "3",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertIn("SSR corrections: on", result.stdout)
            self.assertIn("valid solutions: 3", result.stdout)

    def test_ppp_cli_detects_cycle_slip_from_geometry_free(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_cycle_slip_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, _ = build_synthetic_ppp_inputs_with_cycle_slip(temp_root)
            output_path = temp_root / "ppp_cycle_slip.pos"

            result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--no-estimate-troposphere",
                "--out",
                str(output_path),
                "--max-epochs",
                "4",
                extra_env={"GNSS_PPP_DEBUG": "1"},
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertIn("valid solutions: 4", result.stdout)
            self.assertIn("cycle slip reset G01", result.stderr)
            self.assertIn("reason=geometry-free+melbourne-wubbena", result.stderr)

    def test_ppp_cli_accepts_ntrip_rtcm_ssr_corrections(self) -> None:
        payload = build_rtcm1060(3, 518400) + build_rtcm1059(3, 518400) + build_rtcm1062(3, 518400)

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.bind(("127.0.0.1", 0))
            server.listen(1)
            server.settimeout(2.0)
            port = server.getsockname()[1]

            def caster() -> None:
                conn, _ = server.accept()
                with conn:
                    conn.recv(1024)
                    conn.sendall(b"ICY 200 OK\r\nNtrip-Version: Ntrip/2.0\r\n\r\n")
                    conn.sendall(payload)

            thread = threading.Thread(target=caster)
            thread.start()
            try:
                with tempfile.TemporaryDirectory(prefix="gnss_ppp_ssr_ntrip_cli_") as temp_dir:
                    temp_root = Path(temp_dir)
                    output_path = temp_root / "ppp_ssr_ntrip.pos"

                    result = self.run_gnss(
                        "ppp",
                        "--static",
                        "--obs",
                        str(ROOT_DIR / "data/rover_static.obs"),
                        "--nav",
                        str(ROOT_DIR / "data/navigation_static.nav"),
                        "--ssr-rtcm",
                        f"ntrip://127.0.0.1:{port}/MOUNT1",
                        "--out",
                        str(output_path),
                        "--max-epochs",
                        "3",
                    )
                    self.assertEqual(result.returncode, 0, msg=result.stderr)
                    self.assertTrue(output_path.exists())
                    self.assertIn("SSR corrections: on", result.stdout)
                    self.assertIn("valid solutions: 3", result.stdout)
            finally:
                thread.join()

    def test_clas_ppp_cli_writes_summary_for_named_profile(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            ssr_path = temp_root / "corrections.rtcm3"
            output_path = temp_root / "clas_ppp.pos"
            summary_path = temp_root / "clas_ppp_summary.json"
            ssr_path.write_bytes(
                build_rtcm1060(3, 518400) +
                build_rtcm1059(3, 518400) +
                build_rtcm1062(3, 518400)
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "madoca",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--ssr-rtcm",
                str(ssr_path),
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())
            self.assertIn("Finished CLAS/MADOCA PPP run.", result.stdout)
            self.assertIn("profile: madoca", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "CLAS/MADOCA PPP")
            self.assertEqual(payload["correction_profile"], "madoca")
            self.assertEqual(payload["ssr_transport"], "file")
            self.assertEqual(payload["epochs"], 3)
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)

    def test_clas_ppp_cli_accepts_compact_sampled_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_compact_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            compact_path = temp_root / "corrections.compact.csv"
            output_path = temp_root / "clas_ppp_compact.pos"
            summary_path = temp_root / "clas_ppp_compact_summary.json"
            compact_path.write_text(
                "\n".join(
                    [
                        "# week,tow,system,prn,dx,dy,dz,dclock_m,high_rate_clock_m",
                        "1316,518400.0,G,3,0.0,0.0,0.0,0.0,0.025",
                        "1316,518430.0,G,3,0.0,0.0,0.0,0.0,0.025",
                        "1316,518460.0,G,3,0.0,0.0,0.0,0.0,0.025",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--compact-ssr",
                str(compact_path),
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())
            self.assertIn("Finished CLAS/MADOCA PPP run.", result.stdout)
            self.assertIn("encoding: compact", result.stdout)
            self.assertIn("expanded compact corrections:", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "CLAS/MADOCA PPP")
            self.assertEqual(payload["correction_profile"], "clas")
            self.assertEqual(payload["ssr_transport"], "file")
            self.assertEqual(payload["correction_encoding"], "compact")
            self.assertEqual(payload["epochs"], 3)
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)

    def test_clas_ppp_cli_accepts_direct_qzss_l6_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            l6_path = temp_root / "corrections_l6.bin"
            output_path = temp_root / "clas_ppp_l6.pos"
            summary_path = temp_root / "clas_ppp_l6_summary.json"
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_combined_message(tow_delta=0, iod=3, prn=3, dclock_m=0.025),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518430, iod=3, prn=3, sync=True),
                        build_qzss_cssr_combined_message(tow_delta=30, iod=3, prn=3, dclock_m=0.025),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518460, iod=3, prn=3, sync=True),
                        build_qzss_cssr_combined_message(tow_delta=60, iod=3, prn=3, dclock_m=0.025),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "1316",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())
            self.assertIn("Finished CLAS/MADOCA PPP run.", result.stdout)
            self.assertIn("encoding: qzss_l6", result.stdout)
            self.assertIn("decoded qzss l6 corrections:", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "CLAS/MADOCA PPP")
            self.assertEqual(payload["correction_profile"], "clas")
            self.assertEqual(payload["ssr_transport"], "file")
            self.assertEqual(payload["correction_encoding"], "qzss_l6")
            self.assertEqual(payload["epochs"], 3)
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)

    def test_clas_ppp_cli_accepts_direct_qzss_l6_orbit_clock_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_oc_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            l6_path = temp_root / "corrections_l6_oc.bin"
            output_path = temp_root / "clas_ppp_l6_oc.pos"
            summary_path = temp_root / "clas_ppp_l6_oc_summary.json"
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_orbit_message(tow_delta=0, iod=3, dx=0.0, dy=0.0, dz=0.0, sync=True),
                        build_qzss_cssr_clock_message(tow_delta=0, iod=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518430, iod=3, prn=3, sync=True),
                        build_qzss_cssr_orbit_message(tow_delta=30, iod=3, dx=0.0, dy=0.0, dz=0.0, sync=True),
                        build_qzss_cssr_clock_message(tow_delta=30, iod=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518460, iod=3, prn=3, sync=True),
                        build_qzss_cssr_orbit_message(tow_delta=60, iod=3, dx=0.0, dy=0.0, dz=0.0, sync=True),
                        build_qzss_cssr_clock_message(tow_delta=60, iod=3, dclock_m=0.025, sync=False),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "1316",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())
            self.assertIn("decoded qzss l6 corrections:", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["correction_encoding"], "qzss_l6")
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)

    def test_clas_ppp_cli_accepts_direct_qzss_l6_code_bias_and_ura(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_cbias_ura_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            l6_path = temp_root / "corrections_l6_cbias_ura.bin"
            output_path = temp_root / "clas_ppp_l6_cbias_ura.pos"
            summary_path = temp_root / "clas_ppp_l6_cbias_ura_summary.json"
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(tow_delta=0, iod=3, bias_m=-0.12, sync=True),
                        build_qzss_cssr_ura_message(tow_delta=0, iod=3, ura_index=9, sync=True),
                        build_qzss_cssr_combined_message(tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518430, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(tow_delta=30, iod=3, bias_m=-0.12, sync=True),
                        build_qzss_cssr_ura_message(tow_delta=30, iod=3, ura_index=9, sync=True),
                        build_qzss_cssr_combined_message(tow_delta=30, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518460, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(tow_delta=60, iod=3, bias_m=-0.12, sync=True),
                        build_qzss_cssr_ura_message(tow_delta=60, iod=3, ura_index=9, sync=True),
                        build_qzss_cssr_combined_message(tow_delta=60, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "1316",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())
            self.assertIn("decoded qzss l6 corrections:", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["correction_encoding"], "qzss_l6")
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)

    def test_clas_ppp_cli_accepts_direct_qzss_l6_with_atmos_inventory(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_atmos_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            l6_path = temp_root / "corrections_l6_atmos.bin"
            output_path = temp_root / "clas_ppp_l6_atmos.pos"
            summary_path = temp_root / "clas_ppp_l6_atmos_summary.json"
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_atmos_message(tow_delta=0, iod=3, sync=True),
                        build_qzss_cssr_combined_message(tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518430, iod=3, prn=3, sync=True),
                        build_qzss_cssr_atmos_message(tow_delta=30, iod=3, sync=True),
                        build_qzss_cssr_combined_message(tow_delta=30, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518460, iod=3, prn=3, sync=True),
                        build_qzss_cssr_atmos_message(tow_delta=60, iod=3, sync=True),
                        build_qzss_cssr_combined_message(tow_delta=60, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "1316",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())
            self.assertIn("decoded qzss l6 corrections:", result.stdout)
            self.assertIn("atmos_messages=3", result.stdout)
            self.assertIn("atmos_rows=3", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["correction_encoding"], "qzss_l6")
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)
            self.assertEqual(payload["atmos_messages"], 3)
            self.assertEqual(payload["atmos_rows"], 3)

    def test_clas_ppp_cli_accepts_direct_qzss_l6_with_stec_inventory(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_stec_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            l6_path = temp_root / "corrections_l6_stec.bin"
            output_path = temp_root / "clas_ppp_l6_stec.pos"
            summary_path = temp_root / "clas_ppp_l6_stec_summary.json"
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_stec_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_id=7,
                            stec_type=3,
                            selected_satellites=1,
                            stec_quality=17,
                            stec_c00_tecu=1.5,
                            stec_c01_tecu_per_deg=0.12,
                            stec_c10_tecu_per_deg=-0.10,
                            stec_c11_tecu_per_deg2=0.06,
                            stec_c02_tecu_per_deg2=0.025,
                            stec_c20_tecu_per_deg2=-0.015,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False
                        ),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518430, iod=3, prn=3, sync=True),
                        build_qzss_cssr_stec_message(
                            tow_delta=30,
                            iod=3,
                            sync=True,
                            network_id=7,
                            stec_type=3,
                            selected_satellites=1,
                            stec_quality=17,
                            stec_c00_tecu=1.5,
                            stec_c01_tecu_per_deg=0.12,
                            stec_c10_tecu_per_deg=-0.10,
                            stec_c11_tecu_per_deg2=0.06,
                            stec_c02_tecu_per_deg2=0.025,
                            stec_c20_tecu_per_deg2=-0.015,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=30, iod=3, prn=3, dclock_m=0.025, sync=False
                        ),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518460, iod=3, prn=3, sync=True),
                        build_qzss_cssr_stec_message(
                            tow_delta=60,
                            iod=3,
                            sync=True,
                            network_id=7,
                            stec_type=3,
                            selected_satellites=1,
                            stec_quality=17,
                            stec_c00_tecu=1.5,
                            stec_c01_tecu_per_deg=0.12,
                            stec_c10_tecu_per_deg=-0.10,
                            stec_c11_tecu_per_deg2=0.06,
                            stec_c02_tecu_per_deg2=0.025,
                            stec_c20_tecu_per_deg2=-0.015,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=60, iod=3, prn=3, dclock_m=0.025, sync=False
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "1316",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
                "--require-atmos-messages-min",
                "3",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())
            self.assertIn("decoded qzss l6 corrections:", result.stdout)
            self.assertIn("atmos_messages=3", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["correction_encoding"], "qzss_l6")
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)
            self.assertEqual(payload["atmos_messages"], 3)

    def test_clas_ppp_cli_reports_applied_atmospheric_corrections_for_direct_qzss_l6(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_atmos_apply_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, _, _ = build_synthetic_ppp_inputs_with_atmos(temp_root)
            l6_path = temp_root / "corrections_l6_atmos_apply.bin"
            output_path = temp_root / "clas_ppp_l6_atmos_apply.pos"
            summary_path = temp_root / "clas_ppp_l6_atmos_apply_summary.json"
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=356400, iod=3, prn=1, sync=True),
                        build_qzss_cssr_atmos_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            selected_satellites=1,
                            trop_avail=3,
                            stec_avail=3,
                            trop_quality=9,
                            trop_type=0,
                            trop_t00_m=0.8,
                            stec_quality=17,
                            stec_type=0,
                            stec_c00_tecu=12.5,
                        ),
                        build_qzss_cssr_combined_message(tow_delta=0, iod=3, prn=1, dclock_m=0.0, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=356430, iod=3, prn=1, sync=True),
                        build_qzss_cssr_atmos_message(
                            tow_delta=30,
                            iod=3,
                            sync=True,
                            selected_satellites=1,
                            trop_avail=3,
                            stec_avail=3,
                            trop_quality=9,
                            trop_type=0,
                            trop_t00_m=0.8,
                            stec_quality=17,
                            stec_type=0,
                            stec_c00_tecu=12.5,
                        ),
                        build_qzss_cssr_combined_message(tow_delta=30, iod=3, prn=1, dclock_m=0.0, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=356460, iod=3, prn=1, sync=True),
                        build_qzss_cssr_atmos_message(
                            tow_delta=60,
                            iod=3,
                            sync=True,
                            selected_satellites=1,
                            trop_avail=3,
                            stec_avail=3,
                            trop_quality=9,
                            trop_type=0,
                            trop_t00_m=0.8,
                            stec_quality=17,
                            stec_type=0,
                            stec_c00_tecu=12.5,
                        ),
                        build_qzss_cssr_combined_message(tow_delta=60, iod=3, prn=1, dclock_m=0.0, sync=False),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "2411",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--no-estimate-troposphere",
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
                "--require-atmos-messages-min",
                "3",
                "--require-ppp-atmos-trop-corrections-min",
                "1",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["correction_encoding"], "qzss_l6")
            self.assertEqual(payload["atmos_messages"], 3)
            self.assertGreaterEqual(payload["ppp_atmospheric_trop_corrections"], 1)
            self.assertGreaterEqual(payload["ppp_atmospheric_ionosphere_corrections"], 1)
            self.assertGreater(payload["ppp_atmospheric_trop_meters"], 0.0)

    def test_clas_ppp_cli_uses_nearest_clas_grid_residuals_for_direct_qzss_l6(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_grid_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, _, _ = build_synthetic_ppp_inputs_with_atmos(temp_root)
            l6_path = temp_root / "corrections_l6_grid.bin"
            output_path = temp_root / "clas_ppp_l6_grid.pos"
            summary_path = temp_root / "clas_ppp_l6_grid_summary.json"
            # Synthetic PPP input is centered near Tokyo (35.0N, 139.0E), so CLAS network 7
            # grid 11 (34.77N, 139.37E) is the nearest reference grid in the official grid table.
            network_id = 7
            grid_count = 22
            nearest_grid_index = 10  # network 7 grid no. 11 -> zero-based index
            trop_residuals = build_single_grid_residuals(grid_count, nearest_grid_index, 0.5)
            stec_residuals = build_single_grid_residuals(grid_count, nearest_grid_index, 12.48)
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=356400, iod=3, prn=1, sync=True),
                        build_qzss_cssr_atmos_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_id=network_id,
                            grid_count=grid_count,
                            selected_satellites=1,
                            trop_avail=2,
                            stec_avail=2,
                            trop_residual_size=1,
                            trop_offset_m=0.3,
                            trop_residuals_m=trop_residuals,
                            stec_residual_size=3,
                            stec_residuals_tecu=stec_residuals,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0,
                            iod=3,
                            prn=1,
                            network_id=network_id,
                            dclock_m=0.0,
                            sync=False,
                        ),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=356430, iod=3, prn=1, sync=True),
                        build_qzss_cssr_atmos_message(
                            tow_delta=30,
                            iod=3,
                            sync=True,
                            network_id=network_id,
                            grid_count=grid_count,
                            selected_satellites=1,
                            trop_avail=2,
                            stec_avail=2,
                            trop_residual_size=1,
                            trop_offset_m=0.3,
                            trop_residuals_m=trop_residuals,
                            stec_residual_size=3,
                            stec_residuals_tecu=stec_residuals,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=30,
                            iod=3,
                            prn=1,
                            network_id=network_id,
                            dclock_m=0.0,
                            sync=False,
                        ),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=356460, iod=3, prn=1, sync=True),
                        build_qzss_cssr_atmos_message(
                            tow_delta=60,
                            iod=3,
                            sync=True,
                            network_id=network_id,
                            grid_count=grid_count,
                            selected_satellites=1,
                            trop_avail=2,
                            stec_avail=2,
                            trop_residual_size=1,
                            trop_offset_m=0.3,
                            trop_residuals_m=trop_residuals,
                            stec_residual_size=3,
                            stec_residuals_tecu=stec_residuals,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=60,
                            iod=3,
                            prn=1,
                            network_id=network_id,
                            dclock_m=0.0,
                            sync=False,
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "2411",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--no-estimate-troposphere",
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
                "--require-atmos-messages-min",
                "3",
                "--require-ppp-atmos-trop-corrections-min",
                "1",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["correction_encoding"], "qzss_l6")
            self.assertEqual(payload["atmos_messages"], 3)
            self.assertGreater(payload["ppp_atmospheric_trop_meters"], 1.5)

    def test_clas_ppp_cli_accepts_direct_qzss_l6_gridded_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_gridded_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, _, _ = build_synthetic_ppp_inputs_with_atmos(temp_root)
            l6_path = temp_root / "corrections_l6_gridded.bin"
            output_path = temp_root / "clas_ppp_l6_gridded.pos"
            summary_path = temp_root / "clas_ppp_l6_gridded_summary.json"
            network_id = 7
            grid_count = 22
            nearest_grid_index = 10
            trop_hs_residuals = build_single_grid_residuals(grid_count, nearest_grid_index, 0.24)
            trop_wet_residuals = build_single_grid_residuals(grid_count, nearest_grid_index, -0.06)
            stec_residuals = build_single_grid_residuals(grid_count, nearest_grid_index, 12.48)
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=356400, iod=3, prn=1, sync=True),
                        build_qzss_cssr_gridded_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_id=network_id,
                            trop_type=1,
                            trop_quality=9,
                            grid_count=grid_count,
                            selected_satellites=1,
                            trop_hs_residuals_m=trop_hs_residuals,
                            trop_wet_residuals_m=trop_wet_residuals,
                            stec_residuals_tecu=stec_residuals,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0,
                            iod=3,
                            prn=1,
                            network_id=network_id,
                            dclock_m=0.0,
                            sync=False,
                        ),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=356430, iod=3, prn=1, sync=True),
                        build_qzss_cssr_gridded_message(
                            tow_delta=30,
                            iod=3,
                            sync=True,
                            network_id=network_id,
                            trop_type=1,
                            trop_quality=9,
                            grid_count=grid_count,
                            selected_satellites=1,
                            trop_hs_residuals_m=trop_hs_residuals,
                            trop_wet_residuals_m=trop_wet_residuals,
                            stec_residuals_tecu=stec_residuals,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=30,
                            iod=3,
                            prn=1,
                            network_id=network_id,
                            dclock_m=0.0,
                            sync=False,
                        ),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=356460, iod=3, prn=1, sync=True),
                        build_qzss_cssr_gridded_message(
                            tow_delta=60,
                            iod=3,
                            sync=True,
                            network_id=network_id,
                            trop_type=1,
                            trop_quality=9,
                            grid_count=grid_count,
                            selected_satellites=1,
                            trop_hs_residuals_m=trop_hs_residuals,
                            trop_wet_residuals_m=trop_wet_residuals,
                            stec_residuals_tecu=stec_residuals,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=60,
                            iod=3,
                            prn=1,
                            network_id=network_id,
                            dclock_m=0.0,
                            sync=False,
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "2411",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--no-estimate-troposphere",
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
                "--require-atmos-messages-min",
                "3",
                "--require-ppp-atmos-trop-corrections-min",
                "1",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["correction_encoding"], "qzss_l6")
            self.assertEqual(payload["atmos_messages"], 3)
            self.assertGreaterEqual(payload["ppp_atmospheric_trop_corrections"], 1)
            self.assertGreater(payload["ppp_atmospheric_trop_meters"], 1.0)

    def test_clas_ppp_cli_accepts_direct_qzss_l6_code_phase_bias(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_code_phase_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            l6_path = temp_root / "corrections_l6_code_phase.bin"
            output_path = temp_root / "clas_ppp_l6_code_phase.pos"
            summary_path = temp_root / "clas_ppp_l6_code_phase_summary.json"
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0, iod=3, code_bias_m=-0.12, phase_bias_m=0.015, sync=True
                        ),
                        build_qzss_cssr_combined_message(tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518430, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=30, iod=3, code_bias_m=-0.12, phase_bias_m=0.015, sync=True
                        ),
                        build_qzss_cssr_combined_message(tow_delta=30, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518460, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=60, iod=3, code_bias_m=-0.12, phase_bias_m=0.015, sync=True
                        ),
                        build_qzss_cssr_combined_message(tow_delta=60, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "1316",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())
            self.assertIn("decoded qzss l6 corrections:", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["correction_encoding"], "qzss_l6")
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)

    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_ubx_info_reads_mixed_rawx_from_serial_device(self) -> None:
        payload = build_nav_pvt_message() + build_mixed_rawx_message()
        last_result: subprocess.CompletedProcess[str] | None = None
        last_output_path: Path | None = None

        for _ in range(3):
            master_fd, slave_fd = pty.openpty()
            try:
                slave_path = os.ttyname(slave_fd)
            finally:
                os.close(slave_fd)

            def writer() -> None:
                time.sleep(0.10)
                os.write(master_fd, payload)
                time.sleep(0.05)
                os.write(master_fd, payload)
                time.sleep(0.20)
                os.close(master_fd)

            with tempfile.TemporaryDirectory(prefix="gnss_ubx_serial_test_") as temp_dir:
                output_path = Path(temp_dir) / "serial.obs"
                last_output_path = output_path
                thread = threading.Thread(target=writer)
                thread.start()
                try:
                    result = self.run_gnss(
                        "ubx-info",
                        "--input",
                        f"serial://{slave_path}?baud=115200",
                        "--decode-observations",
                        "--obs-rinex-out",
                        str(output_path),
                        "--limit",
                        "4",
                    )
                finally:
                    thread.join()

                last_result = result
                if "UBX-RXM-RAWX" in result.stdout and "summary: processed_messages=" in result.stdout:
                    exported = output_path.read_text(encoding="ascii")
                    self.assertIn("G12", exported)
                    self.assertIn("E05", exported)
                    self.assertIn("R07", exported)
                    self.assertIn("C19", exported)
                    self.assertIn("J03", exported)
                    return

        self.assertIsNotNone(last_result)
        self.assertEqual(last_result.returncode, 0, msg=last_result.stderr)
        self.assertIsNotNone(last_output_path)
        self.assertIn("UBX-RXM-RAWX", last_result.stdout)

    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_nmea_info_reads_sentences_from_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        payload = (build_nmea_gga_sentence() + build_nmea_rmc_sentence()).encode("ascii")

        def writer() -> None:
            time.sleep(0.05)
            os.write(master_fd, payload)
            time.sleep(0.1)
            os.close(master_fd)

        thread = threading.Thread(target=writer)
        thread.start()
        try:
            result = self.run_gnss(
                "nmea-info",
                "--input",
                f"serial://{slave_path}?baud=9600",
                "--limit",
                "2",
            )
        finally:
            thread.join()

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("gga: time=123519", result.stdout)
        self.assertIn("rmc: time=123520", result.stdout)
        self.assertIn(
            "summary: sentences=2 valid=2 gga=1 rmc=1 valid_positions=2 checksum_errors=0",
            result.stdout,
        )

    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_novatel_info_reads_records_from_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        payload = (build_novatel_bestpos_record() + build_novatel_bestvel_record()).encode("ascii")

        def writer() -> None:
            time.sleep(0.20)
            os.write(master_fd, payload)
            time.sleep(0.1)
            os.close(master_fd)

        thread = threading.Thread(target=writer)
        thread.start()
        try:
            result = self.run_gnss(
                "novatel-info",
                "--input",
                f"serial://{slave_path}?baud=115200",
                "--limit",
                "2",
            )
        finally:
            thread.join()

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("bestpos: week=2200 tow=345600.000", result.stdout)
        self.assertIn("bestvel: week=2200 tow=345600.000", result.stdout)
        self.assertIn(
            "summary: records=2 valid=2 bestpos=1 bestvel=1 valid_positions=1 checksum_errors=0",
            result.stdout,
        )

    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_novatel_info_reads_binary_records_from_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        payload = build_novatel_bestpos_binary_record() + build_novatel_bestvel_binary_record()

        def writer() -> None:
            time.sleep(0.20)
            os.write(master_fd, payload)
            time.sleep(0.1)
            os.close(master_fd)

        thread = threading.Thread(target=writer)
        thread.start()
        try:
            result = self.run_gnss(
                "novatel-info",
                "--input",
                f"serial://{slave_path}?baud=115200",
                "--limit",
                "2",
            )
        finally:
            thread.join()

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("bestpos: week=2200 tow=345600.000", result.stdout)
        self.assertIn("bestvel: week=2200 tow=345600.000", result.stdout)
        self.assertIn(
            "summary: records=2 valid=2 bestpos=1 bestvel=1 valid_positions=1 checksum_errors=0",
            result.stdout,
        )

    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_sbp_info_reads_records_from_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        payload = build_sbp_gps_time_frame() + build_sbp_pos_llh_frame() + build_sbp_vel_ned_frame()

        def writer() -> None:
            time.sleep(0.20)
            os.write(master_fd, payload)
            time.sleep(0.1)
            os.close(master_fd)

        thread = threading.Thread(target=writer)
        thread.start()
        try:
            result = self.run_gnss(
                "sbp-info",
                "--input",
                f"serial://{slave_path}?baud=115200",
                "--limit",
                "3",
            )
        finally:
            thread.join()

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("gps_time: sender=66 week=2200 tow_ms=345600123", result.stdout)
        self.assertIn("pos_llh: sender=66 tow_ms=345600123", result.stdout)
        self.assertIn("vel_ned: sender=66 tow_ms=345600123", result.stdout)
        self.assertIn(
            "summary: frames=3 valid=3 gps_time=1 pos_llh=1 vel_ned=1 valid_positions=1 crc_errors=0",
            result.stdout,
        )

    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_sbf_info_reads_records_from_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        payload = (
            build_sbf_pvt_geodetic_frame()
            + build_sbf_lband_tracker_frame()
            + build_sbf_p2pp_status_frame()
        )

        def writer() -> None:
            time.sleep(0.20)
            os.write(master_fd, payload)
            time.sleep(0.1)
            os.close(master_fd)

        thread = threading.Thread(target=writer)
        thread.start()
        try:
            result = self.run_gnss(
                "sbf-info",
                "--input",
                f"serial://{slave_path}?baud=115200",
                "--limit",
                "3",
            )
        finally:
            thread.join()

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("pvt_geodetic: week=2200 tow_ms=345600123 mode=PPP", result.stdout)
        self.assertIn("lband_tracker: week=2200 tow_ms=345600123", result.stdout)
        self.assertIn("p2pp_status: week=2200 tow_ms=345600123", result.stdout)
        self.assertIn(
            "summary: frames=3 valid=3 pvt_geodetic=1 lband_tracker=1 p2pp_status=1 valid_positions=1 crc_errors=0",
            result.stdout,
        )

    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_trimble_info_reads_records_from_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        payload = build_gsof_genout_packet(
            [
                build_gsof_time_record(),
                build_gsof_llh_record(),
                build_gsof_velocity_record(),
            ]
        )

        def writer() -> None:
            time.sleep(0.20)
            os.write(master_fd, payload)
            time.sleep(0.1)
            os.close(master_fd)

        thread = threading.Thread(target=writer)
        thread.start()
        try:
            result = self.run_gnss(
                "trimble-info",
                "--input",
                f"serial://{slave_path}?baud=115200",
                "--limit",
                "3",
            )
        finally:
            thread.join()

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("time: week=2200 tow=345600.123", result.stdout)
        self.assertIn("llh: lat=35.1234567 lon=139.9876543 height=42.100m", result.stdout)
        self.assertIn("velocity: flags=0x03 horiz=1.250mps heading=90.00deg vertical=-0.125mps", result.stdout)
        self.assertIn(
            "summary: packets=1 valid=1 time=1 llh=1 velocity=1 valid_positions=1 checksum_errors=0",
            result.stdout,
        )

    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_skytraq_info_reads_epoch_and_nack_from_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        payload = build_skytraq_epoch_message() + build_skytraq_nack_message()

        def writer() -> None:
            time.sleep(0.20)
            os.write(master_fd, payload)
            time.sleep(0.1)
            os.close(master_fd)

        thread = threading.Thread(target=writer)
        thread.start()
        try:
            result = self.run_gnss(
                "skytraq-info",
                "--input",
                f"serial://{slave_path}?baud=115200",
                "--limit",
                "2",
            )
        finally:
            thread.join()

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("epoch: iod=7 week=2200 tow=345600.123", result.stdout)
        self.assertIn("nack: msg=0x09", result.stdout)
        self.assertIn(
            "summary: frames=2 valid=2 epoch=1 raw=0 rawx=0 ack=0 nack=1 checksum_errors=0",
            result.stdout,
        )

    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_binex_info_reads_records_from_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        payload = build_binex_metadata_frame() + build_binex_proto_frame()

        def writer() -> None:
            time.sleep(0.20)
            os.write(master_fd, payload)
            time.sleep(0.1)
            os.close(master_fd)

        thread = threading.Thread(target=writer)
        thread.start()
        try:
            result = self.run_gnss(
                "binex-info",
                "--input",
                f"serial://{slave_path}?baud=115200",
                "--decode-metadata",
                "--decode-proto",
                "--limit",
                "2",
            )
        finally:
            thread.join()

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("metadata: subrecord=0x08 payload_bytes=4", result.stdout)
        self.assertIn("proto: subrecord=0x05 payload_bytes=4", result.stdout)
        self.assertIn(
            "summary: frames=2 valid=2 metadata=1 nav=0 proto=1 checksum_errors=0",
            result.stdout,
        )

    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_qzss_l6_info_reads_frames_from_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        payload = (
            build_qzss_l6_frame(
                prn=199,
                facility_id=1,
                subframe_start=True,
                data_part=b"L6-SERIAL-ONE",
            )
            + build_qzss_l6_frame(
                prn=201,
                facility_id=3,
                subframe_start=False,
                data_part=b"L6-SERIAL-TWO",
            )
        )

        def writer() -> None:
            time.sleep(0.20)
            os.write(master_fd, payload)
            time.sleep(0.1)
            os.close(master_fd)

        thread = threading.Thread(target=writer)
        thread.start()
        try:
            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                f"serial://{slave_path}?baud=115200",
                "--limit",
                "2",
                "--show-preview",
            )
        finally:
            thread.join()

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("l6_frame: index=1 prn=199 vendor=5 facility=Hitachi-Ota", result.stdout)
        self.assertIn("preview=L6-SERIAL-ONE", result.stdout)
        self.assertIn("l6_frame: index=2 prn=201 vendor=5 facility=Kobe", result.stdout)
        self.assertIn(
            "summary: frames=2 valid=2 clas_vendor=2 subframe_starts=1 alerts=0 subframes=0 prns=199,201",
            result.stdout,
        )

    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_convert_reads_mixed_rawx_from_serial_device(self) -> None:
        payload = build_nav_pvt_message() + build_mixed_rawx_message()
        last_result: subprocess.CompletedProcess[str] | None = None
        for _ in range(3):
            master_fd, slave_fd = pty.openpty()
            try:
                slave_path = os.ttyname(slave_fd)
            finally:
                os.close(slave_fd)

            def writer() -> None:
                time.sleep(0.10)
                os.write(master_fd, payload)
                time.sleep(0.05)
                os.write(master_fd, payload)
                time.sleep(0.20)
                os.close(master_fd)

            with tempfile.TemporaryDirectory(prefix="gnss_convert_serial_test_") as temp_dir:
                output_path = Path(temp_dir) / "serial_converted.obs"
                thread = threading.Thread(target=writer)
                thread.start()
                try:
                    result = self.run_gnss(
                        "convert",
                        "--format",
                        "ubx",
                        "--input",
                        f"serial://{slave_path}?baud=115200",
                        "--obs-out",
                        str(output_path),
                        "--limit",
                        "4",
                        "--quiet",
                    )
                finally:
                    thread.join()

                last_result = result
                if "summary: processed_messages=" in result.stdout and output_path.exists():
                    exported = output_path.read_text(encoding="ascii")
                    if all(token in exported for token in ("G12", "E05", "R07", "C19", "J03")):
                        self.assertEqual(result.returncode, 0, msg=result.stderr)
                        return

        self.assertIsNotNone(last_result)
        self.assertEqual(last_result.returncode, 0, msg=last_result.stderr)
        self.assertIn("summary: processed_messages=", last_result.stdout)

    def test_convert_exports_sfrbx_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_convert_sfrbx_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.ubx"
            output_path = temp_root / "session_sfrbx.csv"
            input_path.write_bytes(build_sfrbx_message())

            result = self.run_gnss(
                "convert",
                "--format",
                "ubx",
                "--input",
                str(input_path),
                "--sfrbx-out",
                str(output_path),
                "--quiet",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: processed_messages=1", result.stdout)
            self.assertIn("exported_sfrbx_messages=1", result.stdout)
            exported = output_path.read_text(encoding="ascii")
            self.assertIn(
                "system,sv_id,frequency_id,channel,version,frame_kind,frame_id,page_id,word_count,words_hex",
                exported,
            )
            self.assertIn("GPS,12,0,1,2,GPS_LNAV,5,,3,8B0000AA;00000500;CAFEBABE", exported)

    def test_convert_exports_gps_nav_from_ubx_sfrbx(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_convert_nav_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_nav.ubx"
            output_path = temp_root / "session.nav"
            input_path.write_bytes(
                build_gps_lnav_sfrbx_message(1)
                + build_gps_lnav_sfrbx_message(2)
                + build_gps_lnav_sfrbx_message(3)
            )

            result = self.run_gnss(
                "convert",
                "--format",
                "ubx",
                "--input",
                str(input_path),
                "--nav-out",
                str(output_path),
                "--quiet",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: processed_messages=3", result.stdout)
            self.assertIn("exported_nav_messages=1", result.stdout)
            exported = output_path.read_text(encoding="ascii")
            self.assertIn("RINEX VERSION / TYPE", exported)
            self.assertIn("G12", exported)
            self.assertIn("3.456000000000D+05", exported)
            self.assertIn("5.153795890808D+03", exported)

    def test_convert_exports_glonass_nav_from_ubx_sfrbx(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_convert_glo_nav_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_glo_nav.ubx"
            output_path = temp_root / "session_glo.nav"
            input_path.write_bytes(
                build_rawx_message()
                + build_glonass_sfrbx_message(1)
                + build_glonass_sfrbx_message(2)
                + build_glonass_sfrbx_message(3)
                + build_glonass_sfrbx_message(4)
            )

            result = self.run_gnss(
                "convert",
                "--format",
                "ubx",
                "--input",
                str(input_path),
                "--nav-out",
                str(output_path),
                "--quiet",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: processed_messages=5", result.stdout)
            self.assertIn("exported_nav_messages=1", result.stdout)

            exported_lines = output_path.read_text(encoding="ascii").splitlines()
            record_index = next(index for index, line in enumerate(exported_lines) if line.startswith("R07"))
            self.assertEqual(len(exported_lines) - record_index, 4)
            self.assertTrue(exported_lines[record_index + 2].rstrip().endswith(" 1.000000000000D+00"))
            self.assertTrue(exported_lines[record_index + 3].rstrip().endswith(" 9.000000000000D+00"))

    def test_convert_exports_beidou_d1_nav_from_ubx_sfrbx(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_convert_bds_nav_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_bds_nav.ubx"
            output_path = temp_root / "session_bds.nav"
            input_path.write_bytes(
                build_beidou_d1_sfrbx_message(1)
                + build_beidou_d1_sfrbx_message(2)
                + build_beidou_d1_sfrbx_message(3)
            )

            result = self.run_gnss(
                "convert",
                "--format",
                "ubx",
                "--input",
                str(input_path),
                "--nav-out",
                str(output_path),
                "--quiet",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: processed_messages=3", result.stdout)
            self.assertIn("exported_nav_messages=1", result.stdout)

            exported = output_path.read_text(encoding="ascii")
            self.assertIn("C12", exported)
            self.assertIn("5.282625600815D+03", exported)
            self.assertIn("2.300000000000D-08", exported)

    def test_convert_exports_beidou_d2_nav_from_ubx_sfrbx(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_convert_bds_d2_nav_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_bds_d2_nav.ubx"
            output_path = temp_root / "session_bds_d2.nav"
            input_path.write_bytes(
                build_beidou_d2_sfrbx_message(1)
                + build_beidou_d2_sfrbx_message(3)
                + build_beidou_d2_sfrbx_message(4)
                + build_beidou_d2_sfrbx_message(5)
                + build_beidou_d2_sfrbx_message(6)
                + build_beidou_d2_sfrbx_message(7)
                + build_beidou_d2_sfrbx_message(8)
                + build_beidou_d2_sfrbx_message(9)
                + build_beidou_d2_sfrbx_message(10)
            )

            result = self.run_gnss(
                "convert",
                "--format",
                "ubx",
                "--input",
                str(input_path),
                "--nav-out",
                str(output_path),
                "--quiet",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: processed_messages=9", result.stdout)
            self.assertIn("exported_nav_messages=1", result.stdout)

            exported = output_path.read_text(encoding="ascii")
            self.assertIn("C03", exported)
            self.assertIn("5.282625600815D+03", exported)
            self.assertIn("2.300000000000D-08", exported)

    def test_convert_exports_galileo_nav_from_ubx_sfrbx(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_convert_gal_nav_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_gal_nav.ubx"
            output_path = temp_root / "session_gal.nav"
            input_path.write_bytes(
                b"".join(build_galileo_inav_sfrbx_message(word_type) for word_type in range(6))
            )

            result = self.run_gnss(
                "convert",
                "--format",
                "ubx",
                "--input",
                str(input_path),
                "--nav-out",
                str(output_path),
                "--quiet",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: processed_messages=6", result.stdout)
            self.assertIn("exported_nav_messages=1", result.stdout)

            exported = output_path.read_text(encoding="ascii")
            self.assertIn("E05", exported)
            self.assertIn("5.440588203430D+03", exported)
            self.assertIn("1.094304025173D-08", exported)

    def test_replay_solves_bundled_rinex_sequence(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_replay_test_") as temp_dir:
            temp_root = Path(temp_dir)
            output_path = temp_root / "replay.pos"

            result = self.run_gnss(
                "replay",
                "--rover-rinex",
                str(ROOT_DIR / "data" / "rover_kinematic.obs"),
                "--base-rinex",
                str(ROOT_DIR / "data" / "base_kinematic.obs"),
                "--nav-rinex",
                str(ROOT_DIR / "data" / "navigation_kinematic.nav"),
                "--out",
                str(output_path),
                "--max-epochs",
                "20",
                "--quiet",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: aligned_epochs=", result.stdout)
            self.assertIn("written_solutions=", result.stdout)
            self.assertTrue(output_path.exists())
            exported = output_path.read_text(encoding="ascii")
            self.assertIn("LibGNSS++ Position Solution", exported)
            solution_lines = [
                line for line in exported.splitlines()
                if line and not line.startswith("%")
            ]
            self.assertGreater(len(solution_lines), 0)

    def test_replay_supports_moving_base_mode(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_replay_moving_base_") as temp_dir:
            temp_root = Path(temp_dir)
            output_path = temp_root / "replay_moving_base.pos"

            result = self.run_gnss(
                "replay",
                "--rover-rinex",
                str(ROOT_DIR / "data" / "rover_kinematic.obs"),
                "--base-rinex",
                str(ROOT_DIR / "data" / "base_kinematic.obs"),
                "--nav-rinex",
                str(ROOT_DIR / "data" / "navigation_kinematic.nav"),
                "--out",
                str(output_path),
                "--mode",
                "moving-base",
                "--max-epochs",
                "10",
                "--quiet",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("mode=moving-base", result.stdout)
            self.assertTrue(output_path.exists())

    def test_live_command_help_is_available(self) -> None:
        result = self.run_gnss("live", "--help")
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("Usage:", result.stdout)
        self.assertIn("--rover-rtcm", result.stdout)
        self.assertIn("--rover-ubx", result.stdout)
        self.assertIn("--base-hold-seconds", result.stdout)
        self.assertIn("--mode <kinematic|moving-base>", result.stdout)
        self.assertIn("--preset <survey|low-cost|moving-base>", result.stdout)
        self.assertIn("--arfilter", result.stdout)
        self.assertIn("--no-arfilter", result.stdout)
        self.assertIn("--arfilter-margin", result.stdout)
        self.assertIn("--min-hold-count", result.stdout)
        self.assertIn("--hold-ratio-threshold", result.stdout)

    def test_replay_command_help_mentions_arfilter(self) -> None:
        result = self.run_gnss("replay", "--help")
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("--preset <survey|low-cost|moving-base>", result.stdout)
        self.assertIn("--arfilter", result.stdout)
        self.assertIn("--no-arfilter", result.stdout)
        self.assertIn("--arfilter-margin", result.stdout)
        self.assertIn("--min-hold-count", result.stdout)
        self.assertIn("--hold-ratio-threshold", result.stdout)
        self.assertIn("--base-ubx", result.stdout)

    def test_solve_accepts_low_cost_preset_and_hold_knobs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_solve_preset_") as temp_dir:
            output_path = Path(temp_dir) / "solve_preset.pos"
            result = self.run_gnss(
                "solve",
                "--rover",
                str(ROOT_DIR / "data" / "rover_kinematic.obs"),
                "--base",
                str(ROOT_DIR / "data" / "base_kinematic.obs"),
                "--nav",
                str(ROOT_DIR / "data" / "navigation_kinematic.nav"),
                "--preset",
                "low-cost",
                "--min-hold-count",
                "7",
                "--hold-ratio-threshold",
                "2.6",
                "--max-epochs",
                "5",
                "--out",
                str(output_path),
                "--no-kml",
            )
            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("mode: kinematic", result.stdout)
            self.assertTrue(output_path.exists())

    @unittest.skipUnless(ros2_bag_support_available(), "ROS2 rosbag + ublox_msgs support not available")
    def test_moving_base_prepare_exports_ubx_and_reference_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_moving_base_prepare_") as temp_dir:
            temp_root = Path(temp_dir)
            bag_dir = temp_root / "bag"
            build_synthetic_moving_base_rosbag(bag_dir)

            rover_ubx = temp_root / "rover.ubx"
            base_ubx = temp_root / "base.ubx"
            reference_csv = temp_root / "reference.csv"
            summary_json = temp_root / "summary.json"

            result = self.run_gnss(
                "moving-base-prepare",
                "--input",
                str(bag_dir),
                "--rover-ubx-out",
                str(rover_ubx),
                "--base-ubx-out",
                str(base_ubx),
                "--reference-csv",
                str(reference_csv),
                "--summary-json",
                str(summary_json),
                "--max-epochs",
                "2",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Prepared moving-base bag artifacts.", result.stdout)
            self.assertTrue(rover_ubx.exists())
            self.assertTrue(base_ubx.exists())
            self.assertTrue(reference_csv.exists())
            self.assertTrue(summary_json.exists())
            self.assertGreater(rover_ubx.stat().st_size, 0)
            self.assertGreater(base_ubx.stat().st_size, 0)

            summary = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(summary["rover_epochs"], 2)
            self.assertEqual(summary["matched_reference_rows"], 2)
            self.assertEqual(summary["gps_week"], 2200)
            self.assertEqual(summary["date"], "2023-06-14")

            with reference_csv.open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["gps_week"], "2200")
            self.assertEqual(rows[0]["gps_tow_s"], "345600.000")
            self.assertIn("baseline_n_m", rows[0])

    def test_moving_base_prepare_help_mentions_rosbag_and_ubx_exports(self) -> None:
        result = self.run_gnss("moving-base-prepare", "--help")
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("ROS2 bag directory or Zenodo zip", result.stdout)
        self.assertIn("--rover-ubx-out", result.stdout)
        self.assertIn("--base-ubx-out", result.stdout)

    @unittest.skipUnless(ros2_bag_support_available(), "ROS2 rosbag + ublox_msgs support not available")
    def test_scorpion_moving_base_signoff_wraps_prepare_and_existing_solution(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_scorpion_moving_base_") as temp_dir:
            temp_root = Path(temp_dir)
            bag_dir = temp_root / "bag"
            build_synthetic_moving_base_rosbag(bag_dir)

            prepared_reference = temp_root / "prepared_reference.csv"
            prepared_summary = temp_root / "prepared_summary.json"
            prepare_result = self.run_gnss(
                "moving-base-prepare",
                "--input",
                str(bag_dir),
                "--reference-csv",
                str(prepared_reference),
                "--summary-json",
                str(prepared_summary),
                "--quiet",
            )
            self.assertEqual(prepare_result.returncode, 0, msg=prepare_result.stderr)

            solution_path = temp_root / "moving_base.pos"
            with prepared_reference.open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            solution_lines = ["% synthetic scorpion moving-base solution"]
            for row in rows:
                solution_lines.append(
                    " ".join(
                        [
                            row["gps_week"],
                            f"{float(row['gps_tow_s']):.3f}",
                            f"{float(row['rover_ecef_x_m']):.6f}",
                            f"{float(row['rover_ecef_y_m']):.6f}",
                            f"{float(row['rover_ecef_z_m']):.6f}",
                            "35.0",
                            "139.0",
                            "10.0",
                            "4",
                            "12",
                            "1.0",
                        ]
                    )
                )
            solution_path.write_text("\n".join(solution_lines) + "\n", encoding="ascii")

            work_dir = temp_root / "work"
            summary_json = temp_root / "scorpion_summary.json"
            result = self.run_gnss(
                "scorpion-moving-base-signoff",
                "--input",
                str(bag_dir),
                "--use-existing-solution",
                "--out",
                str(solution_path),
                "--work-dir",
                str(work_dir),
                "--summary-json",
                str(summary_json),
                "--require-matched-epochs-min",
                "2",
                "--require-fix-rate-min",
                "90",
                "--require-p95-baseline-error-max",
                "0.01",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Finished SCORPION moving-base sign-off.", result.stdout)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["signoff_profile"], "scorpion-moving-base")
            self.assertEqual(payload["matched_epochs"], 2)
            self.assertEqual(payload["fix_rate_pct"], 100.0)
            self.assertIsNone(payload["products_summary_json"])
            self.assertTrue(Path(payload["prepare_summary_json"]).exists())

    def test_live_signoff_summarizes_existing_log_and_enforces_realtime_gate(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_live_signoff_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            log_path = temp_root / "live.log"
            summary_path = temp_root / "live_summary.json"
            log_path.write_text(
                "\n".join(
                    [
                        "some debug line",
                        "summary: termination=completed rover_decoder_errors=0 base_decoder_errors=0 "
                        "aligned_epochs=3 written_solutions=3 fixed_solutions=1 "
                        "solver_wall_time_s=0.250000 solution_span_s=1.000000 "
                        "realtime_factor=4.000000 effective_epoch_rate_hz=12.000000",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            result = self.run_gnss(
                "live-signoff",
                "--use-existing-log",
                str(log_path),
                "--summary-json",
                str(summary_path),
                "--require-termination",
                "completed",
                "--require-written-solutions-min",
                "3",
                "--require-fixed-solutions-min",
                "1",
                "--require-realtime-factor-min",
                "1.0",
                "--require-effective-epoch-rate-min",
                "10.0",
                "--require-rover-decoder-errors-max",
                "0",
                "--require-base-decoder-errors-max",
                "0",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(summary_path.exists())
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["metrics"]["termination"], "completed")
            self.assertEqual(payload["metrics"]["written_solutions"], 3)
            self.assertEqual(payload["metrics"]["realtime_factor"], 4.0)
            self.assertIn("Finished live sign-off.", result.stdout)

    def test_web_serves_overview_solution_and_status_api(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_web_test_") as temp_dir:
            temp_root = Path(temp_dir)
            lib_pos = temp_root / "lib.pos"
            rtklib_pos = temp_root / "rtklib.pos"
            summary_json = temp_root / "odaiba_summary.json"
            status_json = temp_root / "receiver.status.json"
            live_summary = temp_root / "output" / "live_replay_summary.json"
            visibility_summary = temp_root / "output" / "visibility_static_summary.json"
            visibility_csv = temp_root / "output" / "visibility_static.csv"
            visibility_png = temp_root / "output" / "visibility_static.png"
            moving_base_summary = temp_root / "output" / "scorpion_moving_base_summary.json"
            port_file = temp_root / "port.txt"

            lib_pos.write_text(
                "\n".join(
                    [
                        "% synthetic libgnss++",
                        "2200 100.0 1.0 2.0 3.0 35.000000000 139.000000000 10.0 4 12 3.5",
                        "2200 101.0 2.0 3.0 4.0 35.000001000 139.000001000 10.1 3 11 2.1",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            rtklib_pos.write_text(
                "\n".join(
                    [
                        "% synthetic rtklib",
                        "2200 100.0 35.000000000 139.000000000 10.0 1 12",
                        "2200 101.0 35.000001000 139.000001000 10.1 2 11",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            summary_json.write_text(
                json.dumps(
                    {
                        "all_epochs": {
                            "libgnsspp": {"epochs": 11637, "fix_rate_pct": 8.11},
                            "rtklib": {"epochs": 8241, "fix_rate_pct": 7.22},
                        },
                        "common_epochs": {
                            "libgnsspp": {"median_h_m": 0.733387, "p95_h_m": 5.941091},
                            "rtklib": {"median_h_m": 0.703880, "p95_h_m": 27.673014},
                        },
                    }
                ),
                encoding="utf-8",
            )
            status_json.write_text(
                json.dumps(
                    {
                        "state": "running",
                        "pid": 1234,
                        "pid_running": True,
                        "uptime_seconds": 12.5,
                        "restart_count": 1,
                    }
                ),
                encoding="utf-8",
            )
            live_summary.parent.mkdir(parents=True, exist_ok=True)
            live_summary.write_text(
                json.dumps(
                    {
                        "execution_mode": "live",
                        "metrics": {
                            "termination": "completed",
                            "aligned_epochs": 3,
                            "written_solutions": 3,
                            "fixed_solutions": 1,
                            "realtime_factor": 3.5,
                            "effective_epoch_rate_hz": 12.0,
                            "rover_decoder_errors": 0,
                            "base_decoder_errors": 0,
                        },
                    }
                ),
                encoding="utf-8",
            )
            (temp_root / "output" / "ppc_tokyo_run1_rtk_summary.json").write_text(
                json.dumps(
                    {
                        "matched_epochs": 120,
                        "fix_rate_pct": 96.67,
                        "median_h_m": 0.108,
                        "p95_h_m": 0.110,
                        "solver_wall_time_s": 12.34,
                        "realtime_factor": 1.23,
                        "effective_epoch_rate_hz": 15.67,
                    }
                ),
                encoding="utf-8",
            )
            moving_base_summary.write_text(
                json.dumps(
                    {
                        "matched_epochs": 94,
                        "valid_epochs": 94,
                        "fix_rate_pct": 95.74,
                        "median_baseline_error_m": 0.042,
                        "p95_baseline_error_m": 0.101,
                        "p95_heading_error_deg": 5.85,
                        "termination": "completed",
                        "realtime_factor": 2.17,
                        "effective_epoch_rate_hz": 10.84,
                        "solution_pos": str(temp_root / "output" / "scorpion_moving_base.pos"),
                        "prepare_summary_json": str(temp_root / "output" / "prepare_summary.json"),
                        "products_summary_json": str(temp_root / "output" / "products_summary.json"),
                        "signoff_profile": "scorpion-moving-base",
                    }
                ),
                encoding="utf-8",
            )
            visibility_summary.write_text(
                json.dumps(
                    {
                        "csv": str(visibility_csv),
                        "epochs_processed": 5,
                        "epochs_with_rows": 5,
                        "rows_written": 27,
                        "unique_satellites": 9,
                        "mean_satellites_per_epoch": 5.4,
                        "max_satellites_per_epoch": 7,
                        "mean_elevation_deg": 38.2,
                        "mean_snr_dbhz": 44.7,
                    }
                ),
                encoding="utf-8",
            )
            visibility_csv.write_text(
                "\n".join(
                    [
                        "epoch_index,week,tow,satellite,system,signal,azimuth_deg,elevation_deg,snr_dbhz,has_pseudorange,has_carrier_phase,has_doppler",
                        "1,2200,100.000,G01,GPS,GPS_L1CA,45.0,30.0,42.0,1,1,0",
                        "1,2200,100.000,G02,GPS,GPS_L1CA,120.0,55.0,47.4,1,1,0",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            visibility_png.write_bytes(
                binascii.a2b_base64(
                    b"iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO5+ymsAAAAASUVORK5CYII="
                )
            )

            port = find_free_port()
            process = subprocess.Popen(
                [
                    sys.executable,
                    str(DISPATCHER),
                    "web",
                    "--host",
                    "127.0.0.1",
                    "--port",
                    str(port),
                    "--port-file",
                    str(port_file),
                    "--root",
                    str(temp_root),
                    "--lib-pos",
                    str(lib_pos),
                    "--rtklib-pos",
                    str(rtklib_pos),
                    "--odaiba-summary",
                    str(summary_json),
                    "--rcv-status",
                    str(status_json),
                ],
                cwd=ROOT_DIR,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            try:
                bound_port = int(wait_for_file(port_file))
                with request.urlopen(f"http://127.0.0.1:{bound_port}/") as response:
                    html = response.read().decode("utf-8")
                self.assertIn("libgnss++ local web UI", html)

                with request.urlopen(f"http://127.0.0.1:{bound_port}/api/overview") as response:
                    overview = json.loads(response.read().decode("utf-8"))
                self.assertEqual(overview["odaiba_summary"]["all_epochs"]["libgnsspp"]["epochs"], 11637)
                self.assertEqual(len(overview["live_summaries"]), 1)
                self.assertEqual(overview["live_summaries"][0]["termination"], "completed")
                self.assertEqual(overview["live_summaries"][0]["realtime_factor"], 3.5)
                self.assertEqual(overview["live_summaries"][0]["runtime_status"], "realtime")
                self.assertEqual(len(overview["ppc_summaries"]), 1)
                self.assertEqual(overview["ppc_summaries"][0]["runtime_status"], "realtime")
                self.assertEqual(overview["ppc_summaries"][0]["quality_status"], "excellent")
                self.assertEqual(len(overview["moving_base_summaries"]), 1)
                self.assertEqual(overview["moving_base_summaries"][0]["runtime_status"], "realtime")
                self.assertEqual(overview["moving_base_summaries"][0]["quality_status"], "excellent")
                self.assertAlmostEqual(overview["moving_base_summaries"][0]["p95_baseline_error_m"], 0.101)
                self.assertEqual(overview["moving_base_summaries"][0]["signoff_profile"], "scorpion-moving-base")
                self.assertEqual(len(overview["visibility_summaries"]), 1)
                self.assertEqual(overview["visibility_summaries"][0]["rows_written"], 27)
                self.assertEqual(overview["visibility_summaries"][0]["unique_satellites"], 9)
                self.assertEqual(overview["visibility_summaries"][0]["csv_path"], "output/visibility_static.csv")
                self.assertEqual(overview["visibility_summaries"][0]["png_path"], "output/visibility_static.png")

                with request.urlopen(
                    f"http://127.0.0.1:{bound_port}/api/visibility?path=output/visibility_static.csv"
                ) as response:
                    visibility_payload = json.loads(response.read().decode("utf-8"))
                self.assertTrue(visibility_payload["available"])
                self.assertEqual(len(visibility_payload["rows"]), 2)
                self.assertEqual(visibility_payload["rows"][0]["satellite"], "G01")

                with request.urlopen(
                    f"http://127.0.0.1:{bound_port}/artifact?path=output/visibility_static.png"
                ) as response:
                    artifact_payload = response.read()
                self.assertGreater(len(artifact_payload), 0)

                with request.urlopen(f"http://127.0.0.1:{bound_port}/api/solution?name=libgnsspp") as response:
                    lib_payload = json.loads(response.read().decode("utf-8"))
                self.assertTrue(lib_payload["available"])
                self.assertEqual(lib_payload["epoch_count"], 2)
                self.assertIn("FIXED", lib_payload["status_counts"])

                with request.urlopen(f"http://127.0.0.1:{bound_port}/api/status") as response:
                    status_payload = json.loads(response.read().decode("utf-8"))
                self.assertTrue(status_payload["available"])
                self.assertEqual(status_payload["state"], "running")
            finally:
                process.terminate()
                try:
                    process.communicate(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate(timeout=5)

    def test_live_reports_missing_rtcm_source_path_clearly(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_live_missing_source_") as temp_dir:
            temp_root = Path(temp_dir)
            rover_path = temp_root / "missing_rover.rtcm3"
            base_path = temp_root / "missing_base.rtcm3"
            output_path = temp_root / "live.pos"

            result = self.run_gnss(
                "live",
                "--rover-rtcm",
                str(rover_path),
                "--base-rtcm",
                str(base_path),
                "--mode",
                "moving-base",
                "--out",
                str(output_path),
                "--quiet",
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn(
                f"Error: failed to open rover RTCM source: {rover_path}",
                result.stderr,
            )

    def test_moving_base_signoff_summarizes_existing_solution(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_moving_base_signoff_") as temp_dir:
            temp_root = Path(temp_dir)
            solution_path = temp_root / "moving_base.pos"
            reference_csv = temp_root / "reference.csv"
            summary_path = temp_root / "moving_base_summary.json"

            solution_path.write_text(
                "\n".join(
                    [
                        "% synthetic moving-base solution fixture",
                        "2200 345600.000 3875001.100000 332002.000000 5029000.400000 35.0 139.0 10.0 4 12 1.0",
                        "2200 345601.000 3875001.200000 332002.100000 5029000.450000 35.0 139.0 10.0 4 12 1.0",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            reference_csv.write_text(
                "\n".join(
                    [
                        "gps_week,gps_tow_s,base_ecef_x_m,base_ecef_y_m,base_ecef_z_m,rover_ecef_x_m,rover_ecef_y_m,rover_ecef_z_m",
                        "2200,345600.0,3875000.0,332000.0,5029000.0,3875001.0,332002.0,5029000.5",
                        "2200,345601.0,3875000.1,332000.1,5029000.0,3875001.1,332002.1,5029000.5",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            result = self.run_gnss(
                "moving-base-signoff",
                "--solver",
                "replay",
                "--use-existing-solution",
                "--out",
                str(solution_path),
                "--reference-csv",
                str(reference_csv),
                "--summary-json",
                str(summary_path),
                "--solver-wall-time-s",
                "0.5",
                "--require-valid-epochs-min",
                "2",
                "--require-matched-epochs-min",
                "2",
                "--require-fix-rate-min",
                "90",
                "--require-p95-baseline-error-max",
                "1.0",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Finished moving-base sign-off.", result.stdout)
            self.assertTrue(summary_path.exists())
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["solver"], "replay")
            self.assertEqual(payload["matched_epochs"], 2)
            self.assertEqual(payload["fixed_epochs"], 2)
            self.assertGreater(payload["realtime_factor"], 0.0)

    def test_rcv_dry_run_and_status_snapshot(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_test_") as temp_dir:
            temp_root = Path(temp_dir)
            config_path = temp_root / "receiver.conf"
            status_path = temp_root / "receiver_status.json"
            output_path = temp_root / "receiver.pos"
            config_path.write_text(
                "\n".join(
                    [
                        f"rover_rtcm={temp_root / 'rover.rtcm3'}",
                        f"base_rtcm={temp_root / 'base.rtcm3'}",
                        f"out={output_path}",
                        "max_epochs=1",
                        "base_hold_seconds=0.5",
                        "quiet=true",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            dry_run = self.run_gnss("rcv", "--config", str(config_path), "--dry-run")
            self.assertEqual(dry_run.returncode, 0, msg=dry_run.stderr)
            resolved = json.loads(dry_run.stdout)
            self.assertEqual(resolved["config"]["max_epochs"], "1")
            self.assertIn("--base-hold-seconds", resolved["command"])

            config_path.write_text(
                "\n".join(
                    [
                        f"rover_rtcm={temp_root / 'missing_rover.rtcm3'}",
                        f"base_rtcm={temp_root / 'missing_base.rtcm3'}",
                        f"out={output_path}",
                        "max_epochs=1",
                        "quiet=true",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            run_result = self.run_gnss(
                "rcv",
                "--config",
                str(config_path),
                "--status-out",
                str(status_path),
            )
            self.assertNotEqual(run_result.returncode, 0)
            self.assertTrue(status_path.exists())
            status = json.loads(status_path.read_text(encoding="utf-8"))
            self.assertEqual(status["state"], "failed")
            self.assertIn("command", status)
            self.assertEqual(status["returncode"], run_result.returncode)

    def test_rcv_dry_run_accepts_rover_ubx_config(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_ubx_test_") as temp_dir:
            temp_root = Path(temp_dir)
            config_path = temp_root / "receiver.conf"
            output_path = temp_root / "receiver.pos"
            config_path.write_text(
                "\n".join(
                    [
                        "rover_ubx=serial:///dev/ttyACM0",
                        "rover_ubx_baud=230400",
                        f"base_rtcm={temp_root / 'base.rtcm3'}",
                        f"out={output_path}",
                        "max_epochs=1",
                        "quiet=true",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            dry_run = self.run_gnss("rcv", "--config", str(config_path), "--dry-run")
            self.assertEqual(dry_run.returncode, 0, msg=dry_run.stderr)
            resolved = json.loads(dry_run.stdout)
            self.assertEqual(resolved["config"]["rover_ubx"], "serial:///dev/ttyACM0")
            self.assertIn("--rover-ubx", resolved["command"])
            self.assertIn("--rover-ubx-baud", resolved["command"])

    def test_rcv_start_and_status_report_failed_background_run(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_start_test_") as temp_dir:
            temp_root = Path(temp_dir)
            config_path = temp_root / "receiver.conf"
            status_path = temp_root / "receiver_status.json"
            log_path = temp_root / "receiver.log"
            output_path = temp_root / "receiver.pos"
            config_path.write_text(
                "\n".join(
                    [
                        f"rover_rtcm={temp_root / 'missing_rover.rtcm3'}",
                        f"base_rtcm={temp_root / 'missing_base.rtcm3'}",
                        f"out={output_path}",
                        "max_epochs=1",
                        "quiet=true",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            start_result = self.run_gnss(
                "rcv",
                "start",
                "--config",
                str(config_path),
                "--status-out",
                str(status_path),
                "--log-out",
                str(log_path),
            )
            self.assertEqual(start_result.returncode, 0, msg=start_result.stderr)
            launched = json.loads(start_result.stdout)
            self.assertEqual(launched["state"], "starting")
            self.assertEqual(launched["status_path"], str(status_path))
            self.assertEqual(launched["log_path"], str(log_path))

            status_result = self.run_gnss(
                "rcv",
                "status",
                "--status-out",
                str(status_path),
                "--wait-seconds",
                "5",
            )
            self.assertEqual(status_result.returncode, 0, msg=status_result.stderr)
            status = json.loads(status_result.stdout)
            self.assertEqual(status["state"], "failed")
            self.assertFalse(status["pid_running"])
            self.assertEqual(status["log_path"], str(log_path))
            self.assertGreaterEqual(status["uptime_seconds"], 0.0)
            self.assertTrue(log_path.exists())

    def test_rcv_stop_is_idempotent_when_receiver_is_not_running(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_stop_test_") as temp_dir:
            temp_root = Path(temp_dir)
            status_path = temp_root / "receiver_status.json"
            status_path.write_text(
                json.dumps(
                    {
                        "state": "completed",
                        "pid": 999999,
                        "config_path": str(temp_root / "receiver.conf"),
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            result = self.run_gnss("rcv", "stop", "--status-out", str(status_path))
            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("receiver not running", result.stdout)

    def test_rcv_status_includes_log_tail_and_restart_countdown(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_status_tail_test_") as temp_dir:
            temp_root = Path(temp_dir)
            status_path = temp_root / "receiver_status.json"
            log_path = temp_root / "receiver.log"
            now = time.time()
            next_restart_at = time.strftime("%Y-%m-%dT%H:%M:%S+00:00", time.gmtime(now + 3.0))
            log_path.write_text(
                "line one\nline two\nsummary: messages=5 written_solutions=3\n",
                encoding="utf-8",
            )
            status_path.write_text(
                json.dumps(
                    {
                        "state": "restarting",
                        "pid": 999999,
                        "config_path": str(temp_root / "receiver.conf"),
                        "started_at": "2026-03-26T00:00:00+00:00",
                        "log_path": str(log_path),
                        "restart_count": 2,
                        "next_restart_at": next_restart_at,
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            result = self.run_gnss(
                "rcv",
                "status",
                "--status-out",
                str(status_path),
                "--tail-log-lines",
                "2",
            )
            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(result.stdout)
            self.assertFalse(payload["pid_running"])
            self.assertTrue(payload["log_exists"])
            self.assertEqual(payload["log_tail"], ["line two", "summary: messages=5 written_solutions=3"])
            self.assertEqual(payload["restart_count"], 2)
            self.assertGreaterEqual(payload["next_restart_in_seconds"], 0.0)
            self.assertLessEqual(payload["next_restart_in_seconds"], 5.0)

    def test_rcv_console_accepts_scripted_status_commands(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_console_test_") as temp_dir:
            temp_root = Path(temp_dir)
            config_path = temp_root / "receiver.conf"
            status_path = temp_root / "receiver_status.json"
            log_path = temp_root / "receiver.log"
            output_path = temp_root / "receiver.pos"
            config_path.write_text(
                "\n".join(
                    [
                        f"rover_rtcm={temp_root / 'missing_rover.rtcm3'}",
                        f"base_rtcm={temp_root / 'missing_base.rtcm3'}",
                        f"out={output_path}",
                        "max_epochs=1",
                        "quiet=true",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            log_path.write_text("line one\nsummary: messages=5 written_solutions=3\n", encoding="utf-8")
            status_path.write_text(
                json.dumps(
                    {
                        "state": "completed",
                        "pid": 999999,
                        "config_path": str(config_path),
                        "log_path": str(log_path),
                        "command": ["gnss_live", "--quiet"],
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            result = subprocess.run(
                [
                    sys.executable,
                    str(DISPATCHER),
                    "rcv",
                    "console",
                    "--config",
                    str(config_path),
                    "--status-out",
                    str(status_path),
                ],
                cwd=ROOT_DIR,
                text=True,
                input="show-config\ntail 1\nquit\n",
                capture_output=True,
                check=False,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn('"config_path":', result.stdout)
            self.assertIn('"log_tail": [', result.stdout)
            self.assertIn("summary: messages=5 written_solutions=3", result.stdout)

    def test_rcv_restart_without_existing_status_behaves_like_start(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_restart_test_") as temp_dir:
            temp_root = Path(temp_dir)
            config_path = temp_root / "receiver.conf"
            status_path = temp_root / "receiver_status.json"
            output_path = temp_root / "receiver.pos"
            config_path.write_text(
                "\n".join(
                    [
                        f"rover_rtcm={temp_root / 'missing_rover.rtcm3'}",
                        f"base_rtcm={temp_root / 'missing_base.rtcm3'}",
                        f"out={output_path}",
                        "max_epochs=1",
                        "quiet=true",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            restart_result = self.run_gnss(
                "rcv",
                "restart",
                "--config",
                str(config_path),
                "--status-out",
                str(status_path),
                "--wait-seconds",
                "0.1",
            )
            self.assertEqual(restart_result.returncode, 0, msg=restart_result.stderr)
            launched = json.loads(restart_result.stdout)
            self.assertEqual(launched["state"], "starting")
            self.assertEqual(launched["status_path"], str(status_path))

    def test_rcv_reload_without_existing_status_behaves_like_start(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_reload_test_") as temp_dir:
            temp_root = Path(temp_dir)
            config_path = temp_root / "receiver.conf"
            status_path = temp_root / "receiver_status.json"
            output_path = temp_root / "receiver.pos"
            config_path.write_text(
                "\n".join(
                    [
                        f"rover_rtcm={temp_root / 'missing_rover.rtcm3'}",
                        f"base_rtcm={temp_root / 'missing_base.rtcm3'}",
                        f"out={output_path}",
                        "max_epochs=1",
                        "quiet=true",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            reload_result = self.run_gnss(
                "rcv",
                "reload",
                "--config",
                str(config_path),
                "--status-out",
                str(status_path),
                "--wait-seconds",
                "0.1",
            )
            self.assertEqual(reload_result.returncode, 0, msg=reload_result.stderr)
            launched = json.loads(reload_result.stdout)
            self.assertEqual(launched["state"], "starting")
            self.assertEqual(launched["status_path"], str(status_path))

    def test_rcv_reload_resolves_config_from_existing_status(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_reload_managed_test_") as temp_dir:
            temp_root = Path(temp_dir)
            config_path = temp_root / "receiver.conf"
            status_path = temp_root / "receiver_status.json"
            output_path = temp_root / "receiver.pos"
            config_path.write_text(
                "\n".join(
                    [
                        f"rover_rtcm={temp_root / 'missing_rover.rtcm3'}",
                        f"base_rtcm={temp_root / 'missing_base.rtcm3'}",
                        f"out={output_path}",
                        "max_epochs=1",
                        "quiet=true",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            status_path.write_text(
                json.dumps(
                    {
                        "state": "completed",
                        "pid": 999999,
                        "config_path": str(config_path),
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            reload_result = self.run_gnss(
                "rcv",
                "reload",
                "--status-out",
                str(status_path),
                "--wait-seconds",
                "0.1",
            )
            self.assertEqual(reload_result.returncode, 0, msg=reload_result.stderr)
            launched = json.loads(reload_result.stdout)
            self.assertEqual(launched["state"], "starting")
            self.assertEqual(launched["config_path"], str(config_path))
            self.assertEqual(launched["status_path"], str(status_path))


if __name__ == "__main__":
    unittest.main()
