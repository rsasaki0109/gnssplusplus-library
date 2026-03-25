#!/usr/bin/env python3
"""CLI regression tests for the dispatcher-backed native tools."""

from __future__ import annotations

import os
import json
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import math
from pathlib import Path

if os.name != "nt":
    import pty


ROOT_DIR = Path(__file__).resolve().parents[1]
DISPATCHER = ROOT_DIR / "apps" / "gnss.py"


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


class CLIToolsTest(unittest.TestCase):
    def run_gnss(self, *args: str) -> subprocess.CompletedProcess[str]:
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        return subprocess.run(
            [sys.executable, str(DISPATCHER), *args],
            cwd=ROOT_DIR,
            env=env,
            text=True,
            capture_output=True,
            check=False,
        )

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

    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_ubx_info_reads_mixed_rawx_from_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        payload = build_nav_pvt_message() + build_mixed_rawx_message()

        def writer() -> None:
            time.sleep(0.05)
            os.write(master_fd, payload)
            time.sleep(0.1)
            os.close(master_fd)

        with tempfile.TemporaryDirectory(prefix="gnss_ubx_serial_test_") as temp_dir:
            output_path = Path(temp_dir) / "serial.obs"
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
                    "2",
                )
            finally:
                thread.join()

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("UBX-RXM-RAWX", result.stdout)
            self.assertIn("summary: processed_messages=2", result.stdout)
            exported = output_path.read_text(encoding="ascii")
            self.assertIn("G12", exported)
            self.assertIn("E05", exported)
            self.assertIn("R07", exported)
            self.assertIn("C19", exported)
            self.assertIn("J03", exported)

    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_convert_reads_mixed_rawx_from_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        payload = build_nav_pvt_message() + build_mixed_rawx_message()

        def writer() -> None:
            time.sleep(0.05)
            os.write(master_fd, payload)
            time.sleep(0.1)
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
                    "2",
                    "--quiet",
                )
            finally:
                thread.join()

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: processed_messages=2", result.stdout)
            exported = output_path.read_text(encoding="ascii")
            self.assertIn("G12", exported)
            self.assertIn("E05", exported)
            self.assertIn("R07", exported)
            self.assertIn("C19", exported)
            self.assertIn("J03", exported)

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

    def test_live_command_help_is_available(self) -> None:
        result = self.run_gnss("live", "--help")
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("Usage:", result.stdout)
        self.assertIn("--rover-rtcm", result.stdout)
        self.assertIn("--rover-ubx", result.stdout)
        self.assertIn("--base-hold-seconds", result.stdout)

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


if __name__ == "__main__":
    unittest.main()
