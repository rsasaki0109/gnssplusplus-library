#!/usr/bin/env python3
"""Inspect Swift Binary Protocol logs or serial streams."""

from __future__ import annotations

import argparse
import binascii
import errno
import os
import struct
from dataclasses import dataclass

if os.name != "nt":
    import termios


SBP_PREAMBLE = 0x55
SBP_MSG_GPS_TIME = 0x0102
SBP_MSG_POS_LLH = 0x020A
SBP_MSG_VEL_NED = 0x020E


@dataclass
class GPSTimeRecord:
    sender_id: int
    tow_ms: int
    week: int
    ns_residual: int
    flags: int


@dataclass
class PosLLHRecord:
    sender_id: int
    tow_ms: int
    latitude_deg: float
    longitude_deg: float
    height_m: float
    horizontal_accuracy_mm: int
    vertical_accuracy_mm: int
    satellites: int
    flags: int


@dataclass
class VelNEDRecord:
    sender_id: int
    tow_ms: int
    north_mps: float
    east_mps: float
    down_mps: float
    horizontal_accuracy_mmps: int
    vertical_accuracy_mmps: int
    satellites: int
    flags: int


@dataclass
class Stats:
    total_frames: int = 0
    valid_frames: int = 0
    crc_errors: int = 0
    gps_time_messages: int = 0
    pos_llh_messages: int = 0
    vel_ned_messages: int = 0
    valid_positions: int = 0


@dataclass
class SBPFrame:
    message_type: int
    sender_id: int
    payload: bytes


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--input",
        required=True,
        help="Input path (.sbp, .bin, serial://..., or /dev/tty...).",
    )
    parser.add_argument(
        "--decode-time",
        action="store_true",
        help="Print decoded SBP GPS_TIME records.",
    )
    parser.add_argument(
        "--decode-pos",
        action="store_true",
        help="Print decoded SBP POS_LLH records.",
    )
    parser.add_argument(
        "--decode-vel",
        action="store_true",
        help="Print decoded SBP VEL_NED records.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=-1,
        help="Stop after decoding supported SBP records (default: no limit).",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress per-record printing and show only the summary.",
    )
    return parser.parse_args()


def parse_serial_path(path: str) -> tuple[str, int]:
    default_baud = 115200
    if path.startswith("serial://"):
        raw = path[len("serial://") :]
    else:
        raw = path
    if "?baud=" in raw:
        device, baud_text = raw.split("?baud=", 1)
        return device, int(baud_text)
    return raw, default_baud


def configure_serial(fd: int, baud: int) -> None:
    if os.name == "nt":
        raise RuntimeError("serial input is not supported on this platform")
    baud_map = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
    }
    if baud not in baud_map:
        raise ValueError(f"unsupported serial baud rate: {baud}")
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = baud_map[baud]
    attrs[5] = baud_map[baud]
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


class InputSource:
    def __init__(self, path: str) -> None:
        self.path = path
        self.handle = None
        self.fd: int | None = None

    def open(self) -> bool:
        if self.path.startswith("serial://") or self.path.startswith("/dev/tty"):
            device, baud = parse_serial_path(self.path)
            fd = os.open(device, os.O_RDONLY | os.O_NOCTTY)
            configure_serial(fd, baud)
            self.fd = fd
            return True
        self.handle = open(self.path, "rb")
        return True

    def close(self) -> None:
        if self.handle is not None:
            self.handle.close()
            self.handle = None
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def read(self, size: int = 4096) -> bytes:
        if self.handle is not None:
            return self.handle.read(size)
        if self.fd is not None:
            try:
                return os.read(self.fd, size)
            except OSError as exc:
                if exc.errno == errno.EIO:
                    return b""
                raise
        return b""


def compute_crc(frame_without_preamble: bytes) -> int:
    return binascii.crc_hqx(frame_without_preamble, 0) & 0xFFFF


def iter_frames(buffer: bytearray, stats: Stats) -> list[SBPFrame]:
    frames: list[SBPFrame] = []
    while True:
        if not buffer:
            break
        if buffer[0] != SBP_PREAMBLE:
            preamble_index = buffer.find(bytes([SBP_PREAMBLE]))
            if preamble_index < 0:
                buffer.clear()
                break
            del buffer[:preamble_index]
        if len(buffer) < 8:
            break
        payload_length = buffer[5]
        frame_length = 1 + 2 + 2 + 1 + payload_length + 2
        if len(buffer) < frame_length:
            break
        frame = bytes(buffer[:frame_length])
        del buffer[:frame_length]

        stats.total_frames += 1
        crc_expected = struct.unpack_from("<H", frame, frame_length - 2)[0]
        crc_actual = compute_crc(frame[1:-2])
        if crc_actual != crc_expected:
            stats.crc_errors += 1
            continue

        message_type, sender_id, payload_length = struct.unpack_from("<HHB", frame, 1)
        payload = frame[6:-2]
        if len(payload) != payload_length:
            stats.crc_errors += 1
            continue

        stats.valid_frames += 1
        frames.append(SBPFrame(message_type=message_type, sender_id=sender_id, payload=payload))
    return frames


def decode_frame(frame: SBPFrame, stats: Stats) -> tuple[str, object] | None:
    if frame.message_type == SBP_MSG_GPS_TIME and len(frame.payload) >= 11:
        tow_ms, week, ns_residual, flags = struct.unpack_from("<IHiB", frame.payload, 0)
        stats.gps_time_messages += 1
        return (
            "GPS_TIME",
            GPSTimeRecord(
                sender_id=frame.sender_id,
                tow_ms=tow_ms,
                week=week,
                ns_residual=ns_residual,
                flags=flags,
            ),
        )
    if frame.message_type == SBP_MSG_POS_LLH and len(frame.payload) >= 34:
        tow_ms, latitude_deg, longitude_deg, height_m, h_acc_mm, v_acc_mm, satellites, flags = struct.unpack_from(
            "<IdddHHBB",
            frame.payload,
            0,
        )
        stats.pos_llh_messages += 1
        if flags != 0:
            stats.valid_positions += 1
        return (
            "POS_LLH",
            PosLLHRecord(
                sender_id=frame.sender_id,
                tow_ms=tow_ms,
                latitude_deg=latitude_deg,
                longitude_deg=longitude_deg,
                height_m=height_m,
                horizontal_accuracy_mm=h_acc_mm,
                vertical_accuracy_mm=v_acc_mm,
                satellites=satellites,
                flags=flags,
            ),
        )
    if frame.message_type == SBP_MSG_VEL_NED and len(frame.payload) >= 22:
        tow_ms, north_mmps, east_mmps, down_mmps, h_acc_mmps, v_acc_mmps, satellites, flags = struct.unpack_from(
            "<IiiiHHBB",
            frame.payload,
            0,
        )
        stats.vel_ned_messages += 1
        return (
            "VEL_NED",
            VelNEDRecord(
                sender_id=frame.sender_id,
                tow_ms=tow_ms,
                north_mps=north_mmps / 1000.0,
                east_mps=east_mmps / 1000.0,
                down_mps=down_mmps / 1000.0,
                horizontal_accuracy_mmps=h_acc_mmps,
                vertical_accuracy_mmps=v_acc_mmps,
                satellites=satellites,
                flags=flags,
            ),
        )
    return None


def format_record(kind: str, record: object) -> str:
    if kind == "GPS_TIME":
        assert isinstance(record, GPSTimeRecord)
        return (
            f"gps_time: sender={record.sender_id} week={record.week} tow_ms={record.tow_ms} "
            f"ns_residual={record.ns_residual} flags=0x{record.flags:02X}"
        )
    if kind == "POS_LLH":
        assert isinstance(record, PosLLHRecord)
        return (
            f"pos_llh: sender={record.sender_id} tow_ms={record.tow_ms} "
            f"lat={record.latitude_deg:.7f} lon={record.longitude_deg:.7f} "
            f"height={record.height_m:.3f}m h_acc={record.horizontal_accuracy_mm}mm "
            f"v_acc={record.vertical_accuracy_mm}mm sats={record.satellites} flags=0x{record.flags:02X}"
        )
    assert isinstance(record, VelNEDRecord)
    return (
        f"vel_ned: sender={record.sender_id} tow_ms={record.tow_ms} "
        f"n={record.north_mps:.3f} e={record.east_mps:.3f} d={record.down_mps:.3f}mps "
        f"h_acc={record.horizontal_accuracy_mmps}mmps v_acc={record.vertical_accuracy_mmps}mmps "
        f"sats={record.satellites} flags=0x{record.flags:02X}"
    )


def should_print(kind: str, args: argparse.Namespace) -> bool:
    if args.quiet:
        return False
    if not args.decode_time and not args.decode_pos and not args.decode_vel:
        return True
    return (
        (kind == "GPS_TIME" and args.decode_time)
        or (kind == "POS_LLH" and args.decode_pos)
        or (kind == "VEL_NED" and args.decode_vel)
    )


def main() -> int:
    args = parse_args()
    if args.limit == 0:
        raise SystemExit("--limit must be positive or -1")

    source = InputSource(args.input)
    if not source.open():
        raise SystemExit(f"failed to open input: {args.input}")

    stats = Stats()
    decoded = 0
    buffer = bytearray()
    try:
        while args.limit < 0 or decoded < args.limit:
            chunk = source.read()
            if not chunk:
                break
            buffer.extend(chunk)
            for frame in iter_frames(buffer, stats):
                parsed = decode_frame(frame, stats)
                if parsed is None:
                    continue
                kind, record = parsed
                decoded += 1
                if should_print(kind, args):
                    print(format_record(kind, record))
                if args.limit >= 0 and decoded >= args.limit:
                    break
    finally:
        source.close()

    print(
        "summary: "
        f"frames={stats.total_frames} valid={stats.valid_frames} "
        f"gps_time={stats.gps_time_messages} pos_llh={stats.pos_llh_messages} "
        f"vel_ned={stats.vel_ned_messages} valid_positions={stats.valid_positions} "
        f"crc_errors={stats.crc_errors}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
