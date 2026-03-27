#!/usr/bin/env python3
"""Inspect Trimble GSOF GENOUT packets from file or serial input."""

from __future__ import annotations

import argparse
import math
import os
import struct
from dataclasses import dataclass

if os.name != "nt":
    import termios


GSOF_STX = 0x02
GSOF_ETX = 0x03
GSOF_PACKET_TYPE_GENOUT = 0x40
GSOF_RECORD_TIME = 1
GSOF_RECORD_LLH = 2
GSOF_RECORD_VELOCITY = 8
DEFAULT_SERIAL_BAUD = 115200


@dataclass
class TimeRecord:
    gps_week: int
    gps_tow_ms: int
    sv_used: int
    position_flags_1: int
    position_flags_2: int
    init_count: int


@dataclass
class LlhRecord:
    latitude_deg: float
    longitude_deg: float
    height_m: float


@dataclass
class VelocityRecord:
    flags: int
    horizontal_speed_mps: float
    heading_deg: float
    vertical_speed_mps: float
    local_heading_deg: float | None = None


@dataclass
class Stats:
    packets: int = 0
    valid_packets: int = 0
    checksum_errors: int = 0
    time_records: int = 0
    llh_records: int = 0
    velocity_records: int = 0
    valid_positions: int = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--input",
        required=True,
        help="Input path (.dat, .bin, serial://..., or /dev/tty...).",
    )
    parser.add_argument(
        "--decode-time",
        action="store_true",
        help="Print decoded GSOF time (Type 1) records.",
    )
    parser.add_argument(
        "--decode-llh",
        action="store_true",
        help="Print decoded GSOF lat/lon/height (Type 2) records.",
    )
    parser.add_argument(
        "--decode-vel",
        action="store_true",
        help="Print decoded GSOF velocity (Type 8) records.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=-1,
        help="Stop after decoding records (default: no limit).",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress per-record printing and show only the summary.",
    )
    return parser.parse_args()


def parse_serial_path(path: str) -> tuple[str, int]:
    raw = path[len("serial://"):] if path.startswith("serial://") else path
    if "?baud=" in raw:
        device, baud_text = raw.split("?baud=", 1)
        return device, int(baud_text)
    return raw, DEFAULT_SERIAL_BAUD


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
                if getattr(exc, "errno", None) == 5:
                    return b""
                raise
        return b""


def checksum_matches(frame: bytes) -> bool:
    if len(frame) < 6:
        return False
    length = frame[3]
    if len(frame) != length + 6:
        return False
    checksum = sum(frame[1 : 4 + length]) & 0xFF
    return checksum == frame[4 + length]


def parse_time_record(payload: bytes) -> TimeRecord:
    gps_tow_ms, gps_week, sv_used, flags1, flags2, init_count = struct.unpack(">IHBBBB", payload)
    return TimeRecord(
        gps_week=gps_week,
        gps_tow_ms=gps_tow_ms,
        sv_used=sv_used,
        position_flags_1=flags1,
        position_flags_2=flags2,
        init_count=init_count,
    )


def parse_llh_record(payload: bytes) -> LlhRecord:
    latitude_rad, longitude_rad, height_m = struct.unpack(">ddd", payload)
    return LlhRecord(
        latitude_deg=math.degrees(latitude_rad),
        longitude_deg=math.degrees(longitude_rad),
        height_m=height_m,
    )


def parse_velocity_record(payload: bytes) -> VelocityRecord:
    if len(payload) not in {13, 17}:
        raise ValueError(f"unsupported GSOF velocity payload length: {len(payload)}")
    flags = payload[0]
    horizontal_speed_mps, heading_rad, vertical_speed_mps = struct.unpack(">fff", payload[1:13])
    local_heading_deg = None
    if len(payload) >= 17:
        (local_heading_rad,) = struct.unpack(">f", payload[13:17])
        local_heading_deg = math.degrees(local_heading_rad)
    return VelocityRecord(
        flags=flags,
        horizontal_speed_mps=horizontal_speed_mps,
        heading_deg=math.degrees(heading_rad),
        vertical_speed_mps=vertical_speed_mps,
        local_heading_deg=local_heading_deg,
    )


class GsofAssembler:
    def __init__(self) -> None:
        self.transmission_number: int | None = None
        self.max_page_index: int | None = None
        self.pages: dict[int, bytes] = {}

    def push(self, transmission_number: int, page_index: int, max_page_index: int, payload: bytes) -> list[bytes]:
        if (
            self.transmission_number is None or
            transmission_number != self.transmission_number or
            self.max_page_index is None or
            max_page_index != self.max_page_index
        ):
            self.transmission_number = transmission_number
            self.max_page_index = max_page_index
            self.pages = {}
        self.pages[page_index] = payload
        if any(index not in self.pages for index in range(max_page_index + 1)):
            return []
        records = [self.pages[index] for index in range(max_page_index + 1)]
        self.transmission_number = None
        self.max_page_index = None
        self.pages = {}
        return [b"".join(records)]


def extract_frames(source: InputSource, stats: Stats) -> list[bytes]:
    buffer = bytearray()
    frames: list[bytes] = []
    while True:
        chunk = source.read()
        if not chunk:
            break
        buffer.extend(chunk)
        while True:
            try:
                start = buffer.index(GSOF_STX)
            except ValueError:
                buffer.clear()
                break
            if start > 0:
                del buffer[:start]
            if len(buffer) < 6:
                break
            length = buffer[3]
            frame_size = length + 6
            if len(buffer) < frame_size:
                break
            frame = bytes(buffer[:frame_size])
            del buffer[:frame_size]
            if frame[-1] != GSOF_ETX:
                continue
            stats.packets += 1
            if not checksum_matches(frame):
                stats.checksum_errors += 1
                continue
            stats.valid_packets += 1
            if frame[2] == GSOF_PACKET_TYPE_GENOUT:
                frames.append(frame)
    return frames


def decode_records(payload: bytes, stats: Stats) -> list[tuple[str, object]]:
    decoded: list[tuple[str, object]] = []
    offset = 0
    while offset + 2 <= len(payload):
        record_type = payload[offset]
        record_length = payload[offset + 1]
        offset += 2
        if offset + record_length > len(payload):
            break
        record_payload = payload[offset : offset + record_length]
        offset += record_length

        if record_type == GSOF_RECORD_TIME and record_length == 10:
            stats.time_records += 1
            decoded.append(("TIME", parse_time_record(record_payload)))
        elif record_type == GSOF_RECORD_LLH and record_length == 24:
            stats.llh_records += 1
            llh = parse_llh_record(record_payload)
            if math.isfinite(llh.latitude_deg) and math.isfinite(llh.longitude_deg):
                stats.valid_positions += 1
            decoded.append(("LLH", llh))
        elif record_type == GSOF_RECORD_VELOCITY and record_length in {13, 17}:
            stats.velocity_records += 1
            decoded.append(("VEL", parse_velocity_record(record_payload)))
    return decoded


def print_record(kind: str, record: object) -> None:
    if kind == "TIME":
        time_record = record
        assert isinstance(time_record, TimeRecord)
        tow_seconds = time_record.gps_tow_ms / 1000.0
        print(
            "time:"
            f" week={time_record.gps_week}"
            f" tow={tow_seconds:.3f}"
            f" svs={time_record.sv_used}"
            f" flags1=0x{time_record.position_flags_1:02X}"
            f" flags2=0x{time_record.position_flags_2:02X}"
            f" init={time_record.init_count}"
        )
        return
    if kind == "LLH":
        llh = record
        assert isinstance(llh, LlhRecord)
        print(
            "llh:"
            f" lat={llh.latitude_deg:.7f}"
            f" lon={llh.longitude_deg:.7f}"
            f" height={llh.height_m:.3f}m"
        )
        return
    if kind == "VEL":
        velocity = record
        assert isinstance(velocity, VelocityRecord)
        line = (
            "velocity:"
            f" flags=0x{velocity.flags:02X}"
            f" horiz={velocity.horizontal_speed_mps:.3f}mps"
            f" heading={velocity.heading_deg:.2f}deg"
            f" vertical={velocity.vertical_speed_mps:.3f}mps"
        )
        if velocity.local_heading_deg is not None:
            line += f" local_heading={velocity.local_heading_deg:.2f}deg"
        print(line)


def main() -> int:
    args = parse_args()
    decode_any = args.decode_time or args.decode_llh or args.decode_vel
    if not decode_any:
        args.decode_time = True
        args.decode_llh = True
        args.decode_vel = True

    source = InputSource(args.input)
    if not source.open():
        return 1
    try:
        stats = Stats()
        assembler = GsofAssembler()
        decoded_count = 0
        for frame in extract_frames(source, stats):
            data_bytes = frame[4 : 4 + frame[3]]
            if len(data_bytes) < 3:
                continue
            transmission_number = data_bytes[0]
            page_index = data_bytes[1]
            max_page_index = data_bytes[2]
            for assembled in assembler.push(
                transmission_number,
                page_index,
                max_page_index,
                data_bytes[3:],
            ):
                for kind, record in decode_records(assembled, stats):
                    if not args.quiet:
                        if kind == "TIME" and args.decode_time:
                            print_record(kind, record)
                        elif kind == "LLH" and args.decode_llh:
                            print_record(kind, record)
                        elif kind == "VEL" and args.decode_vel:
                            print_record(kind, record)
                    decoded_count += 1
                    if 0 <= args.limit <= decoded_count:
                        break
                if 0 <= args.limit <= decoded_count:
                    break
            if 0 <= args.limit <= decoded_count:
                break
    finally:
        source.close()

    print(
        "summary:"
        f" packets={stats.packets}"
        f" valid={stats.valid_packets}"
        f" time={stats.time_records}"
        f" llh={stats.llh_records}"
        f" velocity={stats.velocity_records}"
        f" valid_positions={stats.valid_positions}"
        f" checksum_errors={stats.checksum_errors}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
