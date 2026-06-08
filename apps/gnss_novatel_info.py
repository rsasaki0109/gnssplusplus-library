#!/usr/bin/env python3
"""Inspect NovAtel ASCII BESTPOS/BESTVEL logs or serial streams."""

from __future__ import annotations

import argparse
import binascii
import os
from dataclasses import dataclass
import struct
import zlib

if os.name != "nt":
    import termios


@dataclass
class BestPosRecord:
    gps_week: int
    gps_tow: float
    solution_status: str
    position_type: str
    latitude_deg: float
    longitude_deg: float
    height_m: float
    num_satellites: int


@dataclass
class BestVelRecord:
    gps_week: int
    gps_tow: float
    solution_status: str
    velocity_type: str
    horizontal_speed_mps: float
    track_deg: float


@dataclass
class Stats:
    total_records: int = 0
    valid_records: int = 0
    checksum_errors: int = 0
    bestpos_records: int = 0
    bestvel_records: int = 0
    valid_positions: int = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--input",
        required=True,
        help="Input path (.log, .txt, serial://..., or /dev/tty...).",
    )
    parser.add_argument(
        "--decode-bestpos",
        action="store_true",
        help="Print decoded BESTPOS records.",
    )
    parser.add_argument(
        "--decode-bestvel",
        action="store_true",
        help="Print decoded BESTVEL records.",
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
    default_baud = 115200
    if path.startswith("serial://"):
        raw = path[len("serial://"):]
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
                if getattr(exc, "errno", None) == 5:
                    return b""
                raise
        return b""


def valid_crc32(content: str, checksum_text: str) -> bool:
    try:
        expected = int(checksum_text[:8], 16)
    except ValueError:
        return False
    computed = binascii.crc32(content.encode("ascii")) & 0xFFFFFFFF
    return computed == expected


def solution_status_name(value: int) -> str:
    names = {
        0: "SOL_COMPUTED",
        1: "INSUFFICIENT_OBS",
        2: "NO_CONVERGENCE",
        3: "SINGULARITY",
        4: "COV_TRACE",
        5: "TEST_DIST",
        6: "COLD_START",
        7: "V_H_LIMIT",
        8: "VARIANCE",
        9: "RESIDUALS",
        13: "INTEGRITY_WARNING",
    }
    return names.get(value, str(value))


def position_velocity_type_name(value: int) -> str:
    names = {
        0: "NONE",
        16: "SINGLE",
        17: "PSRDIFF",
        32: "L1_FLOAT",
        33: "IONOFREE_FLOAT",
        34: "NARROW_FLOAT",
        48: "L1_INT",
        49: "WIDE_INT",
        50: "NARROW_INT",
        68: "PPP",
        69: "PPP_CONVERGING",
        70: "PPP_BASIC",
        74: "DOPPLER_VELOCITY",
    }
    return names.get(value, str(value))


def parse_record(line: str, stats: Stats) -> tuple[str, object] | None:
    text = line.strip()
    if not text.startswith("#") or "*" not in text:
        return None
    stats.total_records += 1
    content, checksum_text = text[1:].split("*", 1)
    if not valid_crc32(content, checksum_text):
        stats.checksum_errors += 1
        return None
    if ";" not in content:
        return None
    header_text, body_text = content.split(";", 1)
    header_fields = header_text.split(",")
    body_fields = body_text.split(",")
    if len(header_fields) < 7 or len(body_fields) < 2:
        return None

    message_name = header_fields[0]
    try:
        gps_week = int(header_fields[5])
        gps_tow = float(header_fields[6])
    except ValueError:
        return None

    stats.valid_records += 1

    if message_name == "BESTPOSA":
        if len(body_fields) < 15:
            return None
        stats.bestpos_records += 1
        record = BestPosRecord(
            gps_week=gps_week,
            gps_tow=gps_tow,
            solution_status=body_fields[0],
            position_type=body_fields[1],
            latitude_deg=float(body_fields[2]),
            longitude_deg=float(body_fields[3]),
            height_m=float(body_fields[4]),
            num_satellites=int(body_fields[13] or 0),
        )
        if record.solution_status == "SOL_COMPUTED":
            stats.valid_positions += 1
        return "BESTPOS", record

    if message_name == "BESTVELA":
        if len(body_fields) < 6:
            return None
        stats.bestvel_records += 1
        record = BestVelRecord(
            gps_week=gps_week,
            gps_tow=gps_tow,
            solution_status=body_fields[0],
            velocity_type=body_fields[1],
            horizontal_speed_mps=float(body_fields[4]),
            track_deg=float(body_fields[5]),
        )
        return "BESTVEL", record
    return None


def parse_binary_record(frame: bytes, stats: Stats) -> tuple[str, object] | None:
    if len(frame) < 32:
        return None
    stats.total_records += 1
    expected_crc = struct.unpack_from("<I", frame, len(frame) - 4)[0]
    computed_crc = zlib.crc32(frame[:-4]) & 0xFFFFFFFF
    if computed_crc != expected_crc:
        stats.checksum_errors += 1
        return None

    header_length = frame[3]
    message_id = struct.unpack_from("<H", frame, 4)[0]
    message_length = struct.unpack_from("<H", frame, 8)[0]
    gps_week = struct.unpack_from("<H", frame, 14)[0]
    gps_tow = struct.unpack_from("<I", frame, 16)[0] / 1000.0
    body = frame[header_length:header_length + message_length]
    stats.valid_records += 1

    if message_id == 42 and len(body) >= 72:
        stats.bestpos_records += 1
        sol_status = struct.unpack_from("<I", body, 0)[0]
        pos_type = struct.unpack_from("<I", body, 4)[0]
        latitude_deg = struct.unpack_from("<d", body, 8)[0]
        longitude_deg = struct.unpack_from("<d", body, 16)[0]
        height_m = struct.unpack_from("<d", body, 24)[0]
        num_satellites = body[65]
        record = BestPosRecord(
            gps_week=gps_week,
            gps_tow=gps_tow,
            solution_status=solution_status_name(sol_status),
            position_type=position_velocity_type_name(pos_type),
            latitude_deg=latitude_deg,
            longitude_deg=longitude_deg,
            height_m=height_m,
            num_satellites=num_satellites,
        )
        if sol_status == 0:
            stats.valid_positions += 1
        return "BESTPOS", record

    if message_id == 99 and len(body) >= 44:
        stats.bestvel_records += 1
        sol_status = struct.unpack_from("<I", body, 0)[0]
        vel_type = struct.unpack_from("<I", body, 4)[0]
        horizontal_speed_mps = struct.unpack_from("<d", body, 16)[0]
        track_deg = struct.unpack_from("<d", body, 24)[0]
        record = BestVelRecord(
            gps_week=gps_week,
            gps_tow=gps_tow,
            solution_status=solution_status_name(sol_status),
            velocity_type=position_velocity_type_name(vel_type),
            horizontal_speed_mps=horizontal_speed_mps,
            track_deg=track_deg,
        )
        return "BESTVEL", record
    return None


def format_record(kind: str, record: object) -> str:
    if kind == "BESTPOS":
        assert isinstance(record, BestPosRecord)
        return (
            f"bestpos: week={record.gps_week} tow={record.gps_tow:.3f} "
            f"status={record.solution_status} type={record.position_type} "
            f"lat={record.latitude_deg:.7f} lon={record.longitude_deg:.7f} "
            f"hgt={record.height_m:.3f} sats={record.num_satellites}"
        )
    assert isinstance(record, BestVelRecord)
    return (
        f"bestvel: week={record.gps_week} tow={record.gps_tow:.3f} "
        f"status={record.solution_status} type={record.velocity_type} "
        f"speed_mps={record.horizontal_speed_mps:.3f} track_deg={record.track_deg:.3f}"
    )


def extract_next_record(buffer: bytes, stats: Stats) -> tuple[int, tuple[str, object] | None]:
    if not buffer:
        return 0, None

    ascii_pos = buffer.find(b"#")
    binary_pos = buffer.find(b"\xAA\x44\x12")
    candidates = [pos for pos in (ascii_pos, binary_pos) if pos >= 0]
    if not candidates:
        if len(buffer) <= 2:
            return 0, None
        return len(buffer) - 2, None

    next_pos = min(candidates)
    if next_pos > 0:
        return next_pos, None

    if buffer.startswith(b"#"):
        newline_pos = buffer.find(b"\n")
        if newline_pos < 0:
            return 0, None
        consumed = newline_pos + 1
        line = buffer[:newline_pos].decode("ascii", errors="ignore").strip()
        return consumed, parse_record(line, stats)

    if len(buffer) < 10:
        return 0, None
    header_length = buffer[3]
    if header_length < 28:
        return 1, None
    if len(buffer) < header_length:
        return 0, None
    message_length = struct.unpack_from("<H", buffer, 8)[0]
    frame_length = header_length + message_length + 4
    if len(buffer) < frame_length:
        return 0, None
    return frame_length, parse_binary_record(buffer[:frame_length], stats)


def main() -> int:
    args = parse_args()
    if args.limit == 0:
        raise SystemExit("--limit must be positive or -1")

    source = InputSource(args.input)
    if not source.open():
        raise SystemExit(f"failed to open input: {args.input}")

    stats = Stats()
    decoded = 0
    buffer = b""
    try:
        while args.limit < 0 or decoded < args.limit:
            chunk = source.read()
            if not chunk:
                while args.limit < 0 or decoded < args.limit:
                    consumed, parsed = extract_next_record(buffer, stats)
                    if consumed == 0:
                        break
                    buffer = buffer[consumed:]
                    if parsed is None:
                        continue
                    kind, record = parsed
                    decoded += 1
                    should_print = not args.quiet and (
                        (kind == "BESTPOS" and (args.decode_bestpos or (not args.decode_bestpos and not args.decode_bestvel)))
                        or (kind == "BESTVEL" and (args.decode_bestvel or (not args.decode_bestpos and not args.decode_bestvel)))
                    )
                    if should_print:
                        print(format_record(kind, record))
                break
            buffer += chunk
            while args.limit < 0 or decoded < args.limit:
                consumed, parsed = extract_next_record(buffer, stats)
                if consumed == 0:
                    break
                buffer = buffer[consumed:]
                if parsed is None:
                    continue
                kind, record = parsed
                decoded += 1
                should_print = not args.quiet and (
                    (kind == "BESTPOS" and (args.decode_bestpos or (not args.decode_bestpos and not args.decode_bestvel)))
                    or (kind == "BESTVEL" and (args.decode_bestvel or (not args.decode_bestpos and not args.decode_bestvel)))
                )
                if should_print:
                    print(format_record(kind, record))
    finally:
        source.close()

    print(
        "summary: "
        f"records={stats.total_records} valid={stats.valid_records} "
        f"bestpos={stats.bestpos_records} bestvel={stats.bestvel_records} "
        f"valid_positions={stats.valid_positions} checksum_errors={stats.checksum_errors}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
