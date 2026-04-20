#!/usr/bin/env python3
"""Inspect minimal BINEX logs or serial streams."""

from __future__ import annotations

import argparse
import binascii
import errno
import os
from dataclasses import dataclass

if os.name != "nt":
    import termios


BINEX_SYNC_BIG_ENDIAN_REGULAR = 0xE2
BINEX_RECORD_METADATA = 0x00
BINEX_RECORD_NAV = 0x01
BINEX_RECORD_PROTO = 0x7F
DEFAULT_SERIAL_BAUD = 115200


@dataclass
class Stats:
    total_frames: int = 0
    valid_frames: int = 0
    checksum_errors: int = 0
    metadata_records: int = 0
    navigation_records: int = 0
    prototyping_records: int = 0


@dataclass
class BINEXFrame:
    record_id: int
    payload: bytes


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--input",
        required=True,
        help="Input path (.bnx, .binex, serial://..., or /dev/tty...).",
    )
    parser.add_argument(
        "--decode-metadata",
        action="store_true",
        help="Print decoded BINEX 0x00-* metadata record summaries.",
    )
    parser.add_argument(
        "--decode-nav",
        action="store_true",
        help="Print decoded BINEX 0x01-* navigation record summaries.",
    )
    parser.add_argument(
        "--decode-proto",
        action="store_true",
        help="Print decoded BINEX 0x7F-* prototyping record summaries.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=-1,
        help="Stop after decoding supported records (default: no limit).",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress per-record printing and show only the summary.",
    )
    return parser.parse_args()


def parse_serial_path(path: str) -> tuple[str, int]:
    raw = path[len("serial://") :] if path.startswith("serial://") else path
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
                if exc.errno == errno.EIO:
                    return b""
                raise
        return b""


def decode_ubnxi(buffer: bytes, offset: int) -> tuple[int, int] | None:
    value = 0
    consumed = 0
    while offset + consumed < len(buffer):
        current = buffer[offset + consumed]
        if consumed < 3:
            value = (value << 7) | (current & 0x7F)
            consumed += 1
            if (current & 0x80) == 0:
                return value, consumed
        else:
            value = (value << 8) | current
            return value, consumed + 1
    return None


def checksum8(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value & 0xFF


def iter_frames(buffer: bytearray, stats: Stats) -> list[BINEXFrame]:
    frames: list[BINEXFrame] = []
    while True:
        start = buffer.find(bytes((BINEX_SYNC_BIG_ENDIAN_REGULAR,)))
        if start < 0:
            buffer.clear()
            break
        if start > 0:
            del buffer[:start]
        if len(buffer) < 4:
            break

        record_id = buffer[1]
        if record_id not in (BINEX_RECORD_METADATA, BINEX_RECORD_NAV, BINEX_RECORD_PROTO):
            del buffer[0]
            continue

        length_result = decode_ubnxi(bytes(buffer), 2)
        if length_result is None:
            break
        payload_length, header_length = length_result
        frame_without_crc = 2 + header_length + payload_length
        if frame_without_crc - 1 > 4096:
            del buffer[0]
            stats.checksum_errors += 1
            continue
        checksum_length = 1 if frame_without_crc - 1 < 128 else 2
        total_length = frame_without_crc + checksum_length
        if len(buffer) < total_length:
            break

        frame = bytes(buffer[:total_length])
        del buffer[:total_length]
        stats.total_frames += 1

        payload = frame[2 + header_length : frame_without_crc]
        checksum_actual = frame[frame_without_crc:]
        checksum_input = frame[1:frame_without_crc]
        if checksum_length == 1:
            checksum_expected = bytes((checksum8(checksum_input),))
        else:
            checksum_expected = binascii.crc_hqx(checksum_input, 0).to_bytes(2, byteorder="big")
        if checksum_actual != checksum_expected:
            stats.checksum_errors += 1
            continue

        stats.valid_frames += 1
        frames.append(BINEXFrame(record_id=record_id, payload=payload))
    return frames


def decode_frame(frame: BINEXFrame, stats: Stats) -> tuple[str, str] | None:
    subrecord = frame.payload[0] if frame.payload else 0
    payload_bytes = max(0, len(frame.payload) - 1)
    preview = frame.payload[1:9].hex() if len(frame.payload) > 1 else ""
    if frame.record_id == BINEX_RECORD_METADATA:
        stats.metadata_records += 1
        return (
            "METADATA",
            f"metadata: subrecord=0x{subrecord:02X} payload_bytes={payload_bytes} preview={preview}",
        )
    if frame.record_id == BINEX_RECORD_NAV:
        stats.navigation_records += 1
        return (
            "NAV",
            f"nav: subrecord=0x{subrecord:02X} payload_bytes={payload_bytes} preview={preview}",
        )
    if frame.record_id == BINEX_RECORD_PROTO:
        stats.prototyping_records += 1
        return (
            "PROTO",
            f"proto: subrecord=0x{subrecord:02X} payload_bytes={payload_bytes} preview={preview}",
        )
    return None


def should_print(kind: str, args: argparse.Namespace) -> bool:
    if kind == "METADATA":
        return args.decode_metadata
    if kind == "NAV":
        return args.decode_nav
    if kind == "PROTO":
        return args.decode_proto
    return False


def main() -> int:
    args = parse_args()
    source = InputSource(args.input)
    stats = Stats()
    decoded_records = 0
    buffer = bytearray()
    try:
        source.open()
        while True:
            chunk = source.read()
            if not chunk:
                break
            buffer.extend(chunk)
            for frame in iter_frames(buffer, stats):
                decoded = decode_frame(frame, stats)
                if decoded is None:
                    continue
                decoded_records += 1
                kind, line = decoded
                if not args.quiet and should_print(kind, args):
                    print(line)
                if args.limit >= 0 and decoded_records >= args.limit:
                    print(
                        "summary: frames={} valid={} metadata={} nav={} proto={} checksum_errors={}".format(
                            stats.total_frames,
                            stats.valid_frames,
                            stats.metadata_records,
                            stats.navigation_records,
                            stats.prototyping_records,
                            stats.checksum_errors,
                        )
                    )
                    return 0
    finally:
        source.close()

    print(
        "summary: frames={} valid={} metadata={} nav={} proto={} checksum_errors={}".format(
            stats.total_frames,
            stats.valid_frames,
            stats.metadata_records,
            stats.navigation_records,
            stats.prototyping_records,
            stats.checksum_errors,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
