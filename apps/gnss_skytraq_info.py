#!/usr/bin/env python3
"""Inspect SkyTraq binary logs or serial streams."""

from __future__ import annotations

import argparse
import errno
import os
from dataclasses import dataclass

if os.name != "nt":
    import termios


STQ_SYNC1 = 0xA0
STQ_SYNC2 = 0xA1
STQ_MSG_EPOCH = 0xDC
STQ_MSG_RAW = 0xDD
STQ_MSG_RAWX = 0xE5
STQ_MSG_ACK = 0x83
STQ_MSG_NACK = 0x84
DEFAULT_SERIAL_BAUD = 115200


@dataclass
class EpochRecord:
    iod: int
    week: int
    tow_s: float


@dataclass
class RawRecord:
    iod: int
    nsat: int


@dataclass
class RawXRecord:
    version: int
    iod: int
    week: int
    tow_s: float
    period_s: float
    nsat: int


@dataclass
class AckRecord:
    message_id: int


@dataclass
class Stats:
    total_frames: int = 0
    valid_frames: int = 0
    checksum_errors: int = 0
    epoch_messages: int = 0
    raw_messages: int = 0
    rawx_messages: int = 0
    ack_messages: int = 0
    nack_messages: int = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--input",
        required=True,
        help="Input path (.bin, serial://..., or /dev/tty...).",
    )
    parser.add_argument(
        "--decode-epoch",
        action="store_true",
        help="Print decoded SkyTraq epoch (0xDC) messages.",
    )
    parser.add_argument(
        "--decode-raw",
        action="store_true",
        help="Print decoded SkyTraq raw/rawx (0xDD/0xE5) message summaries.",
    )
    parser.add_argument(
        "--decode-ack",
        action="store_true",
        help="Print decoded SkyTraq ACK/NACK (0x83/0x84) messages.",
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


@dataclass
class SkyTraqFrame:
    message_id: int
    payload: bytes


def compute_checksum(payload: bytes) -> int:
    checksum = 0
    for value in payload:
        checksum ^= value
    return checksum & 0xFF


def iter_frames(buffer: bytearray, stats: Stats) -> list[SkyTraqFrame]:
    frames: list[SkyTraqFrame] = []
    while True:
        start = buffer.find(bytes((STQ_SYNC1, STQ_SYNC2)))
        if start < 0:
            if len(buffer) > 1:
                del buffer[:-1]
            break
        if start > 0:
            del buffer[:start]
        if len(buffer) < 7:
            break
        payload_length = int.from_bytes(buffer[2:4], byteorder="big", signed=False)
        frame_length = payload_length + 7
        if len(buffer) < frame_length:
            break
        frame = bytes(buffer[:frame_length])
        del buffer[:frame_length]

        stats.total_frames += 1
        if frame[-2:] != b"\r\n":
            stats.checksum_errors += 1
            continue

        payload = frame[4 : 4 + payload_length]
        if not payload:
            stats.checksum_errors += 1
            continue
        if compute_checksum(payload) != frame[4 + payload_length]:
            stats.checksum_errors += 1
            continue

        stats.valid_frames += 1
        frames.append(SkyTraqFrame(message_id=payload[0], payload=payload))
    return frames


def decode_frame(frame: SkyTraqFrame, stats: Stats) -> tuple[str, object] | None:
    payload = frame.payload
    if frame.message_id == STQ_MSG_EPOCH and len(payload) >= 8:
        stats.epoch_messages += 1
        return (
            "EPOCH",
            EpochRecord(
                iod=payload[1],
                week=int.from_bytes(payload[2:4], byteorder="big", signed=False),
                tow_s=int.from_bytes(payload[4:8], byteorder="big", signed=False) / 1000.0,
            ),
        )
    if frame.message_id == STQ_MSG_RAW and len(payload) >= 3:
        stats.raw_messages += 1
        return (
            "RAW",
            RawRecord(
                iod=payload[1],
                nsat=payload[2],
            ),
        )
    if frame.message_id == STQ_MSG_RAWX and len(payload) >= 14:
        stats.rawx_messages += 1
        return (
            "RAWX",
            RawXRecord(
                version=payload[1],
                iod=payload[2],
                week=int.from_bytes(payload[3:5], byteorder="big", signed=False),
                tow_s=int.from_bytes(payload[5:9], byteorder="big", signed=False) / 1000.0,
                period_s=int.from_bytes(payload[9:11], byteorder="big", signed=False) / 1000.0,
                nsat=payload[13],
            ),
        )
    if frame.message_id == STQ_MSG_ACK and len(payload) >= 2:
        stats.ack_messages += 1
        return ("ACK", AckRecord(message_id=payload[1]))
    if frame.message_id == STQ_MSG_NACK and len(payload) >= 2:
        stats.nack_messages += 1
        return ("NACK", AckRecord(message_id=payload[1]))
    return None


def should_print(kind: str, args: argparse.Namespace) -> bool:
    if args.quiet:
        return False
    if kind == "EPOCH":
        return args.decode_epoch or not (args.decode_epoch or args.decode_raw or args.decode_ack)
    if kind in {"RAW", "RAWX"}:
        return args.decode_raw or not (args.decode_epoch or args.decode_raw or args.decode_ack)
    if kind in {"ACK", "NACK"}:
        return args.decode_ack or not (args.decode_epoch or args.decode_raw or args.decode_ack)
    return False


def print_record(kind: str, record: object) -> None:
    if kind == "EPOCH":
        assert isinstance(record, EpochRecord)
        print(
            f"epoch: iod={record.iod} week={record.week} tow={record.tow_s:.3f}"
        )
        return
    if kind == "RAW":
        assert isinstance(record, RawRecord)
        print(f"raw: iod={record.iod} nsat={record.nsat}")
        return
    if kind == "RAWX":
        assert isinstance(record, RawXRecord)
        print(
            f"rawx: version={record.version} iod={record.iod} "
            f"week={record.week} tow={record.tow_s:.3f} period={record.period_s:.3f} nsat={record.nsat}"
        )
        return
    if kind == "ACK":
        assert isinstance(record, AckRecord)
        print(f"ack: msg=0x{record.message_id:02X}")
        return
    if kind == "NACK":
        assert isinstance(record, AckRecord)
        print(f"nack: msg=0x{record.message_id:02X}")


def main() -> int:
    args = parse_args()
    source = InputSource(args.input)
    stats = Stats()
    buffer = bytearray()
    decoded = 0

    try:
        source.open()
        while args.limit < 0 or decoded < args.limit:
            chunk = source.read()
            if not chunk:
                break
            buffer.extend(chunk)
            for frame in iter_frames(buffer, stats):
                decoded_frame = decode_frame(frame, stats)
                if decoded_frame is None:
                    continue
                kind, record = decoded_frame
                decoded += 1
                if should_print(kind, args):
                    print_record(kind, record)
                if args.limit >= 0 and decoded >= args.limit:
                    break
    finally:
        source.close()

    print(
        "summary: "
        f"frames={stats.total_frames} valid={stats.valid_frames} "
        f"epoch={stats.epoch_messages} raw={stats.raw_messages} rawx={stats.rawx_messages} "
        f"ack={stats.ack_messages} nack={stats.nack_messages} checksum_errors={stats.checksum_errors}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
