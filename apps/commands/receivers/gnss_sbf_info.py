#!/usr/bin/env python3
"""Inspect Septentrio SBF logs or serial streams."""

from __future__ import annotations

import argparse
import binascii
import errno
import math
import os
import struct
from dataclasses import dataclass

if os.name != "nt":
    import termios


SBF_SYNC = b"$@"
SBF_BLOCK_PVT_GEODETIC = 4007
SBF_BLOCK_LBAND_TRACKER_STATUS = 4201
SBF_BLOCK_P2PP_STATUS = 4238
SBF_CRC_SEED = 0


PVT_MODE_NAMES = {
    0: "NoPVT",
    1: "Standalone",
    2: "Differential",
    3: "FixedLocation",
    4: "RTKFixed",
    5: "RTKFloat",
    6: "SBAS",
    7: "MovingBaseFixed",
    8: "MovingBaseFloat",
    10: "PPP",
}

LBAND_STATUS_NAMES = {
    0: "Idle",
    1: "Search",
    2: "FrameSearch",
    3: "Locked",
}

LBAND_SOURCE_NAMES = {
    0: "Unknown",
    1: "Internal",
    2: "LBR",
    3: "NTRIP",
}

P2PP_STATUS_NAMES = {
    0: "Initializing",
    1: "WaitingForConnection",
    2: "Connected",
    3: "Disconnecting",
    4: "Error",
}

P2PP_ERROR_NAMES = {
    1: "No error",
    2: "Configuration",
    3: "Port Acquisition",
    4: "Port Lock",
    5: "Start Daemon",
    6: "Server Authentication",
    7: "Client Authentication",
    8: "Timeout on Activity",
    9: "Timeout on Negotiation",
    10: "Link Negotiation",
    255: "Unspecified",
}


@dataclass
class PVTGeodeticRecord:
    week: int
    tow_ms: int
    mode: int
    error: int
    latitude_deg: float
    longitude_deg: float
    height_m: float
    north_mps: float
    east_mps: float
    up_mps: float
    satellites: int
    horizontal_accuracy_m: float | None
    vertical_accuracy_m: float | None


@dataclass
class LBandTrackerRecord:
    week: int
    tow_ms: int
    trackers: int
    locked_trackers: int
    frequency_hz: int
    service_id: int
    cn0_dbhz: float
    status: int
    source: int


@dataclass
class P2PPStatusRecord:
    week: int
    tow_ms: int
    sessions: int
    connected_sessions: int
    session_id: int
    port: int
    status: int
    error_code: int


@dataclass
class Stats:
    total_frames: int = 0
    valid_frames: int = 0
    crc_errors: int = 0
    pvt_geodetic_messages: int = 0
    lband_tracker_messages: int = 0
    p2pp_status_messages: int = 0
    valid_positions: int = 0


@dataclass
class SBFFrame:
    block_number: int
    revision: int
    payload: bytes


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--input",
        required=True,
        help="Input path (.sbf, .bin, serial://..., or /dev/tty...).",
    )
    parser.add_argument(
        "--decode-pvt",
        action="store_true",
        help="Print decoded PVTGeodetic (4007) records.",
    )
    parser.add_argument(
        "--decode-lband",
        action="store_true",
        help="Print decoded LBandTrackerStatus (4201) records.",
    )
    parser.add_argument(
        "--decode-p2pp",
        action="store_true",
        help="Print decoded P2PPStatus (4238) records.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=-1,
        help="Stop after decoding supported SBF records (default: no limit).",
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


def compute_crc(data: bytes) -> int:
    return binascii.crc_hqx(data, SBF_CRC_SEED) & 0xFFFF


def do_not_use_u2(value: int) -> bool:
    return value == 0xFFFF


def do_not_use_float(value: float) -> bool:
    return value <= -2.0e10


def iter_frames(buffer: bytearray, stats: Stats) -> list[SBFFrame]:
    frames: list[SBFFrame] = []
    while True:
        if len(buffer) < 8:
            break
        if buffer[:2] != SBF_SYNC:
            sync_index = buffer.find(SBF_SYNC)
            if sync_index < 0:
                buffer.clear()
                break
            del buffer[:sync_index]
            if len(buffer) < 8:
                break

        stored_crc = struct.unpack_from("<H", buffer, 2)[0]
        raw_id = struct.unpack_from("<H", buffer, 4)[0]
        length = struct.unpack_from("<H", buffer, 6)[0]
        if length < 8 or (length % 4) != 0:
            del buffer[0]
            continue
        if len(buffer) < length:
            break

        frame_bytes = bytes(buffer[:length])
        del buffer[:length]
        stats.total_frames += 1

        computed_crc = compute_crc(frame_bytes[4:length])
        if computed_crc != stored_crc:
            stats.crc_errors += 1
            continue

        stats.valid_frames += 1
        frames.append(
            SBFFrame(
                block_number=raw_id & 0x1FFF,
                revision=(raw_id >> 13) & 0x7,
                payload=frame_bytes[8:length],
            )
        )
    return frames


def decode_pvt_geodetic(frame: SBFFrame, stats: Stats) -> tuple[str, object] | None:
    if frame.block_number != SBF_BLOCK_PVT_GEODETIC or len(frame.payload) < 87:
        return None
    stats.pvt_geodetic_messages += 1

    tow_ms, week = struct.unpack_from("<IH", frame.payload, 0)
    mode, error = struct.unpack_from("<BB", frame.payload, 6)
    latitude_rad, longitude_rad, height_m = struct.unpack_from("<ddd", frame.payload, 8)
    north_mps, east_mps, up_mps = struct.unpack_from("<fff", frame.payload, 36)
    satellites = frame.payload[66]
    horizontal_accuracy_raw = struct.unpack_from("<H", frame.payload, 82)[0]
    vertical_accuracy_raw = struct.unpack_from("<H", frame.payload, 84)[0]

    horizontal_accuracy_m = None if do_not_use_u2(horizontal_accuracy_raw) else horizontal_accuracy_raw / 100.0
    vertical_accuracy_m = None if do_not_use_u2(vertical_accuracy_raw) else vertical_accuracy_raw / 100.0

    if (mode & 0x0F) != 0 and error == 0 and not do_not_use_float(latitude_rad) and not do_not_use_float(longitude_rad):
        stats.valid_positions += 1

    return (
        "PVT_GEODETIC",
        PVTGeodeticRecord(
            week=week,
            tow_ms=tow_ms,
            mode=mode & 0x0F,
            error=error,
            latitude_deg=math.degrees(latitude_rad),
            longitude_deg=math.degrees(longitude_rad),
            height_m=height_m,
            north_mps=north_mps,
            east_mps=east_mps,
            up_mps=up_mps,
            satellites=satellites,
            horizontal_accuracy_m=horizontal_accuracy_m,
            vertical_accuracy_m=vertical_accuracy_m,
        ),
    )


def decode_lband_tracker(frame: SBFFrame, stats: Stats) -> tuple[str, object] | None:
    if frame.block_number != SBF_BLOCK_LBAND_TRACKER_STATUS or len(frame.payload) < 8:
        return None
    stats.lband_tracker_messages += 1

    tow_ms, week = struct.unpack_from("<IH", frame.payload, 0)
    tracker_count = frame.payload[6]
    subblock_length = frame.payload[7]
    if subblock_length < 23 or len(frame.payload) < 8 + tracker_count * subblock_length:
        return None

    first = frame.payload[8 : 8 + subblock_length]
    (
        frequency_hz,
        _baudrate,
        service_id,
        _freq_offset_hz,
        cn0_raw,
        _avg_power_raw,
        _agc_gain_db,
        _mode,
        status,
        _svid,
        _lock_time_s,
        source,
    ) = struct.unpack_from("<IHHfHhbBBBHB", first, 0)

    locked_trackers = 0
    offset = 8
    for _ in range(tracker_count):
        if offset + subblock_length > len(frame.payload):
            break
        current_status = frame.payload[offset + 18]
        if current_status == 3:
            locked_trackers += 1
        offset += subblock_length

    return (
        "LBAND_TRACKER",
        LBandTrackerRecord(
            week=week,
            tow_ms=tow_ms,
            trackers=tracker_count,
            locked_trackers=locked_trackers,
            frequency_hz=frequency_hz,
            service_id=service_id,
            cn0_dbhz=cn0_raw / 100.0,
            status=status,
            source=source,
        ),
    )


def decode_p2pp_status(frame: SBFFrame, stats: Stats) -> tuple[str, object] | None:
    if frame.block_number != SBF_BLOCK_P2PP_STATUS or len(frame.payload) < 8:
        return None
    stats.p2pp_status_messages += 1

    tow_ms, week = struct.unpack_from("<IH", frame.payload, 0)
    session_count = frame.payload[6]
    subblock_length = frame.payload[7]
    if subblock_length < 4 or len(frame.payload) < 8 + session_count * subblock_length:
        return None

    first = frame.payload[8 : 8 + subblock_length]
    session_id, port, packed_status, error_code = struct.unpack_from("<BBBB", first, 0)
    status = (packed_status >> 1) & 0x7F

    connected_sessions = 0
    offset = 8
    for _ in range(session_count):
        if offset + subblock_length > len(frame.payload):
            break
        current_status = (frame.payload[offset + 2] >> 1) & 0x7F
        if current_status == 2:
            connected_sessions += 1
        offset += subblock_length

    return (
        "P2PP_STATUS",
        P2PPStatusRecord(
            week=week,
            tow_ms=tow_ms,
            sessions=session_count,
            connected_sessions=connected_sessions,
            session_id=session_id,
            port=port,
            status=status,
            error_code=error_code,
        ),
    )


def decode_frame(frame: SBFFrame, stats: Stats) -> tuple[str, object] | None:
    return (
        decode_pvt_geodetic(frame, stats)
        or decode_lband_tracker(frame, stats)
        or decode_p2pp_status(frame, stats)
    )


def format_record(kind: str, record: object) -> str:
    if kind == "PVT_GEODETIC":
        assert isinstance(record, PVTGeodeticRecord)
        mode_name = PVT_MODE_NAMES.get(record.mode, str(record.mode))
        h_acc_text = "NA" if record.horizontal_accuracy_m is None else f"{record.horizontal_accuracy_m:.2f}m"
        v_acc_text = "NA" if record.vertical_accuracy_m is None else f"{record.vertical_accuracy_m:.2f}m"
        return (
            f"pvt_geodetic: week={record.week} tow_ms={record.tow_ms} mode={mode_name} "
            f"lat={record.latitude_deg:.7f} lon={record.longitude_deg:.7f} "
            f"height={record.height_m:.3f}m vn={record.north_mps:.3f} ve={record.east_mps:.3f} "
            f"vu={record.up_mps:.3f}mps sats={record.satellites} h_acc={h_acc_text} v_acc={v_acc_text}"
        )
    if kind == "LBAND_TRACKER":
        assert isinstance(record, LBandTrackerRecord)
        return (
            f"lband_tracker: week={record.week} tow_ms={record.tow_ms} "
            f"trackers={record.trackers} locked={record.locked_trackers} freq_hz={record.frequency_hz} "
            f"service={record.service_id} cn0={record.cn0_dbhz:.2f}dBHz "
            f"status={LBAND_STATUS_NAMES.get(record.status, str(record.status))} "
            f"source={LBAND_SOURCE_NAMES.get(record.source, str(record.source))}"
        )
    assert isinstance(record, P2PPStatusRecord)
    return (
        f"p2pp_status: week={record.week} tow_ms={record.tow_ms} sessions={record.sessions} "
        f"connected={record.connected_sessions} session={record.session_id} port={record.port} "
        f"status={P2PP_STATUS_NAMES.get(record.status, str(record.status))} "
        f"error={P2PP_ERROR_NAMES.get(record.error_code, str(record.error_code))}"
    )


def should_print(kind: str, args: argparse.Namespace) -> bool:
    if args.quiet:
        return False
    if not args.decode_pvt and not args.decode_lband and not args.decode_p2pp:
        return True
    return (
        (kind == "PVT_GEODETIC" and args.decode_pvt)
        or (kind == "LBAND_TRACKER" and args.decode_lband)
        or (kind == "P2PP_STATUS" and args.decode_p2pp)
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
        f"pvt_geodetic={stats.pvt_geodetic_messages} "
        f"lband_tracker={stats.lband_tracker_messages} "
        f"p2pp_status={stats.p2pp_status_messages} "
        f"valid_positions={stats.valid_positions} crc_errors={stats.crc_errors}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
