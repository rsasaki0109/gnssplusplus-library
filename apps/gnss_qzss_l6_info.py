#!/usr/bin/env python3
"""Inspect direct QZSS L6 frames from file or serial input."""

from __future__ import annotations

import argparse
import csv
import errno
import math
import os
from dataclasses import dataclass
from pathlib import Path

if os.name != "nt":
    import termios


PREAMBLE = bytes.fromhex("1ACFFC1D")
FRAME_BYTES = 250
HEADER_BITS = 49
DATA_PART_BITS = 1695
RS_BITS = 256
SUBFRAME_FRAMES = 5
DEFAULT_SERIAL_BAUD = 115200
MAX_PREVIEW_BYTES = 16
CSSR_TYPE = 4073
CSSR_SUBTYPE_MASK = 1
CSSR_SUBTYPE_ORBIT = 2
CSSR_SUBTYPE_CLOCK = 3
CSSR_SUBTYPE_CODE_BIAS = 4
CSSR_SUBTYPE_PHASE_BIAS = 5
CSSR_SUBTYPE_CODE_PHASE_BIAS = 6
CSSR_SUBTYPE_URA = 7
CSSR_SUBTYPE_STEC = 8
CSSR_SUBTYPE_GRIDDED = 9
CSSR_SUBTYPE_SERVICE_INFO = 10
CSSR_SUBTYPE_COMBINED = 11
CSSR_SUBTYPE_ATMOS = 12

COMPACT_SSR_FLUSH_POLICY_LAG_TOLERANT = "lag-tolerant-union"
COMPACT_SSR_FLUSH_POLICY_ORBIT_OR_CLOCK_ONLY = "orbit-or-clock-only"
COMPACT_SSR_FLUSH_POLICY_ORBIT_AND_CLOCK_ONLY = "orbit-and-clock-only"
COMPACT_SSR_FLUSH_POLICIES = (
    COMPACT_SSR_FLUSH_POLICY_LAG_TOLERANT,
    COMPACT_SSR_FLUSH_POLICY_ORBIT_OR_CLOCK_ONLY,
    COMPACT_SSR_FLUSH_POLICY_ORBIT_AND_CLOCK_ONLY,
)
COMPACT_ATMOS_MERGE_POLICY_STEC_COEFF_CARRY = "stec-coeff-carry"
COMPACT_ATMOS_MERGE_POLICY_NO_CARRY = "no-carry"
COMPACT_ATMOS_MERGE_POLICY_NETWORK_LOCKED_STEC_COEFF_CARRY = "network-locked-stec-coeff-carry"
COMPACT_ATMOS_MERGE_POLICIES = (
    COMPACT_ATMOS_MERGE_POLICY_STEC_COEFF_CARRY,
    COMPACT_ATMOS_MERGE_POLICY_NO_CARRY,
    COMPACT_ATMOS_MERGE_POLICY_NETWORK_LOCKED_STEC_COEFF_CARRY,
)
COMPACT_ATMOS_SUBTYPE_MERGE_POLICY_UNION = "union"
COMPACT_ATMOS_SUBTYPE_MERGE_POLICY_GRIDDED_PRIORITY = "gridded-priority"
COMPACT_ATMOS_SUBTYPE_MERGE_POLICY_COMBINED_PRIORITY = "combined-priority"
COMPACT_ATMOS_SUBTYPE_MERGE_POLICIES = (
    COMPACT_ATMOS_SUBTYPE_MERGE_POLICY_UNION,
    COMPACT_ATMOS_SUBTYPE_MERGE_POLICY_GRIDDED_PRIORITY,
    COMPACT_ATMOS_SUBTYPE_MERGE_POLICY_COMBINED_PRIORITY,
)
COMPACT_PHASE_BIAS_MERGE_POLICY_LATEST_UNION = "latest-union"
COMPACT_PHASE_BIAS_MERGE_POLICY_MESSAGE_RESET = "message-reset"
COMPACT_PHASE_BIAS_MERGE_POLICY_SELECTED_MASK_PRUNE = "selected-mask-prune"
COMPACT_PHASE_BIAS_MERGE_POLICIES = (
    COMPACT_PHASE_BIAS_MERGE_POLICY_LATEST_UNION,
    COMPACT_PHASE_BIAS_MERGE_POLICY_MESSAGE_RESET,
    COMPACT_PHASE_BIAS_MERGE_POLICY_SELECTED_MASK_PRUNE,
)
COMPACT_PHASE_BIAS_SOURCE_POLICY_ARRIVAL_ORDER = "arrival-order"
COMPACT_PHASE_BIAS_SOURCE_POLICY_SUBTYPE5_PRIORITY = "subtype5-priority"
COMPACT_PHASE_BIAS_SOURCE_POLICY_SUBTYPE6_PRIORITY = "subtype6-priority"
COMPACT_PHASE_BIAS_SOURCE_POLICIES = (
    COMPACT_PHASE_BIAS_SOURCE_POLICY_ARRIVAL_ORDER,
    COMPACT_PHASE_BIAS_SOURCE_POLICY_SUBTYPE5_PRIORITY,
    COMPACT_PHASE_BIAS_SOURCE_POLICY_SUBTYPE6_PRIORITY,
)
COMPACT_CODE_BIAS_COMPOSITION_POLICY_DIRECT = "direct-values"
COMPACT_CODE_BIAS_COMPOSITION_POLICY_BASE_PLUS_NETWORK = "base-plus-network"
COMPACT_CODE_BIAS_COMPOSITION_POLICY_BASE_ONLY_IF_PRESENT = "base-only-if-present"
COMPACT_CODE_BIAS_COMPOSITION_POLICIES = (
    COMPACT_CODE_BIAS_COMPOSITION_POLICY_DIRECT,
    COMPACT_CODE_BIAS_COMPOSITION_POLICY_BASE_PLUS_NETWORK,
    COMPACT_CODE_BIAS_COMPOSITION_POLICY_BASE_ONLY_IF_PRESENT,
)
COMPACT_CODE_BIAS_BANK_POLICY_PENDING_EPOCH = "pending-epoch"
COMPACT_CODE_BIAS_BANK_POLICY_SAME_30S_BANK = "same-30s-bank"
COMPACT_CODE_BIAS_BANK_POLICY_CLOSE_30S_BANK = "close-30s-bank"
COMPACT_CODE_BIAS_BANK_POLICY_LATEST_PRECEDING_BANK = "latest-preceding-bank"
COMPACT_CODE_BIAS_BANK_POLICIES = (
    COMPACT_CODE_BIAS_BANK_POLICY_PENDING_EPOCH,
    COMPACT_CODE_BIAS_BANK_POLICY_SAME_30S_BANK,
    COMPACT_CODE_BIAS_BANK_POLICY_CLOSE_30S_BANK,
    COMPACT_CODE_BIAS_BANK_POLICY_LATEST_PRECEDING_BANK,
)
COMPACT_BIAS_ROW_MATERIALIZATION_POLICY_OVERLAP_ONLY = "overlap-only"
COMPACT_BIAS_ROW_MATERIALIZATION_POLICY_SELECTED_SATELLITE_BASE_EXTEND = (
    "selected-satellite-base-extend"
)
COMPACT_BIAS_ROW_MATERIALIZATION_POLICY_ALL_BASE_SATELLITE_EXTEND = (
    "all-base-satellite-extend"
)
COMPACT_BIAS_ROW_MATERIALIZATION_POLICIES = (
    COMPACT_BIAS_ROW_MATERIALIZATION_POLICY_OVERLAP_ONLY,
    COMPACT_BIAS_ROW_MATERIALIZATION_POLICY_SELECTED_SATELLITE_BASE_EXTEND,
    COMPACT_BIAS_ROW_MATERIALIZATION_POLICY_ALL_BASE_SATELLITE_EXTEND,
)
COMPACT_ROW_CONSTRUCTION_POLICY_INDEPENDENT = "independent"
COMPACT_ROW_CONSTRUCTION_POLICY_COUPLED_CODE_PHASE = "coupled-code-phase"
COMPACT_ROW_CONSTRUCTION_POLICY_ROW_FIRST_VALUE_SECOND = "row-first-value-second"
COMPACT_ROW_CONSTRUCTION_POLICY_NETWORK_ROW_DRIVEN = "network-row-driven"
COMPACT_ROW_CONSTRUCTION_POLICIES = (
    COMPACT_ROW_CONSTRUCTION_POLICY_INDEPENDENT,
    COMPACT_ROW_CONSTRUCTION_POLICY_COUPLED_CODE_PHASE,
    COMPACT_ROW_CONSTRUCTION_POLICY_ROW_FIRST_VALUE_SECOND,
    COMPACT_ROW_CONSTRUCTION_POLICY_NETWORK_ROW_DRIVEN,
)
COMPACT_PHASE_BIAS_COMPOSITION_POLICY_DIRECT = "direct-values"
COMPACT_PHASE_BIAS_COMPOSITION_POLICY_BASE_PLUS_NETWORK = "base-plus-network"
COMPACT_PHASE_BIAS_COMPOSITION_POLICY_BASE_ONLY_IF_PRESENT = "base-only-if-present"
COMPACT_PHASE_BIAS_COMPOSITION_POLICIES = (
    COMPACT_PHASE_BIAS_COMPOSITION_POLICY_DIRECT,
    COMPACT_PHASE_BIAS_COMPOSITION_POLICY_BASE_PLUS_NETWORK,
    COMPACT_PHASE_BIAS_COMPOSITION_POLICY_BASE_ONLY_IF_PRESENT,
)
COMPACT_PHASE_BIAS_BANK_POLICY_PENDING_EPOCH = "pending-epoch"
COMPACT_PHASE_BIAS_BANK_POLICY_SAME_30S_BANK = "same-30s-bank"
COMPACT_PHASE_BIAS_BANK_POLICY_CLOSE_30S_BANK = "close-30s-bank"
COMPACT_PHASE_BIAS_BANK_POLICY_LATEST_PRECEDING_BANK = "latest-preceding-bank"
COMPACT_PHASE_BIAS_BANK_POLICIES = (
    COMPACT_PHASE_BIAS_BANK_POLICY_PENDING_EPOCH,
    COMPACT_PHASE_BIAS_BANK_POLICY_SAME_30S_BANK,
    COMPACT_PHASE_BIAS_BANK_POLICY_CLOSE_30S_BANK,
    COMPACT_PHASE_BIAS_BANK_POLICY_LATEST_PRECEDING_BANK,
)
PHASE_BIAS_BANK_BUCKET_SECONDS = 30
PHASE_BIAS_BANK_RETENTION_SECONDS = 180
SSR_UPDATE_INTERVALS = (
    1.0,
    2.0,
    5.0,
    10.0,
    15.0,
    30.0,
    60.0,
    120.0,
    240.0,
    300.0,
    600.0,
    900.0,
    1800.0,
    3600.0,
    7200.0,
    10800.0,
)
CSSR_GNSS_LABELS = {
    0: "G",
    1: "R",
    2: "E",
    3: "C",
    # QZSS CLAS uses CSSR GNSS id 4 with PRNs in the 120 range.
    # Normalize to the observation/navigation convention J01...
    4: "J",
    5: "J",
}
CSSR_PRN_BASE = {
    0: 1,
    1: 1,
    2: 1,
    3: 1,
    4: 1,
    5: 193,
}
CSSR_SIGNAL_RTCM_IDS = {
    0: {0: 2, 1: 3, 2: 3, 3: 2, 4: 2, 5: 2, 6: 8, 7: 8, 8: 8, 9: 9, 10: 9, 11: 22, 12: 22, 13: 22},
    1: {0: 2, 1: 3, 2: 8, 3: 9},
    2: {0: 2, 1: 2, 2: 2, 3: 22, 4: 22, 5: 22, 6: 14, 7: 14, 8: 14},
    3: {0: 2, 1: 2, 2: 2, 3: 8, 4: 8, 5: 8, 6: 14, 7: 14, 8: 14},
    4: {0: 2, 1: 2, 2: 2, 3: 2, 4: 8, 5: 8, 6: 8, 7: 22, 8: 22, 9: 22},
    5: {0: 2, 1: 2, 2: 2, 3: 2, 4: 8, 5: 8, 6: 8, 7: 22, 8: 22, 9: 22},
}


def read_bits(data: bytes | bytearray, bit_offset: int, bit_length: int) -> int:
    value = 0
    for relative in range(bit_length):
        absolute = bit_offset + relative
        byte_index = absolute // 8
        bit_in_byte = 7 - (absolute % 8)
        value = (value << 1) | ((data[byte_index] >> bit_in_byte) & 1)
    return value


def read_bitpacked_bytes(data: bytes | bytearray, bit_offset: int, bit_length: int) -> bytes:
    output = bytearray((bit_length + 7) // 8)
    for relative in range(bit_length):
        absolute = bit_offset + relative
        src_byte = absolute // 8
        src_bit = 7 - (absolute % 8)
        bit_value = (data[src_byte] >> src_bit) & 1
        dst_byte = relative // 8
        dst_bit = 7 - (relative % 8)
        output[dst_byte] |= bit_value << dst_bit
    return bytes(output)


def parse_serial_path(path: str) -> tuple[str, int]:
    raw = path[len("serial://") :] if path.startswith("serial://") else path
    if "?baud=" in raw:
        device, baud_text = raw.split("?baud=", 1)
        return device, int(baud_text)
    return raw, DEFAULT_SERIAL_BAUD


def configure_serial(fd: int, baud: int) -> None:
    if os.name == "nt":
        raise RuntimeError("serial QZSS L6 input is not supported on this platform")
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


@dataclass
class Stats:
    frames: int = 0
    valid: int = 0
    clas_vendor: int = 0
    subframe_starts: int = 0
    alerts: int = 0
    subframes: int = 0
    discarded_bytes: int = 0
    prns: set[int] | None = None

    def __post_init__(self) -> None:
        if self.prns is None:
            self.prns = set()


@dataclass
class L6Frame:
    index: int
    prn: int
    vendor_id: int
    facility_id: int
    reserved_bits: int
    subframe_start: bool
    alert: bool
    data_part_bits: int
    data_part: bytes
    rs_parity: bytes

    @property
    def is_clas(self) -> bool:
        return self.vendor_id == 0b101

    @property
    def facility_name(self) -> str:
        if self.facility_id in {0, 1}:
            return "Hitachi-Ota"
        if self.facility_id in {2, 3}:
            return "Kobe"
        return "Unknown"

    @property
    def data_part_hex(self) -> str:
        return self.data_part.hex()

    @property
    def rs_hex(self) -> str:
        return self.rs_parity.hex()

    @property
    def preview_text(self) -> str:
        preview = self.data_part[:MAX_PREVIEW_BYTES]
        rendered = []
        for byte in preview:
            rendered.append(chr(byte) if 32 <= byte <= 126 else ".")
        return "".join(rendered).rstrip(".")


def concatenate_bitpacked_parts(parts: list[bytes], bits_per_part: int) -> bytes:
    total_bits = len(parts) * bits_per_part
    output = bytearray((total_bits + 7) // 8)
    dst_bit_offset = 0
    for part in parts:
        for src_bit_offset in range(bits_per_part):
            src_byte = src_bit_offset // 8
            src_bit = 7 - (src_bit_offset % 8)
            bit_value = (part[src_byte] >> src_bit) & 1
            dst_byte = dst_bit_offset // 8
            dst_bit = 7 - (dst_bit_offset % 8)
            output[dst_byte] |= bit_value << dst_bit
            dst_bit_offset += 1
    return bytes(output)


@dataclass
class L6Subframe:
    index: int
    prn: int
    vendor_id: int
    facility_id: int
    frame_count: int
    alert_frames: int
    data_bits: int
    data_part: bytes
    first_frame_index: int
    last_frame_index: int

    @property
    def facility_name(self) -> str:
        if self.facility_id in {0, 1}:
            return "Hitachi-Ota"
        if self.facility_id in {2, 3}:
            return "Kobe"
        return "Unknown"

    @property
    def data_hex(self) -> str:
        return self.data_part.hex()


@dataclass
class CSSRSatellite:
    system_id: int
    system: str
    prn: int
    sat: str
    signal_slots: list[int]

    @property
    def active_signals(self) -> int:
        return len(self.signal_slots)


@dataclass
class CSSRMaskState:
    iod: int
    tow0: int
    satellites: list[CSSRSatellite]

    @property
    def satellite_count(self) -> int:
        return len(self.satellites)

    @property
    def signal_count(self) -> int:
        return sum(satellite.active_signals for satellite in self.satellites)


@dataclass
class CSSRMessage:
    subframe_index: int
    message_index: int
    ctype: int
    subtype: int
    tow: int
    sync: bool
    iod: int
    udi_seconds: float
    message_bits: int
    detail: str
    correction_count: int = 0


@dataclass
class CompactSSRCorrection:
    week: int
    tow: float
    system: str
    prn: int
    dx: float
    dy: float
    dz: float
    dclock_m: float
    high_rate_clock_m: float = 0.0
    ura_sigma_m: float | None = None
    code_bias_m: dict[int, float] | None = None
    phase_bias_m: dict[int, float] | None = None
    phase_discontinuity: dict[int, int] | None = None
    bias_network_id: int | None = None
    atmos_network_id: int | None = None
    atmos_trop_avail: int | None = None
    atmos_stec_avail: int | None = None
    atmos_grid_count: int | None = None
    atmos_selected_satellites: int | None = None
    atmos_tokens: dict[str, str] | None = None


@dataclass
class CSSRServiceInfoPacket:
    packet_index: int
    first_subframe_index: int
    last_subframe_index: int
    chunk_count: int
    total_bits: int
    packet_bytes: bytes

    @property
    def packet_hex(self) -> str:
        return self.packet_bytes.hex()


@dataclass
class CSSRDecoderState:
    mask: CSSRMaskState | None = None
    message_index: int = 0
    pending_orbit: dict[str, tuple[float, float, float]] | None = None
    pending_clock: dict[str, float] | None = None
    pending_tow: int | None = None
    pending_iod: int | None = None
    pending_udi_seconds: float | None = None
    pending_ura: dict[str, float] | None = None
    pending_code_bias: dict[str, dict[int, float]] | None = None
    pending_base_code_bias: dict[str, dict[int, float]] | None = None
    base_code_bias_banks: dict[int, dict[str, dict[int, float]]] | None = None
    pending_phase_bias: dict[str, dict[int, float]] | None = None
    pending_phase_bias_source: dict[str, dict[int, int]] | None = None
    pending_phase_discontinuity: dict[str, dict[int, int]] | None = None
    pending_base_phase_bias: dict[str, dict[int, float]] | None = None
    base_phase_bias_banks: dict[int, dict[str, dict[int, float]]] | None = None
    pending_bias_network_id: int | None = None
    pending_atmos: dict[str, str] | None = None
    pending_atmos_subtypes: set[int] | None = None
    pending_atmos_by_network: dict[int, dict[str, str]] | None = None
    pending_atmos_subtypes_by_network: dict[int, set[int]] | None = None
    service_info_chunks: list[tuple[int, bytes, int]] | None = None
    service_packet_index: int = 0


class L6SubframeAssembler:
    def __init__(self) -> None:
        self._pending: dict[tuple[int, int, int], list[L6Frame]] = {}
        self._subframe_index = 0

    def push(self, frame: L6Frame) -> list[L6Subframe]:
        key = (frame.prn, frame.vendor_id, frame.facility_id)
        if frame.subframe_start or key not in self._pending:
            self._pending[key] = []
        self._pending[key].append(frame)
        if len(self._pending[key]) < SUBFRAME_FRAMES:
            return []
        assembled = self._pending.pop(key)
        self._subframe_index += 1
        data_part = concatenate_bitpacked_parts(
            [entry.data_part for entry in assembled],
            DATA_PART_BITS,
        )
        return [
            L6Subframe(
                index=self._subframe_index,
                prn=frame.prn,
                vendor_id=frame.vendor_id,
                facility_id=frame.facility_id,
                frame_count=len(assembled),
                alert_frames=sum(1 for entry in assembled if entry.alert),
                data_bits=len(assembled) * DATA_PART_BITS,
                data_part=data_part,
                first_frame_index=assembled[0].index,
                last_frame_index=assembled[-1].index,
            )
        ]


def count_bits(value: int) -> int:
    return value.bit_count() if hasattr(int, "bit_count") else bin(value).count("1")


def read_signed_bits(data: bytes | bytearray, bit_offset: int, bit_length: int) -> int:
    value = read_bits(data, bit_offset, bit_length)
    sign_bit = 1 << (bit_length - 1)
    if value & sign_bit:
        value -= 1 << bit_length
    return value


def decode_scaled_signed(
    data: bytes | bytearray,
    bit_offset: int,
    bit_length: int,
    scale: float,
) -> tuple[float, int]:
    value = read_signed_bits(data, bit_offset, bit_length)
    invalid_sentinel = -(1 << (bit_length - 1))
    if value == invalid_sentinel:
        return float("nan"), bit_offset + bit_length
    return value * scale, bit_offset + bit_length


def cssr_signal_slots_from_mask(sigmask: int) -> list[int]:
    return [index for index in range(16) if ((sigmask >> (15 - index)) & 1) != 0]


def cssr_signal_slot_to_rtcm_id(system_id: int, signal_slot: int) -> int:
    return CSSR_SIGNAL_RTCM_IDS.get(system_id, {}).get(signal_slot, 0)


def ura_meters_from_ssr_index(ura_index: int) -> float:
    if ura_index == 0:
        return 0.15e-3
    if ura_index >= 63:
        return 5.4665
    return (
        math.pow(3.0, float((ura_index >> 3) & 0x07)) *
        (1.0 + float(ura_index & 0x07) / 4.0) -
        1.0
    ) * 1e-3


def svmask_to_satellites(
    system_id: int,
    svmask: int,
    satellite_signal_slots: list[list[int]],
) -> list[CSSRSatellite]:
    system = CSSR_GNSS_LABELS.get(system_id)
    prn_base = CSSR_PRN_BASE.get(system_id)
    if system is None or prn_base is None:
        return []
    satellites: list[CSSRSatellite] = []
    for index in range(40):
        if ((svmask >> (39 - index)) & 1) == 0:
            continue
        prn = prn_base + index
        signal_slots = (
            satellite_signal_slots[len(satellites)]
            if len(satellite_signal_slots) > len(satellites)
            else []
        )
        satellites.append(
            CSSRSatellite(
                system_id=system_id,
                system=system,
                prn=prn,
                sat=f"{system}{prn:02d}",
                signal_slots=list(signal_slots),
            )
        )
    return satellites


def decode_cssr_header(
    payload: bytes,
    bit_offset: int,
    subtype: int,
    state: CSSRDecoderState,
) -> tuple[dict[str, int | float | bool], int]:
    if subtype == CSSR_SUBTYPE_MASK:
        tow = read_bits(payload, bit_offset, 20)
        bit_offset += 20
    else:
        if state.mask is None:
            raise ValueError("CSSR subtype requires a prior mask message")
        tow = state.mask.tow0 + read_bits(payload, bit_offset, 12)
        bit_offset += 12
    update_interval_index = read_bits(payload, bit_offset, 4)
    bit_offset += 4
    sync = bool(read_bits(payload, bit_offset, 1))
    bit_offset += 1
    iod = read_bits(payload, bit_offset, 4)
    bit_offset += 4
    header = {
        "tow": tow,
        "udi_index": update_interval_index,
        "udi_seconds": SSR_UPDATE_INTERVALS[update_interval_index],
        "sync": sync,
        "iod": iod,
    }
    if subtype == CSSR_SUBTYPE_MASK:
        header["ngnss"] = read_bits(payload, bit_offset, 4)
        bit_offset += 4
    return header, bit_offset


def decode_cssr_mask_message(
    subframe: L6Subframe,
    payload: bytes,
    bit_offset: int,
    state: CSSRDecoderState,
) -> tuple[CSSRMessage, int]:
    header, bit_offset = decode_cssr_header(payload, bit_offset, CSSR_SUBTYPE_MASK, state)
    ngnss = int(header["ngnss"])
    satellites_by_system: dict[int, list[CSSRSatellite]] = {}
    systems: list[str] = []
    for _ in range(ngnss):
        system_id = read_bits(payload, bit_offset, 4)
        bit_offset += 4
        svmask = read_bits(payload, bit_offset, 40)
        bit_offset += 40
        sigmask = read_bits(payload, bit_offset, 16)
        bit_offset += 16
        cmi = read_bits(payload, bit_offset, 1)
        bit_offset += 1
        nsat = count_bits(svmask)
        system_signal_slots = cssr_signal_slots_from_mask(sigmask)
        nsig = len(system_signal_slots)
        satellite_signal_slots: list[list[int]] = []
        if cmi:
            for _ in range(nsat):
                cellmask = read_bits(payload, bit_offset, nsig)
                bit_offset += nsig
                satellite_signal_slots.append(
                    [
                        system_signal_slots[k]
                        for k in range(nsig)
                        if ((cellmask >> (nsig - 1 - k)) & 1) != 0
                    ]
                )
        else:
            satellite_signal_slots = [list(system_signal_slots) for _ in range(nsat)]
        satellites = svmask_to_satellites(system_id, svmask, satellite_signal_slots)
        satellites_by_system[system_id] = satellites
        label = CSSR_GNSS_LABELS.get(system_id, f"id{system_id}")
        systems.append(f"{label}:{len(satellites)}")

    ordered_satellites: list[CSSRSatellite] = []
    for system_id in sorted(satellites_by_system):
        ordered_satellites.extend(satellites_by_system[system_id])
    state.mask = CSSRMaskState(
        iod=int(header["iod"]),
        tow0=(int(header["tow"]) // 3600) * 3600,
        satellites=ordered_satellites,
    )
    state.message_index += 1
    return (
        CSSRMessage(
            subframe_index=subframe.index,
            message_index=state.message_index,
            ctype=CSSR_TYPE,
            subtype=CSSR_SUBTYPE_MASK,
            tow=int(header["tow"]),
            sync=bool(header["sync"]),
            iod=int(header["iod"]),
            udi_seconds=float(header["udi_seconds"]),
            message_bits=bit_offset,
            detail="systems=" + ",".join(systems),
        ),
        bit_offset,
    )


def require_mask_state(state: CSSRDecoderState, subtype: int) -> CSSRMaskState:
    if state.mask is None:
        raise ValueError(f"CSSR subtype {subtype} requires a prior mask message")
    return state.mask


def reset_pending_corrections(state: CSSRDecoderState) -> None:
    state.pending_orbit = None
    state.pending_clock = None
    state.pending_tow = None
    state.pending_iod = None
    state.pending_udi_seconds = None
    state.pending_ura = None
    state.pending_code_bias = None
    state.pending_base_code_bias = None
    state.pending_phase_bias = None
    state.pending_phase_bias_source = None
    state.pending_phase_discontinuity = None
    state.pending_base_phase_bias = None
    state.pending_bias_network_id = None
    # Preserve STEC polynomial coefficients (c00, c01, c10, etc.) across epochs.
    # These arrive from subtype 8 at ~30s intervals, while gridded residuals
    # (subtype 9) arrive every second.  Without carry-forward the c00 terms
    # would be lost between subtype-8 updates.
    if state.pending_atmos is not None:
        carry_forward = {k: v for k, v in state.pending_atmos.items()
                         if "stec_c0" in k or "stec_c1" in k or "stec_c2" in k
                         or "stec_type" in k or "stec_quality" in k
                         or k == "atmos_network_id"}
        state.pending_atmos = carry_forward if carry_forward else None
    else:
        state.pending_atmos = None
    state.pending_atmos_subtypes = None
    if state.pending_atmos_by_network is not None:
        state.pending_atmos_by_network = {
            network_id: dict(tokens)
            for network_id, tokens in state.pending_atmos_by_network.items()
            if tokens
        } or None
    else:
        state.pending_atmos_by_network = None
    state.pending_atmos_subtypes_by_network = None


def carry_forward_atmos_tokens(
    atmos_tokens: dict[str, str] | None,
    merge_policy: str,
) -> dict[str, str] | None:
    if merge_policy == COMPACT_ATMOS_MERGE_POLICY_NO_CARRY or atmos_tokens is None:
        return None
    if merge_policy not in COMPACT_ATMOS_MERGE_POLICIES:
        raise ValueError(f"unsupported Compact SSR atmos merge policy: {merge_policy}")
    carry_forward = {
        key: value
        for key, value in atmos_tokens.items()
        if "stec_c0" in key
        or "stec_c1" in key
        or "stec_c2" in key
        or "stec_type" in key
        or "stec_quality" in key
        or key == "atmos_network_id"
    }
    return carry_forward if carry_forward else None


def reset_pending_corrections_with_policy(
    state: CSSRDecoderState,
    atmos_merge_policy: str,
) -> None:
    pending_atmos = state.pending_atmos
    pending_atmos_by_network = state.pending_atmos_by_network
    reset_pending_corrections(state)
    state.pending_atmos = carry_forward_atmos_tokens(pending_atmos, atmos_merge_policy)
    if pending_atmos_by_network is None:
        state.pending_atmos_by_network = None
    elif atmos_merge_policy == COMPACT_ATMOS_MERGE_POLICY_NO_CARRY:
        state.pending_atmos_by_network = None
    else:
        state.pending_atmos_by_network = {
            network_id: dict(tokens)
            for network_id, tokens in pending_atmos_by_network.items()
            if tokens
        } or None


def phase_bias_bank_anchor_tow(tow: int) -> int:
    return tow - (tow % PHASE_BIAS_BANK_BUCKET_SECONDS)


def prune_base_code_bias_banks(state: CSSRDecoderState, current_tow: int) -> None:
    if state.base_code_bias_banks is None:
        return
    min_anchor = phase_bias_bank_anchor_tow(current_tow) - PHASE_BIAS_BANK_RETENTION_SECONDS
    state.base_code_bias_banks = {
        anchor: bank
        for anchor, bank in state.base_code_bias_banks.items()
        if anchor >= min_anchor
    }


def store_base_code_bias_bank_entry(
    state: CSSRDecoderState,
    tow: int,
    satellite_token: str,
    signal_id: int,
    bias_m: float,
) -> None:
    if state.base_code_bias_banks is None:
        state.base_code_bias_banks = {}
    prune_base_code_bias_banks(state, tow)
    anchor = phase_bias_bank_anchor_tow(tow)
    bank = state.base_code_bias_banks.setdefault(anchor, {})
    bank.setdefault(satellite_token, {})[signal_id] = bias_m


def lookup_base_code_bias_bank_entry(
    state: CSSRDecoderState,
    tow: int,
    satellite_token: str,
    signal_id: int,
    bank_policy: str,
) -> float | None:
    if bank_policy not in COMPACT_CODE_BIAS_BANK_POLICIES:
        raise ValueError(
            "unsupported Compact SSR code-bias bank policy: "
            f"{bank_policy}"
        )
    if (
        bank_policy == COMPACT_CODE_BIAS_BANK_POLICY_PENDING_EPOCH
        or not state.base_code_bias_banks
    ):
        return None
    if bank_policy == COMPACT_CODE_BIAS_BANK_POLICY_SAME_30S_BANK:
        anchors = [phase_bias_bank_anchor_tow(tow)]
    elif bank_policy == COMPACT_CODE_BIAS_BANK_POLICY_CLOSE_30S_BANK:
        current_anchor = phase_bias_bank_anchor_tow(tow)
        anchors = [
            anchor
            for anchor in state.base_code_bias_banks
            if anchor <= tow and (current_anchor - anchor) <= PHASE_BIAS_BANK_BUCKET_SECONDS
        ]
        anchors.sort(reverse=True)
    else:
        anchors = [
            anchor
            for anchor in state.base_code_bias_banks
            if anchor <= tow
        ]
        anchors.sort(reverse=True)
    for anchor in anchors:
        bias_bank = state.base_code_bias_banks.get(anchor)
        if bias_bank is None:
            continue
        bias_m = bias_bank.get(satellite_token, {}).get(signal_id)
        if bias_m is not None:
            return bias_m
    return None


def lookup_base_code_bias_bank_rows(
    state: CSSRDecoderState,
    tow: int,
    satellite_token: str,
    bank_policy: str,
) -> dict[int, float]:
    if bank_policy not in COMPACT_CODE_BIAS_BANK_POLICIES:
        raise ValueError(
            "unsupported Compact SSR code-bias bank policy: "
            f"{bank_policy}"
        )
    if (
        bank_policy == COMPACT_CODE_BIAS_BANK_POLICY_PENDING_EPOCH
        or not state.base_code_bias_banks
    ):
        return {}
    if bank_policy == COMPACT_CODE_BIAS_BANK_POLICY_SAME_30S_BANK:
        anchors = [phase_bias_bank_anchor_tow(tow)]
    elif bank_policy == COMPACT_CODE_BIAS_BANK_POLICY_CLOSE_30S_BANK:
        current_anchor = phase_bias_bank_anchor_tow(tow)
        anchors = [
            anchor
            for anchor in state.base_code_bias_banks
            if anchor <= tow and (current_anchor - anchor) <= PHASE_BIAS_BANK_BUCKET_SECONDS
        ]
        anchors.sort(reverse=True)
    else:
        anchors = [anchor for anchor in state.base_code_bias_banks if anchor <= tow]
        anchors.sort(reverse=True)
    for anchor in anchors:
        bias_bank = state.base_code_bias_banks.get(anchor)
        if bias_bank is None:
            continue
        rows = bias_bank.get(satellite_token)
        if rows:
            return dict(rows)
    return {}


def prune_base_phase_bias_banks(state: CSSRDecoderState, current_tow: int) -> None:
    if state.base_phase_bias_banks is None:
        return
    min_anchor = phase_bias_bank_anchor_tow(current_tow) - PHASE_BIAS_BANK_RETENTION_SECONDS
    state.base_phase_bias_banks = {
        anchor: bank
        for anchor, bank in state.base_phase_bias_banks.items()
        if anchor >= min_anchor
    }


def store_base_phase_bias_bank_entry(
    state: CSSRDecoderState,
    tow: int,
    satellite_token: str,
    signal_id: int,
    bias_m: float,
) -> None:
    if state.base_phase_bias_banks is None:
        state.base_phase_bias_banks = {}
    prune_base_phase_bias_banks(state, tow)
    anchor = phase_bias_bank_anchor_tow(tow)
    bank = state.base_phase_bias_banks.setdefault(anchor, {})
    bank.setdefault(satellite_token, {})[signal_id] = bias_m


def lookup_base_phase_bias_bank_entry(
    state: CSSRDecoderState,
    tow: int,
    satellite_token: str,
    signal_id: int,
    bank_policy: str,
) -> float | None:
    if bank_policy not in COMPACT_PHASE_BIAS_BANK_POLICIES:
        raise ValueError(
            "unsupported Compact SSR phase-bias bank policy: "
            f"{bank_policy}"
        )
    if (
        bank_policy == COMPACT_PHASE_BIAS_BANK_POLICY_PENDING_EPOCH
        or not state.base_phase_bias_banks
    ):
        return None
    if bank_policy == COMPACT_PHASE_BIAS_BANK_POLICY_SAME_30S_BANK:
        anchors = [phase_bias_bank_anchor_tow(tow)]
    elif bank_policy == COMPACT_PHASE_BIAS_BANK_POLICY_CLOSE_30S_BANK:
        current_anchor = phase_bias_bank_anchor_tow(tow)
        anchors = [
            anchor
            for anchor in state.base_phase_bias_banks
            if anchor <= tow and (current_anchor - anchor) <= PHASE_BIAS_BANK_BUCKET_SECONDS
        ]
        anchors.sort(reverse=True)
    else:
        anchors = [
            anchor
            for anchor in state.base_phase_bias_banks
            if anchor <= tow
        ]
        anchors.sort(reverse=True)
    for anchor in anchors:
        bias_bank = state.base_phase_bias_banks.get(anchor)
        if bias_bank is None:
            continue
        bias_m = bias_bank.get(satellite_token, {}).get(signal_id)
        if bias_m is not None:
            return bias_m
    return None


def lookup_base_phase_bias_bank_rows(
    state: CSSRDecoderState,
    tow: int,
    satellite_token: str,
    bank_policy: str,
) -> dict[int, float]:
    if bank_policy not in COMPACT_PHASE_BIAS_BANK_POLICIES:
        raise ValueError(
            "unsupported Compact SSR phase-bias bank policy: "
            f"{bank_policy}"
        )
    if (
        bank_policy == COMPACT_PHASE_BIAS_BANK_POLICY_PENDING_EPOCH
        or not state.base_phase_bias_banks
    ):
        return {}
    if bank_policy == COMPACT_PHASE_BIAS_BANK_POLICY_SAME_30S_BANK:
        anchors = [phase_bias_bank_anchor_tow(tow)]
    elif bank_policy == COMPACT_PHASE_BIAS_BANK_POLICY_CLOSE_30S_BANK:
        current_anchor = phase_bias_bank_anchor_tow(tow)
        anchors = [
            anchor
            for anchor in state.base_phase_bias_banks
            if anchor <= tow and (current_anchor - anchor) <= PHASE_BIAS_BANK_BUCKET_SECONDS
        ]
        anchors.sort(reverse=True)
    else:
        anchors = [anchor for anchor in state.base_phase_bias_banks if anchor <= tow]
        anchors.sort(reverse=True)
    for anchor in anchors:
        bias_bank = state.base_phase_bias_banks.get(anchor)
        if bias_bank is None:
            continue
        rows = bias_bank.get(satellite_token)
        if rows:
            return dict(rows)
    return {}


def resolve_base_code_bias_rows(
    state: CSSRDecoderState,
    tow: int,
    satellite_token: str,
    bank_policy: str,
) -> dict[int, float]:
    rows = lookup_base_code_bias_bank_rows(state, tow, satellite_token, bank_policy)
    if state.pending_base_code_bias is not None:
        rows.update(state.pending_base_code_bias.get(satellite_token, {}))
    return rows


def resolve_base_phase_bias_rows(
    state: CSSRDecoderState,
    tow: int,
    satellite_token: str,
    bank_policy: str,
) -> dict[int, float]:
    rows = lookup_base_phase_bias_bank_rows(state, tow, satellite_token, bank_policy)
    if state.pending_base_phase_bias is not None:
        rows.update(state.pending_base_phase_bias.get(satellite_token, {}))
    return rows


def materialize_missing_code_bias_rows(
    state: CSSRDecoderState,
    tow: int,
    *,
    mask_satellites: list[CSSRSatellite],
    selected_satellites: set[str],
    row_materialization_policy: str,
    bank_policy: str,
) -> None:
    if row_materialization_policy not in COMPACT_BIAS_ROW_MATERIALIZATION_POLICIES:
        raise ValueError(
            "unsupported Compact SSR bias row materialization policy: "
            f"{row_materialization_policy}"
        )
    if row_materialization_policy == COMPACT_BIAS_ROW_MATERIALIZATION_POLICY_OVERLAP_ONLY:
        return
    assert state.pending_code_bias is not None
    if row_materialization_policy == COMPACT_BIAS_ROW_MATERIALIZATION_POLICY_SELECTED_SATELLITE_BASE_EXTEND:
        target_satellites = set(selected_satellites)
    else:
        target_satellites = {satellite.sat for satellite in mask_satellites}
    for satellite_token in sorted(target_satellites):
        base_rows = resolve_base_code_bias_rows(state, tow, satellite_token, bank_policy)
        if not base_rows:
            continue
        satellite_biases = state.pending_code_bias.setdefault(satellite_token, {})
        for signal_id, bias_m in base_rows.items():
            satellite_biases.setdefault(signal_id, bias_m)


def materialize_missing_phase_bias_rows(
    state: CSSRDecoderState,
    tow: int,
    *,
    mask_satellites: list[CSSRSatellite],
    selected_satellites: set[str],
    row_materialization_policy: str,
    bank_policy: str,
) -> None:
    if row_materialization_policy not in COMPACT_BIAS_ROW_MATERIALIZATION_POLICIES:
        raise ValueError(
            "unsupported Compact SSR bias row materialization policy: "
            f"{row_materialization_policy}"
        )
    if row_materialization_policy == COMPACT_BIAS_ROW_MATERIALIZATION_POLICY_OVERLAP_ONLY:
        return
    assert state.pending_phase_bias is not None
    assert state.pending_phase_bias_source is not None
    if row_materialization_policy == COMPACT_BIAS_ROW_MATERIALIZATION_POLICY_SELECTED_SATELLITE_BASE_EXTEND:
        target_satellites = set(selected_satellites)
    else:
        target_satellites = {satellite.sat for satellite in mask_satellites}
    for satellite_token in sorted(target_satellites):
        base_rows = resolve_base_phase_bias_rows(state, tow, satellite_token, bank_policy)
        if not base_rows:
            continue
        satellite_biases = state.pending_phase_bias.setdefault(satellite_token, {})
        satellite_sources = state.pending_phase_bias_source.setdefault(satellite_token, {})
        for signal_id, bias_m in base_rows.items():
            if signal_id in satellite_biases:
                continue
            satellite_biases[signal_id] = bias_m
            satellite_sources[signal_id] = CSSR_SUBTYPE_CODE_PHASE_BIAS


def apply_coupled_code_phase_construction(
    state: CSSRDecoderState,
    tow: int,
    *,
    selected_satellites: set[str],
    code_bias_bank_policy: str,
    phase_bias_bank_policy: str,
) -> None:
    assert state.pending_code_bias is not None
    assert state.pending_phase_bias is not None
    assert state.pending_phase_bias_source is not None
    all_sat_tokens = set(state.pending_code_bias) | set(state.pending_phase_bias)
    for satellite_token in sorted(all_sat_tokens & selected_satellites):
        code_signals = set(state.pending_code_bias.get(satellite_token, {}))
        phase_signals = set(state.pending_phase_bias.get(satellite_token, {}))
        missing_code_signals = phase_signals - code_signals
        if missing_code_signals:
            base_code = resolve_base_code_bias_rows(
                state, tow, satellite_token, code_bias_bank_policy,
            )
            sat_code = state.pending_code_bias.setdefault(satellite_token, {})
            for signal_id in sorted(missing_code_signals):
                if signal_id in base_code:
                    sat_code.setdefault(signal_id, base_code[signal_id])
        missing_phase_signals = code_signals - phase_signals
        if missing_phase_signals:
            base_phase = resolve_base_phase_bias_rows(
                state, tow, satellite_token, phase_bias_bank_policy,
            )
            sat_phase = state.pending_phase_bias.setdefault(satellite_token, {})
            sat_sources = state.pending_phase_bias_source.setdefault(satellite_token, {})
            for signal_id in sorted(missing_phase_signals):
                if signal_id in base_phase:
                    sat_phase.setdefault(signal_id, base_phase[signal_id])
                    sat_sources.setdefault(signal_id, CSSR_SUBTYPE_CODE_PHASE_BIAS)


def apply_row_first_value_second_construction(
    state: CSSRDecoderState,
    tow: int,
    *,
    selected_satellites: set[str],
    code_bias_bank_policy: str,
    phase_bias_bank_policy: str,
    code_bias_composition_policy: str,
    phase_bias_composition_policy: str,
) -> None:
    assert state.pending_code_bias is not None
    assert state.pending_phase_bias is not None
    assert state.pending_phase_bias_source is not None
    for satellite_token in sorted(selected_satellites):
        base_code = resolve_base_code_bias_rows(
            state, tow, satellite_token, code_bias_bank_policy,
        )
        base_phase = resolve_base_phase_bias_rows(
            state, tow, satellite_token, phase_bias_bank_policy,
        )
        all_signals = (
            set(state.pending_code_bias.get(satellite_token, {}))
            | set(state.pending_phase_bias.get(satellite_token, {}))
            | set(base_code)
            | set(base_phase)
        )
        if not all_signals:
            continue
        sat_code = state.pending_code_bias.setdefault(satellite_token, {})
        sat_phase = state.pending_phase_bias.setdefault(satellite_token, {})
        sat_sources = state.pending_phase_bias_source.setdefault(satellite_token, {})
        for signal_id in sorted(all_signals):
            if signal_id not in sat_code and signal_id in base_code:
                sat_code[signal_id] = base_code[signal_id]
            if signal_id not in sat_phase and signal_id in base_phase:
                sat_phase[signal_id] = base_phase[signal_id]
                sat_sources.setdefault(signal_id, CSSR_SUBTYPE_CODE_PHASE_BIAS)


def apply_network_row_driven_construction(
    state: CSSRDecoderState,
    *,
    selected_satellites: set[str],
    mask_satellites: list,
) -> None:
    assert state.pending_code_bias is not None
    assert state.pending_phase_bias is not None
    assert state.pending_phase_bias_source is not None
    all_mask_sats = {satellite.sat for satellite in mask_satellites}
    non_network_sats = all_mask_sats - selected_satellites
    for satellite_token in non_network_sats:
        state.pending_code_bias.pop(satellite_token, None)
        state.pending_phase_bias.pop(satellite_token, None)
        state.pending_phase_bias_source.pop(satellite_token, None)
        if state.pending_phase_discontinuity is not None:
            state.pending_phase_discontinuity.pop(satellite_token, None)


def prepare_pending_phase_bias_for_message(
    state: CSSRDecoderState,
    phase_bias_merge_policy: str,
    *,
    reset_message_scope: bool = False,
    selected_satellites: set[str] | None = None,
) -> None:
    if phase_bias_merge_policy not in COMPACT_PHASE_BIAS_MERGE_POLICIES:
        raise ValueError(
            "unsupported Compact SSR phase-bias merge policy: "
            f"{phase_bias_merge_policy}"
        )
    if state.pending_phase_bias is None:
        return
    if phase_bias_merge_policy == COMPACT_PHASE_BIAS_MERGE_POLICY_LATEST_UNION:
        return
    if phase_bias_merge_policy == COMPACT_PHASE_BIAS_MERGE_POLICY_MESSAGE_RESET:
        if reset_message_scope:
            state.pending_phase_bias = {}
            state.pending_phase_bias_source = {}
            state.pending_phase_discontinuity = {}
        return
    if selected_satellites is None:
        return
    state.pending_phase_bias = {
        sat_token: biases
        for sat_token, biases in state.pending_phase_bias.items()
        if sat_token in selected_satellites
    }
    if state.pending_phase_bias_source is not None:
        state.pending_phase_bias_source = {
            sat_token: sources
            for sat_token, sources in state.pending_phase_bias_source.items()
            if sat_token in selected_satellites
        }
    if state.pending_phase_discontinuity is not None:
        state.pending_phase_discontinuity = {
            sat_token: indicators
            for sat_token, indicators in state.pending_phase_discontinuity.items()
            if sat_token in selected_satellites
        }


def should_replace_phase_bias_source(
    existing_source_subtype: int | None,
    incoming_source_subtype: int,
    source_policy: str,
) -> bool:
    if source_policy not in COMPACT_PHASE_BIAS_SOURCE_POLICIES:
        raise ValueError(
            "unsupported Compact SSR phase-bias source policy: "
            f"{source_policy}"
        )
    if existing_source_subtype is None:
        return True
    if source_policy == COMPACT_PHASE_BIAS_SOURCE_POLICY_ARRIVAL_ORDER:
        return True
    if source_policy == COMPACT_PHASE_BIAS_SOURCE_POLICY_SUBTYPE5_PRIORITY:
        return (
            existing_source_subtype != CSSR_SUBTYPE_PHASE_BIAS
            or incoming_source_subtype == CSSR_SUBTYPE_PHASE_BIAS
        )
    return (
        existing_source_subtype != CSSR_SUBTYPE_CODE_PHASE_BIAS
        or incoming_source_subtype == CSSR_SUBTYPE_CODE_PHASE_BIAS
    )


def store_pending_phase_bias(
    state: CSSRDecoderState,
    satellite_token: str,
    signal_id: int,
    bias_m: float,
    phase_discontinuity: int,
    *,
    source_subtype: int,
    source_policy: str,
) -> bool:
    assert state.pending_phase_bias is not None
    assert state.pending_phase_bias_source is not None
    assert state.pending_phase_discontinuity is not None
    satellite_phase_biases = state.pending_phase_bias.setdefault(satellite_token, {})
    satellite_sources = state.pending_phase_bias_source.setdefault(satellite_token, {})
    satellite_discontinuities = state.pending_phase_discontinuity.setdefault(satellite_token, {})
    existing_source = satellite_sources.get(signal_id)
    if not should_replace_phase_bias_source(existing_source, source_subtype, source_policy):
        return False
    satellite_phase_biases[signal_id] = bias_m
    satellite_sources[signal_id] = source_subtype
    satellite_discontinuities[signal_id] = phase_discontinuity
    return True


def compose_phase_bias_value(
    *,
    state: CSSRDecoderState,
    tow: int,
    satellite_token: str,
    signal_id: int,
    network_bias_m: float,
    composition_policy: str,
    bank_policy: str,
) -> float:
    if composition_policy not in COMPACT_PHASE_BIAS_COMPOSITION_POLICIES:
        raise ValueError(
            "unsupported Compact SSR phase-bias composition policy: "
            f"{composition_policy}"
        )
    if composition_policy == COMPACT_PHASE_BIAS_COMPOSITION_POLICY_DIRECT:
        return network_bias_m
    base_bias_m = None
    if state.pending_base_phase_bias is not None:
        base_bias_m = state.pending_base_phase_bias.get(satellite_token, {}).get(signal_id)
    if base_bias_m is None:
        base_bias_m = lookup_base_phase_bias_bank_entry(
            state,
            tow,
            satellite_token,
            signal_id,
            bank_policy,
        )
    if base_bias_m is None:
        return network_bias_m
    if composition_policy == COMPACT_PHASE_BIAS_COMPOSITION_POLICY_BASE_PLUS_NETWORK:
        return base_bias_m + network_bias_m
    return base_bias_m


def compose_code_bias_value(
    *,
    state: CSSRDecoderState,
    tow: int,
    satellite_token: str,
    signal_id: int,
    network_bias_m: float,
    composition_policy: str,
    bank_policy: str,
) -> float:
    if composition_policy not in COMPACT_CODE_BIAS_COMPOSITION_POLICIES:
        raise ValueError(
            "unsupported Compact SSR code-bias composition policy: "
            f"{composition_policy}"
        )
    if composition_policy == COMPACT_CODE_BIAS_COMPOSITION_POLICY_DIRECT:
        return network_bias_m
    base_bias_m = None
    if state.pending_base_code_bias is not None:
        base_bias_m = state.pending_base_code_bias.get(satellite_token, {}).get(signal_id)
    if base_bias_m is None:
        base_bias_m = lookup_base_code_bias_bank_entry(
            state,
            tow,
            satellite_token,
            signal_id,
            bank_policy,
        )
    if base_bias_m is None:
        return network_bias_m
    if composition_policy == COMPACT_CODE_BIAS_COMPOSITION_POLICY_BASE_PLUS_NETWORK:
        return base_bias_m + network_bias_m
    return base_bias_m


def merge_pending_atmos(
    state: CSSRDecoderState,
    atmos_tokens: dict[str, str],
    atmos_merge_policy: str,
    atmos_subtype_merge_policy: str,
    source_subtype: int,
) -> None:
    if atmos_merge_policy not in COMPACT_ATMOS_MERGE_POLICIES:
        raise ValueError(f"unsupported Compact SSR atmos merge policy: {atmos_merge_policy}")
    if atmos_subtype_merge_policy not in COMPACT_ATMOS_SUBTYPE_MERGE_POLICIES:
        raise ValueError(
            "unsupported Compact SSR atmos subtype merge policy: "
            f"{atmos_subtype_merge_policy}"
        )
    pending_subtypes = set() if state.pending_atmos_subtypes is None else set(state.pending_atmos_subtypes)
    if (
        atmos_merge_policy == COMPACT_ATMOS_MERGE_POLICY_NETWORK_LOCKED_STEC_COEFF_CARRY
        and state.pending_atmos is not None
    ):
        carried_network = state.pending_atmos.get("atmos_network_id")
        incoming_network = atmos_tokens.get("atmos_network_id")
        if (
            carried_network is not None
            and incoming_network is not None
            and carried_network != incoming_network
        ):
            state.pending_atmos = None
            pending_subtypes.clear()
    if (
        atmos_subtype_merge_policy == COMPACT_ATMOS_SUBTYPE_MERGE_POLICY_COMBINED_PRIORITY
        and CSSR_SUBTYPE_ATMOS in pending_subtypes
        and source_subtype in {CSSR_SUBTYPE_STEC, CSSR_SUBTYPE_GRIDDED}
    ):
        return
    if (
        atmos_subtype_merge_policy == COMPACT_ATMOS_SUBTYPE_MERGE_POLICY_GRIDDED_PRIORITY
        and source_subtype == CSSR_SUBTYPE_STEC
        and state.pending_atmos is not None
        and any(
            key == "atmos_trop_residuals_m"
            or key == "atmos_trop_hs_residuals_m"
            or key == "atmos_trop_wet_residuals_m"
            or key.startswith("atmos_stec_residuals_tecu:")
            for key in state.pending_atmos
        )
    ):
        return
    incoming_satellites = {
        key.split(":", 1)[1]
        for key in atmos_tokens
        if key.startswith("atmos_stec_") and ":" in key
    }
    if state.pending_atmos is not None:
        if (
            atmos_subtype_merge_policy == COMPACT_ATMOS_SUBTYPE_MERGE_POLICY_COMBINED_PRIORITY
            and source_subtype == CSSR_SUBTYPE_ATMOS
        ):
            state.pending_atmos = None
            pending_subtypes.clear()
        elif (
            atmos_subtype_merge_policy == COMPACT_ATMOS_SUBTYPE_MERGE_POLICY_GRIDDED_PRIORITY
            and source_subtype in {CSSR_SUBTYPE_GRIDDED, CSSR_SUBTYPE_ATMOS}
            and any(
                key == "atmos_trop_residuals_m"
                or key == "atmos_trop_hs_residuals_m"
                or key == "atmos_trop_wet_residuals_m"
                or key.startswith("atmos_stec_residuals_tecu:")
                for key in atmos_tokens
            )
        ):
            keys_to_drop = [
                key
                for key in state.pending_atmos
                if key in {
                    "atmos_trop_type",
                    "atmos_trop_t00_m",
                    "atmos_trop_t01_m_per_deg",
                    "atmos_trop_t10_m_per_deg",
                    "atmos_trop_t11_m_per_deg2",
                }
                or (
                    ":" in key
                    and key.startswith(
                        (
                            "atmos_stec_type:",
                            "atmos_stec_quality:",
                            "atmos_stec_c00_tecu:",
                            "atmos_stec_c01_tecu_per_deg:",
                            "atmos_stec_c10_tecu_per_deg:",
                            "atmos_stec_c11_tecu_per_deg2:",
                            "atmos_stec_c02_tecu_per_deg2:",
                            "atmos_stec_c20_tecu_per_deg2:",
                        )
                    )
                    and key.split(":", 1)[1] in incoming_satellites
                )
            ]
            for key in keys_to_drop:
                state.pending_atmos.pop(key, None)
    if state.pending_atmos is None:
        state.pending_atmos = dict(atmos_tokens)
    else:
        state.pending_atmos.update(atmos_tokens)
    pending_subtypes.add(source_subtype)
    state.pending_atmos_subtypes = pending_subtypes

    network_text = atmos_tokens.get("atmos_network_id")
    try:
        network_id = int(network_text) if network_text is not None else 0
    except ValueError:
        network_id = 0
    if network_id <= 0:
        return
    if state.pending_atmos_by_network is None:
        state.pending_atmos_by_network = {}
    if state.pending_atmos_subtypes_by_network is None:
        state.pending_atmos_subtypes_by_network = {}

    network_tokens = state.pending_atmos_by_network.get(network_id)
    network_subtypes = set(state.pending_atmos_subtypes_by_network.get(network_id, set()))
    if (
        atmos_subtype_merge_policy == COMPACT_ATMOS_SUBTYPE_MERGE_POLICY_COMBINED_PRIORITY
        and CSSR_SUBTYPE_ATMOS in network_subtypes
        and source_subtype in {CSSR_SUBTYPE_STEC, CSSR_SUBTYPE_GRIDDED}
    ):
        return
    if network_tokens is not None:
        if (
            atmos_subtype_merge_policy == COMPACT_ATMOS_SUBTYPE_MERGE_POLICY_COMBINED_PRIORITY
            and source_subtype == CSSR_SUBTYPE_ATMOS
        ):
            network_tokens = None
            network_subtypes.clear()
        elif (
            source_subtype in {CSSR_SUBTYPE_GRIDDED, CSSR_SUBTYPE_ATMOS}
            and any(
                key == "atmos_trop_residuals_m"
                or key == "atmos_trop_hs_residuals_m"
                or key == "atmos_trop_wet_residuals_m"
                or key.startswith("atmos_stec_residuals_tecu:")
                for key in atmos_tokens
            )
        ):
            keys_to_drop = [
                key
                for key in network_tokens
                if key in {
                    "atmos_trop_residuals_m",
                    "atmos_trop_hs_residuals_m",
                    "atmos_trop_wet_residuals_m",
                    "atmos_stec_residual_range",
                }
                or key.startswith("atmos_stec_residual_size:")
                or key.startswith("atmos_stec_residuals_tecu:")
            ]
            for key in keys_to_drop:
                network_tokens.pop(key, None)
    if network_tokens is None:
        network_tokens = dict(atmos_tokens)
    else:
        network_tokens.update(atmos_tokens)
    network_subtypes.add(source_subtype)
    state.pending_atmos_by_network[network_id] = network_tokens
    state.pending_atmos_subtypes_by_network[network_id] = network_subtypes


def ensure_pending_epoch(
    state: CSSRDecoderState,
    tow: int,
    iod: int,
    udi_seconds: float,
    atmos_merge_policy: str,
) -> None:
    if state.base_phase_bias_banks is None:
        state.base_phase_bias_banks = {}
    if state.base_code_bias_banks is None:
        state.base_code_bias_banks = {}
    if state.pending_tow == tow and state.pending_iod == iod:
        return
    reset_pending_corrections_with_policy(state, atmos_merge_policy)
    state.pending_tow = tow
    state.pending_iod = iod
    state.pending_udi_seconds = udi_seconds
    state.pending_orbit = {}
    state.pending_clock = {}
    state.pending_ura = {}
    state.pending_code_bias = {}
    state.pending_base_code_bias = {}
    state.pending_phase_bias = {}
    state.pending_phase_bias_source = {}
    state.pending_phase_discontinuity = {}
    state.pending_base_phase_bias = {}
    # pending_atmos is intentionally NOT reset here — the carry-forward
    # in reset_pending_corrections preserves STEC polynomial coefficients.


def flush_pending_corrections(
    state: CSSRDecoderState,
    gps_week: int | None,
    satellites: list[CSSRSatellite],
    flush_policy: str = COMPACT_SSR_FLUSH_POLICY_LAG_TOLERANT,
) -> list[CompactSSRCorrection]:
    if gps_week is None or state.pending_tow is None:
        reset_pending_corrections(state)
        return []
    orbit_map = state.pending_orbit or {}
    clock_map = state.pending_clock or {}
    ura_map = state.pending_ura or {}
    code_bias_map = state.pending_code_bias or {}
    phase_bias_map = state.pending_phase_bias or {}
    phase_discontinuity_map = state.pending_phase_discontinuity or {}
    atmos = state.pending_atmos or {}
    atmos_by_network = state.pending_atmos_by_network or {}
    extra_atmos_banks = [
        network_atmos
        for _network_id, network_atmos in sorted(atmos_by_network.items())
        if network_atmos and network_atmos != atmos
    ]
    sat_index = {satellite.sat: satellite for satellite in satellites}
    rows: list[CompactSSRCorrection] = []

    def atmos_int(atmos_tokens: dict[str, str], key: str) -> int | None:
        if key not in atmos_tokens:
            return None
        return int(atmos_tokens[key])

    # Include satellites from atmosphere data that may not have orbit/clock corrections
    atmos_sat_tokens: set[str] = set()
    for atmos_bank in ([atmos] if atmos else []) + extra_atmos_banks:
        for key in atmos_bank:
            # atmos keys like "atmos_stec_c00_tecu:G01" contain satellite identifiers after ':'
            if ":" in key:
                sat_part = key.split(":")[-1]
                if sat_part in sat_index:
                    atmos_sat_tokens.add(sat_part)
    if flush_policy == COMPACT_SSR_FLUSH_POLICY_LAG_TOLERANT:
        sat_tokens = (
            set(orbit_map)
            | set(clock_map)
            | set(ura_map)
            | set(code_bias_map)
            | set(phase_bias_map)
            | atmos_sat_tokens
        )
    elif flush_policy == COMPACT_SSR_FLUSH_POLICY_ORBIT_OR_CLOCK_ONLY:
        sat_tokens = set(orbit_map) | set(clock_map)
    elif flush_policy == COMPACT_SSR_FLUSH_POLICY_ORBIT_AND_CLOCK_ONLY:
        sat_tokens = set(orbit_map) & set(clock_map)
    else:
        raise ValueError(f"unsupported Compact SSR flush policy: {flush_policy}")
    for sat_token in sorted(sat_tokens):
        satellite = sat_index.get(sat_token)
        if satellite is None:
            continue
        dx, dy, dz = orbit_map.get(sat_token, (0.0, 0.0, 0.0))
        dclock_m = clock_map.get(sat_token, 0.0)
        code_bias = code_bias_map.get(sat_token, {})
        phase_bias = phase_bias_map.get(sat_token, {})
        phase_discontinuity = phase_discontinuity_map.get(sat_token, {})
        rows.append(
            CompactSSRCorrection(
                week=gps_week,
                tow=float(state.pending_tow),
                system=satellite.system,
                prn=satellite.prn,
                dx=dx,
                dy=dy,
                dz=dz,
                dclock_m=dclock_m,
                ura_sigma_m=ura_map.get(sat_token),
                code_bias_m=dict(code_bias) if code_bias else None,
                phase_bias_m=dict(phase_bias) if phase_bias else None,
                phase_discontinuity=(
                    dict(phase_discontinuity) if phase_discontinuity else None
                ),
                bias_network_id=state.pending_bias_network_id,
                atmos_network_id=atmos_int(atmos, "atmos_network_id"),
                atmos_trop_avail=atmos_int(atmos, "atmos_trop_avail"),
                atmos_stec_avail=atmos_int(atmos, "atmos_stec_avail"),
                atmos_grid_count=atmos_int(atmos, "atmos_grid_count"),
                atmos_selected_satellites=(
                    atmos_int(atmos, "atmos_selected_satellites")
                ),
                atmos_tokens=dict(atmos) if atmos else None,
            )
        )
        for network_atmos in extra_atmos_banks:
            rows.append(
                CompactSSRCorrection(
                    week=gps_week,
                    tow=float(state.pending_tow),
                    system=satellite.system,
                    prn=satellite.prn,
                    dx=0.0,
                    dy=0.0,
                    dz=0.0,
                    dclock_m=0.0,
                    atmos_network_id=atmos_int(network_atmos, "atmos_network_id"),
                    atmos_trop_avail=atmos_int(network_atmos, "atmos_trop_avail"),
                    atmos_stec_avail=atmos_int(network_atmos, "atmos_stec_avail"),
                    atmos_grid_count=atmos_int(network_atmos, "atmos_grid_count"),
                    atmos_selected_satellites=atmos_int(
                        network_atmos, "atmos_selected_satellites"
                    ),
                    atmos_tokens=dict(network_atmos),
                )
            )
    reset_pending_corrections(state)
    return rows


def decode_cssr_orbit_message(
    subframe: L6Subframe,
    payload: bytes,
    bit_offset: int,
    state: CSSRDecoderState,
    atmos_merge_policy: str = COMPACT_ATMOS_MERGE_POLICY_STEC_COEFF_CARRY,
) -> tuple[CSSRMessage, int]:
    mask = require_mask_state(state, CSSR_SUBTYPE_ORBIT)
    header, bit_offset = decode_cssr_header(payload, bit_offset, CSSR_SUBTYPE_ORBIT, state)
    if int(header["iod"]) != mask.iod:
        raise ValueError(
            f"CSSR subtype 2 iod mismatch: mask={mask.iod} message={int(header['iod'])}"
        )
    ensure_pending_epoch(
        state,
        int(header["tow"]),
        int(header["iod"]),
        float(header["udi_seconds"]),
        atmos_merge_policy,
    )
    assert state.pending_orbit is not None
    for satellite in mask.satellites:
        orbit_iode_bits = 10 if satellite.system == "E" else 8
        bit_offset += orbit_iode_bits
        dx, bit_offset = decode_scaled_signed(payload, bit_offset, 15, 0.0016)
        dy, bit_offset = decode_scaled_signed(payload, bit_offset, 13, 0.0064)
        dz, bit_offset = decode_scaled_signed(payload, bit_offset, 13, 0.0064)
        state.pending_orbit[satellite.sat] = (dx, dy, dz)
    state.message_index += 1
    return (
        CSSRMessage(
            subframe_index=subframe.index,
            message_index=state.message_index,
            ctype=CSSR_TYPE,
            subtype=CSSR_SUBTYPE_ORBIT,
            tow=int(header["tow"]),
            sync=bool(header["sync"]),
            iod=int(header["iod"]),
            udi_seconds=float(header["udi_seconds"]),
            message_bits=bit_offset,
            detail=f"satellites={mask.satellite_count}",
        ),
        bit_offset,
    )


def decode_cssr_clock_message(
    subframe: L6Subframe,
    payload: bytes,
    bit_offset: int,
    state: CSSRDecoderState,
    gps_week: int | None,
    flush_policy: str = COMPACT_SSR_FLUSH_POLICY_LAG_TOLERANT,
    atmos_merge_policy: str = COMPACT_ATMOS_MERGE_POLICY_STEC_COEFF_CARRY,
) -> tuple[CSSRMessage, list[CompactSSRCorrection], int]:
    mask = require_mask_state(state, CSSR_SUBTYPE_CLOCK)
    header, bit_offset = decode_cssr_header(payload, bit_offset, CSSR_SUBTYPE_CLOCK, state)
    if int(header["iod"]) != mask.iod:
        raise ValueError(
            f"CSSR subtype 3 iod mismatch: mask={mask.iod} message={int(header['iod'])}"
        )
    ensure_pending_epoch(
        state,
        int(header["tow"]),
        int(header["iod"]),
        float(header["udi_seconds"]),
        atmos_merge_policy,
    )
    assert state.pending_clock is not None
    for satellite in mask.satellites:
        dclock_m, bit_offset = decode_scaled_signed(payload, bit_offset, 15, 0.0016)
        state.pending_clock[satellite.sat] = dclock_m
    corrections: list[CompactSSRCorrection] = []
    if not bool(header["sync"]):
        corrections = flush_pending_corrections(state, gps_week, mask.satellites, flush_policy)
    state.message_index += 1
    return (
        CSSRMessage(
            subframe_index=subframe.index,
            message_index=state.message_index,
            ctype=CSSR_TYPE,
            subtype=CSSR_SUBTYPE_CLOCK,
            tow=int(header["tow"]),
            sync=bool(header["sync"]),
            iod=int(header["iod"]),
            udi_seconds=float(header["udi_seconds"]),
            message_bits=bit_offset,
            detail=f"satellites={mask.satellite_count}",
            correction_count=len(corrections),
        ),
        corrections,
        bit_offset,
    )


def decode_cssr_code_bias_message(
    subframe: L6Subframe,
    payload: bytes,
    bit_offset: int,
    state: CSSRDecoderState,
    gps_week: int | None,
    flush_policy: str = COMPACT_SSR_FLUSH_POLICY_LAG_TOLERANT,
    atmos_merge_policy: str = COMPACT_ATMOS_MERGE_POLICY_STEC_COEFF_CARRY,
    code_bias_composition_policy: str = COMPACT_CODE_BIAS_COMPOSITION_POLICY_DIRECT,
) -> tuple[CSSRMessage, list[CompactSSRCorrection], int]:
    mask = require_mask_state(state, CSSR_SUBTYPE_CODE_BIAS)
    header, bit_offset = decode_cssr_header(payload, bit_offset, CSSR_SUBTYPE_CODE_BIAS, state)
    if int(header["iod"]) != mask.iod:
        raise ValueError(
            f"CSSR subtype 4 iod mismatch: mask={mask.iod} message={int(header['iod'])}"
        )
    ensure_pending_epoch(
        state,
        int(header["tow"]),
        int(header["iod"]),
        float(header["udi_seconds"]),
        atmos_merge_policy,
    )
    assert state.pending_code_bias is not None
    assert state.pending_base_code_bias is not None
    mapped_count = 0
    for satellite in mask.satellites:
        satellite_biases = state.pending_code_bias.setdefault(satellite.sat, {})
        for signal_slot in satellite.signal_slots:
            bias_m, bit_offset = decode_scaled_signed(payload, bit_offset, 11, 0.02)
            signal_id = cssr_signal_slot_to_rtcm_id(satellite.system_id, signal_slot)
            if signal_id == 0 or not math.isfinite(bias_m):
                continue
            satellite_biases[signal_id] = bias_m
            state.pending_base_code_bias.setdefault(satellite.sat, {})[signal_id] = bias_m
            store_base_code_bias_bank_entry(
                state,
                int(header["tow"]),
                satellite.sat,
                signal_id,
                bias_m,
            )
            mapped_count += 1
    corrections: list[CompactSSRCorrection] = []
    if not bool(header["sync"]):
        corrections = flush_pending_corrections(state, gps_week, mask.satellites, flush_policy)
    state.message_index += 1
    return (
        CSSRMessage(
            subframe_index=subframe.index,
            message_index=state.message_index,
            ctype=CSSR_TYPE,
            subtype=CSSR_SUBTYPE_CODE_BIAS,
            tow=int(header["tow"]),
            sync=bool(header["sync"]),
            iod=int(header["iod"]),
            udi_seconds=float(header["udi_seconds"]),
            message_bits=bit_offset,
            detail=f"signals={mask.signal_count} mapped={mapped_count}",
            correction_count=len(corrections),
        ),
        corrections,
        bit_offset,
    )


def decode_cssr_code_phase_bias_message(
    subframe: L6Subframe,
    payload: bytes,
    bit_offset: int,
    state: CSSRDecoderState,
    gps_week: int | None,
    flush_policy: str = COMPACT_SSR_FLUSH_POLICY_LAG_TOLERANT,
    atmos_merge_policy: str = COMPACT_ATMOS_MERGE_POLICY_STEC_COEFF_CARRY,
    phase_bias_merge_policy: str = COMPACT_PHASE_BIAS_MERGE_POLICY_LATEST_UNION,
    phase_bias_source_policy: str = COMPACT_PHASE_BIAS_SOURCE_POLICY_ARRIVAL_ORDER,
    code_bias_composition_policy: str = COMPACT_CODE_BIAS_COMPOSITION_POLICY_DIRECT,
    code_bias_bank_policy: str = COMPACT_CODE_BIAS_BANK_POLICY_PENDING_EPOCH,
    phase_bias_composition_policy: str = COMPACT_PHASE_BIAS_COMPOSITION_POLICY_DIRECT,
    phase_bias_bank_policy: str = COMPACT_PHASE_BIAS_BANK_POLICY_PENDING_EPOCH,
    bias_row_materialization_policy: str = COMPACT_BIAS_ROW_MATERIALIZATION_POLICY_OVERLAP_ONLY,
    row_construction_policy: str = COMPACT_ROW_CONSTRUCTION_POLICY_INDEPENDENT,
) -> tuple[CSSRMessage, list[CompactSSRCorrection], int]:
    mask = require_mask_state(state, CSSR_SUBTYPE_CODE_PHASE_BIAS)
    header, bit_offset = decode_cssr_header(payload, bit_offset, CSSR_SUBTYPE_CODE_PHASE_BIAS, state)
    if int(header["iod"]) != mask.iod:
        raise ValueError(
            f"CSSR subtype 6 iod mismatch: mask={mask.iod} message={int(header['iod'])}"
        )
    code_bias_exists = bool(read_bits(payload, bit_offset, 1))
    bit_offset += 1
    phase_bias_exists = bool(read_bits(payload, bit_offset, 1))
    bit_offset += 1
    network_bias_correction = bool(read_bits(payload, bit_offset, 1))
    bit_offset += 1
    network_id: int | None = None
    selected_mask = (1 << mask.satellite_count) - 1
    if network_bias_correction:
        network_id = read_bits(payload, bit_offset, 5)
        bit_offset += 5
        selected_mask = read_bits(payload, bit_offset, mask.satellite_count)
        bit_offset += mask.satellite_count

    ensure_pending_epoch(
        state,
        int(header["tow"]),
        int(header["iod"]),
        float(header["udi_seconds"]),
        atmos_merge_policy,
    )
    assert state.pending_code_bias is not None
    assert state.pending_base_code_bias is not None
    assert state.pending_phase_bias is not None
    assert state.pending_phase_bias_source is not None
    assert state.pending_phase_discontinuity is not None
    assert state.pending_base_phase_bias is not None
    state.pending_bias_network_id = network_id if network_bias_correction else None
    selected_satellites = {
        satellite.sat
        for index, satellite in enumerate(mask.satellites)
        if ((selected_mask >> (mask.satellite_count - 1 - index)) & 1) != 0
    }
    prepare_pending_phase_bias_for_message(
        state,
        phase_bias_merge_policy,
        reset_message_scope=phase_bias_exists,
        selected_satellites=selected_satellites if network_bias_correction else None,
    )

    mapped_code = 0
    mapped_phase = 0
    selected_satellite_count = 0
    for index, satellite in enumerate(mask.satellites):
        if ((selected_mask >> (mask.satellite_count - 1 - index)) & 1) == 0:
            continue
        selected_satellite_count += 1
        satellite_code_biases = state.pending_code_bias.setdefault(satellite.sat, {})
        for signal_slot in satellite.signal_slots:
            signal_id = cssr_signal_slot_to_rtcm_id(satellite.system_id, signal_slot)
            if code_bias_exists:
                bias_m, bit_offset = decode_scaled_signed(payload, bit_offset, 11, 0.02)
                if signal_id != 0 and math.isfinite(bias_m) and network_bias_correction:
                    bias_m = compose_code_bias_value(
                        state=state,
                        tow=int(header["tow"]),
                        satellite_token=satellite.sat,
                        signal_id=signal_id,
                        network_bias_m=bias_m,
                        composition_policy=code_bias_composition_policy,
                        bank_policy=code_bias_bank_policy,
                    )
                if signal_id != 0 and math.isfinite(bias_m):
                    satellite_code_biases[signal_id] = bias_m
                    if not network_bias_correction:
                        state.pending_base_code_bias.setdefault(satellite.sat, {})[signal_id] = bias_m
                        store_base_code_bias_bank_entry(
                            state,
                            int(header["tow"]),
                            satellite.sat,
                            signal_id,
                            bias_m,
                        )
                    mapped_code += 1
            if phase_bias_exists:
                bias_m, bit_offset = decode_scaled_signed(payload, bit_offset, 15, 0.001)
                phase_discontinuity = read_bits(payload, bit_offset, 2)
                bit_offset += 2
                if signal_id != 0 and math.isfinite(bias_m) and network_bias_correction:
                    bias_m = compose_phase_bias_value(
                        state=state,
                        tow=int(header["tow"]),
                        satellite_token=satellite.sat,
                        signal_id=signal_id,
                        network_bias_m=bias_m,
                        composition_policy=phase_bias_composition_policy,
                        bank_policy=phase_bias_bank_policy,
                    )
                if (
                    signal_id != 0
                    and math.isfinite(bias_m)
                    and store_pending_phase_bias(
                        state,
                        satellite.sat,
                        signal_id,
                        bias_m,
                        phase_discontinuity,
                        source_subtype=CSSR_SUBTYPE_CODE_PHASE_BIAS,
                        source_policy=phase_bias_source_policy,
                    )
                ):
                    if not network_bias_correction:
                        state.pending_base_phase_bias.setdefault(satellite.sat, {})[signal_id] = (
                            state.pending_phase_bias[satellite.sat][signal_id]
                        )
                        store_base_phase_bias_bank_entry(
                            state,
                            int(header["tow"]),
                            satellite.sat,
                            signal_id,
                            state.pending_phase_bias[satellite.sat][signal_id],
                        )
                    mapped_phase += 1

    if network_bias_correction and code_bias_exists:
        materialize_missing_code_bias_rows(
            state,
            int(header["tow"]),
            mask_satellites=mask.satellites,
            selected_satellites=selected_satellites,
            row_materialization_policy=bias_row_materialization_policy,
            bank_policy=code_bias_bank_policy,
        )
    if network_bias_correction and phase_bias_exists:
        materialize_missing_phase_bias_rows(
            state,
            int(header["tow"]),
            mask_satellites=mask.satellites,
            selected_satellites=selected_satellites,
            row_materialization_policy=bias_row_materialization_policy,
            bank_policy=phase_bias_bank_policy,
        )

    if network_bias_correction and row_construction_policy != COMPACT_ROW_CONSTRUCTION_POLICY_INDEPENDENT:
        if row_construction_policy == COMPACT_ROW_CONSTRUCTION_POLICY_COUPLED_CODE_PHASE:
            apply_coupled_code_phase_construction(
                state,
                int(header["tow"]),
                selected_satellites=selected_satellites,
                code_bias_bank_policy=code_bias_bank_policy,
                phase_bias_bank_policy=phase_bias_bank_policy,
            )
        elif row_construction_policy == COMPACT_ROW_CONSTRUCTION_POLICY_ROW_FIRST_VALUE_SECOND:
            apply_row_first_value_second_construction(
                state,
                int(header["tow"]),
                selected_satellites=selected_satellites,
                code_bias_bank_policy=code_bias_bank_policy,
                phase_bias_bank_policy=phase_bias_bank_policy,
                code_bias_composition_policy=code_bias_composition_policy,
                phase_bias_composition_policy=phase_bias_composition_policy,
            )
        elif row_construction_policy == COMPACT_ROW_CONSTRUCTION_POLICY_NETWORK_ROW_DRIVEN:
            apply_network_row_driven_construction(
                state,
                selected_satellites=selected_satellites,
                mask_satellites=mask.satellites,
            )

    corrections: list[CompactSSRCorrection] = []
    if not bool(header["sync"]):
        corrections = flush_pending_corrections(state, gps_week, mask.satellites, flush_policy)
    state.message_index += 1
    return (
        CSSRMessage(
            subframe_index=subframe.index,
            message_index=state.message_index,
            ctype=CSSR_TYPE,
            subtype=CSSR_SUBTYPE_CODE_PHASE_BIAS,
            tow=int(header["tow"]),
            sync=bool(header["sync"]),
            iod=int(header["iod"]),
            udi_seconds=float(header["udi_seconds"]),
            message_bits=bit_offset,
            detail=(
                f"code={int(code_bias_exists)} phase={int(phase_bias_exists)} "
                f"net={network_id if network_id is not None else 0} sats={selected_satellite_count} "
                f"mapped_code={mapped_code} mapped_phase={mapped_phase}"
            ),
            correction_count=len(corrections),
        ),
        corrections,
        bit_offset,
    )


def decode_cssr_phase_bias_message(
    subframe: L6Subframe,
    payload: bytes,
    bit_offset: int,
    state: CSSRDecoderState,
    gps_week: int | None,
    flush_policy: str = COMPACT_SSR_FLUSH_POLICY_LAG_TOLERANT,
    atmos_merge_policy: str = COMPACT_ATMOS_MERGE_POLICY_STEC_COEFF_CARRY,
    phase_bias_merge_policy: str = COMPACT_PHASE_BIAS_MERGE_POLICY_LATEST_UNION,
    phase_bias_source_policy: str = COMPACT_PHASE_BIAS_SOURCE_POLICY_ARRIVAL_ORDER,
    phase_bias_composition_policy: str = COMPACT_PHASE_BIAS_COMPOSITION_POLICY_DIRECT,
    phase_bias_bank_policy: str = COMPACT_PHASE_BIAS_BANK_POLICY_PENDING_EPOCH,
) -> tuple[CSSRMessage, list[CompactSSRCorrection], int]:
    mask = require_mask_state(state, CSSR_SUBTYPE_PHASE_BIAS)
    header, bit_offset = decode_cssr_header(payload, bit_offset, CSSR_SUBTYPE_PHASE_BIAS, state)
    if int(header["iod"]) != mask.iod:
        raise ValueError(
            f"CSSR subtype 5 iod mismatch: mask={mask.iod} message={int(header['iod'])}"
        )

    ensure_pending_epoch(
        state,
        int(header["tow"]),
        int(header["iod"]),
        float(header["udi_seconds"]),
        atmos_merge_policy,
    )
    assert state.pending_phase_bias is not None
    assert state.pending_phase_bias_source is not None
    assert state.pending_phase_discontinuity is not None
    assert state.pending_base_phase_bias is not None
    prepare_pending_phase_bias_for_message(
        state,
        phase_bias_merge_policy,
        reset_message_scope=True,
    )

    mapped_phase = 0
    for satellite in mask.satellites:
        for signal_slot in satellite.signal_slots:
            signal_id = cssr_signal_slot_to_rtcm_id(satellite.system_id, signal_slot)
            bias_m, bit_offset = decode_scaled_signed(payload, bit_offset, 15, 0.001)
            phase_discontinuity = read_bits(payload, bit_offset, 2)
            bit_offset += 2
            if (
                signal_id != 0
                and math.isfinite(bias_m)
                and store_pending_phase_bias(
                    state,
                    satellite.sat,
                    signal_id,
                    bias_m,
                    phase_discontinuity,
                    source_subtype=CSSR_SUBTYPE_PHASE_BIAS,
                    source_policy=phase_bias_source_policy,
                )
            ):
                state.pending_base_phase_bias.setdefault(satellite.sat, {})[signal_id] = (
                    state.pending_phase_bias[satellite.sat][signal_id]
                )
                store_base_phase_bias_bank_entry(
                    state,
                    int(header["tow"]),
                    satellite.sat,
                    signal_id,
                    state.pending_phase_bias[satellite.sat][signal_id],
                )
                mapped_phase += 1

    corrections: list[CompactSSRCorrection] = []
    if not bool(header["sync"]):
        corrections = flush_pending_corrections(state, gps_week, mask.satellites, flush_policy)
    state.message_index += 1
    return (
        CSSRMessage(
            subframe_index=subframe.index,
            message_index=state.message_index,
            ctype=CSSR_TYPE,
            subtype=CSSR_SUBTYPE_PHASE_BIAS,
            tow=int(header["tow"]),
            sync=bool(header["sync"]),
            iod=int(header["iod"]),
            udi_seconds=float(header["udi_seconds"]),
            message_bits=bit_offset,
            detail=f"signals={mask.signal_count} mapped_phase={mapped_phase}",
            correction_count=len(corrections),
        ),
        corrections,
        bit_offset,
    )


def decode_cssr_ura_message(
    subframe: L6Subframe,
    payload: bytes,
    bit_offset: int,
    state: CSSRDecoderState,
    gps_week: int | None,
    flush_policy: str = COMPACT_SSR_FLUSH_POLICY_LAG_TOLERANT,
    atmos_merge_policy: str = COMPACT_ATMOS_MERGE_POLICY_STEC_COEFF_CARRY,
) -> tuple[CSSRMessage, list[CompactSSRCorrection], int]:
    mask = require_mask_state(state, CSSR_SUBTYPE_URA)
    header, bit_offset = decode_cssr_header(payload, bit_offset, CSSR_SUBTYPE_URA, state)
    if int(header["iod"]) != mask.iod:
        raise ValueError(
            f"CSSR subtype 7 iod mismatch: mask={mask.iod} message={int(header['iod'])}"
        )
    ensure_pending_epoch(
        state,
        int(header["tow"]),
        int(header["iod"]),
        float(header["udi_seconds"]),
        atmos_merge_policy,
    )
    assert state.pending_ura is not None
    for satellite in mask.satellites:
        ura_index = read_bits(payload, bit_offset, 6)
        bit_offset += 6
        state.pending_ura[satellite.sat] = ura_meters_from_ssr_index(ura_index)
    corrections: list[CompactSSRCorrection] = []
    if not bool(header["sync"]):
        corrections = flush_pending_corrections(state, gps_week, mask.satellites, flush_policy)
    state.message_index += 1
    return (
        CSSRMessage(
            subframe_index=subframe.index,
            message_index=state.message_index,
            ctype=CSSR_TYPE,
            subtype=CSSR_SUBTYPE_URA,
            tow=int(header["tow"]),
            sync=bool(header["sync"]),
            iod=int(header["iod"]),
            udi_seconds=float(header["udi_seconds"]),
            message_bits=bit_offset,
            detail=f"satellites={mask.satellite_count}",
            correction_count=len(corrections),
        ),
        corrections,
        bit_offset,
    )


def decode_cssr_stec_message(
    subframe: L6Subframe,
    payload: bytes,
    bit_offset: int,
    state: CSSRDecoderState,
    atmos_merge_policy: str = COMPACT_ATMOS_MERGE_POLICY_STEC_COEFF_CARRY,
    atmos_subtype_merge_policy: str = COMPACT_ATMOS_SUBTYPE_MERGE_POLICY_UNION,
) -> tuple[CSSRMessage, int]:
    mask = require_mask_state(state, CSSR_SUBTYPE_STEC)
    header, bit_offset = decode_cssr_header(payload, bit_offset, CSSR_SUBTYPE_STEC, state)
    if int(header["iod"]) != mask.iod:
        raise ValueError(
            f"CSSR subtype 8 iod mismatch: mask={mask.iod} message={int(header['iod'])}"
        )

    stec_type = read_bits(payload, bit_offset, 2)
    bit_offset += 2
    network_id = read_bits(payload, bit_offset, 5)
    bit_offset += 5
    selected_mask = read_bits(payload, bit_offset, mask.satellite_count)
    bit_offset += mask.satellite_count

    atmos_tokens: dict[str, str] = {
        "atmos_network_id": str(network_id),
        "atmos_trop_avail": "0",
        "atmos_stec_avail": "1",
        "atmos_grid_count": "0",
    }

    selected_satellites = 0
    for index, satellite in enumerate(mask.satellites):
        if ((selected_mask >> (mask.satellite_count - 1 - index)) & 1) == 0:
            continue
        selected_satellites += 1
        sat_key = satellite.sat
        stec_quality = read_bits(payload, bit_offset, 6)
        atmos_tokens[f"atmos_stec_quality:{sat_key}"] = str(stec_quality)
        bit_offset += 6
        atmos_tokens[f"atmos_stec_type:{sat_key}"] = str(stec_type)
        stec_c00_tecu, bit_offset = decode_scaled_signed(payload, bit_offset, 14, 0.05)
        if math.isfinite(stec_c00_tecu):
            atmos_tokens[f"atmos_stec_c00_tecu:{sat_key}"] = f"{stec_c00_tecu:.6f}"
        if stec_type > 0:
            stec_c01_tecu_per_deg, bit_offset = decode_scaled_signed(payload, bit_offset, 12, 0.02)
            stec_c10_tecu_per_deg, bit_offset = decode_scaled_signed(payload, bit_offset, 12, 0.02)
            if math.isfinite(stec_c01_tecu_per_deg):
                atmos_tokens[f"atmos_stec_c01_tecu_per_deg:{sat_key}"] = (
                    f"{stec_c01_tecu_per_deg:.6f}"
                )
            if math.isfinite(stec_c10_tecu_per_deg):
                atmos_tokens[f"atmos_stec_c10_tecu_per_deg:{sat_key}"] = (
                    f"{stec_c10_tecu_per_deg:.6f}"
                )
        if stec_type > 1:
            stec_c11_tecu_per_deg2, bit_offset = decode_scaled_signed(payload, bit_offset, 10, 0.02)
            if math.isfinite(stec_c11_tecu_per_deg2):
                atmos_tokens[f"atmos_stec_c11_tecu_per_deg2:{sat_key}"] = (
                    f"{stec_c11_tecu_per_deg2:.6f}"
                )
        if stec_type > 2:
            stec_c02_tecu_per_deg2, bit_offset = decode_scaled_signed(payload, bit_offset, 8, 0.005)
            stec_c20_tecu_per_deg2, bit_offset = decode_scaled_signed(payload, bit_offset, 8, 0.005)
            if math.isfinite(stec_c02_tecu_per_deg2):
                atmos_tokens[f"atmos_stec_c02_tecu_per_deg2:{sat_key}"] = (
                    f"{stec_c02_tecu_per_deg2:.6f}"
                )
            if math.isfinite(stec_c20_tecu_per_deg2):
                atmos_tokens[f"atmos_stec_c20_tecu_per_deg2:{sat_key}"] = (
                    f"{stec_c20_tecu_per_deg2:.6f}"
                )

    ensure_pending_epoch(
        state,
        int(header["tow"]),
        int(header["iod"]),
        float(header["udi_seconds"]),
        atmos_merge_policy,
    )
    atmos_tokens["atmos_selected_satellites"] = str(selected_satellites)
    merge_pending_atmos(
        state,
        atmos_tokens,
        atmos_merge_policy,
        atmos_subtype_merge_policy,
        CSSR_SUBTYPE_STEC,
    )

    state.message_index += 1
    return (
        CSSRMessage(
            subframe_index=subframe.index,
            message_index=state.message_index,
            ctype=CSSR_TYPE,
            subtype=CSSR_SUBTYPE_STEC,
            tow=int(header["tow"]),
            sync=bool(header["sync"]),
            iod=int(header["iod"]),
            udi_seconds=float(header["udi_seconds"]),
            message_bits=bit_offset,
            detail=f"network={network_id} stec_type={stec_type} sats={selected_satellites}",
        ),
        bit_offset,
    )


def decode_cssr_gridded_message(
    subframe: L6Subframe,
    payload: bytes,
    bit_offset: int,
    state: CSSRDecoderState,
    atmos_merge_policy: str = COMPACT_ATMOS_MERGE_POLICY_STEC_COEFF_CARRY,
    atmos_subtype_merge_policy: str = COMPACT_ATMOS_SUBTYPE_MERGE_POLICY_UNION,
) -> tuple[CSSRMessage, int]:
    mask = require_mask_state(state, CSSR_SUBTYPE_GRIDDED)
    header, bit_offset = decode_cssr_header(payload, bit_offset, CSSR_SUBTYPE_GRIDDED, state)
    if int(header["iod"]) != mask.iod:
        raise ValueError(
            f"CSSR subtype 9 iod mismatch: mask={mask.iod} message={int(header['iod'])}"
        )

    trop_type = read_bits(payload, bit_offset, 2)
    bit_offset += 2
    stec_residual_range = read_bits(payload, bit_offset, 1)
    bit_offset += 1
    network_id = read_bits(payload, bit_offset, 5)
    bit_offset += 5
    selected_mask = read_bits(payload, bit_offset, mask.satellite_count)
    bit_offset += mask.satellite_count
    trop_quality = read_bits(payload, bit_offset, 6)
    bit_offset += 6
    grid_count = read_bits(payload, bit_offset, 6)
    bit_offset += 6

    selected_satellites: list[CSSRSatellite] = []
    for index, satellite in enumerate(mask.satellites):
        if ((selected_mask >> (mask.satellite_count - 1 - index)) & 1) == 0:
            continue
        selected_satellites.append(satellite)

    trop_hs_residuals_m: list[str] = []
    trop_wet_residuals_m: list[str] = []
    stec_residuals_tecu: dict[str, list[str]] = {
        satellite.sat: [] for satellite in selected_satellites
    }
    stec_residual_bits = 7 if stec_residual_range == 0 else 16

    for _ in range(grid_count):
        trop_hs_m, bit_offset = decode_scaled_signed(payload, bit_offset, 9, 0.004)
        trop_wet_m, bit_offset = decode_scaled_signed(payload, bit_offset, 8, 0.004)
        trop_hs_residuals_m.append("nan" if not math.isfinite(trop_hs_m) else f"{trop_hs_m:.6f}")
        trop_wet_residuals_m.append("nan" if not math.isfinite(trop_wet_m) else f"{trop_wet_m:.6f}")
        for satellite in selected_satellites:
            residual_tecu, bit_offset = decode_scaled_signed(
                payload,
                bit_offset,
                stec_residual_bits,
                0.04,
            )
            stec_residuals_tecu[satellite.sat].append(
                "nan" if not math.isfinite(residual_tecu) else f"{residual_tecu:.6f}"
            )

    atmos_tokens: dict[str, str] = {
        "atmos_network_id": str(network_id),
        "atmos_trop_avail": "1" if trop_type != 0 else "0",
        "atmos_stec_avail": "2" if selected_satellites else "0",
        "atmos_grid_count": str(grid_count),
        "atmos_trop_quality": str(trop_quality),
        "atmos_trop_type": str(trop_type),
        "atmos_trop_hs_residuals_m": ";".join(trop_hs_residuals_m),
        "atmos_trop_wet_residuals_m": ";".join(trop_wet_residuals_m),
        "atmos_stec_residual_range": str(stec_residual_range),
        "atmos_selected_satellites": str(len(selected_satellites)),
    }
    for satellite in selected_satellites:
        sat_key = satellite.sat
        atmos_tokens[f"atmos_stec_residual_size:{sat_key}"] = str(stec_residual_range)
        atmos_tokens[f"atmos_stec_residuals_tecu:{sat_key}"] = ";".join(stec_residuals_tecu[sat_key])

    ensure_pending_epoch(
        state,
        int(header["tow"]),
        int(header["iod"]),
        float(header["udi_seconds"]),
        atmos_merge_policy,
    )
    merge_pending_atmos(
        state,
        atmos_tokens,
        atmos_merge_policy,
        atmos_subtype_merge_policy,
        CSSR_SUBTYPE_GRIDDED,
    )

    state.message_index += 1
    return (
        CSSRMessage(
            subframe_index=subframe.index,
            message_index=state.message_index,
            ctype=CSSR_TYPE,
            subtype=CSSR_SUBTYPE_GRIDDED,
            tow=int(header["tow"]),
            sync=bool(header["sync"]),
            iod=int(header["iod"]),
            udi_seconds=float(header["udi_seconds"]),
            message_bits=bit_offset,
            detail=(
                f"network={network_id} grids={grid_count} "
                f"trop={trop_type} sats={len(selected_satellites)} "
                f"residual_bits={stec_residual_bits}"
            ),
        ),
        bit_offset,
    )


def flush_service_info_packets(
    state: CSSRDecoderState,
    current_subframe_index: int,
) -> list[CSSRServiceInfoPacket]:
    if not state.service_info_chunks:
        return []
    concatenated = b"".join(chunk for _subframe, chunk, _bits in state.service_info_chunks)
    total_bits = sum(bits for _subframe, _chunk, bits in state.service_info_chunks)
    first_subframe = state.service_info_chunks[0][0]
    state.service_packet_index += 1
    packet = CSSRServiceInfoPacket(
        packet_index=state.service_packet_index,
        first_subframe_index=first_subframe,
        last_subframe_index=current_subframe_index,
        chunk_count=len(state.service_info_chunks),
        total_bits=total_bits,
        packet_bytes=concatenated,
    )
    state.service_info_chunks = None
    return [packet]


def decode_cssr_service_info_message(
    subframe: L6Subframe,
    payload: bytes,
    bit_offset: int,
    state: CSSRDecoderState,
) -> tuple[CSSRMessage, list[CSSRServiceInfoPacket], int]:
    sync = bool(read_bits(payload, bit_offset, 1))
    bit_offset += 1
    message_counter = read_bits(payload, bit_offset, 3)
    bit_offset += 3
    data_size = read_bits(payload, bit_offset, 2)
    bit_offset += 2
    payload_bits = 40 * (data_size + 1)
    auxiliary_frame_data = read_bitpacked_bytes(payload, bit_offset, payload_bits)
    bit_offset += payload_bits

    if message_counter == 0 or state.service_info_chunks is None:
        state.service_info_chunks = []
    state.service_info_chunks.append((subframe.index, auxiliary_frame_data, payload_bits))

    packets: list[CSSRServiceInfoPacket] = []
    if not sync:
        packets = flush_service_info_packets(state, subframe.index)

    state.message_index += 1
    return (
        CSSRMessage(
            subframe_index=subframe.index,
            message_index=state.message_index,
            ctype=CSSR_TYPE,
            subtype=CSSR_SUBTYPE_SERVICE_INFO,
            tow=0,
            sync=sync,
            iod=int(message_counter),
            udi_seconds=0.0,
            message_bits=bit_offset,
            detail=(
                f"info_counter={message_counter} chunksize_bits={payload_bits} "
                f"chunk_bytes={len(auxiliary_frame_data)}"
            ),
            correction_count=len(packets),
        ),
        packets,
        bit_offset,
    )


def decode_cssr_combined_message(
    subframe: L6Subframe,
    payload: bytes,
    bit_offset: int,
    state: CSSRDecoderState,
    gps_week: int | None,
    flush_policy: str = COMPACT_SSR_FLUSH_POLICY_LAG_TOLERANT,
    atmos_merge_policy: str = COMPACT_ATMOS_MERGE_POLICY_STEC_COEFF_CARRY,
) -> tuple[CSSRMessage, list[CompactSSRCorrection], int]:
    mask = require_mask_state(state, CSSR_SUBTYPE_COMBINED)
    header, bit_offset = decode_cssr_header(payload, bit_offset, CSSR_SUBTYPE_COMBINED, state)
    if int(header["iod"]) != mask.iod:
        raise ValueError(
            f"CSSR subtype 11 iod mismatch: mask={mask.iod} message={int(header['iod'])}"
        )
    flg_orbit = bool(read_bits(payload, bit_offset, 1))
    bit_offset += 1
    flg_clock = bool(read_bits(payload, bit_offset, 1))
    bit_offset += 1
    flg_net = bool(read_bits(payload, bit_offset, 1))
    bit_offset += 1
    network_id = None
    selected_mask = 0
    if flg_net:
        network_id = read_bits(payload, bit_offset, 5)
        bit_offset += 5
        selected_mask = read_bits(payload, bit_offset, mask.satellite_count)
        bit_offset += mask.satellite_count
    ensure_pending_epoch(
        state,
        int(header["tow"]),
        int(header["iod"]),
        float(header["udi_seconds"]),
        atmos_merge_policy,
    )
    assert state.pending_orbit is not None
    assert state.pending_clock is not None
    selected_satellites = 0
    for index, satellite in enumerate(mask.satellites):
        if flg_net and ((selected_mask >> (mask.satellite_count - 1 - index)) & 1) == 0:
            continue
        selected_satellites += 1
        if flg_orbit:
            orbit_iode_bits = 10 if satellite.system == "E" else 8
            bit_offset += orbit_iode_bits
            dx, bit_offset = decode_scaled_signed(payload, bit_offset, 15, 0.0016)
            dy, bit_offset = decode_scaled_signed(payload, bit_offset, 13, 0.0064)
            dz, bit_offset = decode_scaled_signed(payload, bit_offset, 13, 0.0064)
        else:
            dx = dy = dz = 0.0
        if flg_clock:
            dclock_m, bit_offset = decode_scaled_signed(payload, bit_offset, 15, 0.0016)
        else:
            dclock_m = 0.0
        state.pending_orbit[satellite.sat] = (dx, dy, dz)
        state.pending_clock[satellite.sat] = dclock_m
    corrections: list[CompactSSRCorrection] = []
    if not bool(header["sync"]):
        corrections = flush_pending_corrections(state, gps_week, mask.satellites, flush_policy)
    state.message_index += 1
    message = CSSRMessage(
        subframe_index=subframe.index,
        message_index=state.message_index,
        ctype=CSSR_TYPE,
        subtype=CSSR_SUBTYPE_COMBINED,
        tow=int(header["tow"]),
        sync=bool(header["sync"]),
        iod=int(header["iod"]),
        udi_seconds=float(header["udi_seconds"]),
        message_bits=bit_offset,
        detail=(
            f"orbit={int(flg_orbit)} clock={int(flg_clock)} "
            f"net={network_id if network_id is not None else 0} satellites={selected_satellites}"
        ),
        correction_count=len(corrections),
    )
    return message, corrections, bit_offset


def decode_cssr_atmos_message(
    subframe: L6Subframe,
    payload: bytes,
    bit_offset: int,
    state: CSSRDecoderState,
    atmos_merge_policy: str = COMPACT_ATMOS_MERGE_POLICY_STEC_COEFF_CARRY,
    atmos_subtype_merge_policy: str = COMPACT_ATMOS_SUBTYPE_MERGE_POLICY_UNION,
) -> tuple[CSSRMessage, int]:
    mask = require_mask_state(state, CSSR_SUBTYPE_ATMOS)
    header, bit_offset = decode_cssr_header(payload, bit_offset, CSSR_SUBTYPE_ATMOS, state)
    if int(header["iod"]) != mask.iod:
        raise ValueError(
            f"CSSR subtype 12 iod mismatch: mask={mask.iod} message={int(header['iod'])}"
        )

    trop_avail = read_bits(payload, bit_offset, 2)
    bit_offset += 2
    stec_avail = read_bits(payload, bit_offset, 2)
    bit_offset += 2
    network_id = read_bits(payload, bit_offset, 5)
    bit_offset += 5
    grid_count = read_bits(payload, bit_offset, 6)
    bit_offset += 6

    atmos_tokens: dict[str, str] = {
        "atmos_network_id": str(network_id),
        "atmos_trop_avail": str(trop_avail),
        "atmos_stec_avail": str(stec_avail),
        "atmos_grid_count": str(grid_count),
    }

    selected_satellites = 0
    if trop_avail != 0:
        trop_quality = read_bits(payload, bit_offset, 6)
        atmos_tokens["atmos_trop_quality"] = str(trop_quality)
        bit_offset += 6
    if (trop_avail & 0x01) != 0:
        atmos_tokens["atmos_trop_source_subtype"] = "12"
        trop_type = read_bits(payload, bit_offset, 2)
        atmos_tokens["atmos_trop_type"] = str(trop_type)
        bit_offset += 2
        trop_t00_m, bit_offset = decode_scaled_signed(payload, bit_offset, 9, 0.004)
        if math.isfinite(trop_t00_m):
            atmos_tokens["atmos_trop_t00_m"] = f"{trop_t00_m:.6f}"
        if trop_type > 0:
            trop_t01_m_per_deg, bit_offset = decode_scaled_signed(payload, bit_offset, 7, 0.002)
            trop_t10_m_per_deg, bit_offset = decode_scaled_signed(payload, bit_offset, 7, 0.002)
            if math.isfinite(trop_t01_m_per_deg):
                atmos_tokens["atmos_trop_t01_m_per_deg"] = f"{trop_t01_m_per_deg:.6f}"
            if math.isfinite(trop_t10_m_per_deg):
                atmos_tokens["atmos_trop_t10_m_per_deg"] = f"{trop_t10_m_per_deg:.6f}"
        if trop_type > 1:
            trop_t11_m_per_deg2, bit_offset = decode_scaled_signed(payload, bit_offset, 7, 0.001)
            if math.isfinite(trop_t11_m_per_deg2):
                atmos_tokens["atmos_trop_t11_m_per_deg2"] = f"{trop_t11_m_per_deg2:.6f}"
    else:
        trop_type = -1
    if (trop_avail & 0x02) != 0:
        trop_size_index = read_bits(payload, bit_offset, 1)
        atmos_tokens["atmos_trop_residual_size"] = str(trop_size_index)
        bit_offset += 1
        trop_offset_m = read_bits(payload, bit_offset, 4) * 0.02
        atmos_tokens["atmos_trop_offset_m"] = f"{trop_offset_m:.6f}"
        bit_offset += 4
        trop_residual_bits = 6 if trop_size_index == 0 else 8
        trop_residuals_m: list[str] = []
        for _ in range(grid_count):
            trop_residual_m, bit_offset = decode_scaled_signed(payload, bit_offset, trop_residual_bits, 0.004)
            trop_residuals_m.append(
                "nan" if not math.isfinite(trop_residual_m) else f"{trop_residual_m:.6f}"
            )
        atmos_tokens["atmos_trop_residuals_m"] = ";".join(trop_residuals_m)
    if stec_avail != 0:
        selected_mask = read_bits(payload, bit_offset, mask.satellite_count)
        bit_offset += mask.satellite_count
        dstec_sizes = (4, 4, 5, 7)
        dstec_scales = (0.04, 0.12, 0.16, 0.24)
        for index, _satellite in enumerate(mask.satellites):
            if ((selected_mask >> (mask.satellite_count - 1 - index)) & 1) == 0:
                continue
            satellite = mask.satellites[index]
            selected_satellites += 1
            sat_key = satellite.sat
            stec_quality = read_bits(payload, bit_offset, 6)
            atmos_tokens[f"atmos_stec_quality:{sat_key}"] = str(stec_quality)
            bit_offset += 6
            if (stec_avail & 0x01) != 0:
                atmos_tokens[f"atmos_stec_source_subtype:{sat_key}"] = "12"
                stec_type = read_bits(payload, bit_offset, 2)
                atmos_tokens[f"atmos_stec_type:{sat_key}"] = str(stec_type)
                bit_offset += 2
                stec_c00_tecu, bit_offset = decode_scaled_signed(payload, bit_offset, 14, 0.05)
                if math.isfinite(stec_c00_tecu):
                    atmos_tokens[f"atmos_stec_c00_tecu:{sat_key}"] = f"{stec_c00_tecu:.6f}"
                if stec_type > 0:
                    stec_c01_tecu_per_deg, bit_offset = decode_scaled_signed(payload, bit_offset, 12, 0.02)
                    stec_c10_tecu_per_deg, bit_offset = decode_scaled_signed(payload, bit_offset, 12, 0.02)
                    if math.isfinite(stec_c01_tecu_per_deg):
                        atmos_tokens[f"atmos_stec_c01_tecu_per_deg:{sat_key}"] = (
                            f"{stec_c01_tecu_per_deg:.6f}"
                        )
                    if math.isfinite(stec_c10_tecu_per_deg):
                        atmos_tokens[f"atmos_stec_c10_tecu_per_deg:{sat_key}"] = (
                            f"{stec_c10_tecu_per_deg:.6f}"
                        )
                if stec_type > 1:
                    stec_c11_tecu_per_deg2, bit_offset = decode_scaled_signed(payload, bit_offset, 10, 0.02)
                    if math.isfinite(stec_c11_tecu_per_deg2):
                        atmos_tokens[f"atmos_stec_c11_tecu_per_deg2:{sat_key}"] = (
                            f"{stec_c11_tecu_per_deg2:.6f}"
                        )
                if stec_type > 2:
                    stec_c02_tecu_per_deg2, bit_offset = decode_scaled_signed(payload, bit_offset, 8, 0.005)
                    stec_c20_tecu_per_deg2, bit_offset = decode_scaled_signed(payload, bit_offset, 8, 0.005)
                    if math.isfinite(stec_c02_tecu_per_deg2):
                        atmos_tokens[f"atmos_stec_c02_tecu_per_deg2:{sat_key}"] = (
                            f"{stec_c02_tecu_per_deg2:.6f}"
                        )
                    if math.isfinite(stec_c20_tecu_per_deg2):
                        atmos_tokens[f"atmos_stec_c20_tecu_per_deg2:{sat_key}"] = (
                            f"{stec_c20_tecu_per_deg2:.6f}"
                        )
            if (stec_avail & 0x02) != 0:
                size_index = read_bits(payload, bit_offset, 2)
                atmos_tokens[f"atmos_stec_residual_size:{sat_key}"] = str(size_index)
                bit_offset += 2
                stec_residuals_tecu: list[str] = []
                for _ in range(grid_count):
                    stec_residual_tecu, bit_offset = decode_scaled_signed(
                        payload,
                        bit_offset,
                        dstec_sizes[size_index],
                        dstec_scales[size_index],
                    )
                    stec_residuals_tecu.append(
                        "nan" if not math.isfinite(stec_residual_tecu) else f"{stec_residual_tecu:.6f}"
                    )
                atmos_tokens[f"atmos_stec_residuals_tecu:{sat_key}"] = ";".join(stec_residuals_tecu)

    ensure_pending_epoch(
        state,
        int(header["tow"]),
        int(header["iod"]),
        float(header["udi_seconds"]),
        atmos_merge_policy,
    )
    atmos_tokens["atmos_selected_satellites"] = str(selected_satellites)
    merge_pending_atmos(
        state,
        atmos_tokens,
        atmos_merge_policy,
        atmos_subtype_merge_policy,
        CSSR_SUBTYPE_ATMOS,
    )

    state.message_index += 1
    return (
        CSSRMessage(
            subframe_index=subframe.index,
            message_index=state.message_index,
            ctype=CSSR_TYPE,
            subtype=CSSR_SUBTYPE_ATMOS,
            tow=int(header["tow"]),
            sync=bool(header["sync"]),
            iod=int(header["iod"]),
            udi_seconds=float(header["udi_seconds"]),
            message_bits=bit_offset,
            detail=(
                f"network={network_id} grids={grid_count} "
                f"trop={trop_avail} stec={stec_avail} sats={selected_satellites}"
            ),
        ),
        bit_offset,
    )


def decode_cssr_messages(
    subframes: list[L6Subframe],
    gps_week: int | None = None,
    flush_policy: str = COMPACT_SSR_FLUSH_POLICY_LAG_TOLERANT,
    atmos_merge_policy: str = COMPACT_ATMOS_MERGE_POLICY_STEC_COEFF_CARRY,
    atmos_subtype_merge_policy: str = COMPACT_ATMOS_SUBTYPE_MERGE_POLICY_UNION,
    phase_bias_merge_policy: str = COMPACT_PHASE_BIAS_MERGE_POLICY_LATEST_UNION,
    phase_bias_source_policy: str = COMPACT_PHASE_BIAS_SOURCE_POLICY_ARRIVAL_ORDER,
    code_bias_composition_policy: str = COMPACT_CODE_BIAS_COMPOSITION_POLICY_DIRECT,
    code_bias_bank_policy: str = COMPACT_CODE_BIAS_BANK_POLICY_PENDING_EPOCH,
    phase_bias_composition_policy: str = COMPACT_PHASE_BIAS_COMPOSITION_POLICY_DIRECT,
    phase_bias_bank_policy: str = COMPACT_PHASE_BIAS_BANK_POLICY_PENDING_EPOCH,
    bias_row_materialization_policy: str = COMPACT_BIAS_ROW_MATERIALIZATION_POLICY_OVERLAP_ONLY,
    row_construction_policy: str = COMPACT_ROW_CONSTRUCTION_POLICY_INDEPENDENT,
) -> tuple[list[CSSRMessage], list[CompactSSRCorrection], list[CSSRServiceInfoPacket]]:
    messages: list[CSSRMessage] = []
    corrections: list[CompactSSRCorrection] = []
    service_info_packets: list[CSSRServiceInfoPacket] = []
    state = CSSRDecoderState()
    for subframe in subframes:
        bit_offset = 0
        while bit_offset + 16 <= subframe.data_bits:
            ctype = read_bits(subframe.data_part, bit_offset, 12)
            if ctype == 0:
                break
            if ctype != CSSR_TYPE:
                break
            subtype = read_bits(subframe.data_part, bit_offset + 12, 4)
            message_start = bit_offset
            bit_offset += 16
            if subtype == CSSR_SUBTYPE_MASK:
                if state.mask is not None:
                    corrections.extend(
                        flush_pending_corrections(
                            state,
                            gps_week,
                            state.mask.satellites,
                            flush_policy,
                        )
                    )
                message, bit_offset = decode_cssr_mask_message(subframe, subframe.data_part, bit_offset, state)
            elif subtype == CSSR_SUBTYPE_ORBIT:
                message, bit_offset = decode_cssr_orbit_message(
                    subframe,
                    subframe.data_part,
                    bit_offset,
                    state,
                    atmos_merge_policy,
                )
            elif subtype == CSSR_SUBTYPE_CLOCK:
                message, message_corrections, bit_offset = decode_cssr_clock_message(
                    subframe,
                    subframe.data_part,
                    bit_offset,
                    state,
                    gps_week,
                    flush_policy,
                    atmos_merge_policy,
                )
                corrections.extend(message_corrections)
            elif subtype == CSSR_SUBTYPE_CODE_BIAS:
                message, message_corrections, bit_offset = decode_cssr_code_bias_message(
                    subframe,
                    subframe.data_part,
                    bit_offset,
                    state,
                    gps_week,
                    flush_policy,
                    atmos_merge_policy,
                    code_bias_composition_policy,
                )
                corrections.extend(message_corrections)
            elif subtype == CSSR_SUBTYPE_PHASE_BIAS:
                message, message_corrections, bit_offset = decode_cssr_phase_bias_message(
                    subframe,
                    subframe.data_part,
                    bit_offset,
                    state,
                    gps_week,
                    flush_policy,
                    atmos_merge_policy,
                    phase_bias_merge_policy,
                    phase_bias_source_policy,
                    phase_bias_composition_policy,
                    phase_bias_bank_policy,
                )
                corrections.extend(message_corrections)
            elif subtype == CSSR_SUBTYPE_CODE_PHASE_BIAS:
                message, message_corrections, bit_offset = decode_cssr_code_phase_bias_message(
                    subframe,
                    subframe.data_part,
                    bit_offset,
                    state,
                    gps_week,
                    flush_policy,
                    atmos_merge_policy,
                    phase_bias_merge_policy,
                    phase_bias_source_policy,
                    code_bias_composition_policy,
                    code_bias_bank_policy,
                    phase_bias_composition_policy,
                    phase_bias_bank_policy,
                    bias_row_materialization_policy,
                    row_construction_policy,
                )
                corrections.extend(message_corrections)
            elif subtype == CSSR_SUBTYPE_URA:
                message, message_corrections, bit_offset = decode_cssr_ura_message(
                    subframe,
                    subframe.data_part,
                    bit_offset,
                    state,
                    gps_week,
                    flush_policy,
                    atmos_merge_policy,
                )
                corrections.extend(message_corrections)
            elif subtype == CSSR_SUBTYPE_STEC:
                message, bit_offset = decode_cssr_stec_message(
                    subframe,
                    subframe.data_part,
                    bit_offset,
                    state,
                    atmos_merge_policy,
                    atmos_subtype_merge_policy,
                )
            elif subtype == CSSR_SUBTYPE_GRIDDED:
                message, bit_offset = decode_cssr_gridded_message(
                    subframe,
                    subframe.data_part,
                    bit_offset,
                    state,
                    atmos_merge_policy,
                    atmos_subtype_merge_policy,
                )
            elif subtype == CSSR_SUBTYPE_SERVICE_INFO:
                message, packets, bit_offset = decode_cssr_service_info_message(
                    subframe,
                    subframe.data_part,
                    bit_offset,
                    state,
                )
                service_info_packets.extend(packets)
            elif subtype == CSSR_SUBTYPE_COMBINED:
                message, message_corrections, bit_offset = decode_cssr_combined_message(
                    subframe,
                    subframe.data_part,
                    bit_offset,
                    state,
                    gps_week,
                    flush_policy,
                    atmos_merge_policy,
                )
                corrections.extend(message_corrections)
            elif subtype == CSSR_SUBTYPE_ATMOS:
                message, bit_offset = decode_cssr_atmos_message(
                    subframe,
                    subframe.data_part,
                    bit_offset,
                    state,
                    atmos_merge_policy,
                    atmos_subtype_merge_policy,
                )
            else:
                break
            message.message_bits = bit_offset - message_start
            messages.append(message)
    if state.mask is not None:
        corrections.extend(
            flush_pending_corrections(
                state,
                gps_week,
                state.mask.satellites,
                flush_policy,
            )
        )
    if state.service_info_chunks:
        service_info_packets.extend(flush_service_info_packets(state, state.service_info_chunks[-1][0]))
    return messages, corrections, service_info_packets


def write_compact_messages(path: Path, messages: list[CSSRMessage]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "subframe_index",
                "message_index",
                "ctype",
                "subtype",
                "tow",
                "udi_seconds",
                "sync",
                "iod",
                "message_bits",
                "correction_count",
                "detail",
            ]
        )
        for message in messages:
            writer.writerow(
                [
                    message.subframe_index,
                    message.message_index,
                    message.ctype,
                    message.subtype,
                    message.tow,
                    f"{message.udi_seconds:.1f}",
                    int(message.sync),
                    message.iod,
                    message.message_bits,
                    message.correction_count,
                    message.detail,
                ]
            )


def write_compact_corrections(path: Path, corrections: list[CompactSSRCorrection]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii", newline="") as handle:
        handle.write(
            "# week,tow,system,prn,dx,dy,dz,dclock_m,high_rate_clock_m[,ura_sigma_m=<m>][,cbias:<id>=<m>...][,pbias:<id>=<m>...][,pdi:<id>=<n>...][,bias_network_id=<n>][,atmos_<name>=<value>...]\n"
        )
        writer = csv.writer(handle)
        for correction in corrections:
            row = [
                correction.week,
                f"{correction.tow:.3f}",
                correction.system,
                correction.prn,
                f"{correction.dx:.6f}",
                f"{correction.dy:.6f}",
                f"{correction.dz:.6f}",
                f"{correction.dclock_m:.6f}",
                f"{correction.high_rate_clock_m:.6f}",
            ]
            if correction.ura_sigma_m is not None:
                row.append(f"ura_sigma_m={correction.ura_sigma_m:.6f}")
            if correction.code_bias_m:
                for signal_id in sorted(correction.code_bias_m):
                    row.append(f"cbias:{signal_id}={correction.code_bias_m[signal_id]:.6f}")
            if correction.phase_bias_m:
                for signal_id in sorted(correction.phase_bias_m):
                    row.append(f"pbias:{signal_id}={correction.phase_bias_m[signal_id]:.6f}")
            if correction.phase_discontinuity:
                for signal_id in sorted(correction.phase_discontinuity):
                    row.append(f"pdi:{signal_id}={correction.phase_discontinuity[signal_id]}")
            if correction.bias_network_id is not None:
                row.append(f"bias_network_id={correction.bias_network_id}")
            if correction.atmos_network_id is not None:
                row.append(f"atmos_network_id={correction.atmos_network_id}")
            if correction.atmos_trop_avail is not None:
                row.append(f"atmos_trop_avail={correction.atmos_trop_avail}")
            if correction.atmos_stec_avail is not None:
                row.append(f"atmos_stec_avail={correction.atmos_stec_avail}")
            if correction.atmos_grid_count is not None:
                row.append(f"atmos_grid_count={correction.atmos_grid_count}")
            if correction.atmos_selected_satellites is not None:
                row.append(f"atmos_selected_satellites={correction.atmos_selected_satellites}")
            if correction.atmos_tokens:
                for key in sorted(correction.atmos_tokens):
                    if key in {
                        "atmos_network_id",
                        "atmos_trop_avail",
                        "atmos_stec_avail",
                        "atmos_grid_count",
                        "atmos_selected_satellites",
                    }:
                        continue
                    row.append(f"{key}={correction.atmos_tokens[key]}")
            writer.writerow(row)


def write_service_info_packets(path: Path, packets: list[CSSRServiceInfoPacket]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "packet_index",
                "first_subframe_index",
                "last_subframe_index",
                "chunk_count",
                "total_bits",
                "packet_hex",
            ]
        )
        for packet in packets:
            writer.writerow(
                [
                    packet.packet_index,
                    packet.first_subframe_index,
                    packet.last_subframe_index,
                    packet.chunk_count,
                    packet.total_bits,
                    packet.packet_hex,
                ]
            )


def decode_source(input_spec: str, limit: int = 0) -> tuple[list[L6Frame], list[L6Subframe], Stats]:
    decoder = L6StreamDecoder()
    assembler = L6SubframeAssembler()
    decoded_frames: list[L6Frame] = []
    assembled_subframes: list[L6Subframe] = []
    with InputSource(input_spec) as source:
        while True:
            if limit and len(decoded_frames) >= limit:
                break
            chunk = source.read(4096)
            if not chunk:
                decoded_frames.extend(decoder.feed(b""))
                break
            for frame in decoder.feed(chunk):
                decoded_frames.append(frame)
                new_subframes = assembler.push(frame)
                for subframe in new_subframes:
                    decoder.stats.subframes += 1
                    assembled_subframes.append(subframe)
                if limit and len(decoded_frames) >= limit:
                    break
            if limit and len(decoded_frames) >= limit:
                break
    return decoded_frames, assembled_subframes, decoder.stats


class InputSource:
    def __init__(self, spec: str):
        self.spec = spec
        self._file = None
        self._fd = None

    def __enter__(self) -> "InputSource":
        if self.spec.startswith("serial://") or self.spec.startswith("/dev/"):
            device, baud = parse_serial_path(self.spec)
            self._fd = os.open(device, os.O_RDONLY | os.O_NOCTTY)
            configure_serial(self._fd, baud)
        else:
            self._file = open(self.spec, "rb")
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._file is not None:
            self._file.close()
        if self._fd is not None:
            os.close(self._fd)

    def read(self, size: int) -> bytes:
        if self._file is not None:
            return self._file.read(size)
        if self._fd is not None:
            try:
                return os.read(self._fd, size)
            except OSError as exc:
                if exc.errno == errno.EIO:
                    return b""
                raise
        return b""


class L6StreamDecoder:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.frame_index = 0
        self.stats = Stats()

    def feed(self, chunk: bytes) -> list[L6Frame]:
        if chunk:
            self.buffer.extend(chunk)
        frames: list[L6Frame] = []
        while True:
            sync_index = self.buffer.find(PREAMBLE)
            if sync_index < 0:
                if len(self.buffer) > len(PREAMBLE) - 1:
                    self.stats.discarded_bytes += len(self.buffer) - (len(PREAMBLE) - 1)
                    del self.buffer[: len(self.buffer) - (len(PREAMBLE) - 1)]
                break
            if sync_index > 0:
                self.stats.discarded_bytes += sync_index
                del self.buffer[:sync_index]
            if len(self.buffer) < FRAME_BYTES:
                break
            raw_frame = bytes(self.buffer[:FRAME_BYTES])
            del self.buffer[:FRAME_BYTES]
            frame = self._parse_frame(raw_frame)
            frames.append(frame)
        return frames

    def _parse_frame(self, raw_frame: bytes) -> L6Frame:
        self.frame_index += 1
        preamble = read_bits(raw_frame, 0, 32)
        if preamble != int.from_bytes(PREAMBLE, "big"):
            raise ValueError("unexpected QZSS L6 preamble after sync")
        prn = read_bits(raw_frame, 32, 8)
        message_type = read_bits(raw_frame, 40, 8)
        alert = bool(read_bits(raw_frame, 48, 1))
        data_part = read_bitpacked_bytes(raw_frame, HEADER_BITS, DATA_PART_BITS)
        rs_parity = read_bitpacked_bytes(raw_frame, HEADER_BITS + DATA_PART_BITS, RS_BITS)

        vendor_id = (message_type >> 5) & 0x7
        facility_id = (message_type >> 3) & 0x3
        reserved_bits = (message_type >> 1) & 0x3
        subframe_start = bool(message_type & 0x1)

        self.stats.frames += 1
        self.stats.valid += 1
        self.stats.prns.add(prn)
        if vendor_id == 0b101:
            self.stats.clas_vendor += 1
        if subframe_start:
            self.stats.subframe_starts += 1
        if alert:
            self.stats.alerts += 1

        return L6Frame(
            index=self.frame_index,
            prn=prn,
            vendor_id=vendor_id,
            facility_id=facility_id,
            reserved_bits=reserved_bits,
            subframe_start=subframe_start,
            alert=alert,
            data_part_bits=DATA_PART_BITS,
            data_part=data_part,
            rs_parity=rs_parity,
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--input", required=True, help="QZSS L6 file or serial:// device.")
    parser.add_argument("--limit", type=int, default=0, help="Stop after N decoded frames.")
    parser.add_argument(
        "--gps-week",
        type=int,
        default=None,
        help="Optional GPS week used when exporting Compact SSR correction rows.",
    )
    parser.add_argument(
        "--compact-flush-policy",
        choices=COMPACT_SSR_FLUSH_POLICIES,
        default=COMPACT_SSR_FLUSH_POLICY_LAG_TOLERANT,
        help=(
            "Compact SSR row emission policy when materializing expanded rows "
            "(default: lag-tolerant-union)."
        ),
    )
    parser.add_argument(
        "--compact-atmos-merge-policy",
        choices=COMPACT_ATMOS_MERGE_POLICIES,
        default=COMPACT_ATMOS_MERGE_POLICY_STEC_COEFF_CARRY,
        help=(
            "Compact SSR atmosphere-token merge policy across subtype/epoch boundaries "
            "(default: stec-coeff-carry)."
        ),
    )
    parser.add_argument(
        "--compact-atmos-subtype-merge-policy",
        choices=COMPACT_ATMOS_SUBTYPE_MERGE_POLICIES,
        default=COMPACT_ATMOS_SUBTYPE_MERGE_POLICY_UNION,
        help=(
            "Compact SSR atmosphere subtype precedence policy within one pending epoch "
            "(default: union)."
        ),
    )
    parser.add_argument(
        "--compact-phase-bias-merge-policy",
        choices=COMPACT_PHASE_BIAS_MERGE_POLICIES,
        default=COMPACT_PHASE_BIAS_MERGE_POLICY_LATEST_UNION,
        help=(
            "Compact SSR phase-bias merge policy within one pending epoch "
            "(default: latest-union)."
        ),
    )
    parser.add_argument(
        "--compact-phase-bias-source-policy",
        choices=COMPACT_PHASE_BIAS_SOURCE_POLICIES,
        default=COMPACT_PHASE_BIAS_SOURCE_POLICY_ARRIVAL_ORDER,
        help=(
            "Compact SSR phase-bias source-row precedence policy when subtype 5 and 6 "
            "both provide the same signal within one pending epoch "
            "(default: arrival-order)."
        ),
    )
    parser.add_argument(
        "--compact-code-bias-composition-policy",
        choices=COMPACT_CODE_BIAS_COMPOSITION_POLICIES,
        default=COMPACT_CODE_BIAS_COMPOSITION_POLICY_DIRECT,
        help=(
            "Compact SSR code-bias composition policy when subtype-6 network rows are "
            "materialized against previously accepted base rows "
            "(default: direct-values)."
        ),
    )
    parser.add_argument(
        "--compact-code-bias-bank-policy",
        choices=COMPACT_CODE_BIAS_BANK_POLICIES,
        default=COMPACT_CODE_BIAS_BANK_POLICY_PENDING_EPOCH,
        help=(
            "Compact SSR code-bias base-bank lookup policy when subtype-6 network rows "
            "need previously accepted base rows "
            "(default: pending-epoch)."
        ),
    )
    parser.add_argument(
        "--compact-phase-bias-composition-policy",
        choices=COMPACT_PHASE_BIAS_COMPOSITION_POLICIES,
        default=COMPACT_PHASE_BIAS_COMPOSITION_POLICY_DIRECT,
        help=(
            "Compact SSR phase-bias composition policy when subtype-6 network rows are "
            "materialized against previously accepted base rows "
            "(default: direct-values)."
        ),
    )
    parser.add_argument(
        "--compact-phase-bias-bank-policy",
        choices=COMPACT_PHASE_BIAS_BANK_POLICIES,
        default=COMPACT_PHASE_BIAS_BANK_POLICY_PENDING_EPOCH,
        help=(
            "Compact SSR phase-bias base-bank lookup policy when subtype-6 network rows "
            "need previously accepted base rows "
            "(default: pending-epoch)."
        ),
    )
    parser.add_argument(
        "--compact-bias-row-materialization",
        choices=COMPACT_BIAS_ROW_MATERIALIZATION_POLICIES,
        default=COMPACT_BIAS_ROW_MATERIALIZATION_POLICY_OVERLAP_ONLY,
        help=(
            "Compact SSR row materialization policy when subtype-6 network rows "
            "are allowed to extend missing code/phase bias signal rows from base banks "
            "(default: overlap-only)."
        ),
    )
    parser.add_argument(
        "--compact-row-construction-policy",
        choices=COMPACT_ROW_CONSTRUCTION_POLICIES,
        default=COMPACT_ROW_CONSTRUCTION_POLICY_INDEPENDENT,
        help=(
            "Compact SSR row construction policy controlling how subtype-4/subtype-6 "
            "row and value construction interact before expanded SSR emission "
            "(default: independent)."
        ),
    )
    parser.add_argument(
        "--extract-data-parts",
        type=Path,
        default=None,
        help="Optional CSV path for extracted L6 data-part and parity hex.",
    )
    parser.add_argument(
        "--extract-subframes",
        type=Path,
        default=None,
        help="Optional CSV path for assembled 5-part CLAS subframes.",
    )
    parser.add_argument(
        "--extract-compact-messages",
        type=Path,
        default=None,
        help="Optional CSV path for decoded Compact SSR message inventory from assembled subframes.",
    )
    parser.add_argument(
        "--extract-compact-corrections",
        type=Path,
        default=None,
        help="Optional CSV path for sampled Compact SSR corrections decoded from raw subframes.",
    )
    parser.add_argument(
        "--extract-service-info",
        type=Path,
        default=None,
        help="Optional CSV path for reassembled subtype 10 service information packets.",
    )
    parser.add_argument(
        "--show-preview",
        action="store_true",
        help="Show a printable preview of the first bytes in the data part.",
    )
    return parser.parse_args()


def write_extracted_frames(path: Path, frames: list[L6Frame]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "frame_index",
                "prn",
                "vendor_id",
                "facility_id",
                "reserved_bits",
                "subframe_start",
                "alert",
                "data_part_bits",
                "data_part_hex",
                "rs_parity_hex",
            ]
        )
        for frame in frames:
            writer.writerow(
                [
                    frame.index,
                    frame.prn,
                    frame.vendor_id,
                    frame.facility_id,
                    frame.reserved_bits,
                    int(frame.subframe_start),
                    int(frame.alert),
                    frame.data_part_bits,
                    frame.data_part_hex,
                    frame.rs_hex,
                ]
            )


def write_extracted_subframes(path: Path, subframes: list[L6Subframe]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "subframe_index",
                "prn",
                "vendor_id",
                "facility_id",
                "frame_count",
                "alert_frames",
                "data_bits",
                "first_frame_index",
                "last_frame_index",
                "subframe_hex",
            ]
        )
        for subframe in subframes:
            writer.writerow(
                [
                    subframe.index,
                    subframe.prn,
                    subframe.vendor_id,
                    subframe.facility_id,
                    subframe.frame_count,
                    subframe.alert_frames,
                    subframe.data_bits,
                    subframe.first_frame_index,
                    subframe.last_frame_index,
                    subframe.data_hex,
                ]
            )


def main() -> int:
    args = parse_args()
    decoded_frames, assembled_subframes, stats = decode_source(args.input, limit=args.limit)
    for frame in decoded_frames:
        preview_suffix = ""
        if args.show_preview:
            preview_suffix = f" preview={frame.preview_text or '<binary>'}"
        print(
            "l6_frame: "
            f"index={frame.index} prn={frame.prn} vendor={frame.vendor_id} "
            f"facility={frame.facility_name} subframe_start={int(frame.subframe_start)} "
            f"alert={int(frame.alert)} data_part_bits={frame.data_part_bits}{preview_suffix}"
        )
    for subframe in assembled_subframes:
        print(
            "l6_subframe: "
            f"index={subframe.index} prn={subframe.prn} vendor={subframe.vendor_id} "
            f"facility={subframe.facility_name} frames={subframe.frame_count} "
            f"alert_frames={subframe.alert_frames} data_bits={subframe.data_bits}"
        )

    compact_messages: list[CSSRMessage] = []
    compact_corrections: list[CompactSSRCorrection] = []
    service_info_packets: list[CSSRServiceInfoPacket] = []
    if (
        args.extract_compact_messages is not None or
        args.extract_compact_corrections is not None or
        args.extract_service_info is not None
    ):
        compact_messages, compact_corrections, service_info_packets = decode_cssr_messages(
            assembled_subframes,
            gps_week=args.gps_week,
            flush_policy=args.compact_flush_policy,
            atmos_merge_policy=args.compact_atmos_merge_policy,
            atmos_subtype_merge_policy=args.compact_atmos_subtype_merge_policy,
            phase_bias_merge_policy=args.compact_phase_bias_merge_policy,
            phase_bias_source_policy=args.compact_phase_bias_source_policy,
            code_bias_composition_policy=args.compact_code_bias_composition_policy,
            code_bias_bank_policy=args.compact_code_bias_bank_policy,
            phase_bias_composition_policy=args.compact_phase_bias_composition_policy,
            phase_bias_bank_policy=args.compact_phase_bias_bank_policy,
            bias_row_materialization_policy=args.compact_bias_row_materialization,
            row_construction_policy=args.compact_row_construction_policy,
        )
        for message in compact_messages:
            print(
                "cssr_message: "
                f"subframe={message.subframe_index} index={message.message_index} "
                f"subtype={message.subtype} tow={message.tow} sync={int(message.sync)} "
                f"iod={message.iod} bits={message.message_bits} detail={message.detail}"
            )

    if args.extract_data_parts is not None:
        write_extracted_frames(args.extract_data_parts, decoded_frames)
        print(f"extracted: frames={len(decoded_frames)} csv={args.extract_data_parts}")
    if args.extract_subframes is not None:
        write_extracted_subframes(args.extract_subframes, assembled_subframes)
        print(f"assembled: subframes={len(assembled_subframes)} csv={args.extract_subframes}")
    if args.extract_compact_messages is not None:
        write_compact_messages(args.extract_compact_messages, compact_messages)
        print(f"decoded: compact_messages={len(compact_messages)} csv={args.extract_compact_messages}")
    if args.extract_compact_corrections is not None:
        if args.gps_week is None:
            raise SystemExit("--gps-week is required when exporting compact corrections")
        write_compact_corrections(args.extract_compact_corrections, compact_corrections)
        print(
            "extracted: "
            f"compact_corrections={len(compact_corrections)} csv={args.extract_compact_corrections}"
        )
    if args.extract_service_info is not None:
        write_service_info_packets(args.extract_service_info, service_info_packets)
        print(f"extracted: service_info_packets={len(service_info_packets)} csv={args.extract_service_info}")

    prn_summary = ",".join(str(prn) for prn in sorted(stats.prns)) or "none"
    print(
        "summary: "
        f"frames={stats.frames} valid={stats.valid} "
        f"clas_vendor={stats.clas_vendor} subframe_starts={stats.subframe_starts} "
        f"alerts={stats.alerts} subframes={stats.subframes} prns={prn_summary} "
        f"discarded_bytes={stats.discarded_bytes}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
