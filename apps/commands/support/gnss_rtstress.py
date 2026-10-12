#!/usr/bin/env python3
"""Recorded-data real-time stress support for ``gnss realtime-stress``.

Everything here runs from recorded data: RTCM3 byte streams that were encoded
from RINEX (see ``gnss_rtcm_encode``) are served over local TCP or a minimal
local NTRIP caster with wall-clock pacing, fault injection is applied on the
server side, ``gnss_live`` is run against the sockets, and per-epoch output,
latency, resource use and accuracy are measured.  No receiver is involved and
nothing here is a field soak.

The module is pure standard library (no numpy, no matplotlib).
"""

from __future__ import annotations

import bisect
import csv
import heapq
import json
import math
import os
import random
import re
import shutil
import socket
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

GPS_WEEK_SECONDS = 604800
OMEGA_E = 7.2921151467e-5
STATUS_NAMES = {0: "NONE", 1: "SPP", 2: "DGPS", 3: "FLOAT", 4: "FIXED", 5: "PPP_FLOAT", 6: "PPP_FIXED", 7: "PROPAGATED"}


# --------------------------------------------------------------------------
# RTCM3 framing helpers
# --------------------------------------------------------------------------

def _build_crc_table() -> list[int]:
    table = []
    for i in range(256):
        crc = i << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
        table.append(crc & 0xFFFFFF)
    return table


_CRC_TABLE = _build_crc_table()


def crc24q(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc = ((crc << 8) & 0xFFFFFF) ^ _CRC_TABLE[((crc >> 16) ^ byte) & 0xFF]
    return crc


def split_frames(chunk: bytes) -> list[bytes]:
    """Split a buffer of back-to-back RTCM3 frames."""
    frames: list[bytes] = []
    i = 0
    n = len(chunk)
    while i + 6 <= n:
        if chunk[i] != 0xD3:
            break
        length = ((chunk[i + 1] & 0x03) << 8) | chunk[i + 2]
        end = i + 3 + length + 3
        if end > n:
            break
        frames.append(chunk[i:end])
        i = end
    return frames


def frame_type(frame: bytes) -> int:
    return (frame[3] << 4) | (frame[4] >> 4)


def _get_bits(buf: bytes, pos: int, length: int) -> int:
    value = 0
    for i in range(length):
        bit = pos + i
        value = (value << 1) | ((buf[bit >> 3] >> (7 - (bit & 7))) & 1)
    return value


def _set_bits(buf: bytearray, pos: int, length: int, value: int) -> None:
    for i in range(length):
        bit = pos + length - 1 - i
        mask = 1 << (7 - (bit & 7))
        if (value >> i) & 1:
            buf[bit >> 3] |= mask
        else:
            buf[bit >> 3] &= ~mask & 0xFF


def _refresh_crc(frame: bytearray) -> bytes:
    crc = crc24q(bytes(frame[:-3]))
    frame[-3] = (crc >> 16) & 0xFF
    frame[-2] = (crc >> 8) & 0xFF
    frame[-1] = crc & 0xFF
    return bytes(frame)


def shift_gps_frame_time(frame: bytes, delta_s: int) -> bytes:
    """Shift a GPS MSM or 1019 frame by ``delta_s`` seconds, keeping it physically consistent.

    ``delta_s`` must be a multiple of 16 (1019 stores toe/toc in 16 s units).  For a 1019
    ephemeris the right ascension at epoch is advanced by ``OMEGA_E * delta`` so a shifted
    broadcast orbit gives the same ECEF satellite track at the shifted time.  Other messages
    (1005, GLONASS, ...) are returned unchanged.
    """
    if delta_s == 0:
        return frame
    msg = frame_type(frame)
    buf = bytearray(frame)
    payload_bit = 24  # payload starts after the 3-byte header
    if 1071 <= msg <= 1077:
        tow_ms = _get_bits(frame, payload_bit + 24, 30)
        new_ms = (tow_ms + delta_s * 1000) % (GPS_WEEK_SECONDS * 1000)
        _set_bits(buf, payload_bit + 24, 30, new_ms)
        return _refresh_crc(buf)
    if msg == 1019:
        if delta_s % 16:
            raise ValueError("1019 shift must be a multiple of 16 s")
        week = _get_bits(frame, payload_bit + 18, 10)
        toc = _get_bits(frame, payload_bit + 56, 16)
        toe = _get_bits(frame, payload_bit + 288, 16)
        omega0 = _get_bits(frame, payload_bit + 320, 32)
        if omega0 >= 1 << 31:
            omega0 -= 1 << 32
        unit = delta_s // 16
        wrap = GPS_WEEK_SECONDS // 16
        carry = (toe + unit) // wrap
        toe_new = (toe + unit) % wrap
        toc_new = (toc + unit) % wrap
        d_omega = int(round(OMEGA_E * delta_s / math.pi * (1 << 31)))
        omega_new = omega0 + d_omega
        omega_new = ((omega_new + (1 << 31)) % (1 << 32)) - (1 << 31)
        _set_bits(buf, payload_bit + 18, 10, (week + carry) % 1024)
        _set_bits(buf, payload_bit + 56, 16, toc_new)
        _set_bits(buf, payload_bit + 288, 16, toe_new)
        _set_bits(buf, payload_bit + 320, 32, omega_new & 0xFFFFFFFF)
        return _refresh_crc(buf)
    return frame


@dataclass
class StreamPlan:
    """An encoded RTCM stream and its per-epoch index."""

    name: str
    data: bytes
    epochs: list[tuple[float, int, int, int, int, int]]  # tow, week, offset, length, n_obs, n_aux

    @classmethod
    def load(cls, name: str, rtcm_path: Path, index_path: Path) -> "StreamPlan":
        data = Path(rtcm_path).read_bytes()
        epochs = []
        for line in Path(index_path).read_text().splitlines():
            parts = line.split()
            if len(parts) < 6:
                continue
            epochs.append((float(parts[0]), int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4]), int(parts[5])))
        if not epochs:
            raise ValueError(f"empty index {index_path}")
        return cls(name=name, data=data, epochs=epochs)

    @property
    def tow0(self) -> float:
        return self.epochs[0][0]

    @property
    def span(self) -> float:
        return self.epochs[-1][0] - self.epochs[0][0]

    def chunk(self, i: int) -> bytes:
        _, _, off, length, _, _ = self.epochs[i]
        return self.data[off:off + length]


def loop_period(plans: Iterable[StreamPlan], gap: float = 0.0, step: float = 16.0) -> float:
    """Loop period: stream span plus one epoch plus ``gap``, rounded up to a 16 s multiple."""
    span = max(p.span for p in plans)
    dts = []
    for p in plans:
        if len(p.epochs) > 1:
            dts.append(p.epochs[1][0] - p.epochs[0][0])
    dt = max(dts) if dts else 1.0
    return math.ceil((span + dt + 1.0 + gap) / step) * step


# --------------------------------------------------------------------------
# Fault model
# --------------------------------------------------------------------------

FAULT_KINDS = (
    "base_outage",       # base data dropped for `dur` s (link up, no data)
    "base_stall",        # base data held and flushed in a burst after `dur` s
    "base_latency",      # constant delay `value` s added to base data
    "base_jitter",       # random delay U(0,value) s on base epochs, may reorder epochs
    "rover_outage",      # rover data dropped for `dur` s
    "corrupt_base",      # flip one byte in a fraction `value` of base frames
    "corrupt_rover",     # flip one byte in a fraction `value` of rover frames
    "rover_disconnect",  # rover server drops the connection and refuses reconnects for `dur` s
    "base_disconnect",   # same for base
    "server_restart",    # both servers go down for `dur` s
    "base_blackhole",    # at `at` the current base connection stays open but nothing more is sent on it
    "rover_blackhole",   # same for rover; a new connection is served normally (`dur` must be 0)
)

BLACKHOLE_KINDS = ("base_blackhole", "rover_blackhole")


@dataclass
class Fault:
    kind: str
    at: float = 0.0       # stream seconds from the start of the window
    dur: float = 0.0      # seconds (0 = open ended for latency/jitter/corrupt)
    value: float = 0.0    # delay / jitter / corruption probability

    def __post_init__(self) -> None:
        if self.kind not in FAULT_KINDS:
            raise ValueError(f"unknown fault kind {self.kind!r}; expected one of {', '.join(FAULT_KINDS)}")
        if self.kind in BLACKHOLE_KINDS and self.dur != 0.0:
            raise ValueError(f"{self.kind} is open ended (the connection stays silent until the client "
                             f"reconnects); dur must be 0, got {self.dur}")

    @property
    def end(self) -> float:
        return self.at + self.dur

    def covers(self, t: float) -> bool:
        if self.dur <= 0.0:
            return t >= self.at
        return self.at <= t < self.at + self.dur

    def to_dict(self) -> dict[str, Any]:
        return {"kind": self.kind, "at": self.at, "dur": self.dur, "value": self.value}

    @classmethod
    def parse(cls, text: str) -> "Fault":
        """Parse ``kind@at+dur=value`` (e.g. ``base_outage@120+10`` or ``corrupt_base@60+30=0.05``)."""
        m = re.fullmatch(r"([a-z_]+)(?:@([0-9.]+))?(?:\+([0-9.]+))?(?:=([0-9.]+))?", text.strip())
        if not m:
            raise ValueError(f"cannot parse fault {text!r}; use kind@at+dur=value")
        kind, at, dur, value = m.groups()
        return cls(kind, float(at or 0.0), float(dur or 0.0), float(value or 0.0))


# --------------------------------------------------------------------------
# Paced TCP / minimal NTRIP caster servers
# --------------------------------------------------------------------------

class StreamClock:
    """Shared stream clock: starts once every registered server has a client."""

    def __init__(self, speed: float, start_delay: float = 0.25) -> None:
        self.speed = float(speed)
        self.start_delay = start_delay
        self._lock = threading.Lock()
        self._expected = 0
        self._arrived = 0
        self.started = threading.Event()
        self.t0 = 0.0

    def register(self) -> None:
        with self._lock:
            self._expected += 1

    def arrive(self) -> None:
        with self._lock:
            self._arrived += 1
            if self._arrived >= self._expected and not self.started.is_set():
                self.t0 = time.monotonic() + self.start_delay
                self.started.set()

    def wall(self, stream_t: float) -> float:
        return self.t0 + stream_t / self.speed

    def stream_time(self, wall: float | None = None) -> float:
        return ((time.monotonic() if wall is None else wall) - self.t0) * self.speed


@dataclass(order=True)
class _Event:
    send_wall: float
    seq: int
    payload: bytes = field(compare=False)
    meta: dict[str, Any] = field(compare=False)


class StreamServer:
    """Serve one recorded RTCM stream to one client at a time, with pacing and faults."""

    def __init__(
        self,
        name: str,
        plan: StreamPlan,
        clock: StreamClock,
        faults: list[Fault],
        ntrip: bool = False,
        mountpoint: str | None = None,
        port: int = 0,
        loops: int = 1,
        shift_period: float = 0.0,
        window: tuple[float, float] | None = None,
        seed: int = 1,
        send_chunk: int = 0,
        prefix: bytes = b"",
        log_limit: int = 400000,
    ) -> None:
        self.name = name
        self.plan = plan
        self.clock = clock
        self.faults = faults
        self.ntrip = ntrip
        self.mountpoint = mountpoint or name.upper()
        self.loops = loops  # 0 = endless
        self.shift_period = shift_period
        self.window = window
        self.rng = random.Random(seed)
        self.send_chunk = send_chunk
        self.prefix = prefix
        self.log_limit = log_limit
        self.last_loop = 0
        self.halt_event = threading.Event()
        self._ended = False
        self.finished_wall = 0.0
        self.stop_event = threading.Event()
        self._listener: socket.socket | None = None
        self._listener_lock = threading.Lock()
        self._client: socket.socket | None = None
        self._client_lock = threading.Lock()
        # stream time at which the current client connected (-inf before the clock starts)
        self._client_since = -math.inf
        # connections silenced by a *_blackhole fault, kept open until stop()
        self._held: list[socket.socket] = []
        self.port = port
        self.log: list[dict[str, Any]] = []
        self.counters: dict[str, int] = {
            "epochs_planned": 0, "epochs_sent": 0, "epochs_dropped": 0, "frames_corrupted": 0,
            "frames_sent": 0, "bytes_sent": 0, "connections": 0, "disconnects_forced": 0,
            "send_errors": 0, "epochs_no_client": 0, "epochs_reordered": 0, "epochs_blackholed": 0,
            "connections_blackholed": 0,
        }
        self.connection_times: list[float] = []
        self.finished = threading.Event()
        self._acceptor: threading.Thread | None = None
        self._sender: threading.Thread | None = None
        self._first_client = threading.Event()
        self._bind()
        self.clock.register()

    # -- sockets ---------------------------------------------------------
    def _bind(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("127.0.0.1", self.port))
        sock.listen(4)
        sock.settimeout(0.05)
        self.port = sock.getsockname()[1]
        with self._listener_lock:
            self._listener = sock

    def _unbind(self) -> None:
        with self._listener_lock:
            if self._listener is not None:
                try:
                    self._listener.close()
                except OSError:
                    pass
                self._listener = None

    @property
    def url(self) -> str:
        if self.ntrip:
            return f"ntrip://127.0.0.1:{self.port}/{self.mountpoint}"
        return f"tcp://127.0.0.1:{self.port}"

    def _blackhole_at(self) -> float | None:
        """Time of the first blackhole fault that silenced the connection made at `_client_since`."""
        for f in self.faults:
            if f.kind == f"{self.name}_blackhole" and self._client_since < f.at:
                return f.at
        return None

    def _is_blackholed(self, stream_t: float) -> bool:
        """True when the current connection predates a blackhole fault that is already in effect."""
        at = self._blackhole_at()
        return at is not None and stream_t >= at

    def _drop_client(self, keep_if_blackholed: bool = False) -> None:
        with self._client_lock:
            client, self._client = self._client, None
            held = False
            if client is not None and keep_if_blackholed and self.clock.started.is_set():
                if self._is_blackholed(self.clock.stream_time()):
                    # A dead peer never sends FIN: keep our end open until the harness stops.
                    self._held.append(client)
                    self.counters["connections_blackholed"] += 1
                    held = True
        if client is not None and not held:
            try:
                client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                client.close()
            except OSError:
                pass

    def _is_down(self) -> bool:
        """True while a disconnect/restart fault window is open for this server."""
        if not self.clock.started.is_set():
            return False
        t = self.clock.stream_time()
        for f in self.faults:
            if f.dur <= 0:
                continue
            if f.kind == "server_restart" or f.kind == f"{self.name}_disconnect":
                if f.at <= t < f.at + f.dur:
                    return True
        return False

    def _handshake(self, conn: socket.socket) -> bool:
        if not self.ntrip:
            return True
        conn.settimeout(2.0)
        data = b""
        try:
            while b"\r\n\r\n" not in data and len(data) < 8192:
                part = conn.recv(1024)
                if not part:
                    return False
                data += part
        except OSError:
            return False
        first = data.split(b"\r\n", 1)[0].decode("latin-1", "replace")
        m = re.match(r"GET /(\S*) HTTP/1\.[01]", first)
        if not m or m.group(1) != self.mountpoint:
            try:
                conn.sendall(b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n")
            except OSError:
                pass
            return False
        conn.sendall(b"ICY 200 OK\r\n\r\n")
        return True

    def _accept_loop(self) -> None:
        while not self.stop_event.is_set():
            down = self._is_down()
            if down:
                if self._listener is not None:
                    self.counters["disconnects_forced"] += 1 if self._client is not None else 0
                    self._drop_client()
                    self._unbind()
                time.sleep(0.01)
                continue
            if self._ended:
                time.sleep(0.01)
                continue
            if self._listener is None:
                try:
                    self._bind()
                except OSError:
                    time.sleep(0.02)
                    continue
            with self._listener_lock:
                listener = self._listener
            if listener is None:
                continue
            try:
                conn, _ = listener.accept()
            except (socket.timeout, OSError):
                continue
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            if not self._handshake(conn):
                conn.close()
                continue
            conn.settimeout(None)
            self._drop_client(keep_if_blackholed=True)
            with self._client_lock:
                self._client = conn
                self._client_since = self.clock.stream_time() if self.clock.started.is_set() else -math.inf
            self.counters["connections"] += 1
            self.connection_times.append(time.monotonic())
            if not self._first_client.is_set():
                self._first_client.set()
                self.clock.arrive()

    # -- scheduling --------------------------------------------------------
    def _epoch_iter(self) -> Iterable[tuple[int, int, float, bytes]]:
        """Yield (loop, index, stream_t, chunk) honouring the optional [start, end) window."""
        loop = 0
        tow0 = self.plan.tow0
        while self.loops == 0 or loop < self.loops:
            shift = int(round(loop * self.shift_period))
            for i, (tow, _, _, _, _, _) in enumerate(self.plan.epochs):
                stream_t = (tow - tow0) + loop * self.shift_period
                chunk = self.plan.chunk(i)
                if i == 0 and self.prefix:
                    chunk = self.prefix + chunk
                if shift:
                    chunk = b"".join(shift_gps_frame_time(f, shift) for f in split_frames(chunk))
                yield loop, i, stream_t, chunk
            loop += 1

    def _apply_faults(self, stream_t: float, chunk: bytes, meta: dict[str, Any]) -> tuple[bytes | None, float]:
        """Return (payload or None when dropped, extra_delay_s)."""
        delay = 0.0
        for f in self.faults:
            kind = f.kind
            if kind in (f"{self.name}_outage",) and f.covers(stream_t):
                meta["fault"] = kind
                return None, 0.0
            if kind == f"{self.name}_stall" and f.covers(stream_t):
                # hold until the end of the window; all held epochs flush together
                delay = max(delay, f.end - stream_t)
                meta["fault"] = kind
            if kind == f"{self.name}_latency" and f.covers(stream_t):
                delay += f.value
                meta["fault"] = kind
            if kind == f"{self.name}_jitter" and f.covers(stream_t):
                delay += self.rng.uniform(0.0, f.value)
                meta["fault"] = kind
        for f in self.faults:
            if f.kind == f"corrupt_{self.name}" and f.covers(stream_t) and f.value > 0:
                frames = split_frames(chunk)
                out = []
                corrupted = 0
                for frame in frames:
                    if self.rng.random() < f.value:
                        buf = bytearray(frame)
                        idx = self.rng.randrange(len(buf))
                        buf[idx] ^= 1 << self.rng.randrange(8)
                        frame = bytes(buf)
                        corrupted += 1
                    out.append(frame)
                if corrupted:
                    chunk = b"".join(out)
                    meta["fault"] = f.kind
                    meta["corrupted_frames"] = corrupted
        return chunk, delay

    def _log(self, meta: dict[str, Any]) -> None:
        self.last_loop = max(self.last_loop, meta["loop"])
        if len(self.log) < self.log_limit:
            self.log.append(meta)

    def _sleep_until(self, deadline: float) -> None:
        while not self.stop_event.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            time.sleep(min(remaining, 0.02) if remaining > 0.003 else 0.0003)

    def _send(self, ev: _Event) -> None:
        with self._client_lock:
            client = self._client
        meta = ev.meta
        meta["send_wall"] = time.monotonic()
        if client is None or self._is_down():
            meta["sent"] = False
            meta["dropped_reason"] = "no_client"
            self.counters["epochs_no_client"] += 1
            self._log(meta)
            return
        if self._is_blackholed(meta["stream_t"]):
            # *_blackhole: the connection stays open but is silent from `at` on.
            meta["sent"] = False
            meta["dropped_reason"] = "blackhole"
            meta["fault"] = f"{self.name}_blackhole"
            self.counters["epochs_blackholed"] += 1
            self._log(meta)
            return
        try:
            if self.send_chunk:
                payload = ev.payload
                for i in range(0, len(payload), self.send_chunk):
                    client.sendall(payload[i:i + self.send_chunk])
            else:
                client.sendall(ev.payload)
            meta["sent"] = True
            meta["done_wall"] = time.monotonic()
            self.counters["epochs_sent"] += 1
            self.counters["bytes_sent"] += len(ev.payload)
            self.counters["frames_sent"] += len(split_frames(ev.payload))
        except OSError:
            meta["sent"] = False
            meta["dropped_reason"] = "send_error"
            self.counters["send_errors"] += 1
            self._drop_client()
        self._log(meta)

    def _send_loop(self) -> None:
        self._first_client.wait()
        self.clock.started.wait()
        heap: list[_Event] = []
        seq = 0
        max_hold = 0.0
        for f in self.faults:
            if f.kind in (f"{self.name}_latency", f"{self.name}_jitter"):
                max_hold = max(max_hold, f.value)
            if f.kind == f"{self.name}_stall":
                max_hold = max(max_hold, f.dur)
        max_hold_wall = max_hold / self.clock.speed if self.clock.speed else max_hold
        last_sent_stream_t = -1.0
        it = iter(self._epoch_iter())
        pending = next(it, None)
        while not self.stop_event.is_set() and not self.halt_event.is_set():
            now = time.monotonic()
            # push every epoch scheduled to be produced within the hold window
            while pending is not None and self.clock.wall(pending[2]) <= now + max_hold_wall + 0.001:
                loop, i, stream_t, chunk = pending
                pending = next(it, None)
                sched = self.clock.wall(stream_t)
                self.counters["epochs_planned"] += 1
                tow_orig = self.plan.epochs[i][0]
                meta: dict[str, Any] = {
                    "loop": loop, "index": i, "tow": tow_orig, "stream_t": stream_t,
                    "tow_out": (tow_orig + loop * self.shift_period) % GPS_WEEK_SECONDS,
                    "sched_wall": sched, "bytes": len(chunk),
                }
                payload, delay = self._apply_faults(stream_t, chunk, meta)
                if payload is None:
                    meta["sent"] = False
                    meta["dropped_reason"] = "fault_outage"
                    meta["send_wall"] = sched
                    self.counters["epochs_dropped"] += 1
                    self._log(meta)
                    continue
                if "corrupted_frames" in meta:
                    self.counters["frames_corrupted"] += meta["corrupted_frames"]
                # Fault times/delays are stream seconds, so they scale with the speed factor and a
                # scenario keeps the same shape at 1x, 5x or 20x.
                send_wall = sched + delay / self.clock.speed
                seq += 1
                heapq.heappush(heap, _Event(send_wall, seq, payload, meta))
            if heap and heap[0].send_wall <= now:
                ev = heapq.heappop(heap)
                if ev.meta["stream_t"] < last_sent_stream_t - 1e-9:
                    self.counters["epochs_reordered"] += 1
                    ev.meta["reordered"] = True
                else:
                    last_sent_stream_t = ev.meta["stream_t"]
                self._send(ev)
                continue
            if pending is None and not heap:
                break
            next_push = self.clock.wall(pending[2]) - max_hold_wall - 0.001 if pending is not None else math.inf
            next_send = heap[0].send_wall if heap else math.inf
            self._sleep_until(min(next_push, next_send))
        self.finished_wall = time.monotonic()
        self.finished.set()

    # -- lifecycle ----------------------------------------------------------
    def start(self) -> None:
        self._acceptor = threading.Thread(target=self._accept_loop, name=f"{self.name}-accept", daemon=True)
        self._sender = threading.Thread(target=self._send_loop, name=f"{self.name}-send", daemon=True)
        self._acceptor.start()
        self._sender.start()

    def _close_held(self) -> None:
        held, self._held = self._held, []
        for conn in held:
            try:
                conn.close()
            except OSError:
                pass

    def stop(self) -> None:
        self.stop_event.set()
        self._drop_client()
        self._close_held()
        self._unbind()
        for t in (self._sender, self._acceptor):
            if t is not None:
                t.join(timeout=3.0)

    def halt(self) -> None:
        """Stop producing epochs (used for endless loops at the wall-clock deadline)."""
        self.halt_event.set()

    def close_after_stream(self, linger: float = 0.5) -> None:
        """Gracefully end the stream: let the client drain, then close."""
        time.sleep(linger)
        self._ended = True
        self._drop_client()
        self._close_held()
        self._unbind()


# --------------------------------------------------------------------------
# Process runner and sampling
# --------------------------------------------------------------------------

def find_binary(name: str, bin_dir: str | os.PathLike | None = None) -> str:
    candidates: list[Path] = []
    if bin_dir:
        candidates += [Path(bin_dir) / name, Path(bin_dir) / "apps" / name]
    env_dir = os.environ.get("GNSSPP_BUILD_DIR")
    if env_dir:
        candidates += [Path(env_dir) / "apps" / name, Path(env_dir) / name]
    here = Path(__file__).resolve()
    root = here.parents[3]
    candidates += [here.parent.parent / name, here.parent.parent.parent / name]
    candidates += sorted(root.glob(f"build*/apps/{name}"))
    for c in candidates:
        if c.is_file() and os.access(c, os.X_OK):
            return str(c)
    found = shutil.which(name)
    if found:
        return found
    raise SystemExit(f"cannot find `{name}`; build it (cmake --build build) or pass --bin-dir / GNSSPP_BUILD_DIR")


def read_proc_sample(pid: int, comm: str | None = None) -> dict[str, float] | None:
    try:
        if comm is not None:
            # Skip the short window between fork() and exec() where /proc shows the parent image.
            with open(f"/proc/{pid}/comm") as fh:
                if fh.read().strip() != comm[:15]:
                    return {"pending": 1.0}
        with open(f"/proc/{pid}/status") as fh:
            status = fh.read()
        with open(f"/proc/{pid}/stat") as fh:
            stat = fh.read()
    except OSError:
        return None
    rss = hwm = threads = 0.0
    for line in status.splitlines():
        if line.startswith("VmRSS:"):
            rss = float(line.split()[1]) / 1024.0
        elif line.startswith("VmHWM:"):
            hwm = float(line.split()[1]) / 1024.0
        elif line.startswith("Threads:"):
            threads = float(line.split()[1])
    fields = stat.rsplit(")", 1)[1].split()
    ticks = float(fields[11]) + float(fields[12])
    return {"rss_mb": rss, "hwm_mb": hwm, "threads": threads, "cpu_s": ticks / os.sysconf("SC_CLK_TCK")}


class ResourceSampler(threading.Thread):
    def __init__(self, pid: int, interval: float = 1.0, comm: str | None = None) -> None:
        super().__init__(daemon=True, name="rt-sampler")
        self.pid = pid
        self.comm = comm
        self.interval = interval
        self.stop_event = threading.Event()
        self.samples: list[dict[str, float]] = []

    def run(self) -> None:
        prev: dict[str, float] | None = None
        while not self.stop_event.is_set():
            s = read_proc_sample(self.pid, self.comm)
            now = time.monotonic()
            if s is None:
                break
            if "pending" in s:
                self.stop_event.wait(0.01)
                continue
            s["mono"] = now
            s["cpu_pct"] = 0.0
            if prev is not None and now > prev["mono"]:
                s["cpu_pct"] = 100.0 * (s["cpu_s"] - prev["cpu_s"]) / (now - prev["mono"])
            self.samples.append(s)
            prev = s
            # sample quickly while the process starts so short-lived runs still get a peak
            self.stop_event.wait(min(self.interval, 0.25) if len(self.samples) < 12 else self.interval)

    def stop(self) -> None:
        self.stop_event.set()
        self.join(timeout=3.0)


class PosTailer(threading.Thread):
    """Follow a growing .pos file and timestamp every row when first observed."""

    def __init__(self, path: Path, compact: bool = False, poll: float = 0.002) -> None:
        super().__init__(daemon=True, name="rt-pos-tail")
        self.path = Path(path)
        self.compact = compact  # keep only time/status, and thin out long runs
        self.poll = poll
        self.stop_event = threading.Event()
        self.rows: list[tuple] = []
        self.count = 0

    def run(self) -> None:
        fh = None
        buf = ""
        while not self.stop_event.is_set() or fh is not None:
            if fh is None:
                if self.path.exists():
                    fh = open(self.path, "r")
                else:
                    if self.stop_event.is_set():
                        break
                    time.sleep(0.01)
                    continue
            chunk = fh.read()
            now = time.monotonic()
            if chunk:
                buf += chunk
                *lines, buf = buf.split("\n")
                for line in lines:
                    if not line or line.startswith("%"):
                        continue
                    p = line.split()
                    if len(p) < 9:
                        continue
                    try:
                        week, tow = int(p[0]), float(p[1])
                        status = int(p[8])
                        if self.compact:
                            row = (now, week, tow, status)
                        else:
                            row = (now, week, tow, status, float(p[2]), float(p[3]), float(p[4]),
                                   int(p[9]) if len(p) > 9 else 0, float(p[11]) if len(p) > 11 else 0.0)
                    except ValueError:
                        continue
                    self.count += 1
                    if self.compact and self.count > 20000 and self.count % 20:
                        continue
                    self.rows.append(row)
                continue
            if self.stop_event.is_set():
                break
            time.sleep(self.poll)
        if fh is not None:
            fh.close()

    def stop(self) -> None:
        self.stop_event.set()
        self.join(timeout=5.0)


@dataclass
class LiveRun:
    """One gnss_live process lifetime."""

    index: int
    command: list[str]
    pos_path: Path
    stdout_path: Path
    started: float = 0.0
    ended: float = 0.0
    exit_code: int | None = None
    summary: dict[str, Any] = field(default_factory=dict)
    killed_by_harness: bool = False
    outlived_stream: bool = False
    alive_at_halt: bool = False
    rows_total: int = 0
    samples: list[dict[str, float]] = field(default_factory=list)
    rows: list[tuple] = field(default_factory=list)


def parse_summary_line(text: str) -> dict[str, Any]:
    metrics: dict[str, Any] = {}
    for line in reversed(text.splitlines()):
        if line.startswith("summary:"):
            for token in line[len("summary:"):].split():
                if "=" not in token:
                    continue
                key, value = token.split("=", 1)
                try:
                    metrics[key] = float(value) if "." in value else int(value)
                except ValueError:
                    metrics[key] = value
            break
    return metrics


def launch_live(command: list[str], pos_path: Path, stdout_path: Path, index: int, compact: bool,
                sample_interval: float) -> tuple[LiveRun, subprocess.Popen, PosTailer, ResourceSampler]:
    run = LiveRun(index=index, command=command, pos_path=pos_path, stdout_path=stdout_path)
    with open(stdout_path, "wb") as out:
        run.started = time.monotonic()
        proc = subprocess.Popen(command, stdout=out, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
    tailer = PosTailer(pos_path, compact=compact)
    tailer.start()
    sampler = ResourceSampler(proc.pid, sample_interval, comm=Path(command[0]).name)
    sampler.start()
    return run, proc, tailer, sampler


def finish_live(run: LiveRun, proc: subprocess.Popen, tailer: PosTailer, sampler: ResourceSampler) -> None:
    run.exit_code = proc.returncode
    run.ended = time.monotonic()
    sampler.stop()
    time.sleep(0.05)
    tailer.stop()
    run.samples = sampler.samples
    run.rows = tailer.rows
    run.rows_total = tailer.count
    try:
        run.summary = parse_summary_line(run.stdout_path.read_text(errors="replace"))
    except OSError:
        run.summary = {}


# --------------------------------------------------------------------------
# Truth, accuracy and statistics
# --------------------------------------------------------------------------

def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    s = sorted(values)
    if len(s) == 1:
        return s[0]
    k = (len(s) - 1) * q
    lo = int(math.floor(k))
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def linear_slope(xs: list[float], ys: list[float]) -> float | None:
    n = len(xs)
    if n < 3:
        return None
    mx = sum(xs) / n
    my = sum(ys) / n
    den = sum((x - mx) ** 2 for x in xs)
    if den == 0:
        return None
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den


class Reference:
    """PPC-style reference.csv (5 Hz ECEF truth) keyed by 0.1 s tow."""

    def __init__(self, path: Path) -> None:
        self.points: dict[int, tuple[float, float, float, float, float]] = {}
        with open(path, newline="") as fh:
            reader = csv.reader(fh)
            next(reader)
            for row in reader:
                try:
                    tow = float(row[0])
                    lat, lon = math.radians(float(row[2])), math.radians(float(row[3]))
                    x, y, z = float(row[5]), float(row[6]), float(row[7])
                except (ValueError, IndexError):
                    continue
                self.points[int(round(tow * 10))] = (x, y, z, lat, lon)

    def enu_error(self, tow: float, x: float, y: float, z: float) -> tuple[float, float, float] | None:
        ref = self.points.get(int(round(tow * 10)))
        if ref is None:
            return None
        rx, ry, rz, lat, lon = ref
        dx, dy, dz = x - rx, y - ry, z - rz
        sl, cl, so, co = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
        e = -so * dx + co * dy
        n = -sl * co * dx - sl * so * dy + cl * dz
        u = cl * co * dx + cl * so * dy + sl * dz
        return e, n, u


def error_stats(errors: list[tuple[float, float, float, int]]) -> dict[str, Any]:
    """errors: (e, n, u, status). Returns count/fix-rate/horizontal & 3D percentiles."""
    count = len(errors)
    if not count:
        return {"epochs": 0}
    h = [math.hypot(e, n) for e, n, _, _ in errors]
    d3 = [math.sqrt(e * e + n * n + u * u) for e, n, u, _ in errors]
    fixed = [x for x, (_, _, _, s) in zip(d3, errors) if s == 4]
    floats = [x for x, (_, _, _, s) in zip(d3, errors) if s == 3]
    return {
        "epochs": count,
        "fix_rate": sum(1 for *_, s in errors if s == 4) / count,
        "float_rate": sum(1 for *_, s in errors if s == 3) / count,
        "other_rate": sum(1 for *_, s in errors if s not in (3, 4)) / count,
        "h_p50": percentile(h, 0.5), "h_p95": percentile(h, 0.95), "h_max": max(h),
        "d3_p50": percentile(d3, 0.5), "d3_p95": percentile(d3, 0.95), "d3_max": max(d3),
        "fix_d3_p95": percentile(fixed, 0.95),
        "fixed_wrong_gt_0p5m": sum(1 for x in fixed if x > 0.5),
        "float_d3_p50": percentile(floats, 0.5),
        "within_0p5m_rate": sum(1 for x in d3 if x <= 0.5) / count,
    }


def summarize_latency(values: list[float]) -> dict[str, Any]:
    if not values:
        return {"count": 0}
    return {"count": len(values), "p50": percentile(values, 0.5), "p95": percentile(values, 0.95),
            "p99": percentile(values, 0.99), "max": max(values), "mean": sum(values) / len(values)}


def load_pos(path: Path) -> dict[int, tuple[float, float, float, int, int]]:
    """tow(ms) -> (x, y, z, status, nsat) for a libgnss++ .pos file (last row wins)."""
    out: dict[int, tuple[float, float, float, int, int]] = {}
    with open(path) as fh:
        for line in fh:
            if line.startswith("%"):
                continue
            p = line.split()
            if len(p) < 10:
                continue
            try:
                out[int(round(float(p[1]) * 1000))] = (float(p[2]), float(p[3]), float(p[4]), int(p[8]), int(p[9]))
            except ValueError:
                continue
    return out


def status_class(status: int) -> str:
    if status == 4:
        return "FIX"
    if status == 3:
        return "FLOAT"
    if status in (1, 2):
        return "SPP"
    return "OTHER"


def compare_to_batch(live: dict[int, tuple], batch: dict[int, tuple]) -> dict[str, Any]:
    common = sorted(set(live) & set(batch))
    only_live = len(set(live) - set(batch))
    only_batch = len(set(batch) - set(live))
    agree = 0
    both_fix_diff: list[float] = []
    all_diff: list[float] = []
    matrix: dict[str, int] = {}
    for key in common:
        lx, ly, lz, ls, _ = live[key]
        bx, by, bz, bs, _ = batch[key]
        lc, bc = status_class(ls), status_class(bs)
        matrix[f"live_{lc}/batch_{bc}"] = matrix.get(f"live_{lc}/batch_{bc}", 0) + 1
        if lc == bc:
            agree += 1
        d = math.sqrt((lx - bx) ** 2 + (ly - by) ** 2 + (lz - bz) ** 2)
        all_diff.append(d)
        if lc == "FIX" and bc == "FIX":
            both_fix_diff.append(d)
    res: dict[str, Any] = {
        "common_epochs": len(common), "live_only_epochs": only_live, "batch_only_epochs": only_batch,
        "live_epochs": len(live), "batch_epochs": len(batch),
        "status_agreement": (agree / len(common)) if common else None,
        "status_matrix": matrix,
    }
    for name, vals in (("all", all_diff), ("both_fixed", both_fix_diff)):
        res[f"pos_diff_{name}"] = {
            "count": len(vals), "p50": percentile(vals, 0.5), "p95": percentile(vals, 0.95),
            "max": max(vals) if vals else None,
            "identical_to_1mm": sum(1 for v in vals if v < 1e-3),
        }
    return res


# --------------------------------------------------------------------------
# Scenario execution
# --------------------------------------------------------------------------

@dataclass
class Prepared:
    """A prepared data window: encoded streams, truth and batch inputs."""

    root: Path
    manifest: dict[str, Any]

    @classmethod
    def load(cls, root: Path) -> "Prepared":
        root = Path(root)
        return cls(root=root, manifest=json.loads((root / "manifest.json").read_text()))

    def path(self, key: str) -> Path:
        return self.root / self.manifest["files"][key]


@dataclass
class Scenario:
    name: str
    speed: float = 1.0
    transport: str = "tcp"            # tcp | ntrip
    faults: list[Fault] = field(default_factory=list)
    start_offset: float = 0.0         # stream seconds into the prepared window
    duration: float = 0.0             # 0 = whole prepared window
    loops: int = 1                    # 0 = endless until max_wall
    max_wall: float = 0.0             # hard wall-clock cap (0 = none)
    supervise: bool = False           # restart gnss_live if it exits before the stream ends
    restart_delay: float = 1.0
    loop_gap: float = 60.0            # silent gap between repeats of the window (stream s)
    reconnect: bool = False           # run gnss_live with --reconnect (needs a build that has it)
    reconnect_delay_ms: int = 200
    live_args: list[str] = field(default_factory=list)
    sample_interval: float = 1.0
    compact: bool = False
    batch_parity: bool = False
    note: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name, "speed": self.speed, "transport": self.transport,
            "faults": [f.to_dict() for f in self.faults], "start_offset": self.start_offset,
            "duration": self.duration, "loops": self.loops, "max_wall": self.max_wall,
            "supervise": self.supervise, "restart_delay": self.restart_delay, "loop_gap": self.loop_gap,
            "reconnect": self.reconnect, "live_args": self.live_args, "note": self.note,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "Scenario":
        d = dict(d)
        d["faults"] = [Fault(**f) if isinstance(f, dict) else Fault.parse(f) for f in d.get("faults", [])]
        known = {k for k in cls.__dataclass_fields__}
        return cls(**{k: v for k, v in d.items() if k in known})


def _slice_plan(plan: StreamPlan, start_offset: float, duration: float) -> StreamPlan:
    tow0 = plan.tow0
    keep = [e for e in plan.epochs
            if e[0] - tow0 >= start_offset - 1e-6 and (duration <= 0 or e[0] - tow0 <= start_offset + duration + 1e-6)]
    if not keep:
        raise SystemExit("scenario window selects no epochs")
    # the server only needs offsets into `data`; slicing the epoch list is enough
    return StreamPlan(name=plan.name, data=plan.data, epochs=keep)


def _prefix_ephemeris(plan: StreamPlan, upto_index: int) -> bytes:
    """Aux messages (station + ephemeris) already sent before a window that starts mid-run."""
    chunks: dict[tuple[int, int], bytes] = {}
    for i in range(upto_index):
        for frame in split_frames(plan.chunk(i)):
            t = frame_type(frame)
            if t in (1005, 1019, 1020, 1042, 1044, 1045, 1046):
                # 1044 carries a 4-bit QZSS PRN, the other ephemeris messages a 6-bit one
                key = (t, _get_bits(frame, 24 + 12, 4 if t == 1044 else 6) if t != 1005 else 0)
                chunks[key] = frame
    return b"".join(chunks.values())


def build_live_command(binary: str, rover_url: str, base_url: str, pos_path: Path, extra: list[str]) -> list[str]:
    return [binary, "--rover-rtcm", rover_url, "--base-rtcm", base_url, "--out", str(pos_path), "--quiet", *extra]


NORMAL_END_TOKENS = ("rover_source_exhausted", "base_source_exhausted", "completed", "max_epochs_reached")


def _stop_process(proc: subprocess.Popen, run: LiveRun) -> None:
    run.killed_by_harness = True
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def binary_supports(binary: str, flag: str) -> bool:
    try:
        out = subprocess.run([binary, "--help"], capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return flag in (out.stdout + out.stderr)


def reconnect_args(scenario: Scenario) -> list[str]:
    """gnss_live reconnect flags sized to outlast the longest link-down or silent fault (plus margin)."""
    if not scenario.reconnect:
        return []
    kinds = ("rover_disconnect", "base_disconnect", "server_restart", "base_outage", "base_stall", "rover_outage")
    down = max([f.dur for f in scenario.faults if f.kind in kinds] + [0.0])
    wall_down = down / max(scenario.speed, 1e-9)
    delay_s = scenario.reconnect_delay_ms / 1000.0
    # A refused connect fails instantly, so the shortest an attempt can take is the retry delay;
    # sizing by that guarantees the budget outlasts the outage (a quiet NTRIP read takes ~1 s longer).
    attempts = max(10, int(math.ceil((wall_down + 3.0) / delay_s)))
    return ["--reconnect", "--reconnect-delay-ms", str(scenario.reconnect_delay_ms), "--reconnect-attempts", str(attempts)]


def run_scenario(prepared: Prepared, scenario: Scenario, out_dir: Path, bin_dir: str | None = None,
                 log: Any = None, reference: Reference | None = None) -> dict[str, Any]:
    """Run one scenario end to end and return its result dict (also written to out_dir)."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    say = log or (lambda msg: None)
    live_bin = find_binary("gnss_live", bin_dir)

    rover_full = StreamPlan.load("rover", prepared.path("rover_rtcm"), prepared.path("rover_index"))
    base_full = StreamPlan.load("base", prepared.path("base_rtcm"), prepared.path("base_index"))
    rover_plan = _slice_plan(rover_full, scenario.start_offset, scenario.duration)
    base_plan = _slice_plan(base_full, scenario.start_offset, scenario.duration)
    base_prefix = b""
    if scenario.start_offset > 0:
        first_idx = base_full.epochs.index(base_plan.epochs[0])
        base_prefix = _prefix_ephemeris(base_full, first_idx)
    # Loop period (stream seconds between repeats of the window, a multiple of 16 s).
    shift_period = loop_period([rover_plan, base_plan], scenario.loop_gap) if scenario.loops != 1 else 0.0

    clock = StreamClock(scenario.speed)
    ntrip = scenario.transport == "ntrip"
    base_srv = StreamServer("base", base_plan, clock, scenario.faults, ntrip=ntrip, mountpoint="BASE",
                            loops=scenario.loops, shift_period=shift_period, prefix=base_prefix)
    rover_srv = StreamServer("rover", rover_plan, clock, scenario.faults, ntrip=ntrip, mountpoint="ROVER",
                             loops=scenario.loops, shift_period=shift_period, seed=2)

    # After the feed ends gnss_live needs time to notice: a soak may have a backlog to drain, and with
    # --reconnect it retries until its attempt budget (sized to outlast the longest outage) is spent.
    drain_timeout = 180.0 if scenario.max_wall > 0 else 30.0
    flags = reconnect_args(scenario)
    if flags:
        drain_timeout = max(drain_timeout, int(flags[-1]) * scenario.reconnect_delay_ms / 1000.0 * 1.3 + 20.0)

    runs: list[LiveRun] = []
    t_launch = time.monotonic()
    base_srv.start()
    rover_srv.start()
    deadline = (t_launch + scenario.max_wall) if scenario.max_wall > 0 else None
    halted = False

    def window_over() -> bool:
        return base_srv.finished.is_set() and rover_srv.finished.is_set()

    try:
        index = 0
        while True:
            pos_path = out_dir / f"solution.{index}.pos"
            stdout_path = out_dir / f"gnss_live.{index}.log"
            if pos_path.exists():
                pos_path.unlink()
            cmd = build_live_command(live_bin, rover_srv.url, base_srv.url, pos_path,
                                     scenario.live_args + reconnect_args(scenario))
            say(f"[{scenario.name}] launch #{index}")
            run, proc, tailer, sampler = launch_live(cmd, pos_path, stdout_path, index, scenario.compact,
                                                     scenario.sample_interval)
            while proc.poll() is None:
                if deadline is not None and not halted and time.monotonic() > deadline:
                    halted = True
                    run.alive_at_halt = True
                    rover_srv.halt()
                    base_srv.halt()
                if window_over():
                    # Stream finished: let the client drain, then close so it can print its summary.
                    rover_srv.close_after_stream(0.5)
                    base_srv.close_after_stream(0.0)
                    try:
                        # a soak that fell behind real time has a backlog to drain before it can exit
                        proc.wait(timeout=drain_timeout)
                    except subprocess.TimeoutExpired:
                        _stop_process(proc, run)
                    break
                time.sleep(0.05)
            finish_live(run, proc, tailer, sampler)
            run.outlived_stream = bool(window_over() and run.ended >= max(rover_srv.finished_wall, base_srv.finished_wall) - 1.0)
            runs.append(run)
            say(f"[{scenario.name}] gnss_live #{index} exited rc={run.exit_code} "
                f"termination={run.summary.get('termination')} rows={run.rows_total}")
            index += 1
            if window_over() or run.killed_by_harness or not scenario.supervise:
                break
            time.sleep(scenario.restart_delay)
        wait_end = time.monotonic() + (0.0 if window_over() else 1.0)
        while not window_over() and time.monotonic() < wait_end:
            time.sleep(0.05)
    finally:
        rover_srv.stop()
        base_srv.stop()

    result = analyze_scenario(scenario, runs, rover_srv, base_srv, clock, shift_period, reference, out_dir)
    result["wall_total_s"] = time.monotonic() - t_launch
    (out_dir / "result.json").write_text(json.dumps(result, indent=2, default=str) + "\n")
    return result


def analyze_scenario(scenario: Scenario, runs: list[LiveRun], rover_srv: StreamServer, base_srv: StreamServer,
                     clock: StreamClock, shift_period: float, reference: Reference | None,
                     out_dir: Path) -> dict[str, Any]:
    tow0 = rover_srv.plan.tow0

    # Scheduled (== available-to-the-client) wall time of every rover epoch, per tow, across loops.
    sched_by_tow: dict[int, list[float]] = {}
    for entry in rover_srv.log:
        if entry.get("sent"):
            sched_by_tow.setdefault(int(round(entry["tow_out"] * 1000)), []).append(entry["sched_wall"])
    for lst in sched_by_tow.values():
        lst.sort()

    latencies: list[float] = []
    err_rows: list[tuple[float, float, float, int, float]] = []
    rows_out: list[dict[str, Any]] = []
    status_seq: list[tuple[float, int]] = []
    # (data stream time, stream time at which gnss_live wrote the row) per output epoch
    prod_seq: list[tuple[float, float]] = []
    total_rows = 0
    for run in runs:
        for r in run.rows:
            mono, tow, status = r[0], r[2], r[3]
            total_rows += 1
            lst = sched_by_tow.get(int(round(tow * 1000)))
            sched = None
            if lst:
                k = bisect.bisect_right(lst, mono + 1e-3) - 1
                if k >= 0:
                    sched = lst[k]
            latency = (mono - sched) if sched is not None else None
            stream_t = clock.stream_time(sched) if sched is not None else tow - tow0
            if latency is not None:
                latencies.append(latency)
            status_seq.append((stream_t, status))
            prod_t = clock.stream_time(mono)
            prod_seq.append((stream_t, prod_t))
            row: dict[str, Any] = {"run": run.index, "mono": mono, "tow": tow, "stream_t": stream_t,
                                   "status": status, "latency_s": latency, "prod_stream_t": prod_t}
            if len(r) > 4:
                row.update(x=r[4], y=r[5], z=r[6], nsat=r[7], ratio=r[8])
                if reference is not None and shift_period == 0.0:
                    err = reference.enu_error(tow, r[4], r[5], r[6])
                    if err is not None:
                        row.update(e=err[0], n=err[1], u=err[2])
                        err_rows.append((err[0], err[1], err[2], status, stream_t))
            rows_out.append(row)
    status_seq.sort()
    if rows_out:
        cols = ["run", "wall_s", "tow", "stream_t", "status", "latency_s", "nsat", "ratio", "e", "n", "u",
                "prod_stream_t"]
        t_base = runs[0].started
        with open(out_dir / "epochs.csv", "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(cols)
            for row in rows_out:
                w.writerow([f"{row['mono'] - t_base:.4f}" if c == "wall_s" else
                            (f"{row[c]:.4f}" if isinstance(row.get(c), float) else row.get(c, ""))
                            for c in cols])

    all_samples: list[dict[str, float]] = []
    for run in runs:
        for s in run.samples:
            all_samples.append({**s, "run": run.index})
    res_summary: dict[str, Any] = {}
    if all_samples:
        t0 = all_samples[0]["mono"]
        with open(out_dir / "resources.csv", "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["run", "t_s", "rss_mb", "hwm_mb", "cpu_pct", "threads"])
            for s in all_samples:
                w.writerow([s["run"], f"{s['mono'] - t0:.2f}", f"{s['rss_mb']:.2f}", f"{s['hwm_mb']:.2f}",
                            f"{s['cpu_pct']:.1f}", int(s["threads"])])
        res_summary = resource_summary(all_samples)

    for srv in (rover_srv, base_srv):
        with open(out_dir / f"server_{srv.name}.csv", "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["loop", "index", "tow", "stream_t", "sched_s", "send_s", "sent", "dropped_reason", "fault",
                        "bytes", "corrupted_frames", "reordered"])
            for e in srv.log[:300000]:
                w.writerow([e["loop"], e["index"], e["tow"], f"{e['stream_t']:.3f}",
                            f"{e['sched_wall'] - clock.t0:.4f}", f"{e.get('send_wall', 0.0) - clock.t0:.4f}",
                            int(bool(e.get("sent"))), e.get("dropped_reason", ""), e.get("fault", ""), e["bytes"],
                            e.get("corrupted_frames", 0), int(bool(e.get("reordered")))])

    summaries = [r.summary for r in runs]
    agg_keys = ("rover_stalls", "base_stalls", "rover_reconnects", "base_reconnects", "rover_decoder_errors", "base_decoder_errors", "skipped_rover_epochs", "held_base_epochs",
                "interpolated_base_epochs", "exact_base_epochs", "written_solutions", "fixed_solutions",
                "aligned_epochs", "rover_messages", "base_messages")
    live_totals = {k: sum(int(s.get(k, 0) or 0) for s in summaries) for k in agg_keys}
    window_len = rover_srv.plan.span
    fault_reports = evaluate_faults(scenario, runs, status_seq, latencies, rover_srv, base_srv, live_totals,
                                    prod_seq=prod_seq, stream_end=window_len + shift_period * rover_srv.last_loop)

    last = runs[-1] if runs else None
    single = len(runs) == 1
    # A finite window survives when gnss_live ran to the end of the stream and exited normally; an
    # endless soak survives when the process was still running when the wall-clock cap halted the feed.
    survived = bool(single and last is not None and (
        last.alive_at_halt or
        (last.exit_code == 0 and not last.killed_by_harness and last.outlived_stream
         and last.summary.get("termination") in NORMAL_END_TOKENS)))
    planned = rover_srv.counters["epochs_planned"]
    result: dict[str, Any] = {
        "scenario": scenario.to_dict(),
        "survived_to_end": survived,
        "process": [
            {"index": r.index, "exit_code": r.exit_code, "killed_by_harness": r.killed_by_harness,
             "wall_s": r.ended - r.started, "termination": r.summary.get("termination"),
             "outlived_stream": r.outlived_stream, "alive_at_halt": r.alive_at_halt,
             "rows": r.rows_total, "summary": r.summary}
            for r in runs
        ],
        "restarts": max(0, len(runs) - 1),
        # supervised relaunches that could not even connect (the server was still down)
        "failed_launches": sum(1 for r in runs if r.rows_total == 0 and r.exit_code not in (0, None)),
        "input": {
            "rover_epochs_planned": planned,
            "rover_epochs_sent": rover_srv.counters["epochs_sent"],
            "rover_epochs_dropped": rover_srv.counters["epochs_dropped"] + rover_srv.counters["epochs_no_client"],
            "base_epochs_planned": base_srv.counters["epochs_planned"],
            "base_epochs_sent": base_srv.counters["epochs_sent"],
            "base_epochs_dropped": base_srv.counters["epochs_dropped"] + base_srv.counters["epochs_no_client"],
            "rover_counters": rover_srv.counters, "base_counters": base_srv.counters,
        },
        "output_epochs": sum(r.rows_total for r in runs),
        "output_vs_sent_rover": (sum(r.rows_total for r in runs) / rover_srv.counters["epochs_sent"]) if rover_srv.counters["epochs_sent"] else None,
        "latency_s": summarize_latency(latencies),
        "resources": res_summary,
        "live_totals": live_totals,
        "status_counts": _status_counts(status_seq),
        "faults": fault_reports,
        "stream_span_s": window_len + shift_period * rover_srv.last_loop,
        "loops_started": rover_srv.last_loop + 1,
    }
    if err_rows:
        result["accuracy"] = {"all": error_stats([(e, n, u, s) for e, n, u, s, _ in err_rows])}
        result["accuracy"].update(window_accuracy(scenario, err_rows))
    return result


def _status_counts(status_seq: list[tuple[float, int]]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for _, s in status_seq:
        name = STATUS_NAMES.get(s, str(s))
        counts[name] = counts.get(name, 0) + 1
    return counts


def resource_summary(samples: list[dict[str, float]]) -> dict[str, Any]:
    if not samples:
        return {}
    t0 = samples[0]["mono"]
    ts = [s["mono"] - t0 for s in samples]
    rss = [s["rss_mb"] for s in samples]
    hwm = max(s.get("hwm_mb", 0.0) for s in samples)
    cpu = [s["cpu_pct"] for s in samples[1:]]
    slope = linear_slope(ts, rss)
    warm = min(120.0, 0.1 * ts[-1]) if ts[-1] > 0 else 0.0
    ts2 = [t for t in ts if t >= warm]
    rss2 = [r for t, r in zip(ts, rss) if t >= warm]
    slope_post = linear_slope(ts2, rss2) if len(ts2) >= 5 else None
    n = len(rss)
    q = max(1, n // 10)
    return {
        "samples": n, "duration_s": ts[-1],
        "rss_start_mb": rss[0], "rss_end_mb": rss[-1], "rss_peak_mb": max(max(rss), hwm), "rss_min_mb": min(rss),
        "rss_first_decile_mean_mb": sum(rss[:q]) / q, "rss_last_decile_mean_mb": sum(rss[-q:]) / q,
        "rss_slope_mb_per_hour": (slope * 3600.0) if slope is not None else None,
        "rss_slope_after_warmup_mb_per_hour": (slope_post * 3600.0) if slope_post is not None else None,
        "cpu_total_s": samples[-1]["cpu_s"], "cpu_pct_mean": (sum(cpu) / len(cpu)) if cpu else None,
        "cpu_pct_p95": percentile(cpu, 0.95), "cpu_pct_max": max(cpu) if cpu else None,
    }


def window_accuracy(scenario: Scenario, err_rows: list[tuple[float, float, float, int, float]]) -> dict[str, Any]:
    """Error statistics around every fault (stream seconds).

    Windowed faults (outage, stall, disconnect, restart): 30 s before, during, the 30 s after and the
    next 60 s. Open-ended faults (latency, jitter, corruption): 60 s before versus everything after.
    """
    out: dict[str, Any] = {}
    horizon = max((t for *_, t in err_rows), default=0.0) + 1.0
    for idx, f in enumerate(scenario.faults):
        if f.dur > 0:
            a, b = f.at, f.end
            spans = (("pre30", a - 30.0, a), ("during", a, b), ("post0_30", b, b + 30.0), ("post30_90", b + 30.0, b + 90.0))
        else:
            spans = (("pre60", f.at - 60.0, f.at), ("post", f.at, horizon))
        for label, lo, hi in spans:
            sel = [(e, n, u, s) for e, n, u, s, t in err_rows if lo <= t < hi]
            out[f"fault{idx}_{f.kind}_{label}"] = error_stats(sel)
    return out


def evaluate_faults(scenario: Scenario, runs: list[LiveRun], status_seq: list[tuple[float, int]],
                    latencies: list[float], rover_srv: StreamServer, base_srv: StreamServer,
                    totals: dict[str, int], prod_seq: list[tuple[float, float]] | None = None,
                    stream_end: float = 0.0) -> list[dict[str, Any]]:
    """Per fault: was it detected/reported by gnss_live, and how long did FLOAT/FIX take to come back."""
    reports = []
    times = [t for t, _ in status_seq]
    early_exit = [r for r in runs if r.summary.get("termination") in
                  ("rover_source_exhausted", "base_source_exhausted") and not r.outlived_stream]
    for idx, f in enumerate(scenario.faults):
        end = f.end if f.dur > 0 else f.at
        rep: dict[str, Any] = {"index": idx, **f.to_dict()}
        after = [(t, s) for t, s in status_seq if t >= end]
        first_float = next((t for t, s in after if s in (3, 4)), None)
        first_fix = next((t for t, s in after if s == 4), None)
        before = [(t, s) for t, s in status_seq if t < f.at]
        rep["status_before_fault"] = STATUS_NAMES.get(before[-1][1]) if before else None
        rep["recovery_float_s"] = (first_float - end) if first_float is not None else None
        rep["recovery_fix_s"] = (first_fix - end) if first_fix is not None else None
        gap = 0.0
        for t1, t2 in zip(times, times[1:]):
            if t2 >= f.at and t1 <= end + 60.0:
                gap = max(gap, t2 - t1)
        rep["max_output_gap_s"] = gap
        if prod_seq is not None:
            # "Recovery" as seen by a user: when the first row for post-fault data was written
            # (production time), alongside the data time of that first row.
            post = [(d, p) for d, p in prod_seq if d >= f.at]
            rep["first_solution_after_s"] = (min(p for _, p in post) - f.at) if post else None
            rep["first_solution_data_s"] = (min(d for d, _ in post) - f.at) if post else None
            rep["solutions_from_30s_after"] = sum(1 for d, _ in prod_seq if f.at + 30.0 <= d < stream_end + 1e-6)
        evidence: list[str] = []
        kind = f.kind
        if kind.startswith("corrupt_"):
            target = kind.split("_", 1)[1]
            srv = rover_srv if target == "rover" else base_srv
            errs = totals.get(f"{target}_decoder_errors", 0)
            rep["frames_corrupted_by_server"] = srv.counters["frames_corrupted"]
            rep["decoder_errors_reported"] = errs
            if errs > 0:
                evidence.append(f"{target}_decoder_errors={errs}")
        elif kind in ("rover_disconnect", "base_disconnect", "server_restart"):
            rep["process_exits_before_stream_end"] = len(early_exit)
            rep["process_restarts"] = max(0, len(runs) - 1)
            rep["connections"] = {"rover": rover_srv.counters["connections"], "base": base_srv.counters["connections"]}
            if rover_srv.counters["connections"] > 1 or base_srv.counters["connections"] > 1:
                evidence.append("client_reconnected")
            if early_exit:
                evidence.append("process_exit:" + str(early_exit[0].summary.get("termination")))
        elif kind in BLACKHOLE_KINDS:
            target = kind.split("_", 1)[0]
            srv = rover_srv if target == "rover" else base_srv
            stalls = totals.get(f"{target}_stalls", 0)
            rep["epochs_blackholed"] = srv.counters["epochs_blackholed"]
            rep["connections_blackholed"] = srv.counters["connections_blackholed"]
            rep["connections"] = {"rover": rover_srv.counters["connections"], "base": base_srv.counters["connections"]}
            rep[f"{target}_stalls_reported"] = stalls
            rep["process_exits_before_stream_end"] = len(early_exit)
            if stalls > 0:
                evidence.append(f"{target}_stalls={stalls}")
            if srv.counters["connections"] > 1:
                evidence.append("client_reconnected")
            if gap >= 1.0:
                evidence.append(f"output_gap={gap:.1f}s")
            if early_exit:
                evidence.append("process_exit:" + str(early_exit[0].summary.get("termination")))
            stalled_exit = [r for r in runs if str(r.summary.get("termination", "")).endswith("_source_stalled")]
            if stalled_exit:
                evidence.append("process_exit:" + str(stalled_exit[0].summary.get("termination")))
        elif kind in ("base_outage", "base_stall", "rover_outage"):
            if kind == "base_stall" and latencies and max(latencies) >= 0.5 * f.dur / max(scenario.speed, 1e-9):
                evidence.append(f"latency_max={max(latencies):.2f}s")
            if totals.get("held_base_epochs", 0) > 0:
                evidence.append(f"held_base_epochs={totals['held_base_epochs']}")
            if totals.get("skipped_rover_epochs", 0) > 0:
                evidence.append(f"skipped_rover_epochs={totals['skipped_rover_epochs']}")
            if gap >= max(1.0, 0.5 * f.dur):
                evidence.append(f"output_gap={gap:.1f}s")
            if early_exit:
                evidence.append("process_exit:" + str(early_exit[0].summary.get("termination")))
        elif kind in ("base_latency", "base_jitter"):
            p95 = percentile(latencies, 0.95) if latencies else None
            rep["latency_p95_s"] = p95
            reordered = base_srv.counters["epochs_reordered"]
            rep["base_epochs_reordered"] = reordered
            if reordered:
                evidence.append(f"base_epochs_reordered={reordered}")
            if p95 is not None and p95 > 0.5 * f.value / max(scenario.speed, 1e-9):
                evidence.append(f"latency_p95={p95:.2f}s")
            if totals.get("held_base_epochs", 0) > 0:
                evidence.append(f"held_base_epochs={totals['held_base_epochs']}")
            if early_exit:
                evidence.append("process_exit:" + str(early_exit[0].summary.get("termination")))
        rep["detected"] = bool(evidence)
        rep["evidence"] = evidence
        reports.append(rep)
    return reports
