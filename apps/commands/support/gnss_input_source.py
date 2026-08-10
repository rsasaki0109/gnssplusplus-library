#!/usr/bin/env python3
"""Binary file and POSIX serial input shared by receiver CLI tools."""

from __future__ import annotations

import errno
import os
from typing import BinaryIO

if os.name != "nt":
    import termios


DEFAULT_SERIAL_BAUD = 115200
DEFAULT_SERIAL_BAUDS = (9600, 19200, 38400, 57600, 115200, 230400)
NMEA_SERIAL_BAUDS = (4800, 9600, 19200, 38400, 57600, 115200)


def is_serial_path(path: str) -> bool:
    return path.startswith("serial://") or path.startswith("/dev/tty")


def parse_serial_path(path: str, default_baud: int = DEFAULT_SERIAL_BAUD) -> tuple[str, int]:
    raw = path[len("serial://") :] if path.startswith("serial://") else path
    if "?baud=" in raw:
        device, baud_text = raw.split("?baud=", 1)
        return device, int(baud_text)
    return raw, default_baud


def configure_serial(
    fd: int,
    baud: int,
    allowed_bauds: tuple[int, ...] = DEFAULT_SERIAL_BAUDS,
) -> None:
    if os.name == "nt":
        raise RuntimeError("serial input is not supported on this platform")
    baud_map = {
        allowed_baud: getattr(termios, f"B{allowed_baud}")
        for allowed_baud in allowed_bauds
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
    def __init__(
        self,
        path: str,
        *,
        default_baud: int = DEFAULT_SERIAL_BAUD,
        allowed_bauds: tuple[int, ...] = DEFAULT_SERIAL_BAUDS,
        eio_as_eof: bool = True,
    ) -> None:
        self.path = path
        self.default_baud = default_baud
        self.allowed_bauds = allowed_bauds
        self.eio_as_eof = eio_as_eof
        self.handle: BinaryIO | None = None
        self.fd: int | None = None

    def open(self) -> bool:
        if is_serial_path(self.path):
            device, baud = parse_serial_path(self.path, self.default_baud)
            fd = os.open(device, os.O_RDONLY | os.O_NOCTTY)
            configure_serial(fd, baud, self.allowed_bauds)
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
                if self.eio_as_eof and getattr(exc, "errno", None) == errno.EIO:
                    return b""
                raise
        return b""
