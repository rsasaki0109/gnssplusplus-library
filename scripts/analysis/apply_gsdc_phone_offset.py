"""Apply the pinned taroz add_position_offset per-phone constants to a native trajectory.

Heading comes from the trajectory itself (no truth, no external positions).  The
phone offset is [UD along heading, RL to the left], matching the upstream
eul2rotm(rpy-[0 0 pi]) mapping for level attitude.  Low-speed epochs hold the
last moving heading; epochs before the first motion use the first moving heading.
"""
from __future__ import annotations

import numpy as np

EARTH_RADIUS_M = 6371008.8


def phone_offset(phone: str) -> tuple[float, float]:
    """Return (RL, UD) in metres, in upstream branch order."""
    if "mi8" in phone:
        return 0.25, -0.35
    if phone == "sm-g988":
        return 0.20, -0.05
    if "sm" in phone or "samsung" in phone:
        return 0.30, -0.25
    if phone == "pixel6pro":
        return -0.20, -0.15
    if "pixel7" in phone:
        return -0.10, -0.20
    if "pixel4" in phone:
        return 0.0, -0.15
    if "pixel5" in phone:
        return -0.10, -0.30
    raise ValueError(f"unknown phone {phone!r}")


def apply_offset(lat_deg: np.ndarray, lon_deg: np.ndarray, phone: str,
                 min_step_m: float = 0.3) -> tuple[np.ndarray, np.ndarray]:
    lat0 = np.radians(lat_deg[0])
    east = np.radians(lon_deg) * EARTH_RADIUS_M * np.cos(lat0)
    north = np.radians(lat_deg) * EARTH_RADIUS_M
    ve, vn = np.gradient(east), np.gradient(north)
    moving = np.hypot(ve, vn) > min_step_m
    heading = np.arctan2(vn, ve)
    if moving.any():
        last = heading[np.argmax(moving)]
        for i in range(len(heading)):
            if moving[i]:
                last = heading[i]
            else:
                heading[i] = last
    else:
        heading[:] = 0.0
    rl, ud = phone_offset(phone)
    east = east + ud * np.cos(heading) - rl * np.sin(heading)
    north = north + ud * np.sin(heading) + rl * np.cos(heading)
    return np.degrees(north / EARTH_RADIUS_M), np.degrees(east / (EARTH_RADIUS_M * np.cos(lat0)))
