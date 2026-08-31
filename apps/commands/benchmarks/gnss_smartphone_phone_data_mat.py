#!/usr/bin/env python3
"""Convert the numeric IMU part of taroz ``phone_data.mat`` to loadImuCsv CSV.

The public taroz MAT file is MATLAB v5 and contains ``gt.Gtime`` MCOS opaque
objects in ``timems``.  The numeric ``utcms``, ``elapsedns`` and ``xyz`` fields
of the ``acc``/``gyro`` structs are sufficient for a deterministic converter;
the opaque time object is deliberately ignored.  This is a schema adapter,
not an estimator: gyro timestamps are anchors, the nearest accelerometer is
selected within the existing 25 ms contract, and Android UTC milliseconds are
converted to GPST with the declared 18 second offset.

The converter is optional and isolated from the C++ solver.  It requires
SciPy only when invoked; no truth or navigation variables are read.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any

GPS_EPOCH_UNIX_SECONDS = 315964800.0
GPS_UTC_LEAP_SECONDS = 18.0
SECONDS_PER_WEEK = 604800.0
PAIR_LIMIT_MS = 25


class MatImuError(ValueError):
    """Raised for an unsupported or unsafe phone_data.mat schema."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_bytes(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _field(struct: Any, name: str) -> Any:
    if hasattr(struct, name):
        return getattr(struct, name)
    if isinstance(struct, dict) and name in struct:
        return struct[name]
    raise MatImuError(f"MAT struct lacks required field {name!r}")


def _vector_field(struct: Any, name: str, columns: int) -> list[tuple[float, ...]]:
    try:
        import numpy as np
    except ImportError as exc:  # pragma: no cover - environment-specific
        raise MatImuError("SciPy/numpy is required for phone_data.mat conversion") from exc
    values = np.asarray(_field(struct, name), dtype=float)
    if values.ndim != 2 or values.shape[1] != columns:
        raise MatImuError(f"MAT field {name!r} must have shape (n,{columns})")
    result: list[tuple[float, ...]] = []
    for row in values:
        converted = tuple(float(value) for value in row)
        if not all(math.isfinite(value) for value in converted):
            raise MatImuError(f"MAT field {name!r} contains non-finite values")
        result.append(converted)
    return result


def _times(struct: Any) -> list[int]:
    try:
        import numpy as np
    except ImportError as exc:  # pragma: no cover
        raise MatImuError("SciPy/numpy is required for phone_data.mat conversion") from exc
    values = np.asarray(_field(struct, "utcms"), dtype=float).reshape(-1)
    result: list[int] = []
    for value in values:
        if not math.isfinite(float(value)) or value < 0:
            raise MatImuError("MAT utcms contains an invalid timestamp")
        result.append(int(round(float(value))))
    return result


def _deduplicate(times: list[int], values: list[tuple[float, float, float]], label: str) -> dict[int, tuple[float, float, float]]:
    if len(times) != len(values):
        raise MatImuError(f"MAT {label} time/value lengths differ")
    result: dict[int, tuple[float, float, float]] = {}
    for timestamp, value in zip(times, values):
        previous = result.get(timestamp)
        if previous is not None and max(abs(a - b) for a, b in zip(previous, value)) > 1e-9:
            raise MatImuError(f"MAT {label} has inconsistent duplicate timestamp {timestamp}")
        result[timestamp] = value
    return result


def _to_gpst(timestamp_ms: int) -> tuple[int, float]:
    gpst = timestamp_ms / 1000.0 - GPS_EPOCH_UNIX_SECONDS + GPS_UTC_LEAP_SECONDS
    if not math.isfinite(gpst) or gpst < 0:
        raise MatImuError("MAT timestamp cannot be represented as GPST")
    week = math.floor(gpst / SECONDS_PER_WEEK)
    return week, gpst - week * SECONDS_PER_WEEK


def convert_mat(input_path: Path, output_path: Path, summary_path: Path | None = None) -> dict[str, Any]:
    try:
        from scipy.io import loadmat
    except ImportError as exc:  # pragma: no cover - environment-specific
        raise MatImuError("SciPy is required for phone_data.mat conversion") from exc
    try:
        data = loadmat(input_path, squeeze_me=True, struct_as_record=False)
    except Exception as exc:
        raise MatImuError(f"failed to decode MATLAB v5 file: {input_path}") from exc
    if "acc" not in data or "gyro" not in data:
        raise MatImuError("phone_data.mat must contain acc and gyro structs")
    acc = data["acc"]
    gyro = data["gyro"]
    acc_stream = _deduplicate(_times(acc), _vector_field(acc, "xyz", 3), "acc")
    gyro_stream = _deduplicate(_times(gyro), _vector_field(gyro, "xyz", 3), "gyro")
    if not acc_stream or not gyro_stream:
        raise MatImuError("phone_data.mat contains an empty acc or gyro stream")
    acc_times = sorted(acc_stream)
    rows: list[str] = [
        "GPS TOW (s),GPS Week,Acc X (m/s^2),Acc Y (m/s^2),Acc Z (m/s^2),"
        "Ang Rate X (deg/s),Ang Rate Y (deg/s),Ang Rate Z (deg/s)\n"
    ]
    omitted = 0
    offsets: list[int] = []
    for timestamp in sorted(gyro_stream):
        index = bisect.bisect_left(acc_times, timestamp)
        choices = []
        if index < len(acc_times):
            choices.append(acc_times[index])
        if index:
            choices.append(acc_times[index - 1])
        if not choices:
            omitted += 1
            continue
        nearest = min(choices, key=lambda candidate: (abs(candidate - timestamp), candidate))
        offset = nearest - timestamp
        if abs(offset) > PAIR_LIMIT_MS:
            omitted += 1
            continue
        week, tow = _to_gpst(timestamp)
        accel = acc_stream[nearest]
        gyro_degps = tuple(math.degrees(value) for value in gyro_stream[timestamp])
        values = (tow, week, *accel, *gyro_degps)
        if not all(math.isfinite(float(value)) for value in values):
            raise MatImuError("MAT conversion produced a non-finite output")
        rows.append(
            f"{tow:.9f},{week},{accel[0]:.9f},{accel[1]:.9f},{accel[2]:.9f},"
            f"{gyro_degps[0]:.9f},{gyro_degps[1]:.9f},{gyro_degps[2]:.9f}\n"
        )
        offsets.append(offset)
    if not offsets:
        raise MatImuError("MAT conversion produced no paired samples")
    _atomic_bytes(output_path, "".join(rows).encode("ascii"))
    summary: dict[str, Any] = {
        "schema_version": "smartphone-r5-phone-data-mat-imu-converter.v1",
        "input": str(input_path),
        "output": str(output_path),
        "input_sha256": _sha256(input_path),
        "output_sha256": _sha256(output_path),
        "mat_version": "MAT v5 numeric acc/gyro fields; gt.Gtime opaque timems ignored",
        "fields": {"acc": ["n", "utcms", "xyz"], "gyro": ["n", "utcms", "xyz"]},
        "acc_rows": len(acc_stream),
        "gyro_rows": len(gyro_stream),
        "paired_rows": len(offsets),
        "omitted_outside_25ms": omitted,
        "pair_limit_ms": PAIR_LIMIT_MS,
        "pair_offset_ms": {"min": min(offsets), "max": max(offsets)},
        "timestamp": {"source": "utcms UTC milliseconds", "gps_utc_leap_seconds": 18},
        "units": {"accel": "m/s^2", "gyro_input": "rad/s", "gyro_output": "deg/s"},
        "axis_policy": "raw Android axes remain explicit; mounting rotation is applied by native FGO entry point",
        "truth_used": False,
        "navigation_used": False,
    }
    if summary_path is not None:
        _atomic_bytes(summary_path, (json.dumps(summary, indent=2, sort_keys=True) + "\n").encode("utf-8"))
        summary["summary_sha256"] = _sha256(summary_path)
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="taroz phone_data.mat")
    parser.add_argument("--output", type=Path, required=True, help="loadImuCsv-compatible CSV")
    parser.add_argument("--summary-json", type=Path)
    args = parser.parse_args()
    try:
        summary = convert_mat(args.input, args.output, args.summary_json)
    except (MatImuError, OSError) as exc:
        parser.error(str(exc))
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
