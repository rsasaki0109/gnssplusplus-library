#!/usr/bin/env python3
"""Promote the public taroz GSDC 2023 MAT results without running MATLAB.

The pinned public archive contains the final ``result_gnss.mat`` and
``result_gnss_imu.mat`` files.  MATLAB class objects in those files are not
directly supported by SciPy, but their MAT-v5 ``MCOS/FileWrapper__`` workspace
is a deterministic container.  This command decodes only the published
``gt.Gpos.llh`` property and never reads a test ground-truth member.

This is an isolated research/promotional lane.  It does not change the native
RTK/SPP defaults and it deliberately does not copy the public MAT archive into
the repository.  All writes use an fsync/rename publish and all batch records
carry source hashes.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import hashlib
import io
import json
import math
import os
from pathlib import Path
import struct
import sys
import tempfile
from typing import Any, Iterable, Sequence
from zipfile import ZipFile, ZipInfo


COMMANDS_DIR = Path(__file__).resolve().parents[1]
if str(COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(COMMANDS_DIR))
from support.gnss_runtime import application_root  # noqa: E402


ROOT = application_root(__file__)
DEFAULT_ARCHIVE = ROOT / "data/gsdc2023/cache/dataset_2023.zip"
DEFAULT_SAMPLE = ROOT / "data/gsdc2023/sample_submission.csv"
SCHEMA = "smartphone-r5-gsdc2023-upstream-mat.v1"
INVENTORY_SCHEMA = SCHEMA + "-mat-inventory"
MAX_MEMBER_BYTES = 256 * 1024 * 1024
INTERPOLATION_MAX_GAP_MS = 10_000
EDGE_HOLD_MAX_GAP_MS = 1_000
MIN_ECEF_NORM_M = 6_000_000.0
MAX_ECEF_NORM_M = 7_000_000.0
# This constant is recorded as a known sample artifact sentinel.  It is not
# read from, or copied out of, sample coordinates by this module.
KNOWN_SAMPLE_DUMMY = (34.640195, -120.589642)
MAX_TRANSITION_SPEED_MPS = 70.0


class UpstreamMatError(ValueError):
    """Raised for malformed or unsafe published MAT/archive data."""


@dataclass(frozen=True)
class Position:
    trip_id: str
    timestamp_ms: int
    latitude: float
    longitude: float
    height_m: float
    source: str


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    if not path.is_file():
        raise UpstreamMatError(f"missing file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_bytes(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    atomic_bytes(path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def _as_bytes(value: Any) -> bytes:
    """Convert a MATLAB scalar byte/string cell to bytes without guessing."""
    try:
        import numpy as np
    except ImportError as exc:  # pragma: no cover - environment-specific
        raise UpstreamMatError("numpy is required for MAT decoding") from exc
    if isinstance(value, bytes):
        return value
    array = np.asarray(value).reshape(-1)
    if array.size != 1:
        raise UpstreamMatError("MCOS string field is not scalar")
    item = array[0]
    if isinstance(item, bytes):
        return item
    if isinstance(item, str):
        return item.encode("utf-8")
    if isinstance(item, np.ndarray):
        return bytes(int(number) for number in item.reshape(-1))
    raise UpstreamMatError(f"unsupported MCOS string field: {type(item)!r}")


def _numeric_cell(value: Any, label: str) -> Any:
    try:
        import numpy as np
    except ImportError as exc:  # pragma: no cover
        raise UpstreamMatError("numpy is required for MAT decoding") from exc
    array = np.asarray(value)
    if array.dtype.kind not in "biufc":
        raise UpstreamMatError(f"MCOS property {label!r} is not numeric")
    return array


def _workspace_from_bytes(workspace: bytes) -> list[Any]:
    """Decode the R2024a FileWrapper workspace into its heap cells.

    The payload is itself a short MAT-v5 stream beginning with the endian
    marker.  A normal 128-byte MAT header is supplied only to SciPy's reader;
    no MATLAB object execution or deserialization is involved.
    """
    try:
        import numpy as np
        from scipy.io.matlab._mio5 import MatFile5Reader
    except ImportError as exc:  # pragma: no cover
        raise UpstreamMatError("SciPy/numpy are required for MCOS MAT decoding") from exc
    if len(workspace) < 8 or workspace[:2] != b"\x00\x01" or workspace[2:4] != b"IM":
        raise UpstreamMatError("unsupported MCOS workspace endian/version marker")
    if workspace[4:8] != b"\x00\x00\x00\x00":
        raise UpstreamMatError("MCOS workspace padding marker is invalid")
    fake_header = (
        b"MATLAB 5.0 MAT-file".ljust(116, b" ")
        + b"\x00" * 8
        + workspace[:4]
        + workspace[8:]
    )
    reader = MatFile5Reader(
        io.BytesIO(fake_header), struct_as_record=True, squeeze_me=False, mat_dtype=False
    )
    reader.initialize_read()
    reader.read_file_header()
    header, _ = reader.read_var_header()
    value = reader.read_var_array(header)
    if not isinstance(value, np.ndarray) or value.dtype.names != ("MCOS",):
        raise UpstreamMatError("MCOS workspace does not contain the FileWrapper struct")
    outer = value["MCOS"][0, 0]
    if not isinstance(outer, np.ndarray) or outer.size != 1:
        raise UpstreamMatError("MCOS FileWrapper object is not scalar")
    wrapper = outer.reshape(-1)[0]
    if not hasattr(wrapper, "dtype") or "arr" not in wrapper.dtype.names:
        raise UpstreamMatError("MCOS FileWrapper opaque object is malformed")
    cells = np.asarray(wrapper["arr"]).reshape(-1)
    if cells.size < 5:
        raise UpstreamMatError("MCOS workspace heap is too small")
    return list(cells)


def _u32(data: bytes, offset: int) -> int:
    if offset < 0 or offset + 4 > len(data):
        raise UpstreamMatError("MCOS segment offset is outside its payload")
    return struct.unpack_from("<I", data, offset)[0]


def _u32_tuple(data: bytes, offset: int, count: int) -> tuple[int, ...]:
    if offset < 0 or offset + 4 * count > len(data):
        raise UpstreamMatError("MCOS table record is outside its payload")
    return struct.unpack_from("<" + "I" * count, data, offset)


def _parse_mcos_properties(cells: Sequence[Any]) -> dict[int, dict[str, Any]]:
    """Return object-id -> property values for the observed MATLAB classdef form."""
    import numpy as np

    segment = bytes(np.asarray(cells[0], dtype=np.uint8).reshape(-1))
    if len(segment) < 40:
        raise UpstreamMatError("MCOS segment table is truncated")
    version, nstrings, h2, h3, h4, h5, h6, h7, h8, h9 = _u32_tuple(segment, 0, 10)
    if version not in (2, 4):
        raise UpstreamMatError(f"unsupported MCOS segment version: {version}")
    bounds = (40, h2, h3, h4, h5, h6, h7, h8, h9)
    if any(value > len(segment) for value in bounds) or any(
        left > right for left, right in zip(bounds, bounds[1:])
    ):
        raise UpstreamMatError("MCOS segment table offsets are not monotonic")
    string_values = segment[40:h2].split(b"\x00")
    while string_values and string_values[-1] == b"":
        string_values.pop()
    if len(string_values) < nstrings:
        raise UpstreamMatError("MCOS string table is incomplete")

    class_names: list[tuple[str, str]] = []
    if (h3 - h2) % 16:
        raise UpstreamMatError("MCOS class table has partial records")
    for offset in range(h2, h3, 16):
        package_index, class_index, _, _ = _u32_tuple(segment, offset, 4)
        if package_index == 0 and class_index == 0:
            class_names.append(("", ""))
            continue
        if class_index > len(string_values) or package_index > len(string_values):
            raise UpstreamMatError("MCOS class name index is invalid")
        class_names.append(
            (
                string_values[package_index - 1].decode("utf-8") if package_index else "",
                string_values[class_index - 1].decode("utf-8"),
            )
        )

    if (h5 - h4) % 24:
        raise UpstreamMatError("MCOS object table has partial records")
    object_records: list[tuple[int, int, int, int]] = []
    for ordinal, offset in enumerate(range(h4, h5, 24)):
        class_index, _, _, _, property_index, object_id = _u32_tuple(segment, offset, 6)
        if object_id:
            if class_index >= len(class_names):
                raise UpstreamMatError("MCOS object class index is invalid")
            # The opaque reference stores the one-based object-table ordinal,
            # while the record's last field is the FileWrapper object id.  Keep
            # both namespaces; result MATs happen to use equal values, but
            # GobsPhone in phone_data.mat does not.
            object_records.append((ordinal, object_id, class_index, property_index))

    property_blocks: list[dict[str, tuple[int, int]]] = []
    offset = h5
    while offset < h6:
        if offset + 8 > h6:
            raise UpstreamMatError("MCOS property block header is truncated")
        count = _u32(segment, offset)
        offset += 4  # count; records start immediately after the count
        if count > (h6 - offset) // 12:
            raise UpstreamMatError("MCOS property block record count is unsafe")
        block: dict[str, tuple[int, int]] = {}
        for _ in range(count):
            name_index, flag, heap_index = _u32_tuple(segment, offset, 3)
            offset += 12
            if not 1 <= name_index <= len(string_values):
                raise UpstreamMatError("MCOS property name index is invalid")
            block[string_values[name_index - 1].decode("utf-8")] = (flag, heap_index)
        offset = (offset + 7) & ~7
        property_blocks.append(block)
    if offset != h6:
        raise UpstreamMatError("MCOS property block alignment is invalid")

    def resolve(flag: int, heap_index: int) -> Any:
        if flag == 1:
            # FileWrapper.arr has the segment table and an initial empty cell
            # before the zero-based property heap used by this format.
            cell_index = heap_index + 2
            if not 0 <= cell_index < len(cells):
                raise UpstreamMatError("MCOS heap index is outside FileWrapper.arr")
            return cells[cell_index]
        if flag == 0:
            if not 0 <= heap_index < len(string_values):
                raise UpstreamMatError("MCOS string property index is invalid")
            return string_values[heap_index].decode("utf-8")
        if flag == 2:
            return bool(heap_index)
        raise UpstreamMatError(f"unsupported MCOS property flag: {flag}")

    resolved: dict[int, dict[str, Any]] = {}
    for ordinal, object_id, class_index, property_index in object_records:
        if not 0 <= property_index < len(property_blocks):
            raise UpstreamMatError("MCOS object property block index is invalid")
        package, class_name = class_names[class_index]
        props: dict[str, Any] = {
            "__class__": f"{package}.{class_name}" if package else class_name
        }
        for name, (flag, heap_index) in property_blocks[property_index].items():
            props[name] = resolve(flag, heap_index)
        resolved[object_id] = props
        resolved[-ordinal] = props
    return resolved


def _opaque_reference(value: Any) -> tuple[str, int, int]:
    import numpy as np

    array = np.asarray(value).reshape(-1)
    if array.size != 1 or not hasattr(array[0], "dtype"):
        raise UpstreamMatError("MAT opaque reference is not scalar")
    opaque = array[0]
    label = _as_bytes(opaque["s0"]).decode("utf-8")
    kind = _as_bytes(opaque["s1"]).decode("utf-8")
    class_name = _as_bytes(opaque["s2"]).decode("utf-8")
    if kind != "MCOS":
        raise UpstreamMatError(f"{label} is not an MCOS object")
    ids = np.asarray(opaque["arr"], dtype=np.uint32).reshape(-1)
    if ids.size < 6 or int(ids[0]) != 0xDD000000:
        raise UpstreamMatError(f"{label} has an invalid MCOS object-id header")
    return class_name, int(ids[-2]), int(ids[-1])


def decode_result_bytes(payload: bytes, *, allow_missing_rows: bool = False) -> dict[str, Any]:
    """Decode one published result MAT into finite LLH and numeric sidecars."""
    try:
        import numpy as np
        from scipy.io import loadmat
        from scipy.io.matlab import varmats_from_mat
    except ImportError as exc:  # pragma: no cover
        raise UpstreamMatError("SciPy/numpy are required for result MAT decoding") from exc
    if len(payload) > MAX_MEMBER_BYTES:
        raise UpstreamMatError("result MAT exceeds the safety size limit")
    opaque_refs: dict[str, tuple[str, int, int]] = {}
    workspace: bytes | None = None
    numeric: dict[str, Any] = {}
    try:
        streams = list(varmats_from_mat(io.BytesIO(payload)))
    except Exception as exc:
        raise UpstreamMatError("failed to split MAT-v5 variables") from exc
    for name, stream in streams:
        try:
            parsed = loadmat(
                stream, struct_as_record=True, squeeze_me=False, mat_dtype=False
            )
        except Exception as exc:
            raise UpstreamMatError(f"failed to decode MAT variable {name!r}") from exc
        if name == "None":
            opaque = parsed.get("None")
            if opaque is None:
                raise UpstreamMatError("opaque result variable is missing")
            class_name, object_id, class_id = _opaque_reference(opaque)
            # The duplicate opaque stream carries its MATLAB variable name in s0.
            label = _as_bytes(opaque["s0"]).decode("utf-8")
            opaque_refs[label] = (class_name, object_id, class_id)
        elif name in {"", "__function_workspace__"} and "__function_workspace__" in parsed:
            value = parsed.get("__function_workspace__")
            if value is None:
                raise UpstreamMatError("MCOS workspace variable is missing")
            workspace = bytes(np.asarray(value, dtype=np.uint8).reshape(-1))
        elif name in {"clkest", "dclkest", "rpyest", "imubiasest"}:
            numeric[name] = np.asarray(parsed[name], dtype=float)
    if workspace is None or "posest" not in opaque_refs:
        raise UpstreamMatError("result MAT lacks posest and MCOS workspace")
    class_name, object_id, _ = opaque_refs["posest"]
    if class_name != "gt.Gpos":
        raise UpstreamMatError(f"posest class mismatch: {class_name!r}")
    objects = _parse_mcos_properties(_workspace_from_bytes(workspace))
    props = objects.get(object_id)
    if props is None or props.get("__class__") != "gt.Gpos":
        raise UpstreamMatError("MCOS workspace has no matching gt.Gpos object")
    llh = np.asarray(_numeric_cell(props.get("llh"), "llh"), dtype=float)
    if llh.ndim != 2 or llh.shape[1] != 3 or llh.shape[0] == 0:
        raise UpstreamMatError(f"gt.Gpos.llh has invalid shape: {llh.shape}")
    finite_rows = np.isfinite(llh).all(axis=1)
    if not finite_rows.any():
        raise UpstreamMatError("gt.Gpos.llh contains no finite rows")
    if not allow_missing_rows and not finite_rows.all():
        raise UpstreamMatError("gt.Gpos.llh has non-finite rows")
    finite_llh = llh[finite_rows]
    if (finite_llh[:, 0] < -90).any() or (finite_llh[:, 0] > 90).any():
        raise UpstreamMatError("gt.Gpos.llh has invalid latitude")
    if (finite_llh[:, 1] < -180).any() or (finite_llh[:, 1] > 180).any():
        raise UpstreamMatError("gt.Gpos.llh has invalid longitude")
    n_value = np.asarray(_numeric_cell(props.get("n"), "n"), dtype=float).reshape(-1)
    if n_value.size != 1 or int(round(float(n_value[0]))) != llh.shape[0]:
        raise UpstreamMatError("gt.Gpos.n does not match gt.Gpos.llh")
    return {
        "llh": llh,
        "finite_rows": finite_rows,
        "numeric": numeric,
        "workspace_sha256": sha256_bytes(workspace),
    }


def timestamps_from_phone_data_bytes(payload: bytes) -> list[int]:
    """Extract the exact ``obs.utcms`` epochs used by upstream submission.m."""
    try:
        import numpy as np
        from scipy.io import loadmat
        from scipy.io.matlab import varmats_from_mat
    except ImportError as exc:  # pragma: no cover
        raise UpstreamMatError("SciPy/numpy are required for phone_data.mat decoding") from exc
    observation_index: int | None = None
    workspace: bytes | None = None
    for name, stream in varmats_from_mat(io.BytesIO(payload)):
        parsed = loadmat(stream, struct_as_record=True, squeeze_me=False, mat_dtype=False)
        if name == "None":
            opaque = parsed.get("None")
            if opaque is None:
                continue
            label = _as_bytes(opaque["s0"]).decode("utf-8")
            if label == "obs":
                _, observation_index, _ = _opaque_reference(opaque)
        elif name == "" and "__function_workspace__" in parsed:
            workspace = bytes(np.asarray(parsed["__function_workspace__"], dtype=np.uint8).reshape(-1))
    if observation_index is None or workspace is None:
        raise UpstreamMatError("phone_data.mat lacks obs MCOS workspace")
    props = _parse_mcos_properties(_workspace_from_bytes(workspace)).get(-observation_index)
    if props is None or props.get("__class__") != "GobsPhone":
        raise UpstreamMatError("phone_data.mat obs reference does not resolve to GobsPhone")
    values = np.asarray(_numeric_cell(props.get("utcms"), "utcms"), dtype=float).reshape(-1)
    if values.size == 0 or not np.isfinite(values).all():
        raise UpstreamMatError("phone_data.mat obs.utcms is empty or non-finite")
    timestamps = [int(round(float(value))) for value in values]
    if any(value < 0 for value in timestamps) or timestamps != sorted(set(timestamps)):
        raise UpstreamMatError("phone_data.mat obs.utcms is not strictly increasing")
    return timestamps


def _csv_header(reader: csv.DictReader, expected: Sequence[str], path: Path) -> None:
    actual = tuple(reader.fieldnames or ())
    if actual != tuple(expected):
        raise UpstreamMatError(
            f"{path} header must be {','.join(expected)}, got {','.join(actual)}"
        )


def read_unique_timestamps(path: Path) -> list[int]:
    """Read the epoch key source used by GobsPhone (device UTC milliseconds)."""
    values: list[int] = []
    seen: set[int] = set()
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            fields = tuple(reader.fieldnames or ())
            if "utcTimeMillis" not in fields:
                raise UpstreamMatError(f"{path} has no utcTimeMillis field")
            for line, row in enumerate(reader, start=2):
                raw = (row.get("utcTimeMillis") or "").strip()
                try:
                    timestamp = int(raw)
                except ValueError as exc:
                    raise UpstreamMatError(f"{path}:{line} has invalid utcTimeMillis") from exc
                if timestamp < 0:
                    raise UpstreamMatError(f"{path}:{line} has negative utcTimeMillis")
                if timestamp not in seen:
                    seen.add(timestamp)
                    values.append(timestamp)
    except OSError as exc:
        raise UpstreamMatError(f"failed to read {path}: {exc}") from exc
    if not values or values != sorted(values):
        raise UpstreamMatError(f"{path} epoch keys are empty or not monotonic")
    return values


def positions_from_result(
    payload: bytes,
    timestamps_ms: Sequence[int],
    trip_id: str,
    source: str,
    *,
    allow_missing_rows: bool = False,
) -> list[Position]:
    result = decode_result_bytes(payload, allow_missing_rows=allow_missing_rows)
    llh = result["llh"]
    finite_rows = result["finite_rows"]
    if len(timestamps_ms) != len(llh):
        raise UpstreamMatError(
            f"{trip_id}: timestamp/result length mismatch {len(timestamps_ms)} != {len(llh)}"
        )
    rows: list[Position] = []
    for timestamp, values, is_finite in zip(timestamps_ms, llh, finite_rows):
        if not is_finite:
            continue
        latitude, longitude, height = (float(value) for value in values)
        if not all(math.isfinite(value) for value in (latitude, longitude, height)):
            raise UpstreamMatError(f"{trip_id}: non-finite position")
        rows.append(Position(trip_id, int(timestamp), latitude, longitude, height, source))
    return rows


def write_positions(path: Path, rows: Sequence[Position], *, key_field: str = "phone") -> None:
    if key_field not in {"phone", "tripId"}:
        raise UpstreamMatError("position key field must be phone or tripId")
    output = io.StringIO(newline="")
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow([key_field, "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"])
    for row in rows:
        writer.writerow(
            [row.trip_id, str(row.timestamp_ms), f"{row.latitude:.9f}", f"{row.longitude:.9f}"]
        )
    atomic_bytes(path, output.getvalue().encode("ascii"))


def _ecef(row: Position) -> tuple[float, float, float]:
    latitude = math.radians(row.latitude)
    longitude = math.radians(row.longitude)
    semi_major = 6378137.0
    eccentricity_sq = 6.6943799901413165e-3
    sin_lat = math.sin(latitude)
    radius = semi_major / math.sqrt(1.0 - eccentricity_sq * sin_lat * sin_lat)
    return (
        (radius + row.height_m) * math.cos(latitude) * math.cos(longitude),
        (radius + row.height_m) * math.cos(latitude) * math.sin(longitude),
        (radius * (1.0 - eccentricity_sq) + row.height_m) * sin_lat,
    )


def continuity_statistics(rows: Sequence[Position]) -> dict[str, Any]:
    by_trip: dict[str, list[Position]] = {}
    for row in rows:
        by_trip.setdefault(row.trip_id, []).append(row)
    speeds: list[float] = []
    for trip_rows in by_trip.values():
        ordered = sorted(trip_rows, key=lambda row: row.timestamp_ms)
        for left, right in zip(ordered, ordered[1:]):
            delta_ms = right.timestamp_ms - left.timestamp_ms
            if delta_ms <= 0:
                raise UpstreamMatError("position timestamps are not strictly increasing")
            left_ecef = _ecef(left)
            right_ecef = _ecef(right)
            speed = math.sqrt(sum((a - b) ** 2 for a, b in zip(left_ecef, right_ecef))) / (delta_ms / 1000.0)
            if not math.isfinite(speed):
                raise UpstreamMatError("position continuity produced a non-finite speed")
            speeds.append(speed)
    return {
        "transition_count": len(speeds),
        "max_speed_mps": max(speeds) if speeds else 0.0,
        "over_bound_count": sum(speed > MAX_TRANSITION_SPEED_MPS for speed in speeds),
        "bound_mps": MAX_TRANSITION_SPEED_MPS,
    }


def _sample_keys(path: Path) -> list[tuple[str, int]]:
    keys: list[tuple[str, int]] = []
    seen: set[tuple[str, int]] = set()
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            _csv_header(
                reader,
                ("tripId", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"),
                path,
            )
            for line, row in enumerate(reader, start=2):
                trip = row.get("tripId") or ""
                if not trip or trip != trip.strip():
                    raise UpstreamMatError(f"{path}:{line} has an invalid tripId")
                try:
                    timestamp = int((row.get("UnixTimeMillis") or "").strip())
                except ValueError as exc:
                    raise UpstreamMatError(f"{path}:{line} has invalid UnixTimeMillis") from exc
                key = (trip, timestamp)
                if key in seen:
                    raise UpstreamMatError(f"{path}:{line} has duplicate sample key")
                seen.add(key)
                keys.append(key)
    except OSError as exc:
        raise UpstreamMatError(f"failed to read sample keys: {path}") from exc
    if not keys:
        raise UpstreamMatError("sample has no keys")
    return keys


def _interp_position(left: Position, right: Position, timestamp: int, source: str) -> Position:
    if right.timestamp_ms <= left.timestamp_ms:
        raise UpstreamMatError("position interpolation timestamps are not increasing")
    alpha = (timestamp - left.timestamp_ms) / (right.timestamp_ms - left.timestamp_ms)
    values = [
        left.latitude + alpha * (right.latitude - left.latitude),
        left.longitude + alpha * (right.longitude - left.longitude),
        left.height_m + alpha * (right.height_m - left.height_m),
    ]
    if not all(math.isfinite(value) for value in values):
        raise UpstreamMatError("position interpolation produced a non-finite value")
    return Position(left.trip_id, timestamp, values[0], values[1], values[2], source)


def reconcile_sample(
    sample_path: Path,
    positions: Sequence[Position],
    output_path: Path,
    manifest_path: Path,
) -> dict[str, Any]:
    """Reorder finite upstream positions onto sample keys without sample values."""
    keys = _sample_keys(sample_path)
    by_key: dict[tuple[str, int], Position] = {}
    by_trip: dict[str, list[Position]] = {}
    for row in positions:
        key = (row.trip_id, row.timestamp_ms)
        if key in by_key:
            raise UpstreamMatError(f"duplicate upstream position key: {key!r}")
        by_key[key] = row
        by_trip.setdefault(row.trip_id, []).append(row)
    for rows in by_trip.values():
        rows.sort(key=lambda row: row.timestamp_ms)
    sample_set = set(keys)
    extras = sorted(set(by_key) - sample_set)
    counts = {"exact": 0, "interpolated": 0, "edge_hold": 0, "unresolved": 0}
    output_rows: list[Position] = []
    for trip_id, timestamp in keys:
        direct = by_key.get((trip_id, timestamp))
        if direct is not None:
            output_rows.append(direct)
            counts["exact"] += 1
            continue
        rows = by_trip.get(trip_id, [])
        if not rows:
            counts["unresolved"] += 1
            continue
        earlier = [row for row in rows if row.timestamp_ms < timestamp]
        later = [row for row in rows if row.timestamp_ms > timestamp]
        left = earlier[-1] if earlier else None
        right = later[0] if later else None
        if left is not None and right is not None:
            if right.timestamp_ms - left.timestamp_ms <= INTERPOLATION_MAX_GAP_MS:
                output_rows.append(_interp_position(left, right, timestamp, "same_trip_linear_interpolation"))
                counts["interpolated"] += 1
                continue
        edge = left if left is not None and right is None else right
        if edge is not None and abs(timestamp - edge.timestamp_ms) <= EDGE_HOLD_MAX_GAP_MS:
            output_rows.append(
                Position(edge.trip_id, timestamp, edge.latitude, edge.longitude, edge.height_m, "same_trip_edge_hold")
            )
            counts["edge_hold"] += 1
            continue
        counts["unresolved"] += 1
    if counts["unresolved"]:
        raise UpstreamMatError(
            f"sample reconciliation has {counts['unresolved']} unresolved keys; refusing dummy/sample fallback"
        )
    if len(output_rows) != len(keys):
        raise UpstreamMatError("sample reconciliation row count mismatch")
    if any(
        abs(row.latitude - KNOWN_SAMPLE_DUMMY[0]) < 1e-10
        and abs(row.longitude - KNOWN_SAMPLE_DUMMY[1]) < 1e-10
        for row in output_rows
    ):
        raise UpstreamMatError("known sample dummy coordinate would be emitted")
    continuity = continuity_statistics(output_rows)
    if continuity["over_bound_count"]:
        raise UpstreamMatError(
            f"sample reconciliation has {continuity['over_bound_count']} transitions over "
            f"{MAX_TRANSITION_SPEED_MPS:g} m/s"
        )
    write_positions(output_path, output_rows, key_field="tripId")
    # Independent re-read checks exact key order and finite coordinates.
    with output_path.open(encoding="ascii", newline="") as handle:
        reader = csv.DictReader(handle)
        _csv_header(reader, ("tripId", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"), output_path)
        reread = [(row["tripId"], int(row["UnixTimeMillis"])) for row in reader]
    if reread != keys:
        raise UpstreamMatError("published submission key order differs from sample")
    manifest: dict[str, Any] = {
        "schema_version": SCHEMA + "-submission-manifest",
        "status": "completed-truth-free-upstream-mat-promotion",
        "truth_used": False,
        "test_truth_materialized": False,
        "official_sample_coordinates_used": False,
        "key_authority": {
            "path": str(sample_path),
            "sha256": sha256_file(sample_path),
            "rows": len(keys),
            "coordinates_read": False,
        },
        "source_counts": counts,
        "continuity": continuity,
        "extra_upstream_keys_dropped": len(extras),
        "extra_key_examples": [list(item) for item in extras[:10]],
        "rules": {
            "interpolation": "same-trip linear latitude/longitude/height only when bracket gap <=10000ms",
            "edge_hold": "same-trip constant finite endpoint only when one-sided gap <=1000ms",
            "unresolved": "fail-closed",
            "sample_coordinate_fallback": "forbidden",
        },
        "artifacts": {
            "submission": {
                "path": str(output_path),
                "sha256": sha256_file(output_path),
                "rows": len(output_rows),
            }
        },
    }
    atomic_json(manifest_path, manifest)
    manifest["artifacts"]["manifest"] = {"path": str(manifest_path), "sha256": sha256_file(manifest_path)}
    return manifest


def _member_role(name: str) -> dict[str, Any] | None:
    parts = name.split("/")
    if parts[0] != "dataset_2023" or parts[1] not in {"train", "test"}:
        return None
    if len(parts) == 4 and parts[2] and parts[3].endswith(".mat"):
        return {
            "split": parts[1],
            "route": parts[2],
            "phone": None,
            "basename": parts[3],
        }
    if len(parts) != 5 or not parts[2] or not parts[3] or not parts[4].endswith(".mat"):
        return None
    return {"split": parts[1], "route": parts[2], "phone": parts[3], "basename": parts[4]}


def _zip_member_hash(zf: ZipFile, info: ZipInfo) -> str:
    digest = hashlib.sha256()
    total = 0
    with zf.open(info, "r") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            total += len(block)
            if total > MAX_MEMBER_BYTES:
                raise UpstreamMatError(f"MAT member exceeds safety limit: {info.filename}")
            digest.update(block)
    if total != info.file_size:
        raise UpstreamMatError(f"MAT member size mismatch: {info.filename}")
    return digest.hexdigest()


def inventory_archive(archive_path: Path, output_path: Path) -> dict[str, Any]:
    if not archive_path.is_file():
        raise UpstreamMatError(f"missing archive: {archive_path}")
    entries: list[dict[str, Any]] = []
    try:
        with ZipFile(archive_path) as zf:
            central_entry_count = len(zf.infolist())
            for info in sorted(zf.infolist(), key=lambda item: item.filename):
                role = _member_role(info.filename)
                if role is None:
                    continue
                digest = _zip_member_hash(zf, info)
                item = {
                    **role,
                    "path": info.filename,
                    "crc32_hex": f"{info.CRC:08x}",
                    "compressed_size_bytes": info.compress_size,
                    "size_bytes": info.file_size,
                    "sha256": digest,
                    "stream_hash_equals_extracted_file_hash": True,
                }
                entries.append(item)
    except OSError as exc:
        raise UpstreamMatError(f"failed to inspect archive: {archive_path}") from exc
    result_entries = [item for item in entries if item["basename"] in {"result_gnss.mat", "result_gnss_imu.mat"}]
    paired: dict[tuple[str, str, str], set[str]] = {}
    for item in result_entries:
        paired.setdefault((item["split"], item["route"], item["phone"]), set()).add(item["basename"])
    missing_pairs = [list(key) for key, names in sorted(paired.items()) if names != {"result_gnss.mat", "result_gnss_imu.mat"}]
    payload: dict[str, Any] = {
        "schema_version": INVENTORY_SCHEMA,
        "status": "completed-mat-member-stream-hash",
        "archive": {
            "path": str(archive_path),
            "sha256": sha256_file(archive_path),
            "central_directory_entry_count": central_entry_count,
        },
        "method": "ZIP decompressed stream SHA256; equivalent to extracted member bytes; no files were redistributed",
        "mat_member_count": len(entries),
        "mat_member_total_bytes": sum(item["size_bytes"] for item in entries),
        "result_member_count": len(result_entries),
        "result_route_phone_count": len(paired),
        "missing_result_pairs": missing_pairs,
        "members": entries,
    }
    atomic_json(output_path, payload)
    payload["inventory_sha256"] = sha256_file(output_path)
    return payload


def _archive_test_positions(archive_path: Path, output_dir: Path) -> tuple[list[Position], list[dict[str, Any]]]:
    rows: list[Position] = []
    routes: list[dict[str, Any]] = []
    with ZipFile(archive_path) as zf:
        names = {info.filename: info for info in zf.infolist()}
        result_names = sorted(
            name for name in names if name.startswith("dataset_2023/test/") and name.endswith("/result_gnss_imu.mat")
        )
        if len(result_names) != 40:
            raise UpstreamMatError(f"expected 40 test result_gnss_imu.mat members, found {len(result_names)}")
        for result_name in result_names:
            parts = result_name.split("/")
            trip_id = f"{parts[2]}/{parts[3]}"
            phone_data_name = f"dataset_2023/test/{parts[2]}/{parts[3]}/phone_data.mat"
            if phone_data_name not in names:
                raise UpstreamMatError(f"missing phone_data key source: {phone_data_name}")
            with zf.open(names[result_name], "r") as handle:
                result_payload = handle.read(MAX_MEMBER_BYTES + 1)
            if len(result_payload) != names[result_name].file_size:
                raise UpstreamMatError(f"result member size mismatch: {result_name}")
            with zf.open(names[phone_data_name], "r") as handle:
                phone_data_payload = handle.read(MAX_MEMBER_BYTES + 1)
            if len(phone_data_payload) != names[phone_data_name].file_size:
                raise UpstreamMatError(f"phone_data member size mismatch: {phone_data_name}")
            timestamps = timestamps_from_phone_data_bytes(phone_data_payload)
            route_rows = positions_from_result(
                result_payload,
                timestamps,
                trip_id,
                "upstream_result_gnss_imu",
                allow_missing_rows=True,
            )
            route_path = output_dir / "routes" / (trip_id.replace("/", "__") + ".csv")
            write_positions(route_path, route_rows, key_field="tripId")
            rows.extend(route_rows)
            routes.append(
                {
                    "trip_id": trip_id,
                    "result_member": result_name,
                    "result_sha256": sha256_bytes(result_payload),
                    "phone_data_member": phone_data_name,
                    "phone_data_sha256": sha256_bytes(phone_data_payload),
                    "rows": len(route_rows),
                    "continuity": continuity_statistics(route_rows),
                    "route_artifact": str(route_path),
                }
            )
    return rows, routes


def build_test_submission(archive_path: Path, sample_path: Path, output_dir: Path) -> dict[str, Any]:
    positions, routes = _archive_test_positions(archive_path, output_dir)
    submission_path = output_dir / "submission.csv"
    manifest_path = output_dir / "submission.manifest.json"
    try:
        manifest = reconcile_sample(sample_path, positions, submission_path, manifest_path)
    except UpstreamMatError as exc:
        atomic_json(
            output_dir / "batch_failure.json",
            {
                "schema_version": SCHEMA + "-test-batch-failure",
                "status": "failed-closed-no-submission",
                "error": str(exc),
                "truth_used": False,
                "test_truth_materialized": False,
                "official_sample_coordinates_used": False,
                "archive": {"path": str(archive_path), "sha256": sha256_file(archive_path)},
                "sample_keys": {"path": str(sample_path), "sha256": sha256_file(sample_path)},
                "routes": routes,
            },
        )
        raise
    manifest["archive"] = {"path": str(archive_path), "sha256": sha256_file(archive_path)}
    manifest["routes"] = routes
    manifest["route_count"] = len(routes)
    manifest["test_truth_read"] = False
    atomic_json(manifest_path, manifest)
    manifest["artifacts"]["manifest"]["sha256"] = sha256_file(manifest_path)
    return manifest


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    inventory = sub.add_parser("inventory", help="hash all train/test MAT members")
    inventory.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    inventory.add_argument("--output", type=Path, required=True)
    convert = sub.add_parser("convert", help="decode one result MAT to keyed positions")
    convert.add_argument("--mat", type=Path, required=True)
    convert.add_argument("--device-gnss", type=Path, help="fallback epoch source when --phone-data is unavailable")
    convert.add_argument("--phone-data", type=Path, help="preferred exact upstream obs.utcms epoch source")
    convert.add_argument("--trip-id", required=True)
    convert.add_argument("--output", type=Path, required=True)
    convert.add_argument("--manifest", type=Path, required=True)
    convert.add_argument("--source", default="upstream_result")
    batch = sub.add_parser("test-batch", help="convert all 40 test result_gnss_imu members")
    batch.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    batch.add_argument("--sample", type=Path, default=DEFAULT_SAMPLE)
    batch.add_argument("--output-dir", type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        if args.command == "inventory":
            result = inventory_archive(args.archive, args.output)
        elif args.command == "convert":
            if args.phone_data is not None:
                timestamps = timestamps_from_phone_data_bytes(args.phone_data.read_bytes())
            elif args.device_gnss is not None:
                timestamps = read_unique_timestamps(args.device_gnss)
            else:
                raise UpstreamMatError("convert requires --phone-data or --device-gnss")
            rows = positions_from_result(
                args.mat.read_bytes(), timestamps, args.trip_id, args.source
            )
            write_positions(args.output, rows)
            result = {
                "schema_version": SCHEMA + "-conversion-manifest",
                "truth_used": False,
                "mat": {"path": str(args.mat), "sha256": sha256_file(args.mat)},
                "device_gnss": (
                    {"path": str(args.device_gnss), "sha256": sha256_file(args.device_gnss)}
                    if args.device_gnss is not None and args.phone_data is None
                    else None
                ),
                "phone_data": (
                    {"path": str(args.phone_data), "sha256": sha256_file(args.phone_data)}
                    if args.phone_data is not None
                    else None
                ),
                "output": {"path": str(args.output), "sha256": sha256_file(args.output), "rows": len(rows)},
                "continuity": continuity_statistics(rows),
                "source": args.source,
            }
            atomic_json(args.manifest, result)
            result["manifest_sha256"] = sha256_file(args.manifest)
        else:
            result = build_test_submission(args.archive, args.sample, args.output_dir)
    except (OSError, UpstreamMatError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
