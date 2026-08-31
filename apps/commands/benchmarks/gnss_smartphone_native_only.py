#!/usr/bin/env python3
"""Enforce the native-only smartphone inference input contract.

This module is deliberately a small provenance boundary.  It accepts raw
Android GNSS/IMU observations (or a raw ``phone_data.mat`` containing the
observation and sensor streams) and broadcast navigation, and hashes only
those approved inputs.  Published optimizer results, truth, sample
coordinates, and prior submissions are never opened as inference inputs.

The contract is independent of the estimator.  A caller must run it before
starting a native solver and retain the emitted manifest beside the solver
output.  All writes are atomic and the JSON contains the exact approved files
that were read, so a later audit does not have to infer provenance from a
directory listing.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import tempfile
from typing import Any, Iterable


SCHEMA_VERSION = "smartphone-r5-native-only-input-contract.v1"

# Match path components as well as basenames.  The broad result/submission
# markers are intentional: an inference command must fail closed even when a
# caller renames a published result file but leaves it under a result cache.
FORBIDDEN_PATH_MARKERS = (
    "result_gnss",
    "ground_truth",
    "gt.mat",
    "sample_submission",
    "submission.csv",
    "submission_",
    "native-fgo-test-v5",
    "upstream-mat",
    "precomputed",
)
FORBIDDEN_FIELD_MARKERS = (
    "latitudedegrees",
    "longitudedegrees",
    "altitudemeters",
    "submissioncoordinates",
    "tripid",
)

RAW_GNSS_CLOCK_FIELDS = ("timenanos", "fullbiasnanos", "receivedsvtimenanos")
RAW_IMU_FIELDS = (
    "messagetype",
    "measurementx",
    "measurementy",
    "measurementz",
)
MAT_IMU_ALIASES = {"acc", "gyro", "accelerometer", "gyroscope", "uncalaccel", "uncalgyro"}


class NativeOnlyInputError(ValueError):
    """Raised when an inference input cannot be proven raw and approved."""


def _normalise_field(value: str) -> str:
    return "".join(character.lower() for character in value.strip() if character.isalnum())


def sha256_file(path: Path) -> str:
    if not path.is_file():
        raise NativeOnlyInputError(f"approved input is not a regular file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write((json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"))
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _reject_path(path: Path, label: str) -> None:
    lowered = "/".join(path.parts).lower()
    for marker in FORBIDDEN_PATH_MARKERS:
        if marker in lowered:
            raise NativeOnlyInputError(
                f"{label} is forbidden for native inference (matched {marker!r}): {path}"
            )


def _read_csv_header(path: Path, label: str) -> tuple[str, ...]:
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.reader(handle)
            header = tuple(next(reader, ()))
    except (OSError, csv.Error) as exc:
        raise NativeOnlyInputError(f"cannot read {label} header: {exc}") from exc
    if not header:
        raise NativeOnlyInputError(f"{label} has an empty header: {path}")
    normalised = tuple(_normalise_field(item) for item in header)
    if len(normalised) != len(set(normalised)):
        raise NativeOnlyInputError(f"{label} has duplicate header fields: {path}")
    for field in normalised:
        if any(marker in field for marker in FORBIDDEN_FIELD_MARKERS):
            raise NativeOnlyInputError(
                f"{label} contains a coordinate/submission field: {header[normalised.index(field)]}"
            )
    return normalised


def _has_any_fields(header: Iterable[str], required: Iterable[str]) -> bool:
    values = set(header)
    return all(item in values for item in required)


def inspect_raw_gnss(path: Path) -> dict[str, Any]:
    """Validate a raw ``device_gnss.csv`` without opening any sibling files."""

    _reject_path(path, "device_gnss.csv")
    if path.name.lower() != "device_gnss.csv":
        raise NativeOnlyInputError("raw GNSS input must be named device_gnss.csv")
    header = _read_csv_header(path, "device_gnss.csv")
    required = {
        "messagetype",
        "utctimemillis",
        "svid",
        "constellationtype",
        "pseudorangeratemeterspersecond",
        "accumulateddeltarangestate",
        "accumulateddeltarangemeters",
        "carrierfrequencyhz",
        "cn0dbhz",
    }
    missing = sorted(required - set(header))
    if missing:
        raise NativeOnlyInputError(
            "device_gnss.csv lacks raw measurement columns: " + ", ".join(missing)
        )
    if not _has_any_fields(header, RAW_GNSS_CLOCK_FIELDS):
        raise NativeOnlyInputError(
            "device_gnss.csv lacks raw TimeNanos/FullBiasNanos/ReceivedSvTimeNanos; "
            "enriched arrival/pseudorange timing is not accepted by native inference"
        )
    timing_mode = "raw_android_clock"
    rows = 0
    raw_rows = 0
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                rows += 1
                if (row.get("MessageType") or "").strip() == "Raw":
                    raw_rows += 1
                if rows > 1_000_000:
                    raise NativeOnlyInputError("device_gnss.csv exceeds the bounded row contract")
    except (OSError, csv.Error) as exc:
        raise NativeOnlyInputError(f"cannot scan device_gnss.csv: {exc}") from exc
    if raw_rows == 0:
        raise NativeOnlyInputError("device_gnss.csv contains no MessageType=Raw rows")
    return {
        "kind": "raw_android_gnss_csv",
        "path": str(path),
        "sha256": sha256_file(path),
        "header_sha256": hashlib.sha256(",".join(header).encode("utf-8")).hexdigest(),
        "header_fields": list(header),
        "rows": rows,
        "raw_rows": raw_rows,
        "timing_mode": timing_mode,
        "truth_or_result_read": False,
    }


def inspect_raw_imu(path: Path) -> dict[str, Any]:
    """Validate a raw Android IMU CSV used by an opt-in native recipe."""

    _reject_path(path, "device_imu.csv")
    if path.name.lower() != "device_imu.csv":
        raise NativeOnlyInputError("raw IMU input must be named device_imu.csv")
    header = _read_csv_header(path, "device_imu.csv")
    if not _has_any_fields(header, RAW_IMU_FIELDS):
        raise NativeOnlyInputError(
            "device_imu.csv must contain MessageType and MeasurementX/Y/Z raw sensor fields"
        )
    allowed_types = {"uncalaccel", "uncalgyro", "accel", "gyro"}
    rows = 0
    sensor_types: set[str] = set()
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                rows += 1
                sensor_type = (row.get("MessageType") or "").strip().lower()
                if sensor_type in allowed_types:
                    sensor_types.add(sensor_type)
                if rows > 2_000_000:
                    raise NativeOnlyInputError("device_imu.csv exceeds the bounded row contract")
    except (OSError, csv.Error) as exc:
        raise NativeOnlyInputError(f"cannot scan device_imu.csv: {exc}") from exc
    if rows == 0 or not ({"uncalaccel", "uncalgyro"} <= sensor_types):
        raise NativeOnlyInputError(
            "device_imu.csv must contain both UncalAccel and UncalGyro raw streams"
        )
    return {
        "kind": "raw_android_imu_csv",
        "path": str(path),
        "sha256": sha256_file(path),
        "header_fields": list(header),
        "rows": rows,
        "sensor_types": sorted(sensor_types),
        "truth_or_result_read": False,
    }


def inspect_phone_data_mat(path: Path) -> dict[str, Any]:
    """Validate a raw MAT sensor/observation container.

    SciPy's normal ``whosmat`` cannot list the MATLAB MCOS FileWrapper used by
    the public dataset.  ``loadmat`` still exposes the numeric ``acc`` and
    ``gyro`` streams, while the opaque ``None`` variable is retained as the
    raw-observation container.  We intentionally do not decode result MAT
    workspaces here.
    """

    _reject_path(path, "phone_data.mat")
    if path.name.lower() != "phone_data.mat":
        raise NativeOnlyInputError("MAT input must be named phone_data.mat")
    try:
        from scipy.io import loadmat

        parsed = loadmat(path, squeeze_me=True, struct_as_record=False)
    except Exception as exc:  # pragma: no cover - SciPy is optional for CSV use
        raise NativeOnlyInputError(f"phone_data.mat is not a supported raw MAT container: {exc}") from exc
    keys = sorted(key for key in parsed if not key.startswith("__"))
    lowered = {key.lower() for key in keys}
    if any(any(marker in key for marker in FORBIDDEN_PATH_MARKERS) for key in lowered):
        raise NativeOnlyInputError("phone_data.mat exposes a forbidden result/submission variable")
    imu_fields = sorted(lowered & MAT_IMU_ALIASES)
    if not ({"acc", "gyro"} <= lowered or {"accelerometer", "gyroscope"} <= lowered):
        raise NativeOnlyInputError(
            "phone_data.mat must expose raw accelerometer and gyroscope streams"
        )
    # The MCOS observation object is represented by the opaque None workspace
    # in the public taroz format.  Plain fixtures may use an explicit obs key.
    has_raw_observation_container = "obs" in lowered or "none" in lowered
    if not has_raw_observation_container:
        raise NativeOnlyInputError(
            "phone_data.mat lacks a raw GNSS observation container (obs/MCOS None)"
        )
    return {
        "kind": "raw_phone_data_mat",
        "path": str(path),
        "sha256": sha256_file(path),
        "mat_keys": keys,
        "raw_sensor_fields": imu_fields,
        "raw_gnss_container": "obs" if "obs" in lowered else "MCOS:None",
        "truth_or_result_read": False,
        "result_workspace_decoded": False,
    }


def inspect_broadcast_nav(path: Path) -> dict[str, Any]:
    """Hash a declared broadcast navigation file without reading truth."""

    _reject_path(path, "broadcast navigation")
    if path.suffix.lower() not in {".nav", ".rnx", ".rinex", ".gz"}:
        raise NativeOnlyInputError("broadcast navigation must be a RINEX navigation file")
    return {
        "kind": "broadcast_navigation",
        "path": str(path),
        "sha256": sha256_file(path),
        "truth_or_result_read": False,
    }


def validate_native_inputs(
    *,
    device_gnss: Path | None = None,
    phone_data: Path | None = None,
    device_imu: Path | None = None,
    broadcast_nav: Path,
    output_manifest: Path | None = None,
) -> dict[str, Any]:
    """Validate and optionally atomically publish a native-only manifest."""

    if (device_gnss is None) == (phone_data is None):
        raise NativeOnlyInputError("provide exactly one of device_gnss or phone_data")
    approved: dict[str, Any] = {}
    if device_gnss is not None:
        approved["gnss"] = inspect_raw_gnss(device_gnss)
    else:
        approved["phone_data"] = inspect_phone_data_mat(phone_data)  # type: ignore[arg-type]
    if device_imu is None:
        raise NativeOnlyInputError(
            "native-only contract requires a separate raw device_imu.csv for this recipe"
        )
    approved["imu"] = inspect_raw_imu(device_imu)
    approved["broadcast_nav"] = inspect_broadcast_nav(broadcast_nav)
    manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "contract": {
            "native_inference_only": True,
            "approved_gnss_inputs": ["raw device_gnss.csv", "raw phone_data.mat"],
            "required_companions": ["raw device_imu.csv", "broadcast RINEX navigation"],
            "forbidden_inputs": [
                "result_gnss.mat",
                "result_gnss_imu.mat",
                "ground_truth.csv/gt.mat",
                "submission coordinates",
                "v5 imported output",
                "sample_submission coordinates",
            ],
            "forbidden_read": True,
            "fail_closed": True,
            "truth_used": False,
        },
        "approved_inputs": approved,
        "read_members": sorted(item["path"] for item in approved.values()),
        "forbidden_members_checked": list(FORBIDDEN_PATH_MARKERS),
        "provenance": {
            "result_gnss_read": False,
            "result_gnss_imu_read": False,
            "ground_truth_read": False,
            "submission_coordinates_read": False,
            "sample_coordinates_read": False,
            "v5_imported_output_read": False,
            "only_approved_files_hashed": True,
        },
    }
    if output_manifest is not None:
        atomic_json(output_manifest, manifest)
        manifest["manifest_sha256"] = sha256_file(output_manifest)
    return manifest


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--device-gnss", type=Path)
    source.add_argument("--phone-data", type=Path)
    parser.add_argument("--device-imu", type=Path, required=True)
    parser.add_argument("--broadcast-nav", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    try:
        manifest = validate_native_inputs(
            device_gnss=args.device_gnss,
            phone_data=args.phone_data,
            device_imu=args.device_imu,
            broadcast_nav=args.broadcast_nav,
            output_manifest=args.manifest,
        )
    except NativeOnlyInputError as exc:
        print(f"native-only input contract rejected: {exc}", file=os.sys.stderr)
        return 2
    print(json.dumps(manifest, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
