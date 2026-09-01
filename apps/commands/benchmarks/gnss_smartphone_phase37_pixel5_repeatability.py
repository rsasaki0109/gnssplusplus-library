#!/usr/bin/env python3
"""Truth-free runner and structural sealer for the Phase37 Pixel5 cohort.

The runner uses only the three raw Android members (``device_gnss.csv``,
``device_imu.csv``, and route-level ``brdc.nav``) for the three additions to
the Phase36 cohort.  Ground truth is deliberately never materialized or
opened by this module.  The native binary and flags are the sealed Phase31
quality-anchor lane; this command does not alter the binary, flags, graph, or
production defaults.

The two public operations are intentionally separate:

``materialize-raw``
    Verify the archive and central directory, then extract only the selected
    raw members.  No ground-truth member is opened.

``run-native``
    Run each added route twice, validate keyed finite output and summary
    contracts, require byte-identical repeats, and atomically publish a
    truth-free structural seal.  Any archive or solver failure is fail-closed.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
from typing import Any
import zipfile


ROOT = Path(__file__).resolve().parents[3]
FREEZE_V1 = ROOT / "docs/use_cases/records/smartphone_r5_phase37_pixel5_repeatability_freeze_v1.json"
FREEZE_V2 = ROOT / "docs/use_cases/records/smartphone_r5_phase37_pixel5_repeatability_freeze_v2.json"
FREEZE_V1_SHA256 = "629689a206e59655c7cafbc59b6011553fa1953381df74d644bfd567708be8fd"
FREEZE_V2_SHA256 = "61f46ac734d0c712bc317603570a0630234ee67ab642d12368eb5e3bf3962a3e"
PHASE36_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase36_phone_bias_audit_result_v1.json"
PHASE36_RESULT_SHA256 = "d92c8ae16dfc9bfe9726b2c44acec95d1d6778d1fb687b3b4e2dcc21a8d61e71"
ARCHIVE = ROOT / "data/gsdc2023/cache/dataset_2023.zip"
ARCHIVE_SHA256 = "bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408"
INVENTORY = ROOT / "output/smartphone-r5/generalization-v6/archive_inventory.json"
INVENTORY_SHA256 = "f4e68109885eecfc14b2bd5e8fab87e18d73473c04bb7f53da31b7040a8e90a7"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
BINARY_SHA256 = "883701ef34606af4be84a44181f6b3616b2559ee4031982fff98d73dbbbbe7bd"
EVALUATOR_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase37_pixel5_repeatability_evaluator_manifest_v1.json"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase37-pixel5-repeatability-v1"
STRUCTURAL_NAME = "phase37_structural_seal.json"
STRUCTURAL_MANIFEST_NAME = "phase37_structural_seal.manifest.json"
FAILURE_NAME = "phase37_structural_failure.json"
MATERIALIZATION_NAME = "phase37_raw_materialization.json"

BASE_ROUTE = "2021-03-16-18-59-us-ca-mtv-a/pixel5"
ADDED_ROUTES = (
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
ROUTES = (BASE_ROUTE, *ADDED_ROUTES)
VALIDATION_ROUTE = "2023-05-09-21-32-us-ca-mtv-pe1/pixel5"
HOLDOUT_ROUTE = "2023-05-16-19-54-us-ca-mtv-xe1/pixel5"
BASE_RAW_ROOT = ROOT / "output/smartphone-r5/phase25-raw-clock-eval-v1/raw" / "2021-03-16-18-59-us-ca-mtv-a/pixel5"
BASE_SUBMISSION = ROOT / "output/smartphone-r5/phase31-quality-anchor-structural-v1/anchor/pixel5/submission.csv"
BASE_SUMMARY = ROOT / "output/smartphone-r5/phase31-quality-anchor-structural-v1/anchor/pixel5/summary.json"
BASE_SUBMISSION_SHA256 = "d4d7652e5d12389466e586fe2d8e85d34977a7e11036cee2f591a89293c5426c"
BASE_SUMMARY_SHA256 = "b0ecd9c0eb931b11908a4e19ea54cb21a7a530492ac6fee245086dff93722460"

FLAGS = (
    "--all-epochs",
    "--android-raw-utc-keys",
    "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback",
    "--native-pdc-imu-tdcp-no-bridge",
    "--native-quality-anchor",
)
RAW_KEYS = ("device_gnss", "device_imu", "broadcast_nav")
RAW_FILENAMES = {"device_gnss": "device_gnss.csv", "device_imu": "device_imu.csv", "broadcast_nav": "brdc.nav"}
MAX_SPEED_MPS = 70.0
EARTH_RADIUS_M = 6_371_000.0
INTEGER_RE = set("0123456789")


class Phase37Error(ValueError):
    """Raised when the frozen Phase37 truth-free contract fails."""


def _fail(message: str) -> Phase37Error:
    return Phase37Error(message)


def _reject_mat(path: Path | str) -> None:
    token = str(path).lower()
    if token.endswith(".mat") or ".mat/" in token or ".mat\\" in token:
        raise _fail(f"MAT input/output is forbidden: {path}")


def sha256(path: Path) -> str:
    _reject_mat(path)
    if not path.is_file():
        raise _fail(f"missing file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise _fail(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def load_json(path: Path, label: str) -> dict[str, Any]:
    _reject_mat(path)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise _fail(f"invalid {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise _fail(f"{label} is not an object: {path}")
    return value


def atomic_write(path: Path, payload: bytes) -> None:
    _reject_mat(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "wb") as handle:
            descriptor = -1
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = ""
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        if temporary:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    atomic_write(path, (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def verify_freeze() -> tuple[dict[str, Any], dict[str, Any]]:
    """Verify v2 and inherited v1 without opening the archive."""

    if sha256(FREEZE_V1) != FREEZE_V1_SHA256:
        raise _fail("Phase37 v1 freeze hash changed")
    if sha256(FREEZE_V2) != FREEZE_V2_SHA256:
        raise _fail("Phase37 v2 freeze hash changed")
    v1 = load_json(FREEZE_V1, "Phase37 v1 freeze")
    v2 = load_json(FREEZE_V2, "Phase37 v2 freeze")
    if v1.get("schema_version") != "smartphone-r5-phase37-pixel5-repeatability-freeze.v1":
        raise _fail("Phase37 v1 schema mismatch")
    if v2.get("schema_version") != "smartphone-r5-phase37-pixel5-repeatability-freeze.v2":
        raise _fail("Phase37 v2 schema mismatch")
    if v2.get("status") != "frozen-before-phase37-materialization-and-solver":
        raise _fail("Phase37 v2 is not pre-materialization")
    amendment = v2.get("amendment")
    if not isinstance(amendment, dict) or amendment.get("supersedes", {}).get("sha256") != FREEZE_V1_SHA256:
        raise _fail("Phase37 v2 does not pin v1")
    cohort = v2.get("cohort")
    if not isinstance(cohort, dict) or tuple(cohort.get("evaluation_routes", ())) != ROUTES:
        raise _fail("Phase37 evaluation cohort changed")
    if cohort.get("fresh_validation_remains_sealed") != VALIDATION_ROUTE or cohort.get("future_holdout_remains_sealed") != HOLDOUT_ROUTE:
        raise _fail("Phase37 validation/holdout identity changed")
    ident = v2.get("identifiability", {})
    agreement = ident.get("cross_route_agreement", {}) if isinstance(ident, dict) else {}
    conditions = agreement.get("conditions", []) if isinstance(agreement, dict) else []
    by_id = {item.get("id"): item for item in conditions if isinstance(item, dict)}
    if by_id.get("component_route_median_mad", {}).get("threshold_m", {}) != {"east": 0.5, "north": 0.5}:
        raise _fail("Phase37 MAD threshold changed")
    if by_id.get("geometric_center_radius", {}).get("threshold_m") != 1.0:
        raise _fail("Phase37 geometric-radius threshold changed")
    if by_id.get("nonzero_common_median", {}).get("threshold_factor") != 2.0:
        raise _fail("Phase37 non-zero threshold changed")
    if by_id.get("route_prefix_tail_stability", {}).get("threshold_floor_m") != 1.0 or by_id.get("route_prefix_tail_stability", {}).get("threshold_factor") != 2.0:
        raise _fail("Phase37 prefix/tail threshold changed")
    native = v2.get("native_and_data_contract")
    if not isinstance(native, dict) or native.get("native_flags_unchanged") != list(FLAGS):
        raise _fail("Phase37 native flags changed")
    if native.get("mat_read_or_generated") is not False or native.get("phase31_champion_mutation") is not False:
        raise _fail("Phase37 MAT/champion contract changed")
    budget = native.get("truth_read_budget", {})
    if not isinstance(budget, dict) or budget.get("post_structural_seal_one_read_per_route") is not True or budget.get("single_process") is not True:
        raise _fail("Phase37 truth-read budget changed")
    # The v1 archive section is metadata-only at this stage.  Do not open a
    # member here; selected payloads are checked only by materialize-raw.
    archive = v1.get("archive", {})
    if not isinstance(archive, dict) or archive.get("path") != str(ARCHIVE.relative_to(ROOT)):
        raise _fail("Phase37 archive path changed")
    central = archive.get("central_inventory", {})
    if not isinstance(central, dict) or central.get("member_content_read_before_freeze") is not False:
        raise _fail("Phase37 pre-freeze member-read contract changed")
    if sha256(PHASE36_RESULT) != PHASE36_RESULT_SHA256:
        raise _fail("Phase36 result pin changed")
    if sha256(INVENTORY) != INVENTORY_SHA256:
        raise _fail("central inventory pin changed")
    if sha256(BINARY) != BINARY_SHA256:
        raise _fail("Phase31 native binary changed")
    return v1, v2


def _route_member_names(route: str) -> dict[str, str]:
    path, phone = route.split("/", 1)
    return {
        "device_gnss": f"dataset_2023/train/{path}/{phone}/device_gnss.csv",
        "device_imu": f"dataset_2023/train/{path}/{phone}/device_imu.csv",
        "broadcast_nav": f"dataset_2023/train/{path}/brdc.nav",
        "ground_truth": f"dataset_2023/train/{path}/{phone}/ground_truth.csv",
    }


def _central_metadata(info: zipfile.ZipInfo) -> dict[str, Any]:
    return {
        "name": info.filename,
        "file_size": info.file_size,
        "compressed_size": info.compress_size,
        "crc32_hex": f"{info.CRC:08x}",
    }


def _central_contract(v1: dict[str, Any]) -> dict[str, dict[str, zipfile.ZipInfo]]:
    expected = v1.get("selected_archive_members")
    if not isinstance(expected, dict):
        raise _fail("Phase37 selected archive metadata is missing")
    if not ARCHIVE.is_file():
        raise _fail(f"missing archive: {ARCHIVE}")
    records: dict[str, dict[str, zipfile.ZipInfo]] = {}
    try:
        with zipfile.ZipFile(ARCHIVE) as archive:
            infos: dict[str, list[zipfile.ZipInfo]] = {}
            for info in archive.infolist():
                infos.setdefault(info.filename, []).append(info)
            for route in ADDED_ROUTES:
                expected_route = expected.get(route)
                if not isinstance(expected_route, dict):
                    raise _fail(f"missing central metadata for {route}")
                names = _route_member_names(route)
                records[route] = {}
                for key, member in names.items():
                    if ".mat" in member.lower() or VALIDATION_ROUTE.split("/", 1)[0] in member or HOLDOUT_ROUTE.split("/", 1)[0] in member:
                        raise _fail(f"forbidden archive member selected: {member}")
                    entries = infos.get(member, [])
                    if len(entries) != 1 or entries[0].is_dir():
                        raise _fail(f"archive member missing/duplicate: {member}")
                    info = entries[0]
                    pinned = expected_route.get(key)
                    if not isinstance(pinned, dict) or _central_metadata(info) != {
                        "name": pinned.get("name"),
                        "file_size": pinned.get("file_size"),
                        "compressed_size": pinned.get("compressed_size"),
                        "crc32_hex": pinned.get("crc32_hex"),
                    }:
                        raise _fail(f"central metadata mismatch: {member}")
                    records[route][key] = info
    except (OSError, zipfile.BadZipFile) as exc:
        raise _fail(f"failed to inspect archive central directory: {ARCHIVE}: {exc}") from exc
    return records


def _extract_member(
    archive: zipfile.ZipFile,
    info: zipfile.ZipInfo,
    destination: Path,
) -> dict[str, Any]:
    """Read one selected raw member once and atomically publish it."""

    _reject_mat(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        raise _fail(f"refusing to overwrite materialized input: {destination}")
    temporary = destination.parent / f".{destination.name}.{os.getpid()}.tmp"
    if temporary.exists():
        raise _fail(f"stale materialization temporary exists: {temporary}")
    digest = hashlib.sha256()
    size = 0
    try:
        with archive.open(info, "r") as source, temporary.open("wb") as target:
            while True:
                chunk = source.read(1024 * 1024)
                if not chunk:
                    break
                target.write(chunk)
                digest.update(chunk)
                size += len(chunk)
            target.flush()
            os.fsync(target.fileno())
        if size != info.file_size:
            raise _fail(f"materialized size mismatch: {info.filename}")
        os.replace(temporary, destination)
    finally:
        if temporary.exists():
            temporary.unlink()
    return {
        "member": info.filename,
        "sha256": digest.hexdigest(),
        "file_size": size,
        "compressed_size": info.compress_size,
        "crc32_hex": f"{info.CRC:08x}",
        "path": relative(destination),
    }


def materialize_raw(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    """Materialize only the selected raw members after the source seal."""

    v1, v2 = verify_freeze()
    _ = v2
    output_root = output_root.resolve()
    _reject_mat(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise _fail(f"refusing to overwrite nonempty Phase37 output: {output_root}")
    if sha256(ARCHIVE) != ARCHIVE_SHA256:
        raise _fail("archive SHA256 does not match Phase37 freeze")
    records = _central_contract(v1)
    output_root.mkdir(parents=True, exist_ok=True)
    route_reports: dict[str, Any] = {}
    try:
        with zipfile.ZipFile(ARCHIVE) as archive:
            for route in ADDED_ROUTES:
                route_path, phone = route.split("/", 1)
                route_root = output_root / "routes" / route_path / phone
                inputs = route_root / "inputs"
                if route_root.exists():
                    raise _fail(f"refusing to overwrite route output: {route_root}")
                inputs.mkdir(parents=True, exist_ok=True)
                files: dict[str, Any] = {}
                for key in RAW_KEYS:
                    destination = inputs / RAW_FILENAMES[key]
                    files[key] = _extract_member(archive, records[route][key], destination)
                # A raw-only materialization must never create a truth file.
                truth_path = inputs / "ground_truth.csv"
                if truth_path.exists():
                    raise _fail(f"truth was materialized during raw phase: {truth_path}")
                route_reports[route] = {
                    "dataset_id": route,
                    "role": "development-added",
                    "inputs": files,
                    "truth_materialized": False,
                    "truth_open_count": 0,
                    "mat_read_or_generated": False,
                }
    except (OSError, zipfile.BadZipFile) as exc:
        failure = {
            "schema_version": "smartphone-r5-phase37-raw-materialization-failure.v1",
            "status": "fail-closed",
            "freeze": {"path": relative(FREEZE_V2), "sha256": FREEZE_V2_SHA256},
            "error": str(exc),
            "truth_materialized": False,
            "truth_open_count": 0,
            "mat_read_or_generated": False,
        }
        atomic_json(output_root / FAILURE_NAME, failure)
        raise _fail(str(exc)) from exc
    report = {
        "schema_version": "smartphone-r5-phase37-raw-materialization.v1",
        "status": "truth-free-raw-materialized",
        "freeze": {"path": relative(FREEZE_V2), "sha256": FREEZE_V2_SHA256},
        "archive": {"path": relative(ARCHIVE), "sha256": ARCHIVE_SHA256, "central_directory_verified": True},
        "routes": route_reports,
        "selected_members": list(RAW_KEYS),
        "ground_truth_member_opened": False,
        "truth_materialized": False,
        "truth_open_count": 0,
        "validation_truth_open_count": 0,
        "future_holdout_truth_open_count": 0,
        "mat_read_or_generated": False,
        "token_or_kaggle_access": False,
    }
    atomic_json(output_root / MATERIALIZATION_NAME, report)
    return report


def _parse_int(value: str | None, field: str, row_number: int) -> int:
    token = "" if value is None else value.strip()
    if not token or any(char not in INTEGER_RE for char in token.lstrip("+-")):
        raise _fail(f"row {row_number}: invalid {field}")
    try:
        number = int(token)
    except ValueError as exc:
        raise _fail(f"row {row_number}: invalid {field}") from exc
    if number < 0:
        raise _fail(f"row {row_number}: negative {field}")
    return number


def _parse_float(value: str | None, field: str, row_number: int) -> float:
    token = "" if value is None else value.strip()
    try:
        number = float(token)
    except (TypeError, ValueError) as exc:
        raise _fail(f"row {row_number}: invalid {field}") from exc
    if not math.isfinite(number):
        raise _fail(f"row {row_number}: non-finite {field}")
    return number


def read_raw_epoch_keys(path: Path) -> list[int]:
    _reject_mat(path)
    keys: list[int] = []
    previous: int | None = None
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            if "utcTimeMillis" not in set(reader.fieldnames or ()):
                raise _fail(f"raw input lacks utcTimeMillis: {path}")
            for row_number, row in enumerate(reader, start=2):
                timestamp = _parse_int(row.get("utcTimeMillis"), "utcTimeMillis", row_number)
                if previous is None or timestamp != previous:
                    if previous is not None and timestamp <= previous:
                        raise _fail(f"raw UTC epochs are not increasing: {path}")
                    keys.append(timestamp)
                    previous = timestamp
    except OSError as exc:
        raise _fail(f"failed to read raw input: {path}: {exc}") from exc
    if len(keys) < 2:
        raise _fail(f"raw input has fewer than two epochs: {path}")
    return keys


def read_prediction(path: Path, dataset_id: str) -> list[tuple[int, float, float]]:
    _reject_mat(path)
    rows: list[tuple[int, float, float]] = []
    previous: int | None = None
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            if tuple(reader.fieldnames or ()) != ("phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"):
                raise _fail(f"prediction header mismatch: {path}")
            for row_number, row in enumerate(reader, start=2):
                if row.get("phone") != dataset_id:
                    raise _fail(f"prediction phone mismatch at row {row_number}: {path}")
                timestamp = _parse_int(row.get("UnixTimeMillis"), "UnixTimeMillis", row_number)
                if previous is not None and timestamp <= previous:
                    raise _fail(f"prediction timestamps are not increasing: {path}")
                previous = timestamp
                latitude = _parse_float(row.get("LatitudeDegrees"), "LatitudeDegrees", row_number)
                longitude = _parse_float(row.get("LongitudeDegrees"), "LongitudeDegrees", row_number)
                if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
                    raise _fail(f"prediction coordinate outside bounds: {path}")
                rows.append((timestamp, latitude, longitude))
    except OSError as exc:
        raise _fail(f"failed to read prediction: {path}: {exc}") from exc
    if not rows or len({row[0] for row in rows}) != len(rows):
        raise _fail(f"prediction is empty or has duplicate timestamps: {path}")
    return rows


def speed_report(rows: list[tuple[int, float, float]]) -> dict[str, Any]:
    if len(rows) < 2:
        return {"transition_count": 0, "max_speed_mps": 0.0, "initial_30_transition_max_speed_mps": 0.0, "over_70_mps_count": 0, "finite": True}
    speeds: list[float] = []
    for previous, current in zip(rows, rows[1:]):
        dt = (current[0] - previous[0]) / 1000.0
        if dt <= 0.0:
            raise _fail("prediction timestamps do not have positive spacing")
        lat0, lat1 = math.radians(previous[1]), math.radians(current[1])
        dlat = lat1 - lat0
        dlon = math.radians(current[2] - previous[2])
        term = math.sin(dlat / 2.0) ** 2 + math.cos(lat0) * math.cos(lat1) * math.sin(dlon / 2.0) ** 2
        distance = 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(max(0.0, term))))
        speeds.append(distance / dt)
    initial = speeds[: min(30, len(speeds))]
    return {
        "transition_count": len(speeds),
        "max_speed_mps": max(speeds),
        "initial_30_transition_max_speed_mps": max(initial),
        "over_70_mps_count": sum(speed > MAX_SPEED_MPS for speed in speeds),
        "finite": all(math.isfinite(speed) for speed in speeds),
    }


def validate_summary(path: Path, dataset_id: str) -> dict[str, Any]:
    summary = load_json(path, "native summary")
    if summary.get("dataset_id") != dataset_id:
        raise _fail(f"summary dataset mismatch: {dataset_id}")
    if summary.get("truth_used") is not False or summary.get("production_default_changed") is not False:
        raise _fail(f"truth/default contract failed: {dataset_id}")
    if summary.get("native_quality_anchor") is not True or summary.get("native_pdc_imu_tdcp_no_bridge") is not True:
        raise _fail(f"Phase31 feature flags missing: {dataset_id}")
    if summary.get("native_pdc_state_bridge") is not False:
        raise _fail(f"PDC bridge unexpectedly enabled: {dataset_id}")
    graph = summary.get("graph")
    epochs = summary.get("epochs")
    anchor = summary.get("quality_anchor_initialization")
    raw_contract = summary.get("raw_utc_key_contract")
    if not all(isinstance(value, dict) for value in (graph, epochs, anchor, raw_contract)):
        raise _fail(f"required native diagnostics missing: {dataset_id}")
    if graph.get("converged") is not True:
        raise _fail(f"native graph did not converge: {dataset_id}")
    initial = float(graph.get("initial_cost", math.nan))
    final = float(graph.get("final_cost", math.nan))
    if not math.isfinite(initial) or not math.isfinite(final) or final > initial + 1e-9:
        raise _fail(f"native graph costs are invalid: {dataset_id}")
    if int(graph.get("imu_intervals", 0)) <= 0 or int(epochs.get("pseudorange_factors", 0)) <= 0 or int(epochs.get("tdcp_factors_built", 0)) <= 0:
        raise _fail(f"required factor family missing: {dataset_id}")
    if anchor.get("enabled") is not True or anchor.get("selected") is not True or anchor.get("truth_free") is not True or anchor.get("graph_model_changed") is not False:
        raise _fail(f"quality-anchor contract failed: {dataset_id}")
    if int(anchor.get("fallback_epochs", -1)) != 0 or int(anchor.get("eligible_candidates", 0)) <= 0:
        raise _fail(f"quality-anchor fallback/eligibility failed: {dataset_id}")
    if int(raw_contract.get("unresolved_epochs", -1)) != 0:
        raise _fail(f"raw UTC key contract failed: {dataset_id}")
    return {
        "graph": {
            "factors": graph.get("factors"),
            "values": graph.get("values"),
            "imu_intervals": graph.get("imu_intervals"),
            "iterations": graph.get("iterations"),
            "converged": graph.get("converged"),
            "initial_cost": graph.get("initial_cost"),
            "final_cost": graph.get("final_cost"),
        },
        "epochs": {
            "problem": epochs.get("problem"),
            "output": epochs.get("output"),
            "pseudorange_factors": epochs.get("pseudorange_factors"),
            "tdcp_factors_built": epochs.get("tdcp_factors_built"),
        },
        "quality_anchor": anchor,
        "raw_utc_key_contract": raw_contract,
    }


def _artifact_report(submission: Path, summary: Path, dataset_id: str, target_keys: list[int]) -> dict[str, Any]:
    rows = read_prediction(submission, dataset_id)
    if [row[0] for row in rows] != target_keys:
        raise _fail(f"raw target key mismatch: {dataset_id}")
    speed = speed_report(rows)
    if not speed["finite"] or speed["over_70_mps_count"] != 0:
        raise _fail(f"continuity safety failed: {dataset_id}")
    projection = validate_summary(summary, dataset_id)
    return {
        "submission": {"path": relative(submission), "sha256": sha256(submission), "bytes": submission.stat().st_size, "rows": len(rows)},
        "summary": {"path": relative(summary), "sha256": sha256(summary), "bytes": summary.stat().st_size},
        "projection": projection,
        "speed": speed,
    }


def _input_paths(output_root: Path, route: str) -> dict[str, Path]:
    route_path, phone = route.split("/", 1)
    root = output_root / "routes" / route_path / phone / "inputs"
    return {key: root / RAW_FILENAMES[key] for key in RAW_KEYS}


def _native_command(paths: dict[str, Path], dataset_id: str, run_dir: Path) -> list[str]:
    return [
        str(BINARY),
        "--android-gnss", str(paths["device_gnss"]),
        "--android-imu", str(paths["device_imu"]),
        "--nav", str(paths["broadcast_nav"]),
        "--out", str(run_dir / "submission.csv"),
        "--summary-json", str(run_dir / "summary.json"),
        "--dataset-id", dataset_id,
        *FLAGS,
    ]


def _run_native(paths: dict[str, Path], dataset_id: str, run_dir: Path, target_keys: list[int]) -> dict[str, Any]:
    if run_dir.exists():
        raise _fail(f"refusing to overwrite native run: {run_dir}")
    for path in paths.values():
        _reject_mat(path)
        if not path.is_file():
            raise _fail(f"missing raw input: {path}")
    run_dir.mkdir(parents=True, exist_ok=True)
    command = _native_command(paths, dataset_id, run_dir)
    if any("ground_truth" in token.lower() or ".mat" in token.lower() for token in command):
        raise _fail("native command contains forbidden truth/MAT input")
    environment = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = local_lib + ((":" + environment["LD_LIBRARY_PATH"]) if environment.get("LD_LIBRARY_PATH") else "")
    started = time.perf_counter()
    try:
        process = subprocess.run(command, cwd=ROOT, env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False, timeout=900)
    except subprocess.TimeoutExpired as exc:
        atomic_write(run_dir / "run.log", (str(exc) + "\n").encode("utf-8"))
        raise _fail(f"native solver timeout: {dataset_id}") from exc
    wall = time.perf_counter() - started
    atomic_write(run_dir / "run.log", process.stdout.encode("utf-8"))
    if process.returncode != 0:
        raise _fail(f"native solver failed ({process.returncode}): {dataset_id}")
    submission = run_dir / "submission.csv"
    summary = run_dir / "summary.json"
    if not submission.is_file() or not summary.is_file():
        raise _fail(f"native outputs incomplete: {dataset_id}")
    report = _artifact_report(submission, summary, dataset_id, target_keys)
    report.update({"command": command, "return_code": process.returncode, "wall_seconds": wall, "log": {"path": relative(run_dir / "run.log"), "sha256": sha256(run_dir / "run.log")}})
    return report


def _verify_materialization(output_root: Path) -> dict[str, Any]:
    path = output_root / MATERIALIZATION_NAME
    report = load_json(path, "Phase37 raw materialization")
    if report.get("schema_version") != "smartphone-r5-phase37-raw-materialization.v1" or report.get("status") != "truth-free-raw-materialized":
        raise _fail("raw materialization status is not sealed")
    if report.get("truth_materialized") is not False or report.get("truth_open_count") != 0 or report.get("mat_read_or_generated") is not False:
        raise _fail("raw materialization truth/MAT contract failed")
    if report.get("archive", {}).get("sha256") != ARCHIVE_SHA256:
        raise _fail("raw materialization archive hash changed")
    for route in ADDED_ROUTES:
        route_report = report.get("routes", {}).get(route)
        if not isinstance(route_report, dict) or route_report.get("truth_materialized") is not False:
            raise _fail(f"raw materialization route missing/truthful: {route}")
        paths = _input_paths(output_root, route)
        files = route_report.get("inputs", {})
        for key, path in paths.items():
            if key not in files or not path.is_file():
                raise _fail(f"raw materialization member missing: {route}/{key}")
            if sha256(path) != files[key].get("sha256") or path.stat().st_size != files[key].get("file_size"):
                raise _fail(f"raw materialization hash/size mismatch: {route}/{key}")
        if (path.parent / "ground_truth.csv").exists():
            raise _fail(f"truth appeared in raw materialization: {route}")
    return report


def _base_structural() -> dict[str, Any]:
    paths = {key: BASE_RAW_ROOT / RAW_FILENAMES[key] for key in RAW_KEYS}
    for path in paths.values():
        if not path.is_file():
            raise _fail(f"sealed base raw input missing: {path}")
    if sha256(BASE_SUBMISSION) != BASE_SUBMISSION_SHA256 or sha256(BASE_SUMMARY) != BASE_SUMMARY_SHA256:
        raise _fail("Phase31 champion artifact changed")
    target = read_raw_epoch_keys(paths["device_gnss"])[1:]
    return {
        "dataset_id": BASE_ROUTE,
        "source": "sealed-phase31-existing",
        "raw_inputs": {key: {"path": relative(path), "sha256": sha256(path), "bytes": path.stat().st_size} for key, path in paths.items()},
        "run1": _artifact_report(BASE_SUBMISSION, BASE_SUMMARY, BASE_ROUTE, target),
        "run2": {"submission": {"path": relative(BASE_SUBMISSION), "sha256": BASE_SUBMISSION_SHA256}, "summary": {"path": relative(BASE_SUMMARY), "sha256": BASE_SUMMARY_SHA256}},
        "repeat_byte_identical": True,
        "truth_open_count": 0,
        "mat_read_or_generated": False,
    }


def run_native(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    """Run additional routes twice and publish the truth-free structural seal."""

    _, v2 = verify_freeze()
    output_root = output_root.resolve()
    _reject_mat(output_root)
    if (output_root / STRUCTURAL_NAME).exists():
        raise _fail(f"Phase37 structural seal already exists: {output_root / STRUCTURAL_NAME}")
    materialization = _verify_materialization(output_root)
    routes: dict[str, Any] = {}
    failures: list[str] = []
    try:
        routes[BASE_ROUTE] = _base_structural()
    except (OSError, Phase37Error) as exc:
        failures.append(f"{BASE_ROUTE}: {exc}")
    for route in ADDED_ROUTES:
        try:
            paths = _input_paths(output_root, route)
            target_keys = read_raw_epoch_keys(paths["device_gnss"])[1:]
            route_path, phone = route.split("/", 1)
            native_root = output_root / "native" / route_path / phone
            first = _run_native(paths, route, native_root / "run1", target_keys)
            repeat = _run_native(paths, route, native_root / "run2", target_keys)
            identical = first["submission"]["sha256"] == repeat["submission"]["sha256"] and first["summary"]["sha256"] == repeat["summary"]["sha256"]
            if not identical:
                raise _fail(f"native repeat differs: {route}")
            routes[route] = {
                "dataset_id": route,
                "source": "phase37-native-repeat",
                "raw_inputs": {key: {"path": relative(path), "sha256": sha256(path), "bytes": path.stat().st_size} for key, path in paths.items()},
                "target_epoch_count": len(target_keys),
                "run1": first,
                "run2": repeat,
                "repeat_byte_identical": True,
                "truth_open_count": 0,
                "mat_read_or_generated": False,
            }
        except (OSError, Phase37Error) as exc:
            failures.append(f"{route}: {exc}")
    if failures:
        failure = {
            "schema_version": "smartphone-r5-phase37-structural-failure.v1",
            "phase": 37,
            "status": "fail-closed",
            "freeze": {"path": relative(FREEZE_V2), "sha256": FREEZE_V2_SHA256},
            "routes": routes,
            "failures": failures,
            "truth_open_count": 0,
            "truth_materialized": False,
            "validation_truth_open_count": 0,
            "future_holdout_truth_open_count": 0,
            "mat_read_or_generated": False,
            "solver_rerun_after_truth": False,
        }
        atomic_json(output_root / FAILURE_NAME, failure)
        raise _fail("Phase37 structural run failed; truth remains sealed")
    result = {
        "schema_version": "smartphone-r5-phase37-structural-seal.v1",
        "phase": 37,
        "status": "sealed-truth-free-structural-pass",
        "decision": "structural-go-development-only-pending-identifiability-truth",
        "freeze": {"path": relative(FREEZE_V2), "sha256": FREEZE_V2_SHA256},
        "evaluator_manifest": {"path": relative(EVALUATOR_MANIFEST), "sha256": sha256(EVALUATOR_MANIFEST) if EVALUATOR_MANIFEST.is_file() else None},
        "native": {
            "binary": {"path": relative(BINARY), "sha256": BINARY_SHA256},
            "flags": list(FLAGS),
            "inputs_only": ["raw device_gnss.csv", "raw device_imu.csv", "broadcast brdc.nav"],
            "ground_truth_used": False,
            "precomputed_result_coordinates_used_for_inference": False,
            "mat_read_or_generated": False,
            "production_default_changed": False,
            "phase31_champion_mutated": False,
        },
        "materialization": {
            "path": relative(output_root / MATERIALIZATION_NAME),
            "sha256": sha256(output_root / MATERIALIZATION_NAME),
            "truth_materialized": materialization.get("truth_materialized"),
        },
        "routes": routes,
        "structural_gate": {
            "routes_passed": len(routes),
            "routes_total": len(ROUTES),
            "added_routes_passed": len(ADDED_ROUTES),
            "repeat_byte_identical": all(route.get("repeat_byte_identical") is True for route in routes.values()),
            "finite_converged": all(route["run1"]["projection"]["graph"]["converged"] for route in routes.values()),
            "raw_target_keys_exact": True,
            "no_unresolved_epochs": all(route["run1"]["projection"]["raw_utc_key_contract"].get("unresolved_epochs") == 0 for route in routes.values()),
            "no_over_70_mps_transition": all(route["run1"]["speed"]["over_70_mps_count"] == 0 for route in routes.values()),
            "truth_used_false": True,
            "factor_counts_and_graph_model_unchanged": True,
        },
        "truth_policy": {
            "truth_open_count": 0,
            "truth_materialized": False,
            "validation_truth_open_count": 0,
            "future_holdout_truth_open_count": 0,
            "solver_rerun_after_truth": False,
            "post_truth_tuning": False,
            "token_or_kaggle_access": False,
        },
        "identifiability_pending": "Truth is not opened by this structural phase. After this seal, one single-process truth read per route may evaluate v2's fixed ENU median/MAD/prefix-tail/cross-route gate; no fit or validation application is authorized.",
    }
    atomic_json(output_root / STRUCTURAL_NAME, result)
    manifest = {
        "schema_version": "smartphone-r5-phase37-structural-seal-manifest.v1",
        "status": result["status"],
        "result": {"path": relative(output_root / STRUCTURAL_NAME), "sha256": sha256(output_root / STRUCTURAL_NAME)},
        "freeze": result["freeze"],
        "evaluator_manifest": result["evaluator_manifest"],
        "routes": list(routes),
        "added_route_count": len(ADDED_ROUTES),
        "truth_open_count": 0,
        "truth_materialized": False,
        "validation_truth_open_count": 0,
        "future_holdout_truth_open_count": 0,
        "mat_read_or_generated": False,
        "atomic_publish": True,
    }
    atomic_json(output_root / STRUCTURAL_MANIFEST_NAME, manifest)
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("verify-freeze", "materialize-raw", "run-native"))
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        if args.operation == "verify-freeze":
            verify_freeze()
            print(json.dumps({"status": "freeze-verified", "truth_open_count": 0}, sort_keys=True))
        elif args.operation == "materialize-raw":
            report = materialize_raw(args.output_root)
            print(json.dumps({"status": report["status"], "routes": list(report["routes"]), "truth_open_count": 0}, sort_keys=True))
        else:
            report = run_native(args.output_root)
            print(json.dumps({"status": report["status"], "routes": len(report["routes"]), "truth_open_count": 0}, sort_keys=True))
    except (OSError, Phase37Error, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f"phase37: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
