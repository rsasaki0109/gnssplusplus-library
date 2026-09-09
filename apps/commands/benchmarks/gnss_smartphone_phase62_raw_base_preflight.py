#!/usr/bin/env python3
"""Single-pass Phase62 base-RINEX availability preflight.

``--verify-freeze`` only reads the sealed source records and never opens the
archive.  ``--audit`` opens the archive once, reads the settings member once,
and streams each declared base observation member once into a minimal
materialization while calculating its hash and RINEX header/time summary.
No handset raw file, navigation file, truth, MAT/WLS/Sv artifact, or solver is
opened by this command.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import statistics
import sys
import tempfile
from typing import Any, BinaryIO
import zipfile


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase62_raw_base_pseudorange_compensation_freeze_v1.json"
FREEZE_SHA256 = "4f12183845e5d22a3773e698182b3a4e8dc788e0dcfaa59ec0b28afcdf1b59b7"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase62_raw_base_preflight_manifest_v1.json"
ARCHIVE = ROOT / "data/gsdc2023/cache/dataset_2023.zip"
OUTPUT = ROOT / "output/smartphone-r5/phase62-raw-base-preflight-v1"

SCHEMA = "smartphone-r5-phase62-raw-base-preflight.v1"
FREEZE_SCHEMA = "smartphone-r5-phase62-raw-base-pseudorange-compensation-freeze.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase62-raw-base-preflight-manifest.v1"
ARCHIVE_SHA256 = "bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408"
SETTINGS_MEMBER = "dataset_2023/settings_train.csv"
SETTINGS_SHA256 = "cb868652632a90919d9b21decaa9b77627d75d16d75287a5430a92b6cf29e080"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)

ROUTE_CONTRACT: dict[str, dict[str, Any]] = {
    ROUTES[0]: {
        "course": "2021-03-16-18-59-us-ca-mtv-a",
        "phone": "pixel5",
        "base1": "SLAC",
        "rinex": "V2",
        "member": "dataset_2023/train/2021-03-16-18-59-us-ca-mtv-a/SLAC_rnx2.obs",
        "file_size": 10708536,
        "compressed_size": 2311786,
        "crc32_hex": "f85411e1",
    },
    ROUTES[1]: {
        "course": "2021-08-24-20-32-us-ca-mtv-h",
        "phone": "pixel5",
        "base1": "P221",
        "rinex": "V2",
        "member": "dataset_2023/train/2021-08-24-20-32-us-ca-mtv-h/P221_rnx2.obs",
        "file_size": 11854125,
        "compressed_size": 2624020,
        "crc32_hex": "f0a0cbcd",
    },
    ROUTES[2]: {
        "course": "2022-04-01-18-22-us-ca-lax-t",
        "phone": "pixel5",
        "base1": "LBCH",
        "rinex": "V2",
        "member": "dataset_2023/train/2022-04-01-18-22-us-ca-lax-t/LBCH_rnx2.obs",
        "file_size": 719969,
        "compressed_size": 167122,
        "crc32_hex": "6016f9bb",
    },
    ROUTES[3]: {
        "course": "2023-03-08-21-34-us-ca-mtv-u",
        "phone": "pixel5",
        "base1": "P221",
        "rinex": "V2",
        "member": "dataset_2023/train/2023-03-08-21-34-us-ca-mtv-u/P221_rnx2.obs",
        "file_size": 5950275,
        "compressed_size": 1374018,
        "crc32_hex": "087ea99d",
    },
}


class PreflightError(ValueError):
    """Raised when the immutable preflight contract cannot be checked."""


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise PreflightError(f"cannot hash {path}: {exc}") from exc
    return digest.hexdigest()


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise PreflightError(f"invalid {label}: {path}") from exc
    if not isinstance(value, dict):
        raise PreflightError(f"{label} is not an object: {path}")
    return value


def _relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT.resolve()))
    except ValueError:
        return str(path)


def _verify_freeze() -> dict[str, Any]:
    """Verify source contracts without touching the archive or any payload."""
    freeze = _read_json(FREEZE, "Phase62 freeze")
    if freeze.get("schema_version") != FREEZE_SCHEMA:
        raise PreflightError("Phase62 freeze schema mismatch")
    if freeze.get("status") != "source-only-frozen-before-raw-read":
        raise PreflightError("Phase62 freeze is not pre-raw")
    source = freeze.get("source_contract")
    if not isinstance(source, dict):
        raise PreflightError("freeze lacks source contract")
    if source.get("repository_commit") != "29923f9f370f09ebc00f96d8cca375007a18e7d5":
        raise PreflightError("upstream commit mismatch")
    if source.get("function_sha256") != "b0536ccff478b0aff253448ffb7a203c715b8064dd8dc85898e38f1f05d0441e":
        raise PreflightError("upstream function hash mismatch")
    moving = source.get("moving_mean")
    if not isinstance(moving, dict) or moving.get("base_interval_1s_samples") != 151 or moving.get("base_interval_15s_samples") != 11:
        raise PreflightError("moving-mean contract mismatch")
    if source.get("matching") != "same satellite and same frequency before residual correction; no cross-frequency or cross-satellite fill":
        raise PreflightError("same-satellite/frequency contract mismatch")
    if source.get("sign") != "the correction is subtracted: P_rover_corrected = P_rover - pc_f(t_rover); this is the source operation obsr.(f).resPc = obsr.(f).resPc-pc":
        raise PreflightError("correction sign contract mismatch")

    manifest = _read_json(MANIFEST, "Phase62 preflight manifest")
    if manifest.get("schema_version") != MANIFEST_SCHEMA:
        raise PreflightError("preflight manifest schema mismatch")
    if manifest.get("status") != "sealed-before-phase62-base-preflight":
        raise PreflightError("preflight manifest is not sealed")
    if manifest.get("freeze_record") != _relative(FREEZE):
        raise PreflightError("manifest freeze path mismatch")
    if manifest.get("freeze_record_sha256") != FREEZE_SHA256:
        raise PreflightError("manifest freeze hash mismatch")
    if _sha256_file(FREEZE) != FREEZE_SHA256:
        raise PreflightError("freeze record changed after seal")
    evaluator = manifest.get("evaluator_source")
    if not isinstance(evaluator, str):
        raise PreflightError("manifest evaluator path missing")
    evaluator_path = ROOT / evaluator
    if not evaluator_path.is_file():
        raise PreflightError("evaluator source missing")
    if _sha256_file(evaluator_path) != manifest.get("evaluator_source_sha256"):
        raise PreflightError("evaluator source hash mismatch")
    archive = manifest.get("archive")
    if not isinstance(archive, dict) or archive.get("path") != _relative(ARCHIVE) or archive.get("sha256") != ARCHIVE_SHA256:
        raise PreflightError("archive contract mismatch")
    if manifest.get("route_order") != list(ROUTES):
        raise PreflightError("route order mismatch")
    read_contract = manifest.get("read_contract")
    if not isinstance(read_contract, dict) or read_contract.get("archive_open_count") != 1 or read_contract.get("settings_member_read_count") != 1 or read_contract.get("base_member_read_count_per_route") != 1:
        raise PreflightError("single-read contract mismatch")
    if read_contract.get("forbidden_member_read_count") != 0:
        raise PreflightError("forbidden-member read contract mismatch")
    return {"freeze": freeze, "manifest": manifest}


def _safe_member(name: str) -> None:
    lowered = name.lower()
    forbidden = (
        ".mat", "ground_truth", "device_gnss", "device_imu", "wls",
        "svposition", "svelevation", "precomputed", "kaggle", "token",
    )
    if any(token in lowered for token in forbidden):
        raise PreflightError(f"forbidden archive member selected: {name}")
    if name.startswith("/") or ".." in Path(name).parts:
        raise PreflightError(f"unsafe archive member: {name}")


def _rnx_major(value: str) -> int:
    match = re.fullmatch(r"V?(\d+)", value.strip(), flags=re.IGNORECASE)
    if not match:
        raise PreflightError(f"invalid RINEX setting: {value!r}")
    return int(match.group(1))


def _expected_member(course: str, base1: str, rinex: str) -> str:
    return f"dataset_2023/train/{course}/{base1}_rnx{_rnx_major(rinex)}.obs"


def _parse_course_window(course: str, nepoch: int) -> tuple[float, float]:
    try:
        # Course names append the site/route token after the YYYY-MM-DD-HH-MM
        # prefix; only that deterministic timestamp prefix defines this
        # preflight's coarse rover window.
        timestamp_prefix = "-".join(course.split("-")[:5])
        start = datetime.strptime(timestamp_prefix, "%Y-%m-%d-%H-%M").replace(tzinfo=timezone.utc)
    except ValueError as exc:
        raise PreflightError(f"invalid Course timestamp: {course}") from exc
    end = start + timedelta(seconds=max(0, nepoch - 1))
    return start.timestamp(), end.timestamp()


def _parse_epoch(line: str) -> float | None:
    if line.startswith(">"):
        fields = line[1:40].split()
    else:
        fields = line[:40].split()
    if len(fields) < 6:
        return None
    try:
        year = int(fields[0])
        if not line.startswith(">"):
            year += 2000 if year < 80 else 1900
        month, day, hour, minute = (int(fields[index]) for index in range(1, 5))
        second = float(fields[5])
        if not all(math.isfinite(value) for value in (second,)):
            return None
        whole = int(second)
        microsecond = int(round((second - whole) * 1_000_000))
        if microsecond >= 1_000_000:
            whole += 1
            microsecond -= 1_000_000
        return datetime(year, month, day, hour, minute, whole, microsecond, tzinfo=timezone.utc).timestamp()
    except (TypeError, ValueError, OverflowError):
        return None


def _iso(seconds: float | None) -> str | None:
    if seconds is None or not math.isfinite(seconds):
        return None
    return datetime.fromtimestamp(seconds, timezone.utc).isoformat().replace("+00:00", "Z")


def _stream_base_member(archive: zipfile.ZipFile, info: zipfile.ZipInfo, destination: Path) -> dict[str, Any]:
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{destination.name}.", suffix=".tmp", dir=str(destination.parent))
    temporary = Path(temporary_name)
    digest = hashlib.sha256()
    byte_count = 0
    in_header = True
    rinex_version: str | None = None
    marker_name: str | None = None
    approx_position: list[float] | None = None
    header_interval: float | None = None
    epochs: list[float] = []
    try:
        with os.fdopen(descriptor, "wb") as target, archive.open(info, "r") as source:
            descriptor = -1
            for raw_line in source:
                target.write(raw_line)
                digest.update(raw_line)
                byte_count += len(raw_line)
                line = raw_line.decode("ascii", errors="replace").rstrip("\r\n")
                label = line[60:].strip() if len(line) >= 60 else ""
                if in_header:
                    if label == "RINEX VERSION / TYPE":
                        rinex_version = line[:9].strip()
                    elif label == "MARKER NAME":
                        marker_name = line[:60].strip() or None
                    elif label == "APPROX POSITION XYZ":
                        values = line[:60].split()
                        try:
                            parsed = [float(value) for value in values[:3]]
                        except ValueError:
                            parsed = []
                        if len(parsed) == 3 and all(math.isfinite(value) for value in parsed):
                            approx_position = parsed
                    elif label == "INTERVAL":
                        try:
                            candidate = float(line[:60].strip())
                        except ValueError:
                            candidate = math.nan
                        if math.isfinite(candidate) and candidate > 0:
                            header_interval = candidate
                    elif label == "END OF HEADER":
                        in_header = False
                else:
                    epoch = _parse_epoch(line)
                    if epoch is not None:
                        epochs.append(epoch)
            target.flush()
            os.fsync(target.fileno())
        os.replace(temporary, destination)
    except (OSError, KeyError, zipfile.BadZipFile) as exc:
        temporary.unlink(missing_ok=True)
        raise PreflightError(f"failed to materialize {info.filename}: {exc}") from exc
    if byte_count != info.file_size:
        raise PreflightError(f"base byte count mismatch for {info.filename}: {byte_count} != {info.file_size}")
    epochs.sort()
    differences = [right - left for left, right in zip(epochs, epochs[1:]) if right > left]
    return {
        "member": info.filename,
        "path": _relative(destination),
        "sha256": digest.hexdigest(),
        "bytes": byte_count,
        "rinex_version": rinex_version,
        "station_id": marker_name,
        "approx_position_xyz_m": approx_position,
        "header_interval_s": header_interval,
        "epoch_count": len(epochs),
        "time_first_utc": _iso(epochs[0] if epochs else None),
        "time_last_utc": _iso(epochs[-1] if epochs else None),
        "time_first_unix_s": epochs[0] if epochs else None,
        "time_last_unix_s": epochs[-1] if epochs else None,
        "observed_interval_median_s": statistics.median(differences) if differences else None,
    }


def _read_settings(archive: zipfile.ZipFile, info: zipfile.ZipInfo) -> tuple[list[dict[str, str]], str, int]:
    with archive.open(info, "r") as source:
        payload = source.read()
    digest = _sha256_bytes(payload)
    if digest != SETTINGS_SHA256:
        raise PreflightError(f"settings hash mismatch: {digest} != {SETTINGS_SHA256}")
    try:
        text = payload.decode("utf-8-sig")
        rows = list(csv.DictReader(text.splitlines()))
    except (UnicodeDecodeError, csv.Error) as exc:
        raise PreflightError("invalid settings_train.csv") from exc
    return rows, digest, len(payload)


def _audit(archive_path: Path, output_root: Path, contract: dict[str, Any]) -> dict[str, Any]:
    if not archive_path.is_file():
        raise PreflightError(f"missing archive: {archive_path}")
    if output_root.exists() and any(output_root.iterdir()):
        raise PreflightError(f"output is non-empty; refusing a second preflight: {output_root}")
    archive_digest = _sha256_file(archive_path)
    if archive_digest != ARCHIVE_SHA256:
        raise PreflightError(f"archive hash mismatch: {archive_digest} != {ARCHIVE_SHA256}")
    output_root.mkdir(parents=True, exist_ok=True)
    member_reads: dict[str, int] = {route: 0 for route in ROUTES}
    forbidden_reads = 0
    route_results: dict[str, Any] = {}
    with zipfile.ZipFile(archive_path, "r") as archive:
        infos = archive.infolist()
        by_name: dict[str, zipfile.ZipInfo] = {}
        duplicates: list[str] = []
        for info in infos:
            if info.filename in by_name:
                duplicates.append(info.filename)
            by_name[info.filename] = info
        if duplicates:
            raise PreflightError(f"archive duplicate names: {duplicates[:3]}")
        settings_names = [name for name in by_name if Path(name).name.lower() == "settings_train.csv"]
        if settings_names != [SETTINGS_MEMBER]:
            raise PreflightError(f"unexpected settings members: {settings_names}")
        settings_rows, settings_digest, settings_bytes = _read_settings(archive, by_name[SETTINGS_MEMBER])
        for route in ROUTES:
            expected = ROUTE_CONTRACT[route]
            matching = [row for row in settings_rows if row.get("Course", "").strip() == expected["course"] and row.get("Phone", "").strip().lower() == expected["phone"]]
            if len(matching) != 1:
                route_results[route] = {"status": "no-go", "reason": f"settings row count {len(matching)}"}
                continue
            row = matching[0]
            base1 = row.get("Base1", "").strip()
            rinex = row.get("RINEX", "").strip()
            try:
                expected_member = _expected_member(expected["course"], base1, rinex)
                course_window = _parse_course_window(expected["course"], int(row.get("Nepoch", "0")))
            except (PreflightError, ValueError) as exc:
                route_results[route] = {"status": "no-go", "reason": str(exc), "settings": {"Base1": base1, "RINEX": rinex}}
                continue
            candidate_members = sorted(name for name in by_name if name.startswith(f"dataset_2023/train/{expected['course']}/") and name.lower().endswith(".obs"))
            if base1 != expected["base1"] or rinex.upper() != expected["rinex"] or expected_member != expected["member"]:
                route_results[route] = {"status": "no-go", "reason": "settings Base1/RINEX differs from sealed route contract", "settings": {"Base1": base1, "RINEX": rinex}, "candidate_members": candidate_members}
                continue
            info = by_name.get(expected_member)
            if info is None:
                route_results[route] = {"status": "no-go", "reason": "declared base member is absent", "settings": {"Base1": base1, "RINEX": rinex}, "candidate_members": candidate_members}
                continue
            _safe_member(info.filename)
            if info.file_size != expected["file_size"] or info.compress_size != expected["compressed_size"] or f"{info.CRC:08x}" != expected["crc32_hex"]:
                route_results[route] = {"status": "no-go", "reason": "base central-directory metadata mismatch", "central": {"bytes": info.file_size, "compressed_bytes": info.compress_size, "crc32_hex": f"{info.CRC:08x}"}}
                continue
            destination = output_root / "routes" / route.replace("/", "__") / "base.obs"
            member_reads[route] += 1
            summary = _stream_base_member(archive, info, destination)
            first = summary.get("time_first_unix_s")
            last = summary.get("time_last_unix_s")
            overlap = bool(first is not None and last is not None and last >= course_window[0] and first <= course_window[1])
            route_results[route] = {
                "status": "pass" if overlap and summary["station_id"] and summary["approx_position_xyz_m"] else "no-go",
                "settings": {"Course": expected["course"], "Phone": expected["phone"], "Base1": base1, "RINEX": rinex, "Nepoch": int(row["Nepoch"])},
                "candidate_members": candidate_members,
                "central": {"bytes": info.file_size, "compressed_bytes": info.compress_size, "crc32_hex": f"{info.CRC:08x}"},
                "course_window_utc": {"first": _iso(course_window[0]), "last": _iso(course_window[1])},
                "base_time_overlap_course_window": overlap,
                "base": {key: value for key, value in summary.items() if key not in {"time_first_unix_s", "time_last_unix_s"}},
            }
    gates = {
        "all_four_routes_have_verified_base": all(route_results.get(route, {}).get("status") == "pass" for route in ROUTES),
        "settings_member_hash": settings_digest == SETTINGS_SHA256,
        "settings_member_bytes": settings_bytes,
        "archive_hash": archive_digest,
        "base_member_reads_each": all(member_reads[route] == 1 for route in ROUTES),
        "forbidden_member_reads": forbidden_reads == 0,
    }
    return {
        "schema_version": SCHEMA,
        "status": "pass" if all(gates.values()) else "no-go-base-preflight",
        "archive": {"path": _relative(archive_path), "sha256": archive_digest, "zip_open_count": 1},
        "settings": {"member": SETTINGS_MEMBER, "sha256": settings_digest, "bytes": settings_bytes, "read_count": 1},
        "routes": route_results,
        "gates": gates,
        "read_accounting": {"archive_stream_reads": 1, "archive_open_count": 1, "settings_member_read_count": 1, "base_member_read_count_per_route": member_reads, "forbidden_member_read_count": forbidden_reads, "device_gnss_read_count": 0, "truth_read_count": 0, "mat_read_count": 0, "nav_read_count": 0, "solver_invocation_count": 0},
        "source_only": True,
    }


def _write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            descriptor = -1
            json.dump(value, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        temporary.unlink(missing_ok=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--verify-freeze", action="store_true")
    mode.add_argument("--audit", action="store_true")
    parser.add_argument("--archive", type=Path, default=ARCHIVE)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args(argv)
    try:
        contract = _verify_freeze()
        if args.verify_freeze:
            print(json.dumps({"status": "verified", "archive_open_count": 0, "payload_reads": 0}, sort_keys=True))
            return 0
        result = _audit(args.archive.resolve(), args.output.resolve(), contract)
        _write_json(args.output.resolve() / "phase62_raw_base_preflight.json", result)
        print(json.dumps({"status": result["status"], "output": _relative(args.output.resolve()), "archive_open_count": 1}, sort_keys=True))
        return 0
    except PreflightError as exc:
        print(f"phase62 preflight failed closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
