#!/usr/bin/env python3
"""Run the frozen native-FGO lane on every GSDC test route.

This is a truth-free, development-only batch.  The test authorization is
content addressed and must be verified before a test member is materialized.
Each route first tries the frozen Eigen native-FGO recipe.  A failed FGO
route may use the already-authorized sparse WLS extractor; a failed WLS
fallback remains a sealed route failure and prevents submission publication.
The official sample is used only for its exact key order/schema and, for
unresolved first/omitted epochs, its finite coordinate values are an explicit
non-truth fallback.  No test truth member is ever opened.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import resource
import shutil
import subprocess
import sys
import tempfile
import time
from typing import Any
import zipfile

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))

import gnss_smartphone_gnss_adapter as adapter  # noqa: E402
import gnss_smartphone_native_fgo_eval as native  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_wls as wls  # noqa: E402


BATCH_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-test-batch.v1"
AUTH_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-test-authorization.v1"
AUTH_MANIFEST_SCHEMA = (
    "smartphone-r5-gsdc2023-native-fgo-test-authorization-manifest.v1"
)
RUN_MANIFEST_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-test-batch-run-manifest.v1"
ROUTE_MANIFEST_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-test-route-manifest.v1"
SAMPLE_FIELDS = (
    "tripId",
    "UnixTimeMillis",
    "LatitudeDegrees",
    "LongitudeDegrees",
)
SAMPLE_SCHEMA = "smartphone-r5-gsdc2023-official-sample-submission.v2"
SKIP_EPOCHS = 1
LEAP_SECONDS = 18
MAX_RUNTIME_SECONDS = 900
MAX_ADDRESS_SPACE_BYTES = 8 * 1024 * 1024 * 1024
MAX_ARTIFACT_BYTES = 1024 * 1024 * 1024
FGO_RECIPE_HASH = "4633bfd3a86cf34ebd86820ed59ee7192b3cbf23fc75ce9e72fc1f2c88fb39f6"
FGO_CORE_HASH = "e27eb31f4fcb597a1c5b392d8b558f429b25cd5c5a4b39b6474491209649f7b1"
ARCHIVE_SHA256 = "bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408"
INVENTORY_SHA256 = "dbf89222e6860d4fb31f12c9c46402033ed31e905f8fe3f6615e5131cb9e9495"
SAMPLE_SHA256 = "b0c4853076f715d6bdca46e5c8c99e575f7982d8ee4fc9c0fe417507badcb780"
WLS_AUTH = (
    ROOT
    / "output"
    / "smartphone-r5"
    / "wls-test-batch-edge-completeness-fallback-v2-1"
    / "test_authorization.json"
)
WLS_AUTH_MANIFEST = WLS_AUTH.with_name("test_authorization_manifest.json")
WLS_AUTH_SHA256 = "12d1ae36377e94a0ee3a6b946dd7fcf4d35d3b8ade360e2d532cf96e97f19a43"
WLS_AUTH_MANIFEST_SHA256 = "96fc259ccdfb4eb930f5b2b14387396d48b194772257b874cc0af8cf876a290d"

DEFAULT_ARCHIVE = ROOT / "data" / "gsdc2023" / "cache" / "dataset_2023.zip"
DEFAULT_INVENTORY = ROOT / "output" / "smartphone-r5" / "wls-test-batch-v1" / "test_inventory.json"
DEFAULT_SAMPLE = ROOT / "data" / "gsdc2023" / "sample_submission.csv"
DEFAULT_AUTH = ROOT / "docs" / "use_cases" / "records" / "smartphone_r5_gsdc2023_native_fgo_test_batch_freeze.json"
DEFAULT_AUTH_MANIFEST = DEFAULT_AUTH.with_name(
    "smartphone_r5_gsdc2023_native_fgo_test_batch_freeze_manifest.json"
)
DEFAULT_OUTPUT = ROOT / "output" / "smartphone-r5" / "native-fgo-test-v1"
DEFAULT_FGO_BINARY = ROOT / "build" / "apps" / "gnss_fgo"
DEFAULT_SPP_BINARY = ROOT / "build" / "apps" / "gnss_spp"


class TestBatchError(ValueError):
    """Raised when the frozen truth-free test contract cannot be proven."""


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise TestBatchError(f"missing file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise TestBatchError(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def _resolve(path: str | Path) -> Path:
    candidate = Path(path)
    return candidate if candidate.is_absolute() else ROOT / candidate


def _relative(path: str | Path) -> Path:
    candidate = Path(path)
    if candidate.is_absolute() or ".." in candidate.parts:
        raise TestBatchError(f"authorization path must be repository-relative: {path}")
    return ROOT / candidate


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise TestBatchError(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise TestBatchError(f"{label} must be a JSON object: {path}")
    return payload


def _atomic_bytes(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    _atomic_bytes(
        path,
        (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"),
    )


def _artifact(path: Path, *, name: str | None = None) -> dict[str, Any]:
    if not path.is_file():
        raise TestBatchError(f"missing artifact: {path}")
    size = path.stat().st_size
    if size > MAX_ARTIFACT_BYTES:
        raise TestBatchError(f"artifact exceeds size limit: {path}")
    return {
        "path": name if name is not None else str(path.relative_to(ROOT)),
        "bytes": size,
        "sha256": _sha256(path),
    }


def _safe_id(value: str) -> str:
    return value.replace("/", "__")


def _test_member(route: str, phone: str, basename: str) -> str:
    return f"dataset_2023/test/{route}/{phone}/{basename}"


def _nav_member(route: str) -> str:
    return f"dataset_2023/test/{route}/brdc.nav"


def _central_metadata(archive_path: Path, member: str) -> dict[str, Any]:
    try:
        with zipfile.ZipFile(archive_path) as source:
            entries = [info for info in source.infolist() if info.filename == member]
    except (OSError, zipfile.BadZipFile) as exc:
        raise TestBatchError(f"failed to inspect central metadata: {member}") from exc
    if len(entries) != 1 or entries[0].is_dir():
        raise TestBatchError(f"test member is not a unique file: {member}")
    info = entries[0]
    return {
        "name": info.filename,
        "file_size": info.file_size,
        "compressed_size": info.compress_size,
        "crc32_hex": f"{info.CRC:08x}",
    }


def _materialize_member(
    archive_path: Path,
    member: str,
    output: Path,
    expected: dict[str, Any],
) -> dict[str, Any]:
    """Extract one already-authorized member using metadata and atomic rename."""

    if output.is_file():
        if output.stat().st_size != int(expected["file_size"]):
            raise TestBatchError(f"existing materialized size differs: {member}")
        artifact = _artifact(output)
        return {"member": member, "metadata": expected, **artifact}
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.name}.", suffix=".tmp", dir=str(output.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as target:
            with zipfile.ZipFile(archive_path) as source:
                info = source.getinfo(member)
                actual = {
                    "name": info.filename,
                    "file_size": info.file_size,
                    "compressed_size": info.compress_size,
                    "crc32_hex": f"{info.CRC:08x}",
                }
                if actual != expected:
                    raise TestBatchError(f"central metadata changed: {member}")
                with source.open(member, "r") as input_stream:
                    shutil.copyfileobj(input_stream, target)
            target.flush()
            os.fsync(target.fileno())
        if temporary.stat().st_size != int(expected["file_size"]):
            raise TestBatchError(f"materialized size differs: {member}")
        os.replace(temporary, output)
    except (OSError, KeyError, zipfile.BadZipFile):
        raise TestBatchError(f"failed to materialize test member: {member}")
    finally:
        temporary.unlink(missing_ok=True)
    return {"member": member, "metadata": expected, **_artifact(output)}


def _source_hash_value(value: Any) -> str | None:
    if isinstance(value, dict):
        value = value.get("sha256")
    return value if isinstance(value, str) else None


def _verify_source_hashes(payload: dict[str, Any]) -> None:
    values = payload.get("source_files")
    if not isinstance(values, dict) or not values:
        raise TestBatchError("test authorization source_files are missing")
    for relative, expected in values.items():
        if not isinstance(relative, str):
            raise TestBatchError("test authorization source path is invalid")
        actual = _sha256(_relative(relative))
        if actual != _source_hash_value(expected):
            raise TestBatchError(f"test authorization source hash differs: {relative}")


def _read_sample(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise TestBatchError(f"official sample is missing: {path}")
    rows: list[dict[str, Any]] = []
    seen: set[tuple[str, int]] = set()
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            if tuple(reader.fieldnames or ()) != SAMPLE_FIELDS:
                raise TestBatchError("official sample header is not exact")
            for line, raw in enumerate(reader, start=2):
                if None in raw:
                    raise TestBatchError(f"official sample row {line} has extra fields")
                trip_id = raw.get("tripId") or ""
                timestamp_text = raw.get("UnixTimeMillis") or ""
                if (
                    not trip_id
                    or trip_id != trip_id.strip()
                    or not trip_id.isprintable()
                    or any(char.isspace() for char in trip_id)
                    or trip_id.count("/") != 1
                    or timestamp_text != timestamp_text.strip()
                ):
                    raise TestBatchError(f"official sample row {line} key is invalid")
                try:
                    timestamp = int(timestamp_text)
                except ValueError as exc:
                    raise TestBatchError(f"official sample row {line} timestamp is invalid") from exc
                if timestamp < 0:
                    raise TestBatchError(f"official sample row {line} timestamp is negative")
                coordinates: tuple[float, float]
                try:
                    coordinates = (
                        float(raw.get("LatitudeDegrees") or ""),
                        float(raw.get("LongitudeDegrees") or ""),
                    )
                except ValueError as exc:
                    raise TestBatchError(f"official sample row {line} fallback coordinate is invalid") from exc
                if not all(math.isfinite(value) for value in coordinates):
                    raise TestBatchError(f"official sample row {line} fallback coordinate is non-finite")
                if not -90.0 <= coordinates[0] <= 90.0 or not -180.0 <= coordinates[1] <= 180.0:
                    raise TestBatchError(f"official sample row {line} fallback coordinate is out of range")
                key = (trip_id, timestamp)
                if key in seen:
                    raise TestBatchError(f"official sample duplicate key: {key!r}")
                seen.add(key)
                rows.append(
                    {
                        "trip_id": trip_id,
                        "timestamp": timestamp,
                        "timestamp_text": timestamp_text,
                        "latitude": coordinates[0],
                        "longitude": coordinates[1],
                    }
                )
    except OSError as exc:
        raise TestBatchError(f"failed to read official sample: {path}") from exc
    if len(rows) != 71936:
        raise TestBatchError(f"official sample row count differs: {len(rows)}")
    return {
        "schema_version": SAMPLE_SCHEMA,
        "fields": list(SAMPLE_FIELDS),
        "rows": rows,
        "key_count": len(rows),
        "artifact": _artifact(path),
    }


def _verify_auth(
    auth_path: Path,
    auth_manifest_path: Path,
    archive_path: Path,
    inventory_path: Path,
    sample_path: Path,
    fgo_binary: Path,
    spp_binary: Path,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], str, str, str]:
    auth = _load_json(auth_path, "native-FGO test authorization")
    if auth.get("schema_version") != AUTH_SCHEMA or auth.get("status") not in {
        "sealed-before-test-payload-access",
        "sealed-recovery-before-test-rerun",
    }:
        raise TestBatchError("native-FGO test authorization schema/status differs")
    if auth.get("status") == "sealed-recovery-before-test-rerun":
        recovery = auth.get("recovery")
        if not isinstance(recovery, dict) or recovery.get("truth_open_count") != 0 or recovery.get("algorithm_unchanged") is not True:
            raise TestBatchError("test recovery authorization is not truth-free and unchanged")
    contract = auth.get("test_execution_contract")
    if not isinstance(contract, dict):
        raise TestBatchError("native-FGO test authorization contract is missing")
    for key, expected in (
        ("authorized", True),
        ("role", "test"),
        ("truth_free_phase", True),
        ("truth_access_forbidden", True),
        ("no_post_test_tuning", True),
        ("no_external_mutation", True),
        ("skip_epochs", SKIP_EPOCHS),
    ):
        if contract.get(key) != expected:
            raise TestBatchError(f"test authorization differs for {key}")
    allowlist = contract.get("dataset_allowlist")
    if not isinstance(allowlist, list) or len(allowlist) != 40 or len(set(allowlist)) != 40:
        raise TestBatchError("test authorization allowlist must contain 40 unique IDs")
    if any(not isinstance(value, str) or value.count("/") != 1 for value in allowlist):
        raise TestBatchError("test authorization allowlist contains an invalid ID")
    if auth.get("algorithm_parameter_hash") != FGO_RECIPE_HASH or auth.get("algorithm_core_hash") != FGO_CORE_HASH:
        raise TestBatchError("native-FGO algorithm hash differs from frozen recipe")
    archive_hash = _sha256(archive_path)
    inventory_hash = _sha256(inventory_path)
    if archive_hash != ARCHIVE_SHA256 or inventory_hash != INVENTORY_SHA256:
        raise TestBatchError("archive or test inventory hash differs")
    archive_contract = auth.get("archive")
    inventory_contract = auth.get("inventory")
    if not isinstance(archive_contract, dict) or archive_contract.get("sha256") != archive_hash:
        raise TestBatchError("test authorization archive contract differs")
    if not isinstance(inventory_contract, dict) or inventory_contract.get("sha256") != inventory_hash:
        raise TestBatchError("test authorization inventory contract differs")
    if archive_contract.get("central_directory_only") is not True:
        raise TestBatchError("test authorization archive is not central-directory-only")
    inventory = _load_json(inventory_path, "test inventory")
    if inventory.get("schema_version") != "smartphone-r5-gsdc2023-test-central-inventory.v1":
        raise TestBatchError("test inventory schema differs")
    inventory_archive = inventory.get("archive")
    if not isinstance(inventory_archive, dict) or inventory_archive.get("central_directory_only") is not True:
        raise TestBatchError("test inventory central-directory contract differs")
    if inventory_archive.get("member_content_read") is not False:
        raise TestBatchError("test inventory records payload reads")
    truth_policy = inventory.get("truth_policy")
    if not isinstance(truth_policy, dict) or truth_policy.get("test_truth_payload_opened") is not False or truth_policy.get("truth_materialization_forbidden") is not True:
        raise TestBatchError("test inventory truth policy is not closed")
    records = inventory.get("test", {}).get("records")
    if not isinstance(records, list) or len(records) != 40:
        raise TestBatchError("test inventory must contain 40 route-phone records")
    ids = [str(row.get("dataset_id")) for row in records if isinstance(row, dict)]
    if len(ids) != 40 or set(ids) != set(allowlist):
        raise TestBatchError("test inventory IDs differ from authorization allowlist")
    if len({str(row.get("route")) for row in records}) != 40:
        raise TestBatchError("test inventory is not one phone per route")
    for row in records:
        if not isinstance(row, dict) or row.get("required_files_complete") is not True or row.get("truth_present") is not False:
            raise TestBatchError("test inventory record is incomplete or truth-bearing")
        device = row.get("central_directory_files", {}).get("device_gnss.csv")
        nav = row.get("central_directory_broadcast_nav")
        if not isinstance(device, dict) or not isinstance(nav, dict):
            raise TestBatchError(f"test inventory central metadata is malformed: {row.get('dataset_id')}")
        route, phone = str(row["dataset_id"]).split("/", 1)
        if _central_metadata(archive_path, str(device["name"])) != device:
            raise TestBatchError(f"test device central metadata differs: {row['dataset_id']}")
        if _central_metadata(archive_path, str(nav["name"])) != nav:
            raise TestBatchError(f"test navigation central metadata differs: {row['dataset_id']}")
        if device["name"] != _test_member(route, phone, "device_gnss.csv") or nav["name"] != _nav_member(route):
            raise TestBatchError(f"test inventory member path differs: {row['dataset_id']}")
    sample = _read_sample(sample_path)
    if sample["artifact"]["sha256"] != SAMPLE_SHA256:
        raise TestBatchError("official sample hash differs")
    sample_contract = auth.get("sample_submission")
    if not isinstance(sample_contract, dict) or sample_contract.get("sha256") != SAMPLE_SHA256 or sample_contract.get("key_count") != 71936 or sample_contract.get("header") != list(SAMPLE_FIELDS):
        raise TestBatchError("test authorization official sample contract differs")
    source_hashes = auth.get("source_files")
    _verify_source_hashes(auth)
    binary_contract = auth.get("release_binaries")
    if not isinstance(binary_contract, dict):
        raise TestBatchError("test authorization release binaries are missing")
    for label, path in (("gnss_fgo", fgo_binary), ("gnss_spp", spp_binary)):
        row = binary_contract.get(label)
        if not isinstance(row, dict) or row.get("sha256") != _sha256(path):
            raise TestBatchError(f"test authorization binary differs: {label}")
    upstream = auth.get("holdout_gate")
    if not isinstance(upstream, dict) or upstream.get("status") != "holdout-pass-development-only" or upstream.get("truth_open_count") != 1 or upstream.get("test_batch_authorized") is not True:
        raise TestBatchError("native-FGO holdout gate does not authorize test batch")
    for label in ("evaluation_record", "evaluation_manifest", "evaluation_report", "evaluation_output_manifest"):
        row = upstream.get(label)
        if not isinstance(row, dict):
            raise TestBatchError(f"holdout gate evidence is missing: {label}")
        evidence_path = _relative(str(row.get("path", "")))
        if _sha256(evidence_path) != row.get("sha256"):
            raise TestBatchError(f"holdout gate evidence hash differs: {label}")
    wls_contract = auth.get("wls_fallback")
    if not isinstance(wls_contract, dict) or wls_contract.get("authorization_sha256") != WLS_AUTH_SHA256 or wls_contract.get("manifest_sha256") != WLS_AUTH_MANIFEST_SHA256:
        raise TestBatchError("WLS fallback authorization contract differs")
    if _sha256(WLS_AUTH) != WLS_AUTH_SHA256 or _sha256(WLS_AUTH_MANIFEST) != WLS_AUTH_MANIFEST_SHA256:
        raise TestBatchError("WLS fallback authorization hash differs")
    manifest = _load_json(auth_manifest_path, "native-FGO test authorization manifest")
    if manifest.get("schema_version") != AUTH_MANIFEST_SCHEMA:
        raise TestBatchError("native-FGO test authorization manifest schema differs")
    record_ref = manifest.get("authorization_record")
    auth_hash = _sha256(auth_path)
    if not isinstance(record_ref, dict) or record_ref.get("path") != str(auth_path.relative_to(ROOT)) or record_ref.get("sha256") != auth_hash:
        raise TestBatchError("test authorization record hash differs")
    if manifest.get("archive_sha256") != archive_hash or manifest.get("inventory_sha256") != inventory_hash or manifest.get("sample_sha256") != SAMPLE_SHA256:
        raise TestBatchError("test authorization manifest input hashes differ")
    if manifest.get("dataset_allowlist") != allowlist or manifest.get("truth_open_count") != 0 or manifest.get("truth_materialized_count") != 0:
        raise TestBatchError("test authorization manifest contract differs")
    return auth, manifest, inventory, archive_hash, inventory_hash, auth_hash


def _child_limits() -> None:
    resource.setrlimit(resource.RLIMIT_AS, (MAX_ADDRESS_SPACE_BYTES, MAX_ADDRESS_SPACE_BYTES))
    resource.setrlimit(resource.RLIMIT_CPU, (MAX_RUNTIME_SECONDS, MAX_RUNTIME_SECONDS + 1))


def _run_child(command: list[str], work_dir: Path, label: str) -> dict[str, Any]:
    started = time.perf_counter()
    work_dir.mkdir(parents=True, exist_ok=True)
    stdout_path = work_dir / f"{label}.stdout.log"
    stderr_path = work_dir / f"{label}.stderr.log"
    environment = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = local_lib + os.pathsep + environment.get("LD_LIBRARY_PATH", "")
    timed_out = False
    return_code: int | None = None
    with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
        try:
            completed = subprocess.run(
                command,
                cwd=ROOT,
                env=environment,
                stdin=subprocess.DEVNULL,
                stdout=stdout,
                stderr=stderr,
                timeout=MAX_RUNTIME_SECONDS + 5,
                preexec_fn=_child_limits,
                check=False,
            )
            return_code = completed.returncode
        except subprocess.TimeoutExpired:
            timed_out = True
    result = {
        "command": command,
        "return_code": return_code,
        "timed_out": timed_out,
        "wall_seconds": time.perf_counter() - started,
        "child_max_rss_kib": resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
        "stdout": stdout_path.name,
        "stderr": stderr_path.name,
    }
    if timed_out or return_code != 0:
        raise TestBatchError(f"{label} failed")
    return result


def _adapter_command(device: Path, nav: Path, output: Path, dataset_id: str, phone: str, auth_path: Path, auth_manifest: Path) -> list[str]:
    return [
        sys.executable,
        str(ROOT / "apps" / "gnss.py"),
        "smartphone-gnss-adapter",
        "--device-gnss", str(device),
        "--output-dir", str(output),
        "--dataset-id", dataset_id,
        "--device-model", phone,
        "--source-url", "https://taroz.net/data/dataset_2023.zip",
        "--source-terms", "Google Smartphone Decimeter Challenge dataset; archive hash frozen",
        "--role", "test",
        "--skip-epochs", str(SKIP_EPOCHS),
        "--truth-free",
        "--experimental-galileo-e1",
        "--broadcast-nav", str(nav),
        "--sealed-test-authorization", str(auth_path),
        "--sealed-test-authorization-manifest", str(auth_manifest),
    ]


def _spp_command(obs: Path, nav: Path, output: Path, binary: Path) -> list[str]:
    return [
        str(binary), "--obs", str(obs), "--nav", str(nav),
        "--out", str(output / "libgnsspp_spp.pos"),
        "--summary-json", str(output / "libgnsspp_spp_summary.json"),
        "--clock-csv", str(output / "libgnsspp_spp_clock.csv"),
        "--timing-csv", str(output / "libgnsspp_spp_timing.csv"), "--quiet",
    ]


def _validate_adapter(output: Path, dataset_id: str) -> dict[str, Any]:
    summary = _load_json(output / "summary.json", "test adapter summary")
    if summary.get("truth_free") is not True or summary.get("decision") != "truth-free-pipeline":
        raise TestBatchError("test adapter was not truth-free")
    dataset = summary.get("dataset")
    if not isinstance(dataset, dict) or dataset.get("id") != dataset_id or dataset.get("role") != "test":
        raise TestBatchError("test adapter dataset identity differs")
    truth = summary.get("truth", {})
    if truth.get("used") is not False or summary.get("inputs", {}).get("ground_truth") is not None:
        raise TestBatchError("test adapter touched truth")
    if not isinstance(summary.get("navigation"), dict):
        raise TestBatchError("test adapter did not validate navigation")
    names = ("observations.csv", "rover.obs", "receiver_wls.csv", "reference.csv", "summary.json")
    return {name: _artifact(output / name, name=f"adapter/{name}") for name in names}


def _validate_spp(output: Path) -> dict[str, Any]:
    position = output / "libgnsspp_spp.pos"
    rows = smoother._read_positions(position, LEAP_SECONDS)
    if not rows or not all(math.isfinite(float(value)) for row in rows for value in (*row.ecef, row.latitude, row.longitude, row.height)):
        raise TestBatchError("SPP seed is invalid")
    names = ("libgnsspp_spp.pos", "libgnsspp_spp_summary.json", "libgnsspp_spp_clock.csv", "libgnsspp_spp_timing.csv")
    return {name: _artifact(output / name, name=f"spp/{name}") for name in names}


def _position_map(path: Path) -> dict[int, tuple[float, float]]:
    rows = smoother._read_positions(path, LEAP_SECONDS)
    result: dict[int, tuple[float, float]] = {}
    for row in rows:
        values = (float(row.latitude), float(row.longitude))
        if row.timestamp_ms in result or not all(math.isfinite(value) for value in values):
            raise TestBatchError("position output has duplicate/nonfinite timestamp")
        if not -90.0 <= values[0] <= 90.0 or not -180.0 <= values[1] <= 180.0:
            raise TestBatchError("position output is out of range")
        result[row.timestamp_ms] = values
    if not result:
        raise TestBatchError("position output is empty")
    return result


def _cache_key(
    record: dict[str, Any],
    archive_hash: str,
    inventory_hash: str,
    auth_hash: str,
    fgo_binary: Path,
    spp_binary: Path,
) -> str:
    payload = {
        "dataset_id": record["dataset_id"],
        "device_metadata": record["central_directory_files"]["device_gnss.csv"],
        "nav_metadata": record["central_directory_broadcast_nav"],
        "archive_sha256": archive_hash,
        "inventory_sha256": inventory_hash,
        "authorization_sha256": auth_hash,
        "algorithm_parameter_hash": FGO_RECIPE_HASH,
        "algorithm_core_hash": FGO_CORE_HASH,
        "fgo_binary_sha256": _sha256(fgo_binary),
        "spp_binary_sha256": _sha256(spp_binary),
        "skip_epochs": SKIP_EPOCHS,
    }
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()


def _load_cached(cache_dir: Path, dataset_id: str) -> dict[str, Any] | None:
    manifest_path = cache_dir / "route_cache_manifest.json"
    if not manifest_path.is_file():
        return None
    manifest = _load_json(manifest_path, "cached route manifest")
    if manifest.get("schema_version") != ROUTE_MANIFEST_SCHEMA or manifest.get("dataset_id") != dataset_id or manifest.get("truth_used") is not False:
        return None
    position = manifest.get("selected_position")
    if not isinstance(position, dict) or not isinstance(position.get("path"), str):
        return None
    position_path = cache_dir / position["path"]
    if not position_path.is_file() or _sha256(position_path) != position.get("sha256"):
        return None
    return manifest


def _save_cache_manifest(cache_dir: Path, manifest: dict[str, Any]) -> dict[str, Any]:
    path = cache_dir / "route_cache_manifest.json"
    _atomic_json(path, manifest)
    manifest["manifest_sha256"] = _sha256(path)
    return manifest


def _run_native_route(
    record: dict[str, Any],
    route_root: Path,
    cache_dir: Path,
    archive_path: Path,
    auth_path: Path,
    auth_manifest_path: Path,
    fgo_binary: Path,
    spp_binary: Path,
) -> dict[str, Any]:
    dataset_id = str(record["dataset_id"])
    route, phone = dataset_id.split("/", 1)
    device_meta = record["central_directory_files"]["device_gnss.csv"]
    nav_meta = record["central_directory_broadcast_nav"]
    inputs = route_root / "inputs"
    device_path = inputs / "device_gnss.csv"
    nav_path = inputs / "brdc.nav"
    materialized = {
        "device_gnss": _materialize_member(archive_path, _test_member(route, phone, "device_gnss.csv"), device_path, device_meta),
        "broadcast_nav": _materialize_member(archive_path, _nav_member(route), nav_path, nav_meta),
    }
    cached = _load_cached(cache_dir, dataset_id)
    if cached is not None:
        position_path = cache_dir / str(cached["selected_position"]["path"])
        result = dict(cached)
        result["resumed"] = True
        result["materialized"] = materialized
        result["position_map"] = _position_map(position_path)
        return result
    work = Path(tempfile.mkdtemp(prefix=f".{cache_dir.name}.", dir=str(cache_dir.parent)))
    try:
        adapter_dir = work / "adapter"
        adapter_run = _run_child(_adapter_command(device_path, nav_path, adapter_dir, dataset_id, phone, auth_path, auth_manifest_path), work, "adapter")
        adapter_artifacts = _validate_adapter(adapter_dir, dataset_id)
        spp_dir = work / "spp"
        spp_dir.mkdir(parents=True, exist_ok=True)
        spp_run = _run_child(_spp_command(adapter_dir / "rover.obs", nav_path, spp_dir, spp_binary), work, "spp")
        spp_artifacts = _validate_spp(spp_dir)
        entry = {
            "obs": str((adapter_dir / "rover.obs").resolve()),
            "nav": str(nav_path.resolve()),
            "seed_pos": str((spp_dir / "libgnsspp_spp.pos").resolve()),
            "device_gnss": str(device_path.resolve()),
        }
        fgo_dir = work / "fgo"
        fgo_command = native._command(fgo_binary.resolve(), entry, fgo_dir)
        fgo_run = _run_child(fgo_command, fgo_dir, "fgo")
        summary = native._validate_summary(native._load_json(fgo_dir / "fgo_summary.json", "test FGO summary"))
        native._validate_pos(fgo_dir / "fgo.pos", int(summary["valid_solutions"]))
        fgo_names = ("fgo.pos", "fgo_summary.json", "fgo_epoch_debug.csv", "fgo_factor_debug.csv", "fgo_cost_trace.csv")
        fgo_artifacts = {name: _artifact(fgo_dir / name, name=f"fgo/{name}") for name in fgo_names}
        artifacts = {**adapter_artifacts, **spp_artifacts, **fgo_artifacts}
        for label, base in (("adapter", work), ("spp", work), ("fgo", fgo_dir)):
            for log_name in (f"{label}.stdout.log", f"{label}.stderr.log"):
                log_path = base / log_name
                if log_path.is_file():
                    artifacts[f"{label}/{log_name}"] = _artifact(log_path, name=f"{label}/{log_name}")
        factor_keys = (
            "input_epochs", "optimized_epochs", "valid_solutions", "pseudorange_factors",
            "tdcp_factors", "tdcp_factors_inserted", "motion_factors",
            "single_difference_doppler_factors", "single_difference_tdcp_factors",
            "carrier_phase_factors", "double_difference_pseudorange_factors",
            "double_difference_carrier_factors",
        )
        selected_position = fgo_dir / "fgo.pos"
        manifest = {
            "schema_version": ROUTE_MANIFEST_SCHEMA,
            "status": "native-fgo-success",
            "dataset_id": dataset_id,
            "route": route,
            "phone": phone,
            "lane": "native_fgo",
            "truth_free": True,
            "truth_used": False,
            "resumed": False,
            "materialized": materialized,
            "adapter_run": adapter_run,
            "spp_run": spp_run,
            "fgo_run": fgo_run,
            "factor_coverage": {key: summary[key] for key in factor_keys},
            "selected_position": _artifact(selected_position, name="fgo/fgo.pos"),
            "artifacts": artifacts,
            "algorithm_parameter_hash": FGO_RECIPE_HASH,
            "algorithm_core_hash": FGO_CORE_HASH,
            "fallback": "WLS is attempted only if this route fails before native-FGO publication",
            "runtime": {"wall_seconds": adapter_run["wall_seconds"] + spp_run["wall_seconds"] + fgo_run["wall_seconds"], "child_max_rss_kib": max(adapter_run["child_max_rss_kib"], spp_run["child_max_rss_kib"], fgo_run["child_max_rss_kib"])},
        }
        cache_dir.mkdir(parents=True, exist_ok=True)
        for source, relative in ((selected_position, Path("fgo/fgo.pos")),):
            target = cache_dir / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
        for name in ("fgo_summary.json", "fgo_epoch_debug.csv", "fgo_factor_debug.csv", "fgo_cost_trace.csv"):
            target = cache_dir / "fgo" / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(fgo_dir / name, target)
        manifest["selected_position"] = _artifact(cache_dir / "fgo/fgo.pos", name="fgo/fgo.pos")
        manifest["cache_artifacts"] = {name: _artifact(cache_dir / f"fgo/{name}", name=f"fgo/{name}") for name in ("fgo.pos", "fgo_summary.json", "fgo_epoch_debug.csv", "fgo_factor_debug.csv", "fgo_cost_trace.csv")}
        _save_cache_manifest(cache_dir, manifest)
        result = dict(manifest)
        result["position_map"] = _position_map(cache_dir / "fgo/fgo.pos")
        return result
    finally:
        shutil.rmtree(work, ignore_errors=True)


def _run_wls_fallback(
    record: dict[str, Any],
    route_root: Path,
    cache_dir: Path,
    device_path: Path,
    auth_path: Path,
    auth_manifest_path: Path,
    reason: str,
) -> dict[str, Any]:
    dataset_id = str(record["dataset_id"])
    fallback_dir = cache_dir / "wls_fallback"
    position = fallback_dir / "wls.pos"
    if not position.is_file():
        fallback_dir.mkdir(parents=True, exist_ok=True)
        wls.extract_to_directory(
            device_path,
            fallback_dir,
            skip_epochs=SKIP_EPOCHS,
            role="test",
            dataset_id=dataset_id,
            sealed_test_authorization=WLS_AUTH,
            sealed_test_authorization_manifest=WLS_AUTH_MANIFEST,
            truth_free=True,
            allow_missing_wls_epochs=True,
            allow_invalid_wls_epochs=True,
        )
    position_map = _position_map(position)
    manifest = {
        "schema_version": ROUTE_MANIFEST_SCHEMA,
        "status": "wls-fallback-success",
        "dataset_id": dataset_id,
        "route": record["route"],
        "phone": record["phone"],
        "lane": "wls_fallback",
        "truth_free": True,
        "truth_used": False,
        "selected_position": _artifact(position, name="wls_fallback/wls.pos"),
        "wls_manifest": _artifact(fallback_dir / "wls_manifest.json", name="wls_fallback/wls_manifest.json"),
        "wls_summary": _artifact(fallback_dir / "wls_summary.json", name="wls_fallback/wls_summary.json"),
        "fallback_reason": reason,
        "wls_authorization_sha256": _sha256(auth_path),
        "algorithm_parameter_hash": FGO_RECIPE_HASH,
        "algorithm_core_hash": FGO_CORE_HASH,
        "runtime": {"child_max_rss_kib": resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss},
    }
    _save_cache_manifest(cache_dir, manifest)
    manifest["position_map"] = position_map
    return manifest


def _write_route_manifest(route_root: Path, result: dict[str, Any], cache_key: str) -> dict[str, Any]:
    payload = dict(result)
    payload.pop("position_map", None)
    payload["cache_key"] = cache_key
    path = route_root / "route_manifest.json"
    _atomic_json(path, payload)
    return _artifact(path)


def _submission_bytes(sample: dict[str, Any], row_map: dict[tuple[str, int], tuple[float, float]]) -> tuple[bytes, dict[str, Any]]:
    buffer = io.StringIO(newline="")
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerow(SAMPLE_FIELDS)
    exact = 0
    sample_fallback = 0
    route_counts: dict[str, dict[str, int]] = {}
    for row in sample["rows"]:
        key = (row["trip_id"], row["timestamp"])
        route = row["trip_id"].split("/", 1)[0]
        stats = route_counts.setdefault(route, {"sample_rows": 0, "exact_native_fgo_or_wls": 0, "official_sample_coordinate_fallback": 0})
        stats["sample_rows"] += 1
        if key in row_map:
            latitude, longitude = row_map[key]
            exact += 1
            stats["exact_native_fgo_or_wls"] += 1
        else:
            latitude, longitude = row["latitude"], row["longitude"]
            sample_fallback += 1
            stats["official_sample_coordinate_fallback"] += 1
        if not all(math.isfinite(float(value)) for value in (latitude, longitude)):
            raise TestBatchError(f"submission coordinate is non-finite: {key!r}")
        writer.writerow((row["trip_id"], row["timestamp_text"], f"{latitude:.12f}", f"{longitude:.12f}"))
    extra = len(set(row_map) - {(row["trip_id"], row["timestamp"]) for row in sample["rows"]})
    return buffer.getvalue().encode("utf-8"), {
        "header": list(SAMPLE_FIELDS),
        "row_count": len(sample["rows"]),
        "exact_position_count": exact,
        "official_sample_coordinate_fallback_count": sample_fallback,
        "extra_position_count_dropped": extra,
        "nearest_matching": False,
        "remap": False,
        "interpolation": False,
        "key_synthesis": False,
        "official_sample_coordinates_used_only_for_missing_truth_free_keys": True,
        "route_counts": route_counts,
    }


def _verify_submission(path: Path, sample: dict[str, Any]) -> dict[str, Any]:
    seen: set[tuple[str, str]] = set()
    count = 0
    with path.open(encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if tuple(reader.fieldnames or ()) != SAMPLE_FIELDS:
            raise TestBatchError("published submission header differs")
        for expected, row in zip(sample["rows"], reader):
            count += 1
            if None in row:
                raise TestBatchError("published submission has extra fields")
            key = (row.get("tripId") or "", row.get("UnixTimeMillis") or "")
            expected_key = (expected["trip_id"], expected["timestamp_text"])
            if key != expected_key:
                raise TestBatchError(f"published submission key/order differs at row {count}")
            if key in seen:
                raise TestBatchError("published submission contains duplicate keys")
            seen.add(key)
            try:
                latitude = float(row.get("LatitudeDegrees") or "")
                longitude = float(row.get("LongitudeDegrees") or "")
            except ValueError as exc:
                raise TestBatchError("published submission coordinate is invalid") from exc
            if not all(math.isfinite(value) for value in (latitude, longitude)) or not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
                raise TestBatchError("published submission coordinate is invalid")
        if next(reader, None) is not None:
            raise TestBatchError("published submission contains extra rows")
    if count != len(sample["rows"]) or len(seen) != len(sample["rows"]):
        raise TestBatchError("published submission row/key count differs")
    return {"header_exact": True, "row_count": count, "duplicate_keys": 0, "missing_keys": 0, "extra_keys": 0, "nonfinite_coordinates": 0, "exact_sample_order": True}


def run_batch(
    archive_path: Path,
    inventory_path: Path,
    sample_path: Path,
    auth_path: Path,
    auth_manifest_path: Path,
    output_dir: Path,
    fgo_binary: Path,
    spp_binary: Path,
) -> dict[str, Any]:
    started = time.perf_counter()
    rss_before = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    auth, auth_manifest, inventory, archive_hash, inventory_hash, auth_hash = _verify_auth(
        auth_path, auth_manifest_path, archive_path, inventory_path, sample_path, fgo_binary, spp_binary
    )
    sample = _read_sample(sample_path)
    records = sorted(inventory["test"]["records"], key=lambda row: str(row["dataset_id"]))
    output_dir.mkdir(parents=True, exist_ok=True)
    cache_root = output_dir / "cache"
    cache_root.mkdir(parents=True, exist_ok=True)
    route_results: list[dict[str, Any]] = []
    failures: list[dict[str, Any]] = []
    row_map: dict[tuple[str, int], tuple[float, float]] = {}
    route_manifest_artifacts: list[dict[str, Any]] = []
    for record in records:
        dataset_id = str(record["dataset_id"])
        route_root = output_dir / "routes" / _safe_id(dataset_id)
        cache_key = _cache_key(record, archive_hash, inventory_hash, auth_hash, fgo_binary, spp_binary)
        cache_dir = cache_root / cache_key
        try:
            device_path = route_root / "inputs" / "device_gnss.csv"
            result: dict[str, Any]
            try:
                result = _run_native_route(record, route_root, cache_dir, archive_path, auth_path, auth_manifest_path, fgo_binary, spp_binary)
            except Exception as exc:  # noqa: BLE001 - WLS fallback is explicit and sealed
                reason = f"native_fgo_failure:{type(exc).__name__}:{exc}"
                device_meta = record["central_directory_files"]["device_gnss.csv"]
                nav_meta = record["central_directory_broadcast_nav"]
                _materialize_member(archive_path, _test_member(str(record["route"]), str(record["phone"]), "device_gnss.csv"), device_path, device_meta)
                _materialize_member(archive_path, _nav_member(str(record["route"])), route_root / "inputs" / "brdc.nav", nav_meta)
                result = _run_wls_fallback(record, route_root, cache_dir, device_path, auth_path, auth_manifest_path, reason)
            positions = result.pop("position_map")
            for timestamp, coordinate in positions.items():
                row_map[(dataset_id, timestamp)] = coordinate
            route_manifest_artifacts.append(_write_route_manifest(route_root, result, cache_key))
            result["cache_key"] = cache_key
            route_results.append(result)
        except Exception as exc:  # noqa: BLE001 - retain route-level failure and continue audit
            failures.append({"dataset_id": dataset_id, "route": record["route"], "error": str(exc), "truth_open_count": 0})
    if failures:
        run_manifest = {
            "schema_version": RUN_MANIFEST_SCHEMA,
            "status": "failed-truth-free-test-batch",
            "official_sample_verified": True,
            "archive": {"path": str(archive_path.relative_to(ROOT)), "sha256": archive_hash},
            "inventory": {"path": str(inventory_path.relative_to(ROOT)), "sha256": inventory_hash},
            "authorization": {"path": str(auth_path.relative_to(ROOT)), "sha256": auth_hash, "manifest_sha256": _sha256(auth_manifest_path)},
            "routes": {"inventory_count": len(records), "completed_count": len(route_results), "failed_count": len(failures), "failures": failures, "route_manifests": route_manifest_artifacts},
            "truth_policy": {"truth_open_count": 0, "truth_materialized_count": 0, "ground_truth_members_read": False},
            "production_policy": {"production_rtk_spp_default_changed": False, "kaggle_external_submission": False},
            "no_post_test_tuning": True,
            "timing": {"total_wall_seconds": time.perf_counter() - started, "process_peak_rss_kib": max(rss_before, resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)},
        }
        _atomic_json(output_dir / "test_batch_run_manifest.json", run_manifest)
        raise TestBatchError(f"{len(failures)} test route(s) failed; no submission was published")
    output_bytes, contract = _submission_bytes(sample, row_map)
    submission_path = output_dir / "submission.csv"
    _atomic_bytes(submission_path, output_bytes)
    verification = _verify_submission(submission_path, sample)
    submission_manifest = {
        "schema_version": "smartphone-r5-gsdc2023-native-fgo-test-submission-manifest.v1",
        "status": "completed-truth-free-official-sample-key-order",
        "truth_free": True,
        "truth_used": False,
        "official_sample_verified": True,
        "official_sample": sample["artifact"],
        "submission": _artifact(submission_path),
        "verification": verification,
        "contract": contract,
        "truth_open_count": 0,
        "external_kaggle_submission": False,
    }
    _atomic_json(output_dir / "submission.manifest.json", submission_manifest)
    run_manifest = {
        "schema_version": RUN_MANIFEST_SCHEMA,
        "status": "completed-truth-free-test-batch",
        "official_sample_verified": True,
        "cannot_submit_without_external_approval": True,
        "archive": {"path": str(archive_path.relative_to(ROOT)), "sha256": archive_hash},
        "inventory": {"path": str(inventory_path.relative_to(ROOT)), "sha256": inventory_hash},
        "sample_submission": sample["artifact"],
        "authorization": {"path": str(auth_path.relative_to(ROOT)), "sha256": auth_hash, "manifest_sha256": _sha256(auth_manifest_path)},
        "algorithm_parameter_hash": FGO_RECIPE_HASH,
        "algorithm_core_hash": FGO_CORE_HASH,
        "routes": {"inventory_count": len(records), "completed_count": len(route_results), "failed_count": 0, "native_fgo_count": sum(1 for row in route_results if row.get("lane") == "native_fgo"), "wls_fallback_count": sum(1 for row in route_results if row.get("lane") == "wls_fallback"), "route_manifests": route_manifest_artifacts},
        "submission": {"artifact": _artifact(submission_path), "manifest": _artifact(output_dir / "submission.manifest.json"), "verification": verification, "contract": contract},
        "truth_policy": {"truth_open_count": 0, "truth_materialized_count": 0, "ground_truth_members_read": False},
        "production_policy": {"production_rtk_spp_default_changed": False, "kaggle_external_submission": False},
        "no_post_test_tuning": True,
        "timing": {"total_wall_seconds": time.perf_counter() - started, "process_peak_rss_kib": max(rss_before, resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)},
    }
    _atomic_json(output_dir / "test_batch_run_manifest.json", run_manifest)
    return run_manifest


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="gnss_smartphone_native_fgo_test_batch")
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--inventory", type=Path, default=DEFAULT_INVENTORY)
    parser.add_argument("--sample-submission", type=Path, default=DEFAULT_SAMPLE)
    parser.add_argument("--authorization", type=Path, default=DEFAULT_AUTH)
    parser.add_argument("--authorization-manifest", type=Path, default=DEFAULT_AUTH_MANIFEST)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--fgo-binary", type=Path, default=DEFAULT_FGO_BINARY)
    parser.add_argument("--spp-binary", type=Path, default=DEFAULT_SPP_BINARY)
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        report = run_batch(
            _resolve(args.archive), _resolve(args.inventory), _resolve(args.sample_submission),
            _resolve(args.authorization), _resolve(args.authorization_manifest), _resolve(args.output_dir),
            _resolve(args.fgo_binary), _resolve(args.spp_binary),
        )
    except (TestBatchError, OSError, ValueError, KeyError, TypeError, zipfile.BadZipFile) as exc:
        print(f"native FGO test batch failed: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"status": report["status"], "routes": report["routes"], "submission": report["submission"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
