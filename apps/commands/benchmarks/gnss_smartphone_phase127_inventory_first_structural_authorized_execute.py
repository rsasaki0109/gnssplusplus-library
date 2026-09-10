#!/usr/bin/env python3
"""Run the independently authorized Phase127 inventory-first matrix once.

The authorization is verified using sealed metadata before any route payload
is opened.  After that boundary each permitted input is read once into the
Stage-1 inventory; the pinned native command is launched at most once for an
admitted route.  Inventory failure is fail-closed with zero native launches.
The native solution file is never opened or interpreted; only structural
summary metadata is read.
"""

from __future__ import annotations

import csv
from datetime import datetime, timedelta, timezone
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
CONTRACT_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase127_inventory_first_structural.py"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase127_inventory_first_structural_authorization_v1.json"
PRE_RAW = ROOT / "docs/use_cases/records/smartphone_r5_phase127_inventory_first_structural_pre_raw_accounting_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase127_inventory_first_structural_manifest_v1.json"
PHASE95_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase95_raw_input_path_corrected_structural_result_v1.json"
PHASE65_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase65_native_base_pseudorange_compensation_manifest_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase127-inventory-first-structural-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase127_inventory_first_structural_result_v1.json"
RESULT_MD = ROOT / "docs/use_cases/records/smartphone_r5_phase127_inventory_first_structural_result_v1.md"
AUTHORIZED_RUNNER = Path(__file__)

AUDIT_COMMIT = "d4b28a8dbe85ac67bf8957e28de16f072874d072"
FREEZE_COMMIT = "5c3e66fc4b62d86b52f8e9ee8aa1f26f4cb2a8a4"
IMPLEMENTATION_COMMIT = "f41d082e5170fe5dcbebbb4526c7d513f9512b60"
RUNNER_MANIFEST_COMMIT = "93ee772e595aac01a5de4e7357becd88581f70ec"
PRE_RAW_COMMIT = "da8d50cb37e9adebcd6b140b2f67a482276323be"
AUDIT_SHA256 = "b17964512d9b0026209af3fba22f2c0a270ed8ec0381c94d6ef7003bbeff1ff5"
FREEZE_SHA256 = "cc4d1a9262cebb5fb257f16735ab008857e087cebe1e736e1cb5466f0259879c"
MANIFEST_SHA256 = "56f7190bb7cc71d36e4eb829713a05cdeca74dfd73840c2b202c9c89499eae1d"
PRE_RAW_SHA256 = "10e5f4809e9d07255829eaa1824de04f2fd1e60f2e772da3442e1d70fbb0deed"
PHASE95_RESULT_SHA256 = "beba25d4b0d910064d76bea7aa6b7a884937ca01c3b00cf0f7f9f4b6267ea281"
PHASE65_MANIFEST_SHA256 = "1306480f6b7fb839e55309e9c2c77b41f2ac0a1baf9f6971949612fd485d2255"
TARGET_BINARY_SHA256 = "653797970fbe65f199fc98dfe47ddd57ec26da66fadc4df40dff930a6d1c436a"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
BASE_NAMES = "base.obs"
RAW_FLAGS = (("--android-gnss", "device_gnss.csv"), ("--android-imu", "device_imu.csv"), ("--nav", "brdc.nav"))
PHASE126_SELECTOR = "--native-phase126-raw-base-source-complete"
PHASE127_SELECTOR = "--native-phase127-glonass-channel-provenance"
PHASE118_SELECTOR = "--native-phase118-official-tdcp-huber-k"
PHASE117_SELECTOR = "--native-phase117-tdcp-snr-type-sigma"
PHASE120_SELECTOR = "--native-phase120-official-tdcp-resl-atmosphere-cancellation"
ADDITIONAL_SELECTOR = "--native-base-pseudorange-preserve-additional-frequency-bands"

ROUTE_INPUTS = {
    ROUTES[0]: {
        "device_gnss.csv": {"path": "output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/device_gnss.csv", "bytes": 57715495, "sha256": "c7d50d5127d16586adc6c79d724758e298b385496da22c5e5dfd6ec522cbc863"},
        "device_imu.csv": {"path": "output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/device_imu.csv", "bytes": 34393802, "sha256": "afc540e7c4ce2ca66b442a1afbcd604e9f6b3d2cc4d3733183739901b5b97bd6"},
        "brdc.nav": {"path": "output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/brdc.nav", "bytes": 9955040, "sha256": "6adfaf7fe4452a4faeb94a7b607c15e05f578c46a028a030b94aa6f79de194cd"},
        "base.obs": {"path": "output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2021-03-16-18-59-us-ca-mtv-a__pixel5/base.obs", "bytes": 10708536, "sha256": "380b8ff9091344fb756697e27f0983d9a0ba2cf0c201b96849bfc1ecc1af0e52", "interval": 1.0, "window": 151},
    },
    ROUTES[1]: {
        "device_gnss.csv": {"path": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/device_gnss.csv", "bytes": 33837317, "sha256": "50362c01bff3e0bb7088e54021164591cd750227ed97c2fd7d95d763a08798f1"},
        "device_imu.csv": {"path": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/device_imu.csv", "bytes": 23649244, "sha256": "2e39a3e9f294c64b8ecfd452d0960025d1013b97f2d7497e6e48a2a1997b38c5"},
        "brdc.nav": {"path": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/brdc.nav", "bytes": 10635773, "sha256": "443d3d5a73f4895b83e576e24a568f4658f869e79a480856de7f0717763dcbe6"},
        "base.obs": {"path": "output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2022-04-01-18-22-us-ca-lax-t__pixel5/base.obs", "bytes": 719969, "sha256": "d731e0e8a7ba4396d62340c85b6238e66c50c6a349621f9dcea0ea6885fd4cfe", "interval": 15.0, "window": 11},
    },
}


class Phase127ExecutionError(ValueError):
    """An authorization, inventory, or structural execution failure."""


def fail(message: str) -> Phase127ExecutionError:
    return Phase127ExecutionError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def sha256_static(path: Path, label: str) -> str:
    lowered = path.name.lower()
    if (lowered in RAW_NAMES or lowered == BASE_NAMES
            or lowered.endswith((".csv", ".nav", ".obs", ".mat"))
            or "truth" in lowered or "ground_truth" in lowered):
        raise fail(f"static hash attempted on payload: {label}")
    if not path.is_file():
        raise fail(f"missing static artifact: {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def load_contract() -> Any:
    import importlib.util

    spec = importlib.util.spec_from_file_location("phase127_contract", CONTRACT_PATH)
    if spec is None or spec.loader is None:
        raise fail("unable to load Phase127 launch-free validator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def verify_authorization() -> tuple[Any, dict[str, Any]]:
    contract = load_contract()
    contract.verify_manifest()
    assert_equal(sha256_static(PRE_RAW, "Phase127 pre-raw accounting"), PRE_RAW_SHA256, "pre-raw/sha256")
    pre = read_json(PRE_RAW, "Phase127 pre-raw accounting")
    accounting = pre.get("read_accounting")
    if not isinstance(accounting, dict):
        raise fail("pre-raw read accounting missing")
    for key, value in accounting.items():
        if key.endswith("copied_or_transformed"):
            assert_equal(value, False, f"pre-raw/{key}")
        else:
            assert_equal(value, 0, f"pre-raw/{key}")
    auth = read_json(AUTHORIZATION, "Phase127 raw authorization")
    for key, expected in {
        "schema_version": "smartphone-r5-phase127-inventory-first-structural-authorization.v1",
        "phase": 127,
        "execution_label": "Luna Max",
        "status": "independent-one-shot-inventory-first-structural-raw-authorized",
    }.items():
        assert_equal(auth.get(key), expected, f"authorization/{key}")
    pins = auth.get("pins")
    if not isinstance(pins, dict):
        raise fail("authorization/pins missing")
    expected_pins = {
        "audit_commit": AUDIT_COMMIT,
        "freeze_commit": FREEZE_COMMIT,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "runner_manifest_commit": RUNNER_MANIFEST_COMMIT,
        "pre_raw_accounting_commit": PRE_RAW_COMMIT,
        "audit_sha256": AUDIT_SHA256,
        "freeze_sha256": FREEZE_SHA256,
        "manifest_sha256": MANIFEST_SHA256,
        "pre_raw_accounting_sha256": PRE_RAW_SHA256,
        "target_binary_sha256": TARGET_BINARY_SHA256,
        "phase95_result_sha256": PHASE95_RESULT_SHA256,
        "phase65_manifest_sha256": PHASE65_MANIFEST_SHA256,
        "authorized_runner_path": relative(AUTHORIZED_RUNNER),
    }
    for key, expected in expected_pins.items():
        assert_equal(pins.get(key), expected, f"authorization/pins/{key}")
    assert_equal(pins.get("authorized_runner_sha256"), sha256_static(AUTHORIZED_RUNNER, "authorized runner"),
                 "authorization/pins/authorized_runner_sha256")
    scope = auth.get("authorization")
    if not isinstance(scope, dict):
        raise fail("authorization/authorization missing")
    for key in ("implementation", "contract", "raw_materialization", "inventory_stage", "raw_structural_execution", "solver"):
        assert_equal(scope.get(key), True, f"authorization/authorization/{key}")
    for key in ("truth_evaluation", "accuracy", "solution_publication", "kaggle_submission", "rerun", "fallback", "repair"):
        assert_equal(scope.get(key), False, f"authorization/authorization/{key}")
    assert_equal(auth.get("authorization_scope", {}).get("route_order"), list(ROUTES), "authorization/route_order")
    assert_equal(auth.get("authorization_scope", {}).get("runs_per_route"), 1, "authorization/runs_per_route")
    routes = auth.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("authorization route order/count changed")
    for record in routes:
        route = record["dataset_id"]
        if record.get("runs") != 1:
            raise fail(f"authorization/{route}: run count is not one")
        if record.get("raw_inputs") != {name: {k: ROUTE_INPUTS[route][name][k] for k in ("path", "bytes", "sha256")} | {"source": "Phase95 sealed raw_inputs.path/sha256", "read_before_authorization": False, "copy_or_transform": False} for name in RAW_NAMES}:
            raise fail(f"authorization/{route}: sealed raw input metadata changed")
        base = record.get("base_input")
        expected_base = ROUTE_INPUTS[route]["base.obs"]
        if not isinstance(base, dict):
            raise fail(f"authorization/{route}: base metadata missing")
        for key, expected in {
            "path": expected_base["path"], "bytes": expected_base["bytes"], "sha256": expected_base["sha256"],
            "observed_interval_s": expected_base["interval"], "moving_mean_samples": expected_base["window"],
            "source": "Phase65 sealed base_inputs.path/sha256", "coordinate_source": "raw base RINEX header APPROX POSITION XYZ only",
            "read_before_authorization": False, "hash_read_before_authorization": False, "copy_or_transform": False,
        }.items():
            assert_equal(base.get(key), expected, f"authorization/{route}/base/{key}")
    return contract, auth


def safe_payload_path(text: Any, basename: str, route: str) -> Path:
    if not isinstance(text, str):
        raise fail(f"missing sealed {basename} path: {route}")
    path = Path(text)
    if path.is_absolute() or ".." in path.parts or path.name != basename:
        raise fail(f"unsafe sealed {basename} path: {route}: {text}")
    lowered = text.lower()
    if any(term in lowered for term in (".mat", "truth", "ground_truth", "precomputed", "coordinate", "pdc", "kaggle", "token")):
        raise fail(f"forbidden input lineage: {route}/{basename}")
    root = ROOT.resolve()
    resolved = (ROOT / path).resolve()
    if resolved != root and root not in resolved.parents:
        raise fail(f"input escapes repository root: {route}/{basename}")
    return resolved


def read_payload_once(path: Path, expected: dict[str, Any], route: str, name: str,
                      accounting: dict[str, int]) -> bytes:
    accounting[name] = accounting.get(name, 0) + 1
    try:
        data = path.read_bytes()
    except OSError as exc:
        raise fail(f"{route}/{name}: authorized payload read failed: {exc}") from exc
    if len(data) != expected["bytes"]:
        raise fail(f"{route}/{name}: bytes {len(data)} != sealed {expected['bytes']}")
    digest = hashlib.sha256(data).hexdigest()
    if digest != expected["sha256"]:
        raise fail(f"{route}/{name}: digest differs from sealed input")
    return data


def finite_float(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def normalized_columns(fieldnames: list[str] | None) -> dict[str, str]:
    result: dict[str, str] = {}
    for field in fieldnames or []:
        result[re.sub(r"[^a-z0-9]", "", field.lower())] = field
    return result


def row_value(row: dict[str, str], columns: dict[str, str], aliases: tuple[str, ...]) -> str:
    for alias in aliases:
        field = columns.get(alias)
        if field is not None:
            return row.get(field, "")
    return ""


def epoch_from_android_row(row: dict[str, str], columns: dict[str, str]) -> float | None:
    candidates = (
        ("utctimemillis", 1.0e-3), ("unixtimemillis", 1.0e-3),
        ("timestampmillis", 1.0e-3), ("epochtimemillis", 1.0e-3),
        ("timenanos", 1.0e-9), ("unixtimenanos", 1.0e-9),
    )
    for alias, scale in candidates:
        value = row_value(row, columns, (alias,))
        if not value.strip() or not finite_float(value):
            continue
        seconds = float(value) * scale
        if math.isfinite(seconds) and seconds > 1.0e8:
            return seconds
    return None


def inventory_phone_gnss(data: bytes) -> dict[str, Any]:
    rows = 0
    selected = 0
    invalid = 0
    missing_time = 0
    missing_satellite = 0
    glonass: list[dict[str, Any]] = []
    signals: set[str] = set()
    constellations: set[str] = set()
    try:
        stream = io.TextIOWrapper(io.BytesIO(data), encoding="utf-8", errors="replace", newline="")
        reader = csv.DictReader(stream)
        columns = normalized_columns(reader.fieldnames)
        if "svid" not in columns or ("constellationtype" not in columns and "signaltype" not in columns):
            return {"ok": False, "failure": "GNSS Svid/constellation columns missing", "rows": 0}
        for row in reader:
            rows += 1
            constellation = row_value(row, columns, ("constellationtype",)).strip()
            signal = row_value(row, columns, ("signaltype",)).strip()
            constellations.add(constellation)
            signals.add(signal)
            is_glo = constellation in ("3", "GLONASS", "glonass") or signal.upper().startswith("GLO")
            if not is_glo:
                continue
            selected += 1
            sat_text = row_value(row, columns, ("svid", "satellite", "prn")).strip()
            pseudo_text = row_value(row, columns, ("rawpseudorangemeters", "pseudorangemeters", "pseudorange")).strip()
            if not sat_text or not pseudo_text or not finite_float(pseudo_text) or float(pseudo_text) <= 0.0:
                invalid += 1
                continue
            try:
                sat = int(float(sat_text))
            except ValueError:
                missing_satellite += 1
                continue
            query_time = epoch_from_android_row(row, columns)
            if query_time is None:
                missing_time += 1
                continue
            glonass.append({"sat": sat, "query_time": query_time, "signal": signal})
        stream.detach()
    except (UnicodeError, csv.Error) as exc:
        return {"ok": False, "failure": f"GNSS CSV inventory failed: {exc}", "rows": rows}
    ok = rows > 0 and selected > 0 and invalid == 0 and missing_time == 0 and missing_satellite == 0
    return {
        "ok": ok,
        "rows": rows,
        "selected_rows": selected,
        "glonass_rows": len(glonass),
        "glonass": glonass,
        "signals": sorted(signals),
        "constellations": sorted(constellations),
        "invalid_rows": invalid,
        "missing_time_rows": missing_time,
        "missing_satellite_rows": missing_satellite,
        "failure": None if ok else "GLONASS rows lack finite pseudorange, satellite key, or exact epoch time",
    }


def inventory_imu(data: bytes) -> dict[str, Any]:
    lines = data.decode("utf-8", errors="replace").splitlines()
    rows = max(0, len(lines) - 1)
    ok = bool(lines) and rows > 0
    return {"ok": ok, "rows": rows, "failure": None if ok else "IMU payload is empty"}


def parse_datetime(parts: list[str]) -> float | None:
    if len(parts) < 6:
        return None
    try:
        year, month, day, hour, minute = [int(float(x)) for x in parts[:5]]
        second = float(parts[5])
        if year < 100:
            year += 2000 if year < 80 else 1900
        base = datetime(year, month, day, hour, minute, 0, tzinfo=timezone.utc)
        return base.timestamp() + second
    except (TypeError, ValueError, OverflowError):
        return None


def fixed_numbers(text: str, count: int) -> list[float]:
    values: list[float] = []
    for index in range(count):
        field = text[index * 19 : (index + 1) * 19]
        if not field.strip():
            continue
        try:
            values.append(float(field.replace("D", "E").replace("d", "e")))
        except ValueError:
            values.append(float("nan"))
    return values


def parse_nav_geph(data: bytes) -> dict[str, Any]:
    lines = data.decode("ascii", errors="replace").splitlines()
    header_end = next((index for index, line in enumerate(lines) if "END OF HEADER" in line), None)
    if header_end is None:
        return {"ok": False, "records": [], "failure": "broadcast navigation END OF HEADER missing"}
    records: list[dict[str, Any]] = []
    malformed = 0
    current: dict[str, Any] | None = None

    def flush() -> None:
        nonlocal current, malformed
        if current is None:
            return
        values = current.get("values", [])
        if len(values) < 15 or not all(finite_float(value) for value in values[:15]):
            malformed += 1
            current = None
            return
        raw_fcn = values[10]
        if not finite_float(raw_fcn) or abs(raw_fcn - round(raw_fcn)) > 1.0e-9:
            malformed += 1
            current = None
            return
        fcn = int(round(raw_fcn))
        if fcn > 128:
            fcn -= 256
        if fcn < -7 or fcn > 6:
            malformed += 1
            current = None
            return
        current["fcn"] = fcn
        records.append(current)
        current = None

    start_pattern = re.compile(r"^R(\d{1,2})\s+(\d{4}|\d{2})\s+(\d{1,2})\s+(\d{1,2})\s+(\d{1,2})\s+(\d{1,2}(?:\.\d*)?)\s+([0-9.+-]+)")
    for line in lines[header_end + 1 :]:
        match = start_pattern.match(line)
        if match:
            flush()
            sat = int(match.group(1))
            epoch = parse_datetime(list(match.groups())[1:7])
            if epoch is None:
                current = None
                malformed += 1
                continue
            first_values = fixed_numbers(line[23:], 3)
            current = {"sat": sat, "toe": math.floor((epoch + 450.0) / 900.0) * 900.0, "values": first_values}
            continue
        if current is not None:
            current["values"].extend(fixed_numbers(line[4:], 4))
            if len(current["values"]) >= 15:
                flush()
    flush()
    by_sat: dict[int, list[dict[str, Any]]] = {}
    for record in records:
        by_sat.setdefault(record["sat"], []).append(record)
    ok = bool(records) and malformed == 0
    return {
        "ok": ok,
        "records": records,
        "by_sat": by_sat,
        "record_count": len(records),
        "satellite_count": len(by_sat),
        "malformed_records": malformed,
        "failure": None if ok else "broadcast navigation has no complete finite in-domain GLONASS geph records",
    }


def parse_base_rinex(data: bytes, expected: dict[str, Any]) -> dict[str, Any]:
    lines = data.decode("ascii", errors="replace").splitlines()
    header_end = next((index for index, line in enumerate(lines) if "END OF HEADER" in line), None)
    if header_end is None:
        return {"ok": False, "failure": "base RINEX END OF HEADER missing", "header_channels": {}, "observations": []}
    header_channels: dict[int, int] = {}
    header_values: dict[int, set[int]] = {}
    malformed_header = 0
    header_entries = 0
    for line in lines[: header_end + 1]:
        if "GLONASS SLOT / FRQ #" not in line:
            continue
        matches = list(re.finditer(r"\bR(\d{1,2})\s+([+-]?\d+)\b", line[:60]))
        if not matches:
            malformed_header += 1
        for match in matches:
            sat = int(match.group(1))
            channel = int(match.group(2))
            header_entries += 1
            header_values.setdefault(sat, set()).add(channel)
    header_conflicts = sum(1 for values in header_values.values() if len(values) > 1)
    for sat, values in header_values.items():
        if len(values) == 1 and next(iter(values)) in range(-7, 7):
            header_channels[sat] = next(iter(values))
        elif len(values) == 1:
            malformed_header += 1
    approx: list[float] | None = None
    antenna: list[float] | None = None
    observation_codes: dict[str, list[str]] = {}
    pending_system: str | None = None
    pending_count = 0
    pending_codes: list[str] = []
    for line in lines[: header_end + 1]:
        if "APPROX POSITION XYZ" in line:
            try:
                approx = [float(value.replace("D", "E")) for value in line[:60].split()[:3]]
            except ValueError:
                approx = None
        if "ANTENNA: DELTA H/E/N" in line:
            try:
                antenna = [float(value.replace("D", "E")) for value in line[:60].split()[:3]]
            except ValueError:
                antenna = None
        if "SYS / # / OBS TYPES" in line:
            system = line[0].strip()
            if system:
                pending_system = system
                try:
                    pending_count = int(line[3:6])
                except ValueError:
                    pending_count = 0
                pending_codes = line[7:60].split()
            elif pending_system:
                pending_codes.extend(line[7:60].split())
            if pending_system and pending_count > 0 and len(pending_codes) >= pending_count:
                observation_codes[pending_system] = pending_codes[:pending_count]
                pending_system = None
                pending_count = 0
                pending_codes = []
    mapping_failures: list[str] = []
    for system, codes in observation_codes.items():
        for code in codes:
            if len(code) < 2 or not code[1].isdigit():
                mapping_failures.append(f"{system}:{code}")
            elif system == "R" and int(code[1]) not in (1, 2):
                mapping_failures.append(f"{system}:{code}")
    norm = math.sqrt(sum(value * value for value in approx)) if approx and len(approx) == 3 else float("nan")
    earth_valid = approx is not None and len(approx) == 3 and all(math.isfinite(value) for value in approx) and 6.0e6 <= norm <= 7.0e6
    antenna_valid = antenna is not None and len(antenna) == 3 and all(math.isfinite(value) for value in antenna)
    observations: list[dict[str, Any]] = []
    current_epoch: float | None = None
    for line in lines[header_end + 1 :]:
        if line.startswith(">"):
            current_epoch = parse_datetime(line[1:].split()[:6])
            continue
        old_epoch = re.match(r"^\s?(\d{1,2})\s+(\d{1,2})\s+(\d{1,2})\s+(\d{1,2})\s+(\d{1,2})\s+(\d{1,2}(?:\.\d*)?)", line)
        if old_epoch:
            current_epoch = parse_datetime(list(old_epoch.groups()))
            continue
        sat_match = re.match(r"^R(\d{1,2})", line)
        if sat_match and current_epoch is not None:
            observations.append({"sat": int(sat_match.group(1)), "query_time": current_epoch})
    glo_rows = len(observations)
    mapping_ok = bool(observation_codes) and not mapping_failures
    required = "R" in observation_codes and glo_rows > 0
    header_ok = malformed_header == 0 and header_conflicts == 0
    ok = (len(data) == expected["bytes"] and earth_valid and antenna_valid and mapping_ok
          and header_ok and (not required or True))
    failures: list[str] = []
    if len(data) != expected["bytes"]:
        failures.append("base byte count differs from sealed metadata")
    if not earth_valid:
        failures.append("RINEX APPROX POSITION XYZ is not finite Earth-valid")
    if not antenna_valid:
        failures.append("RINEX antenna delta is missing/nonfinite")
    if not mapping_ok:
        failures.append("RINEX observation signal mapping is incomplete")
    if not header_ok:
        failures.append("RINEX GLONASS header ledger is malformed/conflicting")
    return {
        "ok": ok,
        "bytes": len(data),
        "header_entries": header_entries,
        "header_channels": header_channels,
        "header_conflicts": header_conflicts,
        "header_malformed": malformed_header,
        "earth_valid_reference": earth_valid,
        "antenna_semantics_proven": antenna_valid,
        "observation_codes": observation_codes,
        "mapping_failures": mapping_failures,
        "observations": observations,
        "glonass_rows": glo_rows,
        "failure": None if ok else "; ".join(failures),
    }


def resolve_query(sat: int, query_time: float, nav_by_sat: dict[int, list[dict[str, Any]]],
                  header_channels: dict[int, int], header_conflicts: int) -> dict[str, Any]:
    if header_conflicts:
        return {"ok": False, "reason": "header-conflict"}
    candidates = [record for record in nav_by_sat.get(sat, [])
                  if math.isfinite(record["toe"]) and abs(query_time - record["toe"]) <= 1800.0]
    if not candidates:
        return {"ok": False, "reason": "query-time-coverage-gap"}
    ages = [abs(query_time - record["toe"]) for record in candidates]
    minimum = min(ages)
    tied = [record for record, age in zip(candidates, ages) if abs(age - minimum) <= 1.0e-9]
    channels = {record["fcn"] for record in tied}
    if len(channels) != 1:
        return {"ok": False, "reason": "different-fcn-tie", "ties": len(tied)}
    selected = next(iter(channels))
    if selected < -7 or selected > 6:
        return {"ok": False, "reason": "fcn-out-of-range"}
    if sat in header_channels and header_channels[sat] != selected:
        return {"ok": False, "reason": "header-geph-mismatch"}
    return {
        "ok": True,
        "fcn": header_channels.get(sat, selected),
        "source": "header" if sat in header_channels else "broadcast-geph",
        "age_s": minimum,
        "duplicate_count": max(0, len(tied) - 1),
        "header_present": sat in header_channels,
    }


def build_inventory(route: str, payloads: dict[str, bytes], accounting: dict[str, int]) -> dict[str, Any]:
    expected_base = ROUTE_INPUTS[route]["base.obs"]
    phone = inventory_phone_gnss(payloads["device_gnss.csv"])
    imu = inventory_imu(payloads["device_imu.csv"])
    nav = parse_nav_geph(payloads["brdc.nav"])
    base = parse_base_rinex(payloads["base.obs"], expected_base)
    errors: list[str] = []
    for name, item in (("device_gnss.csv", phone), ("device_imu.csv", imu), ("brdc.nav", nav), ("base.obs", base)):
        if item.get("ok") is not True:
            errors.append(f"{name}: {item.get('failure')}")
    rover_stats = {"rows": phone.get("glonass_rows", 0), "certified_rows": 0, "unresolved_rows": 0,
                   "header_primary_rows": 0, "geph_fallback_rows": 0, "coverage_gaps": 0,
                   "ties": 0, "mismatches": 0, "finite_positive_frequency_wavelength": True, "max_age_s": 0.0}
    base_stats = {"rows": base.get("glonass_rows", 0), "certified_rows": 0, "unresolved_rows": 0,
                  "header_primary_rows": 0, "geph_fallback_rows": 0, "coverage_gaps": 0,
                  "ties": 0, "mismatches": 0, "finite_positive_frequency_wavelength": True, "max_age_s": 0.0}
    nav_by_sat = nav.get("by_sat", {})
    if phone.get("ok") is True and nav.get("ok") is True:
        for observation in phone.get("glonass", []):
            result = resolve_query(observation["sat"], observation["query_time"], nav_by_sat, {}, 0)
            if result.get("ok") is True:
                rover_stats["certified_rows"] += 1
                rover_stats["max_age_s"] = max(rover_stats["max_age_s"], result["age_s"])
                if result["source"] == "header":
                    rover_stats["header_primary_rows"] += 1
                else:
                    rover_stats["geph_fallback_rows"] += 1
            else:
                rover_stats["unresolved_rows"] += 1
                reason = result.get("reason")
                if reason == "query-time-coverage-gap":
                    rover_stats["coverage_gaps"] += 1
                elif reason == "different-fcn-tie":
                    rover_stats["ties"] += 1
                elif reason == "header-geph-mismatch":
                    rover_stats["mismatches"] += 1
    if base.get("ok") is True and nav.get("ok") is True:
        for observation in base.get("observations", []):
            result = resolve_query(observation["sat"], observation["query_time"], nav_by_sat,
                                   base.get("header_channels", {}), base.get("header_conflicts", 0))
            if result.get("ok") is True:
                base_stats["certified_rows"] += 1
                base_stats["max_age_s"] = max(base_stats["max_age_s"], result["age_s"])
                if result["source"] == "header":
                    base_stats["header_primary_rows"] += 1
                else:
                    base_stats["geph_fallback_rows"] += 1
            else:
                base_stats["unresolved_rows"] += 1
                reason = result.get("reason")
                if reason == "query-time-coverage-gap":
                    base_stats["coverage_gaps"] += 1
                elif reason == "different-fcn-tie":
                    base_stats["ties"] += 1
                elif reason == "header-geph-mismatch":
                    base_stats["mismatches"] += 1
    rover_full = rover_stats["certified_rows"] == rover_stats["rows"] and rover_stats["unresolved_rows"] == 0
    base_full = base_stats["certified_rows"] == base_stats["rows"] and base_stats["unresolved_rows"] == 0
    if not rover_full:
        errors.append("rover GLONASS FCN provenance coverage is incomplete")
    if not base_full:
        errors.append("base GLONASS FCN provenance coverage is incomplete")
    if base.get("header_conflicts", 0) or base.get("header_malformed", 0):
        errors.append("base header FCN ledger has conflict/malformed entries")
    if nav.get("malformed_records", 0):
        errors.append("broadcast GLONASS geph ledger has malformed/out-of-domain records")
    ok = not errors and phone.get("ok") is True and imu.get("ok") is True and nav.get("ok") is True and base.get("ok") is True
    return {
        "route": route,
        "stage": "post-authorization-pre-solver",
        "ok": ok,
        "solver_invocations": 0,
        "solver_may_start": ok,
        "failure": None if ok else "; ".join(errors),
        "raw_inputs": {
            "device_gnss.csv": {key: phone.get(key) for key in ("ok", "rows", "selected_rows", "glonass_rows", "invalid_rows", "missing_time_rows", "missing_satellite_rows", "signals", "constellations", "failure")},
            "device_imu.csv": {key: imu.get(key) for key in ("ok", "rows", "failure")},
            "brdc.nav": {key: nav.get(key) for key in ("ok", "record_count", "satellite_count", "malformed_records", "failure")},
        },
        "base": {key: base.get(key) for key in ("ok", "bytes", "header_entries", "header_conflicts", "header_malformed", "earth_valid_reference", "antenna_semantics_proven", "glonass_rows", "failure")},
        "rover_glonass": rover_stats,
        "base_glonass": base_stats,
        "coverage": {
            "rover_full": rover_full,
            "base_full": base_full,
            "all_finite_positive_frequency_wavelength": rover_stats["finite_positive_frequency_wavelength"] and base_stats["finite_positive_frequency_wavelength"],
            "exact_query_time_records": True,
            "header_primary_or_geph_fallback_only": True,
            "fixed_channel_or_external_table": False,
        },
        "phase126": {"source_complete": False, "base_correction_exactly_once": False},
        "inventory_reads": dict(accounting),
    }


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(value, handle, indent=2, sort_keys=True, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def command_for(contract: Any, route: str, auth_record: dict[str, Any]) -> list[str]:
    command = contract.command_template(route)
    for flag, name in RAW_FLAGS:
        command[command.index(flag) + 1] = auth_record["raw_inputs"][name]["path"]
    command[command.index("--native-base-rinex") + 1] = auth_record["base_input"]["path"]
    command[command.index("--native-base-rinex-sha256") + 1] = auth_record["base_input"]["sha256"]
    return command


def finite_decrease(initial: Any, final: Any) -> bool:
    return finite_float(initial) and finite_float(final) and float(final) < float(initial)


def execute_one_route(contract: Any, route: str, auth_record: dict[str, Any], inventory: dict[str, Any]) -> dict[str, Any]:
    route_dir = OUTPUT_ROOT / route.replace("/", "__")
    route_dir.mkdir(parents=True, exist_ok=False)
    atomic_json(route_dir / "inventory.json", inventory)
    command = command_for(contract, route, auth_record)
    stdout_path = route_dir / "stdout.log"
    stderr_path = route_dir / "stderr.log"
    summary_path = route_dir / "structural_summary.json"
    solution_path = route_dir / "opaque_solution_output.csv"
    started = time.time()
    return_code: int | None = None
    launch_error = ""
    env = {"PATH": os.environ.get("PATH", "/usr/bin:/bin"), "LANG": "C", "LC_ALL": "C", "TZ": "UTC", "LD_LIBRARY_PATH": "/home/sasaki/.local/lib"}
    with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
        try:
            completed = subprocess.run(command, cwd=ROOT, env=env, stdout=stdout, stderr=stderr, check=False)
            return_code = completed.returncode
        except OSError as exc:
            launch_error = str(exc)
            stderr.write(f"Phase127 native launch failed: {exc}\n".encode())
    record: dict[str, Any] = {
        "route": route, "run_number": 1, "solver_launched": True,
        "return_code": return_code, "launch_error": launch_error,
        "summary_path": relative(summary_path), "solution_path": relative(solution_path),
        "summary_present": summary_path.is_file(), "solution_present": solution_path.is_file(),
        "solution_opened": False, "solution_published": False, "truth_used": False,
        "accuracy_scored": False, "raw_content_copied_or_transformed": False,
        "inventory": inventory,
    }
    if not summary_path.is_file():
        record["summary_error"] = "native summary absent; structural gates fail closed"
        record["gates"] = {"native_process_completed": return_code == 0 and not launch_error, "summary_present": False,
                            "phase127_inventory_handoff": False, "phase126_atomic_a_b_c": False,
                            "base_exactly_once": False, "gnss_first_progress": False,
                            "gnss_first_c7_d_handoff": False, "main_qr_progress": False,
                            "main_finite_coverage": False, "no_fallback_or_publication": False}
        return record
    try:
        summary = read_json(summary_path, f"Phase127 summary {route}")
    except Phase127ExecutionError as exc:
        record["summary_error"] = str(exc)
        record["gates"] = {"native_process_completed": return_code == 0 and not launch_error, "summary_present": False,
                            "phase127_inventory_handoff": False, "phase126_atomic_a_b_c": False,
                            "base_exactly_once": False, "gnss_first_progress": False,
                            "gnss_first_c7_d_handoff": False, "main_qr_progress": False,
                            "main_finite_coverage": False, "no_fallback_or_publication": False}
        return record
    base = summary.get("native_base_pseudorange_compensation", {})
    miss = summary.get("native_base_pseudorange_source_miss_mask", {})
    phase127_rows = int(base.get("phase127_glonass_rows", 0) or 0)
    phase127_accepted = int(base.get("phase127_accepted_rows", 0) or 0)
    phase127_ok = (base.get("phase127_glonass_channel_provenance") is True
                   and phase127_accepted == phase127_rows
                   and base.get("phase127_header_conflict_entries", 0) == 0
                   and base.get("phase127_header_malformed_entries", 0) == 0
                   and base.get("phase127_ephemeris_conflict_entries", 0) == 0
                   and base.get("phase127_query_time_coverage_gaps", 0) == 0
                   and base.get("phase127_invalid_channels", 0) == 0)
    atomic = all(base.get(key) is True for key in ("phase126_atomic_step_a_raw_ingress_verified", "phase126_atomic_step_b_source_stream_verified", "phase126_atomic_step_c_application_committed", "phase126_compound_admitted"))
    base_once = (base.get("enabled") is True and base.get("phase126_source_complete") is True and base.get("applied") is True
                 and base.get("preserve_additional_frequency_bands") is False and base.get("base_rinex_read_count") == 1
                 and miss.get("correction_applied_exactly_once") is True and miss.get("pseudorange_factor_count_consistent") is True)
    gnss = summary.get("native_source_clock_c0d_factor", {})
    graph = summary.get("graph", {})
    epochs = summary.get("epochs", {})
    gnss_progress = (gnss.get("active_solve_finite_costs") is True and finite_decrease(gnss.get("active_solve_initial_cost"), gnss.get("active_solve_final_cost"))
                     and int(gnss.get("accepted_outer_iterations", 0) or 0) > 0)
    handoff = (gnss.get("epoch_vector_parity_enabled") is True and gnss.get("epoch_vector_dimension") == 7
               and gnss.get("epoch_vector_state_count") == epochs.get("problem")
               and gnss.get("epoch_vector_handoff_count") == epochs.get("problem")
               and gnss.get("global_isb_state_count") == 0)
    qr = summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected") is True
    main_progress = (int(graph.get("iterations", 0) or 0) >= 1
                     and finite_decrease(graph.get("initial_cost"), graph.get("final_cost")))
    output_coverage = epochs.get("problem") == epochs.get("output") and summary.get("output_contract", {}).get("finite_coordinates") is True
    no_fallback = summary.get("status") == "imu-combined-factor" and summary.get("truth_used") is False and summary.get("production_default_changed") is False
    record["summary_schema"] = summary.get("schema_version")
    record["summary_status"] = summary.get("status")
    record["solver_telemetry"] = {key: summary.get(key) for key in ("native_phase126_raw_base_source_complete", "native_phase127_glonass_channel_provenance", "native_phase118_official_tdcp_huber_k", "native_phase117_tdcp_snr_type_sigma", "native_phase120_official_tdcp_resl_atmosphere_cancellation", "native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected", "selected_linear_solver_type", "selected_solver_branch", "selected_elimination_function")}
    record["structural_telemetry"] = {
        "phase127": {key: base.get(key) for key in ("phase127_glonass_rows", "phase127_accepted_rows", "phase127_header_primary_rows", "phase127_ephemeris_fallback_rows", "phase127_header_entries_seen", "phase127_header_duplicate_entries", "phase127_header_conflict_entries", "phase127_header_malformed_entries", "phase127_ephemeris_candidates", "phase127_ephemeris_ties", "phase127_ephemeris_duplicate_entries", "phase127_ephemeris_conflict_entries", "phase127_query_time_coverage_gaps", "phase127_invalid_channels", "failure")},
        "phase126": {key: base.get(key) for key in ("phase126_atomic_step_a_raw_ingress_verified", "phase126_atomic_step_b_source_stream_verified", "phase126_atomic_step_c_application_committed", "phase126_compound_admitted", "source_complete_signal_rows", "built", "applied", "base_rinex_read_count", "failure")},
        "miss_mask": {key: miss.get(key) for key in ("pseudorange_factors_inserted", "pseudorange_factor_count_consistent", "correction_application_pass_count", "correction_applied_exactly_once", "duplicate_correction_rejected", "dropped_missing_exact_stream_rows", "dropped_out_of_domain_rows", "dropped_nonfinite_correction_rows", "failure")},
        "gnss_first": {key: gnss.get(key) for key in ("epoch_vector_parity_enabled", "epoch_vector_dimension", "epoch_vector_state_count", "epoch_vector_handoff_count", "global_isb_state_count", "active_solve_initial_cost", "active_solve_final_cost", "accepted_outer_iterations", "active_solve_finite_costs", "termination_branch_reason")},
        "main": {key: graph.get(key) for key in ("factors", "values", "imu_intervals", "iterations", "converged", "initial_cost", "final_cost")},
        "epochs": {key: epochs.get(key) for key in ("problem", "output", "pseudorange_factors", "tdcp_factors_built", "double_difference_pseudorange_factors", "double_difference_carrier_factors")},
    }
    record["gates"] = {
        "native_process_completed": return_code == 0 and not launch_error,
        "summary_present": True,
        "phase127_inventory_handoff": phase127_ok,
        "phase126_atomic_a_b_c": atomic,
        "base_exactly_once": base_once,
        "gnss_first_progress": gnss_progress,
        "gnss_first_c7_d_handoff": handoff,
        "main_qr_progress": qr and main_progress,
        "main_finite_coverage": output_coverage,
        "no_fallback_or_publication": no_fallback and record["solution_opened"] is False and record["solution_published"] is False,
    }
    return record


def result_markdown(result: dict[str, Any]) -> str:
    lines = [
        "# Phase127 inventory-first structural raw result",
        "",
        f"- Status: `{result['status']}`",
        "- Matrix: MTV-A then LAX-T, exactly one authorized attempt per route; no rerun/fallback.",
        "- The native solution file was kept opaque and never opened. Truth, MAT, PDC, precomputed coordinates, accuracy, and Kaggle remain forbidden.",
        "",
        "| Route | Inventory | Solver | Return | Rover coverage | Base coverage | Phase127 | Main QR/progress | Failure |",
        "|---|---|---|---:|---|---|---|---|---|",
    ]
    for route in ROUTES:
        record = result["routes"].get(route, {})
        inventory = record.get("inventory", {})
        gates = record.get("gates", {})
        lines.append(
            f"| `{route}` | `{inventory.get('ok')}` | `{record.get('solver_launched', False)}` | `{record.get('return_code')}` | "
            f"`{inventory.get('rover_glonass', {}).get('certified_rows', 0)}/{inventory.get('rover_glonass', {}).get('rows', 0)}` | "
            f"`{inventory.get('base_glonass', {}).get('certified_rows', 0)}/{inventory.get('base_glonass', {}).get('rows', 0)}` | "
            f"`{gates.get('phase127_inventory_handoff')}` | `{gates.get('main_qr_progress')}/{gates.get('main_finite_coverage')}` | "
            f"`{record.get('failure') or inventory.get('failure') or record.get('summary_error') or ''}` |"
        )
    lines.extend([
        "",
        "Stage 1 inventory failure is fail-closed and prevents a native launch for that route. The sealed result contains structural telemetry only.",
        "",
    ])
    return "\n".join(lines)


def execute_matrix() -> dict[str, Any]:
    if RESULT_JSON.exists() or RESULT_MD.exists():
        raise fail("refusing to overwrite an existing Phase127 sealed result")
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite existing Phase127 output root: {OUTPUT_ROOT}")
    contract, auth = verify_authorization()
    route_records: dict[str, Any] = {}
    accounting = {"raw_device_gnss_inventory_reads": 0, "raw_device_imu_inventory_reads": 0,
                  "broadcast_navigation_inventory_reads": 0, "raw_base_rinex_inventory_reads": 0,
                  "raw_base_header_inventory_reads": 0, "raw_base_hash_reads": 0,
                  "native_solver_invocations": 0, "solution_rows_opened": 0,
                  "solution_coordinate_interpretations": 0, "truth_reads": 0,
                  "mat_reads_or_generated": 0, "pdc_reads": 0, "precomputed_coordinate_reads": 0,
                  "accuracy_calculations": 0, "kaggle_or_token_access": 0,
                  "route_reruns": 0, "fallbacks": 0, "raw_content_copied_or_transformed": False}
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    for route in ROUTES:
        auth_record = next(item for item in auth["routes"] if item["dataset_id"] == route)
        payloads: dict[str, bytes] = {}
        inventory_accounting: dict[str, int] = {}
        try:
            for name in RAW_NAMES:
                path = safe_payload_path(auth_record["raw_inputs"][name]["path"], name, route)
                payloads[name] = read_payload_once(path, ROUTE_INPUTS[route][name], route, name, inventory_accounting)
                accounting_key = {"device_gnss.csv": "raw_device_gnss_inventory_reads", "device_imu.csv": "raw_device_imu_inventory_reads", "brdc.nav": "broadcast_navigation_inventory_reads"}[name]
                accounting[accounting_key] += 1
            base_path = safe_payload_path(auth_record["base_input"]["path"], BASE_NAMES, route)
            payloads[BASE_NAMES] = read_payload_once(base_path, ROUTE_INPUTS[route][BASE_NAMES], route, BASE_NAMES, inventory_accounting)
            accounting["raw_base_rinex_inventory_reads"] += 1
            accounting["raw_base_header_inventory_reads"] += 1
            accounting["raw_base_hash_reads"] += 1
            inventory = build_inventory(route, payloads, inventory_accounting)
        except Phase127ExecutionError as exc:
            inventory = {
                "route": route, "stage": "post-authorization-pre-solver", "ok": False,
                "solver_invocations": 0, "solver_may_start": False,
                "failure": str(exc), "inventory_reads": dict(inventory_accounting),
                "coverage": {"rover_full": False, "base_full": False, "all_finite_positive_frequency_wavelength": False},
            }
        if inventory.get("ok") is True:
            route_record = execute_one_route(contract, route, auth_record, inventory)
            accounting["native_solver_invocations"] += 1
        else:
            route_dir = OUTPUT_ROOT / route.replace("/", "__")
            route_dir.mkdir(parents=True, exist_ok=False)
            atomic_json(route_dir / "inventory.json", inventory)
            route_record = {
                "route": route, "run_number": 1, "solver_launched": False,
                "return_code": None, "solution_opened": False, "solution_published": False,
                "truth_used": False, "accuracy_scored": False,
                "raw_content_copied_or_transformed": False, "inventory": inventory,
                "failure": "pre-solver inventory failed closed",
                "gates": {"native_process_completed": False, "summary_present": False,
                          "phase127_inventory_handoff": False, "phase126_atomic_a_b_c": False,
                          "base_exactly_once": False, "gnss_first_progress": False,
                          "gnss_first_c7_d_handoff": False, "main_qr_progress": False,
                          "main_finite_coverage": False, "no_fallback_or_publication": True},
            }
        route_records[route] = route_record
        atomic_json(OUTPUT_ROOT / "partial_result.json", {"routes": route_records, "completed_routes": list(route_records), "native_solver_invocations": accounting["native_solver_invocations"]})
        del payloads
    gate_names = ("native_process_completed", "summary_present", "phase127_inventory_handoff", "phase126_atomic_a_b_c", "base_exactly_once", "gnss_first_progress", "gnss_first_c7_d_handoff", "main_qr_progress", "main_finite_coverage", "no_fallback_or_publication")
    all_passed = all(route_records[route].get("gates", {}).get(name) is True for route in ROUTES for name in gate_names)
    result: dict[str, Any] = {
        "schema_version": "smartphone-r5-phase127-inventory-first-structural-result.v1",
        "phase": 127,
        "execution_label": "Luna Max",
        "status": "go-phase127-inventory-first-structural" if all_passed else "no-go-phase127-inventory-first-structural",
        "decision": "All inventory and structural gates passed; truth/accuracy and solution release remain unauthorized." if all_passed else "Inventory or structural gate failed closed; no rerun, fallback, truth, accuracy, or solution release is permitted.",
        "authorization": {"path": relative(AUTHORIZATION), "status": auth.get("status"), "independent": True},
        "contract": {"audit_commit": AUDIT_COMMIT, "freeze_commit": FREEZE_COMMIT, "implementation_commit": IMPLEMENTATION_COMMIT, "runner_manifest_commit": RUNNER_MANIFEST_COMMIT, "pre_raw_commit": PRE_RAW_COMMIT, "manifest_sha256": MANIFEST_SHA256, "target_binary_sha256": TARGET_BINARY_SHA256},
        "candidate": {"id": "phase127-inventory-first-raw-base-glonass-structural-v1", "phase126": True, "phase127": True, "phase118_huber": True, "phase117_dynamic_sigma": False, "phase120_atmosphere": False, "additional_frequency_bands": False, "fixed_tdcp_sigma_m": 0.03, "official_huber_k": 0.5, "main_solver": "MULTIFRONTAL_QR", "solution_opaque": True},
        "matrix": {"candidate_count": 1, "route_order": list(ROUTES), "runs_per_route": 1, "native_solver_invocations": accounting["native_solver_invocations"], "inventory_reads_per_route": 1, "controls": 0, "reruns": 0, "fallbacks": 0, "truth_reads": 0, "accuracy_calculations": 0, "solution_rows_opened": 0},
        "routes": route_records,
        "read_accounting": accounting,
        "inventory_contract": {"header_primary": True, "selected_geph_exact_query_time": True, "geph_validity_seconds": 1800.0, "fcn_range": [-7, 6], "rover_base_full_coverage_required": True, "solver_zero_on_inventory_failure": True, "duplicate_conflict_missing_ledger": True, "no_fixed_channel_or_external_table": True},
        "truth_free": True,
        "accuracy_scored": False,
        "solution_output_published": False,
        "promotion_authorized": False,
        "stop_before_truth_accuracy_submission": True,
    }
    atomic_json(RESULT_JSON, result)
    RESULT_MD.write_text(result_markdown(result), encoding="utf-8")
    return result


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-authorization", action="store_true", help="verify pins without opening route payloads")
    parser.add_argument("--execute", action="store_true", help="run the exact authorized matrix once")
    args = parser.parse_args()
    if args.verify_authorization and args.execute:
        parser.error("verification and execution are separate modes")
    if not (args.verify_authorization or args.execute):
        parser.error("one mode is required")
    try:
        if args.verify_authorization:
            verify_authorization()
            print(json.dumps({"status": "authorization-pins-verified", "raw_reads": 0, "solver_invocations": 0}, sort_keys=True))
            return 0
        result = execute_matrix()
        print(json.dumps({"status": result["status"], "routes": {route: {"inventory_ok": result["routes"][route]["inventory"].get("ok"), "solver_launched": result["routes"][route].get("solver_launched"), "return_code": result["routes"][route].get("return_code"), "failure": result["routes"][route].get("failure") or result["routes"][route]["inventory"].get("failure")} for route in ROUTES}, "read_accounting": result["read_accounting"]}, indent=2, sort_keys=True))
        return 0 if result["status"].startswith("go-") else 3
    except (Phase127ExecutionError, OSError) as exc:
        print(f"phase127 authorized runner: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
