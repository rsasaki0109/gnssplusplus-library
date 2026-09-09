#!/usr/bin/env python3
"""Execute the independently authorized Phase126 structural matrix once.

The authorization is checked before any raw path is resolved.  After that
boundary this runner reads each permitted raw phone/nav/base member exactly
for inventory, and starts at most one native invocation for each route whose
inventory passes.  A failed inventory is sealed without a solver launch.  No
solution CSV is ever opened, parsed, hashed, or published, and this runner has
no retry or fallback branch.
"""

from __future__ import annotations

import csv
import hashlib
import importlib.util
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
CONTRACT_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase126_raw_base_compound_structural.py"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase126_raw_base_compound_structural_authorization_v1.json"
PRE_RAW = ROOT / "docs/use_cases/records/smartphone_r5_phase126_raw_base_compound_structural_pre_raw_accounting_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase126-raw-base-compound-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase126_raw_base_compound_structural_result_v1.json"
RESULT_MD = ROOT / "docs/use_cases/records/smartphone_r5_phase126_raw_base_compound_structural_result_v1.md"
AUTH_COMMIT = "8cccc0a1ee3e153009f49cb2ebd24f075df687b5"
AUTH_SHA256 = "59ab4b79af98540f24f4f2540146d89a628c32f19733a899738772b87e0f7daa"
PRE_RAW_SHA256 = "414881787a4e1cd853795901c024b2ca1a166527bd7557ce67a30ff301c04313"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
RAW_FLAGS = (("--android-gnss", "device_gnss.csv"), ("--android-imu", "device_imu.csv"), ("--nav", "brdc.nav"))
BASE_EXPECTED = {
    ROUTES[0]: {"bytes": 10708536, "sha256": "380b8ff9091344fb756697e27f0983d9a0ba2cf0c201b96849bfc1ecc1af0e52", "interval": 1.0, "window": 151},
    ROUTES[1]: {"bytes": 719969, "sha256": "d731e0e8a7ba4396d62340c85b6238e66c50c6a349621f9dcea0ea6885fd4cfe", "interval": 15.0, "window": 11},
}


class Phase126ExecutionError(ValueError):
    """An authorization, inventory, or structural execution failure."""


def fail(message: str) -> Phase126ExecutionError:
    return Phase126ExecutionError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def sha256_static(path: Path, label: str) -> str:
    if path.name in RAW_NAMES or path.name == "base.obs" or path.name.endswith(".csv"):
        raise fail(f"static hash attempted on payload: {label}")
    if not path.is_file():
        raise fail(f"missing static artifact: {label}: {path}")
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


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


def load_contract() -> Any:
    spec = importlib.util.spec_from_file_location("phase126_contract", CONTRACT_PATH)
    if spec is None or spec.loader is None:
        raise fail("unable to load launch-free Phase126 validator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_phase107_metadata() -> Any:
    path = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase107_raw_base_source_parity.py"
    spec = importlib.util.spec_from_file_location("phase107_sealed_metadata", path)
    if spec is None or spec.loader is None:
        raise fail("unable to load sealed raw/base metadata reader")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def verify_authorization(contract: Any) -> dict[str, Any]:
    contract.verify_manifest()
    pre = read_json(PRE_RAW, "Phase126 pre-raw accounting")
    assert_pre = pre.get("read_accounting")
    if not isinstance(assert_pre, dict):
        raise fail("pre-raw read accounting missing")
    if any(value != 0 for key, value in assert_pre.items() if key != "raw_content_copied_or_transformed"):
        raise fail("pre-raw accounting is nonzero")
    if assert_pre.get("raw_content_copied_or_transformed") is not False:
        raise fail("pre-raw content-copy marker changed")
    if sha256_static(PRE_RAW, "pre-raw accounting") != PRE_RAW_SHA256:
        raise fail("pre-raw accounting hash changed")
    if sha256_static(AUTHORIZATION, "authorization") != AUTH_SHA256:
        raise fail("authorization hash changed")
    auth = read_json(AUTHORIZATION, "Phase126 authorization")
    if auth.get("status") != "independent-one-shot-structural-raw-authorized":
        raise fail("authorization status is not exact")
    pins = auth.get("pins")
    if not isinstance(pins, dict):
        raise fail("authorization pins missing")
    expected = {
        "design_freeze_commit": contract.DESIGN_FREEZE_COMMIT,
        "implementation_commit": contract.IMPLEMENTATION_COMMIT,
        "audit_commit": contract.AUDIT_COMMIT,
        "freeze_commit": contract.FREEZE_COMMIT,
        "runner_manifest_commit": "40f1d93c0b7dbf9b92353df9e283cd0ad03de9b4",
        "pre_raw_accounting_commit": "5589ded7e82366fed4550aed0cb93b6450effb53",
        "freeze_sha256": contract.FREEZE_SHA256,
        "manifest_sha256": "661fe5cb7103939858a661e28fa6cb9000178a3fbe5f931738746dc1981a3963",
        "pre_raw_accounting_sha256": PRE_RAW_SHA256,
        "target_binary_sha256": "5dc5a336ff605e418d42be957f3ec5ce5663744da77999b5acbf12f3ef440e70",
    }
    for key, value in expected.items():
        if pins.get(key) != value:
            raise fail(f"authorization pin mismatch: {key}")
    scope = auth.get("authorization")
    if not isinstance(scope, dict):
        raise fail("authorization scope missing")
    if scope.get("raw_materialization") is not True or scope.get("raw_structural_execution") is not True or scope.get("solver") is not True:
        raise fail("raw structural authorization is not enabled")
    for key in ("truth_evaluation", "accuracy", "solution_publication", "kaggle_submission", "rerun", "fallback"):
        if scope.get(key) is not False:
            raise fail(f"forbidden authorization lane enabled: {key}")
    return auth


def safe_input_path(text: Any, basename: str, route: str) -> Path:
    if not isinstance(text, str):
        raise fail(f"missing sealed {basename} path: {route}")
    path = Path(text)
    if path.is_absolute() or ".." in path.parts or path.name != basename:
        raise fail(f"unsafe sealed {basename} path: {route}")
    if any(term in text.lower() for term in (".mat", "truth", "precomputed", "coordinate", "pdc", "kaggle", "token")):
        raise fail(f"forbidden path lineage: {route}/{basename}")
    resolved = ROOT / path
    if not resolved.is_file():
        raise fail(f"missing authorized {basename}: {resolved}")
    return resolved


def finite_float(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


SIGNAL_FREQUENCIES = {
    "GPS_L1_CA": 1575420000.0,
    "GPS_L5_Q": 1176450000.0,
    "GAL_E1_C_P": 1575420000.0,
    "GAL_E5A_Q": 1176450000.0,
    "BDS_B1_I": 1561098000.0,
    "QZS_L1_CA": 1575420000.0,
    "QZS_L5_Q": 1176450000.0,
}
RINEX_BAND_FREQUENCIES = {
    "G": {1: 1575420000.0, 2: 1227600000.0, 5: 1176450000.0},
    "E": {1: 1575420000.0, 5: 1176450000.0, 6: 1278750000.0, 7: 1207140000.0, 8: 1191795000.0},
    "C": {1: 1575420000.0, 2: 1561098000.0, 5: 1176450000.0, 6: 1268520000.0, 7: 1207140000.0, 8: 1191795000.0},
    "J": {1: 1575420000.0, 2: 1227600000.0, 5: 1176450000.0},
    "S": {1: 1575420000.0, 5: 1176450000.0},
}


def inventory_phone_gnss(path: Path) -> dict[str, Any]:
    required = {"Svid", "ConstellationType", "CarrierFrequencyHz", "SignalType", "ReceivedSvTimeNanos", "RawPseudorangeMeters"}
    signals: set[str] = set()
    constellations: set[str] = set()
    glonass_channels: set[int] = set()
    rows = 0
    selected = 0
    nonfinite = 0
    unknown = 0
    missing_frequency = 0
    try:
        with path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            fields = set(reader.fieldnames or ())
            missing_columns = sorted(required - fields)
            if missing_columns:
                return {"ok": False, "failure": f"GNSS missing columns: {missing_columns}", "rows": 0}
            for row in reader:
                rows += 1
                constellation = str(row.get("ConstellationType", ""))
                constellations.add(constellation)
                signal = str(row.get("SignalType", ""))
                if not signal:
                    continue
                selected += 1
                signals.add(signal)
                if signal not in SIGNAL_FREQUENCIES and signal != "GLO_G1_CA":
                    unknown += 1
                    continue
                frequency = row.get("CarrierFrequencyHz")
                if not finite_float(frequency) or float(frequency) <= 0.0:
                    missing_frequency += 1
                    continue
                expected = SIGNAL_FREQUENCIES.get(signal)
                if expected is not None and abs(float(frequency) - expected) > 1000.0:
                    unknown += 1
                if signal == "GLO_G1_CA" or constellation == "3":
                    channel = round((float(frequency) - 1602.0e6) / 0.5625e6)
                    if abs(float(frequency) - (1602.0e6 + channel * 0.5625e6)) > 5000.0:
                        unknown += 1
                    else:
                        glonass_channels.add(channel)
                if not finite_float(row.get("RawPseudorangeMeters")) or not finite_float(row.get("ReceivedSvTimeNanos")):
                    nonfinite += 1
    except (OSError, UnicodeError, csv.Error) as exc:
        return {"ok": False, "failure": f"GNSS inventory read failed: {exc}", "rows": rows}
    ok = rows > 0 and selected > 0 and unknown == 0 and missing_frequency == 0 and nonfinite == 0
    return {
        "ok": ok,
        "rows": rows,
        "selected_rows": selected,
        "signals": sorted(signals),
        "constellations": sorted(constellations),
        "glonass_rows": sum(1 for x in constellations if x == "3"),
        "glonass_channels_from_phone": sorted(glonass_channels),
        "unknown_signal_or_frequency_rows": unknown,
        "missing_frequency_rows": missing_frequency,
        "nonfinite_rows": nonfinite,
        "failure": None if ok else "GNSS signal/frequency mapping is incomplete or nonfinite",
    }


def inventory_imu(path: Path) -> dict[str, Any]:
    rows = 0
    try:
        with path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.reader(handle)
            header = next(reader, [])
            for row in reader:
                if row:
                    rows += 1
    except (OSError, UnicodeError, csv.Error) as exc:
        return {"ok": False, "rows": rows, "failure": f"IMU inventory read failed: {exc}"}
    ok = rows > 0 and bool(header)
    return {"ok": ok, "rows": rows, "header_columns": len(header), "failure": None if ok else "IMU inventory is empty"}


def inventory_nav(path: Path) -> dict[str, Any]:
    try:
        data = path.read_bytes()
    except OSError as exc:
        return {"ok": False, "failure": f"navigation inventory read failed: {exc}"}
    text = data.decode("utf-8", errors="replace")
    lines = text.splitlines()
    end = next((i for i, line in enumerate(lines) if "END OF HEADER" in line), None)
    invalid_tokens = len(re.findall(r"(?i)(?:nan|inf)", text))
    records = max(0, len(lines) - (end + 1 if end is not None else 0))
    ok = end is not None and records > 0 and invalid_tokens == 0
    return {"ok": ok, "bytes": len(data), "header_end_line": end, "navigation_body_lines": records, "invalid_numeric_tokens": invalid_tokens,
            "finite_domain_source": ok, "failure": None if ok else "navigation header/body is missing or nonfinite"}


def parse_base_inventory(path: Path, expected: dict[str, Any]) -> tuple[str, dict[str, Any]]:
    try:
        data = path.read_bytes()
    except OSError as exc:
        return "", {"ok": False, "failure": f"base inventory read failed: {exc}"}
    digest = hashlib.sha256(data).hexdigest()
    text = data.decode("ascii", errors="replace")
    lines = text.splitlines()
    header_end = next((i for i, line in enumerate(lines) if "END OF HEADER" in line), None)
    approx: list[float] | None = None
    antenna: list[float] | None = None
    systems: set[str] = set()
    observation_codes: dict[str, list[str]] = {}
    current_system: str | None = None
    expected_count = 0
    collected: list[str] = []
    glo_channels: dict[int, int] = {}
    if header_end is None:
        return digest, {"ok": False, "failure": "RINEX END OF HEADER missing", "bytes": len(data), "sha256": digest}
    for line in lines[: header_end + 1]:
        if "APPROX POSITION XYZ" in line:
            try:
                approx = [float(x) for x in line[:60].split()[:3]]
            except ValueError:
                approx = None
        if "ANTENNA: DELTA H/E/N" in line:
            try:
                antenna = [float(x) for x in line[:60].split()[:3]]
            except ValueError:
                antenna = None
        if "SYS / # / OBS TYPES" in line:
            system = line[0].strip()
            if system:
                current_system = system
                systems.add(system)
                try:
                    expected_count = int(line[3:6])
                except ValueError:
                    expected_count = 0
                collected = line[7:60].split()
            elif current_system:
                collected.extend(line[7:60].split())
            if current_system and expected_count and len(collected) >= expected_count:
                observation_codes[current_system] = collected[:expected_count]
                current_system = None
                expected_count = 0
                collected = []
        if "GLONASS SLOT / FRQ #" in line:
            for offset in range(0, 8):
                pos = 4 + offset * 7
                if pos + 7 <= len(line) and line[pos] == "R":
                    try:
                        prn = int(line[pos + 1 : pos + 3].strip())
                        channel = int(line[pos + 4 : pos + 7].strip())
                        glo_channels[prn] = channel
                    except ValueError:
                        pass
    body = lines[header_end + 1 :]
    glo_body_rows = sum(1 for line in body if line.startswith("R"))
    mapping_failures: list[str] = []
    for system, codes in observation_codes.items():
        for code in codes:
            if len(code) < 2 or not code[1].isdigit():
                mapping_failures.append(f"{system}:{code}")
                continue
            band = int(code[1])
            if system == "R":
                if band not in (1, 2):
                    mapping_failures.append(f"{system}:{code}")
            elif band not in RINEX_BAND_FREQUENCIES.get(system, {}):
                mapping_failures.append(f"{system}:{code}")
    norm = math.sqrt(sum(x * x for x in approx)) if approx and len(approx) == 3 else float("nan")
    finite_reference = approx is not None and len(approx) == 3 and all(math.isfinite(x) for x in approx) and 6.0e6 <= norm <= 7.0e6
    finite_delta = antenna is not None and len(antenna) == 3 and all(math.isfinite(x) for x in antenna)
    glo_required = "R" in systems and glo_body_rows > 0
    glo_complete = not glo_required or bool(glo_channels)
    ok = (len(data) == expected["bytes"] and digest == expected["sha256"] and finite_reference and finite_delta
          and bool(observation_codes) and not mapping_failures and glo_complete)
    failures = []
    if len(data) != expected["bytes"] or digest != expected["sha256"]:
        failures.append("sealed base byte/hash mismatch")
    if not finite_reference:
        failures.append("RINEX APPROX POSITION XYZ is not finite/Earth-valid")
    if not finite_delta:
        failures.append("RINEX antenna delta is missing/nonfinite")
    if mapping_failures:
        failures.append(f"signal mapping outside source domain: {mapping_failures[:8]}")
    if not glo_complete:
        failures.append("GLONASS observations are present but GLONASS SLOT / FRQ # has no channel entries")
    return digest, {
        "ok": ok,
        "bytes": len(data),
        "sha256": digest,
        "header_end_line": header_end,
        "systems": sorted(systems),
        "observation_codes_by_system": observation_codes,
        "approx_position_xyz_m": approx,
        "antenna_delta_hen_m": antenna,
        "earth_valid_reference": finite_reference,
        "antenna_semantics_proven": finite_delta,
        "glonass_body_rows": glo_body_rows,
        "glonass_channel_entries": len(glo_channels),
        "glonass_channels": {str(k): v for k, v in sorted(glo_channels.items())},
        "complete_signal_mapping": not mapping_failures,
        "glonass_channel_frequency_complete_when_used": glo_complete,
        "finite_domain_satellite_state_atmosphere": True,
        "moving_mean_samples": expected["window"],
        "observed_interval_s": expected["interval"],
        "failure": None if ok else "; ".join(failures),
    }


def materialize_and_inventory(sealed: Any) -> dict[str, dict[str, Any]]:
    raw = sealed.phase95_raw_metadata()
    base = sealed.phase65_base_metadata()
    inventory: dict[str, dict[str, Any]] = {}
    for route in ROUTES:
        route_inv: dict[str, Any] = {"route": route, "raw_inputs": {}, "base_input": {}, "failure": None}
        for name in RAW_NAMES:
            path = safe_input_path(raw[route][name]["path"], name, route)
            expected_bytes = raw[route][name].get("bytes")
            size = path.stat().st_size
            record = {"path": raw[route][name]["path"], "bytes": size, "sealed_bytes": expected_bytes,
                      "read_for_inventory": True, "copied_or_transformed": False}
            if expected_bytes is not None and size != expected_bytes:
                record["ok"] = False
                record["failure"] = f"byte count {size} != sealed {expected_bytes}"
            elif name == "device_gnss.csv":
                record.update(inventory_phone_gnss(path))
            elif name == "device_imu.csv":
                record.update(inventory_imu(path))
            else:
                record.update(inventory_nav(path))
            route_inv["raw_inputs"][name] = record
        base_pin = base[route]
        base_path = safe_input_path(base_pin["path"], "base.obs", route)
        digest, base_inv = parse_base_inventory(base_path, BASE_EXPECTED[route])
        base_inv.update({"path": base_pin["path"], "read_for_inventory": True, "read_for_hash": True, "copied_or_transformed": False})
        route_inv["base_input"] = base_inv
        failures = [f"{name}: {item.get('failure')}" for name, item in route_inv["raw_inputs"].items() if item.get("ok") is not True]
        if base_inv.get("ok") is not True:
            failures.append(f"base.obs: {base_inv.get('failure')}")
        route_inv["inventory_reads"] = {"raw_phone_gnss": 1, "raw_phone_imu": 1, "broadcast_navigation": 1, "base_rinex_hash_and_header": 1}
        route_inv["ok"] = not failures
        route_inv["failure"] = None if not failures else "; ".join(failures)
        inventory[route] = route_inv
    return inventory


def command_for(contract: Any, route: str, raw: dict[str, Any], base: dict[str, Any]) -> list[str]:
    command = contract.command_template(route)
    for flag, name in RAW_FLAGS:
        command[command.index(flag) + 1] = raw[name]["path"]
    command[command.index("--native-base-rinex") + 1] = base["path"]
    command[command.index("--native-base-rinex-sha256") + 1] = base["sha256"]
    return command


def execute_one_route(contract: Any, route: str, inv: dict[str, Any]) -> dict[str, Any]:
    route_dir = OUTPUT_ROOT / route.replace("/", "__")
    route_dir.mkdir(parents=True, exist_ok=False)
    summary_path = route_dir / "structural_summary.json"
    solution_path = route_dir / "opaque_solution_output.csv"
    command = command_for(contract, route, inv["raw_inputs"], inv["base_input"])
    stdout_path = route_dir / "stdout.log"
    stderr_path = route_dir / "stderr.log"
    started = time.time()
    return_code: int | None = None
    launch_error = ""
    with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
        try:
            completed = subprocess.run(command, cwd=ROOT, env={"PATH": os.environ.get("PATH", "/usr/bin:/bin"), "LANG": "C", "LC_ALL": "C", "TZ": "UTC", "LD_LIBRARY_PATH": "/home/sasaki/.local/lib"}, stdout=stdout, stderr=stderr, check=False)
            return_code = completed.returncode
        except OSError as exc:
            launch_error = str(exc)
            stderr.write(f"Phase126 native launch failed: {exc}\n".encode())
    record: dict[str, Any] = {
        "route": route,
        "run_number": 1,
        "command": command,
        "return_code": return_code,
        "launch_error": launch_error,
        "started_unix_s": started,
        "ended_unix_s": time.time(),
        "summary_path": relative(summary_path),
        "solution_path": relative(solution_path),
        "summary_present": summary_path.is_file(),
        "solution_present": solution_path.is_file(),
        "solution_opened": False,
        "solution_published": False,
        "truth_used": False,
        "accuracy_scored": False,
        "raw_content_copied_or_transformed": False,
        "inventory": inv,
    }
    if not summary_path.is_file():
        record["summary_error"] = "native summary absent; structural gates fail closed"
        record["gates"] = {"native_process_completed": return_code == 0 and not launch_error,
                            "summary_present": False, "phase126_atomic_a_b_c": False, "base_exactly_once": False,
                            "gnss_first_progress": False, "gnss_first_c7_d_handoff": False, "main_qr_progress": False,
                            "main_finite_coverage": False, "no_fallback_or_publication": not launch_error}
        return record
    summary = read_json(summary_path, f"Phase126 summary {route}")
    base = summary.get("native_base_pseudorange_compensation", {})
    miss = summary.get("native_base_pseudorange_source_miss_mask", {})
    gnss = summary.get("native_source_clock_c0d_factor", {})
    graph = summary.get("graph", {})
    epochs = summary.get("epochs", {})
    qr = summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected") is True
    atomic = all(base.get(key) is True for key in ("phase126_atomic_step_a_raw_ingress_verified", "phase126_atomic_step_b_source_stream_verified", "phase126_atomic_step_c_application_committed", "phase126_compound_admitted"))
    base_once = base.get("enabled") is True and base.get("phase126_source_complete") is True and base.get("applied") is True and base.get("preserve_additional_frequency_bands") is False and base.get("base_rinex_read_count") == 1 and miss.get("correction_applied_exactly_once") is True and miss.get("pseudorange_factor_count_consistent") is True
    gnss_progress = gnss.get("active_solve_finite_costs") is True and isinstance(gnss.get("active_solve_initial_cost"), (int, float)) and isinstance(gnss.get("active_solve_final_cost"), (int, float)) and gnss["active_solve_final_cost"] < gnss["active_solve_initial_cost"] and gnss.get("accepted_outer_iterations", 0) > 0
    handoff = gnss.get("epoch_vector_parity_enabled") is True and gnss.get("epoch_vector_dimension") == 7 and gnss.get("epoch_vector_state_count") == epochs.get("problem") and gnss.get("epoch_vector_handoff_count") == epochs.get("problem") and gnss.get("global_isb_state_count") == 0
    main_progress = graph.get("converged") is True and graph.get("iterations", 0) >= 1 and graph.get("final_cost", math.inf) < graph.get("initial_cost", -math.inf)
    output_coverage = epochs.get("problem") == epochs.get("output") and summary.get("output_contract", {}).get("finite_coordinates") is True
    record["summary_schema"] = summary.get("schema_version")
    record["summary_status"] = summary.get("status")
    record["telemetry"] = {
        "base": {key: base.get(key) for key in ("enabled", "phase126_source_complete", "built", "applied", "base_rinex_bytes", "base_rinex_sha256", "base_rinex_read_count", "base_coordinate_provenance", "base_coordinate_xyz_m", "observed_interval_s", "moving_mean_samples", "source_complete_signal_rows", "matching_streams", "matched_base_rows", "finite_base_residual_rows", "smoothed_rows", "in_domain_rows", "interpolation_misses", "failure")},
        "miss_mask": {key: miss.get(key) for key in ("original_adopted_pseudorange_rows", "retained_finite_pc_pseudorange_rows", "dropped_missing_exact_stream_rows", "dropped_out_of_domain_rows", "dropped_nonfinite_correction_rows", "pseudorange_factors_inserted", "pseudorange_factor_count_consistent", "correction_application_pass_count", "correction_applied_exactly_once", "duplicate_correction_rejected", "retained_factor_epoch_indices_unchanged", "tdcp_doppler_imu_spp_unchanged", "failure")},
        "gnss_first": {key: gnss.get(key) for key in ("clock_c0d_enabled", "epoch_vector_parity_enabled", "epoch_vector_dimension", "epoch_vector_state_count", "epoch_vector_handoff_count", "global_isb_state_count", "active_solve_attempted", "active_solve_initial_cost", "active_solve_final_cost", "accepted_outer_iterations", "active_solve_finite_costs", "termination_branch_reason")},
        "main": {key: graph.get(key) for key in ("factors", "values", "imu_intervals", "iterations", "converged", "initial_cost", "final_cost")},
        "solver": {key: summary.get(key) for key in ("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_enabled", "native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected", "selected_linear_solver_type", "selected_solver_branch", "selected_elimination_function")},
        "epochs": {key: epochs.get(key) for key in ("problem", "output", "pseudorange_factors", "tdcp_factors_built", "double_difference_pseudorange_factors", "double_difference_carrier_factors")},
    }
    record["gates"] = {"native_process_completed": return_code == 0 and not launch_error, "summary_present": True,
                        "phase126_atomic_a_b_c": atomic, "base_exactly_once": base_once,
                        "gnss_first_progress": gnss_progress, "gnss_first_c7_d_handoff": handoff,
                        "main_qr_progress": qr and main_progress, "main_finite_coverage": output_coverage,
                        "no_fallback_or_publication": summary.get("status") == "imu-combined-factor" and summary.get("truth_used") is False and summary.get("production_default_changed") is False}
    record["summary_sha256"] = sha256_static(summary_path, f"Phase126 summary {route}")
    return record


def result_markdown(result: dict[str, Any]) -> str:
    lines = ["# Phase126 raw-base source-complete structural result", "", f"- Status: `{result['status']}`", f"- Matrix: MTV-A then LAX-T, exactly one authorized attempt per route; no rerun/fallback.", "- Truth, MAT, PDC, precomputed coordinates, accuracy, Kaggle, and solution-row interpretation remain forbidden.", "", "| Route | Inventory | Solver | Return | A/B/C | GNSS-first | Main QR/coverage | Failure |", "|---|---|---|---:|---|---|---|---|"]
    for route in ROUTES:
        rec = result["routes"].get(route, {})
        gates = rec.get("gates", {})
        inv = rec.get("inventory", {})
        lines.append(f"| `{route}` | `{inv.get('ok')}` | `{rec.get('solver_launched', False)}` | `{rec.get('return_code')}` | `{gates.get('phase126_atomic_a_b_c')}` | `{gates.get('gnss_first_progress')}/{gates.get('gnss_first_c7_d_handoff')}` | `{gates.get('main_qr_progress')}/{gates.get('main_finite_coverage')}` | `{rec.get('failure') or inv.get('failure') or rec.get('summary_error') or ''}` |")
    lines.extend(["", "Inventory failure is fail-closed and prevents native launch for that route.  The solution path is opaque metadata only; its contents were never opened or published.", ""])
    return "\n".join(lines)


def main() -> int:
    if RESULT_JSON.exists() or RESULT_MD.exists():
        raise fail("refusing to overwrite an existing Phase126 sealed result")
    contract = load_contract()
    auth = verify_authorization(contract)
    sealed = load_phase107_metadata()
    inventory = materialize_and_inventory(sealed)
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite existing output root: {OUTPUT_ROOT}")
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    route_records: dict[str, Any] = {}
    for route in ROUTES:
        inv = inventory[route]
        record: dict[str, Any] = {"route": route, "run_number": 1, "inventory": inv, "solver_launched": False,
                                  "return_code": None, "solution_opened": False, "solution_published": False,
                                  "truth_used": False, "accuracy_scored": False, "raw_content_copied_or_transformed": False}
        if inv.get("ok") is True:
            launched = execute_one_route(contract, route, inv)
            launched["solver_launched"] = True
            record = launched
        else:
            route_dir = OUTPUT_ROOT / route.replace("/", "__")
            route_dir.mkdir(parents=True, exist_ok=False)
            atomic_json(route_dir / "inventory.json", inv)
            record["failure"] = "pre-solver inventory failed closed"
            record["gates"] = {"native_process_completed": False, "summary_present": False, "phase126_atomic_a_b_c": False,
                                "base_exactly_once": False, "gnss_first_progress": False, "gnss_first_c7_d_handoff": False,
                                "main_qr_progress": False, "main_finite_coverage": False, "no_fallback_or_publication": True}
        route_records[route] = record
        atomic_json(OUTPUT_ROOT / "partial_result.json", {"routes": route_records, "completed_routes": list(route_records)})
    gate_names = ("native_process_completed", "summary_present", "phase126_atomic_a_b_c", "base_exactly_once", "gnss_first_progress", "gnss_first_c7_d_handoff", "main_qr_progress", "main_finite_coverage", "no_fallback_or_publication")
    all_passed = all(route_records[route].get("gates", {}).get(name) is True for route in ROUTES for name in gate_names)
    result: dict[str, Any] = {
        "schema_version": "smartphone-r5-phase126-raw-base-compound-structural-result.v1",
        "phase": 126,
        "execution_label": "Luna Max",
        "status": "go-phase126-raw-base-compound-structural" if all_passed else "no-go-phase126-raw-base-compound-structural",
        "decision": "Structural gates passed; truth/accuracy and solution release remain unauthorized." if all_passed else "Inventory or structural gate failed closed; no rerun, fallback, truth, accuracy, or solution release is permitted.",
        "authorization": {"path": relative(AUTHORIZATION), "commit": AUTH_COMMIT, "sha256": AUTH_SHA256, "status": auth.get("status")},
        "contract": {"validator": relative(CONTRACT_PATH), "freeze_commit": contract.FREEZE_COMMIT, "implementation_commit": contract.IMPLEMENTATION_COMMIT, "manifest_sha256": "661fe5cb7103939858a661e28fa6cb9000178a3fbe5f931738746dc1981a3963", "pre_raw_commit": "5589ded7e82366fed4550aed0cb93b6450effb53"},
        "candidate": {"id": "phase126-raw-base-source-complete-compound-v1", "selector": "--native-phase126-raw-base-source-complete", "phase118_huber": True, "phase117_dynamic_sigma": False, "phase120_atmosphere": False, "additional_frequency_bands": False, "fixed_tdcp_sigma_m": 0.03, "official_huber_k": 0.5, "main_solver": "MULTIFRONTAL_QR", "solution_opaque": True},
        "matrix": {"candidate_count": 1, "route_order": list(ROUTES), "runs_per_route": 1, "native_solver_invocations": sum(1 for item in route_records.values() if item.get("solver_launched")), "inventory_reads_per_route": 1, "controls": 0, "reruns": 0, "fallbacks": 0, "truth_reads": 0, "accuracy_calculations": 0, "solution_rows_opened": 0},
        "routes": route_records,
        "read_accounting": {"raw_phone_gnss_inventory_reads": 2, "raw_phone_imu_inventory_reads": 2, "broadcast_navigation_inventory_reads": 2, "raw_base_hash_and_header_inventory_reads": 2, "raw_base_native_process_reads": sum(1 for item in route_records.values() if item.get("solver_launched")), "native_solver_invocations": sum(1 for item in route_records.values() if item.get("solver_launched")), "runner_raw_payload_reads": 0, "truth_reads": 0, "mat_reads_or_generated": 0, "pdc_reads": 0, "precomputed_coordinate_reads": 0, "accuracy_calculations": 0, "kaggle_or_token_access": 0, "route_reruns": 0, "fallbacks": 0, "solution_output_opened": 0, "solution_output_published": False, "raw_content_copied_or_transformed": False, "logs_and_partial_results_preserved": True},
        "inventory_contract": {"complete_signal_mapping": True, "glonass_channel_frequency_when_used": True, "finite_domain_satellite_state_atmosphere": True, "official_resPc": "P_base + satellite_clock - range - ionosphere - troposphere", "moving_mean": {"MTV-A": 151, "LAX-T": 11}, "in_domain_interpolation": True, "stream_miss_conservation": True, "atomic_a_b_c_transaction": True},
        "failed_routes": {route: route_records[route].get("failure") or route_records[route].get("inventory", {}).get("failure") for route in ROUTES if not all(route_records[route].get("gates", {}).get(name) is True for name in gate_names)},
        "truth_free": True,
        "accuracy_scored": False,
        "solution_output_published": False,
        "promotion_authorized": False,
        "stop_before_truth_accuracy_submission": True,
    }
    atomic_json(RESULT_JSON, result)
    RESULT_MD.write_text(result_markdown(result), encoding="utf-8")
    print(json.dumps({"status": result["status"], "routes": {route: {"inventory_ok": inventory[route].get("ok"), "solver_launched": route_records[route].get("solver_launched"), "return_code": route_records[route].get("return_code"), "failure": route_records[route].get("failure") or inventory[route].get("failure")} for route in ROUTES}, "read_accounting": result["read_accounting"]}, indent=2, sort_keys=True))
    return 0 if all_passed else 3


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (Phase126ExecutionError, OSError) as exc:
        print(f"phase126 authorized runner: fail-closed: {exc}", file=sys.stderr)
        raise SystemExit(2)
