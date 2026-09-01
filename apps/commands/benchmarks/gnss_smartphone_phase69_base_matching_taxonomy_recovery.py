#!/usr/bin/env python3
"""Truth-free recovery of the Phase68 RINEX3 matching taxonomy audit.

Phase68 failed closed before producing a route report because the pinned
RINEX3 base records are long single physical lines.  This recovery keeps the
same raw/base pins and taxonomy, but frames satellite records by their actual
physical line shape and rejects an epoch marker as a satellite record.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase69_base_matching_taxonomy_recovery_freeze_v1.json"
FREEZE_SHA256 = "1f37b0cbb96c1973eb1727f4ea5b21b168389d4bd8fb6ead89c55e2668a83343"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase69_base_matching_taxonomy_recovery_manifest_v1.json"
PHASE68_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase68_base_matching_taxonomy.py"
PHASE68_EVALUATOR_SHA256 = "1374bc1bd2863b99b4488121c240d74fea541dae623ecb73cb982cff582755c5"
PHASE68_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase68_base_matching_taxonomy_freeze_v1.json"
PHASE68_FREEZE_SHA256 = "f409f7038add71cd0a41df9278fc99651d637dd3697c10913c9da1fbb8c53b43"
PHASE68_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase68_base_matching_taxonomy_manifest_v1.json"
PHASE68_MANIFEST_SHA256 = "045886c2a9b231248866c478fca95d4a8146e837c8a3001cf9d03c1b9829d053"
PHASE68_FAILURE_RECORD = ROOT / "docs/use_cases/records/smartphone_r5_phase68_base_matching_taxonomy_failure_v1.json"
PHASE68_FAILURE_RECORD_SHA256 = "f7f7edec127751c24fef3dc4dfa714d15edc7849a9c794e0a76a9ad9709d8957"
PHASE68_FAILURE_OUTPUT_SHA256 = "8217027f0aa6bd7e809f9b93cd5d2080ebdfe231ff989c7b68be5ced62514a14"
PHASE68_FAILURE_OUTPUT = ROOT / "output/smartphone-r5/phase68-base-matching-taxonomy-v1/phase68_base_matching_taxonomy_failure.json"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase69-base-matching-taxonomy-recovery-v1"
TIME_TOLERANCE_S = 1.0e-6


def _load_phase68() -> Any:
    spec = importlib.util.spec_from_file_location("phase68_taxonomy_contract", PHASE68_EVALUATOR)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load sealed Phase68 helper")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


P68 = _load_phase68()
ROUTES = P68.ROUTES


class Phase69Error(ValueError):
    """Raised when the immutable Phase69 recovery contract fails."""


def fail(message: str) -> Phase69Error:
    return Phase69Error(message)


def digest_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def digest_file(path: Path) -> str:
    P68.P65.reject_forbidden(path)
    try:
        with path.open("rb") as handle:
            return digest_bytes(handle.read())
    except OSError as exc:
        raise fail(f"cannot hash {path}: {exc}") from exc


def load_object(path: Path, label: str) -> dict[str, Any]:
    P68.P65.reject_forbidden(path)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise fail(f"invalid {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    P68.atomic_json(path, value)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def read_once(path: Path, expected_sha256: str, expected_bytes: int | None = None) -> bytes:
    P68.P65.reject_forbidden(path)
    try:
        with path.open("rb") as handle:
            payload = handle.read()
    except OSError as exc:
        raise fail(f"cannot read pinned input {path}: {exc}") from exc
    actual = digest_bytes(payload)
    if actual != expected_sha256:
        raise fail(f"input hash mismatch for {path}: {actual} != {expected_sha256}")
    if expected_bytes is not None and len(payload) != expected_bytes:
        raise fail(f"input byte mismatch for {path}: {len(payload)} != {expected_bytes}")
    return payload


def verify_freeze() -> dict[str, Any]:
    if FREEZE_SHA256.startswith("TODO"):
        raise fail("Phase69 freeze SHA is not sealed")
    if digest_file(FREEZE) != FREEZE_SHA256:
        raise fail("Phase69 freeze hash changed")
    freeze = load_object(FREEZE, "Phase69 freeze")
    if freeze.get("status") != "frozen-before-phase69-raw-base-reread":
        raise fail("Phase69 freeze status changed")
    authority = freeze.get("authority", {})
    if authority.get("phase68_evaluator_sha256") != PHASE68_EVALUATOR_SHA256:
        raise fail("Phase68 evaluator pin changed")
    if authority.get("phase68_failure_record_sha256") != PHASE68_FAILURE_RECORD_SHA256:
        raise fail("Phase68 failure-record pin changed")
    if digest_file(PHASE68_EVALUATOR) != PHASE68_EVALUATOR_SHA256:
        raise fail("Phase68 evaluator changed")
    if digest_file(PHASE68_FREEZE) != PHASE68_FREEZE_SHA256:
        raise fail("Phase68 freeze changed")
    if digest_file(PHASE68_MANIFEST) != PHASE68_MANIFEST_SHA256:
        raise fail("Phase68 manifest changed")
    if digest_file(PHASE68_FAILURE_RECORD) != PHASE68_FAILURE_RECORD_SHA256:
        raise fail("Phase68 failure record changed")
    if PHASE68_FAILURE_OUTPUT.is_file() and digest_file(PHASE68_FAILURE_OUTPUT) != PHASE68_FAILURE_OUTPUT_SHA256:
        raise fail("Phase68 failure output changed")
    try:
        P68.verify_freeze()
    except Exception as exc:
        raise fail(f"Phase68 sealed contract no longer verifies: {exc}") from exc
    if tuple(freeze.get("route_order", ())) != ROUTES:
        raise fail("Phase69 route order changed")
    if set(freeze.get("raw_inputs", {})) != set(ROUTES) or set(freeze.get("base_inputs", {})) != set(ROUTES):
        raise fail("Phase69 input pins are incomplete")
    expected_reads = {
        "single_process": True,
        "raw_device_gnss_reads": 4,
        "base_rinex_reads": 4,
        "nav_reads": 0,
        "imu_reads": 0,
        "solver_invocations": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "validation_holdout_reads": 0,
        "archive_reopens": 0,
        "phase68_partial_output_reuse": False,
        "new_output_root_required": True,
    }
    for key, expected in expected_reads.items():
        if freeze.get("read_contract", {}).get(key) != expected:
            raise fail(f"Phase69 read contract changed: {key}")
    recovery = freeze.get("recovery_contract", {})
    if recovery.get("fixture_required") is not True or recovery.get("taxonomy_preserved") is not True:
        raise fail("Phase69 fixture/taxonomy contract changed")
    manifest = load_object(MANIFEST, "Phase69 manifest")
    if manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256:
        raise fail("Phase69 manifest freeze pin changed")
    if manifest.get("evaluator", {}).get("sha256") != digest_file(Path(__file__)):
        raise fail("Phase69 evaluator pin changed")
    if manifest.get("routes") != list(ROUTES):
        raise fail("Phase69 manifest routes changed")
    if manifest.get("read_accounting", {}).get("truth_reads") != 0:
        raise fail("Phase69 manifest truth accounting changed")
    if manifest.get("read_accounting", {}).get("phase68_partial_output_reuse") is not False:
        raise fail("Phase69 prior-output policy changed")
    return freeze


def _record_is_satellite_line(line: str) -> bool:
    return len(line) >= 3 and line[0].isalpha() and line[1:3].isdigit()


def _record_lines(lines: list[str], cursor: int, obs_count: int) -> tuple[list[str], int]:
    if cursor >= len(lines):
        raise fail("RINEX3 ended before satellite record")
    first = lines[cursor]
    if first.startswith(">"):
        raise fail("RINEX3 epoch marker was presented as a satellite record")
    if len(first) > 80:
        # The pinned files and native reader use one long satellite line.
        return [first], cursor + 1
    line_count = max(1, (obs_count + 4) // 5)
    records = [first]
    for _ in range(line_count - 1):
        if cursor + 1 >= len(lines):
            raise fail("RINEX3 ended inside a continued satellite record")
        next_line = lines[cursor + 1]
        if next_line.startswith(">") or _record_is_satellite_line(next_line):
            raise fail("RINEX3 satellite record ended before its declared continuation")
        records.append(next_line)
        cursor += 1
    return records, cursor + 1


def observation_value(record_lines: list[str], obs_index: int, version: float) -> float:
    if version < 3.0:
        return P68.observation_value(record_lines, obs_index, version)
    first = record_lines[0][3:]
    if len(record_lines) > 1:
        continuation = "".join(line[3:] if line.startswith("   ") else line for line in record_lines[1:])
        fields = first + continuation
    else:
        fields = first
    start = obs_index * 16
    return P68.parse_float_field(fields[start:start + 14])


def parse_header(lines: list[str]) -> tuple[int, float, dict[str, list[str]], list[str], dict[int, int]]:
    """Parse the Phase68 header contract without importing its buggy body parser."""
    version = math.nan
    system_types: dict[str, list[str]] = {}
    global_types: list[str] = []
    glonass_channels: dict[int, int] = {}
    end_header = -1
    active_system: str | None = None
    expected_count = 0
    for index, line in enumerate(lines):
        label = line[60:].strip() if len(line) >= 60 else ""
        if index == 0:
            try:
                version = float(line[:20].strip())
            except ValueError as exc:
                raise fail("RINEX header version is invalid") from exc
        if label == "SYS / # / OBS TYPES":
            system = line[:1].strip()
            if system:
                active_system = system
                try:
                    expected_count = int(line[3:6].strip() or "0")
                except ValueError as exc:
                    raise fail("RINEX observation-type count is invalid") from exc
                system_types.setdefault(system, [])
            if active_system is None:
                raise fail("RINEX observation-type continuation has no active system")
            values = [
                line[position:position + 3].strip()
                for position in range(7, max(7, min(len(line), 60) - 2), 4)
            ]
            system_types[active_system].extend(value for value in values if value)
            if expected_count and len(system_types[active_system]) >= expected_count:
                system_types[active_system] = system_types[active_system][:expected_count]
                active_system = None
                expected_count = 0
        elif label == "# / TYPES OF OBSERV":
            try:
                count = int(line[:6].strip() or "0")
            except ValueError as exc:
                raise fail("RINEX global observation-type count is invalid") from exc
            values = [line[position:position + 6].strip() for position in range(6, min(len(line), 60), 6)]
            global_types.extend(value for value in values if value)
            if count and len(global_types) >= count:
                global_types = global_types[:count]
        elif label == "GLONASS SLOT / FRQ #":
            for match in P68.re.finditer(r"R\s*(\d{1,2})\s+([+-]?\d+)", line[:60]):
                glonass_channels[int(match.group(1))] = int(match.group(2))
        elif label == "END OF HEADER":
            end_header = index
            break
    if end_header < 0:
        raise fail("RINEX END OF HEADER is missing")
    return end_header, version, system_types, global_types, glonass_channels


def parse_rinex(payload: bytes) -> Any:
    try:
        text = payload.decode("ascii")
    except UnicodeDecodeError as exc:
        raise fail(f"base RINEX is not ASCII: {exc}") from exc
    lines = text.splitlines()
    end_header, version, system_types, global_types, glonass_channels = parse_header(lines)
    index = P68.BaseIndex(version=version, observation_types=system_types, glonass_channels=glonass_channels)
    cursor = end_header + 1
    if version < 3.0:
        # The pinned members are RINEX3; keep the inherited parser for the
        # unsupported legacy branch so this recovery cannot silently broaden
        # its input contract.
        raise fail("Phase69 recovery requires RINEX3 base members")
    while cursor < len(lines):
        line = lines[cursor]
        if not line.startswith(">"):
            cursor += 1
            continue
        parsed = P68.parse_epoch_v3(line)
        if parsed is None:
            raise fail(f"invalid RINEX3 epoch line at {cursor + 1}")
        epoch_time, flag, satellite_count, *_ = parsed
        cursor += 1
        if flag >= 2:
            cursor += satellite_count
            continue
        for _ in range(satellite_count):
            if cursor >= len(lines):
                raise fail("RINEX3 ended inside an epoch")
            first = lines[cursor]
            if first.startswith(">"):
                raise fail("RINEX3 epoch marker was presented as a satellite record")
            sat_token = first[:3]
            system = P68.system_from_rinex_char(sat_token[:1])
            if system is None:
                raise fail(f"unsupported RINEX3 system {sat_token[:1]!r}")
            try:
                svid = int(sat_token[1:3])
            except ValueError as exc:
                raise fail(f"invalid RINEX3 satellite token {sat_token!r}") from exc
            obs_types = system_types.get(sat_token[:1], global_types)
            records, cursor = _record_lines(lines, cursor, len(obs_types))
            entries: list[tuple[str, int, str, Any, float]] = []
            channel = glonass_channels.get(svid) if system == "GLONASS" else None
            for obs_index, obs_type in enumerate(obs_types):
                if not obs_type or obs_type[0] not in {"C", "P"}:
                    continue
                value = observation_value(records, obs_index, version)
                signal = P68.rinex_signal(system, obs_type, channel)
                if signal is not None:
                    entries.append((system, svid, obs_type, signal, value))
            P68.add_base_epoch(index, epoch_time, entries)
    all_times = [time for values in index.exact_streams.values() for time in values]
    if all_times:
        index.time_min_s = min(all_times)
        index.time_max_s = max(all_times)
    return index


def route_report(route: str, raw_payload: bytes, base_payload: bytes) -> dict[str, Any]:
    raw = P68.parse_raw_csv(raw_payload)
    base = parse_rinex(base_payload)
    classification = P68.classify_rows(raw, base)
    return {
        "raw": {
            "adopted_rows_definition": "diagnostic source-filtered raw pseudorange proxy; not asserted equal to the native adopted FGO factors",
            "input_rows": raw.input_rows,
            "raw_rows": raw.raw_rows,
            "adopted_rows": len(raw.adopted_rows),
            "unsupported_signal_rows": raw.unsupported_signal_rows,
            "invalid_timing_rows": raw.invalid_timing_rows,
            "invalid_quality_rows": raw.invalid_quality_rows,
            "invalid_pseudorange_rows": raw.invalid_pseudorange_rows,
            "masked_code_rows": raw.masked_code_rows,
            "duplicate_epoch_signal_rows": raw.duplicate_epoch_signal_rows,
            "signal_census": dict(sorted(raw.signal_census.items())),
            "canonical_frequency_census": dict(sorted(raw.canonical_census.items())),
            "time_min_s": raw.time_min_s,
            "time_max_s": raw.time_max_s,
        },
        "base": {
            "selected_rows_definition": "diagnostic finite RINEX code stream under the pinned source mapping; not asserted equal to native adopted FGO factors",
            "rinex_version": base.version,
            "base_rows": base.base_rows,
            "finite_code_rows": base.finite_code_rows,
            "selected_rows": base.selected_rows,
            "exact_stream_count": len(base.exact_streams),
            "canonical_frequency_stream_count": len(base.frequency_streams),
            "satellite_stream_count": len(base.satellite_streams),
            "signal_census": dict(sorted(base.signal_census.items())),
            "canonical_frequency_census": dict(sorted(base.canonical_census.items())),
            "duplicate_epoch_frequency_rows": base.duplicate_epoch_frequency_rows,
            "duplicate_epoch_frequency_events": base.duplicate_epoch_frequency_events,
            "time_min_s": base.time_min_s,
            "time_max_s": base.time_max_s,
            "glonass_channels": {str(key): value for key, value in sorted(base.glonass_channels.items())},
        },
        "classification": classification,
    }


def run_audit(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    freeze = verify_freeze()
    output_root = output_root.resolve()
    P68.P65.reject_forbidden(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty Phase69 output: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    reports: dict[str, Any] = {}
    read_counts = {"raw_device_gnss": 0, "base_rinex": 0}
    try:
        for route in ROUTES:
            raw_pin = freeze["raw_inputs"][route]
            base_pin = freeze["base_inputs"][route]
            raw_payload = read_once(ROOT / raw_pin["device_gnss.csv"], raw_pin["sha256"])
            read_counts["raw_device_gnss"] += 1
            base_payload = read_once(ROOT / base_pin["path"], base_pin["sha256"], int(base_pin["bytes"]))
            read_counts["base_rinex"] += 1
            reports[route] = route_report(route, raw_payload, base_payload)
        counts = {
            key: sum(report["classification"]["counts"][key] for report in reports.values())
            for key in next(iter(reports.values()))["classification"]["counts"]
        }
        result = {
            "schema_version": "smartphone-r5-phase69-base-matching-taxonomy-recovery-result.v1",
            "phase": 69,
            "execution_label": "Luna Max",
            "status": "sealed-truth-free-matching-taxonomy-recovery",
            "truth_free": True,
            "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
            "manifest": {"path": relative(MANIFEST), "sha256": digest_file(MANIFEST)},
            "prior_phase68": {
                "failure_record": relative(PHASE68_FAILURE_RECORD),
                "failure_record_sha256": PHASE68_FAILURE_RECORD_SHA256,
                "failure_output_sha256": PHASE68_FAILURE_OUTPUT_SHA256,
                "reread": False,
                "partial_output_reused": False,
            },
            "routes": reports,
            "native_adopted_fgo_equality_claim": False,
            "proxy_scope_note": "Raw adopted and base selected counts are diagnostic proxies. This result does not claim equality to native adopted FGO factor populations.",
            "aggregate": {
                "adopted_rows": sum(report["classification"]["adopted_rows"] for report in reports.values()),
                "classification_counts": counts,
                "duplicate_epoch_frequency_events": sum(report["base"]["duplicate_epoch_frequency_events"] for report in reports.values()),
                "duplicate_epoch_frequency_rows": sum(report["base"]["duplicate_epoch_frequency_rows"] for report in reports.values()),
                "classification_sum_checks": all(report["classification"]["sum_check"] for report in reports.values()),
            },
            "read_accounting": {
                "single_process": True,
                "raw_device_gnss_reads": read_counts["raw_device_gnss"],
                "base_rinex_reads": read_counts["base_rinex"],
                "nav_reads": 0,
                "imu_reads": 0,
                "solver_invocations": 0,
                "truth_reads": 0,
                "mat_reads_or_generated": 0,
                "validation_holdout_reads": 0,
                "archive_reopens": 0,
                "phase68_partial_output_reuse": False,
            },
            "gate_policy": {
                "phase67_coverage_gate_unchanged": True,
                "native_correction_authorized": False,
                "canonicalization_authorized": False,
                "interpolation_change_authorized": False,
                "zero_point_782": "not evaluated",
            },
        }
        result_path = output_root / "phase69_base_matching_taxonomy_recovery_result.json"
        atomic_json(result_path, result)
        manifest = {
            "schema_version": "smartphone-r5-phase69-base-matching-taxonomy-recovery-output-manifest.v1",
            "phase": 69,
            "status": "sealed-truth-free-diagnostic-recovery",
            "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
            "evaluator": {"path": relative(Path(__file__)), "sha256": digest_file(Path(__file__))},
            "result": {"path": relative(result_path), "sha256": digest_file(result_path), "bytes": result_path.stat().st_size},
            "routes": list(ROUTES),
            "raw_device_gnss_reads": read_counts["raw_device_gnss"],
            "base_rinex_reads": read_counts["base_rinex"],
            "solver_invocations": 0,
            "truth_reads": 0,
            "phase68_partial_output_reuse": False,
            "all_classification_sum_checks": result["aggregate"]["classification_sum_checks"],
        }
        atomic_json(output_root / "phase69_base_matching_taxonomy_recovery_output_manifest.json", manifest)
        return result
    except Phase69Error as exc:
        atomic_json(
            output_root / "phase69_base_matching_taxonomy_recovery_failure.json",
            {
                "schema_version": "smartphone-r5-phase69-base-matching-taxonomy-recovery-failure.v1",
                "status": "fail-closed",
                "error": str(exc),
                "read_accounting": {**read_counts, "truth_reads": 0, "solver_invocations": 0},
                "phase68_partial_output_reuse": False,
            },
        )
        raise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--run-audit", action="store_true")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        if args.verify_freeze:
            verify_freeze()
        if args.run_audit:
            result = run_audit(args.output_root)
            print(json.dumps({"status": result["status"], "routes": len(result["routes"]), "adopted_rows": result["aggregate"]["adopted_rows"], "truth_reads": result["read_accounting"]["truth_reads"]}, sort_keys=True))
        elif not args.verify_freeze:
            parser.error("one of --verify-freeze or --run-audit is required")
        return 0
    except Phase69Error as exc:
        print(f"phase69 matching taxonomy recovery failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
