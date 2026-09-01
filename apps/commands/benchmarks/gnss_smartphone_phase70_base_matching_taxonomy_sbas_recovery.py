#!/usr/bin/env python3
"""Truth-free Phase70 recovery of the RINEX3 SBAS matching taxonomy audit."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase70_base_matching_taxonomy_sbas_recovery_freeze_v1.json"
FREEZE_SHA256 = "e5d1572fa8ac4b2afa31998f7571a7d55d84f1890f01189a107719d22241ec17"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase70_base_matching_taxonomy_sbas_recovery_manifest_v1.json"
PHASE69_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase69_base_matching_taxonomy_recovery.py"
PHASE69_EVALUATOR_SHA256 = "e440d712efd07605cae5197d2445a1875d14b38280e7186a2d81b58a0a866f2e"
PHASE69_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase69_base_matching_taxonomy_recovery_freeze_v1.json"
PHASE69_FREEZE_SHA256 = "1f37b0cbb96c1973eb1727f4ea5b21b168389d4bd8fb6ead89c55e2668a83343"
PHASE69_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase69_base_matching_taxonomy_recovery_manifest_v1.json"
PHASE69_MANIFEST_SHA256 = "b5d11c4e964bc8629fa9fb70a99d81a2672a3802b670cf3e5986da84492c9a91"
PHASE69_FAILURE_RECORD = ROOT / "docs/use_cases/records/smartphone_r5_phase69_base_matching_taxonomy_recovery_failure_v1.json"
PHASE69_FAILURE_RECORD_SHA256 = "8419e717bf8f8bd409e41ac9834be5bea5e76f26e18e8cb2ef43c365197b992a"
PHASE69_FAILURE_OUTPUT = ROOT / "output/smartphone-r5/phase69-base-matching-taxonomy-recovery-v1/phase69_base_matching_taxonomy_recovery_failure.json"
PHASE69_FAILURE_OUTPUT_SHA256 = "071aff78a1b6426b066f89660700876d2b926837d2786dda357126fc7a698bc3"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase70-base-matching-taxonomy-sbas-recovery-v1"


def _load_phase69() -> Any:
    spec = importlib.util.spec_from_file_location("phase69_taxonomy_contract", PHASE69_EVALUATOR)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load sealed Phase69 helper")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


P69 = _load_phase69()
ROUTES = P69.ROUTES


class Phase70Error(ValueError):
    """Raised when the immutable Phase70 recovery contract fails."""


def fail(message: str) -> Phase70Error:
    return Phase70Error(message)


def digest_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def digest_file(path: Path) -> str:
    P69.P68.P65.reject_forbidden(path)
    if not path.is_file():
        raise fail(f"missing pinned artifact: {path}")
    try:
        with path.open("rb") as handle:
            return digest_bytes(handle.read())
    except OSError as exc:
        raise fail(f"cannot hash {path}: {exc}") from exc


def load_object(path: Path, label: str) -> dict[str, Any]:
    P69.P68.P65.reject_forbidden(path)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise fail(f"invalid {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def read_once(path: Path, expected_sha256: str, expected_bytes: int | None = None) -> bytes:
    P69.P68.P65.reject_forbidden(path)
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


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    P69.P68.atomic_json(path, value)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def verify_freeze() -> dict[str, Any]:
    if FREEZE_SHA256.startswith("TODO"):
        raise fail("Phase70 freeze SHA is not sealed")
    if digest_file(FREEZE) != FREEZE_SHA256:
        raise fail("Phase70 freeze hash changed")
    freeze = load_object(FREEZE, "Phase70 freeze")
    if freeze.get("status") != "frozen-before-phase70-raw-base-reread":
        raise fail("Phase70 freeze status changed")
    authority = freeze.get("authority", {})
    if authority.get("phase69_evaluator_sha256") != PHASE69_EVALUATOR_SHA256:
        raise fail("Phase69 evaluator pin changed")
    if authority.get("phase69_manifest_sha256") != PHASE69_MANIFEST_SHA256:
        raise fail("Phase69 manifest pin changed")
    if authority.get("phase69_failure_record_sha256") != PHASE69_FAILURE_RECORD_SHA256:
        raise fail("Phase69 failure record pin changed")
    for path, expected in (
        (PHASE69_EVALUATOR, PHASE69_EVALUATOR_SHA256),
        (PHASE69_FREEZE, PHASE69_FREEZE_SHA256),
        (PHASE69_MANIFEST, PHASE69_MANIFEST_SHA256),
        (PHASE69_FAILURE_RECORD, PHASE69_FAILURE_RECORD_SHA256),
        (PHASE69_FAILURE_OUTPUT, PHASE69_FAILURE_OUTPUT_SHA256),
    ):
        if digest_file(path) != expected:
            raise fail(f"sealed Phase69 artifact changed: {path}")
    try:
        phase69_freeze = P69.verify_freeze()
    except Exception as exc:
        raise fail(f"Phase69 sealed contract no longer verifies: {exc}") from exc
    if tuple(freeze.get("route_order", ())) != ROUTES or tuple(phase69_freeze.get("route_order", ())) != ROUTES:
        raise fail("Phase70 route order changed")
    if freeze.get("input_pin_source", {}).get("raw_and_base_pins") != "Phase69 freeze exact raw CSV and base RINEX pins":
        raise fail("Phase70 input pin source changed")
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
        "phase69_partial_output_reuse": False,
        "new_output_root_required": True,
    }
    for key, expected in expected_reads.items():
        if freeze.get("read_contract", {}).get(key) != expected:
            raise fail(f"Phase70 read contract changed: {key}")
    system_contract = freeze.get("source_system_contract", {})
    expected_systems = {"G": "GPS", "R": "GLONASS", "E": "Galileo", "C": "BeiDou", "J": "QZSS", "S": "SBAS", "I": "NavIC"}
    if system_contract.get("rinex_system_characters") != expected_systems:
        raise fail("Phase70 RINEX system mapping changed")
    if system_contract.get("no_data_selected_system_expansion") is not True:
        raise fail("Phase70 non-adopted-system policy changed")
    manifest = load_object(MANIFEST, "Phase70 manifest")
    if manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256:
        raise fail("Phase70 manifest freeze pin changed")
    if manifest.get("evaluator", {}).get("sha256") != digest_file(Path(__file__)):
        raise fail("Phase70 evaluator pin changed")
    if manifest.get("routes") != list(ROUTES):
        raise fail("Phase70 manifest route order changed")
    if manifest.get("read_accounting", {}).get("truth_reads") != 0:
        raise fail("Phase70 manifest truth accounting changed")
    if manifest.get("read_accounting", {}).get("phase69_partial_output_reuse") is not False:
        raise fail("Phase70 prior-output policy changed")
    return freeze


def system_from_rinex_char(value: str) -> str | None:
    return {
        "G": "GPS",
        "R": "GLONASS",
        "E": "Galileo",
        "C": "BeiDou",
        "J": "QZSS",
        "S": "SBAS",
        "I": "NavIC",
    }.get(value)


def parse_rinex(payload: bytes) -> Any:
    """Use Phase69 framing and header parsing, adding native S/I recognition."""
    try:
        text = payload.decode("ascii")
    except UnicodeDecodeError as exc:
        raise fail(f"base RINEX is not ASCII: {exc}") from exc
    lines = text.splitlines()
    end_header, version, system_types, global_types, glonass_channels = P69.parse_header(lines)
    if version < 3.0:
        raise fail("Phase70 recovery requires RINEX3 base members")
    index = P69.P68.BaseIndex(version=version, observation_types=system_types, glonass_channels=glonass_channels)
    index.unselected_system_record_count = 0
    index.unselected_system_census = {}
    cursor = end_header + 1
    while cursor < len(lines):
        line = lines[cursor]
        if not line.startswith(">"):
            cursor += 1
            continue
        parsed = P69.P68.parse_epoch_v3(line)
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
            system = system_from_rinex_char(sat_token[:1])
            if system is None:
                raise fail(f"unsupported RINEX3 system {sat_token[:1]!r}")
            try:
                svid = int(sat_token[1:3])
            except ValueError as exc:
                raise fail(f"invalid RINEX3 satellite token {sat_token!r}") from exc
            obs_types = system_types.get(sat_token[:1], global_types)
            records, cursor = P69._record_lines(lines, cursor, len(obs_types))
            if system in {"SBAS", "NavIC"}:
                index.unselected_system_record_count += 1
                index.unselected_system_census[system] = index.unselected_system_census.get(system, 0) + 1
                # The source recognizes these systems, but signal_policy has
                # no selected pseudorange signal. Their records are consumed
                # and deliberately absent from the matching index.
                continue
            entries: list[tuple[str, int, str, Any, float]] = []
            channel = glonass_channels.get(svid) if system == "GLONASS" else None
            for obs_index, obs_type in enumerate(obs_types):
                if not obs_type or obs_type[0] not in {"C", "P"}:
                    continue
                value = P69.observation_value(records, obs_index, version)
                signal = P69.P68.rinex_signal(system, obs_type, channel)
                if signal is not None:
                    entries.append((system, svid, obs_type, signal, value))
            P69.P68.add_base_epoch(index, epoch_time, entries)
    all_times = [time for values in index.exact_streams.values() for time in values]
    if all_times:
        index.time_min_s = min(all_times)
        index.time_max_s = max(all_times)
    return index


def route_report(route: str, raw_payload: bytes, base_payload: bytes) -> dict[str, Any]:
    raw = P69.P68.parse_raw_csv(raw_payload)
    base = parse_rinex(base_payload)
    classification = P69.P68.classify_rows(raw, base)
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
            "unselected_system_record_count": base.unselected_system_record_count,
            "unselected_system_census": dict(sorted(base.unselected_system_census.items())),
            "time_min_s": base.time_min_s,
            "time_max_s": base.time_max_s,
            "glonass_channels": {str(key): value for key, value in sorted(base.glonass_channels.items())},
        },
        "classification": classification,
    }


def run_audit(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    phase70_freeze = verify_freeze()
    phase69_freeze = P69.load_object(P69.FREEZE, "Phase69 freeze")
    output_root = output_root.resolve()
    P69.P68.P65.reject_forbidden(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty Phase70 output: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    reports: dict[str, Any] = {}
    read_counts = {"raw_device_gnss": 0, "base_rinex": 0}
    try:
        for route in ROUTES:
            raw_pin = phase69_freeze["raw_inputs"][route]
            base_pin = phase69_freeze["base_inputs"][route]
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
            "schema_version": "smartphone-r5-phase70-base-matching-taxonomy-sbas-recovery-result.v1",
            "phase": 70,
            "execution_label": "Luna Max",
            "status": "sealed-truth-free-matching-taxonomy-sbas-recovery",
            "truth_free": True,
            "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
            "manifest": {"path": relative(MANIFEST), "sha256": digest_file(MANIFEST)},
            "prior_phase69": {
                "failure_record": relative(PHASE69_FAILURE_RECORD),
                "failure_record_sha256": PHASE69_FAILURE_RECORD_SHA256,
                "failure_output_sha256": PHASE69_FAILURE_OUTPUT_SHA256,
                "reread": False,
                "partial_output_reused": False,
            },
            "routes": reports,
            "native_adopted_fgo_equality_claim": False,
            "proxy_scope_note": "Raw adopted and base selected counts are diagnostic proxies. This result does not claim equality to native adopted FGO factor populations.",
            "aggregate": {
                "adopted_rows": sum(report["classification"]["adopted_rows"] for report in reports.values()),
                "classification_counts": counts,
                "unselected_system_record_count": sum(report["base"]["unselected_system_record_count"] for report in reports.values()),
                "unselected_system_census": {
                    key: sum(report["base"]["unselected_system_census"].get(key, 0) for report in reports.values())
                    for key in sorted({key for report in reports.values() for key in report["base"]["unselected_system_census"]})
                },
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
                "phase69_partial_output_reuse": False,
            },
            "gate_policy": {
                "phase67_coverage_gate_unchanged": True,
                "native_correction_authorized": False,
                "canonicalization_authorized": False,
                "interpolation_change_authorized": False,
                "zero_point_782": "not evaluated",
            },
        }
        result_path = output_root / "phase70_base_matching_taxonomy_sbas_recovery_result.json"
        atomic_json(result_path, result)
        output_manifest = {
            "schema_version": "smartphone-r5-phase70-base-matching-taxonomy-sbas-recovery-output-manifest.v1",
            "phase": 70,
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
            "phase69_partial_output_reuse": False,
            "all_classification_sum_checks": result["aggregate"]["classification_sum_checks"],
        }
        atomic_json(output_root / "phase70_base_matching_taxonomy_sbas_recovery_output_manifest.json", output_manifest)
        return result
    except Phase70Error as exc:
        atomic_json(
            output_root / "phase70_base_matching_taxonomy_sbas_recovery_failure.json",
            {
                "schema_version": "smartphone-r5-phase70-base-matching-taxonomy-sbas-recovery-failure.v1",
                "status": "fail-closed",
                "error": str(exc),
                "read_accounting": {**read_counts, "truth_reads": 0, "solver_invocations": 0},
                "phase68_partial_output_reuse": False,
                "phase69_partial_output_reuse": False,
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
    except Phase70Error as exc:
        print(f"phase70 matching taxonomy SBAS recovery failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
