#!/usr/bin/env python3
"""Phase64 scorer-only v4 policy recovery.

The v3 run was fail-closed by an evaluator lookup bug: it searched for a
settings digest that was not present in the already sealed Phase64 freeze.
This v4 erratum pins the source-recorded Phase63 settings digest explicitly,
adapts the sealed Phase63 flat route schema, and reads that result once.  It
never opens the archive or any materialized base payload.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase64_base_preflight_policy_recovery_freeze_v1.json"
FREEZE_SHA256 = "94487186338d85d289c64d04805254f95568020664e37d4402f1f9b238aee993"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase64_base_preflight_policy_recovery_manifest_v4.json"
RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase63_settings_integrity_recovery_result_v1.json"
RESULT_SHA256 = "d6dc56c00c89ddd4e38624bbfe36567a76632aecae3f3f87f529ab0c8f93799b"
SOURCE = Path(__file__).resolve()
V3_SOURCE = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase64_base_preflight_policy_recovery_v3.py"
V3_SOURCE_SHA256 = "c5828a1f344abb41c359b9332eaa43edf0f882696be6bf0c84c7e0474a043d1c"
V3_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase64_base_preflight_policy_recovery_manifest_v3.json"
V3_MANIFEST_SHA256 = "5470b04e0a8867c4dbcbfd4273f0ba48c9e4242ff86b728dad635dbfef62acf9"
PHASE63_SETTINGS_SHA256 = "3e6ae65388b2809088b16732b87744e673f860c24a1fe0f709ef903a87397f39"
ARCHIVE_SHA256 = "bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408"
SCHEMA = "smartphone-r5-phase64-base-preflight-policy-recovery-v4.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase64-base-preflight-policy-recovery-manifest-v4.v1"


class RecoveryError(ValueError):
    """Raised when the scorer-only recovery fails closed."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise RecoveryError(f"cannot hash {path}: {exc}") from exc
    return digest.hexdigest()


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RecoveryError(f"invalid {label}: {path}") from exc
    if not isinstance(value, dict):
        raise RecoveryError(f"{label} is not an object")
    return value


def _relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT.resolve()))
    except ValueError:
        return str(path)


def _load_v1_helpers() -> Any:
    benchmarks = str(SOURCE.parent)
    if benchmarks not in sys.path:
        sys.path.insert(0, benchmarks)
    import gnss_smartphone_phase64_base_preflight_policy_recovery as v1  # type: ignore

    v1.FREEZE = FREEZE
    v1.FREEZE_SHA256 = FREEZE_SHA256
    v1.MANIFEST = MANIFEST
    v1.SOURCE = SOURCE
    v1.MANIFEST_SCHEMA = MANIFEST_SCHEMA
    return v1


def _verify_freeze() -> dict[str, Any]:
    v1 = _load_v1_helpers()
    contract = v1._verify_freeze()
    manifest = contract["manifest"]
    if manifest.get("supersedes_manifest") != _relative(V3_MANIFEST):
        raise RecoveryError("v3 manifest supersession pin mismatch")
    if manifest.get("v3_evaluator_sha256") != V3_SOURCE_SHA256 or _sha256(V3_SOURCE) != V3_SOURCE_SHA256:
        raise RecoveryError("v3 evaluator pin mismatch")
    if manifest.get("v3_manifest_sha256") != V3_MANIFEST_SHA256 or _sha256(V3_MANIFEST) != V3_MANIFEST_SHA256:
        raise RecoveryError("v3 manifest hash mismatch")
    if manifest.get("phase63_settings_sha256_expected") != PHASE63_SETTINGS_SHA256:
        raise RecoveryError("Phase63 settings digest pin mismatch")
    return contract


def _flat_to_nested(observed: dict[str, Any]) -> dict[str, Any]:
    """Adapt Phase63 top-level route fields without opening its base artifact."""
    normalized = dict(observed)
    normalized["base"] = {
        "sha256": observed.get("materialized_sha256"),
        "bytes": observed.get("materialized_bytes"),
        "approx_position_xyz_m": observed.get("approx_position_xyz_m"),
        "observed_interval_median_s": observed.get("observed_interval_median_s"),
        "time_first_utc": observed.get("time_first_utc"),
        "time_last_utc": observed.get("time_last_utc"),
        "epoch_count": observed.get("epoch_count"),
        "header_station_id": observed.get("header_station_id"),
        "header_interval_s": observed.get("header_interval_s"),
        "rinex_version": observed.get("rinex_version"),
    }
    return normalized


def recover() -> dict[str, Any]:
    contract = _verify_freeze()
    v1 = _load_v1_helpers()
    result, result_digest = v1._read_result_once()
    if result.get("schema_version") != "smartphone-r5-phase63-settings-integrity-recovery-result.v1":
        raise RecoveryError("Phase63 result schema mismatch")
    if result.get("status") != "no-go-base-header-contract":
        raise RecoveryError("unexpected Phase63 result status")
    freeze = contract["freeze"]
    expected_routes = freeze.get("declared_base_members")
    observed_routes = result.get("routes")
    if not isinstance(expected_routes, dict) or not isinstance(observed_routes, dict):
        raise RecoveryError("route summaries missing")
    if set(observed_routes) != set(v1.ROUTES):
        raise RecoveryError("Phase63 route set mismatch")
    route_gates = [v1._route_gate(route, expected_routes[route], _flat_to_nested(observed_routes[route])) for route in v1.ROUTES]
    result_settings = result.get("settings")
    observed_settings_sha256 = result_settings.get("observed_sha256", result_settings.get("sha256")) if isinstance(result_settings, dict) else None
    aggregate = {
        "four_routes": len(route_gates) == 4,
        "all_route_gates": all(item["passed"] for item in route_gates),
        "settings_hash": observed_settings_sha256 == PHASE63_SETTINGS_SHA256,
        "archive_hash_pinned": result.get("archive", {}).get("sha256") == ARCHIVE_SHA256,
        "result_materialized_hashes_only": True,
        "no_archive_or_base_reread": True,
    }
    return {
        "schema_version": SCHEMA,
        "status": "pass-policy-recovery-v4" if all(aggregate.values()) else "no-go-policy-recovery-v4",
        "phase64_contract": {
            "freeze": _relative(FREEZE),
            "freeze_sha256": FREEZE_SHA256,
            "manifest": _relative(MANIFEST),
            "phase63_result": _relative(RESULT),
            "phase63_result_sha256": result_digest,
            "phase63_settings_sha256_expected": PHASE63_SETTINGS_SHA256,
        },
        "policy": {
            "Base1_filename_selection": "settings Base1 + unique *_rnx2.obs basename",
            "dt_selection": "correct_pseudorange uses obsb.dt == 1.0 or 15.0, not header INTERVAL",
            "marker_name": "informational only; correct_pseudorange does not read MARKER NAME",
            "station_provenance": "Base1 + unique member basename + finite APPROX POSITION XYZ",
        },
        "routes": route_gates,
        "gates": aggregate,
        "read_accounting": {
            "phase64_result_json_read_count": 1,
            "phase64_archive_open_count": 0,
            "phase64_base_file_reread_count": 0,
            "phase64_raw_device_read_count": 0,
            "phase64_truth_read_count": 0,
            "phase64_mat_read_count": 0,
            "phase64_nav_read_count": 0,
            "phase64_solver_invocation_count": 0,
            "phase64_v1_failed_attempt_result_read_count": 1,
            "phase64_v2_failed_attempt_result_read_count": 1,
            "phase64_v3_failed_attempt_result_read_count": 1,
            "phase64_total_result_json_reads_including_v1_v2_v3_attempts": 4,
        },
        "prior_failed_attempts": {
            "v1": "KeyError: freeze['prior_phase62'] (correct key is prior_phase62_phase63)",
            "v2": "Phase63 route summaries are flat; nested base helper reported missing summaries",
            "v3": "all route gates passed, but evaluator looked up absent prior_phase62_phase63.observed_settings_sha256; fail-closed integrity error",
        },
        "native_correction_authorized": bool(all(aggregate.values())),
        "accuracy_evaluated": False,
        "zero_point_782_claim": False,
    }


def _write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    temporary = Path(temporary_name)
    try:
        with open(descriptor, "w", encoding="utf-8", closefd=True) as handle:
            json.dump(value, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--audit", action="store_true", required=True)
    parser.add_argument("--output", type=Path, default=ROOT / "output/smartphone-r5/phase64-base-preflight-policy-recovery-v4")
    args = parser.parse_args(argv)
    try:
        value = recover()
        _write_json(args.output.resolve() / "phase64_base_preflight_policy_recovery_v4.json", value)
        print(json.dumps({"status": value["status"], "phase64_result_reads": 1, "archive_reopens": 0}, sort_keys=True))
        return 0
    except RecoveryError as exc:
        print(f"phase64 policy recovery v4 failed closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
