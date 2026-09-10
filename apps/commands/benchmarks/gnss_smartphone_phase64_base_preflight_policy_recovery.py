#!/usr/bin/env python3
"""Scorer-only Phase64 recovery of the Phase63 base preflight.

This command deliberately reads the sealed Phase63 result JSON once and
re-evaluates its recorded hashes/metrics.  It never opens the archive or a
materialized base file, and it does not invoke a solver or any payload reader.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase64_base_preflight_policy_recovery_freeze_v1.json"
FREEZE_SHA256 = "94487186338d85d289c64d04805254f95568020664e37d4402f1f9b238aee993"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase64_base_preflight_policy_recovery_manifest_v1.json"
RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase63_settings_integrity_recovery_result_v1.json"
RESULT_SHA256 = "d6dc56c00c89ddd4e38624bbfe36567a76632aecae3f3f87f529ab0c8f93799b"
OUTPUT = ROOT / "output/smartphone-r5/phase64-base-preflight-policy-recovery-v1"
SOURCE = Path(__file__).resolve()
SCHEMA = "smartphone-r5-phase64-base-preflight-policy-recovery.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase64-base-preflight-policy-recovery-manifest.v1"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)


class RecoveryError(ValueError):
    """Raised when the scorer-only recovery contract fails closed."""


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


def _verify_freeze() -> dict[str, Any]:
    freeze = _load_json(FREEZE, "Phase64 freeze")
    if freeze.get("schema_version") != "smartphone-r5-phase64-base-preflight-policy-recovery-freeze.v1":
        raise RecoveryError("Phase64 freeze schema mismatch")
    if freeze.get("status") != "source-only-frozen-before-phase64-result-read":
        raise RecoveryError("Phase64 freeze is not pre-result-read")
    if not FREEZE_SHA256 or _sha256(FREEZE) != FREEZE_SHA256:
        raise RecoveryError("Phase64 freeze hash mismatch")
    source = freeze.get("source_semantics")
    if not isinstance(source, dict):
        raise RecoveryError("Phase64 source semantics missing")
    if source.get("repository_commit") != "29923f9f370f09ebc00f96d8cca375007a18e7d5":
        raise RecoveryError("upstream commit mismatch")
    if source.get("preprocessing_m", {}).get("sha256") != "976629d187e7fab5868eb8e5676a4d40520eb23db1254f7320d3b4270d7dffcf":
        raise RecoveryError("preprocessing.m hash mismatch")
    if source.get("correct_pseudorange_m", {}).get("sha256") != "b0536ccff478b0aff253448ffb7a203c715b8064dd8dc85898e38f1f05d0441e":
        raise RecoveryError("correct_pseudorange.m hash mismatch")
    if freeze.get("route_order") != list(ROUTES):
        raise RecoveryError("Phase64 route order mismatch")
    manifest = _load_json(MANIFEST, "Phase64 manifest")
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("status") != "sealed-before-phase64-result-read":
        raise RecoveryError("Phase64 manifest schema/status mismatch")
    if manifest.get("freeze_record") != _relative(FREEZE) or manifest.get("freeze_record_sha256") != FREEZE_SHA256:
        raise RecoveryError("Phase64 manifest freeze pin mismatch")
    if manifest.get("evaluator_source") != _relative(SOURCE) or manifest.get("evaluator_source_sha256") != _sha256(SOURCE):
        raise RecoveryError("Phase64 evaluator hash mismatch")
    if manifest.get("phase63_result") != _relative(RESULT) or manifest.get("phase63_result_sha256") != RESULT_SHA256:
        raise RecoveryError("Phase63 result pin mismatch")
    if manifest.get("archive_reopen_count") != 0 or manifest.get("base_file_reread_count") != 0:
        raise RecoveryError("Phase64 no-reopen contract mismatch")
    return {"freeze": freeze, "manifest": manifest}


def _finite_xyz(value: Any) -> bool:
    return isinstance(value, list) and len(value) == 3 and all(
        isinstance(item, (int, float)) and math.isfinite(float(item)) for item in value
    )


def _read_result_once() -> tuple[dict[str, Any], str]:
    try:
        payload = RESULT.read_bytes()
    except OSError as exc:
        raise RecoveryError(f"cannot read sealed Phase63 result: {exc}") from exc
    digest = hashlib.sha256(payload).hexdigest()
    if digest != RESULT_SHA256:
        raise RecoveryError(f"Phase63 result hash mismatch: {digest} != {RESULT_SHA256}")
    try:
        result = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise RecoveryError("invalid sealed Phase63 result") from exc
    if not isinstance(result, dict):
        raise RecoveryError("Phase63 result is not an object")
    return result, digest


def _route_gate(route: str, expected: dict[str, Any], observed: dict[str, Any]) -> dict[str, Any]:
    settings = observed.get("settings")
    base = observed.get("base")
    central = observed.get("central")
    if not isinstance(settings, dict) or not isinstance(base, dict) or not isinstance(central, dict):
        return {"route": route, "passed": False, "reason": "missing sealed route summary"}
    settings_ok = (
        settings.get("Base1") == expected["Base1"]
        and settings.get("RINEX") == expected["RINEX"]
        and observed.get("member") == expected["member"]
    )
    hash_bytes_ok = (
        base.get("sha256") == expected["materialized_sha256"]
        and base.get("bytes") == expected["bytes"]
        and central.get("bytes") == expected["bytes"]
    )
    xyz_ok = _finite_xyz(base.get("approx_position_xyz_m"))
    dt = base.get("observed_interval_median_s")
    dt_ok = isinstance(dt, (int, float)) and math.isfinite(float(dt)) and any(abs(float(dt) - allowed) <= 1.0e-9 for allowed in (1.0, 15.0)) and abs(float(dt) - expected["observed_dt_s"]) <= 1.0e-9
    time_ok = bool(observed.get("course_window_overlap")) and bool(base.get("time_first_utc")) and bool(base.get("time_last_utc")) and int(base.get("epoch_count", 0)) > 0
    provenance_ok = settings_ok and xyz_ok and bool(expected["member"].split("/")[-1])
    passed = settings_ok and hash_bytes_ok and xyz_ok and dt_ok and time_ok and provenance_ok
    return {
        "route": route,
        "passed": passed,
        "settings_mapping": settings_ok,
        "hash_bytes": hash_bytes_ok,
        "finite_approx_position": xyz_ok,
        "observed_dt_exact": dt_ok,
        "time_span_overlap": time_ok,
        "station_provenance": provenance_ok,
        "header_station_id_informational": base.get("header_station_id"),
        "header_interval_informational": base.get("header_interval_s"),
        "header_rinex_version_informational": base.get("rinex_version"),
    }


def recover() -> dict[str, Any]:
    contract = _verify_freeze()
    result, result_digest = _read_result_once()
    if result.get("schema_version") != "smartphone-r5-phase63-settings-integrity-recovery-result.v1":
        raise RecoveryError("Phase63 result schema mismatch")
    if result.get("status") != "no-go-base-header-contract":
        raise RecoveryError("unexpected Phase63 result status")
    freeze = contract["freeze"]
    expected_routes = freeze.get("declared_base_members")
    observed_routes = result.get("routes")
    if not isinstance(expected_routes, dict) or not isinstance(observed_routes, dict):
        raise RecoveryError("route summaries missing")
    if set(observed_routes) != set(ROUTES):
        raise RecoveryError("Phase63 route set mismatch")
    route_gates = [_route_gate(route, expected_routes[route], observed_routes[route]) for route in ROUTES]
    aggregate = {
        "four_routes": len(route_gates) == 4,
        "all_route_gates": all(item["passed"] for item in route_gates),
        "settings_hash": result.get("settings", {}).get("sha256") == freeze["prior_phase62"].get("observed_settings_sha256"),
        "archive_hash_pinned": result.get("archive", {}).get("sha256") == "bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408",
        "result_materialized_hashes_only": True,
        "no_archive_or_base_reread": True,
    }
    read_accounting = {
        "phase64_result_json_read_count": 1,
        "phase64_archive_open_count": 0,
        "phase64_base_file_reread_count": 0,
        "phase64_raw_device_read_count": 0,
        "phase64_truth_read_count": 0,
        "phase64_mat_read_count": 0,
        "phase64_nav_read_count": 0,
        "phase64_solver_invocation_count": 0,
        "phase63_historical_archive_open_count_pinned": result.get("archive", {}).get("zip_open_count"),
        "phase63_historical_base_reads_pinned": result.get("read_accounting", {}).get("base_member_read_count_per_route"),
    }
    return {
        "schema_version": SCHEMA,
        "status": "pass-policy-recovery" if all(aggregate.values()) else "no-go-policy-recovery",
        "phase64_contract": {
            "freeze": _relative(FREEZE),
            "freeze_sha256": FREEZE_SHA256,
            "manifest": _relative(MANIFEST),
            "phase63_result": _relative(RESULT),
            "phase63_result_sha256": result_digest,
        },
        "policy": {
            "Base1_filename_selection": "settings Base1 selects ${Base1}_rnx2.obs",
            "dt_selection": "correct_pseudorange uses obsb.dt == 1.0 or 15.0, not header INTERVAL",
            "marker_name": "informational only; correct_pseudorange does not read MARKER NAME",
            "station_provenance": "Base1 + unique member basename + finite APPROX POSITION XYZ",
        },
        "routes": route_gates,
        "gates": aggregate,
        "read_accounting": read_accounting,
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
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--audit", action="store_true", required=True)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args(argv)
    try:
        value = recover()
        _write_json(args.output.resolve() / "phase64_base_preflight_policy_recovery.json", value)
        print(json.dumps({"status": value["status"], "output": _relative(args.output.resolve()), "phase64_result_reads": 1, "archive_reopens": 0}, sort_keys=True))
        return 0
    except RecoveryError as exc:
        print(f"phase64 policy recovery failed closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
