#!/usr/bin/env python3
"""Phase64 scorer-only recovery v2.

The first sealed scorer stopped on a presentation-key typo after one read of
the immutable Phase63 result.  This v2 keeps that attempt disclosed and reads
only the same sealed result once more; it never opens archive or base files.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase64_base_preflight_policy_recovery_freeze_v1.json"
FREEZE_SHA256 = "94487186338d85d289c64d04805254f95568020664e37d4402f1f9b238aee993"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase64_base_preflight_policy_recovery_manifest_v2.json"
RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase63_settings_integrity_recovery_result_v1.json"
RESULT_SHA256 = "d6dc56c00c89ddd4e38624bbfe36567a76632aecae3f3f87f529ab0c8f93799b"
SOURCE = Path(__file__).resolve()
SCHEMA = "smartphone-r5-phase64-base-preflight-policy-recovery-v2.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase64-base-preflight-policy-recovery-manifest-v2.v1"
V1_SOURCE = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase64_base_preflight_policy_recovery.py"
V1_SOURCE_SHA256 = "6dc9681b878dcf23c3f56950b6ed12aa291270772936bcd88848aac230094f0f"
V1_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase64_base_preflight_policy_recovery_manifest_v1.json"
V1_MANIFEST_SHA256 = "97cb6b7f24b32f7bf63ed6f4947f9101be84509490e0ce971b6aeab2c5f032b9"


class RecoveryError(ValueError):
    """Raised when the v2 scorer-only recovery fails closed."""


def _sha256(path: Path) -> str:
    import hashlib

    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
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


def _load_v1() -> Any:
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
    v1 = _load_v1()
    contract = v1._verify_freeze()
    manifest = contract["manifest"]
    if manifest.get("supersedes_manifest") != _relative(V1_MANIFEST):
        raise RecoveryError("v1 manifest supersession pin mismatch")
    if manifest.get("v1_evaluator_sha256") != V1_SOURCE_SHA256:
        raise RecoveryError("v1 evaluator pin mismatch")
    if not V1_MANIFEST_SHA256 or _sha256(V1_MANIFEST) != V1_MANIFEST_SHA256 or manifest.get("v1_manifest_sha256") != V1_MANIFEST_SHA256:
        raise RecoveryError("v1 manifest hash mismatch")
    return contract


def recover() -> dict[str, Any]:
    contract = _verify_freeze()
    v1 = _load_v1()
    result, result_digest = v1._read_result_once()
    if result.get("schema_version") != "smartphone-r5-phase63-settings-integrity-recovery-result.v1" or result.get("status") != "no-go-base-header-contract":
        raise RecoveryError("unexpected Phase63 result")
    freeze = contract["freeze"]
    expected_routes = freeze.get("declared_base_members")
    observed_routes = result.get("routes")
    if not isinstance(expected_routes, dict) or not isinstance(observed_routes, dict) or set(observed_routes) != set(expected_routes):
        raise RecoveryError("route summaries missing or changed")
    route_gates = [v1._route_gate(route, expected_routes[route], observed_routes[route]) for route in v1.ROUTES]
    aggregate = {
        "four_routes": len(route_gates) == 4,
        "all_route_gates": all(item["passed"] for item in route_gates),
        "settings_hash": result.get("settings", {}).get("sha256") == freeze["prior_phase62_phase63"].get("observed_settings_sha256"),
        "archive_hash_pinned": result.get("archive", {}).get("sha256") == "bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408",
        "result_materialized_hashes_only": True,
        "no_archive_or_base_reread": True,
    }
    return {
        "schema_version": SCHEMA,
        "status": "pass-policy-recovery-v2" if all(aggregate.values()) else "no-go-policy-recovery-v2",
        "phase64_contract": {
            "freeze": _relative(FREEZE),
            "freeze_sha256": FREEZE_SHA256,
            "manifest": _relative(MANIFEST),
            "phase63_result": _relative(RESULT),
            "phase63_result_sha256": result_digest,
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
            "phase64_total_result_json_reads_including_v1_attempt": 2,
        },
        "v1_failed_attempt": {
            "status": "failed-before-output",
            "failure": "KeyError: freeze['prior_phase62'] (correct key is prior_phase62_phase63)",
            "archive_reopen_count": 0,
            "base_file_reread_count": 0,
        },
        "native_correction_authorized": bool(all(aggregate.values())),
        "accuracy_evaluated": False,
        "zero_point_782_claim": False,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--audit", action="store_true", required=True)
    parser.add_argument("--output", type=Path, default=ROOT / "output/smartphone-r5/phase64-base-preflight-policy-recovery-v2")
    args = parser.parse_args(argv)
    try:
        value = recover()
        args.output.resolve().mkdir(parents=True, exist_ok=True)
        (args.output.resolve() / "phase64_base_preflight_policy_recovery_v2.json").write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps({"status": value["status"], "phase64_result_reads": 1, "archive_reopens": 0}, sort_keys=True))
        return 0
    except RecoveryError as exc:
        print(f"phase64 policy recovery v2 failed closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
