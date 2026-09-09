#!/usr/bin/env python3
"""Phase63 recovery of the Phase62 settings-member integrity contract.

The Phase62 evaluator is imported unchanged and reused only after this
Phase63-specific freeze/manifest is verified.  ``--verify-freeze`` never
opens the archive.  ``--audit`` permits one archive open, one settings read,
and one stream/materialization read for each declared base RINEX member.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase63_settings_integrity_recovery_freeze_v1.json"
FREEZE_SHA256 = "41f0218caf540ff4fb81efcc78f048de62e3a3953f1e16a8a04a838303c40bca"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase63_settings_integrity_recovery_manifest_v1.json"
SOURCE = Path(__file__).resolve()
REUSED_SOURCE = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase62_raw_base_preflight.py"
REUSED_SOURCE_SHA256 = "a392ddeb438399817b13345b6c4b69a30943b4f9d9111536715752f77621f4b5"
ARCHIVE = ROOT / "data/gsdc2023/cache/dataset_2023.zip"
OUTPUT = ROOT / "output/smartphone-r5/phase63-settings-integrity-recovery-v1"

ARCHIVE_SHA256 = "bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408"
SETTINGS_SHA256 = "3e6ae65388b2809088b16732b87744e673f860c24a1fe0f709ef903a87397f39"
SETTINGS_MEMBER = "dataset_2023/settings_train.csv"
SCHEMA = "smartphone-r5-phase63-settings-integrity-recovery.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase63-settings-integrity-recovery-manifest.v1"


class RecoveryError(ValueError):
    """Raised when the Phase63 immutable recovery contract fails."""


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
    """Verify Phase63 records without opening archive or payload members."""
    freeze = _load_json(FREEZE, "Phase63 freeze")
    if freeze.get("schema_version") != "smartphone-r5-phase63-settings-integrity-recovery-freeze.v1":
        raise RecoveryError("Phase63 freeze schema mismatch")
    if freeze.get("status") != "source-only-frozen-before-phase63-archive-read":
        raise RecoveryError("Phase63 freeze is not pre-archive-read")
    prior = freeze.get("prior_phase62")
    if not isinstance(prior, dict) or prior.get("observed_settings_sha256") != SETTINGS_SHA256:
        raise RecoveryError("observed settings SHA differs from recovery contract")
    source = freeze.get("source_contract")
    if not isinstance(source, dict) or source.get("upstream_commit") != "29923f9f370f09ebc00f96d8cca375007a18e7d5" or source.get("correct_pseudorange_sha256") != "b0536ccff478b0aff253448ffb7a203c715b8064dd8dc85898e38f1f05d0441e":
        raise RecoveryError("source contract mismatch")
    manifest = _load_json(MANIFEST, "Phase63 manifest")
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("status") != "sealed-before-phase63-base-preflight":
        raise RecoveryError("Phase63 manifest schema/status mismatch")
    if manifest.get("freeze_record") != _relative(FREEZE):
        raise RecoveryError("Phase63 manifest freeze path mismatch")
    if not FREEZE_SHA256 or _sha256(FREEZE) != FREEZE_SHA256 or manifest.get("freeze_record_sha256") != FREEZE_SHA256:
        raise RecoveryError("Phase63 freeze hash mismatch")
    if manifest.get("evaluator_source") != _relative(SOURCE) or _sha256(SOURCE) != manifest.get("evaluator_source_sha256"):
        raise RecoveryError("Phase63 evaluator hash mismatch")
    if _sha256(REUSED_SOURCE) != REUSED_SOURCE_SHA256 or manifest.get("reused_evaluator_sha256") != REUSED_SOURCE_SHA256:
        raise RecoveryError("reused Phase62 evaluator hash mismatch")
    archive = manifest.get("archive")
    if not isinstance(archive, dict) or archive.get("path") != _relative(ARCHIVE) or archive.get("sha256") != ARCHIVE_SHA256:
        raise RecoveryError("Phase63 archive contract mismatch")
    if archive.get("settings_member") != SETTINGS_MEMBER or archive.get("settings_sha256") != SETTINGS_SHA256:
        raise RecoveryError("Phase63 settings contract mismatch")
    if manifest.get("route_order") != [
        "2021-03-16-18-59-us-ca-mtv-a/pixel5",
        "2021-08-24-20-32-us-ca-mtv-h/pixel5",
        "2022-04-01-18-22-us-ca-lax-t/pixel5",
        "2023-03-08-21-34-us-ca-mtv-u/pixel5",
    ]:
        raise RecoveryError("Phase63 route order mismatch")
    read_contract = manifest.get("read_contract")
    if not isinstance(read_contract, dict) or read_contract.get("archive_open_count") != 1 or read_contract.get("settings_member_read_count") != 1 or read_contract.get("base_member_read_count_per_route") != 1 or read_contract.get("forbidden_member_read_count") != 0:
        raise RecoveryError("Phase63 read contract mismatch")
    return {"freeze": freeze, "manifest": manifest}


def _load_reused() -> Any:
    benchmarks = str(SOURCE.parent)
    if benchmarks not in sys.path:
        sys.path.insert(0, benchmarks)
    import gnss_smartphone_phase62_raw_base_preflight as reused  # type: ignore

    reused.FREEZE = FREEZE
    reused.FREEZE_SHA256 = FREEZE_SHA256
    reused.MANIFEST = MANIFEST
    reused.ARCHIVE_SHA256 = ARCHIVE_SHA256
    reused.SETTINGS_SHA256 = SETTINGS_SHA256
    reused.SCHEMA = SCHEMA
    reused.OUTPUT = OUTPUT
    return reused


def run_audit(archive_path: Path = ARCHIVE, output_root: Path = OUTPUT) -> dict[str, Any]:
    contract = _verify_freeze()
    reused = _load_reused()
    result = reused._audit(archive_path.resolve(), output_root.resolve(), contract)
    result["phase63_contract"] = {
        "freeze_record": _relative(FREEZE),
        "freeze_record_sha256": FREEZE_SHA256,
        "settings_sha256": SETTINGS_SHA256,
        "reused_evaluator_sha256": REUSED_SOURCE_SHA256,
    }
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--verify-freeze", action="store_true")
    mode.add_argument("--audit", action="store_true")
    parser.add_argument("--archive", type=Path, default=ARCHIVE)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args(argv)
    try:
        _verify_freeze()
        if args.verify_freeze:
            print(json.dumps({"status": "verified", "archive_open_count": 0, "payload_reads": 0}, sort_keys=True))
            return 0
        result = run_audit(args.archive, args.output)
        path = args.output.resolve() / "phase63_settings_integrity_recovery.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps({"status": result["status"], "output": _relative(path), "archive_open_count": 1}, sort_keys=True))
        return 0
    except RecoveryError as exc:
        print(f"phase63 recovery failed closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
