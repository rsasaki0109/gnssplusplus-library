#!/usr/bin/env python3
"""Validate a trajectory bundle without running positioning."""

from __future__ import annotations
import argparse, hashlib, json, math, os, re
from pathlib import Path

SCHEMA = "libgnsspp.trajectory_bundle.v1"
REQUIRED_ARTIFACTS = {
    "raw.pos", "accepted.pos", "raw.kml", "accepted.kml", "trajectory.png",
    "segments.csv", "ros2_metadata.json", "summary.json", "bundle.log",
}
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


def _finite_triplet(value) -> bool:
    return isinstance(value, list) and len(value) == 3 and all(
        isinstance(component, (int, float)) and math.isfinite(float(component))
        for component in value
    )


def _valid_digest_record(record, *, allow_missing: bool = False) -> bool:
    if not isinstance(record, dict):
        return False
    if allow_missing and record.get("state") == "missing":
        return record.get("bytes") is None and record.get("sha256") is None
    return (
        isinstance(record.get("bytes"), int)
        and record["bytes"] >= 0
        and isinstance(record.get("sha256"), str)
        and bool(SHA256_RE.fullmatch(record["sha256"]))
    )


def _finite_tree(value) -> bool:
    if isinstance(value, dict):
        return all(_finite_tree(child) for child in value.values())
    if isinstance(value, list):
        return all(_finite_tree(child) for child in value)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    return math.isfinite(float(value))


def run(argv=None) -> int:
    p = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME", "gnss trajectory-bundle-validate"))
    p.add_argument("bundle", type=Path)
    a = p.parse_args(argv)
    bundle = a.bundle.resolve()
    manifest_path = bundle / "manifest.json"
    try:
        m=json.loads(manifest_path.read_text())
        if m.get("schema_version") != SCHEMA: raise ValueError("unsupported schema")
        if m.get("state") != "usable": raise ValueError("bundle is not usable")
        run_metadata = m.get("run")
        if not isinstance(run_metadata, dict) or not isinstance(run_metadata.get("argv"), list) or not run_metadata.get("argv") or not all(isinstance(value, str) for value in run_metadata["argv"]):
            raise ValueError("run invocation missing")
        if not isinstance(run_metadata.get("software_revision"), str) or not run_metadata["software_revision"]:
            raise ValueError("software revision missing")
        inputs = m.get("inputs")
        if not isinstance(inputs, dict) or not _valid_digest_record(inputs.get("solution")):
            raise ValueError("solution input digest missing")
        if inputs.get("reference") is not None and not _valid_digest_record(inputs.get("reference")):
            raise ValueError("reference input digest invalid")
        frames=m.get("frames", {})
        if not isinstance(frames, dict) or not isinstance(frames.get("target_frame"), str) or not frames["target_frame"].strip():
            raise ValueError("target frame missing")
        if not _finite_triplet(frames.get("antenna_to_target_lever_arm_m")):
            raise ValueError("finite three-axis lever arm missing")
        if not isinstance(frames.get("source_position"), str) or not frames["source_position"].strip():
            raise ValueError("source position frame missing")
        if not isinstance(frames.get("lever_arm_application"), str) or not frames["lever_arm_application"].strip():
            raise ValueError("lever-arm application contract missing")
        if not isinstance(frames.get("time_system"), str) or not frames["time_system"].strip():
            raise ValueError("time system missing")
        artifacts = m.get("artifacts")
        if not isinstance(artifacts, dict) or not REQUIRED_ARTIFACTS.issubset(artifacts):
            missing = sorted(REQUIRED_ARTIFACTS - set(artifacts or {}))
            raise ValueError(f"required artifact records missing: {', '.join(missing)}")
        for name, rec in artifacts.items():
            path_name = Path(name)
            if path_name.is_absolute() or path_name.name != name or name in (".", ".."):
                raise ValueError(f"unsafe artifact path: {name}")
            if not _valid_digest_record(rec, allow_missing=True):
                raise ValueError(f"artifact digest invalid: {name}")
            path = bundle / path_name
            if not path.is_file() or path.stat().st_size != rec["bytes"]:
                raise ValueError(f"artifact missing/size mismatch: {name}")
            if hashlib.sha256(path.read_bytes()).hexdigest() != rec["sha256"]:
                raise ValueError(f"artifact hash mismatch: {name}")
        gate = m.get("gate")
        if not isinstance(gate, dict) or not isinstance(gate.get("thresholds"), dict) or not isinstance(gate.get("failure_reasons"), list) or gate.get("status") != "pass" or gate.get("failure_reasons"):
            raise ValueError("stored consumer gate failed")
        summary_path = bundle / "summary.json"
        summary = json.loads(summary_path.read_text())
        if summary.get("schema_version") != SCHEMA or summary.get("state") != "usable":
            raise ValueError("summary contract mismatch")
        if summary.get("gate", {}).get("status") != "pass":
            raise ValueError("summary gate failed")
        if not isinstance(summary.get("metrics"), dict) or not _finite_tree(summary["metrics"]):
            raise ValueError("summary metrics are non-finite")
        if summary.get("gate") != gate or summary.get("metrics") != m.get("metrics") or summary.get("populations") != m.get("populations"):
            raise ValueError("manifest and summary disagree")
        label = m.get("label")
        if label == "ground_truth" and (inputs.get("reference") is None or not isinstance(summary.get("truth_role"), str) or "independent" not in summary["truth_role"]):
            raise ValueError("ground_truth lacks independent truth")
        if label not in ("ground_truth", "candidate_trajectory"):
            raise ValueError("unknown bundle label")
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        print(f"Trajectory bundle validation failed: {exc}"); return 1
    print("Trajectory bundle validation passed (no solver executed)"); return 0

if __name__ == "__main__": raise SystemExit(run())
