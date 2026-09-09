#!/usr/bin/env python3
"""Truth-free native/WLS lane stability selector.

The selector is deliberately small and has no route, device-model, or truth
input.  It proves both lane artifacts first, then chooses the native
Galileo-E1/Hatch segment output only when every reported segment is stable;
otherwise it chooses the raw handset WLS position.  Every published output
is written atomically and the selector manifest records the exact input
hashes and the stability evidence used for the decision.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import sys
from typing import Any

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_kaggle as kaggle  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-wls-stability-selector.v1"
SELECTOR_MANIFEST_SCHEMA_VERSION = "smartphone-r5-wls-stability-selector-manifest.v1"
SEGMENT_STABILITY_SCHEMA_VERSION = "smartphone-segment-stability-gate.v1"
WLS_MANIFEST_SCHEMA_VERSION = "smartphone-wls-position-manifest.v1"
WLS_SUMMARY_SCHEMA_VERSION = "smartphone-wls-position.v1"


class StabilitySelectorError(ValueError):
    """Raised when a truth-free lane contract cannot be proven."""


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise StabilitySelectorError(f"missing artifact: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise StabilitySelectorError(f"failed to hash artifact {path}: {exc}") from exc
    return digest.hexdigest()


def _artifact(path: Path) -> dict[str, Any]:
    return {"path": str(path), "sha256": _sha256(path), "bytes": path.stat().st_size}


def _load_json(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file():
        raise StabilitySelectorError(f"missing {label}: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise StabilitySelectorError(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise StabilitySelectorError(f"{label} must be a JSON object: {path}")
    return payload


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    smoother._atomic_write(
        path,
        (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"),
    )


def _finite(value: Any, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise StabilitySelectorError(f"{label} is not finite") from exc
    if not math.isfinite(number):
        raise StabilitySelectorError(f"{label} is not finite")
    return number


def _validate_segment_report(path: Path) -> dict[str, Any]:
    """Validate a truth-free segment report and return its decision summary."""

    report = _load_json(path, "native segment-stability report")
    if report.get("schema_version") != SEGMENT_STABILITY_SCHEMA_VERSION:
        raise StabilitySelectorError("segment-stability report schema is invalid")
    if report.get("truth_used") is not False:
        raise StabilitySelectorError("segment-stability report is not truth-free")
    policy = report.get("decision_policy")
    if policy is not None:
        if not isinstance(policy, dict) or policy.get("truth_used") is not False:
            raise StabilitySelectorError("segment-stability decision policy is not truth-free")
    population = report.get("population")
    segments = report.get("segments")
    if not isinstance(population, dict) or not isinstance(segments, list) or not segments:
        raise StabilitySelectorError("segment-stability report has no segment population")
    required_population = (
        "segment_count",
        "stable_segment_count",
        "unstable_segment_count",
        "fallback_epochs",
    )
    for key in required_population:
        if key not in population:
            raise StabilitySelectorError(f"segment population lacks {key}")
        if isinstance(population[key], bool) or not isinstance(population[key], int):
            raise StabilitySelectorError(f"segment population {key} is not an integer")
        if population[key] < 0:
            raise StabilitySelectorError(f"segment population {key} is negative")
    stable_flags: list[bool] = []
    normalized_segments: list[dict[str, Any]] = []
    for index, segment in enumerate(segments):
        if not isinstance(segment, dict):
            raise StabilitySelectorError(f"segment {index} is not an object")
        stable = segment.get("stable")
        if not isinstance(stable, bool):
            raise StabilitySelectorError(f"segment {index} has no boolean stable flag")
        stable_flags.append(stable)
        for key in (
            "segment_id",
            "start_index",
            "end_index",
            "epoch_count",
            "rejected_epochs",
            "max_consecutive_rejects",
        ):
            if key not in segment:
                raise StabilitySelectorError(f"segment {index} lacks {key}")
            if isinstance(segment[key], bool) or not isinstance(segment[key], int):
                raise StabilitySelectorError(f"segment {index} {key} is not an integer")
            if segment[key] < 0:
                raise StabilitySelectorError(f"segment {index} {key} is negative")
        for key in ("reject_fraction", "max_prediction_duration_s"):
            if key not in segment:
                raise StabilitySelectorError(f"segment {index} lacks {key}")
            _finite(segment[key], f"segment {index} {key}")
        innovation = segment.get("max_normalized_innovation_sigma")
        if innovation is not None:
            _finite(innovation, f"segment {index} max_normalized_innovation_sigma")
        normalized_segments.append(
            {
                "segment_id": segment["segment_id"],
                "stable": stable,
                "start_index": segment["start_index"],
                "end_index": segment["end_index"],
                "epoch_count": segment["epoch_count"],
                "rejected_epochs": segment["rejected_epochs"],
                "reject_fraction": float(segment["reject_fraction"]),
                "max_consecutive_rejects": segment["max_consecutive_rejects"],
                "max_prediction_duration_s": float(segment["max_prediction_duration_s"]),
                "max_normalized_innovation_sigma": (
                    None if innovation is None else float(innovation)
                ),
                "decision": segment.get("decision"),
                "reason": segment.get("reason"),
            }
        )
    expected_count = len(segments)
    if population["segment_count"] != expected_count:
        raise StabilitySelectorError("segment count does not match segment stats")
    if population["stable_segment_count"] != sum(stable_flags):
        raise StabilitySelectorError("stable segment count does not match segment stats")
    if population["unstable_segment_count"] != expected_count - sum(stable_flags):
        raise StabilitySelectorError("unstable segment count does not match segment stats")
    return {
        "path": str(path),
        "sha256": _sha256(path),
        "population": {
            key: population[key]
            for key in (
                "segment_count",
                "stable_segment_count",
                "unstable_segment_count",
                "fallback_epochs",
            )
        },
        "segments": normalized_segments,
        "all_segments_stable": all(stable_flags),
    }


def _verify_artifact_hash(artifact: Any, path: Path, label: str) -> None:
    if not isinstance(artifact, dict):
        raise StabilitySelectorError(f"WLS manifest lacks {label} artifact")
    expected = artifact.get("sha256")
    if not isinstance(expected, str) or not expected:
        raise StabilitySelectorError(f"WLS manifest lacks {label} hash")
    actual = _sha256(path)
    if actual != expected:
        raise StabilitySelectorError(f"WLS {label} hash mismatch")


def _validate_wls_manifest(
    manifest_path: Path,
    position_path: Path,
    device_path: Path,
) -> dict[str, Any]:
    """Fail closed unless the WLS integrity and artifact contract is intact."""

    manifest = _load_json(manifest_path, "WLS integrity manifest")
    if manifest.get("schema_version") != WLS_MANIFEST_SCHEMA_VERSION:
        raise StabilitySelectorError("WLS manifest schema is invalid")
    if manifest.get("truth_free") is not True or manifest.get("truth_used") is not False:
        raise StabilitySelectorError("WLS manifest is not explicitly truth-free")
    inputs = manifest.get("inputs")
    if not isinstance(inputs, dict) or inputs.get("ground_truth") is not None:
        raise StabilitySelectorError("WLS manifest contains a ground-truth input")
    device_artifact = inputs.get("device_gnss")
    if not isinstance(device_artifact, dict):
        raise StabilitySelectorError("WLS manifest lacks device GNSS input")
    expected_device_hash = device_artifact.get("sha256")
    if not isinstance(expected_device_hash, str) or not expected_device_hash:
        raise StabilitySelectorError("WLS manifest lacks device GNSS hash")
    actual_device_hash = _sha256(device_path)
    if actual_device_hash != expected_device_hash:
        raise StabilitySelectorError("WLS device GNSS input hash mismatch")
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise StabilitySelectorError("WLS manifest lacks artifacts")
    _verify_artifact_hash(artifacts.get("position"), position_path, "position")
    geodetic_path = Path(str(dict(artifacts.get("geodetic") or {}).get("path", "")))
    summary_path = Path(str(dict(artifacts.get("summary") or {}).get("path", "")))
    if not geodetic_path.is_file() or not summary_path.is_file():
        raise StabilitySelectorError("WLS manifest references missing companion artifact")
    _verify_artifact_hash(artifacts.get("geodetic"), geodetic_path, "geodetic")
    _verify_artifact_hash(artifacts.get("summary"), summary_path, "summary")
    summary = _load_json(summary_path, "WLS summary")
    if summary.get("schema_version") != WLS_SUMMARY_SCHEMA_VERSION:
        raise StabilitySelectorError("WLS summary schema is invalid")
    if summary.get("truth_free") is not True or summary.get("inputs", {}).get("ground_truth") is not None:
        raise StabilitySelectorError("WLS summary is not truth-free")
    validation = summary.get("validation")
    if not isinstance(validation, dict):
        raise StabilitySelectorError("WLS summary lacks validation")
    if validation.get("all_epoch_rows_consistent") is not True:
        raise StabilitySelectorError("WLS integrity validation rejected inconsistent epochs")
    if validation.get("all_selected_rows_finite") is not True:
        raise StabilitySelectorError("WLS integrity validation rejected non-finite epochs")
    counts = validation.get("classification_counts")
    if not isinstance(counts, dict):
        raise StabilitySelectorError("WLS summary lacks classification counts")
    for key, value in counts.items():
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise StabilitySelectorError(f"WLS classification count is invalid: {key}")
        if value != 0:
            raise StabilitySelectorError(f"WLS integrity classification is non-zero: {key}")
    return {
        "manifest": _artifact(manifest_path),
        "summary": _artifact(summary_path),
        "position": _artifact(position_path),
        "device_gnss": {
            "path": str(device_path),
            "sha256": actual_device_hash,
        },
        "validation": validation,
        "populations": summary.get("populations"),
    }


def _validate_position_keys(
    position_path: Path,
    device_path: Path,
    *,
    skip_epochs: int,
    label: str,
) -> dict[str, Any]:
    if isinstance(skip_epochs, bool) or not isinstance(skip_epochs, int) or skip_epochs < 0:
        raise StabilitySelectorError("skip_epochs must be a non-negative integer")
    try:
        positions = smoother._read_positions(position_path, 18)
        device_epochs = smoother._read_device_epochs(device_path, skip_epochs)
    except (OSError, ValueError, smoother.SmootherError) as exc:
        raise StabilitySelectorError(f"{label} key validation failed") from exc
    timestamps = [row.timestamp_ms for row in positions]
    if timestamps != device_epochs:
        raise StabilitySelectorError(f"{label} position keys do not match device epochs")
    for index, row in enumerate(positions):
        values = (*[float(value) for value in row.ecef], row.latitude, row.longitude, row.height)
        if not all(math.isfinite(value) for value in values):
            raise StabilitySelectorError(f"{label} epoch {index} contains a non-finite value")
        if not -90.0 <= row.latitude <= 90.0 or not -180.0 <= row.longitude <= 180.0:
            raise StabilitySelectorError(f"{label} epoch {index} has an invalid geodetic coordinate")
    return {
        "position": _artifact(position_path),
        "device_epoch_count": len(device_epochs),
        "first_timestamp_ms": device_epochs[0],
        "last_timestamp_ms": device_epochs[-1],
        "exact_key_match": True,
        "all_values_finite": True,
    }


def _position_reference(artifact: Any, path: Path, label: str) -> dict[str, Any]:
    if isinstance(artifact, dict):
        expected = artifact.get("sha256")
        if expected is not None and expected != _sha256(path):
            raise StabilitySelectorError(f"{label} input hash mismatch")
    return _artifact(path)


def select_and_publish(
    native_position_path: Path,
    native_stability_report_path: Path,
    wls_position_path: Path,
    wls_manifest_path: Path,
    device_gnss_path: Path,
    output_dir: Path,
    *,
    phone: str,
    dataset_id: str | None = None,
    skip_epochs: int = 1,
    selected_position_name: str = "selected.pos",
    selected_submission_name: str = "submission.csv",
    manifest_name: str = "selector_manifest.json",
) -> dict[str, Any]:
    """Validate both lanes and atomically publish the truth-free selection.

    The function intentionally has no truth parameter.  ``phone`` is only the
    Kaggle submission key and ``dataset_id`` is an audit label; neither can
    affect the lane decision.
    """

    native_stability = _validate_segment_report(native_stability_report_path)
    wls_integrity = _validate_wls_manifest(
        wls_manifest_path, wls_position_path, device_gnss_path
    )
    native_keys = _validate_position_keys(
        native_position_path,
        device_gnss_path,
        skip_epochs=skip_epochs,
        label="native",
    )
    wls_keys = _validate_position_keys(
        wls_position_path,
        device_gnss_path,
        skip_epochs=skip_epochs,
        label="WLS",
    )
    # Validate the native report's raw source hash when the report carries it;
    # this catches stale or hand-edited segment statistics without requiring
    # the selector to inspect any truth-bearing file.
    native_report = _load_json(native_stability_report_path, "native segment-stability report")
    raw_position = native_report.get("raw_position")
    native_raw_artifact = None
    if raw_position is not None:
        if not isinstance(raw_position, dict):
            raise StabilitySelectorError("native raw_position evidence is malformed")
        raw_path = Path(str(raw_position.get("path", "")))
        if not raw_path.is_file():
            raise StabilitySelectorError("native raw_position evidence is missing")
        native_raw_artifact = _position_reference(raw_position, raw_path, "native raw")

    all_stable = bool(native_stability["all_segments_stable"])
    selected_lane = "native_stable" if all_stable else "wls_raw"
    reason = (
        "all_native_segments_stable"
        if all_stable
        else "one_or_more_native_segments_unstable"
    )
    selected_source = native_position_path if all_stable else wls_position_path
    output_dir.mkdir(parents=True, exist_ok=True)
    selected_position = output_dir / selected_position_name
    selected_submission = output_dir / selected_submission_name
    selector_manifest_path = output_dir / manifest_name
    input_resolved = {
        native_position_path.resolve(),
        native_stability_report_path.resolve(),
        wls_position_path.resolve(),
        wls_manifest_path.resolve(),
        device_gnss_path.resolve(),
    }
    for output_path in (selected_position, selected_submission, selector_manifest_path):
        if output_path.resolve() in input_resolved:
            raise StabilitySelectorError(f"selector output collides with input: {output_path}")
    try:
        smoother._atomic_write(selected_position, selected_source.read_bytes())
    except OSError as exc:
        raise StabilitySelectorError("failed to atomically publish selected POS") from exc
    try:
        submission_manifest = kaggle.generate_submission(
            selected_position,
            selected_submission,
            phone,
            device_gnss_path=device_gnss_path,
            dataset_id=dataset_id,
            skip_epochs=skip_epochs,
            gps_utc_leap_seconds=18,
        )
    except (OSError, ValueError) as exc:
        raise StabilitySelectorError("failed to atomically publish selected submission") from exc
    submission_manifest_path = selected_submission.with_name(
        selected_submission.name + ".manifest.json"
    )
    if not submission_manifest_path.is_file():
        raise StabilitySelectorError("submission generator did not publish its manifest")
    manifest: dict[str, Any] = {
        "schema_version": SELECTOR_MANIFEST_SCHEMA_VERSION,
        "selector_schema_version": SCHEMA_VERSION,
        "truth_free": True,
        "truth_used": False,
        "decision": selected_lane,
        "reason": reason,
        "inputs": {
            "device_gnss": _artifact(device_gnss_path),
            "native_position": native_keys,
            "native_stability_report": native_stability,
            "native_raw_position": native_raw_artifact,
            "wls_position": wls_keys,
            "wls_integrity": wls_integrity,
        },
        "selection": {
            "rule": "native stable output iff every native segment is stable; otherwise raw WLS POS",
            "runtime_inputs": ["native_segment_stability_report", "WLS_integrity_manifest", "position_keys"],
            "device_model_used": False,
            "truth_used": False,
            "skip_epochs": skip_epochs,
            "selected_source": str(selected_source),
        },
        "artifacts": {
            "selected_position": _artifact(selected_position),
            "selected_submission": {
                **_artifact(selected_submission),
                "manifest": _artifact(submission_manifest_path),
            },
            "manifest": {"path": str(selector_manifest_path)},
        },
        "segment_stats": native_stability,
        "wls_integrity_summary": {
            "validation": wls_integrity["validation"],
            "populations": wls_integrity["populations"],
        },
    }
    _atomic_json(selector_manifest_path, manifest)
    # Keep the object returned to callers independent of a self-hash cycle;
    # the evaluator/outer manifest records this manifest's final hash.
    manifest["artifacts"]["manifest"] = _artifact(selector_manifest_path)
    return manifest


def load_selector_manifest(path: Path) -> dict[str, Any]:
    """Load and verify a published selector manifest and selected artifacts."""

    manifest = _load_json(path, "selector manifest")
    if manifest.get("schema_version") != SELECTOR_MANIFEST_SCHEMA_VERSION:
        raise StabilitySelectorError("selector manifest schema is invalid")
    if manifest.get("truth_free") is not True or manifest.get("truth_used") is not False:
        raise StabilitySelectorError("selector manifest is not truth-free")
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise StabilitySelectorError("selector manifest lacks artifacts")
    selected = artifacts.get("selected_position")
    submission = artifacts.get("selected_submission")
    if not isinstance(selected, dict) or not isinstance(submission, dict):
        raise StabilitySelectorError("selector manifest lacks selected artifacts")
    selected_path = Path(str(selected.get("path", "")))
    submission_path = Path(str(submission.get("path", "")))
    if _sha256(selected_path) != selected.get("sha256"):
        raise StabilitySelectorError("selected POS hash does not match selector manifest")
    if _sha256(submission_path) != submission.get("sha256"):
        raise StabilitySelectorError("selected submission hash does not match selector manifest")
    submission_manifest = submission.get("manifest")
    if not isinstance(submission_manifest, dict):
        raise StabilitySelectorError("selector manifest lacks submission manifest")
    submission_manifest_path = Path(str(submission_manifest.get("path", "")))
    if _sha256(submission_manifest_path) != submission_manifest.get("sha256"):
        raise StabilitySelectorError("selected submission manifest hash mismatch")
    return manifest


__all__ = [
    "SCHEMA_VERSION",
    "SELECTOR_MANIFEST_SCHEMA_VERSION",
    "StabilitySelectorError",
    "load_selector_manifest",
    "select_and_publish",
]
