#!/usr/bin/env python3
"""Evaluate a truth-free native/WLS stability selector.

The seven-route aggregate is assembled from the already audited development
routes.  A new Pixel7Pro route is selected from ZIP central-directory metadata
only, then its device GNSS/IMU/nav members are materialized without truth.
Galileo-E1/Hatch30, raw WLS, and the stability selector are published before
the new route's truth is opened once for scoring.  The designated holdout is
never opened, materialized, or scored.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import shutil
import sys
import tempfile
import time
from typing import Any
import zipfile

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_generalization as generalization  # noqa: E402
import gnss_smartphone_reacquisition_conservative_eval as conservative  # noqa: E402
import gnss_smartphone_reacquisition_eval as previous  # noqa: E402
import gnss_smartphone_segment_stability as segment_stability  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402
import gnss_smartphone_wls as wls  # noqa: E402
import gnss_smartphone_wls_eval as wls_eval  # noqa: E402
import gnss_smartphone_wls_stability_selector as selector  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-wls-stability-selector-evaluation.v1"
SELECTION_SCHEMA_VERSION = "smartphone-r5-wls-stability-selector-selection.v1"
DEFAULT_ARCHIVE = ROOT / "data" / "gsdc2023" / "cache" / "dataset_2023.zip"
DEFAULT_PROFILE = ROOT / "configs" / "benchmarks" / "smartphone_r5_gsdc2023.json"
DEFAULT_INVENTORY = (
    ROOT / "output" / "smartphone-r5" / "generalization-v6" / "archive_inventory.json"
)
DEFAULT_OUTPUT_DIR = ROOT / "output" / "smartphone-r5" / "wls-stability-selector-v1"
DEFAULT_SELECTION_RECORD = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_wls_stability_selector_selection.json"
)
DEFAULT_EXISTING_REPORT = (
    ROOT / "output" / "smartphone-r5" / "wls-position-v1" / "wls_position_report.json"
)
DEFAULT_DEVICE_FAMILY_REPORT = (
    ROOT
    / "output"
    / "smartphone-r5"
    / "wls-device-family-v1"
    / "wls_device_family_report.json"
)
HOLDOUT_ID = previous.HOLDOUT_ID
MAIN_ID = previous.MAIN_ID
PRIOR_NEW_ROUTE_ID = "2022-11-15-00-53-us-ca-mtv-a/pixel7pro"
NEW_VALIDATION_ID = "2023-05-16-19-55-us-ca-mtv-xe1/pixel7pro"
EXISTING_SIX_IDS = (
    "2021-01-04-21-50-us-ca-e1highway280driveroutea/mi8",
    "2022-08-04-20-07-us-ca-sjc-q/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel6pro",
    "2021-03-16-20-40-us-ca-mtv-b/pixel4xl",
    "2021-07-14-20-50-us-ca-mtv-e/pixel4",
    MAIN_ID,
)
KNOWN_SEVEN_IDS = (*EXISTING_SIX_IDS, PRIOR_NEW_ROUTE_ID)
USED_IDS = (*KNOWN_SEVEN_IDS, HOLDOUT_ID)
SEGMENT_MAX_REJECTS = 15
SEGMENT_MAX_PREDICTION_DURATION_S = 15.0
BASELINE_CONFIG = {
    "process_noise": 1.0,
    "measurement_floor_m": 1.0,
    "outlier_gate_sigma": 5.0,
    "segment_gap_s": 10.0,
}
DIAGNOSTIC_KEYS = wls_eval.DIAGNOSTIC_KEYS
SCREEN_FIELDS = (
    "SignalType",
    "ConstellationType",
    "CarrierFrequencyHz",
    "CodeType",
    "Svid",
    "RawPseudorangeMeters",
    "utcTimeMillis",
    "WlsPositionXEcefMeters",
    "WlsPositionYEcefMeters",
    "WlsPositionZEcefMeters",
)


class StabilitySelectorEvaluationError(ValueError):
    """Raised when the frozen selector evaluation contract is violated."""


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise StabilitySelectorEvaluationError(f"missing artifact: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise StabilitySelectorEvaluationError(f"failed to hash {path}") from exc
    return digest.hexdigest()


def _artifact(path: Path) -> dict[str, Any]:
    return {"path": str(path), "sha256": _sha256(path), "bytes": path.stat().st_size}


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    smoother._atomic_write(
        path,
        (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"),
    )


def _load_json(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file():
        raise StabilitySelectorEvaluationError(f"missing {label}: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise StabilitySelectorEvaluationError(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise StabilitySelectorEvaluationError(f"{label} must be a JSON object")
    return payload


def _safe_id(dataset_id: str) -> str:
    return dataset_id.replace("/", "__")


def _load_selection_record(path: Path) -> dict[str, Any]:
    record = _load_json(path, "selector selection record")
    if record.get("schema_version") != SELECTION_SCHEMA_VERSION:
        raise StabilitySelectorEvaluationError("selector selection record schema is invalid")
    if record.get("status") != "selection-frozen-before-evaluation":
        raise StabilitySelectorEvaluationError("selector selection record is not pre-evaluation frozen")
    selected = record.get("selected_candidate")
    if not isinstance(selected, dict) or selected.get("dataset_id") != NEW_VALIDATION_ID:
        raise StabilitySelectorEvaluationError("selector record does not freeze the expected route")
    sealed = record.get("sealed_data_policy")
    if not isinstance(sealed, dict) or sealed.get("designated_holdout_id") != HOLDOUT_ID:
        raise StabilitySelectorEvaluationError("selector record holdout differs from frozen profile")
    if any(sealed.get(key) for key in ("holdout_content_opened", "holdout_truth_opened", "holdout_materialized")):
        raise StabilitySelectorEvaluationError("selector record claims holdout access")
    excluded = tuple(record.get("previously_used_ids_excluded", ()))
    if any(dataset_id not in excluded for dataset_id in USED_IDS):
        raise StabilitySelectorEvaluationError("selector record does not exclude every used/sealed route")
    if NEW_VALIDATION_ID in excluded:
        raise StabilitySelectorEvaluationError("new validation appears in its own exclusion set")
    frozen = selected.get("central_directory_members")
    if not isinstance(frozen, dict):
        raise StabilitySelectorEvaluationError("selector record lacks frozen central metadata")
    return record


def _load_frozen_inventory(path: Path, record: dict[str, Any]) -> dict[str, Any]:
    archive = record.get("archive")
    if not isinstance(archive, dict):
        raise StabilitySelectorEvaluationError("selector record lacks archive metadata")
    expected = archive.get("central_directory_inventory_sha256")
    if not isinstance(expected, str) or not expected:
        raise StabilitySelectorEvaluationError("selector record lacks inventory hash")
    if _sha256(path) != expected:
        raise StabilitySelectorEvaluationError("central-directory inventory hash changed")
    inventory = _load_json(path, "central-directory inventory")
    inventory_archive = inventory.get("archive")
    if not isinstance(inventory_archive, dict):
        raise StabilitySelectorEvaluationError("inventory lacks archive metadata")
    if inventory_archive.get("central_directory_only") is not True:
        raise StabilitySelectorEvaluationError("inventory was not central-directory-only")
    if inventory_archive.get("member_content_read") is not False:
        raise StabilitySelectorEvaluationError("inventory claims member content access")
    return inventory


def _verify_candidate(inventory: dict[str, Any], record: dict[str, Any]) -> dict[str, Any]:
    rows = {
        str(row.get("dataset_id")): row
        for row in inventory.get("train", {}).get("records", [])
        if isinstance(row, dict)
    }
    candidate = rows.get(NEW_VALIDATION_ID)
    if candidate is None:
        raise StabilitySelectorEvaluationError("new validation is absent from central inventory")
    if candidate.get("dataset_id") in USED_IDS:
        raise StabilitySelectorEvaluationError("candidate collides with a used or sealed route")
    if candidate.get("phone") != "pixel7pro":
        raise StabilitySelectorEvaluationError("new validation is not exact Pixel7Pro")
    if not candidate.get("required_files_complete") or not candidate.get("broadcast_nav_present"):
        raise StabilitySelectorEvaluationError("candidate lacks complete phone files or unique nav")
    if int(candidate.get("broadcast_nav_duplicate_count", 0)) != 0:
        raise StabilitySelectorEvaluationError("candidate broadcast nav is duplicated")
    frozen = dict(record["selected_candidate"].get("central_directory_members", {}))
    actual_phone = dict(candidate.get("central_directory_files", {}))
    frozen_phone = {
        name: value
        for name, value in frozen.items()
        if name in generalization.REQUIRED_PHONE_MEMBERS
    }
    if frozen_phone != actual_phone:
        raise StabilitySelectorEvaluationError("candidate central phone metadata changed")
    if frozen.get("brdc.nav") != candidate.get("central_directory_broadcast_nav"):
        raise StabilitySelectorEvaluationError("candidate central nav metadata changed")
    return candidate


def _member_hashes_without_truth(
    archive_path: Path, candidate: dict[str, Any]
) -> dict[str, dict[str, Any]]:
    route, phone = candidate["route"], candidate["phone"]
    names = generalization._member_names(route, phone)
    selected = {
        key: names[key] for key in ("device_gnss", "device_imu", "broadcast_nav")
    }
    return generalization._discover_member_hashes(archive_path, selected)


def _extract_member_atomic(
    archive_path: Path,
    member: str,
    output: Path,
    *,
    expected_size: int | None = None,
) -> dict[str, Any]:
    """Extract one selected archive member atomically and hash its bytes."""

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary: str | None = None
    try:
        with zipfile.ZipFile(archive_path) as archive:
            matches = [info for info in archive.infolist() if info.filename == member]
            if len(matches) != 1 or matches[0].is_dir():
                raise StabilitySelectorEvaluationError(f"archive member is not unique: {member}")
            info = matches[0]
            if expected_size is not None and info.file_size != expected_size:
                raise StabilitySelectorEvaluationError(f"archive member size changed: {member}")
            descriptor, temporary = tempfile.mkstemp(
                prefix=f".{output.name}.", suffix=".tmp", dir=str(output.parent)
            )
            digest = hashlib.sha256()
            size = 0
            with os.fdopen(descriptor, "wb") as target, archive.open(info, "r") as source:
                for chunk in iter(lambda: source.read(1024 * 1024), b""):
                    target.write(chunk)
                    digest.update(chunk)
                    size += len(chunk)
                target.flush()
                os.fsync(target.fileno())
            os.replace(temporary, output)
            temporary = None
            return {"path": str(output), "sha256": digest.hexdigest(), "bytes": size}
    except (OSError, zipfile.BadZipFile) as exc:
        if isinstance(exc, StabilitySelectorEvaluationError):
            raise
        raise StabilitySelectorEvaluationError(f"failed to extract archive member: {member}") from exc
    finally:
        if temporary is not None:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def _materialize_truth_free_inputs(
    archive_path: Path,
    candidate: dict[str, Any],
    output_root: Path,
    archive_sha256: str,
    member_hashes: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    route, phone = candidate["route"], candidate["phone"]
    candidate_root = output_root / "materialized" / "routes" / route / phone
    inputs = candidate_root / "inputs"
    if inputs.exists() and any(inputs.iterdir()):
        raise StabilitySelectorEvaluationError(f"refusing to overwrite materialized inputs: {inputs}")
    member_names = generalization._member_names(route, phone)
    artifacts: dict[str, Any] = {}
    names = {
        "device_gnss": "device_gnss.csv",
        "device_imu": "device_imu.csv",
        "broadcast_nav": "brdc.nav",
    }
    for key, filename in names.items():
        metadata = _extract_member_atomic(
            archive_path,
            member_names[key],
            inputs / filename,
            expected_size=int(member_hashes[key]["file_size"]),
        )
        if metadata["sha256"] != member_hashes[key]["sha256"]:
            raise StabilitySelectorEvaluationError(f"materialized {key} hash mismatch")
        artifacts[key] = metadata
    manifest = {
        "schema_version": "smartphone-r5-wls-stability-selector-materialization.v1",
        "dataset_id": candidate["dataset_id"],
        "role": "development",
        "archive": {"path": str(archive_path), "sha256": archive_sha256},
        "central_directory_contract": {
            "selected_members_only": True,
            "member_content_read_before_generation": ["device_gnss", "device_imu", "broadcast_nav"],
            "truth_member_content_read_before_generation": False,
            "missing_duplicate_or_crc_error": "fail-closed",
        },
        "inputs": {
            key: artifacts[key] for key in ("device_gnss", "device_imu", "broadcast_nav")
        },
        "ground_truth": None,
        "truth_used_for_materialization": False,
        "truth_deferred_until_after_selector": True,
    }
    manifest_path = candidate_root / "materialization_manifest.json"
    _atomic_json(manifest_path, manifest)
    return {
        "root": candidate_root,
        "inputs": inputs,
        "manifest": manifest_path,
        "manifest_sha256": _sha256(manifest_path),
        "artifacts": artifacts,
    }


def _candidate_profile_without_truth(
    base_profile: dict[str, Any],
    candidate: dict[str, Any],
    archive_sha256: str,
    member_hashes: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    profile = copy.deepcopy(base_profile)
    profile["profile_id"] = (
        f"gsdc2023-wls-stability-selector-{candidate['dataset_id'].replace('/', '-')}-v1"
    )
    profile["archive"]["sha256"] = archive_sha256
    dataset = dict(profile["datasets"]["development"])
    profile["datasets"]["development"] = {
        "id": candidate["dataset_id"],
        "device_model": candidate["phone"],
        "device_gnss_sha256": member_hashes["device_gnss"]["sha256"],
        "device_imu_sha256": member_hashes["device_imu"]["sha256"],
        "ground_truth_sha256": "deferred-until-after-truth-free-selector",
        "broadcast_nav_sha256": member_hashes["broadcast_nav"]["sha256"],
        "skip_epochs": int(dataset.get("skip_epochs", 1)),
    }
    profile["wls_stability_selector"] = {
        "truth_free_selection": True,
        "truth_deferred_until_after_selector": True,
        "member_hashes_without_truth": member_hashes,
    }
    return profile


def _screen_device(device_path: Path, dataset_id: str) -> dict[str, Any]:
    """Screen E1 and WLS fields without reading ground truth."""

    total_rows = 0
    e1_rows = 0
    invalid_e1_rows = 0
    e1_epochs: set[int] = set()
    e1_svids: set[int] = set()
    wls_finite_rows = 0
    wls_invalid_rows = 0
    signal_rows: dict[str, int] = {}
    last_timestamp: int | None = None
    try:
        with device_path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            fields = set(reader.fieldnames or ())
            missing = sorted(set(SCREEN_FIELDS) - fields)
            if missing:
                raise StabilitySelectorEvaluationError(
                    "selected device member lacks capability fields: " + ", ".join(missing)
                )
            for row_number, raw in enumerate(reader, start=2):
                total_rows += 1
                signal = (raw.get("SignalType") or "").strip()
                signal_rows[signal] = signal_rows.get(signal, 0) + 1
                try:
                    timestamp = int(float(raw["utcTimeMillis"]))
                    if last_timestamp is not None and timestamp < last_timestamp:
                        raise StabilitySelectorEvaluationError(
                            f"device timestamp moved backwards at row {row_number}"
                        )
                    last_timestamp = timestamp
                except (KeyError, TypeError, ValueError) as exc:
                    raise StabilitySelectorEvaluationError(
                        f"device timestamp is invalid at row {row_number}"
                    ) from exc
                try:
                    wls_values = tuple(float(raw[field]) for field in (
                        "WlsPositionXEcefMeters",
                        "WlsPositionYEcefMeters",
                        "WlsPositionZEcefMeters",
                    ))
                except (KeyError, TypeError, ValueError):
                    wls_values = (math.nan, math.nan, math.nan)
                if all(math.isfinite(value) for value in wls_values):
                    wls_finite_rows += 1
                else:
                    wls_invalid_rows += 1
                if signal != "GAL_E1_C_P":
                    continue
                e1_rows += 1
                try:
                    valid = (
                        (raw["ConstellationType"] or "").strip() == "6"
                        and math.isfinite(float(raw["CarrierFrequencyHz"]))
                        and abs(float(raw["CarrierFrequencyHz"]) - 1575420000.0) <= 1.0
                        and not (raw["CodeType"] or "").strip()
                        and 1 <= int(float(raw["Svid"])) <= 36
                        and math.isfinite(float(raw["RawPseudorangeMeters"]))
                        and bool((raw["RawPseudorangeMeters"] or "").strip())
                    )
                    svid = int(float(raw["Svid"]))
                except (KeyError, TypeError, ValueError):
                    valid = False
                    svid = -1
                if valid:
                    e1_epochs.add(timestamp)
                    e1_svids.add(svid)
                else:
                    invalid_e1_rows += 1
    except OSError as exc:
        raise StabilitySelectorEvaluationError(f"failed to screen device GNSS: {device_path}") from exc
    result = {
        "dataset_id": dataset_id,
        "device_gnss": _artifact(device_path),
        "total_device_rows": total_rows,
        "signal_rows": dict(sorted(signal_rows.items())),
        "valid_galileo_e1_rows": e1_rows - invalid_e1_rows,
        "galileo_e1_rows": e1_rows,
        "galileo_e1_epochs": len(e1_epochs),
        "galileo_e1_svid_count": len(e1_svids),
        "invalid_galileo_e1_rows": invalid_e1_rows,
        "wls_finite_rows": wls_finite_rows,
        "wls_invalid_rows": wls_invalid_rows,
        "truth_opened": False,
        "screen_rule": {
            "signal_type": "GAL_E1_C_P",
            "constellation_type": "6",
            "carrier_frequency_hz": 1575420000,
            "carrier_frequency_tolerance_hz": 1.0,
            "code_type": "empty",
            "raw_pseudorange": "finite and present",
            "wls_ecef": "finite source fields; epoch-level consistency is WLS extractor contract",
        },
    }
    if result["valid_galileo_e1_rows"] < 1 or result["invalid_galileo_e1_rows"]:
        raise StabilitySelectorEvaluationError(f"Galileo E1 capability screen failed: {result}")
    if result["wls_invalid_rows"]:
        raise StabilitySelectorEvaluationError(f"WLS capability screen failed: {result}")
    return result


def _write_native_stability_outputs(
    native_position_path: Path,
    device_path: Path,
    output_dir: Path,
) -> tuple[Path, Path, dict[str, Any], dict[str, Any]]:
    native_positions = smoother._read_positions(native_position_path, 18)
    device_epochs = smoother._read_device_epochs(device_path, 1)
    native_keys = [row.timestamp_ms for row in native_positions]
    if any(timestamp not in set(device_epochs) for timestamp in native_keys):
        raise StabilitySelectorEvaluationError("native POS contains a key absent from device epochs")
    if any(b <= a for a, b in zip(native_keys, native_keys[1:])):
        raise StabilitySelectorEvaluationError("native POS keys are not strictly increasing")
    baseline = smoother.smooth_positions(
        native_positions, device_epochs, smoother.SmootherConfig(**BASELINE_CONFIG)
    )
    stable_result, stability = segment_stability.apply_segment_stability(
        baseline,
        native_positions,
        device_epochs,
        max_consecutive_rejects=SEGMENT_MAX_REJECTS,
        max_prediction_duration_s=SEGMENT_MAX_PREDICTION_DURATION_S,
        reject_fraction_max=None,
        measurement_floor_m=BASELINE_CONFIG["measurement_floor_m"],
        leap_seconds=18,
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    smoother.write_outputs(
        stable_result,
        smoother.SmootherConfig(**BASELINE_CONFIG),
        output_dir,
        position_path=native_position_path,
        device_path=device_path,
        skip_epochs=1,
        leap_seconds=18,
    )
    report_path = output_dir / "segment_stability.json"
    report = {
        **stability,
        "truth_used": False,
        "position_lane": "native-galileo-e1-hatch30-segment-stability",
        "raw_position": _artifact(native_position_path),
    }
    _atomic_json(report_path, report)
    manifest_path = output_dir / "smoother_manifest.json"
    manifest = _load_json(manifest_path, "native smoother manifest")
    manifest["position_lane"] = "native-galileo-e1-hatch30-segment-stability"
    manifest["truth_used"] = False
    manifest["segment_stability"] = {
        "path": str(report_path),
        "sha256": _sha256(report_path),
        "population": stability["population"],
        "thresholds": stability["thresholds"],
    }
    _atomic_json(manifest_path, manifest)
    return output_dir / "smoothed.pos", report_path, stability, {
        "position": _artifact(output_dir / "smoothed.pos"),
        "report": _artifact(report_path),
        "manifest": _artifact(manifest_path),
    }


def _run_truth_free_candidate(
    archive_path: Path,
    profile: dict[str, Any],
    candidate: dict[str, Any],
    archive_sha256: str,
    output_dir: Path,
    *,
    max_epochs: int,
) -> dict[str, Any]:
    """Generate both lanes and the selector output without opening truth."""

    member_hashes = _member_hashes_without_truth(archive_path, candidate)
    candidate_profile = _candidate_profile_without_truth(
        profile, candidate, archive_sha256, member_hashes
    )
    materialized = _materialize_truth_free_inputs(
        archive_path, candidate, output_dir, archive_sha256, member_hashes
    )
    device_path = materialized["inputs"] / "device_gnss.csv"
    screen = _screen_device(device_path, candidate["dataset_id"])
    route_root = output_dir / "routes" / _safe_id(candidate["dataset_id"])
    profile_path = route_root / "candidate_profile.json"
    _atomic_json(profile_path, candidate_profile)
    generated = conservative._run_adapter_and_spp(
        candidate,
        candidate_profile,
        materialized,
        profile_path,
        output_dir,
        max_epochs=max_epochs,
    )
    native_position_path = Path(generated["position"])
    wls_dir = route_root / "wls"
    rss_before = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    wls_started = time.perf_counter()
    wls_manifest_payload = wls.extract_to_directory(
        device_path,
        wls_dir,
        skip_epochs=1,
        max_epochs=max_epochs,
        role="development",
        dataset_id=candidate["dataset_id"],
    )
    wls_wall = time.perf_counter() - wls_started
    rss_after = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    wls_position_path = wls_dir / "wls.pos"
    native_output_dir = route_root / "native-segment-stability"
    native_stable_path, native_report_path, native_stability, native_artifacts = (
        _write_native_stability_outputs(native_position_path, device_path, native_output_dir)
    )
    # Validate the complete truth-free decision and publish selected POS plus
    # Kaggle-compatible submission before the new route's truth is extracted.
    selector_output_dir = route_root / "selector"
    selector_manifest = selector.select_and_publish(
        native_stable_path,
        native_report_path,
        wls_position_path,
        wls_dir / "wls_manifest.json",
        device_path,
        selector_output_dir,
        phone=candidate["phone"],
        dataset_id=candidate["dataset_id"],
        skip_epochs=1,
    )
    device_epochs = smoother._read_device_epochs(device_path, 1)
    native_positions = smoother._read_positions(native_position_path, 18)
    wls_positions = smoother._read_positions(wls_position_path, 18)
    if max_epochs > 0:
        device_epochs = device_epochs[:max_epochs]
    native_keys = [row.timestamp_ms for row in native_positions]
    if any(timestamp not in set(device_epochs) for timestamp in native_keys):
        raise StabilitySelectorEvaluationError("native candidate contains a key absent from device epochs")
    if [row.timestamp_ms for row in wls_positions] != device_epochs:
        raise StabilitySelectorEvaluationError("WLS candidate keys changed after generation")
    return {
        "dataset_id": candidate["dataset_id"],
        "candidate": candidate,
        "candidate_profile": {"path": str(profile_path), "sha256": _sha256(profile_path)},
        "materialization": {
            "path": str(materialized["manifest"]),
            "sha256": materialized["manifest_sha256"],
            "truth_used_for_materialization": False,
        },
        "inputs": {
            "device_gnss": _artifact(device_path),
            "device_imu": _artifact(materialized["inputs"] / "device_imu.csv"),
            "broadcast_nav": _artifact(materialized["inputs"] / "brdc.nav"),
        },
        "truth_path": materialized["inputs"] / "ground_truth.csv",
        "truth_member": generalization._member_names(candidate["route"], candidate["phone"])["ground_truth"],
        "truth_size": int(candidate["central_directory_files"]["ground_truth.csv"]["file_size"]),
        "screen": screen,
        "device_path": device_path,
        "native_position_path": native_position_path,
        "wls_position_path": wls_position_path,
        "native_stable_path": native_stable_path,
        "native_report_path": native_report_path,
        "native_stability": native_stability,
        "native_artifacts": native_artifacts,
        "wls_manifest_payload": wls_manifest_payload,
        "wls_manifest_path": wls_dir / "wls_manifest.json",
        "wls_summary_path": wls_dir / "wls_summary.json",
        "selector_manifest": selector_manifest,
        "selector_manifest_path": selector_output_dir / "selector_manifest.json",
        "selected_position_path": selector_output_dir / "selected.pos",
        "selected_submission_path": selector_output_dir / "submission.csv",
        "truth_free_generation_complete": True,
        "timing": {
            "adapter_spp_stages": generated["stages"],
            "wls_extraction_wall_s": wls_wall,
            "peak_rss_kb_process_max": max(int(rss_before), int(rss_after)),
        },
    }


def _extract_truth_after_selector(
    archive_path: Path, route: dict[str, Any]
) -> tuple[Path, dict[str, Any], dict[int, tuple[float, float, float]]]:
    artifact = _extract_member_atomic(
        archive_path,
        route["truth_member"],
        route["truth_path"],
        expected_size=route["truth_size"],
    )
    truth = smoother_eval._read_truth(route["truth_path"])
    if not truth:
        raise StabilitySelectorEvaluationError("new validation truth contains no rows")
    return route["truth_path"], artifact, truth


def _raw_rows(positions: list[smoother.PositionRow]) -> list[smoother.SmoothedRow]:
    return previous._raw_rows(positions)


def _score(
    rows: list[smoother.SmoothedRow],
    positions: list[smoother.PositionRow],
    epochs: list[int],
    truth: dict[int, tuple[float, float, float]],
) -> dict[str, Any]:
    return previous._score(rows, positions, epochs, truth)


def _score_new_route(route: dict[str, Any], truth: dict[int, tuple[float, float, float]]) -> dict[str, Any]:
    native_positions = smoother._read_positions(route["native_position_path"], 18)
    wls_positions = smoother._read_positions(route["wls_position_path"], 18)
    selected_positions = smoother._read_positions(route["selected_position_path"], 18)
    epochs = smoother._read_device_epochs(route["device_path"], 1)
    selected_lane = str(route["selector_manifest"]["decision"])
    selected_source_positions = native_positions if selected_lane == "native_stable" else wls_positions
    stable_positions = smoother._read_positions(route["native_stable_path"], 18)
    return {
        "native_raw": _score(_raw_rows(native_positions), native_positions, epochs, truth),
        "wls_raw": _score(_raw_rows(wls_positions), wls_positions, epochs, truth),
        "native_stable": _score(
            _raw_rows(stable_positions), native_positions, epochs, truth
        ),
        "selector": _score(
            _raw_rows(selected_positions), selected_source_positions, epochs, truth
        ),
        "selected_lane": selected_lane,
    }


def _known_route_inputs(
    existing_report: dict[str, Any],
    prior_report: dict[str, Any],
    dataset_id: str,
) -> dict[str, Path | dict[str, Any]]:
    """Resolve truth-free inputs for one of the seven already audited routes."""

    if dataset_id != PRIOR_NEW_ROUTE_ID:
        artifact = existing_report.get("route_artifacts", {}).get(dataset_id)
        if not isinstance(artifact, dict):
            raise StabilitySelectorEvaluationError(f"existing report lacks route: {dataset_id}")
        native = artifact.get("native_position")
        wls_position = artifact.get("wls_position")
        wls_manifest = artifact.get("wls_manifest")
        device = artifact.get("device_gnss")
        stability = artifact.get("native_segment_stability")
        if not all(isinstance(value, dict) for value in (native, wls_position, wls_manifest)):
            raise StabilitySelectorEvaluationError(f"existing route lacks lane artifacts: {dataset_id}")
        if not isinstance(stability, dict):
            raise StabilitySelectorEvaluationError(f"existing route lacks segment report: {dataset_id}")
        native_path = Path(str(native["path"]))
        # The main Pixel7Pro stable report has a distinct smoothed POS.  Other
        # routes are unstable and their native POS is not selected, but still
        # undergoes key validation in the selector.
        if dataset_id == MAIN_ID:
            native_path = (
                ROOT
                / "output"
                / "smartphone-r5"
                / "segment-stability-v1"
                / "routes"
                / "2023-05-24-20-26-us-ca-sjc-ge2__pixel7pro"
                / "segment_r15_d15"
                / "smoothed.pos"
            )
        return {
            "device": Path(str(device)),
            "native": native_path,
            "native_raw": Path(str(native["path"])),
            "wls": Path(str(wls_position["path"])),
            "wls_manifest": Path(str(wls_manifest["path"])),
            "stability": stability,
        }
    route = prior_report.get("route")
    if not isinstance(route, dict):
        raise StabilitySelectorEvaluationError("prior device-family report lacks route")
    inputs = route.get("inputs")
    native_artifact = route.get("native_segment_stability", {}).get("position")
    wls_manifest = route.get("wls_manifest")
    if not isinstance(inputs, dict) or not isinstance(native_artifact, dict) or not isinstance(wls_manifest, dict):
        raise StabilitySelectorEvaluationError("prior route lacks lane artifacts")
    wls_manifest_path = Path(str(wls_manifest["path"]))
    manifest = _load_json(wls_manifest_path, "prior WLS manifest")
    wls_position = Path(str(manifest["artifacts"]["position"]["path"]))
    return {
        "device": Path(str(inputs["device_gnss"]["path"])),
        "native": Path(str(native_artifact["path"])),
        "native_raw": Path(str(route["segment_reports"]["native"].get("raw_position", {}).get("path", native_artifact["path"]))),
        "wls": wls_position,
        "wls_manifest": wls_manifest_path,
        "stability": route["segment_reports"]["native"],
    }


def _canonical_stability_report(
    stability: dict[str, Any], native_raw: Path
) -> dict[str, Any]:
    """Strip the existing score report down to truth-free selector evidence."""

    population = stability.get("population")
    segments = stability.get("segments")
    if not isinstance(population, dict) or not isinstance(segments, list):
        raise StabilitySelectorEvaluationError("existing route segment evidence is malformed")
    report = {
        "schema_version": selector.SEGMENT_STABILITY_SCHEMA_VERSION,
        "truth_used": False,
        "decision_policy": {"truth_used": False},
        "population": population,
        "segments": segments,
        "raw_position": _artifact(native_raw),
    }
    return report


def _publish_known_selectors(
    output_dir: Path,
    existing_report: dict[str, Any],
    prior_report: dict[str, Any],
) -> dict[str, dict[str, Any]]:
    published: dict[str, dict[str, Any]] = {}
    existing_scores = existing_report.get("scores", {})
    prior_scores = prior_report.get("new_route_scores", {})
    for dataset_id in KNOWN_SEVEN_IDS:
        inputs = _known_route_inputs(existing_report, prior_report, dataset_id)
        route_dir = output_dir / "known-routes" / _safe_id(dataset_id)
        route_dir.mkdir(parents=True, exist_ok=True)
        report_path = route_dir / "native_segment_stability.json"
        report = _canonical_stability_report(inputs["stability"], Path(inputs["native_raw"]))
        _atomic_json(report_path, report)
        result = selector.select_and_publish(
            Path(inputs["native"]),
            report_path,
            Path(inputs["wls"]),
            Path(inputs["wls_manifest"]),
            Path(inputs["device"]),
            route_dir / "selector",
            phone=dataset_id.rsplit("/", 1)[1],
            dataset_id=dataset_id,
            skip_epochs=1,
        )
        published[dataset_id] = {
            "decision": result["decision"],
            "reason": result["reason"],
            "manifest": _artifact(route_dir / "selector" / "selector_manifest.json"),
            "selected_position": _artifact(route_dir / "selector" / "selected.pos"),
            "selected_submission": _artifact(route_dir / "selector" / "submission.csv"),
            "native_stability_report": _artifact(report_path),
            "device_gnss": _artifact(Path(inputs["device"])),
        }
    return published


def _metric(metrics: dict[str, Any], path: tuple[str, ...]) -> float:
    value: Any = metrics
    for key in path:
        value = value[key]
    try:
        number = float(value)
    except (TypeError, ValueError, KeyError):
        return math.inf
    return number if math.isfinite(number) else math.inf


def _aggregate(metrics: list[dict[str, Any]]) -> dict[str, Any]:
    if not metrics:
        raise StabilitySelectorEvaluationError("cannot aggregate an empty lane")
    return wls_eval._aggregate(metrics)


def _known_lane_maps(
    existing_report: dict[str, Any], prior_report: dict[str, Any]
) -> dict[str, dict[str, Any]]:
    scores = existing_report.get("scores")
    if not isinstance(scores, dict):
        raise StabilitySelectorEvaluationError("existing WLS report lacks scores")
    required = ("wls_raw", "native_segment_stability_recommended")
    if any(not isinstance(scores.get(key), dict) for key in required):
        raise StabilitySelectorEvaluationError("existing WLS report lacks known lane scores")
    prior_scores = prior_report.get("new_route_scores")
    if not isinstance(prior_scores, dict):
        raise StabilitySelectorEvaluationError("prior family report lacks new-route scores")
    if not isinstance(prior_scores.get("wls_raw"), dict) or not isinstance(prior_scores.get("native_segment_stability"), dict):
        raise StabilitySelectorEvaluationError("prior family report lacks required lane scores")
    native = {dataset_id: scores["native_segment_stability_recommended"][dataset_id] for dataset_id in EXISTING_SIX_IDS}
    wls_scores = {dataset_id: scores["wls_raw"][dataset_id] for dataset_id in EXISTING_SIX_IDS}
    native[PRIOR_NEW_ROUTE_ID] = prior_scores["native_segment_stability"]
    wls_scores[PRIOR_NEW_ROUTE_ID] = prior_scores["wls_raw"]
    return {"native_segment_stability": native, "wls_raw": wls_scores}


def _known_aggregates(
    lane_maps: dict[str, dict[str, Any]],
    decisions: dict[str, dict[str, Any]],
) -> dict[str, dict[str, Any]]:
    native = [lane_maps["native_segment_stability"][dataset_id] for dataset_id in KNOWN_SEVEN_IDS]
    wls_scores = [lane_maps["wls_raw"][dataset_id] for dataset_id in KNOWN_SEVEN_IDS]
    selected = []
    for dataset_id in KNOWN_SEVEN_IDS:
        decision = decisions[dataset_id]["decision"]
        selected.append(
            lane_maps["native_segment_stability"][dataset_id]
            if decision == "native_stable"
            else lane_maps["wls_raw"][dataset_id]
        )
    return {
        "native_only": _aggregate(native),
        "wls_only": _aggregate(wls_scores),
        "selector": _aggregate(selected),
    }


def _new_route_gate(scores: dict[str, Any], profile: dict[str, Any]) -> dict[str, Any]:
    selected = scores["selector"]
    other = scores["native_stable"] if scores["selected_lane"] == "wls_raw" else scores["wls_raw"]
    failures: list[str] = []
    if _metric(selected, ("availability_ratio",)) < _metric(other, ("availability_ratio",)) - 1e-12:
        failures.append("availability_regression")
    for path, name in (
        (("horizontal_wgs84_m", "p50_m"), "h_p50_regression"),
        (("horizontal_wgs84_m", "p95_m"), "h_p95_regression"),
    ):
        if _metric(selected, path) > _metric(other, path) + 1e-12:
            failures.append(name)
    for key in DIAGNOSTIC_KEYS:
        if _metric(selected, ("kaggle_diagnostic_score_variants_m", key)) > _metric(
            other, ("kaggle_diagnostic_score_variants_m", key)
        ) + 1e-12:
            failures.append(f"{key}_regression")
    threshold = float(profile["thresholds"]["vertical_p95_max"])
    if _metric(selected, ("vertical_p95_abs_m",)) > threshold + 1e-12:
        failures.append("v_p95_profile_threshold")
    return {
        "passed": not failures,
        "failures": failures,
        "selected_lane": scores["selected_lane"],
        "horizontal_official_like_metric": "wgs84_vincenty__linear_n_minus_1",
        "selected": selected,
        "other_lane": other,
        "vertical_p95_threshold_m": threshold,
    }


def _known_gate(
    aggregates: dict[str, dict[str, Any]], profile: dict[str, Any]
) -> dict[str, Any]:
    selector_aggregate = aggregates["selector"]
    failures: list[str] = []
    for reference_name in ("native_only", "wls_only"):
        reference = aggregates[reference_name]
        if _metric(selector_aggregate, ("mean_availability_ratio",)) < _metric(
            reference, ("mean_availability_ratio",)
        ) - 1e-12:
            failures.append(f"{reference_name}:availability_regression")
        for path, label in (
            (("mean_horizontal_wgs84_p50_m",), "h_p50"),
            (("mean_horizontal_wgs84_p95_m",), "h_p95"),
        ):
            if _metric(selector_aggregate, path) >= _metric(reference, path) - 1e-12:
                failures.append(f"{reference_name}:{label}_not_strictly_better")
        for key in DIAGNOSTIC_KEYS:
            current = _metric(selector_aggregate, ("mean_kaggle_diagnostic_score_variants_m", key))
            baseline = _metric(reference, ("mean_kaggle_diagnostic_score_variants_m", key))
            if current >= baseline - 1e-12:
                failures.append(f"{reference_name}:{key}_not_strictly_better")
    threshold = float(profile["thresholds"]["vertical_p95_max"])
    if _metric(selector_aggregate, ("mean_vertical_p95_abs_m",)) > threshold + 1e-12:
        failures.append("v_p95_profile_threshold")
    return {
        "passed": not failures,
        "failures": failures,
        "vertical_p95_threshold_m": threshold,
        "horizontal_comparison": {
            "strict_metrics": [
                "mean_horizontal_wgs84_p50_m",
                "mean_horizontal_wgs84_p95_m",
                *[f"mean_kaggle_diagnostic_score_variants_m.{key}" for key in DIAGNOSTIC_KEYS],
            ]
        },
    }


def _recommendation_profile(
    output_dir: Path,
    known_gate: dict[str, Any],
    new_gate: dict[str, Any],
    candidate_selector_manifest: Path,
) -> dict[str, Any]:
    return {
        "schema_version": "smartphone-r5-wls-stability-selector-profile.v1",
        "scope": "development-only recommended Kaggle submission lane",
        "truth_used_at_runtime": False,
        "production_default_changed": False,
        "lane_rule": {
            "native_stable": "all native Galileo E1/Hatch30 segment stats have stable=true",
            "wls_raw": "one or more native segments is unstable, or native/WLS integrity is not provable",
            "fail_closed": "malformed WLS manifest, key mismatch, and non-finite values reject publication",
        },
        "runtime_inputs": [
            "native segment-stability report",
            "WLS integrity manifest",
            "exact device epoch keys",
        ],
        "known_seven_route_gate": known_gate,
        "new_validation_gate": new_gate,
        "selector_manifest": _artifact(candidate_selector_manifest),
        "promotion_scope": "development-only; production RTK/SPP default remains unchanged",
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-wls-stability-selector-eval")
    )
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--inventory", type=Path, default=DEFAULT_INVENTORY)
    parser.add_argument("--selection-record", type=Path, default=DEFAULT_SELECTION_RECORD)
    parser.add_argument("--existing-report", type=Path, default=DEFAULT_EXISTING_REPORT)
    parser.add_argument("--device-family-report", type=Path, default=DEFAULT_DEVICE_FAMILY_REPORT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--max-epochs", type=int, default=-1)
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.max_epochs == 0 or args.max_epochs < -1:
            raise StabilitySelectorEvaluationError("max-epochs must be -1 or a positive integer")
        record = _load_selection_record(args.selection_record)
        record_hash = _sha256(args.selection_record)
        profile = previous._load_profile(args.profile)
        if str(dict(profile["datasets"]["holdout"])["id"]) != HOLDOUT_ID:
            raise StabilitySelectorEvaluationError("profile holdout differs from sealed contract")
        frozen_inventory = _load_frozen_inventory(args.inventory, record)
        _verify_candidate(frozen_inventory, record)
        expected_archive_hash = str(dict(profile["archive"]).get("sha256", ""))
        if not expected_archive_hash:
            raise StabilitySelectorEvaluationError("profile archive hash is empty")
        archive_hash = generalization._sha256(args.archive)
        if archive_hash != expected_archive_hash:
            raise StabilitySelectorEvaluationError("archive hash differs from frozen profile")
        if args.output_dir.exists() and any(args.output_dir.iterdir()):
            raise StabilitySelectorEvaluationError(
                f"refusing to overwrite non-empty output: {args.output_dir}"
            )
        args.output_dir.mkdir(parents=True, exist_ok=True)

        # Re-check central metadata immediately before opening the selected
        # device member.  This call uses ZIP central-directory metadata only.
        inventory = generalization.inventory_archive(args.archive)
        candidate = _verify_candidate(inventory, record)
        inventory["selection"] = {
            "selected_candidate_id": NEW_VALIDATION_ID,
            "known_seven_route_ids": list(KNOWN_SEVEN_IDS),
            "previously_used_ids_excluded": list(USED_IDS),
            "holdout_excluded": HOLDOUT_ID,
            "selection_member_content_read": False,
            "selection_truth_opened": False,
        }
        inventory["archive"].update(
            {
                "sha256": archive_hash,
                "archive_hash_verified_before_member_extraction": True,
                "member_content_read": False,
                "member_sha256_computed": False,
            }
        )
        inventory_path = args.output_dir / "archive_inventory.json"
        _atomic_json(inventory_path, inventory)

        started = time.perf_counter()
        route = _run_truth_free_candidate(
            args.archive,
            profile,
            candidate,
            archive_hash,
            args.output_dir,
            max_epochs=args.max_epochs,
        )
        expected_screen = record.get("capability_screen_expected")
        if not isinstance(expected_screen, dict):
            raise StabilitySelectorEvaluationError("selection record lacks frozen capability screen")
        for key in (
            "valid_galileo_e1_rows",
            "galileo_e1_rows",
            "galileo_e1_epochs",
            "galileo_e1_svid_count",
            "invalid_galileo_e1_rows",
            "wls_finite_rows",
            "wls_invalid_rows",
        ):
            if route["screen"].get(key) != expected_screen.get(key):
                raise StabilitySelectorEvaluationError(
                    f"capability screen changed for {NEW_VALIDATION_ID}: {key}"
                )
        # The known seven selector artifacts are also truth-free.  Their score
        # values are reused from the already audited reports below; no known
        # route truth is reopened by this command.
        existing_report = _load_json(args.existing_report, "existing WLS report")
        prior_report = _load_json(args.device_family_report, "prior device-family report")
        known_publications = _publish_known_selectors(args.output_dir, existing_report, prior_report)

        # This is the first and only truth-bearing operation for the newly
        # selected route, and it occurs after both lanes and selector output.
        truth_path, truth_artifact, truth = _extract_truth_after_selector(args.archive, route)
        materialization_path = Path(route["materialization"]["path"])
        materialization = _load_json(materialization_path, "truth-free materialization manifest")
        materialization["ground_truth"] = truth_artifact
        materialization["truth_deferred_until_after_selector"] = False
        materialization["truth_opened_after_selector"] = True
        _atomic_json(materialization_path, materialization)
        route["materialization"] = {
            "path": str(materialization_path),
            "sha256": _sha256(materialization_path),
            "truth_used_for_materialization": False,
            "truth_opened_after_selector": True,
        }
        candidate_scores = _score_new_route(route, truth)
        lane_maps = _known_lane_maps(existing_report, prior_report)
        known_aggregates = _known_aggregates(lane_maps, known_publications)
        new_gate = _new_route_gate(candidate_scores, profile)
        known_gate = _known_gate(known_aggregates, profile)
        promotion = (
            "promote-development-only-recommended-selector"
            if new_gate["passed"] and known_gate["passed"]
            else "no-go-selector"
        )
        recommendation_path: Path | None = None
        recommendation: dict[str, Any] | None = None
        if promotion == "promote-development-only-recommended-selector":
            recommendation_path = args.output_dir / "recommended_submission_lane_profile.json"
            recommendation = _recommendation_profile(
                args.output_dir,
                known_gate,
                new_gate,
                route["selector_manifest_path"],
            )
            _atomic_json(recommendation_path, recommendation)

        report_path = args.output_dir / "wls_stability_selector_report.json"
        report = {
            "schema_version": SCHEMA_VERSION,
            "decision": "development-only-truth-free-wls-stability-selector-evaluation",
            "selection_record": str(args.selection_record),
            "selection_record_sha256": record_hash,
            "archive": {
                "path": str(args.archive),
                "sha256": archive_hash,
                "profile_sha256": _sha256(args.profile),
                "inventory": _artifact(inventory_path),
            },
            "roles": {
                "known_seven_routes": list(KNOWN_SEVEN_IDS),
                "new_validation": NEW_VALIDATION_ID,
                "development_main": MAIN_ID,
                "holdout": HOLDOUT_ID,
                "all_used_or_sealed_excluded_from_new_selection": list(USED_IDS),
            },
            "truth_free_contract": {
                "central_directory_selection": True,
                "new_route_truth_free_generation_completed_before_truth_open": True,
                "new_route_truth_parsed_once_after_generation": True,
                "truth_dependent_runtime_selection": False,
                "selector_runtime_uses_device_model": False,
                "holdout_content_opened": False,
                "holdout_truth_opened": False,
                "holdout_materialized": False,
            },
            "candidate_metadata": candidate,
            "capability_screen": route["screen"],
            "route": {
                "inputs": route["inputs"],
                "candidate_profile": route["candidate_profile"],
                "materialization": route["materialization"],
                "truth": {"path": str(truth_path), "artifact": truth_artifact},
                "native": {
                    "position": _artifact(route["native_position_path"]),
                    "stable_position": _artifact(route["native_stable_path"]),
                    "stability_report": _artifact(route["native_report_path"]),
                    "stability": route["native_stability"],
                    "artifacts": route["native_artifacts"],
                },
                "wls": {
                    "position": _artifact(route["wls_position_path"]),
                    "manifest": _artifact(route["wls_manifest_path"]),
                    "summary": _artifact(route["wls_summary_path"]),
                },
                "selector": {
                    "decision": route["selector_manifest"]["decision"],
                    "reason": route["selector_manifest"]["reason"],
                    "manifest": _artifact(route["selector_manifest_path"]),
                    "selected_position": _artifact(route["selected_position_path"]),
                    "selected_submission": _artifact(route["selected_submission_path"]),
                },
                "timing": route["timing"],
                "truth_free_generation_complete": route["truth_free_generation_complete"],
            },
            "new_route_scores": candidate_scores,
            "known_seven_route_decisions": known_publications,
            "known_seven_route_aggregates": known_aggregates,
            "gates": {
                "new_validation": new_gate,
                "known_seven_route": known_gate,
                "promotion_decision": promotion,
            },
            "recommendation_profile": (
                _artifact(recommendation_path) if recommendation_path is not None else None
            ),
            "existing_reports": {
                "wls_position_report": _artifact(args.existing_report),
                "device_family_report": _artifact(args.device_family_report),
            },
            "timing": {
                "total_wall_s": time.perf_counter() - started,
                "truth_free_generation_before_truth_open": True,
            },
        }
        _atomic_json(report_path, report)
        manifest_path = args.output_dir / "wls_stability_selector_manifest.json"
        manifest = {
            "schema_version": "smartphone-r5-wls-stability-selector-evaluation-manifest.v1",
            "report": _artifact(report_path),
            "selection_record": {"path": str(args.selection_record), "sha256": record_hash},
            "inventory": _artifact(inventory_path),
            "new_route": {
                "dataset_id": NEW_VALIDATION_ID,
                "truth_free_generation": True,
                "truth_parsed_once_after_selector": True,
                "native": report["route"]["native"],
                "wls": report["route"]["wls"],
                "selector": report["route"]["selector"],
            },
            "known_seven_route_decisions": known_publications,
            "recommendation_profile": (
                _artifact(recommendation_path) if recommendation_path is not None else None
            ),
            "holdout_content_opened": False,
            "holdout_truth_opened": False,
            "holdout_materialized": False,
        }
        _atomic_json(manifest_path, manifest)
        print(f"Smartphone WLS stability selector evaluation complete: {report_path}")
        print(f"Promotion decision: {promotion}")
        return 0
    except (
        OSError,
        ValueError,
        KeyError,
        TypeError,
        json.JSONDecodeError,
        zipfile.BadZipFile,
        generalization.GeneralizationError,
        conservative.ConservativeEvaluationError,
        previous.ReacquisitionError,
        smoother.SmootherError,
        segment_stability.SegmentStabilityError,
        wls.WlsPositionError,
        selector.StabilitySelectorError,
        StabilitySelectorEvaluationError,
    ) as exc:
        print(f"Smartphone WLS stability selector evaluation failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(run())
